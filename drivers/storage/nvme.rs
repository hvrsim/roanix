//!
//! # NVM Express Block Driver
//!
//! The driver initializes the controller and its admin queue, discovers
//! namespaces, and publishes them as block devices. Each I/O queue owns
//! its command rings, DMA buffer, and physical region page list.
//!
//! A request prefers the current CPU's queue, borrows an idle peer's queue
//! when needed, and waits outside the ticket lock if all queues are busy.
//! On x86, the PCI affinity extension routes I/O completion vectors to their
//! preferred CPUs. Queues share vectors when fewer are available.
//!
//! Interrupt callbacks signal queue completion without borrowing the
//! controller state that probe is still building.
//!
//! Removal withdraws namespace registrations before releasing their
//! callback context. The controller must stop DMA before queue memory is freed.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_ptr_alignment,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays
)]

extern crate alloc;

use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::{
    ffi::{CStr, c_void},
    mem::size_of,
    ops::{Deref, DerefMut},
    ptr, slice,
    sync::atomic::{AtomicU32, AtomicUsize, Ordering},
};

use ddk::{
    BlockRegistration, Bus, Device, Dma, DriverRegistration, Error, Event, Irq, Mmio, Module,
    OwnedEvent, Result, TicketLock, raw,
};

#[cfg(target_arch = "x86_64")]
use ddk::{
    Mutex, TypedInterface,
    pci::{MsiAffinityOps, MsiOps},
};

const CAP: usize = 0x00;
const CC: usize = 0x14;
const CSTS: usize = 0x1c;
const AQA: usize = 0x24;
const ASQ: usize = 0x28;
const ACQ: usize = 0x30;
#[cfg(target_arch = "riscv64")]
const INTMS: usize = 0x0c;
#[cfg(target_arch = "riscv64")]
const INTMC: usize = 0x10;
const DOORBELL_BASE: usize = 0x1000;
const CC_ENABLE: u32 = 1;
const CSTS_READY: u32 = 1;
const CSTS_FATAL: u32 = 1 << 1;

const ADMIN_CREATE_SQ: u8 = 0x01;
const ADMIN_CREATE_CQ: u8 = 0x05;
const ADMIN_IDENTIFY: u8 = 0x06;
const ADMIN_SET_FEATURES: u8 = 0x09;
const FEATURE_NUMBER_OF_QUEUES: u32 = 0x07;
const IO_FLUSH: u8 = 0x00;
const IO_WRITE: u8 = 0x01;
const IO_READ: u8 = 0x02;

const PAGE_SIZE: usize = 4096;
const ADMIN_DEPTH: u16 = 64;
const IO_DEPTH: u16 = 256;
const DATA_BYTES: usize = 128 * 1024;
const COMMAND_TIMEOUT_NS: u64 = 5_000_000_000;
const MAX_CPUS: u32 = 64;

/// A submission queue entry in the controller's 64-byte layout.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Command {
    /// Opcode, command flags, and command identifier.
    cdw0: u32,

    /// Namespace identifier for namespace-scoped commands.
    nsid: u32,

    /// Reserved command doublewords kept zero.
    reserved2: [u32; 2],

    /// Optional metadata pointer.
    mptr: u64,

    /// First physical region page pointer.
    prp1: u64,

    /// Second page pointer or page-list address.
    prp2: u64,

    /// Command-specific doubleword ten.
    cdw10: u32,

    /// Command-specific doubleword eleven.
    cdw11: u32,

    /// Command-specific doubleword twelve.
    cdw12: u32,

    /// Command-specific doubleword thirteen.
    cdw13: u32,

    /// Command-specific doubleword fourteen.
    cdw14: u32,

    /// Command-specific doubleword fifteen.
    cdw15: u32,
}

impl Command {
    /// Encodes a namespace read or write, including the zero-based block count.
    fn read_write(opcode: u8, nsid: u32, lba: u64, blocks: u16) -> Self {
        Self {
            cdw0: u32::from(opcode),
            nsid,
            cdw10: lba as u32,
            cdw11: (lba >> 32) as u32,
            cdw12: u32::from(blocks - 1),
            ..Self::default()
        }
    }
}

/// A controller-written completion queue entry.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Completion {
    /// Command-specific completion result.
    result: u32,

    /// Reserved doubleword written by the controller.
    reserved: u32,

    /// Controller's updated submission queue head.
    sq_head: u16,

    /// ID of the submitting queue.
    sq_id: u16,

    /// Identifier of the completed command.
    cid: u16,

    /// Completion status with the ring phase in bit zero.
    status: u16,
}

const _: () = assert!(size_of::<Command>() == 64);
const _: () = assert!(size_of::<Completion>() == 16);

/// Submission and completion rings with one command admitted at a time.
struct QueuePair {
    /// Coherent ring written by software.
    submission: Dma,

    /// Coherent ring written by the controller.
    completion: Dma,

    /// NVM Express queue ID used to locate the doorbells.
    id: u16,

    /// Number of entries in each ring.
    depth: u16,

    /// Next submission ring slot.
    sq_tail: u16,

    /// Next completion ring slot.
    cq_head: u16,

    /// Expected phase bit for the next completion.
    phase: bool,

    /// Identifier reserved for the next command.
    next_cid: u16,

    /// Notification from the completion IRQ handler.
    event: OwnedEvent,

    /// Whether queue reuse is blocked until a valid completion or reset.
    failed: bool,
}

impl QueuePair {
    /// Allocates coherent submission and completion rings at a supported queue depth.
    fn new(device: Device, id: u16, requested_depth: u16, maximum_entries: u16) -> Result<Self> {
        let depth = requested_depth.min(maximum_entries).max(2);
        let submission = Dma::allocate(
            Some(device),
            usize::from(depth) * size_of::<Command>(),
            PAGE_SIZE,
            ddk::DMA_ZERO,
        )?;

        let completion = Dma::allocate(
            Some(device),
            usize::from(depth) * size_of::<Completion>(),
            PAGE_SIZE,
            ddk::DMA_ZERO,
        )?;

        let event = OwnedEvent::create()?;
        Ok(Self {
            submission,
            completion,
            id,
            depth,
            sq_tail: 0,
            cq_head: 0,
            phase: true,
            next_cid: 1,
            event,
            failed: false,
        })
    }

    /// Publishes one command, waits for its matching completion, and advances the queue.
    fn submit(&mut self, registers: &Mmio, stride: usize, command: Command) -> Result<u32> {
        if self.failed {
            return Err(Error::from_status(ddk::EIO));
        }

        // Only a matching completion allows the queue and its DMA buffers to be reused.
        self.failed = true;
        let cid = self.post_command(registers, stride, command);
        let completion = self.wait_for_completion(registers)?;
        self.consume_completion(registers, stride, cid, completion)
    }

    /// Writes a submission entry and rings the doorbell after publishing it to DMA.
    fn post_command(&mut self, registers: &Mmio, stride: usize, mut command: Command) -> u16 {
        let cid = self.next_cid;
        self.next_cid = self.next_cid.wrapping_add(1).max(1);
        command.cdw0 = (command.cdw0 & 0xffff) | (u32::from(cid) << 16);

        // SAFETY: queue allocation alignment and depth guarantee this points to
        // one initialized command-sized slot exclusively owned by this queue.
        unsafe {
            ptr::write_volatile(
                self.submission
                    .as_ptr()
                    .cast::<Command>()
                    .add(usize::from(self.sq_tail)),
                command,
            );
        }

        self.sq_tail = (self.sq_tail + 1) % self.depth;
        ddk::device_barrier();
        registers.write32(self.sq_doorbell(stride), u32::from(self.sq_tail));

        cid
    }

    /// Waits for the next completion phase while guarding against missed IRQ wakeups.
    fn wait_for_completion(&self, registers: &Mmio) -> Result<Completion> {
        // SAFETY: cq_head stays within the coherent completion ring until the
        // caller consumes the completion.
        let entry = unsafe {
            self.completion
                .as_ptr()
                .cast::<Completion>()
                .add(usize::from(self.cq_head))
        };

        let started = ddk::monotonic_ns();
        self.event.reset()?;

        loop {
            // SAFETY: a volatile load observes device writes to the phase bit.
            let status = unsafe { ptr::read_volatile(ptr::addr_of!((*entry).status)) };
            if status & 1 == u16::from(self.phase) {
                ddk::device_barrier();

                // SAFETY: a matching phase publishes the entire completion.
                return Ok(unsafe { ptr::read_volatile(entry) });
            }

            let elapsed = ddk::monotonic_ns().wrapping_sub(started);
            if elapsed >= COMMAND_TIMEOUT_NS {
                return Err(Error::from_status(ddk::EIO));
            }

            self.event.reset()?;

            // SAFETY: the entry is still live. Recheck the phase after resetting
            // the event so a concurrent completion cannot leave the thread asleep.
            let status = unsafe { ptr::read_volatile(ptr::addr_of!((*entry).status)) };
            if status & 1 == u16::from(self.phase) {
                continue;
            }

            rearm_completion_interrupt(registers);

            // Check the phase again after a timed wait because the completion
            // and the deadline can become ready together.
            self.event.wait_timeout(COMMAND_TIMEOUT_NS - elapsed)?;
        }
    }

    /// Acknowledges a matching completion and returns its result or an I/O error.
    fn consume_completion(
        &mut self,
        registers: &Mmio,
        stride: usize,
        cid: u16,
        completion: Completion,
    ) -> Result<u32> {
        if completion.cid != cid {
            return Err(Error::from_status(ddk::EIO));
        }

        self.failed = false;
        self.cq_head += 1;
        if self.cq_head == self.depth {
            self.cq_head = 0;
            self.phase = !self.phase;
        }

        registers.write32(self.cq_doorbell(stride), u32::from(self.cq_head));

        rearm_completion_interrupt(registers);

        if completion.status >> 1 != 0 {
            return Err(Error::from_status(ddk::EIO));
        }

        Ok(completion.result)
    }

    /// Returns the register offset for this submission queue's tail doorbell.
    fn sq_doorbell(&self, stride: usize) -> usize {
        DOORBELL_BASE + usize::from(self.id) * 2 * stride
    }

    /// Returns the register offset for this completion queue's head doorbell.
    fn cq_doorbell(&self, stride: usize) -> usize {
        DOORBELL_BASE + (usize::from(self.id) * 2 + 1) * stride
    }
}

/// Rearms the level-triggered completion IRQ after checking the queue.
fn rearm_completion_interrupt(registers: &Mmio) {
    #[cfg(target_arch = "riscv64")]
    registers.write32(INTMC, 1);

    #[cfg(target_arch = "x86_64")]
    let _ = registers;
}

/// A queue pair and the DMA memory reused for namespace transfers.
struct IoQueue {
    /// Rings used to submit and complete namespace I/O.
    pair: QueuePair,

    /// Reusable coherent buffer for transfer data.
    data: Dma,

    /// Coherent list of page addresses for multi-page transfers.
    prp_list: Dma,
}

impl IoQueue {
    /// Allocates an I/O queue and its reusable transfer buffer and PRP list.
    fn new(device: Device, id: u16, maximum_entries: u16) -> Result<Self> {
        Ok(Self {
            pair: QueuePair::new(device, id, IO_DEPTH, maximum_entries)?,
            data: Dma::allocate(Some(device), DATA_BYTES, PAGE_SIZE, ddk::DMA_ZERO)?,
            prp_list: Dma::allocate(Some(device), PAGE_SIZE, PAGE_SIZE, ddk::DMA_ZERO)?,
        })
    }

    /// Describes the transfer buffer with inline PRPs or the queue's DMA page list.
    fn set_prps(&mut self, command: &mut Command, length: usize) -> Result<()> {
        if self.pair.failed {
            return Err(Error::from_status(ddk::EIO));
        }
        if length == 0 || length > self.data.len() {
            return Err(Error::from_status(ddk::EINVAL));
        }
        command.prp1 = self.data.device_address();

        let pages = length.div_ceil(PAGE_SIZE);
        command.prp2 = match pages {
            1 => 0,
            2 => self.data.device_address() + PAGE_SIZE as u64,
            _ => {
                if (pages - 1) * size_of::<u64>() > self.prp_list.len() {
                    return Err(Error::from_status(ddk::ENOSPC));
                }

                for index in 1..pages {
                    // SAFETY: the bounds check above proves each entry lies in
                    // the exclusively borrowed coherent PRP-list allocation.
                    unsafe {
                        ptr::write_volatile(
                            self.prp_list.as_ptr().cast::<u64>().add(index - 1),
                            self.data.device_address() + (index * PAGE_SIZE) as u64,
                        );
                    }
                }

                self.prp_list.device_address()
            }
        };
        Ok(())
    }

    /// Attaches the queue's DMA buffer to a data command and submits it.
    fn submit_data(
        &mut self,
        registers: &Mmio,
        stride: usize,
        mut command: Command,
        length: usize,
    ) -> Result<()> {
        self.set_prps(&mut command, length)?;
        self.pair.submit(registers, stride, command).map(|_| ())
    }

    /// Reads into the queue DMA buffer and copies the completed bytes to the caller.
    fn read(
        &mut self,
        registers: &Mmio,
        stride: usize,
        nsid: u32,
        lba: u64,
        blocks: u16,
        output: &mut [u8],
    ) -> Result<()> {
        let command = Command::read_write(IO_READ, nsid, lba, blocks);
        self.submit_data(registers, stride, command, output.len())?;

        // SAFETY: the completed device-to-host transfer initialized this
        // coherent range, and `output` is disjoint writable caller storage.
        unsafe { ptr::copy_nonoverlapping(self.data.as_ptr(), output.as_mut_ptr(), output.len()) };
        Ok(())
    }

    /// Copies the caller's data to the queue DMA buffer before submitting a write.
    fn write(
        &mut self,
        registers: &Mmio,
        stride: usize,
        nsid: u32,
        lba: u64,
        blocks: u16,
        input: &[u8],
    ) -> Result<()> {
        if self.pair.failed || input.len() > self.data.len() {
            return Err(Error::from_status(ddk::EIO));
        }

        // SAFETY: the exclusively borrowed coherent allocation is valid for
        // the checked transfer length and is disjoint from caller input.
        unsafe { ptr::copy_nonoverlapping(input.as_ptr(), self.data.as_ptr(), input.len()) };

        let command = Command::read_write(IO_WRITE, nsid, lba, blocks);
        self.submit_data(registers, stride, command, input.len())
    }

    /// Submits a flush for the specified namespace and waits for completion.
    fn flush(&mut self, registers: &Mmio, stride: usize, nsid: u32) -> Result<()> {
        self.pair.submit(
            registers,
            stride,
            Command {
                cdw0: u32::from(IO_FLUSH),
                nsid,
                ..Command::default()
            },
        )?;
        Ok(())
    }
}

/// A value that can be borrowed by one queue user at a time.
struct QueueLease<T> {
    /// Idle queue, absent while a guard owns it.
    value: TicketLock<Option<T>>,

    /// Event signaled when a queue becomes available.
    ready: OwnedEvent,
}

impl<T> QueueLease<T> {
    /// Creates a lease around a value that is initially available to take.
    fn new(value: T) -> Result<Self> {
        Ok(Self {
            value: TicketLock::new(Some(value)),
            ready: OwnedEvent::create()?,
        })
    }

    /// Acquires the value if idle and resets its return event under the same lock.
    fn try_take(&self) -> Result<Option<QueueGuard<'_, T>>> {
        let mut value = self.value.lock_irqsave();

        // Reset and transfer ownership under the same lock used by return.
        // A later return cannot be lost between checking and waiting.
        self.ready.reset()?;
        Ok(value.take().map(|value| QueueGuard {
            owner: self,
            value: Some(value),
        }))
    }

    /// Waits for the current guard to return the value and then acquires it.
    fn take(&self) -> Result<QueueGuard<'_, T>> {
        loop {
            if let Some(guard) = self.try_take()? {
                return Ok(guard);
            }

            self.ready.wait()?;
        }
    }
}

/// Exclusive ownership of a leased queue until the guard is dropped.
struct QueueGuard<'a, T> {
    /// Lease to which this guard returns its queue.
    owner: &'a QueueLease<T>,

    /// Queue held until the guard releases it.
    value: Option<T>,
}

impl<T> Deref for QueueGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.value.as_ref().expect("live queue lease")
    }
}

impl<T> DerefMut for QueueGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.value.as_mut().expect("live queue lease")
    }
}

impl<T> Drop for QueueGuard<'_, T> {
    /// Returns the value to the lease and signals waiting borrowers.
    fn drop(&mut self) {
        let mut slot = self.owner.value.lock_irqsave();
        *slot = self.value.take();
        let _ = self.owner.ready.signal();
    }
}

/// One exposed NVM Express namespace and its block registration.
struct Namespace {
    /// Stable address of the controller owning this namespace.
    controller: usize,

    /// NVM Express namespace identifier.
    nsid: u32,

    /// Bytes per logical block.
    block_size: usize,

    /// Number of addressable logical blocks.
    block_count: u64,

    /// Receipt retained while the block device is published.
    block: TicketLock<Option<BlockRegistration>>,
}

impl Namespace {
    /// Builds a namespace if its Identify data describes a supported block format.
    fn from_identify(
        controller: usize,
        nsid: u32,
        data: &[u8],
        max_transfer: usize,
    ) -> Option<Self> {
        let block_count = le64(&data[..8]);
        if block_count == 0 {
            return None;
        }

        let format = usize::from(data[26] & 0xf);
        let descriptor = 128 + format * 4;
        if descriptor + 4 > data.len() || le16(&data[descriptor..descriptor + 2]) != 0 {
            return None;
        }

        let shift = data[descriptor + 2];
        if !(9..=16).contains(&shift) {
            return None;
        }

        let block_size = 1usize << shift;
        if block_size > max_transfer {
            return None;
        }

        Some(Self {
            controller,
            nsid,
            block_size,
            block_count,
            block: TicketLock::new(None),
        })
    }

    /// Validates the NVM Express block count and computes the ABI buffer length.
    fn transfer_geometry(&self, blocks: u32) -> Option<(u16, usize)> {
        let blocks = u16::try_from(blocks).ok().filter(|&blocks| blocks != 0)?;
        let length = usize::from(blocks).checked_mul(self.block_size)?;
        Some((blocks, length))
    }
}

/// Shared volatile registers whose access follows the controller's admission rules.
struct RegisterWindow(Mmio);

impl core::ops::Deref for RegisterWindow {
    type Target = Mmio;
    fn deref(&self) -> &Mmio {
        &self.0
    }
}

// SAFETY: NVMe uses volatile register accesses. Queues own distinct doorbells,
// and IRQ masking uses INTMS while consumers acknowledge through INTMC. Other
// controller configuration is serialized by startup/removal admission.
unsafe impl Sync for RegisterWindow {}

/// State available to IRQ callbacks while probe builds the controller.
///
/// The handler borrows this allocation independently while probe has exclusive
/// access to the controller.
struct InterruptState {
    /// Shared interrupt mask registers on the `INTx` path.
    #[cfg(target_arch = "riscv64")]
    registers: Arc<RegisterWindow>,

    /// Queue event identities published to the IRQ handler.
    event_ids: [AtomicUsize; MAX_CPUS as usize + 1],

    /// Vector indices that notify the corresponding queues.
    event_vectors: [AtomicU32; MAX_CPUS as usize + 1],

    /// Virtual IRQs installed for each vector index.
    interrupt_virqs: [AtomicU32; MAX_CPUS as usize + 1],

    /// Number of installed completion notifications.
    event_count: AtomicU32,
}

/// An NVM Express controller and the resources it owns for all queues and namespaces.
struct Controller {
    /// PCI function owning the controller.
    device: Device,

    /// Shared register window kept live through IRQ teardown.
    registers: Arc<RegisterWindow>,

    /// Byte spacing between successive doorbell registers.
    doorbell_stride: usize,

    /// Firmware-provided budget for controller readiness changes.
    timeout_ns: u64,

    /// Largest queue depth the controller supports.
    maximum_entries: u16,

    /// Negotiated limit on I/O queues.
    maximum_io_queues: u16,

    /// Exclusive access to the admin queue.
    admin: QueueLease<QueuePair>,

    /// Probe-owned DMA buffer retained until admin discovery finishes.
    identify_buffer: Option<Dma>,

    /// Leased data queues available to block requests.
    io: Vec<QueueLease<IoQueue>>,

    /// Stable namespace contexts and publication receipts.
    // Boxing is intentional: the block ABI stores stable namespace context
    // pointers while this vector is allowed to grow during discovery.
    #[allow(clippy::vec_box)]
    namespaces: Vec<Box<Namespace>>,

    /// Largest byte count published for one block request.
    max_transfer: usize,

    /// IRQ action receipts drained before controller memory is freed.
    interrupts: Vec<Irq>,

    /// State retained for direct access by the interrupt handler.
    irq_state: Arc<InterruptState>,

    /// Number of requested IRQs available to the queues.
    vector_count: u16,

    /// Whether the PCI message interrupt setup must be released.
    #[cfg(target_arch = "x86_64")]
    msi_active: bool,
}

// SAFETY: hardware permits concurrent writes to distinct volatile doorbells.
// Queue leases grant exclusive access to the rings and DMA buffers. Callback
// state is immutable or synchronized once published.
unsafe impl Sync for Controller {}

impl Controller {
    /// Resets the controller and prepares the admin queue and its coherent mappings.
    fn create(device: Device, registers: Mmio) -> Result<Box<Self>> {
        let registers = Arc::new(RegisterWindow(registers));
        let capabilities = registers.read64(CAP);
        if capabilities & (1u64 << 37) == 0 {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        let min_page_shift = 12 + ((capabilities >> 48) & 0xf) as u32;
        let max_page_shift = 12 + ((capabilities >> 52) & 0xf) as u32;
        if !(min_page_shift..=max_page_shift).contains(&12) {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        let maximum_entries = ((capabilities & 0xffff) + 1).min(u64::from(u16::MAX)) as u16;
        let timeout_ns = ((capabilities >> 24) & 0xff).max(1) * 500_000_000;
        let doorbell_stride = 4usize << ((capabilities >> 32) & 0xf);

        let maximum_doorbell = registers
            .len()
            .saturating_sub(DOORBELL_BASE + size_of::<u32>())
            / doorbell_stride;
        let maximum_io_queues = maximum_doorbell
            .saturating_sub(1)
            .checked_div(2)
            .unwrap_or(0)
            .min(usize::from(u16::MAX)) as u16;
        if maximum_io_queues == 0 {
            return Err(Error::from_status(ddk::EINVAL));
        }

        stop_controller(&registers, timeout_ns)?;

        let admin = QueuePair::new(device, 0, ADMIN_DEPTH, maximum_entries)?;
        registers.write32(
            AQA,
            u32::from(admin.depth - 1) | (u32::from(admin.depth - 1) << 16),
        );
        registers.write64(ASQ, admin.submission.device_address());
        registers.write64(ACQ, admin.completion.device_address());

        // Establish the rollback owner before enabling any device access.
        let controller = Box::new(Self {
            device,
            registers: registers.clone(),
            doorbell_stride,
            timeout_ns,
            maximum_entries,
            maximum_io_queues,
            admin: QueueLease::new(admin)?,
            identify_buffer: None,
            io: Vec::new(),
            namespaces: Vec::new(),
            max_transfer: DATA_BYTES,
            interrupts: Vec::new(),
            irq_state: Arc::new(InterruptState {
                #[cfg(target_arch = "riscv64")]
                registers: registers.clone(),
                event_ids: [const { AtomicUsize::new(0) }; MAX_CPUS as usize + 1],
                event_vectors: [const { AtomicU32::new(0) }; MAX_CPUS as usize + 1],
                interrupt_virqs: [const { AtomicU32::new(0) }; MAX_CPUS as usize + 1],
                event_count: AtomicU32::new(0),
            }),
            vector_count: 1,
            #[cfg(target_arch = "x86_64")]
            msi_active: false,
        });

        // NVM command set, round-robin arbitration, 4 KiB pages, 64-byte SQEs,
        // and 16-byte CQEs. Drop quiesces the device on every later failure.
        ddk::device_barrier();
        controller
            .registers
            .write32(CC, CC_ENABLE | (6 << 16) | (4 << 20));

        wait_ready(&controller.registers, true, timeout_ns)?;
        Ok(controller)
    }

    /// Installs handlers for the available virtual IRQs before enabling delivery.
    fn request_interrupts(&mut self, virqs: &[u32]) -> Result<()> {
        if virqs.is_empty() {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        self.vector_count = virqs.len().min(usize::from(u16::MAX)) as u16;
        self.irq_state.event_ids[0].store(self.admin.take()?.event.id(), Ordering::Release);
        self.irq_state.event_count.store(1, Ordering::Release);

        let context = Arc::as_ptr(&self.irq_state).cast_mut().cast::<c_void>();

        for (index, virq) in virqs.iter().enumerate() {
            self.irq_state.interrupt_virqs[index].store(*virq, Ordering::Release);
            let flags = if cfg!(target_arch = "riscv64") {
                ddk::IRQ_SHARED
            } else {
                0
            };
            self.interrupts.push(unsafe {
                Irq::request(self.device, *virq, c"nvme", flags, interrupt, context)
            }?);
        }
        Ok(())
    }

    /// Borrows the admin queue for a synchronous command.
    fn admin_command(&self, command: Command) -> Result<u32> {
        self.admin
            .take()?
            .submit(&self.registers, self.doorbell_stride, command)
    }

    /// Issues Identify with the selected namespace and structure selector.
    fn identify(&self, nsid: u32, cns: u32, buffer: &Dma) -> Result<()> {
        self.admin_command(Command {
            cdw0: u32::from(ADMIN_IDENTIFY),
            nsid,
            prp1: buffer.device_address(),
            cdw10: cns,
            ..Command::default()
        })?;
        Ok(())
    }

    /// Discovers controller limits and namespaces and prepares the I/O queues.
    fn configure(&mut self) -> Result<()> {
        self.identify_buffer = Some(Dma::allocate(
            Some(self.device),
            PAGE_SIZE,
            PAGE_SIZE,
            ddk::DMA_ZERO,
        )?);

        let identify = self
            .identify_buffer
            .as_ref()
            .expect("owned Identify buffer");
        self.identify(0, 1, identify)?;

        // SAFETY: Identify Controller completed successfully into all 4096
        // bytes of this coherent allocation.
        let controller_data = unsafe { slice::from_raw_parts(identify.as_ptr(), PAGE_SIZE) };
        let namespace_count = le32(&controller_data[516..520]);

        let mdts = controller_data[77];
        if mdts != 0 {
            let bytes = PAGE_SIZE.checked_shl(u32::from(mdts)).unwrap_or(usize::MAX);
            self.max_transfer = self.max_transfer.min(bytes);
        }

        self.create_io_queues()?;
        self.discover_namespaces(namespace_count)
    }

    /// Identifies each namespace and retains stable contexts for supported formats.
    fn discover_namespaces(&mut self, namespace_count: u32) -> Result<()> {
        let controller_pointer = ptr::from_ref(self) as usize;

        let identify = self
            .identify_buffer
            .as_ref()
            .expect("owned Identify buffer");

        for nsid in 1..=namespace_count {
            self.identify(nsid, 0, identify)?;

            // SAFETY: Identify Namespace repopulated the complete buffer.
            let data = unsafe { slice::from_raw_parts(identify.as_ptr(), PAGE_SIZE) };
            if let Some(namespace) =
                Namespace::from_identify(controller_pointer, nsid, data, self.max_transfer)
            {
                self.namespaces.push(Box::new(namespace));
            }
        }

        if self.namespaces.is_empty() {
            return Err(Error::from_status(ddk::ENODEV));
        }
        Ok(())
    }

    /// Negotiates the queue count and creates each completion and submission pair.
    fn create_io_queues(&mut self) -> Result<()> {
        let wanted = (ddk::cpu_count().clamp(1, MAX_CPUS) as u16).min(self.maximum_io_queues);
        let result = self.admin_command(Command {
            cdw0: u32::from(ADMIN_SET_FEATURES),
            cdw10: FEATURE_NUMBER_OF_QUEUES,
            cdw11: u32::from(wanted - 1) | (u32::from(wanted - 1) << 16),
            ..Command::default()
        })?;

        let submission_count = (result as u16).saturating_add(1);
        let completion_count = ((result >> 16) as u16).saturating_add(1);
        let count = wanted.min(submission_count).min(completion_count).max(1);
        for id in 1..=count {
            self.create_io_queue(id)?;
        }

        Ok(())
    }

    /// Allocates one I/O pair and registers its rings and completion notification.
    fn create_io_queue(&mut self, id: u16) -> Result<()> {
        // Publish ownership before any command can name this queue's DMA.
        self.io.push(QueueLease::new(IoQueue::new(
            self.device,
            id,
            self.maximum_entries,
        )?)?);
        let queue = self.io.last().expect("owned I/O queue").take()?;

        let vector = if self.vector_count <= 1 {
            0
        } else {
            (id - 1) % (self.vector_count - 1) + 1
        };

        let queue_size_and_id = u32::from(id) | (u32::from(queue.pair.depth - 1) << 16);
        self.admin_command(Command {
            cdw0: u32::from(ADMIN_CREATE_CQ),
            prp1: queue.pair.completion.device_address(),
            cdw10: queue_size_and_id,
            // Physically contiguous queue, interrupt enabled, and the
            // MSI-X/MSI vector selected for this completion queue.
            cdw11: 1 | (1 << 1) | (u32::from(vector) << 16),
            ..Command::default()
        })?;

        self.irq_state.event_ids[usize::from(id)].store(queue.pair.event.id(), Ordering::Release);
        self.irq_state.event_vectors[usize::from(id)].store(u32::from(vector), Ordering::Release);
        self.irq_state
            .event_count
            .store(u32::from(id) + 1, Ordering::Release);

        self.admin_command(Command {
            cdw0: u32::from(ADMIN_CREATE_SQ),
            prp1: queue.pair.submission.device_address(),
            cdw10: queue_size_and_id,
            cdw11: 1 | (u32::from(id) << 16),
            ..Command::default()
        })?;

        Ok(())
    }

    /// Prefers the current CPU's queue, borrows an idle peer's queue, or waits for admission.
    fn queue(&self) -> Result<QueueGuard<'_, IoQueue>> {
        let first = ddk::cpu_current() as usize % self.io.len();

        let mut events = [self.io[0].ready.event(); MAX_CPUS as usize];
        for (event, queue) in events.iter_mut().zip(&self.io) {
            *event = queue.ready.event();
        }

        loop {
            for step in 0..self.io.len() {
                if let Some(queue) = self.io[(first + step) % self.io.len()].try_take()? {
                    return Ok(queue);
                }
            }

            Event::wait_any(&events[..self.io.len()])?;
        }
    }

    /// Reads namespace blocks through an available queue.
    fn read_blocks(
        &self,
        namespace: &Namespace,
        lba: u64,
        blocks: u16,
        output: &mut [u8],
    ) -> Result<()> {
        let mut queue = self.queue()?;
        queue.read(
            &self.registers,
            self.doorbell_stride,
            namespace.nsid,
            lba,
            blocks,
            output,
        )
    }

    /// Writes namespace blocks through an available queue.
    fn write_blocks(
        &self,
        namespace: &Namespace,
        lba: u64,
        blocks: u16,
        input: &[u8],
    ) -> Result<()> {
        let mut queue = self.queue()?;
        queue.write(
            &self.registers,
            self.doorbell_stride,
            namespace.nsid,
            lba,
            blocks,
            input,
        )
    }

    /// Registers block devices for the discovered namespaces and retains their receipts.
    fn publish_blocks(&self, controller_id: u32) -> Result<()> {
        for namespace in &self.namespaces {
            let mut name_storage = [0u8; 32];
            let operations = raw::BlockOps {
                size: raw::BLOCK_OPS_SIZE,
                flags: 0,
                context: ptr::from_ref(namespace.as_ref()).cast_mut().cast(),
                logical_block_size: namespace.block_size as u32,
                max_transfer_blocks: (self.max_transfer / namespace.block_size) as u32,
                block_count: namespace.block_count,
                read: Some(block_read),
                write: Some(block_write),
                flush: Some(block_flush),
                discard: None,
            };

            // SAFETY: discovery is complete and all callback-visible state is
            // immutable or synchronized. The box remains bound until every
            // successful registration has been withdrawn.
            let registration = unsafe {
                BlockRegistration::register(
                    namespace_name(controller_id, namespace.nsid, &mut name_storage),
                    &operations,
                )?
            };
            *namespace.block.lock_irqsave() = Some(registration);
        }
        Ok(())
    }

    /// Unregisters namespace block devices before their callback storage can be released.
    fn remove_blocks(&self) -> Result<()> {
        for namespace in self.namespaces.iter().rev() {
            let registration = { namespace.block.lock_irqsave().take() };
            if let Some(mut registration) = registration {
                if let Err(error) = registration.unregister() {
                    *namespace.block.lock_irqsave() = Some(registration);
                    return Err(error);
                }
            }
        }
        Ok(())
    }

    /// Requests partition rescans after publishing the namespace block devices.
    fn scan_blocks(&self) {
        for namespace in &self.namespaces {
            let registration = { namespace.block.lock_irqsave().take() };
            if let Some(registration) = registration {
                if registration.rescan().is_err() {
                    ddk::log(
                        ddk::LOG_WARN,
                        c"keeping disk online after namespace partition rescan failed",
                    );
                }
                *namespace.block.lock_irqsave() = Some(registration);
            }
        }
    }

    /// Clears the controller enable bit and waits for it to stop processing queues.
    fn disable(&self) -> Result<()> {
        stop_controller(&self.registers, self.timeout_ns)
    }
}

impl Drop for Controller {
    /// Stops DMA and interrupt delivery before freeing controller resources.
    fn drop(&mut self) {
        assert!(self.disable().is_ok(), "NVMe failed to halt DMA");

        for interrupt in &mut self.interrupts {
            let _ = interrupt.release();
        }
        #[cfg(target_arch = "x86_64")]
        if self.msi_active {
            pci_msi_disable(self.device);
            self.msi_active = false;
        }
    }
}

/// Signals queue completion events associated with the interrupt that fired.
unsafe extern "C" fn interrupt(context: *mut c_void, virq: u32) -> u32 {
    if context.is_null() {
        return ddk::IRQ_NONE;
    }

    // SAFETY: IRQ receipts are drained before the retained InterruptState is
    // released. The shared notification fields are updated atomically.
    let controller = unsafe { &*context.cast::<InterruptState>() };

    // Quiet the level source before completing its PLIC claim. Consumers
    // unmask after checking/acknowledging their queues, retriggering delivery
    // when another queue completed while this interrupt was being handled.
    #[cfg(target_arch = "riscv64")]
    controller.registers.write32(INTMS, 1);

    let vector = controller
        .interrupt_virqs
        .iter()
        .position(|entry| entry.load(Ordering::Acquire) == virq)
        .unwrap_or(0) as u32;

    let count = controller.event_count.load(Ordering::Acquire) as usize;
    for (index, event) in controller.event_ids[..count.min(controller.event_ids.len())]
        .iter()
        .enumerate()
    {
        if controller.event_vectors[index].load(Ordering::Acquire) != vector {
            continue;
        }

        if let Some(event) = Event::from_id(event.load(Ordering::Acquire)) {
            let _ = event.signal();
        }
    }

    ddk::IRQ_HANDLED | ddk::IRQ_RESCHEDULE
}

/// Clears EN and waits for RDY to confirm the controller has stopped DMA.
fn stop_controller(registers: &Mmio, timeout_ns: u64) -> Result<()> {
    let command = registers.read32(CC);
    registers.write32(CC, command & !CC_ENABLE);
    ddk::device_barrier();

    // An earlier stop can still be pending after EN has been cleared.
    wait_ready(registers, false, timeout_ns)
}

/// Polls the ready bit until it matches, the controller reports fatal status, or time runs out.
fn wait_ready(registers: &Mmio, ready: bool, timeout_ns: u64) -> Result<()> {
    let started = ddk::monotonic_ns();
    loop {
        let status = registers.read32(CSTS);
        if ready && status & CSTS_FATAL != 0 {
            return Err(Error::from_status(ddk::EIO));
        }
        if (status & CSTS_READY != 0) == ready {
            return Ok(());
        }
        if ddk::monotonic_ns().wrapping_sub(started) >= timeout_ns {
            return Err(Error::from_status(ddk::EIO));
        }
        ddk::sleep_ns(100_000);
    }
}

fn le16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}

fn le32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes(bytes[..4].try_into().unwrap_or([0; 4]))
}

fn le64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(bytes[..8].try_into().unwrap_or([0; 8]))
}

/// Recovers a namespace from its live block callback context.
fn namespace(pointer: *mut c_void) -> Option<&'static Namespace> {
    if pointer.is_null() {
        None
    } else {
        // SAFETY: block registration stores a pointer to a boxed namespace and
        // is withdrawn before the box is released.
        Some(unsafe { &*pointer.cast::<Namespace>() })
    }
}

/// Follows the stable back-pointer from a live namespace to its controller.
fn controller(namespace: &Namespace) -> &'static Controller {
    // SAFETY: the namespace is owned by this controller and every namespace
    // registration is withdrawn before the controller is destroyed.
    unsafe { &*(namespace.controller as *const Controller) }
}

/// Validates the block read callback buffer and forwards the request to the controller.
unsafe extern "C" fn block_read(
    context: *mut c_void,
    lba: u64,
    blocks: u32,
    output: *mut u8,
) -> i32 {
    let Some(namespace) = namespace(context) else {
        return ddk::EINVAL;
    };

    let Some((blocks, length)) = namespace.transfer_geometry(blocks) else {
        return ddk::EINVAL;
    };

    // SAFETY: the block ABI supplies exactly the geometry-derived writable range.
    let output = match unsafe { ddk::output_bytes(output, length) } {
        Ok(value) => value,
        Err(error) => return error.status(),
    };

    controller(namespace)
        .read_blocks(namespace, lba, blocks, output)
        .map_or_else(Error::status, |()| ddk::OK)
}

/// Validates the block write callback buffer and forwards the request to the controller.
unsafe extern "C" fn block_write(
    context: *mut c_void,
    lba: u64,
    blocks: u32,
    input: *const u8,
) -> i32 {
    let Some(namespace) = namespace(context) else {
        return ddk::EINVAL;
    };

    let Some((blocks, length)) = namespace.transfer_geometry(blocks) else {
        return ddk::EINVAL;
    };

    // SAFETY: the block ABI supplies exactly the geometry-derived readable range.
    let input = match unsafe { ddk::input_bytes(input, length) } {
        Ok(value) => value,
        Err(error) => return error.status(),
    };

    controller(namespace)
        .write_blocks(namespace, lba, blocks, input)
        .map_or_else(Error::status, |()| ddk::OK)
}

/// Flushes pending writes to the namespace described by the callback context.
unsafe extern "C" fn block_flush(context: *mut c_void) -> i32 {
    let Some(namespace) = namespace(context) else {
        return ddk::EINVAL;
    };
    let controller = controller(namespace);
    let result = controller.queue().and_then(|mut queue| {
        queue.flush(
            &controller.registers,
            controller.doorbell_stride,
            namespace.nsid,
        )
    });
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Writes a decimal component into the output buffer and returns its length.
fn decimal(mut value: u32, output: &mut [u8]) -> usize {
    let mut digits = [0u8; 10];
    let mut count = 0;
    loop {
        digits[count] = b'0' + (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    for index in 0..count {
        output[index] = digits[count - index - 1];
    }
    count
}

/// Builds the NUL-terminated block device name from the controller and namespace IDs.
fn namespace_name(controller: u32, nsid: u32, output: &mut [u8; 32]) -> &CStr {
    output[..4].copy_from_slice(b"nvme");
    let mut length = 4 + decimal(controller, &mut output[4..]);
    output[length] = b'n';
    length += 1;
    length += decimal(nsid, &mut output[length..]);
    output[length] = 0;
    // SAFETY: the function constructed ASCII bytes followed by exactly one NUL.
    unsafe { CStr::from_bytes_with_nul_unchecked(&output[..=length]) }
}

static NEXT_CONTROLLER: AtomicU32 = AtomicU32::new(0);
#[cfg(target_arch = "x86_64")]
static MSI_REJECTED: AtomicU32 = AtomicU32::new(0);
#[cfg(target_arch = "x86_64")]
static ATTACHED_CONTROLLERS: AtomicU32 = AtomicU32::new(0);

#[cfg(target_arch = "x86_64")]
static MSI_BINDING: Mutex<Option<PciMsiBinding>> = Mutex::new(None);

/// Retained PCI service, with explicit vector destinations when available.
#[cfg(target_arch = "x86_64")]
enum PciMsiBinding {
    /// Provider with the per-vector affinity extension.
    WithAffinity(TypedInterface<MsiAffinityOps>),

    /// Provider exposing only the original message interrupt operations.
    Legacy(TypedInterface<MsiOps>),
}

#[cfg(target_arch = "x86_64")]
impl PciMsiBinding {
    /// Prefers the affinity extension and accepts an older provider as fallback.
    fn bind() -> Result<Self> {
        MsiAffinityOps::bind()
            .map(Self::WithAffinity)
            .or_else(|_| MsiOps::bind().map(Self::Legacy))
    }

    /// Returns the common operations and their provider context.
    fn parts(&self) -> (&MsiOps, *mut c_void) {
        match self {
            Self::WithAffinity(binding) => (&binding.ops().base, binding.context()),
            Self::Legacy(binding) => (binding.ops(), binding.context()),
        }
    }
}

/// Runs an MSI operation while keeping the provider binding locked and live.
#[cfg(target_arch = "x86_64")]
fn with_msi<T>(call: impl FnOnce(&PciMsiBinding) -> Result<T>) -> Result<T> {
    let binding = MSI_BINDING.lock();
    let binding = binding.as_ref().ok_or(Error::from_status(ddk::ENOTSUP))?;
    call(binding)
}

/// Requests masked admin and I/O routes in the same order used to assign queues.
#[cfg(target_arch = "x86_64")]
fn pci_msi_enable(device: Device) -> Result<Vec<u32>> {
    let requested = ddk::cpu_count().clamp(1, MAX_CPUS) + 1;
    let mut virqs = [0u32; MAX_CPUS as usize + 1];
    let mut actual = 0u32;
    with_msi(|binding| {
        let (operations, context) = binding.parts();
        let status = match binding {
            PciMsiBinding::WithAffinity(binding) => {
                let enable = binding
                    .ops()
                    .enable_with_affinity
                    .ok_or(Error::from_status(ddk::ENOTSUP))?;
                // Vector zero serves the admin queue. The following vectors
                // start again at CPU zero for the preferred I/O queues.
                let mut cpus = [0; MAX_CPUS as usize + 1];
                for (destination, cpu) in cpus[1..].iter_mut().zip(0..requested - 1) {
                    *destination = cpu;
                }
                // SAFETY: both arrays cover the request and the provider is retained.
                unsafe {
                    enable(
                        context,
                        device.as_raw(),
                        requested,
                        cpus.as_ptr(),
                        virqs.as_mut_ptr(),
                        &raw mut actual,
                    )
                }
            }
            PciMsiBinding::Legacy(_) => {
                let enable = operations.enable.ok_or(Error::from_status(ddk::ENOTSUP))?;
                // SAFETY: the output array covers the request and the provider is retained.
                unsafe {
                    enable(
                        context,
                        device.as_raw(),
                        requested,
                        virqs.as_mut_ptr(),
                        &raw mut actual,
                    )
                }
            }
        };
        if status == ddk::OK {
            Ok(())
        } else {
            Err(Error::from_status(status))
        }
    })?;
    if actual == 0 || actual > requested {
        pci_msi_disable(device);
        return Err(Error::from_status(ddk::EIO));
    }
    Ok(virqs[..actual as usize].to_vec())
}

/// Enables message delivery after the IRQ handlers have been installed.
#[cfg(target_arch = "x86_64")]
fn pci_msi_unmask(device: Device) -> Result<()> {
    with_msi(|binding| {
        let (operations, context) = binding.parts();
        let unmask = operations.unmask.ok_or(Error::from_status(ddk::ENOTSUP))?;
        // SAFETY: the device has an active configuration from `enable`.
        let status = unsafe { unmask(context, device.as_raw()) };
        if status == ddk::OK {
            Ok(())
        } else {
            Err(Error::from_status(status))
        }
    })
}

/// Releases message routes while the PCI MSI provider is still bound.
#[cfg(target_arch = "x86_64")]
fn pci_msi_disable(device: Device) {
    let _ = with_msi(|binding| {
        let (operations, context) = binding.parts();
        let disable = operations.disable.ok_or(Error::from_status(ddk::ENOTSUP))?;
        // SAFETY: disabling is idempotent for a PCI function owned by us.
        unsafe { disable(context, device.as_raw()) };
        Ok(())
    });
}

/// Gets the required PCI MSI or MSI-X vectors on x86.
#[cfg(target_arch = "x86_64")]
fn interrupt_routes(device: Device) -> Result<Vec<u32>> {
    pci_msi_enable(device).inspect_err(|_| {
        MSI_REJECTED.store(1, Ordering::Release);
        ddk::log(
            ddk::LOG_ERROR,
            c"NVMe probe refused because x86_64 requires MSI-X or MSI",
        );
    })
}

/// Gets the PLIC-routed PCI interrupt on RISC-V.
#[cfg(target_arch = "riscv64")]
fn interrupt_routes(device: Device) -> Result<Vec<u32>> {
    ddk::irq_of_device(device, 0)
        .inspect_err(|_| {
            ddk::log(
                ddk::LOG_ERROR,
                c"NVMe requires a PLIC-routed PCI interrupt on RISC-V",
            );
        })
        .map(|virq| alloc::vec![virq])
}

/// Maps the controller, installs interrupt delivery, and discovers its namespaces.
fn attach_controller(device: Device) -> Result<()> {
    let resource = ddk::pci::memory_bar(device, 0)?;
    if resource.length < DOORBELL_BASE as u64 + 8 {
        return Err(Error::from_status(ddk::EINVAL));
    }

    let registers = Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE)?;
    let virqs = interrupt_routes(device)?;

    let controller = Controller::create(device, registers);

    #[cfg(target_arch = "x86_64")]
    let controller = controller.inspect_err(|_| pci_msi_disable(device));

    let mut controller = controller?;

    #[cfg(target_arch = "x86_64")]
    {
        controller.msi_active = true;
    }

    controller.request_interrupts(&virqs)?;

    #[cfg(target_arch = "x86_64")]
    pci_msi_unmask(device)?;

    let controller_id = NEXT_CONTROLLER.fetch_add(1, Ordering::Relaxed);
    controller.configure()?;

    publish_controller(device, controller, controller_id)
}

/// Transfers ownership to the PCI binding and publishes the discovered disks.
fn publish_controller(
    device: Device,
    controller: Box<Controller>,
    controller_id: u32,
) -> Result<()> {
    let pointer = Box::into_raw(controller);

    // SAFETY: ownership transfers before any namespace is published.
    if let Err(error) = unsafe { device.set_data(pointer.cast()) } {
        // SAFETY: no block callback can reach this allocation yet.
        drop(unsafe { Box::from_raw(pointer) });
        return Err(error);
    }

    // SAFETY: the binding now owns a fully configured controller.
    let controller = unsafe { &*pointer };

    if let Err(error) = controller.publish_blocks(controller_id) {
        if controller.remove_blocks().is_ok() {
            // SAFETY: all published contexts have been withdrawn.
            let _ = unsafe { device.set_data(ptr::null_mut()) };
            drop(unsafe { Box::from_raw(pointer) });
            return Err(error);
        }

        // An external user holds a disk. Keep the binding and its resources
        // live so checked removal can reclaim them later.
        ddk::log(
            ddk::LOG_WARN,
            c"keeping controller active after incomplete namespace publication",
        );
    }

    controller.scan_blocks();

    #[cfg(target_arch = "x86_64")]
    ATTACHED_CONTROLLERS.fetch_add(1, Ordering::AcqRel);

    Ok(())
}

/// Attaches a PCI controller and reports the result through the driver ABI.
unsafe extern "C" fn probe(
    _context: *mut c_void,
    pointer: *const raw::Device,
    _match_data: usize,
) -> i32 {
    // SAFETY: the PCI bus supplies a live device during probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };

    attach_controller(device).map_or_else(Error::status, |()| ddk::OK)
}

/// Withdraws block devices and tears down the controller after checked removal succeeds.
unsafe extern "C" fn remove(_context: *mut c_void, pointer: *const raw::Device) -> i32 {
    // SAFETY: the driver core supplies the same live device used for probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::OK;
    };

    // SAFETY: only this driver stores a pointer in its bound device.
    let controller = unsafe { device.data() }.cast::<Controller>();
    if controller.is_null() {
        return ddk::OK;
    }

    // SAFETY: probe transferred a unique box and device removal serializes this
    // callback against another remove. Keep ownership in device data until all
    // externally reachable namespace contexts have been revoked.
    let controller_ref = unsafe { &*controller };
    if let Err(error) = controller_ref.remove_blocks() {
        return error.status();
    }

    // SAFETY: clearing prevents any later callback from claiming this box.
    let _ = unsafe { device.set_data(ptr::null_mut()) };
    #[cfg(target_arch = "x86_64")]
    ATTACHED_CONTROLLERS.fetch_sub(1, Ordering::AcqRel);

    // SAFETY: every node is gone, so no callback can retain a context pointer.
    drop(unsafe { Box::from_raw(controller) });
    ddk::OK
}

/// Disables the controller before system shutdown, leaving its registration intact.
unsafe extern "C" fn shutdown(_context: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the driver core supplies a live bound device.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };

    // SAFETY: probe stored a controller pointer for the binding lifetime.
    let controller = unsafe { device.data() }.cast::<Controller>();
    if !controller.is_null() {
        // SAFETY: shutdown is serialized against removal by the driver core.
        assert!(
            unsafe { &*controller }.disable().is_ok(),
            "NVMe failed to shut down"
        );
    }
}

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_ID,
    flags: 0,
    key: ptr::null(),
    value: ptr::null(),
    id0: 0,
    mask0: 0,
    id1: 0x01_08_02,
    mask1: 0x00ff_ffff,
    data: 0,
    score: 0,
}];

const DRIVER: raw::DriverDef = raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"nvme".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: None,
    shutdown: Some(shutdown),
    context: ptr::null_mut(),
    try_remove: Some(remove),
};

static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

/// Binds the required services and registers the storage driver on the PCI bus.
fn init_inner() -> Result<()> {
    #[cfg(target_arch = "x86_64")]
    {
        MSI_REJECTED.store(0, Ordering::Release);
        ATTACHED_CONTROLLERS.store(0, Ordering::Release);
        let binding = PciMsiBinding::bind().inspect_err(|_error| {
            ddk::log(
                ddk::LOG_ERROR,
                c"NVMe cannot load on x86_64 without the PCI MSI-X/MSI service",
            );
        })?;
        *MSI_BINDING.lock() = Some(binding);
    }

    let bus = Bus::find(c"pci")?;
    let definition = raw::DriverDef {
        bus: bus.as_raw(),
        ..DRIVER
    };

    // SAFETY: the match table and callbacks have static lifetimes.
    let registration = unsafe { DriverRegistration::register(&definition) }?;
    *REGISTRATION.lock_irqsave() = Some(registration);
    #[cfg(target_arch = "x86_64")]
    if MSI_REJECTED.load(Ordering::Acquire) != 0
        && ATTACHED_CONTROLLERS.load(Ordering::Acquire) == 0
    {
        ddk::log(
            ddk::LOG_ERROR,
            c"NVMe driver load failed: no matched controller supports MSI-X or MSI",
        );
        return Err(Error::from_status(ddk::ENOTSUP));
    }
    Ok(())
}

/// Initializes the module and unwinds partial registration on failure.
fn init(module: Module) -> Result<()> {
    let result = init_inner();
    if result.is_err() {
        exit(module);
    }
    result
}

/// Unregisters the driver and releases the retained PCI services.
fn exit(_module: Module) {
    if let Some(mut registration) = { REGISTRATION.lock_irqsave().take() } {
        let _ = registration.unregister();
    }

    #[cfg(target_arch = "x86_64")]
    MSI_BINDING.lock().take();
}

ddk::module!(b"nvme\0", b"NVM Express storage driver\0", init, exit,);
