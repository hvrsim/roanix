#![no_std]
#![no_main]
#![allow(unsafe_code)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_ptr_alignment,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays,
    clippy::too_many_lines
)]

//! NVM Express 1.x block driver with one interrupt-driven I/O queue per CPU.
//!
//! Queue memory, transfer buffers, and PRP lists are allocated once during
//! probe. Callers prefer their CPU's queue, borrow idle queues under load, and
//! sleep for admission when all queues are busy. No spin lock spans an I/O wait.

extern crate alloc;

mod lease;
use lease::{Guard as QueueGuard, Lease};

use alloc::{boxed::Box, vec::Vec};
use core::{
    ffi::{CStr, c_void},
    mem::size_of,
    ptr, slice,
    sync::atomic::{AtomicU32, AtomicUsize, Ordering, fence},
};

#[cfg(target_arch = "x86_64")]
use ddk::InterfaceBinding;
use ddk::{
    BlockRegistration, Bus, Device, Dma, DriverRegistration, Error, Event, Irq, Mmio, Module,
    RESOURCE_MEMORY, Result, TicketLock, raw,
};

const CAP: usize = 0x00;
const CC: usize = 0x14;
const CSTS: usize = 0x1c;
const AQA: usize = 0x24;
const ASQ: usize = 0x28;
const ACQ: usize = 0x30;
const DOORBELL_BASE: usize = 0x1000;
const CC_ENABLE: u32 = 1;
const CSTS_READY: u32 = 1;
const CSTS_FATAL: u32 = 1 << 1;

const ADMIN_CREATE_SQ: u8 = 0x01;
const ADMIN_DELETE_CQ: u8 = 0x04;
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

#[cfg(target_arch = "x86_64")]
#[repr(C)]
struct PciMsiOps {
    size: u32,
    enable: Option<
        unsafe extern "C" fn(*mut c_void, *const raw::Device, u32, *mut u32, *mut u32) -> i32,
    >,
    unmask: Option<unsafe extern "C" fn(*mut c_void, *const raw::Device) -> i32>,
    disable: Option<unsafe extern "C" fn(*mut c_void, *const raw::Device)>,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Command {
    cdw0: u32,
    nsid: u32,
    reserved2: [u32; 2],
    mptr: u64,
    prp1: u64,
    prp2: u64,
    cdw10: u32,
    cdw11: u32,
    cdw12: u32,
    cdw13: u32,
    cdw14: u32,
    cdw15: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Completion {
    result: u32,
    reserved: u32,
    sq_head: u16,
    sq_id: u16,
    cid: u16,
    status: u16,
}

const _: () = assert!(size_of::<Command>() == 64);
const _: () = assert!(size_of::<Completion>() == 16);

struct QueuePair {
    submission: Dma,
    completion: Dma,
    id: u16,
    depth: u16,
    sq_tail: u16,
    cq_head: u16,
    phase: bool,
    next_cid: u16,
    event: Event,
    failed: bool,
}

impl QueuePair {
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
        let event = Event::create()?;
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

    fn submit(&mut self, registers: &Mmio, stride: usize, mut command: Command) -> Result<u32> {
        if self.failed {
            return Err(Error::from_status(ddk::EIO));
        }
        // Until a matching completion is consumed, neither the queue nor its
        // DMA buffers may be reused, including after a timeout/wait error.
        self.failed = true;
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
        fence(Ordering::Release);
        registers.write32(self.sq_doorbell(stride), u32::from(self.sq_tail));

        let started = ddk::monotonic_ns();
        self.event.reset()?;
        let completion = loop {
            // SAFETY: `cq_head` is always within the coherent completion ring.
            let entry = unsafe {
                self.completion
                    .as_ptr()
                    .cast::<Completion>()
                    .add(usize::from(self.cq_head))
            };
            // SAFETY: the controller owns writes to this coherent slot; a
            // volatile load observes its phase transition without fabricating
            // a Rust reference to asynchronously-mutated memory.
            let status = unsafe { ptr::read_volatile(ptr::addr_of!((*entry).status)) };
            if status & 1 == u16::from(self.phase) {
                fence(Ordering::Acquire);
                // SAFETY: the matching phase publishes the entire completion.
                break unsafe { ptr::read_volatile(entry) };
            }
            let elapsed = ddk::monotonic_ns().wrapping_sub(started);
            if elapsed >= COMMAND_TIMEOUT_NS {
                return Err(Error::from_status(ddk::EIO));
            }
            self.event.reset()?;
            // Close the reset-vs-completion race before sleeping.
            let status = unsafe { ptr::read_volatile(ptr::addr_of!((*entry).status)) };
            if status & 1 == u16::from(self.phase) {
                continue;
            }
            if !self.event.wait_timeout(COMMAND_TIMEOUT_NS - elapsed)? {
                return Err(Error::from_status(ddk::EIO));
            }
        };
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
        if completion.status >> 1 != 0 {
            return Err(Error::from_status(ddk::EIO));
        }
        Ok(completion.result)
    }

    fn sq_doorbell(&self, stride: usize) -> usize {
        DOORBELL_BASE + usize::from(self.id) * 2 * stride
    }

    fn cq_doorbell(&self, stride: usize) -> usize {
        DOORBELL_BASE + (usize::from(self.id) * 2 + 1) * stride
    }
}

impl Drop for QueuePair {
    fn drop(&mut self) {
        let _ = self.event.destroy();
    }
}

struct IoQueue {
    pair: QueuePair,
    data: Dma,
    prp_list: Dma,
}

impl IoQueue {
    fn new(device: Device, id: u16, maximum_entries: u16) -> Result<Self> {
        Ok(Self {
            pair: QueuePair::new(device, id, IO_DEPTH, maximum_entries)?,
            data: Dma::allocate(Some(device), DATA_BYTES, PAGE_SIZE, ddk::DMA_ZERO)?,
            prp_list: Dma::allocate(Some(device), PAGE_SIZE, PAGE_SIZE, ddk::DMA_ZERO)?,
        })
    }

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

    fn read_dma(
        &mut self,
        registers: &Mmio,
        stride: usize,
        nsid: u32,
        lba: u64,
        blocks: u16,
        length: usize,
    ) -> Result<()> {
        let mut command = Command {
            cdw0: u32::from(IO_READ),
            nsid,
            cdw10: lba as u32,
            cdw11: (lba >> 32) as u32,
            cdw12: u32::from(blocks - 1),
            ..Command::default()
        };
        self.set_prps(&mut command, length)?;
        self.pair.submit(registers, stride, command)?;
        Ok(())
    }

    fn read(
        &mut self,
        registers: &Mmio,
        stride: usize,
        nsid: u32,
        lba: u64,
        blocks: u16,
        output: &mut [u8],
    ) -> Result<()> {
        self.read_dma(registers, stride, nsid, lba, blocks, output.len())?;
        // SAFETY: the completed device-to-host transfer initialized this
        // coherent range, and `output` is disjoint writable caller storage.
        unsafe { ptr::copy_nonoverlapping(self.data.as_ptr(), output.as_mut_ptr(), output.len()) };
        Ok(())
    }

    fn write_dma(
        &mut self,
        registers: &Mmio,
        stride: usize,
        nsid: u32,
        lba: u64,
        blocks: u16,
        length: usize,
    ) -> Result<()> {
        let mut command = Command {
            cdw0: u32::from(IO_WRITE),
            nsid,
            cdw10: lba as u32,
            cdw11: (lba >> 32) as u32,
            cdw12: u32::from(blocks - 1),
            ..Command::default()
        };
        self.set_prps(&mut command, length)?;
        self.pair.submit(registers, stride, command)?;
        Ok(())
    }

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
        self.write_dma(registers, stride, nsid, lba, blocks, input.len())
    }

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

struct Namespace {
    controller: usize,
    nsid: u32,
    block_size: usize,
    block: Option<BlockRegistration>,
}

struct Controller {
    device: Device,
    registers: Mmio,
    doorbell_stride: usize,
    timeout_ns: u64,
    maximum_entries: u16,
    maximum_io_queues: u16,
    admin: Lease<QueuePair>,
    io: Vec<Lease<IoQueue>>,
    // Boxing is intentional: the block ABI stores stable namespace context
    // pointers while this vector is allowed to grow during discovery.
    #[allow(clippy::vec_box)]
    namespaces: Vec<Box<Namespace>>,
    max_transfer: usize,
    interrupts: Vec<Irq>,
    event_ids: [AtomicUsize; MAX_CPUS as usize + 1],
    event_vectors: [AtomicU32; MAX_CPUS as usize + 1],
    interrupt_virqs: [AtomicU32; MAX_CPUS as usize + 1],
    event_count: AtomicU32,
    vector_count: u16,
    #[cfg(target_arch = "riscv64")]
    legacy_virq: AtomicU32,
    #[cfg(target_arch = "x86_64")]
    msi_active: bool,
}

// SAFETY: register accesses are volatile and hardware permits concurrent
// writes to distinct queue doorbells. Admin and I/O queue memory are protected
// by their individual ticket locks; all other fields are immutable after probe.
unsafe impl Sync for Controller {}

impl Controller {
    fn create(device: Device, registers: Mmio) -> Result<Box<Self>> {
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

        let current = registers.read32(CC);
        if current & CC_ENABLE != 0 {
            registers.write32(CC, current & !CC_ENABLE);
            wait_ready(&registers, false, timeout_ns)?;
        }
        let admin = QueuePair::new(device, 0, ADMIN_DEPTH, maximum_entries)?;
        registers.write32(
            AQA,
            u32::from(admin.depth - 1) | (u32::from(admin.depth - 1) << 16),
        );
        registers.write64(ASQ, admin.submission.device_address());
        registers.write64(ACQ, admin.completion.device_address());
        // NVM command set, round-robin arbitration, 4 KiB pages, 64-byte SQEs,
        // and 16-byte CQEs.
        registers.write32(CC, CC_ENABLE | (6 << 16) | (4 << 20));
        wait_ready(&registers, true, timeout_ns)?;

        Ok(Box::new(Self {
            device,
            registers,
            doorbell_stride,
            timeout_ns,
            maximum_entries,
            maximum_io_queues,
            admin: Lease::new(admin)?,
            io: Vec::new(),
            namespaces: Vec::new(),
            max_transfer: DATA_BYTES,
            interrupts: Vec::new(),
            event_ids: [const { AtomicUsize::new(0) }; MAX_CPUS as usize + 1],
            event_vectors: [const { AtomicU32::new(0) }; MAX_CPUS as usize + 1],
            interrupt_virqs: [const { AtomicU32::new(0) }; MAX_CPUS as usize + 1],
            event_count: AtomicU32::new(0),
            vector_count: 1,
            #[cfg(target_arch = "riscv64")]
            legacy_virq: AtomicU32::new(0),
            #[cfg(target_arch = "x86_64")]
            msi_active: false,
        }))
    }

    fn request_interrupts(&mut self, virqs: &[u32]) -> Result<()> {
        if virqs.is_empty() {
            return Err(Error::from_status(ddk::ENOTSUP));
        }
        self.vector_count = virqs.len().min(usize::from(u16::MAX)) as u16;
        self.event_ids[0].store(self.admin.take()?.event.id(), Ordering::Release);
        self.event_count.store(1, Ordering::Release);
        #[cfg(target_arch = "riscv64")]
        {
            self.legacy_virq.store(virqs[0], Ordering::Release);
        }
        let context = ptr::from_mut(self).cast::<c_void>();
        for (index, virq) in virqs.iter().enumerate() {
            self.interrupt_virqs[index].store(*virq, Ordering::Release);
            let flags = if cfg!(target_arch = "riscv64") {
                ddk::IRQ_SHARED
            } else {
                0
            };
            self.interrupts.push(Irq::request(
                self.device,
                *virq,
                c"nvme",
                flags,
                interrupt,
                context,
            )?);
        }
        Ok(())
    }

    fn admin_command(&self, command: Command) -> Result<u32> {
        self.admin
            .take()?
            .submit(&self.registers, self.doorbell_stride, command)
    }

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

    fn configure(&mut self, controller_id: u32) -> Result<()> {
        let identify = Dma::allocate(Some(self.device), PAGE_SIZE, PAGE_SIZE, ddk::DMA_ZERO)?;
        self.identify(0, 1, &identify)?;
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
        let controller_pointer = ptr::from_ref(self) as usize;
        for nsid in 1..=namespace_count {
            self.identify(nsid, 0, &identify)?;
            // SAFETY: Identify Namespace repopulated the complete buffer.
            let data = unsafe { slice::from_raw_parts(identify.as_ptr(), PAGE_SIZE) };
            let blocks = le64(&data[..8]);
            if blocks == 0 {
                continue;
            }
            let format = usize::from(data[26] & 0xf);
            let descriptor = 128 + format * 4;
            if descriptor + 4 > data.len() || le16(&data[descriptor..descriptor + 2]) != 0 {
                continue;
            }
            let shift = data[descriptor + 2];
            if !(9..=16).contains(&shift) {
                continue;
            }
            let block_size = 1usize << shift;
            if block_size > self.max_transfer {
                continue;
            }
            self.namespaces.push(Box::new(Namespace {
                controller: controller_pointer,
                nsid,
                block_size,
                block: None,
            }));
            let namespace = self
                .namespaces
                .last_mut()
                .expect("namespace was just pushed");
            let mut name_storage = [0u8; 32];
            let operations = raw::BlockOps {
                size: raw::BLOCK_OPS_SIZE,
                flags: 0,
                context: (&raw mut **namespace).cast(),
                logical_block_size: block_size as u32,
                max_transfer_blocks: (self.max_transfer / block_size) as u32,
                block_count: blocks,
                read: Some(block_read),
                write: Some(block_write),
                flush: Some(block_flush),
                discard: None,
            };
            // SAFETY: callbacks are static and the boxed namespace plus its
            // controller outlive the registration. The table itself is copied.
            namespace.block = Some(unsafe {
                BlockRegistration::register(
                    namespace_name(controller_id, nsid, &mut name_storage),
                    &operations,
                )?
            });
        }
        if self.namespaces.is_empty() {
            return Err(Error::from_status(ddk::ENODEV));
        }
        Ok(())
    }

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
            let queue = IoQueue::new(self.device, id, self.maximum_entries)?;
            let vector = if self.vector_count <= 1 {
                0
            } else {
                (id - 1) % (self.vector_count - 1) + 1
            };
            self.admin_command(Command {
                cdw0: u32::from(ADMIN_CREATE_CQ),
                prp1: queue.pair.completion.device_address(),
                cdw10: u32::from(id) | (u32::from(queue.pair.depth - 1) << 16),
                // Physically contiguous queue, interrupt enabled, and the
                // MSI-X/MSI vector selected for this completion queue.
                cdw11: 1 | (1 << 1) | (u32::from(vector) << 16),
                ..Command::default()
            })?;
            self.event_ids[usize::from(id)].store(queue.pair.event.id(), Ordering::Release);
            self.event_vectors[usize::from(id)].store(u32::from(vector), Ordering::Release);
            self.event_count.store(u32::from(id) + 1, Ordering::Release);
            if let Err(error) = self.admin_command(Command {
                cdw0: u32::from(ADMIN_CREATE_SQ),
                prp1: queue.pair.submission.device_address(),
                cdw10: u32::from(id) | (u32::from(queue.pair.depth - 1) << 16),
                cdw11: 1 | (u32::from(id) << 16),
                ..Command::default()
            }) {
                let _ = self.admin_command(Command {
                    cdw0: u32::from(ADMIN_DELETE_CQ),
                    cdw10: u32::from(id),
                    ..Command::default()
                });
                self.event_ids[usize::from(id)].store(0, Ordering::Release);
                self.event_count.store(u32::from(id), Ordering::Release);
                return Err(error);
            }
            self.io.push(Lease::new(queue)?);
        }
        Ok(())
    }

    fn queue(&self) -> Result<QueueGuard<'_, IoQueue>> {
        let first = ddk::cpu_current() as usize % self.io.len();
        let mut events = [self.io[0].ready; MAX_CPUS as usize];
        for (event, queue) in events.iter_mut().zip(&self.io) {
            *event = queue.ready;
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

    fn read_blocks(
        &self,
        namespace: &Namespace,
        lba: u64,
        blocks: u32,
        output: &mut [u8],
    ) -> Result<()> {
        let bytes = blocks as usize * namespace.block_size;
        if blocks == 0 || output.len() != bytes || blocks > u16::MAX.into() {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let mut queue = self.queue()?;
        queue.read(
            &self.registers,
            self.doorbell_stride,
            namespace.nsid,
            lba,
            blocks as u16,
            output,
        )
    }

    fn write_blocks(
        &self,
        namespace: &Namespace,
        lba: u64,
        blocks: u32,
        input: &[u8],
    ) -> Result<()> {
        let bytes = blocks as usize * namespace.block_size;
        if blocks == 0 || input.len() != bytes || blocks > u16::MAX.into() {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let mut queue = self.queue()?;
        queue.write(
            &self.registers,
            self.doorbell_stride,
            namespace.nsid,
            lba,
            blocks as u16,
            input,
        )
    }

    fn remove_blocks(&mut self) -> Result<()> {
        for namespace in self.namespaces.iter_mut().rev() {
            if let Some(mut registration) = namespace.block.take() {
                if let Err(error) = registration.unregister() {
                    namespace.block = Some(registration);
                    return Err(error);
                }
            }
        }
        Ok(())
    }

    fn scan_blocks(&self) -> Result<()> {
        for namespace in &self.namespaces {
            namespace
                .block
                .as_ref()
                .ok_or(Error::from_status(ddk::ENODEV))?
                .rescan()?;
        }
        Ok(())
    }

    fn disable(&self) {
        let command = self.registers.read32(CC);
        if command & CC_ENABLE != 0 {
            self.registers.write32(CC, command & !CC_ENABLE);
            let _ = wait_ready(&self.registers, false, self.timeout_ns);
        }
    }
}

impl Drop for Controller {
    fn drop(&mut self) {
        self.disable();
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

unsafe extern "C" fn interrupt(context: *mut c_void, virq: u32) -> u32 {
    if context.is_null() {
        return ddk::IRQ_NONE;
    }
    // SAFETY: every IRQ receipt is released before the boxed controller and
    // its event slots are destroyed.
    let controller = unsafe { &*context.cast::<Controller>() };
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

fn wait_ready(registers: &Mmio, ready: bool, timeout_ns: u64) -> Result<()> {
    let started = ddk::monotonic_ns();
    loop {
        let status = registers.read32(CSTS);
        if status & CSTS_FATAL != 0 {
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

fn namespace(pointer: *mut c_void) -> Option<&'static Namespace> {
    if pointer.is_null() {
        None
    } else {
        // SAFETY: block registration stores a pointer to a boxed namespace and
        // is withdrawn before the box is released.
        Some(unsafe { &*pointer.cast::<Namespace>() })
    }
}

fn controller(namespace: &Namespace) -> &'static Controller {
    // SAFETY: the namespace is owned by this controller and every namespace
    // registration is withdrawn before the controller is destroyed.
    unsafe { &*(namespace.controller as *const Controller) }
}

unsafe extern "C" fn block_read(
    context: *mut c_void,
    lba: u64,
    blocks: u32,
    output: *mut u8,
) -> i32 {
    let Some(namespace) = namespace(context) else {
        return ddk::EINVAL;
    };
    let Some(length) = (blocks as usize).checked_mul(namespace.block_size) else {
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

unsafe extern "C" fn block_write(
    context: *mut c_void,
    lba: u64,
    blocks: u32,
    input: *const u8,
) -> i32 {
    let Some(namespace) = namespace(context) else {
        return ddk::EINVAL;
    };
    let Some(length) = (blocks as usize).checked_mul(namespace.block_size) else {
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
static MSI_BINDING: TicketLock<Option<InterfaceBinding>> = TicketLock::new(None);

#[cfg(target_arch = "x86_64")]
fn with_msi<T>(call: impl FnOnce(&PciMsiOps, *mut c_void) -> Result<T>) -> Result<T> {
    let binding = MSI_BINDING.lock();
    let binding = binding.as_ref().ok_or(Error::from_status(ddk::ENOTSUP))?;
    // SAFETY: the binding keeps the provider and its immutable v1 operation
    // table live for the duration of this call.
    let operations = unsafe { &*binding.operations().cast::<PciMsiOps>() };
    if operations.size < size_of::<PciMsiOps>() as u32 {
        return Err(Error::from_status(ddk::ENOTSUP));
    }
    call(operations, binding.context())
}

#[cfg(target_arch = "x86_64")]
fn pci_msi_enable(device: Device) -> Result<Vec<u32>> {
    let requested = ddk::cpu_count().clamp(1, MAX_CPUS) + 1;
    let mut virqs = [0u32; MAX_CPUS as usize + 1];
    let mut actual = 0u32;
    with_msi(|operations, context| {
        let enable = operations.enable.ok_or(Error::from_status(ddk::ENOTSUP))?;
        // SAFETY: the output array has `requested` entries and the binding
        // guarantees this operation table's ABI and lifetime.
        let status = unsafe {
            enable(
                context,
                device.as_raw(),
                requested,
                virqs.as_mut_ptr(),
                &raw mut actual,
            )
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

#[cfg(target_arch = "x86_64")]
fn pci_msi_unmask(device: Device) -> Result<()> {
    with_msi(|operations, context| {
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

#[cfg(target_arch = "x86_64")]
fn pci_msi_disable(device: Device) {
    let _ = with_msi(|operations, context| {
        let disable = operations.disable.ok_or(Error::from_status(ddk::ENOTSUP))?;
        // SAFETY: disabling is idempotent for a PCI function owned by us.
        unsafe { disable(context, device.as_raw()) };
        Ok(())
    });
}

unsafe extern "C" fn probe(
    _context: *mut c_void,
    pointer: *const raw::Device,
    _match_data: usize,
) -> i32 {
    // SAFETY: the PCI bus supplies a live device during probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };
    let result = (|| -> Result<()> {
        #[cfg(target_arch = "x86_64")]
        let virqs = match pci_msi_enable(device) {
            Ok(virqs) => virqs,
            Err(error) => {
                MSI_REJECTED.store(1, Ordering::Release);
                ddk::log(
                    ddk::LOG_ERROR,
                    c"NVMe requires MSI-X or MSI on x86_64; refusing polling/INTx fallback",
                );
                return Err(error);
            }
        };
        #[cfg(target_arch = "riscv64")]
        let virqs = match ddk::irq_of_device(device, 0) {
            Ok(virq) => alloc::vec![virq],
            Err(error) => {
                ddk::log(
                    ddk::LOG_ERROR,
                    c"NVMe requires a PLIC-routed PCI interrupt on RISC-V",
                );
                return Err(error);
            }
        };
        let resource = device.resource(RESOURCE_MEMORY, 0)?;
        if resource.length < DOORBELL_BASE as u64 + 8 {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let registers = Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE)?;
        let mut controller = match Controller::create(device, registers) {
            Ok(controller) => controller,
            Err(error) => {
                #[cfg(target_arch = "x86_64")]
                pci_msi_disable(device);
                return Err(error);
            }
        };
        #[cfg(target_arch = "x86_64")]
        {
            controller.msi_active = true;
        }
        controller.request_interrupts(&virqs)?;
        #[cfg(target_arch = "x86_64")]
        pci_msi_unmask(device)?;
        let controller_id = NEXT_CONTROLLER.fetch_add(1, Ordering::Relaxed);
        if let Err(error) = controller.configure(controller_id) {
            if controller.remove_blocks().is_err() {
                // A concurrently mounted device still owns this context. Leaking
                // is safer than invalidating its callbacks; the module lease
                // held by that endpoint also prevents code unload.
                let _ = Box::leak(controller);
            }
            return Err(error);
        }
        let pointer = Box::into_raw(controller);
        // SAFETY: the box now remains owned through this device-private pointer
        // until the matching remove callback reconstructs it.
        if let Err(error) = unsafe { device.set_data(pointer.cast()) } {
            // SAFETY: set_data failed, so ownership was not transferred.
            let mut controller = unsafe { Box::from_raw(pointer) };
            if controller.remove_blocks().is_err() {
                let _ = Box::leak(controller);
            }
            return Err(error);
        }
        // SAFETY: device data now owns the controller at its final address, so
        // partition scanning may re-enter the immutable block callbacks.
        unsafe { &*pointer }.scan_blocks()?;
        #[cfg(target_arch = "x86_64")]
        ATTACHED_CONTROLLERS.fetch_add(1, Ordering::AcqRel);
        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

unsafe extern "C" fn remove(_context: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the driver core supplies the same live device used for probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };
    // SAFETY: only this driver stores a pointer in its bound device.
    let controller = unsafe { device.data() }.cast::<Controller>();
    if controller.is_null() {
        return;
    }
    // SAFETY: probe transferred a unique box and device removal serializes this
    // callback against another remove. Keep ownership in device data until all
    // externally reachable namespace contexts have been revoked.
    let controller_ref = unsafe { &mut *controller };
    if controller_ref.remove_blocks().is_err() {
        return;
    }
    // SAFETY: clearing prevents any later callback from claiming this box.
    let _ = unsafe { device.set_data(ptr::null_mut()) };
    #[cfg(target_arch = "x86_64")]
    ATTACHED_CONTROLLERS.fetch_sub(1, Ordering::AcqRel);
    // SAFETY: every node is gone, so no callback can retain a context pointer.
    drop(unsafe { Box::from_raw(controller) });
}

unsafe extern "C" fn shutdown(_context: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the driver core supplies a live bound device.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };
    // SAFETY: probe stored a controller pointer for the binding lifetime.
    let controller = unsafe { device.data() }.cast::<Controller>();
    if !controller.is_null() {
        // SAFETY: shutdown is serialized against removal by the driver core.
        unsafe { &*controller }.disable();
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

static DRIVER: TicketLock<raw::DriverDef> = TicketLock::new(raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"nvme".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: Some(remove),
    shutdown: Some(shutdown),
    context: ptr::null_mut(),
});

static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

fn init_inner() -> Result<()> {
    #[cfg(target_arch = "x86_64")]
    {
        MSI_REJECTED.store(0, Ordering::Release);
        ATTACHED_CONTROLLERS.store(0, Ordering::Release);
        let binding = InterfaceBinding::bind(c"pci.msi", 1).inspect_err(|_error| {
            ddk::log(
                ddk::LOG_ERROR,
                c"NVMe cannot load on x86_64 without the PCI MSI-X/MSI service",
            );
        })?;
        // SAFETY: the binding owns a provider reference and the table size is
        // checked again at every use.
        let operations = unsafe { &*binding.operations().cast::<PciMsiOps>() };
        if operations.size < size_of::<PciMsiOps>() as u32
            || operations.enable.is_none()
            || operations.unmask.is_none()
            || operations.disable.is_none()
        {
            ddk::log(
                ddk::LOG_ERROR,
                c"NVMe cannot load: PCI MSI-X/MSI service is incomplete",
            );
            return Err(Error::from_status(ddk::ENOTSUP));
        }
        *MSI_BINDING.lock() = Some(binding);
    }
    let bus = Bus::find(c"pci")?;
    let definition = {
        let mut driver = DRIVER.lock_irqsave();
        driver.bus = bus.as_raw();
        let pointer = ptr::from_ref(&*driver);
        // SAFETY: `DRIVER` is static and registration prevents mutation until
        // its receipt is unregistered during module exit.
        unsafe { &*pointer }
    };
    // SAFETY: the definition and match table have static storage duration.
    let registration = unsafe { DriverRegistration::register(definition) }?;
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

fn init(module: Module) -> Result<()> {
    let result = init_inner();
    if result.is_err() {
        exit(module);
    }
    result
}

fn exit(_module: Module) {
    if let Some(mut registration) = REGISTRATION.lock_irqsave().take() {
        let _ = registration.unregister();
    }
    #[cfg(target_arch = "x86_64")]
    MSI_BINDING.lock().take();
}

ddk::module!(
    b"nvme\0",
    b"Queue-per-core NVM Express block driver\0",
    init,
    exit,
);
