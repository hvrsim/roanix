//!
//! # xHCI Controller and Transfers
//!
//! This module owns the controller's register mapping, coherent rings,
//! and per-slot endpoint state. The hardware lock protects register and
//! ring updates shared by thread code and the interrupt handler.
//!
//! The transaction gate serializes command sequences and control transfers.
//! Completion waits release the hardware lock. A transfer retains endpoint
//! admission until its caller has copied the completed DMA data.
//!

#![allow(clippy::cast_ptr_alignment)] // DMA allocations satisfy context and TRB alignment.
use crate::ring::{self, EventRing, Ring, Trb};
use alloc::{boxed::Box, sync::Arc, vec, vec::Vec};
use core::{
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};
use ddk::{
    Device, DeviceRegistration, Dma, Error, Event, Irq, Mmio, OwnedEvent, Result, TicketLock,
    Worker,
};

/// Time limit for controller commands and USB control requests.
pub const TIMEOUT: u64 = 2_000_000_000;

/// Transfer admission held before a TRB is published.
pub const RESERVED_TRANSFER: u64 = 1;

/// Returns the generic I/O error for an unsuccessful xHCI operation.
pub fn error() -> Error {
    Error::from_status(ddk::EIO)
}

/// Sleepable admission gate for one controller transaction at a time.
pub struct Gate {
    /// Atomic ownership flag for the current transaction.
    busy: AtomicBool,

    /// Event signaled when a transaction releases admission.
    ready: OwnedEvent,
}

/// Exclusive transaction admission released on drop.
pub struct Guard<'a>(&'a Gate);
impl Gate {
    fn new() -> Result<Self> {
        Ok(Self {
            busy: AtomicBool::new(false),
            ready: OwnedEvent::create()?,
        })
    }

    /// Waits for exclusive admission to a controller transaction.
    pub fn lock(&self) -> Result<Guard<'_>> {
        loop {
            self.ready.reset()?;
            if self
                .busy
                .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
            {
                return Ok(Guard(self));
            }
            self.ready.wait()?;
        }
    }
}

impl Drop for Guard<'_> {
    /// Releases transaction admission before waking another waiter.
    fn drop(&mut self) {
        self.0.busy.store(false, Ordering::Release);
        let _ = self.0.ready.signal();
    }
}

/// Coherent endpoint storage shared temporarily with the admitted caller.
pub struct TransferBuffer(pub Dma);

// SAFETY: endpoint admission gives one caller exclusive access to the bytes.
// The hardware starts after the OUT copy and completes before the IN copy.
unsafe impl Sync for TransferBuffer {}

/// DMA and completion state for the single transfer admitted to an endpoint.
pub struct Endpoint {
    /// Producer ring for the endpoint's transfer requests.
    pub ring: Ring,

    /// Coherent storage used for the current transfer.
    pub buffer: Arc<TransferBuffer>,

    /// Notification for completion or cancellation.
    pub event: OwnedEvent,

    /// Completion TRB address, zero when idle or reserved while staging a transfer.
    pub pending: u64,

    /// Completion code once the interrupt handler records the final event.
    completion_code: Option<u32>,

    /// Number of bytes submitted for the current transfer.
    pub requested: usize,

    /// Number of bytes made available by completion.
    pub received: usize,

    /// Whether a STALL must be recovered before the next transfer.
    pub halted: bool,

    /// Whether the endpoint is being torn down.
    pub cancelled: bool,

    /// Whether the controller has accepted this endpoint context.
    pub configured: bool,
}

impl Endpoint {
    /// Allocates the transfer ring, coherent buffer, and completion event for an endpoint.
    pub fn new(device: Device, flags: u32, size: usize) -> Result<Self> {
        // Align the device address so a single Normal TRB fits in one 64 KiB window.
        let buffer = Dma::allocate(Some(device), size, size, flags | ddk::DMA_ZERO)?;
        if buffer.device_address() & (size as u64 - 1) != 0 {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        Ok(Self {
            ring: Ring::new(device, flags)?,
            buffer: Arc::new(TransferBuffer(buffer)),
            event: OwnedEvent::create()?,
            pending: 0,
            completion_code: None,
            requested: 0,
            received: 0,
            halted: false,
            cancelled: false,
            configured: false,
        })
    }

    /// Admits one transfer and resets its completion state while the hardware lock is held.
    pub fn prepare(&mut self, length: usize) -> Result<Event> {
        if length > self.buffer.0.len() {
            return Err(Error::from_status(ddk::EINVAL));
        }

        if self.cancelled {
            return Err(Error::from_status(ddk::ENODEV));
        }

        if self.halted {
            return Err(Error::from_status(usb::STALL));
        }

        if self.pending != 0 {
            return Err(Error::from_status(ddk::EBUSY));
        }
        self.event.reset()?;
        self.completion_code = None;
        self.requested = length;
        self.received = length;
        Ok(self.event.event())
    }

    /// Records a transfer event and wakes the waiter once the transfer terminates.
    pub fn complete(&mut self, event: Trb, control: bool) {
        if self.pending <= RESERVED_TRANSFER {
            return;
        }

        let code = event.status >> 24;
        let final_trb = event.parameter & !15 == self.pending;

        // A short control data stage may precede the final status stage.
        if code == 13 || (final_trb && code == 1 && !control) {
            let remaining = (event.status & 0xff_ffff) as usize;
            self.received = self.requested.saturating_sub(remaining);
        }

        if final_trb || !matches!(code, 1 | 13) {
            self.completion_code = Some(code);
            let _ = self.event.signal();
        }
    }
}

/// Controller contexts and USB child registrations for one addressed device.
pub struct Slot {
    /// Device context written by the controller.
    pub context: Dma,

    /// Input context prepared by software for configure commands.
    pub input: Dma,

    /// Transfer state indexed by xHCI endpoint ID.
    pub endpoints: [Option<Endpoint>; 32],

    /// Receipts for USB interfaces published from this slot.
    pub children: Vec<DeviceRegistration>,

    /// Whether transfers may still enter the slot.
    pub online: bool,

    /// Port speed used when building endpoint contexts.
    pub speed: u32,

    /// Bitmap of cancelled endpoint IDs, including unconfigured endpoints.
    pub cancelled: u32,
}

impl Slot {
    /// Allocates slot contexts and installs the default control endpoint.
    pub fn new(device: Device, flags: u32, speed: u32) -> Result<Box<Self>> {
        let context = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
        let input = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
        let mut ep0 = Endpoint::new(device, flags, 4096)?;
        ep0.configured = true;
        let mut endpoints = core::array::from_fn(|_| None);
        endpoints[1] = Some(ep0);
        Ok(Box::new(Self {
            context,
            input,
            endpoints,
            children: Vec::new(),
            online: true,
            speed,
            cancelled: 0,
        }))
    }

    /// Rejects new transfers and wakes waiters before device registrations are withdrawn.
    pub fn retire(&mut self) {
        self.online = false;
        for endpoint in self.endpoints.iter_mut().flatten() {
            endpoint.cancelled = true;
            let _ = endpoint.event.signal();
        }
    }
}

/// Registers, rings, and slot state shared with the interrupt handler.
pub struct Hardware {
    /// Mapped xHCI register space.
    pub registers: Mmio,

    /// Offset of the operational register block.
    pub op: usize,

    /// Offset of the primary interrupter's registers.
    pub runtime: usize,

    /// Offset of the doorbell array.
    pub doorbell: usize,

    /// Hardware-selected size of one context entry.
    pub context_size: usize,

    /// DMA constraints used for controller buffers.
    pub flags: u32,

    /// Number of hardware root ports.
    pub max_ports: usize,

    /// Device Context Base Address Array programmed into the controller.
    pub dcbaa: Dma,

    /// Software-owned command ring.
    pub command: Ring,

    /// Controller-produced event ring.
    pub events: EventRing,

    /// Device state indexed by hardware slot ID.
    pub slots: Vec<Option<Box<Slot>>>,

    /// Slot IDs assigned to root ports, with zero for empty ports.
    pub ports: Vec<u8>,

    /// Device address of the command awaiting completion.
    pub command_address: u64,

    /// Completion event captured by the IRQ handler.
    pub command_result: Option<Trb>,

    /// Buffers and table retained while hardware uses its scratchpad pointers.
    _scratch: Vec<Dma>,
}

impl Hardware {
    /// Resets the controller and allocates the registers, rings, and contexts needed to start
    /// it.
    pub fn create(device: Device) -> Result<Self> {
        let resource = ddk::pci::memory_bar(device, 0)?;
        let registers = Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE)?;
        if registers.len() < 0x400 {
            return Err(error());
        }

        let op = usize::from(registers.read8(0));
        let hcs1 = registers.read32(4);
        let hcs2 = registers.read32(8);
        let hcc = registers.read32(0x10);
        let max_slots = (hcs1 as usize & 0xff).min(32);
        let max_ports = (hcs1 >> 24) as usize;
        let doorbell = (registers.read32(0x14) & !3) as usize;
        let runtime = (registers.read32(0x18) & !31) as usize + 0x20;
        if max_slots == 0
            || max_ports == 0
            || op < 0x20
            || op + 0x400 + max_ports * 16 > registers.len()
            || runtime + 0x20 > registers.len()
            || doorbell + (max_slots + 1) * 4 > registers.len()
        {
            return Err(error());
        }
        claim_firmware_ownership(&registers, hcc)?;

        registers.write32(op, registers.read32(op) & !1);
        wait(&registers, op + 4, 1, 1)?;
        registers.write32(op, 2);
        wait(&registers, op, 2, 0)?;
        wait(&registers, op + 4, 1 << 11, 0)?;
        if registers.read32(op + 8) & 1 == 0 {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        let flags = if hcc & 1 == 0 { ddk::DMA_ADDRESS32 } else { 0 };
        let dcbaa = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
        let command = Ring::new(device, flags)?;
        let events = EventRing::new(device, flags)?;
        let count = (((hcs2 >> 21) & 31) << 5 | ((hcs2 >> 27) & 31)) as usize;
        let mut scratch = Vec::new();
        if count != 0 {
            let table = Dma::allocate(Some(device), count * 8, 64, flags | ddk::DMA_ZERO)?;
            for index in 0..count {
                let page = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
                put64(&table, index, page.device_address());
                scratch.push(page);
            }
            put64(&dcbaa, 0, table.device_address());
            scratch.push(table);
        }

        Ok(Self {
            registers,
            op,
            runtime,
            doorbell,
            context_size: if hcc & 4 != 0 { 64 } else { 32 },
            flags,
            max_ports,
            dcbaa,
            command,
            events,
            slots: (0..=max_slots).map(|_| None).collect(),
            ports: vec![0; max_ports],
            command_address: 0,
            command_result: None,
            _scratch: scratch,
        })
    }

    /// Programs the prepared DMA structures and starts the interrupter and controller.
    pub fn start(&self) {
        for port in 0..self.max_ports {
            self.registers.write32(self.portsc(port), 1 << 9);
        }
        self.registers
            .write64(self.op + 0x30, self.dcbaa.device_address());
        self.registers
            .write64(self.op + 0x18, self.command.address() | 1);
        self.registers
            .write32(self.op + 0x38, (self.slots.len() - 1) as u32);
        self.registers.write32(self.runtime + 8, 1);
        self.registers
            .write64(self.runtime + 0x10, self.events.table.device_address());
        self.registers
            .write64(self.runtime + 0x18, self.events.dequeue());
        self.registers.write32(self.runtime + 4, 0);
        self.registers.write32(self.runtime, 3);
        ring::barrier();
        self.registers.write32(self.op, 1 | 4);
    }

    /// Returns the PORTSC offset for a one-based xHCI port number.
    pub fn portsc(&self, port: usize) -> usize {
        self.op + 0x400 + port * 16
    }

    /// Finds a configured endpoint in an online slot or returns a device error.
    pub fn endpoint_mut(&mut self, slot: usize, dci: usize) -> Result<&mut Endpoint> {
        self.slots
            .get_mut(slot)
            .and_then(Option::as_mut)
            .filter(|s| s.online)
            .and_then(|s| s.endpoints.get_mut(dci))
            .and_then(Option::as_mut)
            .ok_or(Error::from_status(ddk::ENODEV))
    }
}

/// An xHCI controller with its worker, interrupt, and transaction admission.
pub struct Controller {
    /// Driver-issued ID used to find the host from a USB child.
    pub id: u32,

    /// Owning PCI function.
    pub device: Device,

    /// IRQ-safe register and ring state.
    pub hardware: TicketLock<Hardware>,

    /// Admission gate for serialized controller operations.
    pub transaction: Gate,

    /// Notification from the command completion interrupt.
    pub command_event: OwnedEvent,

    /// Notification for root-port work.
    pub port_event: OwnedEvent,

    /// Flag telling the root-port worker to exit.
    pub stopping: AtomicBool,

    /// Flag preventing new commands after a controller failure.
    pub failed: AtomicBool,

    /// IRQ action receipt retained until teardown.
    pub irq: TicketLock<Option<Irq>>,

    /// Root-port worker receipt retained until join.
    pub worker: TicketLock<Option<Worker>>,

    /// PCI message interrupt configuration on x86.
    #[cfg(target_arch = "x86_64")]
    pub msi: crate::pci::Msi,
}

impl Controller {
    /// Creates the controller instance and reserves its command and port notifications.
    pub fn new(device: Device, id: u32) -> Result<Arc<Self>> {
        let hardware = Hardware::create(device)?;
        let transaction = Gate::new()?;
        let command_event = OwnedEvent::create()?;
        let port_event = OwnedEvent::create()?;
        #[cfg(target_arch = "x86_64")]
        let msi = crate::pci::Msi::new(device)?;
        Ok(Arc::new(Self {
            id,
            device,
            hardware: TicketLock::new(hardware),
            transaction,
            command_event,
            port_event,
            stopping: AtomicBool::new(false),
            failed: AtomicBool::new(false),
            irq: TicketLock::new(None),
            worker: TicketLock::new(None),
            #[cfg(target_arch = "x86_64")]
            msi,
        }))
    }

    /// Submits one command and waits for completion. The caller must hold the transaction gate.
    pub fn command(&self, trb: Trb) -> Result<Trb> {
        if self.failed.load(Ordering::Acquire) {
            return Err(error());
        }
        self.command_event.reset()?;
        {
            let mut h = self.hardware.lock_irqsave();
            h.command_result = None;
            h.command_address = h.command.push(trb);
            h.registers.write32(h.doorbell, 0);
        }
        self.command_event.wait_timeout(TIMEOUT)?;
        let result = { self.hardware.lock_irqsave().command_result.take() };
        let Some(result) = result else {
            self.failed.store(true, Ordering::Release);
            return Err(error());
        };

        if result.status >> 24 != 1 {
            return Err(error());
        }

        Ok(result)
    }

    /// Waits for completion and keeps admission until the caller finishes its DMA copy.
    pub fn wait_transfer(
        &self,
        slot: usize,
        dci: usize,
        event: Event,
        timeout: Option<u64>,
    ) -> Result<usize> {
        let started = ddk::monotonic_ns();
        loop {
            let elapsed = ddk::monotonic_ns().wrapping_sub(started);
            {
                let mut h = self.hardware.lock_irqsave();
                let endpoint = h.endpoint_mut(slot, dci)?;
                if endpoint.cancelled {
                    return Err(Error::from_status(ddk::ENODEV));
                }

                if let Some(code) = endpoint.completion_code {
                    if matches!(code, 1 | 13) {
                        return Ok(endpoint.received);
                    }
                    endpoint.pending = 0;
                    endpoint.halted = code == 6;
                    endpoint.cancelled = code != 6;
                    return Err(if endpoint.halted {
                        Error::from_status(usb::STALL)
                    } else {
                        error()
                    });
                }

                if timeout.is_some_and(|limit| elapsed >= limit) {
                    // The controller may still own the TD and its DMA buffer.
                    endpoint.cancelled = true;
                    return Err(error());
                }
            }

            if let Err(failure) = event.wait_timeout(timeout.map_or(1_000_000_000, |n| n - elapsed))
            {
                if let Ok(endpoint) = self.hardware.lock_irqsave().endpoint_mut(slot, dci) {
                    endpoint.cancelled = true;
                }

                return Err(failure);
            }
        }
    }

    /// Restarts a stalled xHCI endpoint at the next software-owned TRB.
    ///
    /// The caller holds the controller transaction gate across these commands.
    pub fn recover_halt(&self, slot: usize, dci: usize) -> Result<()> {
        let dequeue = {
            let mut h = self.hardware.lock_irqsave();
            let endpoint = h.endpoint_mut(slot, dci)?;
            if endpoint.cancelled {
                return Err(Error::from_status(ddk::ENODEV));
            }

            if !endpoint.halted {
                return Ok(());
            }
            endpoint.ring.producer()
        };

        let target = ((slot as u32) << 24) | ((dci as u32) << 16);
        let recovery = self
            .command(Trb {
                control: (14 << 10) | target,
                ..Trb::default()
            })
            .and_then(|_| {
                self.command(Trb {
                    parameter: dequeue,
                    control: (16 << 10) | target,
                    ..Trb::default()
                })
            });
        let mut h = self.hardware.lock_irqsave();
        let endpoint = h.endpoint_mut(slot, dci)?;
        if recovery.is_err() {
            endpoint.cancelled = true;
        } else {
            endpoint.halted = false;
        }
        recovery.map(|_| ())
    }

    /// Builds a control transfer and copies a completed IN payload into the caller's buffer.
    pub fn control(&self, slot: usize, setup: u64, buffer: &mut [u8]) -> Result<usize> {
        if buffer.len() > 4096 || (setup >> 48) as usize != buffer.len() {
            return Err(Error::from_status(ddk::EINVAL));
        }

        let input = setup & 0x80 != 0;
        let event = {
            let mut h = self.hardware.lock_irqsave();
            let endpoint = h.endpoint_mut(slot, 1)?;
            let event = endpoint.prepare(buffer.len())?;
            if !input && !buffer.is_empty() {
                // SAFETY: endpoint DMA storage has room for this bounded slice.
                unsafe {
                    ptr::copy_nonoverlapping(
                        buffer.as_ptr(),
                        endpoint.buffer.0.as_ptr(),
                        buffer.len(),
                    );
                }
            }
            endpoint.ring.push(Trb {
                parameter: setup,
                status: 8,
                control: (2 << 10)
                    | (1 << 6)
                    | (if buffer.is_empty() {
                        0
                    } else if input {
                        3
                    } else {
                        2
                    } << 16),
            });
            if !buffer.is_empty() {
                endpoint.ring.push(Trb {
                    parameter: endpoint.buffer.0.device_address(),
                    status: buffer.len() as u32,
                    control: (3 << 10) | (u32::from(input) << 16) | 4,
                });
            }
            endpoint.pending = endpoint.ring.push(Trb {
                parameter: 0,
                status: 0,
                control: (4 << 10) | (1 << 5) | (u32::from(buffer.is_empty() || !input) << 16),
            });
            h.registers.write32(h.doorbell + slot * 4, 1);
            event
        };

        let count = match self.wait_transfer(slot, 1, event, Some(TIMEOUT)) {
            Err(failure) if failure.status() == usb::STALL => {
                self.recover_halt(slot, 1)?;
                return Err(failure);
            }
            result => result?,
        };

        let mut h = self.hardware.lock_irqsave();
        let endpoint = h.endpoint_mut(slot, 1)?;
        if input {
            // SAFETY: completion published count bytes within both buffers.
            unsafe {
                ptr::copy_nonoverlapping(endpoint.buffer.0.as_ptr(), buffer.as_mut_ptr(), count);
            }
        }
        endpoint.pending = 0;
        Ok(count)
    }

    /// Quiesces the slots before joining a worker that may be waiting for USB.
    pub fn retire_slots(&self) {
        for slot in self.hardware.lock_irqsave().slots.iter_mut().flatten() {
            slot.retire();
        }
    }
}

impl Drop for Controller {
    /// Releases IRQ delivery and halts DMA before the controller's buffers can be dropped.
    fn drop(&mut self) {
        if let Some(mut irq) = self.irq.get_mut().take() {
            let _ = irq.release();
        }

        let h = self.hardware.get_mut();
        h.registers.write32(h.op, 0);
        // Returning DMA pages to the allocator while the controller can still
        // write them would corrupt unrelated memory. This target aborts on
        // panic, so an unresponsive controller cannot reach field destruction.
        assert!(
            wait(&h.registers, h.op + 4, 1, 1).is_ok(),
            "xHCI failed to halt DMA"
        );
    }
}

// Claim ownership from firmware before resetting or touching DMA state.
fn claim_firmware_ownership(registers: &Mmio, hcc: u32) -> Result<()> {
    let mut extended = ((hcc >> 16) as usize) * 4;
    for _ in 0..256 {
        if extended == 0 {
            break;
        }

        if extended + 8 > registers.len() {
            return Err(error());
        }

        let capability = registers.read32(extended);
        if capability & 0xff == 1 {
            registers.write32(extended, capability | (1 << 24));
            wait(registers, extended, 1 << 16, 0)?;
            registers.write32(extended + 4, 0xe000_0000);
        }

        let next = ((capability >> 8) & 0xff) as usize * 4;
        if next == 0 {
            break;
        }
        extended += next;
    }

    Ok(())
}

/// Polls a masked register field until it reaches the expected value or times out.
pub fn wait(registers: &Mmio, offset: usize, mask: u32, value: u32) -> Result<()> {
    let start = ddk::monotonic_ns();
    loop {
        if registers.read32(offset) & mask == value {
            return Ok(());
        }

        if ddk::monotonic_ns().wrapping_sub(start) >= TIMEOUT {
            return Err(error());
        }
        ddk::sleep_ns(1_000_000);
    }
}

/// Stores one 32-bit xHCI context word in coherent DMA memory.
pub fn put32(memory: &Dma, index: usize, value: u32) {
    assert!((index + 1) * 4 <= memory.len());

    // SAFETY: bounds and DMA alignment cover this exclusively CPU-owned word.
    unsafe {
        ptr::write(memory.as_ptr().cast::<u32>().add(index), value);
    }
}

/// Loads one 32-bit xHCI context word from coherent DMA memory.
pub fn get32(memory: &Dma, index: usize) -> u32 {
    assert!((index + 1) * 4 <= memory.len());

    // SAFETY: bounds and DMA alignment cover this initialized context word.
    unsafe { ptr::read_volatile(memory.as_ptr().cast::<u32>().add(index)) }
}

/// Stores one 64-bit entry in the coherent xHCI pointer table.
pub fn put64(memory: &Dma, index: usize, value: u64) {
    assert!((index + 1) * 8 <= memory.len());

    // SAFETY: bounds and DMA alignment cover this exclusively CPU-owned word.
    unsafe {
        ptr::write(memory.as_ptr().cast::<u64>().add(index), value);
    }
}
