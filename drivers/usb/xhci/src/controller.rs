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

pub const TIMEOUT: u64 = 2_000_000_000;
pub fn error() -> Error {
    Error::from_status(ddk::EIO)
}
#[cfg(target_arch = "x86_64")]
pub fn status(value: i32) -> Result<()> {
    if value == ddk::OK {
        Ok(())
    } else {
        Err(Error::from_status(value))
    }
}

pub struct Gate {
    busy: AtomicBool,
    ready: OwnedEvent,
}
pub struct Guard<'a>(&'a Gate);
impl Gate {
    fn new() -> Result<Self> {
        Ok(Self {
            busy: AtomicBool::new(false),
            ready: OwnedEvent::create()?,
        })
    }
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
    fn drop(&mut self) {
        self.0.busy.store(false, Ordering::Release);
        let _ = self.0.ready.signal();
    }
}

pub struct Endpoint {
    pub ring: Ring,
    pub buffer: Dma,
    pub event: OwnedEvent,
    pub pending: u64,
    pub complete: bool,
    pub code: u32,
    pub requested: usize,
    pub received: usize,
    pub cancelled: bool,
    pub configured: bool,
}
impl Endpoint {
    pub fn new(device: Device, flags: u32) -> Result<Self> {
        Ok(Self {
            ring: Ring::new(device, flags)?,
            buffer: Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?,
            event: OwnedEvent::create()?,
            pending: 0,
            complete: false,
            code: 0,
            requested: 0,
            received: 0,
            cancelled: false,
            configured: false,
        })
    }
    pub fn prepare(&mut self, length: usize) -> Result<Event> {
        if self.cancelled {
            return Err(Error::from_status(ddk::ENODEV));
        }
        if self.pending != 0 {
            return Err(Error::from_status(ddk::EBUSY));
        }
        self.event.reset()?;
        self.complete = false;
        self.code = 0;
        self.requested = length;
        self.received = length;
        Ok(self.event.event())
    }
}

pub struct Slot {
    pub context: Dma,
    pub input: Dma,
    pub endpoints: [Option<Endpoint>; 32],
    pub children: Vec<DeviceRegistration>,
    pub online: bool,
    pub speed: u32,
    pub cancelled: u32,
}
impl Slot {
    pub fn new(device: Device, flags: u32, speed: u32) -> Result<Box<Self>> {
        let context = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
        let input = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
        let mut ep0 = Endpoint::new(device, flags)?;
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
}

pub struct Hardware {
    pub registers: Mmio,
    pub op: usize,
    pub runtime: usize,
    pub doorbell: usize,
    pub context_size: usize,
    pub flags: u32,
    pub max_ports: usize,
    pub dcbaa: Dma,
    pub command: Ring,
    pub events: EventRing,
    pub slots: Vec<Option<Box<Slot>>>,
    pub ports: Vec<u8>,
    pub command_address: u64,
    pub command_result: Option<Trb>,
    _scratch: Vec<Dma>,
}
impl Hardware {
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
        // Claim ownership from firmware before resetting or touching DMA state.
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
                wait(&registers, extended, 1 << 16, 0)?;
                registers.write32(extended + 4, 0xe000_0000);
            }
            let next = ((capability >> 8) & 0xff) as usize * 4;
            if next == 0 {
                break;
            }
            extended += next;
        }
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
    pub fn portsc(&self, port: usize) -> usize {
        self.op + 0x400 + port * 16
    }
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

pub struct Controller {
    pub id: u32,
    pub device: Device,
    pub hardware: TicketLock<Hardware>,
    pub transaction: Gate,
    pub command_event: OwnedEvent,
    pub port_event: OwnedEvent,
    pub stopping: AtomicBool,
    pub failed: AtomicBool,
    pub irq: TicketLock<Option<Irq>>,
    pub worker: TicketLock<Option<Worker>>,
    #[cfg(target_arch = "x86_64")]
    pub msi: crate::pci::Msi,
}
impl Controller {
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
    // Caller owns transaction across related enumeration/control commands.
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
    pub fn wait_transfer(&self, slot: usize, dci: usize, event: Event) -> Result<usize> {
        let started = ddk::monotonic_ns();
        loop {
            {
                let mut h = self.hardware.lock_irqsave();
                let endpoint = h.endpoint_mut(slot, dci)?;
                if endpoint.cancelled {
                    return Err(Error::from_status(ddk::ENODEV));
                }
                if endpoint.complete {
                    return if matches!(endpoint.code, 1 | 13) {
                        // The caller retains admission until its DMA copy finishes.
                        Ok(endpoint.received)
                    } else {
                        endpoint.pending = 0;
                        Err(error())
                    };
                }
            }
            let elapsed = ddk::monotonic_ns().wrapping_sub(started);
            // Interrupt IN may wait indefinitely for the next report; removal
            // cancels it. Control requests have a bounded device timeout.
            if dci == 1 && elapsed >= TIMEOUT {
                self.hardware.lock_irqsave().slots[slot]
                    .as_mut()
                    .ok_or_else(error)?
                    .online = false;
                return Err(error());
            }
            event.wait_timeout(if dci == 1 {
                TIMEOUT - elapsed
            } else {
                1_000_000_000
            })?;
        }
    }
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
                        endpoint.buffer.as_ptr(),
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
                    parameter: endpoint.buffer.device_address(),
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
        let count = self.wait_transfer(slot, 1, event)?;
        let mut h = self.hardware.lock_irqsave();
        let endpoint = h.endpoint_mut(slot, 1)?;
        if input {
            // SAFETY: completion published count bytes within both buffers.
            unsafe {
                ptr::copy_nonoverlapping(endpoint.buffer.as_ptr(), buffer.as_mut_ptr(), count);
            }
        }
        endpoint.pending = 0;
        Ok(count)
    }
    pub fn cancel(&self, slot: usize) {
        let mut h = self.hardware.lock_irqsave();
        if let Some(Some(slot)) = h.slots.get_mut(slot) {
            for endpoint in slot.endpoints.iter_mut().flatten() {
                endpoint.cancelled = true;
                let _ = endpoint.event.signal();
            }
        }
    }
}
impl Drop for Controller {
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
pub fn put32(memory: &Dma, index: usize, value: u32) {
    assert!((index + 1) * 4 <= memory.len());
    // SAFETY: bounds and DMA alignment cover this exclusively CPU-owned word.
    unsafe {
        ptr::write(memory.as_ptr().cast::<u32>().add(index), value);
    }
}
pub fn get32(memory: &Dma, index: usize) -> u32 {
    assert!((index + 1) * 4 <= memory.len());
    // SAFETY: bounds and DMA alignment cover this initialized context word.
    unsafe { ptr::read_volatile(memory.as_ptr().cast::<u32>().add(index)) }
}
pub fn put64(memory: &Dma, index: usize, value: u64) {
    assert!((index + 1) * 8 <= memory.len());
    // SAFETY: bounds and DMA alignment cover this exclusively CPU-owned word.
    unsafe {
        ptr::write(memory.as_ptr().cast::<u64>().add(index), value);
    }
}
