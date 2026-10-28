//! Coherent xHCI rings. Register and DMA ordering is explicit on RISC-V.
#![allow(clippy::cast_ptr_alignment)] // DMA allocations satisfy context and TRB alignment.
use core::ptr;
use ddk::{Device, Dma, Result};

pub const ENTRIES: usize = 256;
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Trb {
    pub parameter: u64,
    pub status: u32,
    pub control: u32,
}
const _: () = assert!(core::mem::size_of::<Trb>() == 16);
impl Trb {
    pub fn kind(self) -> u32 {
        (self.control >> 10) & 0x3f
    }
}

pub use ddk::device_barrier as barrier;

pub struct Ring {
    pub memory: Dma,
    index: usize,
    cycle: bool,
}
impl Ring {
    pub fn new(device: Device, flags: u32) -> Result<Self> {
        Ok(Self {
            memory: Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?,
            index: 0,
            cycle: true,
        })
    }
    pub fn address(&self) -> u64 {
        self.memory.device_address()
    }
    pub fn push(&mut self, mut trb: Trb) -> u64 {
        trb.control = (trb.control & !1) | u32::from(self.cycle);
        let address = self.address() + (self.index * 16) as u64;
        // SAFETY: producer ownership and one outstanding TD per endpoint keep
        // this slot idle; the cycle bit publishes the fully written TRB.
        unsafe {
            write_trb(self.memory.as_ptr().cast::<Trb>().add(self.index), trb);
        }
        self.index += 1;
        if self.index == ENTRIES - 1 {
            let link = Trb {
                parameter: self.address(),
                status: 0,
                control: (6 << 10) | 2 | u32::from(self.cycle),
            };
            // SAFETY: the final slot is reserved exclusively for the link TRB.
            unsafe {
                write_trb(self.memory.as_ptr().cast::<Trb>().add(self.index), link);
            }
            self.index = 0;
            self.cycle = !self.cycle;
        }
        barrier();
        address
    }
}
pub struct EventRing {
    pub memory: Dma,
    pub table: Dma,
    index: usize,
    cycle: bool,
}
impl EventRing {
    pub fn new(device: Device, flags: u32) -> Result<Self> {
        let memory = Dma::allocate(Some(device), 4096, 4096, flags | ddk::DMA_ZERO)?;
        let table = Dma::allocate(Some(device), 64, 64, flags | ddk::DMA_ZERO)?;
        // SAFETY: the aligned table has one initialized 16-byte ERST entry.
        unsafe {
            ptr::write(table.as_ptr().cast::<u64>(), memory.device_address());
            ptr::write(table.as_ptr().add(8).cast::<u32>(), ENTRIES as u32);
        }
        Ok(Self {
            memory,
            table,
            index: 0,
            cycle: true,
        })
    }
    pub fn pop(&mut self) -> Option<Trb> {
        // SAFETY: index stays within the coherent event segment.
        let entry = unsafe { self.memory.as_ptr().cast::<Trb>().add(self.index) };
        // SAFETY: volatile status load observes the controller's publication.
        if unsafe { ptr::read_volatile(ptr::addr_of!((*entry).control)) } & 1
            != u32::from(self.cycle)
        {
            return None;
        }
        barrier();
        // SAFETY: a matching cycle publishes the full event.
        let trb = unsafe { ptr::read_volatile(entry) };
        self.index += 1;
        if self.index == ENTRIES {
            self.index = 0;
            self.cycle = !self.cycle;
        }
        Some(trb)
    }
    pub fn dequeue(&self) -> u64 {
        self.memory.device_address() + (self.index * 16) as u64
    }
}

/// # Safety
/// entry must point to an idle, aligned slot in the producer-owned ring.
unsafe fn write_trb(entry: *mut Trb, trb: Trb) {
    // SAFETY: upheld by the caller; control is published after payload stores.
    unsafe {
        ptr::write_volatile(ptr::addr_of_mut!((*entry).parameter), trb.parameter);
        ptr::write_volatile(ptr::addr_of_mut!((*entry).status), trb.status);
        barrier();
        ptr::write_volatile(ptr::addr_of_mut!((*entry).control), trb.control);
    }
}
