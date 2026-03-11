//!
//! # Memory Subsystem
//!
//! Responsible for all things memory-related, such as physical/virtual
//! memory management, allocations, and TLB control.
//!

use bitflags::bitflags;
use limine::{
    memory_map::Entry,
    request::{HhdmRequest, MemoryMapRequest},
};
use log::info;

pub mod addr;
pub mod phys;

pub use addr::{align_down, align_up, pages_for_len, PhysAddr, VirtAddr, PAGE_SIZE};

bitflags! {
    /// Virtual memory mapping permissions and attributes used by low-level paging.
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    pub struct VmFlags: u32 {
        /// Permit reads.
        const READ    = 1 << 0;
        /// Permit writes.
        const WRITE   = 1 << 1;
        /// Permit instruction fetch.
        const EXECUTE = 1 << 2;
        /// User-accessible mapping.
        const USER    = 1 << 3;
        /// Global mapping that should survive context switches.
        const GLOBAL  = 1 << 4;
        /// Device/uncached style mapping.
        const DEVICE  = 1 << 5;
    }
}

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static HHDM_REQUEST: HhdmRequest = HhdmRequest::new();

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static MEMORY_MAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

/// Performs early memory subsystem initialization.
pub fn early() {
    info!("mem: hhdm=0x{:x}", hhdm_offset());
    phys::init();
}

/// Returns HHDM offset provided by Limine.
pub fn hhdm_offset() -> u64 {
    HHDM_REQUEST
        .get_response()
        .expect("mem: Limine HHDM response missing")
        .offset()
}

/// Returns Limine memory-map entries by reference.
pub fn memory_map_entries() -> &'static [&'static Entry] {
    MEMORY_MAP_REQUEST
        .get_response()
        .expect("mem: Limine memory map response missing")
        .entries()
}

/// Converts physical address to HHDM virtual address.
pub fn phys_to_virt(pa: PhysAddr) -> VirtAddr {
    let raw = pa
        .as_u64()
        .checked_add(hhdm_offset())
        .expect("mem: HHDM conversion overflow");
    VirtAddr::new(raw)
}

/// Converts HHDM virtual address back to physical address.
pub fn virt_to_phys_hhdm(va: VirtAddr) -> Option<PhysAddr> {
    let raw = va.as_u64();
    let off = hhdm_offset();
    if raw < off {
        return None;
    }

    Some(PhysAddr::new(raw - off))
}
