//! Logical memory pages backed by authoritative PFN database entries.

use alloc::sync::Arc;
use core::ptr;

use crate::{
    mem::{self, PAGE_SIZE, PhysAddr, phys, phys::PageOwnerKind},
    sys::sync::Mutex,
};

use super::{
    Error, Result,
    swap::{self, SwapHandle},
};

const PAGE_BYTES: usize = PAGE_SIZE as usize;

enum PageBacking {
    Zero,
    Resident(&'static phys::Page),
    Swapped(SwapHandle),
}

/// Current storage location of a managed page.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageLocation {
    /// The page is logically zero and consumes no physical frame.
    Zero,
    /// The page has a resident physical frame.
    Resident,
    /// The page is held by a swap tier.
    Swapped,
}

/// Snapshot of one page's reclaim-relevant state.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PageInfo {
    /// Stable managed-page identifier.
    pub id: u64,
    /// Page index within its object or anonymous map.
    pub index: u64,
    /// Current storage location.
    pub location: PageLocation,
    /// Number of page-table mappings referencing the resident frame.
    pub mappings: u32,
    /// Number of temporary or permanent wires.
    pub wires: u32,
    /// Whether the resident frame is dirty.
    pub dirty: bool,
    /// Whether the resident frame was recently referenced.
    pub referenced: bool,
}

/// Logical object or anonymous page that may be zero, resident, or swapped.
pub struct VmPage {
    id: u64,
    index: u64,
    owner_kind: PageOwnerKind,
    backing: Mutex<PageBacking>,
    permanent: bool,
}

impl VmPage {
    pub(super) fn new_zero(index: u64, owner_kind: PageOwnerKind) -> Arc<Self> {
        Arc::new(Self {
            id: super::allocate_page_id(),
            index,
            owner_kind,
            backing: Mutex::new(PageBacking::Zero),
            permanent: false,
        })
    }

    pub(super) fn new_shared_zero() -> Result<Arc<Self>> {
        let frame = super::allocate_physical_page()?;
        let page = Arc::new(Self {
            id: super::allocate_page_id(),
            index: 0,
            owner_kind: PageOwnerKind::Kernel,
            backing: Mutex::new(PageBacking::Resident(frame)),
            permanent: true,
        });
        frame.bind_owner(&page, PageOwnerKind::Kernel, page.id, 0);
        frame.wire();
        frame.mark_referenced();
        Ok(page)
    }

    /// Returns the stable page identifier.
    pub fn id(&self) -> u64 {
        self.id
    }

    /// Returns the page index within its owner.
    pub fn index(&self) -> u64 {
        self.index
    }

    /// Returns a state snapshot suitable for diagnostics.
    pub fn info(&self) -> PageInfo {
        match &*self.backing.lock() {
            PageBacking::Zero => PageInfo {
                id: self.id,
                index: self.index,
                location: PageLocation::Zero,
                mappings: 0,
                wires: 0,
                dirty: false,
                referenced: false,
            },
            PageBacking::Swapped(_) => PageInfo {
                id: self.id,
                index: self.index,
                location: PageLocation::Swapped,
                mappings: 0,
                wires: 0,
                dirty: false,
                referenced: false,
            },
            PageBacking::Resident(frame) => {
                let flags = frame.flags();
                PageInfo {
                    id: self.id,
                    index: self.index,
                    location: PageLocation::Resident,
                    mappings: frame.map_count(),
                    wires: frame.wire_count(),
                    dirty: flags.contains(phys::PageFlags::DIRTY),
                    referenced: flags.contains(phys::PageFlags::REFERENCED),
                }
            }
        }
    }

    /// Reads bytes from one page.
    pub fn read(self: &Arc<Self>, offset: usize, output: &mut [u8]) -> Result<()> {
        validate_range(offset, output.len())?;
        if output.is_empty() {
            return Ok(());
        }

        loop {
            let backing = self.backing.lock();
            match &*backing {
                PageBacking::Zero => {
                    output.fill(0);
                    return Ok(());
                }
                PageBacking::Resident(frame) => {
                    frame.mark_referenced();
                    // SAFETY: the backing lock prevents reclamation and the
                    // validated range remains within the HHDM-mapped frame.
                    unsafe {
                        ptr::copy_nonoverlapping(
                            mem::phys_to_virt(frame.paddr()).as_ptr::<u8>().add(offset),
                            output.as_mut_ptr(),
                            output.len(),
                        );
                    }
                    return Ok(());
                }
                PageBacking::Swapped(_) => drop(backing),
            }
            self.ensure_resident()?;
        }
    }

    /// Writes bytes into one page.
    pub fn write(self: &Arc<Self>, offset: usize, input: &[u8]) -> Result<()> {
        validate_range(offset, input.len())?;
        if input.is_empty() {
            return Ok(());
        }

        let (frame, became_resident) = {
            let mut backing = self.backing.lock();
            let became_resident = self.ensure_resident_locked(&mut backing)?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: resident page lost backing");
            };
            frame.mark_referenced();
            frame.mark_dirty();
            // SAFETY: the backing lock prevents reclamation and the validated
            // range remains within the HHDM-mapped frame.
            unsafe {
                ptr::copy_nonoverlapping(
                    input.as_ptr(),
                    mem::phys_to_virt(frame.paddr())
                        .as_mut_ptr::<u8>()
                        .add(offset),
                    input.len(),
                );
            }
            (*frame, became_resident)
        };
        if became_resident {
            phys::activate_managed(frame, self);
        }
        Ok(())
    }

    /// Zeroes a byte range within one page.
    pub fn zero(self: &Arc<Self>, offset: usize, length: usize) -> Result<()> {
        validate_range(offset, length)?;
        if length == 0 {
            return Ok(());
        }

        let (frame, became_resident) = {
            let mut backing = self.backing.lock();
            let became_resident = self.ensure_resident_locked(&mut backing)?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: resident page lost backing");
            };
            frame.mark_referenced();
            frame.mark_dirty();
            // SAFETY: the backing lock prevents reclamation and the validated
            // range remains within the HHDM-mapped frame.
            unsafe {
                ptr::write_bytes(
                    mem::phys_to_virt(frame.paddr())
                        .as_mut_ptr::<u8>()
                        .add(offset),
                    0,
                    length,
                );
            }
            (*frame, became_resident)
        };
        if became_resident {
            phys::activate_managed(frame, self);
        }
        Ok(())
    }

    /// Creates an independent anonymous page containing the same bytes.
    pub fn copy_to(self: &Arc<Self>, index: u64) -> Result<Arc<Self>> {
        let mut bytes = [0u8; PAGE_BYTES];
        self.read(0, &mut bytes)?;
        let copy = Self::new_zero(index, PageOwnerKind::Anonymous);
        copy.write(0, &bytes)?;
        Ok(copy)
    }

    /// Pins the resident frame for a new hardware mapping.
    pub(super) fn acquire_mapping(self: &Arc<Self>) -> Result<PhysAddr> {
        let (frame, became_resident) = {
            let mut backing = self.backing.lock();
            let became_resident = self.ensure_resident_locked(&mut backing)?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: mapped page is not resident");
            };
            if !self.permanent {
                frame.add_mapping();
            }
            frame.mark_referenced();
            (*frame, became_resident)
        };
        if became_resident {
            phys::activate_managed(frame, self);
        }
        Ok(frame.paddr())
    }

    /// Releases one hardware mapping after its TLB grace period.
    pub(super) fn release_mapping(&self) {
        if self.permanent {
            return;
        }
        let backing = self.backing.lock();
        let PageBacking::Resident(frame) = &*backing else {
            panic!("mem/page: releasing mapping from nonresident page");
        };
        frame.remove_mapping();
    }

    pub(super) fn add_reverse_mapping(&self, pmap: &Arc<super::pmap::PmapInner>, address: u64) {
        if self.permanent {
            return;
        }
        let backing = self.backing.lock();
        let PageBacking::Resident(frame) = &*backing else {
            panic!("mem/page: adding reverse mapping to nonresident page");
        };
        frame.add_reverse_mapping(pmap, address);
    }

    pub(super) fn remove_reverse_mapping(&self, pmap: *const super::pmap::PmapInner, address: u64) {
        if self.permanent {
            return;
        }
        let backing = self.backing.lock();
        let PageBacking::Resident(frame) = &*backing else {
            panic!("mem/page: removing reverse mapping from nonresident page");
        };
        frame.remove_reverse_mapping(pmap, address);
    }

    pub(super) fn reverse_mapping(&self, index: usize) -> Option<phys::ReverseMapping> {
        if self.permanent {
            return None;
        }
        let backing = self.backing.lock();
        let PageBacking::Resident(frame) = &*backing else {
            return None;
        };
        frame.reverse_mapping(index)
    }

    pub(super) fn first_reverse_mapping(&self) -> Option<phys::ReverseMapping> {
        self.reverse_mapping(0)
    }

    pub(super) fn try_reclaim(self: &Arc<Self>) -> bool {
        if self.permanent {
            return false;
        }

        let mapped = {
            let backing = self.backing.lock();
            matches!(&*backing, PageBacking::Resident(frame) if frame.map_count() != 0)
        };
        if mapped {
            super::pmap::remove_all_mappings(self);
        }

        let mut backing = self.backing.lock();
        let PageBacking::Resident(frame) = &*backing else {
            return false;
        };
        let frame = *frame;
        if !frame.reclaimable() || !frame.try_busy() {
            drop(backing);
            phys::activate_managed(frame, self);
            return false;
        }

        // SAFETY: the busy frame is unmapped, unwired, owner-locked by
        // `backing`, and permanently HHDM-mapped.
        let bytes = unsafe { &*mem::phys_to_virt(frame.paddr()).as_ptr::<[u8; PAGE_BYTES]>() };
        let handle = match swap::store(bytes) {
            Ok(handle) => handle,
            Err(_) => {
                let _ = frame.unbusy();
                drop(backing);
                phys::activate_managed(frame, self);
                return false;
            }
        };

        *backing = PageBacking::Swapped(handle);
        let _ = frame.unbusy();
        frame.mark_clean();
        frame.clear_owner();
        super::page_reclaimed();
        // SAFETY: the swap handle is authoritative and the frame has no
        // mappings, wires, loans, owner, or page-daemon membership.
        unsafe { phys::free_page(frame) };
        true
    }

    pub(super) fn reactivate(self: &Arc<Self>) {
        let backing = self.backing.lock();
        if let PageBacking::Resident(frame) = &*backing {
            phys::activate_managed(frame, self);
        }
    }

    pub(super) fn record_hardware_state(&self, referenced: bool, dirty: bool) {
        let backing = self.backing.lock();
        if let PageBacking::Resident(frame) = &*backing {
            if referenced {
                frame.mark_referenced();
            }
            if dirty {
                frame.mark_dirty();
            }
        }
    }

    pub(super) fn take_referenced(&self) -> bool {
        let backing = self.backing.lock();
        matches!(&*backing, PageBacking::Resident(frame) if frame.take_referenced())
    }

    fn ensure_resident(self: &Arc<Self>) -> Result<()> {
        let (frame, became_resident) = {
            let mut backing = self.backing.lock();
            let became_resident = self.ensure_resident_locked(&mut backing)?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: page-in did not create resident backing");
            };
            (*frame, became_resident)
        };
        if became_resident {
            phys::activate_managed(frame, self);
        }
        Ok(())
    }

    fn ensure_resident_locked(self: &Arc<Self>, backing: &mut PageBacking) -> Result<bool> {
        match *backing {
            PageBacking::Resident(_) => Ok(false),
            PageBacking::Zero => {
                let frame = super::allocate_physical_page()?;
                frame.bind_owner(self, self.owner_kind, self.id, self.index);
                frame.mark_referenced();
                *backing = PageBacking::Resident(frame);
                Ok(true)
            }
            PageBacking::Swapped(handle) => {
                let frame = super::allocate_physical_page()?;
                // SAFETY: the new managed frame is exclusively owned and
                // writable through its HHDM mapping.
                let output = unsafe {
                    &mut *mem::phys_to_virt(frame.paddr()).as_mut_ptr::<[u8; PAGE_BYTES]>()
                };
                if swap::load(handle, output).is_err() {
                    // SAFETY: the frame was never published or bound.
                    unsafe { phys::free_page(frame) };
                    return Err(Error::CorruptSwap);
                }
                swap::free(handle);
                frame.bind_owner(self, self.owner_kind, self.id, self.index);
                frame.mark_referenced();
                *backing = PageBacking::Resident(frame);
                Ok(true)
            }
        }
    }
}

impl Drop for VmPage {
    fn drop(&mut self) {
        let backing = self.backing.get_mut();
        match *backing {
            PageBacking::Zero => {}
            PageBacking::Resident(frame) => {
                phys::remove_managed(frame);
                frame.clear_owner();
                assert_eq!(frame.map_count(), 0, "mem/page: dropping mapped page");
                assert_eq!(frame.wire_count(), 0, "mem/page: dropping wired page");
                // SAFETY: the final logical-page owner holds the unreferenced
                // physical frame exclusively.
                unsafe { phys::free_page(frame) };
            }
            PageBacking::Swapped(handle) => swap::free(handle),
        }
    }
}

fn validate_range(offset: usize, length: usize) -> Result<()> {
    if offset
        .checked_add(length)
        .is_none_or(|end| end > PAGE_BYTES)
    {
        Err(Error::InvalidAddress)
    } else {
        Ok(())
    }
}

pub(super) fn owner_kind_for_object(kind: super::ObjectKind) -> PageOwnerKind {
    match kind {
        super::ObjectKind::Anonymous => PageOwnerKind::Anonymous,
        super::ObjectKind::Vnode => PageOwnerKind::File,
        super::ObjectKind::Kernel => PageOwnerKind::Kernel,
    }
}
