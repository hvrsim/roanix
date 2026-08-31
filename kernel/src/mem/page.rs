//! Logical memory pages backed by authoritative PFN database entries.

use alloc::{sync::Arc, vec::Vec};
use core::{
    ptr,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{
    mem::{self, PAGE_SIZE, PhysAddr, phys, phys::PageOwnerKind},
    sys::sync::Mutex,
};

use super::{
    Error, Result,
    pmap::{PmapInner, ReverseMapping},
    swap::{self, SwapError, SwapHandle},
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
    /// Every hardware mapping of this page.
    ///
    /// The list belongs to the logical page rather than the frame so that it
    /// survives eviction and so that per-frame metadata stays free of owning
    /// collections.
    mappings: Mutex<Vec<ReverseMapping>>,
    /// Swap configuration generation that rejected these unchanged contents.
    swap_reject_generation: AtomicU64,
    permanent: bool,
}

impl VmPage {
    pub(super) fn new_zero(index: u64, owner_kind: PageOwnerKind) -> Arc<Self> {
        Arc::new(Self {
            id: super::allocate_page_id(),
            index,
            owner_kind,
            backing: Mutex::new(PageBacking::Zero),
            mappings: Mutex::new(Vec::new()),
            swap_reject_generation: AtomicU64::new(0),
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
            mappings: Mutex::new(Vec::new()),
            swap_reject_generation: AtomicU64::new(0),
            permanent: true,
        });
        frame.bind_owner(&page, PageOwnerKind::Kernel);
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

        let frame = {
            let mut backing = self.backing.lock();
            self.ensure_resident_locked(&mut backing)?;
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
            *frame
        };
        self.mark_modified(frame);
        Ok(())
    }

    /// Zeroes a byte range within one page.
    pub fn zero(self: &Arc<Self>, offset: usize, length: usize) -> Result<()> {
        validate_range(offset, length)?;
        if length == 0 {
            return Ok(());
        }

        let frame = {
            let mut backing = self.backing.lock();
            self.ensure_resident_locked(&mut backing)?;
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
            *frame
        };
        self.mark_modified(frame);
        Ok(())
    }

    /// Creates an independent anonymous page containing the same bytes.
    ///
    /// The copy is performed frame to frame through the direct map, so no
    /// page-sized buffer is placed on the kernel stack.
    pub fn copy_to(self: &Arc<Self>, index: u64) -> Result<Arc<Self>> {
        let copy = Self::new_zero(index, PageOwnerKind::Anonymous);
        // The destination is unreachable by any other CPU until it is
        // published, so its frame can be materialized up front.
        let destination = {
            let mut backing = copy.backing.lock();
            copy.ensure_resident_locked(&mut backing)?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: fresh page is not resident");
            };
            *frame
        };

        loop {
            let backing = self.backing.lock();
            if let PageBacking::Resident(source) = &*backing {
                source.mark_referenced();
                // SAFETY: both frames are distinct, permanently direct-mapped,
                // and the source lock prevents its reclamation for the copy.
                unsafe {
                    ptr::copy_nonoverlapping(
                        mem::phys_to_virt(source.paddr()).as_ptr::<u8>(),
                        mem::phys_to_virt(destination.paddr()).as_mut_ptr::<u8>(),
                        PAGE_BYTES,
                    );
                }
                break;
            }
            drop(backing);
            self.ensure_resident()?;
        }

        destination.mark_dirty();
        phys::activate_managed(destination, &copy);
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
        if let PageBacking::Resident(frame) = &*backing {
            frame.remove_mapping();
        }
    }

    /// Pins the resident frame of this page for a direct-map window.
    ///
    /// The wire blocks reclamation, so the returned physical address stays
    /// valid for as long as the caller holds the page. Pair every call with
    /// [`Self::release_window`]; the page must not drop while wired.
    pub(super) fn acquire_window(self: &Arc<Self>) -> Result<PhysAddr> {
        let (frame, became_resident) = {
            let mut backing = self.backing.lock();
            let became_resident = self.ensure_resident_locked(&mut backing)?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: window page is not resident");
            };
            if !self.permanent {
                frame.wire();
            }
            (*frame, became_resident)
        };
        if became_resident {
            phys::activate_managed(frame, self);
        }
        Ok(frame.paddr())
    }

    /// Releases one direct-map window pinned by [`Self::acquire_window`].
    pub(super) fn release_window(&self) {
        if self.permanent {
            return;
        }
        let backing = self.backing.lock();
        if let PageBacking::Resident(frame) = &*backing {
            frame.unwire();
        }
    }

    pub(super) fn add_reverse_mapping(&self, pmap: &Arc<PmapInner>, address: u64) {
        if self.permanent {
            return;
        }
        self.mappings.lock().push(ReverseMapping {
            pmap: Arc::downgrade(pmap),
            address,
        });
    }

    pub(super) fn remove_reverse_mapping(&self, pmap: *const PmapInner, address: u64) {
        if self.permanent {
            return;
        }
        let mut mappings = self.mappings.lock();
        if let Some(index) = mappings
            .iter()
            .position(|mapping| mapping.pmap.as_ptr() == pmap && mapping.address == address)
        {
            mappings.swap_remove(index);
        }
    }

    /// Detaches the reverse-mapping list for traversal.
    ///
    /// Callers must lock individual pmaps, and the pmap lock is ordered before
    /// this list, so the list cannot be held across that acquisition. Moving
    /// it out costs nothing and lets the traversal run unlocked.
    pub(super) fn take_reverse_mappings(&self) -> Vec<ReverseMapping> {
        if self.permanent {
            return Vec::new();
        }
        core::mem::take(&mut *self.mappings.lock())
    }

    /// Returns unprocessed mappings to the page.
    ///
    /// Entries recorded while the list was detached are preserved. A restored
    /// entry may have become stale, which every consumer already tolerates by
    /// revalidating against the pmap before acting on it.
    pub(super) fn restore_reverse_mappings(&self, mappings: Vec<ReverseMapping>) {
        if mappings.is_empty() {
            return;
        }
        let mut current = self.mappings.lock();
        if current.is_empty() {
            *current = mappings;
        } else {
            current.extend(mappings);
        }
    }

    pub(super) fn try_reclaim(self: &Arc<Self>) -> bool {
        if self.permanent {
            return false;
        }

        if self.swap_rejected_for_current_configuration() {
            let backing = self.backing.lock();
            if let PageBacking::Resident(frame) = &*backing {
                phys::park_unswappable(frame, self);
            }
            return false;
        }

        let mapped = {
            let backing = self.backing.lock();
            matches!(&*backing, PageBacking::Resident(frame) if frame.map_count() != 0)
        };
        if mapped {
            super::pmap::try_remove_all_mappings(self);
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
            Err(error) => {
                let _ = frame.unbusy();
                if error == SwapError::Incompressible {
                    self.swap_reject_generation
                        .store(swap::configuration_generation(), Ordering::Relaxed);
                }
                drop(backing);
                if error == SwapError::Incompressible {
                    phys::park_unswappable(frame, self);
                } else {
                    phys::activate_managed(frame, self);
                }
                return false;
            }
        };

        self.swap_reject_generation.store(0, Ordering::Relaxed);
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

    pub(super) fn record_hardware_state(self: &Arc<Self>, referenced: bool, dirty: bool) {
        let frame = {
            let backing = self.backing.lock();
            let PageBacking::Resident(frame) = &*backing else {
                return;
            };
            if referenced {
                frame.mark_referenced();
            }
            if dirty {
                frame.mark_dirty();
            }
            *frame
        };
        if dirty {
            self.mark_modified(frame);
        }
    }

    /// Requeues a parked page when it changed or the available swap tiers did.
    pub(super) fn reconsider_unswappable(self: &Arc<Self>) {
        let backing = self.backing.lock();
        let PageBacking::Resident(frame) = &*backing else {
            return;
        };
        if self.swap_rejected_for_current_configuration() {
            phys::park_unswappable(frame, self);
        } else {
            phys::activate_managed(frame, self);
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
                frame.bind_owner(self, self.owner_kind);
                frame.mark_referenced();
                *backing = PageBacking::Resident(frame);
                Ok(true)
            }
            PageBacking::Swapped(handle) => {
                let frame = super::allocate_pagein_physical_page()?;
                // SAFETY: the new managed frame is exclusively owned and
                // writable through its HHDM mapping.
                let output = unsafe {
                    &mut *mem::phys_to_virt(frame.paddr()).as_mut_ptr::<[u8; PAGE_BYTES]>()
                };
                if let Err(error) = swap::load_and_free(handle, output) {
                    // SAFETY: the frame was never published or bound.
                    unsafe { phys::free_page(frame) };
                    return Err(match error {
                        SwapError::Corrupt => Error::CorruptSwap,
                        SwapError::Full | SwapError::Incompressible | SwapError::Io => {
                            Error::SwapUnavailable
                        }
                    });
                }
                self.swap_reject_generation.store(0, Ordering::Relaxed);
                frame.bind_owner(self, self.owner_kind);
                frame.mark_referenced();
                *backing = PageBacking::Resident(frame);
                Ok(true)
            }
        }
    }

    fn mark_modified(self: &Arc<Self>, frame: &'static phys::Page) {
        self.swap_reject_generation.store(0, Ordering::Relaxed);
        phys::activate_managed(frame, self);
    }

    fn swap_rejected_for_current_configuration(&self) -> bool {
        let generation = self.swap_reject_generation.load(Ordering::Relaxed);
        generation != 0 && generation == swap::configuration_generation()
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
