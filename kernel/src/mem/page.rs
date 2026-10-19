//! Logical memory pages backed by authoritative PFN database entries.

use alloc::sync::Arc;
use core::{
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, AtomicUsize, Ordering},
};

use crate::{
    mem::{self, PAGE_SIZE, PhysAddr, phys, phys::PageOwnerKind},
    sys::{
        event::SequenceEvent,
        smp::{IrqSpinLock, IrqSpinLockGuard},
    },
};

use super::{
    Error, Result,
    metadata::NoSleep,
    owner::{Owned, OwnedGuard, OwnerLock, OwnerRef, OwnerWrite},
    pmap::{PmapInner, ReverseMappings},
    swap::{self, SwapError, SwapHandle},
};

const PAGE_BYTES: usize = PAGE_SIZE as usize;
#[repr(align(64))]
struct PageWait(SequenceEvent);
static PAGE_WAITS: [PageWait; 64] = [const { PageWait(SequenceEvent::new()) }; 64];

enum PageBacking {
    Zero,
    File,
    Resident(&'static phys::Page),
    Swapped(SwapHandle),
    Busy {
        location: PageLocation,
        wanted: bool,
    },
}

/// Current storage location of a managed page.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageLocation {
    /// Clean file page whose contents can be read again from its pager.
    File,
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

/// Shared ownership of a fallibly allocated logical page.
pub type PageRef = Arc<VmPage, NoSleep>;
/// Non-owning reference retained by physical-page metadata.
pub type PageWeak = alloc::sync::Weak<VmPage, NoSleep>;

/// Logical object or anonymous page that may be zero, resident, or swapped.
pub struct VmPage {
    id: u64,
    index: u64,
    owner_kind: PageOwnerKind,
    backing: Owned<PageBacking>,
    resident: AtomicPtr<phys::Page>,
    executable: AtomicBool,
    /// A resident page may still have authoritative swap backing. Protected
    /// by exactly the same owner lock as residency and index membership.
    resident_swap: Owned<Option<SwapHandle>>,
    /// Every hardware mapping of this page.
    ///
    /// The list belongs to the logical page rather than the frame so that it
    /// survives eviction and so that per-frame metadata stays free of owning
    /// collections.
    mappings: IrqSpinLock<ReverseMappings>,
    /// Swap configuration generation that rejected these unchanged contents.
    swap_reject_generation: AtomicU64,
    pub(super) write_windows: AtomicUsize,
    pager: Option<Arc<dyn super::VmPager>>,
    pager_owner: Option<alloc::sync::Weak<super::object::ObjectPages>>,
}

/// A resident frame wire retained across a fault's map-lock revalidation.
pub(super) struct PagePin {
    page: PageRef,
    address: PhysAddr,
    wired: bool,
}

/// A resident neighbor is protected by its owner's existing hold, not by
/// wiring it and accidentally changing replacement policy membership.
pub(super) struct NeighborPin<'a> {
    pin: PagePin,
    _owner: &'a OwnerWrite<'a>,
}
impl core::ops::Deref for NeighborPin<'_> {
    type Target = PagePin;
    fn deref(&self) -> &PagePin {
        &self.pin
    }
}

impl PagePin {
    pub(super) fn address(&self) -> PhysAddr {
        self.address
    }

    pub(super) fn page(&self) -> &PageRef {
        &self.page
    }

    pub(super) fn acquire_mapping(&self) -> PhysAddr {
        let frame = phys::phys_to_page(self.address).expect("mem: pinned frame has no metadata");
        frame.add_mapping();
        frame.mark_referenced();
        self.address
    }
}
impl Drop for PagePin {
    fn drop(&mut self) {
        if !self.wired {
            return;
        }
        let frame = phys::phys_to_page(self.address).expect("mem: pinned frame has no metadata");
        frame.unwire();
    }
}

impl VmPage {
    pub(super) fn owner(&self) -> &OwnerRef {
        self.backing.owner()
    }

    pub(super) fn new_zero_in(
        index: u64,
        owner_kind: PageOwnerKind,
        owner: OwnerRef,
    ) -> Result<PageRef> {
        Arc::try_new_in(
            Self {
                id: super::allocate_page_id(),
                index,
                owner_kind,
                backing: Owned::new(owner.clone(), PageBacking::Zero),
                resident_swap: Owned::new(owner, None),
                resident: AtomicPtr::new(ptr::null_mut()),
                executable: AtomicBool::new(false),
                mappings: IrqSpinLock::new(ReverseMappings::default()),
                swap_reject_generation: AtomicU64::new(0),
                write_windows: AtomicUsize::new(0),
                pager: None,
                pager_owner: None,
            },
            NoSleep,
        )
        .map_err(|_| Error::OutOfMemory)
    }

    /// Establishes pager identity before I/O. The owner publishes this page
    /// in its index; resident_backing then claims PG_BUSY under that same
    /// owner lock, and competing faults use PG_WANTED instead of a fill gate.
    pub(super) fn file_in(
        index: u64,
        pager: Arc<dyn super::VmPager>,
        owner: OwnerRef,
        pager_owner: Option<alloc::sync::Weak<super::object::ObjectPages>>,
    ) -> Result<PageRef> {
        Arc::try_new_in(
            Self {
                id: super::allocate_page_id(),
                index,
                owner_kind: PageOwnerKind::File,
                backing: Owned::new(owner.clone(), PageBacking::File),
                resident_swap: Owned::new(owner, None),
                resident: AtomicPtr::new(ptr::null_mut()),
                executable: AtomicBool::new(false),
                mappings: IrqSpinLock::new(ReverseMappings::default()),
                swap_reject_generation: AtomicU64::new(0),
                write_windows: AtomicUsize::new(0),
                pager: Some(pager),
                pager_owner,
            },
            NoSleep,
        )
        .map_err(|_| Error::OutOfMemory)
    }

    pub(super) fn is_executable(&self) -> bool {
        self.executable.load(Ordering::Relaxed)
    }
    pub(super) fn note_executable(&self, frame: &'static phys::Page) {
        self.executable.store(true, Ordering::Relaxed);
        frame.note_executable();
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
        self.info_backing(&self.backing.read())
    }

    pub(super) fn info_locked(&self, owner: &OwnerWrite<'_>) -> PageInfo {
        self.info_backing(self.backing.get(owner))
    }

    fn info_backing(&self, backing: &PageBacking) -> PageInfo {
        match backing {
            PageBacking::Busy { location, .. } => PageInfo {
                id: self.id,
                index: self.index,
                location: *location,
                mappings: 0,
                wires: 0,
                dirty: false,
                referenced: false,
            },
            PageBacking::File => PageInfo {
                id: self.id,
                index: self.index,
                location: PageLocation::File,
                mappings: 0,
                wires: 0,
                dirty: false,
                referenced: false,
            },
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
    pub fn read(self: &PageRef, offset: usize, output: &mut [u8]) -> Result<()> {
        // SAFETY: the caller lends a writable kernel buffer for this transfer.
        unsafe { self.read_raw(offset, output.as_mut_ptr(), output.len()) }
    }

    /// Copies into kernel storage or a pinned user window, allowing overlap.
    ///
    /// # Safety
    /// `output` must remain writable for `length` bytes throughout the call.
    /// It must not overlap any live Rust reference to the destination bytes.
    pub(super) unsafe fn read_raw(
        self: &PageRef,
        offset: usize,
        output: *mut u8,
        length: usize,
    ) -> Result<()> {
        validate_range(offset, length)?;
        if length == 0 {
            return Ok(());
        }

        loop {
            let backing = self.backing.lock();
            match &*backing {
                PageBacking::Zero => {
                    // SAFETY: the caller retains this writable destination.
                    unsafe { ptr::write_bytes(output, 0, length) };
                    return Ok(());
                }
                PageBacking::Resident(frame) => {
                    frame.mark_referenced();
                    // SAFETY: the backing lock prevents reclamation and the
                    // validated range remains within the HHDM-mapped frame.
                    // The user buffer may alias this page through mmap.
                    unsafe {
                        ptr::copy(
                            mem::phys_to_virt(frame.paddr()).as_ptr::<u8>().add(offset),
                            output,
                            length,
                        );
                    }
                    return Ok(());
                }
                PageBacking::Swapped(_) | PageBacking::File | PageBacking::Busy { .. } => {
                    drop(backing)
                }
            }
            self.ensure_resident()?;
        }
    }

    /// Writes bytes into one page.
    pub fn write(self: &PageRef, offset: usize, input: &[u8]) -> Result<()> {
        // SAFETY: the caller lends a readable kernel buffer for this transfer.
        unsafe { self.write_raw(offset, input.as_ptr(), input.len()) }
    }

    /// Copies from kernel storage or a pinned user window, allowing overlap.
    ///
    /// # Safety
    /// `input` must remain readable for `length` bytes throughout the call.
    /// It must not overlap a live mutable Rust reference to the source bytes.
    pub(super) unsafe fn write_raw(
        self: &PageRef,
        offset: usize,
        input: *const u8,
        length: usize,
    ) -> Result<()> {
        validate_range(offset, length)?;
        if length == 0 {
            return Ok(());
        }

        {
            let backing = self.resident_backing()?;
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!("mem/page: resident page lost backing");
            };
            frame.mark_referenced();
            frame.mark_dirty();
            // SAFETY: the backing lock prevents reclamation and the validated
            // range remains within the HHDM-mapped frame.
            // The user buffer may alias this page through mmap.
            unsafe {
                ptr::copy(
                    input,
                    mem::phys_to_virt(frame.paddr())
                        .as_mut_ptr::<u8>()
                        .add(offset),
                    length,
                );
            }
            self.mark_modified(frame);
        }
        Ok(())
    }

    /// Zeroes a byte range within one page.
    pub fn zero(self: &PageRef, offset: usize, length: usize) -> Result<()> {
        validate_range(offset, length)?;
        if length == 0 {
            return Ok(());
        }

        {
            let backing = self.backing.lock();
            // Zeroing an already logical-zero page is a metadata no-op. In
            // particular, sparse-file truncation must not allocate a frame.
            if matches!(*backing, PageBacking::Zero) {
                return Ok(());
            }
            drop(backing);
            let backing = self.resident_backing()?;
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
            self.mark_modified(frame);
        }
        Ok(())
    }

    /// Zeroes a pinned resident range while its UVM owner lock is already held.
    pub(super) fn zero_locked(
        self: &PageRef,
        offset: usize,
        length: usize,
        owner: &mut OwnerWrite<'_>,
    ) -> Result<()> {
        validate_range(offset, length)?;
        let backing = self.backing.get_locked_mut(owner);
        if matches!(backing, PageBacking::Zero) {
            return Ok(());
        }
        let PageBacking::Resident(frame) = backing else {
            return Err(Error::OutOfMemory);
        };
        frame.mark_referenced();
        frame.mark_dirty();
        // SAFETY: the caller's resident pin retains the frame and the owner
        // capability serializes its state; the validated bytes lie in it.
        unsafe {
            ptr::write_bytes(
                mem::phys_to_virt(frame.paddr())
                    .as_mut_ptr::<u8>()
                    .add(offset),
                0,
                length,
            )
        };
        self.mark_modified(frame);
        Ok(())
    }

    /// Creates an independent anonymous page containing the same bytes.
    ///
    /// The copy is performed frame to frame through the direct map, so no
    /// page-sized buffer is placed on the kernel stack.
    pub fn copy_to(self: &PageRef, index: u64) -> Result<PageRef> {
        self.copy_to_in(index, OwnerLock::new()?)
    }

    pub(super) fn copy_to_in(self: &PageRef, index: u64, owner: OwnerRef) -> Result<PageRef> {
        let copy = Self::new_zero_in(index, PageOwnerKind::Anonymous, owner)?;
        // The destination enters the reclaim queues when materialized. Keep
        // a wire until the complete copy is ready for publication.
        let destination = copy.pin_resident()?;

        loop {
            let backing = self.backing.lock();
            if matches!(*backing, PageBacking::Zero) {
                break;
            }
            if let PageBacking::Resident(source) = &*backing {
                source.mark_referenced();
                // SAFETY: both frames are distinct, permanently direct-mapped,
                // and the source lock prevents its reclamation for the copy.
                unsafe {
                    ptr::copy_nonoverlapping(
                        mem::phys_to_virt(source.paddr()).as_ptr::<u8>(),
                        mem::phys_to_virt(destination.address).as_mut_ptr::<u8>(),
                        PAGE_BYTES,
                    );
                }
                break;
            }
            drop(backing);
            self.ensure_resident()?;
        }

        {
            let backing = copy.backing.lock();
            let PageBacking::Resident(frame) = &*backing else {
                unreachable!()
            };
            frame.mark_dirty();
        }
        Ok(copy)
    }

    pub(super) fn copy_wired_locked(
        self: &PageRef,
        index: u64,
        owner: &mut OwnerWrite<'_>,
    ) -> Result<Option<PageRef>> {
        let backing = self.backing.get(owner);
        let PageBacking::Resident(source) = backing else {
            debug_assert_eq!(self.write_windows.load(Ordering::Acquire), 0);
            return Ok(None);
        };
        let copy = Self::new_zero_in(index, PageOwnerKind::Anonymous, self.owner().clone())?;
        let frame = phys::alloc_page(phys::PageUse::Managed, phys::PageAllocFlags::empty())
            .ok_or(Error::OutOfMemory)?;
        // SAFETY: the source lock retains its resident frame, and the new
        // destination is exclusively owned and has not entered reclaim queues.
        unsafe {
            ptr::copy_nonoverlapping(
                mem::phys_to_virt(source.paddr()).as_ptr::<u8>(),
                mem::phys_to_virt(frame.paddr()).as_mut_ptr::<u8>(),
                PAGE_BYTES,
            );
        }
        *copy.backing.get_locked_mut(owner) = PageBacking::Resident(frame);
        copy.resident
            .store(ptr::from_ref(frame).cast_mut(), Ordering::Release);
        frame.bind_owner(&copy, PageOwnerKind::Anonymous);
        frame.mark_dirty();
        frame.mark_referenced();
        phys::activate_managed(frame, &copy);
        Ok(Some(copy))
    }

    /// Releases one hardware mapping after its TLB grace period. The mapping
    /// reference itself retains residency, so no owner-lock reacquisition is
    /// needed (or allowed while a fault already holds that owner lock).
    pub(super) fn release_mapping(&self) {
        let frame = self.resident.load(Ordering::Acquire);
        assert!(!frame.is_null(), "mem: removing a nonresident mapping");
        // SAFETY: the outstanding mapping reference prevents frame retirement;
        // nothing accesses the frame after its final mapping is removed.
        unsafe { &*frame }.remove_mapping();
    }

    /// Pins the resident frame of this page for a direct-map window.
    ///
    /// The wire blocks reclamation, so the returned physical address stays
    /// valid for as long as the caller retains the returned PagePin.
    pub(super) fn acquire_window(self: &PageRef) -> Result<PhysAddr> {
        let backing = self.resident_backing()?;
        let PageBacking::Resident(frame) = &*backing else {
            unreachable!()
        };
        frame.wire();
        Ok(frame.paddr())
    }

    /// PGO_LOCKED neighbor lookup: never allocates, pages in, or waits for
    /// busy pages. The returned borrow keeps the owner hold in force.
    pub(super) fn pin_neighbor_locked<'a>(
        self: &'a PageRef,
        owner: &'a OwnerWrite<'a>,
    ) -> Option<NeighborPin<'a>> {
        let PageBacking::Resident(frame) = self.backing.get(owner) else {
            return None;
        };
        if frame.loan_count() != 0 {
            return None;
        }
        if frame.queue() == phys::PageQueue::None && frame.wire_count() == 0 {
            phys::enqueue_managed(frame);
        }
        Some(NeighborPin {
            pin: PagePin {
                page: self.clone(),
                address: frame.paddr(),
                wired: false,
            },
            _owner: owner,
        })
    }

    pub(super) fn deactivate_locked(self: &PageRef, owner: &mut OwnerWrite<'_>) {
        super::pmap::harvest_page_state_locked(self, owner);
        if let PageBacking::Resident(frame) = self.backing.get(owner)
            && frame.wire_count() == 0
            && frame.loan_count() == 0
        {
            phys::deactivate_managed(frame);
        }
    }

    pub(super) fn pin_resident(self: &PageRef) -> Result<PagePin> {
        Ok(PagePin {
            address: self.acquire_window()?,
            page: self.clone(),
            wired: true,
        })
    }

    pub(super) fn add_reverse_mapping(&self, pmap: &Arc<PmapInner>, address: u64) -> Result<()> {
        let mapping = ReverseMappings::allocate(pmap, address)?;
        self.mappings.lock().push(mapping);
        Ok(())
    }

    pub(super) fn remove_reverse_mapping(&self, pmap: *const PmapInner, address: u64) {
        let removed = self.mappings.lock().remove(pmap, address);
        drop(removed);
    }

    /// The MD PV interlock. P->V callers try the pmap lock while this is
    /// held, then hand off directly and drop PV before waiting or modifying.
    pub(super) fn reverse_mappings(&self) -> IrqSpinLockGuard<'_, ReverseMappings> {
        self.mappings.lock()
    }

    pub(super) fn reclaim_owned<'a>(self: &'a PageRef, owner: OwnerWrite<'a>) -> bool {
        let mut backing = self.backing.with_guard(owner);
        let PageBacking::Resident(frame) = &*backing else {
            return false;
        };
        if self.swap_rejected_for_current_configuration() {
            phys::activate_managed(frame, self);
            return false;
        }
        if frame.wire_count() != 0 || frame.loan_count() != 0 {
            phys::activate_managed(frame, self);
            return false;
        }
        if frame.map_count() != 0 {
            super::pmap::remove_all_mappings(self);
        }
        let PageBacking::Resident(frame) = &*backing else {
            unreachable!();
        };
        let frame = *frame;
        if !frame.reclaimable() || !frame.try_busy() {
            phys::activate_managed(frame, self);
            return false;
        }

        frame.resolve_unknown();
        // A fault may have reactivated this candidate since the scanner
        // detached it. Remove its current membership before releasing storage.
        phys::remove_managed(frame);
        self.resident.store(ptr::null_mut(), Ordering::Release);
        if self.pager.is_some() && !frame.flags().contains(phys::PageFlags::DIRTY) {
            *backing = PageBacking::File;
            if let Some(object) = self
                .pager_owner
                .as_ref()
                .and_then(alloc::sync::Weak::upgrade)
            {
                object.retire(self, backing.capability_mut());
            }
            let _ = frame.unbusy();
            frame.clear_owner();
            super::page_reclaimed();
            // SAFETY: the clean file backing is authoritative; the frame has
            // no mappings or wires and was removed from the reclaim queue.
            unsafe { phys::free_page(frame) };
            return true;
        }

        let retained = self
            .resident_swap
            .get_locked_mut(backing.capability_mut())
            .take();
        if let Some(handle) = retained {
            if !frame.flags().contains(phys::PageFlags::DIRTY) {
                swap::set_resident(handle, false);
                *backing = PageBacking::Swapped(handle);
                let _ = frame.unbusy();
                frame.clear_owner();
                super::page_reclaimed();
                // SAFETY: the retained slot is authoritative and no hardware,
                // wire or policy queue still references this frame.
                unsafe { phys::free_page(frame) };
                return true;
            }
            // Dirty contents are authoritative in the busy frame. Release
            // stale backing before allocating the replacement swap slot.
            swap::free(handle);
        }

        *backing = PageBacking::Busy {
            location: PageLocation::Resident,
            wanted: false,
        };
        drop(backing);

        // SAFETY: the busy state excludes new mappings and wires. This call
        // owns the unmapped frame until the swap operation completes.
        let bytes = unsafe { &*mem::phys_to_virt(frame.paddr()).as_ptr::<[u8; PAGE_BYTES]>() };
        let stored = swap::store(bytes);
        let mut backing = self.backing.lock();
        let PageBacking::Busy { wanted, .. } = *backing else {
            unreachable!()
        };
        let event = &PAGE_WAITS[self.id as usize % PAGE_WAITS.len()].0;
        let handle = match stored {
            Ok(handle) => handle,
            Err(error) => {
                *backing = PageBacking::Resident(frame);
                self.resident
                    .store(ptr::from_ref(frame).cast_mut(), Ordering::Release);
                let _ = frame.unbusy();
                if error == SwapError::Incompressible {
                    self.swap_reject_generation
                        .store(swap::configuration_generation(), Ordering::Relaxed);
                }
                phys::activate_managed(frame, self);
                drop(backing);
                if wanted {
                    event.notify();
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
        drop(backing);
        if wanted {
            event.notify();
        }
        true
    }

    pub(super) fn frame_locked(&self, owner: &OwnerWrite<'_>) -> Option<&'static phys::Page> {
        match self.backing.get(owner) {
            PageBacking::Resident(frame) => Some(*frame),
            _ => None,
        }
    }

    pub(super) fn drop_swap_locked(self: &PageRef, owner: &mut OwnerWrite<'_>) -> bool {
        let Some(frame) = self.frame_locked(owner) else {
            return false;
        };
        let Some(handle) = self.resident_swap.get_locked_mut(owner).take() else {
            return false;
        };
        swap::free(handle);
        frame.mark_dirty();
        true
    }

    pub(super) fn record_hardware_state(self: &PageRef, referenced: bool, dirty: bool) {
        let mut owner = self.owner().write();
        self.record_hardware_state_locked(referenced, dirty, &mut owner);
    }

    pub(super) fn record_hardware_state_locked(
        self: &PageRef,
        referenced: bool,
        dirty: bool,
        owner: &mut OwnerWrite<'_>,
    ) {
        let PageBacking::Resident(frame) = self.backing.get_locked_mut(owner) else {
            return;
        };
        if referenced {
            frame.mark_referenced();
        }
        if dirty {
            frame.mark_dirty();
            self.mark_modified(frame);
        }
    }

    fn ensure_resident(self: &PageRef) -> Result<()> {
        self.resident_backing().map(|_| ())
    }

    fn resident_backing<'a>(self: &'a PageRef) -> Result<OwnedGuard<'a, PageBacking>> {
        let event = &PAGE_WAITS[self.id as usize % PAGE_WAITS.len()].0;
        let mut backing = loop {
            let mut backing = self.backing.lock();
            if matches!(*backing, PageBacking::Resident(_)) {
                return Ok(backing);
            }
            if let PageBacking::Busy { wanted, .. } = &mut *backing {
                let ticket = event.observe();
                *wanted = true;
                drop(backing);
                event.wait(ticket);
                continue;
            }
            break backing;
        };
        let location = match *backing {
            PageBacking::Zero => PageLocation::Zero,
            PageBacking::File => PageLocation::File,
            PageBacking::Swapped(_) => PageLocation::Swapped,
            _ => unreachable!(),
        };
        let previous = core::mem::replace(
            &mut *backing,
            PageBacking::Busy {
                location,
                wanted: false,
            },
        );
        drop(backing);

        // The busy state owns the source until completion. Reclaim and
        // pager I/O run without the backing lock; competing faults wait above.
        let loaded = (|| {
            let frame = if matches!(previous, PageBacking::Zero) {
                super::allocate_physical_page()?
            } else {
                super::allocate_pagein_physical_page()?
            };
            // SAFETY: this unpublished destination frame is exclusively ours.
            let output =
                unsafe { &mut *mem::phys_to_virt(frame.paddr()).as_mut_ptr::<[u8; PAGE_BYTES]>() };
            let result = match previous {
                PageBacking::Zero => Ok(()),
                PageBacking::File => self
                    .pager
                    .as_ref()
                    .ok_or(Error::Io)
                    .and_then(|pager| pager.read(self.index * PAGE_SIZE, output)),
                PageBacking::Swapped(handle) => {
                    swap::load(handle, output).map_err(|error| match error {
                        SwapError::Corrupt => Error::CorruptSwap,
                        _ => Error::SwapUnavailable,
                    })
                }
                _ => unreachable!(),
            };
            if let Err(error) = result {
                // SAFETY: no owner, mapping, wire or queue references this frame.
                unsafe { phys::free_page(frame) };
                return Err(error);
            }
            // A page-in is clean relative to its retained swap/file backing.
            // A new anonymous zero page has no backing slot and is dirty.
            if matches!(previous, PageBacking::Zero) {
                frame.mark_dirty();
            } else {
                frame.mark_clean();
                if matches!(previous, PageBacking::Swapped(_)) {
                    frame.mark_unknown();
                }
            }
            Ok(frame)
        })();

        let mut backing = self.backing.lock();
        let PageBacking::Busy { wanted, .. } = *backing else {
            unreachable!()
        };
        match loaded {
            Ok(frame) => {
                if let PageBacking::Swapped(handle) = previous {
                    swap::set_resident(handle, true);
                    *self.resident_swap.get_locked_mut(backing.capability_mut()) = Some(handle);
                }
                self.swap_reject_generation.store(0, Ordering::Relaxed);
                frame.bind_owner(self, self.owner_kind);
                frame.mark_referenced();
                *backing = PageBacking::Resident(frame);
                self.resident
                    .store(ptr::from_ref(frame).cast_mut(), Ordering::Release);
                phys::activate_managed(frame, self);
                if wanted {
                    event.notify();
                }
                Ok(backing)
            }
            Err(error) => {
                *backing = previous;
                drop(backing);
                if wanted {
                    event.notify();
                }
                Err(error)
            }
        }
    }

    fn mark_modified(self: &PageRef, frame: &'static phys::Page) {
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
        if let Some(handle) = self.resident_swap.get_mut().take() {
            swap::free(handle);
        }
        let backing = self.backing.get_mut();
        match *backing {
            PageBacking::Zero | PageBacking::File => {}
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
            PageBacking::Busy { .. } => unreachable!("mem: dropping a busy page"),
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
