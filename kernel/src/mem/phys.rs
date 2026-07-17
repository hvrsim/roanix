//!
//! # Physical Memory Manager
//!
//! PFN database, physical-page lifecycle, free/zero queues, per-CPU caches,
//! and managed-page queues.
//!

use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    mem::size_of,
    ptr,
    sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, AtomicUsize, Ordering},
};

use bitflags::bitflags;
use intrusive_collections::{LinkedList, LinkedListLink, intrusive_adapter};
use limine::memory_map::{Entry, EntryType};
use log::{debug, info};

use crate::{
    arch,
    mem::{
        self, PhysAddr, VirtAddr, VmFlags,
        addr::{PAGE_SIZE, align_down, align_up, pages_for_len},
        page::VmPage,
        pmap::PmapInner,
    },
    sys::smp::IrqSpinLock,
};

/// Virtual base address where the PFN database is mapped.
pub const PAGEDB_ADDR: u64 = 0xFFFF_B000_0000_0000;

const PAGEDB: *mut Page = PAGEDB_ADDR as *mut Page;
const MAX_PAGE_CPUS: usize = 256;
const PAGE_CACHE_CAPACITY: usize = 16;
const PAGE_CACHE_REFILL: usize = 8;

bitflags! {
    /// Allocation requirements for one physical page.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct PageAllocFlags: u8 {
        /// Return a page whose contents are all zero.
        const ZERO = 1 << 0;
    }
}

bitflags! {
    /// State flags carried by every PFN database entry.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct PageFlags: u32 {
        /// Page contents differ from their backing store.
        const DIRTY = 1 << 0;
        /// Page was accessed during the current aging interval.
        const REFERENCED = 1 << 1;
        /// Page is exclusively held for an in-progress transition.
        const BUSY = 1 << 2;
        /// A waiter observed the busy page.
        const WANTED = 1 << 3;
        /// Page should be released when its busy transition completes.
        const RELEASED = 1 << 4;
        /// Page is present in an object or anonymous-page index.
        const TABLED = 1 << 5;
        /// Page belongs to anonymous memory.
        const ANON = 1 << 6;
        /// Page belongs to a vnode-backed object.
        const FILE = 1 << 7;
    }
}

/// Allocator-visible page state.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageState {
    /// Memory is not available to the allocator.
    Reserved = 0,
    /// Free page with unspecified contents.
    Free = 1,
    /// Free page whose contents are all zero.
    Zero = 2,
    /// Page is allocated to a subsystem.
    Allocated = 3,
}

/// Current physical-page consumer.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageUse {
    /// No active consumer.
    None = 0,
    /// Pageable object or anonymous memory.
    Managed = 1,
    /// Kernel heap backing.
    KernelHeap = 2,
    /// Architecture page-table page.
    PageTable = 3,
    /// PFN database and bootstrap mappings.
    PfnDatabase = 4,
    /// Firmware, device, or otherwise unavailable memory.
    Reserved = 5,
}

/// Managed-page owner category.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageOwnerKind {
    /// No managed owner.
    None = 0,
    /// Anonymous overlay page.
    Anonymous = 1,
    /// Vnode-backed page-cache page.
    File = 2,
    /// Kernel pageable object.
    Kernel = 3,
}

/// Page-daemon queue membership.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageQueue {
    /// Not queued for reclamation.
    None = 0,
    /// Recently used page.
    Active = 1,
    /// Reclamation candidate.
    Inactive = 2,
}

/// Authoritative metadata for one physical page frame.
#[repr(C, align(64))]
pub struct Page {
    free_link: LinkedListLink,
    pageq_link: LinkedListLink,
    paddr: u64,
    state: AtomicU8,
    usage: AtomicU8,
    queue: AtomicU8,
    owner_kind: AtomicU8,
    flags: AtomicU32,
    wire_count: AtomicU32,
    loan_count: AtomicU32,
    map_count: AtomicU32,
    owner_id: AtomicU64,
    offset: AtomicU64,
    owner: IrqSpinLock<Option<Weak<VmPage>>>,
    reverse_mappings: IrqSpinLock<Vec<ReverseMapping>>,
}

pub(super) struct ReverseMapping {
    pub(super) pmap: Weak<PmapInner>,
    pub(super) address: u64,
}

impl Page {
    fn new(paddr: u64) -> Self {
        Self {
            free_link: LinkedListLink::new(),
            pageq_link: LinkedListLink::new(),
            paddr,
            state: AtomicU8::new(PageState::Reserved as u8),
            usage: AtomicU8::new(PageUse::Reserved as u8),
            queue: AtomicU8::new(PageQueue::None as u8),
            owner_kind: AtomicU8::new(PageOwnerKind::None as u8),
            flags: AtomicU32::new(0),
            wire_count: AtomicU32::new(0),
            loan_count: AtomicU32::new(0),
            map_count: AtomicU32::new(0),
            owner_id: AtomicU64::new(0),
            offset: AtomicU64::new(0),
            owner: IrqSpinLock::new(None),
            reverse_mappings: IrqSpinLock::new(Vec::new()),
        }
    }

    /// Returns the physical base address.
    pub fn paddr(&self) -> PhysAddr {
        PhysAddr::new(self.paddr)
    }

    /// Returns the PFN index.
    pub fn pfn(&self) -> usize {
        (self.paddr / PAGE_SIZE) as usize
    }

    /// Returns allocator state.
    pub fn state(&self) -> PageState {
        decode_page_state(self.state.load(Ordering::Acquire))
    }

    /// Returns the current allocation consumer.
    pub fn usage(&self) -> PageUse {
        decode_page_use(self.usage.load(Ordering::Acquire))
    }

    /// Returns page-daemon queue membership.
    pub fn queue(&self) -> PageQueue {
        decode_page_queue(self.queue.load(Ordering::Acquire))
    }

    /// Returns managed owner category.
    pub fn owner_kind(&self) -> PageOwnerKind {
        decode_owner_kind(self.owner_kind.load(Ordering::Acquire))
    }

    /// Returns current page flags.
    pub fn flags(&self) -> PageFlags {
        PageFlags::from_bits_retain(self.flags.load(Ordering::Acquire))
    }

    /// Returns the managed owner identifier, or zero if unowned.
    pub fn owner_id(&self) -> u64 {
        self.owner_id.load(Ordering::Acquire)
    }

    /// Returns the page index within its owner.
    pub fn offset(&self) -> u64 {
        self.offset.load(Ordering::Acquire)
    }

    /// Returns the current wire count.
    pub fn wire_count(&self) -> u32 {
        self.wire_count.load(Ordering::Acquire)
    }

    /// Returns the current loan count.
    pub fn loan_count(&self) -> u32 {
        self.loan_count.load(Ordering::Acquire)
    }

    /// Returns the current hardware mapping count.
    pub fn map_count(&self) -> u32 {
        self.map_count.load(Ordering::Acquire)
    }

    pub(crate) fn bind_owner(
        &'static self,
        owner: &Arc<VmPage>,
        kind: PageOwnerKind,
        owner_id: u64,
        offset: u64,
    ) {
        assert_eq!(self.usage(), PageUse::Managed);
        assert_eq!(self.owner_kind(), PageOwnerKind::None);
        *self.owner.lock() = Some(Arc::downgrade(owner));
        self.owner_id.store(owner_id, Ordering::Release);
        self.offset.store(offset, Ordering::Release);
        self.owner_kind.store(kind as u8, Ordering::Release);
        let owner_flag = match kind {
            PageOwnerKind::Anonymous => PageFlags::ANON,
            PageOwnerKind::File => PageFlags::FILE,
            PageOwnerKind::Kernel | PageOwnerKind::None => PageFlags::empty(),
        };
        self.flags
            .fetch_or((PageFlags::TABLED | owner_flag).bits(), Ordering::AcqRel);
        MANAGED_PAGES.fetch_add(1, Ordering::AcqRel);
    }

    pub(crate) fn clear_owner(&self) {
        if self.owner_kind() != PageOwnerKind::None {
            MANAGED_PAGES.fetch_sub(1, Ordering::AcqRel);
        }
        *self.owner.lock() = None;
        self.owner_id.store(0, Ordering::Release);
        self.offset.store(0, Ordering::Release);
        self.owner_kind
            .store(PageOwnerKind::None as u8, Ordering::Release);
        self.flags.fetch_and(
            !(PageFlags::TABLED | PageFlags::ANON | PageFlags::FILE).bits(),
            Ordering::AcqRel,
        );
    }

    /// Adds one wire that prevents reclamation.
    pub fn wire(&self) {
        self.wire_count.fetch_add(1, Ordering::AcqRel);
    }

    /// Removes one wire.
    pub fn unwire(&self) {
        let previous = self.wire_count.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "mem/phys: page wire count underflow");
    }

    /// Adds one transient page loan.
    pub fn loan(&self) {
        self.loan_count.fetch_add(1, Ordering::AcqRel);
    }

    /// Removes one transient page loan.
    pub fn unloan(&self) {
        let previous = self.loan_count.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "mem/phys: page loan count underflow");
    }

    pub(crate) fn add_mapping(&self) {
        self.map_count.fetch_add(1, Ordering::AcqRel);
    }

    pub(crate) fn remove_mapping(&self) {
        let previous = self.map_count.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "mem/phys: page mapping count underflow");
    }

    pub(super) fn add_reverse_mapping(&self, pmap: &Arc<PmapInner>, address: u64) {
        self.reverse_mappings.lock().push(ReverseMapping {
            pmap: Arc::downgrade(pmap),
            address,
        });
    }

    pub(super) fn remove_reverse_mapping(&self, pmap: *const PmapInner, address: u64) {
        let mut mappings = self.reverse_mappings.lock();
        if let Some(index) = mappings
            .iter()
            .position(|mapping| mapping.pmap.as_ptr() == pmap && mapping.address == address)
        {
            mappings.swap_remove(index);
        }
    }

    pub(super) fn reverse_mapping(&self, index: usize) -> Option<ReverseMapping> {
        let mappings = self.reverse_mappings.lock();
        let mapping = mappings.get(index)?;
        Some(ReverseMapping {
            pmap: mapping.pmap.clone(),
            address: mapping.address,
        })
    }

    pub(crate) fn mark_dirty(&self) {
        self.flags
            .fetch_or(PageFlags::DIRTY.bits(), Ordering::Release);
    }

    pub(crate) fn mark_clean(&self) {
        self.flags
            .fetch_and(!PageFlags::DIRTY.bits(), Ordering::Release);
    }

    pub(crate) fn mark_referenced(&self) {
        self.flags
            .fetch_or(PageFlags::REFERENCED.bits(), Ordering::Release);
    }

    pub(crate) fn take_referenced(&self) -> bool {
        self.flags
            .fetch_and(!PageFlags::REFERENCED.bits(), Ordering::AcqRel)
            & PageFlags::REFERENCED.bits()
            != 0
    }

    pub(crate) fn try_busy(&self) -> bool {
        let mut flags = self.flags.load(Ordering::Acquire);
        loop {
            if flags & PageFlags::BUSY.bits() != 0 {
                self.flags
                    .fetch_or(PageFlags::WANTED.bits(), Ordering::AcqRel);
                return false;
            }
            match self.flags.compare_exchange_weak(
                flags,
                flags | PageFlags::BUSY.bits(),
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return true,
                Err(observed) => flags = observed,
            }
        }
    }

    pub(crate) fn unbusy(&self) -> PageFlags {
        let previous = self.flags.fetch_and(
            !(PageFlags::BUSY | PageFlags::WANTED).bits(),
            Ordering::AcqRel,
        );
        PageFlags::from_bits_retain(previous)
    }

    pub(crate) fn reclaimable(&self) -> bool {
        self.wire_count() == 0
            && self.loan_count() == 0
            && self.map_count() == 0
            && !self.flags().contains(PageFlags::BUSY)
    }

    fn owner(&self) -> Option<Arc<VmPage>> {
        self.owner.lock().as_ref()?.upgrade()
    }

    fn prepare_allocation(&self, usage: PageUse) {
        assert!(matches!(self.state(), PageState::Free | PageState::Zero));
        assert!(!self.free_link.is_linked());
        assert!(!self.pageq_link.is_linked());
        self.flags.store(0, Ordering::Release);
        self.wire_count.store(0, Ordering::Release);
        self.loan_count.store(0, Ordering::Release);
        self.map_count.store(0, Ordering::Release);
        self.owner_id.store(0, Ordering::Release);
        self.offset.store(0, Ordering::Release);
        self.owner_kind
            .store(PageOwnerKind::None as u8, Ordering::Release);
        *self.owner.lock() = None;
        assert!(self.reverse_mappings.lock().is_empty());
        self.queue.store(PageQueue::None as u8, Ordering::Release);
        self.usage.store(usage as u8, Ordering::Release);
        self.state
            .store(PageState::Allocated as u8, Ordering::Release);
    }

    fn prepare_free(&self, zeroed: bool) {
        assert_eq!(self.state(), PageState::Allocated);
        assert_eq!(self.owner_kind(), PageOwnerKind::None);
        assert_eq!(self.wire_count(), 0);
        assert_eq!(self.loan_count(), 0);
        assert_eq!(self.map_count(), 0);
        let stale_capacity = {
            let mut mappings = self.reverse_mappings.lock();
            assert!(mappings.is_empty());
            core::mem::take(&mut *mappings)
        };
        drop(stale_capacity);
        assert!(!self.pageq_link.is_linked());
        self.flags.store(0, Ordering::Release);
        self.usage.store(PageUse::None as u8, Ordering::Release);
        self.queue.store(PageQueue::None as u8, Ordering::Release);
        self.state.store(
            if zeroed {
                PageState::Zero
            } else {
                PageState::Free
            } as u8,
            Ordering::Release,
        );
    }
}

// SAFETY: PFN entries have stable addresses and all interior mutation is
// atomic or protected by IRQ-safe locks and allocator queue locks.
unsafe impl Send for Page {}
// SAFETY: shared access observes atomic fields or lock-protected owner state.
unsafe impl Sync for Page {}

intrusive_adapter!(FreePageAdapter = &'static Page: Page { free_link: LinkedListLink });
intrusive_adapter!(ManagedPageAdapter = &'static Page: Page { pageq_link: LinkedListLink });

struct PmmState {
    free: LinkedList<FreePageAdapter>,
    zero: LinkedList<FreePageAdapter>,
    active: LinkedList<ManagedPageAdapter>,
    inactive: LinkedList<ManagedPageAdapter>,
}

struct PageCache {
    dirty: [u32; PAGE_CACHE_CAPACITY],
    dirty_len: usize,
    zero: [u32; PAGE_CACHE_CAPACITY],
    zero_len: usize,
}

impl PageCache {
    const fn new() -> Self {
        Self {
            dirty: [0; PAGE_CACHE_CAPACITY],
            dirty_len: 0,
            zero: [0; PAGE_CACHE_CAPACITY],
            zero_len: 0,
        }
    }

    fn pop(&mut self, zero: bool) -> Option<u32> {
        let (slots, len) = if zero {
            (&mut self.zero, &mut self.zero_len)
        } else {
            (&mut self.dirty, &mut self.dirty_len)
        };
        if *len == 0 {
            return None;
        }
        *len -= 1;
        Some(slots[*len])
    }

    fn push(&mut self, pfn: u32, zero: bool) -> bool {
        let (slots, len) = if zero {
            (&mut self.zero, &mut self.zero_len)
        } else {
            (&mut self.dirty, &mut self.dirty_len)
        };
        if *len == PAGE_CACHE_CAPACITY {
            return false;
        }
        slots[*len] = pfn;
        *len += 1;
        true
    }
}

static PMM: IrqSpinLock<Option<PmmState>> = IrqSpinLock::new(None);
static PAGE_CACHES: [IrqSpinLock<PageCache>; MAX_PAGE_CPUS] =
    [const { IrqSpinLock::new(PageCache::new()) }; MAX_PAGE_CPUS];
static READY: AtomicBool = AtomicBool::new(false);
static DATABASE_ENTRIES: AtomicUsize = AtomicUsize::new(0);
static TOTAL_PAGES: AtomicUsize = AtomicUsize::new(0);
static USED_PAGES: AtomicUsize = AtomicUsize::new(0);
static FREE_PAGES: AtomicUsize = AtomicUsize::new(0);
static ZERO_PAGES: AtomicUsize = AtomicUsize::new(0);
static MANAGED_PAGES: AtomicUsize = AtomicUsize::new(0);

static mut BOOTSTRAP_NEXT: u64 = 0;
static mut BOOTSTRAP_END: u64 = 0;

/// Snapshot of physical page usage.
#[derive(Copy, Clone, Debug, Default)]
pub struct PhysStats {
    /// Usable physical pages known to the allocator.
    pub total_pages: usize,
    /// Pages currently allocated.
    pub used_pages: usize,
    /// Free pages, including pre-zeroed pages and per-CPU caches.
    pub free_pages: usize,
    /// Free pages known to contain zeros.
    pub zero_pages: usize,
    /// Resident pages owned by anonymous memory or memory objects.
    pub managed_pages: usize,
}

/// Initializes the PFN database and page queues from the boot memory map.
pub fn init() {
    let mmap = mem::memory_map_entries();
    let mut max_usable_end = 0u64;
    let mut usable_pages = 0usize;

    debug!("mem/phys: memory map structure:");
    for entry in mmap.iter() {
        let end = entry.base.saturating_add(entry.length);
        debug!(
            "mem/phys: \t[{:016x}-{:016x}] {}",
            entry.base,
            end,
            entry_type_name(entry)
        );
        if let Some((start, usable_end)) = usable_page_range(entry) {
            max_usable_end = max_usable_end.max(usable_end);
            usable_pages += ((usable_end - start) / PAGE_SIZE) as usize;
        }
    }
    assert!(max_usable_end != 0, "mem/phys: no usable memory");

    let database_entries = pages_for_len(max_usable_end) as usize;
    assert!(
        database_entries <= u32::MAX as usize,
        "mem/phys: PFN cache encoding supports at most {} pages",
        u32::MAX
    );
    let pagedb_pages = pages_for_len((size_of::<Page>() * database_entries) as u64);
    let l1 = pagedb_pages.div_ceil(512);
    let l2 = l1.div_ceil(512);
    let l3 = l2.div_ceil(512);
    let l4 = l3.div_ceil(512);
    let bootstrap_pages = l1 + l2 + l3 + l4 + 8;
    let chunk_size = (pagedb_pages + bootstrap_pages) * PAGE_SIZE;

    let mut pagedb_phys_base = None;
    let mut largest_usable = 0u64;
    for entry in mmap {
        let Some((start, end)) = usable_page_range(entry) else {
            continue;
        };
        let span = end.saturating_sub(start);
        largest_usable = largest_usable.max(span);
        if span >= chunk_size {
            pagedb_phys_base = Some(start);
            break;
        }
    }
    let pagedb_phys_base = pagedb_phys_base.unwrap_or_else(|| {
        panic!(
            "mem/phys: no usable chunk for PFNDB (need {} KiB, largest {} KiB)",
            chunk_size / 1024,
            largest_usable / 1024
        )
    });
    let pagedb_phys_end = pagedb_phys_base + pagedb_pages * PAGE_SIZE;
    let bootstrap_end = pagedb_phys_base + chunk_size;

    // SAFETY: bootstrap allocation is single-threaded and bounded by the
    // reserved PFN database chunk.
    unsafe {
        BOOTSTRAP_NEXT = pagedb_phys_end;
        BOOTSTRAP_END = bootstrap_end;
    }

    // SAFETY: the PFN database virtual range and selected physical chunk are
    // exclusively reserved during early memory initialization.
    unsafe {
        let root = arch::paging::active_root();
        let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL;
        for i in 0..pagedb_pages {
            let va = VirtAddr::new(PAGEDB_ADDR + i * PAGE_SIZE);
            let pa = PhysAddr::new(pagedb_phys_base + i * PAGE_SIZE);
            arch::paging::map_page(root, va, pa, flags)
                .unwrap_or_else(|error| panic!("mem/phys: map PFNDB page {i}: {error:?}"));
        }
        for index in 0..database_entries {
            ptr::write(PAGEDB.add(index), Page::new(index as u64 * PAGE_SIZE));
        }
    }

    let mut state = PmmState {
        free: LinkedList::new(FreePageAdapter::NEW),
        zero: LinkedList::new(FreePageAdapter::NEW),
        active: LinkedList::new(ManagedPageAdapter::NEW),
        inactive: LinkedList::new(ManagedPageAdapter::NEW),
    };
    let mut used_pages = 0usize;
    let mut free_pages = 0usize;

    for entry in mmap {
        let Some((start, end)) = usable_page_range(entry) else {
            continue;
        };
        let mut pa = start;
        while pa < end {
            // SAFETY: every usable page is covered by the initialized PFN DB.
            let page = unsafe { &*PAGEDB.add((pa / PAGE_SIZE) as usize) };
            if pa >= pagedb_phys_base && pa < bootstrap_end {
                page.state
                    .store(PageState::Allocated as u8, Ordering::Relaxed);
                page.usage
                    .store(PageUse::PfnDatabase as u8, Ordering::Relaxed);
                used_pages += 1;
            } else {
                page.state.store(PageState::Free as u8, Ordering::Relaxed);
                page.usage.store(PageUse::None as u8, Ordering::Relaxed);
                state.free.push_back(page);
                free_pages += 1;
            }
            pa += PAGE_SIZE;
        }
    }

    DATABASE_ENTRIES.store(database_entries, Ordering::Release);
    TOTAL_PAGES.store(usable_pages, Ordering::Release);
    USED_PAGES.store(used_pages, Ordering::Release);
    FREE_PAGES.store(free_pages, Ordering::Release);
    ZERO_PAGES.store(0, Ordering::Release);
    MANAGED_PAGES.store(0, Ordering::Release);
    *PMM.lock() = Some(state);
    READY.store(true, Ordering::Release);

    // SAFETY: publication ends all bootstrap allocations.
    unsafe {
        BOOTSTRAP_NEXT = 0;
        BOOTSTRAP_END = 0;
    }

    info!(
        "mem/phys: PFNDB phys=[0x{:x}-0x{:x}] entries={} metadata={} KiB used={} free={}",
        pagedb_phys_base,
        pagedb_phys_base + pagedb_pages * PAGE_SIZE,
        database_entries,
        pagedb_pages * PAGE_SIZE / 1024,
        used_pages,
        free_pages,
    );
}

/// Converts a page-aligned physical address to its PFN database entry.
pub fn phys_to_page(pa: PhysAddr) -> Option<&'static Page> {
    if !READY.load(Ordering::Acquire) || !pa.is_page_aligned() {
        return None;
    }
    let index = (pa.as_u64() / PAGE_SIZE) as usize;
    page_by_index(index)
}

/// Allocates one physical page for `usage`.
pub fn alloc_page(usage: PageUse, flags: PageAllocFlags) -> Option<&'static Page> {
    if !READY.load(Ordering::Acquire) {
        return None;
    }
    assert!(
        !matches!(
            usage,
            PageUse::None | PageUse::Reserved | PageUse::PfnDatabase
        ),
        "mem/phys: invalid dynamic page use {usage:?}"
    );

    let want_zero = flags.contains(PageAllocFlags::ZERO);
    let (page, was_zero) = take_page(want_zero)?;
    FREE_PAGES.fetch_sub(1, Ordering::AcqRel);
    USED_PAGES.fetch_add(1, Ordering::AcqRel);
    if was_zero {
        ZERO_PAGES.fetch_sub(1, Ordering::AcqRel);
    }
    page.prepare_allocation(usage);
    if want_zero && !was_zero {
        zero_page(page.paddr());
    }
    Some(page)
}

/// Allocates one zeroed physical page for `usage`.
pub fn alloc_zeroed_page(usage: PageUse) -> Option<&'static Page> {
    alloc_page(usage, PageAllocFlags::ZERO)
}

/// Allocates one zeroed page and returns its physical address.
///
/// Before PFN database publication, page-table callers consume the bounded
/// bootstrap range reserved alongside the database.
pub fn alloc_zeroed_phys(usage: PageUse) -> Option<PhysAddr> {
    if let Some(page) = alloc_zeroed_page(usage) {
        return Some(page.paddr());
    }
    if READY.load(Ordering::Acquire) {
        return None;
    }

    // SAFETY: only single-threaded bootstrap page-table construction reaches
    // this path before the PFN database is published.
    let pa = unsafe {
        if BOOTSTRAP_NEXT >= BOOTSTRAP_END {
            return None;
        }
        let pa = BOOTSTRAP_NEXT;
        BOOTSTRAP_NEXT += PAGE_SIZE;
        PhysAddr::new(pa)
    };
    zero_page(pa);
    Some(pa)
}

/// Frees an allocated page whose contents should be treated as dirty.
///
/// # Safety
///
/// No virtual mapping, device, owner, loan, or concurrent access may reference
/// `page` after this call.
pub unsafe fn free_page(page: &'static Page) {
    // SAFETY: upheld by the caller; free-page publication happens after every
    // ownership and mapping invariant is checked by `prepare_free`.
    unsafe { free_page_inner(page, false) };
}

/// Frees an allocated page known to contain only zeros.
///
/// # Safety
///
/// The same requirements as [`free_page`] apply, and every byte must be zero.
pub unsafe fn free_zeroed_page(page: &'static Page) {
    // SAFETY: upheld by the caller.
    unsafe { free_page_inner(page, true) };
}

/// Converts up to `maximum` globally queued dirty pages into zero pages.
pub fn zero_free_pages(maximum: usize) -> usize {
    if maximum == 0 || !READY.load(Ordering::Acquire) {
        return 0;
    }
    let mut pages = [None; PAGE_CACHE_REFILL];
    let target = maximum.min(PAGE_CACHE_REFILL);
    let count = {
        let mut guard = PMM.lock();
        let state = guard.as_mut().expect("mem/phys: not initialized");
        let mut count = 0usize;
        while count < target {
            let Some(page) = state.free.pop_front() else {
                break;
            };
            pages[count] = Some(page);
            count += 1;
        }
        count
    };

    for page in pages[..count].iter().flatten() {
        zero_page(page.paddr());
        page.state.store(PageState::Zero as u8, Ordering::Release);
    }
    if count != 0 {
        let mut guard = PMM.lock();
        let state = guard.as_mut().expect("mem/phys: not initialized");
        for page in pages[..count].iter().flatten() {
            state.zero.push_back(*page);
        }
        ZERO_PAGES.fetch_add(count, Ordering::AcqRel);
    }
    count
}

/// Returns current physical page accounting.
pub fn stats() -> Option<PhysStats> {
    READY.load(Ordering::Acquire).then(|| PhysStats {
        total_pages: TOTAL_PAGES.load(Ordering::Acquire),
        used_pages: USED_PAGES.load(Ordering::Acquire),
        free_pages: FREE_PAGES.load(Ordering::Acquire),
        zero_pages: ZERO_PAGES.load(Ordering::Acquire),
        managed_pages: MANAGED_PAGES.load(Ordering::Acquire),
    })
}

pub(crate) fn activate_managed(page: &'static Page, owner: &Arc<VmPage>) {
    if page.queue() != PageQueue::None {
        return;
    }
    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    if page.queue() == PageQueue::None {
        *page.owner.lock() = Some(Arc::downgrade(owner));
        page.queue.store(PageQueue::Active as u8, Ordering::Release);
        state.active.push_back(page);
    }
}

pub(crate) fn remove_managed(page: &'static Page) {
    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    match page.queue() {
        PageQueue::None => {}
        PageQueue::Active => {
            // SAFETY: queue state and the PMM lock prove list membership.
            unsafe {
                state
                    .active
                    .cursor_mut_from_ptr(page as *const Page)
                    .remove();
            }
        }
        PageQueue::Inactive => {
            // SAFETY: queue state and the PMM lock prove list membership.
            unsafe {
                state
                    .inactive
                    .cursor_mut_from_ptr(page as *const Page)
                    .remove();
            }
        }
    }
    page.queue.store(PageQueue::None as u8, Ordering::Release);
}

pub(crate) fn age_active(maximum: usize) {
    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    for _ in 0..maximum {
        let Some(page) = state.active.pop_front() else {
            break;
        };
        let _ = page.take_referenced();
        page.queue
            .store(PageQueue::Inactive as u8, Ordering::Release);
        state.inactive.push_back(page);
    }
}

pub(crate) fn next_inactive_owner() -> Option<Arc<VmPage>> {
    loop {
        let (page, owner) = {
            let mut guard = PMM.lock();
            let state = guard.as_mut().expect("mem/phys: not initialized");
            let page = state.inactive.pop_front()?;
            page.queue.store(PageQueue::None as u8, Ordering::Release);
            let owner = page.owner();
            (page, owner)
        };
        let Some(owner) = owner else {
            continue;
        };
        if page.take_referenced() {
            activate_managed(page, &owner);
            continue;
        }
        return Some(owner);
    }
}

pub(crate) fn managed_queues_empty() -> bool {
    let guard = PMM.lock();
    let state = guard.as_ref().expect("mem/phys: not initialized");
    state.active.is_empty() && state.inactive.is_empty()
}

fn take_page(want_zero: bool) -> Option<(&'static Page, bool)> {
    let cache_id = cache_id();
    if let Some(result) = pop_cache(cache_id, want_zero) {
        return Some(result);
    }
    refill_cache(cache_id, want_zero);
    if let Some(result) = pop_cache(cache_id, want_zero) {
        return Some(result);
    }
    steal_cached_page(cache_id, want_zero)
}

fn pop_cache(cache_id: usize, want_zero: bool) -> Option<(&'static Page, bool)> {
    let mut cache = PAGE_CACHES[cache_id].lock();
    if want_zero {
        if let Some(pfn) = cache.pop(true) {
            return page_by_index(pfn as usize).map(|page| (page, true));
        }
        if let Some(pfn) = cache.pop(false) {
            return page_by_index(pfn as usize).map(|page| (page, false));
        }
    } else {
        if let Some(pfn) = cache.pop(false) {
            return page_by_index(pfn as usize).map(|page| (page, false));
        }
        if let Some(pfn) = cache.pop(true) {
            return page_by_index(pfn as usize).map(|page| (page, true));
        }
    }
    None
}

fn refill_cache(cache_id: usize, want_zero: bool) {
    let mut cache = PAGE_CACHES[cache_id].lock();
    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    for _ in 0..PAGE_CACHE_REFILL {
        let page = if want_zero {
            state
                .zero
                .pop_front()
                .map(|page| (page, true))
                .or_else(|| state.free.pop_front().map(|page| (page, false)))
        } else {
            state
                .free
                .pop_front()
                .map(|page| (page, false))
                .or_else(|| state.zero.pop_front().map(|page| (page, true)))
        };
        let Some((page, zero)) = page else {
            return;
        };
        if !cache.push(page.pfn() as u32, zero) {
            if zero {
                state.zero.push_front(page);
            } else {
                state.free.push_front(page);
            }
            return;
        }
    }
}

fn steal_cached_page(local_id: usize, want_zero: bool) -> Option<(&'static Page, bool)> {
    for (cache_id, cache_lock) in PAGE_CACHES.iter().enumerate() {
        if cache_id == local_id {
            continue;
        }
        let mut cache = cache_lock.lock();
        let result = if want_zero {
            cache
                .pop(true)
                .map(|pfn| (pfn, true))
                .or_else(|| cache.pop(false).map(|pfn| (pfn, false)))
        } else {
            cache
                .pop(false)
                .map(|pfn| (pfn, false))
                .or_else(|| cache.pop(true).map(|pfn| (pfn, true)))
        };
        if let Some((pfn, zero)) = result {
            return page_by_index(pfn as usize).map(|page| (page, zero));
        }
    }
    None
}

unsafe fn free_page_inner(page: &'static Page, zeroed: bool) {
    page.prepare_free(zeroed);
    USED_PAGES.fetch_sub(1, Ordering::AcqRel);
    FREE_PAGES.fetch_add(1, Ordering::AcqRel);
    if zeroed {
        ZERO_PAGES.fetch_add(1, Ordering::AcqRel);
    }

    let cache_id = cache_id();
    let mut cache = PAGE_CACHES[cache_id].lock();
    if cache.push(page.pfn() as u32, zeroed) {
        return;
    }

    let mut spill = [0u32; PAGE_CACHE_REFILL];
    let mut count = 0usize;
    while count < PAGE_CACHE_REFILL {
        let Some(pfn) = cache.pop(zeroed) else {
            break;
        };
        spill[count] = pfn;
        count += 1;
    }
    assert!(cache.push(page.pfn() as u32, zeroed));
    drop(cache);

    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    for pfn in spill[..count].iter().copied() {
        let page = page_by_index(pfn as usize).expect("mem/phys: invalid cached PFN");
        if zeroed {
            state.zero.push_back(page);
        } else {
            state.free.push_back(page);
        }
    }
}

fn page_by_index(index: usize) -> Option<&'static Page> {
    if index >= DATABASE_ENTRIES.load(Ordering::Acquire) {
        return None;
    }
    // SAFETY: the PFN database is permanently mapped and `index` is bounded.
    Some(unsafe { &*PAGEDB.add(index) })
}

fn cache_id() -> usize {
    let id = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
    assert!(
        id < MAX_PAGE_CPUS,
        "mem/phys: cpu {id} exceeds page-cache capacity"
    );
    id
}

fn entry_type_name(entry: &Entry) -> &'static str {
    match entry.entry_type {
        EntryType::USABLE => "usable",
        EntryType::RESERVED => "reserved",
        EntryType::ACPI_RECLAIMABLE => "acpi_reclaimable",
        EntryType::ACPI_NVS => "acpi_nvs",
        EntryType::BAD_MEMORY => "bad_memory",
        EntryType::BOOTLOADER_RECLAIMABLE => "bootloader_reclaimable",
        EntryType::EXECUTABLE_AND_MODULES => "executable_and_modules",
        EntryType::FRAMEBUFFER => "framebuffer",
        _ => "unknown",
    }
}

fn usable_page_range(entry: &Entry) -> Option<(u64, u64)> {
    if entry.entry_type != EntryType::USABLE {
        return None;
    }
    let start = align_up(entry.base, PAGE_SIZE);
    let end = align_down(entry.base.saturating_add(entry.length), PAGE_SIZE);
    (start < end).then_some((start, end))
}

fn zero_page(pa: PhysAddr) {
    // SAFETY: callers exclusively own `pa`, and HHDM maps the complete frame.
    unsafe {
        ptr::write_bytes(
            mem::phys_to_virt(pa).as_mut_ptr::<u8>(),
            0,
            PAGE_SIZE as usize,
        );
    }
}

fn decode_page_state(value: u8) -> PageState {
    match value {
        0 => PageState::Reserved,
        1 => PageState::Free,
        2 => PageState::Zero,
        3 => PageState::Allocated,
        _ => panic!("mem/phys: invalid page state {value}"),
    }
}

fn decode_page_use(value: u8) -> PageUse {
    match value {
        0 => PageUse::None,
        1 => PageUse::Managed,
        2 => PageUse::KernelHeap,
        3 => PageUse::PageTable,
        4 => PageUse::PfnDatabase,
        5 => PageUse::Reserved,
        _ => panic!("mem/phys: invalid page use {value}"),
    }
}

fn decode_page_queue(value: u8) -> PageQueue {
    match value {
        0 => PageQueue::None,
        1 => PageQueue::Active,
        2 => PageQueue::Inactive,
        _ => panic!("mem/phys: invalid page queue {value}"),
    }
}

fn decode_owner_kind(value: u8) -> PageOwnerKind {
    match value {
        0 => PageOwnerKind::None,
        1 => PageOwnerKind::Anonymous,
        2 => PageOwnerKind::File,
        3 => PageOwnerKind::Kernel,
        _ => panic!("mem/phys: invalid page owner {value}"),
    }
}
