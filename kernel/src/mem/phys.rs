//!
//! # Physical Memory Manager
//!
//! Module for managing physical memory (RAM), including APIs for alloc/free.
//!

use core::{
    mem::size_of,
    ptr,
    sync::atomic::{AtomicU8, Ordering},
};

use intrusive_collections::{LinkedList, LinkedListLink, intrusive_adapter};
use limine::memory_map::{Entry, EntryType};
use log::{debug, info};

use crate::{
    arch,
    mem::{
        self, PhysAddr, VirtAddr, VmFlags,
        addr::{PAGE_SIZE, align_down, align_up, pages_for_len},
    },
    sys::smp::IrqSpinLock,
};

/// Virtual base address where the PFN database (`PAGEDB`) is mapped.
pub const PAGEDB_ADDR: u64 = 0xFFFF_B000_0000_0000;

/// Pointer to the contiguous PFN database backing all physical page metadata.
const PAGEDB: *mut Page = PAGEDB_ADDR as *mut Page;

/// Per-page allocation state tracked by the physical memory manager.
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum PageState {
    /// Page is not currently allocatable (reserved or unknown at init).
    Reserved = 0,
    /// Page is allocated and in use.
    Used = 1,
    /// Page is free but contents are not guaranteed to be zero.
    Free = 2,
    /// Page is free and already zeroed.
    Zero = 3,
}

/// Metadata record for one physical page frame.
#[repr(C)]
pub struct Page {
    /// Intrusive linked-list link used by allocator free queues.
    link: LinkedListLink,
    /// Physical base address of this page.
    paddr: u64,
    /// Atomic page state, safe for concurrent readers.
    state: AtomicU8,
}

impl Page {
    /// Constructs a PFN entry for the given physical page base address.
    const fn new(paddr: u64) -> Self {
        Self {
            link: LinkedListLink::new(),
            paddr,
            state: AtomicU8::new(PageState::Reserved as u8),
        }
    }

    /// Returns this page's base physical address.
    pub fn paddr(&self) -> PhysAddr {
        PhysAddr::new(self.paddr)
    }

    /// Returns current allocator state of this page.
    pub fn state(&self) -> PageState {
        match self.state.load(Ordering::Relaxed) {
            0 => PageState::Reserved,
            1 => PageState::Used,
            2 => PageState::Free,
            3 => PageState::Zero,
            other => panic!("mem/phys: invalid page state {}", other),
        }
    }
}

// SAFETY: `Page` metadata lives in a permanently mapped global PFN database and
// concurrent mutation is coordinated through the PMM lock plus atomic state.
unsafe impl Send for Page {}
// SAFETY: shared references only expose immutable metadata reads and the atomic
// page state.
unsafe impl Sync for Page {}

intrusive_adapter!(PageAdapter = &'static Page: Page { link: LinkedListLink });

/// Internal PMM state guarded by a global IRQ-safe spinlock.
struct PmmState {
    /// Number of currently allocated pages.
    used_pages: usize,
    /// Number of free dirty pages.
    free_pages: usize,
    /// Number of physical pages covered by the PFN database.
    total_pages: usize,
    /// Number of pages consumed by the PFN database mapping itself.
    pagedb_page_count: usize,
    /// Physical base address of PFN database storage.
    pagedb_phys_base: u64,
    /// Queue of free dirty pages.
    free: LinkedList<PageAdapter>,
}

/// Global PMM instance. `None` means allocator has not been initialized yet.
static PMM: IrqSpinLock<Option<PmmState>> = IrqSpinLock::new(None);

/// Next bootstrap page-table allocation cursor used only during early init.
static mut BOOTSTRAP_NEXT: u64 = 0;

/// End of bootstrap page-table allocation range.
static mut BOOTSTRAP_END: u64 = 0;

/// Snapshot of physical page allocator usage.
#[derive(Copy, Clone, Debug, Default)]
pub struct PhysStats {
    /// Total physical pages tracked by the PFN database.
    pub total_pages: usize,
    /// Physical pages currently allocated.
    pub used_pages: usize,
    /// Physical pages currently free.
    pub free_pages: usize,
}

/// Initializes the physical memory manager and PFN database from Limine memory map entries.
pub fn init() {
    let mmap = mem::memory_map_entries();
    let mut max_usable_end = 0u64;

    debug!("mem/phys: memory map structure:");
    for e in mmap.iter() {
        let end = e.base.saturating_add(e.length);

        debug!(
            "mem/phys: \t[{:016x}-{:016x}] {}",
            e.base,
            end,
            entry_type_name(e)
        );

        if let Some((_, usable_end)) = usable_page_range(e) {
            max_usable_end = max_usable_end.max(usable_end);
        }
    }

    assert!(max_usable_end != 0, "mem/phys: no usable memory");

    let total_pages = pages_for_len(max_usable_end) as usize;
    let pagedb_pages = pages_for_len((size_of::<Page>() * total_pages) as u64);

    let l1 = pagedb_pages.div_ceil(512);
    let l2 = l1.div_ceil(512);
    let l3 = l2.div_ceil(512);
    let l4 = l3.div_ceil(512);

    let bootstrap_pages = l1 + l2 + l3 + l4 + 8;
    let chunk_size = (pagedb_pages + bootstrap_pages) * PAGE_SIZE;

    let mut pagedb_phys_base = None;
    let mut largest_usable = 0u64;

    for e in mmap {
        let Some((start, end)) = usable_page_range(e) else {
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

    // SAFETY: physical-memory initialization is single-threaded and these
    // cursors are only consumed by bootstrap page-table allocation.
    unsafe {
        BOOTSTRAP_NEXT = pagedb_phys_end;
        BOOTSTRAP_END = bootstrap_end;
    }

    // SAFETY: the selected physical chunk is exclusively reserved here, and
    // the PFN database virtual range is unused before these mappings.
    unsafe {
        let root = arch::paging::active_root();
        let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL;

        for i in 0..pagedb_pages {
            let va = VirtAddr::new(PAGEDB_ADDR + i * PAGE_SIZE);
            let pa = PhysAddr::new(pagedb_phys_base + i * PAGE_SIZE);
            arch::paging::map_page(root, va, pa, flags)
                .unwrap_or_else(|err| panic!("mem/phys: failed to map pagedb page {i}: {:?}", err));
        }

        for i in 0..total_pages {
            ptr::write(PAGEDB.add(i), Page::new((i as u64) * PAGE_SIZE));
        }
    }

    let mut state = PmmState {
        used_pages: 0,
        free_pages: 0,
        total_pages,
        pagedb_page_count: pagedb_pages as usize,
        pagedb_phys_base,
        free: LinkedList::new(PageAdapter::NEW),
    };

    for e in mmap {
        let Some((start, end)) = usable_page_range(e) else {
            continue;
        };
        let mut pa = start;

        while pa < end {
            // SAFETY: every usable physical page lies below `max_usable_end`,
            // which sized and initialized the PFN database.
            let page = unsafe { &*PAGEDB.add((pa / PAGE_SIZE) as usize) };

            if pa >= pagedb_phys_base && pa < bootstrap_end {
                page.state.store(PageState::Used as u8, Ordering::Relaxed);
                state.used_pages += 1;
            } else {
                page.state.store(PageState::Free as u8, Ordering::Relaxed);
                state.free.push_back(page);
                state.free_pages += 1;
            }

            pa += PAGE_SIZE;
        }
    }

    let bootstrap_reserved_pages = ((bootstrap_end - pagedb_phys_base) / PAGE_SIZE) as usize;
    assert_eq!(
        state.used_pages, bootstrap_reserved_pages,
        "mem/phys: bootstrap PFNDB reservation accounting mismatch"
    );

    info!(
        "mem/phys: pagedb active: phys=[0x{:x}-0x{:x}] used={} free={} bootstrap_pages={}",
        state.pagedb_phys_base,
        state.pagedb_phys_base + state.pagedb_page_count as u64 * PAGE_SIZE,
        state.used_pages,
        state.free_pages,
        bootstrap_pages,
    );

    *PMM.lock() = Some(state);

    // SAFETY: PMM publication ends all bootstrap allocations before the
    // single-threaded initialization phase completes.
    unsafe {
        BOOTSTRAP_NEXT = 0;
        BOOTSTRAP_END = 0;
    }
}

/// Converts a page-aligned physical address to its PFN database entry.
///
/// Returns `None` if PMM is not initialized, the address is unaligned,
/// or the page index falls outside PFN database coverage.
pub fn phys_to_page(pa: PhysAddr) -> Option<&'static Page> {
    if !pa.is_page_aligned() {
        return None;
    }

    let guard = PMM.lock();
    let st = guard.as_ref()?;
    let idx = (pa.as_u64() / PAGE_SIZE) as usize;

    if idx >= st.total_pages {
        return None;
    }
    drop(guard);

    // SAFETY: `idx` was bounds-checked against the initialized PFN database,
    // whose mapping remains live for the kernel lifetime.
    Some(unsafe { &*PAGEDB.add(idx) })
}

/// Allocates one physical page, zeroing it before returning.
pub fn alloc_page() -> Option<&'static Page> {
    let page = {
        let mut guard = PMM.lock();
        let st = guard.as_mut()?;
        let page = st.free.pop_front()?;
        st.free_pages -= 1;
        page.state.store(PageState::Used as u8, Ordering::Relaxed);
        st.used_pages += 1;
        page
    };

    zero_page(page.paddr());
    Some(page)
}

/// Allocates one zeroed physical page.
pub fn alloc_zeroed_page() -> Option<&'static Page> {
    alloc_page()
}

/// Allocates one zeroed physical page and returns its physical address.
///
/// If called before PMM init is complete, this falls back to the bootstrap
/// range used for page tables.
pub fn alloc_zeroed_phys() -> Option<PhysAddr> {
    if let Some(page) = alloc_zeroed_page() {
        return Some(page.paddr());
    }

    // SAFETY: this fallback is reachable only during single-threaded PMM
    // initialization before `PMM` is published.
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

/// Frees a page previously returned by allocation and places it on the free list.
///
/// # Safety
///
/// `page` must refer to a live PMM-managed page that is no longer accessed
/// through any virtual mapping, device, or alias once it is returned.
pub unsafe fn free_page(page: &'static Page) {
    let mut guard = PMM.lock();
    let st = guard.as_mut().expect("mem/phys: not initialized");

    if page.state() != PageState::Used {
        panic!(
            "mem/phys: free_page on non-used page (state={:?})",
            page.state()
        );
    }

    page.state.store(PageState::Free as u8, Ordering::Relaxed);
    st.used_pages -= 1;

    st.free.push_back(page);
    st.free_pages += 1;
}

/// Returns a point-in-time snapshot of physical page allocator usage.
pub fn stats() -> Option<PhysStats> {
    let guard = PMM.lock();
    let st = guard.as_ref()?;
    Some(PhysStats {
        total_pages: st.total_pages,
        used_pages: st.used_pages,
        free_pages: st.free_pages,
    })
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
    // SAFETY: callers exclusively own `pa`, and the HHDM permanently maps the
    // complete page writable.
    unsafe {
        ptr::write_bytes(
            mem::phys_to_virt(pa).as_mut_ptr::<u8>(),
            0,
            PAGE_SIZE as usize,
        );
    }
}
