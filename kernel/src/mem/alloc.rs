//!
//! # Kernel Heap Allocator
//!
//! Fast page-backed heap allocator used as the kernel's global allocator.
//!

use core::{
    alloc::{GlobalAlloc, Layout},
    cmp,
    mem::size_of,
    ops::{Deref, DerefMut},
    ptr::{self, null_mut},
    sync::atomic::{AtomicU64, Ordering},
};

use log::info;

use crate::{
    arch,
    mem::{self, pages_for_len, phys, PhysAddr, VirtAddr, VmFlags, PAGE_SIZE},
    sys::{self, sync::Mutex},
};

/// Base virtual address for the kernel heap window.
const HEAP_BASE: u64 = 0xFFFF_A000_0000_0000;
/// Total virtual address space reserved for heap growth.
const HEAP_SIZE: u64 = 1 << 32;
/// Number of heap pages inside the reserved window.
const HEAP_PAGES: usize = (HEAP_SIZE / PAGE_SIZE) as usize;
/// Bit width needed to encode a heap page index.
const HEAP_PAGE_INDEX_BITS: u32 = 20;
/// Mask used to decode a heap page index from a retired-page entry.
const HEAP_PAGE_INDEX_MASK: u64 = (1u64 << HEAP_PAGE_INDEX_BITS) - 1;
/// Sentinel value representing an invalid page index.
const NONE_PAGE: u32 = u32::MAX;
/// Slab size classes used for hot-path allocations.
const SIZE_CLASSES: [usize; 8] = [8, 16, 32, 64, 128, 256, 512, 1024];
/// Maximum CPUs tracked by heap TLB shootdown state.
const MAX_TLB_CPUS: usize = 256;
/// Maximum number of unmapped heap pages quarantined awaiting remote TLB flush.
const TLB_RETIRE_CAPACITY: usize = 1 << 16;
/// Maximum retired pages reclaimed in one heap-lock critical section.
const RETIRED_RECLAIM_BATCH: u64 = 256;

/// Marker stored inside freed slab objects.
#[repr(C)]
struct FreeNode {
    next: *mut FreeNode,
}

/// Per-slab header placed at the front of each slab page.
#[repr(C, align(64))]
struct SlabHeader {
    next: u32,
    prev: u32,
    class: u16,
    capacity: u16,
    free_count: u16,
    _reserved: u16,
    free_head: *mut FreeNode,
}

/// Heap page ownership state.
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum PageKind {
    Free = 0,
    Busy = 1,
    Slab = 2,
    LargeHead = 3,
    LargeTail = 4,
    Retired = 5,
}

/// Side metadata for one heap page.
#[derive(Copy, Clone)]
struct HeapPageMeta {
    kind: PageKind,
    class: u8,
    _reserved: u16,
    aux: u32,
}

impl HeapPageMeta {
    const FREE: Self = Self {
        kind: PageKind::Free,
        class: 0,
        _reserved: 0,
        aux: 0,
    };
}

/// Global allocator state guarded by the kernel heap mutex.
struct HeapState {
    initialized: bool,
    search_hint: usize,
    partial: [u32; SIZE_CLASSES.len()],
    empty_cache: [u32; SIZE_CLASSES.len()],
    pages: [HeapPageMeta; HEAP_PAGES],
}

struct TlbShootdownState {
    reserved_seq: AtomicU64,
    published_seq: AtomicU64,
    reclaimed_seq: AtomicU64,
    peak_pending: AtomicU64,
    entries: [AtomicU64; TLB_RETIRE_CAPACITY],
    seen: [AtomicU64; MAX_TLB_CPUS],
}

impl HeapState {
    const fn new() -> Self {
        Self {
            initialized: false,
            search_hint: 0,
            partial: [NONE_PAGE; SIZE_CLASSES.len()],
            empty_cache: [NONE_PAGE; SIZE_CLASSES.len()],
            pages: [HeapPageMeta::FREE; HEAP_PAGES],
        }
    }

    fn init(&mut self) {
        if self.initialized {
            return;
        }

        self.initialized = true;
    }

    fn alloc(&mut self, layout: Layout) -> *mut u8 {
        self.collect_retired();

        if !self.initialized {
            return null_mut();
        }

        if layout.size() == 0 {
            return layout.align() as *mut u8;
        }

        if let Some(class) = class_index(layout) {
            return self.alloc_slab(class).unwrap_or(null_mut());
        }

        self.alloc_large(layout).unwrap_or(null_mut())
    }

    fn dealloc(&mut self, ptr: *mut u8, layout: Layout) -> Option<u64> {
        self.collect_retired();

        if layout.size() == 0 {
            return None;
        }

        let page_idx = ptr_page_index(ptr).expect("mem/alloc: pointer outside heap window");
        let meta = self.pages[page_idx];

        match meta.kind {
            PageKind::Slab => self.dealloc_slab(page_idx, ptr),
            PageKind::LargeHead => self.dealloc_large(page_idx),
            PageKind::LargeTail => panic!("mem/alloc: dealloc on non-head large allocation"),
            _ => panic!("mem/alloc: invalid heap page state during free"),
        }
    }

    fn usable_size(&self, ptr: *mut u8) -> Option<usize> {
        let page_idx = ptr_page_index(ptr)?;
        let meta = self.pages[page_idx];

        match meta.kind {
            PageKind::Slab => Some(SIZE_CLASSES[meta.class as usize]),
            PageKind::LargeHead => Some(meta.aux as usize * PAGE_SIZE as usize),
            _ => None,
        }
    }

    fn alloc_slab(&mut self, class: usize) -> Option<*mut u8> {
        let mut page_idx = self.partial[class];
        if page_idx == NONE_PAGE {
            page_idx = self.grow_slab(class)?;
        }

        let slab = unsafe { slab_header(page_idx as usize) };
        let node = slab.free_head;
        if node.is_null() {
            panic!("mem/alloc: slab partial list contained empty slab");
        }

        let was_empty = slab.free_count == slab.capacity;
        slab.free_head = unsafe { (*node).next };
        slab.free_count -= 1;

        if was_empty && self.empty_cache[class] == page_idx {
            self.empty_cache[class] = NONE_PAGE;
        }

        if slab.free_count == 0 {
            self.remove_partial(class, page_idx);
        }

        Some(node.cast::<u8>())
    }

    fn dealloc_slab(&mut self, page_idx: usize, ptr: *mut u8) -> Option<u64> {
        let class = self.pages[page_idx].class as usize;
        let slab = unsafe { slab_header(page_idx) };
        let was_full = slab.free_count == 0;
        let node = ptr.cast::<FreeNode>();

        unsafe {
            (*node).next = slab.free_head;
        }
        slab.free_head = node;
        slab.free_count += 1;

        if was_full {
            self.insert_partial(class, page_idx as u32);
        }

        if slab.free_count != slab.capacity {
            return None;
        }

        let cache = self.empty_cache[class];
        if cache == NONE_PAGE || cache == page_idx as u32 {
            self.empty_cache[class] = page_idx as u32;
            return None;
        }

        self.remove_partial(class, page_idx as u32);
        self.release_page(page_idx)
    }

    fn grow_slab(&mut self, class: usize) -> Option<u32> {
        let page_idx = self.reserve_run(1, 1)?;
        let page_va = heap_page_virt(page_idx);
        let root = arch::paging::active_root();
        let phys_page = phys::alloc_zeroed_page()?;

        let mapped =
            unsafe { arch::paging::map_page(root, page_va, phys_page.paddr(), heap_flags()) };
        if mapped.is_err() {
            unsafe { phys::free_page(phys_page) };
            self.clear_run(page_idx, 1);
            return None;
        }

        unsafe { init_slab_page(page_idx, class) };
        self.pages[page_idx] = HeapPageMeta {
            kind: PageKind::Slab,
            class: class as u8,
            _reserved: 0,
            aux: 0,
        };
        self.insert_partial(class, page_idx as u32);

        Some(page_idx as u32)
    }

    fn alloc_large(&mut self, layout: Layout) -> Option<*mut u8> {
        let pages = pages_for_len(layout.size() as u64) as usize;
        let align_pages = cmp::max(1, pages_for_len(layout.align() as u64) as usize);
        let start = self.reserve_run(pages, align_pages)?;
        let root = arch::paging::active_root();
        let mut mapped = 0usize;

        while mapped < pages {
            let phys_page = match phys::alloc_zeroed_page() {
                Some(page) => page,
                None => {
                    self.rollback_large(root, start, mapped);
                    self.clear_run(start, pages);
                    return None;
                }
            };

            let virt = heap_page_virt(start + mapped);
            let res =
                unsafe { arch::paging::map_page(root, virt, phys_page.paddr(), heap_flags()) };
            if res.is_err() {
                unsafe { phys::free_page(phys_page) };
                self.rollback_large(root, start, mapped);
                self.clear_run(start, pages);
                return None;
            }

            mapped += 1;
        }

        self.pages[start] = HeapPageMeta {
            kind: PageKind::LargeHead,
            class: 0,
            _reserved: 0,
            aux: pages as u32,
        };

        for idx in (start + 1)..(start + pages) {
            self.pages[idx] = HeapPageMeta {
                kind: PageKind::LargeTail,
                class: 0,
                _reserved: 0,
                aux: (start + pages - idx) as u32,
            };
        }

        Some(heap_page_virt(start).as_mut_ptr())
    }

    fn dealloc_large(&mut self, start: usize) -> Option<u64> {
        let pages = self.pages[start].aux as usize;
        let root = arch::paging::active_root();
        let mut last_retired = None;

        for idx in start..(start + pages) {
            let page = self.take_mapped_page(root, idx, "large");
            last_retired = self.retire_page(idx, page, start + pages - idx);
        }

        self.search_hint = self.search_hint.min(start);
        last_retired
    }

    fn reserve_run(&mut self, pages: usize, align_pages: usize) -> Option<usize> {
        if pages == 0 || pages > HEAP_PAGES {
            return None;
        }

        let start = self.find_run(self.search_hint, pages, align_pages)?;
        for idx in start..(start + pages) {
            self.pages[idx] = HeapPageMeta {
                kind: PageKind::Busy,
                class: 0,
                _reserved: 0,
                aux: (start + pages - idx) as u32,
            };
        }

        self.search_hint = start + pages;
        Some(start)
    }

    fn clear_run(&mut self, start: usize, pages: usize) {
        for idx in start..(start + pages) {
            self.pages[idx] = HeapPageMeta::FREE;
        }

        self.search_hint = self.search_hint.min(start);
    }

    fn find_run(&self, hint: usize, pages: usize, align_pages: usize) -> Option<usize> {
        if let Some(run) = self.find_run_range(hint, HEAP_PAGES, pages, align_pages) {
            return Some(run);
        }

        self.find_run_range(0, hint, pages, align_pages)
    }

    fn find_run_range(
        &self,
        start: usize,
        end: usize,
        pages: usize,
        align_pages: usize,
    ) -> Option<usize> {
        let mut idx = align_up_pages(start, align_pages);
        while idx + pages <= end {
            let mut ok = true;

            for probe in idx..(idx + pages) {
                let meta = self.pages[probe];
                if meta.kind != PageKind::Free {
                    idx = align_up_pages(probe + occupied_span(meta), align_pages);
                    ok = false;
                    break;
                }
            }

            if ok {
                return Some(idx);
            }
        }

        None
    }

    fn insert_partial(&mut self, class: usize, page_idx: u32) {
        let slab = unsafe { slab_header(page_idx as usize) };
        slab.prev = NONE_PAGE;
        slab.next = self.partial[class];

        if slab.next != NONE_PAGE {
            unsafe { slab_header(slab.next as usize) }.prev = page_idx;
        }

        self.partial[class] = page_idx;
    }

    fn remove_partial(&mut self, class: usize, page_idx: u32) {
        let slab = unsafe { slab_header(page_idx as usize) };
        let next = slab.next;
        let prev = slab.prev;

        if prev != NONE_PAGE {
            unsafe { slab_header(prev as usize) }.next = next;
        } else {
            self.partial[class] = next;
        }

        if next != NONE_PAGE {
            unsafe { slab_header(next as usize) }.prev = prev;
        }

        slab.next = NONE_PAGE;
        slab.prev = NONE_PAGE;
    }

    fn release_page(&mut self, page_idx: usize) -> Option<u64> {
        let root = arch::paging::active_root();
        let page = self.take_mapped_page(root, page_idx, "slab");
        let seq = self.retire_page(page_idx, page, 1);
        self.search_hint = self.search_hint.min(page_idx);
        seq
    }

    fn rollback_large(&mut self, root: PhysAddr, start: usize, mapped: usize) {
        for idx in start..(start + mapped) {
            let page = self.take_mapped_page(root, idx, "rollback");
            unsafe { phys::free_page(page) };
        }
    }

    fn take_mapped_page(
        &self,
        root: PhysAddr,
        page_idx: usize,
        context: &str,
    ) -> &'static phys::Page {
        let virt = heap_page_virt(page_idx);
        let phys = unsafe { arch::paging::unmap_page(root, virt) }
            .unwrap_or_else(|_| panic!("mem/alloc: failed to unmap {context} heap page"))
            .unwrap_or_else(|| panic!("mem/alloc: missing {context} heap mapping"));
        phys::phys_to_page(phys)
            .unwrap_or_else(|| panic!("mem/alloc: {context} heap page missing PFN metadata"))
    }

    fn retire_page(
        &mut self,
        page_idx: usize,
        page: &'static phys::Page,
        span: usize,
    ) -> Option<u64> {
        if sys::smp::online_cpus() <= 1 {
            unsafe { phys::free_page(page) };
            self.pages[page_idx] = HeapPageMeta::FREE;
            return None;
        }

        let seq = queue_retired_page(page_idx, page.paddr());
        self.pages[page_idx] = HeapPageMeta {
            kind: PageKind::Retired,
            class: 0,
            _reserved: 0,
            aux: span as u32,
        };
        Some(seq)
    }

    fn collect_retired(&mut self) {
        let shootdown = &HEAP_TLB_SHOOTDOWN;
        let published = shootdown.published_seq.load(Ordering::Acquire);
        let mut reclaimed = shootdown.reclaimed_seq.load(Ordering::Relaxed);
        if reclaimed >= published {
            return;
        }

        let mut safe_seq = shootdown.seen[0].load(Ordering::Acquire).min(published);
        if sys::smp::online_cpus() > 1 {
            let cpu_count = tracked_cpu_count();
            safe_seq = published;

            for cpu_id in 0..cpu_count {
                if sys::smp::is_online(cpu_id) {
                    safe_seq = safe_seq.min(shootdown.seen[cpu_id].load(Ordering::Acquire));
                }
            }
        }

        let reclaim_target = safe_seq.min(reclaimed.saturating_add(RETIRED_RECLAIM_BATCH));
        while reclaimed < reclaim_target {
            reclaimed += 1;
            self.reclaim_retired_page(load_retired_entry(reclaimed));
        }

        shootdown.reclaimed_seq.store(reclaimed, Ordering::Release);
    }

    fn reclaim_retired_page(&mut self, entry: u64) {
        let (page_idx, paddr) = decode_retired_entry(entry);
        assert_eq!(
            self.pages[page_idx].kind,
            PageKind::Retired,
            "mem/alloc: reclaim encountered non-retired page {}",
            page_idx
        );

        let page = phys::phys_to_page(paddr)
            .unwrap_or_else(|| panic!("mem/alloc: retired heap page missing PFN metadata"));
        unsafe { phys::free_page(page) };
        self.pages[page_idx] = HeapPageMeta::FREE;
        self.search_hint = self.search_hint.min(page_idx);
    }
}

/// Global allocator facade wired into Rust's allocation hooks.
///
/// This type is installed as the kernel's single heap allocator through
/// [`GLOBAL_ALLOCATOR`], which delegates all operations to the page-backed
/// heap state guarded by [`HEAP`].
pub struct KernelAllocator;

static HEAP: Mutex<HeapState> = Mutex::new(HeapState::new());
static HEAP_TLB_SHOOTDOWN: TlbShootdownState = TlbShootdownState::new();

/// Snapshot of kernel heap usage.
#[derive(Copy, Clone, Debug, Default)]
pub struct HeapStats {
    /// Whether the heap allocator has been initialized.
    pub initialized: bool,
    /// Total pages reserved for the heap window.
    pub total_pages: usize,
    /// Pages currently free inside the heap window.
    pub free_pages: usize,
    /// Temporarily reserved heap pages not yet classified as slab/large.
    pub reserved_pages: usize,
    /// Heap pages retired pending TLB visibility on all CPUs.
    pub retired_pages: usize,
    /// Heap pages currently backing slab allocators.
    pub slab_pages: usize,
    /// Heap pages currently backing large allocations.
    pub large_pages: usize,
    /// Bytes currently occupied by live slab allocations.
    pub slab_used_bytes: usize,
    /// Free bytes sitting inside active slab pages.
    pub slab_free_bytes: usize,
    /// Bytes backing live large allocations.
    pub large_bytes: usize,
}

struct HeapGuard<'a> {
    guard: crate::sys::sync::MutexGuard<'a, HeapState>,
}

#[global_allocator]
static GLOBAL_ALLOCATOR: KernelAllocator = KernelAllocator;

unsafe impl GlobalAlloc for KernelAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        heap_lock().alloc(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let retire_seq = {
            let mut heap = heap_lock();
            heap.dealloc(ptr, layout)
        };
        if let Some(seq) = retire_seq {
            publish_retired_page(seq);
        }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = heap_lock().alloc(layout);
        if !ptr.is_null() && layout.size() != 0 {
            ptr::write_bytes(ptr, 0, layout.size());
        }
        ptr
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if layout.size() == 0 {
            let new_layout = Layout::from_size_align_unchecked(new_size, layout.align());
            return self.alloc(new_layout);
        }

        if new_size == 0 {
            self.dealloc(ptr, layout);
            return layout.align() as *mut u8;
        }

        let current = {
            let heap = heap_lock();
            heap.usable_size(ptr)
        };

        if let Some(current) = current {
            if new_size <= current {
                return ptr;
            }
        }

        let new_layout = Layout::from_size_align_unchecked(new_size, layout.align());
        let new_ptr = self.alloc(new_layout);
        if new_ptr.is_null() {
            return null_mut();
        }

        ptr::copy_nonoverlapping(ptr, new_ptr, cmp::min(layout.size(), new_size));
        self.dealloc(ptr, layout);
        new_ptr
    }
}

/// Initializes the global heap allocator.
pub fn init() {
    let mut heap = heap_lock();
    heap.init();
    if let Some(cpu) = arch::thiscpu_opt() {
        register_tlb_cpu(cpu.id);
    }

    info!(
        "mem/alloc: heap window active: virt=[0x{:x}-0x{:x}] size={} MiB",
        HEAP_BASE,
        HEAP_BASE + HEAP_SIZE,
        HEAP_SIZE / (1024 * 1024),
    );
}

/// Returns a point-in-time snapshot of heap allocator usage.
pub fn stats() -> HeapStats {
    let mut heap = heap_lock();
    heap.collect_retired();
    heap.stats()
}

/// Handles kernel allocation failures.
#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    panic!(
        "mem/alloc: allocation failure (size={} align={})",
        layout.size(),
        layout.align()
    );
}

#[inline(always)]
fn class_index(layout: Layout) -> Option<usize> {
    let need = cmp::max(
        layout.size(),
        cmp::max(layout.align(), size_of::<FreeNode>()),
    );

    SIZE_CLASSES.iter().position(|&size| size >= need)
}

#[inline(always)]
fn heap_flags() -> VmFlags {
    VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL
}

#[inline(always)]
fn align_up_pages(value: usize, align: usize) -> usize {
    debug_assert!(align.is_power_of_two());
    (value + (align - 1)) & !(align - 1)
}

#[inline(always)]
fn occupied_span(meta: HeapPageMeta) -> usize {
    match meta.kind {
        PageKind::Free => 1,
        PageKind::Busy | PageKind::LargeTail | PageKind::Retired => {
            cmp::max(meta.aux as usize, 1)
        }
        PageKind::Slab => 1,
        PageKind::LargeHead => cmp::max(meta.aux as usize, 1),
    }
}

#[inline(always)]
fn heap_page_virt(page_idx: usize) -> VirtAddr {
    VirtAddr::new(HEAP_BASE + page_idx as u64 * PAGE_SIZE)
}

#[inline(always)]
fn ptr_page_index(ptr: *mut u8) -> Option<usize> {
    let raw = ptr as usize as u64;
    if raw < HEAP_BASE || raw >= HEAP_BASE + HEAP_SIZE {
        return None;
    }

    Some(((raw - HEAP_BASE) / PAGE_SIZE) as usize)
}

unsafe fn slab_header(page_idx: usize) -> &'static mut SlabHeader {
    &mut *heap_page_virt(page_idx).as_mut_ptr::<SlabHeader>()
}

/// Flushes allocator-requested heap TLB invalidations on the current CPU.
pub(crate) fn flush_remote_tlb_shootdown() {
    let Some(cpu) = arch::thiscpu_opt() else {
        return;
    };
    let cpu_id = cpu.id;
    if cpu_id >= MAX_TLB_CPUS {
        return;
    }

    let shootdown = &HEAP_TLB_SHOOTDOWN;
    let published = shootdown.published_seq.load(Ordering::Acquire);
    let seen = shootdown.seen[cpu_id].load(Ordering::Acquire);
    if published <= seen {
        return;
    }

    for seq in (seen + 1)..=published {
        let (page_idx, _) = decode_retired_entry(load_retired_entry(seq));
        arch::paging::flush_page(heap_page_virt(page_idx));
    }

    shootdown.seen[cpu_id].store(published, Ordering::Release);
}

/// Marks `cpu_id` online for allocator shootdown tracking.
pub fn register_tlb_cpu(cpu_id: usize) {
    if cpu_id >= MAX_TLB_CPUS {
        panic!(
            "mem/alloc: cpu {} exceeds heap TLB tracking capacity {}",
            cpu_id, MAX_TLB_CPUS
        );
    }

    let published = HEAP_TLB_SHOOTDOWN.published_seq.load(Ordering::Acquire);
    HEAP_TLB_SHOOTDOWN.seen[cpu_id].store(published, Ordering::Release);
}

unsafe fn init_slab_page(page_idx: usize, class: usize) {
    let base = heap_page_virt(page_idx).as_mut_ptr::<u8>();
    let slot_size = SIZE_CLASSES[class];
    let slots_offset = mem::align_up(size_of::<SlabHeader>() as u64, slot_size as u64) as usize;
    let capacity = ((PAGE_SIZE as usize) - slots_offset) / slot_size;
    assert!(
        capacity != 0,
        "mem/alloc: invalid slab capacity for class {class}"
    );

    ptr::write(
        base.cast::<SlabHeader>(),
        SlabHeader {
            next: NONE_PAGE,
            prev: NONE_PAGE,
            class: class as u16,
            capacity: capacity as u16,
            free_count: capacity as u16,
            _reserved: 0,
            free_head: null_mut(),
        },
    );

    let slab = &mut *base.cast::<SlabHeader>();
    let mut free_head = null_mut();

    for slot in (0..capacity).rev() {
        let node = base.add(slots_offset + slot * slot_size).cast::<FreeNode>();
        ptr::write(node, FreeNode { next: free_head });
        free_head = node;
    }

    slab.free_head = free_head;
}

impl HeapState {
    fn stats(&self) -> HeapStats {
        let mut stats = HeapStats {
            initialized: self.initialized,
            total_pages: HEAP_PAGES,
            ..HeapStats::default()
        };

        for (page_idx, meta) in self.pages.iter().copied().enumerate() {
            match meta.kind {
                PageKind::Free => stats.free_pages += 1,
                PageKind::Busy => stats.reserved_pages += 1,
                PageKind::Retired => stats.retired_pages += 1,
                PageKind::Slab => {
                    stats.slab_pages += 1;

                    let slab = unsafe { slab_header(page_idx) };
                    let slot_size = SIZE_CLASSES[meta.class as usize];
                    let free_slots = slab.free_count as usize;
                    let used_slots = slab.capacity as usize - free_slots;
                    stats.slab_free_bytes += free_slots * slot_size;
                    stats.slab_used_bytes += used_slots * slot_size;
                }
                PageKind::LargeHead => {
                    let pages = meta.aux as usize;
                    stats.large_pages += pages;
                    stats.large_bytes += pages * PAGE_SIZE as usize;
                }
                PageKind::LargeTail => {}
            }
        }
        stats
    }
}

impl TlbShootdownState {
    const fn new() -> Self {
        Self {
            reserved_seq: AtomicU64::new(0),
            published_seq: AtomicU64::new(0),
            reclaimed_seq: AtomicU64::new(0),
            peak_pending: AtomicU64::new(0),
            entries: [const { AtomicU64::new(0) }; TLB_RETIRE_CAPACITY],
            seen: [const { AtomicU64::new(0) }; MAX_TLB_CPUS],
        }
    }
}

fn tracked_cpu_count() -> usize {
    let cpu_count = sys::smp::cpu_count();
    assert!(
        cpu_count <= MAX_TLB_CPUS,
        "mem/alloc: cpu count {} exceeds heap TLB tracking capacity {}",
        cpu_count,
        MAX_TLB_CPUS
    );
    cpu_count
}

fn tracked_cpu_id() -> usize {
    let cpu_id = arch::thiscpu().id;
    assert!(
        cpu_id < MAX_TLB_CPUS,
        "mem/alloc: cpu {} exceeds heap TLB tracking capacity {}",
        cpu_id,
        MAX_TLB_CPUS
    );
    cpu_id
}

fn queue_retired_page(page_idx: usize, paddr: PhysAddr) -> u64 {
    let shootdown = &HEAP_TLB_SHOOTDOWN;
    let this_cpu = tracked_cpu_id();
    let reclaimed = shootdown.reclaimed_seq.load(Ordering::Acquire);
    let reserved = shootdown.reserved_seq.load(Ordering::Relaxed);
    let pending = reserved.saturating_sub(reclaimed).saturating_add(1);

    if reserved.saturating_sub(reclaimed) >= TLB_RETIRE_CAPACITY as u64 {
        panic!("mem/alloc: heap TLB retire queue exhausted");
    }

    let seq = shootdown.reserved_seq.fetch_add(1, Ordering::AcqRel).wrapping_add(1);
    if seq == 0 {
        panic!("mem/alloc: heap TLB retire sequence wrapped");
    }

    shootdown.entries[retire_slot(seq)]
        .store(encode_retired_entry(page_idx, paddr), Ordering::Relaxed);
    shootdown.seen[this_cpu].store(seq, Ordering::Release);
    update_peak_pending(&shootdown.peak_pending, pending);
    seq
}

fn publish_retired_page(seq: u64) {
    let shootdown = &HEAP_TLB_SHOOTDOWN;
    shootdown.published_seq.store(seq, Ordering::Release);
    let _ = sys::smp::send_ipi(flush_remote_tlb_shootdown, sys::smp::IpiTarget::All);
}

#[inline(always)]
fn retire_slot(seq: u64) -> usize {
    (seq as usize - 1) % TLB_RETIRE_CAPACITY
}

#[inline(always)]
fn load_retired_entry(seq: u64) -> u64 {
    HEAP_TLB_SHOOTDOWN.entries[retire_slot(seq)].load(Ordering::Relaxed)
}

fn update_peak_pending(peak: &AtomicU64, current: u64) {
    let mut observed = peak.load(Ordering::Relaxed);
    while current > observed {
        match peak.compare_exchange_weak(observed, current, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => return,
            Err(next) => observed = next,
        }
    }
}

#[inline]
fn heap_lock() -> HeapGuard<'static> {
    HeapGuard::new(HEAP.lock())
}

impl<'a> HeapGuard<'a> {
    #[inline]
    fn new(guard: crate::sys::sync::MutexGuard<'a, HeapState>) -> Self {
        Self { guard }
    }
}

impl Deref for HeapGuard<'_> {
    type Target = HeapState;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

impl DerefMut for HeapGuard<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.guard
    }
}

#[inline(always)]
fn encode_retired_entry(page_idx: usize, paddr: PhysAddr) -> u64 {
    assert!(
        page_idx < HEAP_PAGES,
        "mem/alloc: invalid heap page index {page_idx}"
    );
    let pfn = paddr.as_u64() / PAGE_SIZE;
    assert!(
        pfn < (1u64 << (64 - HEAP_PAGE_INDEX_BITS)),
        "mem/alloc: PFN 0x{:x} exceeds heap shootdown encoding",
        pfn
    );

    (pfn << HEAP_PAGE_INDEX_BITS) | page_idx as u64
}

#[inline(always)]
fn decode_retired_entry(entry: u64) -> (usize, PhysAddr) {
    let page_idx = (entry & HEAP_PAGE_INDEX_MASK) as usize;
    let pfn = entry >> HEAP_PAGE_INDEX_BITS;
    (page_idx, PhysAddr::new(pfn * PAGE_SIZE))
}
