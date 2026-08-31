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
};

use log::debug;

use crate::{
    arch,
    mem::{self, PAGE_SIZE, PhysAddr, VirtAddr, VmFlags, pages_for_len, phys, tlb},
    sys::{self, sync::Mutex},
};

/// Base virtual address for the kernel heap window.
pub(crate) const HEAP_BASE: u64 = 0xFFFF_A000_0000_0000;
/// Total virtual address space reserved for heap growth.
const HEAP_SIZE: u64 = 1 << 32;
/// Number of heap pages inside the reserved window.
const HEAP_PAGES: usize = (HEAP_SIZE / PAGE_SIZE) as usize;
/// Sentinel value representing an invalid page index.
const NONE_PAGE: u32 = u32::MAX;
/// Slab size classes used for hot-path allocations.
///
/// The largest class is half a page, so every request the slab layer accepts
/// leaves room for the slab header without wasting a whole page.
const SIZE_CLASSES: [usize; 9] = [8, 16, 32, 64, 128, 256, 512, 1024, 2048];
/// Maximum number of unmapped heap pages quarantined awaiting remote TLB flush.
const TLB_RETIRE_CAPACITY: usize = 1 << 12;
/// Maximum retired pages reclaimed in one heap-lock critical section.
const RETIRED_RECLAIM_BATCH: usize = 256;

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
    retired: RetireQueue,
}

/// One heap page awaiting the invalidation that makes its frame reusable.
#[derive(Copy, Clone)]
struct RetiredPage {
    page_idx: u32,
    paddr: u64,
    /// Shootdown sequence that must be applied on every CPU first.
    sequence: u64,
}

/// Bounded queue of heap pages whose frames cannot be reused yet.
///
/// Unmapping a heap page only invalidates the local translation, so a frame
/// stays quarantined until every CPU has acknowledged the invalidation. The
/// queue is drained opportunistically from allocation and deallocation, which
/// keeps those paths free of inter-processor round trips.
struct RetireQueue {
    entries: [RetiredPage; TLB_RETIRE_CAPACITY],
    head: usize,
    len: usize,
    peak: usize,
}

impl RetireQueue {
    const fn new() -> Self {
        Self {
            entries: [RetiredPage {
                page_idx: 0,
                paddr: 0,
                sequence: 0,
            }; TLB_RETIRE_CAPACITY],
            head: 0,
            len: 0,
            peak: 0,
        }
    }

    fn push(&mut self, entry: RetiredPage) -> bool {
        if self.len == TLB_RETIRE_CAPACITY {
            return false;
        }
        self.entries[(self.head + self.len) % TLB_RETIRE_CAPACITY] = entry;
        self.len += 1;
        self.peak = self.peak.max(self.len);
        true
    }

    fn peek(&self) -> Option<RetiredPage> {
        (self.len != 0).then(|| self.entries[self.head])
    }

    fn pop(&mut self) -> Option<RetiredPage> {
        let entry = self.peek()?;
        self.head = (self.head + 1) % TLB_RETIRE_CAPACITY;
        self.len -= 1;
        Some(entry)
    }
}

impl HeapState {
    const fn new() -> Self {
        Self {
            initialized: false,
            search_hint: 0,
            partial: [NONE_PAGE; SIZE_CLASSES.len()],
            empty_cache: [NONE_PAGE; SIZE_CLASSES.len()],
            pages: [HeapPageMeta::FREE; HEAP_PAGES],
            retired: RetireQueue::new(),
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

    fn dealloc(&mut self, ptr: *mut u8, layout: Layout) {
        self.collect_retired();

        if layout.size() == 0 {
            return;
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

        // SAFETY: partial lists only contain mapped slab pages and the heap
        // lock gives this operation unique access.
        let slab = unsafe { slab_header(page_idx as usize) };
        let node = slab.free_head;
        if node.is_null() {
            panic!("mem/alloc: slab partial list contained empty slab");
        }

        let was_empty = slab.free_count == slab.capacity;
        // SAFETY: `node` came from this slab's validated free list.
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

    fn dealloc_slab(&mut self, page_idx: usize, ptr: *mut u8) {
        let class = self.pages[page_idx].class as usize;
        // SAFETY: `page_idx` was derived from an allocation owned by this slab
        // and the heap lock gives unique access.
        let slab = unsafe { slab_header(page_idx) };
        let was_full = slab.free_count == 0;
        let node = ptr.cast::<FreeNode>();

        // SAFETY: `ptr` is a live slot in this slab and is being returned once.
        unsafe {
            (*node).next = slab.free_head;
        }
        slab.free_head = node;
        slab.free_count += 1;

        if was_full {
            self.insert_partial(class, page_idx as u32);
        }

        if slab.free_count != slab.capacity {
            return;
        }

        let cache = self.empty_cache[class];
        if cache == NONE_PAGE || cache == page_idx as u32 {
            self.empty_cache[class] = page_idx as u32;
            return;
        }

        self.remove_partial(class, page_idx as u32);
        self.release_page(page_idx);
    }

    fn grow_slab(&mut self, class: usize) -> Option<u32> {
        let page_idx = self.reserve_run(1, 1)?;
        let page_va = heap_page_virt(page_idx);
        let root = arch::paging::active_root();
        let Some(phys_page) = phys::alloc_zeroed_page(phys::PageUse::KernelHeap) else {
            // `reserve_run` changes side metadata before physical allocation.
            // Roll it back on pressure or this virtual page remains Busy
            // forever and slowly shrinks the heap window after each failure.
            self.clear_run(page_idx, 1);
            return None;
        };

        // SAFETY: the reserved heap virtual page is currently unmapped and the
        // new physical page is exclusively owned by the allocator.
        let mapped =
            unsafe { arch::paging::map_page(root, page_va, phys_page.paddr(), heap_flags()) };
        if mapped.is_err() {
            // SAFETY: mapping failed, so no alias to this page was published.
            unsafe { phys::free_page(phys_page) };
            self.clear_run(page_idx, 1);
            return None;
        }

        // SAFETY: the page was just mapped exclusively for this slab class.
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
            let phys_page = match phys::alloc_zeroed_page(phys::PageUse::KernelHeap) {
                Some(page) => page,
                None => {
                    self.rollback_large(root, start, mapped);
                    self.clear_run(start, pages);
                    return None;
                }
            };

            let virt = heap_page_virt(start + mapped);
            // SAFETY: this reserved virtual page is currently unmapped and the
            // physical page is exclusively owned by this allocation.
            let res =
                unsafe { arch::paging::map_page(root, virt, phys_page.paddr(), heap_flags()) };
            if res.is_err() {
                // SAFETY: mapping failed, so no alias to this page was exposed.
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

    fn dealloc_large(&mut self, start: usize) {
        let pages = self.pages[start].aux as usize;
        let root = arch::paging::active_root();

        for idx in start..(start + pages) {
            let page = self.take_mapped_page(root, idx, "large");
            self.retire_page(idx, page, start + pages - idx);
        }

        self.search_hint = self.search_hint.min(start);
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

            let mut probe = idx;
            while probe < idx + pages {
                let meta = self.pages[probe];
                if meta.kind != PageKind::Free {
                    idx = align_up_pages(probe + occupied_span(meta), align_pages);
                    ok = false;
                    break;
                }
                probe += 1;
            }

            if ok {
                return Some(idx);
            }
        }

        None
    }

    fn insert_partial(&mut self, class: usize, page_idx: u32) {
        // SAFETY: partial-list links are only manipulated under the heap lock.
        let slab = unsafe { slab_header(page_idx as usize) };
        slab.prev = NONE_PAGE;
        slab.next = self.partial[class];

        if slab.next != NONE_PAGE {
            // SAFETY: `slab.next` names another mapped slab in this list.
            unsafe { slab_header(slab.next as usize) }.prev = page_idx;
        }

        self.partial[class] = page_idx;
    }

    fn remove_partial(&mut self, class: usize, page_idx: u32) {
        // SAFETY: partial-list links are only manipulated under the heap lock.
        let slab = unsafe { slab_header(page_idx as usize) };
        let next = slab.next;
        let prev = slab.prev;

        if prev != NONE_PAGE {
            // SAFETY: `prev` names another mapped slab in this list.
            unsafe { slab_header(prev as usize) }.next = next;
        } else {
            self.partial[class] = next;
        }

        if next != NONE_PAGE {
            // SAFETY: `next` names another mapped slab in this list.
            unsafe { slab_header(next as usize) }.prev = prev;
        }

        slab.next = NONE_PAGE;
        slab.prev = NONE_PAGE;
    }

    fn release_page(&mut self, page_idx: usize) {
        let root = arch::paging::active_root();
        let page = self.take_mapped_page(root, page_idx, "slab");
        self.retire_page(page_idx, page, 1);
        self.search_hint = self.search_hint.min(page_idx);
    }

    fn rollback_large(&mut self, root: PhysAddr, start: usize, mapped: usize) {
        for idx in start..(start + mapped) {
            let page = self.take_mapped_page(root, idx, "rollback");
            // SAFETY: rollback removed the only heap mapping of this page.
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
        // SAFETY: the heap lock serializes page-table changes for this reserved
        // heap address, and the mapping is no longer accessible after removal.
        let phys = unsafe { arch::paging::unmap_page(root, virt) }
            .unwrap_or_else(|_| panic!("mem/alloc: failed to unmap {context} heap page"))
            .unwrap_or_else(|| panic!("mem/alloc: missing {context} heap mapping"));
        phys::phys_to_page(phys)
            .unwrap_or_else(|| panic!("mem/alloc: {context} heap page missing PFN metadata"))
    }

    /// Quarantines an unmapped heap page until its frame can be reused.
    ///
    /// Returns the virtual address whose invalidation must be published once
    /// the heap lock is released.
    fn retire_page(&mut self, page_idx: usize, page: &'static phys::Page, span: usize) {
        if sys::smp::online_cpus() <= 1 {
            // SAFETY: the page was unmapped above and no remote TLB can retain
            // an alias in the single-CPU case.
            unsafe { phys::free_page(page) };
            self.pages[page_idx] = HeapPageMeta::FREE;
            return;
        }

        self.pages[page_idx] = HeapPageMeta {
            kind: PageKind::Retired,
            class: 0,
            _reserved: 0,
            aux: span as u32,
        };

        let sequence = tlb::publish_async(heap_page_virt(page_idx));
        if self.enqueue_retired(page_idx, page.paddr(), sequence) {
            return;
        }

        // A saturated queue means some CPU is far behind. Forcing every CPU to
        // discard its translations makes the whole quarantine reclaimable at
        // once, which is preferable to failing a deallocation.
        let sequence = tlb::flush_everything();
        self.drain_retired();
        assert!(
            self.enqueue_retired(page_idx, page.paddr(), sequence),
            "mem/alloc: heap retire queue still full after a global flush"
        );
    }

    /// Records the sequence that makes a quarantined page reclaimable.
    fn enqueue_retired(&mut self, page_idx: usize, paddr: PhysAddr, sequence: u64) -> bool {
        self.retired.push(RetiredPage {
            page_idx: page_idx as u32,
            paddr: paddr.as_u64(),
            sequence,
        })
    }

    /// Releases quarantined pages whose invalidation every CPU has applied.
    fn collect_retired(&mut self) {
        if self.retired.len == 0 {
            return;
        }
        let applied = tlb::acknowledged_through();
        for _ in 0..RETIRED_RECLAIM_BATCH {
            let Some(entry) = self.retired.peek() else {
                break;
            };
            if entry.sequence > applied {
                break;
            }
            self.retired.pop();
            self.reclaim_retired_page(entry);
        }
    }

    /// Releases every quarantined page after a synchronous global flush.
    fn drain_retired(&mut self) {
        while let Some(entry) = self.retired.pop() {
            self.reclaim_retired_page(entry);
        }
    }

    fn reclaim_retired_page(&mut self, entry: RetiredPage) {
        let page_idx = entry.page_idx as usize;
        assert_eq!(
            self.pages[page_idx].kind,
            PageKind::Retired,
            "mem/alloc: reclaim encountered non-retired page {page_idx}"
        );

        let page = phys::phys_to_page(PhysAddr::new(entry.paddr))
            .unwrap_or_else(|| panic!("mem/alloc: retired heap page missing PFN metadata"));
        // SAFETY: every online CPU acknowledged the invalidation of this heap
        // address before the entry became reclaimable, so no stale alias can
        // reach the frame.
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

struct HeapGuard<'a> {
    guard: crate::sys::sync::MutexGuard<'a, HeapState>,
}

#[global_allocator]
static GLOBAL_ALLOCATOR: KernelAllocator = KernelAllocator;

// SAFETY: all allocator entry points serialize heap metadata through `HEAP`;
// returned regions are disjoint and page mappings remain live until dealloc.
unsafe impl GlobalAlloc for KernelAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        heap_lock().alloc(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        heap_lock().dealloc(ptr, layout);
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = heap_lock().alloc(layout);
        if !ptr.is_null() && layout.size() != 0 {
            // SAFETY: a successful allocation is writable for `layout.size()`.
            unsafe { ptr::write_bytes(ptr, 0, layout.size()) };
        }
        ptr
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if layout.size() == 0 {
            // SAFETY: `layout.align()` is a valid allocation alignment.
            let new_layout = unsafe { Layout::from_size_align_unchecked(new_size, layout.align()) };
            // SAFETY: delegated to the `GlobalAlloc` contract of `realloc`.
            return unsafe { self.alloc(new_layout) };
        }

        if new_size == 0 {
            // SAFETY: delegated to the `GlobalAlloc` contract of `realloc`.
            unsafe { self.dealloc(ptr, layout) };
            return layout.align() as *mut u8;
        }

        let current = {
            let heap = heap_lock();
            heap.usable_size(ptr)
        };

        if let Some(current) = current
            && new_size <= current
        {
            return ptr;
        }

        // SAFETY: `layout.align()` is a valid allocation alignment.
        let new_layout = unsafe { Layout::from_size_align_unchecked(new_size, layout.align()) };
        // SAFETY: delegated to the `GlobalAlloc` contract of `realloc`.
        let new_ptr = unsafe { self.alloc(new_layout) };
        if new_ptr.is_null() {
            return null_mut();
        }

        // SAFETY: both allocations are live, disjoint, and valid for the
        // copied minimum size.
        unsafe {
            ptr::copy_nonoverlapping(ptr, new_ptr, cmp::min(layout.size(), new_size));
            self.dealloc(ptr, layout);
        }
        new_ptr
    }
}

/// Initializes the global heap allocator.
pub fn init() {
    let mut heap = heap_lock();
    heap.init();

    debug!(
        "heap window [0x{:x}-0x{:x}], {} MiB",
        HEAP_BASE,
        HEAP_BASE + HEAP_SIZE,
        HEAP_SIZE / (1024 * 1024),
    );
}

/// Handles kernel allocation failures.
#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    panic!(
        "kernel heap exhausted: cannot allocate {} bytes aligned to {}",
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
        PageKind::Busy | PageKind::LargeTail | PageKind::Retired => cmp::max(meta.aux as usize, 1),
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
    if !(HEAP_BASE..HEAP_BASE + HEAP_SIZE).contains(&raw) {
        return None;
    }

    Some(((raw - HEAP_BASE) / PAGE_SIZE) as usize)
}

unsafe fn slab_header(page_idx: usize) -> &'static mut SlabHeader {
    // SAFETY: callers only request mapped slab pages while holding the heap
    // lock, which guarantees unique access to the header.
    unsafe { &mut *heap_page_virt(page_idx).as_mut_ptr::<SlabHeader>() }
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

    // SAFETY: the caller reserved and mapped this page exclusively for the
    // selected slab class; all computed slots remain within the page.
    unsafe {
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
