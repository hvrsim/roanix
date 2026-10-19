//!
//! # Kernel Heap Allocator
//!
//! Fast page-backed heap allocator used as the kernel's global allocator.
//!

use core::{
    alloc::{GlobalAlloc, Layout},
    cell::UnsafeCell,
    cmp,
    mem::size_of,
    ptr::{self, null_mut},
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};

use log::debug;

use crate::{
    arch,
    mem::{self, PAGE_SIZE, PhysAddr, VirtAddr, VmFlags, pages_for_len, phys, tlb},
    sys::{self, smp::IrqSpinLock, sync::Mutex},
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
/// Bounded retention per CPU and size class. Cached slots remain allocated
/// in their slabs, so freeing a different slot cannot unmap a cached pointer.
const MAGAZINE_CAPACITY: usize = 8;
const SLAB_RESERVE: usize = 16;

struct Magazine {
    slots: [*mut u8; MAGAZINE_CAPACITY],
    len: usize,
}

impl Magazine {
    const fn new() -> Self {
        Self {
            slots: [null_mut(); MAGAZINE_CAPACITY],
            len: 0,
        }
    }

    fn pop(&mut self) -> Option<*mut u8> {
        if self.len == 0 {
            return None;
        }
        self.len -= 1;
        Some(core::mem::replace(&mut self.slots[self.len], null_mut()))
    }

    fn push(&mut self, pointer: *mut u8) {
        self.slots[self.len] = pointer;
        self.len += 1;
    }
}

#[repr(align(64))]
struct HeapCache {
    classes: [Magazine; SIZE_CLASSES.len()],
}

// SAFETY: cache slots exclusively own allocated objects. Their pointers move
// only under the cache lock and are never dereferenced while cached.
unsafe impl Send for HeapCache {}

static HEAP_CACHES: [IrqSpinLock<HeapCache>; tlb::MAX_TLB_CPUS] = [const {
    IrqSpinLock::new(HeapCache {
        classes: [const { Magazine::new() }; SIZE_CLASSES.len()],
    })
}; tlb::MAX_TLB_CPUS];

fn local_cache() -> Option<&'static IrqSpinLock<HeapCache>> {
    if sys::smp::online_cpus() <= 1 {
        return None;
    }
    HEAP_CACHES.get(arch::thiscpu_opt()?.id)
}

fn allocate(layout: Layout) -> *mut u8 {
    if !READY.load(Ordering::Acquire) {
        return null_mut();
    }
    if layout.size() == 0 {
        return layout.align() as *mut u8;
    }
    if let Some(class) = class_index(layout)
        && let Some(cache) = local_cache()
        && let Some(pointer) = cache.lock().classes[class].pop()
    {
        return pointer;
    }
    let pointer = allocate_uncached(layout);
    if pointer.is_null() {
        drain_caches();
        return allocate_uncached(layout);
    }
    pointer
}

fn allocate_uncached(layout: Layout) -> *mut u8 {
    if let Some(class) = class_index(layout) {
        let owner = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
        POOLS[owner]
            .lock()
            .alloc_slab(class, owner)
            .unwrap_or(null_mut())
    } else {
        alloc_large(layout).unwrap_or(null_mut())
    }
}

fn alloc_large(layout: Layout) -> Option<*mut u8> {
    let pages = pages_for_len(layout.size() as u64) as usize;
    let align_pages = cmp::max(1, pages_for_len(layout.align() as u64) as usize);
    let start = {
        let mut arena = arena_lock();
        arena.collect_retired();
        arena.reserve_run(pages, align_pages)?
    };
    let root = heap_root();
    let mut mapped = 0usize;

    while mapped < pages {
        let phys_page = match phys::alloc_zeroed_page(phys::PageUse::KernelHeap) {
            Some(page) => page,
            None => {
                rollback_large(root, start, mapped);
                arena_lock().clear_run(start, pages);
                return None;
            }
        };

        let virt = heap_page_virt(start + mapped);
        // SAFETY: this reserved virtual page is currently unmapped and the
        // physical page is exclusively owned by this allocation.
        let res = unsafe { arch::paging::map_page(root, virt, phys_page.paddr(), heap_flags()) };
        if res.is_err() {
            // SAFETY: mapping failed, so no alias to this page was exposed.
            unsafe { phys::free_page(phys_page) };
            rollback_large(root, start, mapped);
            arena_lock().clear_run(start, pages);
            return None;
        }

        mapped += 1;
    }

    let mut arena = arena_lock();
    arena.pages[start] = HeapPageMeta {
        kind: PageKind::LargeHead,
        aux: pages as u32,
    };

    for idx in (start + 1)..(start + pages) {
        arena.pages[idx] = HeapPageMeta {
            kind: PageKind::LargeTail,
            aux: (start + pages - idx) as u32,
        };
    }

    Some(heap_page_virt(start).as_mut_ptr())
}

fn rollback_large(root: PhysAddr, start: usize, mapped: usize) {
    for idx in start..(start + mapped) {
        let page = take_mapped_page(root, idx, "rollback");
        // SAFETY: rollback removed the only heap mapping of this page.
        unsafe { phys::free_page(page) };
    }
}

fn take_mapped_page(root: PhysAddr, page_idx: usize, context: &str) -> &'static phys::Page {
    let virt = heap_page_virt(page_idx);
    // SAFETY: the caller owns this reserved heap address, and the mapping is no longer accessible after removal.
    let phys = unsafe { arch::paging::unmap_page(root, virt) }
        .unwrap_or_else(|_| panic!("mem/alloc: failed to unmap {context} heap page"))
        .unwrap_or_else(|| panic!("mem/alloc: missing {context} heap mapping"));
    phys::phys_to_page(phys)
        .unwrap_or_else(|| panic!("mem/alloc: {context} heap page missing PFN metadata"))
}

fn deallocate_uncached(pointer: *mut u8, layout: Layout) {
    if layout.size() == 0 {
        return;
    }
    if let Some(class) = class_index(layout) {
        let index = ptr_page_index(pointer).expect("mem/alloc: pointer outside heap window");
        // SAFETY: a live slot keeps its slab mapped. The owner is immutable;
        // only the disjoint slab state changes while the slot remains live.
        let owner = unsafe { (*heap_page_virt(index).as_mut_ptr::<SlabHeader>()).owner };
        POOLS[owner].lock().dealloc_slab(index, pointer, class);
    } else {
        arena_lock().dealloc(pointer, layout);
    }
}

#[cold]
fn drain_caches() {
    super::pmap::drain_pools();
    super::object::drain_pool();
    super::anon::drain_pool();
    super::map::drain_pool();
    super::metadata::drain();
    // Cache retention must not turn reclaimable objects into an allocation
    // failure. Detach each cache before touching the global heap.
    for cache in HEAP_CACHES.iter().take(sys::smp::cpu_count()) {
        let classes = core::mem::replace(
            &mut cache.lock().classes,
            [const { Magazine::new() }; SIZE_CLASSES.len()],
        );
        for (class, mut magazine) in classes.into_iter().enumerate() {
            let layout = Layout::from_size_align(SIZE_CLASSES[class], SIZE_CLASSES[class]).unwrap();
            while let Some(pointer) = magazine.pop() {
                deallocate_uncached(pointer, layout);
            }
        }
    }
    for pool in POOLS.iter().take(sys::smp::cpu_count().max(1)) {
        pool.lock().trim();
    }
    let mut heap = arena_lock();
    heap.flush_pending();
    if heap.retired.len != 0 {
        tlb::flush_everything();
        heap.drain_retired();
    }
}

fn cache_deallocation(pointer: *mut u8, layout: Layout) -> bool {
    if layout.size() == 0 {
        return false;
    }
    let Some(class) = class_index(layout) else {
        return false;
    };
    let Some(cache) = local_cache() else {
        return false;
    };
    let mut spilled = [null_mut(); MAGAZINE_CAPACITY / 2];
    {
        let mut cache = cache.lock();
        let magazine = &mut cache.classes[class];
        if magazine.len == MAGAZINE_CAPACITY {
            for slot in &mut spilled {
                *slot = magazine.pop().unwrap();
            }
        }
        magazine.push(pointer);
    }
    if !spilled[0].is_null() {
        // Never take the sleeping heap mutex while holding a cache spinlock.
        // Spill outside the cache lock; slots may belong to other CPU pools.
        for pointer in spilled {
            deallocate_uncached(pointer, layout);
        }
    }
    true
}

/// Marker stored inside freed slab objects.
#[repr(C)]
struct FreeNode {
    next: *mut FreeNode,
}

/// Per-slab header placed at the front of each slab page.
#[repr(C, align(64))]
struct SlabHeader {
    owner: usize,
    state: UnsafeCell<SlabState>,
}

#[repr(C)]
struct SlabState {
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
    LargeHead = 3,
    LargeTail = 4,
    Retired = 5,
}

/// Side metadata for one heap page.
#[derive(Copy, Clone)]
struct HeapPageMeta {
    kind: PageKind,
    aux: u32,
}

impl HeapPageMeta {
    const FREE: Self = Self {
        kind: PageKind::Free,
        aux: 0,
    };
}

/// Global allocator state guarded by the kernel heap mutex.
struct HeapArena {
    search_hint: usize,
    pages: [HeapPageMeta; HEAP_PAGES],
    retired: RetireQueue,
    pending: [Option<(usize, &'static phys::Page)>; tlb::Shootdown::INLINE],
    pending_count: usize,
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

#[repr(align(64))]
struct SlabPool {
    partial: [u32; SIZE_CLASSES.len()],
    empty_cache: [u32; SIZE_CLASSES.len()],
    next_page: usize,
    spare_pages: usize,
}

static READY: AtomicBool = AtomicBool::new(false);
static HEAP_ROOT: AtomicU64 = AtomicU64::new(0);
static POOLS: [Mutex<SlabPool>; tlb::MAX_TLB_CPUS] = [const {
    Mutex::new(SlabPool {
        partial: [NONE_PAGE; SIZE_CLASSES.len()],
        empty_cache: [NONE_PAGE; SIZE_CLASSES.len()],
        next_page: 0,
        spare_pages: 0,
    })
}; tlb::MAX_TLB_CPUS];

impl SlabPool {
    fn alloc_slab(&mut self, class: usize, owner: usize) -> Option<*mut u8> {
        let mut page_idx = self.partial[class];
        if page_idx == NONE_PAGE {
            page_idx = self.grow_slab(class, owner)?;
        }

        // SAFETY: the owning pool lock exclusively protects this mapped slab.
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

    fn dealloc_slab(&mut self, page_idx: usize, ptr: *mut u8, class: usize) {
        // SAFETY: the live slot pins the slab and its owning pool is locked.
        let slab = unsafe { slab_header(page_idx) };
        debug_assert_eq!(slab.class as usize, class);
        debug_assert!(slab.free_count < slab.capacity);
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
        arena_lock().release_page(page_idx);
    }

    fn grow_slab(&mut self, class: usize, owner: usize) -> Option<u32> {
        if self.spare_pages == 0 {
            let mut arena = arena_lock();
            arena.collect_retired();
            let (start, count) = arena
                .reserve_run(SLAB_RESERVE, 1)
                .map(|start| (start, SLAB_RESERVE))
                .or_else(|| arena.reserve_run(1, 1).map(|start| (start, 1)))?;
            for page in &mut arena.pages[start..start + count] {
                // Slabs are retired independently, so no span may skip a
                // freed neighbor when the shared arena searches for space.
                page.aux = 1;
            }
            self.next_page = start;
            self.spare_pages = count;
        }
        let page_idx = self.next_page;
        let page_va = heap_page_virt(page_idx);
        let root = heap_root();
        let phys_page = phys::alloc_zeroed_page(phys::PageUse::KernelHeap)?;

        // SAFETY: the reserved heap virtual page is currently unmapped and the
        // new physical page is exclusively owned by the allocator.
        let mapped =
            unsafe { arch::paging::map_page(root, page_va, phys_page.paddr(), heap_flags()) };
        if mapped.is_err() {
            // SAFETY: mapping failed, so no alias to this page was published.
            unsafe { phys::free_page(phys_page) };
            return None;
        }

        // SAFETY: the page was just mapped exclusively for this slab class.
        unsafe { init_slab_page(page_idx, class, owner) };
        self.next_page += 1;
        self.spare_pages -= 1;
        self.insert_partial(class, page_idx as u32);

        Some(page_idx as u32)
    }

    fn trim(&mut self) {
        let mut arena = arena_lock();
        arena.clear_run(self.next_page, self.spare_pages);
        self.spare_pages = 0;
        for class in 0..SIZE_CLASSES.len() {
            let page = self.empty_cache[class];
            if page != NONE_PAGE {
                self.remove_partial(class, page);
                self.empty_cache[class] = NONE_PAGE;
                arena.release_page(page as usize);
            }
        }
    }

    fn insert_partial(&mut self, class: usize, page_idx: u32) {
        // SAFETY: the owning pool lock protects its partial-list links.
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
        // SAFETY: the owning pool lock protects its partial-list links.
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
}

impl HeapArena {
    const fn new() -> Self {
        Self {
            search_hint: 0,
            pages: [HeapPageMeta::FREE; HEAP_PAGES],
            retired: RetireQueue::new(),
            pending: [None; tlb::Shootdown::INLINE],
            pending_count: 0,
        }
    }

    fn dealloc(&mut self, ptr: *mut u8, layout: Layout) {
        self.collect_retired();

        if layout.size() == 0 {
            return;
        }

        let page_idx = ptr_page_index(ptr).expect("mem/alloc: pointer outside heap window");
        let meta = self.pages[page_idx];

        match meta.kind {
            PageKind::LargeHead => self.dealloc_large(page_idx),
            PageKind::LargeTail => panic!("mem/alloc: dealloc on non-head large allocation"),
            _ => panic!("mem/alloc: invalid heap page state during free"),
        }
    }

    fn dealloc_large(&mut self, start: usize) {
        let pages = self.pages[start].aux as usize;
        let root = heap_root();

        for base in (start..start + pages).step_by(tlb::Shootdown::INLINE) {
            let count = (start + pages - base).min(tlb::Shootdown::INLINE);
            let mut retired = [None; tlb::Shootdown::INLINE];
            for (offset, page) in retired[..count].iter_mut().enumerate() {
                *page = Some(take_mapped_page(root, base + offset, "large"));
            }
            // One publication and IPI round for the whole batch. Publishing
            // before every leaf was removed would allow premature reuse.
            let sequence = tlb::publish_async_range(heap_page_virt(base), count);
            for (offset, page) in retired[..count].iter_mut().enumerate() {
                let idx = base + offset;
                self.quarantine_page(idx, page.take().unwrap(), start + pages - idx, sequence);
            }
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

    fn release_page(&mut self, page_idx: usize) {
        let root = heap_root();
        let page = take_mapped_page(root, page_idx, "slab");
        if sys::smp::online_cpus() <= 1 {
            self.quarantine_page(page_idx, page, 1, 0);
        } else {
            self.pages[page_idx] = HeapPageMeta {
                kind: PageKind::Retired,
                aux: 1,
            };
            self.pending[self.pending_count] = Some((page_idx, page));
            self.pending_count += 1;
            if self.pending_count == self.pending.len() {
                self.flush_pending();
            }
        }
        self.search_hint = self.search_hint.min(page_idx);
    }

    fn flush_pending(&mut self) {
        let count = core::mem::replace(&mut self.pending_count, 0);
        if count == 0 {
            return;
        }
        let mut addresses = [0; tlb::Shootdown::INLINE];
        for (index, address) in addresses[..count].iter_mut().enumerate() {
            *address = heap_page_virt(self.pending[index].unwrap().0).as_u64();
        }
        let sequence = tlb::publish_async_pages(&addresses[..count]);
        for index in 0..count {
            let (page_idx, page) = self.pending[index].take().unwrap();
            self.quarantine_page(page_idx, page, 1, sequence);
        }
    }

    fn quarantine_page(
        &mut self,
        page_idx: usize,
        page: &'static phys::Page,
        span: usize,
        sequence: u64,
    ) {
        if sys::smp::online_cpus() <= 1 {
            // SAFETY: the page was unmapped above and no remote TLB can retain
            // an alias in the single-CPU case.
            unsafe { phys::free_page(page) };
            self.pages[page_idx] = HeapPageMeta::FREE;
            return;
        }

        self.pages[page_idx] = HeapPageMeta {
            kind: PageKind::Retired,
            aux: span as u32,
        };

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
        self.flush_pending();
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
/// Small allocations use CPU slab pools; page runs are owned by [`HEAP`].
pub struct KernelAllocator;

static HEAP: Mutex<HeapArena> = Mutex::new(HeapArena::new());

#[global_allocator]
static GLOBAL_ALLOCATOR: KernelAllocator = KernelAllocator;

// SAFETY: pools serialize their slab state, and HEAP owns disjoint page runs.
// Cached slots retain their allocation until returned to a pool or caller;
// retired frames remain quarantined through remote TLB acknowledgement.
unsafe impl GlobalAlloc for KernelAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        allocate(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if !cache_deallocation(ptr, layout) {
            deallocate_uncached(ptr, layout);
        }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = allocate(layout);
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

        // Keep the backing class consistent with the layout used on the next
        // free. In particular, shrinking a large allocation into a slab class
        // must not retain that large backing in a small-object magazine.
        // SAFETY: `layout.align()` is a valid allocation alignment.
        let new_layout = unsafe { Layout::from_size_align_unchecked(new_size, layout.align()) };
        let current = class_index(layout).map_or_else(
            || pages_for_len(layout.size() as u64) as usize * PAGE_SIZE as usize,
            |class| SIZE_CLASSES[class],
        );
        if new_size <= current && class_index(layout) == class_index(new_layout) {
            return ptr;
        }

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
    HEAP_ROOT.store(arch::paging::active_root().as_u64(), Ordering::Relaxed);
    READY.store(true, Ordering::Release);

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

unsafe fn slab_header<'a>(page_idx: usize) -> &'a mut SlabState {
    // SAFETY: callers retain the owning pool lock for the returned borrow,
    // and the slab remains mapped until it is removed from that pool.
    unsafe {
        &mut *(*heap_page_virt(page_idx).as_mut_ptr::<SlabHeader>())
            .state
            .get()
    }
}

unsafe fn init_slab_page(page_idx: usize, class: usize, owner: usize) {
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
                owner,
                state: UnsafeCell::new(SlabState {
                    next: NONE_PAGE,
                    prev: NONE_PAGE,
                    class: class as u16,
                    capacity: capacity as u16,
                    free_count: capacity as u16,
                    _reserved: 0,
                    free_head: null_mut(),
                }),
            },
        );

        let slab = slab_header(page_idx);
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
fn heap_root() -> PhysAddr {
    PhysAddr::new(HEAP_ROOT.load(Ordering::Relaxed))
}

#[inline]
fn arena_lock() -> crate::sys::sync::MutexGuard<'static, HeapArena> {
    HEAP.lock()
}
