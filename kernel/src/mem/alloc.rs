//!
//! # Kernel Heap Allocator
//!
//! Fast page-backed heap allocator used as the kernel's global allocator.
//!

use core::{
    alloc::{GlobalAlloc, Layout},
    cmp,
    mem::size_of,
    ptr::{self, null_mut},
};

use log::info;

use crate::{
    arch,
    mem::{self, pages_for_len, phys, PhysAddr, VirtAddr, VmFlags, PAGE_SIZE},
    sys::sync::Mutex,
};

/// Base virtual address for the kernel heap window.
const HEAP_BASE: u64 = 0xFFFF_A000_0000_0000;
/// Total virtual address space reserved for heap growth.
const HEAP_SIZE: u64 = 1 << 30;
/// Number of heap pages inside the reserved window.
const HEAP_PAGES: usize = (HEAP_SIZE / PAGE_SIZE) as usize;
/// Sentinel value representing an invalid page index.
const NONE_PAGE: u32 = u32::MAX;
/// Slab size classes used for hot-path allocations.
const SIZE_CLASSES: [usize; 8] = [8, 16, 32, 64, 128, 256, 512, 1024];

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

    fn dealloc_slab(&mut self, page_idx: usize, ptr: *mut u8) {
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

        let head = start as u32;
        for idx in (start + 1)..(start + pages) {
            self.pages[idx] = HeapPageMeta {
                kind: PageKind::LargeTail,
                class: 0,
                _reserved: 0,
                aux: head,
            };
        }

        Some(heap_page_virt(start).as_mut_ptr())
    }

    fn dealloc_large(&mut self, start: usize) {
        let pages = self.pages[start].aux as usize;
        let root = arch::paging::active_root();

        for idx in start..(start + pages) {
            let virt = heap_page_virt(idx);
            let phys = unsafe { arch::paging::unmap_page(root, virt) }
                .expect("mem/alloc: failed to unmap large heap page")
                .expect("mem/alloc: missing large heap mapping");
            let page = phys::phys_to_page(phys).expect("mem/alloc: heap page missing PFN metadata");
            unsafe { phys::free_page(page) };
            self.pages[idx] = HeapPageMeta::FREE;
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
                aux: 0,
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
                if self.pages[probe].kind != PageKind::Free {
                    idx = align_up_pages(probe + 1, align_pages);
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

    fn release_page(&mut self, page_idx: usize) {
        let root = arch::paging::active_root();
        let virt = heap_page_virt(page_idx);
        let phys = unsafe { arch::paging::unmap_page(root, virt) }
            .expect("mem/alloc: failed to unmap slab page")
            .expect("mem/alloc: missing slab page mapping");
        let page = phys::phys_to_page(phys).expect("mem/alloc: slab page missing PFN metadata");

        unsafe { phys::free_page(page) };
        self.pages[page_idx] = HeapPageMeta::FREE;
        self.search_hint = self.search_hint.min(page_idx);
    }

    fn rollback_large(&mut self, root: PhysAddr, start: usize, mapped: usize) {
        for idx in start..(start + mapped) {
            let virt = heap_page_virt(idx);
            let phys = unsafe { arch::paging::unmap_page(root, virt) }
                .expect("mem/alloc: failed to roll back heap mapping")
                .expect("mem/alloc: missing heap mapping during rollback");
            let page =
                phys::phys_to_page(phys).expect("mem/alloc: heap rollback PFN lookup failed");
            unsafe { phys::free_page(page) };
        }
    }
}

/// Global allocator facade wired into Rust's allocation hooks.
pub struct KernelAllocator;

static HEAP: Mutex<HeapState> = Mutex::new(HeapState::new());

#[global_allocator]
static GLOBAL_ALLOCATOR: KernelAllocator = KernelAllocator;

unsafe impl GlobalAlloc for KernelAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        HEAP.lock().alloc(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        HEAP.lock().dealloc(ptr, layout);
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = HEAP.lock().alloc(layout);
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
            let heap = HEAP.lock();
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
    let mut heap = HEAP.lock();
    heap.init();

    info!(
        "mem/alloc: heap window active: virt=[0x{:x}-0x{:x}] size={} MiB",
        HEAP_BASE,
        HEAP_BASE + HEAP_SIZE,
        HEAP_SIZE / (1024 * 1024),
    );
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
