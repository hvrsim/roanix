//! Register-window mapping.
//!
//! Device registers live at arbitrary physical addresses that are usually not
//! covered by a cacheable direct map, so the framework maintains its own kernel
//! virtual window and a small range allocator over it.
//!
//! The window occupies exactly one top-level page-table entry and its
//! intermediate tables are created during initialization, before the first user
//! address space exists. Because a new address space copies the kernel half of
//! the root table, every later mapping inside the window is automatically
//! visible from every address space without further synchronization.
//!
//! Once a window is mapped, drivers access it with inline load and store
//! helpers. No framework call is involved in a register access.

use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::{
    arch,
    mem::{self, PAGE_SIZE, PhysAddr, VirtAddr, VmFlags, phys},
    sys::sync::{Mutex, Once},
};

use super::super::{
    core::module::Module,
    error::{Error, Result},
};

/// Base of the framework's device-mapping window.
///
/// This is aligned to, and no larger than, one top-level page-table entry so
/// that sharing the kernel half of the root table shares the whole window. The
/// kernel's other high-half regions are the direct map, the heap, and the frame
/// database; the assertions below keep this entry distinct from all of them.
const WINDOW_BASE: u64 = 0xFFFF_C000_0000_0000;
/// Size of the device-mapping window.
const WINDOW_SIZE: u64 = 512 << 30;
/// Number of pages in the window.
const WINDOW_PAGES: u64 = WINDOW_SIZE / PAGE_SIZE;
/// Pages left unmapped between adjacent mappings to catch overruns.
const GUARD_PAGES: u64 = 1;

/// Top-level page-table entry covering an address.
const fn top_level_index(address: u64) -> u64 {
    (address >> 39) & 0x1ff
}

const _: () = assert!(
    WINDOW_SIZE == 512 << 30,
    "the window must occupy exactly one top-level entry"
);
const _: () = assert!(
    top_level_index(WINDOW_BASE) != top_level_index(crate::mem::phys::PAGEDB_ADDR),
    "the device window overlaps the frame database"
);
const _: () = assert!(
    top_level_index(WINDOW_BASE) != top_level_index(crate::mem::HEAP_BASE),
    "the device window overlaps the kernel heap"
);

/// Mapping attribute flags.
pub mod flags {
    /// Strongly ordered uncached device memory. This is the default.
    pub const DEVICE: u32 = 0;
    /// Write-combining memory for streaming writes such as a framebuffer.
    pub const WRITE_COMBINE: u32 = 1 << 0;
    /// Ordinary cacheable memory.
    pub const CACHED: u32 = 1 << 1;
    /// Map without write permission.
    pub const READONLY: u32 = 1 << 2;
}

/// A live register window owned by a module.
pub struct Mapping {
    base: VirtAddr,
    physical: PhysAddr,
    pages: u64,
    length: usize,
    owner: Option<Arc<Module>>,
}

impl Mapping {
    /// Returns the kernel virtual address of the first mapped byte.
    pub const fn address(&self) -> VirtAddr {
        self.base
    }

    /// Returns the physical address of the first mapped byte.
    pub const fn physical(&self) -> PhysAddr {
        self.physical
    }

    /// Returns the number of bytes the caller asked for.
    pub const fn len(&self) -> usize {
        self.length
    }

    /// Returns whether the window is empty.
    pub const fn is_empty(&self) -> bool {
        self.length == 0
    }
}

struct Range {
    start: u64,
    pages: u64,
}

struct Allocator {
    free: Vec<Range>,
}

impl Allocator {
    fn new() -> Self {
        Self {
            free: alloc::vec![Range {
                // Page zero stays permanently mapped so the window's
                // intermediate page tables always exist.
                start: 1,
                pages: WINDOW_PAGES - 1,
            }],
        }
    }

    fn allocate(&mut self, pages: u64) -> Option<u64> {
        let needed = pages.checked_add(GUARD_PAGES)?;
        for (index, range) in self.free.iter_mut().enumerate() {
            if range.pages < needed {
                continue;
            }
            let start = range.start;
            range.start += needed;
            range.pages -= needed;
            if range.pages == 0 {
                self.free.remove(index);
            }
            return Some(start);
        }
        None
    }

    fn release(&mut self, start: u64, pages: u64) {
        let total = pages + GUARD_PAGES;
        let position = self
            .free
            .iter()
            .position(|range| range.start > start)
            .unwrap_or(self.free.len());
        self.free.insert(position, Range { start, pages: total });
        self.coalesce(position);
    }

    fn coalesce(&mut self, position: usize) {
        if position + 1 < self.free.len() {
            let next_start = self.free[position + 1].start;
            let end = self.free[position].start + self.free[position].pages;
            if end == next_start {
                let next_pages = self.free[position + 1].pages;
                self.free[position].pages += next_pages;
                self.free.remove(position + 1);
            }
        }
        if position > 0 {
            let previous_end = self.free[position - 1].start + self.free[position - 1].pages;
            if previous_end == self.free[position].start {
                let pages = self.free[position].pages;
                self.free[position - 1].pages += pages;
                self.free.remove(position);
            }
        }
    }
}

struct State {
    allocator: Mutex<Allocator>,
    mappings: Mutex<Vec<Arc<Mapping>>>,
    mapped_bytes: AtomicU64,
}

static STATE: Once<State> = Once::new();

pub(crate) fn init() {
    STATE.call_once(|| {
        reserve_window();
        State {
            allocator: Mutex::new(Allocator::new()),
            mappings: Mutex::new(Vec::new()),
            mapped_bytes: AtomicU64::new(0),
        }
    });
}

/// Maps the window's first page so its page tables exist before any user
/// address space copies the kernel half of the root table.
fn reserve_window() {
    let root = arch::paging::active_root();
    let base = VirtAddr::new(WINDOW_BASE);
    // SAFETY: the kernel root is valid and this only reads the hierarchy.
    if let Some(existing) = unsafe { arch::paging::translate(root, base) } {
        panic!(
            "driver/io: device window base {:#x} is already mapped to {:#x}",
            WINDOW_BASE,
            existing.as_u64()
        );
    }

    let Some(page) = phys::alloc_zeroed_page(phys::PageUse::KernelHeap) else {
        panic!("driver/io: no memory to reserve the device window");
    };
    // SAFETY: the window base is unmapped at this point and the page is owned
    // exclusively by this permanent reservation.
    unsafe {
        arch::paging::map_page(
            root,
            base,
            page.paddr(),
            VmFlags::READ | VmFlags::GLOBAL,
        )
    }
    .unwrap_or_else(|error| {
        panic!("driver/io: failed to reserve the device window: {error:?}")
    });
}

fn state() -> Result<&'static State> {
    STATE.get().ok_or(Error::NotInitialized)
}

fn page_virt(page: u64) -> VirtAddr {
    VirtAddr::new(WINDOW_BASE + page * PAGE_SIZE)
}

fn vm_flags(attributes: u32) -> VmFlags {
    let mut flags = VmFlags::READ | VmFlags::GLOBAL;
    if attributes & flags::READONLY == 0 {
        flags |= VmFlags::WRITE;
    }
    if attributes & flags::WRITE_COMBINE != 0 {
        flags |= VmFlags::WRITE_COMBINE;
    } else if attributes & flags::CACHED == 0 {
        flags |= VmFlags::DEVICE;
    }
    flags
}

/// Maps `length` bytes of physical memory starting at `physical`.
///
/// The returned window starts at the byte corresponding to `physical`, even
/// when the underlying mapping had to be aligned down to a page boundary.
pub fn map(
    owner: Option<&Arc<Module>>,
    physical: u64,
    length: usize,
    attributes: u32,
) -> Result<Arc<Mapping>> {
    if length == 0 {
        return Err(Error::InvalidArgument);
    }
    let state = state()?;
    let offset = physical % PAGE_SIZE;
    let aligned = physical - offset;
    let span = (offset as usize)
        .checked_add(length)
        .ok_or(Error::InvalidArgument)?;
    let pages = (span as u64).div_ceil(PAGE_SIZE);
    if aligned.checked_add(pages * PAGE_SIZE).is_none() {
        return Err(Error::InvalidArgument);
    }

    let start = state
        .allocator
        .lock()
        .allocate(pages)
        .ok_or(Error::NoSpace)?;
    let root = mem::kernel_page_root();
    let vm_flags = vm_flags(attributes);

    for index in 0..pages {
        let virt = page_virt(start + index);
        let phys = PhysAddr::new(aligned + index * PAGE_SIZE);
        // SAFETY: the range allocator guarantees these virtual pages are
        // currently unmapped, and the caller is responsible for the physical
        // range describing real device registers.
        if unsafe { arch::paging::map_page(root, virt, phys, vm_flags) }.is_err() {
            for undo in 0..index {
                // SAFETY: these pages were just mapped by this loop.
                unsafe {
                    let _ = arch::paging::unmap_page(root, page_virt(start + undo));
                }
            }
            state.allocator.lock().release(start, pages);
            return Err(Error::NoSpace);
        }
    }
    mem::flush_tlb_range(page_virt(start), pages * PAGE_SIZE);

    let mapping = Arc::new(Mapping {
        base: VirtAddr::new(page_virt(start).as_u64() + offset),
        physical: PhysAddr::new(physical),
        pages,
        length,
        owner: owner.cloned(),
    });
    state.mappings.lock().push(mapping.clone());
    state
        .mapped_bytes
        .fetch_add(pages * PAGE_SIZE, Ordering::Relaxed);
    Ok(mapping)
}

/// Unmaps a window.
pub fn unmap(mapping: &Arc<Mapping>) -> Result<()> {
    let state = state()?;
    let removed = {
        let mut mappings = state.mappings.lock();
        let position = mappings
            .iter()
            .position(|entry| Arc::ptr_eq(entry, mapping))
            .ok_or(Error::NotFound)?;
        mappings.remove(position)
    };
    release(&removed);
    Ok(())
}

fn release(mapping: &Arc<Mapping>) {
    let Ok(state) = state() else {
        return;
    };
    let root = mem::kernel_page_root();
    let start = (mapping.base.as_u64() - WINDOW_BASE) / PAGE_SIZE;
    for index in 0..mapping.pages {
        // SAFETY: this window owns these virtual pages and no driver may access
        // them after the mapping is released.
        unsafe {
            let _ = arch::paging::unmap_page(root, page_virt(start + index));
        }
    }
    mem::flush_tlb_range(page_virt(start), mapping.pages * PAGE_SIZE);
    state.allocator.lock().release(start, mapping.pages);
    state
        .mapped_bytes
        .fetch_sub(mapping.pages * PAGE_SIZE, Ordering::Relaxed);
}

pub(in super::super) fn remove_module_mappings(module: &Arc<Module>) {
    let Ok(state) = state() else {
        return;
    };
    let owned: Vec<Arc<Mapping>> = {
        let mut mappings = state.mappings.lock();
        let mut owned = Vec::new();
        mappings.retain(|mapping| {
            let matches = mapping
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module));
            if matches {
                owned.push(mapping.clone());
            }
            !matches
        });
        owned
    };
    for mapping in owned {
        release(&mapping);
    }
}

/// Returns the number of bytes currently mapped into the device window.
pub fn mapped_bytes() -> u64 {
    STATE
        .get()
        .map_or(0, |state| state.mapped_bytes.load(Ordering::Relaxed))
}

/// Returns the kernel virtual address of a physical address in the direct map.
///
/// This is only valid for physical memory the boot protocol already mapped,
/// such as firmware tables, and never for device registers.
pub fn direct_map(physical: u64) -> Result<VirtAddr> {
    if !mem::is_direct_mapped(PhysAddr::new(physical)) {
        return Err(Error::InvalidArgument);
    }
    Ok(mem::phys_to_virt(PhysAddr::new(physical)))
}
