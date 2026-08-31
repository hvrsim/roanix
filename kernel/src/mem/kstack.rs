//!
//! # Guarded kernel stacks
//!
//! Kernel thread stacks are carved out of a dedicated virtual arena instead of
//! the general heap so every stack can be preceded by a permanently unmapped
//! guard page. A stack overflow then faults on the guard page immediately
//! rather than silently corrupting neighbouring heap objects.
//!
//! Each slot is laid out as:
//!
//! ```text
//! | guard page (unmapped) | stack pages ... | <- stack top
//! ```
//!
//! The arena occupies a single top-level page-table entry. Memory
//! initialization pre-populates every kernel-half top-level entry before the
//! first user address space is created, so later slot mappings are shared
//! automatically without a permanent sentinel stack.
//!

use core::sync::atomic::{AtomicUsize, Ordering};

use crate::{
    arch,
    mem::{PAGE_SIZE, PhysAddr, VirtAddr, VmFlags, phys},
    sys::smp::IrqSpinLock,
};

/// Base of the kernel stack arena.
const ARENA_BASE: u64 = 0xFFFF_D000_0000_0000;

/// Total arena size. Kept at 1 GiB so the arena never spans more than one
/// top-level page-table entry on any supported paging mode.
const ARENA_SIZE: u64 = 1 << 30;

/// Pages reserved per slot, including the leading guard page.
const SLOT_PAGES: usize = 16;

/// Usable stack pages per slot.
pub(crate) const STACK_PAGES: usize = SLOT_PAGES - 1;

/// Usable stack bytes per slot.
pub(crate) const STACK_SIZE: usize = STACK_PAGES * (PAGE_SIZE as usize);

const SLOT_SIZE: u64 = SLOT_PAGES as u64 * PAGE_SIZE;
const SLOT_COUNT: usize = (ARENA_SIZE / SLOT_SIZE) as usize;
const BITMAP_WORDS: usize = SLOT_COUNT.div_ceil(64);

const _: () = assert!(ARENA_SIZE.is_power_of_two());
const _: () = assert!(SLOT_COUNT * SLOT_PAGES * (PAGE_SIZE as usize) == ARENA_SIZE as usize);

struct Arena {
    words: [u64; BITMAP_WORDS],
    hint: usize,
}

static ARENA: IrqSpinLock<Arena> = IrqSpinLock::new(Arena::new());
/// Serializes page-table mutation inside the arena.
///
/// `map_page` creates intermediate tables with a non-atomic read-modify-write,
/// so two CPUs allocating slots in the same 2 MiB region could otherwise both
/// install a table and lose one of them. The arena occupies its own top-level
/// entry, so no other subsystem shares these tables.
static MAPPING: IrqSpinLock<()> = IrqSpinLock::new(());
static LIVE_STACKS: AtomicUsize = AtomicUsize::new(0);

/// A mapped kernel stack with a guard page below it.
pub(crate) struct KernelStack {
    slot: usize,
}

impl Arena {
    const fn new() -> Self {
        Self {
            words: [0; BITMAP_WORDS],
            hint: 0,
        }
    }

    fn take(&mut self) -> Option<usize> {
        for offset in 0..BITMAP_WORDS {
            let word_index = (self.hint + offset) % BITMAP_WORDS;
            let word = self.words[word_index];
            if word == u64::MAX {
                continue;
            }

            let bit = word.trailing_ones() as usize;
            let slot = word_index * 64 + bit;
            if slot >= SLOT_COUNT {
                continue;
            }

            self.words[word_index] |= 1u64 << bit;
            self.hint = word_index;
            return Some(slot);
        }

        None
    }

    fn release(&mut self, slot: usize) {
        assert!(slot < SLOT_COUNT, "mem/kstack: slot {slot} out of range");
        let mask = 1u64 << (slot % 64);
        assert!(
            self.words[slot / 64] & mask != 0,
            "mem/kstack: slot {slot} released twice"
        );
        self.words[slot / 64] &= !mask;
        self.hint = self.hint.min(slot / 64);
    }
}

impl KernelStack {
    /// Returns the lowest usable stack address, immediately above the guard page.
    pub(crate) fn base(&self) -> VirtAddr {
        VirtAddr::new(slot_base(self.slot) + PAGE_SIZE)
    }

    /// Returns the exclusive top of the stack.
    pub(crate) fn top(&self) -> VirtAddr {
        VirtAddr::new(slot_base(self.slot) + SLOT_SIZE)
    }
}

impl Drop for KernelStack {
    fn drop(&mut self) {
        let root = crate::mem::kernel_page_root();
        let base = self.base().as_u64();
        let mut pages: [Option<&'static phys::Page>; STACK_PAGES] = [None; STACK_PAGES];

        {
            let _mapping = MAPPING.lock();
            for (index, slot) in pages.iter_mut().enumerate() {
                let virt = VirtAddr::new(base + index as u64 * PAGE_SIZE);
                // SAFETY: this slot is exclusively owned by the stack being
                // dropped, and no thread can still be executing on it.
                let phys = unsafe { arch::paging::unmap_page(root, virt) }
                    .unwrap_or_else(|_| panic!("mem/kstack: failed to unmap stack page"))
                    .unwrap_or_else(|| panic!("mem/kstack: missing stack mapping"));
                *slot = Some(
                    phys::phys_to_page(phys)
                        .unwrap_or_else(|| panic!("mem/kstack: stack page missing PFN metadata")),
                );
            }
        }

        // Remote CPUs must drop these translations before the frames or slot
        // are reused. The range fits in one shootdown batch, avoiding a full
        // global TLB flush on every thread exit.
        crate::mem::flush_tlb_range(self.base(), STACK_SIZE as u64);

        for page in pages.into_iter().flatten() {
            // SAFETY: the only mapping of this page was removed above and every
            // CPU has acknowledged the invalidation.
            unsafe { phys::free_page(page) };
        }

        ARENA.lock().release(self.slot);
        LIVE_STACKS.fetch_sub(1, Ordering::Relaxed);
    }
}

fn slot_base(slot: usize) -> u64 {
    ARENA_BASE + slot as u64 * SLOT_SIZE
}

fn map_slot(slot: usize) -> Option<()> {
    let root = crate::mem::kernel_page_root();
    let base = slot_base(slot) + PAGE_SIZE;

    // Allocate first so the page-table lock is only held for the mapping
    // writes rather than across page zeroing.
    let mut pages: [Option<&'static phys::Page>; STACK_PAGES] = [None; STACK_PAGES];
    for slot in pages.iter_mut() {
        let Some(page) = phys::alloc_zeroed_page(phys::PageUse::KernelHeap) else {
            free_unmapped(&mut pages);
            return None;
        };
        *slot = Some(page);
    }

    let _mapping = MAPPING.lock();
    for index in 0..STACK_PAGES {
        let page = pages[index].expect("mem/kstack: missing preallocated stack page");
        let virt = VirtAddr::new(base + index as u64 * PAGE_SIZE);
        // SAFETY: the slot is exclusively owned by this allocation, the target
        // range is reserved for kernel stacks, and the arena's page tables are
        // serialized by `MAPPING`.
        let mapped = unsafe {
            arch::paging::map_page(
                root,
                virt,
                page.paddr(),
                VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL,
            )
        };
        if mapped.is_err() {
            unmap_partial(root, base, index);
            free_unmapped(&mut pages[index..]);
            return None;
        }
        pages[index] = None;
    }

    Some(())
}

/// Frees physical pages that were allocated but never mapped.
fn free_unmapped(pages: &mut [Option<&'static phys::Page>]) {
    for page in pages.iter_mut().filter_map(Option::take) {
        // SAFETY: these pages were never mapped, so they have no alias.
        unsafe { phys::free_page(page) };
    }
}

fn unmap_partial(root: PhysAddr, base: u64, mapped: usize) {
    for index in 0..mapped {
        let virt = VirtAddr::new(base + index as u64 * PAGE_SIZE);
        // SAFETY: rollback only touches mappings this allocation installed.
        if let Ok(Some(phys)) = unsafe { arch::paging::unmap_page(root, virt) }
            && let Some(page) = phys::phys_to_page(phys)
        {
            // SAFETY: rollback removed the only mapping of this page.
            unsafe { phys::free_page(page) };
        }
    }
}

/// Allocates a zeroed kernel stack preceded by an unmapped guard page.
pub(crate) fn allocate() -> Option<KernelStack> {
    let slot = ARENA.lock().take()?;
    if map_slot(slot).is_none() {
        ARENA.lock().release(slot);
        return None;
    }

    LIVE_STACKS.fetch_add(1, Ordering::Relaxed);
    Some(KernelStack { slot })
}

/// Returns whether `address` falls inside the kernel stack arena.
pub(crate) fn contains_address(address: u64) -> bool {
    (ARENA_BASE..ARENA_BASE + ARENA_SIZE).contains(&address)
}

/// Returns whether `address` names a kernel stack guard page.
pub(crate) fn is_guard_address(address: u64) -> bool {
    contains_address(address) && (address - ARENA_BASE) % SLOT_SIZE < PAGE_SIZE
}

/// Logs a kernel stack overflow when `address` names a guard page.
///
/// Turning the fault into a named diagnostic is the whole point of the guard
/// page; without it an overflow reports as an anonymous unmapped access.
pub(crate) fn report_guard_fault(address: u64, cpu_id: usize, tid: usize) {
    if !is_guard_address(address) {
        return;
    }

    log::error!(
        "kernel stack overflow at 0x{address:X} on cpu {cpu_id}, tid {tid} ({} live stacks)",
        live_stacks()
    );
}

/// Returns the number of live kernel stacks.
pub(crate) fn live_stacks() -> usize {
    LIVE_STACKS.load(Ordering::Relaxed)
}
