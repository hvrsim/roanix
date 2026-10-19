//!
//! # Virtual Memory Paging
//!
//! x86_64 paging backend for the [`mem`] subsystem.
//!

use core::sync::atomic::{AtomicU64, Ordering};

use x86_64::{
    PhysAddr as X86PhysAddr, VirtAddr as X86VirtAddr,
    instructions::tlb,
    registers::control::{Cr3, Cr4, Cr4Flags},
    structures::paging::PhysFrame,
};

use crate::mem::{self, PAGE_SIZE, PhysAddr, VirtAddr, VmFlags, phys};

/// Mask extracting the physical address bits from a page-table entry.
const ENTRY_ADDR_MASK: u64 = 0x000f_ffff_ffff_f000;

/// Entry is present.
const PTE_PRESENT: u64 = 1 << 0;
/// Entry is writable.
const PTE_WRITE: u64 = 1 << 1;
/// Entry is user-accessible.
const PTE_USER: u64 = 1 << 2;
/// Write-through caching policy.
const PTE_PWT: u64 = 1 << 3;
/// Cache-disable policy.
const PTE_PCD: u64 = 1 << 4;
/// Hardware accessed bit.
const PTE_ACCESSED: u64 = 1 << 5;
/// Hardware dirty bit.
const PTE_DIRTY: u64 = 1 << 6;
/// Entry maps a huge page.
const PTE_HUGE: u64 = 1 << 7;
/// Global TLB entry.
const PTE_GLOBAL: u64 = 1 << 8;
/// Page-attribute-table selector for a 4 KiB leaf entry.
///
/// For 4 KiB leaves the PAT index is `(PAT << 2) | (PCD << 1) | PWT`, so this
/// bit alone selects PAT entry 4, which [`init_memory_types`] programs to
/// write-combining.
const PTE_PAT4K: u64 = 1 << 7;
/// Execute-disable bit.
const PTE_NX: u64 = 1 << 63;

/// Model-specific register holding the page-attribute table.
const IA32_PAT: u32 = 0x277;
/// Uncacheable memory type.
const PAT_UC: u64 = 0x00;
/// Write-combining memory type.
const PAT_WC: u64 = 0x01;
/// Write-through memory type.
const PAT_WT: u64 = 0x04;
/// Uncacheable memory type that a WC MTRR may override.
const PAT_UC_MINUS: u64 = 0x07;
/// Write-back memory type.
const PAT_WB: u64 = 0x06;

/// Programs the page-attribute table on the current CPU.
///
/// Entries 0 through 3 keep their architectural defaults so existing mappings
/// are unaffected, and entry 4 is redefined from write-back to write-combining
/// so [`VmFlags::WRITE_COMBINE`] can be expressed with the PAT bit alone. Every
/// CPU must run this before it uses a write-combining mapping.
pub fn init_memory_types() {
    let value = PAT_WB
        | (PAT_WT << 8)
        | (PAT_UC_MINUS << 16)
        | (PAT_UC << 24)
        | (PAT_WC << 32)
        | (PAT_WT << 40)
        | (PAT_UC_MINUS << 48)
        | (PAT_UC << 56);
    // SAFETY: IA32_PAT exists on every CPU supporting long mode, and the value
    // only redefines entry 4, which no existing mapping selects.
    unsafe {
        core::arch::asm!(
            "wrmsr",
            in("ecx") IA32_PAT,
            in("eax") value as u32,
            in("edx") (value >> 32) as u32,
            options(nostack, preserves_flags),
        );
    }
}

/// Paging operation errors.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum PagingError {
    /// One or more addresses are not page-aligned.
    UnalignedAddress,
    /// Address cannot be represented by architecture constraints.
    InvalidAddress,
    /// Failed to allocate a required page-table page.
    OutOfMemory,
    /// Mapping already exists at target virtual address.
    AlreadyMapped,
    /// Mapping does not exist.
    NotMapped,
    /// Encountered a huge-page mapping where a table was required.
    HugePageConflict,
}

/// Architecture paging result type.
pub type Result<T> = core::result::Result<T, PagingError>;

/// Returns active page table root physical address.
pub fn active_root() -> PhysAddr {
    let (frame, _) = Cr3::read();
    PhysAddr::new(frame.start_address().as_u64())
}

/// Activates the given page-table root.
///
/// # Safety
///
/// `root` must refer to a valid, currently mapped top-level x86_64 page table
/// whose contents are suitable for immediate execution on the current CPU.
pub unsafe fn activate_root(root: PhysAddr) -> Result<()> {
    if !root.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let frame = PhysFrame::from_start_address(X86PhysAddr::new(root.as_u64()))
        .map_err(|_| PagingError::InvalidAddress)?;
    let (_, flags) = Cr3::read();
    // SAFETY: the caller guarantees `root` is a valid active page-table root.
    unsafe { Cr3::write(frame, flags) };
    Ok(())
}

/// Maps one 4 KiB page.
///
/// # Safety
///
/// `root` must point to a valid writable page-table hierarchy owned by the
/// caller. The caller must ensure the mapping change is synchronized against
/// other CPUs and that `phys` is safe to expose at `virt` with `flags`.
pub unsafe fn map_page(
    root: PhysAddr,
    virt: VirtAddr,
    phys: PhysAddr,
    flags: VmFlags,
) -> Result<()> {
    if !root.is_page_aligned() || !virt.is_page_aligned() || !phys.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let v = virt.as_u64();
    let [idx_l4, idx_l3, idx_l2, idx_l1] = page_table_indexes(v);
    let mut table = root;
    let user = flags.contains(VmFlags::USER);

    // SAFETY: the caller guarantees the complete hierarchy is valid and
    // writable for the duration of this walk.
    unsafe {
        for idx in [idx_l4, idx_l3, idx_l2] {
            let parent = table_entry(table, idx);
            let mut entry = parent.load(Ordering::Acquire);
            if entry & PTE_PRESENT == 0 {
                let frame = phys::alloc_zeroed_phys(phys::PageUse::PageTable)
                    .ok_or(PagingError::OutOfMemory)?;
                let bits = PTE_PRESENT | PTE_WRITE | if user { PTE_USER } else { 0 };
                let new = frame.as_u64() | bits;
                entry = match parent.compare_exchange(
                    entry,
                    new,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => new,
                    Err(installed) => {
                        free_table_page(frame);
                        installed
                    }
                };
            }
            if entry & PTE_HUGE != 0 {
                return Err(PagingError::HugePageConflict);
            }
            table = PhysAddr::new(entry & ENTRY_ADDR_MASK);
        }

        let leaf_ptr = table_entry(table, idx_l1);
        if leaf_ptr.load(Ordering::Acquire) & PTE_PRESENT != 0 {
            return Err(PagingError::AlreadyMapped);
        }

        let mut bits = PTE_PRESENT;
        if flags.contains(VmFlags::WRITE) {
            bits |= PTE_WRITE;
        }
        if flags.contains(VmFlags::USER) {
            bits |= PTE_USER;
        }
        if flags.contains(VmFlags::GLOBAL) {
            bits |= PTE_GLOBAL;
        }
        if flags.contains(VmFlags::WRITE_COMBINE) {
            bits |= PTE_PAT4K;
        } else if flags.contains(VmFlags::DEVICE) {
            bits |= PTE_PWT | PTE_PCD;
        }
        if !flags.contains(VmFlags::EXECUTE) {
            bits |= PTE_NX;
        }

        leaf_ptr.store((phys.as_u64() & ENTRY_ADDR_MASK) | bits, Ordering::Release);
        tlb::flush(X86VirtAddr::new(v));
        Ok(())
    }
}

/// Replaces one existing 4 KiB mapping.
///
/// # Safety
///
/// The caller must own and synchronize the page-table hierarchy and guarantee
/// that `phys` remains valid for the resulting mapping.
pub unsafe fn remap_page(
    root: PhysAddr,
    virt: VirtAddr,
    phys: PhysAddr,
    flags: VmFlags,
) -> Result<()> {
    if !root.is_page_aligned() || !virt.is_page_aligned() || !phys.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }
    // SAFETY: the caller guarantees exclusive page-table mutation.
    unsafe {
        let leaf_ptr = walk_to_leaf(root, virt)?;
        if leaf_ptr.load(Ordering::Acquire) & PTE_PRESENT == 0 {
            return Err(PagingError::NotMapped);
        }
        let previous = leaf_ptr.swap(
            (phys.as_u64() & ENTRY_ADDR_MASK) | leaf_bits(flags),
            Ordering::AcqRel,
        );
        record_hardware_state(previous);
        tlb::flush(X86VirtAddr::new(virt.as_u64()));
    }
    Ok(())
}

/// Reads and clears the hardware accessed and dirty bits for one mapping.
///
/// # Safety
///
/// `root` must remain a valid page-table hierarchy and the caller must
/// serialize leaf updates.
pub unsafe fn take_accessed_dirty(root: PhysAddr, virt: VirtAddr) -> Result<(bool, bool)> {
    if !root.is_page_aligned() || !virt.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }
    // SAFETY: the caller guarantees a stable, writable hierarchy.
    unsafe {
        let leaf_ptr = walk_to_leaf(root, virt)?;
        let leaf = leaf_ptr;
        let value = leaf.load(Ordering::Acquire);
        if value & PTE_PRESENT == 0 {
            return Err(PagingError::NotMapped);
        }
        let previous = if value & (PTE_ACCESSED | PTE_DIRTY) != 0 {
            leaf.fetch_and(!(PTE_ACCESSED | PTE_DIRTY), Ordering::AcqRel)
        } else {
            value
        };
        let accessed = previous & PTE_ACCESSED != 0;
        let dirty = previous & PTE_DIRTY != 0;
        if accessed || dirty {
            tlb::flush(X86VirtAddr::new(virt.as_u64()));
        }
        Ok((accessed, dirty))
    }
}

/// Unmaps one 4 KiB page and returns the removed physical address.
///
/// # Safety
///
/// `root` must point to a valid writable page-table hierarchy owned by the
/// caller, and the caller must ensure no concurrent user will access `virt`
/// while the mapping is being removed.
pub unsafe fn unmap_page(root: PhysAddr, virt: VirtAddr) -> Result<Option<PhysAddr>> {
    if !root.is_page_aligned() || !virt.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    // SAFETY: the caller guarantees the hierarchy remains valid and
    // exclusively writable during the removal.
    unsafe {
        let leaf_ptr = walk_to_leaf(root, virt)?;
        let leaf = leaf_ptr.load(Ordering::Acquire);
        if leaf & PTE_PRESENT == 0 {
            return Err(PagingError::NotMapped);
        }

        let leaf = leaf_ptr.swap(0, Ordering::AcqRel);
        record_hardware_state(leaf);
        tlb::flush(X86VirtAddr::new(virt.as_u64()));
        Ok(Some(PhysAddr::new(leaf & ENTRY_ADDR_MASK)))
    }
}

/// Flushes the current CPU's TLB entry for `virt`.
pub fn flush_page(virt: VirtAddr) {
    tlb::flush(X86VirtAddr::new(virt.as_u64()));
}

/// Flushes every non-global translation on the current CPU.
pub fn flush_all() {
    tlb::flush_all();
}

/// Flushes every translation on the current CPU, including global entries.
///
/// A plain `CR3` reload preserves entries marked `PTE_GLOBAL`, so kernel
/// mappings must be invalidated by briefly clearing `CR4.PGE`.
pub fn flush_all_global() {
    let cr4 = Cr4::read();
    if !cr4.contains(Cr4Flags::PAGE_GLOBAL) {
        tlb::flush_all();
        return;
    }
    // SAFETY: clearing and restoring `CR4.PGE` only invalidates cached
    // translations; the surrounding paging configuration is unchanged.
    unsafe {
        Cr4::write(cr4 & !Cr4Flags::PAGE_GLOBAL);
        Cr4::write(cr4);
    }
}

/// Replaces the permissions of one existing leaf without changing its frame.
///
/// # Safety
///
/// `root` must own a valid hierarchy and the caller must serialize leaf
/// updates for `virt`.
pub unsafe fn protect_page(root: PhysAddr, virt: VirtAddr, flags: VmFlags) -> Result<PhysAddr> {
    if !root.is_page_aligned() || !virt.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }
    // SAFETY: the caller guarantees a stable, exclusively writable hierarchy.
    unsafe {
        let leaf_ptr = walk_to_leaf(root, virt)?;
        let leaf = leaf_ptr.load(Ordering::Acquire);
        if leaf & PTE_PRESENT == 0 {
            return Err(PagingError::NotMapped);
        }
        let frame = leaf & ENTRY_ADDR_MASK;
        leaf_ptr
            .try_update(Ordering::AcqRel, Ordering::Acquire, |entry| {
                Some(frame | leaf_bits(flags) | (entry & (PTE_ACCESSED | PTE_DIRTY)))
            })
            .expect("x86/paging: unconditional leaf update failed");
        tlb::flush(X86VirtAddr::new(virt.as_u64()));
        Ok(PhysAddr::new(frame))
    }
}

/// Pre-allocates every kernel-half top-level entry of `root`.
///
/// User roots copy the kernel half by value, so all shared next-level tables
/// must exist before the first user root is created. Once present, later
/// kernel mappings mutate the shared tables and are visible in every space.
///
/// # Safety
///
/// `root` must be the permanent kernel top-level table.
pub unsafe fn populate_kernel_tables(root: PhysAddr) -> Result<()> {
    // SAFETY: the caller guarantees `root` is the kernel hierarchy, which is
    // exclusively mutated during early initialization.
    unsafe {
        for index in 256..512 {
            let parent = table_entry(root, index);
            if parent.load(Ordering::Acquire) & PTE_PRESENT != 0 {
                continue;
            }
            let frame = phys::alloc_zeroed_phys(phys::PageUse::PageTable)
                .ok_or(PagingError::OutOfMemory)?;
            parent.store(frame.as_u64() | PTE_PRESENT | PTE_WRITE, Ordering::Release);
        }
    }
    Ok(())
}

/// Creates an empty user root sharing the kernel half of `kernel_root`.
///
/// # Safety
///
/// `kernel_root` must remain a valid top-level table for the kernel lifetime.
pub unsafe fn create_user_root(kernel_root: PhysAddr) -> Result<PhysAddr> {
    let root = phys::alloc_zeroed_phys(phys::PageUse::PageTable).ok_or(PagingError::OutOfMemory)?;
    // SAFETY: both roots are valid, page-aligned top-level tables and the new
    // root is exclusively owned. `populate_kernel_tables` guarantees every
    // copied entry points at a shared table that outlives this root.
    unsafe {
        for index in 256..512 {
            let entry = table_entry(kernel_root, index).load(Ordering::Acquire);
            table_entry(root, index).store(entry, Ordering::Release);
        }
    }
    Ok(root)
}

/// Destroys a user root while preserving shared kernel-half tables.
///
/// # Safety
///
/// `root` must be inactive and exclusively owned by the caller.
pub unsafe fn destroy_user_root(root: PhysAddr) {
    // SAFETY: the caller guarantees this hierarchy is inactive and exclusive.
    unsafe {
        for index in 0..256 {
            let entry = table_entry(root, index).load(Ordering::Acquire);
            if entry & PTE_PRESENT == 0 {
                continue;
            }
            if entry & PTE_HUGE == 0 {
                destroy_table(PhysAddr::new(entry & ENTRY_ADDR_MASK), 3);
            }
            table_entry(root, index).store(0, Ordering::Release);
        }
        free_table_page(root);
    }
}

/// Translates a virtual address to physical using `root`.
///
/// # Safety
///
/// `root` must refer to a valid page-table hierarchy that remains mapped for
/// the duration of the walk.
pub unsafe fn translate(root: PhysAddr, virt: VirtAddr) -> Option<PhysAddr> {
    let v = virt.as_u64();
    let [idx_l4, idx_l3, idx_l2, idx_l1] = page_table_indexes(v);

    // SAFETY: the caller guarantees that every present table remains mapped
    // for the duration of this read-only walk.
    unsafe {
        let l4e = table_entry(root, idx_l4).load(Ordering::Acquire);
        if l4e & PTE_PRESENT == 0 {
            return None;
        }

        let l3_tbl = PhysAddr::new(l4e & ENTRY_ADDR_MASK);
        let l3e = table_entry(l3_tbl, idx_l3).load(Ordering::Acquire);
        if l3e & PTE_PRESENT == 0 {
            return None;
        }
        if l3e & PTE_HUGE != 0 {
            let mask = (1u64 << 30) - 1;
            let base = (l3e & ENTRY_ADDR_MASK) & !mask;
            return Some(PhysAddr::new(base | (v & mask)));
        }

        let l2_tbl = PhysAddr::new(l3e & ENTRY_ADDR_MASK);
        let l2e = table_entry(l2_tbl, idx_l2).load(Ordering::Acquire);
        if l2e & PTE_PRESENT == 0 {
            return None;
        }
        if l2e & PTE_HUGE != 0 {
            let mask = (1u64 << 21) - 1;
            let base = (l2e & ENTRY_ADDR_MASK) & !mask;
            return Some(PhysAddr::new(base | (v & mask)));
        }

        let l1_tbl = PhysAddr::new(l2e & ENTRY_ADDR_MASK);
        let l1e = table_entry(l1_tbl, idx_l1).load(Ordering::Acquire);
        if l1e & PTE_PRESENT == 0 {
            return None;
        }

        let base = (l1e & ENTRY_ADDR_MASK) & !(PAGE_SIZE - 1);
        Some(PhysAddr::new(base | (v & (PAGE_SIZE - 1))))
    }
}

fn page_table_indexes(virt: u64) -> [usize; 4] {
    [
        ((virt >> 39) & 0x1ff) as usize,
        ((virt >> 30) & 0x1ff) as usize,
        ((virt >> 21) & 0x1ff) as usize,
        ((virt >> 12) & 0x1ff) as usize,
    ]
}

/// Walks an existing hierarchy and returns the leaf entry pointer for `virt`.
///
/// # Safety
///
/// `root` must own a hierarchy that remains mapped for the returned borrow.
unsafe fn walk_to_leaf<'a>(root: PhysAddr, virt: VirtAddr) -> Result<&'a AtomicU64> {
    let [idx_l4, idx_l3, idx_l2, idx_l1] = page_table_indexes(virt.as_u64());
    let mut table = root;
    // SAFETY: the caller guarantees every present table remains mapped.
    unsafe {
        for idx in [idx_l4, idx_l3, idx_l2] {
            let entry = table_entry(table, idx).load(Ordering::Acquire);
            if entry & PTE_PRESENT == 0 {
                return Err(PagingError::NotMapped);
            }
            if entry & PTE_HUGE != 0 {
                return Err(PagingError::HugePageConflict);
            }
            table = PhysAddr::new(entry & ENTRY_ADDR_MASK);
        }
        Ok(table_entry(table, idx_l1))
    }
}

unsafe fn table_entry<'a>(table: PhysAddr, idx: usize) -> &'a AtomicU64 {
    // SAFETY: callers guarantee `table` is a mapped page table and `idx` is a
    // valid index; the table remains mapped for the returned borrow.
    unsafe { &*mem::phys_to_virt(table).as_mut_ptr::<AtomicU64>().add(idx) }
}

fn record_hardware_state(entry: u64) {
    if let Some(page) = phys::phys_to_page(PhysAddr::new(entry & ENTRY_ADDR_MASK)) {
        if entry & PTE_ACCESSED != 0 {
            page.mark_referenced();
        }
        if entry & PTE_DIRTY != 0 {
            page.mark_dirty();
        }
    }
}

fn leaf_bits(flags: VmFlags) -> u64 {
    let mut bits = PTE_PRESENT;
    if flags.contains(VmFlags::WRITE) {
        bits |= PTE_WRITE;
    }
    if flags.contains(VmFlags::USER) {
        bits |= PTE_USER;
    }
    if flags.contains(VmFlags::GLOBAL) {
        bits |= PTE_GLOBAL;
    }
    if flags.contains(VmFlags::WRITE_COMBINE) {
        // PAT entry 4 is programmed to write-combining, and for 4 KiB leaves
        // that entry is selected by the PAT bit alone.
        bits |= PTE_PAT4K;
    } else if flags.contains(VmFlags::DEVICE) {
        bits |= PTE_PWT | PTE_PCD;
    }
    if !flags.contains(VmFlags::EXECUTE) {
        bits |= PTE_NX;
    }
    bits
}

unsafe fn destroy_table(table: PhysAddr, level: usize) {
    // SAFETY: the caller owns the inactive user hierarchy.
    unsafe {
        for index in 0..512 {
            let entry = table_entry(table, index).load(Ordering::Acquire);
            if entry & PTE_PRESENT == 0 {
                continue;
            }
            if level > 1 && entry & PTE_HUGE == 0 {
                destroy_table(PhysAddr::new(entry & ENTRY_ADDR_MASK), level - 1);
            }
            table_entry(table, index).store(0, Ordering::Release);
        }
        free_table_page(table);
    }
}

unsafe fn free_table_page(table: PhysAddr) {
    let page = phys::phys_to_page(table).expect("x86/paging: table page missing PFN metadata");
    // SAFETY: the inactive hierarchy no longer references this table.
    unsafe { phys::free_page(page) };
}
