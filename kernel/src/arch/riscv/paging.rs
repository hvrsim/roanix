//!
//! # Virtual Memory Paging
//!
//! riscv64 paging backend for the [`mem`] subsystem.
//!

use core::{
    arch::asm,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::mem::{self, PhysAddr, VirtAddr, VmFlags, phys};

/// Number of entries in one RISC-V page table.
const ENTRIES_PER_TABLE: usize = 512;
/// SATP mode field bit shift.
const SATP_MODE_SHIFT: u64 = 60;
/// SATP ASID field bit shift.
const SATP_ASID_SHIFT: u64 = 44;
/// SATP physical page number mask.
const SATP_PPN_MASK: u64 = (1u64 << 44) - 1;

/// Valid entry.
const PTE_V: u64 = 1 << 0;
/// Read permission.
const PTE_R: u64 = 1 << 1;
/// Write permission.
const PTE_W: u64 = 1 << 2;
/// Execute permission.
const PTE_X: u64 = 1 << 3;
/// User permission.
const PTE_U: u64 = 1 << 4;
/// Global mapping flag.
const PTE_G: u64 = 1 << 5;
/// Accessed flag.
const PTE_A: u64 = 1 << 6;
/// Dirty flag.
const PTE_D: u64 = 1 << 7;

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
    /// Encountered a leaf mapping where a table was required.
    HugePageConflict,
}

/// Architecture paging result type.
pub type Result<T> = core::result::Result<T, PagingError>;

/// Returns active page table root physical address.
pub fn active_root() -> PhysAddr {
    let satp = read_satp();
    PhysAddr::new((satp & SATP_PPN_MASK) << 12)
}

/// Activates the given page-table root while preserving SATP mode/ASID.
///
/// # Safety
///
/// `root` must refer to a valid, currently mapped top-level RISC-V page table
/// whose contents are suitable for immediate execution on the current hart.
pub unsafe fn activate_root(root: PhysAddr) -> Result<()> {
    if !root.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let satp = read_satp();
    let mode = satp_mode(satp);
    let asid = (satp >> SATP_ASID_SHIFT) & 0xffff;
    let ppn = root.as_u64() >> 12;
    if ppn > SATP_PPN_MASK {
        return Err(PagingError::InvalidAddress);
    }

    // SAFETY: the caller guarantees the newly encoded root is valid.
    unsafe { write_satp((mode << SATP_MODE_SHIFT) | (asid << SATP_ASID_SHIFT) | ppn) };
    sfence_vma(None);
    Ok(())
}

/// Maps one 4 KiB page.
///
/// # Safety
///
/// `root` must point to a valid writable page-table hierarchy owned by the
/// caller. The caller must ensure the mapping change is synchronized against
/// other harts and that `phys` is safe to expose at `virt` with `flags`.
pub unsafe fn map_page(
    root: PhysAddr,
    virt: VirtAddr,
    phys: PhysAddr,
    flags: VmFlags,
) -> Result<()> {
    if !root.is_page_aligned() || !virt.is_page_aligned() || !phys.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let levels = paging_levels(read_satp());

    let v = virt.as_u64();
    let mut table = root;

    // SAFETY: the caller guarantees the hierarchy is valid and writable for
    // the duration of this walk.
    unsafe {
        for level in (1..levels).rev() {
            let idx = table_index(v, level);
            debug_assert!(idx < ENTRIES_PER_TABLE);
            let entry_ptr = pte_ptr(table, idx);
            let pte = entry_ptr.read_volatile();

            if pte & PTE_V != 0 {
                if pte & (PTE_R | PTE_W | PTE_X) != 0 {
                    return Err(PagingError::HugePageConflict);
                }
                table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
                continue;
            }

            let frame = phys::alloc_zeroed_phys(phys::PageUse::PageTable)
                .ok_or(PagingError::OutOfMemory)?;
            entry_ptr.write_volatile(((frame.as_u64() >> 12) << 10) | PTE_V);
            table = frame;
        }

        let leaf_idx = table_index(v, 0);
        debug_assert!(leaf_idx < ENTRIES_PER_TABLE);
        let leaf_ptr = pte_ptr(table, leaf_idx);
        if leaf_ptr.read_volatile() & PTE_V != 0 {
            return Err(PagingError::AlreadyMapped);
        }

        let mut norm = flags;
        if !norm.intersects(VmFlags::READ | VmFlags::WRITE | VmFlags::EXECUTE) {
            norm |= VmFlags::READ;
        }
        if norm.contains(VmFlags::WRITE) {
            norm |= VmFlags::READ;
        }

        let mut bits = PTE_V | PTE_A;
        if norm.contains(VmFlags::READ) {
            bits |= PTE_R;
        }
        if norm.contains(VmFlags::WRITE) {
            bits |= PTE_W | PTE_D;
        }
        if norm.contains(VmFlags::EXECUTE) {
            bits |= PTE_X;
        }
        if norm.contains(VmFlags::USER) {
            bits |= PTE_U;
        }
        if norm.contains(VmFlags::GLOBAL) {
            bits |= PTE_G;
        }

        leaf_ptr.write_volatile(((phys.as_u64() >> 12) << 10) | bits);
        sfence_vma(Some(virt));
        Ok(())
    }
}

/// Replaces one existing 4 KiB mapping.
///
/// # Safety
///
/// The caller must own and synchronize the hierarchy and guarantee that
/// `phys` remains valid for the resulting mapping.
pub unsafe fn remap_page(
    root: PhysAddr,
    virt: VirtAddr,
    phys: PhysAddr,
    flags: VmFlags,
) -> Result<()> {
    if !root.is_page_aligned() || !virt.is_page_aligned() || !phys.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }
    let levels = paging_levels(read_satp());
    let mut table = root;
    // SAFETY: the caller guarantees exclusive page-table mutation.
    unsafe {
        for level in (1..levels).rev() {
            let pte = pte_ptr(table, table_index(virt.as_u64(), level)).read_volatile();
            if pte & PTE_V == 0 {
                return Err(PagingError::NotMapped);
            }
            if pte & (PTE_R | PTE_W | PTE_X) != 0 {
                return Err(PagingError::HugePageConflict);
            }
            table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
        }
        let leaf = pte_ptr(table, table_index(virt.as_u64(), 0));
        if leaf.read_volatile() & PTE_V == 0 {
            return Err(PagingError::NotMapped);
        }
        leaf.write_volatile(((phys.as_u64() >> 12) << 10) | leaf_bits(flags));
    }
    sfence_vma(Some(virt));
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
    let levels = paging_levels(read_satp());
    let mut table = root;
    // SAFETY: the caller guarantees a stable, writable hierarchy.
    unsafe {
        for level in (1..levels).rev() {
            let pte = pte_ptr(table, table_index(virt.as_u64(), level)).read_volatile();
            if pte & PTE_V == 0 {
                return Err(PagingError::NotMapped);
            }
            if pte & (PTE_R | PTE_W | PTE_X) != 0 {
                return Err(PagingError::HugePageConflict);
            }
            table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
        }
        let leaf_ptr = pte_ptr(table, table_index(virt.as_u64(), 0));
        let leaf = &*leaf_ptr.cast::<AtomicU64>();
        let value = leaf.load(Ordering::Acquire);
        if value & PTE_V == 0 || value & (PTE_R | PTE_W | PTE_X) == 0 {
            return Err(PagingError::NotMapped);
        }
        let previous = if value & (PTE_A | PTE_D) != 0 {
            leaf.fetch_and(!(PTE_A | PTE_D), Ordering::AcqRel)
        } else {
            value
        };
        let accessed = previous & PTE_A != 0;
        let dirty = previous & PTE_D != 0;
        if accessed || dirty {
            sfence_vma(Some(virt));
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

    let levels = paging_levels(read_satp());

    let v = virt.as_u64();
    let mut table = root;

    // SAFETY: the caller guarantees the hierarchy remains valid and
    // exclusively writable during the removal.
    unsafe {
        for level in (1..levels).rev() {
            let idx = table_index(v, level);
            debug_assert!(idx < ENTRIES_PER_TABLE);
            let pte = pte_ptr(table, idx).read_volatile();

            if pte & PTE_V == 0 {
                return Err(PagingError::NotMapped);
            }
            if pte & (PTE_R | PTE_W | PTE_X) != 0 {
                return Err(PagingError::HugePageConflict);
            }

            table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
        }

        let leaf_idx = table_index(v, 0);
        debug_assert!(leaf_idx < ENTRIES_PER_TABLE);
        let leaf_ptr = pte_ptr(table, leaf_idx);
        let pte = leaf_ptr.read_volatile();
        if pte & PTE_V == 0 || pte & (PTE_R | PTE_W | PTE_X) == 0 {
            return Err(PagingError::NotMapped);
        }

        leaf_ptr.write_volatile(0);
        sfence_vma(Some(virt));
        Ok(Some(PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12)))
    }
}

/// Flushes the current hart's TLB entry for `virt`.
pub fn flush_page(virt: VirtAddr) {
    sfence_vma(Some(virt));
}

/// Flushes every translation on the current hart.
pub fn flush_all() {
    sfence_vma(None);
}

/// Creates an empty user root sharing the kernel half of `kernel_root`.
///
/// # Safety
///
/// `kernel_root` must remain valid for the kernel lifetime.
pub unsafe fn create_user_root(kernel_root: PhysAddr) -> Result<PhysAddr> {
    let root = phys::alloc_zeroed_phys(phys::PageUse::PageTable).ok_or(PagingError::OutOfMemory)?;
    // SAFETY: both roots are valid top-level tables and the new root is
    // exclusively owned.
    unsafe {
        for index in 256..ENTRIES_PER_TABLE {
            let entry = pte_ptr(kernel_root, index).read_volatile();
            pte_ptr(root, index).write_volatile(entry);
        }
    }
    Ok(root)
}

/// Destroys a user root while preserving shared kernel-half tables.
///
/// # Safety
///
/// `root` must be inactive and exclusively owned.
pub unsafe fn destroy_user_root(root: PhysAddr) {
    let levels = paging_levels(read_satp());
    // SAFETY: the caller owns this inactive hierarchy.
    unsafe {
        for index in 0..256 {
            let pte = pte_ptr(root, index).read_volatile();
            if pte & PTE_V == 0 {
                continue;
            }
            if pte & (PTE_R | PTE_W | PTE_X) == 0 {
                destroy_table(
                    PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12),
                    levels - 1,
                );
            }
            pte_ptr(root, index).write_volatile(0);
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
    let levels = paging_levels(read_satp());

    let v = virt.as_u64();
    let mut table = root;

    // SAFETY: the caller guarantees that every present table remains mapped
    // for the duration of this read-only walk.
    unsafe {
        for level in (0..levels).rev() {
            let idx = table_index(v, level);
            debug_assert!(idx < ENTRIES_PER_TABLE);
            let pte = pte_ptr(table, idx).read_volatile();
            if pte & PTE_V == 0 {
                return None;
            }

            if pte & (PTE_R | PTE_W | PTE_X) != 0 {
                let shift = 12 + (level as u64) * 9;
                let mask = (1u64 << shift) - 1;
                let base = (PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12).as_u64()) & !mask;
                return Some(PhysAddr::new(base | (v & mask)));
            }

            if level == 0 {
                return None;
            }

            table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
        }

        None
    }
}

/// Reads and returns the current `satp` CSR value.
#[inline(always)]
fn read_satp() -> u64 {
    let satp: u64;
    // SAFETY: reading SATP is valid in supervisor mode and only snapshots the
    // current address-space configuration.
    unsafe {
        asm!(
            "csrr {}, satp",
            out(reg) satp,
            options(nomem, nostack, preserves_flags)
        );
    }
    satp
}

fn satp_mode(satp: u64) -> u64 {
    satp >> SATP_MODE_SHIFT
}

fn paging_levels(satp: u64) -> usize {
    match satp_mode(satp) {
        8 => 3,  // SV39
        9 => 4,  // SV48
        10 => 5, // SV57
        _ => 3,
    }
}

fn table_index(virt: u64, level: usize) -> usize {
    ((virt >> (12 + level * 9)) & 0x1ff) as usize
}

unsafe fn pte_ptr(table: PhysAddr, idx: usize) -> *mut u64 {
    // SAFETY: callers guarantee `table` is mapped and `idx` is a valid
    // page-table index.
    unsafe { mem::phys_to_virt(table).as_mut_ptr::<u64>().add(idx) }
}

fn leaf_bits(flags: VmFlags) -> u64 {
    let mut norm = flags;
    if !norm.intersects(VmFlags::READ | VmFlags::WRITE | VmFlags::EXECUTE) {
        norm |= VmFlags::READ;
    }
    if norm.contains(VmFlags::WRITE) {
        norm |= VmFlags::READ;
    }
    let mut bits = PTE_V | PTE_A;
    if norm.contains(VmFlags::READ) {
        bits |= PTE_R;
    }
    if norm.contains(VmFlags::WRITE) {
        bits |= PTE_W | PTE_D;
    }
    if norm.contains(VmFlags::EXECUTE) {
        bits |= PTE_X;
    }
    if norm.contains(VmFlags::USER) {
        bits |= PTE_U;
    }
    if norm.contains(VmFlags::GLOBAL) {
        bits |= PTE_G;
    }
    bits
}

unsafe fn destroy_table(table: PhysAddr, level: usize) {
    // SAFETY: the caller owns this inactive user hierarchy.
    unsafe {
        for index in 0..ENTRIES_PER_TABLE {
            let pte = pte_ptr(table, index).read_volatile();
            if pte & PTE_V == 0 {
                continue;
            }
            if level > 1 && pte & (PTE_R | PTE_W | PTE_X) == 0 {
                destroy_table(
                    PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12),
                    level - 1,
                );
            }
            pte_ptr(table, index).write_volatile(0);
        }
        free_table_page(table);
    }
}

unsafe fn free_table_page(table: PhysAddr) {
    let page = phys::phys_to_page(table).expect("riscv/paging: table page missing PFN metadata");
    // SAFETY: the inactive hierarchy no longer references this table.
    unsafe { phys::free_page(page) };
}

/// Writes a new value to the `satp` CSR.
#[inline(always)]
unsafe fn write_satp(value: u64) {
    // SAFETY: the caller guarantees `value` encodes a valid SATP state.
    unsafe {
        asm!(
            "csrw satp, {}",
            in(reg) value,
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// Executes `sfence.vma` globally or for one virtual address when provided.
#[inline(always)]
fn sfence_vma(addr: Option<VirtAddr>) {
    match addr {
        Some(virt) => {
            // SAFETY: invalidating the current hart's translation for this
            // virtual address has no memory-safety preconditions.
            unsafe {
                asm!(
                    "sfence.vma {addr}, x0",
                    addr = in(reg) virt.as_u64(),
                    options(nomem, nostack, preserves_flags)
                );
            }
        }
        None => {
            // SAFETY: a global local-hart TLB invalidation is always valid.
            unsafe {
                asm!(
                    "sfence.vma x0, x0",
                    options(nomem, nostack, preserves_flags)
                );
            }
        }
    }
}
