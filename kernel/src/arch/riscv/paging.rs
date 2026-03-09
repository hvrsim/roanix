//!
//! # Virtual Memory Paging
//!
//! riscv64 paging backend for the [`mem`] subsystem.
//!

use core::arch::asm;

use crate::mem::{self, phys, PhysAddr, VirtAddr, VmFlags};

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
pub unsafe fn activate_root(root: PhysAddr) -> Result<()> {
    if !root.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let satp = read_satp();

    let mode = satp >> SATP_MODE_SHIFT;
    let asid = (satp >> SATP_ASID_SHIFT) & 0xffff;
    let ppn = root.as_u64() >> 12;
    if ppn > SATP_PPN_MASK {
        return Err(PagingError::InvalidAddress);
    }

    write_satp((mode << SATP_MODE_SHIFT) | (asid << SATP_ASID_SHIFT) | ppn);
    sfence_vma(None);
    Ok(())
}

/// Maps one 4 KiB page.
pub unsafe fn map_page(
    root: PhysAddr,
    virt: VirtAddr,
    phys: PhysAddr,
    flags: VmFlags,
) -> Result<()> {
    if !root.is_page_aligned() || !virt.is_page_aligned() || !phys.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let satp = read_satp();
    let levels = match satp >> SATP_MODE_SHIFT {
        8 => 3,  // SV39
        9 => 4,  // SV48
        10 => 5, // SV57
        _ => 3,
    };

    let v = virt.as_u64();
    let mut table = root;

    for level in (1..levels).rev() {
        let idx = ((v >> (12 + level * 9)) & 0x1ff) as usize;
        debug_assert!(idx < ENTRIES_PER_TABLE);
        let entry_ptr = mem::phys_to_virt(table).as_mut_ptr::<u64>().add(idx);
        let pte = entry_ptr.read_volatile();

        if pte & PTE_V != 0 {
            if pte & (PTE_R | PTE_W | PTE_X) != 0 {
                return Err(PagingError::HugePageConflict);
            }
            table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
            continue;
        }

        let frame = phys::alloc_zeroed_phys().ok_or(PagingError::OutOfMemory)?;
        entry_ptr.write_volatile(((frame.as_u64() >> 12) << 10) | PTE_V);
        table = frame;
    }

    let leaf_idx = ((v >> 12) & 0x1ff) as usize;
    debug_assert!(leaf_idx < ENTRIES_PER_TABLE);
    let leaf_ptr = mem::phys_to_virt(table).as_mut_ptr::<u64>().add(leaf_idx);
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

/// Unmaps one 4 KiB page and returns the removed physical address.
pub unsafe fn unmap_page(root: PhysAddr, virt: VirtAddr) -> Result<Option<PhysAddr>> {
    if !root.is_page_aligned() || !virt.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let satp = read_satp();
    let levels = match satp >> SATP_MODE_SHIFT {
        8 => 3,  // SV39
        9 => 4,  // SV48
        10 => 5, // SV57
        _ => 3,
    };

    let v = virt.as_u64();
    let mut table = root;

    for level in (1..levels).rev() {
        let idx = ((v >> (12 + level * 9)) & 0x1ff) as usize;
        debug_assert!(idx < ENTRIES_PER_TABLE);
        let pte = mem::phys_to_virt(table)
            .as_mut_ptr::<u64>()
            .add(idx)
            .read_volatile();

        if pte & PTE_V == 0 {
            return Err(PagingError::NotMapped);
        }
        if pte & (PTE_R | PTE_W | PTE_X) != 0 {
            return Err(PagingError::HugePageConflict);
        }

        table = PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12);
    }

    let leaf_idx = ((v >> 12) & 0x1ff) as usize;
    debug_assert!(leaf_idx < ENTRIES_PER_TABLE);
    let leaf_ptr = mem::phys_to_virt(table).as_mut_ptr::<u64>().add(leaf_idx);
    let pte = leaf_ptr.read_volatile();
    if pte & PTE_V == 0 || pte & (PTE_R | PTE_W | PTE_X) == 0 {
        return Err(PagingError::NotMapped);
    }

    leaf_ptr.write_volatile(0);
    sfence_vma(Some(virt));
    Ok(Some(PhysAddr::new(((pte >> 10) & SATP_PPN_MASK) << 12)))
}

/// Translates a virtual address to physical using `root`.
pub unsafe fn translate(root: PhysAddr, virt: VirtAddr) -> Option<PhysAddr> {
    let satp = read_satp();
    let levels = match satp >> SATP_MODE_SHIFT {
        8 => 3,  // SV39
        9 => 4,  // SV48
        10 => 5, // SV57
        _ => 3,
    };

    let v = virt.as_u64();
    let mut table = root;

    for level in (0..levels).rev() {
        let idx = ((v >> (12 + level * 9)) & 0x1ff) as usize;
        debug_assert!(idx < ENTRIES_PER_TABLE);
        let pte = mem::phys_to_virt(table)
            .as_mut_ptr::<u64>()
            .add(idx)
            .read_volatile();
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

/// Reads and returns the current `satp` CSR value.
#[inline(always)]
fn read_satp() -> u64 {
    let satp: u64;
    unsafe {
        asm!(
            "csrr {}, satp",
            out(reg) satp,
            options(nomem, nostack, preserves_flags)
        );
    }
    satp
}

/// Writes a new value to the `satp` CSR.
#[inline(always)]
unsafe fn write_satp(value: u64) {
    asm!(
        "csrw satp, {}",
        in(reg) value,
        options(nomem, nostack, preserves_flags)
    );
}

/// Executes `sfence.vma` globally or for one virtual address when provided.
#[inline(always)]
fn sfence_vma(addr: Option<VirtAddr>) {
    match addr {
        Some(virt) => unsafe {
            asm!(
                "sfence.vma {addr}, x0",
                addr = in(reg) virt.as_u64(),
                options(nomem, nostack, preserves_flags)
            );
        },
        None => unsafe {
            asm!(
                "sfence.vma x0, x0",
                options(nomem, nostack, preserves_flags)
            );
        },
    }
}
