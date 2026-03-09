//!
//! # Virtual Memory Paging
//!
//! x86_64 paging backend for the [`mem`] subsystem.
//!

use x86_64::{
    instructions::tlb, registers::control::Cr3, structures::paging::PhysFrame,
    PhysAddr as X86PhysAddr, VirtAddr as X86VirtAddr,
};

use crate::mem::{self, phys, PhysAddr, VirtAddr, VmFlags, PAGE_SIZE};

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
/// Entry maps a huge page.
const PTE_HUGE: u64 = 1 << 7;
/// Global TLB entry.
const PTE_GLOBAL: u64 = 1 << 8;
/// Execute-disable bit.
const PTE_NX: u64 = 1 << 63;

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
pub unsafe fn activate_root(root: PhysAddr) -> Result<()> {
    if !root.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let frame = PhysFrame::from_start_address(X86PhysAddr::new(root.as_u64()))
        .map_err(|_| PagingError::InvalidAddress)?;
    let (_, flags) = Cr3::read();
    Cr3::write(frame, flags);
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

    let v = virt.as_u64();
    let idx_l4 = ((v >> 39) & 0x1ff) as usize;
    let idx_l3 = ((v >> 30) & 0x1ff) as usize;
    let idx_l2 = ((v >> 21) & 0x1ff) as usize;
    let idx_l1 = ((v >> 12) & 0x1ff) as usize;
    let mut table = root;
    let user = flags.contains(VmFlags::USER);

    for idx in [idx_l4, idx_l3, idx_l2] {
        let entry_ptr = mem::phys_to_virt(table).as_mut_ptr::<u64>().add(idx);
        let entry = entry_ptr.read_volatile();

        if entry & PTE_PRESENT != 0 {
            if entry & PTE_HUGE != 0 {
                return Err(PagingError::HugePageConflict);
            }
            table = PhysAddr::new(entry & ENTRY_ADDR_MASK);
            continue;
        }

        let frame = phys::alloc_zeroed_phys().ok_or(PagingError::OutOfMemory)?;
        let mut bits = PTE_PRESENT | PTE_WRITE;
        if user {
            bits |= PTE_USER;
        }
        entry_ptr.write_volatile(frame.as_u64() | bits);
        table = frame;
    }

    let leaf_ptr = mem::phys_to_virt(table).as_mut_ptr::<u64>().add(idx_l1);
    if leaf_ptr.read_volatile() & PTE_PRESENT != 0 {
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
    if flags.contains(VmFlags::DEVICE) {
        bits |= PTE_PWT | PTE_PCD;
    }
    if !flags.contains(VmFlags::EXECUTE) {
        bits |= PTE_NX;
    }

    leaf_ptr.write_volatile((phys.as_u64() & ENTRY_ADDR_MASK) | bits);
    tlb::flush(X86VirtAddr::new(v));
    Ok(())
}

/// Unmaps one 4 KiB page and returns the removed physical address.
pub unsafe fn unmap_page(root: PhysAddr, virt: VirtAddr) -> Result<Option<PhysAddr>> {
    if !root.is_page_aligned() || !virt.is_page_aligned() {
        return Err(PagingError::UnalignedAddress);
    }

    let v = virt.as_u64();
    let idx_l4 = ((v >> 39) & 0x1ff) as usize;
    let idx_l3 = ((v >> 30) & 0x1ff) as usize;
    let idx_l2 = ((v >> 21) & 0x1ff) as usize;
    let idx_l1 = ((v >> 12) & 0x1ff) as usize;
    let mut table = root;

    for idx in [idx_l4, idx_l3, idx_l2] {
        let entry = mem::phys_to_virt(table)
            .as_mut_ptr::<u64>()
            .add(idx)
            .read_volatile();
        if entry & PTE_PRESENT == 0 || entry & PTE_HUGE != 0 {
            return Err(PagingError::NotMapped);
        }
        table = PhysAddr::new(entry & ENTRY_ADDR_MASK);
    }

    let leaf_ptr = mem::phys_to_virt(table).as_mut_ptr::<u64>().add(idx_l1);
    let leaf = leaf_ptr.read_volatile();
    if leaf & PTE_PRESENT == 0 {
        return Err(PagingError::NotMapped);
    }

    leaf_ptr.write_volatile(0);
    tlb::flush(X86VirtAddr::new(v));
    Ok(Some(PhysAddr::new(leaf & ENTRY_ADDR_MASK)))
}

/// Translates a virtual address to physical using `root`.
pub unsafe fn translate(root: PhysAddr, virt: VirtAddr) -> Option<PhysAddr> {
    let v = virt.as_u64();
    let idx_l4 = ((v >> 39) & 0x1ff) as usize;
    let idx_l3 = ((v >> 30) & 0x1ff) as usize;
    let idx_l2 = ((v >> 21) & 0x1ff) as usize;
    let idx_l1 = ((v >> 12) & 0x1ff) as usize;

    let l4e = mem::phys_to_virt(root)
        .as_mut_ptr::<u64>()
        .add(idx_l4)
        .read_volatile();
    if l4e & PTE_PRESENT == 0 {
        return None;
    }

    let l3_tbl = PhysAddr::new(l4e & ENTRY_ADDR_MASK);
    let l3e = mem::phys_to_virt(l3_tbl)
        .as_mut_ptr::<u64>()
        .add(idx_l3)
        .read_volatile();
    if l3e & PTE_PRESENT == 0 {
        return None;
    }
    if l3e & PTE_HUGE != 0 {
        let mask = (1u64 << 30) - 1;
        let base = (l3e & ENTRY_ADDR_MASK) & !mask;
        return Some(PhysAddr::new(base | (v & mask)));
    }

    let l2_tbl = PhysAddr::new(l3e & ENTRY_ADDR_MASK);
    let l2e = mem::phys_to_virt(l2_tbl)
        .as_mut_ptr::<u64>()
        .add(idx_l2)
        .read_volatile();
    if l2e & PTE_PRESENT == 0 {
        return None;
    }
    if l2e & PTE_HUGE != 0 {
        let mask = (1u64 << 21) - 1;
        let base = (l2e & ENTRY_ADDR_MASK) & !mask;
        return Some(PhysAddr::new(base | (v & mask)));
    }

    let l1_tbl = PhysAddr::new(l2e & ENTRY_ADDR_MASK);
    let l1e = mem::phys_to_virt(l1_tbl)
        .as_mut_ptr::<u64>()
        .add(idx_l1)
        .read_volatile();
    if l1e & PTE_PRESENT == 0 {
        return None;
    }

    let base = (l1e & ENTRY_ADDR_MASK) & !(PAGE_SIZE - 1);
    Some(PhysAddr::new(base | (v & (PAGE_SIZE - 1))))
}
