//!
//! # Address Types
//!
//! Small typed wrappers and alignment helpers used throughout the memory
//! subsystem.
//!

use core::{fmt, ptr};

/// Log2 of page size.
pub const PAGE_SHIFT: u64 = 12;

/// Base page size in bytes.
pub const PAGE_SIZE: u64 = 1 << PAGE_SHIFT;

/// Align `value` down to `align` bytes.
#[must_use]
#[inline(always)]
pub const fn align_down(value: u64, align: u64) -> u64 {
    assert!(align.is_power_of_two(), "alignment must be a power of two");
    value & !(align - 1)
}

/// Align `value` up to `align` bytes.
#[must_use]
#[inline(always)]
pub const fn align_up(value: u64, align: u64) -> u64 {
    assert!(align.is_power_of_two(), "alignment must be a power of two");
    let mask = align - 1;
    match value.checked_add(mask) {
        Some(v) => v & !mask,
        None => panic!("alignment overflow"),
    }
}

/// Returns how many pages are needed to cover `len` bytes.
#[must_use]
#[inline(always)]
pub const fn pages_for_len(len: u64) -> u64 {
    if len == 0 {
        0
    } else {
        align_up(len, PAGE_SIZE) / PAGE_SIZE
    }
}

/// Physical address wrapper.
#[must_use]
#[repr(transparent)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct PhysAddr(u64);

impl PhysAddr {
    /// Creates a physical address from raw `u64`.
    pub const fn new(raw: u64) -> Self {
        Self(raw)
    }

    /// Zero address.
    pub const fn zero() -> Self {
        Self(0)
    }

    /// Returns raw address value.
    pub const fn as_u64(self) -> u64 {
        self.0
    }

    /// Returns whether this address is aligned to base page size.
    pub const fn is_page_aligned(self) -> bool {
        (self.0 & (PAGE_SIZE - 1)) == 0
    }

    /// Align this address down to base page size.
    pub const fn align_down(self) -> Self {
        Self(align_down(self.0, PAGE_SIZE))
    }

    /// Align this address up to base page size.
    pub const fn align_up(self) -> Self {
        Self(align_up(self.0, PAGE_SIZE))
    }

    /// Adds `bytes` and returns `None` on overflow.
    pub const fn checked_add(self, bytes: u64) -> Option<Self> {
        match self.0.checked_add(bytes) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }
}

impl fmt::LowerHex for PhysAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.0, f)
    }
}

/// Virtual address wrapper.
#[must_use]
#[repr(transparent)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct VirtAddr(u64);

impl VirtAddr {
    /// Creates a virtual address from raw `u64`.
    pub const fn new(raw: u64) -> Self {
        Self(raw)
    }

    /// Captures the exposed address of `ptr`.
    ///
    /// This intentionally discards pointer provenance. It is intended for
    /// architecture registers, page tables, and HHDM/MMIO address arithmetic.
    pub fn from_ptr<T>(ptr: *const T) -> Self {
        Self(ptr.addr() as u64)
    }

    /// Zero address.
    pub const fn zero() -> Self {
        Self(0)
    }

    /// Returns raw address value.
    pub const fn as_u64(self) -> u64 {
        self.0
    }

    /// Returns whether this address is aligned to base page size.
    pub const fn is_page_aligned(self) -> bool {
        (self.0 & (PAGE_SIZE - 1)) == 0
    }

    /// Align this address down to base page size.
    pub const fn align_down(self) -> Self {
        Self(align_down(self.0, PAGE_SIZE))
    }

    /// Align this address up to base page size.
    pub const fn align_up(self) -> Self {
        Self(align_up(self.0, PAGE_SIZE))
    }

    /// Adds `bytes` and returns `None` on overflow.
    pub const fn checked_add(self, bytes: u64) -> Option<Self> {
        match self.0.checked_add(bytes) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Reconstructs an immutable pointer with exposed provenance.
    ///
    /// Dereferencing the result additionally requires a valid mapping,
    /// alignment for `T`, initialization, and Rust aliasing guarantees.
    pub const fn as_ptr<T>(self) -> *const T {
        ptr::with_exposed_provenance(self.0 as usize)
    }

    /// Reconstructs a mutable pointer with exposed provenance.
    ///
    /// Dereferencing the result additionally requires exclusive access, a
    /// valid mapping, alignment for `T`, and initialization.
    pub const fn as_mut_ptr<T>(self) -> *mut T {
        ptr::with_exposed_provenance_mut(self.0 as usize)
    }
}

impl fmt::LowerHex for VirtAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.0, f)
    }
}
