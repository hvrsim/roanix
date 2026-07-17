//! Memory subsystem operation errors.

use core::fmt;

/// Result type used by the virtual memory subsystem.
pub type Result<T> = core::result::Result<T, Error>;

/// Virtual memory operation failure.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// A required physical page could not be allocated.
    OutOfMemory,
    /// A virtual address or range is invalid.
    InvalidAddress,
    /// A requested range overlaps an existing mapping.
    AlreadyMapped,
    /// No mapping contains the requested address.
    NotMapped,
    /// The requested access violates mapping permissions.
    Protection,
    /// A requested operation exceeds an object or account limit.
    LimitExceeded,
    /// The page could not be represented by the available swap tiers.
    SwapUnavailable,
    /// Swapped page data failed validation or decompression.
    CorruptSwap,
    /// An architecture page-table operation failed.
    Pmap,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::OutOfMemory => "out of memory",
            Self::InvalidAddress => "invalid virtual address",
            Self::AlreadyMapped => "virtual range already mapped",
            Self::NotMapped => "virtual address is not mapped",
            Self::Protection => "virtual memory protection failure",
            Self::LimitExceeded => "virtual memory account limit exceeded",
            Self::SwapUnavailable => "no swap tier accepted the page",
            Self::CorruptSwap => "corrupt swapped page",
            Self::Pmap => "architecture page-table operation failed",
        })
    }
}
