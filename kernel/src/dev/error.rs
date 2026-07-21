//! Device framework errors.

use core::fmt;

/// Result type used by the device and driver subsystems.
pub type Result<T> = core::result::Result<T, Error>;

/// Recoverable device framework failure.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// The device framework has not been initialized.
    NotInitialized,
    /// A supplied argument is invalid.
    InvalidArgument,
    /// The requested object does not exist.
    NotFound,
    /// An object with the same identity already exists.
    AlreadyExists,
    /// The object has the wrong device-tree kind.
    WrongKind,
    /// The caller does not own the object.
    PermissionDenied,
    /// The object has dependents and cannot be removed.
    Busy,
    /// The driver ABI version or record size is unsupported.
    AbiMismatch,
    /// The requested operation is unsupported.
    Unsupported,
    /// A caller-provided output buffer is too small.
    NoSpace,
    /// A driver callback reported a failure.
    CallbackFailed(i32),
    /// A filesystem operation failed.
    Filesystem,
    /// The kernel could not allocate memory for the operation.
    OutOfMemory,
}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotInitialized => formatter.write_str("device framework not initialized"),
            Self::InvalidArgument => formatter.write_str("invalid argument"),
            Self::NotFound => formatter.write_str("not found"),
            Self::AlreadyExists => formatter.write_str("already exists"),
            Self::WrongKind => formatter.write_str("wrong device-tree node kind"),
            Self::PermissionDenied => formatter.write_str("permission denied"),
            Self::Busy => formatter.write_str("resource busy"),
            Self::AbiMismatch => formatter.write_str("driver ABI mismatch"),
            Self::Unsupported => formatter.write_str("operation unsupported"),
            Self::NoSpace => formatter.write_str("insufficient output space"),
            Self::CallbackFailed(status) => write!(formatter, "driver callback failed ({status})"),
            Self::Filesystem => formatter.write_str("filesystem operation failed"),
            Self::OutOfMemory => formatter.write_str("out of memory"),
        }
    }
}
