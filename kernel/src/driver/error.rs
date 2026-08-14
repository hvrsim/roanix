//! Framework error type and its stable ABI status encoding.

/// Framework result alias.
pub type Result<T> = core::result::Result<T, Error>;

/// Failure reported by a framework operation.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// An argument was malformed, out of range, or violated an invariant.
    InvalidArgument,
    /// The requested object does not exist.
    NotFound,
    /// An object with the same identity already exists.
    AlreadyExists,
    /// The object exists but has the wrong type for this operation.
    WrongKind,
    /// The caller does not own the object or lacks the required rights.
    PermissionDenied,
    /// The object is in use and cannot change state right now.
    Busy,
    /// The operation is not implemented by this provider.
    Unsupported,
    /// A bounded table, identifier space, or address range is exhausted.
    NoSpace,
    /// The kernel heap or physical allocator could not satisfy a request.
    OutOfMemory,
    /// Hardware or transport reported a failure.
    Io,
    /// A non-blocking operation would have blocked.
    WouldBlock,
    /// A blocking operation was interrupted.
    Interrupted,
    /// The operation would create a dependency cycle or lock inversion.
    Deadlock,
    /// A prerequisite is not available yet; retry after the topology changes.
    ///
    /// Returning this from a probe callback parks the device on the deferred
    /// list instead of reporting a hard failure.
    Deferred,
    /// The subsystem has not been initialized.
    NotInitialized,
    /// A bounded wait expired.
    TimedOut,
    /// The device was removed while the operation was in flight.
    NoDevice,
    /// A caller-supplied buffer was too small.
    BufferTooSmall,
    /// The target is not a terminal device.
    NotTty,
    /// The object does not support seeking.
    IllegalSeek,
    /// The filesystem layer rejected an operation.
    Filesystem,
    /// A driver callback returned an unmapped negative status.
    Callback(i32),
}

/// Successful status returned across the C boundary.
pub const STATUS_OK: i32 = 0;

macro_rules! status_table {
    ($($variant:ident => $value:expr),* $(,)?) => {
        impl Error {
            /// Returns the stable negative status for this error.
            pub const fn to_status(self) -> i32 {
                match self {
                    $(Self::$variant => $value,)*
                    Self::Callback(status) => status,
                }
            }

            /// Reconstructs an error from a driver-supplied negative status.
            pub const fn from_status(status: i32) -> Self {
                match status {
                    $($value => Self::$variant,)*
                    other => Self::Callback(other),
                }
            }
        }
    };
}

status_table! {
    InvalidArgument => -1,
    NotFound => -2,
    AlreadyExists => -3,
    WrongKind => -4,
    PermissionDenied => -5,
    Busy => -6,
    Unsupported => -7,
    NoSpace => -8,
    OutOfMemory => -9,
    Io => -10,
    WouldBlock => -11,
    Interrupted => -12,
    Deadlock => -13,
    Deferred => -14,
    NotInitialized => -15,
    TimedOut => -16,
    NoDevice => -17,
    BufferTooSmall => -18,
    NotTty => -19,
    IllegalSeek => -20,
    Filesystem => -21,
}

impl Error {
    /// Returns whether a probe attempt should be retried later.
    pub const fn is_deferred(self) -> bool {
        matches!(self, Self::Deferred)
    }
}

impl From<crate::fs::Error> for Error {
    fn from(error: crate::fs::Error) -> Self {
        match error {
            crate::fs::Error::NotFound => Self::NotFound,
            crate::fs::Error::AlreadyExists => Self::AlreadyExists,
            crate::fs::Error::Busy | crate::fs::Error::NotEmpty => Self::Busy,
            crate::fs::Error::PermissionDenied | crate::fs::Error::ReadOnly => {
                Self::PermissionDenied
            }
            crate::fs::Error::OutOfMemory => Self::OutOfMemory,
            crate::fs::Error::NoSpace => Self::NoSpace,
            crate::fs::Error::InvalidArgument | crate::fs::Error::NameTooLong => {
                Self::InvalidArgument
            }
            crate::fs::Error::WouldBlock => Self::WouldBlock,
            crate::fs::Error::Interrupted => Self::Interrupted,
            crate::fs::Error::NotTty => Self::NotTty,
            crate::fs::Error::IllegalSeek => Self::IllegalSeek,
            crate::fs::Error::Unsupported => Self::Unsupported,
            _ => Self::Filesystem,
        }
    }
}

impl From<Error> for crate::fs::Error {
    fn from(error: Error) -> Self {
        match error {
            Error::InvalidArgument | Error::BufferTooSmall => Self::InvalidArgument,
            Error::NotFound | Error::NoDevice => Self::NotFound,
            Error::AlreadyExists => Self::AlreadyExists,
            Error::PermissionDenied => Self::PermissionDenied,
            Error::Busy | Error::Deferred => Self::Busy,
            Error::Unsupported | Error::WrongKind => Self::Unsupported,
            Error::NoSpace => Self::NoSpace,
            Error::OutOfMemory => Self::OutOfMemory,
            Error::WouldBlock => Self::WouldBlock,
            Error::Interrupted => Self::Interrupted,
            Error::NotTty => Self::NotTty,
            Error::IllegalSeek => Self::IllegalSeek,
            _ => Self::Io,
        }
    }
}

/// Converts a framework result into the ABI status convention.
pub fn to_status(result: Result<()>) -> i32 {
    match result {
        Ok(()) => STATUS_OK,
        Err(error) => error.to_status(),
    }
}

/// Converts a driver-reported status into a framework result.
pub fn from_status(status: i32) -> Result<()> {
    if status >= 0 {
        Ok(())
    } else {
        Err(Error::from_status(status))
    }
}
