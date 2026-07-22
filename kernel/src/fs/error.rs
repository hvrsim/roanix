//! Filesystem error values shared by VFS backends.

use core::fmt;

/// Result type used by the VFS and filesystem implementations.
pub type Result<T> = core::result::Result<T, Error>;

/// Filesystem-independent operation failure.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// A path component or node does not exist.
    NotFound,
    /// A directory entry already exists.
    AlreadyExists,
    /// An operation required a directory.
    NotDirectory,
    /// The open object does not support seeking.
    IllegalSeek,
    /// An operation is invalid for a directory.
    IsDirectory,
    /// The target is not a terminal device.
    NotTty,
    /// A blocking operation was interrupted by terminal input.
    Interrupted,
    /// A directory still contains entries.
    NotEmpty,
    /// The operation crosses filesystem boundaries.
    CrossDevice,
    /// Too many symbolic links were followed.
    SymlinkLoop,
    /// A path or component exceeds a VFS limit.
    NameTooLong,
    /// The path or operation argument is invalid.
    InvalidArgument,
    /// The operation is not supported by this filesystem.
    Unsupported,
    /// The filesystem or mount is busy.
    Busy,
    /// A nonblocking operation would have to wait.
    WouldBlock,
    /// The filesystem has no remaining capacity.
    NoSpace,
    /// A file would exceed the implementation limit.
    FileTooLarge,
    /// The requested access mode is not permitted.
    PermissionDenied,
    /// The filesystem is read-only.
    ReadOnly,
    /// An open file does not permit the requested operation.
    BadFileDescriptor,
    /// The kernel could not allocate memory for the operation.
    OutOfMemory,
    /// A low-level I/O operation failed.
    Io,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NotFound => "not found",
            Self::AlreadyExists => "already exists",
            Self::NotDirectory => "not a directory",
            Self::IllegalSeek => "illegal seek",
            Self::IsDirectory => "is a directory",
            Self::NotTty => "not a terminal",
            Self::Interrupted => "operation interrupted",
            Self::NotEmpty => "directory not empty",
            Self::CrossDevice => "cross-device operation",
            Self::SymlinkLoop => "too many symbolic links",
            Self::NameTooLong => "name too long",
            Self::InvalidArgument => "invalid argument",
            Self::Unsupported => "operation not supported",
            Self::Busy => "resource busy",
            Self::WouldBlock => "operation would block",
            Self::NoSpace => "no space left",
            Self::FileTooLarge => "file too large",
            Self::PermissionDenied => "permission denied",
            Self::ReadOnly => "read-only filesystem",
            Self::BadFileDescriptor => "bad file descriptor",
            Self::OutOfMemory => "out of memory",
            Self::Io => "I/O error",
        })
    }
}
