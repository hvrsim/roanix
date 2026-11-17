//! Open file descriptions and shared seek offsets.

use crate::sys::event::EventSource;

use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU32, Ordering};

use bitflags::bitflags;

use crate::{
    mem::{IoSink, IoSource},
    sys::sync::Mutex,
};

use super::{
    error::{Error, Result},
    vfs::PathAnchor,
    vnode::{DirEntry, IoctlContext, PollEvents, TerminalState, Vnode, VnodeAttr, VnodeKind},
};

bitflags! {
    /// Open-file access and creation behavior.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct OpenFlags: u32 {
        /// Permit reads.
        const READ      = 1 << 0;
        /// Permit writes.
        const WRITE     = 1 << 1;
        /// Create the final component when it does not exist.
        const CREATE    = 1 << 2;
        /// Fail creation when the final component already exists.
        const EXCLUSIVE = 1 << 3;
        /// Truncate an existing regular file to zero bytes.
        const TRUNCATE  = 1 << 4;
        /// Write at the end of the file atomically.
        const APPEND    = 1 << 5;
        /// Require the opened vnode to be a directory.
        const DIRECTORY = 1 << 6;
        /// Do not follow a final symbolic link.
        const NOFOLLOW  = 1 << 7;
        /// Return rather than waiting for device input.
        const NONBLOCK  = 1 << 8;
        /// Do not acquire a controlling terminal.
        const NOCTTY    = 1 << 9;
        /// Retain a namespace location without opening the underlying node.
        const PATH      = 1 << 10;
    }
}

/// Shared open file description.
pub type FileRef = Arc<OpenFile>;

/// Seek origin for an open file description.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SeekFrom {
    /// Absolute byte offset.
    Start(u64),
    /// Signed displacement from the current offset.
    Current(i64),
    /// Signed displacement from the current file size.
    End(i64),
}

/// Open vnode plus shared file offset and access flags.
pub struct OpenFile {
    anchor: PathAnchor,
    flags: AtomicU32,
    file_context: Option<usize>,
    offset: FileOffset,
}

enum FileOffset {
    Seekable(Mutex<u64>),
    Stream {
        read: Mutex<u64>,
        write: Mutex<u64>,
        read_snapshot: core::sync::atomic::AtomicU64,
    },
}

impl FileOffset {
    fn new(kind: VnodeKind, initial: u64) -> Self {
        if matches!(
            kind,
            VnodeKind::CharacterDevice | VnodeKind::Fifo | VnodeKind::Socket
        ) {
            Self::Stream {
                read: Mutex::new(initial),
                write: Mutex::new(initial),
                read_snapshot: core::sync::atomic::AtomicU64::new(initial),
            }
        } else {
            Self::Seekable(Mutex::new(initial))
        }
    }

    fn read(&self) -> &Mutex<u64> {
        match self {
            Self::Seekable(offset) | Self::Stream { read: offset, .. } => offset,
        }
    }

    fn write(&self) -> &Mutex<u64> {
        match self {
            Self::Seekable(offset) | Self::Stream { write: offset, .. } => offset,
        }
    }

    fn seekable(&self) -> Option<&Mutex<u64>> {
        match self {
            Self::Seekable(offset) => Some(offset),
            Self::Stream { .. } => None,
        }
    }

    fn publish_read(&self, offset: u64) {
        if let Self::Stream { read_snapshot, .. } = self {
            read_snapshot.store(offset, Ordering::Release);
        }
    }

    fn poll_offset(&self) -> u64 {
        match self {
            Self::Seekable(offset) => *offset.lock(),
            Self::Stream { read_snapshot, .. } => read_snapshot.load(Ordering::Acquire),
        }
    }
}

impl OpenFile {
    pub(crate) fn new(anchor: PathAnchor, flags: OpenFlags) -> Result<FileRef> {
        if flags.contains(OpenFlags::PATH) {
            return Ok(Arc::new(Self {
                anchor,
                offset: FileOffset::new(VnodeKind::Regular, 0),
                flags: AtomicU32::new(flags.bits()),
                file_context: None,
            }));
        }
        let vnode = anchor.vnode();
        let file_context = vnode.open(flags.bits())?;
        // Truncation happens only once the filesystem has accepted the open, so
        // a rejected open never destroys file contents.
        let prepared = if flags.contains(OpenFlags::TRUNCATE) && vnode.kind() == VnodeKind::Regular
        {
            vnode.truncate(0)
        } else {
            Ok(())
        };
        let offset = match prepared.and_then(|()| vnode.initial_offset(file_context, flags.bits()))
        {
            Ok(offset) => offset,
            Err(error) => {
                vnode.close(file_context, flags.bits());
                return Err(error);
            }
        };
        let kind = vnode.kind();
        Ok(Arc::new(Self {
            anchor,
            offset: FileOffset::new(kind, offset),
            flags: AtomicU32::new(flags.bits()),
            file_context: Some(file_context),
        }))
    }

    /// Path descriptions have no device or filesystem open context.
    pub(crate) fn is_path(&self) -> bool {
        self.file_context.is_none()
    }

    fn io_context(&self) -> Result<usize> {
        self.file_context.ok_or(Error::BadFileDescriptor)
    }

    /// Returns the underlying vnode.
    pub fn vnode(&self) -> &Vnode {
        self.anchor.vnode()
    }

    /// Returns the stable namespace location used for directory-relative lookup.
    pub(crate) fn path_anchor(&self) -> PathAnchor {
        self.anchor.clone()
    }

    /// Returns the shared position without changing it or invoking a device.
    pub(crate) fn position(&self) -> u64 {
        self.offset.poll_offset()
    }

    /// Returns open flags.
    pub fn flags(&self) -> OpenFlags {
        OpenFlags::from_bits_retain(self.flags.load(Ordering::Acquire))
    }

    /// Changes mutable open-file status flags.
    pub fn set_status_flags(&self, append: bool, nonblocking: bool) {
        let mut next = self.flags();
        next.set(OpenFlags::APPEND, append);
        next.set(OpenFlags::NONBLOCK, nonblocking);
        self.flags.store(next.bits(), Ordering::Release);
    }

    /// Reads from and advances the shared offset.
    pub fn read(&self, sink: &mut IoSink<'_>) -> Result<usize> {
        let flags = self.flags();
        if !flags.contains(OpenFlags::READ) {
            return Err(Error::BadFileDescriptor);
        }
        let mut offset = self.offset.read().lock();
        let read =
            self.vnode()
                .read_at_with_flags(self.io_context()?, *offset, sink, flags.bits())?;
        *offset = offset.checked_add(read as u64).ok_or(Error::FileTooLarge)?;
        self.offset.publish_read(*offset);
        Ok(read)
    }

    /// Writes to and advances the shared offset.
    pub fn write(&self, source: &IoSource<'_>) -> Result<usize> {
        let flags = self.flags();
        if !flags.contains(OpenFlags::WRITE) {
            return Err(Error::BadFileDescriptor);
        }
        let mut offset = self.offset.write().lock();
        if flags.contains(OpenFlags::APPEND) && self.vnode().kind() == VnodeKind::Regular {
            let (written, new_offset) = self.vnode().append(source)?;
            *offset = new_offset;
            return Ok(written);
        }

        let written =
            self.vnode()
                .write_at_with_flags(self.io_context()?, *offset, source, flags.bits())?;
        *offset = offset
            .checked_add(written as u64)
            .ok_or(Error::FileTooLarge)?;
        Ok(written)
    }

    /// Reads without changing the shared offset.
    pub fn read_at(&self, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        let flags = self.flags();
        if !flags.contains(OpenFlags::READ) {
            return Err(Error::BadFileDescriptor);
        }
        self.vnode()
            .read_at_with_flags(self.io_context()?, offset, sink, flags.bits())
    }

    /// Writes without changing the shared offset.
    pub fn write_at(&self, offset: u64, source: &IoSource<'_>) -> Result<usize> {
        if !self.flags().contains(OpenFlags::WRITE) {
            return Err(Error::BadFileDescriptor);
        }
        self.vnode().write_at(offset, source)
    }

    /// Changes and returns the shared offset.
    pub fn seek(&self, from: SeekFrom) -> Result<u64> {
        self.io_context()?;
        let mut offset = self.offset.seekable().ok_or(Error::IllegalSeek)?.lock();
        let base = match from {
            SeekFrom::Start(value) => {
                *offset = value;
                return Ok(value);
            }
            SeekFrom::Current(_) => *offset,
            SeekFrom::End(_) => self.vnode().getattr()?.size,
        };
        let displacement = match from {
            SeekFrom::Current(value) | SeekFrom::End(value) => value,
            SeekFrom::Start(_) => unreachable!(),
        };
        let next = if displacement >= 0 {
            base.checked_add(displacement as u64)
        } else {
            base.checked_sub(displacement.unsigned_abs())
        }
        .ok_or(Error::InvalidArgument)?;
        *offset = next;
        Ok(next)
    }

    /// Returns current vnode metadata.
    pub fn getattr(&self) -> Result<VnodeAttr> {
        self.vnode().getattr()
    }

    /// Changes permission bits on the underlying vnode.
    pub fn set_mode(&self, mode: u16) -> Result<()> {
        self.io_context()?;
        self.vnode().setattr(super::vnode::SetAttr {
            size: None,
            mode: Some(mode),
        })
    }

    /// Returns requested events that are immediately ready.
    pub fn poll(&self, events: PollEvents) -> Result<PollEvents> {
        let Some(context) = self.file_context else {
            return Ok(PollEvents::NVAL);
        };
        let offset = self.offset.poll_offset();
        self.vnode()
            .poll_with_flags(context, offset, events, self.flags().bits())
    }

    /// Appends events that can wake a readiness rescan.
    pub fn poll_events<'a>(
        &'a self,
        mut events: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        let Some(context) = self.file_context else {
            return true;
        };
        let flags = self.flags();
        if !flags.contains(OpenFlags::READ) {
            events.remove(PollEvents::IN | PollEvents::RDNORM);
        }
        if !flags.contains(OpenFlags::WRITE) {
            events.remove(PollEvents::OUT | PollEvents::WRNORM);
        }
        self.vnode().poll_events(context, events, output)
    }

    /// Returns terminal job-control state for this open file.
    pub(crate) fn terminal_state(&self) -> Option<TerminalState> {
        self.file_context?;
        self.vnode().terminal_state()
    }

    /// Resolves backing storage for a mapping of this open file description.
    pub fn mapping_object(
        &self,
        offset: u64,
        length: u64,
    ) -> Result<(Arc<crate::mem::VmObject>, u64)> {
        self.vnode()
            .mapping_object(self.io_context()?, offset, length)
    }

    /// Performs a device- or filesystem-specific control operation.
    pub fn ioctl(
        &self,
        context: IoctlContext,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> Result<u64> {
        self.vnode()
            .ioctl(self.io_context()?, context, request, value, argument)
    }

    /// Reads a directory batch using the shared offset as the directory cookie.
    pub fn readdir(&self, maximum: usize) -> Result<Vec<DirEntry>> {
        self.io_context()?;
        if self.vnode().kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        let mut cursor = self.offset.seekable().ok_or(Error::IllegalSeek)?.lock();
        let (entries, next) = self.vnode().readdir(*cursor, maximum)?;
        *cursor = next;
        Ok(entries)
    }
}

impl Drop for OpenFile {
    fn drop(&mut self) {
        if let Some(context) = self.file_context {
            self.vnode()
                .close(context, self.flags.load(Ordering::Relaxed));
        }
    }
}
