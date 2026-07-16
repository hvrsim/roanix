//! Open file descriptions and shared seek offsets.

use alloc::{sync::Arc, vec::Vec};

use bitflags::bitflags;

use crate::sys::sync::Mutex;

use super::{
    error::{Error, Result},
    vnode::{DirEntry, Vnode, VnodeAttr, VnodeKind},
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
    vnode: Vnode,
    flags: OpenFlags,
    offset: Mutex<u64>,
}

impl OpenFile {
    pub(crate) fn new(vnode: Vnode, flags: OpenFlags) -> Result<FileRef> {
        vnode.open(flags.bits())?;
        Ok(Arc::new(Self {
            vnode,
            flags,
            offset: Mutex::new(0),
        }))
    }

    /// Returns the underlying vnode.
    pub fn vnode(&self) -> &Vnode {
        &self.vnode
    }

    /// Returns open flags.
    pub fn flags(&self) -> OpenFlags {
        self.flags
    }

    /// Reads from and advances the shared offset.
    pub fn read(&self, buffer: &mut [u8]) -> Result<usize> {
        if !self.flags.contains(OpenFlags::READ) {
            return Err(Error::BadFileDescriptor);
        }
        let mut offset = self.offset.lock();
        let read = self.vnode.read_at(*offset, buffer)?;
        *offset = offset.saturating_add(read as u64);
        Ok(read)
    }

    /// Writes to and advances the shared offset.
    pub fn write(&self, buffer: &[u8]) -> Result<usize> {
        if !self.flags.contains(OpenFlags::WRITE) {
            return Err(Error::BadFileDescriptor);
        }
        let mut offset = self.offset.lock();
        if self.flags.contains(OpenFlags::APPEND) {
            let (written, new_offset) = self.vnode.append(buffer)?;
            *offset = new_offset;
            return Ok(written);
        }

        let written = self.vnode.write_at(*offset, buffer)?;
        *offset = offset.saturating_add(written as u64);
        Ok(written)
    }

    /// Reads without changing the shared offset.
    pub fn read_at(&self, offset: u64, buffer: &mut [u8]) -> Result<usize> {
        if !self.flags.contains(OpenFlags::READ) {
            return Err(Error::BadFileDescriptor);
        }
        self.vnode.read_at(offset, buffer)
    }

    /// Writes without changing the shared offset.
    pub fn write_at(&self, offset: u64, buffer: &[u8]) -> Result<usize> {
        if !self.flags.contains(OpenFlags::WRITE) {
            return Err(Error::BadFileDescriptor);
        }
        self.vnode.write_at(offset, buffer)
    }

    /// Changes and returns the shared offset.
    pub fn seek(&self, from: SeekFrom) -> Result<u64> {
        let mut offset = self.offset.lock();
        let base = match from {
            SeekFrom::Start(value) => {
                *offset = value;
                return Ok(value);
            }
            SeekFrom::Current(_) => *offset,
            SeekFrom::End(_) => self.vnode.getattr()?.size,
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
        self.vnode.getattr()
    }

    /// Reads a directory batch using the shared offset as the directory cookie.
    pub fn readdir(&self, maximum: usize) -> Result<Vec<DirEntry>> {
        if self.vnode.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        let mut cursor = self.offset.lock();
        let (entries, next) = self.vnode.readdir(*cursor, maximum)?;
        *cursor = next;
        Ok(entries)
    }
}

impl Drop for OpenFile {
    fn drop(&mut self) {
        self.vnode.close(self.flags.bits());
    }
}
