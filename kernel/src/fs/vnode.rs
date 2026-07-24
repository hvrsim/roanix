//! Filesystem-independent vnode and filesystem contracts.

use alloc::{
    boxed::Box,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    any::Any,
    fmt,
    sync::atomic::{AtomicU64, Ordering},
};

use bitflags::bitflags;

use crate::{mem::VmObject, sys::event::Event};

use super::error::{Error, Result};

static NEXT_FILESYSTEM_ID: AtomicU64 = AtomicU64::new(1);

/// Unique identifier for one mounted filesystem instance.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct FilesystemId(u64);

impl FilesystemId {
    /// Allocates a new globally unique filesystem identifier.
    pub(crate) fn allocate() -> Self {
        Self(NEXT_FILESYSTEM_ID.fetch_add(1, Ordering::Relaxed))
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Filesystem-local node number.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NodeId(u64);

impl NodeId {
    /// Creates a node identifier from a filesystem-provided number.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric node identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Stable identity of a vnode across the VFS namespace.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct VnodeKey {
    /// Filesystem instance containing the node.
    pub filesystem: FilesystemId,
    /// Filesystem-local node number.
    pub node: NodeId,
}

/// Filesystem-independent vnode type.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VnodeKind {
    /// Regular byte-addressable file.
    Regular,
    /// Directory containing named children.
    Directory,
    /// Symbolic link containing another path.
    Symlink,
    /// Character device.
    CharacterDevice,
    /// Block device.
    BlockDevice,
    /// Named pipe.
    Fifo,
    /// Local socket endpoint.
    Socket,
}

bitflags! {
    /// Readiness events used by descriptor polling.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct PollEvents: u16 {
        /// Data can be read without blocking.
        const IN = 0x0001;
        /// Exceptional data is available.
        const PRI = 0x0002;
        /// Data can be written without blocking.
        const OUT = 0x0004;
        /// An asynchronous error is pending.
        const ERR = 0x0008;
        /// The peer has closed its endpoint.
        const HUP = 0x0010;
        /// The descriptor is invalid.
        const NVAL = 0x0020;
        /// Normal data can be read without blocking.
        const RDNORM = 0x0040;
        /// Priority data can be read without blocking.
        const RDBAND = 0x0080;
        /// Normal data can be written without blocking.
        const WRNORM = 0x0100;
        /// Priority data can be written without blocking.
        const WRBAND = 0x0200;
    }
}

/// Metadata returned for a vnode.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct VnodeAttr {
    /// Stable vnode identity.
    pub key: VnodeKey,
    /// Node type.
    pub kind: VnodeKind,
    /// Logical byte size.
    pub size: u64,
    /// Number of directory links.
    pub links: u64,
    /// Unix-style permission and type-independent mode bits.
    pub mode: u16,
    /// Monotonic access timestamp in nanoseconds.
    pub accessed_ns: u64,
    /// Monotonic content-modification timestamp in nanoseconds.
    pub modified_ns: u64,
    /// Monotonic metadata-change timestamp in nanoseconds.
    pub changed_ns: u64,
}

/// Optional metadata changes requested by the caller.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct SetAttr {
    /// New file size, if truncation is requested.
    pub size: Option<u64>,
    /// New permission bits.
    pub mode: Option<u16>,
}

/// Caller metadata supplied to device-control operations.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct IoctlContext {
    /// Process issuing the operation.
    pub process_id: usize,
    /// Process group of the caller.
    pub process_group: i32,
    /// Session containing the caller.
    pub session_id: i32,
    /// Whether the caller is its session leader.
    pub is_session_leader: bool,
}

/// Job-control state exposed by terminal vnodes.
#[derive(Copy, Clone)]
pub(crate) struct TerminalState {
    /// Session that owns the controlling terminal.
    pub session: i32,
    /// Foreground process group.
    pub foreground_group: i32,
    /// Whether background writes should stop their process group.
    pub stop_background_output: bool,
}

/// Node type requested from a directory create operation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CreateKind {
    /// Create an empty regular file.
    Regular,
    /// Create an empty directory.
    Directory,
    /// Create a symbolic link containing the supplied target.
    Symlink(Box<[u8]>),
}

/// One directory entry returned by `readdir`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DirEntry {
    /// Entry name, excluding `.` and `..` path separators.
    pub name: Box<[u8]>,
    /// Stable target identity.
    pub key: VnodeKey,
    /// Target vnode type.
    pub kind: VnodeKind,
}

/// Filesystem capacity and usage snapshot.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct StatFs {
    /// Total addressable bytes.
    pub total_bytes: u64,
    /// Currently allocated bytes.
    pub used_bytes: u64,
    /// Maximum node count.
    pub total_nodes: u64,
    /// Currently allocated node count.
    pub used_nodes: u64,
}

/// Mounted filesystem implementation.
pub trait FileSystem: Send + Sync {
    /// Returns this filesystem instance's stable identifier.
    fn id(&self) -> FilesystemId;

    /// Returns a short human-readable filesystem type name.
    fn name(&self) -> &'static str;

    /// Returns the filesystem root vnode.
    fn root(&self) -> Vnode;

    /// Returns a capacity and usage snapshot.
    fn statfs(&self) -> StatFs;

    /// Flushes filesystem state. Memory-only filesystems may return success.
    fn sync(&self) -> Result<()> {
        Ok(())
    }
}

/// Shared filesystem reference stored by a mount.
pub type FileSystemRef = Arc<dyn FileSystem>;

/// Filesystem-specific operations behind a generic vnode.
pub trait VnodeOps: Any + Send + Sync {
    /// Supports checked downcasting by the owning filesystem.
    fn as_any(&self) -> &dyn Any;

    /// Returns the initial byte offset for a new open file description.
    fn initial_offset(&self, _vnode: &Vnode, _file_context: usize, _flags: u32) -> Result<u64> {
        Ok(0)
    }

    /// Notifies the filesystem that an open file description was created.
    fn open(&self, _vnode: &Vnode, _flags: u32) -> Result<usize> {
        Ok(0)
    }

    /// Notifies the filesystem that an open file description was destroyed.
    fn close(&self, _vnode: &Vnode, _file_context: usize, _flags: u32) {}

    /// Returns current vnode metadata.
    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr>;

    /// Applies supported metadata changes.
    fn setattr(&self, _vnode: &Vnode, _attr: SetAttr) -> Result<()> {
        Err(Error::Unsupported)
    }

    /// Looks up one direct child.
    fn lookup(&self, _directory: &Vnode, _name: &[u8]) -> Result<Vnode> {
        Err(Error::NotDirectory)
    }

    /// Returns the logical parent directory.
    fn parent(&self, _directory: &Vnode) -> Result<Vnode> {
        Err(Error::NotDirectory)
    }

    /// Creates a direct child.
    fn create(
        &self,
        _directory: &Vnode,
        _name: &[u8],
        _kind: CreateKind,
        _mode: u16,
    ) -> Result<Vnode> {
        Err(Error::NotDirectory)
    }

    /// Adds another directory link to an existing non-directory vnode.
    fn link(&self, _directory: &Vnode, _name: &[u8], _target: &Vnode) -> Result<()> {
        Err(Error::NotDirectory)
    }

    /// Removes one direct child.
    fn unlink(&self, _directory: &Vnode, _name: &[u8], _remove_directory: bool) -> Result<()> {
        Err(Error::NotDirectory)
    }

    /// Atomically moves or replaces a directory entry.
    fn rename(
        &self,
        _source_directory: &Vnode,
        _source_name: &[u8],
        _target_directory: &Vnode,
        _target_name: &[u8],
    ) -> Result<()> {
        Err(Error::NotDirectory)
    }

    /// Reads file data at an explicit byte offset.
    fn read_at(&self, _vnode: &Vnode, _offset: u64, _buffer: &mut [u8]) -> Result<usize> {
        Err(Error::IsDirectory)
    }

    /// Reads file data with the originating open flags.
    fn read_at_with_flags(
        &self,
        vnode: &Vnode,
        _file_context: usize,
        offset: u64,
        buffer: &mut [u8],
        _flags: u32,
    ) -> Result<usize> {
        self.read_at(vnode, offset, buffer)
    }

    /// Writes file data at an explicit byte offset.
    fn write_at(&self, _vnode: &Vnode, _offset: u64, _buffer: &[u8]) -> Result<usize> {
        Err(Error::IsDirectory)
    }

    /// Writes file data with the originating open flags.
    fn write_at_with_flags(
        &self,
        vnode: &Vnode,
        _file_context: usize,
        offset: u64,
        buffer: &[u8],
        _flags: u32,
    ) -> Result<usize> {
        self.write_at(vnode, offset, buffer)
    }

    /// Returns events that are immediately ready for this open file.
    fn poll(
        &self,
        vnode: &Vnode,
        _file_context: usize,
        _offset: u64,
        events: PollEvents,
        flags: u32,
    ) -> Result<PollEvents> {
        let flags = super::file::OpenFlags::from_bits_retain(flags);
        let mut supported = PollEvents::empty();
        if flags.contains(super::file::OpenFlags::READ)
            && matches!(vnode.kind(), VnodeKind::Regular | VnodeKind::Directory)
        {
            supported |= PollEvents::IN | PollEvents::RDNORM;
        }
        if flags.contains(super::file::OpenFlags::WRITE) && vnode.kind() == VnodeKind::Regular {
            supported |= PollEvents::OUT | PollEvents::WRNORM;
        }
        Ok(events & supported)
    }

    /// Appends events that can wake a readiness rescan.
    fn poll_events<'a>(
        &'a self,
        _vnode: &Vnode,
        _file_context: usize,
        _events: PollEvents,
        _output: &mut Vec<&'a Event>,
    ) -> bool {
        false
    }

    /// Returns terminal job-control state when this vnode is a TTY.
    fn terminal_state(&self, _vnode: &Vnode) -> Option<TerminalState> {
        None
    }

    /// Appends bytes atomically and returns `(written, new_offset)`.
    fn append(&self, _vnode: &Vnode, _buffer: &[u8]) -> Result<(usize, u64)> {
        Err(Error::Unsupported)
    }

    /// Changes a regular file's logical size.
    fn truncate(&self, _vnode: &Vnode, _size: u64) -> Result<()> {
        Err(Error::IsDirectory)
    }

    /// Returns the unified page-cache object used for memory mappings.
    fn memory_object(&self, _vnode: &Vnode) -> Result<Arc<VmObject>> {
        Err(Error::Unsupported)
    }

    /// Reads a symbolic-link target.
    fn readlink(&self, _vnode: &Vnode) -> Result<Box<[u8]>> {
        Err(Error::InvalidArgument)
    }

    /// Reads directory entries beginning at `cursor`.
    fn readdir(
        &self,
        _directory: &Vnode,
        _cursor: u64,
        _maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        Err(Error::NotDirectory)
    }

    /// Flushes vnode state.
    fn fsync(&self, _vnode: &Vnode) -> Result<()> {
        Ok(())
    }

    /// Performs a filesystem- or device-specific control operation.
    fn ioctl(
        &self,
        _vnode: &Vnode,
        _file_context: usize,
        _context: IoctlContext,
        _request: u64,
        _value: u64,
        _argument: &mut [u8],
    ) -> Result<u64> {
        Err(Error::NotTty)
    }
}

struct VnodeInner {
    key: VnodeKey,
    kind: VnodeKind,
    operations: Box<dyn VnodeOps>,
}

/// Filesystem-independent handle to one active node.
///
/// The `Arc` count is the vnode use count from the vnode architecture. A
/// filesystem must return clones of the same `Vnode` for the same active node.
#[derive(Clone)]
pub struct Vnode {
    inner: Arc<VnodeInner>,
}

impl Vnode {
    /// Creates a generic vnode around filesystem-private operations.
    pub(crate) fn new(key: VnodeKey, kind: VnodeKind, operations: Box<dyn VnodeOps>) -> Self {
        Self {
            inner: Arc::new(VnodeInner {
                key,
                kind,
                operations,
            }),
        }
    }

    /// Returns stable vnode identity.
    pub fn key(&self) -> VnodeKey {
        self.inner.key
    }

    /// Returns the vnode type without invoking the filesystem.
    pub fn kind(&self) -> VnodeKind {
        self.inner.kind
    }

    fn inotify_mask(&self, mask: u32) -> u32 {
        mask | if self.kind() == VnodeKind::Directory {
            crate::proc::inotify::IN_ISDIR
        } else {
            0
        }
    }

    /// Creates a non-owning vnode reference.
    pub fn downgrade(&self) -> VnodeWeak {
        VnodeWeak {
            inner: Arc::downgrade(&self.inner),
        }
    }

    /// Returns the number of active strong references.
    pub fn strong_count(&self) -> usize {
        Arc::strong_count(&self.inner)
    }

    /// Returns current vnode metadata.
    pub fn getattr(&self) -> Result<VnodeAttr> {
        self.inner.operations.getattr(self)
    }

    pub(crate) fn initial_offset(&self, file_context: usize, flags: u32) -> Result<u64> {
        self.inner
            .operations
            .initial_offset(self, file_context, flags)
    }

    /// Applies supported metadata changes.
    pub fn setattr(&self, attr: SetAttr) -> Result<()> {
        self.inner.operations.setattr(self, attr)?;
        crate::proc::inotify::notify(
            self.key(),
            self.inotify_mask(crate::proc::inotify::IN_ATTRIB),
            0,
            None,
        );
        Ok(())
    }

    /// Looks up one direct child.
    pub(super) fn lookup(&self, name: &[u8]) -> Result<Self> {
        self.inner.operations.lookup(self, name)
    }

    /// Returns the logical parent directory.
    pub(super) fn parent(&self) -> Result<Self> {
        self.inner.operations.parent(self)
    }

    /// Creates a direct child.
    pub(super) fn create(&self, name: &[u8], kind: CreateKind, mode: u16) -> Result<Self> {
        let child = self.inner.operations.create(self, name, kind, mode)?;
        let mask = crate::proc::inotify::IN_CREATE
            | if child.kind() == VnodeKind::Directory {
                crate::proc::inotify::IN_ISDIR
            } else {
                0
            };
        crate::proc::inotify::notify(self.key(), mask, 0, Some(name));
        Ok(child)
    }

    /// Adds a hard link in this directory.
    pub(super) fn link(&self, name: &[u8], target: &Self) -> Result<()> {
        self.inner.operations.link(self, name, target)?;
        crate::proc::inotify::notify(self.key(), crate::proc::inotify::IN_CREATE, 0, Some(name));
        crate::proc::inotify::notify(target.key(), crate::proc::inotify::IN_ATTRIB, 0, None);
        Ok(())
    }

    /// Removes one direct child.
    pub(super) fn unlink(&self, name: &[u8], remove_directory: bool) -> Result<()> {
        let target = self.lookup(name)?;
        self.inner.operations.unlink(self, name, remove_directory)?;
        let directory_flag = if target.kind() == VnodeKind::Directory {
            crate::proc::inotify::IN_ISDIR
        } else {
            0
        };
        crate::proc::inotify::notify(
            self.key(),
            crate::proc::inotify::IN_DELETE | directory_flag,
            0,
            Some(name),
        );
        crate::proc::inotify::notify(
            target.key(),
            crate::proc::inotify::IN_DELETE_SELF | directory_flag,
            0,
            None,
        );
        Ok(())
    }

    /// Atomically renames a direct child.
    pub(super) fn rename(
        &self,
        source_name: &[u8],
        target_directory: &Self,
        target_name: &[u8],
    ) -> Result<()> {
        let source = self.lookup(source_name)?;
        let replaced = target_directory.lookup(target_name).ok();
        self.inner
            .operations
            .rename(self, source_name, target_directory, target_name)?;
        let cookie = crate::proc::inotify::next_cookie();
        let directory_flag = if source.kind() == VnodeKind::Directory {
            crate::proc::inotify::IN_ISDIR
        } else {
            0
        };
        crate::proc::inotify::notify(
            self.key(),
            crate::proc::inotify::IN_MOVED_FROM | directory_flag,
            cookie,
            Some(source_name),
        );
        crate::proc::inotify::notify(
            target_directory.key(),
            crate::proc::inotify::IN_MOVED_TO | directory_flag,
            cookie,
            Some(target_name),
        );
        crate::proc::inotify::notify(
            source.key(),
            crate::proc::inotify::IN_MOVE_SELF | directory_flag,
            cookie,
            None,
        );
        if let Some(replaced) = replaced.filter(|replaced| replaced.key() != source.key()) {
            crate::proc::inotify::notify(
                replaced.key(),
                crate::proc::inotify::IN_DELETE_SELF
                    | if replaced.kind() == VnodeKind::Directory {
                        crate::proc::inotify::IN_ISDIR
                    } else {
                        0
                    },
                0,
                None,
            );
        }
        Ok(())
    }

    /// Reads bytes at an explicit offset.
    pub fn read_at(&self, offset: u64, buffer: &mut [u8]) -> Result<usize> {
        let read = self.inner.operations.read_at(self, offset, buffer)?;
        if read != 0 {
            crate::proc::inotify::notify(
                self.key(),
                self.inotify_mask(crate::proc::inotify::IN_ACCESS),
                0,
                None,
            );
        }
        Ok(read)
    }

    pub(crate) fn read_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        buffer: &mut [u8],
        flags: u32,
    ) -> Result<usize> {
        let read =
            self.inner
                .operations
                .read_at_with_flags(self, file_context, offset, buffer, flags)?;
        if read != 0 {
            crate::proc::inotify::notify(
                self.key(),
                self.inotify_mask(crate::proc::inotify::IN_ACCESS),
                0,
                None,
            );
        }
        Ok(read)
    }

    /// Writes bytes at an explicit offset.
    pub fn write_at(&self, offset: u64, buffer: &[u8]) -> Result<usize> {
        let written = self.inner.operations.write_at(self, offset, buffer)?;
        if written != 0 {
            crate::proc::inotify::notify(
                self.key(),
                self.inotify_mask(crate::proc::inotify::IN_MODIFY),
                0,
                None,
            );
        }
        Ok(written)
    }

    pub(crate) fn write_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        buffer: &[u8],
        flags: u32,
    ) -> Result<usize> {
        let written =
            self.inner
                .operations
                .write_at_with_flags(self, file_context, offset, buffer, flags)?;
        if written != 0 {
            crate::proc::inotify::notify(
                self.key(),
                self.inotify_mask(crate::proc::inotify::IN_MODIFY),
                0,
                None,
            );
        }
        Ok(written)
    }

    pub(crate) fn poll_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        events: PollEvents,
        flags: u32,
    ) -> Result<PollEvents> {
        self.inner
            .operations
            .poll(self, file_context, offset, events, flags)
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        file_context: usize,
        events: PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        self.inner
            .operations
            .poll_events(self, file_context, events, output)
    }

    pub(crate) fn terminal_state(&self) -> Option<TerminalState> {
        self.inner.operations.terminal_state(self)
    }

    /// Atomically appends bytes.
    pub fn append(&self, buffer: &[u8]) -> Result<(usize, u64)> {
        let result = self.inner.operations.append(self, buffer)?;
        if result.0 != 0 {
            crate::proc::inotify::notify(
                self.key(),
                self.inotify_mask(crate::proc::inotify::IN_MODIFY),
                0,
                None,
            );
        }
        Ok(result)
    }

    /// Changes file size.
    pub fn truncate(&self, size: u64) -> Result<()> {
        self.inner.operations.truncate(self, size)?;
        crate::proc::inotify::notify(
            self.key(),
            self.inotify_mask(crate::proc::inotify::IN_MODIFY),
            0,
            None,
        );
        Ok(())
    }

    /// Returns this vnode's unified page-cache object.
    pub fn memory_object(&self) -> Result<Arc<VmObject>> {
        self.inner.operations.memory_object(self)
    }

    /// Reads a symbolic-link target.
    pub fn readlink(&self) -> Result<Box<[u8]>> {
        let target = self.inner.operations.readlink(self)?;
        crate::proc::inotify::notify(
            self.key(),
            self.inotify_mask(crate::proc::inotify::IN_ACCESS),
            0,
            None,
        );
        Ok(target)
    }

    /// Reads a batch of directory entries.
    pub fn readdir(&self, cursor: u64, maximum: usize) -> Result<(Vec<DirEntry>, u64)> {
        let entries = self.inner.operations.readdir(self, cursor, maximum)?;
        crate::proc::inotify::notify(
            self.key(),
            self.inotify_mask(crate::proc::inotify::IN_ACCESS),
            0,
            None,
        );
        Ok(entries)
    }

    /// Flushes vnode state.
    pub fn fsync(&self) -> Result<()> {
        self.inner.operations.fsync(self)
    }

    /// Performs a control operation.
    pub fn ioctl(
        &self,
        file_context: usize,
        context: IoctlContext,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> Result<u64> {
        self.inner
            .operations
            .ioctl(self, file_context, context, request, value, argument)
    }

    pub(crate) fn open(&self, flags: u32) -> Result<usize> {
        let context = self.inner.operations.open(self, flags)?;
        crate::proc::inotify::notify(
            self.key(),
            self.inotify_mask(crate::proc::inotify::IN_OPEN),
            0,
            None,
        );
        Ok(context)
    }

    pub(crate) fn close(&self, file_context: usize, flags: u32) {
        self.inner.operations.close(self, file_context, flags);
        let mask = if flags & (1 << 1) != 0 {
            crate::proc::inotify::IN_CLOSE_WRITE
        } else {
            crate::proc::inotify::IN_CLOSE_NOWRITE
        };
        crate::proc::inotify::notify(self.key(), self.inotify_mask(mask), 0, None);
    }

    /// Downcasts filesystem-private vnode operations.
    pub(super) fn operations_as<T: Any>(&self) -> Option<&T> {
        self.inner.operations.as_any().downcast_ref::<T>()
    }
}

impl fmt::Debug for Vnode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Vnode")
            .field("key", &self.key())
            .field("kind", &self.kind())
            .finish_non_exhaustive()
    }
}

impl PartialEq for Vnode {
    fn eq(&self, other: &Self) -> bool {
        self.key() == other.key()
    }
}

impl Eq for Vnode {}

/// Non-owning vnode reference used for parent links.
#[derive(Clone, Default)]
pub struct VnodeWeak {
    inner: Weak<VnodeInner>,
}

impl VnodeWeak {
    /// Creates an empty weak vnode reference.
    pub fn new() -> Self {
        Self::default()
    }

    /// Upgrades the reference while the vnode remains active.
    pub fn upgrade(&self) -> Option<Vnode> {
        self.inner.upgrade().map(|inner| Vnode { inner })
    }
}
