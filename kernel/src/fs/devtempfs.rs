//! Driver-managed temporary device filesystem.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    any::Any,
    sync::atomic::{AtomicU16, AtomicU64, Ordering},
};

use crate::{
    sys::{
        clock,
        event::Event,
        sync::{Mutex, Once},
    },
};

use super::{
    error::{Error, Result},
    path,
    vnode::{
        DirEntry, FileSystem, FilesystemId, IoctlContext, NodeId, PollEvents, SetAttr, StatFs,
        TerminalState, Vnode, VnodeAttr, VnodeKey, VnodeKind, VnodeOps,
    },
};

const DEVTEMPFS_NAME: &str = "devtempfs";
const ROOT_NODE_ID: u64 = 1;
const NODE_REVOKED: u64 = 1 << 63;
const NODE_OPEN_ONE: u64 = 1 << 32;
const NODE_OPEN_MASK: u64 = ((1 << 31) - 1) << 32;
const NODE_ACTIVE_MASK: u64 = u32::MAX as u64;

/// Opaque token identifying the code that owns a devtempfs node.
///
/// The filesystem never interprets this value. It only uses it to decide which
/// nodes disappear when a module is unloaded, which keeps devtempfs independent
/// of the driver framework.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct OwnerId(u64);

impl OwnerId {
    /// Owner used by kernel-created nodes such as the filesystem root.
    pub const KERNEL: Self = Self(0);

    /// Creates an owner token from a caller-defined value.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric value.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Stable devtempfs node identifier.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DevNodeId(u64);
impl DevNodeId {
    const fn new(value: u64) -> Self {
        Self(value)
    }

    pub(crate) const fn from_raw(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Device vnode type created by a driver.
#[repr(u32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DeviceNodeKind {
    /// Byte-stream character device.
    Character = 1,
    /// Random-access block device.
    Block = 2,
}

impl DeviceNodeKind {
    fn vnode_kind(self) -> VnodeKind {
        match self {
            Self::Character => VnodeKind::CharacterDevice,
            Self::Block => VnodeKind::BlockDevice,
        }
    }
}

/// File operations implemented by a driver-owned device node.
pub trait DeviceNodeOps: Send + Sync {
    /// Returns the initial byte offset for a new open file description.
    fn initial_offset(&self, _file_context: usize, _flags: u32) -> Result<u64> {
        Ok(0)
    }

    /// Opens a new file description.
    fn open(&self, _flags: u32) -> Result<usize> {
        Ok(0)
    }

    /// Closes a file description.
    fn close(&self, _file_context: usize, _flags: u32) {}

    /// Reads bytes at an explicit offset.
    fn read_at(&self, _offset: u64, _buffer: &mut [u8]) -> Result<usize> {
        Err(Error::Unsupported)
    }

    /// Reads bytes while observing the originating open flags.
    fn read_at_with_flags(
        &self,
        _file_context: usize,
        offset: u64,
        buffer: &mut [u8],
        _flags: u32,
    ) -> Result<usize> {
        self.read_at(offset, buffer)
    }

    /// Writes bytes at an explicit offset.
    fn write_at(&self, _offset: u64, _buffer: &[u8]) -> Result<usize> {
        Err(Error::Unsupported)
    }

    /// Writes bytes while observing the originating open flags.
    fn write_at_with_flags(
        &self,
        _file_context: usize,
        offset: u64,
        buffer: &[u8],
        _flags: u32,
    ) -> Result<usize> {
        self.write_at(offset, buffer)
    }

    /// Returns requested events that are immediately ready.
    fn poll(
        &self,
        _file_context: usize,
        _offset: u64,
        _events: PollEvents,
        _flags: u32,
    ) -> Result<PollEvents> {
        Ok(PollEvents::empty())
    }

    /// Appends events that can wake a readiness rescan.
    fn poll_events<'a>(
        &'a self,
        _file_context: usize,
        _events: PollEvents,
        _output: &mut Vec<&'a Event>,
    ) -> bool {
        false
    }

    /// Returns terminal job-control state for TTY devices.
    fn terminal_state(&self) -> Option<TerminalState> {
        None
    }

    /// Returns the current logical size.
    fn size(&self) -> u64 {
        0
    }

    /// Flushes device state.
    fn sync(&self) -> Result<()> {
        Ok(())
    }

    /// Performs a device-specific control operation.
    fn ioctl(
        &self,
        _file_context: usize,
        _context: IoctlContext,
        _request: u64,
        _value: u64,
        _argument: &mut [u8],
    ) -> Result<u64> {
        Err(Error::NotTty)
    }
}

/// Mounted driver-managed filesystem.
pub struct Devtempfs {
    id: FilesystemId,
    next_node: AtomicU64,
    used_nodes: AtomicU64,
    state: Mutex<DevtempfsState>,
    root: Once<Vnode>,
}

struct DevtempfsState {
    records: BTreeMap<DevNodeId, DevRecord>,
}

struct DevRecord {
    owner: OwnerId,
    parent: DevNodeId,
    vnode: Vnode,
    children: BTreeMap<Box<[u8]>, DevNodeId>,
}

struct DevtempfsNode {
    filesystem: Weak<Devtempfs>,
    id: DevNodeId,
    owner: OwnerId,
    kind: VnodeKind,
    mode: AtomicU16,
    links: AtomicU64,
    accessed_ns: AtomicU64,
    modified_ns: AtomicU64,
    changed_ns: AtomicU64,
    lifecycle: AtomicU64,
    data: DevtempfsData,
}

struct NodeActivity<'a> {
    lifecycle: &'a AtomicU64,
}

enum DevtempfsData {
    Directory,
    Device(Arc<dyn DeviceNodeOps>),
}

impl Drop for NodeActivity<'_> {
    fn drop(&mut self) {
        let previous = self.lifecycle.fetch_sub(1, Ordering::Release);
        assert!(
            previous & NODE_ACTIVE_MASK != 0,
            "devtempfs: active operation count underflow"
        );
    }
}

static DEVTEMPFS: Once<Arc<Devtempfs>> = Once::new();

impl Devtempfs {
    /// Creates an empty devtempfs instance.
    pub fn new() -> Result<Arc<Self>> {
        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            next_node: AtomicU64::new(ROOT_NODE_ID + 1),
            used_nodes: AtomicU64::new(0),
            state: Mutex::new(DevtempfsState {
                records: BTreeMap::new(),
            }),
            root: Once::new(),
        });
        let root_id = DevNodeId::new(ROOT_NODE_ID);
        let root = filesystem.make_node(
            root_id,
            OwnerId::KERNEL,
            VnodeKind::Directory,
            0o755,
            DevtempfsData::Directory,
        );
        filesystem.state.lock().records.insert(
            root_id,
            DevRecord {
                owner: OwnerId::KERNEL,
                parent: root_id,
                vnode: root.clone(),
                children: BTreeMap::new(),
            },
        );
        filesystem.used_nodes.store(1, Ordering::Release);
        filesystem.root.call_once(|| root);
        Ok(filesystem)
    }

    /// Returns the root directory identifier.
    pub const fn root_id(&self) -> DevNodeId {
        DevNodeId::new(ROOT_NODE_ID)
    }

    /// Creates a driver-owned directory.
    pub fn create_dir(
        self: &Arc<Self>,
        owner: OwnerId,
        parent: DevNodeId,
        name: &[u8],
        mode: u16,
    ) -> Result<DevNodeId> {
        self.create_node(
            owner,
            parent,
            name,
            VnodeKind::Directory,
            mode,
            DevtempfsData::Directory,
        )
    }

    /// Creates a driver-owned character or block device node.
    pub fn create_device(
        self: &Arc<Self>,
        owner: OwnerId,
        parent: DevNodeId,
        name: &[u8],
        kind: DeviceNodeKind,
        mode: u16,
        operations: Arc<dyn DeviceNodeOps>,
    ) -> Result<DevNodeId> {
        self.create_node(
            owner,
            parent,
            name,
            kind.vnode_kind(),
            mode,
            DevtempfsData::Device(operations),
        )
    }

    /// Removes one empty, unopened node owned by a driver.
    pub fn remove_node(&self, owner: OwnerId, node: DevNodeId) -> Result<()> {
        if node == self.root_id() {
            return Err(Error::PermissionDenied);
        }
        let mut state = self.state.lock();
        let record = state.records.get(&node).ok_or(Error::NotFound)?;
        if record.owner != owner {
            return Err(Error::PermissionDenied);
        }
        if !record.children.is_empty() {
            return Err(Error::NotEmpty);
        }
        let operations = node_operations(&record.vnode)?;
        operations.try_revoke()?;
        remove_record(&mut state, node)?;
        self.used_nodes.fetch_sub(1, Ordering::AcqRel);
        Ok(())
    }

    pub(crate) fn owner(&self, node: DevNodeId) -> Result<OwnerId> {
        self.state
            .lock()
            .records
            .get(&node)
            .map(|record| record.owner)
            .ok_or(Error::NotFound)
    }

    /// Returns the child of `parent` named `name`.
    pub fn lookup_child(&self, parent: DevNodeId, name: &[u8]) -> Result<DevNodeId> {
        self.state
            .lock()
            .records
            .get(&parent)
            .ok_or(Error::NotFound)?
            .children
            .get(name)
            .copied()
            .ok_or(Error::NotFound)
    }

    /// Returns the names of every entry directly below `parent`.
    pub fn child_names(&self, parent: DevNodeId) -> Result<Vec<Box<[u8]>>> {
        Ok(self
            .state
            .lock()
            .records
            .get(&parent)
            .ok_or(Error::NotFound)?
            .children
            .keys()
            .cloned()
            .collect())
    }

    pub(crate) fn can_remove_owner(&self, owner: OwnerId) -> Result<()> {
        let state = self.state.lock();
        validate_owner_removal(&state, owner)
    }

    pub(crate) fn remove_owner(&self, owner: OwnerId) -> Result<()> {
        let mut state = self.state.lock();
        revoke_owner_nodes(&state, owner)?;
        remove_owned_records(self, &mut state, owner)
    }

    pub(crate) fn force_remove_owner(&self, owner: OwnerId) -> Result<()> {
        let mut state = self.state.lock();
        validate_no_foreign_children(&state, owner)?;
        for record in state
            .records
            .values()
            .filter(|record| record.owner == owner)
        {
            node_operations(&record.vnode)?.force_revoke();
        }
        remove_owned_records(self, &mut state, owner)
    }

    fn create_node(
        self: &Arc<Self>,
        owner: OwnerId,
        parent: DevNodeId,
        name: &[u8],
        kind: VnodeKind,
        mode: u16,
        data: DevtempfsData,
    ) -> Result<DevNodeId> {
        path::validate_leaf_name(name)?;
        let id = DevNodeId::new(self.next_node.fetch_add(1, Ordering::Relaxed));
        assert!(id.get() != 0, "devtempfs: node identifier wrapped");
        let vnode = self.make_node(id, owner, kind, mode, data);

        let mut state = self.state.lock();
        let parent_record = state.records.get_mut(&parent).ok_or(Error::NotFound)?;
        if parent_record.vnode.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        if parent_record.children.contains_key(name) {
            return Err(Error::AlreadyExists);
        }
        let name = Box::<[u8]>::from(name);
        parent_record.children.insert(name, id);
        if kind == VnodeKind::Directory {
            node_operations(&parent_record.vnode)?
                .links
                .fetch_add(1, Ordering::AcqRel);
        }
        state.records.insert(
            id,
            DevRecord {
                owner,
                parent,
                vnode,
                children: BTreeMap::new(),
            },
        );
        self.used_nodes.fetch_add(1, Ordering::AcqRel);
        Ok(id)
    }

    fn make_node(
        self: &Arc<Self>,
        id: DevNodeId,
        owner: OwnerId,
        kind: VnodeKind,
        mode: u16,
        data: DevtempfsData,
    ) -> Vnode {
        let now = clock::monotonic_ns();
        let links = if kind == VnodeKind::Directory { 2 } else { 1 };
        Vnode::new(
            VnodeKey {
                filesystem: self.id,
                node: NodeId::new(id.get()),
            },
            kind,
            Box::new(DevtempfsNode {
                filesystem: Arc::downgrade(self),
                id,
                owner,
                kind,
                mode: AtomicU16::new(mode),
                links: AtomicU64::new(links),
                accessed_ns: AtomicU64::new(now),
                modified_ns: AtomicU64::new(now),
                changed_ns: AtomicU64::new(now),
                lifecycle: AtomicU64::new(0),
                data,
            }),
        )
    }
}

fn remove_owned_records(
    filesystem: &Devtempfs,
    state: &mut DevtempfsState,
    owner: OwnerId,
) -> Result<()> {
    let mut nodes: Vec<_> = state
        .records
        .iter()
        .filter(|(_, record)| record.owner == owner)
        .map(|(id, _)| *id)
        .collect();
    nodes.sort_unstable_by_key(|node| core::cmp::Reverse(node.get()));
    for node in nodes {
        remove_record(state, node)?;
        filesystem.used_nodes.fetch_sub(1, Ordering::AcqRel);
    }
    Ok(())
}

impl FileSystem for Devtempfs {
    fn id(&self) -> FilesystemId {
        self.id
    }

    fn name(&self) -> &'static str {
        DEVTEMPFS_NAME
    }

    fn root(&self) -> Vnode {
        self.root
            .get()
            .cloned()
            .expect("devtempfs: root not initialized")
    }

    fn statfs(&self) -> StatFs {
        StatFs {
            total_bytes: 0,
            used_bytes: 0,
            total_nodes: u64::MAX,
            used_nodes: self.used_nodes.load(Ordering::Acquire),
        }
    }
}

impl DevtempfsNode {
    fn filesystem(&self) -> Result<Arc<Devtempfs>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }

    fn device_operations(&self) -> Result<&Arc<dyn DeviceNodeOps>> {
        match &self.data {
            DevtempfsData::Device(operations) => Ok(operations),
            DevtempfsData::Directory => Err(Error::IsDirectory),
        }
    }

    fn touch_accessed(&self) {
        self.accessed_ns
            .store(clock::monotonic_ns(), Ordering::Release);
    }

    fn touch_modified(&self) {
        let now = clock::monotonic_ns();
        self.modified_ns.store(now, Ordering::Release);
        self.changed_ns.store(now, Ordering::Release);
    }

    fn ensure_live(&self) -> Result<()> {
        if self.lifecycle.load(Ordering::Acquire) & NODE_REVOKED != 0 {
            return Err(Error::NotFound);
        }
        Ok(())
    }

    fn begin_activity(&self) -> Result<NodeActivity<'_>> {
        loop {
            let state = self.lifecycle.load(Ordering::Acquire);
            if state & NODE_REVOKED != 0 || state & NODE_ACTIVE_MASK == NODE_ACTIVE_MASK {
                return Err(Error::Busy);
            }
            if self
                .lifecycle
                .compare_exchange_weak(state, state + 1, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return Ok(NodeActivity {
                    lifecycle: &self.lifecycle,
                });
            }
        }
    }

    fn mark_open(&self) -> Result<()> {
        loop {
            let state = self.lifecycle.load(Ordering::Acquire);
            if state & NODE_REVOKED != 0 || state & NODE_OPEN_MASK == NODE_OPEN_MASK {
                return Err(Error::Busy);
            }
            if self
                .lifecycle
                .compare_exchange_weak(
                    state,
                    state + NODE_OPEN_ONE,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                return Ok(());
            }
        }
    }

    fn mark_closed(&self) {
        let previous = self.lifecycle.fetch_sub(NODE_OPEN_ONE, Ordering::AcqRel);
        assert!(
            previous & NODE_OPEN_MASK != 0,
            "devtempfs: open count underflow"
        );
    }

    fn is_idle(&self) -> bool {
        self.lifecycle.load(Ordering::Acquire) == 0
    }

    fn try_revoke(&self) -> Result<()> {
        self.lifecycle
            .compare_exchange(0, NODE_REVOKED, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| Error::Busy)
    }

    fn restore(&self) {
        let _ =
            self.lifecycle
                .compare_exchange(NODE_REVOKED, 0, Ordering::Release, Ordering::Relaxed);
    }

    fn force_revoke(&self) {
        self.lifecycle.fetch_or(NODE_REVOKED, Ordering::AcqRel);
    }
}

impl VnodeOps for DevtempfsNode {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn initial_offset(&self, _vnode: &Vnode, file_context: usize, flags: u32) -> Result<u64> {
        match &self.data {
            DevtempfsData::Directory => {
                self.ensure_live()?;
                Ok(0)
            }
            DevtempfsData::Device(operations) => {
                let _activity = self.begin_activity()?;
                operations.initial_offset(file_context, flags)
            }
        }
    }

    fn open(&self, _vnode: &Vnode, flags: u32) -> Result<usize> {
        if matches!(&self.data, DevtempfsData::Directory) {
            self.mark_open()?;
            return Ok(0);
        }
        let _activity = self.begin_activity()?;
        let operations = self.device_operations()?;
        let file_context = operations.open(flags)?;
        if let Err(error) = self.mark_open() {
            operations.close(file_context, flags);
            return Err(error);
        }
        Ok(file_context)
    }

    fn close(&self, _vnode: &Vnode, file_context: usize, flags: u32) {
        if matches!(&self.data, DevtempfsData::Directory) {
            self.mark_closed();
            return;
        }
        let Ok(_activity) = self.begin_activity() else {
            self.mark_closed();
            return;
        };
        let Ok(operations) = self.device_operations() else {
            self.mark_closed();
            return;
        };
        operations.close(file_context, flags);
        self.mark_closed();
    }

    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        self.ensure_live()?;
        let size = match &self.data {
            DevtempfsData::Directory => 0,
            DevtempfsData::Device(operations) => {
                let _activity = self.begin_activity()?;
                operations.size()
            }
        };
        Ok(VnodeAttr {
            key: vnode.key(),
            kind: self.kind,
            size,
            links: self.links.load(Ordering::Acquire),
            mode: self.mode.load(Ordering::Acquire),
            accessed_ns: self.accessed_ns.load(Ordering::Acquire),
            modified_ns: self.modified_ns.load(Ordering::Acquire),
            changed_ns: self.changed_ns.load(Ordering::Acquire),
        })
    }

    fn setattr(&self, _vnode: &Vnode, attr: SetAttr) -> Result<()> {
        self.ensure_live()?;
        if attr.size.is_some() {
            return Err(Error::Unsupported);
        }
        if let Some(mode) = attr.mode {
            self.mode.store(mode, Ordering::Release);
            self.changed_ns
                .store(clock::monotonic_ns(), Ordering::Release);
        }
        Ok(())
    }

    fn lookup(&self, directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        self.ensure_live()?;
        if self.kind != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        if name == b"." {
            return Ok(directory.clone());
        }
        if name == b".." {
            return self.parent(directory);
        }
        path::validate_leaf_name(name)?;
        let filesystem = self.filesystem()?;
        let state = filesystem.state.lock();
        let record = state.records.get(&self.id).ok_or(Error::NotFound)?;
        let child = record.children.get(name).ok_or(Error::NotFound)?;
        let vnode = state
            .records
            .get(child)
            .map(|record| record.vnode.clone())
            .ok_or(Error::NotFound)?;
        self.touch_accessed();
        Ok(vnode)
    }

    fn parent(&self, directory: &Vnode) -> Result<Vnode> {
        self.ensure_live()?;
        if self.kind != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        let filesystem = self.filesystem()?;
        let state = filesystem.state.lock();
        let record = state.records.get(&self.id).ok_or(Error::NotFound)?;
        Ok(state
            .records
            .get(&record.parent)
            .map(|parent| parent.vnode.clone())
            .unwrap_or_else(|| directory.clone()))
    }

    fn read_at(&self, _vnode: &Vnode, offset: u64, buffer: &mut [u8]) -> Result<usize> {
        self.read_at_with_flags(_vnode, 0, offset, buffer, 0)
    }

    fn read_at_with_flags(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        offset: u64,
        buffer: &mut [u8],
        flags: u32,
    ) -> Result<usize> {
        let _activity = self.begin_activity()?;
        let read =
            self.device_operations()?
                .read_at_with_flags(file_context, offset, buffer, flags)?;
        if read > buffer.len() {
            return Err(Error::Io);
        }
        self.touch_accessed();
        Ok(read)
    }

    fn write_at(&self, _vnode: &Vnode, offset: u64, buffer: &[u8]) -> Result<usize> {
        self.write_at_with_flags(_vnode, 0, offset, buffer, 0)
    }

    fn write_at_with_flags(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        offset: u64,
        buffer: &[u8],
        flags: u32,
    ) -> Result<usize> {
        let _activity = self.begin_activity()?;
        let written =
            self.device_operations()?
                .write_at_with_flags(file_context, offset, buffer, flags)?;
        if written > buffer.len() {
            return Err(Error::Io);
        }
        self.touch_modified();
        Ok(written)
    }

    fn poll(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        offset: u64,
        events: PollEvents,
        flags: u32,
    ) -> Result<PollEvents> {
        match &self.data {
            DevtempfsData::Directory => {
                self.ensure_live()?;
                let flags = crate::fs::OpenFlags::from_bits_retain(flags);
                Ok(if flags.contains(crate::fs::OpenFlags::READ) {
                    events & (PollEvents::IN | PollEvents::RDNORM)
                } else {
                    PollEvents::empty()
                })
            }
            DevtempfsData::Device(operations) => {
                let _activity = self.begin_activity()?;
                operations.poll(file_context, offset, events, flags)
            }
        }
    }

    fn poll_events<'a>(
        &'a self,
        _vnode: &Vnode,
        file_context: usize,
        events: PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        match &self.data {
            DevtempfsData::Directory => false,
            DevtempfsData::Device(operations) => {
                let Ok(_activity) = self.begin_activity() else {
                    return false;
                };
                operations.poll_events(file_context, events, output)
            }
        }
    }

    fn terminal_state(&self, _vnode: &Vnode) -> Option<TerminalState> {
        match &self.data {
            DevtempfsData::Directory => None,
            DevtempfsData::Device(operations) => operations.terminal_state(),
        }
    }

    fn readdir(
        &self,
        _directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        self.ensure_live()?;
        if self.kind != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        let filesystem = self.filesystem()?;
        let state = filesystem.state.lock();
        let record = state.records.get(&self.id).ok_or(Error::NotFound)?;
        let mut children: Vec<_> = record
            .children
            .iter()
            .filter_map(|(name, id)| {
                if id.get() <= cursor {
                    return None;
                }
                state.records.get(id).map(|child| {
                    (
                        id.get(),
                        DirEntry {
                            name: name.clone(),
                            key: child.vnode.key(),
                            kind: child.vnode.kind(),
                        },
                    )
                })
            })
            .collect();
        children.sort_unstable_by_key(|(id, _)| *id);
        children.truncate(maximum);
        let next = children.last().map_or(cursor, |(id, _)| *id);
        Ok((children.into_iter().map(|(_, entry)| entry).collect(), next))
    }

    fn fsync(&self, _vnode: &Vnode) -> Result<()> {
        match &self.data {
            DevtempfsData::Directory => self.ensure_live(),
            DevtempfsData::Device(operations) => {
                let _activity = self.begin_activity()?;
                operations.sync()
            }
        }
    }

    fn ioctl(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        context: IoctlContext,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> Result<u64> {
        let _activity = self.begin_activity()?;
        self.device_operations()?
            .ioctl(file_context, context, request, value, argument)
    }
}

/// Returns the mounted global devtempfs instance.
pub fn global() -> Result<&'static Arc<Devtempfs>> {
    DEVTEMPFS.get().ok_or(Error::Io)
}

pub(crate) fn mount_global() -> Result<()> {
    let filesystem = Devtempfs::new()?;
    match super::create_dir("/dev", 0o755) {
        Ok(_) | Err(Error::AlreadyExists) => {}
        Err(error) => return Err(error),
    }
    let mounted: Arc<dyn FileSystem> = filesystem.clone();
    super::mount("/dev", mounted)?;
    DEVTEMPFS.call_once(|| filesystem);
    Ok(())
}

fn node_operations(vnode: &Vnode) -> Result<&DevtempfsNode> {
    vnode
        .operations_as::<DevtempfsNode>()
        .ok_or(Error::CrossDevice)
}

fn remove_record(state: &mut DevtempfsState, node: DevNodeId) -> Result<()> {
    let record = state.records.remove(&node).ok_or(Error::NotFound)?;
    if let Some(parent) = state.records.get_mut(&record.parent) {
        parent.children.retain(|_, child| *child != node);
        if record.vnode.kind() == VnodeKind::Directory {
            let operations = node_operations(&parent.vnode)?;
            let previous = operations.links.fetch_sub(1, Ordering::AcqRel);
            assert!(
                previous > 2,
                "devtempfs: parent directory link count underflow"
            );
        }
    }
    Ok(())
}

fn validate_owner_removal(state: &DevtempfsState, owner: OwnerId) -> Result<()> {
    validate_no_foreign_children(state, owner)?;
    state
        .records
        .values()
        .filter(|record| record.owner == owner)
        .try_for_each(|record| {
            node_operations(&record.vnode)?
                .is_idle()
                .then_some(())
                .ok_or(Error::Busy)
        })
}

fn revoke_owner_nodes(state: &DevtempfsState, owner: OwnerId) -> Result<()> {
    validate_no_foreign_children(state, owner)?;
    let mut revoked: Vec<&DevtempfsNode> = Vec::new();
    for record in state
        .records
        .values()
        .filter(|record| record.owner == owner)
    {
        let node = node_operations(&record.vnode)?;
        if let Err(error) = node.try_revoke() {
            for node in revoked {
                node.restore();
            }
            return Err(error);
        }
        revoked.push(node);
    }
    Ok(())
}

fn validate_no_foreign_children(state: &DevtempfsState, owner: OwnerId) -> Result<()> {
    state
        .records
        .values()
        .filter(|record| record.owner == owner)
        .try_for_each(|record| {
            if record.children.values().any(|child| {
                state
                    .records
                    .get(child)
                    .is_some_and(|child| child.owner != owner)
            }) {
                return Err(Error::Busy);
            }
            Ok(())
        })
}

