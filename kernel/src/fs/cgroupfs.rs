//! Filesystem interface to native process containment, without resource controllers.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    format,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    any::Any,
    sync::atomic::{AtomicUsize, Ordering},
};

use super::{
    Error, OpenFlags, Result,
    vnode::{
        CreateKind, DirEntry, FileSystem, FilesystemId, NodeId, PollEvents, SetAttr, StatFs, Vnode,
        VnodeAttr, VnodeCache, VnodeKey, VnodeKind, VnodeOps,
    },
};
use crate::{
    mem::{IoSink, IoSource},
    proc::cgroup::{self, Group},
    sys::{
        event::EventSource,
        sync::{Mutex, Once},
    },
};

static FILESYSTEM: Once<Arc<Cgroupfs>> = Once::new();

/// All mounts expose the same hierarchy and vnode identities.
pub struct Cgroupfs {
    id: FilesystemId,
    root: Once<Vnode>,
    nodes: VnodeCache,
    next_reader: AtomicUsize,
    readers: Mutex<BTreeMap<usize, Arc<Mutex<Reader>>>>,
}

struct Reader {
    bytes: Vec<u8>,
    generation: u64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u64)]
enum Kind {
    Directory,
    Procs,
    Events,
    Controllers,
    SubtreeControl,
    Type,
    Kill,
}

struct CgroupNode {
    filesystem: Weak<Cgroupfs>,
    group: Arc<Group>,
    kind: Kind,
}

impl Kind {
    fn vnode_kind(self) -> VnodeKind {
        if self == Self::Directory {
            VnodeKind::Directory
        } else {
            VnodeKind::Regular
        }
    }
    fn writable(self) -> bool {
        matches!(
            self,
            Self::Procs | Self::SubtreeControl | Self::Type | Self::Kill
        )
    }
}

fn controls(group: &Group) -> impl Iterator<Item = (&'static [u8], Kind)> {
    [
        (b"cgroup.procs".as_slice(), Kind::Procs),
        (b"cgroup.controllers", Kind::Controllers),
        (b"cgroup.subtree_control", Kind::SubtreeControl),
        (b"cgroup.type", Kind::Type),
        (b"cgroup.events", Kind::Events),
        (b"cgroup.kill", Kind::Kill),
    ]
    .into_iter()
    .filter(move |(_, kind)| !group.is_root() || !matches!(kind, Kind::Events | Kind::Kill))
}

fn identity(group: u64, kind: Kind) -> u64 {
    (group << 4) | kind as u64
}

impl Cgroupfs {
    /// Returns the persistent hierarchy, including memberships predating mount.
    pub fn global() -> Result<Arc<Self>> {
        if let Some(filesystem) = FILESYSTEM.get() {
            return Ok(filesystem.clone());
        }
        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            root: Once::new(),
            nodes: VnodeCache::new(),
            next_reader: AtomicUsize::new(1),
            readers: Mutex::new(BTreeMap::new()),
        });
        let root = filesystem.node(cgroup::root(), Kind::Directory)?;
        filesystem.root.call_once(|| root);
        Ok(FILESYSTEM.call_once(|| filesystem).clone())
    }

    fn node(self: &Arc<Self>, group: Arc<Group>, kind: Kind) -> Result<Vnode> {
        let id = identity(group.id, kind);
        self.nodes.get_or_create(id, || {
            Ok(Vnode::new(
                VnodeKey {
                    filesystem: self.id,
                    node: NodeId::new(id),
                },
                kind.vnode_kind(),
                Box::new(CgroupNode {
                    filesystem: Arc::downgrade(self),
                    group,
                    kind,
                }),
            ))
        })
    }
}

impl FileSystem for Cgroupfs {
    fn id(&self) -> FilesystemId {
        self.id
    }
    fn name(&self) -> &str {
        "cgroup2"
    }
    fn root(&self) -> Vnode {
        self.root
            .get()
            .cloned()
            .expect("cgroupfs: root initialized")
    }
    fn statfs(&self) -> StatFs {
        StatFs {
            filesystem_type: 0x63677270,
            block_size: crate::mem::PAGE_SIZE,
            ..StatFs::default()
        }
    }
}

/// Called after membership changes, outside the cgroup tree lock.
pub(crate) fn notify_population(group: u64) {
    if let Some(filesystem) = FILESYSTEM.get() {
        crate::proc::inotify::notify(
            VnodeKey {
                filesystem: filesystem.id,
                node: NodeId::new(identity(group, Kind::Events)),
            },
            crate::proc::inotify::IN_MODIFY,
            0,
            None,
        );
    }
}

impl CgroupNode {
    fn filesystem(&self) -> Result<Arc<Cgroupfs>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }
    fn reader(&self, context: usize) -> Result<Arc<Mutex<Reader>>> {
        self.filesystem()?
            .readers
            .lock()
            .get(&context)
            .cloned()
            .ok_or(Error::BadFileDescriptor)
    }
    fn contents(&self) -> Result<Vec<u8>> {
        self.group.ensure_exists()?;
        match self.kind {
            Kind::Procs => cgroup::members(self.group.id),
            Kind::Events => Ok(format!(
                "populated {}\n",
                u8::from(cgroup::population(self.group.id)?)
            )
            .into_bytes()),
            Kind::Controllers | Kind::SubtreeControl => Ok(Vec::new()),
            Kind::Type => Ok(b"domain\n".to_vec()),
            Kind::Directory => Err(Error::IsDirectory),
            Kind::Kill => Err(Error::PermissionDenied),
        }
    }
}

impl VnodeOps for CgroupNode {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn cache_names(&self) -> bool {
        false
    }
    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        self.group.ensure_exists()?;
        Ok(VnodeAttr {
            key: vnode.key(),
            kind: self.kind.vnode_kind(),
            size: 0,
            links: if self.kind == Kind::Directory { 2 } else { 1 },
            mode: if self.kind == Kind::Directory {
                cgroup::mode(self.group.id)?
            } else if self.kind == Kind::Kill {
                0o200
            } else if self.kind.writable() {
                0o644
            } else {
                0o444
            },
            accessed_ns: 0,
            modified_ns: 0,
            changed_ns: 0,
        })
    }

    fn setattr(&self, _vnode: &Vnode, attr: SetAttr) -> Result<()> {
        self.group.ensure_exists()?;
        if attr.size.is_some() {
            return Err(Error::Unsupported);
        }
        if let Some(mode) = attr.mode {
            if self.kind != Kind::Directory {
                return Err(Error::Unsupported);
            }
            cgroup::set_mode(self.group.id, mode)?;
        }
        Ok(())
    }

    fn open(&self, _vnode: &Vnode, flags: u32) -> Result<usize> {
        self.group.ensure_exists()?;
        let flags = OpenFlags::from_bits_retain(flags);
        if flags.contains(OpenFlags::WRITE) && !self.kind.writable()
            || flags.contains(OpenFlags::READ) && self.kind == Kind::Kill
        {
            return Err(Error::PermissionDenied);
        }
        let filesystem = self.filesystem()?;
        let context = filesystem
            .next_reader
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .map_err(|_| Error::NoSpace)?;
        filesystem.readers.lock().insert(
            context,
            Arc::new(Mutex::new(Reader {
                bytes: Vec::new(),
                generation: self.group.generation.load(Ordering::Acquire),
            })),
        );
        Ok(context)
    }

    fn close(&self, _vnode: &Vnode, context: usize, _flags: u32) {
        if let Ok(filesystem) = self.filesystem() {
            filesystem.readers.lock().remove(&context);
        }
    }

    fn lookup(&self, _directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        self.group.ensure_exists()?;
        if self.kind != Kind::Directory {
            return Err(Error::NotDirectory);
        }
        let filesystem = self.filesystem()?;
        if let Some((_, kind)) = controls(&self.group).find(|(control, _)| *control == name) {
            return filesystem.node(self.group.clone(), kind);
        }
        filesystem.node(cgroup::lookup(self.group.id, name)?, Kind::Directory)
    }

    fn parent(&self, _directory: &Vnode) -> Result<Vnode> {
        if self.kind != Kind::Directory {
            return Err(Error::NotDirectory);
        }
        self.filesystem()?
            .node(cgroup::parent(self.group.id)?, Kind::Directory)
    }

    fn create(&self, directory: &Vnode, name: &[u8], kind: CreateKind, mode: u16) -> Result<Vnode> {
        if self.kind != Kind::Directory {
            return Err(Error::NotDirectory);
        }
        if self.lookup(directory, name).is_ok() {
            return Err(Error::AlreadyExists);
        }
        if kind != CreateKind::Directory || name.starts_with(b"cgroup.") {
            return Err(Error::PermissionDenied);
        }
        self.filesystem()?
            .node(cgroup::create(self.group.id, name, mode)?, Kind::Directory)
    }

    fn unlink(&self, _directory: &Vnode, name: &[u8], directory_only: bool) -> Result<()> {
        if self.kind != Kind::Directory {
            return Err(Error::NotDirectory);
        }
        if !directory_only {
            return Err(Error::PermissionDenied);
        }
        if controls(&self.group).any(|(control, _)| control == name) {
            return Err(Error::NotDirectory);
        }
        cgroup::remove(self.group.id, name)
    }

    fn readdir(
        &self,
        _directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        if self.kind != Kind::Directory {
            return Err(Error::NotDirectory);
        }
        self.group.ensure_exists()?;
        let mut entries = alloc::vec![
            (b".".to_vec(), self.group.clone(), Kind::Directory, 1),
            (
                b"..".to_vec(),
                cgroup::parent(self.group.id)?,
                Kind::Directory,
                2
            ),
        ];
        for (name, kind) in controls(&self.group) {
            entries.push((name.to_vec(), self.group.clone(), kind, kind as u64 + 3));
        }
        // Group IDs are monotonic and never reused, so cursors remain stable
        // across sibling creation/removal even though the name index is sorted.
        let mut children = cgroup::children(self.group.id)?;
        children.sort_unstable_by_key(|(_, group)| group.id);
        for (name, group) in children {
            let cookie = group.id + 16;
            entries.push((name, group, Kind::Directory, cookie));
        }
        entries.sort_unstable_by_key(|(_, _, _, cookie)| *cookie);
        let mut output = Vec::new();
        let mut next = cursor;
        for (name, group, kind, cookie) in entries {
            if cookie <= cursor {
                continue;
            }
            if output.len() == maximum {
                break;
            }
            output.push(DirEntry {
                name: name.into(),
                key: VnodeKey {
                    filesystem: self.filesystem()?.id,
                    node: NodeId::new(identity(group.id, kind)),
                },
                kind: kind.vnode_kind(),
                offset: cookie,
            });
            next = cookie;
        }
        Ok((output, next))
    }

    fn read_at_with_flags(
        &self,
        _vnode: &Vnode,
        context: usize,
        offset: u64,
        sink: &mut IoSink<'_>,
        _flags: u32,
    ) -> Result<usize> {
        self.group.ensure_exists()?;
        let reader = self.reader(context)?;
        let mut reader = reader.lock();
        if offset == 0 {
            // Capture the generation before sampling so a concurrent change
            // may cause an extra notification, but can never be acknowledged
            // by a read that returned older contents.
            let generation = self.group.generation.load(Ordering::Acquire);
            reader.bytes = self.contents()?;
            reader.generation = generation;
        }
        let start = usize::try_from(offset)
            .map_err(|_| Error::InvalidArgument)?
            .min(reader.bytes.len());
        let count = sink.len().min(reader.bytes.len() - start);
        sink.store(0, &reader.bytes[start..start + count])
            .map_err(|_| Error::InvalidArgument)?;
        Ok(count)
    }

    fn write_at(&self, _vnode: &Vnode, _offset: u64, source: &IoSource<'_>) -> Result<usize> {
        self.group.ensure_exists()?;
        if !self.kind.writable() {
            return Err(Error::PermissionDenied);
        }
        if source.len() > 4096 {
            return Err(Error::InvalidArgument);
        }
        let mut bytes = alloc::vec![0; source.len()];
        source
            .load(0, &mut bytes)
            .map_err(|_| Error::InvalidArgument)?;
        let bytes = bytes.trim_ascii();
        match self.kind {
            Kind::Procs => {
                if bytes.is_empty() {
                    return Err(Error::InvalidArgument);
                }
                let pid = bytes.iter().try_fold(0usize, |pid, digit| {
                    if !digit.is_ascii_digit() {
                        return Err(Error::InvalidArgument);
                    }
                    pid.checked_mul(10)
                        .and_then(|pid| pid.checked_add(usize::from(digit - b'0')))
                        .ok_or(Error::InvalidArgument)
                })?;
                let pid = if pid == 0 {
                    crate::proc::current().ok_or(Error::NotFound)?.pid()
                } else {
                    pid
                };
                cgroup::migrate(pid, self.group.id)?;
            }
            Kind::Kill if bytes == b"1" => cgroup::kill(self.group.id)?,
            Kind::SubtreeControl if bytes.is_empty() => (),
            // No controllers or threaded domains are implemented. Never
            // accept a request that claims to have enabled one.
            Kind::SubtreeControl | Kind::Type => return Err(Error::Unsupported),
            _ => return Err(Error::InvalidArgument),
        }
        Ok(source.len())
    }

    fn truncate(&self, _vnode: &Vnode, size: u64) -> Result<()> {
        self.group.ensure_exists()?;
        if self.kind.writable() && size == 0 {
            Ok(())
        } else {
            Err(Error::Unsupported)
        }
    }

    fn poll(
        &self,
        _vnode: &Vnode,
        context: usize,
        _offset: u64,
        requested: PollEvents,
        flags: u32,
    ) -> Result<PollEvents> {
        if self.group.ensure_exists().is_err() {
            return Ok(PollEvents::ERR | PollEvents::HUP);
        }
        let mut ready = PollEvents::empty();
        let flags = OpenFlags::from_bits_retain(flags);
        if flags.contains(OpenFlags::READ) {
            ready |= requested & (PollEvents::IN | PollEvents::RDNORM);
        }
        if flags.contains(OpenFlags::WRITE) {
            ready |= requested & (PollEvents::OUT | PollEvents::WRNORM);
        }
        if self.kind == Kind::Events
            && self.reader(context)?.lock().generation
                != self.group.generation.load(Ordering::Acquire)
        {
            ready |= (requested & PollEvents::PRI) | PollEvents::ERR;
        }
        Ok(ready)
    }

    fn poll_events<'a>(
        &'a self,
        _vnode: &Vnode,
        _context: usize,
        _events: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        output.push(self.group.event.clone().into());
        true
    }
}
