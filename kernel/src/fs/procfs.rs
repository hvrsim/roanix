//! Read-only views of live process and mount-namespace state.
//!
//! Directory names are deliberately uncached: process exit, descriptor reuse,
//! and fork happen outside VFS namespace mutations. Regular files snapshot on
//! open, so short reads cannot splice together different registry generations.

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
    Error, OpenFlags, PollEvents, Result,
    vnode::{
        CreateKind, DirEntry, FileSystem, FilesystemId, NodeId, SetAttr, StatFs, Vnode, VnodeAttr,
        VnodeCache, VnodeKey, VnodeKind, VnodeOps,
    },
};
use crate::{
    mem::{IoSink, IoSource},
    proc::{self, Descriptor},
    sys::{
        event::EventSource,
        sync::{Mutex, Once},
    },
};

/// A procfs instance in the global process and mount namespaces.
pub struct Procfs {
    id: FilesystemId,
    root: Once<Vnode>,
    nodes: VnodeCache,
    next_reader: AtomicUsize,
    readers: Mutex<BTreeMap<usize, Arc<[u8]>>>,
}

#[derive(Clone, Copy)]
enum Node {
    Root,
    SelfLink,
    MountsLink,
    Filesystems,
    Cgroups,
    Cmdline,
    Process(usize),
    Executable(usize),
    Status(usize),
    Cgroup(usize),
    MountInfo(usize),
    Mounts(usize),
    Fds(usize),
    FdInfo(usize),
    FdLink(usize, i32),
    DescriptorInfo(usize, i32),
}

struct ProcNode {
    filesystem: Weak<Procfs>,
    node: Node,
}

impl Node {
    fn identity(self) -> Result<u64> {
        let (pid, tag) = match self {
            Self::Root => (0, 1),
            Self::SelfLink => (0, 2),
            Self::MountsLink => (0, 3),
            Self::Filesystems => (0, 4),
            Self::Cgroups => (0, 5),
            Self::Cmdline => (0, 6),
            Self::Process(pid) => (pid, 1),
            Self::Executable(pid) => (pid, 8),
            Self::Status(pid) => (pid, 2),
            Self::Cgroup(pid) => (pid, 7),
            Self::MountInfo(pid) => (pid, 3),
            Self::Mounts(pid) => (pid, 4),
            Self::Fds(pid) => (pid, 5),
            Self::FdInfo(pid) => (pid, 6),
            Self::FdLink(pid, fd) => (pid, 0x10000 + fd as u64),
            Self::DescriptorInfo(pid, fd) => (pid, 0x20000 + fd as u64),
        };
        let pid = u32::try_from(pid).map_err(|_| Error::NoSpace)?;
        Ok((u64::from(pid) << 32) | tag)
    }

    fn kind(self) -> VnodeKind {
        match self {
            Self::Root | Self::Process(_) | Self::Fds(_) | Self::FdInfo(_) => VnodeKind::Directory,
            Self::SelfLink | Self::MountsLink | Self::Executable(_) | Self::FdLink(..) => {
                VnodeKind::Symlink
            }
            _ => VnodeKind::Regular,
        }
    }

    fn process(self) -> Option<usize> {
        match self {
            Self::Process(pid)
            | Self::Executable(pid)
            | Self::Status(pid)
            | Self::Cgroup(pid)
            | Self::MountInfo(pid)
            | Self::Mounts(pid)
            | Self::Fds(pid)
            | Self::FdInfo(pid)
            | Self::FdLink(pid, _)
            | Self::DescriptorInfo(pid, _) => Some(pid),
            _ => None,
        }
    }
}

impl Procfs {
    /// Creates a view of the current process registry; it does not own processes.
    pub fn new() -> Result<Arc<Self>> {
        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            root: Once::new(),
            nodes: VnodeCache::new(),
            next_reader: AtomicUsize::new(1),
            readers: Mutex::new(BTreeMap::new()),
        });
        let root = filesystem.node(Node::Root)?;
        filesystem.root.call_once(|| root);
        Ok(filesystem)
    }

    fn node(self: &Arc<Self>, node: Node) -> Result<Vnode> {
        let id = node.identity()?;
        self.nodes.get_or_create(id, || {
            Ok(Vnode::new(
                VnodeKey {
                    filesystem: self.id,
                    node: NodeId::new(id),
                },
                node.kind(),
                Box::new(ProcNode {
                    filesystem: Arc::downgrade(self),
                    node,
                }),
            ))
        })
    }
}

impl FileSystem for Procfs {
    fn id(&self) -> FilesystemId {
        self.id
    }
    fn name(&self) -> &str {
        "proc"
    }
    fn root(&self) -> Vnode {
        self.root.get().cloned().expect("procfs: root initialized")
    }
    fn read_only(&self) -> bool {
        true
    }
    fn statfs(&self) -> StatFs {
        StatFs {
            filesystem_type: 0x9fa0,
            block_size: crate::mem::PAGE_SIZE,
            ..StatFs::default()
        }
    }
}

impl ProcNode {
    fn filesystem(&self) -> Result<Arc<Procfs>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }

    fn ensure_exists(&self) -> Result<()> {
        if let Some(pid) = self.node.process() {
            let process = proc::find(pid).ok_or(Error::NotFound)?;
            if let Node::FdLink(_, fd) | Node::DescriptorInfo(_, fd) = self.node {
                process.descriptor(fd).ok_or(Error::NotFound)?;
            }
            if matches!(self.node, Node::Executable(_)) {
                process.executable().ok_or(Error::NotFound)?;
            }
        }
        Ok(())
    }

    fn contents(&self) -> Result<Vec<u8>> {
        self.ensure_exists()?;
        match self.node {
            Node::Cmdline => {
                let mut bytes = crate::sys::cmdline::get().to_vec();
                bytes.push(b'\n');
                Ok(bytes)
            }
            Node::Filesystems => Ok(
                b"nodev\ttmpfs\nnodev\tdevfs\nnodev\tproc\nnodev\tsysfs\nnodev\tcgroup2\n\text4\n\tvfat\n"
                    .to_vec(),
            ),
            // The native kernel currently has no cgroup controllers.
            Node::Cgroups => Ok(b"#subsys_name\thierarchy\tnum_cgroups\tenabled\n".to_vec()),
            Node::Cgroup(pid) => proc::cgroup::process_path(pid),
            Node::MountInfo(_) => super::vfs::proc_mounts(true),
            Node::Mounts(_) => super::vfs::proc_mounts(false),
            Node::Status(pid) => {
                let process = proc::find(pid).ok_or(Error::NotFound)?;
                // Only fields backed by native process state are exposed. In
                // particular, do not invent Linux capability or namespace state.
                Ok(format!("Tgid:\t{pid}\nPid:\t{pid}\nPPid:\t{}\nUid:\t0\t0\t0\t0\nGid:\t0\t0\t0\t0\nThreads:\t{}\n",
                    process.parent_pid(), process.thread_count()).into_bytes())
            }
            Node::DescriptorInfo(pid, fd) => {
                let process = proc::find(pid).ok_or(Error::NotFound)?;
                let (descriptor, close_on_exec) =
                    process.descriptor_info(fd).ok_or(Error::NotFound)?;
                let flags = super::syscall::descriptor_open_flags(&descriptor, close_on_exec);
                let mut bytes = format!(
                    "pos:\t{}\nflags:\t{flags:o}\n",
                    match &descriptor {
                        Descriptor::File(file) => file.position(),
                        _ => 0,
                    }
                )
                .into_bytes();
                if let Descriptor::File(file) = descriptor {
                    bytes.extend_from_slice(
                        format!(
                            "mnt_id:\t{}\nino:\t{}\n",
                            file.path_anchor().mount_id(),
                            file.vnode().key().node.get()
                        )
                        .as_bytes(),
                    );
                }
                Ok(bytes)
            }
            _ => Err(Error::InvalidArgument),
        }
    }

    fn children(&self) -> Result<Vec<(Vec<u8>, Node, u64)>> {
        self.ensure_exists()?;
        let mut entries = Vec::new();
        match self.node {
            Node::Root => {
                for (name, node) in [
                    (b"cmdline".as_slice(), Node::Cmdline),
                    (b"self".as_slice(), Node::SelfLink),
                    (b"mounts", Node::MountsLink),
                    (b"filesystems", Node::Filesystems),
                    (b"cgroups", Node::Cgroups),
                ] {
                    entries.push((name.to_vec(), node, entries.len() as u64 + 3));
                }
                for pid in proc::process_ids() {
                    entries.push((
                        format!("{pid}").into_bytes(),
                        Node::Process(pid),
                        pid as u64 + 32,
                    ));
                }
            }
            Node::Process(pid) => {
                for (name, node) in [
                    (b"status".as_slice(), Node::Status(pid)),
                    (b"cgroup", Node::Cgroup(pid)),
                    (b"mountinfo", Node::MountInfo(pid)),
                    (b"mounts", Node::Mounts(pid)),
                    (b"fd", Node::Fds(pid)),
                    (b"fdinfo", Node::FdInfo(pid)),
                ] {
                    entries.push((name.to_vec(), node, entries.len() as u64 + 3));
                }
                if proc::find(pid)
                    .and_then(|process| process.executable())
                    .is_some()
                {
                    entries.push((
                        b"exe".to_vec(),
                        Node::Executable(pid),
                        entries.len() as u64 + 3,
                    ));
                }
            }
            Node::Fds(pid) | Node::FdInfo(pid) => {
                let process = proc::find(pid).ok_or(Error::NotFound)?;
                for fd in process.descriptor_numbers() {
                    let node = if matches!(self.node, Node::Fds(_)) {
                        Node::FdLink(pid, fd)
                    } else {
                        Node::DescriptorInfo(pid, fd)
                    };
                    entries.push((format!("{fd}").into_bytes(), node, fd as u64 + 3));
                }
            }
            _ => return Err(Error::NotDirectory),
        }
        Ok(entries)
    }
}

impl VnodeOps for ProcNode {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn cache_names(&self) -> bool {
        false
    }

    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        self.ensure_exists()?;
        let kind = self.node.kind();
        Ok(VnodeAttr {
            key: vnode.key(),
            kind,
            size: 0,
            links: if kind == VnodeKind::Directory { 2 } else { 1 },
            mode: match kind {
                VnodeKind::Directory => 0o555,
                VnodeKind::Symlink => 0o777,
                _ => 0o444,
            },
            accessed_ns: 0,
            modified_ns: 0,
            changed_ns: 0,
        })
    }

    fn open(&self, _vnode: &Vnode, flags: u32) -> Result<usize> {
        if OpenFlags::from_bits_retain(flags).contains(OpenFlags::WRITE) {
            return Err(Error::ReadOnly);
        }
        self.ensure_exists()?;
        if self.node.kind() == VnodeKind::Directory {
            return Ok(0);
        }
        let contents: Arc<[u8]> = self.contents()?.into();
        let filesystem = self.filesystem()?;
        let id = filesystem
            .next_reader
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .map_err(|_| Error::NoSpace)?;
        filesystem.readers.lock().insert(id, contents);
        Ok(id)
    }

    fn close(&self, _vnode: &Vnode, context: usize, _flags: u32) {
        if context != 0
            && let Ok(filesystem) = self.filesystem()
        {
            filesystem.readers.lock().remove(&context);
        }
    }

    fn poll_events<'a>(
        &'a self,
        _vnode: &Vnode,
        _context: usize,
        events: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        if !matches!(self.node, Node::MountInfo(_) | Node::Mounts(_)) {
            return false;
        }
        // These snapshot files stay readable. Namespace notifications supply
        // fresh readiness epochs for libmount's edge-triggered monitor.
        if events.intersects(PollEvents::IN | PollEvents::RDNORM) {
            output.push(super::vfs::mount_changed().into());
        }
        true
    }

    fn lookup(&self, _directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        self.ensure_exists()?;
        let node = match self.node {
            Node::Root => match name {
                b"cmdline" => Node::Cmdline,
                b"self" => Node::SelfLink,
                b"mounts" => Node::MountsLink,
                b"filesystems" => Node::Filesystems,
                b"cgroups" => Node::Cgroups,
                _ => {
                    let pid = decimal(name)?;
                    proc::find(pid).ok_or(Error::NotFound)?;
                    Node::Process(pid)
                }
            },
            Node::Process(pid) => match name {
                b"exe" => {
                    proc::find(pid)
                        .and_then(|process| process.executable())
                        .ok_or(Error::NotFound)?;
                    Node::Executable(pid)
                }
                b"status" => Node::Status(pid),
                b"cgroup" => Node::Cgroup(pid),
                b"mountinfo" => Node::MountInfo(pid),
                b"mounts" => Node::Mounts(pid),
                b"fd" => Node::Fds(pid),
                b"fdinfo" => Node::FdInfo(pid),
                _ => return Err(Error::NotFound),
            },
            Node::Fds(pid) | Node::FdInfo(pid) => {
                let fd = i32::try_from(decimal(name)?).map_err(|_| Error::NotFound)?;
                proc::find(pid)
                    .and_then(|process| process.descriptor(fd))
                    .ok_or(Error::NotFound)?;
                if matches!(self.node, Node::Fds(_)) {
                    Node::FdLink(pid, fd)
                } else {
                    Node::DescriptorInfo(pid, fd)
                }
            }
            _ => return Err(Error::NotDirectory),
        };
        self.filesystem()?.node(node)
    }

    fn parent(&self, _directory: &Vnode) -> Result<Vnode> {
        self.ensure_exists()?;
        let parent = match self.node {
            Node::Root | Node::Process(_) => Node::Root,
            Node::Fds(pid) | Node::FdInfo(pid) => Node::Process(pid),
            _ => return Err(Error::NotDirectory),
        };
        self.filesystem()?.node(parent)
    }

    fn readdir(
        &self,
        directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        let filesystem = self.filesystem()?;
        let mut output = Vec::new();
        let mut next = cursor;
        for (name, node, cookie) in [
            (b".".as_slice(), self.node, 1),
            (
                b"..",
                match self.node {
                    Node::Root | Node::Process(_) => Node::Root,
                    Node::Fds(pid) | Node::FdInfo(pid) => Node::Process(pid),
                    _ => return Err(Error::NotDirectory),
                },
                2,
            ),
        ] {
            if cookie <= cursor || output.len() == maximum {
                continue;
            }
            let vnode = if cookie == 1 {
                directory.clone()
            } else {
                filesystem.node(node)?
            };
            output.push(DirEntry {
                name: Arc::from(name),
                key: vnode.key(),
                kind: VnodeKind::Directory,
                offset: cookie,
            });
            next = cookie;
        }
        for (name, node, cookie) in self.children()? {
            if cookie <= cursor {
                continue;
            }
            if output.len() == maximum {
                break;
            }
            let vnode = filesystem.node(node)?;
            output.push(DirEntry {
                name: name.into(),
                key: vnode.key(),
                kind: vnode.kind(),
                offset: cookie,
            });
            next = cookie;
        }
        Ok((output, next))
    }

    fn read_at(&self, _vnode: &Vnode, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        copy_read(&self.contents()?, offset, sink)
    }

    fn read_at_with_flags(
        &self,
        _vnode: &Vnode,
        context: usize,
        offset: u64,
        sink: &mut IoSink<'_>,
        _flags: u32,
    ) -> Result<usize> {
        if self.node.kind() == VnodeKind::Directory {
            return Err(Error::IsDirectory);
        }
        let filesystem = self.filesystem()?;
        let bytes = filesystem
            .readers
            .lock()
            .get(&context)
            .cloned()
            .ok_or(Error::BadFileDescriptor)?;
        copy_read(&bytes, offset, sink)
    }

    fn file_link(&self, _vnode: &Vnode) -> Result<Option<super::FileRef>> {
        if let Node::Executable(pid) = self.node {
            return proc::find(pid)
                .and_then(|process| process.executable())
                .map(Some)
                .ok_or(Error::NotFound);
        }
        if let Node::FdLink(pid, fd) = self.node {
            return match proc::find(pid).and_then(|process| process.descriptor(fd)) {
                Some(Descriptor::File(file)) => Ok(Some(file)),
                Some(_) => Err(Error::Unsupported),
                None => Err(Error::NotFound),
            };
        }
        Ok(None)
    }

    fn readlink(&self, vnode: &Vnode) -> Result<Box<[u8]>> {
        match self.node {
            Node::SelfLink => Ok(format!("{}", proc::current().ok_or(Error::NotFound)?.pid())
                .into_bytes()
                .into_boxed_slice()),
            Node::MountsLink => Ok(b"self/mounts".to_vec().into_boxed_slice()),
            Node::Executable(_) | Node::FdLink(..) => {
                let file = self.file_link(vnode)?.ok_or(Error::NotFound)?;
                Ok(super::vfs::proc_path(file.path_anchor())?.into_boxed_slice())
            }
            _ => Err(Error::InvalidArgument),
        }
    }

    fn create(
        &self,
        _directory: &Vnode,
        _name: &[u8],
        _kind: CreateKind,
        _mode: u16,
    ) -> Result<Vnode> {
        Err(Error::ReadOnly)
    }
    fn setattr(&self, _vnode: &Vnode, _attr: SetAttr) -> Result<()> {
        Err(Error::ReadOnly)
    }
    fn write_at(&self, _vnode: &Vnode, _offset: u64, _source: &IoSource<'_>) -> Result<usize> {
        Err(Error::ReadOnly)
    }
    fn unlink(&self, _directory: &Vnode, _name: &[u8], _directory_only: bool) -> Result<()> {
        Err(Error::ReadOnly)
    }
    fn link(&self, _directory: &Vnode, _name: &[u8], _target: &Vnode) -> Result<()> {
        Err(Error::ReadOnly)
    }
    fn rename(
        &self,
        _source_directory: &Vnode,
        _source_name: &[u8],
        _target_directory: &Vnode,
        _target_name: &[u8],
    ) -> Result<()> {
        Err(Error::ReadOnly)
    }
}

fn copy_read(bytes: &[u8], offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
    let offset = usize::try_from(offset)
        .map_err(|_| Error::InvalidArgument)?
        .min(bytes.len());
    let count = sink.len().min(bytes.len() - offset);
    sink.store(0, &bytes[offset..offset + count])
        .map_err(|_| Error::InvalidArgument)?;
    Ok(count)
}

fn decimal(bytes: &[u8]) -> Result<usize> {
    if bytes.is_empty() || (bytes.len() > 1 && bytes[0] == b'0') {
        return Err(Error::NotFound);
    }
    bytes.iter().try_fold(0usize, |value, digit| {
        if !digit.is_ascii_digit() {
            return Err(Error::NotFound);
        }
        value
            .checked_mul(10)
            .and_then(|value| value.checked_add(usize::from(digit - b'0')))
            .ok_or(Error::NotFound)
    })
}
