//! Live device metadata for libudev and the device manager.
//!
//! The device namespace is derived from devfs under its registry lock, so
//! device removal and name reuse cannot expose stale enumeration entries.

use super::vnode::{
    DirEntry, FileSystem, FilesystemId, NodeId, StatFs, Vnode, VnodeAttr, VnodeCache, VnodeKey,
    VnodeKind, VnodeOps,
};
use super::{
    Error, OpenFlags, Result,
    devfs::{self, DeviceInfo},
};
use crate::{
    mem::{IoSink, IoSource},
    sys::sync::{Mutex, Once},
};
use alloc::{
    boxed::Box,
    collections::BTreeMap,
    format,
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::any::Any;

static SEQUENCE: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// Publishes one native device lifecycle event using the Linux uevent format.
pub(crate) fn emit(action: &str, device: &DeviceInfo) {
    // Seat tags belong to the input parent. Publish it before its event node
    // appears and remove it only after the event node has gone away.
    let input = device.subsystem() == "input";
    if input && action == "add" {
        emit_full(action, device, true, None);
    }
    emit_full(action, device, false, None);
    if input && action == "remove" {
        emit_full(action, device, true, None);
    }
}

fn emit_full(action: &str, device: &DeviceInfo, parent: bool, uuid: Option<&str>) {
    let sequence = SEQUENCE.fetch_add(1, core::sync::atomic::Ordering::AcqRel) + 1;
    let mut path = device.syspath();
    if parent {
        path.truncate(path.rfind('/').unwrap_or(path.len()));
    }
    let metadata = if parent {
        Vec::new()
    } else {
        Data::Uevent(device.clone()).bytes()
    };
    let mut payload = format!(
        "{action}@/{path}\0ACTION={action}\0DEVPATH=/{path}\0SUBSYSTEM={}\0SEQNUM={sequence}\0",
        device.subsystem()
    )
    .into_bytes();
    if let Some(uuid) = uuid {
        payload.extend(format!("SYNTH_UUID={uuid}\0").into_bytes());
    }
    payload.extend(
        metadata
            .into_iter()
            .map(|byte| if byte == b'\n' { 0 } else { byte }),
    );
    crate::proc::unix::broadcast_uevent(&payload);
}

/// Kernel-owned sysfs device view.
pub struct Sysfs {
    id: FilesystemId,
    root: Once<Vnode>,
    nodes: VnodeCache,
    identities: Mutex<BTreeMap<String, (u64, u64)>>,
    next_id: core::sync::atomic::AtomicU64,
}

#[derive(Clone)]
enum Data {
    Directory,
    Sequence,
    Link(String),
    File(Vec<u8>),
    Uevent(DeviceInfo),
    InputParent(DeviceInfo),
    InputAttribute(DeviceInfo, &'static str),
}
#[derive(Clone)]
struct Entry {
    generation: u64,
    data: Data,
}
struct SysNode {
    filesystem: Weak<Sysfs>,
    path: String,
    generation: u64,
}
type Tree = BTreeMap<String, Entry>;

impl Data {
    fn kind(&self) -> VnodeKind {
        match self {
            Self::Directory => VnodeKind::Directory,
            Self::Link(_) => VnodeKind::Symlink,
            _ => VnodeKind::Regular,
        }
    }
    fn bytes(&self) -> Vec<u8> {
        match self {
            Self::File(bytes) => bytes.clone(),
            Self::Sequence => {
                format!("{}\n", SEQUENCE.load(core::sync::atomic::Ordering::Acquire)).into_bytes()
            }
            Self::Uevent(device) => {
                let (major, minor) = device.numbers();
                let devtype = if device.path.starts_with("dri/") {
                    "DEVTYPE=drm_minor\n"
                } else {
                    ""
                };
                format!(
                    "MAJOR={major}\nMINOR={minor}\nDEVNAME={}\n{devtype}",
                    device.path
                )
                .into_bytes()
            }
            _ => Vec::new(),
        }
    }
}
fn insert(tree: &mut Tree, path: String, generation: u64, data: Data) {
    tree.insert(path, Entry { generation, data });
}
fn directory(tree: &mut Tree, path: &str, generation: u64) {
    insert(tree, String::from(path), generation, Data::Directory);
}

static TREE: Once<Mutex<(u64, Arc<Tree>)>> = Once::new();

fn tree() -> Result<Arc<Tree>> {
    let cache = TREE.call_once(|| Mutex::new((u64::MAX, Arc::new(Tree::new()))));
    let mut cache = cache.lock();
    loop {
        let generation = devfs::global()?.generation();
        if cache.0 == generation {
            return Ok(cache.1.clone());
        }
        let next = build_tree()?;
        if generation != devfs::global()?.generation() {
            continue;
        }
        *cache = (generation, Arc::new(next));
        return Ok(cache.1.clone());
    }
}

fn build_tree() -> Result<Tree> {
    let mut tree = Tree::new();
    for path in [
        "",
        "devices",
        "devices/virtual",
        "class",
        "dev",
        "dev/char",
        "dev/block",
        "bus",
        "fs",
        "fs/cgroup",
        "kernel",
    ] {
        directory(&mut tree, path, 0);
    }
    insert(
        &mut tree,
        String::from("kernel/uevent_seqnum"),
        0,
        Data::Sequence,
    );
    for device in devfs::global()?.devices() {
        let name = device
            .path
            .rsplit('/')
            .next()
            .ok_or(Error::InvalidArgument)?;
        let subsystem = device.subsystem();
        directory(&mut tree, &format!("class/{subsystem}"), 0);
        directory(&mut tree, &format!("devices/virtual/{subsystem}"), 0);
        let base = if subsystem == "input" {
            let index = name.strip_prefix("event").unwrap_or(name);
            let parent = format!("devices/virtual/input/input{index}");
            directory(&mut tree, &parent, device.id);
            for child in ["id", "capabilities"] {
                directory(&mut tree, &format!("{parent}/{child}"), device.id);
            }
            for attribute in [
                "name",
                "properties",
                "id/bustype",
                "id/vendor",
                "id/product",
                "id/version",
                "capabilities/ev",
                "capabilities/key",
                "capabilities/rel",
                "capabilities/abs",
                "capabilities/msc",
                "capabilities/led",
            ] {
                insert(
                    &mut tree,
                    format!("{parent}/{attribute}"),
                    device.id,
                    Data::InputAttribute(device.clone(), attribute),
                );
            }
            insert(
                &mut tree,
                format!("{parent}/subsystem"),
                device.id,
                Data::Link(String::from("/sys/class/input")),
            );
            insert(
                &mut tree,
                format!("{parent}/uevent"),
                device.id,
                Data::InputParent(device.clone()),
            );
            insert(
                &mut tree,
                format!("class/input/input{index}"),
                device.id,
                Data::Link(format!("/sys/{parent}")),
            );
            format!("{parent}/{name}")
        } else {
            format!("devices/virtual/{subsystem}/{name}")
        };
        directory(&mut tree, &base, device.id);
        insert(
            &mut tree,
            format!("{base}/subsystem"),
            device.id,
            Data::Link(format!("/sys/class/{subsystem}")),
        );
        let (major, minor) = device.numbers();
        insert(
            &mut tree,
            format!("{base}/dev"),
            device.id,
            Data::File(format!("{major}:{minor}\n").into_bytes()),
        );
        insert(
            &mut tree,
            format!("class/{subsystem}/{name}"),
            device.id,
            Data::Link(format!("/sys/{base}")),
        );
        insert(
            &mut tree,
            format!(
                "dev/{}/{major}:{minor}",
                if device.block { "block" } else { "char" }
            ),
            device.id,
            Data::Link(format!("/sys/{base}")),
        );
        insert(
            &mut tree,
            format!("{base}/uevent"),
            device.id,
            Data::Uevent(device),
        );
    }
    Ok(tree)
}

impl Sysfs {
    /// Creates a live view of the native device registry.
    pub fn new() -> Result<Arc<Self>> {
        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            root: Once::new(),
            nodes: VnodeCache::new(),
            identities: Mutex::new(BTreeMap::new()),
            next_id: core::sync::atomic::AtomicU64::new(1),
        });
        let root = filesystem.node("", tree()?.as_ref())?;
        filesystem.root.call_once(|| root);
        Ok(filesystem)
    }
    fn node(self: &Arc<Self>, path: &str, tree: &Tree) -> Result<Vnode> {
        let entry = tree.get(path).ok_or(Error::NotFound)?;
        let mut identities = self.identities.lock();
        let record = identities
            .entry(String::from(path))
            .or_insert((u64::MAX, 0));
        if record.0 != entry.generation {
            *record = (
                entry.generation,
                self.next_id
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed),
            );
        }
        let id = record.1;
        drop(identities);
        self.nodes.get_or_create(id, || {
            Ok(Vnode::new(
                VnodeKey {
                    filesystem: self.id,
                    node: NodeId::new(id),
                },
                entry.data.kind(),
                Box::new(SysNode {
                    filesystem: Arc::downgrade(self),
                    path: String::from(path),
                    generation: entry.generation,
                }),
            ))
        })
    }
}
impl FileSystem for Sysfs {
    fn id(&self) -> FilesystemId {
        self.id
    }
    fn name(&self) -> &str {
        "sysfs"
    }
    fn root(&self) -> Vnode {
        self.root.get().cloned().expect("sysfs root initialized")
    }
    fn statfs(&self) -> StatFs {
        StatFs {
            filesystem_type: 0x62656572,
            block_size: crate::mem::PAGE_SIZE,
            ..StatFs::default()
        }
    }
}
impl SysNode {
    fn entry<'a>(&self, tree: &'a Tree) -> Result<&'a Entry> {
        tree.get(&self.path)
            .filter(|e| e.generation == self.generation)
            .ok_or(Error::NotFound)
    }
    fn filesystem(&self) -> Result<Arc<Sysfs>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }
}
impl VnodeOps for SysNode {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn cache_names(&self) -> bool {
        false
    }
    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        let tree = tree()?;
        let entry = self.entry(&tree)?;
        let kind = entry.data.kind();
        Ok(VnodeAttr {
            key: vnode.key(),
            kind,
            size: 0,
            links: if kind == VnodeKind::Directory { 2 } else { 1 },
            mode: match entry.data {
                Data::Directory => 0o555,
                Data::Link(_) => 0o777,
                Data::Uevent(_) | Data::InputParent(_) => 0o644,
                _ => 0o444,
            },
            accessed_ns: 0,
            modified_ns: 0,
            changed_ns: 0,
        })
    }
    fn open(&self, _vnode: &Vnode, flags: u32) -> Result<usize> {
        let tree = tree()?;
        let entry = self.entry(&tree)?;
        if OpenFlags::from_bits_retain(flags).contains(OpenFlags::WRITE)
            && !matches!(entry.data, Data::Uevent(_) | Data::InputParent(_))
        {
            return Err(Error::ReadOnly);
        }
        Ok(0)
    }
    fn lookup(&self, _directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        super::path::validate_leaf_name(name)?;
        let name = core::str::from_utf8(name).map_err(|_| Error::NotFound)?;
        let tree = tree()?;
        if !matches!(self.entry(&tree)?.data, Data::Directory) {
            return Err(Error::NotDirectory);
        }
        let path = if self.path.is_empty() {
            String::from(name)
        } else {
            format!("{}/{name}", self.path)
        };
        self.filesystem()?.node(&path, &tree)
    }
    fn parent(&self, _vnode: &Vnode) -> Result<Vnode> {
        self.filesystem()?.node(
            self.path.rsplit_once('/').map_or("", |(parent, _)| parent),
            tree()?.as_ref(),
        )
    }
    fn readdir(
        &self,
        _directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        let tree = tree()?;
        if !matches!(self.entry(&tree)?.data, Data::Directory) {
            return Err(Error::NotDirectory);
        }
        let filesystem = self.filesystem()?;
        let prefix = if self.path.is_empty() {
            String::new()
        } else {
            format!("{}/", self.path)
        };
        let parent = self.path.rsplit_once('/').map_or("", |(parent, _)| parent);
        let mut children = alloc::vec![
            (String::from("."), self.path.clone()),
            (String::from(".."), String::from(parent))
        ];
        for path in tree.keys() {
            if let Some(name) = path.strip_prefix(&prefix)
                && !name.is_empty()
                && !name.contains('/')
            {
                children.push((String::from(name), path.clone()));
            }
        }
        let mut entries = Vec::new();
        let mut next = cursor;
        for (index, (name, path)) in children
            .into_iter()
            .enumerate()
            .skip(cursor as usize)
            .take(maximum)
        {
            let node = filesystem.node(&path, &tree)?;
            next = index as u64 + 1;
            entries.push(DirEntry {
                name: name.into_bytes().into(),
                key: node.key(),
                kind: node.kind(),
                offset: next,
            });
        }
        Ok((entries, next))
    }
    fn read_at(&self, _vnode: &Vnode, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        let tree = tree()?;
        let entry = self.entry(&tree)?;
        if matches!(entry.data, Data::Directory) {
            return Err(Error::IsDirectory);
        }
        let bytes = match &entry.data {
            Data::InputAttribute(device, attribute) => device.input_attribute(attribute)?,
            _ => entry.data.bytes(),
        };
        let offset = usize::try_from(offset)
            .unwrap_or(usize::MAX)
            .min(bytes.len());
        let count = sink.len().min(bytes.len() - offset);
        sink.store(0, &bytes[offset..offset + count])
            .map_err(|_| Error::InvalidArgument)?;
        Ok(count)
    }
    fn write_at(&self, _vnode: &Vnode, offset: u64, source: &IoSource<'_>) -> Result<usize> {
        if offset != 0 || source.is_empty() || source.len() > 256 {
            return Err(Error::InvalidArgument);
        }
        let tree = tree()?;
        let (device, parent) = match &self.entry(&tree)?.data {
            Data::Uevent(device) => (device, false),
            Data::InputParent(device) => (device, true),
            _ => return Err(Error::ReadOnly),
        };
        let mut bytes = alloc::vec![0; source.len()];
        source
            .load(0, &mut bytes)
            .map_err(|_| Error::InvalidArgument)?;
        let text = core::str::from_utf8(&bytes)
            .map_err(|_| Error::InvalidArgument)?
            .trim();
        let (action, uuid) = if let Some((action, uuid)) = text.split_once(' ') {
            if uuid.len() != 36
                || !uuid.bytes().enumerate().all(|(i, b)| {
                    if matches!(i, 8 | 13 | 18 | 23) {
                        b == b'-'
                    } else {
                        b.is_ascii_hexdigit()
                    }
                })
            {
                return Err(Error::InvalidArgument);
            }
            (action, Some(uuid))
        } else {
            (text, Some("0"))
        };
        if !matches!(
            action,
            "add" | "remove" | "change" | "move" | "online" | "offline" | "bind" | "unbind"
        ) {
            return Err(Error::InvalidArgument);
        }
        emit_full(action, device, parent, uuid);
        Ok(bytes.len())
    }
    fn readlink(&self, _vnode: &Vnode) -> Result<Box<[u8]>> {
        let tree = tree()?;
        match &self.entry(&tree)?.data {
            Data::Link(path) => Ok(path.as_bytes().into()),
            _ => Err(Error::InvalidArgument),
        }
    }
}
