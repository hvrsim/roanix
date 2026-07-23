//! Global mount namespace, path traversal, and convenience VFS operations.

use alloc::{
    collections::{BTreeMap, VecDeque},
    sync::{Arc, Weak},
};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::sys::sync::{Mutex, Once};

use super::{
    error::{Error, Result},
    file::{FileRef, OpenFile, OpenFlags},
    path::{self, MAX_SYMLINK_DEPTH},
    vnode::{CreateKind, FileSystemRef, StatFs, Vnode, VnodeKey, VnodeKind},
};

static NEXT_MOUNT_ID: AtomicU64 = AtomicU64::new(1);
static NAMESPACE: Once<Namespace> = Once::new();

/// Stable identifier for one namespace mount.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct MountId(u64);

struct Mount {
    id: MountId,
    filesystem: FileSystemRef,
    root: Vnode,
    covered: Option<Vnode>,
    parent: Weak<Mount>,
}

type MountRef = Arc<Mount>;

struct NamespaceState {
    root: MountRef,
    mounted_at: BTreeMap<VnodeKey, MountRef>,
}

struct Namespace {
    state: Mutex<NamespaceState>,
}

struct Resolved {
    vnode: Vnode,
    mount: MountRef,
}

impl Mount {
    fn root(filesystem: FileSystemRef) -> Result<MountRef> {
        let root = filesystem.root();
        if root.kind() != VnodeKind::Directory || root.key().filesystem != filesystem.id() {
            return Err(Error::NotDirectory);
        }
        Ok(Arc::new(Self {
            id: MountId(NEXT_MOUNT_ID.fetch_add(1, Ordering::Relaxed)),
            filesystem,
            root,
            covered: None,
            parent: Weak::new(),
        }))
    }

    fn child(filesystem: FileSystemRef, covered: Vnode, parent: &MountRef) -> Result<MountRef> {
        let root = filesystem.root();
        if root.kind() != VnodeKind::Directory
            || covered.kind() != VnodeKind::Directory
            || root.key().filesystem != filesystem.id()
        {
            return Err(Error::NotDirectory);
        }
        Ok(Arc::new(Self {
            id: MountId(NEXT_MOUNT_ID.fetch_add(1, Ordering::Relaxed)),
            filesystem,
            root,
            covered: Some(covered),
            parent: Arc::downgrade(parent),
        }))
    }
}

impl Namespace {
    fn new(filesystem: FileSystemRef) -> Result<Self> {
        Ok(Self {
            state: Mutex::new(NamespaceState {
                root: Mount::root(filesystem)?,
                mounted_at: BTreeMap::new(),
            }),
        })
    }

    fn root(&self) -> MountRef {
        self.state.lock().root.clone()
    }

    fn mounted_at(&self, key: VnodeKey) -> Option<MountRef> {
        self.state.lock().mounted_at.get(&key).cloned()
    }
}

/// Installs the first root filesystem.
pub(crate) fn init_root(filesystem: FileSystemRef) -> Result<()> {
    if NAMESPACE.get().is_some() {
        return Err(Error::Busy);
    }
    let namespace = Namespace::new(filesystem)?;
    NAMESPACE.call_once(|| namespace);
    Ok(())
}

/// Looks up a path from the global root namespace.
pub fn lookup(path: &str) -> Result<Vnode> {
    lookup_bytes(path.as_bytes())
}

/// Looks up a path without following a final symbolic link.
pub fn lookup_nofollow(path: &str) -> Result<Vnode> {
    Ok(resolve(path.as_bytes(), false)?.vnode)
}

/// Looks up a byte path from the global root namespace.
pub fn lookup_bytes(path: &[u8]) -> Result<Vnode> {
    Ok(resolve(path, true)?.vnode)
}

/// Opens or creates a vnode and returns a shared open file description.
pub fn open(path: &str, flags: OpenFlags, mode: u16) -> Result<FileRef> {
    if !flags.intersects(OpenFlags::READ | OpenFlags::WRITE) {
        return Err(Error::InvalidArgument);
    }
    if flags.contains(OpenFlags::TRUNCATE) && !flags.contains(OpenFlags::WRITE) {
        return Err(Error::PermissionDenied);
    }

    let follow_final = !flags.contains(OpenFlags::NOFOLLOW);
    let vnode = match resolve(path.as_bytes(), follow_final) {
        Ok(resolved) => {
            if flags.contains(OpenFlags::CREATE | OpenFlags::EXCLUSIVE) {
                return Err(Error::AlreadyExists);
            }
            resolved.vnode
        }
        Err(Error::NotFound) if flags.contains(OpenFlags::CREATE) => {
            let (directory, name) = resolve_parent(path.as_bytes())?;
            match directory.vnode.create(name, CreateKind::Regular, mode) {
                Ok(vnode) => vnode,
                Err(Error::AlreadyExists) if !flags.contains(OpenFlags::EXCLUSIVE) => {
                    directory.vnode.lookup(name)?
                }
                Err(error) => return Err(error),
            }
        }
        Err(error) => return Err(error),
    };

    if flags.contains(OpenFlags::NOFOLLOW) && vnode.kind() == VnodeKind::Symlink {
        return Err(Error::SymlinkLoop);
    }
    if flags.contains(OpenFlags::DIRECTORY) && vnode.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }
    if vnode.kind() == VnodeKind::Directory && flags.contains(OpenFlags::WRITE) {
        return Err(Error::IsDirectory);
    }
    if flags.contains(OpenFlags::TRUNCATE) && vnode.kind() == VnodeKind::Regular {
        vnode.truncate(0)?;
    }

    OpenFile::new(vnode, flags)
}

/// Creates an empty regular file.
pub fn create_file(path: &str, mode: u16) -> Result<Vnode> {
    let (directory, name) = resolve_parent(path.as_bytes())?;
    directory.vnode.create(name, CreateKind::Regular, mode)
}

/// Creates an empty directory.
pub fn create_dir(path: &str, mode: u16) -> Result<Vnode> {
    let (directory, name) = resolve_parent(path.as_bytes())?;
    directory.vnode.create(name, CreateKind::Directory, mode)
}

/// Creates a symbolic link.
pub fn symlink(target: &str, path: &str) -> Result<Vnode> {
    let (directory, name) = resolve_parent(path.as_bytes())?;
    path::validate_symlink_target(target.as_bytes())?;
    directory.vnode.create(
        name,
        CreateKind::Symlink(alloc::boxed::Box::<[u8]>::from(target.as_bytes())),
        0o777,
    )
}

/// Adds a hard link to an existing non-directory vnode.
pub fn link(existing: &str, new_path: &str) -> Result<()> {
    let target = resolve(existing.as_bytes(), true)?.vnode;
    if target.kind() == VnodeKind::Directory {
        return Err(Error::PermissionDenied);
    }
    let (directory, name) = resolve_parent(new_path.as_bytes())?;
    directory.vnode.link(name, &target)
}

/// Removes a non-directory path.
pub fn unlink(path: &str) -> Result<()> {
    let namespace = namespace()?;
    let (directory, name) = resolve_parent(path.as_bytes())?;
    let state = namespace.state.lock();
    let target = directory.vnode.lookup(name)?;
    if state.mounted_at.contains_key(&target.key()) {
        return Err(Error::Busy);
    }
    let result = directory.vnode.unlink(name, false);
    drop(state);
    result
}

/// Removes an empty directory.
pub fn remove_dir(path: &str) -> Result<()> {
    let namespace = namespace()?;
    let (directory, name) = resolve_parent(path.as_bytes())?;
    let state = namespace.state.lock();
    let target = directory.vnode.lookup(name)?;
    if state.mounted_at.contains_key(&target.key()) {
        return Err(Error::Busy);
    }
    let result = directory.vnode.unlink(name, true);
    drop(state);
    result
}

/// Atomically renames or replaces a path within one filesystem.
pub fn rename(source: &str, target: &str) -> Result<()> {
    let namespace = namespace()?;
    let (source_directory, source_name) = resolve_parent(source.as_bytes())?;
    let (target_directory, target_name) = resolve_parent(target.as_bytes())?;
    let state = namespace.state.lock();
    let source_vnode = source_directory.vnode.lookup(source_name)?;
    let target_vnode = target_directory.vnode.lookup(target_name).ok();
    if state.mounted_at.contains_key(&source_vnode.key())
        || target_vnode
            .as_ref()
            .is_some_and(|vnode| state.mounted_at.contains_key(&vnode.key()))
    {
        return Err(Error::Busy);
    }
    let result = source_directory
        .vnode
        .rename(source_name, &target_directory.vnode, target_name);
    drop(state);
    result
}

/// Mounts a filesystem over an existing directory.
pub fn mount(path: &str, filesystem: FileSystemRef) -> Result<MountId> {
    let namespace = namespace()?;
    let (parent, name) = resolve_parent(path.as_bytes())?;
    let mut state = namespace.state.lock();
    let covered = parent.vnode.lookup(name)?;
    if covered.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }
    if state.mounted_at.contains_key(&covered.key()) {
        return Err(Error::Busy);
    }
    let mount = Mount::child(filesystem, covered.clone(), &parent.mount)?;
    let id = mount.id;
    state.mounted_at.insert(covered.key(), mount);
    Ok(id)
}

/// Returns filesystem statistics for the mount containing `path`.
pub fn statfs(path: &str) -> Result<StatFs> {
    Ok(resolve(path.as_bytes(), true)?.mount.filesystem.statfs())
}

/// Flushes every filesystem currently present in the namespace.
pub fn sync_all() -> Result<()> {
    let namespace = namespace()?;
    let filesystems = {
        let state = namespace.state.lock();
        let mut filesystems = alloc::vec![state.root.filesystem.clone()];
        filesystems.extend(
            state
                .mounted_at
                .values()
                .map(|mount| mount.filesystem.clone()),
        );
        filesystems
    };
    for filesystem in filesystems {
        filesystem.sync()?;
    }
    Ok(())
}

fn namespace() -> Result<&'static Namespace> {
    NAMESPACE.get().ok_or(Error::Io)
}

fn resolve_parent(path: &[u8]) -> Result<(Resolved, &[u8])> {
    let (parent, name) = path::split_parent(path)?;
    Ok((resolve(parent, true)?, name))
}

fn resolve(path: &[u8], follow_final: bool) -> Result<Resolved> {
    let namespace = namespace()?;
    let parsed = path::parse(path)?;
    let root_mount = namespace.root();
    let mut current = Resolved {
        vnode: root_mount.root.clone(),
        mount: root_mount,
    };
    let mut components = parsed.components;
    let mut symlink_depth = 0usize;

    while let Some(component) = components.pop_front() {
        if component.as_ref() == b"." {
            continue;
        }
        if component.as_ref() == b".." {
            current = ascend(current)?;
            continue;
        }

        if current.vnode.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        let parent = current.vnode.clone();
        let child = parent.lookup(&component)?;
        let final_component = components.is_empty();

        if child.kind() == VnodeKind::Symlink && (follow_final || !final_component) {
            symlink_depth += 1;
            if symlink_depth > MAX_SYMLINK_DEPTH {
                return Err(Error::SymlinkLoop);
            }
            let target = path::parse(&child.readlink()?)?;
            if target.absolute {
                let root_mount = namespace.root();
                current = Resolved {
                    vnode: root_mount.root.clone(),
                    mount: root_mount,
                };
            } else {
                current.vnode = parent;
            }
            prepend(&mut components, target.components);
            continue;
        }

        current.vnode = child;
        if let Some(mount) = namespace.mounted_at(current.vnode.key()) {
            current.vnode = mount.root.clone();
            current.mount = mount;
        }
    }

    Ok(current)
}

fn ascend(mut current: Resolved) -> Result<Resolved> {
    if current.vnode.key() == current.mount.root.key() {
        let Some(covered) = current.mount.covered.as_ref() else {
            return Ok(current);
        };
        let Some(parent_mount) = current.mount.parent.upgrade() else {
            return Err(Error::Io);
        };
        current.vnode = covered.parent()?;
        current.mount = parent_mount;
        return Ok(current);
    }

    current.vnode = current.vnode.parent()?;
    Ok(current)
}

fn prepend(
    target: &mut VecDeque<alloc::boxed::Box<[u8]>>,
    mut prefix: VecDeque<alloc::boxed::Box<[u8]>>,
) {
    while let Some(component) = prefix.pop_back() {
        target.push_front(component);
    }
}
