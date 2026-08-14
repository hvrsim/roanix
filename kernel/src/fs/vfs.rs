//! Global mount namespace, path traversal, and convenience VFS operations.

use alloc::{
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use crate::sys::sync::{Mutex, Once};

use super::{
    error::{Error, Result},
    file::{FileRef, OpenFile, OpenFlags},
    path::{self, MAX_SYMLINK_DEPTH},
    vnode::{CreateKind, FileSystemRef, StatFs, Vnode, VnodeKey, VnodeKind},
};

static NEXT_MOUNT_ID: AtomicU64 = AtomicU64::new(1);
static NAMESPACE: Once<Namespace> = Once::new();

/// Number of filesystems mounted over a directory.
///
/// Path traversal consults this before locking the namespace, so a namespace
/// without submounts resolves every component lock-free.
static COVERED_MOUNTS: AtomicUsize = AtomicUsize::new(0);

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

#[derive(Clone)]
pub(crate) struct PathAnchor {
    vnode: Vnode,
    mount: MountRef,
}

impl PathAnchor {
    /// Returns the vnode at this stable namespace location.
    pub(crate) fn vnode(&self) -> &Vnode {
        &self.vnode
    }
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
        if COVERED_MOUNTS.load(Ordering::Acquire) == 0 {
            return None;
        }
        self.state.lock().mounted_at.get(&key).cloned()
    }

    fn is_mount_point(&self, key: VnodeKey) -> bool {
        if COVERED_MOUNTS.load(Ordering::Acquire) == 0 {
            return false;
        }
        self.state.lock().mounted_at.contains_key(&key)
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
pub fn lookup(path: &[u8]) -> Result<Vnode> {
    Ok(resolve(None, path, true)?.vnode)
}

/// Looks up a path without following a final symbolic link.
pub fn lookup_nofollow(path: &[u8]) -> Result<Vnode> {
    Ok(resolve(None, path, false)?.vnode)
}

/// Returns an anchor for the namespace root.
pub(crate) fn root_anchor() -> Result<PathAnchor> {
    let root = namespace()?.root();
    Ok(PathAnchor {
        vnode: root.root.clone(),
        mount: root,
    })
}

/// Resolves a path relative to a stable directory anchor.
pub(crate) fn resolve_at(base: &PathAnchor, path: &[u8], follow_final: bool) -> Result<PathAnchor> {
    resolve(Some(base), path, follow_final)
}

/// Opens or creates a vnode and returns a shared open file description.
pub fn open(path: &[u8], flags: OpenFlags, mode: u16) -> Result<FileRef> {
    let root = root_anchor()?;
    open_at(&root, path, flags, mode)
}

/// Opens or creates a vnode relative to a stable directory anchor.
pub(crate) fn open_at(
    base: &PathAnchor,
    path: &[u8],
    flags: OpenFlags,
    mode: u16,
) -> Result<FileRef> {
    if !flags.intersects(OpenFlags::READ | OpenFlags::WRITE) {
        return Err(Error::InvalidArgument);
    }
    if flags.contains(OpenFlags::TRUNCATE) && !flags.contains(OpenFlags::WRITE) {
        return Err(Error::PermissionDenied);
    }

    let follow_final = !flags.contains(OpenFlags::NOFOLLOW);
    let (anchor, created) = match resolve(Some(base), path, follow_final) {
        Ok(resolved) => {
            if flags.contains(OpenFlags::CREATE | OpenFlags::EXCLUSIVE) {
                return Err(Error::AlreadyExists);
            }
            (resolved, false)
        }
        Err(Error::NotFound) if flags.contains(OpenFlags::CREATE) => {
            let (directory, parent) = resolve_parent(Some(base), path)?;
            // A trailing separator names a directory, so it can never create a
            // regular file.
            if parent.trailing_slash {
                return Err(Error::IsDirectory);
            }
            match directory
                .vnode
                .create(parent.name, CreateKind::Regular, mode)
            {
                Ok(vnode) => (
                    PathAnchor {
                        vnode,
                        mount: directory.mount,
                    },
                    true,
                ),
                Err(Error::AlreadyExists) if !flags.contains(OpenFlags::EXCLUSIVE) => (
                    PathAnchor {
                        vnode: directory.vnode.lookup(parent.name)?,
                        mount: directory.mount,
                    },
                    false,
                ),
                Err(error) => return Err(error),
            }
        }
        Err(error) => return Err(error),
    };
    let vnode = anchor.vnode().clone();

    if flags.contains(OpenFlags::NOFOLLOW) && vnode.kind() == VnodeKind::Symlink {
        return Err(Error::SymlinkLoop);
    }
    if flags.contains(OpenFlags::DIRECTORY) && vnode.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }
    if vnode.kind() == VnodeKind::Directory && flags.contains(OpenFlags::WRITE) {
        return Err(Error::IsDirectory);
    }
    // A file created by this call is accessible regardless of the mode it was
    // created with, matching the POSIX open description.
    if !created {
        check_access(&vnode, flags)?;
    }

    OpenFile::new(anchor, flags)
}

/// Creates an empty regular file.
pub fn create_file(path: &[u8], mode: u16) -> Result<Vnode> {
    let root = root_anchor()?;
    let (directory, parent) = resolve_parent(Some(&root), path)?;
    if parent.trailing_slash {
        return Err(Error::IsDirectory);
    }
    directory
        .vnode
        .create(parent.name, CreateKind::Regular, mode)
}

/// Creates an empty directory.
pub fn create_dir(path: &[u8], mode: u16) -> Result<Vnode> {
    let root = root_anchor()?;
    create_dir_at(&root, path, mode)
}

/// Creates an empty directory relative to a stable directory anchor.
pub(crate) fn create_dir_at(base: &PathAnchor, path: &[u8], mode: u16) -> Result<Vnode> {
    let (directory, parent) = resolve_parent(Some(base), path)?;
    directory
        .vnode
        .create(parent.name, CreateKind::Directory, mode)
}

/// Creates a symbolic link.
pub fn symlink(target: &[u8], path: &[u8]) -> Result<Vnode> {
    let root = root_anchor()?;
    symlink_at(target, &root, path)
}

/// Creates a symbolic link relative to a stable directory anchor.
pub(crate) fn symlink_at(target: &[u8], base: &PathAnchor, path: &[u8]) -> Result<Vnode> {
    let (directory, parent) = resolve_parent(Some(base), path)?;
    // A symbolic link is not a directory, so a trailing separator cannot name
    // the object being created.
    if parent.trailing_slash {
        return Err(Error::NotDirectory);
    }
    path::validate_symlink_target(target)?;
    directory.vnode.create(
        parent.name,
        CreateKind::Symlink(Box::<[u8]>::from(target)),
        0o777,
    )
}

/// Adds a hard link to an existing non-directory vnode.
pub fn link(existing: &[u8], new_path: &[u8]) -> Result<()> {
    let root = root_anchor()?;
    link_at(&root, existing, true, &root, new_path)
}

/// Adds a hard link using independently anchored source and destination paths.
pub(crate) fn link_at(
    existing_base: &PathAnchor,
    existing: &[u8],
    follow_existing: bool,
    new_base: &PathAnchor,
    new_path: &[u8],
) -> Result<()> {
    let target = resolve(Some(existing_base), existing, follow_existing)?;
    if target.vnode.kind() == VnodeKind::Directory {
        return Err(Error::PermissionDenied);
    }
    let (directory, parent) = resolve_parent(Some(new_base), new_path)?;
    if parent.trailing_slash {
        return Err(Error::NotDirectory);
    }
    if target.mount.id != directory.mount.id {
        return Err(Error::CrossDevice);
    }
    directory.vnode.link(parent.name, &target.vnode)
}

/// Removes a non-directory path.
pub fn unlink(path: &[u8]) -> Result<()> {
    let root = root_anchor()?;
    unlink_at(&root, path, false)
}

/// Removes a path relative to a stable directory anchor.
pub(crate) fn unlink_at(base: &PathAnchor, path: &[u8], remove_directory: bool) -> Result<()> {
    let namespace = namespace()?;
    let (directory, parent) = resolve_parent(Some(base), path)?;
    // The target is resolved only when a check needs it, so an ordinary removal
    // costs a single directory lookup inside the filesystem.
    if parent.trailing_slash || COVERED_MOUNTS.load(Ordering::Acquire) != 0 {
        let target = directory.vnode.lookup(parent.name)?;
        // A trailing separator names a directory; removing a non-directory
        // through such a path is a type mismatch rather than a missing entry.
        if parent.trailing_slash && target.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        // The namespace lock guards only the mount table. Holding it across the
        // filesystem operation would serialise every removal in the system and
        // invert the namespace/filesystem lock order.
        if namespace.is_mount_point(target.key()) {
            return Err(Error::Busy);
        }
    }
    directory.vnode.unlink(parent.name, remove_directory)
}

/// Removes an empty directory.
pub fn remove_dir(path: &[u8]) -> Result<()> {
    let root = root_anchor()?;
    unlink_at(&root, path, true)
}

/// Atomically renames or replaces a path within one filesystem.
pub fn rename(source: &[u8], target: &[u8]) -> Result<()> {
    let root = root_anchor()?;
    rename_at(&root, source, &root, target)
}

/// Atomically renames paths using independently anchored directories.
pub(crate) fn rename_at(
    source_base: &PathAnchor,
    source: &[u8],
    target_base: &PathAnchor,
    target: &[u8],
) -> Result<()> {
    let namespace = namespace()?;
    let (source_directory, source_parent) = resolve_parent(Some(source_base), source)?;
    let (target_directory, target_parent) = resolve_parent(Some(target_base), target)?;
    if source_directory.mount.id != target_directory.mount.id {
        return Err(Error::CrossDevice);
    }

    // As with removal, the endpoints are resolved only when a check needs them.
    if source_parent.trailing_slash
        || target_parent.trailing_slash
        || COVERED_MOUNTS.load(Ordering::Acquire) != 0
    {
        let source_vnode = source_directory.vnode.lookup(source_parent.name)?;
        let target_vnode = target_directory.vnode.lookup(target_parent.name).ok();
        if source_parent.trailing_slash && source_vnode.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        // A trailing separator on the destination requires a directory, whether
        // it already exists or is about to be created by the rename.
        if target_parent.trailing_slash {
            let directory = target_vnode
                .as_ref()
                .map_or_else(|| source_vnode.kind(), Vnode::kind);
            if directory != VnodeKind::Directory {
                return Err(Error::NotDirectory);
            }
        }
        if namespace.is_mount_point(source_vnode.key())
            || target_vnode
                .as_ref()
                .is_some_and(|vnode| namespace.is_mount_point(vnode.key()))
        {
            return Err(Error::Busy);
        }
    }

    source_directory.vnode.rename(
        source_parent.name,
        &target_directory.vnode,
        target_parent.name,
    )
}

/// Mounts a filesystem over an existing directory.
pub fn mount(path: &[u8], filesystem: FileSystemRef) -> Result<MountId> {
    let namespace = namespace()?;
    let (parent, split) = resolve_parent(None, path)?;
    let covered = parent.vnode.lookup(split.name)?;
    if covered.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }

    let mut state = namespace.state.lock();
    if state.mounted_at.contains_key(&covered.key()) {
        return Err(Error::Busy);
    }
    let mount = Mount::child(filesystem, covered.clone(), &parent.mount)?;
    let id = mount.id;
    state.mounted_at.insert(covered.key(), mount);
    COVERED_MOUNTS.store(state.mounted_at.len(), Ordering::Release);
    Ok(id)
}

/// Detaches the filesystem mounted at `path`.
pub fn unmount(path: &[u8]) -> Result<()> {
    let namespace = namespace()?;
    let anchor = resolve(None, path, true)?;
    let mut state = namespace.state.lock();
    // Only the root of a covering mount is a mount point; anything else is not
    // something that can be detached.
    if anchor.vnode.key() != anchor.mount.root.key() {
        return Err(Error::InvalidArgument);
    }
    let Some(covered) = anchor.mount.covered.as_ref() else {
        // The root mount covers nothing and cannot be detached.
        return Err(Error::InvalidArgument);
    };
    // A mount covered by another mount must be detached from the top down.
    if state.mounted_at.contains_key(&anchor.mount.root.key()) {
        return Err(Error::Busy);
    }
    let key = covered.key();
    let Some(mount) = state.mounted_at.get(&key) else {
        return Err(Error::InvalidArgument);
    };
    if mount.id != anchor.mount.id {
        return Err(Error::InvalidArgument);
    }
    // The traversal above holds one reference in `anchor`, and the table holds
    // the other. Anything else means the mount is still in use.
    if Arc::strong_count(mount) > 2 {
        return Err(Error::Busy);
    }
    state.mounted_at.remove(&key);
    COVERED_MOUNTS.store(state.mounted_at.len(), Ordering::Release);
    Ok(())
}

/// Returns filesystem statistics for the mount containing `path`.
pub fn statfs(path: &[u8]) -> Result<StatFs> {
    Ok(resolve(None, path, true)?.mount.filesystem.statfs())
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

/// Rejects an access mode the vnode permission bits do not grant.
///
/// There is no per-process credential, so a mode bit set in any of the three
/// permission triads grants the corresponding access.
fn check_access(vnode: &Vnode, flags: OpenFlags) -> Result<()> {
    let mode = vnode.getattr()?.mode;
    if flags.contains(OpenFlags::READ) && mode & 0o444 == 0 {
        return Err(Error::PermissionDenied);
    }
    if flags.contains(OpenFlags::WRITE) && mode & 0o222 == 0 {
        return Err(Error::PermissionDenied);
    }
    Ok(())
}

/// Rejects traversal through a directory that grants no search permission.
fn check_search(directory: &Vnode) -> Result<()> {
    if directory.getattr()?.mode & 0o111 == 0 {
        return Err(Error::PermissionDenied);
    }
    Ok(())
}

fn resolve_parent<'a>(
    base: Option<&PathAnchor>,
    path: &'a [u8],
) -> Result<(PathAnchor, path::ParentPath<'a>)> {
    let parent = path::split_parent(path)?;
    Ok((resolve(base, parent.parent, true)?, parent))
}

fn resolve(base: Option<&PathAnchor>, path: &[u8], follow_final: bool) -> Result<PathAnchor> {
    let namespace = namespace()?;
    let parsed = path::parse(path)?;
    let mut current = if parsed.absolute {
        root_anchor()?
    } else if let Some(base) = base {
        base.clone()
    } else {
        root_anchor()?
    };
    let mut components = parsed.components;
    // A path that ends with a separator resolves as though a trailing `.` were
    // appended, so the final component is always followed and must be a
    // directory.
    let mut require_directory = parsed.trailing_slash;
    let follow_final = follow_final || parsed.trailing_slash;
    // Only symbolic-link expansion needs owned components; an ordinary lookup
    // never allocates.
    let mut pending: VecDeque<Box<[u8]>> = VecDeque::new();
    let mut symlink_depth = 0usize;

    loop {
        let owned = pending.pop_front();
        let component: &[u8] = match &owned {
            Some(value) => value,
            None => match components.next() {
                Some(value) => value,
                None => break,
            },
        };

        if component == b"." {
            continue;
        }
        if component == b".." {
            current = ascend(current)?;
            continue;
        }

        if current.vnode.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        check_search(&current.vnode)?;
        let parent = current.vnode.clone();
        let child = parent.lookup(component)?;
        let final_component = pending.is_empty() && components.peek().is_none();

        if child.kind() == VnodeKind::Symlink && (follow_final || !final_component) {
            symlink_depth += 1;
            if symlink_depth > MAX_SYMLINK_DEPTH {
                return Err(Error::SymlinkLoop);
            }
            let link = child.readlink()?;
            let target = path::parse(&link)?;
            require_directory |= target.trailing_slash && final_component;
            if target.absolute {
                current = root_anchor()?;
            } else {
                current.vnode = parent;
            }
            let expanded: Vec<Box<[u8]>> = target.components.map(Box::<[u8]>::from).collect();
            for component in expanded.into_iter().rev() {
                pending.push_front(component);
            }
            continue;
        }

        current.vnode = child;
        if let Some(mount) = namespace.mounted_at(current.vnode.key()) {
            current.vnode = mount.root.clone();
            current.mount = mount;
        }
    }

    if require_directory && current.vnode.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }
    Ok(current)
}

fn ascend(mut current: PathAnchor) -> Result<PathAnchor> {
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
