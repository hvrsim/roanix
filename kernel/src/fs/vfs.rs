//! Global mount namespace, path traversal, and convenience VFS operations.

use alloc::{
    collections::{BTreeMap, VecDeque},
    format,
    sync::Arc,
    vec::Vec,
};
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use crate::sys::{
    event::Event,
    sync::{Once, RwLock},
};

use super::{
    error::{Error, Result},
    file::{FileRef, OpenFile, OpenFlags},
    location::{self, Key as LocationKey, Location},
    path::{self, MAX_SYMLINK_DEPTH},
    vnode::{CreateKind, FileSystemRef, StatFs, Vnode, VnodeKey, VnodeKind},
};

static NEXT_MOUNT_ID: AtomicU64 = AtomicU64::new(1);
static NAMESPACE: Once<Namespace> = Once::new();
static MOUNT_CHANGED: Event = Event::new();

/// Notifies procfs mount-table watchers after a committed namespace change.
pub(super) fn mount_changed() -> &'static Event {
    &MOUNT_CHANGED
}

/// Number of filesystems mounted over a directory.
///
/// Mutation checks consult this before checking mount membership. Path walks
/// use the finer per-vnode covered hint to skip the shared namespace lock.
static COVERED_MOUNTS: AtomicUsize = AtomicUsize::new(0);

/// Stable identifier for one namespace mount.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct MountId(u64);

struct Mount {
    id: MountId,
    filesystem: FileSystemRef,
    root: Vnode,
    source: Vec<u8>,
    covered: Option<Vnode>,
    // Children pin their parent. The namespace owns both; parents never own
    // children, so this introduces no cycle and prevents detaching a subtree.
    parent: Option<MountRef>,
}

type MountRef = Arc<Mount>;

struct NamespaceState {
    root: MountRef,
    mounted_at: BTreeMap<VnodeKey, MountRef>,
}

struct Namespace {
    state: RwLock<NamespaceState>,
}

#[derive(Clone)]
pub(crate) struct PathAnchor {
    vnode: Vnode,
    mount: MountRef,
    location: Option<Arc<Location>>,
}

impl PathAnchor {
    /// Returns the vnode at this stable namespace location.
    pub(crate) fn vnode(&self) -> &Vnode {
        &self.vnode
    }

    /// Identifies the mount that supplied this namespace location.
    pub(crate) fn mount_id(&self) -> u64 {
        self.mount.id.0
    }

    /// Returns the filesystem that supplied this namespace location.
    pub(crate) fn filesystem(&self) -> &FileSystemRef {
        &self.mount.filesystem
    }
}

impl Mount {
    fn root(filesystem: FileSystemRef, source: &[u8]) -> Result<MountRef> {
        let root = filesystem.root();
        if root.kind() != VnodeKind::Directory || root.key().filesystem != filesystem.id() {
            return Err(Error::NotDirectory);
        }
        Ok(Arc::new(Self {
            id: MountId(NEXT_MOUNT_ID.fetch_add(1, Ordering::Relaxed)),
            filesystem,
            root,
            source: copy_bytes(source)?,
            covered: None,
            parent: None,
        }))
    }

    fn child(
        filesystem: FileSystemRef,
        covered: Vnode,
        parent: &MountRef,
        source: &[u8],
    ) -> Result<MountRef> {
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
            source: copy_bytes(source)?,
            covered: Some(covered),
            parent: Some(parent.clone()),
        }))
    }
}

impl Namespace {
    fn new(filesystem: FileSystemRef) -> Result<Self> {
        Ok(Self {
            state: RwLock::new(NamespaceState {
                root: Mount::root(filesystem.clone(), filesystem.name().as_bytes())?,
                mounted_at: BTreeMap::new(),
            }),
        })
    }

    fn root(&self) -> MountRef {
        self.state.read().root.clone()
    }

    fn mounted_at(&self, vnode: &Vnode) -> Option<MountRef> {
        if !vnode.is_covered() {
            return None;
        }
        self.state.read().mounted_at.get(&vnode.key()).cloned()
    }

    fn is_mount_point(&self, key: VnodeKey) -> bool {
        if COVERED_MOUNTS.load(Ordering::Acquire) == 0 {
            return false;
        }
        self.state.read().mounted_at.contains_key(&key)
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

/// Replaces the bootstrap root and drops every mount attached to it.
///
/// This is intentionally narrower than a general-purpose `pivot_root`: early
/// userspace has no reason to retain the initramfs once the real system volume
/// is available. Callers must recreate any kernel filesystems they need below
/// the new root after this operation succeeds.
pub(crate) fn replace_root(filesystem: FileSystemRef, source: &[u8]) -> Result<()> {
    let _topology = super::namespace::topology();
    let namespace = namespace()?;
    let root = Mount::root(filesystem, source)?;
    let mut state = namespace.state.write();
    let old_root = core::mem::replace(&mut state.root, root);
    let old_mounts = core::mem::take(&mut state.mounted_at);
    for mount in old_mounts.values() {
        if let Some(covered) = &mount.covered {
            covered.set_covered(false);
        }
    }
    COVERED_MOUNTS.store(0, Ordering::Release);
    drop(state);
    drop(_topology);
    // Cache references and VM objects can run destructors; keep them out of
    // the namespace lock, just like filesystem callbacks during traversal.
    super::namecache::purge(old_root.filesystem.id());
    for mount in old_mounts.values() {
        super::namecache::purge(mount.filesystem.id());
    }
    MOUNT_CHANGED.wake_all();
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
        location: Some(location::root()),
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
    let flags = if flags.contains(OpenFlags::PATH) {
        flags & (OpenFlags::PATH | OpenFlags::DIRECTORY | OpenFlags::NOFOLLOW)
    } else {
        flags
    };
    if !flags.intersects(OpenFlags::READ | OpenFlags::WRITE | OpenFlags::PATH) {
        return Err(Error::InvalidArgument);
    }
    if flags.contains(OpenFlags::TRUNCATE) && !flags.contains(OpenFlags::WRITE) {
        return Err(Error::PermissionDenied);
    }

    // Registration must be atomic with respect to unlink and rename.
    let mutation = super::namespace::mutation();
    let follow_final = !flags.contains(OpenFlags::NOFOLLOW);
    let (anchor, created) = match resolve_tracked(Some(base), path, follow_final, true) {
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
                        location: Some(location::retain(
                            tracked_location(&directory)?,
                            location_key(&directory, parent.name),
                            vnode.key(),
                        )),
                        vnode,
                        mount: directory.mount,
                    },
                    true,
                ),
                Err(Error::AlreadyExists) if !flags.contains(OpenFlags::EXCLUSIVE) => (
                    PathAnchor {
                        location: Some(location::retain(
                            tracked_location(&directory)?,
                            location_key(&directory, parent.name),
                            directory.vnode.lookup(parent.name)?.key(),
                        )),
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
    let vnode = anchor.vnode();

    if flags.contains(OpenFlags::NOFOLLOW)
        && !flags.contains(OpenFlags::PATH)
        && vnode.kind() == VnodeKind::Symlink
    {
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
        check_access(vnode, flags)?;
    }

    drop(mutation);
    OpenFile::new(anchor, flags)
}

/// Creates an empty regular file.
pub fn create_file(path: &[u8], mode: u16) -> Result<Vnode> {
    let _mutation = super::namespace::mutation();
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

/// Creates an empty regular file directly below a stable directory anchor.
pub(crate) fn create_file_at(base: &PathAnchor, name: &[u8], mode: u16) -> Result<Vnode> {
    let _mutation = super::namespace::mutation();
    base.vnode().create(name, CreateKind::Regular, mode)
}

/// Creates a socket rendezvous inode and retains its mount location.
pub(crate) fn create_socket_at(
    base: &PathAnchor,
    path: &[u8],
    mode: u16,
) -> Result<(Vnode, PathAnchor)> {
    let _mutation = super::namespace::mutation();
    let (directory, parent) = resolve_parent(Some(base), path)?;
    if parent.trailing_slash {
        return Err(Error::NotDirectory);
    }
    let attributes = directory.vnode.getattr()?;
    if attributes.mode & 0o222 == 0 || attributes.mode & 0o111 == 0 {
        return Err(Error::PermissionDenied);
    }
    let node = directory
        .vnode
        .create(parent.name, CreateKind::Socket, mode)?;
    Ok((node, directory))
}

/// Creates a FIFO under a retained directory anchor.
pub(crate) fn create_fifo_at(base: &PathAnchor, path: &[u8], mode: u16) -> Result<Vnode> {
    let _mutation = super::namespace::mutation();
    let (directory, parent) = resolve_parent(Some(base), path)?;
    if parent.trailing_slash {
        return Err(Error::NotDirectory);
    }
    let attrs = directory.vnode.getattr()?;
    if attrs.mode & 0o222 == 0 || attrs.mode & 0o111 == 0 {
        return Err(Error::PermissionDenied);
    }
    directory.vnode.create(parent.name, CreateKind::Fifo, mode)
}

/// Creates an empty directory directly below a stable directory anchor.
pub(crate) fn create_dir_at(base: &PathAnchor, path: &[u8], mode: u16) -> Result<Vnode> {
    let _mutation = super::namespace::mutation();
    // Existing names, including mount roots and dot components, fail before
    // asking a read-only backend to create anything. Backends still arbitrate
    // concurrent creators after this lookup.
    match resolve(Some(base), path, false) {
        Ok(_) => return Err(Error::AlreadyExists),
        Err(Error::NotFound) => {}
        Err(error) => return Err(error),
    }
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
    let _mutation = super::namespace::mutation();
    let (directory, parent) = resolve_parent(Some(base), path)?;
    // A symbolic link is not a directory, so a trailing separator cannot name
    // the object being created.
    if parent.trailing_slash {
        return Err(Error::NotDirectory);
    }
    path::validate_symlink_target(target)?;
    directory
        .vnode
        .create(parent.name, CreateKind::Symlink(copy_bytes(target)?), 0o777)
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
    let _mutation = super::namespace::mutation();
    let target = if existing.is_empty() {
        existing_base.clone()
    } else {
        resolve(Some(existing_base), existing, follow_existing)?
    };
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
    let _topology = super::namespace::topology();
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
        // invert the namespace/filesystem lock order. The separate mutation
        // gate excludes mount publication until this operation has finished.
        if namespace.is_mount_point(target.key()) {
            return Err(Error::Busy);
        }
    }
    let retained = location::find(&location_key(&directory, parent.name));
    directory.vnode.unlink(parent.name, remove_directory)?;
    if let Some(retained) = retained {
        retained.unlink();
    }
    Ok(())
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
    let _topology = super::namespace::topology();
    let namespace = namespace()?;
    let (source_directory, source_parent) = resolve_parent(Some(source_base), source)?;
    let (target_directory, target_parent) = resolve_parent(Some(target_base), target)?;
    if source_directory.mount.id != target_directory.mount.id {
        return Err(Error::CrossDevice);
    }

    let source_vnode = source_directory.vnode.lookup(source_parent.name)?;
    let target_vnode = match target_directory.vnode.lookup(target_parent.name) {
        Ok(vnode) => Some(vnode),
        Err(Error::NotFound) => None,
        Err(error) => return Err(error),
    };
    if source_parent.trailing_slash && source_vnode.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }
    if target_parent.trailing_slash {
        let kind = target_vnode
            .as_ref()
            .map_or_else(|| source_vnode.kind(), Vnode::kind);
        if kind != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
    }
    if COVERED_MOUNTS.load(Ordering::Acquire) != 0
        && (namespace.is_mount_point(source_vnode.key())
            || target_vnode
                .as_ref()
                .is_some_and(|vnode| namespace.is_mount_point(vnode.key())))
    {
        return Err(Error::Busy);
    }
    let same_inode = target_vnode
        .as_ref()
        .is_some_and(|node| node.key() == source_vnode.key());
    let source_location = location::find(&location_key(&source_directory, source_parent.name));
    let target_key = location_key(&target_directory, target_parent.name);
    let target_location = location::find(&target_key);
    // Prepare every fallible path operation before committing the rename.
    let new_parent = source_location
        .as_ref()
        .map(|_| tracked_location(&target_directory))
        .transpose()?;
    source_directory.vnode.rename(
        source_parent.name,
        &target_directory.vnode,
        target_parent.name,
    )?;
    if !same_inode {
        if let Some(target) = target_location {
            target.unlink();
        }
        if let (Some(source), Some(parent)) = (source_location, new_parent) {
            source.rename(parent, target_key);
        }
    }
    if !same_inode && source_vnode.kind() == VnodeKind::Directory {
        drop(_topology);
        MOUNT_CHANGED.wake_all();
    }
    Ok(())
}

/// Mounts a filesystem over an existing directory.
pub fn mount(path: &[u8], filesystem: FileSystemRef) -> Result<MountId> {
    let source = copy_bytes(filesystem.name().as_bytes())?;
    mount_with_source(path, filesystem, &source)
}

/// Mounts a filesystem while retaining the source name reported in procfs.
pub(crate) fn mount_with_source(
    path: &[u8],
    filesystem: FileSystemRef,
    source: &[u8],
) -> Result<MountId> {
    let _topology = super::namespace::topology();
    let namespace = namespace()?;
    let (parent, split) = resolve_parent(None, path)?;
    let mut covered = parent.vnode.lookup(split.name)?;
    let parent_mount = if covered.kind() == VnodeKind::Symlink {
        let target = resolve(None, path, true)?;
        if target.vnode.key() == target.mount.root.key() {
            return Err(Error::Busy);
        }
        covered = target.vnode;
        target.mount
    } else {
        parent.mount
    };
    if covered.kind() != VnodeKind::Directory {
        return Err(Error::NotDirectory);
    }

    // Prepare the filesystem before taking the table lock. The topology gate
    // keeps both the parent mount and covered directory attached throughout.
    let mount = Mount::child(filesystem, covered.clone(), &parent_mount, source)?;
    let mut state = namespace.state.write();
    if state.mounted_at.contains_key(&covered.key()) {
        return Err(Error::Busy);
    }
    let id = mount.id;
    state.mounted_at.insert(covered.key(), mount);
    // A walker seeing true must take the shared namespace lock and therefore
    // observes the entry above. A false observation may precede this mount.
    covered.set_covered(true);
    COVERED_MOUNTS.store(state.mounted_at.len(), Ordering::Release);
    drop(state);
    drop(_topology);
    MOUNT_CHANGED.wake_all();
    Ok(id)
}

/// Renders the live global mount namespace for procfs. No filesystem callback
/// runs while the namespace table is locked. The topology gate also prevents
/// renames from making a directory path inconsistent during the snapshot.
pub(super) fn proc_mounts(mountinfo: bool) -> Result<Vec<u8>> {
    let _topology = super::namespace::topology();
    let namespace = namespace()?;
    let mounts = {
        let state = namespace.state.read();
        let mut mounts = Vec::new();
        mounts
            .try_reserve_exact(state.mounted_at.len() + 1)
            .map_err(|_| Error::OutOfMemory)?;
        mounts.push(state.root.clone());
        mounts.extend(state.mounted_at.values().cloned());
        mounts
    };
    let mut output = Vec::new();
    for mount in mounts {
        let path = match (&mount.covered, &mount.parent) {
            (Some(covered), Some(parent)) => directory_path(PathAnchor {
                vnode: covered.clone(),
                mount: parent.clone(),
                location: None,
            })?,
            _ => b"/".to_vec(),
        };
        let options = if mount.filesystem.read_only() {
            b"ro".as_slice()
        } else {
            b"rw"
        };
        if mountinfo {
            let device = mount.filesystem.id().get();
            let major = ((device >> 8) & 0xfff) | ((device >> 32) & 0xfffff000);
            let minor = (device & 0xff) | ((device >> 12) & 0xffffff00);
            let parent = mount
                .parent
                .as_ref()
                .map_or(mount.id.0, |parent| parent.id.0);
            output.extend_from_slice(
                format!("{} {parent} {major}:{minor} / ", mount.id.0).as_bytes(),
            );
            escape_mount_field(&mut output, &path);
            output.push(b' ');
            output.extend_from_slice(options);
            output.extend_from_slice(b" - ");
            output.extend_from_slice(mount.filesystem.name().as_bytes());
            output.push(b' ');
            escape_mount_field(&mut output, &mount.source);
            output.push(b' ');
            output.extend_from_slice(options);
        } else {
            escape_mount_field(&mut output, &mount.source);
            output.push(b' ');
            escape_mount_field(&mut output, &path);
            output.push(b' ');
            output.extend_from_slice(mount.filesystem.name().as_bytes());
            output.push(b' ');
            output.extend_from_slice(options);
            output.extend_from_slice(b" 0 0");
        }
        output.push(b'\n');
    }
    Ok(output)
}

fn escape_mount_field(output: &mut Vec<u8>, field: &[u8]) {
    for byte in field {
        match byte {
            b' ' => output.extend_from_slice(b"\\040"),
            b'\t' => output.extend_from_slice(b"\\011"),
            b'\n' => output.extend_from_slice(b"\\012"),
            b'\\' => output.extend_from_slice(b"\\134"),
            byte => output.push(*byte),
        }
    }
}

// Called with the topology gate held, so namespace mutations cannot detach an
// ancestor or change its name while we find the directory's canonical path.
pub(super) fn proc_path(anchor: PathAnchor) -> Result<Vec<u8>> {
    let _topology = super::namespace::topology();
    if let Some(location) = &anchor.location {
        return location.path();
    }
    directory_path(anchor)
}

fn location_key(directory: &PathAnchor, name: &[u8]) -> LocationKey {
    LocationKey::new(directory.mount.id, directory.vnode.key(), name)
}

// Relative opens can start at an untracked cwd. Reconstruct only that base;
// subsequent components retain their edges directly during the path walk.
// The caller holds either the namespace mutation or topology gate.
fn tracked_location(anchor: &PathAnchor) -> Result<Arc<Location>> {
    if let Some(location) = &anchor.location {
        return Ok(location.clone());
    }
    let mut current = anchor.clone();
    let mut edges = Vec::new();
    let mut length = 1usize;
    let root = loop {
        if let Some(location) = current.location {
            break location;
        }
        if current.vnode.key() == current.mount.root.key() {
            match (&current.mount.covered, &current.mount.parent) {
                (Some(covered), Some(parent)) => {
                    current.vnode = covered.clone();
                    current.mount = parent.clone();
                    continue;
                }
                _ => break location::root(),
            }
        }
        let child = current.vnode.key();
        let parent = current.vnode.parent()?;
        let name = child_name(&parent, child)?;
        length = length
            .checked_add(name.len() + 1)
            .ok_or(Error::NameTooLong)?;
        if length > path::MAX_PATH_LEN {
            return Err(Error::NameTooLong);
        }
        current.vnode = parent;
        edges.push((location_key(&current, &name), child));
    };
    Ok(edges.into_iter().rev().fold(root, |parent, (key, child)| {
        location::retain(parent, key, child)
    }))
}

// Filesystems expose parent identity for directories; the name comes from
// that parent's entries. This also respects filesystems without name caching.
fn child_name(parent: &Vnode, child: VnodeKey) -> Result<Vec<u8>> {
    let mut cursor = 0;
    loop {
        let (entries, next) = parent.readdir(cursor, 64)?;
        if let Some(entry) = entries.iter().find(|entry| {
            entry.key == child && entry.name.as_ref() != b"." && entry.name.as_ref() != b".."
        }) {
            return copy_bytes(&entry.name);
        }
        if entries.is_empty() || next == cursor {
            return Err(Error::NotFound);
        }
        cursor = next;
    }
}

fn directory_path(mut anchor: PathAnchor) -> Result<Vec<u8>> {
    let mut names = Vec::new();
    let mut bytes = 1usize;
    loop {
        if anchor.vnode.key() == anchor.mount.root.key() {
            match (&anchor.mount.covered, &anchor.mount.parent) {
                (Some(covered), Some(parent)) => {
                    anchor = PathAnchor {
                        vnode: covered.clone(),
                        mount: parent.clone(),
                        location: None,
                    };
                    continue;
                }
                _ => break,
            }
        }
        let parent = anchor.vnode.parent()?;
        let name = child_name(&parent, anchor.vnode.key())?;
        bytes = bytes
            .checked_add(name.len() + 1)
            .ok_or(Error::NameTooLong)?;
        if bytes > path::MAX_PATH_LEN {
            return Err(Error::NameTooLong);
        }
        names.push(name);
        anchor.vnode = parent;
    }
    let mut output = Vec::new();
    output
        .try_reserve_exact(bytes)
        .map_err(|_| Error::OutOfMemory)?;
    output.push(b'/');
    for name in names.iter().rev() {
        if output.len() > 1 {
            output.push(b'/');
        }
        output.extend_from_slice(name);
    }
    Ok(output)
}

/// Detaches the filesystem mounted at `path`.
pub fn unmount(path: &[u8], follow_final: bool) -> Result<()> {
    let _topology = super::namespace::topology();
    let namespace = namespace()?;
    let anchor = resolve(None, path, follow_final)?;
    let mut state = namespace.state.write();
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
    let detached = state.mounted_at.remove(&key);
    covered.set_covered(false);
    COVERED_MOUNTS.store(state.mounted_at.len(), Ordering::Release);
    drop(state);
    drop(_topology);
    super::namecache::purge(anchor.mount.filesystem.id());
    drop(detached);
    MOUNT_CHANGED.wake_all();
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
        let state = namespace.state.read();
        let mut filesystems = Vec::new();
        filesystems
            .try_reserve(state.mounted_at.len().saturating_add(1))
            .map_err(|_| Error::OutOfMemory)?;
        filesystems.push(state.root.filesystem.clone());
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
    resolve_tracked(base, path, follow_final, false)
}

fn resolve_tracked(
    base: Option<&PathAnchor>,
    path: &[u8],
    follow_final: bool,
    track: bool,
) -> Result<PathAnchor> {
    let namespace = namespace()?;
    let parsed = path::parse(path)?;
    let mut current = if parsed.absolute {
        root_anchor()?
    } else if let Some(base) = base {
        base.clone()
    } else {
        root_anchor()?
    };
    if track {
        current.location = Some(tracked_location(&current)?);
    }
    let mut components = parsed.components;
    // A path that ends with a separator resolves as though a trailing `.` were
    // appended, so the final component is always followed and must be a
    // directory.
    let mut require_directory = parsed.trailing_slash;
    let follow_final = follow_final || parsed.trailing_slash;
    // Only symbolic-link expansion needs owned path components; cache hits
    // on ordinary paths do not allocate.
    let mut pending: VecDeque<Vec<u8>> = VecDeque::new();
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

        if current.vnode.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        check_search(&current.vnode)?;
        if component == b"." {
            continue;
        }
        if component == b".." {
            current = ascend(current)?;
            continue;
        }
        let parent = current.vnode.clone();
        let child = parent.lookup(component)?;
        let final_component = pending.is_empty() && components.peek().is_none();

        if child.kind() == VnodeKind::Symlink && (follow_final || !final_component) {
            symlink_depth += 1;
            if symlink_depth > MAX_SYMLINK_DEPTH {
                return Err(Error::SymlinkLoop);
            }
            if let Some(file) = child.file_link()? {
                current = file.path_anchor();
                continue;
            }
            let link = child.readlink()?;
            let target = path::parse(&link)?;
            require_directory |= target.trailing_slash && final_component;
            if target.absolute {
                current = root_anchor()?;
            } else {
                current.vnode = parent;
            }
            let component_count = target.components.clone().count();
            let mut expanded = Vec::new();
            expanded
                .try_reserve_exact(component_count)
                .map_err(|_| Error::OutOfMemory)?;
            expanded.extend(target.components);
            pending
                .try_reserve(component_count)
                .map_err(|_| Error::OutOfMemory)?;
            for component in expanded.into_iter().rev() {
                pending.push_front(copy_bytes(component)?);
            }
            continue;
        }

        current.location = if track {
            Some(location::retain(
                tracked_location(&current)?,
                location_key(&current, component),
                child.key(),
            ))
        } else {
            None
        };
        current.vnode = child;
        if let Some(mount) = namespace.mounted_at(&current.vnode) {
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
    current.location = current
        .location
        .as_ref()
        .and_then(|location| location.parent());
    if current.vnode.key() == current.mount.root.key() {
        let Some(covered) = current.mount.covered.as_ref() else {
            return Ok(current);
        };
        let Some(parent_mount) = current.mount.parent.clone() else {
            return Err(Error::Io);
        };
        current.vnode = covered.parent()?;
        current.mount = parent_mount;
        return Ok(current);
    }

    current.vnode = current.vnode.parent()?;
    Ok(current)
}

fn copy_bytes(bytes: &[u8]) -> Result<Vec<u8>> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(bytes.len())
        .map_err(|_| Error::OutOfMemory)?;
    copy.extend_from_slice(bytes);
    Ok(copy)
}
