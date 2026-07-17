//! Sparse temporary filesystem backed by unified page-cache objects.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    any::Any,
    cmp,
    sync::atomic::{AtomicU16, AtomicU64, Ordering},
};

use crate::{
    mem::{self, ObjectKind, PAGE_SIZE, PageAccount, VmObject},
    sys::{
        clock,
        smp::IrqSpinLock,
        sync::{Mutex, Once},
    },
};

use super::{
    error::{Error, Result},
    path,
    vnode::{
        CreateKind, DirEntry, FileSystem, FilesystemId, NodeId, SetAttr, StatFs, Vnode, VnodeAttr,
        VnodeKey, VnodeKind, VnodeOps, VnodeWeak,
    },
};

const TMPFS_NAME: &str = "tmpfs";

/// Memory-backed filesystem instance.
pub struct Tmpfs {
    id: FilesystemId,
    next_node: AtomicU64,
    page_account: Arc<PageAccount>,
    used_nodes: AtomicU64,
    rename_lock: Mutex<()>,
    root: Once<Vnode>,
}

struct TmpfsNode {
    filesystem: Weak<Tmpfs>,
    kind: VnodeKind,
    mode: AtomicU16,
    links: AtomicU64,
    size: AtomicU64,
    accessed_ns: AtomicU64,
    modified_ns: AtomicU64,
    changed_ns: AtomicU64,
    data: TmpfsData,
}

enum TmpfsData {
    File(Mutex<TmpfsFile>),
    Directory(TmpfsDirectory),
    Symlink(Arc<VmObject>),
}

struct TmpfsFile {
    object: Arc<VmObject>,
}

struct TmpfsDirectory {
    parent: IrqSpinLock<VnodeWeak>,
    contents: Mutex<TmpfsDirectoryData>,
}

struct TmpfsDirectoryData {
    entries: BTreeMap<Box<[u8]>, TmpfsDirEntry>,
    cookies: BTreeMap<u64, Box<[u8]>>,
    next_cookie: u64,
}

struct TmpfsDirEntry {
    vnode: Vnode,
    cookie: u64,
}

impl TmpfsDirectoryData {
    fn get(&self, name: &[u8]) -> Option<&Vnode> {
        self.entries.get(name).map(|entry| &entry.vnode)
    }

    fn contains_key(&self, name: &[u8]) -> bool {
        self.entries.contains_key(name)
    }

    fn insert(&mut self, name: &[u8], vnode: Vnode) -> Result<()> {
        if self.entries.contains_key(name) {
            return Err(Error::AlreadyExists);
        }
        self.ensure_insert_capacity()?;
        let cookie = self.next_cookie;
        self.next_cookie += 1;
        let name = Box::<[u8]>::from(name);
        self.cookies.insert(cookie, name.clone());
        self.entries.insert(name, TmpfsDirEntry { vnode, cookie });
        Ok(())
    }

    fn ensure_insert_capacity(&self) -> Result<()> {
        self.next_cookie.checked_add(1).ok_or(Error::NoSpace)?;
        Ok(())
    }

    fn remove(&mut self, name: &[u8]) -> Option<Vnode> {
        let entry = self.entries.remove(name)?;
        self.cookies.remove(&entry.cookie);
        Some(entry.vnode)
    }

    fn rename_within(&mut self, source_name: &[u8], target_name: &[u8]) -> Result<Vnode> {
        let entry = self.entries.remove(source_name).ok_or(Error::NotFound)?;
        self.cookies.remove(&entry.cookie);
        let target_name = Box::<[u8]>::from(target_name);
        self.cookies.insert(entry.cookie, target_name.clone());
        let vnode = entry.vnode.clone();
        self.entries.insert(target_name, entry);
        Ok(vnode)
    }

    fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

impl Tmpfs {
    /// Creates an empty tmpfs filesystem.
    pub fn new() -> Result<Arc<Self>> {
        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            next_node: AtomicU64::new(1),
            page_account: PageAccount::unlimited(),
            used_nodes: AtomicU64::new(0),
            rename_lock: Mutex::new(()),
            root: Once::new(),
        });
        let root = filesystem.allocate_node(CreateKind::Directory, 0o755, VnodeWeak::new())?;
        filesystem.root.call_once(|| root);
        Ok(filesystem)
    }

    fn allocate_node(
        self: &Arc<Self>,
        kind: CreateKind,
        mode: u16,
        parent: VnodeWeak,
    ) -> Result<Vnode> {
        let id = NodeId::new(self.next_node.fetch_add(1, Ordering::Relaxed));
        let now_ns = clock::monotonic_ns();
        let (vnode_kind, size, links, data) = match kind {
            CreateKind::Regular => (
                VnodeKind::Regular,
                0,
                1,
                TmpfsData::File(Mutex::new(TmpfsFile {
                    object: VmObject::with_page_account(
                        ObjectKind::Vnode,
                        self.page_account.clone(),
                    ),
                })),
            ),
            CreateKind::Directory => (
                VnodeKind::Directory,
                0,
                2,
                TmpfsData::Directory(TmpfsDirectory {
                    parent: IrqSpinLock::new(parent),
                    contents: Mutex::new(TmpfsDirectoryData {
                        entries: BTreeMap::new(),
                        cookies: BTreeMap::new(),
                        next_cookie: 2,
                    }),
                }),
            ),
            CreateKind::Symlink(target) => {
                let size = target.len() as u64;
                let object =
                    VmObject::with_page_account(ObjectKind::Vnode, self.page_account.clone());
                let written = match object.write_at(0, &target) {
                    Ok(written) => written,
                    Err(error) => return Err(mem_error(error)),
                };
                if written != target.len() {
                    return Err(Error::NoSpace);
                }
                (VnodeKind::Symlink, size, 1, TmpfsData::Symlink(object))
            }
        };
        let node = TmpfsNode {
            filesystem: Arc::downgrade(self),
            kind: vnode_kind,
            mode: AtomicU16::new(mode),
            links: AtomicU64::new(links),
            size: AtomicU64::new(size),
            accessed_ns: AtomicU64::new(now_ns),
            modified_ns: AtomicU64::new(now_ns),
            changed_ns: AtomicU64::new(now_ns),
            data,
        };
        let vnode = Vnode::new(
            VnodeKey {
                filesystem: self.id,
                node: id,
            },
            vnode_kind,
            Box::new(node),
        );
        self.used_nodes.fetch_add(1, Ordering::AcqRel);
        Ok(vnode)
    }
}

impl FileSystem for Tmpfs {
    fn id(&self) -> FilesystemId {
        self.id
    }

    fn name(&self) -> &'static str {
        TMPFS_NAME
    }

    fn root(&self) -> Vnode {
        self.root
            .get()
            .cloned()
            .expect("tmpfs: root not initialized")
    }

    fn statfs(&self) -> StatFs {
        StatFs {
            total_bytes: u64::MAX,
            used_bytes: self.page_account.used().saturating_mul(PAGE_SIZE),
            total_nodes: u64::MAX,
            used_nodes: self.used_nodes.load(Ordering::Acquire),
        }
    }
}

impl TmpfsNode {
    fn filesystem(&self) -> Result<Arc<Tmpfs>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }

    fn directory(&self) -> Result<&TmpfsDirectory> {
        match &self.data {
            TmpfsData::Directory(directory) => Ok(directory),
            _ => Err(Error::NotDirectory),
        }
    }

    fn file(&self) -> Result<&Mutex<TmpfsFile>> {
        match &self.data {
            TmpfsData::File(file) => Ok(file),
            TmpfsData::Directory(_) => Err(Error::IsDirectory),
            TmpfsData::Symlink(_) => Err(Error::InvalidArgument),
        }
    }

    fn touch_accessed(&self) {
        self.accessed_ns
            .store(clock::monotonic_ns(), Ordering::Release);
    }

    fn touch_modified(&self) {
        let now_ns = clock::monotonic_ns();
        self.modified_ns.store(now_ns, Ordering::Release);
        self.changed_ns.store(now_ns, Ordering::Release);
    }

    fn touch_changed(&self) {
        self.changed_ns
            .store(clock::monotonic_ns(), Ordering::Release);
    }

    fn write_locked(&self, file: &mut TmpfsFile, offset: u64, buffer: &[u8]) -> Result<usize> {
        let end = offset
            .checked_add(buffer.len() as u64)
            .ok_or(Error::FileTooLarge)?;
        let mut written = 0usize;

        while written < buffer.len() {
            let position = offset + written as u64;
            let page_offset = (position % PAGE_SIZE) as usize;
            let count = cmp::min(PAGE_SIZE as usize - page_offset, buffer.len() - written);
            match file
                .object
                .write_at(position, &buffer[written..written + count])
            {
                Ok(count) => written += count,
                Err(_) if written != 0 => {
                    self.size
                        .fetch_max(offset + written as u64, Ordering::AcqRel);
                    self.touch_modified();
                    return Ok(written);
                }
                Err(error) => return Err(mem_error(error)),
            }
        }

        self.size.fetch_max(end, Ordering::AcqRel);
        self.touch_modified();
        Ok(written)
    }

    fn truncate_locked(&self, file: &mut TmpfsFile, size: u64) -> Result<()> {
        let old_size = self.size.load(Ordering::Acquire);
        if size < old_size {
            file.object.truncate(size).map_err(mem_error)?;
        }
        self.size.store(size, Ordering::Release);
        self.touch_modified();
        Ok(())
    }

    fn child_node(vnode: &Vnode) -> Result<&TmpfsNode> {
        vnode.operations_as::<TmpfsNode>().ok_or(Error::CrossDevice)
    }

    fn validate_same_filesystem(&self, vnode: &Vnode) -> Result<()> {
        let filesystem = self.filesystem()?;
        if vnode.key().filesystem != filesystem.id {
            return Err(Error::CrossDevice);
        }
        Ok(())
    }

    fn ensure_directory_move_is_acyclic(source: &Vnode, target_directory: &Vnode) -> Result<()> {
        if source.kind() != VnodeKind::Directory {
            return Ok(());
        }

        let mut ancestor = target_directory.clone();
        loop {
            if ancestor == *source {
                return Err(Error::InvalidArgument);
            }
            let parent = ancestor.parent()?;
            if parent == ancestor {
                return Ok(());
            }
            ancestor = parent;
        }
    }

    fn replace_target(target: Option<&Vnode>, source: &Vnode) -> Result<()> {
        let Some(target) = target else {
            return Ok(());
        };
        if target == source {
            return Ok(());
        }
        match (source.kind(), target.kind()) {
            (VnodeKind::Directory, VnodeKind::Directory) => {}
            (VnodeKind::Directory, _) => return Err(Error::NotDirectory),
            (_, VnodeKind::Directory) => return Err(Error::IsDirectory),
            _ => {}
        }
        Ok(())
    }

    fn remove_link(target: &Vnode) -> Result<()> {
        let node = Self::child_node(target)?;
        if target.kind() == VnodeKind::Directory {
            node.links.store(0, Ordering::Release);
        } else {
            let previous = node.links.fetch_sub(1, Ordering::AcqRel);
            assert!(previous != 0, "tmpfs: link count underflow");
        }
        node.touch_changed();
        Ok(())
    }
}

impl VnodeOps for TmpfsNode {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        Ok(VnodeAttr {
            key: vnode.key(),
            kind: self.kind,
            size: self.size.load(Ordering::Acquire),
            links: self.links.load(Ordering::Acquire),
            mode: self.mode.load(Ordering::Acquire),
            accessed_ns: self.accessed_ns.load(Ordering::Acquire),
            modified_ns: self.modified_ns.load(Ordering::Acquire),
            changed_ns: self.changed_ns.load(Ordering::Acquire),
        })
    }

    fn setattr(&self, vnode: &Vnode, attr: SetAttr) -> Result<()> {
        if let Some(size) = attr.size {
            self.truncate(vnode, size)?;
        }
        if let Some(mode) = attr.mode {
            self.mode.store(mode, Ordering::Release);
            self.touch_changed();
        }
        Ok(())
    }

    fn lookup(&self, directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        let directory_data = self.directory()?;
        if name == b"." {
            return Ok(directory.clone());
        }
        if name == b".." {
            return self.parent(directory);
        }
        path::validate_leaf_name(name)?;
        if self.links.load(Ordering::Acquire) == 0 {
            return Err(Error::NotFound);
        }
        let child = directory_data
            .contents
            .lock()
            .get(name)
            .cloned()
            .ok_or(Error::NotFound)?;
        self.touch_accessed();
        Ok(child)
    }

    fn parent(&self, directory: &Vnode) -> Result<Vnode> {
        let directory_data = self.directory()?;
        Ok(directory_data
            .parent
            .lock()
            .upgrade()
            .unwrap_or_else(|| directory.clone()))
    }

    fn create(&self, directory: &Vnode, name: &[u8], kind: CreateKind, mode: u16) -> Result<Vnode> {
        path::validate_leaf_name(name)?;
        let filesystem = self.filesystem()?;
        let directory_data = self.directory()?;
        let mut entries = directory_data.contents.lock();
        if self.links.load(Ordering::Acquire) == 0 {
            return Err(Error::NotFound);
        }
        if entries.contains_key(name) {
            return Err(Error::AlreadyExists);
        }
        entries.ensure_insert_capacity()?;

        let child = filesystem.allocate_node(kind, mode, directory.downgrade())?;
        if child.kind() == VnodeKind::Directory {
            self.links.fetch_add(1, Ordering::AcqRel);
        }
        entries.insert(name, child.clone())?;
        self.touch_modified();
        Ok(child)
    }

    fn link(&self, _directory: &Vnode, name: &[u8], target: &Vnode) -> Result<()> {
        path::validate_leaf_name(name)?;
        self.validate_same_filesystem(target)?;
        if target.kind() == VnodeKind::Directory {
            return Err(Error::PermissionDenied);
        }

        let mut entries = self.directory()?.contents.lock();
        if self.links.load(Ordering::Acquire) == 0 {
            return Err(Error::NotFound);
        }
        if entries.contains_key(name) {
            return Err(Error::AlreadyExists);
        }
        entries.insert(name, target.clone())?;
        let target_node = Self::child_node(target)?;
        target_node.links.fetch_add(1, Ordering::AcqRel);
        target_node.touch_changed();
        self.touch_modified();
        Ok(())
    }

    fn unlink(&self, _directory: &Vnode, name: &[u8], remove_directory: bool) -> Result<()> {
        path::validate_leaf_name(name)?;
        let filesystem = self.filesystem()?;
        let _rename = remove_directory.then(|| filesystem.rename_lock.lock());
        let mut entries = self.directory()?.contents.lock();
        if self.links.load(Ordering::Acquire) == 0 {
            return Err(Error::NotFound);
        }
        let target = entries.get(name).cloned().ok_or(Error::NotFound)?;

        let target_directory_guard = match (remove_directory, target.kind()) {
            (true, VnodeKind::Directory) => {
                let guard = Self::child_node(&target)?.directory()?.contents.lock();
                if !guard.is_empty() {
                    return Err(Error::NotEmpty);
                }
                Some(guard)
            }
            (true, _) => return Err(Error::NotDirectory),
            (false, VnodeKind::Directory) => return Err(Error::IsDirectory),
            (false, _) => None,
        };

        entries.remove(name).expect("tmpfs: target vanished");
        if target.kind() == VnodeKind::Directory {
            let previous = self.links.fetch_sub(1, Ordering::AcqRel);
            assert!(previous > 2, "tmpfs: parent directory link count underflow");
        }
        Self::remove_link(&target)?;
        drop(target_directory_guard);
        self.touch_modified();
        Ok(())
    }

    fn rename(
        &self,
        source_directory: &Vnode,
        source_name: &[u8],
        target_directory: &Vnode,
        target_name: &[u8],
    ) -> Result<()> {
        path::validate_leaf_name(source_name)?;
        path::validate_leaf_name(target_name)?;
        self.validate_same_filesystem(target_directory)?;

        let filesystem = self.filesystem()?;
        let _rename = filesystem.rename_lock.lock();
        let target_node = Self::child_node(target_directory)?;
        let source_directory_data = self.directory()?;
        let target_directory_data = target_node.directory()?;

        if source_directory == target_directory {
            let mut entries = source_directory_data.contents.lock();
            if self.links.load(Ordering::Acquire) == 0 {
                return Err(Error::NotFound);
            }
            let source = entries.get(source_name).cloned().ok_or(Error::NotFound)?;
            if source_name == target_name {
                return Ok(());
            }
            let target = entries.get(target_name).cloned();
            Self::replace_target(target.as_ref(), &source)?;
            if target.as_ref() == Some(&source) {
                return Ok(());
            }
            let target_directory_guard = if target
                .as_ref()
                .is_some_and(|target| target.kind() == VnodeKind::Directory)
            {
                let guard = Self::child_node(target.as_ref().expect("tmpfs: missing target"))?
                    .directory()?
                    .contents
                    .lock();
                if !guard.is_empty() {
                    return Err(Error::NotEmpty);
                }
                Some(guard)
            } else {
                None
            };
            if let Some(target) = target.as_ref() {
                if target.kind() == VnodeKind::Directory {
                    let previous = self.links.fetch_sub(1, Ordering::AcqRel);
                    assert!(previous > 2, "tmpfs: parent directory link count underflow");
                }
                Self::remove_link(target)?;
                entries
                    .remove(target_name)
                    .expect("tmpfs: rename target vanished");
            }
            entries.rename_within(source_name, target_name)?;
            drop(target_directory_guard);
            Self::child_node(&source)?.touch_changed();
            self.touch_modified();
            return Ok(());
        }

        let source = {
            source_directory_data
                .contents
                .lock()
                .get(source_name)
                .cloned()
                .ok_or(Error::NotFound)?
        };
        Self::ensure_directory_move_is_acyclic(&source, target_directory)?;

        let (mut source_entries, mut target_entries) =
            if source_directory.key().node < target_directory.key().node {
                (
                    source_directory_data.contents.lock(),
                    target_directory_data.contents.lock(),
                )
            } else {
                let target_entries = target_directory_data.contents.lock();
                let source_entries = source_directory_data.contents.lock();
                (source_entries, target_entries)
            };
        if self.links.load(Ordering::Acquire) == 0 || target_node.links.load(Ordering::Acquire) == 0
        {
            return Err(Error::NotFound);
        }
        let source = source_entries
            .get(source_name)
            .cloned()
            .ok_or(Error::NotFound)?;
        let target = target_entries.get(target_name).cloned();
        Self::replace_target(target.as_ref(), &source)?;
        if target.as_ref() == Some(&source) {
            return Ok(());
        }
        let target_directory_guard = if target
            .as_ref()
            .is_some_and(|target| target.kind() == VnodeKind::Directory)
        {
            let guard = Self::child_node(target.as_ref().expect("tmpfs: missing target"))?
                .directory()?
                .contents
                .lock();
            if !guard.is_empty() {
                return Err(Error::NotEmpty);
            }
            Some(guard)
        } else {
            None
        };
        target_entries.ensure_insert_capacity()?;

        if let Some(target) = target.as_ref() {
            if target.kind() == VnodeKind::Directory {
                let previous = target_node.links.fetch_sub(1, Ordering::AcqRel);
                assert!(previous > 2, "tmpfs: target directory link count underflow");
            }
            Self::remove_link(target)?;
            target_entries
                .remove(target_name)
                .expect("tmpfs: rename target vanished");
        }
        let source = source_entries
            .remove(source_name)
            .expect("tmpfs: source vanished");
        target_entries.insert(target_name, source.clone())?;
        drop(target_directory_guard);

        if source.kind() == VnodeKind::Directory {
            let source_node = Self::child_node(&source)?;
            *source_node.directory()?.parent.lock() = target_directory.downgrade();
            let previous = self.links.fetch_sub(1, Ordering::AcqRel);
            assert!(previous > 2, "tmpfs: source directory link count underflow");
            target_node.links.fetch_add(1, Ordering::AcqRel);
        }
        Self::child_node(&source)?.touch_changed();
        self.touch_modified();
        target_node.touch_modified();
        Ok(())
    }

    fn read_at(&self, _vnode: &Vnode, offset: u64, buffer: &mut [u8]) -> Result<usize> {
        let file = self.file()?.lock();
        let size = self.size.load(Ordering::Acquire);
        if offset >= size || buffer.is_empty() {
            return Ok(0);
        }
        let count = cmp::min(buffer.len() as u64, size - offset) as usize;
        let read = file
            .object
            .read_at(offset, &mut buffer[..count])
            .map_err(mem_error)?;
        drop(file);
        self.touch_accessed();
        Ok(read)
    }

    fn write_at(&self, _vnode: &Vnode, offset: u64, buffer: &[u8]) -> Result<usize> {
        if buffer.is_empty() {
            return Ok(0);
        }
        let mut file = self.file()?.lock();
        self.write_locked(&mut file, offset, buffer)
    }

    fn append(&self, _vnode: &Vnode, buffer: &[u8]) -> Result<(usize, u64)> {
        let mut file = self.file()?.lock();
        if buffer.is_empty() {
            return Ok((0, self.size.load(Ordering::Acquire)));
        }
        let offset = self.size.load(Ordering::Acquire);
        let written = self.write_locked(&mut file, offset, buffer)?;
        Ok((written, offset.saturating_add(written as u64)))
    }

    fn truncate(&self, _vnode: &Vnode, size: u64) -> Result<()> {
        let mut file = self.file()?.lock();
        self.truncate_locked(&mut file, size)
    }

    fn memory_object(&self, _vnode: &Vnode) -> Result<Arc<VmObject>> {
        Ok(self.file()?.lock().object.clone())
    }

    fn readlink(&self, _vnode: &Vnode) -> Result<Box<[u8]>> {
        match &self.data {
            TmpfsData::Symlink(object) => {
                let size = usize::try_from(self.size.load(Ordering::Acquire))
                    .map_err(|_| Error::FileTooLarge)?;
                let mut target = alloc::vec![0; size];
                let read = object.read_at(0, &mut target).map_err(mem_error)?;
                if read != size {
                    return Err(Error::Io);
                }
                self.touch_accessed();
                Ok(target.into_boxed_slice())
            }
            _ => Err(Error::InvalidArgument),
        }
    }

    fn readdir(
        &self,
        directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        let directory_data = self.directory()?;
        let mut entries = Vec::new();
        if maximum == 0 {
            return Ok((entries, cursor));
        }

        let mut next = cursor;
        if next == 0 && entries.len() < maximum {
            entries.push(DirEntry {
                name: Box::<[u8]>::from(&b"."[..]),
                key: directory.key(),
                kind: VnodeKind::Directory,
            });
            next = 1;
        }
        if next == 1 && entries.len() < maximum {
            let parent = self.parent(directory)?;
            entries.push(DirEntry {
                name: Box::<[u8]>::from(&b".."[..]),
                key: parent.key(),
                kind: VnodeKind::Directory,
            });
            next = 2;
        }

        let stored = directory_data.contents.lock();
        for (cookie, name) in stored.cookies.range(next.max(2)..) {
            if entries.len() == maximum {
                break;
            }
            let vnode = stored
                .get(name)
                .expect("tmpfs: directory cookie index is inconsistent");
            entries.push(DirEntry {
                name: name.clone(),
                key: vnode.key(),
                kind: vnode.kind(),
            });
            next = cookie.saturating_add(1);
        }
        drop(stored);
        self.touch_accessed();
        Ok((entries, next))
    }
}

impl Drop for TmpfsNode {
    fn drop(&mut self) {
        if let Some(filesystem) = self.filesystem.upgrade() {
            filesystem.used_nodes.fetch_sub(1, Ordering::AcqRel);
        }
    }
}

fn mem_error(error: mem::Error) -> Error {
    match error {
        mem::Error::OutOfMemory => Error::OutOfMemory,
        mem::Error::LimitExceeded | mem::Error::SwapUnavailable => Error::NoSpace,
        mem::Error::InvalidAddress => Error::FileTooLarge,
        mem::Error::CorruptSwap | mem::Error::Pmap => Error::Io,
        mem::Error::AlreadyMapped | mem::Error::NotMapped | mem::Error::Protection => {
            Error::InvalidArgument
        }
    }
}
