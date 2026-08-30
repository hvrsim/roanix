#![no_std]
#![no_main]
#![allow(unsafe_code)]
// Values crossing the module boundary carry the C ABI widths fixed by
// include/roanix/api.h, and both supported targets use 64-bit pointers, so
// casts between them cannot lose information in practice. Large arrays appear
// only inside `const fn` constructors evaluated for statics, never on the
// stack at runtime.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays
)]

//! Loadable sparse temporary filesystem provider.
//!
//! Metadata and namespace policy live in this module. File contents remain in
//! kernel-owned VM objects obtained through the stable page-cache services.

extern crate alloc;

use alloc::{
    collections::BTreeMap,
    sync::{Arc, Weak},
};
use core::{
    cmp,
    ffi::c_void,
    mem::ManuallyDrop,
    ptr::{self, NonNull},
    sync::atomic::{AtomicU16, AtomicU64, Ordering},
};

use ddk::{self, Module, TicketLock, raw};

const ROOT_ID: u64 = 1;
const FIRST_COOKIE: u64 = 2;
const MAX_DIRECTORY_DEPTH: usize = 4096;
const MAX_SYMLINK_LENGTH: usize = 4096;
const TMPFS_MEMORY_NUMERATOR: u64 = 1;
const TMPFS_MEMORY_DENOMINATOR: u64 = 2;
const TMPFS_FALLBACK_PAGES: u64 = 4096;

struct PageAccount(NonNull<raw::FsPageAccount>);

// SAFETY: the receipt is opaque and kernel-side accounting is synchronized.
unsafe impl Send for PageAccount {}
// SAFETY: as above.
unsafe impl Sync for PageAccount {}

impl Drop for PageAccount {
    fn drop(&mut self) {
        // SAFETY: the mount owns this unique account receipt.
        unsafe { ddk::fs_page_account_release(self.0) };
    }
}

struct MemoryObject(NonNull<raw::FsMemoryObject>);

// SAFETY: the kernel VM object serializes page-cache operations.
unsafe impl Send for MemoryObject {}
// SAFETY: as above.
unsafe impl Sync for MemoryObject {}

impl MemoryObject {
    fn new(account: NonNull<raw::FsPageAccount>) -> core::result::Result<Self, i32> {
        ddk::fs_memory_object_create(account)
            .map(Self)
            .map_err(ddk::Error::status)
    }

    fn retain(&self) -> core::result::Result<*mut c_void, i32> {
        // SAFETY: this mount owns a live object receipt.
        unsafe { ddk::fs_memory_object_retain(self.0) }.map_err(ddk::Error::status)?;
        Ok(self.0.as_ptr().cast())
    }

    fn read(&self, offset: u64, output: &mut [u8]) -> core::result::Result<usize, i32> {
        ddk::fs_memory_object_read(self.0, offset, output).map_err(ddk::Error::status)
    }

    fn write(&self, offset: u64, input: &[u8]) -> core::result::Result<usize, i32> {
        ddk::fs_memory_object_write(self.0, offset, input).map_err(ddk::Error::status)
    }

    fn truncate(&self, size: u64) -> core::result::Result<(), i32> {
        ddk::fs_memory_object_truncate(self.0, size)
            .map(|_| ())
            .map_err(ddk::Error::status)
    }
}

impl Drop for MemoryObject {
    fn drop(&mut self) {
        // SAFETY: this wrapper owns exactly one object receipt.
        unsafe { ddk::fs_memory_object_release(self.0) };
    }
}

struct Mount {
    next_node: AtomicU64,
    used_nodes: AtomicU64,
    account: PageAccount,
    rename_lock: TicketLock<()>,
    root: Arc<Node>,
}

// SAFETY: every mutable namespace field is protected by a ticket lock.
unsafe impl Send for Mount {}
// SAFETY: as above.
unsafe impl Sync for Mount {}

struct Node {
    mount: Weak<Mount>,
    id: u64,
    kind: u32,
    mode: AtomicU16,
    links: AtomicU64,
    size: AtomicU64,
    accessed_ns: AtomicU64,
    modified_ns: AtomicU64,
    changed_ns: AtomicU64,
    data: NodeData,
}

// SAFETY: nodes only expose mutable state through atomics and ticket locks.
unsafe impl Send for Node {}
// SAFETY: as above.
unsafe impl Sync for Node {}

enum NodeData {
    File(TicketLock<MemoryObject>),
    Directory(Directory),
    Symlink(MemoryObject),
}

struct Directory {
    parent: TicketLock<Weak<Node>>,
    contents: TicketLock<DirectoryData>,
}

struct DirectoryData {
    entries: BTreeMap<Arc<[u8]>, DirEntry>,
    cookies: BTreeMap<u64, Arc<[u8]>>,
    next_cookie: u64,
}

struct DirEntry {
    node: Arc<Node>,
    cookie: u64,
}

impl DirectoryData {
    const fn empty() -> Self {
        Self {
            entries: BTreeMap::new(),
            cookies: BTreeMap::new(),
            next_cookie: FIRST_COOKIE,
        }
    }

    fn get(&self, name: &[u8]) -> Option<&Arc<Node>> {
        self.entries.get(name).map(|entry| &entry.node)
    }

    fn insert(&mut self, name: &[u8], node: Arc<Node>) -> core::result::Result<(), i32> {
        if self.entries.contains_key(name) {
            return Err(ddk::EEXIST);
        }
        self.ensure_insert_capacity()?;
        let name = Arc::<[u8]>::from(name);
        self.cookies.insert(self.next_cookie, name.clone());
        self.entries.insert(
            name,
            DirEntry {
                node,
                cookie: self.next_cookie,
            },
        );
        self.next_cookie += 1;
        Ok(())
    }

    fn ensure_insert_capacity(&self) -> core::result::Result<(), i32> {
        self.next_cookie.checked_add(1).ok_or(ddk::ENOSPC)?;
        Ok(())
    }

    fn remove(&mut self, name: &[u8]) -> Option<Arc<Node>> {
        let entry = self.entries.remove(name)?;
        self.cookies.remove(&entry.cookie);
        Some(entry.node)
    }

    fn rename_within(
        &mut self,
        source_name: &[u8],
        target_name: &[u8],
    ) -> core::result::Result<Arc<Node>, i32> {
        let entry = self.entries.remove(source_name).ok_or(ddk::ENOENT)?;
        self.cookies.remove(&entry.cookie);
        let target_name = Arc::<[u8]>::from(target_name);
        self.cookies.insert(entry.cookie, target_name.clone());
        let node = entry.node.clone();
        self.entries.insert(target_name, entry);
        Ok(node)
    }
}

impl Mount {
    fn new(limit: u64) -> core::result::Result<Arc<Self>, i32> {
        let account = PageAccount(ddk::fs_page_account_create(limit).map_err(ddk::Error::status)?);
        Ok(Arc::new_cyclic(|weak| {
            let now = ddk::monotonic_ns();
            let root = Arc::new(Node {
                mount: weak.clone(),
                id: ROOT_ID,
                kind: ddk::FS_KIND_DIRECTORY,
                mode: AtomicU16::new(0o755),
                links: AtomicU64::new(2),
                size: AtomicU64::new(0),
                accessed_ns: AtomicU64::new(now),
                modified_ns: AtomicU64::new(now),
                changed_ns: AtomicU64::new(now),
                data: NodeData::Directory(Directory {
                    parent: TicketLock::new(Weak::new()),
                    contents: TicketLock::new(DirectoryData::empty()),
                }),
            });
            Self {
                next_node: AtomicU64::new(ROOT_ID + 1),
                used_nodes: AtomicU64::new(1),
                account,
                rename_lock: TicketLock::new(()),
                root,
            }
        }))
    }

    fn allocate_node(
        self: &Arc<Self>,
        kind: u32,
        mode: u16,
        parent: Weak<Node>,
        target: &[u8],
    ) -> core::result::Result<Arc<Node>, i32> {
        let id = self.next_node.fetch_add(1, Ordering::Relaxed);
        if id == 0 {
            return Err(ddk::ENOSPC);
        }
        let now = ddk::monotonic_ns();
        let (links, size, data) = match kind {
            ddk::FS_KIND_REGULAR => (
                1,
                0,
                NodeData::File(TicketLock::new(MemoryObject::new(self.account.0)?)),
            ),
            ddk::FS_KIND_DIRECTORY => (
                2,
                0,
                NodeData::Directory(Directory {
                    parent: TicketLock::new(parent),
                    contents: TicketLock::new(DirectoryData::empty()),
                }),
            ),
            ddk::FS_KIND_SYMLINK => {
                let object = MemoryObject::new(self.account.0)?;
                let written = object.write(0, target)?;
                if written != target.len() {
                    return Err(ddk::ENOSPC);
                }
                (
                    1,
                    u64::try_from(target.len()).map_err(|_| ddk::EINVAL)?,
                    NodeData::Symlink(object),
                )
            }
            _ => return Err(ddk::EINVAL),
        };
        self.used_nodes.fetch_add(1, Ordering::AcqRel);
        Ok(Arc::new(Node {
            mount: Arc::downgrade(self),
            id,
            kind,
            mode: AtomicU16::new(mode),
            links: AtomicU64::new(links),
            size: AtomicU64::new(size),
            accessed_ns: AtomicU64::new(now),
            modified_ns: AtomicU64::new(now),
            changed_ns: AtomicU64::new(now),
            data,
        }))
    }
}

impl Drop for Node {
    fn drop(&mut self) {
        if let Some(mount) = self.mount.upgrade() {
            let previous = mount.used_nodes.fetch_sub(1, Ordering::AcqRel);
            debug_assert!(previous != 0, "tmpfs node count underflow");
        }
    }
}

impl Node {
    fn directory(&self) -> core::result::Result<&Directory, i32> {
        match &self.data {
            NodeData::Directory(directory) => Ok(directory),
            _ => Err(ddk::ENOTDIR),
        }
    }

    fn file(&self) -> core::result::Result<&TicketLock<MemoryObject>, i32> {
        match &self.data {
            NodeData::File(file) => Ok(file),
            NodeData::Directory(_) => Err(ddk::EISDIR),
            NodeData::Symlink(_) => Err(ddk::EINVAL),
        }
    }

    fn touch_accessed(&self) {
        self.accessed_ns
            .store(ddk::monotonic_ns(), Ordering::Release);
    }

    fn touch_modified(&self) {
        let now = ddk::monotonic_ns();
        self.modified_ns.store(now, Ordering::Release);
        self.changed_ns.store(now, Ordering::Release);
    }

    fn touch_changed(&self) {
        self.changed_ns
            .store(ddk::monotonic_ns(), Ordering::Release);
    }

    fn parent(&self, fallback: &Arc<Node>) -> core::result::Result<Arc<Node>, i32> {
        let directory = self.directory()?;
        Ok(directory
            .parent
            .lock()
            .upgrade()
            .unwrap_or_else(|| fallback.clone()))
    }

    fn mounted_at(&self, mount: &Arc<Mount>) -> bool {
        self.mount
            .upgrade()
            .is_some_and(|node_mount| Arc::ptr_eq(&node_mount, mount))
    }
}

fn default_page_limit() -> u64 {
    ddk::fs_total_physical_pages()
        .saturating_mul(TMPFS_MEMORY_NUMERATOR)
        .checked_div(TMPFS_MEMORY_DENOMINATOR)
        .unwrap_or(0)
        .max(TMPFS_FALLBACK_PAGES)
}

fn validate_name(name: &[u8]) -> core::result::Result<(), i32> {
    if name.len() > raw::FS_DIRECTORY_NAME_MAX {
        Err(ddk::ENAMETOOLONG)
    } else if name.is_empty()
        || name == b"."
        || name == b".."
        || name.contains(&0)
        || name.contains(&b'/')
    {
        Err(ddk::EINVAL)
    } else {
        Ok(())
    }
}

fn validate_symlink_target(target: &[u8]) -> core::result::Result<(), i32> {
    if target.len() > MAX_SYMLINK_LENGTH {
        Err(ddk::ENAMETOOLONG)
    } else if target.is_empty() || target.contains(&0) {
        Err(ddk::EINVAL)
    } else {
        Ok(())
    }
}

fn vnode(node: &Arc<Node>) -> raw::FsVnode {
    raw::FsVnode {
        receipt: Arc::into_raw(node.clone()).cast_mut().cast(),
        node_id: node.id,
        kind: node.kind,
        reserved: 0,
    }
}

unsafe fn borrow_arc<T>(receipt: *mut c_void) -> core::result::Result<ManuallyDrop<Arc<T>>, i32> {
    if receipt.is_null() {
        return Err(ddk::EINVAL);
    }
    // SAFETY: provider receipts are Arc pointers retained by their matching
    // mount or vnode owner. ManuallyDrop preserves that owned reference.
    Ok(ManuallyDrop::new(unsafe {
        Arc::from_raw(receipt.cast::<T>())
    }))
}

unsafe fn borrow_mount(
    receipt: *mut c_void,
) -> core::result::Result<ManuallyDrop<Arc<Mount>>, i32> {
    // SAFETY: forwarded from the provider callback contract.
    unsafe { borrow_arc(receipt) }
}

unsafe fn borrow_node(receipt: *mut c_void) -> core::result::Result<ManuallyDrop<Arc<Node>>, i32> {
    // SAFETY: forwarded from the provider callback contract.
    unsafe { borrow_arc(receipt) }
}

fn checked_node(
    mount: &Arc<Mount>,
    receipt: *mut c_void,
) -> core::result::Result<ManuallyDrop<Arc<Node>>, i32> {
    // SAFETY: VFS only passes a currently-owned provider vnode receipt.
    let node = unsafe { borrow_node(receipt) }?;
    if !node.mounted_at(mount) {
        return Err(ddk::EINVAL);
    }
    Ok(node)
}

fn checked_bytes<'a>(pointer: *const u8, length: usize) -> core::result::Result<&'a [u8], i32> {
    if length == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(ddk::EINVAL);
    }
    // SAFETY: the provider ABI guarantees a readable range for this callback.
    Ok(unsafe { core::slice::from_raw_parts(pointer, length) })
}

fn checked_bytes_mut<'a>(
    pointer: *mut u8,
    length: usize,
) -> core::result::Result<&'a mut [u8], i32> {
    if length == 0 {
        return Ok(&mut []);
    }
    if pointer.is_null() {
        return Err(ddk::EINVAL);
    }
    // SAFETY: the provider ABI guarantees a writable range for this callback.
    Ok(unsafe { core::slice::from_raw_parts_mut(pointer, length) })
}

unsafe extern "C" fn mount(
    _context: *mut c_void,
    options: *const raw::FsMountOptions,
    output: *mut *mut c_void,
) -> i32 {
    if options.is_null() || output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: mount options are an immediate fixed-size ABI input.
    let options = unsafe { &*options };
    if options.size < core::mem::size_of::<raw::FsMountOptions>() as u32 {
        return ddk::EINVAL;
    }
    let limit = if options.page_limit == 0 {
        default_page_limit()
    } else {
        options.page_limit
    };
    match Mount::new(limit) {
        Ok(mount) => {
            // SAFETY: output is writable for one opaque receipt.
            unsafe { *output = Arc::into_raw(mount).cast_mut().cast() };
            ddk::OK
        }
        Err(error) => error,
    }
}

unsafe extern "C" fn unmount(_context: *mut c_void, receipt: *mut c_void) -> i32 {
    if receipt.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS consumes the mount receipt exactly once after all vnodes.
    drop(unsafe { Arc::from_raw(receipt.cast::<Mount>()) });
    ddk::OK
}

unsafe extern "C" fn root(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    output: *mut raw::FsVnode,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS passes its live mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    // SAFETY: output is writable for one fixed record.
    unsafe { *output = vnode(&mount.root) };
    ddk::OK
}

unsafe extern "C" fn statfs(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    output: *mut raw::FsStat,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS passes its live mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(limit) = ddk::fs_page_account_limit(mount.account.0) else {
        return ddk::EIO;
    };
    let Ok(used) = ddk::fs_page_account_used(mount.account.0) else {
        return ddk::EIO;
    };
    // SAFETY: output is writable for one fixed record.
    unsafe {
        *output = raw::FsStat {
            total_bytes: limit.saturating_mul(4096),
            used_bytes: used.saturating_mul(4096),
            total_nodes: u64::MAX,
            used_nodes: mount.used_nodes.load(Ordering::Acquire),
        };
    }
    ddk::OK
}

unsafe extern "C" fn sync(_context: *mut c_void, mount_receipt: *mut c_void) -> i32 {
    // SAFETY: validates the opaque receipt before accepting the operation.
    if unsafe { borrow_mount(mount_receipt) }.is_err() {
        ddk::EINVAL
    } else {
        ddk::OK
    }
}

unsafe extern "C" fn vnode_release(
    _context: *mut c_void,
    _mount_receipt: *mut c_void,
    receipt: *mut c_void,
) {
    if !receipt.is_null() {
        // SAFETY: VFS consumes every vnode receipt exactly once.
        drop(unsafe { Arc::from_raw(receipt.cast::<Node>()) });
    }
}

unsafe extern "C" fn getattr(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    output: *mut raw::FsAttr,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS holds the mount receipt for this callback.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    // SAFETY: output is writable for a fixed ABI record.
    unsafe {
        *output = raw::FsAttr {
            size: node.size.load(Ordering::Acquire),
            links: node.links.load(Ordering::Acquire),
            accessed_ns: node.accessed_ns.load(Ordering::Acquire),
            modified_ns: node.modified_ns.load(Ordering::Acquire),
            changed_ns: node.changed_ns.load(Ordering::Acquire),
            mode: node.mode.load(Ordering::Acquire),
            kind: node.kind,
            reserved: 0,
        };
    }
    ddk::OK
}

unsafe extern "C" fn setattr(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    attributes: *const raw::FsSetAttr,
) -> i32 {
    if attributes.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: fixed ABI input remains valid through this callback.
    let attributes = unsafe { &*attributes };
    // SAFETY: VFS holds the mount receipt for this callback.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if attributes.valid & !((ddk::FS_SETATTR_SIZE) | ddk::FS_SETATTR_MODE) != 0 {
        return ddk::EINVAL;
    }
    if attributes.valid & ddk::FS_SETATTR_SIZE != 0 {
        let file = match node.file() {
            Ok(file) => file,
            Err(error) => return error,
        };
        let object = file.lock();
        let old_size = node.size.load(Ordering::Acquire);
        if attributes.size < old_size
            && let Err(error) = object.truncate(attributes.size)
        {
            return error;
        }
        node.size.store(attributes.size, Ordering::Release);
        node.touch_modified();
    }
    if attributes.valid & ddk::FS_SETATTR_MODE != 0 {
        node.mode.store(attributes.mode, Ordering::Release);
        node.touch_changed();
    }
    ddk::OK
}

unsafe extern "C" fn lookup(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    name: *const u8,
    name_length: usize,
    output: *mut raw::FsVnode,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    let Ok(name) = checked_bytes(name, name_length) else {
        return ddk::EINVAL;
    };
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let child = if name == b"." {
        Arc::clone(&directory)
    } else if name == b".." {
        match directory.parent(&directory) {
            Ok(parent) => parent,
            Err(error) => return error,
        }
    } else {
        if validate_name(name).is_err() {
            return ddk::EINVAL;
        }
        let data = match directory.directory() {
            Ok(data) => data,
            Err(error) => return error,
        };
        if directory.links.load(Ordering::Acquire) == 0 {
            return ddk::ENOENT;
        }
        let Some(child) = data.contents.lock().get(name).cloned() else {
            return ddk::ENOENT;
        };
        child
    };
    directory.touch_accessed();
    // SAFETY: output is writable for one vnode ABI record.
    unsafe { *output = vnode(&child) };
    ddk::OK
}

unsafe extern "C" fn parent(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    output: *mut raw::FsVnode,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let parent = match directory.parent(&directory) {
        Ok(parent) => parent,
        Err(error) => return error,
    };
    // SAFETY: output is writable for one vnode ABI record.
    unsafe { *output = vnode(&parent) };
    ddk::OK
}

unsafe extern "C" fn create(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    name: *const u8,
    name_length: usize,
    create_kind: u32,
    target: *const u8,
    target_length: usize,
    mode: u16,
    output: *mut raw::FsVnode,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    let Ok(name) = checked_bytes(name, name_length) else {
        return ddk::EINVAL;
    };
    let Ok(target) = checked_bytes(target, target_length) else {
        return ddk::EINVAL;
    };
    if validate_name(name).is_err() {
        return ddk::EINVAL;
    }
    let kind = match create_kind {
        ddk::FS_CREATE_REGULAR => ddk::FS_KIND_REGULAR,
        ddk::FS_CREATE_DIRECTORY => ddk::FS_KIND_DIRECTORY,
        ddk::FS_CREATE_SYMLINK => {
            if let Err(error) = validate_symlink_target(target) {
                return error;
            }
            ddk::FS_KIND_SYMLINK
        }
        _ => return ddk::EINVAL,
    };
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let mount = Arc::clone(&mount);
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let data = match directory.directory() {
        Ok(data) => data,
        Err(error) => return error,
    };
    let mut entries = data.contents.lock();
    if directory.links.load(Ordering::Acquire) == 0 {
        return ddk::ENOENT;
    }
    if entries.entries.contains_key(name) {
        return ddk::EEXIST;
    }
    if let Err(error) = entries.ensure_insert_capacity() {
        return error;
    }
    let child = match mount.allocate_node(kind, mode, Arc::downgrade(&directory), target) {
        Ok(child) => child,
        Err(error) => return error,
    };
    if let Err(error) = entries.insert(name, child.clone()) {
        return error;
    }
    if kind == ddk::FS_KIND_DIRECTORY {
        directory.links.fetch_add(1, Ordering::AcqRel);
    }
    directory.touch_modified();
    // SAFETY: output is writable for one vnode ABI record.
    unsafe { *output = vnode(&child) };
    ddk::OK
}

unsafe extern "C" fn link(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    directory_receipt: *mut c_void,
    name: *const u8,
    name_length: usize,
    target_receipt: *mut c_void,
) -> i32 {
    let Ok(name) = checked_bytes(name, name_length) else {
        return ddk::EINVAL;
    };
    if validate_name(name).is_err() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, directory_receipt) else {
        return ddk::EINVAL;
    };
    let Ok(target) = checked_node(&mount, target_receipt) else {
        return ddk::EINVAL;
    };
    if target.kind == ddk::FS_KIND_DIRECTORY {
        return ddk::EPERM;
    }
    let data = match directory.directory() {
        Ok(data) => data,
        Err(error) => return error,
    };
    let mut entries = data.contents.lock();
    if directory.links.load(Ordering::Acquire) == 0 {
        return ddk::ENOENT;
    }
    if entries.entries.contains_key(name) {
        return ddk::EEXIST;
    }
    if let Err(error) = entries.ensure_insert_capacity() {
        return error;
    }
    if let Err(error) = entries.insert(name, Arc::clone(&target)) {
        return error;
    }
    target.links.fetch_add(1, Ordering::AcqRel);
    target.touch_changed();
    directory.touch_modified();
    ddk::OK
}

fn decrement_if_above(counter: &AtomicU64, minimum: u64) {
    let mut current = counter.load(Ordering::Acquire);
    while current > minimum {
        match counter.compare_exchange_weak(
            current,
            current - 1,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => return,
            Err(actual) => current = actual,
        }
    }
}

fn remove_link(node: &Arc<Node>) {
    if node.kind == ddk::FS_KIND_DIRECTORY {
        node.links.store(0, Ordering::Release);
    } else {
        decrement_if_above(&node.links, 0);
    }
    node.touch_changed();
}

unsafe extern "C" fn unlink(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    directory_receipt: *mut c_void,
    name: *const u8,
    name_length: usize,
    remove_directory: u8,
) -> i32 {
    let Ok(name) = checked_bytes(name, name_length) else {
        return ddk::EINVAL;
    };
    if validate_name(name).is_err() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, directory_receipt) else {
        return ddk::EINVAL;
    };
    let _rename = (remove_directory != 0).then(|| mount.rename_lock.lock());
    let data = match directory.directory() {
        Ok(data) => data,
        Err(error) => return error,
    };
    let mut entries = data.contents.lock();
    if directory.links.load(Ordering::Acquire) == 0 {
        return ddk::ENOENT;
    }
    let Some(target) = entries.get(name).cloned() else {
        return ddk::ENOENT;
    };
    let target_directory_guard = match (remove_directory != 0, target.kind) {
        (true, ddk::FS_KIND_DIRECTORY) => {
            let data = match target.directory() {
                Ok(data) => data,
                Err(error) => return error,
            };
            let guard = data.contents.lock();
            if !guard.entries.is_empty() {
                return ddk::ENOTEMPTY;
            }
            Some(guard)
        }
        (true, _) => return ddk::ENOTDIR,
        (false, ddk::FS_KIND_DIRECTORY) => return ddk::EISDIR,
        (false, _) => None,
    };
    if entries.remove(name).is_none() {
        return ddk::EIO;
    }
    if target.kind == ddk::FS_KIND_DIRECTORY {
        decrement_if_above(&directory.links, 2);
    }
    remove_link(&target);
    drop(target_directory_guard);
    directory.touch_modified();
    ddk::OK
}

fn validate_rename_target(
    source: &Arc<Node>,
    target: Option<&Arc<Node>>,
) -> core::result::Result<(), i32> {
    let Some(target) = target else {
        return Ok(());
    };
    if Arc::ptr_eq(source, target) {
        return Ok(());
    }
    if source.kind == ddk::FS_KIND_DIRECTORY {
        if target.kind == ddk::FS_KIND_DIRECTORY {
            Ok(())
        } else {
            Err(ddk::ENOTDIR)
        }
    } else if target.kind == ddk::FS_KIND_DIRECTORY {
        Err(ddk::EISDIR)
    } else {
        Ok(())
    }
}

fn ensure_acyclic(
    source: &Arc<Node>,
    target_directory: &Arc<Node>,
) -> core::result::Result<(), i32> {
    if source.kind != ddk::FS_KIND_DIRECTORY {
        return Ok(());
    }
    let mut ancestor = target_directory.clone();
    for _ in 0..MAX_DIRECTORY_DEPTH {
        if Arc::ptr_eq(&ancestor, source) {
            return Err(ddk::EINVAL);
        }
        let parent = ancestor.parent(&ancestor)?;
        if Arc::ptr_eq(&parent, &ancestor) {
            return Ok(());
        }
        ancestor = parent;
    }
    Err(ddk::ELOOP)
}

// The rename path validates both endpoints, resolves ancestor chains for the
// no-descendant rule, and rewrites namespace links; splitting it would scatter
// one transaction across helpers that share all of its locals.
#[allow(clippy::too_many_lines)]
unsafe extern "C" fn rename(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    source_directory_receipt: *mut c_void,
    source_name: *const u8,
    source_name_length: usize,
    target_directory_receipt: *mut c_void,
    target_name: *const u8,
    target_name_length: usize,
) -> i32 {
    let Ok(source_name) = checked_bytes(source_name, source_name_length) else {
        return ddk::EINVAL;
    };
    let Ok(target_name) = checked_bytes(target_name, target_name_length) else {
        return ddk::EINVAL;
    };
    if validate_name(source_name).is_err() || validate_name(target_name).is_err() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(source_directory) = checked_node(&mount, source_directory_receipt) else {
        return ddk::EINVAL;
    };
    let Ok(target_directory) = checked_node(&mount, target_directory_receipt) else {
        return ddk::EINVAL;
    };
    let _rename = mount.rename_lock.lock();
    let source_data = match source_directory.directory() {
        Ok(data) => data,
        Err(error) => return error,
    };
    let target_data = match target_directory.directory() {
        Ok(data) => data,
        Err(error) => return error,
    };

    if Arc::ptr_eq(&source_directory, &target_directory) {
        let mut entries = source_data.contents.lock();
        if source_directory.links.load(Ordering::Acquire) == 0 {
            return ddk::ENOENT;
        }
        let Some(source) = entries.get(source_name).cloned() else {
            return ddk::ENOENT;
        };
        if source_name == target_name {
            return ddk::OK;
        }
        let target = entries.get(target_name).cloned();
        if let Err(error) = validate_rename_target(&source, target.as_ref()) {
            return error;
        }
        if target
            .as_ref()
            .is_some_and(|target| Arc::ptr_eq(target, &source))
        {
            return ddk::OK;
        }
        let target_directory_guard = if target
            .as_ref()
            .is_some_and(|target| target.kind == ddk::FS_KIND_DIRECTORY)
        {
            let target = target.as_ref().expect("target checked above");
            let data = match target.directory() {
                Ok(data) => data,
                Err(error) => return error,
            };
            let guard = data.contents.lock();
            if !guard.entries.is_empty() {
                return ddk::ENOTEMPTY;
            }
            Some(guard)
        } else {
            None
        };
        if let Some(target) = target.as_ref() {
            if target.kind == ddk::FS_KIND_DIRECTORY {
                decrement_if_above(&source_directory.links, 2);
            }
            remove_link(target);
            if entries.remove(target_name).is_none() {
                return ddk::EIO;
            }
        }
        if entries.rename_within(source_name, target_name).is_err() {
            return ddk::EIO;
        }
        drop(target_directory_guard);
        source.touch_changed();
        source_directory.touch_modified();
        return ddk::OK;
    }

    let source_preview = {
        let entries = source_data.contents.lock();
        entries.get(source_name).cloned()
    };
    let Some(source_preview) = source_preview else {
        return ddk::ENOENT;
    };
    if let Err(error) = ensure_acyclic(&source_preview, &target_directory) {
        return error;
    }

    let (mut source_entries, mut target_entries) = if source_directory.id < target_directory.id {
        (source_data.contents.lock(), target_data.contents.lock())
    } else {
        let target_entries = target_data.contents.lock();
        let source_entries = source_data.contents.lock();
        (source_entries, target_entries)
    };
    if source_directory.links.load(Ordering::Acquire) == 0
        || target_directory.links.load(Ordering::Acquire) == 0
    {
        return ddk::ENOENT;
    }
    let Some(source) = source_entries.get(source_name).cloned() else {
        return ddk::ENOENT;
    };
    if let Err(error) = ensure_acyclic(&source, &target_directory) {
        return error;
    }
    let target = target_entries.get(target_name).cloned();
    if let Err(error) = validate_rename_target(&source, target.as_ref()) {
        return error;
    }
    if target
        .as_ref()
        .is_some_and(|target| Arc::ptr_eq(target, &source))
    {
        return ddk::OK;
    }
    if let Err(error) = target_entries.ensure_insert_capacity() {
        return error;
    }
    let target_directory_guard = if target
        .as_ref()
        .is_some_and(|target| target.kind == ddk::FS_KIND_DIRECTORY)
    {
        let target = target.as_ref().expect("target checked above");
        let data = match target.directory() {
            Ok(data) => data,
            Err(error) => return error,
        };
        let guard = data.contents.lock();
        if !guard.entries.is_empty() {
            return ddk::ENOTEMPTY;
        }
        Some(guard)
    } else {
        None
    };
    if let Some(target) = target.as_ref() {
        if target.kind == ddk::FS_KIND_DIRECTORY {
            decrement_if_above(&target_directory.links, 2);
        }
        remove_link(target);
        if target_entries.remove(target_name).is_none() {
            return ddk::EIO;
        }
    }
    let Some(source) = source_entries.remove(source_name) else {
        return ddk::EIO;
    };
    if let Err(error) = target_entries.insert(target_name, source.clone()) {
        return error;
    }
    drop(target_directory_guard);
    if source.kind == ddk::FS_KIND_DIRECTORY {
        let Ok(directory) = source.directory() else {
            return ddk::EIO;
        };
        *directory.parent.lock() = Arc::downgrade(&target_directory);
        decrement_if_above(&source_directory.links, 2);
        target_directory.links.fetch_add(1, Ordering::AcqRel);
    }
    source.touch_changed();
    source_directory.touch_modified();
    target_directory.touch_modified();
    ddk::OK
}

unsafe extern "C" fn open(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    _flags: u32,
    output: *mut usize,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    if checked_node(&mount, vnode_receipt).is_err() {
        return ddk::EINVAL;
    }
    // SAFETY: output is writable for one scalar file context.
    unsafe { *output = 0 };
    ddk::OK
}

unsafe extern "C" fn close(
    _context: *mut c_void,
    _mount_receipt: *mut c_void,
    _vnode_receipt: *mut c_void,
    _file: usize,
    _flags: u32,
) {
}

unsafe extern "C" fn initial_offset(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    _file: usize,
    _flags: u32,
    output: *mut u64,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    if checked_node(&mount, vnode_receipt).is_err() {
        return ddk::EINVAL;
    }
    // SAFETY: output is writable for one scalar offset.
    unsafe { *output = 0 };
    ddk::OK
}

unsafe extern "C" fn read(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    _file: usize,
    offset: u64,
    output: *mut u8,
    length: usize,
    _flags: u32,
) -> i64 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return i64::from(ddk::EINVAL);
    };
    let file = match node.file() {
        Ok(file) => file,
        Err(error) => return i64::from(error),
    };
    let Ok(output) = checked_bytes_mut(output, length) else {
        return i64::from(ddk::EINVAL);
    };
    let size = node.size.load(Ordering::Acquire);
    if offset >= size || output.is_empty() {
        return 0;
    }
    let count = cmp::min(
        u64::try_from(output.len()).unwrap_or(u64::MAX),
        size - offset,
    ) as usize;
    let result = file.lock().read(offset, &mut output[..count]);
    match result {
        Ok(read) => {
            node.touch_accessed();
            i64::try_from(read).unwrap_or(i64::MAX)
        }
        Err(error) => i64::from(error),
    }
}

fn write_file(
    node: &Node,
    object: &MemoryObject,
    offset: u64,
    input: &[u8],
) -> core::result::Result<usize, i32> {
    let end = offset
        .checked_add(u64::try_from(input.len()).map_err(|_| ddk::EINVAL)?)
        .ok_or(ddk::EFBIG)?;
    let written = object.write(offset, input)?;
    node.size.fetch_max(
        offset.saturating_add(u64::try_from(written).map_err(|_| ddk::EINVAL)?),
        Ordering::AcqRel,
    );
    if written != 0 {
        node.touch_modified();
    }
    let _ = end;
    Ok(written)
}

unsafe extern "C" fn write(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    _file: usize,
    offset: u64,
    input: *const u8,
    length: usize,
    _flags: u32,
) -> i64 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(input) = checked_bytes(input, length) else {
        return i64::from(ddk::EINVAL);
    };
    if input.is_empty() {
        return 0;
    }
    let file = match node.file() {
        Ok(file) => file,
        Err(error) => return i64::from(error),
    };
    match write_file(&node, &file.lock(), offset, input) {
        Ok(written) => i64::try_from(written).unwrap_or(i64::MAX),
        Err(error) => i64::from(error),
    }
}

unsafe extern "C" fn append(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    vectors: *const raw::FsIoVec,
    count: usize,
    next_offset: *mut u64,
) -> i64 {
    if next_offset.is_null() || (count != 0 && vectors.is_null()) {
        return i64::from(ddk::EINVAL);
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return i64::from(ddk::EINVAL);
    };
    let file = match node.file() {
        Ok(file) => file,
        Err(error) => return i64::from(error),
    };
    // SAFETY: `vectors` describes `count` stable source-window descriptors.
    let vectors = if count == 0 {
        &[][..]
    } else {
        // SAFETY: the non-null pointer was validated above and VFS keeps all
        // `count` descriptors readable for this callback.
        unsafe { core::slice::from_raw_parts(vectors, count) }
    };
    let mut total = 0usize;
    for vector in vectors {
        if vector.length != 0 && vector.data.is_null() {
            return i64::from(ddk::EINVAL);
        }
        total = match total.checked_add(vector.length) {
            Some(total) => total,
            None => return i64::from(ddk::EINVAL),
        };
    }
    let object = file.lock();
    let mut offset = node.size.load(Ordering::Acquire);
    let start = offset;
    for vector in vectors {
        let Ok(bytes) = checked_bytes(vector.data, vector.length) else {
            return i64::from(ddk::EINVAL);
        };
        let written = match write_file(&node, &object, offset, bytes) {
            Ok(written) => written,
            Err(error) => return i64::from(error),
        };
        offset = match offset.checked_add(u64::try_from(written).unwrap_or(u64::MAX)) {
            Some(offset) => offset,
            None => return i64::from(ddk::EINVAL),
        };
        if written != bytes.len() {
            break;
        }
    }
    let written = usize::try_from(offset.saturating_sub(start)).unwrap_or(usize::MAX);
    // SAFETY: caller supplied writable next-offset storage.
    unsafe { *next_offset = offset };
    i64::try_from(written.min(total)).unwrap_or(i64::MAX)
}

unsafe extern "C" fn truncate(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    size: u64,
) -> i32 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let file = match node.file() {
        Ok(file) => file,
        Err(error) => return error,
    };
    let object = file.lock();
    let old = node.size.load(Ordering::Acquire);
    if size < old
        && let Err(error) = object.truncate(size)
    {
        return error;
    }
    node.size.store(size, Ordering::Release);
    node.touch_modified();
    ddk::OK
}

unsafe extern "C" fn memory_object(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    output: *mut *mut c_void,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let file = match node.file() {
        Ok(file) => file,
        Err(error) => return error,
    };
    let object = file.lock();
    match object.retain() {
        Ok(receipt) => {
            // SAFETY: output is writable for one owned object receipt.
            unsafe { *output = receipt };
            ddk::OK
        }
        Err(error) => error,
    }
}

unsafe extern "C" fn readlink(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    output: *mut u8,
    capacity: usize,
    written: *mut usize,
) -> i32 {
    if written.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let NodeData::Symlink(object) = &node.data else {
        return ddk::EINVAL;
    };
    let Ok(size) = usize::try_from(node.size.load(Ordering::Acquire)) else {
        return ddk::EINVAL;
    };
    if capacity < size {
        return ddk::EINVAL;
    }
    let Ok(output) = checked_bytes_mut(output, size) else {
        return ddk::EINVAL;
    };
    match object.read(0, output) {
        Ok(read) if read == size => {
            node.touch_accessed();
            // SAFETY: written is writable scalar output storage.
            unsafe { *written = read };
            ddk::OK
        }
        Ok(_) => ddk::EIO,
        Err(error) => error,
    }
}

fn emit_entry(
    output: *mut raw::FsDirEntry,
    index: usize,
    node: &Arc<Node>,
    name: &[u8],
    offset: u64,
) -> core::result::Result<(), i32> {
    if name.len() > raw::FS_DIRECTORY_NAME_MAX {
        return Err(ddk::EINVAL);
    }
    let mut entry = raw::FsDirEntry {
        vnode: vnode(node),
        name_length: u16::try_from(name.len()).map_err(|_| ddk::EINVAL)?,
        reserved: [0; 6],
        offset,
        name: [0; raw::FS_DIRECTORY_NAME_MAX],
    };
    entry.name[..name.len()].copy_from_slice(name);
    // SAFETY: readdir validated capacity and writes each slot at most once.
    unsafe { *output.add(index) = entry };
    Ok(())
}

unsafe extern "C" fn readdir(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    cursor: u64,
    output: *mut raw::FsDirEntry,
    capacity: usize,
    count: *mut usize,
    next: *mut u64,
) -> i32 {
    if count.is_null() || next.is_null() || (capacity != 0 && output.is_null()) {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let data = match directory.directory() {
        Ok(data) => data,
        Err(error) => return error,
    };
    let mut emitted = 0usize;
    let mut offset = cursor;
    if capacity != 0 && offset == 0 {
        if let Err(error) = emit_entry(output, emitted, &directory, b".", 1) {
            return error;
        }
        emitted += 1;
        offset = 1;
    }
    if emitted < capacity && offset == 1 {
        let parent = match directory.parent(&directory) {
            Ok(parent) => parent,
            Err(error) => return error,
        };
        if let Err(error) = emit_entry(output, emitted, &parent, b"..", FIRST_COOKIE) {
            return error;
        }
        emitted += 1;
        offset = FIRST_COOKIE;
    }
    if emitted < capacity {
        let entries = data.contents.lock();
        for (cookie, name) in entries.cookies.range(offset.max(FIRST_COOKIE)..) {
            if emitted == capacity {
                break;
            }
            let Some(node) = entries.get(name) else {
                return ddk::EIO;
            };
            offset = cookie.saturating_add(1);
            if let Err(error) = emit_entry(output, emitted, node, name, offset) {
                return error;
            }
            emitted += 1;
        }
    }
    directory.touch_accessed();
    // SAFETY: scalar outputs are writable by the provider ABI contract.
    unsafe {
        *count = emitted;
        *next = offset;
    }
    ddk::OK
}

unsafe extern "C" fn fsync(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
) -> i32 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    if checked_node(&mount, vnode_receipt).is_err() {
        ddk::EINVAL
    } else {
        ddk::OK
    }
}

static OPERATIONS: raw::FsProviderOps = raw::FsProviderOps {
    size: raw::FS_PROVIDER_OPS_SIZE,
    context: ptr::null_mut(),
    mount: Some(mount),
    unmount: Some(unmount),
    root: Some(root),
    statfs: Some(statfs),
    sync: Some(sync),
    vnode_release: Some(vnode_release),
    getattr: Some(getattr),
    setattr: Some(setattr),
    lookup: Some(lookup),
    parent: Some(parent),
    create: Some(create),
    link: Some(link),
    unlink: Some(unlink),
    rename: Some(rename),
    open: Some(open),
    close: Some(close),
    initial_offset: Some(initial_offset),
    read: Some(read),
    write: Some(write),
    append: Some(append),
    truncate: Some(truncate),
    memory_object: Some(memory_object),
    readlink: Some(readlink),
    readdir: Some(readdir),
    fsync: Some(fsync),
    poll: None,
    poll_events: None,
    terminal_state: None,
    ioctl: None,
};

static REGISTRATION: TicketLock<Option<usize>> = TicketLock::new(None);

fn tmpfs_init(_module: Module) -> ddk::Result<()> {
    let registration = ddk::fs_provider_register(c"tmpfs", &OPERATIONS)?;
    *REGISTRATION.lock() = Some(registration.as_ptr() as usize);
    Ok(())
}

fn tmpfs_exit(_module: Module) {
    let Some(registration) = REGISTRATION.lock().take() else {
        return;
    };
    let Some(registration) = NonNull::new(registration as *mut raw::FsProvider) else {
        return;
    };
    // SAFETY: this module owns the registration receipt and the loader only
    // reaches exit after all provider mount leases have been released.
    let _ = unsafe { ddk::fs_provider_unregister(registration) };
}

ddk::module!(
    b"tmpfs\0",
    b"Loadable sparse temporary filesystem provider\0",
    tmpfs_init,
    tmpfs_exit,
);
