#![no_std]
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

//! Loadable device filesystem and the kernel-facing device-node namespace
//! broker. Hardware callbacks remain in kernel endpoint receipts; this module
//! owns device namespace policy and filesystem operations.

extern crate alloc;

use alloc::{
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    ffi::c_void,
    mem::ManuallyDrop,
    ptr::{self, NonNull},
    sync::atomic::{AtomicU16, AtomicU64, Ordering},
};

use ddk::{self, DevfsEndpoint, Module, TicketLock, raw};

const ROOT_ID: u64 = 1;
const DIRECTORY_CURSOR_BASE: u64 = 2;
const NODE_REVOKED: u64 = 1 << 63;
const NODE_OPEN_ONE: u64 = 1 << 32;
const NODE_OPEN_MASK: u64 = ((1 << 31) - 1) << 32;
const NODE_ACTIVE_MASK: u64 = u32::MAX as u64;

struct Namespace {
    next_node: AtomicU64,
    used_nodes: AtomicU64,
    records: TicketLock<BTreeMap<u64, Record>>,
    root: Arc<Node>,
}

// SAFETY: namespace mutation is serialized by `records`; node mutable state is
// independently atomic or lock-protected.
unsafe impl Send for Namespace {}
// SAFETY: as above.
unsafe impl Sync for Namespace {}

struct Mount {
    namespace: Arc<Namespace>,
}

// SAFETY: a mount only owns an Arc to synchronized namespace state.
unsafe impl Send for Mount {}
// SAFETY: as above.
unsafe impl Sync for Mount {}

struct Record {
    owner: u64,
    parent: u64,
    node: Arc<Node>,
    children: BTreeMap<Arc<[u8]>, u64>,
    ordered: BTreeMap<u64, Arc<[u8]>>,
}

struct Node {
    namespace: Weak<Namespace>,
    id: u64,
    kind: u32,
    mode: AtomicU16,
    links: AtomicU64,
    accessed_ns: AtomicU64,
    modified_ns: AtomicU64,
    changed_ns: AtomicU64,
    lifecycle: AtomicU64,
    data: NodeData,
}

// SAFETY: endpoint calls are explicitly synchronized by their driver ABI;
// namespace and lifecycle metadata use locks and atomics.
unsafe impl Send for Node {}
// SAFETY: as above.
unsafe impl Sync for Node {}

enum NodeData {
    Directory,
    Device(Endpoint),
}

struct Endpoint(*mut raw::DevfsEndpoint);

// SAFETY: this opaque receipt is only called through the kernel service table.
unsafe impl Send for Endpoint {}
// SAFETY: as above.
unsafe impl Sync for Endpoint {}

impl Drop for Endpoint {
    fn drop(&mut self) {
        // SAFETY: a node owns this endpoint receipt until it is finally
        // dropped after namespace removal and outstanding VFS references.
        unsafe { DevfsEndpoint::release(self.0) };
    }
}

struct NodeActivity<'a>(&'a AtomicU64);

impl Drop for NodeActivity<'_> {
    fn drop(&mut self) {
        let mut state = self.0.load(Ordering::Acquire);
        while state & NODE_ACTIVE_MASK != 0 {
            match self.0.compare_exchange_weak(
                state,
                state - 1,
                Ordering::Release,
                Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(actual) => state = actual,
            }
        }
    }
}

static NAMESPACE: TicketLock<Option<Arc<Namespace>>> = TicketLock::new(None);
static PROVIDER_REGISTRATION: TicketLock<Option<usize>> = TicketLock::new(None);
static BROKER_REGISTRATION: TicketLock<Option<usize>> = TicketLock::new(None);

impl Namespace {
    fn new() -> Arc<Self> {
        Arc::new_cyclic(|weak| {
            let now = ddk::monotonic_ns();
            let root = Arc::new(Node {
                namespace: weak.clone(),
                id: ROOT_ID,
                kind: ddk::FS_KIND_DIRECTORY,
                mode: AtomicU16::new(0o755),
                links: AtomicU64::new(2),
                accessed_ns: AtomicU64::new(now),
                modified_ns: AtomicU64::new(now),
                changed_ns: AtomicU64::new(now),
                lifecycle: AtomicU64::new(0),
                data: NodeData::Directory,
            });
            let mut records = BTreeMap::new();
            records.insert(
                ROOT_ID,
                Record {
                    owner: 0,
                    parent: ROOT_ID,
                    node: root.clone(),
                    children: BTreeMap::new(),
                    ordered: BTreeMap::new(),
                },
            );
            Self {
                next_node: AtomicU64::new(ROOT_ID + 1),
                used_nodes: AtomicU64::new(1),
                records: TicketLock::new(records),
                root,
            }
        })
    }

    fn make_node(self: &Arc<Self>, id: u64, kind: u32, mode: u16, data: NodeData) -> Arc<Node> {
        let now = ddk::monotonic_ns();
        Arc::new(Node {
            namespace: Arc::downgrade(self),
            id,
            kind,
            mode: AtomicU16::new(mode),
            links: AtomicU64::new(if kind == ddk::FS_KIND_DIRECTORY { 2 } else { 1 }),
            accessed_ns: AtomicU64::new(now),
            modified_ns: AtomicU64::new(now),
            changed_ns: AtomicU64::new(now),
            lifecycle: AtomicU64::new(0),
            data,
        })
    }

    fn create_directory(
        self: &Arc<Self>,
        owner: u64,
        parent: u64,
        name: &[u8],
        mode: u16,
    ) -> core::result::Result<u64, i32> {
        self.create_node(
            owner,
            parent,
            name,
            ddk::FS_KIND_DIRECTORY,
            mode,
            NodeData::Directory,
        )
    }

    fn create_device(
        self: &Arc<Self>,
        owner: u64,
        parent: u64,
        name: &[u8],
        kind: u32,
        mode: u16,
        endpoint: *mut raw::DevfsEndpoint,
    ) -> core::result::Result<u64, i32> {
        if endpoint.is_null()
            || !matches!(
                kind,
                ddk::FS_KIND_CHARACTER_DEVICE | ddk::FS_KIND_BLOCK_DEVICE
            )
        {
            return Err(ddk::EINVAL);
        }
        validate_name(name)?;
        let mut records = self.records.lock();
        let parent_record = records.get(&parent).ok_or(ddk::ENOENT)?;
        if parent_record.node.kind != ddk::FS_KIND_DIRECTORY {
            return Err(ddk::ENOTDIR);
        }
        if parent_record.children.contains_key(name) {
            return Err(ddk::EEXIST);
        }
        let id = self.next_node.fetch_add(1, Ordering::Relaxed);
        if id == 0 {
            return Err(ddk::ENOSPC);
        }
        // Ownership crosses the broker boundary only after all validations
        // above succeed. A failed broker call therefore leaves the endpoint
        // receipt with the kernel caller.
        let node = self.make_node(id, kind, mode, NodeData::Device(Endpoint(endpoint)));
        let name = Arc::<[u8]>::from(name);
        let parent_record = records.get_mut(&parent).ok_or(ddk::ENOENT)?;
        parent_record.children.insert(name.clone(), id);
        parent_record.ordered.insert(id, name);
        records.insert(
            id,
            Record {
                owner,
                parent,
                node,
                children: BTreeMap::new(),
                ordered: BTreeMap::new(),
            },
        );
        self.used_nodes.fetch_add(1, Ordering::AcqRel);
        Ok(id)
    }

    fn create_node(
        self: &Arc<Self>,
        owner: u64,
        parent: u64,
        name: &[u8],
        kind: u32,
        mode: u16,
        data: NodeData,
    ) -> core::result::Result<u64, i32> {
        validate_name(name)?;
        let mut records = self.records.lock();
        let parent_record = records.get(&parent).ok_or(ddk::ENOENT)?;
        if parent_record.node.kind != ddk::FS_KIND_DIRECTORY {
            return Err(ddk::ENOTDIR);
        }
        if parent_record.children.contains_key(name) {
            return Err(ddk::EEXIST);
        }
        let id = self.next_node.fetch_add(1, Ordering::Relaxed);
        if id == 0 {
            return Err(ddk::ENOSPC);
        }
        let node = self.make_node(id, kind, mode, data);
        let name = Arc::<[u8]>::from(name);
        let parent_record = records.get_mut(&parent).ok_or(ddk::ENOENT)?;
        parent_record.children.insert(name.clone(), id);
        parent_record.ordered.insert(id, name);
        if kind == ddk::FS_KIND_DIRECTORY {
            parent_record.node.links.fetch_add(1, Ordering::AcqRel);
        }
        records.insert(
            id,
            Record {
                owner,
                parent,
                node,
                children: BTreeMap::new(),
                ordered: BTreeMap::new(),
            },
        );
        self.used_nodes.fetch_add(1, Ordering::AcqRel);
        Ok(id)
    }

    fn remove(&self, owner: u64, id: u64) -> core::result::Result<(), i32> {
        if id == ROOT_ID {
            return Err(ddk::EPERM);
        }
        let mut records = self.records.lock();
        let record = records.get(&id).ok_or(ddk::ENOENT)?;
        if record.owner != owner {
            return Err(ddk::EPERM);
        }
        if !record.children.is_empty() {
            return Err(ddk::ENOTEMPTY);
        }
        record.node.try_revoke()?;
        self.remove_record(&mut records, id)
    }

    fn remove_owner(&self, owner: u64, force: bool) -> core::result::Result<(), i32> {
        let mut records = self.records.lock();
        validate_no_foreign_children(&records, owner)?;
        let ids: Vec<u64> = records
            .iter()
            .filter_map(|(id, record)| (record.owner == owner).then_some(*id))
            .collect();
        if !force {
            for id in &ids {
                let record = records.get(id).ok_or(ddk::ENOENT)?;
                if !record.node.is_idle() {
                    return Err(ddk::EBUSY);
                }
            }
        }
        for id in &ids {
            let record = records.get(id).ok_or(ddk::ENOENT)?;
            if force {
                record.node.force_revoke();
            } else if let Err(error) = record.node.try_revoke() {
                for restored in &ids {
                    if restored == id {
                        break;
                    }
                    if let Some(record) = records.get(restored) {
                        record.node.restore();
                    }
                }
                return Err(error);
            }
        }
        let mut ids = ids;
        ids.sort_unstable_by(|left, right| right.cmp(left));
        for id in ids {
            self.remove_record(&mut records, id)?;
        }
        Ok(())
    }

    fn remove_record(
        &self,
        records: &mut BTreeMap<u64, Record>,
        id: u64,
    ) -> core::result::Result<(), i32> {
        let record = records.remove(&id).ok_or(ddk::ENOENT)?;
        if let Some(parent) = records.get_mut(&record.parent) {
            if let Some(name) = parent.ordered.remove(&id) {
                parent.children.remove(&name);
            } else {
                parent.children.retain(|_, child| *child != id);
            }
            if record.node.kind == ddk::FS_KIND_DIRECTORY {
                decrement_if_above(&parent.node.links, 2);
            }
        }
        self.used_nodes.fetch_sub(1, Ordering::AcqRel);
        Ok(())
    }
}

impl Node {
    fn namespace_matches(&self, namespace: &Arc<Namespace>) -> bool {
        self.namespace
            .upgrade()
            .is_some_and(|node_namespace| Arc::ptr_eq(&node_namespace, namespace))
    }

    fn endpoint(&self) -> core::result::Result<&Endpoint, i32> {
        match &self.data {
            NodeData::Device(endpoint) => Ok(endpoint),
            NodeData::Directory => Err(ddk::EISDIR),
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

    fn ensure_live(&self) -> core::result::Result<(), i32> {
        if self.lifecycle.load(Ordering::Acquire) & NODE_REVOKED != 0 {
            Err(ddk::ENOENT)
        } else {
            Ok(())
        }
    }

    fn begin_activity(&self) -> core::result::Result<NodeActivity<'_>, i32> {
        loop {
            let state = self.lifecycle.load(Ordering::Acquire);
            if state & NODE_REVOKED != 0 || state & NODE_ACTIVE_MASK == NODE_ACTIVE_MASK {
                return Err(ddk::EBUSY);
            }
            if self
                .lifecycle
                .compare_exchange_weak(state, state + 1, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return Ok(NodeActivity(&self.lifecycle));
            }
        }
    }

    fn mark_open(&self) -> core::result::Result<(), i32> {
        loop {
            let state = self.lifecycle.load(Ordering::Acquire);
            if state & NODE_REVOKED != 0 || state & NODE_OPEN_MASK == NODE_OPEN_MASK {
                return Err(ddk::EBUSY);
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
        let mut state = self.lifecycle.load(Ordering::Acquire);
        while state & NODE_OPEN_MASK != 0 {
            match self.lifecycle.compare_exchange_weak(
                state,
                state - NODE_OPEN_ONE,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return,
                Err(actual) => state = actual,
            }
        }
    }

    fn is_idle(&self) -> bool {
        self.lifecycle.load(Ordering::Acquire) == 0
    }

    fn try_revoke(&self) -> core::result::Result<(), i32> {
        self.lifecycle
            .compare_exchange(0, NODE_REVOKED, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| ddk::EBUSY)
    }

    fn force_revoke(&self) {
        self.lifecycle.fetch_or(NODE_REVOKED, Ordering::AcqRel);
    }

    fn restore(&self) {
        let _ =
            self.lifecycle
                .compare_exchange(NODE_REVOKED, 0, Ordering::Release, Ordering::Relaxed);
    }
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

fn validate_no_foreign_children(
    records: &BTreeMap<u64, Record>,
    owner: u64,
) -> core::result::Result<(), i32> {
    for record in records.values().filter(|record| record.owner == owner) {
        if record
            .children
            .values()
            .any(|child| records.get(child).is_some_and(|child| child.owner != owner))
        {
            return Err(ddk::EBUSY);
        }
    }
    Ok(())
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

fn namespace() -> core::result::Result<Arc<Namespace>, i32> {
    NAMESPACE.lock().clone().ok_or(ddk::ENODEV)
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
    // SAFETY: matching provider ownership retains the Arc through callbacks.
    Ok(ManuallyDrop::new(unsafe {
        Arc::from_raw(receipt.cast::<T>())
    }))
}

unsafe fn borrow_mount(
    receipt: *mut c_void,
) -> core::result::Result<ManuallyDrop<Arc<Mount>>, i32> {
    // SAFETY: forwarded from the provider ABI callback contract.
    unsafe { borrow_arc(receipt) }
}

unsafe fn borrow_node(receipt: *mut c_void) -> core::result::Result<ManuallyDrop<Arc<Node>>, i32> {
    // SAFETY: forwarded from the provider ABI callback contract.
    unsafe { borrow_arc(receipt) }
}

fn checked_node(
    mount: &Mount,
    receipt: *mut c_void,
) -> core::result::Result<ManuallyDrop<Arc<Node>>, i32> {
    // SAFETY: VFS retains every vnode receipt it passes to provider callbacks.
    let node = unsafe { borrow_node(receipt) }?;
    if !node.namespace_matches(&mount.namespace) {
        return Err(ddk::EINVAL);
    }
    Ok(node)
}

fn input<'a>(pointer: *const u8, length: usize) -> core::result::Result<&'a [u8], i32> {
    if length == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(ddk::EINVAL);
    }
    // SAFETY: the ABI gives this provider a readable buffer for the callback.
    Ok(unsafe { core::slice::from_raw_parts(pointer, length) })
}

fn output<'a>(pointer: *mut u8, length: usize) -> core::result::Result<&'a mut [u8], i32> {
    if length == 0 {
        return Ok(&mut []);
    }
    if pointer.is_null() {
        return Err(ddk::EINVAL);
    }
    // SAFETY: the ABI gives this provider a writable buffer for the callback.
    Ok(unsafe { core::slice::from_raw_parts_mut(pointer, length) })
}

unsafe extern "C" fn provider_mount(
    _context: *mut c_void,
    options: *const raw::FsMountOptions,
    output: *mut *mut c_void,
) -> i32 {
    if options.is_null() || output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: options is a fixed C ABI input.
    if unsafe { (*options).size } < core::mem::size_of::<raw::FsMountOptions>() as u32 {
        return ddk::EINVAL;
    }
    let namespace = {
        let mut namespace = NAMESPACE.lock();
        if let Some(namespace) = namespace.as_ref() {
            Arc::clone(namespace)
        } else {
            let created = Namespace::new();
            *namespace = Some(Arc::clone(&created));
            created
        }
    };
    let mount = Arc::new(Mount { namespace });
    // SAFETY: output is writable for the opaque mount receipt.
    unsafe { *output = Arc::into_raw(mount).cast_mut().cast() };
    ddk::OK
}

unsafe extern "C" fn provider_unmount(_context: *mut c_void, receipt: *mut c_void) -> i32 {
    if receipt.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS consumes the mount receipt exactly once after child vnodes.
    drop(unsafe { Arc::from_raw(receipt.cast::<Mount>()) });
    ddk::OK
}

unsafe extern "C" fn provider_root(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    output: *mut raw::FsVnode,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    // SAFETY: output is writable for a vnode record.
    unsafe { *output = vnode(&mount.namespace.root) };
    ddk::OK
}

unsafe extern "C" fn provider_statfs(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    output: *mut raw::FsStat,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    // SAFETY: output is writable for a fixed ABI record.
    unsafe {
        *output = raw::FsStat {
            total_bytes: 0,
            used_bytes: 0,
            total_nodes: u64::MAX,
            used_nodes: mount.namespace.used_nodes.load(Ordering::Acquire),
        };
    }
    ddk::OK
}

unsafe extern "C" fn provider_sync(_context: *mut c_void, mount_receipt: *mut c_void) -> i32 {
    // SAFETY: validates the mount receipt without consuming it.
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
        // SAFETY: VFS consumes each vnode receipt exactly once.
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
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if let Err(error) = node.ensure_live() {
        return error;
    }
    let size = match &node.data {
        NodeData::Directory => 0,
        NodeData::Device(endpoint) => {
            let Ok(_activity) = node.begin_activity() else {
                return ddk::EBUSY;
            };
            // SAFETY: endpoint is owned by this live node.
            unsafe { DevfsEndpoint::size(endpoint.0) }
        }
    };
    // SAFETY: output is writable for a fixed ABI record.
    unsafe {
        *output = raw::FsAttr {
            size,
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
    // SAFETY: fixed ABI input remains live during the callback.
    let attributes = unsafe { &*attributes };
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if let Err(error) = node.ensure_live() {
        return error;
    }
    if attributes.valid & !(ddk::FS_SETATTR_SIZE | ddk::FS_SETATTR_MODE) != 0 {
        return ddk::EINVAL;
    }
    if attributes.valid & ddk::FS_SETATTR_SIZE != 0 {
        return ddk::ENOTSUP;
    }
    if attributes.valid & ddk::FS_SETATTR_MODE != 0 {
        node.mode.store(attributes.mode, Ordering::Release);
        node.changed_ns
            .store(ddk::monotonic_ns(), Ordering::Release);
    }
    ddk::OK
}

unsafe extern "C" fn lookup(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    name: *const u8,
    name_length: usize,
    output_vnode: *mut raw::FsVnode,
) -> i32 {
    if output_vnode.is_null() {
        return ddk::EINVAL;
    }
    let Ok(name) = input(name, name_length) else {
        return ddk::EINVAL;
    };
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if let Err(error) = directory.ensure_live() {
        return error;
    }
    if directory.kind != ddk::FS_KIND_DIRECTORY {
        return ddk::ENOTDIR;
    }
    let child = if name == b"." {
        Arc::clone(&directory)
    } else if name == b".." {
        let records = mount.namespace.records.lock();
        let parent = records
            .get(&directory.id)
            .map_or(ROOT_ID, |record| record.parent);
        records
            .get(&parent)
            .map_or_else(|| Arc::clone(&directory), |record| record.node.clone())
    } else {
        if validate_name(name).is_err() {
            return ddk::EINVAL;
        }
        let records = mount.namespace.records.lock();
        let Some(record) = records.get(&directory.id) else {
            return ddk::ENOENT;
        };
        let Some(id) = record.children.get(name) else {
            return ddk::ENOENT;
        };
        let Some(record) = records.get(id) else {
            return ddk::ENOENT;
        };
        record.node.clone()
    };
    directory.touch_accessed();
    // SAFETY: output is writable for one vnode ABI record.
    unsafe { *output_vnode = vnode(&child) };
    ddk::OK
}

unsafe extern "C" fn parent(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    output_vnode: *mut raw::FsVnode,
) -> i32 {
    if output_vnode.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if let Err(error) = directory.ensure_live() {
        return error;
    }
    if directory.kind != ddk::FS_KIND_DIRECTORY {
        return ddk::ENOTDIR;
    }
    let records = mount.namespace.records.lock();
    let parent = records
        .get(&directory.id)
        .map_or(ROOT_ID, |record| record.parent);
    let node = records
        .get(&parent)
        .map_or_else(|| Arc::clone(&directory), |record| record.node.clone());
    // SAFETY: output is writable for one vnode ABI record.
    unsafe { *output_vnode = vnode(&node) };
    ddk::OK
}

unsafe extern "C" fn reject_mutation(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
) -> i32 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if let Err(error) = node.ensure_live() {
        return error;
    }
    if node.kind == ddk::FS_KIND_DIRECTORY {
        ddk::EPERM
    } else {
        ddk::ENOTDIR
    }
}

unsafe extern "C" fn create(
    context: *mut c_void,
    mount: *mut c_void,
    directory: *mut c_void,
    _name: *const u8,
    _name_length: usize,
    _kind: u32,
    _target: *const u8,
    _target_length: usize,
    _mode: u16,
    _output: *mut raw::FsVnode,
) -> i32 {
    // SAFETY: only validates opaque receipts; no caller buffer is read.
    unsafe { reject_mutation(context, mount, directory) }
}

unsafe extern "C" fn link(
    context: *mut c_void,
    mount: *mut c_void,
    directory: *mut c_void,
    _name: *const u8,
    _name_length: usize,
    _target: *mut c_void,
) -> i32 {
    // SAFETY: only validates opaque receipts; no caller buffer is read.
    unsafe { reject_mutation(context, mount, directory) }
}

unsafe extern "C" fn unlink(
    context: *mut c_void,
    mount: *mut c_void,
    directory: *mut c_void,
    _name: *const u8,
    _name_length: usize,
    _remove_directory: u8,
) -> i32 {
    // SAFETY: only validates opaque receipts; no caller buffer is read.
    unsafe { reject_mutation(context, mount, directory) }
}

unsafe extern "C" fn rename(
    context: *mut c_void,
    mount: *mut c_void,
    source_directory: *mut c_void,
    _source_name: *const u8,
    _source_name_length: usize,
    _target_directory: *mut c_void,
    _target_name: *const u8,
    _target_name_length: usize,
) -> i32 {
    // SAFETY: only validates source opaque receipts; namespace mutation is
    // denied uniformly for devfs user operations.
    unsafe { reject_mutation(context, mount, source_directory) }
}

unsafe extern "C" fn open(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    flags: u32,
    output_file: *mut usize,
) -> i32 {
    if output_file.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let file = match &node.data {
        NodeData::Directory => {
            if let Err(error) = node.mark_open() {
                return error;
            }
            0
        }
        NodeData::Device(endpoint) => {
            let Ok(_activity) = node.begin_activity() else {
                return ddk::EBUSY;
            };
            // SAFETY: endpoint belongs to this live node.
            let file = match unsafe { DevfsEndpoint::open(endpoint.0, flags) } {
                Ok(file) => file,
                Err(error) => return error.status(),
            };
            if let Err(error) = node.mark_open() {
                // SAFETY: undo the successfully opened endpoint state.
                unsafe { DevfsEndpoint::close(endpoint.0, file, flags) };
                return error;
            }
            file
        }
    };
    // SAFETY: output is writable scalar storage.
    unsafe { *output_file = file };
    ddk::OK
}

unsafe extern "C" fn close(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    flags: u32,
) {
    // SAFETY: VFS retains mount/vnode receipts until close returns.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return;
    };
    if let NodeData::Device(endpoint) = &node.data {
        node.lifecycle.fetch_add(1, Ordering::AcqRel);
        // SAFETY: close balances the matching endpoint open even after forced
        // revocation, while the node still owns its endpoint receipt.
        unsafe { DevfsEndpoint::close(endpoint.0, file, flags) };
        node.lifecycle.fetch_sub(1, Ordering::AcqRel);
    }
    node.mark_closed();
}

unsafe extern "C" fn initial_offset(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    flags: u32,
    output_offset: *mut u64,
) -> i32 {
    if output_offset.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let offset = match &node.data {
        NodeData::Directory => {
            if let Err(error) = node.ensure_live() {
                return error;
            }
            0
        }
        NodeData::Device(endpoint) => {
            let Ok(_activity) = node.begin_activity() else {
                return ddk::EBUSY;
            };
            // SAFETY: endpoint belongs to this live node.
            match unsafe { DevfsEndpoint::initial_offset(endpoint.0, file, flags) } {
                Ok(offset) => offset,
                Err(error) => return error.status(),
            }
        }
    };
    // SAFETY: output is writable scalar storage.
    unsafe { *output_offset = offset };
    ddk::OK
}

unsafe extern "C" fn read(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    offset: u64,
    buffer: *mut u8,
    length: usize,
    flags: u32,
) -> i64 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(buffer) = output(buffer, length) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(activity) = node.begin_activity() else {
        return i64::from(ddk::EBUSY);
    };
    let endpoint = match node.endpoint() {
        Ok(endpoint) => endpoint,
        Err(error) => return i64::from(error),
    };
    // SAFETY: endpoint is live and buffer aliases the VFS transfer window.
    let result = unsafe { DevfsEndpoint::read(endpoint.0, file, offset, buffer, flags) };
    drop(activity);
    match result {
        Ok(read) => {
            node.touch_accessed();
            i64::try_from(read).unwrap_or(i64::MAX)
        }
        Err(error) => i64::from(error.status()),
    }
}

unsafe extern "C" fn write(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    offset: u64,
    buffer: *const u8,
    length: usize,
    flags: u32,
) -> i64 {
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(buffer) = input(buffer, length) else {
        return i64::from(ddk::EINVAL);
    };
    let Ok(activity) = node.begin_activity() else {
        return i64::from(ddk::EBUSY);
    };
    let endpoint = match node.endpoint() {
        Ok(endpoint) => endpoint,
        Err(error) => return i64::from(error),
    };
    // SAFETY: endpoint is live and buffer aliases the VFS transfer window.
    let result = unsafe { DevfsEndpoint::write(endpoint.0, file, offset, buffer, flags) };
    drop(activity);
    match result {
        Ok(written) => {
            node.touch_modified();
            i64::try_from(written).unwrap_or(i64::MAX)
        }
        Err(error) => i64::from(error.status()),
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
    // SAFETY: readdir validates capacity and fills each output slot once.
    unsafe { *output.add(index) = entry };
    Ok(())
}

unsafe extern "C" fn readdir(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    cursor: u64,
    output_entries: *mut raw::FsDirEntry,
    capacity: usize,
    count: *mut usize,
    next: *mut u64,
) -> i32 {
    if count.is_null() || next.is_null() || (capacity != 0 && output_entries.is_null()) {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(directory) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    if let Err(error) = directory.ensure_live() {
        return error;
    }
    if directory.kind != ddk::FS_KIND_DIRECTORY {
        return ddk::ENOTDIR;
    }
    let mut emitted = 0usize;
    let mut offset = cursor;
    if capacity != 0 && offset == 0 {
        if let Err(error) = emit_entry(output_entries, emitted, &directory, b".", 1) {
            return error;
        }
        emitted += 1;
        offset = 1;
    }
    let records = mount.namespace.records.lock();
    if emitted < capacity && offset == 1 {
        let parent_id = records
            .get(&directory.id)
            .map_or(ROOT_ID, |record| record.parent);
        let parent = records
            .get(&parent_id)
            .map_or_else(|| Arc::clone(&directory), |record| record.node.clone());
        if let Err(error) = emit_entry(
            output_entries,
            emitted,
            &parent,
            b"..",
            DIRECTORY_CURSOR_BASE,
        ) {
            return error;
        }
        emitted += 1;
        offset = DIRECTORY_CURSOR_BASE;
    }
    if emitted < capacity {
        let Some(record) = records.get(&directory.id) else {
            return ddk::ENOENT;
        };
        for (id, name) in record.ordered.range(offset.max(DIRECTORY_CURSOR_BASE)..) {
            if emitted == capacity {
                break;
            }
            let Some(child) = records.get(id) else {
                continue;
            };
            offset = id.saturating_add(1);
            if let Err(error) = emit_entry(output_entries, emitted, &child.node, name, offset) {
                return error;
            }
            emitted += 1;
        }
    }
    drop(records);
    directory.touch_accessed();
    // SAFETY: scalar output pointers are valid by the provider ABI contract.
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
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    match &node.data {
        NodeData::Directory => node.ensure_live().map_or_else(|error| error, |()| ddk::OK),
        NodeData::Device(endpoint) => {
            let Ok(_activity) = node.begin_activity() else {
                return ddk::EBUSY;
            };
            // SAFETY: endpoint belongs to this live node.
            unsafe { DevfsEndpoint::sync(endpoint.0) }.map_or_else(ddk::Error::status, |()| ddk::OK)
        }
    }
}

unsafe extern "C" fn poll(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    offset: u64,
    events: u16,
    flags: u32,
    ready: *mut u16,
) -> i32 {
    if ready.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let result = match &node.data {
        NodeData::Directory => {
            if let Err(error) = node.ensure_live() {
                return error;
            }
            if flags & 0b11 != 0 {
                events & (ddk::POLL_IN | ddk::POLL_RDNORM)
            } else {
                0
            }
        }
        NodeData::Device(endpoint) => {
            let Ok(_activity) = node.begin_activity() else {
                return ddk::EBUSY;
            };
            // SAFETY: endpoint belongs to this live node.
            match unsafe { DevfsEndpoint::poll(endpoint.0, file, offset, events, flags) } {
                Ok(ready) => ready,
                Err(error) => return error.status(),
            }
        }
    };
    // SAFETY: ready is scalar output storage.
    unsafe { *ready = result };
    ddk::OK
}

unsafe extern "C" fn poll_events(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    events: u16,
    output_events: *mut usize,
    capacity: usize,
    count: *mut usize,
) -> i32 {
    if count.is_null() || (capacity != 0 && output_events.is_null()) {
        return ddk::EINVAL;
    }
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let NodeData::Device(endpoint) = &node.data else {
        // SAFETY: count is valid scalar output.
        unsafe { *count = 0 };
        return ddk::OK;
    };
    let Ok(_activity) = node.begin_activity() else {
        return ddk::EBUSY;
    };
    let selectors = [
        (ddk::DEVFS_EVENT_READABLE, ddk::POLL_IN | ddk::POLL_RDNORM),
        (ddk::DEVFS_EVENT_WRITABLE, ddk::POLL_OUT | ddk::POLL_WRNORM),
        (ddk::DEVFS_EVENT_HANGUP, ddk::POLL_HUP),
    ];
    let mut emitted = 0;
    for (selector, interested) in selectors {
        if emitted == capacity || events & interested == 0 {
            continue;
        }
        // SAFETY: endpoint belongs to this live node.
        let event = unsafe { DevfsEndpoint::event(endpoint.0, file, selector) };
        if event != 0 {
            // SAFETY: capacity validates this output slot.
            unsafe { *output_events.add(emitted) = event };
            emitted += 1;
        }
    }
    // SAFETY: count is valid scalar output.
    unsafe { *count = emitted };
    ddk::OK
}

unsafe extern "C" fn terminal_state(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    output: *mut raw::FsTerminalState,
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
    let Ok(endpoint) = node.endpoint() else {
        return ddk::ENOTTY;
    };
    let Ok(_activity) = node.begin_activity() else {
        return ddk::EBUSY;
    };
    // SAFETY: endpoint and output record are live for this immediate call.
    unsafe { DevfsEndpoint::terminal_state(endpoint.0, &mut *output) }
        .map_or_else(ddk::Error::status, |()| ddk::OK)
}

unsafe extern "C" fn ioctl(
    _context: *mut c_void,
    mount_receipt: *mut c_void,
    vnode_receipt: *mut c_void,
    file: usize,
    process: u64,
    group: i32,
    session: i32,
    session_leader: u8,
    request: u64,
    value: u64,
    argument: *mut u8,
    length: usize,
    output_result: *mut u64,
) -> i32 {
    if output_result.is_null() {
        return ddk::EINVAL;
    }
    let Ok(argument) = output(argument, length) else {
        return ddk::EINVAL;
    };
    // SAFETY: VFS retains the mount receipt.
    let Ok(mount) = (unsafe { borrow_mount(mount_receipt) }) else {
        return ddk::EINVAL;
    };
    let Ok(node) = checked_node(&mount, vnode_receipt) else {
        return ddk::EINVAL;
    };
    let Ok(activity) = node.begin_activity() else {
        return ddk::EBUSY;
    };
    let Ok(endpoint) = node.endpoint() else {
        return ddk::ENOTTY;
    };
    // SAFETY: endpoint is live and argument is the VFS-owned ioctl buffer.
    let result = unsafe {
        DevfsEndpoint::ioctl(
            endpoint.0,
            file,
            process,
            group,
            session,
            session_leader != 0,
            request,
            value,
            argument,
        )
    };
    drop(activity);
    match result {
        Ok(result) => {
            // SAFETY: output_result is valid scalar output storage.
            unsafe { *output_result = result };
            ddk::OK
        }
        Err(error) => error.status(),
    }
}

unsafe extern "C" fn broker_root(_context: *mut c_void, output: *mut u64) -> i32 {
    if output.is_null() || namespace().is_err() {
        return ddk::ENODEV;
    }
    // SAFETY: output is writable scalar storage.
    unsafe { *output = ROOT_ID };
    ddk::OK
}

unsafe extern "C" fn broker_mkdir(
    _context: *mut c_void,
    owner: u64,
    parent: u64,
    name: *const u8,
    name_length: usize,
    mode: u16,
    output: *mut u64,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    let Ok(name) = input(name, name_length) else {
        return ddk::EINVAL;
    };
    let Ok(namespace) = namespace() else {
        return ddk::ENODEV;
    };
    match namespace.create_directory(owner, parent, name, mode) {
        Ok(node) => {
            // SAFETY: output is writable scalar storage.
            unsafe { *output = node };
            ddk::OK
        }
        Err(error) => error,
    }
}

unsafe extern "C" fn broker_create(
    _context: *mut c_void,
    owner: u64,
    parent: u64,
    name: *const u8,
    name_length: usize,
    kind: u32,
    mode: u16,
    endpoint: *mut raw::DevfsEndpoint,
    output: *mut u64,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    let Ok(name) = input(name, name_length) else {
        return ddk::EINVAL;
    };
    let Ok(namespace) = namespace() else {
        return ddk::ENODEV;
    };
    match namespace.create_device(owner, parent, name, kind, mode, endpoint) {
        Ok(node) => {
            // SAFETY: success transfers endpoint ownership to the namespace
            // node and writes its ID to the caller's output.
            unsafe { *output = node };
            ddk::OK
        }
        Err(error) => error,
    }
}

unsafe extern "C" fn broker_remove(_context: *mut c_void, owner: u64, node: u64) -> i32 {
    match namespace().and_then(|namespace| namespace.remove(owner, node)) {
        Ok(()) => ddk::OK,
        Err(error) => error,
    }
}

unsafe extern "C" fn broker_remove_owner(_context: *mut c_void, owner: u64, force: u8) -> i32 {
    match namespace().and_then(|namespace| namespace.remove_owner(owner, force != 0)) {
        Ok(()) => ddk::OK,
        Err(error) => error,
    }
}

unsafe extern "C" fn broker_lookup(
    _context: *mut c_void,
    path: *const u8,
    path_length: usize,
    output: *mut u64,
) -> i32 {
    if output.is_null() {
        return ddk::EINVAL;
    }
    let Ok(path) = input(path, path_length) else {
        return ddk::EINVAL;
    };
    let Ok(namespace) = namespace() else {
        return ddk::ENODEV;
    };
    let mut current = ROOT_ID;
    let records = namespace.records.lock();
    for part in path
        .split(|byte| *byte == b'/')
        .filter(|part| !part.is_empty())
    {
        let Some(record) = records.get(&current) else {
            return ddk::ENOENT;
        };
        let Some(next) = record.children.get(part) else {
            return ddk::ENOENT;
        };
        current = *next;
    }
    // SAFETY: output is writable scalar storage.
    unsafe { *output = current };
    ddk::OK
}

unsafe extern "C" fn broker_children(
    _context: *mut c_void,
    parent: u64,
    output: *mut raw::DevfsBrokerEntry,
    capacity: usize,
    count: *mut usize,
) -> i32 {
    if count.is_null() || (capacity != 0 && output.is_null()) {
        return ddk::EINVAL;
    }
    let Ok(namespace) = namespace() else {
        return ddk::ENODEV;
    };
    let records = namespace.records.lock();
    let Some(record) = records.get(&parent) else {
        return ddk::ENOENT;
    };
    if record.node.kind != ddk::FS_KIND_DIRECTORY {
        return ddk::ENOTDIR;
    }
    let required = record.children.len();
    // SAFETY: `count` is writable scalar output under the broker ABI.
    unsafe { *count = required };
    if capacity == 0 {
        return ddk::OK;
    }
    if capacity < required {
        return ddk::ENOSPC;
    }
    for (index, (name, node)) in record.children.iter().enumerate() {
        if name.len() > raw::FS_DIRECTORY_NAME_MAX {
            return ddk::EIO;
        }
        let mut entry = raw::DevfsBrokerEntry {
            node: *node,
            name_length: name.len() as u16,
            reserved: [0; 6],
            name: [0; raw::FS_DIRECTORY_NAME_MAX],
        };
        entry.name[..name.len()].copy_from_slice(name);
        // SAFETY: capacity was checked against every child before iteration.
        unsafe { *output.add(index) = entry };
    }
    ddk::OK
}

static PROVIDER_OPERATIONS: raw::FsProviderOps = raw::FsProviderOps {
    size: raw::FS_PROVIDER_OPS_SIZE,
    context: ptr::null_mut(),
    mount: Some(provider_mount),
    unmount: Some(provider_unmount),
    root: Some(provider_root),
    statfs: Some(provider_statfs),
    sync: Some(provider_sync),
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
    append: None,
    truncate: None,
    memory_object: None,
    readlink: None,
    readdir: Some(readdir),
    fsync: Some(fsync),
    poll: Some(poll),
    poll_events: Some(poll_events),
    terminal_state: Some(terminal_state),
    ioctl: Some(ioctl),
};

static BROKER_OPERATIONS: raw::DevfsBrokerOps = raw::DevfsBrokerOps {
    size: raw::DEVFS_BROKER_OPS_SIZE,
    context: ptr::null_mut(),
    root: Some(broker_root),
    mkdir: Some(broker_mkdir),
    create: Some(broker_create),
    remove: Some(broker_remove),
    remove_owner: Some(broker_remove_owner),
    lookup: Some(broker_lookup),
    children: Some(broker_children),
};

fn devfs_init(_module: Module) -> ddk::Result<()> {
    let provider = ddk::fs_provider_register(c"devfs", &PROVIDER_OPERATIONS)?;
    *PROVIDER_REGISTRATION.lock() = Some(provider.as_ptr() as usize);
    let broker = ddk::devfs_broker_register(&BROKER_OPERATIONS)?;
    *BROKER_REGISTRATION.lock() = Some(broker.as_ptr() as usize);
    Ok(())
}

fn devfs_exit(_module: Module) {
    if let Some(registration) = BROKER_REGISTRATION.lock().take()
        && let Some(registration) = NonNull::new(registration as *mut raw::DevfsBroker)
    {
        // SAFETY: module teardown owns this broker receipt after all namespace
        // users and provider leases have gone away.
        let _ = unsafe { ddk::devfs_broker_unregister(registration) };
    }
    if let Some(registration) = PROVIDER_REGISTRATION.lock().take()
        && let Some(registration) = NonNull::new(registration as *mut raw::FsProvider)
    {
        // SAFETY: module teardown owns this provider registration receipt.
        let _ = unsafe { ddk::fs_provider_unregister(registration) };
    }
    *NAMESPACE.lock() = None;
}

ddk::module!(
    b"devfs\0",
    b"Loadable device filesystem provider\0",
    devfs_init,
    devfs_exit,
);
