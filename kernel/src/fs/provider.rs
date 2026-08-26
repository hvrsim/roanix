//! Language-neutral filesystem-provider ABI and VFS adapters.
//!
//! Filesystem implementations are loadable providers.  The ABI deliberately
//! contains only C-layout records, opaque receipts, and function pointers; a
//! provider never receives a Rust VFS object.  A mounted adapter owns a module
//! lease and every live vnode retains that adapter, so callbacks never need to
//! pin a module on the hot path.

use alloc::{boxed::Box, collections::BTreeMap, sync::Arc, vec, vec::Vec};
use core::{
    any::Any,
    ffi::c_void,
    mem::{MaybeUninit, offset_of, size_of},
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};

use crate::{
    driver::{
        core::module::{self, Module, ModuleLease},
        error as driver_error,
        obj::{ObjHeader, ObjKind, ObjState},
    },
    mem::{IoSink, IoSource, ObjectKind, PageAccount, VmObject},
    sys::{
        event::Event,
        sync::{Mutex, Once},
    },
};

use super::{
    error::{Error, Result},
    file::OpenFlags,
    vnode::{
        CreateKind, DirEntry, FileSystem, FileSystemRef, FilesystemId, IoctlContext, NodeId,
        PollEvents, SetAttr, StatFs, TerminalState, Vnode, VnodeAttr, VnodeKind, VnodeOps,
        VnodeWeak,
    },
};

/// Current filesystem-provider operation-table size.
pub const FS_PROVIDER_OPS_SIZE: u32 = size_of::<FsProviderOps>() as u32;
/// Required prefix size for a filesystem-provider operation table.
///
/// Later callbacks are optional append-only extensions and are zero-filled
/// when an older provider declares only this prefix.
pub const FS_PROVIDER_REQUIRED_OPS_SIZE: u32 = offset_of!(FsProviderOps, initial_offset) as u32;
/// Current device-filesystem broker operation-table size.
pub const DEVFS_BROKER_OPS_SIZE: u32 = size_of::<DevfsBrokerOps>() as u32;
/// Required prefix size for a device-filesystem broker operation table.
pub const DEVFS_BROKER_REQUIRED_OPS_SIZE: u32 = 64;

/// Regular-file kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_REGULAR: u32 = 1;
/// Directory kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_DIRECTORY: u32 = 2;
/// Symbolic-link kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_SYMLINK: u32 = 3;
/// Character-device kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_CHARACTER_DEVICE: u32 = 4;
/// Block-device kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_BLOCK_DEVICE: u32 = 5;
/// FIFO kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_FIFO: u32 = 6;
/// Socket kind in [`FsVnode`] and [`FsCreate`].
pub const FS_KIND_SOCKET: u32 = 7;

/// [`FsSetAttr::valid`] bit for `size`.
pub const FS_SETATTR_SIZE: u32 = 1 << 0;
/// [`FsSetAttr::valid`] bit for `mode`.
pub const FS_SETATTR_MODE: u32 = 1 << 1;

/// Construct an ordinary regular file.
pub const FS_CREATE_REGULAR: u32 = 1;
/// Construct a directory.
pub const FS_CREATE_DIRECTORY: u32 = 2;
/// Construct a symbolic link.
pub const FS_CREATE_SYMLINK: u32 = 3;

/// Event selector for a readable device endpoint.
pub const DEVFS_EVENT_READABLE: u32 = 1;
/// Event selector for a writable device endpoint.
pub const DEVFS_EVENT_WRITABLE: u32 = 2;
/// Event selector for a hung-up device endpoint.
pub const DEVFS_EVENT_HANGUP: u32 = 3;

/// Options supplied while creating one mounted provider instance.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsMountOptions {
    /// Size of this record.
    pub size: u32,
    /// Reserved for append-only flags.
    pub flags: u32,
    /// Optional provider-specific page limit; zero selects the provider default.
    pub page_limit: u64,
}

/// Opaque provider vnode receipt plus stable filesystem-local identity.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsVnode {
    /// Provider-owned receipt.  It is consumed by `vnode_release`.
    pub receipt: *mut c_void,
    /// Stable identity within the mounted filesystem.
    pub node_id: u64,
    /// One of the `FS_KIND_*` constants.
    pub kind: u32,
    /// Reserved for append-only ABI evolution.
    pub reserved: u32,
}

impl FsVnode {
    const EMPTY: Self = Self {
        receipt: ptr::null_mut(),
        node_id: 0,
        kind: 0,
        reserved: 0,
    };
}

/// Vnode attributes exchanged with a provider.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsAttr {
    /// Logical byte size.
    pub size: u64,
    /// Number of directory links.
    pub links: u64,
    /// Monotonic access timestamp in nanoseconds.
    pub accessed_ns: u64,
    /// Monotonic content-modification timestamp in nanoseconds.
    pub modified_ns: u64,
    /// Monotonic metadata-change timestamp in nanoseconds.
    pub changed_ns: u64,
    /// Unix permission bits.
    pub mode: u16,
    /// One of the `FS_KIND_*` constants.
    pub kind: u32,
    /// Reserved for append-only ABI evolution.
    pub reserved: u16,
}

/// Optional attribute changes requested by VFS.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsSetAttr {
    /// Bits drawn from `FS_SETATTR_*`.
    pub valid: u32,
    /// Reserved for alignment and ABI evolution.
    pub reserved: u32,
    /// Requested logical size.
    pub size: u64,
    /// Requested permission bits.
    pub mode: u16,
    /// Reserved for append-only ABI evolution.
    pub reserved2: [u8; 6],
}

/// One buffer segment used by an atomic append operation.
///
/// The kernel resolves each source window before entering the provider.  The
/// provider may consume the segments directly without a bounce buffer.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsIoVec {
    /// First readable byte.
    pub data: *const u8,
    /// Number of readable bytes.
    pub length: usize,
}

/// One entry emitted by a batched directory read.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsDirEntry {
    /// Retained target vnode receipt.
    pub vnode: FsVnode,
    /// Number of bytes initialized in [`Self::name`].
    pub name_length: u16,
    /// Reserved for append-only ABI evolution.
    pub reserved: [u8; 6],
    /// Cursor value immediately after this entry.
    pub offset: u64,
    /// Inline entry name, which avoids retaining provider-owned pointers after
    /// the callback returns.
    pub name: [u8; FS_DIRECTORY_NAME_MAX],
}

impl FsDirEntry {
    const EMPTY: Self = Self {
        vnode: FsVnode::EMPTY,
        name_length: 0,
        reserved: [0; 6],
        offset: 0,
        name: [0; FS_DIRECTORY_NAME_MAX],
    };
}

/// Maximum byte length of one directory entry name in the provider ABI.
pub const FS_DIRECTORY_NAME_MAX: usize = 255;

/// Capacity snapshot returned by a filesystem provider.
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct FsStat {
    /// Total addressable bytes.
    pub total_bytes: u64,
    /// Currently committed bytes.
    pub used_bytes: u64,
    /// Maximum node count.
    pub total_nodes: u64,
    /// Current node count.
    pub used_nodes: u64,
}

/// Terminal state returned by a device vnode.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FsTerminalState {
    /// Session owning the controlling terminal.
    pub session: i32,
    /// Foreground process group.
    pub foreground_group: i32,
    /// Whether background output stops its process group.
    pub stop_background_output: u8,
    /// Reserved for append-only ABI evolution.
    pub reserved: [u8; 3],
}

/// One device node emitted by a devfs broker namespace listing.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DevfsBrokerEntry {
    /// Provider-assigned device-node identity.
    pub node: u64,
    /// Number of initialized bytes in [`Self::name`].
    pub name_length: u16,
    /// Reserved for append-only ABI evolution.
    pub reserved: [u8; 6],
    /// Inline device-node name.
    pub name: [u8; FS_DIRECTORY_NAME_MAX],
}

impl DevfsBrokerEntry {
    const EMPTY: Self = Self {
        node: 0,
        name_length: 0,
        reserved: [0; 6],
        name: [0; FS_DIRECTORY_NAME_MAX],
    };
}

/// C-compatible operation table for a filesystem provider.
///
/// All receipts are opaque to the kernel except for their ownership rule.
/// Successful callbacks that return an [`FsVnode`] transfer exactly one vnode
/// receipt to VFS.  VFS calls `vnode_release` exactly once if it accepts that
/// result, or immediately if validation rejects it.
#[repr(C)]
#[derive(Copy, Clone)]
#[allow(missing_docs)]
pub struct FsProviderOps {
    pub size: u32,
    pub context: *mut c_void,
    pub mount:
        Option<unsafe extern "C" fn(*mut c_void, *const FsMountOptions, *mut *mut c_void) -> i32>,
    pub unmount: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
    pub root: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut FsVnode) -> i32>,
    pub statfs: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut FsStat) -> i32>,
    pub sync: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
    pub vnode_release: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void)>,
    pub getattr:
        Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut FsAttr) -> i32>,
    pub setattr: Option<
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const FsSetAttr) -> i32,
    >,
    pub lookup: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            *const u8,
            usize,
            *mut FsVnode,
        ) -> i32,
    >,
    pub parent:
        Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut FsVnode) -> i32>,
    pub create: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            *const u8,
            usize,
            u32,
            *const u8,
            usize,
            u16,
            *mut FsVnode,
        ) -> i32,
    >,
    pub link: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            *const u8,
            usize,
            *mut c_void,
        ) -> i32,
    >,
    pub unlink: Option<
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const u8, usize, u8) -> i32,
    >,
    pub rename: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            *const u8,
            usize,
            *mut c_void,
            *const u8,
            usize,
        ) -> i32,
    >,
    pub open:
        Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, u32, *mut usize) -> i32>,
    pub close: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, usize, u32)>,
    pub initial_offset: Option<
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, usize, u32, *mut u64) -> i32,
    >,
    pub read: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            usize,
            u64,
            *mut u8,
            usize,
            u32,
        ) -> i64,
    >,
    pub write: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            usize,
            u64,
            *const u8,
            usize,
            u32,
        ) -> i64,
    >,
    pub append: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            *const FsIoVec,
            usize,
            *mut u64,
        ) -> i64,
    >,
    pub truncate: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, u64) -> i32>,
    pub memory_object: Option<
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut *mut c_void) -> i32,
    >,
    pub readlink: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            *mut u8,
            usize,
            *mut usize,
        ) -> i32,
    >,
    pub readdir: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            u64,
            *mut FsDirEntry,
            usize,
            *mut usize,
            *mut u64,
        ) -> i32,
    >,
    pub fsync: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void) -> i32>,
    pub poll: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            usize,
            u64,
            u16,
            u32,
            *mut u16,
        ) -> i32,
    >,
    pub poll_events: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            usize,
            u16,
            *mut usize,
            usize,
            *mut usize,
        ) -> i32,
    >,
    pub terminal_state: Option<
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut FsTerminalState) -> i32,
    >,
    #[allow(clippy::type_complexity)]
    pub ioctl: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *mut c_void,
            *mut c_void,
            usize,
            u64,
            i32,
            i32,
            u8,
            u64,
            u64,
            *mut u8,
            usize,
            *mut u64,
        ) -> i32,
    >,
}

/// C-compatible control table published by the global devfs provider.
///
/// Hardware-driver callbacks remain owned by a kernel endpoint receipt.  The
/// devfs module stores that receipt and reaches its operations through the
/// append-only API services, while the endpoint holds the hardware module
/// lease.
#[repr(C)]
#[derive(Copy, Clone)]
#[allow(missing_docs)]
pub struct DevfsBrokerOps {
    pub size: u32,
    pub context: *mut c_void,
    pub root: Option<unsafe extern "C" fn(*mut c_void, *mut u64) -> i32>,
    pub mkdir:
        Option<unsafe extern "C" fn(*mut c_void, u64, u64, *const u8, usize, u16, *mut u64) -> i32>,
    pub create: Option<
        unsafe extern "C" fn(
            *mut c_void,
            u64,
            u64,
            *const u8,
            usize,
            u32,
            u16,
            *mut c_void,
            *mut u64,
        ) -> i32,
    >,
    pub remove: Option<unsafe extern "C" fn(*mut c_void, u64, u64) -> i32>,
    pub remove_owner: Option<unsafe extern "C" fn(*mut c_void, u64, u8) -> i32>,
    pub lookup: Option<unsafe extern "C" fn(*mut c_void, *const u8, usize, *mut u64) -> i32>,
    pub children: Option<
        unsafe extern "C" fn(*mut c_void, u64, *mut DevfsBrokerEntry, usize, *mut usize) -> i32,
    >,
}

// SAFETY: the immutable callback table is synchronized by the module that
// registers it; C has no auto trait equivalent for that ABI contract.
unsafe impl Send for FsProviderOps {}
// SAFETY: the registering module guarantees synchronized callback access.
unsafe impl Sync for FsProviderOps {}
// SAFETY: the registering module guarantees synchronized callback access.
unsafe impl Send for DevfsBrokerOps {}
// SAFETY: the registering module guarantees synchronized callback access.
unsafe impl Sync for DevfsBrokerOps {}

pub(crate) struct Provider {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    operations: FsProviderOps,
    registered: AtomicBool,
}

// SAFETY: the ABI requires callback tables to support the concurrency their
// provider declares.  The table itself is copied and immutable.
unsafe impl Send for Provider {}
// SAFETY: as above.
unsafe impl Sync for Provider {}

struct ProviderRegistry {
    providers: Mutex<BTreeMap<Box<str>, Arc<Provider>>>,
}

static PROVIDERS: Once<ProviderRegistry> = Once::new();

/// Initializes the provider registry before early modules are loaded.
pub(crate) fn init() {
    PROVIDERS.call_once(|| ProviderRegistry {
        providers: Mutex::new(BTreeMap::new()),
    });
}

fn registry() -> core::result::Result<&'static ProviderRegistry, crate::driver::Error> {
    PROVIDERS.get().ok_or(crate::driver::Error::NotInitialized)
}

/// Registers a filesystem provider and returns an opaque registration receipt.
///
/// # Safety
/// `operations` must identify a readable size-prefixed C table whose callback
/// targets and context remain live until the registration is removed.
pub(crate) unsafe fn register(
    owner: Option<&Arc<Module>>,
    name: &str,
    operations: *const FsProviderOps,
) -> crate::driver::Result<Arc<Provider>> {
    if name.is_empty() || name.len() > 64 {
        return Err(crate::driver::Error::InvalidArgument);
    }
    // SAFETY: forwarded from this function's contract.  The helper copies only
    // the caller-declared prefix after reading its mandatory size field.
    let operations = unsafe { copy_provider_ops(operations) }?;
    if operations.mount.is_none()
        || operations.unmount.is_none()
        || operations.root.is_none()
        || operations.vnode_release.is_none()
        || operations.getattr.is_none()
        || operations.lookup.is_none()
        || operations.parent.is_none()
        || operations.open.is_none()
        || operations.close.is_none()
    {
        return Err(crate::driver::Error::InvalidArgument);
    }

    let registry = registry()?;
    let mut providers = registry.providers.lock();
    if providers.contains_key(name) {
        return Err(crate::driver::Error::AlreadyExists);
    }
    let header = ObjHeader::new_with(
        ObjKind::Filesystem,
        Some(name),
        owner.map(|module| module.id().get()),
        None,
    );
    header.register()?;
    let provider = Arc::new(Provider {
        header,
        name: name.into(),
        owner: owner.cloned(),
        operations,
        registered: AtomicBool::new(true),
    });
    providers.insert(provider.name.clone(), provider.clone());
    Ok(provider)
}

/// Removes a provider registration returned by [`register`].
pub(crate) fn unregister(provider: &Arc<Provider>) -> crate::driver::Result<()> {
    let registry = registry()?;
    let mut providers = registry.providers.lock();
    let Some(current) = providers.get(provider.name.as_ref()) else {
        return Err(crate::driver::Error::NotFound);
    };
    if !Arc::ptr_eq(current, provider) {
        return Err(crate::driver::Error::InvalidArgument);
    }
    providers.remove(provider.name.as_ref());
    provider.registered.store(false, Ordering::Release);
    provider.header.set_state(ObjState::Removing);
    provider.header.poison();
    Ok(())
}

/// Removes registrations owned by a module while its teardown is in progress.
pub(crate) fn remove_module_providers(module: &Arc<Module>) {
    let Ok(registry) = registry() else {
        return;
    };
    let mut providers = registry.providers.lock();
    providers.retain(|_, provider| {
        let owned = provider
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module));
        if owned {
            provider.registered.store(false, Ordering::Release);
            provider.header.set_state(ObjState::Removing);
            provider.header.poison();
        }
        !owned
    });
}

/// Mounts a registered provider by name.
pub(crate) fn mount_named(name: &str, options: FsMountOptions) -> Result<FileSystemRef> {
    let provider = {
        let registry = registry().map_err(|_| Error::Io)?;
        registry
            .providers
            .lock()
            .get(name)
            .cloned()
            .filter(|provider| provider.registered.load(Ordering::Acquire))
            .ok_or(Error::NotFound)?
    };
    let lease = module::lease_owner(provider.owner.as_ref()).map_err(|_| Error::Io)?;
    let mount_callback = provider.operations.mount.expect("validated provider mount");
    let mut receipt = ptr::null_mut();
    // SAFETY: `provider` copied a valid ABI table, the lease keeps callback
    // code and context resident, and both local output pointers are valid.
    let status = unsafe {
        mount_callback(
            provider.operations.context,
            &raw const options,
            &raw mut receipt,
        )
    };
    if status < 0 || receipt.is_null() {
        return Err(if status < 0 {
            fs_status(status)
        } else {
            Error::Io
        });
    }
    let mount = Arc::new(ProviderMount {
        header: ObjHeader::new_with(
            ObjKind::Mount,
            Some(&provider.name),
            provider.owner.as_ref().map(|owner| owner.id().get()),
            Some(provider.header.id()),
        ),
        provider,
        receipt,
        vnodes: Mutex::new(BTreeMap::new()),
        _lease: lease,
    });
    if mount.header.register().is_err() {
        drop(mount);
        return Err(Error::Io);
    }
    let id = FilesystemId::allocate();
    Ok(Arc::new(ProviderFilesystem { id, mount }))
}

struct ProviderMount {
    header: ObjHeader,
    provider: Arc<Provider>,
    receipt: *mut c_void,
    vnodes: Mutex<BTreeMap<NodeId, VnodeWeak>>,
    _lease: ModuleLease,
}

// SAFETY: provider concurrency is part of its ABI contract.  The mount receipt
// is only passed to that provider while its lease is held.
unsafe impl Send for ProviderMount {}
// SAFETY: as above.
unsafe impl Sync for ProviderMount {}

impl ProviderMount {
    fn release_vnode(&self, receipt: *mut c_void) {
        if receipt.is_null() {
            return;
        }
        let callback = self
            .provider
            .operations
            .vnode_release
            .expect("validated vnode release");
        // SAFETY: the receipt was returned by this provider and remains owned
        // by this adapter until this call consumes it.
        unsafe { callback(self.provider.operations.context, self.receipt, receipt) };
    }

    fn root(self: &Arc<Self>, filesystem: FilesystemId) -> Result<Vnode> {
        let callback = self
            .provider
            .operations
            .root
            .expect("validated provider root");
        let mut raw = FsVnode::EMPTY;
        // SAFETY: mount and output receipt belong to this live adapter.
        let status =
            unsafe { callback(self.provider.operations.context, self.receipt, &raw mut raw) };
        if status < 0 {
            return Err(fs_status(status));
        }
        self.vnode_from(filesystem, raw)
    }

    fn vnode_from(self: &Arc<Self>, filesystem: FilesystemId, raw: FsVnode) -> Result<Vnode> {
        let Some(kind) = vnode_kind(raw.kind) else {
            self.release_vnode(raw.receipt);
            return Err(Error::InvalidArgument);
        };
        if raw.receipt.is_null() || raw.node_id == 0 {
            self.release_vnode(raw.receipt);
            return Err(Error::Io);
        }
        let node = NodeId::new(raw.node_id);
        let mut vnodes = self.vnodes.lock();
        if let Some(existing) = vnodes.get(&node).and_then(VnodeWeak::upgrade) {
            drop(vnodes);
            self.release_vnode(raw.receipt);
            return if existing.kind() == kind {
                Ok(existing)
            } else {
                Err(Error::Io)
            };
        }
        let header = ObjHeader::new_with(
            ObjKind::Vnode,
            None,
            self.provider.owner.as_ref().map(|owner| owner.id().get()),
            Some(self.header.id()),
        );
        if header.register().is_err() {
            drop(vnodes);
            self.release_vnode(raw.receipt);
            return Err(Error::Io);
        }
        let adapter = ProviderVnode {
            header,
            mount: self.clone(),
            receipt: raw.receipt,
            node,
        };
        let vnode = Vnode::new(
            super::vnode::VnodeKey { filesystem, node },
            kind,
            Box::new(adapter),
        );
        vnodes.insert(node, vnode.downgrade());
        Ok(vnode)
    }

    fn discard_vnode(&self, node: NodeId) {
        let mut vnodes = self.vnodes.lock();
        if vnodes
            .get(&node)
            .is_some_and(|vnode| vnode.upgrade().is_none())
        {
            vnodes.remove(&node);
        }
    }
}

impl Drop for ProviderMount {
    fn drop(&mut self) {
        self.header.set_state(ObjState::Removing);
        let callback = self
            .provider
            .operations
            .unmount
            .expect("validated provider unmount");
        // SAFETY: this adapter owns the mount receipt and its lease keeps the
        // provider callback executable until the receipt is consumed.
        let _ = unsafe { callback(self.provider.operations.context, self.receipt) };
        self.header.poison();
    }
}

struct ProviderFilesystem {
    id: FilesystemId,
    mount: Arc<ProviderMount>,
}

impl FileSystem for ProviderFilesystem {
    fn id(&self) -> FilesystemId {
        self.id
    }

    fn name(&self) -> &str {
        &self.mount.provider.name
    }

    fn root(&self) -> Vnode {
        self.mount
            .root(self.id)
            .expect("filesystem provider returned no root vnode")
    }

    fn statfs(&self) -> StatFs {
        let Some(callback) = self.mount.provider.operations.statfs else {
            return StatFs::default();
        };
        let mut stat = FsStat::default();
        // SAFETY: the mount lease keeps the callback and context valid; `stat`
        // is writable local output storage.
        if unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                &raw mut stat,
            )
        } < 0
        {
            return StatFs::default();
        }
        StatFs {
            total_bytes: stat.total_bytes,
            used_bytes: stat.used_bytes,
            total_nodes: stat.total_nodes,
            used_nodes: stat.used_nodes,
        }
    }

    fn sync(&self) -> Result<()> {
        let Some(callback) = self.mount.provider.operations.sync else {
            return Ok(());
        };
        // SAFETY: mount receipt and provider callback are retained by this
        // filesystem object for the whole call.
        status(unsafe { callback(self.mount.provider.operations.context, self.mount.receipt) })
    }
}

struct ProviderVnode {
    header: ObjHeader,
    mount: Arc<ProviderMount>,
    receipt: *mut c_void,
    node: NodeId,
}

// SAFETY: the provider defines vnode callback concurrency.  The receipt is
// immutable from the adapter's perspective and survives until `Drop`.
unsafe impl Send for ProviderVnode {}
// SAFETY: as above.
unsafe impl Sync for ProviderVnode {}

impl ProviderVnode {
    fn callback<T>(&self, callback: Option<T>) -> Result<T> {
        callback.ok_or(Error::Unsupported)
    }

    fn raw_target(&self, vnode: &Vnode) -> Result<*mut c_void> {
        let target = vnode
            .operations_as::<ProviderVnode>()
            .ok_or(Error::CrossDevice)?;
        if !Arc::ptr_eq(&self.mount, &target.mount) {
            return Err(Error::CrossDevice);
        }
        Ok(target.receipt)
    }

    fn vnode_from_for(&self, vnode: &Vnode, raw: FsVnode) -> Result<Vnode> {
        self.mount.vnode_from(vnode.key().filesystem, raw)
    }

    fn read_or_write(
        &self,
        file_context: usize,
        offset: u64,
        mut sink: Option<&mut IoSink<'_>>,
        source: Option<&IoSource<'_>>,
        flags: u32,
    ) -> Result<usize> {
        let total = sink
            .as_ref()
            .map_or_else(|| source.map_or(0, IoSource::len), |sink| sink.len());
        let mut done = 0usize;
        while done < total {
            let window_flags = if done == 0 {
                flags
            } else {
                flags | OpenFlags::NONBLOCK.bits()
            };
            let value = if let Some(sink) = sink.as_deref_mut() {
                let mut window = sink
                    .window(done, total - done)
                    .map_err(|_| Error::InvalidArgument)?;
                if window.is_empty() {
                    break;
                }
                let capacity = window.len();
                let callback = self.callback(self.mount.provider.operations.read)?;
                // SAFETY: the window is writable for exactly `capacity` bytes
                // and the provider lease keeps its callback resident.
                let value = unsafe {
                    callback(
                        self.mount.provider.operations.context,
                        self.mount.receipt,
                        self.receipt,
                        file_context,
                        offset.saturating_add(done as u64),
                        window.as_mut_ptr(),
                        capacity,
                        window_flags,
                    )
                };
                if value >= 0
                    && usize::try_from(value)
                        .ok()
                        .is_some_and(|count| count > capacity)
                {
                    return Err(Error::Io);
                }
                value
            } else {
                let source = source.expect("one transfer source is present");
                let window = source
                    .window(done, total - done)
                    .map_err(|_| Error::InvalidArgument)?;
                if window.is_empty() {
                    break;
                }
                let capacity = window.len();
                let callback = self.callback(self.mount.provider.operations.write)?;
                // SAFETY: the source window is readable for exactly `capacity`
                // bytes and the provider lease keeps its callback resident.
                let value = unsafe {
                    callback(
                        self.mount.provider.operations.context,
                        self.mount.receipt,
                        self.receipt,
                        file_context,
                        offset.saturating_add(done as u64),
                        window.as_ptr(),
                        capacity,
                        window_flags,
                    )
                };
                if value >= 0
                    && usize::try_from(value)
                        .ok()
                        .is_some_and(|count| count > capacity)
                {
                    return Err(Error::Io);
                }
                value
            };
            if value < 0 {
                if done != 0 {
                    break;
                }
                return Err(fs_status(value as i32));
            }
            let count = usize::try_from(value).map_err(|_| Error::Io)?;
            done = done.saturating_add(count);
            let capacity = total - done.saturating_sub(count);
            if count < capacity {
                break;
            }
        }
        Ok(done)
    }
}

impl Drop for ProviderVnode {
    fn drop(&mut self) {
        self.header.set_state(ObjState::Removing);
        self.mount.discard_vnode(self.node);
        self.mount.release_vnode(self.receipt);
        self.header.poison();
    }
}

impl VnodeOps for ProviderVnode {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn initial_offset(&self, vnode: &Vnode, file_context: usize, flags: u32) -> Result<u64> {
        let Some(callback) = self.mount.provider.operations.initial_offset else {
            return Ok(0);
        };
        let mut offset = 0;
        // SAFETY: callback, mount, vnode receipt, and output storage remain
        // valid for this immediate invocation.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                file_context,
                flags,
                &raw mut offset,
            )
        })?;
        let _ = vnode;
        Ok(offset)
    }

    fn open(&self, _vnode: &Vnode, flags: u32) -> Result<usize> {
        let callback = self.callback(self.mount.provider.operations.open)?;
        let mut context = 0;
        // SAFETY: callback lifetime is guarded by `mount`; context is writable
        // local storage.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                flags,
                &raw mut context,
            )
        })?;
        Ok(context)
    }

    fn close(&self, _vnode: &Vnode, file_context: usize, flags: u32) {
        if let Some(callback) = self.mount.provider.operations.close {
            // SAFETY: the context was returned by `open` on this provider
            // receipt, and the vnode-held mount lease keeps code resident.
            unsafe {
                callback(
                    self.mount.provider.operations.context,
                    self.mount.receipt,
                    self.receipt,
                    file_context,
                    flags,
                )
            };
        }
    }

    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        let callback = self.callback(self.mount.provider.operations.getattr)?;
        let mut attr = MaybeUninit::<FsAttr>::zeroed();
        // SAFETY: `attr` is writable output storage and the adapter owns both
        // opaque receipts for the duration of the callback.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                attr.as_mut_ptr(),
            )
        })?;
        // SAFETY: a successful provider callback initializes every scalar
        // field of the fixed C record.
        let attr = unsafe { attr.assume_init() };
        if vnode_kind(attr.kind) != Some(vnode.kind()) {
            return Err(Error::Io);
        }
        Ok(VnodeAttr {
            key: vnode.key(),
            kind: vnode.kind(),
            size: attr.size,
            links: attr.links,
            mode: attr.mode,
            accessed_ns: attr.accessed_ns,
            modified_ns: attr.modified_ns,
            changed_ns: attr.changed_ns,
        })
    }

    fn setattr(&self, _vnode: &Vnode, attr: SetAttr) -> Result<()> {
        let callback = self.callback(self.mount.provider.operations.setattr)?;
        let mut raw = FsSetAttr {
            valid: 0,
            reserved: 0,
            size: 0,
            mode: 0,
            reserved2: [0; 6],
        };
        if let Some(size) = attr.size {
            raw.valid |= FS_SETATTR_SIZE;
            raw.size = size;
        }
        if let Some(mode) = attr.mode {
            raw.valid |= FS_SETATTR_MODE;
            raw.mode = mode;
        }
        // SAFETY: `raw` is a fully initialized C record valid for this call.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                &raw const raw,
            )
        })
    }

    fn lookup(&self, directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        let callback = self.callback(self.mount.provider.operations.lookup)?;
        let mut raw = FsVnode::EMPTY;
        // SAFETY: `name` and `raw` are valid for the callback's duration.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                name.as_ptr(),
                name.len(),
                &raw mut raw,
            )
        })?;
        self.vnode_from_for(directory, raw)
    }

    fn parent(&self, directory: &Vnode) -> Result<Vnode> {
        let callback = self.callback(self.mount.provider.operations.parent)?;
        let mut raw = FsVnode::EMPTY;
        // SAFETY: opaque inputs and local output storage are valid.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                &raw mut raw,
            )
        })?;
        self.vnode_from_for(directory, raw)
    }

    fn create(&self, directory: &Vnode, name: &[u8], kind: CreateKind, mode: u16) -> Result<Vnode> {
        let callback = self.callback(self.mount.provider.operations.create)?;
        let (kind, target): (u32, &[u8]) = match &kind {
            CreateKind::Regular => (FS_CREATE_REGULAR, &[]),
            CreateKind::Directory => (FS_CREATE_DIRECTORY, &[]),
            CreateKind::Symlink(target) => (FS_CREATE_SYMLINK, target),
        };
        let mut raw = FsVnode::EMPTY;
        // SAFETY: all buffers and output storage remain valid for this call.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                name.as_ptr(),
                name.len(),
                kind,
                target.as_ptr(),
                target.len(),
                mode,
                &raw mut raw,
            )
        })?;
        self.vnode_from_for(directory, raw)
    }

    fn link(&self, _directory: &Vnode, name: &[u8], target: &Vnode) -> Result<()> {
        let callback = self.callback(self.mount.provider.operations.link)?;
        let target = self.raw_target(target)?;
        // SAFETY: the target receipt belongs to the same retained mount.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                name.as_ptr(),
                name.len(),
                target,
            )
        })
    }

    fn unlink(&self, _directory: &Vnode, name: &[u8], remove_directory: bool) -> Result<()> {
        let callback = self.callback(self.mount.provider.operations.unlink)?;
        // SAFETY: the name bytes are valid during this immediate callback.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                name.as_ptr(),
                name.len(),
                u8::from(remove_directory),
            )
        })
    }

    fn rename(
        &self,
        _source_directory: &Vnode,
        source_name: &[u8],
        target_directory: &Vnode,
        target_name: &[u8],
    ) -> Result<()> {
        let callback = self.callback(self.mount.provider.operations.rename)?;
        let target = self.raw_target(target_directory)?;
        // SAFETY: both names and receipts remain valid during this callback.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                source_name.as_ptr(),
                source_name.len(),
                target,
                target_name.as_ptr(),
                target_name.len(),
            )
        })
    }

    fn read_at(&self, _vnode: &Vnode, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        self.read_or_write(0, offset, Some(sink), None, 0)
    }

    fn read_at_with_flags(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        offset: u64,
        sink: &mut IoSink<'_>,
        flags: u32,
    ) -> Result<usize> {
        self.read_or_write(file_context, offset, Some(sink), None, flags)
    }

    fn write_at(&self, _vnode: &Vnode, offset: u64, source: &IoSource<'_>) -> Result<usize> {
        self.read_or_write(0, offset, None, Some(source), 0)
    }

    fn write_at_with_flags(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        offset: u64,
        source: &IoSource<'_>,
        flags: u32,
    ) -> Result<usize> {
        self.read_or_write(file_context, offset, None, Some(source), flags)
    }

    fn append(&self, _vnode: &Vnode, source: &IoSource<'_>) -> Result<(usize, u64)> {
        let callback = self.callback(self.mount.provider.operations.append)?;
        // The window guards are kept alive until the callback returns so the
        // frames behind every vector stay wired for the whole call.
        let mut windows = Vec::new();
        let mut offset = 0usize;
        while offset < source.len() {
            let window = source
                .window(offset, source.len() - offset)
                .map_err(|_| Error::InvalidArgument)?;
            if window.is_empty() {
                return Err(Error::InvalidArgument);
            }
            offset = offset.saturating_add(window.len());
            windows.push(window);
        }
        let vectors: Vec<FsIoVec> = windows
            .iter()
            .map(|window| FsIoVec {
                data: window.as_ptr(),
                length: window.len(),
            })
            .collect();
        let mut next = 0;
        // SAFETY: vector metadata points directly at pinned source windows; no
        // whole-request bounce buffer is made.
        let result = unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                vectors.as_ptr(),
                vectors.len(),
                &raw mut next,
            )
        };
        if result < 0 {
            return Err(fs_status(result as i32));
        }
        let written = usize::try_from(result).map_err(|_| Error::Io)?;
        if written > source.len() {
            return Err(Error::Io);
        }
        Ok((written, next))
    }

    fn truncate(&self, _vnode: &Vnode, size: u64) -> Result<()> {
        let callback = self.callback(self.mount.provider.operations.truncate)?;
        // SAFETY: mount and vnode receipts remain valid for this call.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                size,
            )
        })
    }

    fn memory_object(&self, _vnode: &Vnode) -> Result<Arc<VmObject>> {
        let callback = self.callback(self.mount.provider.operations.memory_object)?;
        let mut object = ptr::null_mut();
        // SAFETY: successful callback transfers one retained kernel object
        // receipt into `object`.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                &raw mut object,
            )
        })?;
        // SAFETY: providers can only receive this opaque receipt from the
        // kernel object services and success transfers its owned reference.
        unsafe { take_memory_object(object) }
    }

    fn readlink(&self, vnode: &Vnode) -> Result<Box<[u8]>> {
        let callback = self.callback(self.mount.provider.operations.readlink)?;
        let size = usize::try_from(self.getattr(vnode)?.size).map_err(|_| Error::FileTooLarge)?;
        let mut target = vec![0; size];
        let mut written = 0;
        // SAFETY: the vector is writable for exactly its capacity.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                target.as_mut_ptr(),
                target.len(),
                &raw mut written,
            )
        })?;
        if written != target.len() {
            return Err(Error::Io);
        }
        Ok(target.into_boxed_slice())
    }

    fn readdir(
        &self,
        directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        let callback = self.callback(self.mount.provider.operations.readdir)?;
        if maximum == 0 {
            return Ok((Vec::new(), cursor));
        }
        let mut raw_entries = vec![FsDirEntry::EMPTY; maximum];
        let mut count = 0usize;
        let mut next = cursor;
        // SAFETY: output storage is initialized and writable for `maximum`
        // entries; names returned by the provider are copied before return.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                cursor,
                raw_entries.as_mut_ptr(),
                raw_entries.len(),
                &raw mut count,
                &raw mut next,
            )
        })?;
        if count > raw_entries.len() {
            return Err(Error::Io);
        }
        let mut entries = Vec::with_capacity(count);
        for entry in raw_entries.into_iter().take(count) {
            let name_length = usize::from(entry.name_length);
            if name_length > entry.name.len() {
                self.mount.release_vnode(entry.vnode.receipt);
                return Err(Error::Io);
            }
            let name = &entry.name[..name_length];
            let vnode = self.vnode_from_for(directory, entry.vnode)?;
            entries.push(DirEntry {
                name: Arc::from(name),
                key: vnode.key(),
                kind: vnode.kind(),
                offset: entry.offset,
            });
        }
        Ok((entries, next))
    }

    fn fsync(&self, _vnode: &Vnode) -> Result<()> {
        let Some(callback) = self.mount.provider.operations.fsync else {
            return Ok(());
        };
        // SAFETY: mount and vnode receipts remain valid for this call.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
            )
        })
    }

    fn poll(
        &self,
        _vnode: &Vnode,
        file_context: usize,
        offset: u64,
        events: PollEvents,
        flags: u32,
    ) -> Result<PollEvents> {
        let Some(callback) = self.mount.provider.operations.poll else {
            return Ok(PollEvents::empty());
        };
        let mut ready = 0;
        // SAFETY: output storage is valid and opaque receipts are held.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                file_context,
                offset,
                events.bits(),
                flags,
                &raw mut ready,
            )
        })?;
        Ok(PollEvents::from_bits_truncate(ready))
    }

    fn poll_events<'a>(
        &'a self,
        _vnode: &Vnode,
        file_context: usize,
        events: PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        let Some(callback) = self.mount.provider.operations.poll_events else {
            return false;
        };
        let mut receipts = [0usize; 3];
        let mut count = 0usize;
        // SAFETY: output array lives for the whole immediate callback.
        let status = unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                file_context,
                events.bits(),
                receipts.as_mut_ptr(),
                receipts.len(),
                &raw mut count,
            )
        };
        if status < 0 || count > receipts.len() {
            return false;
        }
        let mut found = false;
        for receipt in receipts.into_iter().take(count) {
            if let Some(event) = crate::driver::abi::events::resolve(receipt) {
                output.push(event);
                found = true;
            }
        }
        found
    }

    fn terminal_state(&self, _vnode: &Vnode) -> Option<TerminalState> {
        let callback = self.mount.provider.operations.terminal_state?;
        let mut state = FsTerminalState {
            session: 0,
            foreground_group: 0,
            stop_background_output: 0,
            reserved: [0; 3],
        };
        // SAFETY: output storage and retained receipts are valid for the call.
        if unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                &raw mut state,
            )
        } < 0
        {
            return None;
        }
        Some(TerminalState {
            session: state.session,
            foreground_group: state.foreground_group,
            stop_background_output: state.stop_background_output != 0,
        })
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
        let callback = self.callback(self.mount.provider.operations.ioctl)?;
        let mut result = 0;
        // SAFETY: argument buffer and scalar identity values are valid for the
        // duration of this direct C ABI call.
        status(unsafe {
            callback(
                self.mount.provider.operations.context,
                self.mount.receipt,
                self.receipt,
                file_context,
                context.process_id as u64,
                context.process_group,
                context.session_id,
                u8::from(context.is_session_leader),
                request,
                value,
                argument.as_mut_ptr(),
                argument.len(),
                &raw mut result,
            )
        })?;
        Ok(result)
    }
}

fn vnode_kind(kind: u32) -> Option<VnodeKind> {
    Some(match kind {
        FS_KIND_REGULAR => VnodeKind::Regular,
        FS_KIND_DIRECTORY => VnodeKind::Directory,
        FS_KIND_SYMLINK => VnodeKind::Symlink,
        FS_KIND_CHARACTER_DEVICE => VnodeKind::CharacterDevice,
        FS_KIND_BLOCK_DEVICE => VnodeKind::BlockDevice,
        FS_KIND_FIFO => VnodeKind::Fifo,
        FS_KIND_SOCKET => VnodeKind::Socket,
        _ => return None,
    })
}

fn fs_status(status: i32) -> Error {
    driver_error::Error::from_status(status).into()
}

fn status(status: i32) -> Result<()> {
    if status < 0 {
        Err(fs_status(status))
    } else {
        Ok(())
    }
}

/// Copies a provider operation table without reading beyond an older prefix.
///
/// # Safety
/// `source` must address at least its size field and the declared readable
/// bytes of the table.
unsafe fn copy_provider_ops(source: *const FsProviderOps) -> crate::driver::Result<FsProviderOps> {
    if source.is_null() {
        return Err(crate::driver::Error::InvalidArgument);
    }
    // SAFETY: the caller guarantees the mandatory size prefix is readable.
    let declared = unsafe { (*source).size as usize };
    if declared < FS_PROVIDER_REQUIRED_OPS_SIZE as usize {
        return Err(crate::driver::Error::Unsupported);
    }
    let copied = declared.min(size_of::<FsProviderOps>());
    let mut destination = MaybeUninit::<FsProviderOps>::zeroed();
    // SAFETY: both ranges are valid for `copied` bytes by the ABI contract.
    unsafe {
        ptr::copy_nonoverlapping(
            source.cast::<u8>(),
            destination.as_mut_ptr().cast::<u8>(),
            copied,
        );
        Ok(destination.assume_init())
    }
}

// --- Page-cache services ---------------------------------------------------

/// Creates a page commitment account owned by the caller.
pub fn page_account_create(limit: u64) -> *mut c_void {
    Arc::into_raw(PageAccount::new(limit)).cast_mut().cast()
}

/// Drops one page-account receipt.
///
/// # Safety
/// `receipt` must be an owned value returned by [`page_account_create`].
pub unsafe fn page_account_release(receipt: *mut c_void) {
    if !receipt.is_null() {
        // SAFETY: forwarded from this function's safety contract.
        drop(unsafe { Arc::from_raw(receipt.cast::<PageAccount>()) });
    }
}

/// Returns the account's page limit.
///
/// # Safety
/// `receipt` must be a live page-account receipt.
pub unsafe fn page_account_limit(receipt: *mut c_void) -> Result<u64> {
    // SAFETY: forwarded from this function's safety contract; ManuallyDrop
    // keeps the foreign receipt's owned reference intact.
    let account = unsafe { borrow_arc::<PageAccount>(receipt) }?;
    Ok(account.limit().unwrap_or(u64::MAX))
}

/// Returns the account's committed-page count.
///
/// # Safety
/// `receipt` must be a live page-account receipt.
pub unsafe fn page_account_used(receipt: *mut c_void) -> Result<u64> {
    // SAFETY: forwarded from this function's safety contract.
    let account = unsafe { borrow_arc::<PageAccount>(receipt) }?;
    Ok(account.used())
}

/// Creates a page-cache object charged to a page account.
///
/// # Safety
/// `account` must be a live page-account receipt.
pub unsafe fn memory_object_create(account: *mut c_void) -> Result<*mut c_void> {
    // SAFETY: forwarded from this function's safety contract.
    let account = unsafe { borrow_arc::<PageAccount>(account) }?;
    Ok(Arc::into_raw(VmObject::with_page_account(
        ObjectKind::Vnode,
        Arc::clone(&account),
    ))
    .cast_mut()
    .cast())
}

/// Retains one memory-object receipt.
///
/// # Safety
/// `receipt` must be a live object receipt returned by
/// [`memory_object_create`].
pub unsafe fn memory_object_retain(receipt: *mut c_void) -> Result<()> {
    if receipt.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the receipt owns a live Arc allocation by this function's
    // contract, so incrementing its strong count creates a new owned receipt.
    unsafe { Arc::increment_strong_count(receipt.cast::<VmObject>()) };
    Ok(())
}

/// Drops one memory-object receipt.
///
/// # Safety
/// `receipt` must be an owned object receipt.
pub unsafe fn memory_object_release(receipt: *mut c_void) {
    if !receipt.is_null() {
        // SAFETY: forwarded from this function's safety contract.
        drop(unsafe { Arc::from_raw(receipt.cast::<VmObject>()) });
    }
}

/// Reads directly into a provider buffer without a whole-request copy.
///
/// # Safety
/// `receipt` must be live and `buffer` must be writable for `length` bytes.
pub unsafe fn memory_object_read(
    receipt: *mut c_void,
    offset: u64,
    buffer: *mut u8,
    length: usize,
) -> Result<usize> {
    // SAFETY: forwarded from this function's safety contract.
    let object = unsafe { borrow_arc::<VmObject>(receipt) }?;
    let bytes = mutable_bytes(buffer, length)?;
    object.read_at(offset, bytes).map_err(mem_error)
}

/// Writes directly from a provider buffer without a whole-request copy.
///
/// # Safety
/// `receipt` must be live and `buffer` must be readable for `length` bytes.
pub unsafe fn memory_object_write(
    receipt: *mut c_void,
    offset: u64,
    buffer: *const u8,
    length: usize,
) -> Result<usize> {
    // SAFETY: forwarded from this function's safety contract.
    let object = unsafe { borrow_arc::<VmObject>(receipt) }?;
    let bytes = immutable_bytes(buffer, length)?;
    object.write_at(offset, bytes).map_err(mem_error)
}

/// Truncates a page-cache object and releases pages beyond `size`.
///
/// # Safety
/// `receipt` must be a live memory-object receipt.
pub unsafe fn memory_object_truncate(receipt: *mut c_void, size: u64) -> Result<u64> {
    // SAFETY: forwarded from this function's safety contract.
    let object = unsafe { borrow_arc::<VmObject>(receipt) }?;
    object.truncate(size).map_err(mem_error)
}

/// Returns the number of committed pages in an object.
///
/// # Safety
/// `receipt` must be a live memory-object receipt.
pub unsafe fn memory_object_page_count(receipt: *mut c_void) -> Result<u64> {
    // SAFETY: forwarded from this function's safety contract.
    let object = unsafe { borrow_arc::<VmObject>(receipt) }?;
    Ok(object.page_count())
}

/// Takes an owned memory-object receipt into a kernel `Arc`.
///
/// # Safety
/// `receipt` must be an owned result from a provider's `memory_object`
/// callback.  That callback transfers ownership to the caller.
unsafe fn take_memory_object(receipt: *mut c_void) -> Result<Arc<VmObject>> {
    if receipt.is_null() {
        return Err(Error::Io);
    }
    // SAFETY: forwarded from this function's safety contract.
    Ok(unsafe { Arc::from_raw(receipt.cast::<VmObject>()) })
}

/// Returns physical memory pages used to calculate a provider default limit.
pub fn total_physical_pages() -> u64 {
    crate::mem::phys::stats()
        .map(|stats| stats.total_pages as u64)
        .unwrap_or(0)
}

/// # Safety
/// `receipt` must identify a live Arc allocation of `T`.
unsafe fn borrow_arc<T>(receipt: *mut c_void) -> Result<core::mem::ManuallyDrop<Arc<T>>> {
    if receipt.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: forwarded from this function's safety contract.  ManuallyDrop
    // leaves the receipt's ownership count untouched.
    Ok(core::mem::ManuallyDrop::new(unsafe {
        Arc::from_raw(receipt.cast::<T>())
    }))
}

fn mutable_bytes<'a>(pointer: *mut u8, length: usize) -> Result<&'a mut [u8]> {
    if length == 0 {
        return Ok(&mut []);
    }
    if pointer.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: callers of the ABI service validate this range contract.
    Ok(unsafe { core::slice::from_raw_parts_mut(pointer, length) })
}

fn immutable_bytes<'a>(pointer: *const u8, length: usize) -> Result<&'a [u8]> {
    if length == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: callers of the ABI service validate this range contract.
    Ok(unsafe { core::slice::from_raw_parts(pointer, length) })
}

fn mem_error(error: crate::mem::Error) -> Error {
    match error {
        crate::mem::Error::OutOfMemory => Error::OutOfMemory,
        crate::mem::Error::LimitExceeded | crate::mem::Error::SwapUnavailable => Error::NoSpace,
        crate::mem::Error::InvalidAddress => Error::FileTooLarge,
        crate::mem::Error::CorruptSwap | crate::mem::Error::Pmap => Error::Io,
        crate::mem::Error::AlreadyMapped
        | crate::mem::Error::NotMapped
        | crate::mem::Error::Protection => Error::InvalidArgument,
    }
}

// --- Device-filesystem broker ---------------------------------------------

pub(crate) struct DevfsBroker {
    owner: Option<Arc<Module>>,
    operations: DevfsBrokerOps,
}

// SAFETY: the devfs module synchronizes its own immutable callback table.
unsafe impl Send for DevfsBroker {}
// SAFETY: as above.
unsafe impl Sync for DevfsBroker {}

static DEVFS_BROKER: Once<Mutex<Option<Arc<DevfsBroker>>>> = Once::new();

pub(crate) fn init_broker() {
    DEVFS_BROKER.call_once(|| Mutex::new(None));
}

fn devfs_broker_slot() -> crate::driver::Result<&'static Mutex<Option<Arc<DevfsBroker>>>> {
    DEVFS_BROKER
        .get()
        .ok_or(crate::driver::Error::NotInitialized)
}

/// Registers the single devfs control provider.
///
/// # Safety
/// `operations` must identify a complete, immutable C-compatible table whose
/// callbacks remain executable until it is unregistered.
pub(crate) unsafe fn register_devfs_broker(
    owner: Option<&Arc<Module>>,
    operations: *const DevfsBrokerOps,
) -> crate::driver::Result<Arc<DevfsBroker>> {
    if operations.is_null() {
        return Err(crate::driver::Error::InvalidArgument);
    }
    // SAFETY: the ABI caller supplies a readable size-prefixed table.
    let declared = unsafe { (*operations).size as usize };
    if declared < DEVFS_BROKER_REQUIRED_OPS_SIZE as usize {
        return Err(crate::driver::Error::Unsupported);
    }
    let mut copied = MaybeUninit::<DevfsBrokerOps>::zeroed();
    let copied_bytes = declared.min(size_of::<DevfsBrokerOps>());
    // SAFETY: the declared prefix is readable and destination is
    // zero-initialized, so omitted append-only callbacks remain absent.
    unsafe {
        ptr::copy_nonoverlapping(
            operations.cast::<u8>(),
            copied.as_mut_ptr().cast::<u8>(),
            copied_bytes,
        );
    }
    // SAFETY: copied bytes initialized the declared prefix and the rest was
    // explicitly zeroed for optional appended callbacks.
    let operations = unsafe { copied.assume_init() };
    if operations.root.is_none()
        || operations.mkdir.is_none()
        || operations.create.is_none()
        || operations.remove.is_none()
        || operations.remove_owner.is_none()
        || operations.lookup.is_none()
    {
        return Err(crate::driver::Error::InvalidArgument);
    }
    let slot = devfs_broker_slot()?;
    let mut registered = slot.lock();
    if registered.is_some() {
        return Err(crate::driver::Error::AlreadyExists);
    }
    let broker = Arc::new(DevfsBroker {
        owner: owner.cloned(),
        operations,
    });
    *registered = Some(broker.clone());
    Ok(broker)
}

/// Unregisters the global devfs control provider.
pub(crate) fn unregister_devfs_broker(broker: &Arc<DevfsBroker>) -> crate::driver::Result<()> {
    let slot = devfs_broker_slot()?;
    let mut registered = slot.lock();
    let Some(current) = registered.as_ref() else {
        return Err(crate::driver::Error::NotFound);
    };
    if !Arc::ptr_eq(current, broker) {
        return Err(crate::driver::Error::InvalidArgument);
    }
    *registered = None;
    Ok(())
}

/// Removes a broker owned by an unloading module.
pub(crate) fn remove_module_devfs_broker(module: &Arc<Module>) {
    let Ok(slot) = devfs_broker_slot() else {
        return;
    };
    let mut registered = slot.lock();
    if registered.as_ref().is_some_and(|broker| {
        broker
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module))
    }) {
        *registered = None;
    }
}

fn broker() -> crate::driver::Result<(Arc<DevfsBroker>, ModuleLease)> {
    let broker = devfs_broker_slot()?
        .lock()
        .clone()
        .ok_or(crate::driver::Error::NoDevice)?;
    // Device-node registration calls enter module code directly.  Hold a
    // scoped lease across each call so unload cannot unmap that code after
    // this broker has been fetched from the registry.
    let lease = module::lease_owner(broker.owner.as_ref())?;
    Ok((broker, lease))
}

/// Returns the current devfs root ID.
pub(crate) fn devfs_root() -> crate::driver::Result<u64> {
    let (broker, _lease) = broker()?;
    let callback = broker.operations.root.expect("validated devfs root");
    let mut root = 0;
    // SAFETY: broker lease retains callback code and output is local storage.
    driver_status(unsafe { callback(broker.operations.context, &raw mut root) })?;
    Ok(root)
}

/// Creates a devfs directory through the registered provider.
pub(crate) fn devfs_mkdir(
    owner: u64,
    parent: u64,
    name: &[u8],
    mode: u16,
) -> crate::driver::Result<u64> {
    let (broker, _lease) = broker()?;
    let callback = broker.operations.mkdir.expect("validated devfs mkdir");
    let mut node = 0;
    // SAFETY: name and output storage stay valid for the direct call.
    driver_status(unsafe {
        callback(
            broker.operations.context,
            owner,
            parent,
            name.as_ptr(),
            name.len(),
            mode,
            &raw mut node,
        )
    })?;
    Ok(node)
}

/// Creates a devfs device node that owns `endpoint`.
pub(crate) fn devfs_create(
    owner: u64,
    parent: u64,
    name: &[u8],
    kind: u32,
    mode: u16,
    endpoint: *mut c_void,
) -> crate::driver::Result<u64> {
    let (broker, _lease) = broker()?;
    let callback = broker.operations.create.expect("validated devfs create");
    let mut node = 0;
    // SAFETY: the endpoint is an owned opaque receipt; the provider consumes
    // it only after successful creation, and all other inputs live for call.
    driver_status(unsafe {
        callback(
            broker.operations.context,
            owner,
            parent,
            name.as_ptr(),
            name.len(),
            kind,
            mode,
            endpoint,
            &raw mut node,
        )
    })?;
    Ok(node)
}

/// Removes one devfs node owned by `owner`.
pub(crate) fn devfs_remove(owner: u64, node: u64) -> crate::driver::Result<()> {
    let (broker, _lease) = broker()?;
    let callback = broker.operations.remove.expect("validated devfs remove");
    // SAFETY: broker callback lifetime is retained by its lease.
    driver_status(unsafe { callback(broker.operations.context, owner, node) })
}

/// Removes every devfs node owned by `owner`.
pub(crate) fn devfs_remove_owner(owner: u64, force: bool) -> crate::driver::Result<()> {
    let (broker, _lease) = broker()?;
    let callback = broker
        .operations
        .remove_owner
        .expect("validated devfs remove-owner");
    // SAFETY: broker callback lifetime is retained by its lease.
    driver_status(unsafe { callback(broker.operations.context, owner, u8::from(force)) })
}

/// Resolves an absolute devfs path.
pub(crate) fn devfs_lookup(path: &[u8]) -> crate::driver::Result<u64> {
    let (broker, _lease) = broker()?;
    let callback = broker.operations.lookup.expect("validated devfs lookup");
    let mut node = 0;
    // SAFETY: path and output storage are valid for this direct call.
    driver_status(unsafe {
        callback(
            broker.operations.context,
            path.as_ptr(),
            path.len(),
            &raw mut node,
        )
    })?;
    Ok(node)
}

/// Lists direct children of a device-filesystem node.
pub(crate) fn devfs_children(parent: u64) -> crate::driver::Result<Vec<DevfsBrokerEntry>> {
    let (broker, _lease) = broker()?;
    let callback = broker
        .operations
        .children
        .ok_or(crate::driver::Error::Unsupported)?;
    for _ in 0..3 {
        let mut needed = 0usize;
        // SAFETY: the zero-capacity query has no output entries and `needed`
        // is valid scalar output storage.
        driver_status(unsafe {
            callback(
                broker.operations.context,
                parent,
                ptr::null_mut(),
                0,
                &raw mut needed,
            )
        })?;
        let mut entries = vec![DevfsBrokerEntry::EMPTY; needed];
        let mut written = needed;
        // SAFETY: `entries` is writable for its advertised capacity and the
        // broker callback remains resident while the mounted provider holds
        // its module lease.
        let status = unsafe {
            callback(
                broker.operations.context,
                parent,
                entries.as_mut_ptr(),
                entries.len(),
                &raw mut written,
            )
        };
        if status >= 0 && written <= entries.len() {
            entries.truncate(written);
            return Ok(entries);
        }
        if status != crate::driver::Error::NoSpace.to_status() {
            return Err(crate::driver::Error::from_status(status));
        }
    }
    Err(crate::driver::Error::Busy)
}

fn driver_status(status: i32) -> crate::driver::Result<()> {
    if status < 0 {
        Err(crate::driver::Error::from_status(status))
    } else {
        Ok(())
    }
}
