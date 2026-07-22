//! Versioned C ABI shared by C, C++, Rust, and other native drivers.

use alloc::{
    alloc::{alloc, alloc_zeroed, dealloc},
    sync::Arc,
};
use core::{alloc::Layout, mem, ptr, slice, str};

use log::Level;

use crate::{
    fs::{
        self, IoctlContext,
        devtempfs::{self, DevNodeId, DeviceNodeKind, DeviceNodeOps},
    },
    sys::sync::Mutex,
};

use super::{
    BusId, DeviceNodeId, DriverId, Error, ResourceCallback, ResourceFlags, ResourceKey,
    ResourceMethod, ResourceValue, driver,
    interrupt::{
        self, InterruptControllerId, InterruptControllerV1, InterruptHandlerFn, InterruptId,
    },
    tree::{self, NodeKind},
};

/// Version implemented by this host table and module descriptor.
pub const DRIVER_ABI_V1: u32 = 1;

/// Successful ABI operation.
pub const STATUS_OK: i32 = 0;
/// Invalid argument.
pub const STATUS_INVALID_ARGUMENT: i32 = -1;
/// Object not found.
pub const STATUS_NOT_FOUND: i32 = -2;
/// Object already exists.
pub const STATUS_ALREADY_EXISTS: i32 = -3;
/// Object kind mismatch.
pub const STATUS_WRONG_KIND: i32 = -4;
/// Permission denied.
pub const STATUS_PERMISSION_DENIED: i32 = -5;
/// Resource busy.
pub const STATUS_BUSY: i32 = -6;
/// ABI version mismatch.
pub const STATUS_ABI_MISMATCH: i32 = -7;
/// Operation unsupported.
pub const STATUS_UNSUPPORTED: i32 = -8;
/// Insufficient output space.
pub const STATUS_NO_SPACE: i32 = -9;
/// Generic I/O or callback failure.
pub const STATUS_IO: i32 = -10;
/// Kernel allocation failure.
pub const STATUS_OUT_OF_MEMORY: i32 = -11;

static MMIO_MAP_LOCK: Mutex<()> = Mutex::new(());

/// ABI byte slice. The pointed-to memory remains owned by the caller.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct AbiSlice {
    /// First byte, or null when `len` is zero.
    pub data: *const u8,
    /// Number of readable bytes.
    pub len: usize,
}

impl AbiSlice {
    /// Borrows this ABI slice.
    ///
    /// # Safety
    ///
    /// The foreign caller must provide a readable range for the duration of
    /// the returned borrow.
    pub unsafe fn as_slice<'a>(self) -> super::Result<&'a [u8]> {
        if self.len == 0 {
            return Ok(&[]);
        }
        if self.data.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: guaranteed by the caller contract and null-checked above.
        Ok(unsafe { slice::from_raw_parts(self.data, self.len) })
    }
}

/// Driver initialization callback.
pub type DriverInitFn =
    unsafe extern "C" fn(host: *const DriverHostApiV1, driver: u64, context: usize) -> i32;
/// Driver finalization callback.
pub type DriverFiniFn = unsafe extern "C" fn(driver: u64, context: usize);
/// Device open callback.
pub type DeviceOpenFn = unsafe extern "C" fn(context: usize, flags: u32) -> i32;
/// Device close callback.
pub type DeviceCloseFn = unsafe extern "C" fn(context: usize, flags: u32);
/// Device read callback. Non-negative values are byte counts.
pub type DeviceReadFn =
    unsafe extern "C" fn(context: usize, offset: u64, data: *mut u8, len: usize) -> i64;
/// Device write callback. Non-negative values are byte counts.
pub type DeviceWriteFn =
    unsafe extern "C" fn(context: usize, offset: u64, data: *const u8, len: usize) -> i64;
/// Device size callback.
pub type DeviceSizeFn = unsafe extern "C" fn(context: usize) -> u64;
/// Device synchronization callback.
pub type DeviceSyncFn = unsafe extern "C" fn(context: usize) -> i32;
/// Device-control callback. Non-negative values are successful return values.
pub type DeviceIoctlFn = unsafe extern "C" fn(
    context: usize,
    process_id: usize,
    process_group: i32,
    session_id: i32,
    is_session_leader: u8,
    request: u64,
    value: u64,
    argument: *mut u8,
    argument_len: usize,
) -> i64;

/// Version-1 driver module descriptor.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverModuleV1 {
    /// Size of this record.
    pub size: u32,
    /// Must equal [`DRIVER_ABI_V1`].
    pub abi_version: u32,
    /// UTF-8 driver name.
    pub name: AbiSlice,
    /// Opaque module context returned to callbacks.
    pub context: usize,
    /// Required initialization callback.
    pub init: Option<DriverInitFn>,
    /// Optional finalization callback.
    pub fini: Option<DriverFiniFn>,
}

/// Linker-section entry used by statically linked Rust drivers.
#[repr(transparent)]
pub struct LinkedDriverV1(*const DriverModuleV1);

impl LinkedDriverV1 {
    /// Creates a persistent linker-section entry.
    pub const fn new(module: &'static DriverModuleV1) -> Self {
        Self(module)
    }
}

// SAFETY: the wrapper points only to an immutable static descriptor; callback
// thread safety is part of the driver ABI contract.
unsafe impl Sync for LinkedDriverV1 {}

/// Version-1 device vnode callback table.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverDeviceOpsV1 {
    /// Size of this record.
    pub size: u32,
    /// Must equal [`DRIVER_ABI_V1`].
    pub abi_version: u32,
    /// Opaque device context.
    pub context: usize,
    /// Optional open callback.
    pub open: Option<DeviceOpenFn>,
    /// Optional close callback.
    pub close: Option<DeviceCloseFn>,
    /// Optional read callback.
    pub read: Option<DeviceReadFn>,
    /// Optional write callback.
    pub write: Option<DeviceWriteFn>,
    /// Optional size callback.
    pub size_bytes: Option<DeviceSizeFn>,
    /// Optional synchronization callback.
    pub sync: Option<DeviceSyncFn>,
    /// Optional control callback.
    pub ioctl: Option<DeviceIoctlFn>,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct DriverDeviceOpsV1Prefix {
    size: u32,
    abi_version: u32,
    context: usize,
    open: Option<DeviceOpenFn>,
    close: Option<DeviceCloseFn>,
    read: Option<DeviceReadFn>,
    write: Option<DeviceWriteFn>,
    size_bytes: Option<DeviceSizeFn>,
    sync: Option<DeviceSyncFn>,
}

/// Stable host service table passed to every driver.
#[repr(C)]
pub struct DriverHostApiV1 {
    /// Size of this record.
    pub size: u32,
    /// Host ABI version.
    pub abi_version: u32,
    /// Returns the root bus.
    pub root_bus: unsafe extern "C" fn(out_bus: *mut u64) -> i32,
    /// Registers a nested bus.
    pub register_bus:
        unsafe extern "C" fn(driver: u64, parent: u64, name: AbiSlice, out_bus: *mut u64) -> i32,
    /// Registers a device.
    pub register_device:
        unsafe extern "C" fn(driver: u64, parent: u64, name: AbiSlice, out_device: *mut u64) -> i32,
    /// Removes an owned hierarchy node.
    pub remove_node: unsafe extern "C" fn(driver: u64, node: u64) -> i32,
    /// Publishes immutable data on a bus.
    pub publish_data_resource: unsafe extern "C" fn(
        driver: u64,
        bus: u64,
        key: ResourceKey,
        flags: u64,
        data: AbiSlice,
        out_resource: *mut u64,
    ) -> i32,
    /// Publishes a callable operation on a bus.
    pub publish_method_resource: unsafe extern "C" fn(
        driver: u64,
        bus: u64,
        key: ResourceKey,
        flags: u64,
        context: usize,
        callback: Option<ResourceCallback>,
        out_resource: *mut u64,
    ) -> i32,
    /// Copies inherited resource data to a driver buffer.
    pub read_resource: unsafe extern "C" fn(
        node: u64,
        key: ResourceKey,
        output: *mut u8,
        output_len: usize,
        written: *mut usize,
    ) -> i32,
    /// Invokes an inherited resource method.
    pub invoke_resource: unsafe extern "C" fn(
        node: u64,
        key: ResourceKey,
        input: AbiSlice,
        output: *mut u8,
        output_len: usize,
        written: *mut usize,
    ) -> i32,
    /// Returns the devtempfs root directory.
    pub devfs_root: unsafe extern "C" fn(out_node: *mut u64) -> i32,
    /// Creates a driver-owned devtempfs directory.
    pub devfs_create_dir: unsafe extern "C" fn(
        driver: u64,
        parent: u64,
        name: AbiSlice,
        mode: u16,
        out_node: *mut u64,
    ) -> i32,
    /// Creates a driver-owned character or block node.
    pub devfs_create_device: unsafe extern "C" fn(
        driver: u64,
        parent: u64,
        name: AbiSlice,
        kind: u32,
        mode: u16,
        device: u64,
        operations: *const DriverDeviceOpsV1,
        out_node: *mut u64,
    ) -> i32,
    /// Removes a driver-owned devtempfs node.
    pub devfs_remove_node: unsafe extern "C" fn(driver: u64, node: u64) -> i32,
    /// Allocates uninitialized kernel heap memory.
    pub allocate: unsafe extern "C" fn(size: usize, align: usize) -> *mut u8,
    /// Allocates zeroed kernel heap memory.
    pub allocate_zeroed: unsafe extern "C" fn(size: usize, align: usize) -> *mut u8,
    /// Releases kernel heap memory with its original layout.
    pub deallocate: unsafe extern "C" fn(data: *mut u8, size: usize, align: usize) -> i32,
    /// Writes a driver message to the kernel log.
    pub log: unsafe extern "C" fn(level: u32, message: AbiSlice) -> i32,
    /// Registers an interrupt controller on an owned bus.
    pub register_interrupt_controller: unsafe extern "C" fn(
        driver: u64,
        bus: u64,
        controller: *const InterruptControllerV1,
        out_controller: *mut u64,
    ) -> i32,
    /// Removes an idle interrupt controller.
    pub unregister_interrupt_controller: unsafe extern "C" fn(driver: u64, controller: u64) -> i32,
    /// Routes an interrupt for an owned device node.
    pub request_interrupt: unsafe extern "C" fn(
        driver: u64,
        node: u64,
        specifier: AbiSlice,
        flags: u64,
        target_cpu: u32,
        handler: Option<InterruptHandlerFn>,
        context: usize,
        out_interrupt: *mut u64,
    ) -> i32,
    /// Releases an owned interrupt route.
    pub release_interrupt: unsafe extern "C" fn(driver: u64, interrupt: u64) -> i32,
    /// Masks an owned interrupt route.
    pub mask_interrupt: unsafe extern "C" fn(driver: u64, interrupt: u64) -> i32,
    /// Unmasks an owned interrupt route.
    pub unmask_interrupt: unsafe extern "C" fn(driver: u64, interrupt: u64) -> i32,
    /// Retargets an owned interrupt route.
    pub set_interrupt_affinity:
        unsafe extern "C" fn(driver: u64, interrupt: u64, target_cpu: u32) -> i32,
    /// Establishes a persistent device mapping in the kernel direct map.
    pub map_mmio: unsafe extern "C" fn(
        driver: u64,
        physical: u64,
        size: usize,
        out_address: *mut usize,
    ) -> i32,
}

/// Static version-1 host service table.
pub static HOST_API_V1: DriverHostApiV1 = DriverHostApiV1 {
    size: mem::size_of::<DriverHostApiV1>() as u32,
    abi_version: DRIVER_ABI_V1,
    root_bus: host_root_bus,
    register_bus: host_register_bus,
    register_device: host_register_device,
    remove_node: host_remove_node,
    publish_data_resource: host_publish_data_resource,
    publish_method_resource: host_publish_method_resource,
    read_resource: host_read_resource,
    invoke_resource: host_invoke_resource,
    devfs_root: host_devfs_root,
    devfs_create_dir: host_devfs_create_dir,
    devfs_create_device: host_devfs_create_device,
    devfs_remove_node: host_devfs_remove_node,
    allocate: host_allocate,
    allocate_zeroed: host_allocate_zeroed,
    deallocate: host_deallocate,
    log: host_log,
    register_interrupt_controller: host_register_interrupt_controller,
    unregister_interrupt_controller: host_unregister_interrupt_controller,
    request_interrupt: host_request_interrupt,
    release_interrupt: host_release_interrupt,
    mask_interrupt: host_mask_interrupt,
    unmask_interrupt: host_unmask_interrupt,
    set_interrupt_affinity: host_set_interrupt_affinity,
    map_mmio: host_map_mmio,
};

struct ForeignDeviceOps {
    operations: DriverDeviceOpsV1,
}

impl DeviceNodeOps for ForeignDeviceOps {
    fn open(&self, flags: u32) -> fs::Result<()> {
        let Some(callback) = self.operations.open else {
            return Ok(());
        };
        // SAFETY: the copied callback table was validated when the node was
        // created and devtempfs pins the owning driver around this call.
        callback_status(unsafe { callback(self.operations.context, flags) })
    }

    fn close(&self, flags: u32) {
        let Some(callback) = self.operations.close else {
            return;
        };
        // SAFETY: devtempfs pins the owning driver around this call.
        unsafe { callback(self.operations.context, flags) };
    }

    fn read_at(&self, offset: u64, buffer: &mut [u8]) -> fs::Result<usize> {
        let callback = self.operations.read.ok_or(fs::Error::Unsupported)?;
        let data = if buffer.is_empty() {
            ptr::null_mut()
        } else {
            buffer.as_mut_ptr()
        };
        // SAFETY: `buffer` is writable for `buffer.len()` and the callback is
        // pinned by devtempfs.
        callback_count(
            unsafe { callback(self.operations.context, offset, data, buffer.len()) },
            buffer.len(),
        )
    }

    fn write_at(&self, offset: u64, buffer: &[u8]) -> fs::Result<usize> {
        let callback = self.operations.write.ok_or(fs::Error::Unsupported)?;
        let data = if buffer.is_empty() {
            ptr::null()
        } else {
            buffer.as_ptr()
        };
        // SAFETY: `buffer` is readable for `buffer.len()` and the callback is
        // pinned by devtempfs.
        callback_count(
            unsafe { callback(self.operations.context, offset, data, buffer.len()) },
            buffer.len(),
        )
    }

    fn size(&self) -> u64 {
        let Some(callback) = self.operations.size_bytes else {
            return 0;
        };
        // SAFETY: devtempfs pins the owning driver around this call.
        unsafe { callback(self.operations.context) }
    }

    fn sync(&self) -> fs::Result<()> {
        let Some(callback) = self.operations.sync else {
            return Ok(());
        };
        // SAFETY: devtempfs pins the owning driver around this call.
        callback_status(unsafe { callback(self.operations.context) })
    }

    fn ioctl(
        &self,
        context: IoctlContext,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> fs::Result<u64> {
        let callback = self.operations.ioctl.ok_or(fs::Error::Unsupported)?;
        let pointer = if argument.is_empty() {
            ptr::null_mut()
        } else {
            argument.as_mut_ptr()
        };
        // SAFETY: devtempfs pins the owning driver and `argument` is writable
        // for the duration of this call.
        let result = unsafe {
            callback(
                self.operations.context,
                context.process_id,
                context.process_group,
                context.session_id,
                u8::from(context.is_session_leader),
                request,
                value,
                pointer,
                argument.len(),
            )
        };
        if result < 0 {
            return Err(status_to_fs(i32::try_from(result).unwrap_or(STATUS_IO)));
        }
        Ok(result as u64)
    }
}

/// Loads a module descriptor through the exported C ABI.
///
/// # Safety
///
/// `module` and `out_driver` must be valid pointers following the version-1
/// ABI. Module callback code must remain mapped until unload succeeds.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn roanix_driver_load_v1(
    module: *const DriverModuleV1,
    out_driver: *mut u64,
) -> i32 {
    if !is_aligned(module) || !is_aligned(out_driver) {
        return STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: pointer validity is required by the exported function contract.
    let result = unsafe { driver::load(&*module) };
    match result {
        Ok(id) => {
            // SAFETY: checked non-null/aligned and guaranteed writable by the
            // caller contract.
            unsafe { out_driver.write(id.get()) };
            STATUS_OK
        }
        Err(error) => status(error),
    }
}

/// Unloads a module through the exported C ABI.
#[unsafe(no_mangle)]
pub extern "C" fn roanix_driver_unload_v1(driver_id: u64) -> i32 {
    match driver::unload(DriverId::new(driver_id)) {
        Ok(()) => STATUS_OK,
        Err(error) => status(error),
    }
}

unsafe extern "C" fn host_root_bus(out_bus: *mut u64) -> i32 {
    match super::root_bus().and_then(|bus| {
        // SAFETY: the host ABI requires a writable output pointer.
        unsafe { write_out(out_bus, bus.node().get()) }
    }) {
        Ok(()) => STATUS_OK,
        Err(error) => status(error),
    }
}

unsafe extern "C" fn host_register_bus(
    driver_id: u64,
    parent: u64,
    name: AbiSlice,
    out_bus: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let parent_node = DeviceNodeId::from_raw(parent);
        let parent_info = tree::node_info(parent_node)?;
        let _parent = driver::parent_guard(owner, parent_info.owner)?;
        let parent = BusId::from_node(parent_node)?;
        // SAFETY: required by the host function ABI.
        let name = unsafe { abi_name(name)? };
        let bus = super::register_bus(owner, parent, name)?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_bus, bus.node().get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_register_device(
    driver_id: u64,
    parent: u64,
    name: AbiSlice,
    out_device: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let parent_node = DeviceNodeId::from_raw(parent);
        let parent_info = tree::node_info(parent_node)?;
        let _parent = driver::parent_guard(owner, parent_info.owner)?;
        let parent = BusId::from_node(parent_node)?;
        // SAFETY: required by the host function ABI.
        let name = unsafe { abi_name(name)? };
        let device = super::register_device(owner, parent, name)?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_device, device.node().get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_remove_node(driver_id: u64, node: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| super::remove_node(owner, DeviceNodeId::from_raw(node)));
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_publish_data_resource(
    driver_id: u64,
    bus: u64,
    key: ResourceKey,
    flags: u64,
    data: AbiSlice,
    out_resource: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let bus = BusId::from_node(DeviceNodeId::from_raw(bus))?;
        // SAFETY: required by the host function ABI.
        let bytes: Arc<[u8]> = Arc::from(unsafe { data.as_slice()? });
        let id = super::publish_resource(
            owner,
            bus,
            key,
            ResourceFlags::from_bits(flags),
            ResourceValue::Data(bytes),
        )?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_resource, id.get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_publish_method_resource(
    driver_id: u64,
    bus: u64,
    key: ResourceKey,
    flags: u64,
    context: usize,
    callback: Option<ResourceCallback>,
    out_resource: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let bus = BusId::from_node(DeviceNodeId::from_raw(bus))?;
        let callback = callback.ok_or(Error::InvalidArgument)?;
        // SAFETY: the publishing driver promises the callback follows the
        // resource ABI and remains executable until its objects are removed.
        let method = unsafe { ResourceMethod::new_driver(owner, context, callback) };
        let id = super::publish_resource(
            owner,
            bus,
            key,
            ResourceFlags::from_bits(flags),
            ResourceValue::Method(method),
        )?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_resource, id.get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_read_resource(
    node: u64,
    key: ResourceKey,
    output: *mut u8,
    output_len: usize,
    written: *mut usize,
) -> i32 {
    let result = (|| {
        let resource = super::resolve_resource(DeviceNodeId::from_raw(node), key)?;
        let data = resource.data()?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(written, data.len())? };
        if output_len < data.len() {
            return Err(Error::NoSpace);
        }
        // SAFETY: required by the host function ABI.
        let output = unsafe { output_slice(output, output_len)? };
        output[..data.len()].copy_from_slice(&data);
        Ok(())
    })();
    match result {
        Ok(()) => STATUS_OK,
        Err(error) => status(error),
    }
}

unsafe extern "C" fn host_invoke_resource(
    node: u64,
    key: ResourceKey,
    input: AbiSlice,
    output: *mut u8,
    output_len: usize,
    written: *mut usize,
) -> i32 {
    let result = (|| {
        let resource = super::resolve_resource(DeviceNodeId::from_raw(node), key)?;
        if ranges_overlap(input.data, input.len, output as *const u8, output_len)? {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: required by the host function ABI.
        let input = unsafe { input.as_slice()? };
        // SAFETY: required by the host function ABI.
        let output = unsafe { output_slice(output, output_len)? };
        let count = resource.invoke(input, output)?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(written, count) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_devfs_root(out_node: *mut u64) -> i32 {
    let result = devtempfs::global()
        .map_err(|_| Error::Filesystem)
        .and_then(|filesystem| {
            // SAFETY: required by the host function ABI.
            unsafe { write_out(out_node, filesystem.root_id().get()) }
        });
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_devfs_create_dir(
    driver_id: u64,
    parent: u64,
    name: AbiSlice,
    mode: u16,
    out_node: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let filesystem = devtempfs::global().map_err(|_| Error::Filesystem)?;
        let parent = DevNodeId::from_raw(parent);
        let _parent =
            driver::parent_guard(owner, filesystem.owner(parent).map_err(fs_to_device_error)?)?;
        // SAFETY: required by the host function ABI.
        let name = unsafe { name.as_slice()? };
        let node = filesystem
            .create_dir(owner, parent, name, mode)
            .map_err(fs_to_device_error)?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_node, node.get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_devfs_create_device(
    driver_id: u64,
    parent: u64,
    name: AbiSlice,
    kind: u32,
    mode: u16,
    device: u64,
    operations: *const DriverDeviceOpsV1,
    out_node: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let filesystem = devtempfs::global().map_err(|_| Error::Filesystem)?;
        let parent = DevNodeId::from_raw(parent);
        let _parent =
            driver::parent_guard(owner, filesystem.owner(parent).map_err(fs_to_device_error)?)?;
        // SAFETY: required by the host function ABI.
        let name = unsafe { name.as_slice()? };
        let device = DeviceNodeId::from_raw(device);
        let info = tree::node_info(device)?;
        if info.owner != owner || info.kind != NodeKind::Device {
            return Err(Error::PermissionDenied);
        }
        // SAFETY: required by the host function ABI.
        let operations = unsafe { read_operations(operations)? };
        let kind = match kind {
            1 => DeviceNodeKind::Character,
            2 => DeviceNodeKind::Block,
            _ => return Err(Error::InvalidArgument),
        };
        let operations: Arc<dyn DeviceNodeOps> = Arc::new(ForeignDeviceOps { operations });
        let node = filesystem
            .create_device(owner, parent, name, kind, mode, device, operations)
            .map_err(fs_to_device_error)?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_node, node.get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_devfs_remove_node(driver_id: u64, node: u64) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        devtempfs::global()
            .map_err(|_| Error::Filesystem)?
            .remove_node(owner, DevNodeId::from_raw(node))
            .map_err(fs_to_device_error)
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_allocate(size: usize, align: usize) -> *mut u8 {
    let Ok(layout) = Layout::from_size_align(size, align) else {
        return ptr::null_mut();
    };
    if size == 0 {
        return align as *mut u8;
    }
    // SAFETY: `layout` was validated above.
    unsafe { alloc(layout) }
}

unsafe extern "C" fn host_allocate_zeroed(size: usize, align: usize) -> *mut u8 {
    let Ok(layout) = Layout::from_size_align(size, align) else {
        return ptr::null_mut();
    };
    if size == 0 {
        return align as *mut u8;
    }
    // SAFETY: `layout` was validated above.
    unsafe { alloc_zeroed(layout) }
}

unsafe extern "C" fn host_deallocate(data: *mut u8, size: usize, align: usize) -> i32 {
    let Ok(layout) = Layout::from_size_align(size, align) else {
        return STATUS_INVALID_ARGUMENT;
    };
    if size == 0 {
        return STATUS_OK;
    }
    if data.is_null() || (data as usize) % align != 0 {
        return STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: the ABI requires this pointer and layout to match a successful
    // host allocation that has not already been freed.
    unsafe { dealloc(data, layout) };
    STATUS_OK
}

unsafe extern "C" fn host_log(level: u32, message: AbiSlice) -> i32 {
    // SAFETY: required by the host function ABI.
    let Ok(bytes) = (unsafe { message.as_slice() }) else {
        return STATUS_INVALID_ARGUMENT;
    };
    let Ok(message) = str::from_utf8(bytes) else {
        return STATUS_INVALID_ARGUMENT;
    };
    let level = match level {
        1 => Level::Error,
        2 => Level::Warn,
        3 => Level::Info,
        4 => Level::Debug,
        5 => Level::Trace,
        _ => return STATUS_INVALID_ARGUMENT,
    };
    log::log!(level, "driver: {message}");
    STATUS_OK
}

unsafe extern "C" fn host_register_interrupt_controller(
    driver_id: u64,
    bus: u64,
    controller: *const InterruptControllerV1,
    out_controller: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let bus = BusId::from_node(DeviceNodeId::from_raw(bus))?;
        // SAFETY: required by the host function ABI.
        let controller = unsafe { read_interrupt_controller(controller)? };
        // SAFETY: the foreign driver guarantees the copied callback table
        // follows the interrupt-controller ABI for its registered lifetime.
        let id = unsafe { interrupt::register_controller(owner, bus, controller)? };
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_controller, id.get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_unregister_interrupt_controller(driver_id: u64, controller: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner).and_then(|()| {
        interrupt::unregister_controller(owner, InterruptControllerId::from_raw(controller))
    });
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_request_interrupt(
    driver_id: u64,
    node: u64,
    specifier: AbiSlice,
    flags: u64,
    target_cpu: u32,
    handler: Option<InterruptHandlerFn>,
    context: usize,
    out_interrupt: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let handler = handler.ok_or(Error::InvalidArgument)?;
        // SAFETY: required by the host function ABI.
        let specifier = unsafe { specifier.as_slice()? };
        let id = interrupt::request_interrupt(
            owner,
            DeviceNodeId::from_raw(node),
            specifier,
            flags,
            target_cpu,
            handler,
            context,
        )?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_interrupt, id.get()) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_release_interrupt(driver_id: u64, interrupt_id: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| interrupt::release_interrupt(owner, InterruptId::from_raw(interrupt_id)));
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_mask_interrupt(driver_id: u64, interrupt_id: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| interrupt::mask_interrupt(owner, InterruptId::from_raw(interrupt_id)));
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_unmask_interrupt(driver_id: u64, interrupt_id: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| interrupt::unmask_interrupt(owner, InterruptId::from_raw(interrupt_id)));
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_set_interrupt_affinity(
    driver_id: u64,
    interrupt_id: u64,
    target_cpu: u32,
) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner).and_then(|()| {
        interrupt::set_interrupt_affinity(owner, InterruptId::from_raw(interrupt_id), target_cpu)
    });
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_map_mmio(
    driver_id: u64,
    physical: u64,
    size: usize,
    out_address: *mut usize,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        let _owner = driver::mutation_guard(owner)?;
        if size == 0 {
            return Err(Error::InvalidArgument);
        }
        let size = u64::try_from(size).map_err(|_| Error::InvalidArgument)?;
        let end = physical.checked_add(size).ok_or(Error::InvalidArgument)?;
        let _mapping = MMIO_MAP_LOCK.lock();
        let start_page = crate::mem::PhysAddr::new(physical).align_down();
        let end_page = crate::mem::PhysAddr::new(end).align_up();
        let root = crate::arch::paging::active_root();
        let flags = crate::mem::VmFlags::READ
            | crate::mem::VmFlags::WRITE
            | crate::mem::VmFlags::GLOBAL
            | crate::mem::VmFlags::DEVICE;
        let mut page = start_page;
        while page < end_page {
            let virtual_page = crate::mem::phys_to_virt(page);
            // SAFETY: the caller identifies this physical range as device
            // MMIO; this only inspects the active kernel page tables.
            let mapped = unsafe { crate::arch::paging::translate(root, virtual_page) };
            if let Some(mapped) = mapped {
                if mapped != page {
                    return Err(Error::AlreadyExists);
                }
            } else {
                // SAFETY: the driver requested a global device mapping and
                // the device subsystem serializes this mutation through its
                // driver callback guard during initialization/control calls.
                unsafe { crate::arch::paging::map_page(root, virtual_page, page, flags) }
                    .map_err(|_| Error::OutOfMemory)?;
            }
            page = page
                .checked_add(crate::mem::PAGE_SIZE)
                .ok_or(Error::InvalidArgument)?;
        }
        let address = crate::mem::phys_to_virt(crate::mem::PhysAddr::new(physical)).as_u64();
        let address = usize::try_from(address).map_err(|_| Error::InvalidArgument)?;
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_address, address) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe fn abi_name<'a>(name: AbiSlice) -> super::Result<&'a str> {
    // SAFETY: forwarded from this function's caller.
    let name = unsafe { name.as_slice()? };
    str::from_utf8(name).map_err(|_| Error::InvalidArgument)
}

unsafe fn output_slice<'a>(output: *mut u8, len: usize) -> super::Result<&'a mut [u8]> {
    if len == 0 {
        return Ok(&mut []);
    }
    if output.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the host ABI requires a writable range of `len` bytes.
    Ok(unsafe { slice::from_raw_parts_mut(output, len) })
}

unsafe fn write_out<T>(output: *mut T, value: T) -> super::Result<()> {
    if !is_aligned(output) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the host ABI requires a writable, aligned output pointer.
    unsafe { output.write(value) };
    Ok(())
}

unsafe fn read_operations(
    operations: *const DriverDeviceOpsV1,
) -> super::Result<DriverDeviceOpsV1> {
    if !is_aligned(operations) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the version-1 ABI guarantees at least the size/version prefix is
    // readable. No reference to the potentially shorter foreign record is
    // created.
    let size = unsafe { core::ptr::addr_of!((*operations).size).read() } as usize;
    // SAFETY: same prefix contract as above.
    let version = unsafe { core::ptr::addr_of!((*operations).abi_version).read() };
    if version != DRIVER_ABI_V1 || size < mem::size_of::<DriverDeviceOpsV1Prefix>() {
        return Err(Error::AbiMismatch);
    }
    // SAFETY: the validated size covers the complete original version-1
    // prefix, which has stable C layout.
    let prefix = unsafe { operations.cast::<DriverDeviceOpsV1Prefix>().read() };
    let ioctl = if size >= mem::size_of::<DriverDeviceOpsV1>() {
        // SAFETY: the reported record size covers the optional tail field.
        unsafe { core::ptr::addr_of!((*operations).ioctl).read() }
    } else {
        None
    };
    Ok(DriverDeviceOpsV1 {
        size: prefix.size,
        abi_version: prefix.abi_version,
        context: prefix.context,
        open: prefix.open,
        close: prefix.close,
        read: prefix.read,
        write: prefix.write,
        size_bytes: prefix.size_bytes,
        sync: prefix.sync,
        ioctl,
    })
}

unsafe fn read_interrupt_controller(
    controller: *const InterruptControllerV1,
) -> super::Result<InterruptControllerV1> {
    if !is_aligned(controller) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the versioned ABI guarantees the size/version prefix is readable.
    let size = unsafe { core::ptr::addr_of!((*controller).size).read() } as usize;
    // SAFETY: the same prefix contract covers this field.
    let version = unsafe { core::ptr::addr_of!((*controller).abi_version).read() };
    if version != interrupt::INTERRUPT_ABI_V1 || size < mem::size_of::<InterruptControllerV1>() {
        return Err(Error::AbiMismatch);
    }
    // SAFETY: the validated record size covers the complete version-1 table.
    Ok(unsafe { controller.read() })
}

fn is_aligned<T>(pointer: *const T) -> bool {
    !pointer.is_null() && (pointer as usize) % mem::align_of::<T>() == 0
}

fn ranges_overlap(
    first: *const u8,
    first_len: usize,
    second: *const u8,
    second_len: usize,
) -> super::Result<bool> {
    if first_len == 0 || second_len == 0 {
        return Ok(false);
    }
    if first.is_null() || second.is_null() {
        return Err(Error::InvalidArgument);
    }
    let first_start = first as usize;
    let second_start = second as usize;
    let first_end = first_start
        .checked_add(first_len)
        .ok_or(Error::InvalidArgument)?;
    let second_end = second_start
        .checked_add(second_len)
        .ok_or(Error::InvalidArgument)?;
    Ok(first_start < second_end && second_start < first_end)
}

fn callback_status(status: i32) -> fs::Result<()> {
    if status == STATUS_OK {
        Ok(())
    } else {
        Err(status_to_fs(status))
    }
}

fn callback_count(value: i64, maximum: usize) -> fs::Result<usize> {
    if value < 0 {
        return Err(status_to_fs(i32::try_from(value).unwrap_or(STATUS_IO)));
    }
    let value = usize::try_from(value).map_err(|_| fs::Error::Io)?;
    if value > maximum {
        return Err(fs::Error::Io);
    }
    Ok(value)
}

fn status_to_fs(status: i32) -> fs::Error {
    match status {
        STATUS_INVALID_ARGUMENT => fs::Error::InvalidArgument,
        STATUS_NOT_FOUND => fs::Error::NotFound,
        STATUS_ALREADY_EXISTS => fs::Error::AlreadyExists,
        STATUS_WRONG_KIND => fs::Error::InvalidArgument,
        STATUS_PERMISSION_DENIED => fs::Error::PermissionDenied,
        STATUS_BUSY => fs::Error::Busy,
        STATUS_UNSUPPORTED => fs::Error::Unsupported,
        STATUS_NO_SPACE => fs::Error::NoSpace,
        STATUS_OUT_OF_MEMORY => fs::Error::OutOfMemory,
        _ => fs::Error::Io,
    }
}

fn fs_to_device_error(error: fs::Error) -> Error {
    match error {
        fs::Error::NotFound => Error::NotFound,
        fs::Error::AlreadyExists => Error::AlreadyExists,
        fs::Error::Busy | fs::Error::NotEmpty => Error::Busy,
        fs::Error::PermissionDenied | fs::Error::ReadOnly => Error::PermissionDenied,
        fs::Error::OutOfMemory => Error::OutOfMemory,
        fs::Error::NoSpace => Error::NoSpace,
        fs::Error::Unsupported => Error::Unsupported,
        fs::Error::InvalidArgument | fs::Error::NameTooLong => Error::InvalidArgument,
        _ => Error::Filesystem,
    }
}

fn status(error: Error) -> i32 {
    match error {
        Error::NotInitialized | Error::Filesystem => STATUS_IO,
        Error::CallbackFailed(status) => normalize_callback_status(status),
        Error::InvalidArgument => STATUS_INVALID_ARGUMENT,
        Error::NotFound => STATUS_NOT_FOUND,
        Error::AlreadyExists => STATUS_ALREADY_EXISTS,
        Error::WrongKind => STATUS_WRONG_KIND,
        Error::PermissionDenied => STATUS_PERMISSION_DENIED,
        Error::Busy => STATUS_BUSY,
        Error::AbiMismatch => STATUS_ABI_MISMATCH,
        Error::Unsupported => STATUS_UNSUPPORTED,
        Error::NoSpace => STATUS_NO_SPACE,
        Error::OutOfMemory => STATUS_OUT_OF_MEMORY,
    }
}

fn normalize_callback_status(status: i32) -> i32 {
    match status {
        STATUS_INVALID_ARGUMENT
        | STATUS_NOT_FOUND
        | STATUS_ALREADY_EXISTS
        | STATUS_WRONG_KIND
        | STATUS_PERMISSION_DENIED
        | STATUS_BUSY
        | STATUS_ABI_MISMATCH
        | STATUS_UNSUPPORTED
        | STATUS_NO_SPACE
        | STATUS_IO
        | STATUS_OUT_OF_MEMORY => status,
        _ => STATUS_IO,
    }
}
