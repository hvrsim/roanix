//! C interface shared by in-tree native drivers and the kernel.

use alloc::{
    alloc::{alloc, alloc_zeroed, dealloc},
    boxed::Box,
    collections::BTreeMap,
    sync::Arc,
    vec::Vec,
};
use core::{alloc::Layout, mem, ptr, slice, str, time::Duration};

use log::Level;

use crate::{
    fs::{
        self, IoctlContext,
        devtempfs::{self, DevNodeId, DeviceNodeKind, DeviceNodeOps},
    },
    sys::{
        clock, debug,
        event::Event,
        random,
        sync::{Mutex, Once},
    },
};

use super::{
    BusId, DeviceNodeId, DriverId, Error, MemoryRegion, ResourceFlags, ResourceKey,
    ResourceLeaseId, ResourceProtocol, ResourceValue,
    binding::{self, DriverClassId},
    console::{ConsoleBackend, SerialSettings, Tty},
    driver,
    interrupt::{
        self, InterruptController, InterruptControllerId, InterruptHandlerFn, InterruptId,
        InterruptThreadFn,
    },
    tree::{self, NodeKind},
};

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
/// Operation unsupported.
pub const STATUS_UNSUPPORTED: i32 = -7;
/// Insufficient output space.
pub const STATUS_NO_SPACE: i32 = -8;
/// Generic I/O or callback failure.
pub const STATUS_IO: i32 = -9;
/// Kernel allocation failure.
pub const STATUS_OUT_OF_MEMORY: i32 = -10;
/// A nonblocking operation would have slept.
pub const STATUS_WOULD_BLOCK: i32 = -11;
/// An operation was interrupted.
pub const STATUS_INTERRUPTED: i32 = -12;
/// The descriptor is not a terminal.
pub const STATUS_NOT_TTY: i32 = -13;
/// The descriptor does not support seeking.
pub const STATUS_ILLEGAL_SEEK: i32 = -14;
/// Acquiring a resource would introduce a dependency cycle.
pub const STATUS_DEADLOCK: i32 = -15;
/// Driver binding should be retried when provider state changes.
pub const STATUS_DEFERRED: i32 = -16;

/// Current incompatible driver ABI generation.
pub const DRIVER_ABI_MAJOR: u16 = 1;
/// Current append-only driver ABI revision.
pub const DRIVER_ABI_MINOR: u16 = 0;

const SERVICE_REVISION: u32 = 1;
const SERVICE_CORE: u64 = 0x524F_414E_434F_5245;
const SERVICE_REGISTRY: u64 = 0x524F_414E_5245_4749;
const SERVICE_MEMORY: u64 = 0x524F_414E_4D45_4D4F;
const SERVICE_INTERRUPT: u64 = 0x524F_414E_4952_5153;
const SERVICE_BINDING: u64 = 0x524F_414E_4249_4E44;

struct MmioPageRecord {
    references: u64,
    mapped_by_devkit: bool,
    permanent: bool,
}

struct MmioMappingRecord {
    owner: DriverId,
    source_lease: Option<ResourceLeaseId>,
    pages: Box<[crate::mem::PhysAddr]>,
}

struct MmioRegistryState {
    next_mapping: u64,
    pages: BTreeMap<crate::mem::PhysAddr, MmioPageRecord>,
    mappings: BTreeMap<u64, MmioMappingRecord>,
}

struct MmioRegistry {
    state: Mutex<MmioRegistryState>,
}

static MMIO_REGISTRY: Once<MmioRegistry> = Once::new();

/// ABI byte slice. The pointed-to memory remains owned by the caller.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct AbiSlice {
    /// First byte, or null when `len` is zero.
    pub data: *const u8,
    /// Number of readable bytes.
    pub len: usize,
}

/// C representation of serial line settings.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverSerialSettings {
    baud: u32,
    data_bits: u8,
    stop_bits: u8,
    parity: u8,
    odd_parity: u8,
}

type ConsoleOpenFn = unsafe extern "C" fn(context: usize) -> i32;
type ConsoleCloseFn = unsafe extern "C" fn(context: usize);
type ConsoleTryReadFn = unsafe extern "C" fn(context: usize, output: *mut u8) -> i32;
type ConsoleReadFn = unsafe extern "C" fn(context: usize, output: *mut u8, len: usize) -> i64;
type ConsoleWriteFn =
    unsafe extern "C" fn(context: usize, data: *const u8, len: usize, nonblocking: u8) -> i32;
type ConsoleConfigureFn =
    unsafe extern "C" fn(context: usize, settings: *const DriverSerialSettings) -> i32;
type ConsoleSimpleFn = unsafe extern "C" fn(context: usize) -> i32;
type ConsoleBreakFn = unsafe extern "C" fn(context: usize, duration_ms: u64) -> i32;
type ConsoleHungUpFn = unsafe extern "C" fn(context: usize) -> i32;
type ConsoleWritableFn = unsafe extern "C" fn(context: usize) -> i32;
type ConsoleQueuedFn = unsafe extern "C" fn(context: usize) -> i64;
type ConsoleDestroyFn = unsafe extern "C" fn(context: usize);

/// Hardware/backend callbacks consumed by the kernel TTY service.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverConsoleOps {
    size: u32,
    flags: u64,
    context: usize,
    open: Option<ConsoleOpenFn>,
    close: Option<ConsoleCloseFn>,
    try_read: Option<ConsoleTryReadFn>,
    write: Option<ConsoleWriteFn>,
    configure: Option<ConsoleConfigureFn>,
    flush: Option<ConsoleSimpleFn>,
    send_break: Option<ConsoleBreakFn>,
    writable: Option<ConsoleWritableFn>,
    hung_up: Option<ConsoleHungUpFn>,
    destroy: Option<ConsoleDestroyFn>,
    read: Option<ConsoleReadFn>,
    flush_input: Option<ConsoleSimpleFn>,
    flush_output: Option<ConsoleSimpleFn>,
    queued_output: Option<ConsoleQueuedFn>,
    readable_event: usize,
    writable_event: usize,
    hangup_event: usize,
}

impl DriverConsoleOps {
    const EMPTY: Self = Self {
        size: 0,
        flags: 0,
        context: 0,
        open: None,
        close: None,
        try_read: None,
        write: None,
        configure: None,
        flush: None,
        send_break: None,
        writable: None,
        hung_up: None,
        destroy: None,
        read: None,
        flush_input: None,
        flush_output: None,
        queued_output: None,
        readable_event: 0,
        writable_event: 0,
        hangup_event: 0,
    };
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
        if self.len > isize::MAX as usize || (self.data as usize).checked_add(self.len).is_none() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: guaranteed by the caller contract and null-checked above.
        Ok(unsafe { slice::from_raw_parts(self.data, self.len) })
    }
}

/// Driver initialization callback.
pub type DriverInitFn =
    unsafe extern "C" fn(bootstrap: *const DriverBootstrap, driver: u64, context: usize) -> i32;
/// Driver finalization callback.
pub type DriverFiniFn = unsafe extern "C" fn(driver: u64, context: usize);
/// Device open callback.
pub type DeviceOpenFn =
    unsafe extern "C" fn(context: usize, flags: u32, file_context: *mut usize) -> i32;
/// Device close callback.
pub type DeviceCloseFn = unsafe extern "C" fn(context: usize, file_context: usize, flags: u32);
/// Device initial-offset callback. Negative values are error statuses.
pub type DeviceInitialOffsetFn =
    unsafe extern "C" fn(context: usize, file_context: usize, flags: u32) -> i64;
/// Device read callback. Non-negative values are byte counts.
pub type DeviceReadFn = unsafe extern "C" fn(
    context: usize,
    file_context: usize,
    offset: u64,
    data: *mut u8,
    len: usize,
    flags: u32,
) -> i64;
/// Device write callback. Non-negative values are byte counts.
pub type DeviceWriteFn = unsafe extern "C" fn(
    context: usize,
    file_context: usize,
    offset: u64,
    data: *const u8,
    len: usize,
    flags: u32,
) -> i64;
/// Device size callback.
pub type DeviceSizeFn = unsafe extern "C" fn(context: usize) -> u64;
/// Device synchronization callback.
pub type DeviceSyncFn = unsafe extern "C" fn(context: usize) -> i32;
/// Device readiness callback. Non-negative values are poll-event bits.
pub type DevicePollFn = unsafe extern "C" fn(
    context: usize,
    file_context: usize,
    offset: u64,
    events: u16,
    flags: u32,
) -> i64;
/// Returns one kernel event handle for an open device description.
pub type DeviceEventFn = unsafe extern "C" fn(context: usize, file_context: usize) -> usize;
/// Device-control callback. Non-negative values are successful return values.
pub type DeviceIoctlFn = unsafe extern "C" fn(
    context: usize,
    file_context: usize,
    process_id: usize,
    process_group: i32,
    session_id: i32,
    is_session_leader: u8,
    request: u64,
    value: u64,
    argument: *mut u8,
    argument_len: usize,
) -> i64;

/// Driver module descriptor.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverModule {
    /// Size of this record.
    pub size: u32,
    /// Incompatible ABI generation required by the module.
    pub abi_major: u16,
    /// Append-only ABI revision required by the module.
    pub abi_minor: u16,
    /// Reserved module behavior flags.
    pub flags: u32,
    /// UTF-8 driver name.
    pub name: AbiSlice,
    /// Opaque module context returned to callbacks.
    pub context: usize,
    /// Required initialization callback.
    pub init: Option<DriverInitFn>,
    /// Optional finalization callback.
    pub fini: Option<DriverFiniFn>,
}

/// Exact node-property requirement used by declarative matching.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverMatchProperty {
    /// Property identifier.
    pub key: ResourceKey,
    /// Exact byte value required for a match.
    pub value: AbiSlice,
}

/// Per-provider driver instance creation callback.
pub type DriverBindFn =
    unsafe extern "C" fn(context: usize, provider: u64, out_instance_context: *mut usize) -> i32;
/// Per-provider driver instance destruction callback.
pub type DriverUnbindFn =
    unsafe extern "C" fn(context: usize, provider: u64, instance_context: usize);

/// Declarative driver class registered by one module.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverClass {
    /// Size of this record.
    pub size: u32,
    /// Higher priorities are considered first.
    pub priority: i32,
    /// Required node kind, or zero for any kind.
    pub node_kind: u32,
    /// Reserved class behavior flags.
    pub flags: u32,
    /// UTF-8 class name.
    pub name: AbiSlice,
    /// Driver-defined class context.
    pub context: usize,
    /// Exact property requirements.
    pub properties: *const DriverMatchProperty,
    /// Number of property requirements.
    pub property_count: usize,
    /// Inherited resources required before binding.
    pub resources: *const ResourceKey,
    /// Number of required resources.
    pub resource_count: usize,
    /// Required instance creation callback.
    pub bind: Option<DriverBindFn>,
    /// Required instance destruction callback.
    pub unbind: Option<DriverUnbindFn>,
}

/// Device vnode callback table.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverDeviceOps {
    /// Size of this record.
    pub size: u32,
    /// Opaque device context.
    pub context: usize,
    /// Optional open callback.
    pub open: Option<DeviceOpenFn>,
    /// Optional close callback.
    pub close: Option<DeviceCloseFn>,
    /// Optional initial-offset callback.
    pub initial_offset: Option<DeviceInitialOffsetFn>,
    /// Optional read callback.
    pub read: Option<DeviceReadFn>,
    /// Optional write callback.
    pub write: Option<DeviceWriteFn>,
    /// Optional size callback.
    pub size_bytes: Option<DeviceSizeFn>,
    /// Optional synchronization callback.
    pub sync: Option<DeviceSyncFn>,
    /// Optional readiness callback.
    pub poll: Option<DevicePollFn>,
    /// Optional control callback.
    pub ioctl: Option<DeviceIoctlFn>,
    /// Optional per-open readable event callback.
    pub readable_event: Option<DeviceEventFn>,
    /// Optional per-open writable event callback.
    pub writable_event: Option<DeviceEventFn>,
    /// Optional per-open hangup event callback.
    pub hangup_event: Option<DeviceEventFn>,
}

impl DriverDeviceOps {
    const EMPTY: Self = Self {
        size: 0,
        context: 0,
        open: None,
        close: None,
        initial_offset: None,
        read: None,
        write: None,
        size_bytes: None,
        sync: None,
        poll: None,
        ioctl: None,
        readable_event: None,
        writable_event: None,
        hangup_event: None,
    };
}

/// Common prefix of every versioned service table.
#[repr(C)]
pub struct DriverServiceHeader {
    /// Size of this record.
    pub size: u32,
    /// Append-only service revision.
    pub revision: u32,
}

/// Returns one immutable kernel service table.
pub type DriverGetServiceFn = unsafe extern "C" fn(
    service: u64,
    minimum_revision: u32,
    output: *mut *const DriverServiceHeader,
) -> i32;

/// Minimal bootstrap passed to every driver module.
#[repr(C)]
pub struct DriverBootstrap {
    /// Size of this record.
    pub size: u32,
    /// Incompatible ABI generation implemented by the kernel.
    pub abi_major: u16,
    /// Append-only ABI revision implemented by the kernel.
    pub abi_minor: u16,
    /// Resolves independently versioned service tables.
    pub get_service: DriverGetServiceFn,
}

/// Zero-copy immutable data resource borrowed through a lease.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverDataResource {
    /// Size of this record.
    pub size: u32,
    /// Resource behavior flags.
    pub flags: u64,
    /// Lease that keeps the resource and provider live.
    pub lease: u64,
    /// Immutable resource bytes.
    pub data: AbiSlice,
}

/// Direct-call protocol resource borrowed through a lease.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverProtocolResource {
    /// Size of this record.
    pub size: u32,
    /// Compatible protocol revision.
    pub revision: u32,
    /// Resource behavior flags.
    pub flags: u64,
    /// Lease that keeps the resource and provider live.
    pub lease: u64,
    /// Provider-defined callback context.
    pub context: usize,
    /// Immutable protocol operation table.
    pub operations: *const u8,
    /// Number of readable operation-table bytes.
    pub operations_size: usize,
}

/// Delegated physical range borrowed through a lease.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverMemoryResource {
    /// Size of this record.
    pub size: u32,
    /// Resource behavior flags.
    pub flags: u64,
    /// Lease that authorizes mapping operations.
    pub lease: u64,
    /// Number of bytes in the delegated range.
    pub length: u64,
}

/// Tracked kernel mapping of a delegated MMIO range.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct DriverMmioMapping {
    /// Size of this record.
    pub size: u32,
    /// Reserved for future mapping flags.
    pub reserved: u32,
    /// Stable mapping handle.
    pub handle: u64,
    /// Kernel virtual address corresponding to the requested offset.
    pub address: usize,
    /// Number of mapped bytes requested by the driver.
    pub length: usize,
}

/// Allocation, synchronization, logging, and kernel utility services.
#[repr(C)]
pub struct DriverCoreApi {
    /// Common service header.
    pub header: DriverServiceHeader,
    /// Allocates uninitialized kernel heap memory.
    pub allocate: unsafe extern "C" fn(size: usize, align: usize) -> *mut u8,
    /// Allocates zeroed kernel heap memory.
    pub allocate_zeroed: unsafe extern "C" fn(size: usize, align: usize) -> *mut u8,
    /// Releases kernel heap memory with its original layout.
    pub deallocate: unsafe extern "C" fn(data: *mut u8, size: usize, align: usize) -> i32,
    /// Writes a driver message to the kernel log.
    pub log: unsafe extern "C" fn(level: u32, message: AbiSlice) -> i32,
    /// Allocates a kernel event object.
    pub event_create: unsafe extern "C" fn(output: *mut usize) -> i32,
    /// Destroys an idle kernel event object.
    pub event_destroy: unsafe extern "C" fn(event: usize) -> i32,
    /// Waits until an event is signaled.
    pub event_wait: unsafe extern "C" fn(event: usize) -> i32,
    /// Clears an event's persistent signal.
    pub event_reset: unsafe extern "C" fn(event: usize) -> i32,
    /// Signals an event and returns its selected waiter count.
    pub event_signal: unsafe extern "C" fn(event: usize) -> i64,
    /// Sleeps the current thread for at least the requested duration.
    pub sleep_ns: unsafe extern "C" fn(nanoseconds: u64),
    /// Fills a driver buffer with random bytes.
    pub random_fill: unsafe extern "C" fn(output: *mut u8, len: usize),
    /// Mixes driver-provided entropy into the random pool.
    pub random_mix: unsafe extern "C" fn(input: *const u8, len: usize),
    /// Returns the oldest readable kernel-log offset.
    pub kmsg_start: unsafe extern "C" fn() -> u64,
    /// Returns the kernel-log end offset.
    pub kmsg_end: unsafe extern "C" fn() -> u64,
    /// Reads kernel-log bytes. Negative values are error statuses.
    pub kmsg_read:
        unsafe extern "C" fn(offset: u64, output: *mut u8, len: usize, nonblocking: u8) -> i64,
    /// Appends one or more kernel-log records.
    pub kmsg_append: unsafe extern "C" fn(input: *const u8, len: usize) -> i32,
    /// Stops regular log records from being mirrored to early consoles.
    pub kmsg_disable_console_output: unsafe extern "C" fn(),
    /// Restores regular log mirroring to registered early consoles.
    pub kmsg_enable_console_output: unsafe extern "C" fn(),
}

/// Device topology and typed-resource services.
#[repr(C)]
pub struct DriverRegistryApi {
    /// Common service header.
    pub header: DriverServiceHeader,
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
    /// Publishes immutable metadata on an owned node.
    pub set_node_property:
        unsafe extern "C" fn(driver: u64, node: u64, key: ResourceKey, value: AbiSlice) -> i32,
    /// Copies immutable metadata attached directly to a node.
    pub read_node_property: unsafe extern "C" fn(
        node: u64,
        key: ResourceKey,
        output: *mut u8,
        output_len: usize,
        written: *mut usize,
    ) -> i32,
    /// Publishes immutable data on a bus.
    pub publish_data_resource: unsafe extern "C" fn(
        driver: u64,
        bus: u64,
        key: ResourceKey,
        flags: u64,
        data: AbiSlice,
        out_resource: *mut u64,
    ) -> i32,
    /// Publishes a delegated physical-memory range.
    pub publish_memory_resource: unsafe extern "C" fn(
        driver: u64,
        node: u64,
        key: ResourceKey,
        flags: u64,
        physical: u64,
        length: u64,
        out_resource: *mut u64,
    ) -> i32,
    /// Publishes a versioned direct-call protocol on a bus.
    pub publish_protocol_resource: unsafe extern "C" fn(
        driver: u64,
        bus: u64,
        key: ResourceKey,
        flags: u64,
        revision: u32,
        context: usize,
        operations: *const u8,
        operations_size: usize,
        out_resource: *mut u64,
    ) -> i32,
    /// Removes an idle resource from an owned node.
    pub remove_resource: unsafe extern "C" fn(driver: u64, node: u64, key: ResourceKey) -> i32,
    /// Acquires immutable inherited data without copying.
    pub acquire_data_resource: unsafe extern "C" fn(
        driver: u64,
        node: u64,
        key: ResourceKey,
        output: *mut DriverDataResource,
    ) -> i32,
    /// Acquires a delegated physical-memory range.
    pub acquire_memory_resource: unsafe extern "C" fn(
        driver: u64,
        node: u64,
        key: ResourceKey,
        output: *mut DriverMemoryResource,
    ) -> i32,
    /// Acquires a compatible direct-call protocol.
    pub acquire_protocol_resource: unsafe extern "C" fn(
        driver: u64,
        node: u64,
        key: ResourceKey,
        minimum_revision: u32,
        output: *mut DriverProtocolResource,
    ) -> i32,
    /// Releases a previously acquired data or protocol lease.
    pub release_resource: unsafe extern "C" fn(driver: u64, lease: u64) -> i32,
}

/// Kernel virtual-memory services used by hardware drivers.
#[repr(C)]
pub struct DriverMemoryApi {
    /// Common service header.
    pub header: DriverServiceHeader,
    /// Establishes a persistent device mapping in the kernel direct map.
    pub map_mmio: unsafe extern "C" fn(
        driver: u64,
        physical: u64,
        size: usize,
        out_address: *mut usize,
    ) -> i32,
    /// Maps a bounded range from an acquired MMIO resource.
    pub map_mmio_resource: unsafe extern "C" fn(
        driver: u64,
        lease: u64,
        offset: u64,
        size: usize,
        output: *mut DriverMmioMapping,
    ) -> i32,
    /// Releases a tracked MMIO mapping.
    pub release_mmio_mapping: unsafe extern "C" fn(driver: u64, mapping: u64) -> i32,
    /// Converts firmware physical memory covered by the HHDM.
    pub firmware_physical_to_virtual:
        unsafe extern "C" fn(physical: u64, out_address: *mut usize) -> i32,
}

/// Interrupt-controller and routed-interrupt services.
#[repr(C)]
pub struct DriverInterruptApi {
    /// Common service header.
    pub header: DriverServiceHeader,
    /// Registers an interrupt controller on an owned bus.
    pub register_interrupt_controller: unsafe extern "C" fn(
        driver: u64,
        bus: u64,
        controller: *const InterruptController,
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
    /// Routes an interrupt with a managed thread-context callback.
    pub request_threaded_interrupt: unsafe extern "C" fn(
        driver: u64,
        node: u64,
        specifier: AbiSlice,
        flags: u64,
        target_cpu: u32,
        handler: Option<InterruptHandlerFn>,
        thread_handler: Option<InterruptThreadFn>,
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
}

/// Declarative driver binding services.
#[repr(C)]
pub struct DriverBindingApi {
    /// Common service header.
    pub header: DriverServiceHeader,
    /// Registers a driver class owned by the calling module.
    pub register_class:
        unsafe extern "C" fn(driver: u64, class: *const DriverClass, out_class: *mut u64) -> i32,
    /// Unregisters an idle driver class.
    pub unregister_class: unsafe extern "C" fn(driver: u64, class: u64) -> i32,
}

/// Kernel device-frontend protocol published as a root resource.
#[repr(C)]
pub struct DriverDeviceFrontendOps {
    /// Size of this operation table.
    pub size: u32,
    /// Compatible append-only protocol revision.
    pub revision: u32,
    /// Returns the devtempfs root directory.
    pub root: unsafe extern "C" fn(context: usize, out_node: *mut u64) -> i32,
    /// Creates a driver-owned devtempfs directory.
    pub create_dir: unsafe extern "C" fn(
        context: usize,
        driver: u64,
        parent: u64,
        name: AbiSlice,
        mode: u16,
        out_node: *mut u64,
    ) -> i32,
    /// Creates a driver-owned character or block node.
    pub create_device: unsafe extern "C" fn(
        context: usize,
        driver: u64,
        parent: u64,
        name: AbiSlice,
        kind: u32,
        mode: u16,
        device: u64,
        operations: *const DriverDeviceOps,
        out_node: *mut u64,
    ) -> i32,
    /// Removes a driver-owned devtempfs node.
    pub remove_node: unsafe extern "C" fn(context: usize, driver: u64, node: u64) -> i32,
}

/// Kernel console/TTY protocol published as a root resource.
#[repr(C)]
pub struct DriverConsoleServiceOps {
    /// Size of this operation table.
    pub size: u32,
    /// Compatible append-only protocol revision.
    pub revision: u32,
    /// Returns the kernel-owned console bus.
    pub bus: unsafe extern "C" fn(context: usize, output: *mut u64) -> i32,
    /// Creates one TTY backed by driver callbacks.
    pub create_tty: unsafe extern "C" fn(
        context: usize,
        driver: u64,
        device: u64,
        parent: u64,
        name: AbiSlice,
        mode: u16,
        path: AbiSlice,
        baud: u32,
        operations: *const DriverConsoleOps,
        out_node: *mut u64,
    ) -> i32,
}

/// Static driver bootstrap.
pub static DRIVER_BOOTSTRAP: DriverBootstrap = DriverBootstrap {
    size: mem::size_of::<DriverBootstrap>() as u32,
    abi_major: DRIVER_ABI_MAJOR,
    abi_minor: DRIVER_ABI_MINOR,
    get_service: host_get_service,
};

static CORE_API: DriverCoreApi = DriverCoreApi {
    header: DriverServiceHeader {
        size: mem::size_of::<DriverCoreApi>() as u32,
        revision: SERVICE_REVISION,
    },
    allocate: host_allocate,
    allocate_zeroed: host_allocate_zeroed,
    deallocate: host_deallocate,
    log: host_log,
    event_create: host_event_create,
    event_destroy: host_event_destroy,
    event_wait: host_event_wait,
    event_reset: host_event_reset,
    event_signal: host_event_signal,
    sleep_ns: host_sleep_ns,
    random_fill: host_random_fill,
    random_mix: host_random_mix,
    kmsg_start: host_kmsg_start,
    kmsg_end: host_kmsg_end,
    kmsg_read: host_kmsg_read,
    kmsg_append: host_kmsg_append,
    kmsg_disable_console_output: host_kmsg_disable_console_output,
    kmsg_enable_console_output: host_kmsg_enable_console_output,
};

static REGISTRY_API: DriverRegistryApi = DriverRegistryApi {
    header: DriverServiceHeader {
        size: mem::size_of::<DriverRegistryApi>() as u32,
        revision: SERVICE_REVISION,
    },
    root_bus: host_root_bus,
    register_bus: host_register_bus,
    register_device: host_register_device,
    remove_node: host_remove_node,
    set_node_property: host_set_node_property,
    read_node_property: host_read_node_property,
    publish_data_resource: host_publish_data_resource,
    publish_memory_resource: host_publish_memory_resource,
    publish_protocol_resource: host_publish_protocol_resource,
    remove_resource: host_remove_resource,
    acquire_data_resource: host_acquire_data_resource,
    acquire_memory_resource: host_acquire_memory_resource,
    acquire_protocol_resource: host_acquire_protocol_resource,
    release_resource: host_release_resource,
};

static MEMORY_API: DriverMemoryApi = DriverMemoryApi {
    header: DriverServiceHeader {
        size: mem::size_of::<DriverMemoryApi>() as u32,
        revision: SERVICE_REVISION,
    },
    map_mmio: host_map_mmio,
    map_mmio_resource: host_map_mmio_resource,
    release_mmio_mapping: host_release_mmio_mapping,
    firmware_physical_to_virtual: host_physical_to_virtual,
};

static INTERRUPT_API: DriverInterruptApi = DriverInterruptApi {
    header: DriverServiceHeader {
        size: mem::size_of::<DriverInterruptApi>() as u32,
        revision: SERVICE_REVISION,
    },
    register_interrupt_controller: host_register_interrupt_controller,
    unregister_interrupt_controller: host_unregister_interrupt_controller,
    request_interrupt: host_request_interrupt,
    request_threaded_interrupt: host_request_threaded_interrupt,
    release_interrupt: host_release_interrupt,
    mask_interrupt: host_mask_interrupt,
    unmask_interrupt: host_unmask_interrupt,
    set_interrupt_affinity: host_set_interrupt_affinity,
};

static BINDING_API: DriverBindingApi = DriverBindingApi {
    header: DriverServiceHeader {
        size: mem::size_of::<DriverBindingApi>() as u32,
        revision: SERVICE_REVISION,
    },
    register_class: host_register_class,
    unregister_class: host_unregister_class,
};

static DEVICE_FRONTEND_OPS: DriverDeviceFrontendOps = DriverDeviceFrontendOps {
    size: mem::size_of::<DriverDeviceFrontendOps>() as u32,
    revision: SERVICE_REVISION,
    root: service_devfs_root,
    create_dir: service_devfs_create_dir,
    create_device: service_devfs_create_device,
    remove_node: service_devfs_remove_node,
};

static CONSOLE_SERVICE_OPS: DriverConsoleServiceOps = DriverConsoleServiceOps {
    size: mem::size_of::<DriverConsoleServiceOps>() as u32,
    revision: SERVICE_REVISION,
    bus: service_console_bus,
    create_tty: service_console_create_tty,
};

pub(crate) fn device_frontend_protocol() -> ResourceProtocol {
    // SAFETY: this immutable static operation table remains valid forever.
    unsafe {
        ResourceProtocol::new(
            SERVICE_REVISION,
            0,
            (&raw const DEVICE_FRONTEND_OPS).cast(),
            mem::size_of::<DriverDeviceFrontendOps>(),
        )
    }
}

pub(crate) fn console_service_protocol() -> ResourceProtocol {
    // SAFETY: this immutable static operation table remains valid forever.
    unsafe {
        ResourceProtocol::new(
            SERVICE_REVISION,
            0,
            (&raw const CONSOLE_SERVICE_OPS).cast(),
            mem::size_of::<DriverConsoleServiceOps>(),
        )
    }
}

struct ForeignDeviceOps {
    operations: DriverDeviceOps,
}

struct ForeignConsoleBackend {
    operations: DriverConsoleOps,
    owner: driver::CallbackOwner,
}

impl Drop for ForeignConsoleBackend {
    fn drop(&mut self) {
        if let Some(callback) = self.operations.destroy {
            if let Ok(_callback) = self.owner.acquire_cleanup() {
                // SAFETY: the cleanup guard pins the copied callback table
                // while the backend releases its driver-owned context.
                unsafe { callback(self.operations.context) };
            }
        }
    }
}

impl ConsoleBackend for ForeignConsoleBackend {
    fn open(&self) -> fs::Result<()> {
        let Some(callback) = self.operations.open else {
            return Ok(());
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        // SAFETY: the callback guard pins the owning driver.
        callback_status(unsafe { callback(self.operations.context) })
    }

    fn close(&self) {
        if let Some(callback) = self.operations.close {
            if let Ok(_callback) = self.owner.acquire_cleanup() {
                // SAFETY: the callback guard pins the owning driver.
                unsafe { callback(self.operations.context) };
            }
        }
    }

    fn try_read(&self) -> Option<u8> {
        let callback = self.operations.try_read?;
        let _callback = self.owner.acquire_control().ok()?;
        let mut byte = 0u8;
        // SAFETY: `byte` is writable for one byte and the driver is pinned.
        let status = unsafe { callback(self.operations.context, &mut byte) };
        (status == 1).then_some(byte)
    }

    fn read(&self, output: &mut [u8]) -> fs::Result<usize> {
        let Some(callback) = self.operations.read else {
            return ConsoleBackend::read_fallback(self, output);
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        let pointer = if output.is_empty() {
            ptr::null_mut()
        } else {
            output.as_mut_ptr()
        };
        // SAFETY: `output` is writable for the duration of this callback.
        let value = unsafe { callback(self.operations.context, pointer, output.len()) };
        callback_count(value, output.len())
    }

    fn write(&self, bytes: &[u8], nonblocking: bool) -> fs::Result<()> {
        let callback = self.operations.write.ok_or(fs::Error::Unsupported)?;
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        let data = if bytes.is_empty() {
            ptr::null()
        } else {
            bytes.as_ptr()
        };
        // SAFETY: `bytes` is readable for the duration of this callback.
        callback_status(unsafe {
            callback(
                self.operations.context,
                data,
                bytes.len(),
                u8::from(nonblocking),
            )
        })
    }

    fn configure(&self, settings: SerialSettings) -> fs::Result<()> {
        let Some(callback) = self.operations.configure else {
            return Ok(());
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        let settings = DriverSerialSettings {
            baud: settings.baud,
            data_bits: settings.data_bits,
            stop_bits: settings.stop_bits,
            parity: u8::from(settings.parity),
            odd_parity: u8::from(settings.odd_parity),
        };
        // SAFETY: `settings` remains readable for the duration of this callback.
        callback_status(unsafe { callback(self.operations.context, &settings) })
    }

    fn flush(&self) -> fs::Result<()> {
        let Some(callback) = self.operations.flush else {
            return Ok(());
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        // SAFETY: the callback guard pins the owning driver.
        callback_status(unsafe { callback(self.operations.context) })
    }

    fn flush_input(&self) -> fs::Result<()> {
        let Some(callback) = self.operations.flush_input else {
            return Ok(());
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        // SAFETY: the callback guard pins the owning driver.
        callback_status(unsafe { callback(self.operations.context) })
    }

    fn flush_output(&self) -> fs::Result<()> {
        let Some(callback) = self.operations.flush_output else {
            return Ok(());
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        // SAFETY: the callback guard pins the owning driver.
        callback_status(unsafe { callback(self.operations.context) })
    }

    fn send_break(&self, duration: u64) -> fs::Result<()> {
        let Some(callback) = self.operations.send_break else {
            return Ok(());
        };
        let _callback = self.owner.acquire_control().map_err(|_| fs::Error::Io)?;
        // SAFETY: the callback guard pins the owning driver.
        callback_status(unsafe { callback(self.operations.context, duration) })
    }

    fn hung_up(&self) -> bool {
        let Some(callback) = self.operations.hung_up else {
            return false;
        };
        let Ok(_callback) = self.owner.acquire_control() else {
            return true;
        };
        // SAFETY: the callback guard pins the owning driver.
        unsafe { callback(self.operations.context) > 0 }
    }

    fn writable(&self) -> bool {
        let Some(callback) = self.operations.writable else {
            return true;
        };
        let Ok(_callback) = self.owner.acquire_control() else {
            return false;
        };
        // SAFETY: the callback guard pins the owning driver.
        unsafe { callback(self.operations.context) > 0 }
    }

    fn queued_output(&self) -> usize {
        let Some(callback) = self.operations.queued_output else {
            return 0;
        };
        let Ok(_callback) = self.owner.acquire_control() else {
            return 0;
        };
        // SAFETY: the callback guard pins the owning driver.
        let value = unsafe { callback(self.operations.context) };
        usize::try_from(value.max(0)).unwrap_or(usize::MAX)
    }

    fn readable_event(&self) -> Option<&Event> {
        event_ref(self.operations.readable_event)
    }

    fn writable_event(&self) -> Option<&Event> {
        event_ref(self.operations.writable_event)
    }

    fn hangup_event(&self) -> Option<&Event> {
        event_ref(self.operations.hangup_event)
    }

    fn reset_on_last_close(&self) -> bool {
        self.operations.flags & 1 != 0
    }
}

impl DeviceNodeOps for ForeignDeviceOps {
    fn open(&self, flags: u32) -> fs::Result<usize> {
        let Some(callback) = self.operations.open else {
            return Ok(0);
        };
        let mut file_context = 0usize;
        // SAFETY: the copied callback table was validated when the node was
        // created and devtempfs pins the owning driver around this call.
        callback_status(unsafe { callback(self.operations.context, flags, &mut file_context) })?;
        Ok(file_context)
    }

    fn close(&self, file_context: usize, flags: u32) {
        let Some(callback) = self.operations.close else {
            return;
        };
        // SAFETY: devtempfs pins the owning driver around this call.
        unsafe { callback(self.operations.context, file_context, flags) };
    }

    fn initial_offset(&self, file_context: usize, flags: u32) -> fs::Result<u64> {
        let Some(callback) = self.operations.initial_offset else {
            return Ok(0);
        };
        // SAFETY: devtempfs pins the owning driver around this call.
        let value = unsafe { callback(self.operations.context, file_context, flags) };
        if value < 0 {
            return Err(status_to_fs(i32::try_from(value).unwrap_or(STATUS_IO)));
        }
        Ok(value as u64)
    }

    fn read_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        buffer: &mut [u8],
        flags: u32,
    ) -> fs::Result<usize> {
        let callback = self.operations.read.ok_or(fs::Error::Unsupported)?;
        let data = if buffer.is_empty() {
            ptr::null_mut()
        } else {
            buffer.as_mut_ptr()
        };
        // SAFETY: `buffer` is writable for `buffer.len()` and the callback is
        // pinned by devtempfs.
        callback_count(
            unsafe {
                callback(
                    self.operations.context,
                    file_context,
                    offset,
                    data,
                    buffer.len(),
                    flags,
                )
            },
            buffer.len(),
        )
    }

    fn write_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        buffer: &[u8],
        flags: u32,
    ) -> fs::Result<usize> {
        let callback = self.operations.write.ok_or(fs::Error::Unsupported)?;
        let data = if buffer.is_empty() {
            ptr::null()
        } else {
            buffer.as_ptr()
        };
        // SAFETY: `buffer` is readable for `buffer.len()` and the callback is
        // pinned by devtempfs.
        callback_count(
            unsafe {
                callback(
                    self.operations.context,
                    file_context,
                    offset,
                    data,
                    buffer.len(),
                    flags,
                )
            },
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

    fn poll(
        &self,
        file_context: usize,
        offset: u64,
        events: fs::PollEvents,
        flags: u32,
    ) -> fs::Result<fs::PollEvents> {
        let Some(callback) = self.operations.poll else {
            return Ok(fs::PollEvents::empty());
        };
        // SAFETY: devtempfs pins the owning driver around this call.
        let value = unsafe {
            callback(
                self.operations.context,
                file_context,
                offset,
                events.bits(),
                flags,
            )
        };
        if value < 0 {
            return Err(status_to_fs(i32::try_from(value).unwrap_or(STATUS_IO)));
        }
        let bits = u16::try_from(value).map_err(|_| fs::Error::Io)?;
        fs::PollEvents::from_bits(bits).ok_or(fs::Error::Io)
    }

    fn poll_events<'a>(
        &'a self,
        file_context: usize,
        events: fs::PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        let mut complete = true;
        let mut append = |requested: bool, callback: Option<DeviceEventFn>| {
            if !requested {
                return;
            }
            let Some(callback) = callback else {
                complete = false;
                return;
            };
            // SAFETY: devtempfs pins the owning driver around this call.
            let handle = unsafe { callback(self.operations.context, file_context) };
            if let Some(event) = event_ref(handle) {
                output.push(event);
            } else {
                complete = false;
            }
        };
        append(
            events.intersects(fs::PollEvents::IN | fs::PollEvents::RDNORM),
            self.operations.readable_event,
        );
        append(
            events.intersects(fs::PollEvents::OUT | fs::PollEvents::WRNORM),
            self.operations.writable_event,
        );
        append(
            events.contains(fs::PollEvents::HUP),
            self.operations.hangup_event,
        );
        complete
    }

    fn ioctl(
        &self,
        file_context: usize,
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
                file_context,
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

unsafe extern "C" fn host_get_service(
    service: u64,
    minimum_revision: u32,
    output: *mut *const DriverServiceHeader,
) -> i32 {
    if minimum_revision > SERVICE_REVISION {
        return STATUS_UNSUPPORTED;
    }
    let service = match service {
        SERVICE_CORE => (&raw const CORE_API).cast::<DriverServiceHeader>(),
        SERVICE_REGISTRY => (&raw const REGISTRY_API).cast::<DriverServiceHeader>(),
        SERVICE_MEMORY => (&raw const MEMORY_API).cast::<DriverServiceHeader>(),
        SERVICE_INTERRUPT => (&raw const INTERRUPT_API).cast::<DriverServiceHeader>(),
        SERVICE_BINDING => (&raw const BINDING_API).cast::<DriverServiceHeader>(),
        _ => return STATUS_NOT_FOUND,
    };
    // SAFETY: the bootstrap contract requires a writable aligned output.
    match unsafe { write_out(output, service) } {
        Ok(()) => STATUS_OK,
        Err(error) => status(error),
    }
}

/// Loads a module descriptor through the exported C ABI.
///
/// # Safety
///
/// `module` and `out_driver` must be valid pointers following the driver
/// interface. Module callback code must remain mapped until unload succeeds.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn roanix_driver_load(
    module: *const DriverModule,
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
pub extern "C" fn roanix_driver_unload(driver_id: u64) -> i32 {
    match driver::unload(DriverId::new(driver_id)) {
        Ok(()) => STATUS_OK,
        Err(error) => status(error),
    }
}

unsafe extern "C" fn host_register_class(
    driver_id: u64,
    class: *const DriverClass,
    out_class: *mut u64,
) -> i32 {
    if !is_aligned(out_class) {
        return STATUS_INVALID_ARGUMENT;
    }
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        // SAFETY: required by the binding service ABI.
        let registration = unsafe { read_driver_class(class)? };
        let id = binding::register_class(owner, registration)?;
        // SAFETY: alignment was validated before class registration.
        unsafe { out_class.write(id.get()) };
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_unregister_class(driver_id: u64, class: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| binding::unregister_class(owner, DriverClassId::new(class)));
    result.map_or_else(status, |_| STATUS_OK)
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
        // SAFETY: required by the host function ABI.
        let name = unsafe { abi_name(name)? };
        let bus = super::register_bus(owner, parent_node, name)?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_bus, bus.node().get()) } {
            let _ = super::remove_node(owner, bus.node());
            return Err(error);
        }
        Ok(())
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
        // SAFETY: required by the host function ABI.
        let name = unsafe { abi_name(name)? };
        let device = super::register_device(owner, parent_node, name)?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_device, device.node().get()) } {
            let _ = super::remove_node(owner, device.node());
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_remove_node(driver_id: u64, node: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| super::remove_node(owner, DeviceNodeId::from_raw(node)));
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_set_node_property(
    driver_id: u64,
    node: u64,
    key: ResourceKey,
    value: AbiSlice,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        // SAFETY: required by the host function ABI.
        let value: Arc<[u8]> = Arc::from(unsafe { value.as_slice()? });
        super::set_property(owner, DeviceNodeId::from_raw(node), key, value)
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_read_node_property(
    node: u64,
    key: ResourceKey,
    output: *mut u8,
    output_len: usize,
    written: *mut usize,
) -> i32 {
    let result = (|| {
        let value = super::property(DeviceNodeId::from_raw(node), key)?;
        // SAFETY: required by the registry service ABI.
        unsafe { write_out(written, value.len())? };
        if output_len < value.len() {
            return Err(Error::NoSpace);
        }
        // SAFETY: required by the registry service ABI.
        let output = unsafe { output_slice(output, output_len)? };
        output[..value.len()].copy_from_slice(&value);
        Ok(())
    })();
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
        let node = DeviceNodeId::from_raw(bus);
        let info = tree::node_info(node)?;
        if info.owner != owner {
            return Err(Error::PermissionDenied);
        }
        // SAFETY: required by the host function ABI.
        let bytes: Arc<[u8]> = Arc::from(unsafe { data.as_slice()? });
        let id = super::publish_resource(
            owner,
            node,
            key,
            ResourceFlags::from_bits(flags),
            ResourceValue::Data(bytes),
        )?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_resource, id.get()) } {
            let _ = super::remove_resource(owner, node, key);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_publish_memory_resource(
    driver_id: u64,
    node: u64,
    key: ResourceKey,
    flags: u64,
    physical: u64,
    length: u64,
    out_resource: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let node = DeviceNodeId::from_raw(node);
        let info = tree::node_info(node)?;
        if info.owner != owner {
            return Err(Error::PermissionDenied);
        }
        let flags = ResourceFlags::from_bits(flags);
        if !flags.contains(ResourceFlags::MMIO) {
            return Err(Error::InvalidArgument);
        }
        let region = MemoryRegion::new(physical, length)?;
        let id = super::publish_resource(owner, node, key, flags, ResourceValue::Memory(region))?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_resource, id.get()) } {
            let _ = super::remove_resource(owner, node, key);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_publish_protocol_resource(
    driver_id: u64,
    bus: u64,
    key: ResourceKey,
    flags: u64,
    revision: u32,
    context: usize,
    operations: *const u8,
    operations_size: usize,
    out_resource: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let node = DeviceNodeId::from_raw(bus);
        let info = tree::node_info(node)?;
        if info.owner != owner {
            return Err(Error::PermissionDenied);
        }
        if revision == 0
            || operations_size < mem::size_of::<DriverServiceHeader>()
            || operations.is_null()
            || !(operations as usize).is_multiple_of(mem::align_of::<usize>())
        {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the validated operation table covers this common prefix.
        let header = unsafe { (operations as *const DriverServiceHeader).read() };
        if header.revision != revision
            || header.size as usize > operations_size
            || header.size < mem::size_of::<DriverServiceHeader>() as u32
        {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: publication requires an immutable, aligned operation table
        // that remains mapped until every lease has been released.
        let protocol =
            unsafe { ResourceProtocol::new(revision, context, operations, operations_size) };
        let id = super::publish_resource(
            owner,
            node,
            key,
            ResourceFlags::from_bits(flags),
            ResourceValue::Protocol(protocol),
        )?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_resource, id.get()) } {
            let _ = super::remove_resource(owner, node, key);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_remove_resource(driver_id: u64, node: u64, key: ResourceKey) -> i32 {
    let owner = DriverId::new(driver_id);
    let result = driver::authorize(owner)
        .and_then(|()| super::remove_resource(owner, DeviceNodeId::from_raw(node), key));
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_acquire_data_resource(
    driver_id: u64,
    node: u64,
    key: ResourceKey,
    output: *mut DriverDataResource,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let (lease, resource) = super::acquire_resource(owner, DeviceNodeId::from_raw(node), key)?;
        let data = match resource.data() {
            Ok(data) => data,
            Err(error) => {
                let _ = super::release_resource(owner, lease);
                return Err(error);
            }
        };
        let record = DriverDataResource {
            size: mem::size_of::<DriverDataResource>() as u32,
            flags: resource.flags().bits(),
            lease: lease.get(),
            data: AbiSlice {
                data: data.as_ptr(),
                len: data.len(),
            },
        };
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(output, record) } {
            let _ = super::release_resource(owner, lease);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_acquire_memory_resource(
    driver_id: u64,
    node: u64,
    key: ResourceKey,
    output: *mut DriverMemoryResource,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let (lease, resource) = super::acquire_resource(owner, DeviceNodeId::from_raw(node), key)?;
        let region = match resource.memory() {
            Ok(region) => region,
            Err(error) => {
                let _ = super::release_resource(owner, lease);
                return Err(error);
            }
        };
        let record = DriverMemoryResource {
            size: mem::size_of::<DriverMemoryResource>() as u32,
            flags: resource.flags().bits(),
            lease: lease.get(),
            length: region.size(),
        };
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(output, record) } {
            let _ = super::release_resource(owner, lease);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_acquire_protocol_resource(
    driver_id: u64,
    node: u64,
    key: ResourceKey,
    minimum_revision: u32,
    output: *mut DriverProtocolResource,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let (lease, resource) = super::acquire_resource(owner, DeviceNodeId::from_raw(node), key)?;
        let protocol = match resource.protocol(minimum_revision) {
            Ok(protocol) => protocol,
            Err(error) => {
                let _ = super::release_resource(owner, lease);
                return Err(error);
            }
        };
        let record = DriverProtocolResource {
            size: mem::size_of::<DriverProtocolResource>() as u32,
            revision: protocol.revision(),
            flags: resource.flags().bits(),
            lease: lease.get(),
            context: protocol.context(),
            operations: protocol.operations(),
            operations_size: protocol.operations_size(),
        };
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(output, record) } {
            let _ = super::release_resource(owner, lease);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_release_resource(driver_id: u64, lease: u64) -> i32 {
    let owner = DriverId::new(driver_id);
    let lease = ResourceLeaseId::new(lease);
    let result = (|| {
        let state = mmio_registry().state.lock();
        if mmio_lease_in_use_locked(&state, owner, lease) {
            return Err(Error::Busy);
        }
        let result = super::tree::release_resource_cleanup(owner, lease);
        drop(state);
        result
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
        if let Err(error) = unsafe { write_out(out_node, node.get()) } {
            let _ = filesystem.remove_node(owner, node);
            return Err(error);
        }
        Ok(())
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
    operations: *const DriverDeviceOps,
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
        if let Err(error) = unsafe { write_out(out_node, node.get()) } {
            let _ = filesystem.remove_node(owner, node);
            return Err(error);
        }
        Ok(())
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
    controller: *const InterruptController,
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
        if let Err(error) = unsafe { write_out(out_controller, id.get()) } {
            let _ = interrupt::unregister_controller(owner, id);
            return Err(error);
        }
        Ok(())
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
            Some(handler),
            None,
            context,
        )?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_interrupt, id.get()) } {
            let _ = interrupt::release_interrupt(owner, id);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_request_threaded_interrupt(
    driver_id: u64,
    node: u64,
    specifier: AbiSlice,
    flags: u64,
    target_cpu: u32,
    handler: Option<InterruptHandlerFn>,
    thread_handler: Option<InterruptThreadFn>,
    context: usize,
    out_interrupt: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let thread_handler = thread_handler.ok_or(Error::InvalidArgument)?;
        // SAFETY: required by the host function ABI.
        let specifier = unsafe { specifier.as_slice()? };
        let id = interrupt::request_interrupt(
            owner,
            DeviceNodeId::from_raw(node),
            specifier,
            flags,
            target_cpu,
            handler,
            Some(thread_handler),
            context,
        )?;
        // SAFETY: required by the host function ABI.
        if let Err(error) = unsafe { write_out(out_interrupt, id.get()) } {
            let _ = interrupt::release_interrupt(owner, id);
            return Err(error);
        }
        Ok(())
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
    if !is_aligned(out_address) {
        return STATUS_INVALID_ARGUMENT;
    }
    let result = (|| {
        let owner = DriverId::new(driver_id);
        let _owner = driver::mutation_guard(owner)?;
        let (address, pages) = mmio_range(physical, size)?;
        let mut state = mmio_registry().state.lock();
        let mapped = ensure_mmio_pages(&mut state, &pages)?;
        for page in pages {
            let entry = state.pages.entry(page).or_insert(MmioPageRecord {
                references: 0,
                mapped_by_devkit: mapped.contains(&page),
                permanent: true,
            });
            entry.permanent = true;
        }
        // SAFETY: required by the host function ABI.
        unsafe { write_out(out_address, address) }
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_map_mmio_resource(
    driver_id: u64,
    lease: u64,
    offset: u64,
    size: usize,
    output: *mut DriverMmioMapping,
) -> i32 {
    if !is_aligned(output) {
        return STATUS_INVALID_ARGUMENT;
    }
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let registry = mmio_registry();
        let mut state = registry.state.lock();
        let resource = super::leased_resource(owner, ResourceLeaseId::new(lease))?;
        if !resource.flags().contains(ResourceFlags::MMIO) {
            return Err(Error::WrongKind);
        }
        let region = resource.memory()?;
        let size_u64 = u64::try_from(size).map_err(|_| Error::InvalidArgument)?;
        let end = offset
            .checked_add(size_u64)
            .filter(|end| *end <= region.size())
            .ok_or(Error::InvalidArgument)?;
        let _ = end;
        let physical = region
            .physical()
            .checked_add(offset)
            .ok_or(Error::InvalidArgument)?;
        let (address, pages) = mmio_range(physical, size)?;
        let next_mapping = state
            .next_mapping
            .checked_add(1)
            .filter(|next| *next != 0)
            .ok_or(Error::NoSpace)?;
        if pages.iter().any(|page| {
            state
                .pages
                .get(page)
                .is_some_and(|entry| entry.references == u64::MAX)
        }) {
            return Err(Error::NoSpace);
        }
        let mapped = ensure_mmio_pages(&mut state, &pages)?;
        for page in &pages {
            let entry = state.pages.entry(*page).or_insert(MmioPageRecord {
                references: 0,
                mapped_by_devkit: mapped.contains(page),
                permanent: false,
            });
            entry.references += 1;
        }
        let handle = state.next_mapping;
        state.next_mapping = next_mapping;
        state.mappings.insert(
            handle,
            MmioMappingRecord {
                owner,
                source_lease: Some(ResourceLeaseId::new(lease)),
                pages,
            },
        );
        let mapping = DriverMmioMapping {
            size: mem::size_of::<DriverMmioMapping>() as u32,
            reserved: 0,
            handle,
            address,
            length: size,
        };
        // SAFETY: alignment was validated before creating the mapping.
        unsafe { output.write(mapping) };
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_release_mmio_mapping(driver_id: u64, mapping: u64) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        let _owner = driver::cleanup_guard(owner)?;
        release_mmio_mapping_locked(&mut mmio_registry().state.lock(), owner, mapping)
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

fn mmio_range(physical: u64, size: usize) -> super::Result<(usize, Box<[crate::mem::PhysAddr]>)> {
    if size == 0 {
        return Err(Error::InvalidArgument);
    }
    let size = u64::try_from(size).map_err(|_| Error::InvalidArgument)?;
    let end = physical.checked_add(size).ok_or(Error::InvalidArgument)?;
    let maximum_physical = u64::MAX
        .checked_sub(crate::mem::hhdm_offset())
        .ok_or(Error::InvalidArgument)?;
    if physical > maximum_physical
        || end
            .checked_sub(1)
            .is_none_or(|last| last > maximum_physical)
    {
        return Err(Error::InvalidArgument);
    }
    let page_mask = crate::mem::PAGE_SIZE - 1;
    let end_page = if end & page_mask == 0 {
        end
    } else {
        end.checked_add(crate::mem::PAGE_SIZE - (end & page_mask))
            .ok_or(Error::InvalidArgument)?
    };
    let mut page = crate::mem::PhysAddr::new(physical).align_down();
    let end_page = crate::mem::PhysAddr::new(end_page);
    let mut pages = Vec::new();
    while page < end_page {
        pages.push(page);
        page = page
            .checked_add(crate::mem::PAGE_SIZE)
            .ok_or(Error::InvalidArgument)?;
    }
    let address = physical
        .checked_add(crate::mem::hhdm_offset())
        .ok_or(Error::InvalidArgument)?;
    let address = usize::try_from(address).map_err(|_| Error::InvalidArgument)?;
    Ok((address, pages.into_boxed_slice()))
}

fn ensure_mmio_pages(
    state: &mut MmioRegistryState,
    pages: &[crate::mem::PhysAddr],
) -> super::Result<Vec<crate::mem::PhysAddr>> {
    let root = crate::arch::paging::active_root();
    let mut missing = Vec::new();
    for page in pages {
        let virtual_page = crate::mem::phys_to_virt(*page);
        // SAFETY: the MMIO registry serializes inspection and mutation of
        // these kernel direct-map leaves.
        match unsafe { crate::arch::paging::translate(root, virtual_page) } {
            Some(mapped) if mapped == *page => {}
            Some(_) => return Err(Error::AlreadyExists),
            None if state.pages.contains_key(page) => return Err(Error::Busy),
            None => missing.push(*page),
        }
    }

    let flags = crate::mem::VmFlags::READ
        | crate::mem::VmFlags::WRITE
        | crate::mem::VmFlags::GLOBAL
        | crate::mem::VmFlags::DEVICE;
    let mut installed = Vec::new();
    for page in &missing {
        let virtual_page = crate::mem::phys_to_virt(*page);
        // SAFETY: the registry lock excludes concurrent MMIO leaf changes and
        // the HHDM address is uniquely determined by this physical page.
        if unsafe { crate::arch::paging::map_page(root, virtual_page, *page, flags) }.is_err() {
            for rollback in installed {
                let virtual_page = crate::mem::phys_to_virt(rollback);
                // SAFETY: each rollback page was installed above while this
                // registry lock remained held.
                let _ = unsafe { crate::arch::paging::unmap_page(root, virtual_page) };
            }
            if !missing.is_empty() {
                crate::mem::synchronize_kernel_mappings();
            }
            return Err(Error::OutOfMemory);
        }
        installed.push(*page);
    }
    Ok(missing)
}

fn release_mmio_mapping_locked(
    state: &mut MmioRegistryState,
    owner: DriverId,
    mapping: u64,
) -> super::Result<()> {
    let record = state.mappings.get(&mapping).ok_or(Error::NotFound)?;
    if record.owner != owner {
        return Err(Error::PermissionDenied);
    }
    let record = state
        .mappings
        .remove(&mapping)
        .expect("dev: MMIO mapping disappeared while locked");
    let mut unmap = Vec::new();
    for page in record.pages.iter().copied() {
        let (remove, mapped_by_devkit) = {
            let entry = state
                .pages
                .get_mut(&page)
                .expect("dev: MMIO page record missing");
            entry.references = entry
                .references
                .checked_sub(1)
                .expect("dev: MMIO page reference underflow");
            (
                entry.references == 0 && !entry.permanent,
                entry.mapped_by_devkit,
            )
        };
        if remove {
            if mapped_by_devkit {
                unmap.push(page);
            }
            state.pages.remove(&page);
        }
    }

    let mut unmap_failed = false;
    if !unmap.is_empty() {
        let root = crate::arch::paging::active_root();
        for page in unmap {
            let virtual_page = crate::mem::phys_to_virt(page);
            // SAFETY: the registry owns this leaf, no live mapping references
            // remain, and the registry lock excludes concurrent reuse.
            if unsafe { crate::arch::paging::unmap_page(root, virtual_page) }.is_err() {
                unmap_failed = true;
            }
        }
        crate::mem::synchronize_kernel_mappings();
    }
    if unmap_failed {
        Err(Error::Busy)
    } else {
        Ok(())
    }
}

fn mmio_registry() -> &'static MmioRegistry {
    MMIO_REGISTRY.call_once(|| MmioRegistry {
        state: Mutex::new(MmioRegistryState {
            next_mapping: 1,
            pages: BTreeMap::new(),
            mappings: BTreeMap::new(),
        }),
    })
}

fn mmio_lease_in_use_locked(
    state: &MmioRegistryState,
    owner: DriverId,
    lease: ResourceLeaseId,
) -> bool {
    state
        .mappings
        .values()
        .any(|mapping| mapping.owner == owner && mapping.source_lease == Some(lease))
}

pub(crate) fn remove_driver_mappings(owner: DriverId) {
    let registry = mmio_registry();
    let mut state = registry.state.lock();
    let mappings: Vec<_> = state
        .mappings
        .iter()
        .filter_map(|(id, mapping)| (mapping.owner == owner).then_some(*id))
        .collect();
    for mapping in mappings {
        if let Err(error) = release_mmio_mapping_locked(&mut state, owner, mapping) {
            log::error!(
                "dev: failed to release MMIO mapping {mapping} for driver {}: {error}",
                owner.get()
            );
        }
    }
}

unsafe extern "C" fn host_physical_to_virtual(physical: u64, out_address: *mut usize) -> i32 {
    let result = physical
        .checked_add(crate::mem::hhdm_offset())
        .ok_or(Error::InvalidArgument)
        .and_then(|address| usize::try_from(address).map_err(|_| Error::InvalidArgument))
        .and_then(|address| {
            // SAFETY: the host interface requires a writable aligned output pointer.
            unsafe { write_out(out_address, address) }
        });
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_console_bus(output: *mut u64) -> i32 {
    let result = super::platform_buses().and_then(|buses| {
        // SAFETY: the host interface requires a writable aligned output pointer.
        unsafe { write_out(output, buses.console.node().get()) }
    });
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn host_console_create_tty(
    driver_id: u64,
    device: u64,
    parent: u64,
    name: AbiSlice,
    mode: u16,
    path: AbiSlice,
    baud: u32,
    operations: *const DriverConsoleOps,
    out_node: *mut u64,
) -> i32 {
    let result = (|| {
        let owner = DriverId::new(driver_id);
        driver::authorize(owner)?;
        let device = DeviceNodeId::from_raw(device);
        let info = tree::node_info(device)?;
        if info.owner != owner || info.kind != NodeKind::Device {
            return Err(Error::PermissionDenied);
        }
        let filesystem = devtempfs::global().map_err(|_| Error::Filesystem)?;
        let parent = DevNodeId::from_raw(parent);
        let _parent =
            driver::parent_guard(owner, filesystem.owner(parent).map_err(fs_to_device_error)?)?;
        // SAFETY: required by the host function interface.
        let name = unsafe { name.as_slice()? };
        // SAFETY: required by the host function interface.
        let path = unsafe { path.as_slice()? };
        let path = str::from_utf8(path).map_err(|_| Error::InvalidArgument)?;
        if path.is_empty() || !path.starts_with('/') {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: required by the host function interface.
        let operations = unsafe { read_console_operations(operations)? };
        let backend: Arc<dyn ConsoleBackend> = Arc::new(ForeignConsoleBackend {
            operations,
            owner: driver::callback_owner(owner)?,
        });
        let tty: Arc<dyn DeviceNodeOps> = Tty::new(backend, Box::<str>::from(path), baud)?;
        let node = filesystem
            .create_device(
                owner,
                parent,
                name,
                DeviceNodeKind::Character,
                mode,
                device,
                tty,
            )
            .map_err(fs_to_device_error)?;
        // SAFETY: required by the host function interface.
        if let Err(error) = unsafe { write_out(out_node, node.get()) } {
            let _ = filesystem.remove_node(owner, node);
            return Err(error);
        }
        Ok(())
    })();
    result.map_or_else(status, |_| STATUS_OK)
}

unsafe extern "C" fn service_devfs_root(context: usize, output: *mut u64) -> i32 {
    let _ = context;
    // SAFETY: forwarded from the device-frontend protocol contract.
    unsafe { host_devfs_root(output) }
}

unsafe extern "C" fn service_devfs_create_dir(
    context: usize,
    driver: u64,
    parent: u64,
    name: AbiSlice,
    mode: u16,
    output: *mut u64,
) -> i32 {
    let _ = context;
    // SAFETY: forwarded from the device-frontend protocol contract.
    unsafe { host_devfs_create_dir(driver, parent, name, mode, output) }
}

unsafe extern "C" fn service_devfs_create_device(
    context: usize,
    driver: u64,
    parent: u64,
    name: AbiSlice,
    kind: u32,
    mode: u16,
    device: u64,
    operations: *const DriverDeviceOps,
    output: *mut u64,
) -> i32 {
    let _ = context;
    // SAFETY: forwarded from the device-frontend protocol contract.
    unsafe {
        host_devfs_create_device(driver, parent, name, kind, mode, device, operations, output)
    }
}

unsafe extern "C" fn service_devfs_remove_node(context: usize, driver: u64, node: u64) -> i32 {
    let _ = context;
    // SAFETY: forwarded from the device-frontend protocol contract.
    unsafe { host_devfs_remove_node(driver, node) }
}

unsafe extern "C" fn service_console_bus(context: usize, output: *mut u64) -> i32 {
    let _ = context;
    // SAFETY: forwarded from the console-service protocol contract.
    unsafe { host_console_bus(output) }
}

unsafe extern "C" fn service_console_create_tty(
    context: usize,
    driver: u64,
    device: u64,
    parent: u64,
    name: AbiSlice,
    mode: u16,
    path: AbiSlice,
    baud: u32,
    operations: *const DriverConsoleOps,
    output: *mut u64,
) -> i32 {
    let _ = context;
    // SAFETY: forwarded from the console-service protocol contract.
    unsafe {
        host_console_create_tty(
            driver, device, parent, name, mode, path, baud, operations, output,
        )
    }
}

unsafe extern "C" fn host_event_create(output: *mut usize) -> i32 {
    let event = Box::into_raw(Box::new(Event::new())) as usize;
    // SAFETY: the host interface requires a writable aligned output pointer.
    if let Err(error) = unsafe { write_out(output, event) } {
        // SAFETY: `event` was just allocated above and has not escaped.
        drop(unsafe { Box::from_raw(event as *mut Event) });
        return status(error);
    }
    STATUS_OK
}

unsafe extern "C" fn host_event_destroy(event: usize) -> i32 {
    if event == 0 || !event.is_multiple_of(mem::align_of::<Event>()) {
        return STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: the driver contract transfers exactly one idle event allocation
    // created by `host_event_create`.
    drop(unsafe { Box::from_raw(event as *mut Event) });
    STATUS_OK
}

unsafe extern "C" fn host_event_wait(event: usize) -> i32 {
    let Some(event) = event_ref(event) else {
        return STATUS_INVALID_ARGUMENT;
    };
    event.wait();
    STATUS_OK
}

unsafe extern "C" fn host_event_reset(event: usize) -> i32 {
    let Some(event) = event_ref(event) else {
        return STATUS_INVALID_ARGUMENT;
    };
    i32::from(event.reset())
}

unsafe extern "C" fn host_event_signal(event: usize) -> i64 {
    let Some(event) = event_ref(event) else {
        return i64::from(STATUS_INVALID_ARGUMENT);
    };
    i64::try_from(event.signal()).unwrap_or(i64::MAX)
}

unsafe extern "C" fn host_sleep_ns(nanoseconds: u64) {
    clock::sleep(Duration::from_nanos(nanoseconds));
}

unsafe extern "C" fn host_random_fill(output: *mut u8, len: usize) {
    // SAFETY: the driver contract requires a writable output range.
    if let Ok(output) = unsafe { output_slice(output, len) } {
        random::fill_bytes(output);
    }
}

unsafe extern "C" fn host_random_mix(input: *const u8, len: usize) {
    // SAFETY: the driver contract requires a readable input range.
    let input = unsafe { AbiSlice { data: input, len }.as_slice() };
    if let Ok(input) = input {
        random::mix_bytes(input);
    }
}

unsafe extern "C" fn host_kmsg_start() -> u64 {
    debug::log_start_offset()
}

unsafe extern "C" fn host_kmsg_end() -> u64 {
    debug::log_end_offset()
}

unsafe extern "C" fn host_kmsg_read(
    offset: u64,
    output: *mut u8,
    len: usize,
    nonblocking: u8,
) -> i64 {
    // SAFETY: the driver contract requires a writable output range.
    let Ok(output) = (unsafe { output_slice(output, len) }) else {
        return i64::from(STATUS_INVALID_ARGUMENT);
    };
    match debug::read_log(offset, output, nonblocking != 0) {
        Ok(read) => i64::try_from(read).unwrap_or(i64::MAX),
        Err(debug::LogReadError::Overrun) => i64::from(STATUS_IO),
        Err(debug::LogReadError::WouldBlock) => i64::from(STATUS_WOULD_BLOCK),
    }
}

unsafe extern "C" fn host_kmsg_append(input: *const u8, len: usize) -> i32 {
    // SAFETY: the driver contract requires a readable input range.
    let input = unsafe { AbiSlice { data: input, len }.as_slice() };
    match input {
        Ok(input) => {
            debug::append_kernel_message(input);
            STATUS_OK
        }
        Err(error) => status(error),
    }
}

unsafe extern "C" fn host_kmsg_disable_console_output() {
    debug::disable_regular_sink_output();
}

unsafe extern "C" fn host_kmsg_enable_console_output() {
    debug::enable_regular_sink_output();
}

fn event_ref(event: usize) -> Option<&'static Event> {
    if event == 0 || !event.is_multiple_of(mem::align_of::<Event>()) {
        return None;
    }
    // SAFETY: event handles are live Box allocations owned by the calling
    // in-tree driver until `event_destroy`.
    Some(unsafe { &*(event as *const Event) })
}

unsafe fn abi_name<'a>(name: AbiSlice) -> super::Result<&'a str> {
    // SAFETY: forwarded from this function's caller.
    let name = unsafe { name.as_slice()? };
    str::from_utf8(name).map_err(|_| Error::InvalidArgument)
}

unsafe fn read_console_operations(
    operations: *const DriverConsoleOps,
) -> super::Result<DriverConsoleOps> {
    if !is_aligned(operations) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the interface guarantees that the size field is readable.
    let size = unsafe { core::ptr::addr_of!((*operations).size).read() } as usize;
    let minimum = mem::offset_of!(DriverConsoleOps, read);
    if size < minimum {
        return Err(Error::InvalidArgument);
    }
    let mut copied = DriverConsoleOps::EMPTY;
    // SAFETY: both records are valid for the copied prefix, and `copied`
    // supplies zero defaults for append-only fields absent from older drivers.
    unsafe {
        ptr::copy_nonoverlapping(
            operations.cast::<u8>(),
            (&raw mut copied).cast::<u8>(),
            size.min(mem::size_of::<DriverConsoleOps>()),
        );
    }
    if (copied.try_read.is_none() && copied.read.is_none()) || copied.write.is_none() {
        return Err(Error::InvalidArgument);
    }
    for event in [
        copied.readable_event,
        copied.writable_event,
        copied.hangup_event,
    ] {
        if event != 0 && event_ref(event).is_none() {
            return Err(Error::InvalidArgument);
        }
    }
    Ok(copied)
}

unsafe fn output_slice<'a>(output: *mut u8, len: usize) -> super::Result<&'a mut [u8]> {
    if len == 0 {
        return Ok(&mut []);
    }
    if output.is_null() {
        return Err(Error::InvalidArgument);
    }
    if len > isize::MAX as usize || (output as usize).checked_add(len).is_none() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the host ABI requires a writable range of `len` bytes.
    Ok(unsafe { slice::from_raw_parts_mut(output, len) })
}

unsafe fn abi_read_array<'a, T>(data: *const T, len: usize) -> super::Result<&'a [T]> {
    if len == 0 {
        return Ok(&[]);
    }
    if !is_aligned(data) {
        return Err(Error::InvalidArgument);
    }
    let bytes = len
        .checked_mul(mem::size_of::<T>())
        .filter(|bytes| *bytes <= isize::MAX as usize)
        .ok_or(Error::InvalidArgument)?;
    if (data as usize).checked_add(bytes).is_none() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the caller guarantees a readable array; alignment and range
    // arithmetic were validated above.
    Ok(unsafe { slice::from_raw_parts(data, len) })
}

unsafe fn write_out<T>(output: *mut T, value: T) -> super::Result<()> {
    if !is_aligned(output) {
        return Err(Error::InvalidArgument);
    }
    if (output as usize).checked_add(mem::size_of::<T>()).is_none() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the host ABI requires a writable, aligned output pointer.
    unsafe { output.write(value) };
    Ok(())
}

unsafe fn read_operations(operations: *const DriverDeviceOps) -> super::Result<DriverDeviceOps> {
    if !is_aligned(operations) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the interface guarantees that the size field is readable. No
    // reference to the potentially shorter foreign record is created.
    let size = unsafe { core::ptr::addr_of!((*operations).size).read() } as usize;
    let minimum = mem::offset_of!(DriverDeviceOps, readable_event);
    if size < minimum {
        return Err(Error::InvalidArgument);
    }
    let mut copied = DriverDeviceOps::EMPTY;
    // SAFETY: both records cover the copied prefix and append-only fields that
    // are absent from older drivers retain initialized null defaults.
    unsafe {
        ptr::copy_nonoverlapping(
            operations.cast::<u8>(),
            (&raw mut copied).cast::<u8>(),
            size.min(mem::size_of::<DriverDeviceOps>()),
        );
    }
    Ok(copied)
}

unsafe fn read_interrupt_controller(
    controller: *const InterruptController,
) -> super::Result<InterruptController> {
    if !is_aligned(controller) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the interface guarantees the size field is readable.
    let size = unsafe { core::ptr::addr_of!((*controller).size).read() } as usize;
    if size < mem::size_of::<InterruptController>() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the validated record size covers the complete callback table.
    Ok(unsafe { controller.read() })
}

unsafe fn read_driver_class(
    class: *const DriverClass,
) -> super::Result<binding::ClassRegistration> {
    const MAX_MATCH_ITEMS: usize = 256;

    if !is_aligned(class) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the binding ABI requires the fixed size field to be readable.
    let size = unsafe { core::ptr::addr_of!((*class).size).read() } as usize;
    if size < mem::size_of::<DriverClass>() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the validated record covers the complete class descriptor.
    let class = unsafe { class.read() };
    if class.flags != 0
        || !matches!(class.node_kind, 0..=2)
        || class.property_count > MAX_MATCH_ITEMS
        || class.resource_count > MAX_MATCH_ITEMS
    {
        return Err(Error::InvalidArgument);
    }
    let bind = class.bind.ok_or(Error::InvalidArgument)?;
    let unbind = class.unbind.ok_or(Error::InvalidArgument)?;
    // SAFETY: required by the binding ABI.
    let name = unsafe { abi_name(class.name)? };
    if name.is_empty() || name.len() > 255 {
        return Err(Error::InvalidArgument);
    }

    // SAFETY: the class ABI requires this readable array for registration.
    let properties = unsafe { abi_read_array(class.properties, class.property_count)? };
    let mut copied_properties = Vec::with_capacity(properties.len());
    for property in properties {
        // SAFETY: each property value follows the same borrowed-slice ABI.
        let value: Arc<[u8]> = Arc::from(unsafe { property.value.as_slice()? });
        copied_properties.push(binding::MatchProperty {
            key: property.key,
            value,
        });
    }

    // SAFETY: the class ABI requires this readable resource-key array.
    let resources = unsafe { abi_read_array(class.resources, class.resource_count)? };

    Ok(binding::ClassRegistration {
        name: Box::<str>::from(name),
        priority: class.priority,
        node_kind: class.node_kind,
        context: class.context,
        properties: copied_properties.into_boxed_slice(),
        resources: Box::from(resources),
        bind,
        unbind,
    })
}

fn is_aligned<T>(pointer: *const T) -> bool {
    !pointer.is_null() && (pointer as usize) % mem::align_of::<T>() == 0
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
        STATUS_BUSY | STATUS_DEADLOCK => fs::Error::Busy,
        STATUS_UNSUPPORTED => fs::Error::Unsupported,
        STATUS_NO_SPACE => fs::Error::NoSpace,
        STATUS_OUT_OF_MEMORY => fs::Error::OutOfMemory,
        STATUS_WOULD_BLOCK | STATUS_DEFERRED => fs::Error::WouldBlock,
        STATUS_INTERRUPTED => fs::Error::Interrupted,
        STATUS_NOT_TTY => fs::Error::NotTty,
        STATUS_ILLEGAL_SEEK => fs::Error::IllegalSeek,
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
        Error::DependencyCycle => STATUS_DEADLOCK,
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
        | STATUS_UNSUPPORTED
        | STATUS_NO_SPACE
        | STATUS_IO
        | STATUS_OUT_OF_MEMORY
        | STATUS_WOULD_BLOCK
        | STATUS_INTERRUPTED
        | STATUS_NOT_TTY
        | STATUS_ILLEGAL_SEEK
        | STATUS_DEADLOCK
        | STATUS_DEFERRED => status,
        _ => STATUS_IO,
    }
}
