//! The service table modules call through.
//!
//! Every kernel service a driver can use appears exactly once in [`Api`]. A
//! module receives the table at its entry point and stores it, so a call is one
//! load and one indirect branch with no lookup and no lock.
//!
//! Operations that do not need the kernel at all - register access, spin locks,
//! byte-order helpers - are deliberately absent. Those are inline in the driver
//! header, which is why the framework ships no driver library.

use alloc::{sync::Arc, vec::Vec};
use core::{
    ffi::{c_char, c_void},
    mem::size_of,
};

use crate::{
    fs::provider::{DevfsBrokerOps, FsProviderOps, FsTerminalState},
    mem::VirtAddr,
    sys::{clock, event::Event, klog, random, smp},
};

use super::{
    super::{
        class::{
            chardev::{self, DevNodeId, NodeOps},
            console::{self, ConsoleOps, TtyProviderOps},
        },
        core::{
            bus::{self, Bus, BusOps},
            class::{self, Class, ClassDevice, ClassOps, Membership},
            device::{self, Device, DeviceBuilder},
            driver::{self, Driver, DriverOps, Registration},
            fwnode,
            iface::{self, Interface, InterfaceRef, Publication},
            match_table::MatchEntry,
            module::{self, Module, ModuleLease},
            probe,
            property::PropValue,
            resource::Resource,
        },
        error::{Error, Result, STATUS_OK},
        io::{
            dma::{self, DmaBuffer},
            mmio::{self, Mapping},
            port,
        },
        irq::{self, DomainOps, HandlerFn, IrqAction, IrqDomain, ThreadFn},
        obj::{self, Object},
        work::{self, Timer, WorkFn, WorkQueue},
    },
    types::{
        ABI_MAJOR, ABI_MINOR, borrow_opt_str, borrow_receipt, borrow_slice, borrow_str,
        claim_receipt, copy_out, receipt, write_out,
    },
};

/// A driver-visible register window.
#[repr(C)]
pub struct MmioWindow {
    /// First mapped byte.
    pub base: *mut c_void,
    /// Number of mapped bytes.
    pub length: usize,
    /// Receipt released by the matching unmap call.
    pub token: *mut c_void,
}

/// A driver-visible coherent buffer.
#[repr(C)]
pub struct DmaWindow {
    /// CPU address of the buffer.
    pub cpu: *mut c_void,
    /// Address the device must be programmed with.
    pub device: u64,
    /// Number of usable bytes.
    pub length: usize,
    /// Receipt released by the matching free call.
    pub token: *mut c_void,
}

/// One entry of a driver's match table.
#[repr(C)]
pub struct MatchDef {
    /// Entry kind.
    pub kind: u32,
    /// Entry flags.
    pub flags: u32,
    /// String operand or property name.
    pub key: *const c_char,
    /// Expected string value for property comparisons.
    pub value: *const c_char,
    /// Expected primary identifier.
    pub id0: u64,
    /// Bits of the primary identifier that participate.
    pub mask0: u64,
    /// Expected secondary identifier.
    pub id1: u64,
    /// Bits of the secondary identifier that participate.
    pub mask1: u64,
    /// Cookie handed to the probe callback.
    pub data: usize,
    /// Extra score contributed by this entry.
    pub score: i32,
}

/// A driver's registration description.
#[repr(C)]
pub struct DriverDef {
    /// Size of this record.
    pub size: u32,
    /// Driver name.
    pub name: *const c_char,
    /// Bus the driver binds on, or null for bus-less devices.
    pub bus: *const c_void,
    /// Score added to every successful match.
    pub priority: i32,
    /// Match table.
    pub matches: *const MatchDef,
    /// Number of match entries.
    pub match_count: usize,
    /// Required probe callback.
    pub probe: Option<unsafe extern "C" fn(*mut c_void, *const c_void, usize) -> i32>,
    /// Optional remove callback.
    pub remove: Option<unsafe extern "C" fn(*mut c_void, *const c_void)>,
    /// Optional shutdown callback.
    pub shutdown: Option<unsafe extern "C" fn(*mut c_void, *const c_void)>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// The complete kernel service table.
///
/// The table is size-prefixed, so later revisions may append entries without
/// breaking modules built against an earlier one.
#[repr(C)]
pub struct Api {
    /// Size of this table.
    pub size: u32,
    /// Append-only revision.
    pub revision: u32,
    /// Incompatible ABI generation implemented by the kernel.
    pub abi_major: u16,
    /// Append-only ABI revision implemented by the kernel.
    pub abi_minor: u16,
    /// Reserved for alignment.
    pub reserved: u32,

    // Diagnostics and memory.
    /// Writes a message to the kernel log.
    pub log: unsafe extern "C" fn(u32, *const c_char, *const c_char),
    /// Allocates uninitialized kernel memory.
    pub alloc: unsafe extern "C" fn(usize, usize) -> *mut c_void,
    /// Allocates zeroed kernel memory.
    pub alloc_zeroed: unsafe extern "C" fn(usize, usize) -> *mut c_void,
    /// Releases kernel memory with its original layout.
    pub free: unsafe extern "C" fn(*mut c_void, usize, usize),

    // Module.
    /// Returns a module's name.
    pub module_name: unsafe extern "C" fn(*const c_void) -> *const c_char,
    /// Finds a loaded module by name.
    pub module_find: unsafe extern "C" fn(*const c_char, *mut *const c_void) -> i32,

    // Device construction.
    /// Starts describing a device.
    pub device_new: unsafe extern "C" fn(*const c_void, *const c_char, *mut *mut c_void) -> i32,
    /// Sets the parent of a device being described.
    pub device_set_parent: unsafe extern "C" fn(*mut c_void, *const c_void) -> i32,
    /// Sets the bus of a device being described.
    pub device_set_bus: unsafe extern "C" fn(*mut c_void, *const c_void) -> i32,
    /// Records the firmware entry a device came from.
    pub device_set_fwnode:
        unsafe extern "C" fn(*mut c_void, u32, u64, *const c_char, *const c_char) -> i32,
    /// Restricts the addressing limit a device can reach with DMA.
    pub device_set_dma_mask: unsafe extern "C" fn(*mut c_void, u64) -> i32,
    /// Adds an integer property.
    pub device_add_int: unsafe extern "C" fn(*mut c_void, *const c_char, u64, u32) -> i32,
    /// Adds a string property.
    pub device_add_string: unsafe extern "C" fn(*mut c_void, *const c_char, *const c_char) -> i32,
    /// Adds a string-list property.
    pub device_add_strings:
        unsafe extern "C" fn(*mut c_void, *const c_char, *const *const c_char, usize) -> i32,
    /// Adds an integer-list property.
    pub device_add_cells:
        unsafe extern "C" fn(*mut c_void, *const c_char, *const u64, usize, u32) -> i32,
    /// Adds an opaque byte property.
    pub device_add_bytes: unsafe extern "C" fn(*mut c_void, *const c_char, *const u8, usize) -> i32,
    /// Adds a hardware resource.
    pub device_add_resource:
        unsafe extern "C" fn(*mut c_void, u32, u32, u64, u64, *const c_char) -> i32,
    /// Adds a firmware-described interrupt specifier.
    pub device_add_irq: unsafe extern "C" fn(*mut c_void, *const c_void, *const u32, usize) -> i32,
    /// Publishes a described device and offers it to drivers.
    pub device_add: unsafe extern "C" fn(*mut c_void, *mut *const c_void) -> i32,
    /// Discards a description without publishing it.
    pub device_discard: unsafe extern "C" fn(*mut c_void),
    /// Removes a device and everything below it.
    pub device_remove: unsafe extern "C" fn(*const c_void) -> i32,

    // Device inspection.
    /// Returns the root of the device tree.
    pub device_root: unsafe extern "C" fn(*mut *const c_void) -> i32,
    /// Returns a device's name.
    pub device_name: unsafe extern "C" fn(*const c_void) -> *const c_char,
    /// Returns a device's parent.
    pub device_parent: unsafe extern "C" fn(*const c_void) -> *const c_void,
    /// Returns the number of children a device has.
    pub device_child_count: unsafe extern "C" fn(*const c_void) -> usize,
    /// Returns one child of a device.
    pub device_child: unsafe extern "C" fn(*const c_void, usize) -> *const c_void,
    /// Returns the driver-private instance pointer.
    pub device_data: unsafe extern "C" fn(*const c_void) -> *mut c_void,
    /// Stores the driver-private instance pointer.
    pub device_set_data: unsafe extern "C" fn(*const c_void, *mut c_void),
    /// Reads an integer property.
    pub device_int: unsafe extern "C" fn(*const c_void, *const c_char, *mut u64) -> i32,
    /// Reads one cell of an integer-list property.
    pub device_cell: unsafe extern "C" fn(*const c_void, *const c_char, usize, *mut u64) -> i32,
    /// Reads one string of a string property.
    pub device_string: unsafe extern "C" fn(
        *const c_void,
        *const c_char,
        usize,
        *mut u8,
        usize,
        *mut usize,
    ) -> i32,
    /// Reads an opaque byte property.
    pub device_bytes:
        unsafe extern "C" fn(*const c_void, *const c_char, *mut u8, usize, *mut usize) -> i32,
    /// Returns the number of elements in a property.
    pub device_property_len: unsafe extern "C" fn(*const c_void, *const c_char) -> usize,
    /// Reads one hardware resource.
    pub device_resource:
        unsafe extern "C" fn(*const c_void, u32, usize, *mut u64, *mut u64, *mut u32) -> i32,
    /// Returns the firmware token recorded for a device.
    pub device_fwnode: unsafe extern "C" fn(*const c_void, *mut u32, *mut u64) -> i32,

    // Bus, driver, and class registration.
    /// Registers a bus type.
    pub bus_register: unsafe extern "C" fn(
        *const c_void,
        *const c_char,
        *const BusRegistration,
        *mut *const c_void,
    ) -> i32,
    /// Unregisters a bus type.
    pub bus_unregister: unsafe extern "C" fn(*const c_void) -> i32,
    /// Finds a registered bus by name.
    pub bus_find: unsafe extern "C" fn(*const c_char, *mut *const c_void) -> i32,
    /// Installs a DMA translation for devices on a bus.
    pub bus_set_dma_ops: unsafe extern "C" fn(*const c_void, *const c_void) -> i32,
    /// Registers a driver.
    pub driver_register:
        unsafe extern "C" fn(*const c_void, *const DriverDef, *mut *const c_void) -> i32,
    /// Unregisters a driver.
    pub driver_unregister: unsafe extern "C" fn(*const c_void) -> i32,
    /// Registers a device class.
    pub class_register: unsafe extern "C" fn(
        *const c_void,
        *const c_char,
        *const ClassRegistration,
        *mut *const c_void,
    ) -> i32,
    /// Unregisters a device class.
    pub class_unregister: unsafe extern "C" fn(*const c_void) -> i32,
    /// Finds a registered class by name.
    pub class_find: unsafe extern "C" fn(*const c_char, *mut *const c_void) -> i32,
    /// Adds a device to a class.
    pub class_add: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        *const c_void,
        *const c_char,
        *const c_void,
        usize,
        *mut c_void,
        *mut *const c_void,
    ) -> i32,
    /// Removes a class membership.
    pub class_remove: unsafe extern "C" fn(*const c_void),
    /// Returns a membership's operation table and context.
    pub class_member_ops:
        unsafe extern "C" fn(*const c_void, *mut *const c_void, *mut *mut c_void) -> i32,
    /// Returns a membership's name.
    pub class_member_name: unsafe extern "C" fn(*const c_void) -> *const c_char,
    /// Returns a membership's device.
    pub class_member_device: unsafe extern "C" fn(*const c_void) -> *const c_void,
    /// Returns the class-private value on a membership.
    pub class_member_data: unsafe extern "C" fn(*const c_void) -> usize,
    /// Stores a class-private value on a membership.
    pub class_member_set_data: unsafe extern "C" fn(*const c_void, usize),
    /// Returns the number of members in a class.
    pub class_member_count: unsafe extern "C" fn(*const c_void) -> usize,
    /// Returns one member of a class.
    pub class_member: unsafe extern "C" fn(*const c_void, usize) -> *const c_void,

    // Interfaces.
    /// Publishes an interface, globally or on a device.
    pub iface_publish: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        *const c_char,
        u32,
        u32,
        *const c_void,
        usize,
        *mut c_void,
        *mut *const c_void,
    ) -> i32,
    /// Withdraws a published interface.
    pub iface_withdraw: unsafe extern "C" fn(*const c_void) -> i32,
    /// Binds to an interface by scope.
    pub iface_bind: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        u32,
        *const c_char,
        u32,
        *mut *const c_void,
        *mut *mut c_void,
        *mut *mut c_void,
    ) -> i32,
    /// Releases a binding.
    pub iface_unbind: unsafe extern "C" fn(*mut c_void),
    /// Returns whether a compatible global provider exists.
    pub iface_available: unsafe extern "C" fn(*const c_char, u32) -> i32,
    /// Returns the number of global providers of a name.
    pub iface_count: unsafe extern "C" fn(*const c_char, u32) -> usize,
    /// Returns the device behind one global provider.
    pub iface_provider: unsafe extern "C" fn(*const c_char, u32, usize) -> *const c_void,
    /// Requests a rescan of devices waiting for a prerequisite.
    pub probe_retrigger: unsafe extern "C" fn(),

    // Interrupts.
    /// Registers an interrupt controller domain.
    pub irq_domain_register: unsafe extern "C" fn(
        *const c_void,
        *const c_char,
        u32,
        u32,
        *const DomainRegistration,
        *mut *const c_void,
    ) -> i32,
    /// Unregisters an interrupt controller domain.
    pub irq_domain_unregister: unsafe extern "C" fn(*const c_void) -> i32,
    /// Maps a hardware interrupt into the virtual interrupt space.
    pub irq_map: unsafe extern "C" fn(*const c_void, u64, u32, *mut u32) -> i32,
    /// Resolves a device's firmware interrupt to a virtual interrupt.
    pub irq_of_device: unsafe extern "C" fn(*const c_void, usize, *mut u32) -> i32,
    /// Attaches a handler to a virtual interrupt.
    pub irq_request: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        u32,
        *const c_char,
        u32,
        Option<HandlerFn>,
        Option<ThreadFn>,
        *mut c_void,
        *mut *mut c_void,
    ) -> i32,
    /// Detaches a handler.
    pub irq_release: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Masks a virtual interrupt.
    pub irq_mask: unsafe extern "C" fn(u32) -> i32,
    /// Unmasks a virtual interrupt.
    pub irq_unmask: unsafe extern "C" fn(u32) -> i32,
    /// Redirects a virtual interrupt to a CPU.
    pub irq_set_affinity: unsafe extern "C" fn(u32, u32) -> i32,
    /// Reserves an architecture vector for a controller.
    pub irq_alloc_vector: unsafe extern "C" fn(u32, *mut u32) -> i32,
    /// Releases an architecture vector.
    pub irq_free_vector: unsafe extern "C" fn(u32) -> i32,
    /// Returns the message address and payload that raise an interrupt.
    pub irq_compose_message: unsafe extern "C" fn(*const c_void, u64, *mut u64, *mut u32) -> i32,

    // Register windows, ports, and DMA.
    /// Maps a physical range into the kernel's device window.
    pub mmio_map: unsafe extern "C" fn(*const c_void, u64, usize, u32, *mut MmioWindow) -> i32,
    /// Unmaps a register window.
    pub mmio_unmap: unsafe extern "C" fn(*mut MmioWindow) -> i32,
    /// Returns the direct-map address of boot-mapped physical memory.
    pub mmio_direct: unsafe extern "C" fn(u64, *mut *mut c_void) -> i32,
    /// Reads a byte from an I/O port.
    pub port_read8: unsafe extern "C" fn(u16) -> u32,
    /// Reads two bytes from an I/O port.
    pub port_read16: unsafe extern "C" fn(u16) -> u32,
    /// Reads four bytes from an I/O port.
    pub port_read32: unsafe extern "C" fn(u16) -> u32,
    /// Writes a byte to an I/O port.
    pub port_write8: unsafe extern "C" fn(u16, u8),
    /// Writes two bytes to an I/O port.
    pub port_write16: unsafe extern "C" fn(u16, u16),
    /// Writes four bytes to an I/O port.
    pub port_write32: unsafe extern "C" fn(u16, u32),
    /// Allocates a coherent buffer.
    pub dma_alloc: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        usize,
        usize,
        u32,
        *mut DmaWindow,
    ) -> i32,
    /// Releases a coherent buffer.
    pub dma_free: unsafe extern "C" fn(*mut DmaWindow) -> i32,
    /// Resolves the device address of an existing kernel buffer.
    pub dma_map: unsafe extern "C" fn(*const c_void, *mut c_void, usize, u32, *mut u64) -> i32,
    /// Releases a mapping created by `dma_map`.
    pub dma_unmap: unsafe extern "C" fn(*const c_void, u64, usize, u32),
    /// Orders CPU and device views of a buffer.
    pub dma_sync: unsafe extern "C" fn(*mut c_void, usize, u32),

    // Deferred work, timers, and events.
    /// Creates a work queue.
    pub work_create: unsafe extern "C" fn(*const c_void, *const c_char, *mut *mut c_void) -> i32,
    /// Appends a callback to a work queue.
    pub work_queue: unsafe extern "C" fn(*mut c_void, Option<WorkFn>, *mut c_void, u64) -> i32,
    /// Waits until a work queue is idle.
    pub work_flush: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Destroys a work queue.
    pub work_destroy: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Creates a timer.
    pub timer_create: unsafe extern "C" fn(
        *const c_void,
        Option<WorkFn>,
        *mut c_void,
        u64,
        *mut *mut c_void,
    ) -> i32,
    /// Arms a timer.
    pub timer_arm: unsafe extern "C" fn(*mut c_void, u64, u64) -> i32,
    /// Disarms a timer.
    pub timer_cancel: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Destroys a timer.
    pub timer_destroy: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Creates an event.
    pub event_create: unsafe extern "C" fn(*const c_void, *mut usize) -> i32,
    /// Destroys an event.
    pub event_destroy: unsafe extern "C" fn(usize) -> i32,
    /// Waits until an event is signalled.
    pub event_wait: unsafe extern "C" fn(usize) -> i32,
    /// Signals an event, waking every waiter.
    pub event_signal: unsafe extern "C" fn(usize) -> i32,
    /// Clears an event's signal.
    pub event_reset: unsafe extern "C" fn(usize) -> i32,

    // Time, entropy, and topology.
    /// Returns nanoseconds since boot.
    pub time_monotonic: unsafe extern "C" fn() -> u64,
    /// Busy-waits for a duration.
    pub time_delay: unsafe extern "C" fn(u64),
    /// Sleeps the calling thread for a duration.
    pub time_sleep: unsafe extern "C" fn(u64),
    /// Fills a buffer with random bytes.
    pub random_fill: unsafe extern "C" fn(*mut u8, usize),
    /// Mixes entropy into the random pool.
    pub random_mix: unsafe extern "C" fn(*const u8, usize),
    /// Returns the number of CPUs.
    pub cpu_count: unsafe extern "C" fn() -> u32,
    /// Returns the calling CPU's identifier.
    pub cpu_current: unsafe extern "C" fn() -> u32,
    /// Returns a CPU's firmware identifier.
    pub cpu_platform_id: unsafe extern "C" fn(u32, *mut u64) -> i32,
    /// Returns whether the caller is in interrupt context.
    pub in_interrupt: unsafe extern "C" fn() -> i32,

    // Firmware blobs.
    /// Copies the ACPI root pointer, when the platform provides one.
    pub firmware_acpi: unsafe extern "C" fn(*mut u8, usize, *mut usize) -> i32,
    /// Copies the device tree, when the platform provides one.
    pub firmware_devicetree: unsafe extern "C" fn(*mut u8, usize, *mut usize) -> i32,

    // Device nodes and terminals.
    /// Returns the device filesystem root.
    pub devfs_root: unsafe extern "C" fn(*mut u64) -> i32,
    /// Creates a directory in the device filesystem.
    pub devfs_mkdir: unsafe extern "C" fn(*const c_void, u64, *const c_char, u16, *mut u64) -> i32,
    /// Creates a device node backed by a driver operation table.
    pub devfs_create: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        u64,
        *const c_char,
        u32,
        u16,
        *const NodeOps,
        *mut u64,
    ) -> i32,
    /// Removes a device-filesystem node.
    pub devfs_remove: unsafe extern "C" fn(*const c_void, u64) -> i32,
    /// Resolves an absolute device-filesystem path.
    pub devfs_lookup: unsafe extern "C" fn(*const c_char, *mut u64) -> i32,
    /// Registers a terminal backed by a console operation table.
    pub tty_register: unsafe extern "C" fn(
        *const c_void,
        *const c_void,
        u64,
        *const c_char,
        u16,
        u32,
        *const ConsoleOps,
        *mut *mut c_void,
    ) -> i32,
    /// Removes a terminal.
    pub tty_unregister: unsafe extern "C" fn(*mut c_void) -> i32,

    // Kernel log.
    /// Returns the severity currently recorded into the kernel log.
    pub klog_level: unsafe extern "C" fn() -> u32,
    /// Changes the recorded severity and returns the previous one.
    pub klog_set_level: unsafe extern "C" fn(u32) -> u32,
    /// Returns the severity currently mirrored to the console.
    pub klog_console_level: unsafe extern "C" fn() -> u32,
    /// Changes the mirrored severity and returns the previous one.
    pub klog_set_console_level: unsafe extern "C" fn(u32) -> u32,

    // Modular terminal provider support, appended in ABI minor 1.
    /// Registers the single terminal-semantics provider.
    pub tty_provider_register:
        unsafe extern "C" fn(*const c_void, *const TtyProviderOps, *mut *mut c_void) -> i32,
    /// Removes the terminal provider after its receipts are gone.
    pub tty_provider_unregister: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Waits until any supplied event is signalled and returns its index.
    pub event_wait_any: unsafe extern "C" fn(*const usize, usize, *mut usize) -> i32,
    /// Waits for an event until a timeout and reports whether it was signalled.
    pub event_wait_timeout: unsafe extern "C" fn(usize, u64, *mut u8) -> i32,
    /// Starts a joinable module worker.
    pub worker_spawn: unsafe extern "C" fn(
        *const c_void,
        Option<unsafe extern "C" fn(*mut c_void)>,
        *mut c_void,
        *mut *mut c_void,
    ) -> i32,
    /// Joins a worker and releases its receipt.
    pub worker_join: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Delivers a signal to a process group.
    pub process_group_signal: unsafe extern "C" fn(i32, u8) -> i32,

    // Filesystem providers, appended in ABI minor 2.
    /// Registers a C-compatible filesystem provider table.
    pub fs_provider_register: unsafe extern "C" fn(
        *const c_void,
        *const c_char,
        *const FsProviderOps,
        *mut *mut c_void,
    ) -> i32,
    /// Removes a filesystem-provider registration.
    pub fs_provider_unregister: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Creates a page-account receipt for a memory-backed provider.
    pub fs_page_account_create: unsafe extern "C" fn(u64, *mut *mut c_void) -> i32,
    /// Drops a page-account receipt.
    pub fs_page_account_release: unsafe extern "C" fn(*mut c_void),
    /// Returns a page-account limit.
    pub fs_page_account_limit: unsafe extern "C" fn(*mut c_void, *mut u64) -> i32,
    /// Returns a page-account committed-page count.
    pub fs_page_account_used: unsafe extern "C" fn(*mut c_void, *mut u64) -> i32,
    /// Creates a kernel-owned page-cache object.
    pub fs_memory_object_create: unsafe extern "C" fn(*mut c_void, *mut *mut c_void) -> i32,
    /// Retains a page-cache object receipt.
    pub fs_memory_object_retain: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Drops a page-cache object receipt.
    pub fs_memory_object_release: unsafe extern "C" fn(*mut c_void),
    /// Reads a page-cache object directly into a provider buffer.
    pub fs_memory_object_read: unsafe extern "C" fn(*mut c_void, u64, *mut u8, usize) -> i64,
    /// Writes a page-cache object directly from a provider buffer.
    pub fs_memory_object_write: unsafe extern "C" fn(*mut c_void, u64, *const u8, usize) -> i64,
    /// Truncates a page-cache object.
    pub fs_memory_object_truncate: unsafe extern "C" fn(*mut c_void, u64, *mut u64) -> i32,
    /// Returns a page-cache object's committed-page count.
    pub fs_memory_object_page_count: unsafe extern "C" fn(*mut c_void, *mut u64) -> i32,
    /// Returns usable physical memory pages for provider default limits.
    pub fs_total_physical_pages: unsafe extern "C" fn() -> u64,
    /// Registers the global devfs broker table.
    pub devfs_broker_register:
        unsafe extern "C" fn(*const c_void, *const DevfsBrokerOps, *mut *mut c_void) -> i32,
    /// Removes a devfs broker registration.
    pub devfs_broker_unregister: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Opens a kernel-owned device endpoint.
    pub devfs_endpoint_open: unsafe extern "C" fn(*mut c_void, u32, *mut usize) -> i32,
    /// Closes one endpoint file context.
    pub devfs_endpoint_close: unsafe extern "C" fn(*mut c_void, usize, u32),
    /// Returns a new endpoint file offset.
    pub devfs_endpoint_initial_offset:
        unsafe extern "C" fn(*mut c_void, usize, u32, *mut u64) -> i32,
    /// Reads directly into a devfs-module buffer.
    pub devfs_endpoint_read:
        unsafe extern "C" fn(*mut c_void, usize, u64, *mut u8, usize, u32) -> i64,
    /// Writes directly from a devfs-module buffer.
    pub devfs_endpoint_write:
        unsafe extern "C" fn(*mut c_void, usize, u64, *const u8, usize, u32) -> i64,
    /// Returns an endpoint's logical size.
    pub devfs_endpoint_size: unsafe extern "C" fn(*mut c_void) -> u64,
    /// Flushes a device endpoint.
    pub devfs_endpoint_sync: unsafe extern "C" fn(*mut c_void) -> i32,
    /// Polls a device endpoint.
    pub devfs_endpoint_poll: unsafe extern "C" fn(*mut c_void, usize, u64, u16, u32) -> i64,
    /// Returns an endpoint event receipt.
    pub devfs_endpoint_event: unsafe extern "C" fn(*mut c_void, usize, u32) -> usize,
    /// Returns terminal state for an endpoint.
    pub devfs_endpoint_terminal_state:
        unsafe extern "C" fn(*mut c_void, *mut FsTerminalState) -> i32,
    /// Performs an endpoint control operation.
    pub devfs_endpoint_ioctl: unsafe extern "C" fn(
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
    ) -> i64,
    /// Drops an endpoint receipt retained by devfs.
    pub devfs_endpoint_release: unsafe extern "C" fn(*mut c_void),
}

/// Bus callbacks supplied from C.
#[repr(C)]
pub struct BusRegistration {
    /// Size of this record.
    pub size: u32,
    /// Optional replacement for match-table scoring.
    pub match_device:
        Option<unsafe extern "C" fn(*mut c_void, *const c_void, *const c_void) -> i32>,
    /// Optional pre-probe hook.
    pub prepare: Option<unsafe extern "C" fn(*mut c_void, *const c_void) -> i32>,
    /// Optional post-remove hook.
    pub cleanup: Option<unsafe extern "C" fn(*mut c_void, *const c_void)>,
    /// Optional shutdown hook.
    pub shutdown: Option<unsafe extern "C" fn(*mut c_void, *const c_void)>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// Class callbacks supplied from C.
#[repr(C)]
pub struct ClassRegistration {
    /// Size of this record.
    pub size: u32,
    /// Optional callback invoked when a member joins.
    pub attach: Option<unsafe extern "C" fn(*mut c_void, *const c_void) -> i32>,
    /// Optional callback invoked when a member leaves.
    pub detach: Option<unsafe extern "C" fn(*mut c_void, *const c_void)>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// Interrupt controller callbacks supplied from C.
#[repr(C)]
pub struct DomainRegistration {
    /// Size of this record.
    pub size: u32,
    /// Decodes firmware specifier cells.
    pub translate:
        Option<unsafe extern "C" fn(*mut c_void, *const u32, usize, *mut u64, *mut u32) -> i32>,
    /// Programs routing for a newly mapped interrupt.
    pub setup: Option<unsafe extern "C" fn(*mut c_void, u64, u32, u32) -> i32>,
    /// Releases routing for an unmapped interrupt.
    pub teardown: Option<unsafe extern "C" fn(*mut c_void, u64, u32)>,
    /// Masks a line.
    pub mask: Option<unsafe extern "C" fn(*mut c_void, u64)>,
    /// Unmasks a line.
    pub unmask: Option<unsafe extern "C" fn(*mut c_void, u64)>,
    /// Signals end of interrupt.
    pub eoi: Option<unsafe extern "C" fn(*mut c_void, u64)>,
    /// Redirects a line to another CPU.
    pub set_affinity: Option<unsafe extern "C" fn(*mut c_void, u64, u32) -> i32>,
    /// Identifies the interrupt pending on this CPU.
    pub claim: Option<unsafe extern "C" fn(*mut c_void, u32, u64, *mut u64) -> i32>,
    /// Completes an interrupt reported by `claim`.
    pub complete: Option<unsafe extern "C" fn(*mut c_void, u32, u64, u64)>,
    /// Reports the address and payload that raise an interrupt.
    pub compose_message: Option<unsafe extern "C" fn(*mut c_void, u64, *mut u64, *mut u32) -> i32>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// Interface binding scopes.
pub mod scope {
    /// Bind to a system-wide interface.
    pub const GLOBAL: u32 = 0;
    /// Bind to an interface published by the given device.
    pub const DEVICE: u32 = 1;
    /// Bind to an interface published by the device or any ancestor.
    pub const ANCESTOR: u32 = 2;
}

/// Guards against drift between this table and `drivers/include/roanix/api.h`.
///
/// Both sides describe the same memory. If a field is added, reordered, or
/// retyped on one side only, these assertions fail at build time rather than
/// letting a module call through a mismatched pointer at run time.
mod layout {
    use super::*;
    use crate::driver::{
        class::{chardev::IoctlIdentity, console::SerialFraming},
        io::dma::DmaOps,
    };

    const _: () = assert!(size_of::<Api>() == 1272);
    const _: () = assert!(core::mem::offset_of!(Api, log) == 16);
    const _: () = assert!(core::mem::offset_of!(Api, klog_set_console_level) == 984);
    const _: () = assert!(core::mem::offset_of!(Api, tty_provider_register) == 992);
    const _: () = assert!(core::mem::offset_of!(Api, fs_provider_register) == 1048);
    const _: () = assert!(size_of::<crate::driver::abi::types::ModuleDef>() == 48);
    const _: () = assert!(size_of::<NodeOps>() == 120);
    const _: () = assert!(size_of::<ConsoleOps>() == 160);
    const _: () = assert!(size_of::<TtyProviderOps>() == 32);
    const _: () = assert!(size_of::<MatchDef>() == 72);
    const _: () = assert!(size_of::<DriverDef>() == 80);
    const _: () = assert!(size_of::<MmioWindow>() == 24);
    const _: () = assert!(size_of::<DmaWindow>() == 32);
    const _: () = assert!(size_of::<BusRegistration>() == 48);
    const _: () = assert!(size_of::<ClassRegistration>() == 32);
    const _: () = assert!(size_of::<DomainRegistration>() == 96);
    const _: () = assert!(size_of::<IoctlIdentity>() == 24);
    const _: () = assert!(size_of::<SerialFraming>() == 8);
    const _: () = assert!(size_of::<DmaOps>() == 32);
}

include!("shims.rs");

/// Resolves one versioned module import from the curated kernel ABI.
///
/// This is intentionally a closed list of C ABI shims. It must never fall
/// through to the kernel's Rust symbols or permit a module to name arbitrary
/// internal functions.
pub(super) fn resolve_import(name: &[u8]) -> Option<usize> {
    match name {
        b"rdf_api_v1_alloc" => Some(shim_alloc as *const () as usize),
        b"rdf_api_v1_alloc_zeroed" => Some(shim_alloc_zeroed as *const () as usize),
        b"rdf_api_v1_free" => Some(shim_free as *const () as usize),
        b"rdf_api_v1_random_fill" => Some(shim_random_fill as *const () as usize),
        b"rdf_api_v1_random_mix" => Some(shim_random_mix as *const () as usize),
        b"rdf_api_v1_devfs_root" => Some(shim_devfs_root as *const () as usize),
        b"rdf_api_v1_devfs_create" => Some(shim_devfs_create as *const () as usize),
        b"rdf_api_v1_devfs_remove" => Some(shim_devfs_remove as *const () as usize),
        b"rdf_api_v1_port_read8" => Some(shim_port_read8 as *const () as usize),
        b"rdf_api_v1_port_write8" => Some(shim_port_write8 as *const () as usize),
        b"rdf_api_v1_event_wait" => Some(shim_event_wait as *const () as usize),
        b"rdf_api_v1_event_signal" => Some(shim_event_signal as *const () as usize),
        b"rdf_api_v1_event_reset" => Some(shim_event_reset as *const () as usize),
        b"rdf_api_v1_event_wait_any" => Some(shim_event_wait_any as *const () as usize),
        b"rdf_api_v1_event_wait_timeout" => Some(shim_event_wait_timeout as *const () as usize),
        b"rdf_api_v1_time_monotonic" => Some(shim_time_monotonic as *const () as usize),
        b"rdf_api_v1_worker_spawn" => Some(shim_worker_spawn as *const () as usize),
        b"rdf_api_v1_worker_join" => Some(shim_worker_join as *const () as usize),
        b"rdf_api_v1_process_group_signal" => Some(shim_process_group_signal as *const () as usize),
        b"rdf_api_v1_tty_provider_register" => {
            Some(shim_tty_provider_register as *const () as usize)
        }
        b"rdf_api_v1_tty_provider_unregister" => {
            Some(shim_tty_provider_unregister as *const () as usize)
        }
        b"rdf_api_v1_time_sleep" => Some(shim_time_sleep as *const () as usize),
        b"rdf_api_v1_fs_page_account_create" => {
            Some(shim_fs_page_account_create as *const () as usize)
        }
        b"rdf_api_v1_fs_page_account_release" => {
            Some(shim_fs_page_account_release as *const () as usize)
        }
        b"rdf_api_v1_fs_page_account_limit" => {
            Some(shim_fs_page_account_limit as *const () as usize)
        }
        b"rdf_api_v1_fs_page_account_used" => Some(shim_fs_page_account_used as *const () as usize),
        b"rdf_api_v1_fs_memory_object_create" => {
            Some(shim_fs_memory_object_create as *const () as usize)
        }
        b"rdf_api_v1_fs_memory_object_retain" => {
            Some(shim_fs_memory_object_retain as *const () as usize)
        }
        b"rdf_api_v1_fs_memory_object_release" => {
            Some(shim_fs_memory_object_release as *const () as usize)
        }
        b"rdf_api_v1_fs_memory_object_read" => {
            Some(shim_fs_memory_object_read as *const () as usize)
        }
        b"rdf_api_v1_fs_memory_object_write" => {
            Some(shim_fs_memory_object_write as *const () as usize)
        }
        b"rdf_api_v1_fs_memory_object_truncate" => {
            Some(shim_fs_memory_object_truncate as *const () as usize)
        }
        b"rdf_api_v1_fs_memory_object_page_count" => {
            Some(shim_fs_memory_object_page_count as *const () as usize)
        }
        b"rdf_api_v1_fs_total_physical_pages" => {
            Some(shim_fs_total_physical_pages as *const () as usize)
        }
        b"rdf_api_v1_devfs_endpoint_open" => Some(shim_devfs_endpoint_open as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_close" => Some(shim_devfs_endpoint_close as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_initial_offset" => {
            Some(shim_devfs_endpoint_initial_offset as *const () as usize)
        }
        b"rdf_api_v1_devfs_endpoint_read" => Some(shim_devfs_endpoint_read as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_write" => Some(shim_devfs_endpoint_write as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_size" => Some(shim_devfs_endpoint_size as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_sync" => Some(shim_devfs_endpoint_sync as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_poll" => Some(shim_devfs_endpoint_poll as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_event" => Some(shim_devfs_endpoint_event as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_terminal_state" => {
            Some(shim_devfs_endpoint_terminal_state as *const () as usize)
        }
        b"rdf_api_v1_devfs_endpoint_ioctl" => Some(shim_devfs_endpoint_ioctl as *const () as usize),
        b"rdf_api_v1_devfs_endpoint_release" => {
            Some(shim_devfs_endpoint_release as *const () as usize)
        }
        _ => None,
    }
}

/// The service table handed to every module.
pub static API: Api = Api {
    size: size_of::<Api>() as u32,
    revision: 2,
    abi_major: ABI_MAJOR,
    abi_minor: ABI_MINOR,
    reserved: 0,

    log: shim_log,
    alloc: shim_alloc,
    alloc_zeroed: shim_alloc_zeroed,
    free: shim_free,

    module_name: shim_module_name,
    module_find: shim_module_find,

    device_new: shim_device_new,
    device_set_parent: shim_device_set_parent,
    device_set_bus: shim_device_set_bus,
    device_set_fwnode: shim_device_set_fwnode,
    device_set_dma_mask: shim_device_set_dma_mask,
    device_add_int: shim_device_add_int,
    device_add_string: shim_device_add_string,
    device_add_strings: shim_device_add_strings,
    device_add_cells: shim_device_add_cells,
    device_add_bytes: shim_device_add_bytes,
    device_add_resource: shim_device_add_resource,
    device_add_irq: shim_device_add_irq,
    device_add: shim_device_add,
    device_discard: shim_device_discard,
    device_remove: shim_device_remove,

    device_root: shim_device_root,
    device_name: shim_device_name,
    device_parent: shim_device_parent,
    device_child_count: shim_device_child_count,
    device_child: shim_device_child,
    device_data: shim_device_data,
    device_set_data: shim_device_set_data,
    device_int: shim_device_int,
    device_cell: shim_device_cell,
    device_string: shim_device_string,
    device_bytes: shim_device_bytes,
    device_property_len: shim_device_property_len,
    device_resource: shim_device_resource,
    device_fwnode: shim_device_fwnode,

    bus_register: shim_bus_register,
    bus_unregister: shim_bus_unregister,
    bus_find: shim_bus_find,
    bus_set_dma_ops: shim_bus_set_dma_ops,
    driver_register: shim_driver_register,
    driver_unregister: shim_driver_unregister,
    class_register: shim_class_register,
    class_unregister: shim_class_unregister,
    class_find: shim_class_find,
    class_add: shim_class_add,
    class_remove: shim_class_remove,
    class_member_ops: shim_class_member_ops,
    class_member_name: shim_class_member_name,
    class_member_device: shim_class_member_device,
    class_member_data: shim_class_member_data,
    class_member_set_data: shim_class_member_set_data,
    class_member_count: shim_class_member_count,
    class_member: shim_class_member,

    iface_publish: shim_iface_publish,
    iface_withdraw: shim_iface_withdraw,
    iface_bind: shim_iface_bind,
    iface_unbind: shim_iface_unbind,
    iface_available: shim_iface_available,
    iface_count: shim_iface_count,
    iface_provider: shim_iface_provider,
    probe_retrigger: shim_probe_retrigger,

    irq_domain_register: shim_irq_domain_register,
    irq_domain_unregister: shim_irq_domain_unregister,
    irq_map: shim_irq_map,
    irq_of_device: shim_irq_of_device,
    irq_request: shim_irq_request,
    irq_release: shim_irq_release,
    irq_mask: shim_irq_mask,
    irq_unmask: shim_irq_unmask,
    irq_set_affinity: shim_irq_set_affinity,
    irq_alloc_vector: shim_irq_alloc_vector,
    irq_free_vector: shim_irq_free_vector,
    irq_compose_message: shim_irq_compose_message,

    mmio_map: shim_mmio_map,
    mmio_unmap: shim_mmio_unmap,
    mmio_direct: shim_mmio_direct,
    port_read8: shim_port_read8,
    port_read16: shim_port_read16,
    port_read32: shim_port_read32,
    port_write8: shim_port_write8,
    port_write16: shim_port_write16,
    port_write32: shim_port_write32,
    dma_alloc: shim_dma_alloc,
    dma_free: shim_dma_free,
    dma_map: shim_dma_map,
    dma_unmap: shim_dma_unmap,
    dma_sync: shim_dma_sync,

    work_create: shim_work_create,
    work_queue: shim_work_queue,
    work_flush: shim_work_flush,
    work_destroy: shim_work_destroy,
    timer_create: shim_timer_create,
    timer_arm: shim_timer_arm,
    timer_cancel: shim_timer_cancel,
    timer_destroy: shim_timer_destroy,
    event_create: shim_event_create,
    event_destroy: shim_event_destroy,
    event_wait: shim_event_wait,
    event_signal: shim_event_signal,
    event_reset: shim_event_reset,

    time_monotonic: shim_time_monotonic,
    time_delay: shim_time_delay,
    time_sleep: shim_time_sleep,
    random_fill: shim_random_fill,
    random_mix: shim_random_mix,
    cpu_count: shim_cpu_count,
    cpu_current: shim_cpu_current,
    cpu_platform_id: shim_cpu_platform_id,
    in_interrupt: shim_in_interrupt,

    firmware_acpi: shim_firmware_acpi,
    firmware_devicetree: shim_firmware_devicetree,

    devfs_root: shim_devfs_root,
    devfs_mkdir: shim_devfs_mkdir,
    devfs_create: shim_devfs_create,
    devfs_remove: shim_devfs_remove,
    devfs_lookup: shim_devfs_lookup,
    tty_register: shim_tty_register,
    tty_unregister: shim_tty_unregister,

    klog_level: shim_klog_level,
    klog_set_level: shim_klog_set_level,
    klog_console_level: shim_klog_console_level,
    klog_set_console_level: shim_klog_set_console_level,

    tty_provider_register: shim_tty_provider_register,
    tty_provider_unregister: shim_tty_provider_unregister,
    event_wait_any: shim_event_wait_any,
    event_wait_timeout: shim_event_wait_timeout,
    worker_spawn: shim_worker_spawn,
    worker_join: shim_worker_join,
    process_group_signal: shim_process_group_signal,

    fs_provider_register: shim_fs_provider_register,
    fs_provider_unregister: shim_fs_provider_unregister,
    fs_page_account_create: shim_fs_page_account_create,
    fs_page_account_release: shim_fs_page_account_release,
    fs_page_account_limit: shim_fs_page_account_limit,
    fs_page_account_used: shim_fs_page_account_used,
    fs_memory_object_create: shim_fs_memory_object_create,
    fs_memory_object_retain: shim_fs_memory_object_retain,
    fs_memory_object_release: shim_fs_memory_object_release,
    fs_memory_object_read: shim_fs_memory_object_read,
    fs_memory_object_write: shim_fs_memory_object_write,
    fs_memory_object_truncate: shim_fs_memory_object_truncate,
    fs_memory_object_page_count: shim_fs_memory_object_page_count,
    fs_total_physical_pages: shim_fs_total_physical_pages,
    devfs_broker_register: shim_devfs_broker_register,
    devfs_broker_unregister: shim_devfs_broker_unregister,
    devfs_endpoint_open: shim_devfs_endpoint_open,
    devfs_endpoint_close: shim_devfs_endpoint_close,
    devfs_endpoint_initial_offset: shim_devfs_endpoint_initial_offset,
    devfs_endpoint_read: shim_devfs_endpoint_read,
    devfs_endpoint_write: shim_devfs_endpoint_write,
    devfs_endpoint_size: shim_devfs_endpoint_size,
    devfs_endpoint_sync: shim_devfs_endpoint_sync,
    devfs_endpoint_poll: shim_devfs_endpoint_poll,
    devfs_endpoint_event: shim_devfs_endpoint_event,
    devfs_endpoint_terminal_state: shim_devfs_endpoint_terminal_state,
    devfs_endpoint_ioctl: shim_devfs_endpoint_ioctl,
    devfs_endpoint_release: shim_devfs_endpoint_release,
};
