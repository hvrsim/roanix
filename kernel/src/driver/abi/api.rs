//! The versioned C ABI functions imported by driver modules.
//!
//! The module loader resolves each used function once before module entry.
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
        borrow_opt_str, borrow_receipt, borrow_slice, borrow_str, claim_receipt, copy_out, receipt,
        write_out,
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

/// Guards C ABI records against drift from the installed headers.
mod layout {
    use super::*;
    use crate::driver::{
        class::{chardev::IoctlIdentity, console::SerialFraming},
        io::dma::DmaOps,
    };

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

/// Resolves one versioned module import from the kernel ABI.
///
/// This closed list never exposes the kernel's internal Rust symbols.
pub(super) fn resolve_import(name: &[u8]) -> Option<usize> {
    macro_rules! resolve {
        ($name:expr; $($service:ident => $function:ident),* $(,)?) => {
            match $name {
                $(
                    concat!("rdf_api_v1_", stringify!($service)) => {
                        Some($function as *const () as usize)
                    }
                )*
                _ => None,
            }
        };
    }

    let name = core::str::from_utf8(name).ok()?;
    resolve!(name;
        log => shim_log,
        alloc => shim_alloc,
        alloc_zeroed => shim_alloc_zeroed,
        free => shim_free,
        module_name => shim_module_name,
        module_find => shim_module_find,
        device_new => shim_device_new,
        device_set_parent => shim_device_set_parent,
        device_set_bus => shim_device_set_bus,
        device_set_fwnode => shim_device_set_fwnode,
        device_set_dma_mask => shim_device_set_dma_mask,
        device_add_int => shim_device_add_int,
        device_add_string => shim_device_add_string,
        device_add_strings => shim_device_add_strings,
        device_add_cells => shim_device_add_cells,
        device_add_bytes => shim_device_add_bytes,
        device_add_resource => shim_device_add_resource,
        device_add_irq => shim_device_add_irq,
        device_add => shim_device_add,
        device_discard => shim_device_discard,
        device_remove => shim_device_remove,
        device_root => shim_device_root,
        device_name => shim_device_name,
        device_parent => shim_device_parent,
        device_child_count => shim_device_child_count,
        device_child => shim_device_child,
        device_data => shim_device_data,
        device_set_data => shim_device_set_data,
        device_int => shim_device_int,
        device_cell => shim_device_cell,
        device_string => shim_device_string,
        device_bytes => shim_device_bytes,
        device_property_len => shim_device_property_len,
        device_resource => shim_device_resource,
        device_fwnode => shim_device_fwnode,
        bus_register => shim_bus_register,
        bus_unregister => shim_bus_unregister,
        bus_find => shim_bus_find,
        bus_set_dma_ops => shim_bus_set_dma_ops,
        driver_register => shim_driver_register,
        driver_unregister => shim_driver_unregister,
        class_register => shim_class_register,
        class_unregister => shim_class_unregister,
        class_find => shim_class_find,
        class_add => shim_class_add,
        class_remove => shim_class_remove,
        class_member_ops => shim_class_member_ops,
        class_member_name => shim_class_member_name,
        class_member_device => shim_class_member_device,
        class_member_data => shim_class_member_data,
        class_member_set_data => shim_class_member_set_data,
        class_member_count => shim_class_member_count,
        class_member => shim_class_member,
        iface_publish => shim_iface_publish,
        iface_withdraw => shim_iface_withdraw,
        iface_bind => shim_iface_bind,
        iface_unbind => shim_iface_unbind,
        iface_available => shim_iface_available,
        iface_count => shim_iface_count,
        iface_provider => shim_iface_provider,
        probe_retrigger => shim_probe_retrigger,
        irq_domain_register => shim_irq_domain_register,
        irq_domain_unregister => shim_irq_domain_unregister,
        irq_map => shim_irq_map,
        irq_unmap => shim_irq_unmap,
        irq_of_device => shim_irq_of_device,
        irq_request => shim_irq_request,
        irq_release => shim_irq_release,
        irq_mask => shim_irq_mask,
        irq_unmask => shim_irq_unmask,
        irq_set_affinity => shim_irq_set_affinity,
        irq_alloc_vector => shim_irq_alloc_vector,
        irq_free_vector => shim_irq_free_vector,
        irq_compose_message => shim_irq_compose_message,
        mmio_map => shim_mmio_map,
        mmio_unmap => shim_mmio_unmap,
        mmio_direct => shim_mmio_direct,
        port_read8 => shim_port_read8,
        port_read16 => shim_port_read16,
        port_read32 => shim_port_read32,
        port_write8 => shim_port_write8,
        port_write16 => shim_port_write16,
        port_write32 => shim_port_write32,
        dma_alloc => shim_dma_alloc,
        dma_free => shim_dma_free,
        dma_map => shim_dma_map,
        dma_unmap => shim_dma_unmap,
        dma_sync => shim_dma_sync,
        work_create => shim_work_create,
        work_queue => shim_work_queue,
        work_flush => shim_work_flush,
        work_destroy => shim_work_destroy,
        timer_create => shim_timer_create,
        timer_arm => shim_timer_arm,
        timer_cancel => shim_timer_cancel,
        timer_destroy => shim_timer_destroy,
        event_create => shim_event_create,
        event_destroy => shim_event_destroy,
        event_wait => shim_event_wait,
        event_signal => shim_event_signal,
        event_reset => shim_event_reset,
        time_monotonic => shim_time_monotonic,
        time_delay => shim_time_delay,
        time_sleep => shim_time_sleep,
        random_fill => shim_random_fill,
        random_mix => shim_random_mix,
        cpu_count => shim_cpu_count,
        cpu_current => shim_cpu_current,
        cpu_platform_id => shim_cpu_platform_id,
        in_interrupt => shim_in_interrupt,
        firmware_acpi => shim_firmware_acpi,
        firmware_devicetree => shim_firmware_devicetree,
        devfs_root => shim_devfs_root,
        devfs_mkdir => shim_devfs_mkdir,
        devfs_create => shim_devfs_create,
        devfs_remove => shim_devfs_remove,
        devfs_lookup => shim_devfs_lookup,
        tty_register => shim_tty_register,
        tty_unregister => shim_tty_unregister,
        klog_level => shim_klog_level,
        klog_set_level => shim_klog_set_level,
        klog_console_level => shim_klog_console_level,
        klog_set_console_level => shim_klog_set_console_level,
        tty_provider_register => shim_tty_provider_register,
        tty_provider_unregister => shim_tty_provider_unregister,
        event_wait_any => shim_event_wait_any,
        event_wait_timeout => shim_event_wait_timeout,
        worker_spawn => shim_worker_spawn,
        worker_join => shim_worker_join,
        process_group_signal => shim_process_group_signal,
        fs_provider_register => shim_fs_provider_register,
        fs_provider_unregister => shim_fs_provider_unregister,
        fs_page_account_create => shim_fs_page_account_create,
        fs_page_account_release => shim_fs_page_account_release,
        fs_page_account_limit => shim_fs_page_account_limit,
        fs_page_account_used => shim_fs_page_account_used,
        fs_memory_object_create => shim_fs_memory_object_create,
        fs_memory_object_retain => shim_fs_memory_object_retain,
        fs_memory_object_release => shim_fs_memory_object_release,
        fs_memory_object_read => shim_fs_memory_object_read,
        fs_memory_object_write => shim_fs_memory_object_write,
        fs_memory_object_truncate => shim_fs_memory_object_truncate,
        fs_memory_object_page_count => shim_fs_memory_object_page_count,
        fs_total_physical_pages => shim_fs_total_physical_pages,
        devfs_broker_register => shim_devfs_broker_register,
        devfs_broker_unregister => shim_devfs_broker_unregister,
        devfs_endpoint_open => shim_devfs_endpoint_open,
        devfs_endpoint_close => shim_devfs_endpoint_close,
        devfs_endpoint_initial_offset => shim_devfs_endpoint_initial_offset,
        devfs_endpoint_read => shim_devfs_endpoint_read,
        devfs_endpoint_write => shim_devfs_endpoint_write,
        devfs_endpoint_size => shim_devfs_endpoint_size,
        devfs_endpoint_sync => shim_devfs_endpoint_sync,
        devfs_endpoint_poll => shim_devfs_endpoint_poll,
        devfs_endpoint_event => shim_devfs_endpoint_event,
        devfs_endpoint_terminal_state => shim_devfs_endpoint_terminal_state,
        devfs_endpoint_ioctl => shim_devfs_endpoint_ioctl,
        devfs_endpoint_release => shim_devfs_endpoint_release,
    )
}
