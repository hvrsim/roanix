//!
//! # Device Subsystem
//!
//! Hierarchical devices, buses, inheritable resources, and foreign drivers.
//!

pub mod abi;
mod binding;
pub mod console;
mod dependency;
mod driver;
mod module;
pub mod error;
pub mod interrupt;
mod platform;
pub mod resource;
mod tree;

pub use driver::{
    DriverInfo, info as driver_info, load as load_driver, loaded as loaded_drivers,
    unload as unload_driver,
};
pub(crate) use driver::{
    callback_guard, close_callback_guard, mutation_guard, parent_guard,
};
pub use error::{Error, Result};
pub use platform::{
    CONSOLE_RESOURCE, CONSOLE_SERVICE_RESOURCE, DEVICE_FRONTEND_RESOURCE,
    FIRMWARE_ACPI_RSDP_RESOURCE, FIRMWARE_DTB_RESOURCE, PlatformBuses,
    buses as platform_buses,
};
pub use resource::{
    MemoryRegion, Resource, ResourceFlags, ResourceId, ResourceKey, ResourceLeaseId,
    ResourceProtocol, ResourceValue, PROPERTY_CLASS, PROPERTY_COMPATIBLE,
    PROPERTY_DEVICE_TYPE, PROPERTY_MODALIAS, PROPERTY_NAMESPACE, PROPERTY_SUBSYSTEM,
};
pub use tree::{
    BusId, DeviceId, DeviceNodeId, DriverId, KERNEL_DRIVER, NodeInfo, NodeKind, acquire_resource,
    children, children_of, leased_resource, node_info, property, publish_resource,
    register_bus, register_device, release_resource, remove_node, remove_resource,
    resolve_resource, root_bus, set_property,
};

/// Initializes the device hierarchy.
pub fn init() {
    tree::init();
    driver::init();
    interrupt::init();
    dependency::init();
    binding::init();
    platform::init().expect("dev: failed to register platform buses");
}

/// Loads packaged shared-object drivers from the initial filesystem.
pub fn start_external_drivers() -> Result<usize> {
    driver::load_directory("/usr/lib/roanix/drivers")
}
