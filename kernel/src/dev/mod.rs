//!
//! # Device Subsystem
//!
//! Hierarchical devices, buses, inheritable resources, and foreign drivers.
//!

pub mod abi;
#[cfg(target_arch = "x86_64")]
mod acpi;
pub mod console;
mod driver;
#[cfg(target_arch = "riscv64")]
pub mod dtb;
pub mod error;
mod platform;
pub mod resource;
mod serial;
mod tree;

pub use driver::{
    DriverInfo, info as driver_info, load as load_driver, loaded as loaded_drivers,
    unload as unload_driver,
};
pub(crate) use driver::{callback_guard, close_callback_guard, mutation_guard, parent_guard};
pub use error::{Error, Result};
pub use platform::{PlatformBuses, buses as platform_buses};
pub use resource::{
    Resource, ResourceCallback, ResourceFlags, ResourceId, ResourceKey, ResourceMethod,
    ResourceValue,
};
pub use tree::{
    BusId, DeviceId, DeviceNodeId, DriverId, KERNEL_DRIVER, NodeInfo, NodeKind, children,
    node_info, publish_resource, register_bus, register_device, remove_node, remove_resource,
    resolve_resource, root_bus,
};

/// Initializes the device hierarchy.
pub fn init() {
    tree::init();
    driver::init();
    platform::init().expect("dev: failed to register platform buses");
    serial::discover().expect("dev: failed to discover platform UARTs");
}

/// Loads statically linked driver descriptors after devtempfs is mounted.
pub fn start_linked_drivers() -> Result<()> {
    driver::load_linked()
}

/// Probes built-in platform drivers and publishes their device nodes.
pub fn start_platform_drivers() -> Result<usize> {
    serial::start()
}
