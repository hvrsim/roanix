//!
//! # Device Subsystem
//!
//! Hierarchical devices, buses, inheritable resources, and foreign drivers.
//!

pub mod abi;
mod driver;
#[cfg(target_arch = "riscv64")]
pub mod dtb;
pub mod error;
pub mod resource;
mod tree;

pub use driver::{
    DriverInfo, info as driver_info, load as load_driver, loaded as loaded_drivers,
    unload as unload_driver,
};
pub(crate) use driver::{callback_guard, close_callback_guard, mutation_guard, parent_guard};
pub use error::{Error, Result};
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
}

/// Loads statically linked driver descriptors after devtempfs is mounted.
pub fn start_linked_drivers() -> Result<()> {
    driver::load_linked()
}
