//! Core object model for hardware modules, devices, buses, drivers, and
//! interfaces.

pub mod bus;
pub mod device;
pub mod driver;
pub mod fwnode;
pub mod iface;
pub mod match_table;
pub mod module;
pub mod name;
pub mod probe;
pub mod property;
pub mod resource;

/// Initializes every core registry.
pub(super) fn init() {
    module::init();
    device::init();
    driver::init();
    bus::init();
    iface::init();
    probe::init();
}
