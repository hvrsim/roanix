//! Core object model: modules, devices, drivers, buses, classes, and
//! interfaces.

pub mod bus;
pub mod class;
pub mod device;
pub mod driver;
pub mod fwnode;
pub mod iface;
pub mod match_table;
pub mod name;
pub mod module;
pub mod probe;
pub mod property;
pub mod resource;

/// Initializes every core registry.
pub(super) fn init() {
    module::init();
    device::init();
    driver::init();
    bus::init();
    class::init();
    iface::init();
    probe::init();
}
