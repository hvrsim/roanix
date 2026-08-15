//!
//! # Driver Framework
//!
//! Roanix drivers are C modules that run in the kernel and may implement any
//! kernel-visible functionality, not just device access. The framework is
//! organized around five ideas:
//!
//! * A **device** is a uniform node in one tree. There is no separate bus node
//!   type, so a bridge, a hub, or a controller is an ordinary device that
//!   happens to have children.
//! * A **bus** describes how a family of devices is enumerated, matched, and
//!   addressed. Buses are registered by modules, so PCI, USB, or I2C support is
//!   added without touching the framework.
//! * A **driver** binds to devices through a match table that understands
//!   device-tree compatible strings, ACPI identifiers, and masked numeric
//!   identifier tables of the kind PCI and USB use.
//! * An **interface** is a versioned operation table published under a name.
//!   It is the only mechanism for one driver to call another, and it has no
//!   ancestry requirement, so any driver can consume any service. A driver that
//!   needs a service which has not appeared yet defers, and the probe engine
//!   retries it when the topology changes.
//! * A **class** groups devices that share a software contract and notifies its
//!   owner as members come and go, which is what allows an entire subsystem to
//!   be implemented as a driver.
//!
//! Handles crossing the C boundary are pointers to reference-counted objects
//! rather than identifiers resolved through a table, so no lock is taken and no
//! lookup is performed to make a framework call. Register access, after the
//! window is mapped, involves no framework call at all.
//!

pub mod abi;
pub mod class;
pub mod core;
pub mod error;
pub mod io;
pub mod irq;
pub mod obj;
pub mod platform;

pub use core::{
    bus::Bus,
    class::{Class, ClassDevice},
    device::{Device, DeviceBuilder},
    driver::Driver,
    iface::{Interface, InterfaceRef},
    module::{Module, ModuleId},
};
pub use error::{Error, Result};

pub mod work;

/// Initializes the driver framework.
///
/// Runs after memory services are available and before any address space other
/// than the kernel's exists, which is what lets the register window's page
/// tables be shared by every process created later.
pub fn init() {
    core::init();
    irq::init();
    io::init();
    work::init();
    class::init();
    platform::init();
}

/// Loads the modules packaged in the initial filesystem.
pub fn load_packaged_modules() -> Result<usize> {
    let loaded = abi::loader::load_directory(b"/usr/lib/roanix/drivers")?;
    log::info!("loaded {loaded} driver module(s)");
    core::probe::report_unbound();
    Ok(loaded)
}
