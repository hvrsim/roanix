//! Kernel-provided platform services.
//!
//! The kernel registers exactly one bus itself: `platform`, the namespace for
//! devices that firmware describes by physical address rather than by
//! enumeration. It carries no hardware knowledge - parsing a device tree or an
//! ACPI namespace, routing interrupts, and driving registers all belong to
//! drivers - but it must exist before the first enumerator loads so that
//! enumerators and device drivers agree on where to meet.

use alloc::sync::Arc;

use crate::sys::sync::Once;

use super::{
    core::bus::{self, Bus, BusOps},
    error::{Error, Result},
};

/// Name of the bus carrying firmware-described devices.
pub const PLATFORM_BUS: &str = "platform";

static PLATFORM: Once<Arc<Bus>> = Once::new();

pub(super) fn init() {
    // SAFETY: the bus is kernel-owned and installs no callbacks.
    let bus = unsafe { bus::register(None, PLATFORM_BUS, BusOps::default()) }
        .expect("driver: failed to register the platform bus");
    PLATFORM.call_once(|| bus);
}

/// Returns the bus carrying firmware-described devices.
pub fn platform_bus() -> Result<Arc<Bus>> {
    PLATFORM.get().cloned().ok_or(Error::NotInitialized)
}
