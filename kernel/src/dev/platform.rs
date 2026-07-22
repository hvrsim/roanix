//! Platform firmware and console bus registration.

use alloc::sync::Arc;

use crate::sys::sync::Once;

use super::{
    BusId, Error, KERNEL_DRIVER, Result,
    resource::{ResourceFlags, ResourceKey, ResourceValue},
};

/// Versioned console-subsystem capability inherited by TTY devices.
pub const CONSOLE_RESOURCE: ResourceKey = ResourceKey::new(0x524F_414E_4958_434F, 1);

const CONSOLE_CAPABILITY_VERSION: u32 = 1;
const CONSOLE_FEATURE_CANONICAL: u32 = 1 << 0;
const CONSOLE_FEATURE_RAW: u32 = 1 << 1;
const CONSOLE_FEATURE_TERMIOS: u32 = 1 << 2;
const CONSOLE_FEATURE_WINSIZE: u32 = 1 << 3;
const CONSOLE_FEATURE_FLOW_CONTROL: u32 = 1 << 4;

/// Registered platform bus handles.
#[derive(Copy, Clone)]
pub struct PlatformBuses {
    /// ACPI or DTB firmware root.
    pub firmware: BusId,
    /// Character-console capability bus.
    pub console: BusId,
}

static BUSES: Once<PlatformBuses> = Once::new();

pub(super) fn init() -> Result<()> {
    if BUSES.get().is_some() {
        return Ok(());
    }

    let root = super::root_bus()?;
    #[cfg(target_arch = "x86_64")]
    let firmware = {
        let bus = super::register_bus(KERNEL_DRIVER, root, "acpi")?;
        super::acpi::publish_rsdp(bus)?;
        bus
    };
    #[cfg(target_arch = "riscv64")]
    let firmware = {
        let bus = super::register_bus(KERNEL_DRIVER, root, "dtb")?;
        let blob: Arc<[u8]> = Arc::from(super::dtb::blob().ok_or(Error::NotFound)?);
        super::publish_resource(
            KERNEL_DRIVER,
            bus,
            super::dtb::DTB_RESOURCE,
            ResourceFlags::CACHEABLE,
            ResourceValue::Data(blob),
        )?;
        bus
    };

    let console = super::register_bus(KERNEL_DRIVER, firmware, "console")?;
    super::publish_resource(
        KERNEL_DRIVER,
        console,
        CONSOLE_RESOURCE,
        ResourceFlags::CACHEABLE,
        ResourceValue::Data(console_capability()),
    )?;
    BUSES.call_once(|| PlatformBuses { firmware, console });
    Ok(())
}

/// Returns the registered platform bus handles.
pub fn buses() -> Result<PlatformBuses> {
    BUSES.get().copied().ok_or(Error::NotInitialized)
}

fn console_capability() -> Arc<[u8]> {
    let features = CONSOLE_FEATURE_CANONICAL
        | CONSOLE_FEATURE_RAW
        | CONSOLE_FEATURE_TERMIOS
        | CONSOLE_FEATURE_WINSIZE
        | CONSOLE_FEATURE_FLOW_CONTROL;
    let mut bytes = [0u8; 8];
    bytes[..4].copy_from_slice(&CONSOLE_CAPABILITY_VERSION.to_le_bytes());
    bytes[4..].copy_from_slice(&features.to_le_bytes());
    Arc::from(bytes)
}
