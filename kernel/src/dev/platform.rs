//! Kernel-owned console bus registration.

use alloc::sync::Arc;

use crate::sys::sync::Once;

use super::{
    BusId, Error, KERNEL_DRIVER, Result,
    abi,
    resource::{ResourceFlags, ResourceKey, ResourceValue},
};

/// Namespace for boot firmware resources supplied by Limine.
pub const FIRMWARE_NAMESPACE: u64 = 0x524F_414E_4958_4657;
/// Validated ACPI RSDP bytes.
pub const FIRMWARE_ACPI_RSDP_RESOURCE: ResourceKey = ResourceKey::new(FIRMWARE_NAMESPACE, 1);
/// Validated flattened device-tree bytes.
pub const FIRMWARE_DTB_RESOURCE: ResourceKey = ResourceKey::new(FIRMWARE_NAMESPACE, 2);

/// Kernel device-node frontend protocol.
pub const DEVICE_FRONTEND_RESOURCE: ResourceKey =
    ResourceKey::new(0x524F_414E_4958_4445, 1);
/// Console-subsystem capability inherited by TTY devices.
pub const CONSOLE_RESOURCE: ResourceKey = ResourceKey::new(0x524F_414E_4958_434F, 1);
/// Kernel console/TTY publication protocol.
pub const CONSOLE_SERVICE_RESOURCE: ResourceKey =
    ResourceKey::new(0x524F_414E_4958_434F, 2);

const CONSOLE_FEATURE_CANONICAL: u32 = 1 << 0;
const CONSOLE_FEATURE_RAW: u32 = 1 << 1;
const CONSOLE_FEATURE_TERMIOS: u32 = 1 << 2;
const CONSOLE_FEATURE_WINSIZE: u32 = 1 << 3;
const CONSOLE_FEATURE_FLOW_CONTROL: u32 = 1 << 4;
const CONSOLE_FEATURE_JOB_CONTROL: u32 = 1 << 5;
const CONSOLE_FEATURE_EVENT_IO: u32 = 1 << 6;

/// Registered platform bus handles.
#[derive(Copy, Clone)]
pub struct PlatformBuses {
    /// Character-console capability bus.
    pub console: BusId,
}

static BUSES: Once<PlatformBuses> = Once::new();

pub(super) fn init() -> Result<()> {
    if BUSES.get().is_some() {
        return Ok(());
    }

    let root = super::root_bus()?;
    publish_firmware_resources(root)?;
    let protocol_flags = ResourceFlags::from_bits(
        ResourceFlags::CACHEABLE.bits()
            | ResourceFlags::MAY_BLOCK.bits()
            | ResourceFlags::ORDERLY.bits()
            | ResourceFlags::CONCURRENT.bits(),
    );
    super::tree::publish_root_resource(
        KERNEL_DRIVER,
        root.node(),
        DEVICE_FRONTEND_RESOURCE,
        protocol_flags,
        ResourceValue::Protocol(abi::device_frontend_protocol()),
    )?;
    super::tree::publish_root_resource(
        KERNEL_DRIVER,
        root.node(),
        CONSOLE_SERVICE_RESOURCE,
        protocol_flags,
        ResourceValue::Protocol(abi::console_service_protocol()),
    )?;

    let console = super::register_bus(KERNEL_DRIVER, root.node(), "console")?;
    super::publish_resource(
        KERNEL_DRIVER,
        console.node(),
        CONSOLE_RESOURCE,
        ResourceFlags::CACHEABLE,
        ResourceValue::Data(console_capability()),
    )?;
    BUSES.call_once(|| PlatformBuses { console });
    Ok(())
}

fn publish_firmware_resources(root: BusId) -> Result<()> {
    #[cfg(target_arch = "x86_64")]
    if let Some(data) = crate::sys::firmware::acpi_rsdp() {
        super::tree::publish_root_resource(
            KERNEL_DRIVER,
            root.node(),
            FIRMWARE_ACPI_RSDP_RESOURCE,
            ResourceFlags::CACHEABLE,
            ResourceValue::Data(Arc::from(data)),
        )?;
    }

    #[cfg(target_arch = "riscv64")]
    if let Some(data) = crate::sys::firmware::dtb() {
        super::tree::publish_root_resource(
            KERNEL_DRIVER,
            root.node(),
            FIRMWARE_DTB_RESOURCE,
            ResourceFlags::CACHEABLE,
            ResourceValue::Data(Arc::from(data)),
        )?;
    }

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
        | CONSOLE_FEATURE_FLOW_CONTROL
        | CONSOLE_FEATURE_JOB_CONTROL
        | CONSOLE_FEATURE_EVENT_IO;
    Arc::from(features.to_le_bytes())
}
