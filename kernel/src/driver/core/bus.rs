//! Bus types.
//!
//! A bus describes how a family of devices is enumerated, addressed, and
//! interrupted. Buses are registered by ordinary driver modules, so support for
//! PCI, USB, I2C, or virtio is added without touching the framework core.
//!
//! The bus is consulted at three points: when a driver is matched against a
//! device, before a driver probes, and after it is removed. This is where
//! bus-specific setup such as enabling PCI decoding or powering a USB port
//! belongs, so individual device drivers stay free of transport details.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::ffi::c_void;

use crate::sys::sync::{Mutex, Once};

use super::{
    super::{
        error::{Error, Result},
        obj::{ObjHeader, ObjKind, framework_object},
    },
    device::Device,
    driver::Driver,
    module::{self, Module},
};

/// Maximum bus-name length in bytes.
pub const MAX_NAME: usize = 64;

/// Scores a driver against a device, overriding table matching.
///
/// A positive return value is a match score, zero means no match, and a
/// negative value is a status.
pub type MatchFn =
    unsafe extern "C" fn(context: *mut c_void, device: *const c_void, driver: *const c_void) -> i32;
/// Prepares a device before its driver probes.
pub type PrepareFn = unsafe extern "C" fn(context: *mut c_void, device: *const c_void) -> i32;
/// Releases bus state after a driver has been removed.
pub type CleanupFn = unsafe extern "C" fn(context: *mut c_void, device: *const c_void);

/// Callbacks a bus may implement.
#[derive(Default)]
pub struct BusOps {
    /// Optional replacement for match-table scoring.
    pub match_device: Option<MatchFn>,
    /// Optional pre-probe hook.
    pub prepare: Option<PrepareFn>,
    /// Optional post-remove hook.
    pub cleanup: Option<CleanupFn>,
    /// Optional shutdown hook invoked during system teardown.
    pub shutdown: Option<CleanupFn>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// A registered bus type.
#[repr(C)]
pub struct Bus {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    ops: BusOps,
    dma_ops: Mutex<*const c_void>,
}

framework_object!(Bus, Bus);

// SAFETY: bus callbacks and the context pointer stay valid until the owning
// module is unloaded, which cannot happen while devices reference the bus.
unsafe impl Send for Bus {}
// SAFETY: the framework serialises bus registration, and callbacks are
// required by the ABI to tolerate concurrent invocation.
unsafe impl Sync for Bus {}

impl Bus {
    /// Returns the bus name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the module that registered this bus.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns the DMA operation table this bus imposes on its devices.
    pub fn dma_ops(&self) -> *const c_void {
        *self.dma_ops.lock()
    }

    /// Installs a DMA operation table for devices on this bus.
    ///
    /// # Safety
    ///
    /// `ops` must address an immutable table following the DMA operation ABI
    /// and must stay valid until the bus is unregistered.
    pub unsafe fn set_dma_ops(&self, ops: *const c_void) {
        *self.dma_ops.lock() = ops;
    }

    /// Scores `driver` against `device` using the bus override, if any.
    ///
    /// Returns `None` when the bus has no override and table matching applies.
    pub(super) fn match_device(
        &self,
        device: &Arc<Device>,
        driver: &Arc<Driver>,
    ) -> Option<Result<i32>> {
        let callback = self.ops.match_device?;
        let device_handle = super::super::obj::handle(device);
        let driver_handle = super::super::obj::handle(driver);
        // SAFETY: registration validated the callback, and both handles address
        // live objects that the caller keeps alive across the call.
        let score = unsafe { callback(self.ops.context, device_handle, driver_handle) };
        Some(if score < 0 {
            Err(Error::from_status(score))
        } else {
            Ok(score)
        })
    }

    pub(super) fn prepare(&self, device: &Arc<Device>) -> Result<()> {
        let Some(callback) = self.ops.prepare else {
            return Ok(());
        };
        let handle = super::super::obj::handle(device);
        // SAFETY: registration validated the callback and the handle addresses
        // a live device for the duration of the call.
        super::super::error::from_status(unsafe { callback(self.ops.context, handle) })
    }

    pub(super) fn cleanup(&self, device: &Arc<Device>) {
        let Some(callback) = self.ops.cleanup else {
            return;
        };
        let handle = super::super::obj::handle(device);
        // SAFETY: registration validated the callback and the handle addresses
        // a live device for the duration of the call.
        unsafe { callback(self.ops.context, handle) };
    }
}

struct Registry {
    buses: Mutex<Vec<Arc<Bus>>>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        buses: Mutex::new(Vec::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// Registers a bus type.
///
/// # Safety
///
/// Every callback in `ops` must follow the bus ABI and stay executable until
/// the bus is unregistered.
pub unsafe fn register(
    owner: Option<&Arc<Module>>,
    name: &str,
    ops: BusOps,
) -> Result<Arc<Bus>> {
    if name.is_empty() || name.len() > MAX_NAME {
        return Err(Error::InvalidArgument);
    }
    let registry = registry()?;
    let mut buses = registry.buses.lock();
    if buses.iter().any(|bus| &*bus.name == name) {
        return Err(Error::AlreadyExists);
    }
    let bus = Arc::new(Bus {
        header: ObjHeader::new(ObjKind::Bus),
        name: String::from(name).into_boxed_str(),
        owner: owner.cloned(),
        ops,
        dma_ops: Mutex::new(core::ptr::null()),
    });
    buses.push(bus.clone());
    drop(buses);
    super::probe::retrigger();
    Ok(bus)
}

/// Unregisters a bus once no device is attached to it.
pub fn unregister(bus: &Arc<Bus>) -> Result<()> {
    let mut attached = false;
    super::device::for_each(|device| {
        if device
            .bus()
            .is_some_and(|current| Arc::ptr_eq(current, bus))
        {
            attached = true;
        }
    });
    if attached {
        return Err(Error::Busy);
    }
    if super::driver::bus_has_drivers(bus) {
        return Err(Error::Busy);
    }
    force_unregister(bus);
    Ok(())
}

fn force_unregister(bus: &Arc<Bus>) {
    if let Ok(registry) = registry() {
        registry
            .buses
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, bus));
    }
    bus.header.poison();
}

/// Returns a registered bus by name.
pub fn find(name: &str) -> Result<Arc<Bus>> {
    registry()?
        .buses
        .lock()
        .iter()
        .find(|bus| &*bus.name == name)
        .cloned()
        .ok_or(Error::NotFound)
}

pub(super) fn remove_module_buses(module: &Arc<Module>) {
    let Ok(registry) = registry() else {
        return;
    };
    let owned: Vec<Arc<Bus>> = registry
        .buses
        .lock()
        .iter()
        .filter(|bus| {
            bus.owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for bus in owned {
        force_unregister(&bus);
    }
}

/// Pins the module owning `bus` across a bus callback.
pub(super) fn pin(bus: &Arc<Bus>) -> Result<module::ModuleGuard> {
    module::pin_owner(bus.owner.as_ref(), false)
}

/// Returns the names of every registered bus.
pub fn list() -> Result<Vec<Box<str>>> {
    Ok(registry()?
        .buses
        .lock()
        .iter()
        .map(|bus| bus.name.to_string().into_boxed_str())
        .collect())
}
