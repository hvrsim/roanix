//! Driver registration and the driver side of binding.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::sys::sync::{Mutex, Once};

use super::{
    super::{
        error::{self, Error, Result},
        obj::{ObjHeader, ObjKind, framework_object},
    },
    bus::Bus,
    device::Device,
    match_table::{self, MatchEntry, MatchResult},
    module::{self, Module},
};

/// Maximum driver-name length in bytes.
pub const MAX_NAME: usize = 64;

/// Claims a device. Returning [`Error::Deferred`] parks the device for retry.
pub type ProbeFn = unsafe extern "C" fn(
    context: *mut c_void,
    device: *const c_void,
    match_data: usize,
) -> i32;
/// Releases a device previously claimed by [`ProbeFn`].
pub type RemoveFn = unsafe extern "C" fn(context: *mut c_void, device: *const c_void);

/// Callbacks a driver implements.
pub struct DriverOps {
    /// Required probe callback.
    pub probe: ProbeFn,
    /// Optional remove callback.
    pub remove: Option<RemoveFn>,
    /// Optional callback invoked during orderly system shutdown.
    pub shutdown: Option<RemoveFn>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// A registered driver.
#[repr(C)]
pub struct Driver {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    bus: Option<Arc<Bus>>,
    priority: i32,
    matches: Box<[MatchEntry]>,
    ops: DriverOps,
    bound: AtomicU64,
}

framework_object!(Driver, Driver);

// SAFETY: driver callbacks and context stay valid until the owning module is
// unloaded, and unload first unbinds every device this driver claimed.
unsafe impl Send for Driver {}
// SAFETY: the framework serialises binding transitions per device, and the ABI
// requires probe and remove to tolerate concurrent invocation on distinct
// devices.
unsafe impl Sync for Driver {}

impl Driver {
    /// Returns the driver name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the module that registered this driver.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns the bus this driver binds on, if it is restricted to one.
    pub fn bus(&self) -> Option<&Arc<Bus>> {
        self.bus.as_ref()
    }

    /// Returns the tie-breaking priority applied to every match.
    pub const fn priority(&self) -> i32 {
        self.priority
    }

    /// Returns the number of devices currently bound to this driver.
    pub fn bound_count(&self) -> u64 {
        self.bound.load(Ordering::Acquire)
    }

    /// Scores this driver against `device`.
    pub(super) fn evaluate(&self, device: &Arc<Device>) -> Result<Option<MatchResult>> {
        if let Some(bus) = device.bus() {
            match self.bus.as_ref() {
                Some(required) if !Arc::ptr_eq(required, bus) => return Ok(None),
                None => return Ok(None),
                _ => {}
            }
            if let Some(result) = bus.match_device(device, &self.arc()?) {
                let score = result?;
                return Ok((score > 0).then_some(MatchResult {
                    index: 0,
                    data: 0,
                    score: score + self.priority,
                }));
            }
        } else if self.bus.is_some() {
            return Ok(None);
        }

        Ok(match_table::best(&self.matches, device).map(|mut result| {
            result.score += self.priority;
            result
        }))
    }

    fn arc(&self) -> Result<Arc<Driver>> {
        let Ok(registry) = registry() else {
            return Err(Error::NotInitialized);
        };
        registry
            .drivers
            .lock()
            .iter()
            .find(|driver| core::ptr::eq(Arc::as_ptr(driver), self))
            .cloned()
            .ok_or(Error::NotFound)
    }

    pub(super) fn probe(&self, device: &Arc<Device>, match_data: usize) -> Result<()> {
        let handle = super::super::obj::handle(device);
        // SAFETY: registration validated the callback, and the handle addresses
        // a live device that the probe engine keeps alive across the call.
        let status = unsafe { (self.ops.probe)(self.ops.context, handle, match_data) };
        error::from_status(status)
    }

    pub(super) fn remove(&self, device: &Arc<Device>) {
        let Some(callback) = self.ops.remove else {
            return;
        };
        let handle = super::super::obj::handle(device);
        // SAFETY: registration validated the callback, and the handle addresses
        // a live device that the probe engine keeps alive across the call.
        unsafe { callback(self.ops.context, handle) };
    }

    pub(super) fn note_bound(&self) {
        self.bound.fetch_add(1, Ordering::AcqRel);
    }

    pub(super) fn note_unbound(&self) {
        let previous = self.bound.fetch_sub(1, Ordering::Release);
        debug_assert!(previous != 0, "driver: bound count underflow");
    }

    /// Pins the module owning this driver across a callback.
    pub(super) fn pin(&self, allow_unloading: bool) -> Result<module::ModuleGuard> {
        module::pin_owner(self.owner.as_ref(), allow_unloading)
    }
}

/// Description supplied when registering a driver.
pub struct Registration {
    /// Driver name, unique among registered drivers.
    pub name: Box<str>,
    /// Bus this driver binds on, or `None` to match bus-less devices.
    pub bus: Option<Arc<Bus>>,
    /// Score added to every successful match, used to order competing drivers.
    pub priority: i32,
    /// Match table describing the hardware this driver supports.
    pub matches: Box<[MatchEntry]>,
    /// Driver callbacks.
    pub ops: DriverOps,
}

struct Registry {
    drivers: Mutex<Vec<Arc<Driver>>>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        drivers: Mutex::new(Vec::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// Registers a driver and immediately offers it every unbound device.
///
/// # Safety
///
/// Every callback in `registration.ops` must follow the driver ABI and stay
/// executable until the driver is unregistered.
pub unsafe fn register(
    owner: Option<&Arc<Module>>,
    registration: Registration,
) -> Result<Arc<Driver>> {
    if registration.name.is_empty() || registration.name.len() > MAX_NAME {
        return Err(Error::InvalidArgument);
    }
    match_table::validate(&registration.matches)?;

    let registry = registry()?;
    let mut drivers = registry.drivers.lock();
    if drivers
        .iter()
        .any(|driver| driver.name == registration.name)
    {
        return Err(Error::AlreadyExists);
    }
    let driver = Arc::new(Driver {
        header: ObjHeader::new(ObjKind::Driver),
        name: registration.name,
        owner: owner.cloned(),
        bus: registration.bus,
        priority: registration.priority,
        matches: registration.matches,
        ops: registration.ops,
        bound: AtomicU64::new(0),
    });
    drivers.push(driver.clone());
    drop(drivers);

    super::probe::offer_driver(&driver);
    Ok(driver)
}

/// Unregisters a driver, unbinding every device it claimed.
pub fn unregister(driver: &Arc<Driver>) -> Result<()> {
    super::probe::detach_driver(driver);
    if driver.bound_count() != 0 {
        return Err(Error::Busy);
    }
    force_unregister(driver);
    super::probe::retrigger();
    Ok(())
}

fn force_unregister(driver: &Arc<Driver>) {
    if let Ok(registry) = registry() {
        registry
            .drivers
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, driver));
    }
    driver.header.poison();
}

/// Returns every registered driver.
pub(super) fn all() -> Vec<Arc<Driver>> {
    registry()
        .map(|registry| registry.drivers.lock().clone())
        .unwrap_or_default()
}

pub(super) fn bus_has_drivers(bus: &Arc<Bus>) -> bool {
    all().iter().any(|driver| {
        driver
            .bus
            .as_ref()
            .is_some_and(|current| Arc::ptr_eq(current, bus))
    })
}

pub(super) fn remove_module_drivers(module: &Arc<Module>) {
    let owned: Vec<Arc<Driver>> = all()
        .into_iter()
        .filter(|driver| {
            driver
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .collect();
    for driver in owned {
        super::probe::detach_driver(&driver);
        force_unregister(&driver);
    }
}

/// Snapshot of one registered driver.
#[derive(Clone, Debug)]
pub struct DriverInfo {
    /// Driver name.
    pub name: Box<str>,
    /// Bus name, if the driver is restricted to one.
    pub bus: Option<Box<str>>,
    /// Number of devices currently bound.
    pub bound: u64,
}

/// Returns a snapshot of every registered driver.
pub fn list() -> Vec<DriverInfo> {
    all()
        .into_iter()
        .map(|driver| DriverInfo {
            name: driver.name.to_string().into_boxed_str(),
            bus: driver
                .bus
                .as_ref()
                .map(|bus| bus.name().to_string().into_boxed_str()),
            bound: driver.bound_count(),
        })
        .collect()
}

/// Returns a registered driver by name.
pub fn find(name: &str) -> Result<Arc<Driver>> {
    registry()?
        .drivers
        .lock()
        .iter()
        .find(|driver| &*driver.name == name)
        .cloned()
        .ok_or(Error::NotFound)
}

/// Creates a registration description from raw parts.
pub fn registration(
    name: &str,
    bus: Option<Arc<Bus>>,
    priority: i32,
    matches: Vec<MatchEntry>,
    ops: DriverOps,
) -> Result<Registration> {
    Ok(Registration {
        name: String::from(name).into_boxed_str(),
        bus,
        priority,
        matches: match_table::build(matches)?,
        ops,
    })
}
