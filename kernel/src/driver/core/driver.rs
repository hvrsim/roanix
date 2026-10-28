//! Driver registration and the driver side of binding.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::{Arc, Weak},
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
        obj::{ObjHeader, ObjKind, ObjState, Object, framework_object},
    },
    bus::Bus,
    device::Device,
    match_table::{self, MatchEntry, MatchResult},
    module::{self, Module},
};

/// Maximum driver-name length in bytes.
pub const MAX_NAME: usize = 64;

/// Claims a device. Returning [`Error::Deferred`] parks the device for retry.
pub type ProbeFn =
    unsafe extern "C" fn(context: *mut c_void, device: *const c_void, match_data: usize) -> i32;
/// Releases a device previously claimed by [`ProbeFn`].
pub type RemoveFn = unsafe extern "C" fn(context: *mut c_void, device: *const c_void);

/// Callbacks a driver implements.
pub struct DriverOps {
    /// Required probe callback.
    pub probe: ProbeFn,
    /// Legacy mandatory remove callback, which must complete disconnection.
    pub remove: Option<RemoveFn>,
    /// Checked removal; errors preserve the binding and framework resources.
    pub try_remove: Option<unsafe extern "C" fn(*mut c_void, *const c_void) -> i32>,
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
    bus_lease: Mutex<Option<module::ModuleLease>>,
    priority: i32,
    matches: Box<[MatchEntry]>,
    ops: DriverOps,
    bound: AtomicU64,
    // Serializes probe admission with withdrawing this driver from matching.
    // The count covers a complete binding transaction, including publication
    // of BOUND, rather than only the module's callback body.
    probes: Mutex<usize>,
    /// Set at registration; lets `evaluate` clone a handle without a registry
    /// lock and linear scan on every match attempt.
    self_ref: Mutex<Weak<Driver>>,
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
    pub(super) fn begin_probe(&self) -> Result<ProbeActivity<'_>> {
        let mut probes = self.probes.lock();
        if self.header.state() != ObjState::Live {
            return Err(Error::NoDevice);
        }
        *probes = probes.checked_add(1).ok_or(Error::Busy)?;
        Ok(ProbeActivity(self))
    }
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
        // A snapshot taken before an unregister claim must not match.
        if self.header.state() != ObjState::Live {
            return Ok(None);
        }
        if let Some(bus) = device.bus() {
            match self.bus.as_ref() {
                Some(required) if !Arc::ptr_eq(required, bus) => return Ok(None),
                None => return Ok(None),
                _ => {}
            }
            if let Some(result) = bus.match_device(device, &self.arc()?) {
                let score = result?;
                return Ok((score > 0).then_some(MatchResult {
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
        self.self_ref.lock().upgrade().ok_or(Error::NotFound)
    }

    pub(super) fn probe(&self, device: &Arc<Device>, match_data: usize) -> Result<()> {
        let handle = super::super::obj::handle(device);
        // SAFETY: registration validated the callback, and the handle addresses
        // a live device that the probe engine keeps alive across the call.
        let status = unsafe { (self.ops.probe)(self.ops.context, handle, match_data) };
        error::from_status(status)
    }

    pub(super) fn remove(&self, device: &Arc<Device>) -> Result<()> {
        let handle = super::super::obj::handle(device);
        if let Some(callback) = self.ops.try_remove {
            // SAFETY: registration pins callback code and the device binding
            // exclusively owns this removal transaction until it returns.
            return error::from_status(unsafe { callback(self.ops.context, handle) });
        }
        if let Some(callback) = self.ops.remove {
            // SAFETY: the legacy callback guarantees complete disconnection.
            unsafe { callback(self.ops.context, handle) };
        }
        Ok(())
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

pub(super) struct ProbeActivity<'a>(&'a Driver);

impl Drop for ProbeActivity<'_> {
    fn drop(&mut self) {
        let mut probes = self.0.probes.lock();
        *probes = probes
            .checked_sub(1)
            .expect("driver probe activity underflow");
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
    let _owner = module::pin_owner(owner, false)?;
    let bus_lease = if owner.is_none() {
        registration
            .bus
            .as_ref()
            .and_then(|bus| bus.owner())
            .map(module::lease)
            .transpose()?
    } else {
        None
    };
    if registration.name.is_empty() || registration.name.len() > MAX_NAME {
        return Err(Error::InvalidArgument);
    }
    if registration.ops.remove.is_some() && registration.ops.try_remove.is_some() {
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
    let header = ObjHeader::new_with(
        ObjKind::Driver,
        Some(&registration.name),
        owner.map(|module| module.id().get()),
        registration.bus.as_ref().map(|bus| bus.object_id()),
    );
    let driver = Arc::new(Driver {
        header,
        name: registration.name,
        owner: owner.cloned(),
        bus: registration.bus,
        bus_lease: Mutex::new(bus_lease),
        priority: registration.priority,
        matches: registration.matches,
        ops: registration.ops,
        bound: AtomicU64::new(0),
        probes: Mutex::new(0),
        self_ref: Mutex::new(Weak::new()),
    });
    *driver.self_ref.lock() = Arc::downgrade(&driver);
    driver.header.register()?;
    drivers.push(driver.clone());
    drop(drivers);

    super::probe::offer_driver(&driver);
    Ok(driver)
}

/// Unregisters a driver, unbinding every device it claimed.
pub fn unregister(driver: &Arc<Driver>) -> Result<()> {
    let registry = registry()?;
    // Claim the driver out of the match registry while holding the registry
    // lock, so no concurrent evaluation can select it between the detach and
    // the final unregister. A rescan holding an older snapshot still sees the
    // Removing state and declines through `evaluate`.
    {
        let probes = driver.probes.lock();
        let mut drivers = registry.drivers.lock();
        if !drivers.iter().any(|entry| Arc::ptr_eq(entry, driver)) {
            return if driver.header.state() == ObjState::Removing {
                Err(Error::Busy)
            } else {
                Ok(())
            };
        }
        if *probes != 0 {
            return Err(Error::Busy);
        }
        driver.header.set_state(ObjState::Removing);
        drivers.retain(|entry| !Arc::ptr_eq(entry, driver));
    }
    let result = super::probe::detach_driver(driver);
    if result.is_err() || driver.bound_count() != 0 {
        // Busy resources retain their binding and the caller's registration
        // receipt. Devices already detached remain available for reprobe.
        let _probes = driver.probes.lock();
        driver.header.set_state(ObjState::Live);
        registry.drivers.lock().push(driver.clone());
        return result.and(Err(Error::Busy));
    }
    driver.header.poison();
    let bus_lease = driver.bus_lease.lock().take();
    drop(bus_lease);
    super::probe::retrigger();
    Ok(())
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

/// Detaches bindings before module exit while unload can still roll back.
pub(super) fn detach_module_drivers(module: &Arc<Module>) -> Result<()> {
    for driver in owned(module) {
        super::probe::detach_driver(&driver)?;
    }
    Ok(())
}

pub(super) fn remove_module_drivers(module: &Arc<Module>) -> Result<()> {
    for driver in owned(module) {
        unregister(&driver)?;
    }
    Ok(())
}

fn owned(module: &Arc<Module>) -> Vec<Arc<Driver>> {
    all()
        .into_iter()
        .filter(|driver| {
            driver
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .collect()
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
