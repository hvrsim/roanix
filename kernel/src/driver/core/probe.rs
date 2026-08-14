//! The probe engine.
//!
//! Binding is deliberately not protected by one global lock: probe callbacks
//! allocate, map registers, sleep, and frequently register child devices, so
//! holding a lock across them would deadlock. Exclusivity is instead obtained
//! per device with an atomic state transition, and only the deferred list is
//! guarded.
//!
//! When a driver cannot proceed because a service it needs has not been
//! published yet, it returns [`Error::Deferred`] and the device is parked. Any
//! topology change - a new module, driver, bus, device, or interface - rescans
//! the parked devices. This is what makes inter-driver dependencies work
//! regardless of the order modules happen to load in.

use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use log::{debug, error};

use crate::sys::sync::{Mutex, Once};

use super::{
    super::error::{Error, Result},
    device::{self, Device, state},
    driver::{self, Driver},
    module,
};

struct Engine {
    deferred: Mutex<Vec<Weak<Device>>>,
    active: AtomicUsize,
    pending: AtomicBool,
}

static ENGINE: Once<Engine> = Once::new();

pub(crate) fn init() {
    ENGINE.call_once(|| Engine {
        deferred: Mutex::new(Vec::new()),
        active: AtomicUsize::new(0),
        pending: AtomicBool::new(false),
    });
}

fn engine() -> Option<&'static Engine> {
    ENGINE.get()
}

/// Offers `device` to every registered driver.
pub(super) fn attach(device: &Arc<Device>) {
    let Some(engine) = engine() else {
        return;
    };
    engine.active.fetch_add(1, Ordering::AcqRel);
    try_bind(device);
    finish_activity(engine);
}

/// Offers every unbound device to a newly registered driver.
pub(super) fn offer_driver(driver: &Arc<Driver>) {
    let Some(engine) = engine() else {
        return;
    };
    engine.active.fetch_add(1, Ordering::AcqRel);
    for device in device::all() {
        if matches!(device.state(), state::UNBOUND | state::DEFERRED) {
            try_bind_with(&device, Some(driver));
        }
    }
    finish_activity(engine);
}

/// Unbinds every device currently claimed by `driver`.
pub(super) fn detach_driver(driver: &Arc<Driver>) {
    for device in device::all() {
        let bound = device
            .driver()
            .is_some_and(|current| Arc::ptr_eq(&current, driver));
        if bound {
            unbind(&device);
        }
    }
}

/// Unbinds `device` from its driver, if any.
pub(super) fn detach(device: &Arc<Device>) {
    unbind(device);
    forget_deferred(device);
}

/// Requests a rescan of parked devices after a topology change.
pub fn retrigger() {
    let Some(engine) = engine() else {
        return;
    };
    if engine.active.load(Ordering::Acquire) != 0 {
        engine.pending.store(true, Ordering::Release);
        return;
    }
    rescan(engine);
}

fn finish_activity(engine: &'static Engine) {
    let previous = engine.active.fetch_sub(1, Ordering::AcqRel);
    if previous == 1 && engine.pending.swap(false, Ordering::AcqRel) {
        rescan(engine);
    }
}

fn rescan(engine: &'static Engine) {
    // A rescan can bind a device, which publishes interfaces and requests
    // another rescan. Bounding the passes keeps a pathological dependency
    // chain from looping forever while still resolving deep chains.
    const MAX_PASSES: usize = 32;

    engine.active.fetch_add(1, Ordering::AcqRel);
    for _ in 0..MAX_PASSES {
        let parked: Vec<Arc<Device>> = {
            let mut deferred = engine.deferred.lock();
            let devices: Vec<Arc<Device>> =
                deferred.iter().filter_map(Weak::upgrade).collect();
            deferred.clear();
            devices
        };
        let unbound: Vec<Arc<Device>> = device::all()
            .into_iter()
            .filter(|device| device.state() == state::UNBOUND)
            .collect();
        if parked.is_empty() && unbound.is_empty() {
            break;
        }

        engine.pending.store(false, Ordering::Release);
        for device in parked.iter().chain(unbound.iter()) {
            if matches!(device.state(), state::UNBOUND | state::DEFERRED) {
                try_bind(device);
            }
        }
        if !engine.pending.swap(false, Ordering::AcqRel) {
            break;
        }
    }
    let previous = engine.active.fetch_sub(1, Ordering::AcqRel);
    debug_assert!(previous != 0, "driver/probe: activity underflow");
}

fn park(device: &Arc<Device>) {
    let Some(engine) = engine() else {
        return;
    };
    let mut deferred = engine.deferred.lock();
    if deferred
        .iter()
        .filter_map(Weak::upgrade)
        .any(|parked| Arc::ptr_eq(&parked, device))
    {
        return;
    }
    deferred.retain(|entry| entry.strong_count() != 0);
    deferred.push(Arc::downgrade(device));
}

fn forget_deferred(device: &Arc<Device>) {
    let Some(engine) = engine() else {
        return;
    };
    engine.deferred.lock().retain(|entry| {
        entry
            .upgrade()
            .is_some_and(|parked| !Arc::ptr_eq(&parked, device))
    });
}

fn try_bind(device: &Arc<Device>) {
    try_bind_with(device, None);
}

/// Attempts to bind `device`, optionally restricted to one candidate driver.
fn try_bind_with(device: &Arc<Device>, only: Option<&Arc<Driver>>) {
    let previous = device.state();
    if !matches!(previous, state::UNBOUND | state::DEFERRED) {
        return;
    }
    if !device.transition(previous, state::PROBING) {
        return;
    }

    let mut candidates: Vec<(i32, usize, Arc<Driver>)> = Vec::new();
    let drivers = match only {
        Some(driver) => alloc::vec![driver.clone()],
        None => driver::all(),
    };
    for candidate in drivers {
        match candidate.evaluate(device) {
            Ok(Some(result)) => candidates.push((result.score, result.data, candidate)),
            Ok(None) => {}
            Err(error) => debug!(
                "driver: bus match for {} failed: {error:?}",
                device.path()
            ),
        }
    }
    if candidates.is_empty() {
        if previous == state::DEFERRED {
            device.set_state(state::DEFERRED);
            park(device);
        } else {
            device.set_state(state::UNBOUND);
        }
        return;
    }
    candidates.sort_unstable_by(|left, right| right.0.cmp(&left.0));

    for (_, match_data, candidate) in candidates {
        match bind_one(device, &candidate, match_data) {
            Ok(()) => {
                device.set_state(state::BOUND);
                return;
            }
            Err(Error::Deferred) => {
                // The strongest candidate is waiting on a prerequisite; binding
                // a weaker driver instead would claim the device with the wrong
                // implementation, so park it until the topology changes.
                device.set_state(state::DEFERRED);
                park(device);
                return;
            }
            Err(error) => {
                error!(
                    "driver: {} failed to probe {}: {error:?}",
                    candidate.name(),
                    device.path()
                );
            }
        }
    }
    device.set_state(state::UNBOUND);
}

fn bind_one(device: &Arc<Device>, driver: &Arc<Driver>, match_data: usize) -> Result<()> {
    let _pin = driver.pin(false)?;
    // Pin the bus before recording the dependency, so no failure path between
    // the two can leave an edge behind that nothing will ever remove.
    let bus_pin = match device.bus() {
        Some(bus) => Some(super::bus::pin(bus)?),
        None => None,
    };

    let provider = device.owner().cloned();
    module::link(driver.owner(), provider.as_ref())?;

    let bus_prepared = match device.bus() {
        Some(bus) => match bus.prepare(device) {
            Ok(()) => true,
            Err(error) => {
                module::unlink(driver.owner(), provider.as_ref());
                return Err(error);
            }
        },
        None => false,
    };
    drop(bus_pin);

    device.set_driver(Some(driver.clone()));
    match driver.probe(device, match_data) {
        Ok(()) => {
            driver.note_bound();
            Ok(())
        }
        Err(error) => {
            device.set_driver(None);
            device.set_drvdata(0);
            release_driver_state(device, driver);
            if bus_prepared
                && let Some(bus) = device.bus()
                && let Ok(_pin) = super::bus::pin(bus)
            {
                bus.cleanup(device);
            }
            module::unlink(driver.owner(), provider.as_ref());
            Err(error)
        }
    }
}

fn unbind(device: &Arc<Device>) {
    let Some(driver) = device.driver() else {
        return;
    };
    let previous = device.state();
    if previous != state::BOUND && previous != state::REMOVING {
        return;
    }
    if previous == state::BOUND && !device.transition(state::BOUND, state::REMOVING) {
        return;
    }

    if let Ok(_pin) = driver.pin(true) {
        driver.remove(device);
    }
    device.set_driver(None);
    device.set_drvdata(0);
    release_driver_state(device, &driver);

    if let Some(bus) = device.bus()
        && let Ok(_bus_pin) = super::bus::pin(bus)
    {
        bus.cleanup(device);
    }
    module::unlink(driver.owner(), device.owner());
    driver.note_unbound();

    if previous == state::BOUND {
        device.set_state(state::UNBOUND);
    }
}

/// Releases framework state a driver may have left attached to `device`.
///
/// Drivers are expected to clean up in their remove callback, but a failed
/// probe or a misbehaving driver must not leak interrupts, class memberships,
/// or published interfaces.
fn release_driver_state(device: &Arc<Device>, driver: &Arc<Driver>) {
    let Some(owner) = driver.owner() else {
        return;
    };
    for member in device.class_devices() {
        if member
            .owner()
            .is_some_and(|module| Arc::ptr_eq(module, owner))
        {
            super::class::remove_device(&member);
        }
    }
    for interface in device.interfaces() {
        if interface
            .owner()
            .is_some_and(|module| Arc::ptr_eq(module, owner))
        {
            let _ = super::iface::withdraw(&interface);
        }
    }
    super::super::irq::release_device_module_interrupts(device, owner);
}

/// Returns the number of devices currently waiting for a prerequisite.
pub fn deferred_count() -> usize {
    engine().map_or(0, |engine| {
        engine
            .deferred
            .lock()
            .iter()
            .filter(|entry| entry.strong_count() != 0)
            .count()
    })
}

/// Reports devices that never found a driver, for boot diagnostics.
pub fn report_unbound() {
    let mut deferred = Vec::new();
    device::for_each(|device| {
        if device.state() == state::DEFERRED {
            deferred.push(device.path().into());
        }
    });
    for path in deferred {
        let path: alloc::string::String = path;
        debug!("driver: {path} is still waiting for a prerequisite");
    }
}
