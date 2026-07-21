//! Driver module registration and lifecycle.

use alloc::{boxed::Box, collections::BTreeMap, string::String, sync::Arc, vec::Vec};
use core::{
    mem, str,
    sync::atomic::{AtomicU8, AtomicU64, Ordering},
};
use log::error;

use crate::{
    fs::devtempfs,
    sys::sync::{Mutex, Once},
};

use super::{
    abi::{DRIVER_ABI_V1, DriverModuleV1, HOST_API_V1},
    error::{Error, Result},
    tree::{self, DriverId, KERNEL_DRIVER},
};

const STATE_LOADING: u8 = 1;
const STATE_LOADED: u8 = 2;
const STATE_UNLOADING: u8 = 3;

/// Snapshot of one loaded driver.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DriverInfo {
    /// Stable driver identifier.
    pub id: DriverId,
    /// Driver-reported name.
    pub name: Box<str>,
}

struct DriverCallbacks {
    context: usize,
    fini: Option<super::abi::DriverFiniFn>,
}

struct DriverRecord {
    info: DriverInfo,
    state: AtomicU8,
    active_callbacks: AtomicU64,
    callbacks: DriverCallbacks,
}

struct DriverRegistryState {
    drivers: BTreeMap<DriverId, Arc<DriverRecord>>,
}

struct DriverRegistry {
    next_driver: AtomicU64,
    state: Mutex<DriverRegistryState>,
}

/// Pins executable driver callbacks against logical unload.
pub(crate) struct CallbackGuard {
    record: Option<Arc<DriverRecord>>,
}

static DRIVERS: Once<DriverRegistry> = Once::new();

impl DriverRegistry {
    fn new() -> Self {
        Self {
            next_driver: AtomicU64::new(1),
            state: Mutex::new(DriverRegistryState {
                drivers: BTreeMap::new(),
            }),
        }
    }

    fn allocate_id(&self) -> DriverId {
        let id = self.next_driver.fetch_add(1, Ordering::Relaxed);
        assert!(id != 0, "dev: driver identifier wrapped");
        DriverId::new(id)
    }
}

impl Drop for CallbackGuard {
    fn drop(&mut self) {
        if let Some(record) = self.record.take() {
            let previous = record.active_callbacks.fetch_sub(1, Ordering::Release);
            assert!(previous != 0, "dev: driver callback count underflow");
        }
    }
}

pub(crate) fn init() {
    DRIVERS.call_once(DriverRegistry::new);
}

/// Loads a version-1 driver descriptor.
///
/// The descriptor is copied, but its callback code and context must remain
/// valid until [`unload`] succeeds.
///
/// # Safety
///
/// Every pointer and callback in `module` must satisfy the version-1 ABI
/// contract and remain valid for the duration described above.
pub unsafe fn load(module: &DriverModuleV1) -> Result<DriverId> {
    if module.abi_version != DRIVER_ABI_V1
        || (module.size as usize) < mem::size_of::<DriverModuleV1>()
    {
        return Err(Error::AbiMismatch);
    }
    let init = module.init.ok_or(Error::InvalidArgument)?;
    // SAFETY: required by this function's caller contract.
    let name_bytes = unsafe { module.name.as_slice()? };
    let name = str::from_utf8(name_bytes).map_err(|_| Error::InvalidArgument)?;
    if name.is_empty() || name.len() > 255 {
        return Err(Error::InvalidArgument);
    }

    let registry = registry()?;
    let id = registry.allocate_id();
    let record = Arc::new(DriverRecord {
        info: DriverInfo {
            id,
            name: String::from(name).into_boxed_str(),
        },
        state: AtomicU8::new(STATE_LOADING),
        active_callbacks: AtomicU64::new(0),
        callbacks: DriverCallbacks {
            context: module.context,
            fini: module.fini,
        },
    });
    registry.state.lock().drivers.insert(id, record.clone());

    // SAFETY: the module contract guarantees this callback follows the ABI and
    // remains executable. The host table is static and immutable.
    let status = unsafe { init(&HOST_API_V1, id.get(), module.context) };
    if status != 0 {
        record.state.store(STATE_UNLOADING, Ordering::Release);
        cleanup_failed_load(id);
        return Err(Error::CallbackFailed(status));
    }
    record.state.store(STATE_LOADED, Ordering::Release);
    Ok(id)
}

/// Logically unloads a driver after all dependents and open nodes are gone.
pub fn unload(id: DriverId) -> Result<()> {
    if id == KERNEL_DRIVER {
        return Err(Error::PermissionDenied);
    }
    let registry = registry()?;
    let record = {
        let state = registry.state.lock();
        let record = state.drivers.get(&id).cloned().ok_or(Error::NotFound)?;
        if record.state.load(Ordering::Acquire) != STATE_LOADED
            || record.active_callbacks.load(Ordering::Acquire) != 0
        {
            return Err(Error::Busy);
        }
        record.state.store(STATE_UNLOADING, Ordering::Release);
        record
    };
    if let Err(error) = tree::prepare_remove_driver(id) {
        record.state.store(STATE_LOADED, Ordering::Release);
        return Err(error);
    }
    if let Ok(filesystem) = devtempfs::global()
        && let Err(error) = filesystem.can_remove_owner(id)
    {
        let _ = tree::restore_driver_resources(id);
        record.state.store(STATE_LOADED, Ordering::Release);
        return Err(map_filesystem_error(error));
    }

    if let Ok(filesystem) = devtempfs::global() {
        filesystem
            .remove_owner(id)
            .map_err(map_filesystem_error)
            .inspect_err(|_| {
                let _ = tree::restore_driver_resources(id);
                record.state.store(STATE_LOADED, Ordering::Release);
            })?;
    }
    tree::remove_driver(id).inspect_err(|_| record.state.store(STATE_LOADED, Ordering::Release))?;

    if let Some(fini) = record.callbacks.fini {
        // SAFETY: the descriptor contract keeps this callback executable until
        // unload completes, and all externally reachable callback objects have
        // been detached.
        unsafe { fini(id.get(), record.callbacks.context) };
    }
    registry.state.lock().drivers.remove(&id);
    Ok(())
}

/// Returns one loaded driver's metadata.
pub fn info(id: DriverId) -> Result<DriverInfo> {
    registry()?
        .state
        .lock()
        .drivers
        .get(&id)
        .map(|record| record.info.clone())
        .ok_or(Error::NotFound)
}

/// Returns snapshots for all loaded drivers.
pub fn loaded() -> Result<Vec<DriverInfo>> {
    Ok(registry()?
        .state
        .lock()
        .drivers
        .values()
        .filter(|record| record.state.load(Ordering::Acquire) == STATE_LOADED)
        .map(|record| record.info.clone())
        .collect())
}

pub(crate) fn authorize(id: DriverId) -> Result<()> {
    if id == KERNEL_DRIVER {
        return Err(Error::PermissionDenied);
    }
    let record = registry()?
        .state
        .lock()
        .drivers
        .get(&id)
        .cloned()
        .ok_or(Error::PermissionDenied)?;
    match record.state.load(Ordering::Acquire) {
        STATE_LOADING | STATE_LOADED => Ok(()),
        _ => Err(Error::PermissionDenied),
    }
}

pub(crate) fn callback_guard(id: DriverId) -> Result<CallbackGuard> {
    callback_guard_with_states(id, false, false)
}

pub(crate) fn close_callback_guard(id: DriverId) -> Result<CallbackGuard> {
    callback_guard_with_states(id, false, true)
}

pub(crate) fn mutation_guard(id: DriverId) -> Result<CallbackGuard> {
    callback_guard_with_states(id, true, false)
}

pub(crate) fn parent_guard(caller: DriverId, parent_owner: DriverId) -> Result<CallbackGuard> {
    if caller == parent_owner || parent_owner == KERNEL_DRIVER {
        return Ok(CallbackGuard { record: None });
    }
    callback_guard(parent_owner)
}

fn callback_guard_with_states(
    id: DriverId,
    allow_loading: bool,
    allow_unloading: bool,
) -> Result<CallbackGuard> {
    if id == KERNEL_DRIVER {
        return Ok(CallbackGuard { record: None });
    }
    let state = registry()?.state.lock();
    let record = state
        .drivers
        .get(&id)
        .cloned()
        .ok_or(Error::PermissionDenied)?;
    let current = record.state.load(Ordering::Acquire);
    if current != STATE_LOADED
        && !(allow_loading && current == STATE_LOADING)
        && !(allow_unloading && current == STATE_UNLOADING)
    {
        return Err(Error::Busy);
    }
    record.active_callbacks.fetch_add(1, Ordering::Acquire);
    Ok(CallbackGuard {
        record: Some(record),
    })
}

/// Loads every statically linked driver descriptor.
pub(crate) fn load_linked() -> Result<()> {
    unsafe extern "C" {
        static __roanix_drivers_start: u8;
        static __roanix_drivers_end: u8;
    }

    let start = core::ptr::addr_of!(__roanix_drivers_start) as usize;
    let end = core::ptr::addr_of!(__roanix_drivers_end) as usize;
    let stride = mem::size_of::<*const DriverModuleV1>();
    if end < start || start % mem::align_of::<*const DriverModuleV1>() != 0 {
        return Err(Error::AbiMismatch);
    }
    let bytes = end - start;
    if bytes % stride != 0 {
        return Err(Error::AbiMismatch);
    }

    let mut cursor = start;
    while cursor < end {
        // SAFETY: the linker script aligns this section and keeps a packed
        // array of descriptor pointers emitted by driver modules.
        let descriptor = unsafe { (cursor as *const *const DriverModuleV1).read() };
        if descriptor.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: linked driver modules promise that section entries point to
        // persistent versioned descriptors.
        unsafe { load(&*descriptor)? };
        cursor += stride;
    }
    Ok(())
}

fn cleanup_failed_load(id: DriverId) {
    if let Ok(filesystem) = devtempfs::global() {
        if let Err(error) = filesystem.force_remove_owner(id) {
            error!(
                "dev: failed to remove devtempfs objects for driver {}: {error}",
                id.get()
            );
        }
    }
    match tree::prepare_remove_driver(id) {
        Ok(()) => {
            if let Err(error) = tree::remove_driver(id) {
                error!(
                    "dev: failed to remove hierarchy objects for driver {}: {error}",
                    id.get()
                );
            }
        }
        Err(error) => {
            error!(
                "dev: failed to revoke hierarchy objects for driver {}: {error}",
                id.get()
            );
        }
    }
    if let Ok(registry) = registry() {
        registry.state.lock().drivers.remove(&id);
    }
}

fn map_filesystem_error(error: crate::fs::Error) -> Error {
    match error {
        crate::fs::Error::NotFound => Error::NotFound,
        crate::fs::Error::AlreadyExists => Error::AlreadyExists,
        crate::fs::Error::Busy | crate::fs::Error::NotEmpty => Error::Busy,
        crate::fs::Error::PermissionDenied | crate::fs::Error::ReadOnly => Error::PermissionDenied,
        crate::fs::Error::OutOfMemory => Error::OutOfMemory,
        crate::fs::Error::NoSpace => Error::NoSpace,
        crate::fs::Error::InvalidArgument | crate::fs::Error::NameTooLong => Error::InvalidArgument,
        _ => Error::Filesystem,
    }
}

fn registry() -> Result<&'static DriverRegistry> {
    DRIVERS.get().ok_or(Error::NotInitialized)
}
