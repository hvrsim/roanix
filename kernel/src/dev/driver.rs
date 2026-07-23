//! Driver module registration and lifecycle.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    format,
    string::String,
    sync::Arc,
    vec::Vec,
};
use core::{
    mem, str,
    sync::atomic::{AtomicU8, AtomicU64, Ordering},
};
use log::error;

use crate::{
    fs::{self, OpenFlags, VnodeKind, devtempfs},
    sys::sync::{Mutex, Once},
};

use super::{
    abi::{DRIVER_ABI_MAJOR, DRIVER_ABI_MINOR, DRIVER_BOOTSTRAP, DriverModule},
    error::{Error, Result},
    module::{self, ModuleImage},
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
    _image: Option<ModuleImage>,
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

/// Lock-free callback owner captured while the driver registry is accessible.
#[derive(Clone)]
pub(crate) struct CallbackOwner {
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

impl CallbackOwner {
    /// Pins an interrupt-context callback owned by a loading or loaded driver.
    pub(crate) fn acquire_irq(&self) -> Result<CallbackGuard> {
        self.acquire(true, false)
    }

    /// Pins a control-plane callback while a driver is loading or loaded.
    pub(crate) fn acquire_control(&self) -> Result<CallbackGuard> {
        self.acquire(true, false)
    }

    /// Pins cleanup code while a driver is loading, loaded, or unloading.
    pub(crate) fn acquire_cleanup(&self) -> Result<CallbackGuard> {
        self.acquire(true, true)
    }

    /// Returns whether interrupt callbacks may currently enter this driver.
    pub(crate) fn is_loaded(&self) -> bool {
        self.record
            .as_ref()
            .is_none_or(|record| record.state.load(Ordering::Acquire) == STATE_LOADED)
    }

    fn acquire(&self, allow_loading: bool, allow_unloading: bool) -> Result<CallbackGuard> {
        let Some(record) = self.record.as_ref() else {
            return Ok(CallbackGuard { record: None });
        };
        if !state_allowed(
            record.state.load(Ordering::Acquire),
            allow_loading,
            allow_unloading,
        ) {
            return Err(Error::Busy);
        }

        record.active_callbacks.fetch_add(1, Ordering::AcqRel);
        if !state_allowed(
            record.state.load(Ordering::Acquire),
            allow_loading,
            allow_unloading,
        ) {
            let previous = record.active_callbacks.fetch_sub(1, Ordering::Release);
            assert!(previous != 0, "dev: driver callback count underflow");
            return Err(Error::Busy);
        }

        Ok(CallbackGuard {
            record: Some(record.clone()),
        })
    }
}

pub(crate) fn init() {
    DRIVERS.call_once(DriverRegistry::new);
}

/// Loads a driver descriptor.
///
/// The descriptor is copied, but its callback code and context must remain
/// valid until [`unload`] succeeds.
///
/// # Safety
///
/// Every pointer and callback in `module` must satisfy the driver interface
/// contract and remain valid for the duration described above.
pub unsafe fn load(module: &DriverModule) -> Result<DriverId> {
    // SAFETY: forwarded from this function's caller.
    unsafe { load_with_image(module, None) }
}

unsafe fn load_with_image(
    module: &DriverModule,
    image: Option<ModuleImage>,
) -> Result<DriverId> {
    if (module.size as usize) < mem::size_of::<DriverModule>() {
        return Err(Error::InvalidArgument);
    }
    if module.abi_major != DRIVER_ABI_MAJOR
        || module.abi_minor > DRIVER_ABI_MINOR
        || module.flags != 0
    {
        return Err(Error::Unsupported);
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
        _image: image,
    });
    registry.state.lock().drivers.insert(id, record.clone());

    // SAFETY: the module contract guarantees this callback follows the ABI and
    // remains executable. The bootstrap is static and immutable.
    let status = unsafe { init(&DRIVER_BOOTSTRAP, id.get(), module.context) };
    if status != 0 {
        if let Err(error) = super::interrupt::cleanup_failed_load(id) {
            record.state.store(STATE_UNLOADING, Ordering::Release);
            error!(
                "dev: retaining failed driver {} because interrupt cleanup failed: {error}",
                id.get()
            );
            return Err(Error::CallbackFailed(status));
        }
        record.state.store(STATE_UNLOADING, Ordering::Release);
        if let Err(error) = cleanup_failed_load(id) {
            error!(
                "dev: retaining failed driver {} because object cleanup failed: {error}",
                id.get()
            );
        }
        return Err(Error::CallbackFailed(status));
    }
    record.state.store(STATE_LOADED, Ordering::Release);
    super::binding::activate_driver(id);
    Ok(id)
}

/// Loads every shared-object driver in `path` in lexical filename order.
pub(crate) fn load_directory(path: &str) -> Result<usize> {
    let directory = fs::open(
        path,
        OpenFlags::READ | OpenFlags::DIRECTORY | OpenFlags::NOFOLLOW,
        0,
    )
    .map_err(map_filesystem_error)?;
    let mut names = Vec::new();
    loop {
        let entries = directory.readdir(64).map_err(map_filesystem_error)?;
        if entries.is_empty() {
            break;
        }
        for entry in entries {
            if entry.kind != VnodeKind::Regular || !entry.name.ends_with(b".so") {
                continue;
            }
            let name = str::from_utf8(&entry.name).map_err(|_| Error::InvalidArgument)?;
            names.push(String::from(name));
        }
    }
    names.sort_unstable();

    let directory = path.trim_end_matches('/');
    for name in &names {
        load_file(&format!("{directory}/{name}"))?;
    }
    Ok(names.len())
}

fn load_file(path: &str) -> Result<DriverId> {
    let (image, descriptor) = module::load(path)?;
    // SAFETY: the module loader validated that the descriptor lies inside the
    // pinned image and that its entry follows the driver interface.
    unsafe { load_with_image(&*descriptor, Some(image)) }
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
        record
    };
    if super::binding::has_foreign_instances(id) {
        return Err(Error::Busy);
    }

    super::interrupt::prepare_remove_driver(id)?;
    let transitioned = {
        let _state = registry.state.lock();
        if record.state.load(Ordering::Acquire) != STATE_LOADED {
            false
        } else {
            record.state.store(STATE_UNLOADING, Ordering::Release);
            if record.active_callbacks.load(Ordering::Acquire) == 0 {
                true
            } else {
                record.state.store(STATE_LOADED, Ordering::Release);
                false
            }
        }
    };
    if !transitioned {
        let _ = super::interrupt::restore_driver(id);
        return Err(Error::Busy);
    }

    if let Err(error) = tree::prepare_remove_driver(id) {
        record.state.store(STATE_LOADED, Ordering::Release);
        let _ = super::interrupt::restore_driver(id);
        return Err(error);
    }
    if let Ok(filesystem) = devtempfs::global()
        && let Err(error) = filesystem.can_remove_owner(id)
    {
        let _ = tree::restore_driver_resources(id);
        record.state.store(STATE_LOADED, Ordering::Release);
        let _ = super::interrupt::restore_driver(id);
        return Err(map_filesystem_error(error));
    }

    super::binding::remove_driver(id);
    if let Ok(filesystem) = devtempfs::global() {
        if let Err(error) = filesystem.remove_owner(id).map_err(map_filesystem_error) {
            let _ = tree::restore_driver_resources(id);
            record.state.store(STATE_LOADED, Ordering::Release);
            let _ = super::interrupt::restore_driver(id);
            return Err(error);
        }
    }
    if let Err(error) = super::interrupt::remove_driver(id) {
        error!(
            "dev: retaining unloaded driver {} because interrupt removal failed: {error}",
            id.get()
        );
        return Err(error);
    }
    if let Some(fini) = record.callbacks.fini {
        // SAFETY: the descriptor contract keeps this callback executable until
        // unload completes. User and interrupt callbacks are quiesced, while
        // consumed resources still pin their providers for final hardware
        // shutdown and driver-owned allocation cleanup.
        unsafe { fini(id.get(), record.callbacks.context) };
    }
    super::abi::remove_driver_mappings(id);
    if let Err(error) = tree::remove_driver(id) {
        error!(
            "dev: retaining unloaded driver {} because hierarchy cleanup failed: {error}",
            id.get()
        );
        return Err(error);
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

pub(crate) fn cleanup_guard(id: DriverId) -> Result<CallbackGuard> {
    callback_guard_with_states(id, true, true)
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

pub(crate) fn callback_owner(id: DriverId) -> Result<CallbackOwner> {
    callback_owner_with_states(id, true, false)
}

fn callback_owner_with_states(
    id: DriverId,
    allow_loading: bool,
    allow_unloading: bool,
) -> Result<CallbackOwner> {
    if id == KERNEL_DRIVER {
        return Ok(CallbackOwner { record: None });
    }
    let state = registry()?.state.lock();
    let record = state
        .drivers
        .get(&id)
        .cloned()
        .ok_or(Error::PermissionDenied)?;
    if !state_allowed(
        record.state.load(Ordering::Acquire),
        allow_loading,
        allow_unloading,
    ) {
        return Err(Error::Busy);
    }
    Ok(CallbackOwner {
        record: Some(record),
    })
}

fn callback_guard_with_states(
    id: DriverId,
    allow_loading: bool,
    allow_unloading: bool,
) -> Result<CallbackGuard> {
    callback_owner_with_states(id, allow_loading, allow_unloading)?
        .acquire(allow_loading, allow_unloading)
}

fn state_allowed(current: u8, allow_loading: bool, allow_unloading: bool) -> bool {
    current == STATE_LOADED
        || (allow_loading && current == STATE_LOADING)
        || (allow_unloading && current == STATE_UNLOADING)
}

fn cleanup_failed_load(id: DriverId) -> Result<()> {
    super::binding::remove_driver(id);
    if let Ok(filesystem) = devtempfs::global() {
        filesystem
            .force_remove_owner(id)
            .map_err(map_filesystem_error)?;
    }
    tree::prepare_remove_driver(id)?;
    super::abi::remove_driver_mappings(id);
    tree::remove_driver(id)?;
    registry()?.state.lock().drivers.remove(&id);
    Ok(())
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
