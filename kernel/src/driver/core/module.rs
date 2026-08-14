//! Module lifetime, reference counting, and the inter-module dependency graph.
//!
//! A module is the unit of code ownership. Every device, driver, bus, class,
//! interface, interrupt, mapping, and worker is attributed to one module, and a
//! module can only be unloaded once nothing else depends on it.
//!
//! Dependencies are recorded explicitly as a directed graph. Edges are created
//! when a module binds an interface published by another module, and when a
//! driver from one module claims a device created by another. Cycles are
//! rejected when the edge is added, so unload order is always well defined.

use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet},
    string::String,
    sync::Arc,
    vec::Vec,
};
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use log::{error, warn};

use crate::sys::sync::{Mutex, Once};

use super::{
    super::{
        error::{Error, Result},
        obj::{ObjHeader, framework_object},
    },
    name::Name,
};

/// Backing storage that must outlive a loadable module's code.
pub trait ModuleBacking: Send + Sync {}

/// Module lifecycle state.
mod state {
    /// The module's init callback is running.
    pub const LOADING: u32 = 1;
    /// The module is fully loaded and may be used.
    pub const LIVE: u32 = 2;
    /// The module is being torn down.
    pub const UNLOADING: u32 = 3;
    /// The module has been torn down.
    pub const DEAD: u32 = 4;
}

/// Stable numeric identity used for dependency bookkeeping.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ModuleId(u64);

impl ModuleId {
    /// Identity reserved for kernel-resident framework objects.
    pub const KERNEL: Self = Self(0);

    /// Returns the numeric identity.
    pub const fn get(self) -> u64 {
        self.0
    }

    /// Returns whether this identity denotes the kernel itself.
    pub const fn is_kernel(self) -> bool {
        self.0 == 0
    }
}

/// Callback invoked once when a module is loaded.
pub type InitFn = unsafe extern "C" fn(module: *const core::ffi::c_void) -> i32;
/// Callback invoked once when a module is unloaded.
pub type ExitFn = unsafe extern "C" fn(module: *const core::ffi::c_void);

/// Entry points and metadata supplied by a module image.
pub struct ModuleDefinition {
    /// Human-readable module name, unique among loaded modules.
    pub name: Box<str>,
    /// Optional description recorded for diagnostics.
    pub description: Option<Box<str>>,
    /// Required initialization callback.
    pub init: InitFn,
    /// Optional teardown callback.
    pub exit: Option<ExitFn>,
    /// Backing image that must stay resident while the module is loaded.
    pub backing: Option<Box<dyn ModuleBacking>>,
}

struct Edges {
    /// Modules this module depends on, with an edge multiplicity.
    depends_on: BTreeMap<ModuleId, u64>,
    /// Modules that depend on this module, with an edge multiplicity.
    dependents: BTreeMap<ModuleId, u64>,
}

/// A loaded unit of driver code.
#[repr(C)]
pub struct Module {
    header: ObjHeader,
    id: ModuleId,
    name: Name,
    description: Option<Box<str>>,
    state: AtomicU32,
    active: AtomicU64,
    exit: Option<ExitFn>,
    edges: Mutex<Edges>,
    backing: Mutex<Option<Box<dyn ModuleBacking>>>,
}

framework_object!(Module, Module);

impl Module {
    /// Returns this module's dependency-graph identity.
    pub const fn id(&self) -> ModuleId {
        self.id
    }

    /// Returns this module's name.
    pub fn name(&self) -> &str {
        self.name.as_str()
    }

    /// Returns this module's name as a NUL-terminated pointer for the ABI.
    pub fn name_c_ptr(&self) -> *const core::ffi::c_char {
        self.name.as_c_ptr()
    }

    /// Returns this module's description.
    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    /// Returns whether the module is loaded and usable.
    pub fn is_live(&self) -> bool {
        self.state.load(Ordering::Acquire) == state::LIVE
    }

    /// Returns whether the module may still register objects.
    pub fn is_registering(&self) -> bool {
        matches!(
            self.state.load(Ordering::Acquire),
            state::LOADING | state::LIVE
        )
    }

    /// Returns the number of modules that currently depend on this one.
    pub fn dependent_count(&self) -> usize {
        self.edges.lock().dependents.len()
    }
}

/// Pins a module so it cannot finish unloading while a callback runs.
pub struct ModuleGuard {
    module: Option<Arc<Module>>,
}

impl Drop for ModuleGuard {
    fn drop(&mut self) {
        if let Some(module) = self.module.take() {
            let previous = module.active.fetch_sub(1, Ordering::Release);
            debug_assert!(previous != 0, "driver: module callback underflow");
        }
    }
}

impl ModuleGuard {
    /// Returns a guard that pins nothing, used for kernel-owned objects.
    pub const fn none() -> Self {
        Self { module: None }
    }

    /// Returns the pinned module, if any.
    pub fn module(&self) -> Option<&Arc<Module>> {
        self.module.as_ref()
    }
}

/// Pins `module` for the duration of a callback.
///
/// Cleanup paths pass `allow_unloading` so teardown can still run after the
/// module has left the live state.
pub fn pin(module: &Arc<Module>, allow_unloading: bool) -> Result<ModuleGuard> {
    if !state_allows(module.state.load(Ordering::Acquire), allow_unloading) {
        return Err(Error::NoDevice);
    }
    module.active.fetch_add(1, Ordering::AcqRel);
    if !state_allows(module.state.load(Ordering::Acquire), allow_unloading) {
        let previous = module.active.fetch_sub(1, Ordering::Release);
        debug_assert!(previous != 0, "driver: module callback underflow");
        return Err(Error::NoDevice);
    }
    Ok(ModuleGuard {
        module: Some(module.clone()),
    })
}

/// Pins an optional module owner, succeeding trivially for kernel objects.
pub fn pin_owner(module: Option<&Arc<Module>>, allow_unloading: bool) -> Result<ModuleGuard> {
    match module {
        Some(module) => pin(module, allow_unloading),
        None => Ok(ModuleGuard::none()),
    }
}

fn state_allows(current: u32, allow_unloading: bool) -> bool {
    current == state::LIVE
        || current == state::LOADING
        || (allow_unloading && current == state::UNLOADING)
}

struct Registry {
    next_id: AtomicU64,
    modules: Mutex<BTreeMap<ModuleId, Arc<Module>>>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        next_id: AtomicU64::new(1),
        modules: Mutex::new(BTreeMap::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// Loads a module and runs its initialization callback.
///
/// # Safety
///
/// `definition.init` and `definition.exit` must follow the module ABI and stay
/// executable until the returned module is unloaded.
pub unsafe fn load(definition: ModuleDefinition) -> Result<Arc<Module>> {
    let registry = registry()?;
    if definition.name.is_empty() || definition.name.len() > 64 {
        return Err(Error::InvalidArgument);
    }
    {
        let modules = registry.modules.lock();
        if modules
            .values()
            .any(|module| module.name.as_str() == &*definition.name)
        {
            return Err(Error::AlreadyExists);
        }
    }

    let id = ModuleId(registry.next_id.fetch_add(1, Ordering::Relaxed));
    let module = Arc::new(Module {
        header: ObjHeader::new(super::super::obj::ObjKind::Module),
        id,
        name: Name::new(&definition.name),
        description: definition.description,
        state: AtomicU32::new(state::LOADING),
        active: AtomicU64::new(0),
        exit: definition.exit,
        edges: Mutex::new(Edges {
            depends_on: BTreeMap::new(),
            dependents: BTreeMap::new(),
        }),
        backing: Mutex::new(definition.backing),
    });
    registry.modules.lock().insert(id, module.clone());

    let handle = super::super::obj::handle(&module);
    // SAFETY: the module contract guarantees `init` follows the ABI and stays
    // executable, and the handle addresses a live module for the whole call.
    let status = unsafe { (definition.init)(handle) };
    if status < 0 {
        module.state.store(state::UNLOADING, Ordering::Release);
        teardown(&module);
        module.state.store(state::DEAD, Ordering::Release);
        module.header.poison();
        registry.modules.lock().remove(&id);
        return Err(Error::from_status(status));
    }

    module.state.store(state::LIVE, Ordering::Release);
    super::probe::retrigger();
    Ok(module)
}

/// Unloads a module once nothing depends on it.
pub fn unload(module: &Arc<Module>) -> Result<()> {
    let registry = registry()?;
    if module.state.load(Ordering::Acquire) != state::LIVE {
        return Err(Error::Busy);
    }
    if !module.edges.lock().dependents.is_empty() {
        return Err(Error::Busy);
    }

    if module
        .state
        .compare_exchange(
            state::LIVE,
            state::UNLOADING,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        return Err(Error::Busy);
    }

    teardown(module);

    if module.active.load(Ordering::Acquire) != 0 {
        module.state.store(state::LIVE, Ordering::Release);
        return Err(Error::Busy);
    }

    if let Some(exit) = module.exit {
        let handle = super::super::obj::handle(module);
        // SAFETY: the module contract keeps `exit` executable until unload
        // completes, and every consumer of this module has been detached.
        unsafe { exit(handle) };
    }

    drop_edges(module);
    module.state.store(state::DEAD, Ordering::Release);
    module.header.poison();
    registry.modules.lock().remove(&module.id);
    *module.backing.lock() = None;
    super::probe::retrigger();
    Ok(())
}

/// Releases every framework object owned by `module`.
///
/// Teardown runs in dependency order: bound drivers are detached before the
/// devices they own disappear, and published services are withdrawn last so
/// consumers can still complete their own teardown.
fn teardown(module: &Arc<Module>) {
    super::driver::remove_module_drivers(module);
    super::device::remove_module_devices(module);
    super::super::class::console::remove_module_terminals(module);
    super::class::remove_module_classes(module);
    super::bus::remove_module_buses(module);
    super::iface::remove_module_interfaces(module);
    super::super::irq::remove_module_interrupts(module);
    super::super::work::remove_module_workers(module);
    super::super::io::dma::remove_module_buffers(module);
    super::super::io::mmio::remove_module_mappings(module);
    super::super::class::chardev::remove_module_nodes(module);
    super::super::abi::events::remove_module_events(module);
}

fn drop_edges(module: &Arc<Module>) {
    let Ok(registry) = registry() else {
        return;
    };
    let providers: Vec<ModuleId> = module.edges.lock().depends_on.keys().copied().collect();
    if providers.is_empty() {
        return;
    }
    let modules = registry.modules.lock();
    for provider in providers {
        if let Some(provider) = modules.get(&provider) {
            provider.edges.lock().dependents.remove(&module.id);
        }
    }
    module.edges.lock().depends_on.clear();
}

/// Records that `consumer` now depends on `provider`.
///
/// Returns [`Error::Deadlock`] if the edge would close a cycle.
pub fn add_dependency(consumer: &Arc<Module>, provider: &Arc<Module>) -> Result<()> {
    if Arc::ptr_eq(consumer, provider) {
        return Ok(());
    }
    if creates_cycle(consumer, provider)? {
        return Err(Error::Deadlock);
    }
    *consumer
        .edges
        .lock()
        .depends_on
        .entry(provider.id)
        .or_insert(0) += 1;
    *provider
        .edges
        .lock()
        .dependents
        .entry(consumer.id)
        .or_insert(0) += 1;
    Ok(())
}

/// Drops one dependency edge previously recorded by [`add_dependency`].
pub fn remove_dependency(consumer: &Arc<Module>, provider: &Arc<Module>) {
    if Arc::ptr_eq(consumer, provider) {
        return;
    }
    decrement(&mut consumer.edges.lock().depends_on, provider.id);
    decrement(&mut provider.edges.lock().dependents, consumer.id);
}

/// Records a dependency between optional owners, ignoring kernel-owned ends.
pub fn link(consumer: Option<&Arc<Module>>, provider: Option<&Arc<Module>>) -> Result<()> {
    match (consumer, provider) {
        (Some(consumer), Some(provider)) => add_dependency(consumer, provider),
        _ => Ok(()),
    }
}

/// Drops a dependency recorded by [`link`].
pub fn unlink(consumer: Option<&Arc<Module>>, provider: Option<&Arc<Module>>) {
    if let (Some(consumer), Some(provider)) = (consumer, provider) {
        remove_dependency(consumer, provider);
    }
}

fn decrement(map: &mut BTreeMap<ModuleId, u64>, key: ModuleId) {
    if let Some(count) = map.get_mut(&key) {
        *count = count.saturating_sub(1);
        if *count == 0 {
            map.remove(&key);
        }
    }
}

/// Returns whether depending on `provider` would make `consumer` reachable
/// from itself.
fn creates_cycle(consumer: &Arc<Module>, provider: &Arc<Module>) -> Result<bool> {
    let registry = registry()?;
    let mut seen = BTreeSet::new();
    let mut queue = Vec::new();
    queue.push(provider.id);
    while let Some(current) = queue.pop() {
        if current == consumer.id {
            return Ok(true);
        }
        if !seen.insert(current) {
            continue;
        }
        let next: Vec<ModuleId> = {
            let modules = registry.modules.lock();
            match modules.get(&current) {
                Some(module) => module.edges.lock().depends_on.keys().copied().collect(),
                None => Vec::new(),
            }
        };
        queue.extend(next);
    }
    Ok(false)
}

/// Snapshot of one loaded module.
#[derive(Clone, Debug)]
pub struct ModuleInfo {
    /// Dependency-graph identity.
    pub id: ModuleId,
    /// Module name.
    pub name: Box<str>,
    /// Number of modules depending on this one.
    pub dependents: usize,
    /// Whether the module is fully loaded.
    pub live: bool,
}

/// Returns a snapshot of every registered module.
pub fn list() -> Result<Vec<ModuleInfo>> {
    Ok(registry()?
        .modules
        .lock()
        .values()
        .map(|module| ModuleInfo {
            id: module.id,
            name: String::from(module.name.as_str()).into_boxed_str(),
            dependents: module.edges.lock().dependents.len(),
            live: module.is_live(),
        })
        .collect())
}

/// Returns a loaded module by name.
pub fn find(name: &str) -> Result<Arc<Module>> {
    registry()?
        .modules
        .lock()
        .values()
        .find(|module| module.name.as_str() == name)
        .cloned()
        .ok_or(Error::NotFound)
}

/// Unloads every module that nothing depends on, repeating until stable.
pub fn unload_all() {
    let Ok(registry) = registry() else {
        return;
    };
    loop {
        let candidates: Vec<Arc<Module>> = registry
            .modules
            .lock()
            .values()
            .filter(|module| module.is_live() && module.edges.lock().dependents.is_empty())
            .cloned()
            .collect();
        if candidates.is_empty() {
            break;
        }
        let mut progress = false;
        for module in candidates {
            match unload(&module) {
                Ok(()) => progress = true,
                Err(error) => warn!(
                    "driver: module {} could not be unloaded: {error:?}",
                    module.name
                ),
            }
        }
        if !progress {
            break;
        }
    }
    let stuck = registry.modules.lock().len();
    if stuck != 0 {
        error!("driver: {stuck} modules remain loaded");
    }
}
