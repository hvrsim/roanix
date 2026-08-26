//! Published interfaces and the bindings that connect drivers to each other.
//!
//! An interface is a versioned operation table published under a name. It is
//! the framework's only mechanism for one driver to call another, and it
//! replaces the previous inheritable-resource design.
//!
//! Two scopes exist. A *global* interface is a system-wide service such as a
//! block layer or a display core. A *device* interface describes a capability
//! of one device, such as the configuration accessor of a PCI bridge. Neither
//! scope requires the consumer to be a descendant of the provider, so a driver
//! can combine services from anywhere in the system.
//!
//! Binding is the slow path: it resolves a name, checks the version, and adds a
//! module dependency edge. Afterwards the consumer holds the operation table
//! directly, so every subsequent call is an ordinary indirect call with no
//! framework involvement.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};

use crate::sys::sync::{Mutex, Once};

use super::{
    super::{
        error::{Error, Result},
        obj::{ObjHeader, ObjKind, Object, framework_object},
    },
    device::Device,
    module::Module,
};

/// Maximum interface-name length in bytes.
pub const MAX_NAME: usize = 128;

/// Interface behaviour flags.
pub mod flags {
    /// Operations may be invoked from interrupt context.
    pub const IRQ_SAFE: u32 = 1 << 0;
    /// Operations may block the calling thread.
    pub const MAY_SLEEP: u32 = 1 << 1;
    /// Operations are safe to invoke concurrently.
    pub const CONCURRENT: u32 = 1 << 2;
    /// Only one provider may publish this name at a time.
    pub const SINGLETON: u32 = 1 << 3;
}

/// A versioned operation table published by a module.
#[repr(C)]
pub struct Interface {
    header: ObjHeader,
    name: Box<str>,
    version: u32,
    flags: u32,
    ops: *const c_void,
    ops_size: usize,
    context: *mut c_void,
    owner: Option<Arc<Module>>,
    device: Option<Weak<Device>>,
    revoked: AtomicBool,
    binds: AtomicU64,
}

framework_object!(Interface, Interface);

// SAFETY: publication requires the operation table to stay immutable and
// executable until the interface is withdrawn, and bindings keep the providing
// module loaded for as long as any consumer holds the table.
unsafe impl Send for Interface {}
// SAFETY: concurrency expectations are declared through `flags` and enforced by
// the provider. The table pointer itself is immutable after publication.
unsafe impl Sync for Interface {}

impl Interface {
    /// Returns the published name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the provider's implementation version.
    pub const fn version(&self) -> u32 {
        self.version
    }

    /// Returns the declared behaviour flags.
    pub const fn flags(&self) -> u32 {
        self.flags
    }

    /// Returns the immutable operation table.
    pub const fn ops(&self) -> *const c_void {
        self.ops
    }

    /// Returns the operation-table size in bytes.
    pub const fn ops_size(&self) -> usize {
        self.ops_size
    }

    /// Returns the provider-defined call context.
    pub const fn context(&self) -> *mut c_void {
        self.context
    }

    /// Returns the module that published this interface.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns the device this interface describes, if it is device-scoped.
    pub fn device(&self) -> Option<Arc<Device>> {
        self.device.as_ref().and_then(Weak::upgrade)
    }

    /// Returns whether the interface is still usable.
    pub fn is_live(&self) -> bool {
        !self.revoked.load(Ordering::Acquire)
    }

    /// Returns the number of active bindings.
    pub fn bind_count(&self) -> u64 {
        self.binds.load(Ordering::Acquire)
    }
}

/// Description supplied when publishing an interface.
pub struct Publication {
    /// Interface name, such as `roanix.block`.
    pub name: Box<str>,
    /// Implementation version. Consumers request a minimum.
    pub version: u32,
    /// Behaviour flags drawn from [`flags`].
    pub flags: u32,
    /// Immutable operation table.
    pub ops: *const c_void,
    /// Operation-table size used for forward-compatible extension.
    pub ops_size: usize,
    /// Provider-defined context passed back to every operation.
    pub context: *mut c_void,
}

/// An active binding to a published interface.
///
/// Dropping the reference releases the provider's module dependency, so a
/// consumer must keep it for as long as it uses the operation table.
pub struct InterfaceRef {
    interface: Arc<Interface>,
    consumer: Option<Arc<Module>>,
}

impl InterfaceRef {
    /// Returns the bound interface.
    pub fn interface(&self) -> &Arc<Interface> {
        &self.interface
    }

    /// Returns the operation table and provider context.
    pub fn ops(&self) -> (*const c_void, *mut c_void) {
        (self.interface.ops, self.interface.context)
    }
}

impl Drop for InterfaceRef {
    fn drop(&mut self) {
        self.interface.binds.fetch_sub(1, Ordering::Release);
        super::module::unlink(self.consumer.as_ref(), self.interface.owner.as_ref());
    }
}

struct Registry {
    global: Mutex<BTreeMap<Box<str>, Vec<Arc<Interface>>>>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        global: Mutex::new(BTreeMap::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

fn validate(publication: &Publication) -> Result<()> {
    if publication.name.is_empty() || publication.name.len() > MAX_NAME {
        return Err(Error::InvalidArgument);
    }
    if publication.ops.is_null() || publication.ops_size == 0 {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

/// Publishes a system-wide interface.
///
/// # Safety
///
/// `publication.ops` must address an immutable table of at least `ops_size`
/// bytes whose function pointers follow the ABI named by `publication.name`,
/// and it must stay valid until the interface is withdrawn.
pub unsafe fn publish_global(
    owner: Option<&Arc<Module>>,
    publication: Publication,
) -> Result<Arc<Interface>> {
    validate(&publication)?;
    let registry = registry()?;
    let mut global = registry.global.lock();
    let entries = global.entry(publication.name.clone()).or_default();
    if publication.flags & flags::SINGLETON != 0 && entries.iter().any(|entry| entry.is_live()) {
        return Err(Error::AlreadyExists);
    }
    let header = ObjHeader::new_with(
        ObjKind::Interface,
        Some(&publication.name),
        owner.map(|module| module.id().get()),
        None,
    );
    let interface = Arc::new(Interface {
        header,
        name: publication.name,
        version: publication.version,
        flags: publication.flags,
        ops: publication.ops,
        ops_size: publication.ops_size,
        context: publication.context,
        owner: owner.cloned(),
        device: None,
        revoked: AtomicBool::new(false),
        binds: AtomicU64::new(0),
    });
    interface.header.register()?;
    entries.push(interface.clone());
    drop(global);
    super::probe::retrigger();
    Ok(interface)
}

/// Publishes an interface describing one device's capability.
///
/// # Safety
///
/// The same requirements as [`publish_global`] apply.
pub unsafe fn publish_device(
    owner: Option<&Arc<Module>>,
    device: &Arc<Device>,
    publication: Publication,
) -> Result<Arc<Interface>> {
    validate(&publication)?;
    let header = ObjHeader::new_with(
        ObjKind::Interface,
        Some(&publication.name),
        owner.map(|module| module.id().get()),
        Some(device.object_id()),
    );
    let interface = Arc::new(Interface {
        header,
        name: publication.name,
        version: publication.version,
        flags: publication.flags,
        ops: publication.ops,
        ops_size: publication.ops_size,
        context: publication.context,
        owner: owner.cloned(),
        device: Some(Arc::downgrade(device)),
        revoked: AtomicBool::new(false),
        binds: AtomicU64::new(0),
    });
    device.attach_interface(interface.clone())?;
    if let Err(error) = interface.header.register() {
        device.detach_interface(&interface);
        return Err(error);
    }
    super::probe::retrigger();
    Ok(interface)
}

/// Withdraws an interface once no consumer is bound to it.
pub fn withdraw(interface: &Arc<Interface>) -> Result<()> {
    if interface.binds.load(Ordering::Acquire) != 0 {
        return Err(Error::Busy);
    }
    if interface
        .revoked
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return Err(Error::NotFound);
    }
    interface
        .header
        .set_state(super::super::obj::ObjState::Removing);
    if interface.binds.load(Ordering::Acquire) != 0 {
        interface.revoked.store(false, Ordering::Release);
        interface
            .header
            .set_state(super::super::obj::ObjState::Live);
        return Err(Error::Busy);
    }
    detach(interface);
    interface.header.poison();
    Ok(())
}

/// Withdraws an interface even if consumers remain bound.
///
/// Used only while tearing a module down, where consumers are being removed in
/// the same pass.
fn force_withdraw(interface: &Arc<Interface>) {
    interface
        .header
        .set_state(super::super::obj::ObjState::Removing);
    interface.revoked.store(true, Ordering::Release);
    detach(interface);
    interface.header.poison();
}

fn detach(interface: &Arc<Interface>) {
    if let Some(device) = interface.device() {
        device.detach_interface(interface);
        return;
    }
    let Ok(registry) = registry() else {
        return;
    };
    let mut global = registry.global.lock();
    if let Some(entries) = global.get_mut(&interface.name) {
        entries.retain(|entry| !Arc::ptr_eq(entry, interface));
        if entries.is_empty() {
            global.remove(&interface.name);
        }
    }
}

fn attach_ref(consumer: Option<&Arc<Module>>, interface: Arc<Interface>) -> Result<InterfaceRef> {
    super::module::link(consumer, interface.owner.as_ref())?;
    interface.binds.fetch_add(1, Ordering::AcqRel);
    if !interface.is_live() {
        interface.binds.fetch_sub(1, Ordering::Release);
        super::module::unlink(consumer, interface.owner.as_ref());
        return Err(Error::NoDevice);
    }
    Ok(InterfaceRef {
        interface,
        consumer: consumer.cloned(),
    })
}

/// Binds to a system-wide interface.
///
/// Returns [`Error::Deferred`] when no compatible provider is loaded yet, which
/// lets a probing driver be retried once one appears.
pub fn bind_global(
    consumer: Option<&Arc<Module>>,
    name: &str,
    min_version: u32,
) -> Result<InterfaceRef> {
    let registry = registry()?;
    let candidate = {
        let global = registry.global.lock();
        global.get(name).and_then(|entries| {
            entries
                .iter()
                .filter(|entry| entry.is_live() && entry.version >= min_version)
                .max_by_key(|entry| entry.version)
                .cloned()
        })
    };
    let Some(interface) = candidate else {
        return Err(Error::Deferred);
    };
    attach_ref(consumer, interface)
}

/// Binds to an interface published by `device`.
pub fn bind_device(
    consumer: Option<&Arc<Module>>,
    device: &Arc<Device>,
    name: &str,
    min_version: u32,
) -> Result<InterfaceRef> {
    let Some(interface) = device.find_interface(name, min_version) else {
        return Err(Error::Deferred);
    };
    attach_ref(consumer, interface)
}

/// Binds to an interface published by `device` or any of its ancestors.
///
/// This walks the device tree upward and is how a child device reaches the
/// services of the bridge or controller it sits behind.
pub fn bind_ancestor(
    consumer: Option<&Arc<Module>>,
    device: &Arc<Device>,
    name: &str,
    min_version: u32,
) -> Result<InterfaceRef> {
    let mut current = Some(device.clone());
    while let Some(node) = current {
        if let Some(interface) = node.find_interface(name, min_version) {
            return attach_ref(consumer, interface);
        }
        current = node.parent();
    }
    Err(Error::Deferred)
}

/// Returns every live provider of a global interface name.
pub fn enumerate(name: &str, min_version: u32) -> Result<Vec<Arc<Interface>>> {
    Ok(registry()?
        .global
        .lock()
        .get(name)
        .map(|entries| {
            entries
                .iter()
                .filter(|entry| entry.is_live() && entry.version >= min_version)
                .cloned()
                .collect()
        })
        .unwrap_or_default())
}

/// Returns whether a compatible global provider exists.
pub fn available(name: &str, min_version: u32) -> bool {
    registry().is_ok_and(|registry| {
        registry.global.lock().get(name).is_some_and(|entries| {
            entries
                .iter()
                .any(|entry| entry.is_live() && entry.version >= min_version)
        })
    })
}

pub(super) fn remove_module_interfaces(module: &Arc<Module>) {
    let Ok(registry) = registry() else {
        return;
    };
    let owned: Vec<Arc<Interface>> = registry
        .global
        .lock()
        .values()
        .flatten()
        .filter(|interface| {
            interface
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for interface in owned {
        force_withdraw(&interface);
    }
}

/// Snapshot of one published interface.
#[derive(Clone, Debug)]
pub struct InterfaceInfo {
    /// Published name.
    pub name: Box<str>,
    /// Implementation version.
    pub version: u32,
    /// Number of active bindings.
    pub binds: u64,
    /// Publishing module name, or `None` for kernel-provided services.
    pub owner: Option<Box<str>>,
}

/// Returns a snapshot of every global interface.
pub fn list() -> Result<Vec<InterfaceInfo>> {
    Ok(registry()?
        .global
        .lock()
        .values()
        .flatten()
        .map(|interface| InterfaceInfo {
            name: String::from(&*interface.name).into_boxed_str(),
            version: interface.version,
            binds: interface.bind_count(),
            owner: interface
                .owner
                .as_ref()
                .map(|owner| String::from(owner.name()).into_boxed_str()),
        })
        .collect())
}
