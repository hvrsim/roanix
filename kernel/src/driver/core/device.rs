//! The device tree.
//!
//! A device is a single uniform node. Unlike the previous framework there is no
//! separate bus object type in the tree: any device may have children, so a
//! PCI-to-PCI bridge, a USB hub, or an I2C controller is just a device that
//! happens to have descendants. What kind of bus its children sit on is
//! described by the [`Bus`] they are registered against.
//!
//! Devices are built in two phases. [`DeviceBuilder`] collects the name,
//! firmware description, properties, and hardware resources; [`add`] publishes
//! the result. After publication the descriptive state is immutable, so drivers
//! read properties and resources without taking any lock.

use alloc::{
    boxed::Box,
    format,
    string::{String, ToString},
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicU32, AtomicU64, AtomicUsize, Ordering};

use crate::sys::sync::{Mutex, Once};

use super::{
    super::{
        error::{Error, Result},
        irq::IrqDomain,
        obj::{ObjHeader, ObjKind, framework_object},
    },
    bus::Bus,
    driver::Driver,
    fwnode::Fwnode,
    iface::Interface,
    module::{self, Module},
    name::Name,
    property::{PropValue, Properties, PropertyBuilder},
    resource::{self, Resource, ResourceBuilder},
};

/// Maximum device-name length in bytes.
pub const MAX_NAME: usize = 96;

/// Device lifecycle state.
pub mod state {
    /// Published and waiting for a driver.
    ///
    /// Devices are born directly in this state; there is no unpublished
    /// device state because `add` constructs and publishes atomically.
    pub const UNBOUND: u32 = 1;
    /// A driver's probe callback is running.
    pub const PROBING: u32 = 2;
    /// A driver has claimed the device.
    pub const BOUND: u32 = 3;
    /// Probing was deferred until a prerequisite appears.
    pub const DEFERRED: u32 = 4;
    /// The device is being removed.
    pub const REMOVING: u32 = 5;
    /// The device has been removed.
    pub const DEAD: u32 = 6;
}

/// An interrupt line described by firmware but not yet routed.
pub struct DeviceIrq {
    domain: Mutex<Option<Arc<IrqDomain>>>,
    explicit_domain: bool,
    firmware_parent: Option<u32>,
    cells: Box<[u32]>,
    virq: AtomicU32,
}

impl DeviceIrq {
    /// Firmware controller identity retained independently for each specifier.
    pub fn firmware_parent(&self) -> Option<u32> {
        self.firmware_parent
    }

    /// Returns the raw firmware specifier cells.
    pub fn cells(&self) -> &[u32] {
        &self.cells
    }

    /// Returns the controller domain that decodes this specifier.
    pub fn domain(&self) -> Option<Arc<IrqDomain>> {
        self.domain.lock().clone()
    }

    /// Returns the resolved virtual interrupt number, if any.
    pub fn virq(&self) -> Option<u32> {
        match self.virq.load(Ordering::Acquire) {
            0 => None,
            value => Some(value),
        }
    }

    pub(crate) fn set_virq(&self, virq: u32) {
        self.virq.store(virq, Ordering::Release);
    }

    pub(crate) fn resolution(&self) -> crate::sys::sync::MutexGuard<'_, Option<Arc<IrqDomain>>> {
        self.domain.lock()
    }

    pub(crate) fn has_explicit_domain(&self) -> bool {
        self.explicit_domain
    }
}

struct DeviceInner {
    children: Vec<Arc<Device>>,
    driver: Option<Arc<Driver>>,
    interfaces: Vec<Arc<Interface>>,
}

/// A node in the device tree.
#[repr(C)]
pub struct Device {
    header: ObjHeader,
    id: u64,
    name: Name,
    path: Box<str>,
    parent: Option<Weak<Device>>,
    bus: Option<Arc<Bus>>,
    bus_lease: Mutex<Option<module::ModuleLease>>,
    owner: Option<Arc<Module>>,
    fwnode: Option<Arc<Fwnode>>,
    properties: Properties,
    resources: Box<[Resource]>,
    irqs: Box<[DeviceIrq]>,
    dma_mask: AtomicU64,
    drvdata: AtomicUsize,
    state: AtomicU32,
    inner: Mutex<DeviceInner>,
    self_ref: Mutex<Weak<Device>>,
}

framework_object!(Device, Device);

impl Device {
    /// Returns the device's name within its parent.
    pub fn name(&self) -> &str {
        self.name.as_str()
    }

    /// Returns the device's name as a NUL-terminated pointer for the ABI.
    pub fn name_c_ptr(&self) -> *const core::ffi::c_char {
        self.name.as_c_ptr()
    }

    /// Returns the device's absolute path in the tree.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// Returns the stable numeric identity used for diagnostics.
    pub const fn id(&self) -> u64 {
        self.id
    }

    /// Returns the parent device, or `None` for the tree root.
    pub fn parent(&self) -> Option<Arc<Device>> {
        self.parent.as_ref().and_then(Weak::upgrade)
    }

    /// Returns the bus this device is attached to.
    pub fn bus(&self) -> Option<&Arc<Bus>> {
        self.bus.as_ref()
    }

    /// Returns the module that created this device.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns the firmware node describing this device.
    pub fn fwnode(&self) -> Option<&Arc<Fwnode>> {
        self.fwnode.as_ref()
    }

    /// Returns this device's immutable properties.
    pub const fn properties(&self) -> &Properties {
        &self.properties
    }

    /// Returns this device's immutable hardware resources.
    pub fn resources(&self) -> &[Resource] {
        &self.resources
    }

    /// Returns this device's firmware-described interrupt lines.
    pub fn irqs(&self) -> &[DeviceIrq] {
        &self.irqs
    }

    /// Returns the `index`-th resource of `kind`.
    pub fn resource(&self, kind: u32, index: usize) -> Option<&Resource> {
        resource::find(&self.resources, kind, index)
    }

    /// Returns the resource of `kind` labelled `name`.
    pub fn resource_named(&self, kind: u32, name: &str) -> Option<&Resource> {
        resource::find_named(&self.resources, kind, name)
    }

    /// Returns the addressing limit this device can reach with DMA.
    pub fn dma_mask(&self) -> u64 {
        self.dma_mask.load(Ordering::Relaxed)
    }

    /// Restricts the addressing limit this device can reach with DMA.
    pub fn set_dma_mask(&self, mask: u64) {
        self.dma_mask.store(mask, Ordering::Relaxed);
    }

    /// Returns the driver-private instance pointer.
    ///
    /// This is an atomic load with no locking so it can be read from interrupt
    /// handlers on the hot path.
    pub fn drvdata(&self) -> usize {
        self.drvdata.load(Ordering::Acquire)
    }

    /// Stores the driver-private instance pointer.
    pub fn set_drvdata(&self, value: usize) {
        self.drvdata.store(value, Ordering::Release);
    }

    /// Returns the current lifecycle state.
    pub fn state(&self) -> u32 {
        self.state.load(Ordering::Acquire)
    }

    pub(crate) fn set_state(&self, value: u32) {
        self.state.store(value, Ordering::Release);
        let object_state = match value {
            state::REMOVING => super::super::obj::ObjState::Removing,
            state::DEAD => super::super::obj::ObjState::Dead,
            _ => super::super::obj::ObjState::Live,
        };
        self.header.set_state(object_state);
    }

    pub(crate) fn transition(&self, from: u32, to: u32) -> bool {
        let changed = self
            .state
            .compare_exchange(from, to, Ordering::AcqRel, Ordering::Acquire)
            .is_ok();
        if changed {
            self.set_state(to);
        }
        changed
    }

    /// Returns the driver currently bound to this device.
    pub fn driver(&self) -> Option<Arc<Driver>> {
        self.inner.lock().driver.clone()
    }

    pub(crate) fn set_driver(&self, driver: Option<Arc<Driver>>) {
        self.inner.lock().driver = driver;
    }

    /// Returns this device's children.
    pub fn children(&self) -> Vec<Arc<Device>> {
        self.inner.lock().children.clone()
    }

    /// Returns a counted reference to this device.
    pub fn arc(&self) -> Option<Arc<Device>> {
        self.self_ref.lock().upgrade()
    }

    pub(super) fn attach_interface(&self, interface: Arc<Interface>) -> Result<()> {
        let mut inner = self.inner.lock();
        if matches!(self.state(), state::REMOVING | state::DEAD)
            || !interface.binding_matches(inner.driver.as_ref())
        {
            return Err(Error::NoDevice);
        }
        if inner
            .interfaces
            .iter()
            .any(|entry| entry.name() == interface.name() && entry.is_live())
        {
            return Err(Error::AlreadyExists);
        }
        inner.interfaces.push(interface);
        Ok(())
    }

    /// Joins interface use under the same lock that closes removal admission.
    pub(super) fn admit_interface(&self, interface: &Arc<Interface>) -> Result<()> {
        let inner = self.inner.lock();
        if matches!(self.state(), state::REMOVING | state::DEAD)
            || !inner
                .interfaces
                .iter()
                .any(|entry| Arc::ptr_eq(entry, interface))
        {
            return Err(Error::NoDevice);
        }
        interface.admit()
    }

    /// Removal owns admission already. Descendants retire first so their
    /// references to ancestor services can drain before this check.
    pub(super) fn interfaces_idle(&self) -> Result<()> {
        if self
            .inner
            .lock()
            .interfaces
            .iter()
            .any(|entry| entry.bind_count() != 0)
        {
            Err(Error::Busy)
        } else {
            Ok(())
        }
    }

    pub(super) fn driver_interfaces_idle(&self, driver: &Arc<Driver>) -> Result<()> {
        if self
            .inner
            .lock()
            .interfaces
            .iter()
            .any(|entry| entry.belongs_to(driver) && entry.bind_count() != 0)
        {
            Err(Error::Busy)
        } else {
            Ok(())
        }
    }

    pub(super) fn detach_interface(&self, interface: &Arc<Interface>) {
        self.inner
            .lock()
            .interfaces
            .retain(|entry| !Arc::ptr_eq(entry, interface));
    }

    pub(super) fn find_interface(&self, name: &str, min_version: u32) -> Option<Arc<Interface>> {
        self.inner
            .lock()
            .interfaces
            .iter()
            .filter(|entry| {
                entry.name() == name && entry.is_live() && entry.version() >= min_version
            })
            .max_by_key(|entry| entry.version())
            .cloned()
    }

    /// Returns every interface this device publishes.
    pub fn interfaces(&self) -> Vec<Arc<Interface>> {
        self.inner.lock().interfaces.clone()
    }

    /// Returns whether `ancestor` is this device or one of its ancestors.
    pub fn is_descendant_of(&self, ancestor: &Arc<Device>) -> bool {
        let mut current = self.arc();
        while let Some(node) = current {
            if Arc::ptr_eq(&node, ancestor) {
                return true;
            }
            current = node.parent();
        }
        false
    }
}

/// Collects the description of a device before it is published.
pub struct DeviceBuilder {
    name: Box<str>,
    parent: Option<Arc<Device>>,
    bus: Option<Arc<Bus>>,
    owner: Option<Arc<Module>>,
    fwnode: Option<Arc<Fwnode>>,
    properties: PropertyBuilder,
    resources: ResourceBuilder,
    irqs: Vec<DeviceIrq>,
    dma_mask: u64,
}

impl DeviceBuilder {
    /// Starts describing a device named `name`.
    pub fn new(owner: Option<&Arc<Module>>, name: &str) -> Result<Self> {
        if name.is_empty() || name.len() > MAX_NAME || name.contains('/') {
            return Err(Error::InvalidArgument);
        }
        Ok(Self {
            name: String::from(name).into_boxed_str(),
            parent: None,
            bus: None,
            owner: owner.cloned(),
            fwnode: None,
            properties: PropertyBuilder::new(),
            resources: ResourceBuilder::new(),
            irqs: Vec::new(),
            dma_mask: u64::MAX,
        })
    }

    /// Sets the parent device.
    pub fn parent(mut self, parent: &Arc<Device>) -> Self {
        self.parent = Some(parent.clone());
        self
    }

    /// Sets the bus the device is attached to.
    pub fn bus(mut self, bus: &Arc<Bus>) -> Self {
        self.bus = Some(bus.clone());
        self
    }

    /// Attaches a firmware description node.
    pub fn fwnode(mut self, fwnode: &Arc<Fwnode>) -> Self {
        self.fwnode = Some(fwnode.clone());
        self
    }

    /// Restricts the addressing limit reachable with DMA.
    pub const fn dma_mask(mut self, mask: u64) -> Self {
        self.dma_mask = mask;
        self
    }

    /// Adds or replaces a property.
    pub fn property(&mut self, name: &str, value: PropValue) -> Result<()> {
        self.properties.set(name, value)
    }

    /// Appends a hardware resource.
    pub fn resource(&mut self, resource: Resource) -> Result<usize> {
        resource.validate()?;
        self.resources.push(resource)
    }

    /// Appends a firmware-described interrupt specifier.
    pub fn irq(&mut self, domain: Option<Arc<IrqDomain>>, cells: &[u32]) -> Result<usize> {
        if self.irqs.len() >= resource::MAX_RESOURCES {
            return Err(Error::NoSpace);
        }
        let index = self.irqs.len();
        self.irqs.push(DeviceIrq {
            explicit_domain: domain.is_some(),
            firmware_parent: None,
            domain: Mutex::new(domain),
            cells: cells.to_vec().into_boxed_slice(),
            virq: AtomicU32::new(0),
        });
        Ok(index)
    }
    /// Adds a specifier whose controller may register after firmware publication.
    pub fn firmware_irq(&mut self, phandle: u32, cells: &[u32]) -> Result<usize> {
        if phandle == 0 || phandle == u32::MAX {
            return Err(Error::InvalidArgument);
        }
        let index = self.irq(None, cells)?;
        self.irqs[index].firmware_parent = Some(phandle);
        Ok(index)
    }
}

struct Registry {
    root: Arc<Device>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| {
        let header = ObjHeader::new_with(ObjKind::Device, Some("devices"), None, None);
        let id = header.id();
        let root = Arc::new(Device {
            header,
            id,
            name: Name::new("devices"),
            path: Box::from("/"),
            parent: None,
            bus: None,
            bus_lease: Mutex::new(None),
            owner: None,
            fwnode: None,
            properties: Properties::empty(),
            resources: Box::new([]),
            irqs: Box::new([]),
            dma_mask: AtomicU64::new(u64::MAX),
            drvdata: AtomicUsize::new(0),
            state: AtomicU32::new(state::UNBOUND),
            inner: Mutex::new(DeviceInner {
                children: Vec::new(),
                driver: None,
                interfaces: Vec::new(),
            }),
            self_ref: Mutex::new(Weak::new()),
        });
        *root.self_ref.lock() = Arc::downgrade(&root);
        root.header
            .register()
            .expect("driver/device: failed to register device root");
        Registry { root }
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// Returns the root of the device tree.
pub fn root() -> Result<Arc<Device>> {
    Ok(registry()?.root.clone())
}

/// Publishes a described device and attempts to bind a driver to it.
pub fn add(builder: DeviceBuilder) -> Result<Arc<Device>> {
    // Join owner admission before publishing anything. Unload closes new pins
    // and checks active users before sweeping the owner's devices.
    let _owner = module::pin_owner(builder.owner.as_ref(), false)?;
    let bus_lease = if builder.owner.is_none() {
        builder
            .bus
            .as_ref()
            .and_then(|bus| bus.owner())
            .map(module::lease)
            .transpose()?
    } else {
        None
    };
    let registry = registry()?;
    let parent = match builder.parent {
        Some(parent) => parent,
        None => registry.root.clone(),
    };
    if matches!(parent.state(), state::REMOVING | state::DEAD) {
        return Err(Error::NoDevice);
    }

    let path = if &*parent.path == "/" {
        format!("/{}", builder.name)
    } else {
        format!("{}/{}", parent.path, builder.name)
    };
    {
        let children = parent.inner.lock();
        if children
            .children
            .iter()
            .any(|child| child.name.as_str() == &*builder.name)
        {
            return Err(Error::AlreadyExists);
        }
    }

    let header = ObjHeader::new_with(
        ObjKind::Device,
        Some(&builder.name),
        builder.owner.as_ref().map(|owner| owner.id().get()),
        Some(parent.id()),
    );
    let id = header.id();
    let device = Arc::new(Device {
        header,
        id,
        name: Name::new(&builder.name),
        path: path.into_boxed_str(),
        parent: Some(Arc::downgrade(&parent)),
        bus: builder.bus,
        bus_lease: Mutex::new(bus_lease),
        owner: builder.owner,
        fwnode: builder.fwnode,
        properties: builder.properties.build(),
        resources: builder.resources.build(),
        irqs: builder.irqs.into_boxed_slice(),
        dma_mask: AtomicU64::new(builder.dma_mask),
        drvdata: AtomicUsize::new(0),
        state: AtomicU32::new(state::UNBOUND),
        inner: Mutex::new(DeviceInner {
            children: Vec::new(),
            driver: None,
            interfaces: Vec::new(),
        }),
        self_ref: Mutex::new(Weak::new()),
    });
    *device.self_ref.lock() = Arc::downgrade(&device);
    device.header.register()?;
    {
        let mut children = parent.inner.lock();
        // Re-check under the same critical section that publishes the child:
        // the earlier duplicate check raced with any concurrent sibling add.
        if children
            .children
            .iter()
            .any(|child| child.name.as_str() == &*builder.name)
        {
            device.header.poison();
            return Err(Error::AlreadyExists);
        }
        if matches!(parent.state(), state::REMOVING | state::DEAD) {
            device.header.poison();
            return Err(Error::NoDevice);
        }
        children.children.push(device.clone());
    }

    // The device keeps its bus handle for its whole lifetime, so its creator
    // must pin the bus owner's image until removal.
    let bus_owner = device.bus().and_then(|bus| bus.owner().cloned());
    if module::link(device.owner(), bus_owner.as_ref()).is_err() {
        parent
            .inner
            .lock()
            .children
            .retain(|child| !Arc::ptr_eq(child, &device));
        device.header.poison();
        return Err(Error::Deadlock);
    }

    super::probe::attach(&device);
    Ok(device)
}

/// Exclusive ownership of a device's removal transition. Dropping an unfinished
/// claim restores admission; no callback may run without owning this claim.
pub(super) struct Removal {
    device: Arc<Device>,
    previous: u32,
    finished: bool,
}

impl Removal {
    pub(super) fn claim(device: &Arc<Device>) -> Result<Option<Self>> {
        // Serialize with child publication, which checks the parent's state
        // under this same lock before adding its child.
        let _inner = device.inner.lock();
        loop {
            let previous = device.state();
            match previous {
                state::DEAD => return Ok(None),
                state::PROBING | state::REMOVING => return Err(Error::Busy),
                _ => {}
            }
            if device.transition(previous, state::REMOVING) {
                return Ok(Some(Self {
                    device: device.clone(),
                    previous,
                    finished: false,
                }));
            }
        }
    }

    pub(super) fn device(&self) -> &Arc<Device> {
        &self.device
    }

    pub(super) fn finish(mut self, state: u32) {
        self.device.set_state(state);
        self.finished = true;
    }
}

impl Drop for Removal {
    fn drop(&mut self) {
        if !self.finished {
            self.device.set_state(self.previous);
        }
    }
}

/// Removes a device and everything below it. Claim the complete subtree before
/// invoking any callback, so a probing/busy child cannot strand a removed parent.
/// Checked callback failures restore the failing device and unprocessed ancestors
/// to their previous states. Descendants already removed remain removed; their
/// successful teardown is never undone or falsely presented as still bound.
pub fn remove(device: &Arc<Device>) -> Result<()> {
    if Arc::ptr_eq(device, &registry()?.root) {
        return Err(Error::PermissionDenied);
    }
    let mut pending = alloc::vec![device.clone()];
    let mut claims = Vec::new();
    while let Some(device) = pending.pop() {
        if let Some(claim) = Removal::claim(&device)? {
            pending.extend(device.children());
            claims.push(claim);
        }
    }
    for claim in claims.into_iter().rev() {
        let device = claim.device();
        device.interfaces_idle()?;
        super::probe::detach(&claim)?;
        for interface in device.interfaces() {
            match super::iface::withdraw(&interface) {
                Ok(()) | Err(Error::NotFound) => {}
                Err(error) => return Err(error),
            }
        }
        super::super::irq::release_device_interrupts(device);
        module::unlink(device.owner(), device.bus().and_then(|bus| bus.owner()));
        if let Some(parent) = device.parent() {
            parent
                .inner
                .lock()
                .children
                .retain(|child| !Arc::ptr_eq(child, device));
        }
        device.header.poison();
        let bus_lease = device.bus_lease.lock().take();
        drop(bus_lease);
        claim.finish(state::DEAD);
    }
    Ok(())
}

/// Applies `visit` to every device in the tree, parents before children.
pub fn for_each<F: FnMut(&Arc<Device>)>(mut visit: F) {
    let Ok(registry) = registry() else {
        return;
    };
    let mut stack = alloc::vec![registry.root.clone()];
    while let Some(device) = stack.pop() {
        visit(&device);
        stack.extend(device.children());
    }
}

/// Returns every device in the tree.
pub fn all() -> Vec<Arc<Device>> {
    let mut devices = Vec::new();
    for_each(|device| devices.push(device.clone()));
    devices
}

/// Returns the device at an absolute tree path.
pub fn lookup(path: &str) -> Result<Arc<Device>> {
    let registry = registry()?;
    let mut current = registry.root.clone();
    for component in path.split('/').filter(|part| !part.is_empty()) {
        let next = current
            .inner
            .lock()
            .children
            .iter()
            .find(|child| child.name.as_str() == component)
            .cloned();
        current = next.ok_or(Error::NotFound)?;
    }
    Ok(current)
}

/// Withdraws every device owned by a module while its callback contexts remain
/// valid. A refusal leaves the failed device and unprocessed receipts published.
pub(super) fn remove_module_devices(module: &Arc<Module>) -> Result<()> {
    let mut owned = Vec::new();
    for_each(|device| {
        if device
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module))
        {
            owned.push(device.clone());
        }
    });
    // Children retire before their parents; remove() itself claims complete
    // subtrees, including children owned by kernel clients or other modules.
    owned.sort_unstable_by_key(|device| {
        core::cmp::Reverse(device.path.bytes().filter(|byte| *byte == b'/').count())
    });
    for device in owned {
        remove(&device)?;
    }
    Ok(())
}

/// Snapshot of one device for diagnostics.
#[derive(Clone, Debug)]
pub struct DeviceInfo {
    /// Absolute tree path.
    pub path: Box<str>,
    /// Bus name, if the device sits on a registered bus.
    pub bus: Option<Box<str>>,
    /// Bound driver name.
    pub driver: Option<Box<str>>,
    /// Lifecycle state.
    pub state: u32,
}

/// Returns a snapshot of the whole device tree.
pub fn list() -> Vec<DeviceInfo> {
    let mut entries = Vec::new();
    for_each(|device| {
        entries.push(DeviceInfo {
            path: device.path.to_string().into_boxed_str(),
            bus: device
                .bus
                .as_ref()
                .map(|bus| bus.name().to_string().into_boxed_str()),
            driver: device
                .driver()
                .map(|driver| driver.name().to_string().into_boxed_str()),
            state: device.state(),
        });
    });
    entries
}
