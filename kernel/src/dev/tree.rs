//! Hierarchical bus and device registry.

use alloc::{boxed::Box, collections::BTreeMap, string::String, sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::sys::sync::{Mutex, Once};

use super::{
    error::{Error, Result},
    resource::{Resource, ResourceFlags, ResourceId, ResourceKey, ResourceValue},
};

const ROOT_NODE_ID: u64 = 1;

/// Driver owning framework objects.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DriverId(u64);

impl DriverId {
    pub(crate) const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Owner used by kernel-native objects.
pub const KERNEL_DRIVER: DriverId = DriverId(0);

/// Stable identifier for a node in the device hierarchy.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DeviceNodeId(u64);

impl DeviceNodeId {
    const fn new(value: u64) -> Self {
        Self(value)
    }

    pub(crate) const fn from_raw(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Type-safe bus handle.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BusId(DeviceNodeId);

impl BusId {
    /// Returns the underlying hierarchy node identifier.
    pub const fn node(self) -> DeviceNodeId {
        self.0
    }

    /// Creates a bus handle from a raw node handle after runtime validation.
    pub fn from_node(node: DeviceNodeId) -> Result<Self> {
        let registry = registry()?.state.lock();
        match registry.nodes.get(&node).map(|record| record.info.kind) {
            Some(NodeKind::Bus) => Ok(Self(node)),
            Some(NodeKind::Device) => Err(Error::WrongKind),
            None => Err(Error::NotFound),
        }
    }
}

/// Type-safe device handle.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DeviceId(DeviceNodeId);

impl DeviceId {
    /// Returns the underlying hierarchy node identifier.
    pub const fn node(self) -> DeviceNodeId {
        self.0
    }

    /// Creates a device handle from a raw node handle after runtime validation.
    pub fn from_node(node: DeviceNodeId) -> Result<Self> {
        let registry = registry()?.state.lock();
        match registry.nodes.get(&node).map(|record| record.info.kind) {
            Some(NodeKind::Device) => Ok(Self(node)),
            Some(NodeKind::Bus) => Err(Error::WrongKind),
            None => Err(Error::NotFound),
        }
    }
}

impl From<BusId> for DeviceNodeId {
    fn from(bus: BusId) -> Self {
        bus.node()
    }
}

impl From<DeviceId> for DeviceNodeId {
    fn from(device: DeviceId) -> Self {
        device.node()
    }
}

/// Device hierarchy node type.
#[repr(u32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum NodeKind {
    /// A bus capable of containing devices and nested buses.
    Bus = 1,
    /// A leaf or controller device.
    Device = 2,
}

/// Owned snapshot of hierarchy metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NodeInfo {
    /// Stable node identifier.
    pub id: DeviceNodeId,
    /// Driver that owns this node.
    pub owner: DriverId,
    /// Node type.
    pub kind: NodeKind,
    /// Parent bus, or `None` for the root bus.
    pub parent: Option<BusId>,
    /// Name within the parent bus.
    pub name: Box<str>,
}

struct NodeRecord {
    info: NodeInfo,
    children: BTreeMap<Box<str>, DeviceNodeId>,
    resources: BTreeMap<ResourceKey, Arc<Resource>>,
}

struct RegistryState {
    nodes: BTreeMap<DeviceNodeId, NodeRecord>,
}

struct DeviceRegistry {
    next_node: AtomicU64,
    next_resource: AtomicU64,
    state: Mutex<RegistryState>,
}

static REGISTRY: Once<DeviceRegistry> = Once::new();

impl DeviceRegistry {
    fn new() -> Self {
        let root_id = DeviceNodeId::new(ROOT_NODE_ID);
        let root = NodeRecord {
            info: NodeInfo {
                id: root_id,
                owner: KERNEL_DRIVER,
                kind: NodeKind::Bus,
                parent: None,
                name: Box::from("root"),
            },
            children: BTreeMap::new(),
            resources: BTreeMap::new(),
        };
        let mut nodes = BTreeMap::new();
        nodes.insert(root_id, root);
        Self {
            next_node: AtomicU64::new(ROOT_NODE_ID + 1),
            next_resource: AtomicU64::new(1),
            state: Mutex::new(RegistryState { nodes }),
        }
    }

    fn allocate_node_id(&self) -> DeviceNodeId {
        let id = self.next_node.fetch_add(1, Ordering::Relaxed);
        assert!(id != 0, "dev: node identifier wrapped");
        DeviceNodeId::new(id)
    }

    fn allocate_resource_id(&self) -> ResourceId {
        let id = self.next_resource.fetch_add(1, Ordering::Relaxed);
        assert!(id != 0, "dev: resource identifier wrapped");
        ResourceId::new(id)
    }
}

/// Initializes the root device hierarchy.
pub(crate) fn init() {
    REGISTRY.call_once(DeviceRegistry::new);
}

/// Returns the top-level kernel bus.
pub fn root_bus() -> Result<BusId> {
    registry()?;
    Ok(BusId(DeviceNodeId::new(ROOT_NODE_ID)))
}

/// Registers a nested bus.
pub fn register_bus(owner: DriverId, parent: BusId, name: &str) -> Result<BusId> {
    let _owner = super::driver::mutation_guard(owner)?;
    let parent_owner = node_info(parent.node())?.owner;
    let _parent = super::driver::parent_guard(owner, parent_owner)?;
    register_node(owner, parent, name, NodeKind::Bus).map(BusId)
}

/// Registers a device below a bus.
pub fn register_device(owner: DriverId, parent: BusId, name: &str) -> Result<DeviceId> {
    let _owner = super::driver::mutation_guard(owner)?;
    let parent_owner = node_info(parent.node())?.owner;
    let _parent = super::driver::parent_guard(owner, parent_owner)?;
    register_node(owner, parent, name, NodeKind::Device).map(DeviceId)
}

/// Returns a metadata snapshot for one node.
pub fn node_info(node: DeviceNodeId) -> Result<NodeInfo> {
    registry()?
        .state
        .lock()
        .nodes
        .get(&node)
        .map(|record| record.info.clone())
        .ok_or(Error::NotFound)
}

/// Returns direct child snapshots in lexical name order.
pub fn children(bus: BusId) -> Result<Vec<NodeInfo>> {
    let state = registry()?.state.lock();
    let record = state.nodes.get(&bus.node()).ok_or(Error::NotFound)?;
    if record.info.kind != NodeKind::Bus {
        return Err(Error::WrongKind);
    }
    Ok(record
        .children
        .values()
        .filter_map(|id| state.nodes.get(id))
        .map(|child| child.info.clone())
        .collect())
}

/// Removes a node and all same-owner descendants.
pub fn remove_node(owner: DriverId, node: DeviceNodeId) -> Result<()> {
    let _owner = super::driver::mutation_guard(owner)?;
    if node.get() == ROOT_NODE_ID {
        return Err(Error::PermissionDenied);
    }
    let registry = registry()?;
    let mut state = registry.state.lock();
    ensure_owned_subtree(&state, owner, node)?;
    revoke_subtree_resources(&state, node)?;
    remove_subtree(&mut state, node);
    Ok(())
}

/// Publishes an inheritable resource on a bus owned by `owner`.
pub fn publish_resource(
    owner: DriverId,
    bus: BusId,
    key: ResourceKey,
    flags: ResourceFlags,
    mut value: ResourceValue,
) -> Result<ResourceId> {
    let _owner = super::driver::mutation_guard(owner)?;
    if flags.contains(ResourceFlags::IRQ_SAFE) {
        return Err(Error::Unsupported);
    }
    let registry = registry()?;
    let id = registry.allocate_resource_id();
    if let ResourceValue::Method(method) = &mut value {
        method.bind_owner(owner);
    }
    let resource = Arc::new(Resource::new(id, key, flags, value));
    let mut state = registry.state.lock();
    let record = state.nodes.get_mut(&bus.node()).ok_or(Error::NotFound)?;
    if record.info.kind != NodeKind::Bus {
        return Err(Error::WrongKind);
    }
    if record.info.owner != owner {
        return Err(Error::PermissionDenied);
    }
    if record.resources.contains_key(&key) {
        return Err(Error::AlreadyExists);
    }
    record.resources.insert(key, resource);
    Ok(id)
}

/// Removes a resource from an owned bus.
pub fn remove_resource(owner: DriverId, bus: BusId, key: ResourceKey) -> Result<()> {
    let _owner = super::driver::mutation_guard(owner)?;
    let mut state = registry()?.state.lock();
    let record = state.nodes.get_mut(&bus.node()).ok_or(Error::NotFound)?;
    if record.info.owner != owner {
        return Err(Error::PermissionDenied);
    }
    let resource = record.resources.get(&key).ok_or(Error::NotFound)?;
    resource.try_revoke()?;
    record.resources.remove(&key);
    Ok(())
}

/// Resolves the nearest matching resource through ancestor buses.
pub fn resolve_resource(node: DeviceNodeId, key: ResourceKey) -> Result<Arc<Resource>> {
    let state = registry()?.state.lock();
    let mut current = match state.nodes.get(&node).ok_or(Error::NotFound)? {
        record if record.info.kind == NodeKind::Bus => Some(BusId(node)),
        record => record.info.parent,
    };

    while let Some(bus) = current {
        let record = state.nodes.get(&bus.node()).ok_or(Error::NotFound)?;
        if let Some(resource) = record.resources.get(&key) {
            return Ok(resource.clone());
        }
        current = record.info.parent;
    }
    Err(Error::NotFound)
}

pub(crate) fn prepare_remove_driver(owner: DriverId) -> Result<()> {
    let state = registry()?.state.lock();
    for record in state
        .nodes
        .values()
        .filter(|record| record.info.owner == owner)
    {
        for child in record.children.values() {
            let child = state.nodes.get(child).ok_or(Error::NotFound)?;
            if child.info.owner != owner {
                return Err(Error::Busy);
            }
        }
    }
    let resources = state
        .nodes
        .values()
        .filter(|record| record.info.owner == owner)
        .flat_map(|record| record.resources.values().cloned())
        .collect();
    revoke_resources(resources)
}

pub(crate) fn restore_driver_resources(owner: DriverId) -> Result<()> {
    let state = registry()?.state.lock();
    for resource in state
        .nodes
        .values()
        .filter(|record| record.info.owner == owner)
        .flat_map(|record| record.resources.values())
    {
        resource.restore();
    }
    Ok(())
}

pub(crate) fn remove_driver(owner: DriverId) -> Result<()> {
    let registry = registry()?;
    let mut state = registry.state.lock();
    for record in state
        .nodes
        .values()
        .filter(|record| record.info.owner == owner)
    {
        for child in record.children.values() {
            if state
                .nodes
                .get(child)
                .is_some_and(|child| child.info.owner != owner)
            {
                return Err(Error::Busy);
            }
        }
    }

    let roots: Vec<_> = state
        .nodes
        .values()
        .filter(|record| {
            record.info.owner == owner
                && record.info.parent.is_some_and(|parent| {
                    state
                        .nodes
                        .get(&parent.node())
                        .is_some_and(|parent| parent.info.owner != owner)
                })
        })
        .map(|record| record.info.id)
        .collect();
    for root in roots {
        remove_subtree(&mut state, root);
    }
    Ok(())
}

fn register_node(
    owner: DriverId,
    parent: BusId,
    name: &str,
    kind: NodeKind,
) -> Result<DeviceNodeId> {
    validate_name(name)?;
    let registry = registry()?;
    let id = registry.allocate_node_id();
    let mut state = registry.state.lock();
    let parent_record = state.nodes.get_mut(&parent.node()).ok_or(Error::NotFound)?;
    if parent_record.info.kind != NodeKind::Bus {
        return Err(Error::WrongKind);
    }
    if parent_record.children.contains_key(name) {
        return Err(Error::AlreadyExists);
    }

    let name: Box<str> = String::from(name).into_boxed_str();
    parent_record.children.insert(name.clone(), id);
    state.nodes.insert(
        id,
        NodeRecord {
            info: NodeInfo {
                id,
                owner,
                kind,
                parent: Some(parent),
                name,
            },
            children: BTreeMap::new(),
            resources: BTreeMap::new(),
        },
    );
    Ok(id)
}

fn validate_name(name: &str) -> Result<()> {
    if name.is_empty() || name.len() > 255 || name.bytes().any(|byte| byte == 0 || byte == b'/') {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

fn ensure_owned_subtree(state: &RegistryState, owner: DriverId, node: DeviceNodeId) -> Result<()> {
    let record = state.nodes.get(&node).ok_or(Error::NotFound)?;
    if record.info.owner != owner {
        return Err(Error::PermissionDenied);
    }
    for child in record.children.values() {
        ensure_owned_subtree(state, owner, *child)?;
    }
    Ok(())
}

fn revoke_subtree_resources(state: &RegistryState, node: DeviceNodeId) -> Result<()> {
    let mut resources = Vec::new();
    collect_subtree_resources(state, node, &mut resources)?;
    revoke_resources(resources)
}

fn revoke_resources(resources: Vec<Arc<Resource>>) -> Result<()> {
    let mut revoked: Vec<Arc<Resource>> = Vec::new();
    for resource in resources {
        if let Err(error) = resource.try_revoke() {
            for resource in revoked {
                resource.restore();
            }
            return Err(error);
        }
        revoked.push(resource);
    }
    Ok(())
}

fn collect_subtree_resources(
    state: &RegistryState,
    node: DeviceNodeId,
    resources: &mut Vec<Arc<Resource>>,
) -> Result<()> {
    let record = state.nodes.get(&node).ok_or(Error::NotFound)?;
    resources.extend(record.resources.values().cloned());
    for child in record.children.values() {
        collect_subtree_resources(state, *child, resources)?;
    }
    Ok(())
}

fn remove_subtree(state: &mut RegistryState, node: DeviceNodeId) {
    let children: Vec<_> = state
        .nodes
        .get(&node)
        .map(|record| record.children.values().copied().collect())
        .unwrap_or_default();
    for child in children {
        remove_subtree(state, child);
    }

    let parent = state.nodes.get(&node).and_then(|record| record.info.parent);
    if let Some(parent) = parent
        && let Some(parent) = state.nodes.get_mut(&parent.node())
    {
        parent.children.retain(|_, child| *child != node);
    }
    state.nodes.remove(&node);
}

fn registry() -> Result<&'static DeviceRegistry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}
