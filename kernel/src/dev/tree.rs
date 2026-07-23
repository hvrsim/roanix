//! Hierarchical bus and device registry.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    string::String,
    sync::Arc,
    vec::Vec,
};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::sys::sync::{Mutex, Once};

use super::{
    error::{Error, Result},
    resource::{
        Resource, ResourceFlags, ResourceId, ResourceKey, ResourceLeaseId, ResourceValue,
    },
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
    /// Provider node, or `None` for the root bus.
    pub parent: Option<DeviceNodeId>,
    /// Name within the parent bus.
    pub name: Box<str>,
}

struct NodeRecord {
    info: NodeInfo,
    children: BTreeMap<Box<str>, DeviceNodeId>,
    properties: BTreeMap<ResourceKey, Arc<[u8]>>,
    resources: BTreeMap<ResourceKey, Arc<Resource>>,
}

struct RegistryState {
    nodes: BTreeMap<DeviceNodeId, NodeRecord>,
    leases: BTreeMap<ResourceLeaseId, LeaseRecord>,
}

struct LeaseRecord {
    consumer: DriverId,
    resource: Arc<Resource>,
}

struct DeviceRegistry {
    next_node: AtomicU64,
    next_resource: AtomicU64,
    next_lease: AtomicU64,
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
            properties: BTreeMap::new(),
            resources: BTreeMap::new(),
        };
        let mut nodes = BTreeMap::new();
        nodes.insert(root_id, root);
        Self {
            next_node: AtomicU64::new(ROOT_NODE_ID + 1),
            next_resource: AtomicU64::new(1),
            next_lease: AtomicU64::new(1),
            state: Mutex::new(RegistryState {
                nodes,
                leases: BTreeMap::new(),
            }),
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

    fn allocate_lease_id(&self) -> ResourceLeaseId {
        let id = self.next_lease.fetch_add(1, Ordering::Relaxed);
        assert!(id != 0, "dev: resource lease identifier wrapped");
        ResourceLeaseId::new(id)
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
pub fn register_bus(owner: DriverId, parent: DeviceNodeId, name: &str) -> Result<BusId> {
    let _owner = super::driver::mutation_guard(owner)?;
    let parent_owner = node_info(parent)?.owner;
    let _parent = super::driver::parent_guard(owner, parent_owner)?;
    register_node(owner, parent, name, NodeKind::Bus).map(BusId)
}

/// Registers a device below a provider node.
pub fn register_device(owner: DriverId, parent: DeviceNodeId, name: &str) -> Result<DeviceId> {
    let _owner = super::driver::mutation_guard(owner)?;
    let parent_owner = node_info(parent)?.owner;
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

pub(crate) fn all_nodes() -> Result<Vec<DeviceNodeId>> {
    Ok(registry()?.state.lock().nodes.keys().copied().collect())
}

pub(crate) fn subtree_nodes(node: DeviceNodeId) -> Result<Vec<DeviceNodeId>> {
    let state = registry()?.state.lock();
    let mut nodes = Vec::new();
    collect_subtree_nodes(&state, node, &mut nodes)?;
    Ok(nodes)
}

/// Returns direct child snapshots in lexical name order.
pub fn children(bus: BusId) -> Result<Vec<NodeInfo>> {
    children_of(bus.node())
}

/// Returns direct child snapshots in lexical name order.
pub fn children_of(node: DeviceNodeId) -> Result<Vec<NodeInfo>> {
    let state = registry()?.state.lock();
    let record = state.nodes.get(&node).ok_or(Error::NotFound)?;
    Ok(record
        .children
        .values()
        .filter_map(|id| state.nodes.get(id))
        .map(|child| child.info.clone())
        .collect())
}

/// Publishes immutable metadata on an owned node.
pub fn set_property(
    owner: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
    value: Arc<[u8]>,
) -> Result<()> {
    let _owner = super::driver::mutation_guard(owner)?;
    let mut state = registry()?.state.lock();
    let record = state.nodes.get_mut(&node).ok_or(Error::NotFound)?;
    if record.info.owner != owner {
        return Err(Error::PermissionDenied);
    }
    if record.properties.contains_key(&key) {
        return Err(Error::AlreadyExists);
    }
    record.properties.insert(key, value);
    drop(state);
    super::binding::rescan();
    Ok(())
}

/// Returns immutable metadata attached directly to a node.
pub fn property(node: DeviceNodeId, key: ResourceKey) -> Result<Arc<[u8]>> {
    registry()?
        .state
        .lock()
        .nodes
        .get(&node)
        .ok_or(Error::NotFound)?
        .properties
        .get(&key)
        .cloned()
        .ok_or(Error::NotFound)
}

/// Removes a node and all same-owner descendants.
pub fn remove_node(owner: DriverId, node: DeviceNodeId) -> Result<()> {
    let _owner = super::driver::mutation_guard(owner)?;
    if node.get() == ROOT_NODE_ID {
        return Err(Error::PermissionDenied);
    }
    let owns_removal = if super::binding::is_blocked(node) {
        false
    } else {
        super::binding::begin_remove_subtree(node)?;
        true
    };
    let registry = registry()?;
    let mut state = registry.state.lock();
    let result = (|| {
        ensure_owned_subtree(&state, owner, node)?;
        revoke_subtree_resources(&state, node)?;
        remove_subtree(&mut state, node);
        Ok(())
    })();
    drop(state);
    if owns_removal {
        super::binding::finish_remove_subtree(node);
    }
    result
}

/// Publishes a resource on a node owned by `owner`.
pub fn publish_resource(
    owner: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
    flags: ResourceFlags,
    value: ResourceValue,
) -> Result<ResourceId> {
    publish_resource_inner(owner, node, key, flags, value, false)
}

pub(crate) fn publish_root_resource(
    owner: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
    flags: ResourceFlags,
    value: ResourceValue,
) -> Result<ResourceId> {
    if node != root_bus()?.node() {
        return Err(Error::PermissionDenied);
    }
    publish_resource_inner(owner, node, key, flags, value, true)
}

fn publish_resource_inner(
    owner: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
    flags: ResourceFlags,
    value: ResourceValue,
    allow_foreign_bus: bool,
) -> Result<ResourceId> {
    let _owner = super::driver::mutation_guard(owner)?;
    let registry = registry()?;
    let id = registry.allocate_resource_id();
    let resource = Arc::new(Resource::new(id, owner, key, flags, value));
    let mut state = registry.state.lock();
    let record = state.nodes.get_mut(&node).ok_or(Error::NotFound)?;
    if record.info.owner != owner && !allow_foreign_bus {
        return Err(Error::PermissionDenied);
    }
    if record.resources.contains_key(&key) {
        return Err(Error::AlreadyExists);
    }
    record.resources.insert(key, resource);
    drop(state);
    super::binding::rescan();
    Ok(id)
}

/// Removes a resource from an owned node.
pub fn remove_resource(owner: DriverId, node: DeviceNodeId, key: ResourceKey) -> Result<()> {
    remove_resource_inner(owner, node, key, false, false)
}

pub(crate) fn remove_root_resource(
    owner: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
) -> Result<()> {
    if node != root_bus()?.node() {
        return Err(Error::PermissionDenied);
    }
    remove_resource_inner(owner, node, key, true, true)
}

fn remove_resource_inner(
    owner: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
    allow_foreign_bus: bool,
    cleanup: bool,
) -> Result<()> {
    let _owner = if cleanup {
        super::driver::close_callback_guard(owner)?
    } else {
        super::driver::mutation_guard(owner)?
    };
    let mut state = registry()?.state.lock();
    let resource = {
        let record = state.nodes.get(&node).ok_or(Error::NotFound)?;
        if record.info.owner != owner && !allow_foreign_bus {
            return Err(Error::PermissionDenied);
        }
        record
            .resources
            .get(&key)
            .cloned()
            .ok_or(Error::NotFound)?
    };
    if state
        .leases
        .values()
        .any(|lease| lease.resource.id() == resource.id())
    {
        return Err(Error::Busy);
    }
    resource.try_revoke()?;
    state
        .nodes
        .get_mut(&node)
        .expect("dev: resource node disappeared while locked")
        .resources
        .remove(&key);
    Ok(())
}

/// Acquires the nearest inherited resource and pins its provider.
pub fn acquire_resource(
    consumer: DriverId,
    node: DeviceNodeId,
    key: ResourceKey,
) -> Result<(ResourceLeaseId, Arc<Resource>)> {
    let _consumer = super::driver::mutation_guard(consumer)?;
    if super::binding::provider_blocked(node) {
        return Err(Error::Busy);
    }
    let registry = registry()?;
    let id = registry.allocate_lease_id();
    let mut state = registry.state.lock();
    let resource = resolve_resource_locked(&state, node, key)?;
    resource.ensure_live()?;
    let provider = resource.owner();
    let _provider = if provider == consumer {
        None
    } else {
        Some(super::driver::callback_guard(provider)?)
    };
    super::dependency::add(consumer, provider)?;
    state.leases.insert(
        id,
        LeaseRecord {
            consumer,
            resource: resource.clone(),
        },
    );
    Ok((id, resource))
}

/// Releases an acquired resource lease.
pub fn release_resource(consumer: DriverId, lease: ResourceLeaseId) -> Result<()> {
    release_resource_inner(consumer, lease, false)
}

pub(crate) fn release_resource_cleanup(
    consumer: DriverId,
    lease: ResourceLeaseId,
) -> Result<()> {
    release_resource_inner(consumer, lease, true)
}

fn release_resource_inner(
    consumer: DriverId,
    lease: ResourceLeaseId,
    cleanup: bool,
) -> Result<()> {
    let _consumer = if cleanup {
        super::driver::cleanup_guard(consumer)?
    } else {
        super::driver::mutation_guard(consumer)?
    };
    let mut state = registry()?.state.lock();
    let record = state.leases.get(&lease).ok_or(Error::NotFound)?;
    if record.consumer != consumer {
        return Err(Error::PermissionDenied);
    }
    let provider = record.resource.owner();
    state.leases.remove(&lease);
    super::dependency::remove(consumer, provider);
    Ok(())
}

/// Returns the resource pinned by an owned lease.
pub fn leased_resource(
    consumer: DriverId,
    lease: ResourceLeaseId,
) -> Result<Arc<Resource>> {
    let _consumer = super::driver::mutation_guard(consumer)?;
    let state = registry()?.state.lock();
    let record = state.leases.get(&lease).ok_or(Error::NotFound)?;
    if record.consumer != consumer {
        return Err(Error::PermissionDenied);
    }
    Ok(record.resource.clone())
}

/// Resolves the nearest matching resource through the provider chain.
pub fn resolve_resource(node: DeviceNodeId, key: ResourceKey) -> Result<Arc<Resource>> {
    let state = registry()?.state.lock();
    resolve_resource_locked(&state, node, key)
}

fn resolve_resource_locked(
    state: &RegistryState,
    node: DeviceNodeId,
    key: ResourceKey,
) -> Result<Arc<Resource>> {
    let mut current = Some(node);
    while let Some(node) = current {
        let record = state.nodes.get(&node).ok_or(Error::NotFound)?;
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
    if state.leases.values().any(|lease| {
        lease.resource.owner() == owner && lease.consumer != owner
    }) {
        return Err(Error::Busy);
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
    if state.leases.values().any(|lease| {
        lease.resource.owner() == owner && lease.consumer != owner
    }) {
        return Err(Error::Busy);
    }

    let dependencies: Vec<_> = state
        .leases
        .values()
        .filter(|lease| lease.consumer == owner)
        .map(|lease| (lease.consumer, lease.resource.owner()))
        .collect();
    state.leases.retain(|_, lease| lease.consumer != owner);
    for (consumer, provider) in dependencies {
        super::dependency::remove(consumer, provider);
    }

    let roots: Vec<_> = state
        .nodes
        .values()
        .filter(|record| {
            record.info.owner == owner
                && record.info.parent.is_some_and(|parent| {
                    state
                        .nodes
                        .get(&parent)
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
    parent: DeviceNodeId,
    name: &str,
    kind: NodeKind,
) -> Result<DeviceNodeId> {
    validate_name(name)?;
    let registry = registry()?;
    let id = registry.allocate_node_id();
    let mut state = registry.state.lock();
    let parent_record = state.nodes.get_mut(&parent).ok_or(Error::NotFound)?;
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
            properties: BTreeMap::new(),
            resources: BTreeMap::new(),
        },
    );
    drop(state);
    super::binding::rescan();
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
    if state.leases.values().any(|lease| {
        resources
            .iter()
            .any(|resource| resource.id() == lease.resource.id())
    }) {
        return Err(Error::Busy);
    }
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

fn collect_subtree_nodes(
    state: &RegistryState,
    node: DeviceNodeId,
    nodes: &mut Vec<DeviceNodeId>,
) -> Result<()> {
    let record = state.nodes.get(&node).ok_or(Error::NotFound)?;
    nodes.push(node);
    for child in record.children.values() {
        collect_subtree_nodes(state, *child, nodes)?;
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
        && let Some(parent) = state.nodes.get_mut(&parent)
    {
        parent.children.retain(|_, child| *child != node);
    }
    state.nodes.remove(&node);
}

fn registry() -> Result<&'static DeviceRegistry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}
