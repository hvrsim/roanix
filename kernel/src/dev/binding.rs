//! Declarative provider-node driver binding.

use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
    vec::Vec,
};
use core::sync::atomic::{AtomicU64, Ordering};

use log::error;

use crate::sys::sync::{Mutex, Once};

use super::{
    DeviceNodeId, DriverId, Error, ResourceKey, Result,
    abi::{DriverBindFn, DriverUnbindFn, STATUS_DEFERRED, STATUS_OK},
    driver::{self, CallbackOwner},
    tree,
};

/// Stable identifier for one registered driver class.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DriverClassId(u64);

impl DriverClassId {
    pub(crate) const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

pub(crate) struct MatchProperty {
    pub key: ResourceKey,
    pub value: Arc<[u8]>,
}

pub(crate) struct ClassRegistration {
    pub name: Box<str>,
    pub priority: i32,
    pub node_kind: u32,
    pub context: usize,
    pub properties: Box<[MatchProperty]>,
    pub resources: Box<[ResourceKey]>,
    pub bind: DriverBindFn,
    pub unbind: DriverUnbindFn,
}

struct ClassRecord {
    id: DriverClassId,
    owner: DriverId,
    name: Box<str>,
    priority: i32,
    node_kind: u32,
    context: usize,
    properties: Box<[MatchProperty]>,
    resources: Box<[ResourceKey]>,
    bind: DriverBindFn,
    unbind: DriverUnbindFn,
    callback_owner: CallbackOwner,
}

struct InstanceRecord {
    class: Arc<ClassRecord>,
    provider: DriverId,
    context: usize,
}

struct BindingState {
    classes: BTreeMap<DriverClassId, Arc<ClassRecord>>,
    instances: BTreeMap<DeviceNodeId, InstanceRecord>,
    pending: BTreeMap<DeviceNodeId, DriverClassId>,
    blocked: BTreeSet<DeviceNodeId>,
    removals: BTreeMap<DeviceNodeId, Box<[DeviceNodeId]>>,
}

struct BindingRegistry {
    next_class: AtomicU64,
    state: Mutex<BindingState>,
}

static REGISTRY: Once<BindingRegistry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| BindingRegistry {
        next_class: AtomicU64::new(1),
        state: Mutex::new(BindingState {
            classes: BTreeMap::new(),
            instances: BTreeMap::new(),
            pending: BTreeMap::new(),
            blocked: BTreeSet::new(),
            removals: BTreeMap::new(),
        }),
    });
}

pub(crate) fn register_class(
    owner: DriverId,
    registration: ClassRegistration,
) -> Result<DriverClassId> {
    let _owner = driver::mutation_guard(owner)?;
    let callback_owner = driver::callback_owner(owner)?;
    let active = callback_owner.is_loaded();
    let registry = registry()?;
    let raw_id = registry
        .next_class
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |current| {
            current.checked_add(1).filter(|next| *next != 0)
        })
        .map_err(|_| Error::NoSpace)?;
    let id = DriverClassId::new(raw_id);
    registry.state.lock().classes.insert(
        id,
        Arc::new(ClassRecord {
            id,
            owner,
            name: registration.name,
            priority: registration.priority,
            node_kind: registration.node_kind,
            context: registration.context,
            properties: registration.properties,
            resources: registration.resources,
            bind: registration.bind,
            unbind: registration.unbind,
            callback_owner,
        }),
    );
    if active {
        rescan();
    }
    Ok(id)
}

pub(crate) fn unregister_class(owner: DriverId, id: DriverClassId) -> Result<()> {
    let _owner = driver::mutation_guard(owner)?;
    let registry = registry()?;
    let mut state = registry.state.lock();
    let class = state.classes.get(&id).ok_or(Error::NotFound)?;
    if class.owner != owner {
        return Err(Error::PermissionDenied);
    }
    if state
        .instances
        .values()
        .any(|instance| instance.class.id == id)
        || state.pending.values().any(|class| *class == id)
    {
        return Err(Error::Busy);
    }
    state.classes.remove(&id);
    Ok(())
}

pub(crate) fn activate_driver(_owner: DriverId) {
    let Some(_) = REGISTRY.get() else {
        return;
    };
    rescan();
}

pub(crate) fn rescan() {
    let Some(_) = REGISTRY.get() else {
        return;
    };
    let Ok(nodes) = tree::all_nodes() else {
        return;
    };
    for node in nodes {
        try_bind_node(node);
    }
}

pub(crate) fn has_foreign_instances(owner: DriverId) -> bool {
    let Some(registry) = REGISTRY.get() else {
        return false;
    };
    let instances: Vec<_> = registry
        .state
        .lock()
        .instances
        .iter()
        .map(|(_, instance)| (instance.provider, instance.class.owner))
        .collect();
    instances
        .into_iter()
        .any(|(provider, consumer)| consumer != owner && provider == owner)
}

pub(crate) fn remove_driver(owner: DriverId) {
    let Some(registry) = REGISTRY.get() else {
        return;
    };
    let nodes: Vec<_> = registry
        .state
        .lock()
        .instances
        .iter()
        .filter_map(|(node, instance)| (instance.class.owner == owner).then_some(*node))
        .collect();
    for node in nodes {
        unbind_node(node);
    }
    registry
        .state
        .lock()
        .classes
        .retain(|_, class| class.owner != owner);
    rescan();
}

pub(crate) fn begin_remove_subtree(root: DeviceNodeId) -> Result<()> {
    let Some(registry) = REGISTRY.get() else {
        return Ok(());
    };
    let mut nodes = tree::subtree_nodes(root)?;
    {
        let mut state = registry.state.lock();
        if nodes
            .iter()
            .any(|node| state.pending.contains_key(node) || state.blocked.contains(node))
        {
            return Err(Error::Busy);
        }
        state.blocked.extend(nodes.iter().copied());
        state
            .removals
            .insert(root, nodes.clone().into_boxed_slice());
    }
    nodes.reverse();
    for node in nodes {
        unbind_node(node);
    }
    Ok(())
}

pub(crate) fn finish_remove_subtree(root: DeviceNodeId) {
    let Some(registry) = REGISTRY.get() else {
        return;
    };
    let mut state = registry.state.lock();
    if let Some(nodes) = state.removals.remove(&root) {
        for node in nodes.iter().copied() {
            state.blocked.remove(&node);
        }
    }
}

pub(crate) fn is_blocked(node: DeviceNodeId) -> bool {
    REGISTRY
        .get()
        .is_some_and(|registry| registry.state.lock().blocked.contains(&node))
}

pub(crate) fn provider_blocked(mut node: DeviceNodeId) -> bool {
    let Some(registry) = REGISTRY.get() else {
        return false;
    };
    let blocked = registry.state.lock().blocked.clone();
    loop {
        if blocked.contains(&node) {
            return true;
        }
        let Ok(info) = tree::node_info(node) else {
            return true;
        };
        let Some(parent) = info.parent else {
            return false;
        };
        node = parent;
    }
}

fn try_bind_node(node: DeviceNodeId) {
    if provider_blocked(node) {
        return;
    }
    let Ok(info) = tree::node_info(node) else {
        return;
    };
    let Ok(_provider) = driver::callback_guard(info.owner) else {
        return;
    };
    let Some(registry) = REGISTRY.get() else {
        return;
    };
    let mut classes: Vec<_> = {
        let state = registry.state.lock();
        if state.instances.contains_key(&node)
            || state.pending.contains_key(&node)
            || state.blocked.contains(&node)
        {
            return;
        }
        state
            .classes
            .values()
            .filter(|class| class.callback_owner.is_loaded())
            .cloned()
            .collect()
    };
    classes.sort_unstable_by(|left, right| {
        right
            .priority
            .cmp(&left.priority)
            .then_with(|| left.id.cmp(&right.id))
    });

    for class in classes {
        if !class_matches(&class, node, info.kind as u32) {
            continue;
        }
        {
            let mut state = registry.state.lock();
            if state.instances.contains_key(&node)
                || state.blocked.contains(&node)
                || !state.classes.contains_key(&class.id)
                || state.pending.insert(node, class.id).is_some()
            {
                return;
            }
        }
        if let Err(error) = super::dependency::add(class.owner, info.owner) {
            registry.state.lock().pending.remove(&node);
            error!(
                "dev/binding: class {} cannot bind node {}: {error}",
                class.name,
                node.get()
            );
            continue;
        }

        let Ok(_callback) = class.callback_owner.acquire_control() else {
            registry.state.lock().pending.remove(&node);
            super::dependency::remove(class.owner, info.owner);
            return;
        };
        let mut instance_context = 0usize;
        // SAFETY: class registration validated this callback and the provider
        // identifier and output remain live for the call.
        let status =
            unsafe { (class.bind)(class.context, node.get(), &mut instance_context) };

        let mut state = registry.state.lock();
        state.pending.remove(&node);
        if status == STATUS_OK {
            if tree::node_info(node).is_err() {
                drop(state);
                call_unbind(&class, node, instance_context);
                super::dependency::remove(class.owner, info.owner);
                return;
            }
            state.instances.insert(
                node,
                InstanceRecord {
                    class,
                    provider: info.owner,
                    context: instance_context,
                },
            );
            return;
        }
        super::dependency::remove(class.owner, info.owner);
        if status == STATUS_DEFERRED {
            return;
        }
        error!(
            "dev/binding: class {} failed to bind node {} with status {}",
            class.name,
            node.get(),
            status
        );
    }
}

fn class_matches(class: &ClassRecord, node: DeviceNodeId, node_kind: u32) -> bool {
    if class.node_kind != 0 && class.node_kind != node_kind {
        return false;
    }
    if class.properties.iter().any(|property| {
        match tree::property(node, property.key) {
            Ok(value) => value.as_ref() != property.value.as_ref(),
            Err(_) => true,
        }
    }) {
        return false;
    }
    !class
        .resources
        .iter()
        .any(|key| {
            tree::resolve_resource(node, *key)
                .and_then(|resource| resource.ensure_live())
                .is_err()
        })
}

fn unbind_node(node: DeviceNodeId) {
    let Some(registry) = REGISTRY.get() else {
        return;
    };
    let instance = {
        let mut state = registry.state.lock();
        let Some(instance) = state.instances.remove(&node) else {
            return;
        };
        state.pending.insert(node, instance.class.id);
        instance
    };
    call_unbind(&instance.class, node, instance.context);
    super::dependency::remove(instance.class.owner, instance.provider);
    registry.state.lock().pending.remove(&node);
}

fn call_unbind(class: &ClassRecord, node: DeviceNodeId, context: usize) {
    let _provider = tree::node_info(node)
        .ok()
        .and_then(|info| driver::callback_guard(info.owner).ok());
    let Ok(_callback) = class.callback_owner.acquire_cleanup() else {
        return;
    };
    // SAFETY: class registration validated this callback and the instance
    // context was produced by the matching bind callback.
    unsafe { (class.unbind)(class.context, node.get(), context) };
}

fn registry() -> Result<&'static BindingRegistry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}
