//! Counted driver dependency graph shared by leases and bound instances.

use alloc::collections::{BTreeMap, BTreeSet};

use crate::sys::sync::{Mutex, Once};

use super::{DriverId, Error, Result};

struct DependencyGraph {
    edges: Mutex<BTreeMap<(DriverId, DriverId), u64>>,
}

static GRAPH: Once<DependencyGraph> = Once::new();

pub(crate) fn init() {
    GRAPH.call_once(|| DependencyGraph {
        edges: Mutex::new(BTreeMap::new()),
    });
}

pub(crate) fn add(consumer: DriverId, provider: DriverId) -> Result<()> {
    if consumer == provider {
        return Ok(());
    }
    let graph = graph()?;
    let mut edges = graph.edges.lock();
    if let Some(count) = edges.get_mut(&(consumer, provider)) {
        *count = count.checked_add(1).ok_or(Error::NoSpace)?;
        return Ok(());
    }
    if reaches(&edges, provider, consumer, &mut BTreeSet::new()) {
        return Err(Error::DependencyCycle);
    }
    edges.insert((consumer, provider), 1);
    Ok(())
}

pub(crate) fn remove(consumer: DriverId, provider: DriverId) {
    if consumer == provider {
        return;
    }
    let Some(graph) = GRAPH.get() else {
        return;
    };
    let mut edges = graph.edges.lock();
    match edges.get(&(consumer, provider)).copied() {
        Some(1) => {
            edges.remove(&(consumer, provider));
        }
        Some(count) => {
            edges.insert((consumer, provider), count - 1);
        }
        None => {}
    }
}

fn reaches(
    edges: &BTreeMap<(DriverId, DriverId), u64>,
    current: DriverId,
    target: DriverId,
    visited: &mut BTreeSet<DriverId>,
) -> bool {
    if current == target {
        return true;
    }
    if !visited.insert(current) {
        return false;
    }
    edges
        .keys()
        .any(|edge| edge.0 == current && reaches(edges, edge.1, target, visited))
}

fn graph() -> Result<&'static DependencyGraph> {
    GRAPH.get().ok_or(Error::NotInitialized)
}
