//! Process membership in the single native cgroup hierarchy.
//!
//! Fork, migration, exit, and subtree kill serialize on one tree lock. No
//! process lock, signal delivery, or event wake runs while that lock is held.
//! Removed groups survive only through open handles and zombie memberships.

use alloc::{
    collections::BTreeMap,
    format,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use super::Process;
use crate::{
    fs::{Error, Result},
    sys::{
        event::Event,
        sync::{Mutex, Once},
    },
};

const ROOT: u64 = 1;
const MAX_GROUPS: usize = 4096;
const MAX_DEPTH: usize = 64;
static TREE: Once<Mutex<Tree>> = Once::new();

/// Stable identity retained by filesystem nodes and zombie processes.
pub(crate) struct Group {
    pub(crate) id: u64,
    pub(crate) path: Vec<u8>,
    pub(crate) event: Arc<Event>,
    pub(crate) generation: AtomicU64,
    removed: AtomicBool,
    depth: usize,
}

struct GroupState {
    group: Arc<Group>,
    parent: Option<u64>,
    children: BTreeMap<Vec<u8>, u64>,
    members: BTreeMap<usize, Weak<Process>>,
    population: usize,
    mode: u16,
}

struct Membership {
    group: Arc<Group>,
    live: bool,
    // An in-flight fork must inherit a subtree kill even if signal delivery
    // has not reached the parent yet. Killed processes cannot migrate away.
    killed: bool,
}

struct Tree {
    next: u64,
    groups: BTreeMap<u64, GroupState>,
    processes: BTreeMap<usize, Membership>,
}

impl Group {
    fn new(id: u64, path: Vec<u8>, depth: usize) -> Arc<Self> {
        Arc::new(Self {
            id,
            path,
            depth,
            event: Arc::new(Event::new()),
            generation: AtomicU64::new(0),
            removed: AtomicBool::new(false),
        })
    }

    pub(crate) fn is_root(&self) -> bool {
        self.id == ROOT
    }
    pub(crate) fn ensure_exists(&self) -> Result<()> {
        if self.removed.load(Ordering::Acquire) {
            Err(Error::NotFound)
        } else {
            Ok(())
        }
    }
}

impl GroupState {
    fn new(group: Arc<Group>, parent: Option<u64>, mode: u16) -> Self {
        Self {
            group,
            parent,
            children: BTreeMap::new(),
            members: BTreeMap::new(),
            population: 0,
            mode,
        }
    }
}

fn tree() -> &'static Mutex<Tree> {
    TREE.call_once(|| {
        let mut groups = BTreeMap::new();
        groups.insert(
            ROOT,
            GroupState::new(Group::new(ROOT, b"/".to_vec(), 0), None, 0o755),
        );
        Mutex::new(Tree {
            next: ROOT + 1,
            groups,
            processes: BTreeMap::new(),
        })
    })
}

impl Tree {
    fn ancestors(&self, mut id: u64) -> Vec<u64> {
        let mut result = Vec::new();
        loop {
            result.push(id);
            let Some(parent) = self.groups[&id].parent else {
                return result;
            };
            id = parent;
        }
    }

    /// Apply both sides of a migration before publishing population changes.
    fn population_change(&mut self, old: Option<u64>, new: Option<u64>) -> Vec<Arc<Group>> {
        let mut before = BTreeMap::new();
        for (id, add) in [(old, false), (new, true)] {
            let Some(id) = id else {
                continue;
            };
            for ancestor in self.ancestors(id) {
                let state = self
                    .groups
                    .get_mut(&ancestor)
                    .expect("cgroup: lost ancestor");
                before.entry(ancestor).or_insert(state.population != 0);
                if add {
                    state.population += 1;
                } else {
                    state.population -= 1;
                }
            }
        }
        before
            .into_iter()
            .filter_map(|(id, populated)| {
                let state = &self.groups[&id];
                if populated == (state.population != 0) {
                    return None;
                }
                state.group.generation.fetch_add(1, Ordering::Release);
                Some(state.group.clone())
            })
            .collect()
    }
}

fn notify(groups: Vec<Arc<Group>>) {
    for group in groups {
        group.event.wake_all();
        crate::fs::cgroupfs::notify_population(group.id);
    }
}

/// Registers a process before it is published or scheduled.
pub(crate) fn register(process: &Arc<Process>, parent: Option<usize>) -> Result<()> {
    let (changed, killed) = {
        let mut tree = tree().lock();
        let (id, killed) = if let Some(parent) = parent {
            let member = tree
                .processes
                .get(&parent)
                .filter(|member| member.live)
                .ok_or(Error::NotFound)?;
            (member.group.id, member.killed)
        } else {
            (ROOT, false)
        };
        let group = tree.groups[&id].group.clone();
        tree.groups
            .get_mut(&id)
            .unwrap()
            .members
            .insert(process.pid(), Arc::downgrade(process));
        tree.processes.insert(
            process.pid(),
            Membership {
                group,
                live: true,
                killed,
            },
        );
        (tree.population_change(None, Some(id)), killed)
    };
    notify(changed);
    if killed {
        super::signal::send_kernel(process, 9);
    }
    Ok(())
}

/// Excludes zombies from membership listings and populated state before waiters wake.
pub(crate) fn exited(pid: usize) {
    let changed = {
        let mut tree = tree().lock();
        let Some(member) = tree.processes.get_mut(&pid) else {
            return;
        };
        if !member.live {
            return;
        }
        member.live = false;
        let id = member.group.id;
        tree.groups.get_mut(&id).unwrap().members.remove(&pid);
        tree.population_change(Some(id), None)
    };
    notify(changed);
}

pub(crate) fn reaped(pid: usize) {
    tree().lock().processes.remove(&pid);
}

pub(crate) fn process_path(pid: usize) -> Result<Vec<u8>> {
    let tree = tree().lock();
    let member = tree.processes.get(&pid).ok_or(Error::NotFound)?;
    let mut bytes = b"0::".to_vec();
    bytes.extend_from_slice(&member.group.path);
    if member.group.removed.load(Ordering::Acquire) {
        bytes.extend_from_slice(b" (deleted)");
    }
    bytes.push(b'\n');
    Ok(bytes)
}

pub(crate) fn root() -> Arc<Group> {
    tree().lock().groups[&ROOT].group.clone()
}

pub(crate) fn parent(id: u64) -> Result<Arc<Group>> {
    let tree = tree().lock();
    let parent = tree
        .groups
        .get(&id)
        .ok_or(Error::NotFound)?
        .parent
        .unwrap_or(ROOT);
    Ok(tree.groups[&parent].group.clone())
}

pub(crate) fn children(id: u64) -> Result<Vec<(Vec<u8>, Arc<Group>)>> {
    let tree = tree().lock();
    let state = tree.groups.get(&id).ok_or(Error::NotFound)?;
    Ok(state
        .children
        .iter()
        .map(|(name, id)| (name.clone(), tree.groups[id].group.clone()))
        .collect())
}

pub(crate) fn lookup(id: u64, name: &[u8]) -> Result<Arc<Group>> {
    let tree = tree().lock();
    let state = tree.groups.get(&id).ok_or(Error::NotFound)?;
    let id = state.children.get(name).ok_or(Error::NotFound)?;
    Ok(tree.groups[id].group.clone())
}

pub(crate) fn create(parent: u64, name: &[u8], mode: u16) -> Result<Arc<Group>> {
    let mut tree = tree().lock();
    let state = tree.groups.get(&parent).ok_or(Error::NotFound)?;
    if state.children.contains_key(name) {
        return Err(Error::AlreadyExists);
    }
    if state.group.depth >= MAX_DEPTH || tree.groups.len() >= MAX_GROUPS {
        return Err(Error::NoSpace);
    }
    let mut path = state.group.path.clone();
    if parent != ROOT {
        path.push(b'/');
    }
    path.extend_from_slice(name);
    if path.len() > crate::fs::path::MAX_PATH_LEN {
        return Err(Error::NameTooLong);
    }
    let group = Group::new(tree.next, path, state.group.depth + 1);
    tree.next = tree
        .next
        .checked_add(1)
        .filter(|id| *id < (u64::MAX >> 4))
        .ok_or(Error::NoSpace)?;
    tree.groups
        .get_mut(&parent)
        .unwrap()
        .children
        .insert(name.to_vec(), group.id);
    tree.groups
        .insert(group.id, GroupState::new(group.clone(), Some(parent), mode));
    Ok(group)
}

pub(crate) fn remove(parent: u64, name: &[u8]) -> Result<()> {
    let removed = {
        let mut tree = tree().lock();
        let id = *tree
            .groups
            .get(&parent)
            .ok_or(Error::NotFound)?
            .children
            .get(name)
            .ok_or(Error::NotFound)?;
        let state = &tree.groups[&id];
        if !state.children.is_empty() {
            return Err(Error::NotEmpty);
        }
        if state.population != 0 {
            return Err(Error::Busy);
        }
        let state = tree.groups.remove(&id).unwrap();
        tree.groups.get_mut(&parent).unwrap().children.remove(name);
        state.group.removed.store(true, Ordering::Release);
        state.group.generation.fetch_add(1, Ordering::Release);
        state.group
    };
    notify(alloc::vec![removed]);
    Ok(())
}

pub(crate) fn mode(id: u64) -> Result<u16> {
    tree()
        .lock()
        .groups
        .get(&id)
        .map(|state| state.mode)
        .ok_or(Error::NotFound)
}
pub(crate) fn set_mode(id: u64, mode: u16) -> Result<()> {
    tree()
        .lock()
        .groups
        .get_mut(&id)
        .ok_or(Error::NotFound)?
        .mode = mode;
    Ok(())
}

pub(crate) fn population(id: u64) -> Result<bool> {
    tree()
        .lock()
        .groups
        .get(&id)
        .map(|state| state.population != 0)
        .ok_or(Error::NotFound)
}

pub(crate) fn members(id: u64) -> Result<Vec<u8>> {
    let tree = tree().lock();
    let state = tree.groups.get(&id).ok_or(Error::NotFound)?;
    let mut bytes = Vec::new();
    for pid in state.members.keys() {
        bytes.extend_from_slice(format!("{pid}\n").as_bytes());
    }
    Ok(bytes)
}

pub(crate) fn migrate(pid: usize, destination: u64) -> Result<()> {
    let changed = {
        let mut tree = tree().lock();
        if !tree.groups.contains_key(&destination) {
            return Err(Error::NotFound);
        }
        let member = tree.processes.get_mut(&pid).ok_or(Error::NotFound)?;
        if !member.live {
            return Err(Error::NotFound);
        }
        if member.killed {
            return Err(Error::Busy);
        }
        let old = member.group.id;
        if old == destination {
            return Ok(());
        }
        let process = tree
            .groups
            .get_mut(&old)
            .unwrap()
            .members
            .remove(&pid)
            .unwrap();
        let target = tree.groups.get_mut(&destination).unwrap();
        target.members.insert(pid, process);
        let group = target.group.clone();
        tree.processes.get_mut(&pid).unwrap().group = group;
        tree.population_change(Some(old), Some(destination))
    };
    notify(changed);
    Ok(())
}

/// Atomically selects a subtree's victims; fork inherits the killed marker.
/// Signal delivery runs unlocked because it can wake and schedule a victim.
pub(crate) fn kill(id: u64) -> Result<()> {
    if id == ROOT {
        return Err(Error::PermissionDenied);
    }
    let victims = {
        let mut tree = tree().lock();
        if !tree.groups.contains_key(&id) {
            return Err(Error::NotFound);
        }
        let ids: Vec<u64> = tree
            .groups
            .keys()
            .copied()
            .filter(|child| tree.ancestors(*child).contains(&id))
            .collect();
        let mut victims = Vec::new();
        for group in ids {
            let members: Vec<_> = tree.groups[&group]
                .members
                .iter()
                .map(|(pid, process)| (*pid, process.clone()))
                .collect();
            for (pid, process) in members {
                tree.processes.get_mut(&pid).unwrap().killed = true;
                if let Some(process) = process.upgrade() {
                    victims.push(process);
                }
            }
        }
        victims
    };
    for process in victims {
        super::signal::send_kernel(&process, 9);
    }
    Ok(())
}
