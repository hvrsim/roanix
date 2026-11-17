//! Retained directory-entry paths for open descriptions.
//!
//! Entries distinguish hard links and retain their ancestors after unlink.
//! The index owns only weak references. Namespace mutations update entries
//! after backend success, while the topology gate excludes open registration.
//! Never release the last strong entry reference while holding INDEX.

use super::{Error, Result, path, vfs::MountId, vnode::VnodeKey};
use crate::sys::sync::{Mutex, Once};
use alloc::{
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(super) struct Key {
    mount: MountId,
    parent: VnodeKey,
    name: Arc<[u8]>,
}
impl Key {
    pub fn new(mount: MountId, parent: VnodeKey, name: &[u8]) -> Self {
        Self {
            mount,
            parent,
            name: Arc::from(name),
        }
    }
}

struct State {
    parent: Option<Arc<Location>>,
    key: Option<Key>,
    deleted: bool,
}
pub(super) struct Location {
    child: Option<VnodeKey>,
    state: Mutex<State>,
}
static INDEX: Once<Mutex<BTreeMap<Key, Weak<Location>>>> = Once::new();
static ROOT: Once<Arc<Location>> = Once::new();
fn index() -> &'static Mutex<BTreeMap<Key, Weak<Location>>> {
    INDEX.call_once(|| Mutex::new(BTreeMap::new()))
}
pub(super) fn root() -> Arc<Location> {
    ROOT.call_once(|| {
        Arc::new(Location {
            child: None,
            state: Mutex::new(State {
                parent: None,
                key: None,
                deleted: false,
            }),
        })
    })
    .clone()
}

pub(super) fn find(key: &Key) -> Option<Arc<Location>> {
    index().lock().get(key).and_then(Weak::upgrade)
}

pub(super) fn retain(parent: Arc<Location>, key: Key, child: VnodeKey) -> Arc<Location> {
    if let Some(existing) = find(&key)
        && existing.child == Some(child)
    {
        return existing;
    }
    // Allocate before taking the index. Retired entries are dropped after it.
    let prepared = Arc::new(Location {
        child: Some(child),
        state: Mutex::new(State {
            parent: Some(parent),
            key: Some(key.clone()),
            deleted: false,
        }),
    });
    let previous;
    {
        let mut entries = index().lock();
        previous = entries.get(&key).and_then(Weak::upgrade);
        if let Some(previous) = &previous {
            if previous.child == Some(child) {
                return previous.clone();
            }
            // Dynamic filesystems can recycle a name without a VFS unlink.
            previous.state.lock().deleted = true;
        }
        entries.insert(key, Arc::downgrade(&prepared));
    }
    prepared
}

impl Location {
    pub fn parent(&self) -> Option<Arc<Self>> {
        self.state.lock().parent.clone()
    }

    // The caller holds the topology gate throughout the path snapshot.
    pub fn path(self: &Arc<Self>) -> Result<Vec<u8>> {
        let mut current = self.clone();
        let mut names = Vec::new();
        let mut bytes = 1usize;
        let mut deleted = false;
        loop {
            let (parent, name) = {
                let state = current.state.lock();
                deleted |= state.deleted;
                (
                    state.parent.clone(),
                    state.key.as_ref().map(|key| key.name.clone()),
                )
            };
            if let Some(name) = name {
                bytes = bytes
                    .checked_add(name.len() + 1)
                    .ok_or(Error::NameTooLong)?;
                if bytes > path::MAX_PATH_LEN {
                    return Err(Error::NameTooLong);
                }
                names.push(name);
            }
            match parent {
                Some(parent) => current = parent,
                None => break,
            }
        }
        let mut output = Vec::new();
        output
            .try_reserve_exact(bytes + if deleted { 10 } else { 0 })
            .map_err(|_| Error::OutOfMemory)?;
        output.push(b'/');
        for name in names.iter().rev() {
            if output.len() > 1 {
                output.push(b'/');
            }
            output.extend_from_slice(name);
        }
        if deleted {
            output.extend_from_slice(b" (deleted)");
        }
        Ok(output)
    }

    pub fn unlink(&self) {
        let mut entries = index().lock();
        let mut state = self.state.lock();
        state.deleted = true;
        if let Some(key) = &state.key {
            entries.remove(key);
        }
    }

    pub fn rename(self: &Arc<Self>, parent: Arc<Self>, key: Key) {
        let next = State {
            parent: Some(parent),
            key: Some(key.clone()),
            deleted: false,
        };
        let previous;
        {
            let mut entries = index().lock();
            let mut state = self.state.lock();
            if let Some(old) = &state.key {
                entries.remove(old);
            }
            previous = core::mem::replace(&mut *state, next);
            entries.insert(key, Arc::downgrade(self));
        }
        drop(previous);
    }
}

impl Drop for Location {
    fn drop(&mut self) {
        let key = self.state.get_mut().key.clone();
        if let Some(key) = key {
            let mut entries = index().lock();
            if entries
                .get(&key)
                .is_some_and(|entry| core::ptr::eq(entry.as_ptr(), self))
            {
                entries.remove(&key);
            }
        }
        // Deep paths must not recursively destroy their entire ancestor chain
        // on the small kernel stack. Shared ancestors stop this walk.
        let mut parent = self.state.get_mut().parent.take();
        while let Some(entry) = parent {
            match Arc::try_unwrap(entry) {
                Ok(mut entry) => parent = entry.state.get_mut().parent.take(),
                Err(_) => break,
            }
        }
    }
}
