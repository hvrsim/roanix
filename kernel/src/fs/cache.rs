//! Weak identity index for VM-backed files.
//!
//! A mapping can outlive its vnode. Keeping the identity index weak lets a
//! later open find the mapping's existing object without pinning every file
//! ever opened until unmount. Vnodes, mappings and the namecache own the hot
//! objects; the VM owns page replacement. Pager construction runs unlocked.

use super::Result;
use crate::{mem::VmObject, sys::sync::Mutex};
use alloc::{
    collections::BTreeMap,
    sync::{Arc, Weak},
};

pub(super) type ObjectCache = IdentityCache<VmObject>;

/// Non-owning identity index shared by vnodes and their VM objects.
pub(super) struct IdentityCache<T> {
    state: Mutex<State<T>>,
}

struct State<T> {
    objects: BTreeMap<u64, Weak<T>>,
    until_sweep: usize,
}

impl<T> IdentityCache<T> {
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(State {
                objects: BTreeMap::new(),
                until_sweep: 64,
            }),
        }
    }

    pub fn get_or_create(
        &self,
        key: u64,
        create: impl FnOnce() -> Result<Arc<T>>,
    ) -> Result<Arc<T>> {
        if let Some(object) = self.state.lock().objects.get(&key).and_then(Weak::upgrade) {
            return Ok(object);
        }
        let candidate = create()?;
        let mut state = self.state.lock();
        // Another opener may have published while our pager was constructed.
        if let Some(object) = state.objects.get(&key).and_then(Weak::upgrade) {
            return Ok(object);
        }
        state.until_sweep -= 1;
        if state.until_sweep == 0 {
            state.objects.retain(|_, object| object.strong_count() != 0);
            // Scanning N live identities every 64 insertions makes sustained
            // opens quadratic. Scale the interval with the surviving index,
            // amortizing sweep work while still collecting transient entries.
            state.until_sweep = state.objects.len().max(64);
        }
        state.objects.insert(key, Arc::downgrade(&candidate));
        Ok(candidate)
    }
}
