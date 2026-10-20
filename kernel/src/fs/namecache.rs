//! Bounded component-name cache shared by every filesystem.
//!
//! Entries describe `(directory, name)`, including authoritative ENOENT results.
//! They never describe mount crossings or `..`: those belong to path traversal.
//! A directory sequence gates publication around namespace mutations, so a slow
//! lookup cannot resurrect a name after unlink or rename. Bucket locks protect
//! references only; filesystem callbacks and vnode destruction run unlocked.

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use super::{
    Error, Result,
    vnode::{FilesystemId, Vnode, VnodeKey},
};
use crate::sys::sync::Mutex;

const BUCKETS: usize = 256;
const WAYS: usize = 8;
const WRITER: u64 = 1 << 16;

/// Directory sequence with the active writer count in the low 16 bits.
/// Counting writers, rather than toggling a bit, permits overlapping mutations.
pub(super) struct Sequence(AtomicU64);

impl Sequence {
    pub const fn new() -> Self {
        Self(AtomicU64::new(0))
    }

    fn snapshot(&self) -> Option<u64> {
        let state = self.0.load(Ordering::Acquire);
        (state as u16 == 0).then_some(state)
    }

    pub fn write(&self, parent: VnodeKey, name: &[u8]) -> Mutation<'_> {
        self.0.fetch_add(WRITER + 1, Ordering::AcqRel);
        // A writer first closes the publication gate, then removes the exact
        // name. Unrelated siblings remain hot after the mutation completes.
        let retired = {
            let mut state = CACHE[bucket(parent, name)].lock();
            state
                .entries
                .iter_mut()
                .find(|entry| {
                    entry
                        .as_ref()
                        .is_some_and(|entry| entry.parent == parent && entry.name == name)
                })
                .and_then(Option::take)
        };
        drop(retired);
        Mutation(self)
    }
}

/// Invalidates both earlier hits and misses, including failed mutations.
/// Backends changing directories outside Vnode methods must hold this guard.
pub(crate) struct Mutation<'a>(&'a Sequence);

impl Drop for Mutation<'_> {
    fn drop(&mut self) {
        self.0.0.fetch_add(WRITER - 1, Ordering::Release);
    }
}

struct Entry {
    parent: VnodeKey,
    name: Vec<u8>,
    target: Option<Vnode>,
    used: u64,
}

struct Bucket {
    entries: [Option<Entry>; WAYS],
    clock: u64,
}

impl Bucket {
    const fn new() -> Self {
        Self {
            entries: [const { None }; WAYS],
            clock: 0,
        }
    }
}

static CACHE: [Mutex<Bucket>; BUCKETS] = [const { Mutex::new(Bucket::new()) }; BUCKETS];
static RECLAIM_HAND: AtomicUsize = AtomicUsize::new(0);

fn bucket(parent: VnodeKey, name: &[u8]) -> usize {
    // FNV-1a with both stable identity words mixed before the component bytes.
    let mut hash = 0xcbf29ce484222325u64;
    for byte in parent
        .filesystem
        .get()
        .to_le_bytes()
        .iter()
        .chain(parent.node.get().to_le_bytes().iter())
        .chain(name.iter())
    {
        hash = (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3);
    }
    // Fold high bits so aligned inode numbers do not cluster in a few buckets.
    (hash ^ (hash >> 32)) as usize & (BUCKETS - 1)
}

pub(super) fn lookup(
    directory: &Vnode,
    name: &[u8],
    sequence: &Sequence,
    load: impl FnOnce() -> Result<Vnode>,
) -> Result<Vnode> {
    let Some(stamp) = sequence.snapshot() else {
        return load();
    };
    let key = directory.key();
    let cache = &CACHE[bucket(key, name)];
    {
        let mut state = cache.lock();
        state.clock = state.clock.wrapping_add(1);
        let clock = state.clock;
        if let Some(entry) = state
            .entries
            .iter_mut()
            .flatten()
            .find(|entry| entry.parent == key && entry.name == name)
        {
            // Recheck after acquiring the bucket: a writer may have started
            // between the snapshot and this hit. The returned Arc pins identity.
            if sequence.snapshot() == Some(stamp) {
                entry.used = clock;
                return entry.target.clone().ok_or(Error::NotFound);
            }
        }
    }
    let result = load();
    if !matches!(&result, Ok(_) | Err(Error::NotFound)) || sequence.snapshot() != Some(stamp) {
        return result;
    }
    let mut owned = Vec::new();
    // Caching is optional: allocation failure must not change lookup results.
    if owned.try_reserve_exact(name.len()).is_err() {
        return result;
    }
    owned.extend_from_slice(name);
    let retired = {
        let mut state = cache.lock();
        if sequence.snapshot() != Some(stamp) {
            return result;
        }
        let slot = state
            .entries
            .iter()
            .position(|entry| {
                entry
                    .as_ref()
                    .is_some_and(|entry| entry.parent == key && entry.name == name)
            })
            .or_else(|| state.entries.iter().position(Option::is_none))
            .unwrap_or_else(|| {
                state
                    .entries
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, entry)| entry.as_ref().unwrap().used)
                    .unwrap()
                    .0
            });
        let used = state.clock;
        state.entries[slot].replace(Entry {
            parent: key,
            name: owned,
            target: result.as_ref().ok().cloned(),
            used,
        })
    };
    drop(retired);
    result
}

/// Releases a bounded set of name references under VM pressure. Try-locking
/// avoids waiting behind pathname users. Destruction runs without cache locks.
pub(crate) fn reclaim(buckets: usize) {
    let start = RECLAIM_HAND.fetch_add(buckets, Ordering::Relaxed);
    for offset in 0..buckets.min(BUCKETS) {
        let index = start.wrapping_add(offset);
        let retired = {
            let Some(mut state) = CACHE[index & (BUCKETS - 1)].try_lock() else {
                continue;
            };
            core::mem::replace(&mut state.entries, [const { None }; WAYS])
        };
        drop(retired);
    }
}

/// Removes references owned by a detached filesystem without disturbing others.
pub(super) fn purge(filesystem: FilesystemId) {
    for cache in &CACHE {
        // Detach a bucket's references with one acquisition. A batch remains
        // stack-resident, and vnode destruction happens after unlocking.
        let retired: [Option<Entry>; WAYS] = {
            let mut state = cache.lock();
            core::array::from_fn(|slot| {
                if state.entries[slot]
                    .as_ref()
                    .is_some_and(|entry| entry.parent.filesystem == filesystem)
                {
                    state.entries[slot].take()
                } else {
                    None
                }
            })
        };
        drop(retired);
    }
}
