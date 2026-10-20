//! Address-map locking with a temporary exclusive-operation owner.
//!
//! Readers may inspect a busy map. Other writers wait while the busy owner drops
//! and reacquires map locks to finish an operation that spans unlocked work.
//! The busy guard clears ownership and wakes waiting writers when released;
//! recursive or mismatched busy ownership is rejected.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use super::VmMap;
use crate::sys::{
    event::SequenceEvent,
    sched,
    sync::{Mutex, ReadGuard, RwLock, WriteGuard},
};
use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

pub(super) struct MapLock {
    entries: RwLock<VmMap>,
    busy: Mutex<Option<usize>>,
    changed: SequenceEvent,
}
fn current() -> usize {
    sched::current_thread_opt().map_or(usize::MAX, |thread| thread as usize)
}
impl MapLock {
    pub(super) fn new(map: VmMap) -> Self {
        Self {
            entries: RwLock::new(map),
            busy: Mutex::new(None),
            changed: SequenceEvent::new(),
        }
    }
    pub(super) fn read(&self) -> ReadGuard<'_, VmMap> {
        self.entries.read()
    }
    pub(super) fn write(&self) -> MapWrite<'_> {
        loop {
            let mut entries = self.entries.write();
            let busy = self.busy.lock();
            if busy.is_none_or(|owner| owner == current()) {
                entries.write_locked();
                return MapWrite {
                    lock: self,
                    entries,
                };
            }
            let ticket = self.changed.observe();
            drop(entries);
            drop(busy);
            self.changed.wait(ticket);
        }
    }
    #[expect(dead_code, reason = "nonblocking map-lock entry point")]
    pub(super) fn try_write(&self) -> Option<MapWrite<'_>> {
        let mut entries = self.entries.try_write()?;
        if self.busy.try_lock()?.is_some() {
            return None;
        }
        entries.write_locked();
        Some(MapWrite {
            lock: self,
            entries,
        })
    }
}
pub(super) struct MapWrite<'a> {
    lock: &'a MapLock,
    entries: WriteGuard<'a, VmMap>,
}
impl<'a> MapWrite<'a> {
    /// The token keeps ownership of VM_MAP_BUSY across releasing the rwlock.
    /// Its borrow keeps the map live; its !Send marker enforces LWP ownership.
    #[expect(dead_code, reason = "vm_map_busy API for pageable/wiring operations")]
    pub(super) fn busy(&self) -> MapBusy<'a> {
        let mut busy = self.lock.busy.lock();
        assert!(busy.is_none(), "mem: map already busy");
        let owner = current();
        *busy = Some(owner);
        MapBusy {
            lock: self.lock,
            owner,
            _nosend: PhantomData,
        }
    }
}
impl Deref for MapWrite<'_> {
    type Target = VmMap;
    fn deref(&self) -> &VmMap {
        &self.entries
    }
}
impl DerefMut for MapWrite<'_> {
    fn deref_mut(&mut self) -> &mut VmMap {
        &mut self.entries
    }
}
pub(super) struct MapBusy<'a> {
    lock: &'a MapLock,
    owner: usize,
    _nosend: PhantomData<*mut ()>,
}
impl Drop for MapBusy<'_> {
    fn drop(&mut self) {
        assert_eq!(
            self.owner,
            current(),
            "mem: busy map released by another LWP"
        );
        let mut busy = self.lock.busy.lock();
        assert_eq!(*busy, Some(self.owner));
        *busy = None;
        self.lock.changed.notify();
    }
}
