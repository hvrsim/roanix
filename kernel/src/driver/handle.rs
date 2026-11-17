//! Opaque identities with counted, IRQ-safe lookups and serialized publication.
//!
//! Thread readers retain values; IRQ readers borrow under a short IRQ lock.
//! Writers allocate a replacement map in sleepable context. No lookup ever
//! dereferences a caller-supplied address, and retired identities are not reused.

use alloc::{collections::BTreeMap, sync::Arc};

use crate::sys::{context, smp::IrqSpinLock, sync::Mutex};

pub(super) struct HandleTable<T: Clone> {
    writer: Mutex<()>,
    entries: IrqSpinLock<Arc<BTreeMap<usize, T>>>,
}

impl<T: Clone> HandleTable<T> {
    pub(super) fn new() -> Self {
        Self {
            writer: Mutex::new(()),
            entries: IrqSpinLock::new(Arc::new(BTreeMap::new())),
        }
    }

    fn snapshot(&self) -> Arc<BTreeMap<usize, T>> {
        self.entries.lock().clone()
    }

    /// Counted lookups are thread-only: the final cloned value may run a
    /// destructor. IRQ callers use `with` and leave ownership in the table.
    pub(super) fn get(&self, id: usize) -> Option<T> {
        context::assert_sleepable("handle: retain");
        self.entries.lock().get(&id).cloned()
    }

    pub(super) fn keys(&self) -> alloc::vec::Vec<usize> {
        context::assert_sleepable("handle: enumerate");
        self.snapshot().keys().copied().collect()
    }

    /// Performs a nonblocking operation while publication retains storage.
    /// The callback must not allocate, sleep, or reenter this table.
    pub(super) fn with<R>(&self, id: usize, action: impl FnOnce(&T) -> R) -> Option<R> {
        let entries = self.entries.lock();
        entries.get(&id).map(action)
    }

    pub(super) fn insert(&self, id: usize, value: T) {
        let _writer = self.writer.lock();
        let mut entries = (*self.snapshot()).clone();
        entries.insert(id, value);
        self.publish(entries);
    }

    pub(super) fn remove(&self, id: usize) -> Option<T> {
        let _writer = self.writer.lock();
        let mut entries = (*self.snapshot()).clone();
        let removed = entries.remove(&id);
        self.publish(entries);
        removed
    }

    pub(super) fn retain(&self, mut keep: impl FnMut(usize, &T) -> bool) {
        let _writer = self.writer.lock();
        let mut entries = (*self.snapshot()).clone();
        entries.retain(|id, value| keep(*id, value));
        self.publish(entries);
    }

    fn publish(&self, entries: BTreeMap<usize, T>) {
        let replacement = Arc::new(entries);
        let previous = core::mem::replace(&mut *self.entries.lock(), replacement);
        // Map destruction can free allocations; keep it outside the IRQ guard.
        drop(previous);
    }
}
