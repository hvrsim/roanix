//! Mount topology and directory mutation interlock.
//!
//! Opens and directory creation share the gate from before path resolution
//! through path registration. Unlink and rename hold it exclusively so retained
//! entry paths change atomically with the backend. Mount, unmount and root
//! replacement also hold it exclusively before resolution. A mount can therefore neither appear between a busy
//! check and unlink/rename, nor publish a directory detached during lookup.
//!
//! Lock order: this gate, namespace table, filesystem locks. The table lock is
//! released before storage callbacks. Ordinary lookups and file I/O do not take
//! the gate. Never recursively acquire it: a waiting writer blocks new readers.

use crate::sys::sync::{ReadGuard, RwLock, WriteGuard};

static OPERATIONS: RwLock<()> = RwLock::new(());

pub(super) fn mutation() -> ReadGuard<'static, ()> {
    OPERATIONS.read()
}

pub(super) fn topology() -> WriteGuard<'static, ()> {
    OPERATIONS.write()
}
