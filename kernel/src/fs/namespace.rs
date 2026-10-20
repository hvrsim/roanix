//! Mount topology and directory mutation interlock.
//!
//! Directory mutations share the gate from before path resolution through the
//! backend operation. Mount, unmount and root replacement hold it exclusively,
//! also before resolution. A mount can therefore neither appear between a busy
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
