//! Sleepable module mutex receipts, including native uncontended bootstrap use.
//! Admission counts include holders, parked
//! waiters and unlocks in progress, so withdrawal cannot free a native mutex
//! whose guard is temporarily represented by a driver-side guard.
use super::super::{
    core::module::{self, Module},
    error::{Error, Result},
};
use crate::sys::sync::{Mutex, Once};
use alloc::{collections::BTreeMap, sync::Arc};
use core::sync::atomic::{AtomicUsize, Ordering};

struct Lock {
    native: Mutex<()>,
    owner: Option<Arc<Module>>,
}
struct Entry {
    lock: Arc<Lock>,
    acquisitions: usize,
}
static LOCKS: Once<Mutex<BTreeMap<usize, Entry>>> = Once::new();
static NEXT: AtomicUsize = AtomicUsize::new(1);

fn locks() -> &'static Mutex<BTreeMap<usize, Entry>> {
    LOCKS.call_once(|| Mutex::new(BTreeMap::new()))
}

/// Allocates an idle mutex receipt owned by this module.
pub fn create(owner: Option<&Arc<Module>>) -> Result<usize> {
    let _pin = module::pin_owner(owner, true)?;
    let id = NEXT
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
        .map_err(|_| Error::NoSpace)?;
    let lock = Arc::new(Lock {
        native: Mutex::new(()),
        owner: owner.cloned(),
    });
    let mut entries = locks().lock();
    if owner.is_some_and(|owner| owner.mutexes_closed.load(Ordering::Acquire)) {
        return Err(Error::NoDevice);
    }
    entries.insert(
        id,
        Entry {
            lock,
            acquisitions: 0,
        },
    );
    Ok(id)
}

fn finish(id: usize) {
    let mut entries = locks().lock();
    let entry = entries
        .get_mut(&id)
        .expect("mutex: active receipt withdrawn");
    entry.acquisitions = entry
        .acquisitions
        .checked_sub(1)
        .expect("mutex: unbalanced release");
}

/// Admits an acquisition, optionally waiting for the current holder.
pub fn acquire(id: usize, wait: bool) -> Result<()> {
    let lock = {
        let mut entries = locks().lock();
        let entry = entries.get_mut(&id).ok_or(Error::NotFound)?;
        entry.acquisitions = entry.acquisitions.checked_add(1).ok_or(Error::NoSpace)?;
        entry.lock.clone()
    };
    let result = (|| {
        let _pin = module::pin_owner(lock.owner.as_ref(), true)?;
        let guard = if wait {
            if lock.native.owned_by_current() {
                return Err(Error::Deadlock);
            }
            lock.native.lock()
        } else {
            // Avoid lockdep's recursion diagnostic: a recursive try is simply
            // unavailable, as it is for another thread's held mutex.
            if lock.native.owned_by_current() {
                return Err(Error::WouldBlock);
            }
            lock.native.try_lock().ok_or(Error::WouldBlock)?
        };
        // The receipt and admission count keep the unit mutex alive. The DDK
        // guard eventually releases it on this same thread through release().
        core::mem::forget(guard);
        Ok(())
    })();
    if result.is_err() {
        finish(id);
    }
    result
}

/// Releases one acquisition on its owning thread.
pub fn release(id: usize) -> Result<()> {
    let lock = {
        let entries = locks().lock();
        let entry = entries.get(&id).ok_or(Error::NotFound)?;
        if entry.acquisitions == 0 || !entry.lock.native.owned_by_current() {
            return Err(Error::PermissionDenied);
        }
        entry.lock.clone()
    };
    // SAFETY: acquisition forgot exactly one native unit guard; current-thread
    // ownership was checked above. Its admission prevents removal until finish.
    unsafe { lock.native.release_driver_guard() };
    finish(id);
    Ok(())
}

/// Withdraws an idle receipt; active holders/waiters preserve it on busy.
pub fn destroy(id: usize) -> Result<()> {
    let mut entries = locks().lock();
    let entry = entries.get(&id).ok_or(Error::NotFound)?;
    if entry.acquisitions != 0 {
        return Err(Error::Busy);
    }
    entries.remove(&id);
    Ok(())
}

/// Refuses unload while any acquisition owned by the module remains.
pub fn check_module(module: &Arc<Module>) -> Result<()> {
    if let Some(locks) = LOCKS.get() {
        for entry in locks.lock().values() {
            if entry
                .lock
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
                && entry.acquisitions != 0
            {
                return Err(Error::Busy);
            }
        }
    }
    Ok(())
}

/// Atomically closes module mutex admission and sweeps idle receipts.
pub fn remove_module(module: &Arc<Module>) -> Result<()> {
    let mut entries = locks().lock();
    let owned = |entry: &Entry| {
        entry
            .lock
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module))
    };
    if entries
        .values()
        .any(|entry| owned(entry) && entry.acquisitions != 0)
    {
        return Err(Error::Busy);
    }
    // This is the final fallible teardown step, after callbacks/workers stop.
    // Serialize closure with creation and acquisition; no partial close on busy.
    module.mutexes_closed.store(true, Ordering::Release);
    entries.retain(|_, entry| !owned(entry));
    Ok(())
}
