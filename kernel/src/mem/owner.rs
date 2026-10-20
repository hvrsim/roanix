//! Shared lock domains for related memory objects and page backing state.
//!
//! An allocated owner lock may protect an object with its pages or multiple
//! anonymous maps that still share slots. Guard capabilities validate the lock's
//! identity before exposing protected data, preventing a second unrelated lock
//! from governing the same mutable backing state.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use alloc::sync::Arc;
use core::{
    cell::UnsafeCell,
    ops::{Deref, DerefMut},
};

use super::{Error, Result, metadata::NoSleep};
use crate::sys::sync::{ReadGuard, RwLock, WriteGuard};

pub(super) type OwnerRef = Arc<OwnerLock, NoSleep>;

pub(super) struct OwnerLock(RwLock<()>);
impl OwnerLock {
    pub(super) fn new() -> Result<OwnerRef> {
        Arc::try_new_in(Self(RwLock::new(())), NoSleep).map_err(|_| Error::OutOfMemory)
    }
    pub(super) fn try_write(&self) -> Option<OwnerWrite<'_>> {
        Some(OwnerWrite {
            owner: self,
            _guard: self.0.try_write()?,
        })
    }
    pub(super) fn write(&self) -> OwnerWrite<'_> {
        OwnerWrite {
            owner: self,
            _guard: self.0.write(),
        }
    }
}

/// An exclusive owner-lock capability, shared by all cells in this domain.
pub(super) struct OwnerWrite<'a> {
    owner: &'a OwnerLock,
    _guard: WriteGuard<'a, ()>,
}

/// Data protected by a separately allocated owner lock.
pub(super) struct Owned<T> {
    owner: OwnerRef,
    value: UnsafeCell<T>,
}
// SAFETY: the value moves with its cell; all access uses its owner lock.
unsafe impl<T: Send> Send for Owned<T> {}
// SAFETY: shared access requires an owner read/write capability; mutable
// access exclusively borrows the unique writer capability for that domain.
unsafe impl<T: Send + Sync> Sync for Owned<T> {}

impl<T> Owned<T> {
    pub(super) fn new(owner: OwnerRef, value: T) -> Self {
        Self {
            owner,
            value: UnsafeCell::new(value),
        }
    }
    pub(super) fn owner(&self) -> &OwnerRef {
        &self.owner
    }
    pub(super) fn lock(&self) -> OwnedGuard<'_, T> {
        OwnedGuard {
            cell: self,
            guard: self.owner.write(),
        }
    }
    pub(super) fn with_guard<'a>(&'a self, guard: OwnerWrite<'a>) -> OwnedGuard<'a, T> {
        let _ = self.get(&guard);
        OwnedGuard { cell: self, guard }
    }
    pub(super) fn read(&self) -> OwnedRead<'_, T> {
        OwnedRead {
            cell: self,
            _guard: self.owner.0.read(),
        }
    }
    pub(super) fn get<'a>(&'a self, guard: &'a OwnerWrite<'_>) -> &'a T {
        assert!(
            core::ptr::eq(guard.owner, &*self.owner),
            "mem: wrong page owner lock"
        );
        // SAFETY: the matching exclusive owner lock is held. The shared
        // capability borrow prevents mutable access for this reference's life.
        unsafe { &*self.value.get() }
    }
    pub(super) fn get_locked_mut<'a>(&'a self, guard: &'a mut OwnerWrite<'_>) -> &'a mut T {
        assert!(
            core::ptr::eq(guard.owner, &*self.owner),
            "mem: wrong page owner lock"
        );
        // SAFETY: the matching exclusive lock is held and its unique mutable
        // capability borrow prevents any other cell access aliasing this one.
        unsafe { &mut *self.value.get() }
    }
    pub(super) fn get_mut(&mut self) -> &mut T {
        self.value.get_mut()
    }
}

pub(super) struct OwnedGuard<'a, T> {
    cell: &'a Owned<T>,
    guard: OwnerWrite<'a>,
}
impl<'a, T> OwnedGuard<'a, T> {
    pub(super) fn capability(&self) -> &OwnerWrite<'_> {
        &self.guard
    }
    pub(super) fn capability_mut(&mut self) -> &mut OwnerWrite<'a> {
        &mut self.guard
    }
}
impl<T> Deref for OwnedGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.cell.get(&self.guard)
    }
}
impl<T> DerefMut for OwnedGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.cell.get_locked_mut(&mut self.guard)
    }
}
pub(super) struct OwnedRead<'a, T> {
    cell: &'a Owned<T>,
    _guard: ReadGuard<'a, ()>,
}
impl<T> Deref for OwnedRead<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: this cell's owner read lock excludes all writer capabilities.
        unsafe { &*self.cell.value.get() }
    }
}
