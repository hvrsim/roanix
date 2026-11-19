//! Sleepable, thread-affine mutex backed by the kernel's adaptive mutex.
use crate::{imports, module, result};
use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicUsize, Ordering},
};

/// Thread-context exclusion for operations that may allocate or sleep.
/// Uncontended access is also supported during single-CPU bootstrap, before
/// scheduler publication. The kernel receipt is created lazily, allowing
/// constant/static initialization.
/// Drop releases an idle receipt. Forgetting a guard strands its acquisition.
/// destruction then retains the receipt and module unload returns busy.
pub struct Mutex<T> {
    /// Kernel mutex identity, allocated on first use.
    receipt: AtomicUsize,

    /// State available to the current lock holder.
    value: UnsafeCell<T>,
}

// SAFETY: ownership transfer permits moving a Send value between threads.
unsafe impl<T: Send> Send for Mutex<T> {}
// SAFETY: the kernel mutex serializes access through the exclusive guard.
unsafe impl<T: Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    /// Creates an unlocked mutex. Its kernel receipt is allocated on first use.
    #[must_use]
    pub const fn new(value: T) -> Self {
        Self {
            receipt: AtomicUsize::new(0),
            value: UnsafeCell::new(value),
        }
    }

    /// Borrows the protected value directly when the caller owns the mutex exclusively.
    pub fn get_mut(&mut self) -> &mut T {
        self.value.get_mut()
    }

    /// Installs a kernel mutex receipt once and discards redundant concurrent allocations.
    fn receipt(&self) -> usize {
        let current = self.receipt.load(Ordering::Acquire);
        if current != 0 {
            return current;
        }

        let mut created = 0;

        // SAFETY: module is live and the output slot is writable.
        unsafe {
            result(imports::mutex_create(
                module().expect("mutex module").as_ptr(),
                &raw mut created,
            ))
        }
        .expect("create driver mutex");
        assert_ne!(created, 0, "kernel returned a null mutex identity");
        match self
            .receipt
            .compare_exchange(0, created, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => created,
            Err(installed) => {
                // SAFETY: the unpublished losing receipt has no acquisitions.
                unsafe { result(imports::mutex_destroy(created)) }.expect("discard unused mutex");
                installed
            }
        }
    }

    /// Acquires the mutex in thread context and releases it when the guard is dropped.
    pub fn lock(&self) -> MutexGuard<'_, T> {
        let receipt = self.receipt();

        // SAFETY: the installed receipt lives through this borrow/guard.
        unsafe { result(imports::mutex_lock(receipt)) }.expect("lock driver mutex");
        MutexGuard {
            mutex: self,
            receipt,
            _nosend: PhantomData,
        }
    }

    /// Attempts to acquire the protected value without waiting for its holder.
    /// This remains thread-only: lazy creation and receipt bookkeeping may sleep.
    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        let receipt = self.receipt();

        // SAFETY: the installed receipt lives through this borrow/guard.
        let status = unsafe { imports::mutex_try_lock(receipt) };
        if status == crate::EAGAIN {
            return None;
        }
        result(status).expect("try lock driver mutex");
        Some(MutexGuard {
            mutex: self,
            receipt,
            _nosend: PhantomData,
        })
    }
}
impl<T> Drop for Mutex<T> {
    /// Destroys an idle receipt if one was allocated.
    fn drop(&mut self) {
        let receipt = *self.receipt.get_mut();
        if receipt != 0 {
            // SAFETY: this owns the receipt. Busy acquisitions remain registered
            // and prevent module unload. They are never forcibly unlocked.
            let _ = unsafe { imports::mutex_destroy(receipt) };
        }
    }
}

/// A mutex acquisition must be released on the acquiring thread.
#[must_use]
pub struct MutexGuard<'a, T> {
    /// Mutex that protects the borrowed value.
    mutex: &'a Mutex<T>,

    /// Kernel identity released when the guard drops.
    receipt: usize,

    /// Keeps the acquisition on its original thread.
    _nosend: PhantomData<*mut ()>,
}
impl<T> Deref for MutexGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: the guard retains exclusive kernel admission.
        unsafe { &*self.mutex.value.get() }
    }
}
impl<T> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: this is the unique guard for the protected value.
        unsafe { &mut *self.mutex.value.get() }
    }
}
impl<T> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        // SAFETY: this uniquely releases its acquisition on the same thread.
        unsafe { result(imports::mutex_unlock(self.receipt)) }.expect("unlock driver mutex");
    }
}
