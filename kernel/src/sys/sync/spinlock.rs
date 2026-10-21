//! Thread-only spinlock: keeps hard interrupts enabled, but pins execution.

use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicBool, Ordering},
};

use crate::sys::smp::{self, PreemptGuard};

/// Protects short thread-only sections without masking interrupts.
/// Hard handlers must use `IrqSpinLock` instead; they may interrupt this owner.
pub struct SpinLock<T> {
    locked: AtomicBool,
    value: UnsafeCell<T>,
}

// SAFETY: transferring the lock transfers its protected value.
unsafe impl<T: Send> Send for SpinLock<T> {}
// SAFETY: all accesses to the value are serialized by the atomic lock.
unsafe impl<T: Send> Sync for SpinLock<T> {}

/// Thread-affine ownership of a spinlock and its preemption restriction.
#[must_use]
pub struct SpinGuard<'a, T> {
    lock: &'a SpinLock<T>,
    _preempt: PreemptGuard,
}

impl<T> SpinLock<T> {
    /// Creates a thread-only spinlock.
    pub const fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            value: UnsafeCell::new(value),
        }
    }

    /// Acquires the lock without permitting preemption or sleeping.
    pub fn lock(&self) -> SpinGuard<'_, T> {
        assert!(!smp::in_interrupt_context(), "spinlock: used in hard IRQ");
        let preempt = smp::disable_preemption();
        while self
            .locked
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            while self.locked.load(Ordering::Relaxed) {
                spin_loop();
            }
        }
        SpinGuard {
            lock: self,
            _preempt: preempt,
        }
    }
}

impl<T> Deref for SpinGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: this guard holds the lock until dropped.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for SpinGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: this unique guard holds exclusive access to the value.
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for SpinGuard<'_, T> {
    fn drop(&mut self) {
        self.lock.locked.store(false, Ordering::Release);
        // The preemption guard drops after publishing the unlocked state.
    }
}
