//! Sleeping reader/writer lock for read-mostly kernel structures.

use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicUsize, Ordering},
};

use super::{Mutex, MutexGuard, assert_mutex_context};
use crate::{arch, sys::event::Event};

const WRITER: usize = 1 << (usize::BITS - 1);

/// Concurrent readers or one exclusive writer. A waiting writer blocks new
/// readers. Recursive acquisition and read-to-write upgrades are unsupported.
pub struct RwLock<T> {
    state: AtomicUsize,
    writer: Mutex<()>,
    drained: Event,
    value: UnsafeCell<T>,
}

// SAFETY: moving the lock transfers its owned value.
unsafe impl<T: Send> Send for RwLock<T> {}
// SAFETY: readers share only `&T`; the writer gate and reader count exclude
// every other guard before exposing `&mut T`.
unsafe impl<T: Send + Sync> Sync for RwLock<T> {}

/// Shared access held until this guard is dropped.
#[must_use]
pub struct ReadGuard<'a, T> {
    lock: &'a RwLock<T>,
    _nosend: PhantomData<*mut ()>,
}

/// Exclusive access held until this guard is dropped.
#[must_use]
pub struct WriteGuard<'a, T> {
    lock: &'a RwLock<T>,
    _writer: MutexGuard<'a, ()>,
}

impl<T> RwLock<T> {
    /// Creates an unlocked value.
    pub const fn new(value: T) -> Self {
        Self {
            state: AtomicUsize::new(0),
            writer: Mutex::new(()),
            drained: Event::new(),
            value: UnsafeCell::new(value),
        }
    }

    /// Acquires shared access without taking the writer mutex when uncontended.
    pub fn read(&self) -> ReadGuard<'_, T> {
        assert_mutex_context();
        let mut state = self.state.load(Ordering::Relaxed);
        loop {
            if state & WRITER != 0 {
                // The writer holds this gate until it clears WRITER. Waiting
                // here uses the mutex's race-free parking and inheritance.
                drop(self.writer.lock());
                state = self.state.load(Ordering::Relaxed);
                continue;
            }
            assert!(state < WRITER - 1, "sync: reader count overflow");
            match self.state.compare_exchange_weak(
                state,
                state + 1,
                Ordering::Acquire,
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    return ReadGuard {
                        lock: self,
                        _nosend: PhantomData,
                    };
                }
                Err(current) => state = current,
            }
        }
    }

    /// Blocks new readers, then waits for existing readers to finish.
    pub fn write(&self) -> WriteGuard<'_, T> {
        let writer = self.writer.lock();
        self.drained.reset();
        self.state.fetch_or(WRITER, Ordering::AcqRel);
        while self.state.load(Ordering::Acquire) != WRITER {
            if arch::irqstate() {
                // Only this writer resets the event, before publishing WRITER.
                // The last reader's persistent signal cannot be lost between
                // the count check and queue registration.
                self.drained.wait();
            } else {
                core::hint::spin_loop();
            }
        }
        WriteGuard {
            lock: self,
            _writer: writer,
        }
    }
}

impl<T> Deref for ReadGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: our counted reader excludes exclusive access.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> Drop for ReadGuard<'_, T> {
    fn drop(&mut self) {
        if self.lock.state.fetch_sub(1, Ordering::Release) == WRITER + 1 {
            self.lock.drained.signal();
        }
    }
}

impl<T> Deref for WriteGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: the writer gate is held and all readers have departed.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for WriteGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: this is the only guard permitted to access the value.
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for WriteGuard<'_, T> {
    fn drop(&mut self) {
        self.lock.state.store(0, Ordering::Release);
        // The writer mutex unlocks after WRITER is cleared.
    }
}
