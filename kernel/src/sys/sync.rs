//!
//! # Synchronization primitives
//!
//! Sleeping synchronization primitives built on the kernel scheduler.
//!

use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    ptr,
    sync::atomic::{AtomicU8, Ordering},
};

use crate::sys::{sched, smp::IrqSpinLock, thread::Thread};

const STATE_LOCKED: u8 = 1 << 0;
const STATE_QUEUED: u8 = 1 << 1;

const SPIN_BACKOFF_MIN: u32 = 1;
const SPIN_BACKOFF_MAX: u32 = 64;
const ACTIVE_SPIN_ITERS: usize = 8;

/// A compact adaptive mutex with an optimistic CAS fast path.
///
/// The uncontended path is a single compare-exchange. Under short contention it
/// spins briefly to avoid scheduler traffic, then falls back to FIFO sleeping
/// waiters to avoid burning CPU time. Waiters sleep through the kernel
/// scheduler, so blocking while holding the lock can delay unrelated threads.
pub struct Mutex<T: ?Sized> {
    state: AtomicU8,
    waiters: IrqSpinLock<WaitQueue>,
    value: UnsafeCell<T>,
}

/// RAII guard returned by [`Mutex::lock`].
#[must_use = "if unused the mutex will immediately unlock"]
pub struct MutexGuard<'a, T: ?Sized> {
    mutex: &'a Mutex<T>,
    _nosend: PhantomData<*mut ()>,
}

struct WaitQueue {
    head: *mut Thread,
    tail: *mut Thread,
}

struct SpinWait {
    step: u32,
}

impl SpinWait {
    const fn new() -> Self {
        Self {
            step: SPIN_BACKOFF_MIN,
        }
    }

    fn spin(&mut self) {
        for _ in 0..self.step {
            spin_loop();
        }

        self.step = (self.step << 1).min(SPIN_BACKOFF_MAX);
    }
}

impl WaitQueue {
    const fn new() -> Self {
        Self {
            head: ptr::null_mut(),
            tail: ptr::null_mut(),
        }
    }

    fn is_empty(&self) -> bool {
        self.head.is_null()
    }

    fn state_bits(&self) -> u8 {
        if self.is_empty() {
            0
        } else {
            STATE_QUEUED
        }
    }

    fn push(&mut self, thread: *mut Thread) {
        assert!(!thread.is_null(), "sync: queued null waiter");
        unsafe {
            (*thread).wait_next = ptr::null_mut();
        }

        if self.tail.is_null() {
            self.head = thread;
        } else {
            unsafe {
                (*self.tail).wait_next = thread;
            }
        }

        self.tail = thread;
    }

    fn pop(&mut self) -> Option<*mut Thread> {
        let thread = self.head;
        if thread.is_null() {
            return None;
        }

        self.head = unsafe { (*thread).wait_next };
        if self.head.is_null() {
            self.tail = ptr::null_mut();
        }
        unsafe {
            (*thread).wait_next = ptr::null_mut();
        }
        Some(thread)
    }
}

// SAFETY: wait queues are only manipulated while holding `waiters`.
unsafe impl Send for WaitQueue {}

// SAFETY: the protected value is only reachable through the mutex protocol.
unsafe impl<T: ?Sized + Send> Send for Mutex<T> {}
// SAFETY: shared references synchronize interior mutation through the mutex.
unsafe impl<T: ?Sized + Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    /// Creates a mutex protecting `value`.
    pub const fn new(value: T) -> Self {
        Self {
            state: AtomicU8::new(0),
            waiters: IrqSpinLock::new(WaitQueue::new()),
            value: UnsafeCell::new(value),
        }
    }

    /// Consumes the mutex and returns the protected value.
    pub fn into_inner(self) -> T {
        self.value.into_inner()
    }
}

impl<T: ?Sized> Mutex<T> {
    /// Locks the mutex and returns a guard for the protected value.
    ///
    /// Contended callers briefly spin, then park and wait in FIFO order.
    #[inline]
    pub fn lock(&self) -> MutexGuard<'_, T> {
        if self.try_lock_fast() {
            return MutexGuard::new(self);
        }

        self.lock_slow()
    }

    /// Attempts to lock the mutex without spinning or parking.
    #[inline]
    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        self.try_lock_fast().then(|| MutexGuard::new(self))
    }

    /// Returns whether the mutex is currently held.
    #[inline]
    pub fn is_locked(&self) -> bool {
        self.state.load(Ordering::Relaxed) & STATE_LOCKED != 0
    }

    /// Returns a mutable reference to the protected value.
    pub fn get_mut(&mut self) -> &mut T {
        self.value.get_mut()
    }

    #[inline]
    fn try_lock_fast(&self) -> bool {
        self.state
            .compare_exchange(0, STATE_LOCKED, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    }

    #[cold]
    #[inline(never)]
    fn lock_slow(&self) -> MutexGuard<'_, T> {
        let mut backoff = SpinWait::new();

        for _ in 0..ACTIVE_SPIN_ITERS {
            let state = self.state.load(Ordering::Relaxed);
            if state == 0 {
                if self.try_lock_fast() {
                    return MutexGuard::new(self);
                }
            } else if state & STATE_QUEUED != 0 {
                break;
            }

            backoff.spin();
        }

        loop {
            let mut waiters = self.waiters.lock();
            let state = self.state.load(Ordering::Acquire);

            if state & STATE_LOCKED == 0 {
                let next = STATE_LOCKED | waiters.state_bits();
                if self
                    .state
                    .compare_exchange(state, next, Ordering::Acquire, Ordering::Relaxed)
                    .is_ok()
                {
                    drop(waiters);
                    return MutexGuard::new(self);
                }
                continue;
            }

            let current = sched::current_thread() as *mut Thread;
            unsafe {
                (&*current).prepare_park();
            }
            waiters.push(current);
            self.state.fetch_or(STATE_QUEUED, Ordering::Release);
            drop(waiters);

            sched::park_current();
        }
    }

    #[inline]
    fn unlock(&self) {
        let waiter = {
            let mut waiters = self.waiters.lock();
            let Some(waiter) = waiters.pop() else {
                self.state.store(0, Ordering::Release);
                return;
            };

            self.state.store(waiters.state_bits(), Ordering::Release);
            waiter
        };

        unsafe {
            // SAFETY: waiters are enqueued from live scheduler threads and
            // removed under the wait-queue lock before waking.
            sched::wake(&mut *waiter);
        }
    }
}

impl<T: Default> Default for Mutex<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

impl<T> From<T> for Mutex<T> {
    fn from(value: T) -> Self {
        Self::new(value)
    }
}

impl<'a, T: ?Sized> MutexGuard<'a, T> {
    #[inline]
    fn new(mutex: &'a Mutex<T>) -> Self {
        Self {
            mutex,
            _nosend: PhantomData,
        }
    }
}

impl<T: ?Sized> Deref for MutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.mutex.value.get() }
    }
}

impl<T: ?Sized> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.mutex.value.get() }
    }
}

impl<T: ?Sized> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        self.mutex.unlock();
    }
}
