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
    sync::atomic::{AtomicU8, Ordering},
};

use intrusive_collections::{intrusive_adapter, LinkedList, LinkedListLink, UnsafeRef};

use crate::{
    arch,
    sys::{sched, smp::IrqSpinLock, thread::Thread},
};

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
///
/// This lock is forbidden in interrupt context. Trap handlers must use
/// [`IrqSpinLock`] instead.
pub struct Mutex<T: ?Sized> {
    state: AtomicU8,
    waiters: IrqSpinLock<Option<WaitQueue>>,
    value: UnsafeCell<T>,
}

/// RAII guard returned by [`Mutex::lock`].
#[must_use = "if unused the mutex will immediately unlock"]
pub struct MutexGuard<'a, T: ?Sized> {
    mutex: &'a Mutex<T>,
    _nosend: PhantomData<*mut ()>,
}

struct WaitQueue {
    list: LinkedList<MutexWaiterAdapter>,
}

struct SpinWait {
    step: u32,
}

struct MutexWaiter {
    link: LinkedListLink,
    thread: *mut Thread,
    park_seq: u64,
}

intrusive_adapter!(MutexWaiterAdapter = UnsafeRef<MutexWaiter>: MutexWaiter { link: LinkedListLink });

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
    fn new() -> Self {
        Self {
            list: LinkedList::new(MutexWaiterAdapter::NEW),
        }
    }

    fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    fn state_bits(&self) -> u8 {
        if self.is_empty() {
            0
        } else {
            STATE_QUEUED
        }
    }

    fn push(&mut self, waiter: &MutexWaiter) {
        self.list
            .push_back(unsafe { UnsafeRef::from_raw(waiter as *const MutexWaiter) });
    }

    fn pop(&mut self) -> Option<UnsafeRef<MutexWaiter>> {
        self.list.pop_front()
    }

    fn remove(&mut self, waiter: *const MutexWaiter) -> bool {
        if unsafe { !(*waiter).link.is_linked() } {
            return false;
        }

        unsafe {
            self.list.cursor_mut_from_ptr(waiter).remove();
        }
        true
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
            waiters: IrqSpinLock::new(None),
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
        assert_mutex_context();
        if self.try_lock_fast() {
            return MutexGuard::new(self);
        }

        self.lock_slow()
    }

    /// Attempts to lock the mutex without spinning or parking.
    #[inline]
    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        assert_mutex_context();
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
            let mut waiters_guard = self.waiters.lock();
            let waiters = waiters_guard.get_or_insert_with(WaitQueue::new);
            let state = self.state.load(Ordering::Acquire);

            if state & STATE_LOCKED == 0 {
                let next = STATE_LOCKED | waiters.state_bits();
                if self
                    .state
                    .compare_exchange(state, next, Ordering::Acquire, Ordering::Relaxed)
                    .is_ok()
                {
                    drop(waiters_guard);
                    return MutexGuard::new(self);
                }
                continue;
            }

            // Parking requires a trap return to complete the context switch.
            // If IRQs are currently masked, keep spinning instead of enqueuing
            // a sleeper that cannot be rescheduled promptly.
            if !arch::irqstate() {
                drop(waiters_guard);
                backoff.spin();
                continue;
            }

            let Some(current) = sched::current_thread_opt() else {
                drop(waiters_guard);
                backoff.spin();
                continue;
            };
            let seq = unsafe { (&*current).prepare_park() };
            let waiter = core::pin::pin!(MutexWaiter::new(current, seq));
            waiters.push(waiter.as_ref().get_ref());
            self.state.fetch_or(STATE_QUEUED, Ordering::Release);
            drop(waiters_guard);

            sched::park_current(seq);

            let mut waiters_guard = self.waiters.lock();
            let waiter_ptr = waiter.as_ref().get_ref() as *const MutexWaiter;
            if let Some(waiters) = waiters_guard.as_mut() {
                if waiters.remove(waiter_ptr) && waiters.is_empty() {
                    self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
                }
            }
        }
    }

    #[inline]
    fn unlock(&self) {
        assert_mutex_context();
        loop {
            let waiter = {
                let mut waiters_guard = self.waiters.lock();
                let Some(waiters) = waiters_guard.as_mut() else {
                    self.state.store(0, Ordering::Release);
                    return;
                };

                // Keep the remaining sleepers attached to the mutex. Draining
                // the whole queue into a stack-local list can strand waiters
                // forever if the unlocking thread is preempted mid-drain.
                let waiter = waiters.pop();
                let queued = !waiters.is_empty();
                if !queued {
                    *waiters_guard = None;
                }
                self.state
                    .store(if queued { STATE_QUEUED } else { 0 }, Ordering::Release);
                waiter
            };

            let Some(waiter) = waiter else {
                return;
            };

            // Retry on stale wake races so we never drop all wakeups for a
            // queue that still has blocked sleepers.
            if sched::wake(waiter.thread, waiter.park_seq) {
                return;
            }
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

impl MutexWaiter {
    fn new(thread: *mut Thread, park_seq: u64) -> Self {
        Self {
            link: LinkedListLink::new(),
            thread,
            park_seq,
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

#[inline]
fn assert_mutex_context() {
    assert!(
        !crate::sys::smp::in_interrupt_context(),
        "sync: sleeping mutex used from interrupt context"
    );
}
