//!
//! # Synchronization primitives
//!
//! Allocation-free one-time initialization and sleeping mutexes built on the
//! kernel scheduler.
//!

use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

use intrusive_collections::{LinkedList, LinkedListLink, UnsafeRef, intrusive_adapter};

use crate::{
    arch,
    sys::{
        sched,
        smp::{self, IrqSpinLock},
        thread::Thread,
    },
};

const STATE_LOCKED: u8 = 1 << 0;
const STATE_QUEUED: u8 = 1 << 1;

const SPIN_BACKOFF_MIN: u32 = 1;
const SPIN_BACKOFF_MAX: u32 = 64;
const ACTIVE_SPIN_ITERS: usize = 8;

const ONCE_STATE_INCOMPLETE: u8 = 0;
const ONCE_STATE_RUNNING: u8 = 1;
const ONCE_STATE_COMPLETE: u8 = 2;
const ONCE_STATE_POISONED: u8 = 3;

/// A value that can be initialized exactly once.
///
/// Readers use an Acquire load on the completed state, while the initializing
/// CPU publishes the value with a Release store. Concurrent initializers spin
/// with bounded exponential backoff until the winning initializer completes.
pub struct Once<T> {
    state: AtomicU8,
    value: UnsafeCell<MaybeUninit<T>>,
}

struct OnceInitGuard<'a> {
    state: &'a AtomicU8,
    complete: bool,
}

// SAFETY: moving `Once<T>` transfers ownership of its possibly initialized
// value, which is sound when `T` is `Send`.
unsafe impl<T: Send> Send for Once<T> {}
// SAFETY: initialization is serialized by `state`, and shared access after
// publication is sound when `T` is both `Send` and `Sync`.
unsafe impl<T: Send + Sync> Sync for Once<T> {}

impl<T> Once<T> {
    /// Creates empty one-time storage.
    pub const fn new() -> Self {
        Self {
            state: AtomicU8::new(ONCE_STATE_INCOMPLETE),
            value: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Returns the initialized value, or `None` if initialization is incomplete.
    #[inline]
    pub fn get(&self) -> Option<&T> {
        (self.state.load(Ordering::Acquire) == ONCE_STATE_COMPLETE)
            .then(|| self.initialized_value())
    }

    /// Initializes the value if needed and returns the stored value.
    ///
    /// If another CPU is already initializing this instance, the caller waits
    /// until that initializer publishes its result. A panicking initializer
    /// poisons the instance and later initialization attempts panic.
    #[inline]
    pub fn call_once(&self, initializer: impl FnOnce() -> T) -> &T {
        if let Some(value) = self.get() {
            return value;
        }

        self.call_once_slow(initializer)
    }

    #[cold]
    #[inline(never)]
    fn call_once_slow(&self, initializer: impl FnOnce() -> T) -> &T {
        let mut initializer = Some(initializer);
        let mut backoff = SPIN_BACKOFF_MIN;

        loop {
            match self.state.compare_exchange(
                ONCE_STATE_INCOMPLETE,
                ONCE_STATE_RUNNING,
                Ordering::Acquire,
                Ordering::Acquire,
            ) {
                Ok(_) => {
                    let mut guard = OnceInitGuard::new(&self.state);
                    let initializer = initializer
                        .take()
                        .expect("once: initializer consumed before ownership");
                    let value = initializer();

                    // SAFETY: the successful state transition grants this CPU
                    // exclusive initialization access, and the value is not
                    // published until the following Release store.
                    unsafe {
                        (*self.value.get()).write(value);
                    }
                    guard.finish();
                    return self.initialized_value();
                }
                Err(ONCE_STATE_COMPLETE) => return self.initialized_value(),
                Err(ONCE_STATE_RUNNING) => {
                    while self.state.load(Ordering::Acquire) == ONCE_STATE_RUNNING {
                        for _ in 0..backoff {
                            spin_loop();
                        }
                        backoff = (backoff << 1).min(SPIN_BACKOFF_MAX);
                    }
                }
                Err(ONCE_STATE_POISONED) => panic!("once: initializer previously panicked"),
                Err(ONCE_STATE_INCOMPLETE) => continue,
                Err(state) => panic!("once: invalid state {state}"),
            }
        }
    }

    #[inline]
    fn initialized_value(&self) -> &T {
        // SAFETY: callers only reach this method after an Acquire observation
        // of `ONCE_STATE_COMPLETE`, whose Release publication follows the
        // value write.
        unsafe { &*(*self.value.get()).as_ptr() }
    }
}

impl<T> Default for Once<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Drop for Once<T> {
    fn drop(&mut self) {
        if *self.state.get_mut() == ONCE_STATE_COMPLETE {
            // SAFETY: the complete state guarantees exactly one initialized
            // value, and `&mut self` guarantees exclusive destruction.
            unsafe {
                self.value.get_mut().assume_init_drop();
            }
        }
    }
}

impl<'a> OnceInitGuard<'a> {
    fn new(state: &'a AtomicU8) -> Self {
        Self {
            state,
            complete: false,
        }
    }

    fn finish(&mut self) {
        self.state.store(ONCE_STATE_COMPLETE, Ordering::Release);
        self.complete = true;
    }
}

impl Drop for OnceInitGuard<'_> {
    fn drop(&mut self) {
        if !self.complete {
            self.state.store(ONCE_STATE_POISONED, Ordering::Release);
        }
    }
}

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
    granted: AtomicBool,
}

#[derive(Copy, Clone)]
struct WakeTarget {
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

    fn push(&mut self, waiter: &MutexWaiter) {
        // SAFETY: the waiter is pinned on the sleeping thread's stack until it
        // is removed, and the wait-queue lock is held.
        self.list
            .push_back(unsafe { UnsafeRef::from_raw(waiter as *const MutexWaiter) });
    }

    fn pop(&mut self) -> Option<UnsafeRef<MutexWaiter>> {
        self.list.pop_front()
    }

    fn remove(&mut self, waiter: *const MutexWaiter) -> bool {
        // SAFETY: callers pass the pinned waiter associated with this queue
        // while holding its lock.
        if unsafe { !(*waiter).link.is_linked() } {
            return false;
        }

        // SAFETY: a linked waiter belongs to this list and cannot move.
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

        if smp::online_cpus() > 1 {
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
        }

        loop {
            let mut waiters_guard = self.waiters.lock();
            let state = self.state.load(Ordering::Acquire);

            if state & STATE_LOCKED == 0 {
                assert!(
                    waiters_guard.as_ref().is_none_or(WaitQueue::is_empty),
                    "sync: unlocked mutex has queued waiters"
                );
                if self
                    .state
                    .compare_exchange(state, STATE_LOCKED, Ordering::Acquire, Ordering::Relaxed)
                    .is_ok()
                {
                    *waiters_guard = None;
                    drop(waiters_guard);
                    return MutexGuard::new(self);
                }
                continue;
            }

            // Parking requires a trap return to complete the context switch.
            // If IRQs are currently masked, keep spinning instead of enqueuing
            // a sleeper that cannot be rescheduled promptly.
            if !waiters_guard.irqs_were_enabled() {
                drop(waiters_guard);
                backoff.spin();
                continue;
            }

            let Some(current) = sched::current_thread_opt() else {
                drop(waiters_guard);
                backoff.spin();
                continue;
            };
            // SAFETY: the scheduler keeps the current thread allocation live
            // while it is executing.
            let seq = unsafe { (&*current).prepare_park() };
            let waiter = core::pin::pin!(MutexWaiter::new(current, seq));
            let waiters = waiters_guard.get_or_insert_with(WaitQueue::new);
            waiters.push(waiter.as_ref().get_ref());

            // Publish the queue only after linking the waiter. If an
            // uncontended unlock raced before this RMW, its Release transition
            // to zero is observed here and this thread takes ownership instead
            // of parking without a waker.
            let previous = self.state.fetch_or(STATE_QUEUED, Ordering::AcqRel);
            if previous & STATE_LOCKED == 0 {
                assert!(
                    waiters.remove(waiter.as_ref().get_ref() as *const MutexWaiter),
                    "sync: newly linked waiter disappeared"
                );
                assert!(
                    waiters.is_empty(),
                    "sync: unlocked mutex had prior queued waiters"
                );
                *waiters_guard = None;
                self.state.store(STATE_LOCKED, Ordering::Relaxed);

                let restore_irqs = waiters_guard.unlock_keep_irqs_disabled();
                assert!(
                    sched::wake(current, seq),
                    "sync: self-acquired mutex had a stale park sequence"
                );
                if restore_irqs {
                    arch::irqset(true);
                }
                return MutexGuard::new(self);
            }

            let restore_irqs = waiters_guard.unlock_keep_irqs_disabled();
            assert!(
                restore_irqs,
                "sync: mutex queued waiter with interrupts already masked"
            );

            sched::park_current(current, seq);

            if waiter.as_ref().get_ref().granted.load(Ordering::Acquire) {
                return MutexGuard::new(self);
            }

            let mut waiters_guard = self.waiters.lock();
            let waiter_ptr = waiter.as_ref().get_ref() as *const MutexWaiter;
            if let Some(waiters) = waiters_guard.as_mut()
                && waiters.remove(waiter_ptr)
                && waiters.is_empty()
            {
                *waiters_guard = None;
                self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
            }
        }
    }

    #[inline]
    fn unlock(&self) {
        assert_mutex_context();
        if self
            .state
            .compare_exchange(STATE_LOCKED, 0, Ordering::Release, Ordering::Relaxed)
            .is_ok()
        {
            return;
        }

        let (target, restore_irqs) = {
            let mut waiters_guard = self.waiters.lock();
            let waiters = waiters_guard
                .as_mut()
                .expect("sync: contended mutex lost its wait queue");
            let waiter = waiters
                .pop()
                .expect("sync: queued mutex has no waiting thread");
            let queued = !waiters.is_empty();
            if !queued {
                *waiters_guard = None;
            }

            // Snapshot the wake target before releasing the queue lock. The
            // granted waiter may immediately return and destroy its stack node
            // once the scheduler wake completes.
            let target = WakeTarget {
                thread: waiter.thread,
                park_seq: waiter.park_seq,
            };
            waiter.granted.store(true, Ordering::Release);
            self.state.store(
                STATE_LOCKED | if queued { STATE_QUEUED } else { 0 },
                Ordering::Release,
            );
            drop(waiter);

            let restore_irqs = waiters_guard.unlock_keep_irqs_disabled();
            (target, restore_irqs)
        };

        let woke = sched::wake(target.thread, target.park_seq);
        if restore_irqs {
            arch::irqset(true);
        }
        assert!(woke, "sync: granted mutex waiter had a stale park sequence");
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
            granted: AtomicBool::new(false),
        }
    }
}

impl<T: ?Sized> Deref for MutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: holding the mutex guard guarantees shared access to `value`.
        unsafe { &*self.mutex.value.get() }
    }
}

impl<T: ?Sized> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: the unique guard guarantees exclusive access to `value`.
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
