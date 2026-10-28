//!
//! # Synchronization primitives
//!
//! Allocation-free one-time initialization and sleeping mutexes built on the
//! kernel scheduler.
//!

use alloc::sync::Arc;
use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

use crate::{
    arch,
    sys::{
        parking::{self, Node},
        sched, smp,
        thread::{Thread, ThreadLifetime},
    },
};

const STATE_LOCKED: u8 = 1 << 0;
const STATE_QUEUED: u8 = 1 << 1;

const SPIN_BACKOFF_MIN: u32 = 1;
const SPIN_BACKOFF_MAX: u32 = 64;
const ACTIVE_SPIN_ITERS: usize = 8;

mod rwlock;
pub use rwlock::{ReadGuard, RwLock, WriteGuard};
mod spinlock;
pub use spinlock::{SpinGuard, SpinLock};
mod wait;
pub use wait::{Completion, Condition, Semaphore};

const ONCE_STATE_INCOMPLETE: u8 = 0;
const ONCE_STATE_RUNNING: u8 = 1;
const ONCE_STATE_COMPLETE: u8 = 2;
const ONCE_STATE_POISONED: u8 = 3;

/// A value that can be initialized exactly once.
///
/// Readers use an Acquire load on the completed state, while the initializing
/// CPU publishes the value with a Release store. Concurrent initializers spin
/// with bounded exponential backoff until the winning initializer completes.
///
/// Waiters spin rather than block, so this is intended for boot-time and
/// short initializers. A long-running initializer will burn CPU on every
/// other core that races with it.
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
/// Uncontended ownership uses a compare-exchange and retains the owning
/// thread's lifetime token. Under short contention it spins briefly to avoid
/// scheduler traffic, then falls back to FIFO sleeping
/// waiters to avoid burning CPU time. Waiters sleep through the kernel
/// scheduler, so blocking while holding the lock can delay unrelated threads.
///
/// This lock is forbidden in interrupt context. Trap handlers must use
/// [`smp::IrqSpinLock`] instead.
pub struct Mutex<T: ?Sized> {
    state: AtomicU8,
    class: Option<&'static super::lockdep::LockClass>,
    /// Retained identity of the owner; its pointer is revoked before reaping.
    owner: smp::IrqSpinLock<Option<Arc<ThreadLifetime>>>,
    value: UnsafeCell<T>,
}

/// RAII guard returned by [`Mutex::lock`].
#[must_use = "if unused the mutex will immediately unlock"]
pub struct MutexGuard<'a, T: ?Sized> {
    mutex: &'a Mutex<T>,
    _nosend: PhantomData<*mut ()>,
}

struct SpinWait {
    step: u32,
}

struct MutexWaiter {
    thread: *mut Thread,
    park_seq: u64,
    /// Priority lent to the lock holder while this waiter is queued.
    priority: u8,
    lifetime: Arc<ThreadLifetime>,
    /// Thread this waiter is currently boosting, or none when not boosting.
    ///
    /// Ownership can transfer while this waiter sleeps, so the boost has to be
    /// released against whichever thread actually received it.
    boosted: smp::IrqSpinLock<Option<Arc<ThreadLifetime>>>,
    granted: AtomicBool,
}

#[derive(Copy, Clone)]
struct WakeTarget {
    thread: *mut Thread,
    park_seq: u64,
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

// SAFETY: the protected value is only reachable through the mutex protocol.
unsafe impl<T: ?Sized + Send> Send for Mutex<T> {}
// SAFETY: shared references synchronize interior mutation through the mutex.
unsafe impl<T: ?Sized + Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    /// Creates a mutex protecting `value`.
    pub const fn new(value: T) -> Self {
        Self {
            state: AtomicU8::new(0),
            class: None,
            owner: smp::IrqSpinLock::new(None),
            value: UnsafeCell::new(value),
        }
    }

    /// Consumes the mutex and returns the protected value.
    pub fn into_inner(self) -> T {
        self.value.into_inner()
    }

    /// Creates a mutex checked against a static lock hierarchy in debug builds.
    pub const fn with_class(value: T, class: &'static super::lockdep::LockClass) -> Self {
        let mut mutex = Self::new(value);
        mutex.class = Some(class);
        mutex
    }
}

impl<T: ?Sized> Mutex<T> {
    /// Locks the mutex and returns a guard for the protected value.
    ///
    /// Contended callers briefly spin, then park and wait in FIFO order.
    #[inline]
    pub fn lock(&self) -> MutexGuard<'_, T> {
        assert_lock_context();
        super::lockdep::check(self.key(), self.class);
        if self.try_lock_fast() {
            return MutexGuard::new(self);
        }

        smp::assert_blockable("sync: mutex");
        self.lock_slow()
    }

    /// Attempts to lock the mutex without spinning or parking.
    #[inline]
    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        assert_lock_context();
        super::lockdep::check(self.key(), self.class);
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
        // Serialize publication with donation lookup. A waiter that observes
        // LOCKED must not miss its owner between the CAS and identity store.
        let mut owner = self.owner.lock();
        let acquired = self
            .state
            .compare_exchange(0, STATE_LOCKED, Ordering::Acquire, Ordering::Relaxed)
            .is_ok();
        if acquired {
            *owner = Self::current_lifetime();
        }
        acquired
    }

    fn current_lifetime() -> Option<Arc<ThreadLifetime>> {
        sched::current_thread_opt().map(|thread| {
            // SAFETY: the currently executing thread cannot be reaped.
            unsafe { &(*thread).lifetime }.clone()
        })
    }

    /// Records the acquiring thread so waiters can lend it their priority.
    #[inline]
    fn publish_owner(&self) {
        *self.owner.lock() = Self::current_lifetime();
    }

    /// Lends the queued waiter's priority to the current lock holder.
    fn lend_to_owner(&self, waiter: &MutexWaiter) {
        let owner = self.owner.lock().clone();
        if let Some(owner) = owner {
            waiter.boost(owner);
        }
    }

    fn key(&self) -> usize {
        &self.state as *const AtomicU8 as usize
    }

    #[cold]
    #[inline(never)]
    fn lock_slow(&self) -> MutexGuard<'_, T> {
        let mut backoff = SpinWait::new();
        if smp::online_cpus() > 1 {
            for _ in 0..ACTIVE_SPIN_ITERS {
                let state = self.state.load(Ordering::Relaxed);
                if state == 0 && self.try_lock_fast() {
                    return MutexGuard::new(self);
                }
                if state & STATE_QUEUED != 0 {
                    break;
                }
                backoff.spin();
            }
        }
        loop {
            let mut queue = parking::queue(self.key());
            let state = self.state.load(Ordering::Acquire);
            if state & STATE_LOCKED == 0 {
                assert!(queue.is_empty(), "sync: unlocked mutex has waiters");
                if self
                    .state
                    .compare_exchange(state, STATE_LOCKED, Ordering::Acquire, Ordering::Relaxed)
                    .is_ok()
                {
                    self.publish_owner();
                    drop(queue);
                    return MutexGuard::new(self);
                }
                continue;
            }
            // Early boot permits uncontended locks, but has no current thread
            // to park. Scheduled callers have already passed context checks.
            let Some(current) = sched::current_thread_opt() else {
                drop(queue);
                backoff.spin();
                continue;
            };
            assert!(
                queue.irqs_were_enabled(),
                "sync: mutex wait with IRQs masked"
            );
            // SAFETY: the current thread is live throughout this operation.
            let seq = unsafe { (&*current).prepare_park() };
            let waiter = core::pin::pin!(MutexWaiter::new(
                current,
                seq,
                sched::thread_priority(current)
            ));
            let node = core::pin::pin!(Node::new(
                self.key(),
                (waiter.as_ref().get_ref() as *const MutexWaiter).cast(),
                0
            ));
            // SAFETY: both pinned records remain live until removal or grant;
            // the mutex is borrowed for this entire operation.
            unsafe {
                queue.push(node.as_ref().get_ref());
            }
            let previous = self.state.fetch_or(STATE_QUEUED, Ordering::AcqRel);
            if previous & STATE_LOCKED == 0 {
                assert!(queue.remove(node.as_ref().get_ref()));
                assert!(queue.is_empty());
                self.state.store(STATE_LOCKED, Ordering::Relaxed);
                self.publish_owner();
                let restore = queue.unlock_keep_irqs_disabled();
                assert!(sched::wake(current, seq));
                arch::irqset(restore);
                return MutexGuard::new(self);
            }
            self.lend_to_owner(waiter.as_ref().get_ref());
            queue.unlock_keep_irqs_disabled();
            sched::park_current(current, seq);
            if waiter.granted.load(Ordering::Acquire) {
                return MutexGuard::new(self);
            }
            let mut queue = parking::queue(self.key());
            queue.remove(node.as_ref().get_ref());
            if queue.is_empty() {
                self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
            }
            waiter.release_boost();
        }
    }

    #[inline]
    fn unlock(&self) {
        assert_mutex_context();
        {
            let mut owner = self.owner.lock();
            if self
                .state
                .compare_exchange(STATE_LOCKED, 0, Ordering::Release, Ordering::Relaxed)
                .is_ok()
            {
                *owner = None;
                return;
            }
            // Keep the outgoing owner visible until the queued handoff below.
            // A high-priority waiter must still be able to donate while this
            // thread is preempted before acquiring the parking bucket.
        }

        let (target, restore) = {
            let mut queue = parking::queue(self.key());
            let node = queue.pop().expect("sync: queued mutex has no waiter");
            // SAFETY: only this mutex's pinned waiter records use this key;
            // the grantee cannot return before the matching scheduler wake.
            let waiter = unsafe { &*node.data.cast::<MutexWaiter>() };
            let target = WakeTarget {
                thread: waiter.thread,
                park_seq: waiter.park_seq,
            };
            *self.owner.lock() = Some(waiter.lifetime.clone());
            waiter.release_boost();
            queue.for_each(|node| {
                // SAFETY: the bucket keeps every matching pinned waiter live.
                unsafe { &*node.data.cast::<MutexWaiter>() }.boost(waiter.lifetime.clone());
            });
            waiter.granted.store(true, Ordering::Release);
            self.state.store(
                STATE_LOCKED | if queue.is_empty() { 0 } else { STATE_QUEUED },
                Ordering::Release,
            );
            (target, queue.unlock_keep_irqs_disabled())
        };
        let woke = sched::wake(target.thread, target.park_seq);
        arch::irqset(restore);
        assert!(woke, "sync: granted mutex waiter had a stale park sequence");
    }
}

impl Mutex<()> {
    /// Checks the thread affinity of a driver guard represented across the ABI.
    pub(crate) fn owned_by_current(&self) -> bool {
        let Some(current) = Self::current_lifetime() else {
            // Native mutexes permit uncontended early-boot use. Before scheduler
            // publication only the bootstrap CPU runs; no TCB exists to retain.
            return sched::in_bootstrap() && self.owner.lock().is_none() && self.is_locked();
        };
        self.owner
            .lock()
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, &current))
    }

    /// # Safety
    /// Exactly one guard acquired on this thread was forgotten, and the native
    /// allocation remains retained until this method returns.
    pub(crate) unsafe fn release_driver_guard(&self) {
        assert!(
            self.owned_by_current(),
            "mutex: driver guard released by non-owner"
        );
        super::lockdep::released(self.key());
        self.unlock();
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
    /// Underlying mutex for integrated predicate waits.
    pub(crate) fn mutex(&self) -> &'a Mutex<T> {
        self.mutex
    }

    #[inline]
    fn new(mutex: &'a Mutex<T>) -> Self {
        super::lockdep::acquired(mutex.key(), mutex.class);
        Self {
            mutex,
            _nosend: PhantomData,
        }
    }
}

impl MutexWaiter {
    fn new(thread: *mut Thread, park_seq: u64, priority: u8) -> Self {
        Self {
            thread,
            park_seq,
            priority,
            // SAFETY: callers create a waiter only for their live current thread.
            lifetime: unsafe { &(*thread).lifetime }.clone(),
            boosted: smp::IrqSpinLock::new(None),
            granted: AtomicBool::new(false),
        }
    }

    /// Boosts `owner` on this waiter's behalf, replacing any previous boost.
    fn boost(&self, owner: Arc<ThreadLifetime>) {
        let mut boosted = self.boosted.lock();
        sched::boost_priority(&owner, self.priority);
        if let Some(previous) = boosted.replace(owner) {
            sched::unboost_priority(&previous);
        }
    }

    /// Releases this waiter's boost, if it holds one.
    fn release_boost(&self) {
        if let Some(previous) = self.boosted.lock().take() {
            sched::unboost_priority(&previous);
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
        super::lockdep::released(self.mutex.key());
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

/// Acquisition rules apply to fast paths too; otherwise code can fail only
/// when contention happens. Early boot has no schedulable current thread.
fn assert_lock_context() {
    assert_mutex_context();
    super::context::assert_sleepable("sync: mutex acquisition");
}
