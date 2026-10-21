//! Completion, resource-count, and predicate waits built on shared parking.

use super::MutexGuard;
use crate::sys::{
    context,
    event::{Event, SequenceEvent},
};
use core::sync::atomic::{AtomicUsize, Ordering};

/// Persistent one-shot completion. Complete may be called from a hard IRQ.
/// For IRQ notifications with unbounded fan-out, use a Service instead.
pub struct Completion(Event);

impl Completion {
    /// Creates an incomplete operation.
    pub const fn new() -> Self {
        Self(Event::new())
    }
    /// Publishes completion and wakes current waiters without allocating.
    pub fn complete(&self) {
        self.0.signal();
    }
    /// Waits in sleepable thread context, including when already complete.
    pub fn wait(&self) {
        self.0.wait();
    }
    /// Returns whether completion has been published with acquire ordering.
    pub fn is_complete(&self) -> bool {
        self.0.is_signaled()
    }
}

impl Default for Completion {
    fn default() -> Self {
        Self::new()
    }
}

/// Counted resources. Acquiring and releasing require sleepable thread context.
pub struct Semaphore {
    available: AtomicUsize,
    changed: SequenceEvent,
}

impl Semaphore {
    /// Creates a semaphore with `permits` resources.
    pub const fn new(permits: usize) -> Self {
        Self {
            available: AtomicUsize::new(permits),
            changed: SequenceEvent::new(),
        }
    }
    /// Tries to consume one permit. The context rule also applies on success.
    pub fn try_acquire(&self) -> bool {
        context::assert_sleepable("semaphore: acquire");
        self.available
            .try_update(Ordering::Acquire, Ordering::Relaxed, |value| {
                value.checked_sub(1)
            })
            .is_ok()
    }
    /// Waits until one permit can be consumed.
    pub fn acquire(&self) {
        loop {
            let ticket = self.changed.observe();
            if self.try_acquire() {
                return;
            }
            self.changed.wait(ticket);
        }
    }
    /// Publishes additional permits; overflow is a programming error.
    pub fn release(&self, permits: usize) {
        context::assert_sleepable("semaphore: release");
        if permits == 0 {
            return;
        }
        self.available
            .try_update(Ordering::Release, Ordering::Relaxed, |value| {
                value.checked_add(permits)
            })
            .expect("semaphore: permit overflow");
        self.changed.notify();
    }
}

/// Predicate notification. Always check the predicate in a loop under its mutex.
/// All waiters and notifiers must use the same predicate mutex for each use.
pub struct Condition(Event);

impl Condition {
    /// Creates a condition with no pending notification.
    pub const fn new() -> Self {
        Self(Event::new())
    }
    /// Releases the predicate mutex, waits, and reacquires it before returning.
    /// Registration precedes unlock; spurious returns are valid.
    pub fn wait<'a, T: ?Sized>(&self, guard: MutexGuard<'a, T>) -> MutexGuard<'a, T> {
        context::assert_sleepable("condition: wait");
        self.0.listen().wait_releasing(guard)
    }
    /// Notifies the oldest queued waiter after updating the predicate.
    pub fn notify_one(&self) {
        self.0.wake_one();
    }
    /// Notifies current waiters after updating the predicate under its mutex.
    pub fn notify_all(&self) {
        self.0.wake_all();
    }
}

impl Default for Condition {
    fn default() -> Self {
        Self::new()
    }
}
