//! Persistent completion and generation notifications on shared parking.

use super::{
    parking::wait::{self, Entry, INLINE_WAITERS, WaitSource},
    sync::MutexGuard,
};
use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};

const SIGNALED: u64 = 1;
const QUEUED: u64 = 2;
const GENERATION_STEP: u64 = 4;
const GENERATION_MASK: u64 = !(SIGNALED | QUEUED);

/// A persistent signal with transient, generation-aware notifications.
///
/// `signal` wakes all waiters and makes future waits ready until `reset`.
/// `wake_one` and `wake_all` notify without setting that persistent level.
/// Capture a [`Self::listen`] before checking a separate predicate to retain a
/// notification that races with registration. Always recheck predicates in a
/// loop: notifications do not transfer ownership or reserve a resource.
///
/// Waiting requires sleepable thread context, including ready fast paths.
/// Signaling, resetting, and waking are allocation-free and IRQ-safe. Wake
/// fan-out and parking-bucket contention are not bounded; use a Service when
/// a hard handler needs bounded notification work.
pub struct Event {
    // Combining the level, epoch and queue hint makes registration revalidation
    // acquire every notification that could have skipped an unpublished waiter.
    state: AtomicU64,
}

/// A borrowed notification epoch. Creating a listener does not enqueue it.
/// Notifications after capture remain observable even before `wait` starts.
/// Epoch equality assumes fewer than 2^62 notifications between capture and wait.
#[must_use = "a listener must be waited on to observe notifications"]
#[derive(Clone, Copy)]
pub struct EventListener<'a> {
    event: &'a Event,
    observed: u64,
}

/// Retains either a native borrowed event or a module event's counted storage
/// until a readiness scan and its wait registration have both finished.
pub enum EventSource<'a> {
    /// Native event kept alive by the descriptor supplying it.
    Borrowed(&'a Event),
    /// Module event retained independently of its revoked driver identity.
    Owned(Arc<Event>),
}

impl<'a> From<&'a Event> for EventSource<'a> {
    fn from(event: &'a Event) -> Self {
        Self::Borrowed(event)
    }
}

impl From<Arc<Event>> for EventSource<'_> {
    fn from(event: Arc<Event>) -> Self {
        Self::Owned(event)
    }
}

impl core::ops::Deref for EventSource<'_> {
    type Target = Event;
    fn deref(&self) -> &Event {
        match self {
            Self::Borrowed(event) => event,
            Self::Owned(event) => event,
        }
    }
}

impl Event {
    /// Creates an unsignaled event.
    pub const fn new() -> Self {
        Self {
            state: AtomicU64::new(0),
        }
    }

    /// Returns the persistent level with acquire ordering.
    pub fn is_signaled(&self) -> bool {
        self.state.load(Ordering::Acquire) & SIGNALED != 0
    }

    /// Captures the current notification generation without allocating.
    pub fn listen(&self) -> EventListener<'_> {
        EventListener {
            event: self,
            observed: self.generation(),
        }
    }

    /// Publishes writes, sets the persistent level, and wakes all waiters.
    /// Returns the number of queued waiters selected.
    pub fn signal(&self) -> usize {
        let previous = self
            .state
            .try_update(Ordering::AcqRel, Ordering::Acquire, |state| {
                Some(state.wrapping_add(GENERATION_STEP) | SIGNALED)
            })
            .unwrap();
        if previous & QUEUED == 0 {
            0
        } else {
            wait::wake_all(self)
        }
    }

    /// Clears the persistent level, returning its previous value.
    /// Does not cancel notifications already observed by listeners.
    pub fn reset(&self) -> bool {
        self.state.fetch_and(!SIGNALED, Ordering::AcqRel) & SIGNALED != 0
    }

    /// Publishes a transient notification and selects the oldest queued waiter.
    /// Unregistered listeners also observe the changed epoch; predicate waits
    /// must allow spurious returns. Returns whether a queued waiter was selected.
    pub fn wake_one(&self) -> bool {
        let previous = self.state.fetch_add(GENERATION_STEP, Ordering::AcqRel);
        previous & QUEUED != 0 && wait::wake_one(self)
    }

    /// Publishes a transient notification and wakes all queued waiters.
    /// Returns the number of queued waiters selected.
    pub fn wake_all(&self) -> usize {
        let previous = self.state.fetch_add(GENERATION_STEP, Ordering::AcqRel);
        if previous & QUEUED == 0 {
            0
        } else {
            wait::wake_all(self)
        }
    }

    /// Waits for the persistent level or a notification after this call begins.
    pub fn wait(&self) {
        self.listen().wait();
    }

    /// Waits for one event and returns its index. Already-ready entries are
    /// checked in index order. Duplicate events are allowed. Up to eight entries
    /// use stack storage; larger sets allocate. Panics on an empty set or when
    /// called outside sleepable thread context.
    pub fn wait_any(events: &[&Event]) -> usize {
        assert!(!events.is_empty(), "event: empty wait set");
        wait::assert_wait_context();
        if events.len() <= INLINE_WAITERS {
            let mut entries = [events[0].listen().entry(); INLINE_WAITERS];
            for (entry, event) in entries.iter_mut().zip(events) {
                *entry = event.listen().entry();
            }
            wait::wait(&entries[..events.len()], || {})
        } else {
            let entries = events
                .iter()
                .map(|event| event.listen().entry())
                .collect::<Vec<_>>();
            wait::wait(&entries, || {})
        }
    }

    pub(crate) fn generation(&self) -> u64 {
        self.state.load(Ordering::Acquire) & GENERATION_MASK
    }
}

impl<'event> EventListener<'event> {
    pub(crate) fn entry(&self) -> Entry<'event> {
        Entry {
            source: self.event,
            ticket: self.observed,
        }
    }

    /// Waits using generations captured before the caller checked readiness.
    /// No notification between that check and wait registration can be lost.
    pub fn wait_any(listeners: &[Self]) -> usize {
        assert!(!listeners.is_empty(), "event: empty listener set");
        wait::assert_wait_context();
        if listeners.len() <= INLINE_WAITERS {
            let mut entries = [listeners[0].entry(); INLINE_WAITERS];
            for (entry, listener) in entries.iter_mut().zip(listeners) {
                *entry = listener.entry();
            }
            wait::wait(&entries[..listeners.len()], || {})
        } else {
            let entries = listeners.iter().map(Self::entry).collect::<Vec<_>>();
            wait::wait(&entries, || {})
        }
    }

    /// Whether this listener has observed a persistent or transient notification.
    pub(crate) fn notified(&self) -> bool {
        !self.event.should_wait(self.observed)
    }

    /// Waits for a signal or notification since this listener was captured.
    pub fn wait(self) {
        wait::wait(core::slice::from_ref(&self.entry()), || {});
    }

    /// Registers before releasing the predicate mutex, waits, then reacquires
    /// it. Check the predicate in a loop under the same mutex. Unlock runs
    /// outside the parking bucket; no protected reference survives that unlock.
    pub fn wait_releasing<'a, T: ?Sized>(self, guard: MutexGuard<'a, T>) -> MutexGuard<'a, T> {
        let mutex = guard.mutex();
        wait::wait(core::slice::from_ref(&self.entry()), || drop(guard));
        mutex.lock()
    }
}

// SAFETY: the state address is unique while borrowed and is only used by the
// shared wait backend. All methods are lock-free atomic state operations.
unsafe impl WaitSource for Event {
    fn key(&self) -> usize {
        &self.state as *const AtomicU64 as usize
    }
    fn should_wait(&self, ticket: u64) -> bool {
        let state = self.state.load(Ordering::Acquire);
        state & SIGNALED == 0 && state & GENERATION_MASK == ticket
    }
    fn set_queued(&self, queued: bool) {
        if queued {
            self.state.fetch_or(QUEUED, Ordering::AcqRel);
        } else {
            self.state.fetch_and(!QUEUED, Ordering::AcqRel);
        }
    }
}

impl Default for Event {
    fn default() -> Self {
        Self::new()
    }
}

/// Internal ticket API for predicates that span unlocked work (VM, semaphores).
/// Shares Event's epoch and registration protocol; no second generation atomic.
pub(crate) struct SequenceEvent(Event);

impl SequenceEvent {
    pub(crate) const fn new() -> Self {
        Self(Event::new())
    }
    pub(crate) fn observe(&self) -> u64 {
        self.0.generation()
    }
    pub(crate) fn notify(&self) {
        self.0.wake_all();
    }
    pub(crate) fn wait(&self, ticket: u64) {
        EventListener {
            event: &self.0,
            observed: ticket,
        }
        .wait();
    }
}
