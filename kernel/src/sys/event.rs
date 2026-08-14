//!
//! # Waitable events
//!
//! Thread-safe level-triggered events with transient wake operations and
//! wait-any support.
//!

use alloc::{boxed::Box, vec::Vec};
use core::{
    ptr,
    sync::atomic::{AtomicPtr, AtomicU8, AtomicUsize, Ordering},
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

const STATE_SIGNALED: u8 = 1 << 0;
const STATE_QUEUED: u8 = 1 << 1;
const NO_WINNER: usize = usize::MAX;
const INLINE_WAITERS: usize = 8;

/// A signallable event on which one or more threads can wait.
///
/// An event has a persistent signaled state. [`Event::signal`] sets that state
/// and wakes all current waiters; later waits also complete immediately until
/// [`Event::reset`] clears it. This level-triggered behavior makes an event
/// suitable for completion objects such as timer expiry and thread exit.
///
/// [`Event::wake_one`] and [`Event::wake_all`] are transient notifications that
/// do not change the signaled state. Callers using them for condition-style
/// waits must protect their condition separately so a notification cannot
/// occur before the waiter is queued.
///
/// Waiting is only valid in ordinary thread context with interrupts enabled.
/// Signaling and waking are IRQ-safe and do not allocate.
pub struct Event {
    state: AtomicU8,
    waiters: IrqSpinLock<Option<WaitQueue>>,
}

struct WaitQueue {
    list: LinkedList<EventWaiterAdapter>,
}

struct EventWaiter {
    link: LinkedListLink,
    event: *const Event,
    group: *mut WaitGroup,
    index: usize,
}

struct WaitGroup {
    thread: *mut Thread,
    park_seq: u64,
    winner: AtomicUsize,
    wake_next: AtomicPtr<WaitGroup>,
}

enum WaitRegistrations {
    Inline {
        slots: [Option<EventWaiter>; INLINE_WAITERS],
        len: usize,
    },
    Heap(Box<[EventWaiter]>),
}

#[derive(Copy, Clone)]
struct WakeTarget {
    thread: *mut Thread,
    park_seq: u64,
}

struct WakeBatch {
    head: *mut WaitGroup,
    tail: *mut WaitGroup,
    count: usize,
}

intrusive_adapter!(EventWaiterAdapter = UnsafeRef<EventWaiter>: EventWaiter {
    link: LinkedListLink
});

impl WaitQueue {
    fn new() -> Self {
        Self {
            list: LinkedList::new(EventWaiterAdapter::NEW),
        }
    }

    fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    fn push(&mut self, waiter: &EventWaiter) {
        // SAFETY: a waiter is pinned on the waiting thread's stack or in a
        // stable boxed slice until it is unlinked under this event's lock.
        self.list
            .push_back(unsafe { UnsafeRef::from_raw(waiter as *const EventWaiter) });
    }

    fn pop(&mut self) -> Option<UnsafeRef<EventWaiter>> {
        self.list.pop_front()
    }

    fn remove(&mut self, waiter: *const EventWaiter) -> bool {
        // SAFETY: callers pass a live registration for this event while
        // holding the event's wait-queue lock.
        if unsafe { !(*waiter).link.is_linked() } {
            return false;
        }

        // SAFETY: a linked waiter belongs to this event's list and cannot move.
        unsafe {
            self.list.cursor_mut_from_ptr(waiter).remove();
        }
        true
    }
}

// SAFETY: every wait-queue link and raw waiter pointer is accessed only while
// holding the containing event's `waiters` lock.
unsafe impl Send for WaitQueue {}

impl EventWaiter {
    fn new(event: &Event, group: *mut WaitGroup, index: usize) -> Self {
        Self {
            link: LinkedListLink::new(),
            event,
            group,
            index,
        }
    }
}

impl WaitGroup {
    fn new(thread: *mut Thread, park_seq: u64) -> Self {
        Self {
            thread,
            park_seq,
            winner: AtomicUsize::new(NO_WINNER),
            wake_next: AtomicPtr::new(ptr::null_mut()),
        }
    }

    fn try_claim(&self, index: usize) -> bool {
        self.winner
            .compare_exchange(NO_WINNER, index, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    fn winner(&self) -> Option<usize> {
        match self.winner.load(Ordering::Acquire) {
            NO_WINNER => None,
            index => Some(index),
        }
    }

    fn target(&self) -> WakeTarget {
        WakeTarget {
            thread: self.thread,
            park_seq: self.park_seq,
        }
    }
}

impl WaitRegistrations {
    fn new(events: &[&Event], group: *mut WaitGroup) -> Self {
        if events.len() <= INLINE_WAITERS {
            let mut slots = core::array::from_fn(|_| None);
            for (index, event) in events.iter().enumerate() {
                slots[index] = Some(EventWaiter::new(event, group, index));
            }
            Self::Inline {
                slots,
                len: events.len(),
            }
        } else {
            Self::Heap(
                events
                    .iter()
                    .enumerate()
                    .map(|(index, event)| EventWaiter::new(event, group, index))
                    .collect::<Vec<_>>()
                    .into_boxed_slice(),
            )
        }
    }

    fn from_slice(waiters: &[EventWaiter]) -> RegistrationSlice<'_> {
        RegistrationSlice(waiters)
    }

    fn len(&self) -> usize {
        match self {
            Self::Inline { len, .. } => *len,
            Self::Heap(waiters) => waiters.len(),
        }
    }

    fn get(&self, index: usize) -> &EventWaiter {
        match self {
            Self::Inline { slots, len } => {
                assert!(index < *len, "event: inline registration out of range");
                slots[index]
                    .as_ref()
                    .expect("event: missing inline registration")
            }
            Self::Heap(waiters) => &waiters[index],
        }
    }
}

/// Uniform read access to a set of pinned registrations.
trait Registrations {
    fn len(&self) -> usize;
    fn get(&self, index: usize) -> &EventWaiter;
}

struct RegistrationSlice<'a>(&'a [EventWaiter]);

impl Registrations for RegistrationSlice<'_> {
    fn len(&self) -> usize {
        self.0.len()
    }

    fn get(&self, index: usize) -> &EventWaiter {
        &self.0[index]
    }
}

impl Registrations for WaitRegistrations {
    fn len(&self) -> usize {
        WaitRegistrations::len(self)
    }

    fn get(&self, index: usize) -> &EventWaiter {
        WaitRegistrations::get(self, index)
    }
}

impl WakeTarget {
    fn wake(self) {
        assert!(
            sched::wake(self.thread, self.park_seq),
            "event: claimed waiter had a stale park sequence"
        );
    }
}

impl WakeBatch {
    const fn new() -> Self {
        Self {
            head: ptr::null_mut(),
            tail: ptr::null_mut(),
            count: 0,
        }
    }

    fn push(&mut self, group: *mut WaitGroup) {
        // SAFETY: a successfully claimed wait group remains pinned until this
        // batch invokes its matching scheduler wake.
        let group_ref = unsafe { &*group };
        group_ref
            .wake_next
            .store(ptr::null_mut(), Ordering::Relaxed);

        if self.tail.is_null() {
            self.head = group;
        } else {
            // SAFETY: `tail` is a previously claimed group in this batch and
            // remains live until the batch reaches it.
            unsafe { &*self.tail }
                .wake_next
                .store(group, Ordering::Relaxed);
        }

        self.tail = group;
        self.count += 1;
    }

    fn wake(self) -> usize {
        let mut group = self.head;
        while !group.is_null() {
            // SAFETY: the claimant owns wake responsibility for this pinned
            // group. Read everything needed before waking because the target
            // may immediately run, unlink its registrations, and return.
            let (next, target) = unsafe {
                let group_ref = &*group;
                (
                    group_ref.wake_next.load(Ordering::Relaxed),
                    group_ref.target(),
                )
            };
            target.wake();
            group = next;
        }
        self.count
    }
}

impl Event {
    /// Creates an unsignaled event with no waiters.
    pub const fn new() -> Self {
        Self {
            state: AtomicU8::new(0),
            waiters: IrqSpinLock::new(None),
        }
    }

    /// Returns whether the event is currently signaled.
    #[inline]
    pub fn is_signaled(&self) -> bool {
        self.state.load(Ordering::Acquire) & STATE_SIGNALED != 0
    }

    /// Sets the persistent signal and wakes all current waiters.
    ///
    /// Writes performed before this call become visible to threads whose wait
    /// completes because of this signal.
    ///
    /// Returns the number of waiting threads selected for wakeup.
    pub fn signal(&self) -> usize {
        let previous = self.state.fetch_or(STATE_SIGNALED, Ordering::AcqRel);
        if previous & STATE_QUEUED == 0 {
            return 0;
        }

        self.claim_all().wake()
    }

    /// Clears the persistent signal.
    ///
    /// Returns whether the event had been signaled.
    pub fn reset(&self) -> bool {
        self.state.fetch_and(!STATE_SIGNALED, Ordering::AcqRel) & STATE_SIGNALED != 0
    }

    /// Wakes the oldest eligible waiter without changing the signaled state.
    ///
    /// This is a transient notification and can be missed by a thread that has
    /// not completed queue registration. Use [`Event::signal`] when the event
    /// must remain observable.
    ///
    /// Returns whether a waiting thread was selected.
    pub fn wake_one(&self) -> bool {
        if self.state.load(Ordering::Acquire) & STATE_QUEUED == 0 {
            return false;
        }

        let Some(target) = self.claim_one() else {
            return false;
        };
        target.wake();
        true
    }

    /// Wakes all current waiters without changing the signaled state.
    ///
    /// This is a transient notification and can be missed by threads that have
    /// not completed queue registration. Use [`Event::signal`] for persistent
    /// completion state.
    ///
    /// Returns the number of waiting threads selected.
    pub fn wake_all(&self) -> usize {
        if self.state.load(Ordering::Acquire) & STATE_QUEUED == 0 {
            return 0;
        }

        self.claim_all().wake()
    }

    /// Blocks the current thread until this event is signaled or woken.
    pub fn wait(&self) {
        assert_wait_context();
        if self.is_signaled() {
            return;
        }

        let current = sched::current_thread();
        // SAFETY: the scheduler keeps the current thread allocation live while
        // it is executing.
        let park_seq = unsafe { (&*current).prepare_park() };
        let group = core::pin::pin!(WaitGroup::new(current, park_seq));
        let group_ptr = group.as_ref().get_ref() as *const WaitGroup as *mut WaitGroup;
        let waiter = core::pin::pin!(EventWaiter::new(self, group_ptr, 0));
        let registrations = WaitRegistrations::from_slice(core::slice::from_ref(
            waiter.as_ref().get_ref(),
        ));

        let winner = wait_with_registrations(
            core::slice::from_ref(&self),
            &registrations,
            group.as_ref().get_ref(),
        );
        assert_eq!(winner, 0, "event: single wait returned invalid winner");
    }

    /// Blocks until any event in `events` is signaled or woken.
    ///
    /// The returned index identifies the event that claimed the wait. If
    /// several events are already signaled, the lowest index is returned.
    ///
    /// # Panics
    ///
    /// Panics if `events` is empty or the caller is not in sleepable thread
    /// context.
    pub fn wait_any(events: &[&Event]) -> usize {
        assert!(!events.is_empty(), "event: cannot wait on an empty set");
        assert_wait_context();

        if let Some(index) = events.iter().position(|event| event.is_signaled()) {
            return index;
        }
        if events.len() == 1 {
            events[0].wait();
            return 0;
        }

        let current = sched::current_thread();
        // SAFETY: the scheduler keeps the current thread allocation live while
        // it is executing.
        let park_seq = unsafe { (&*current).prepare_park() };
        let group = core::pin::pin!(WaitGroup::new(current, park_seq));
        let group_ptr = group.as_ref().get_ref() as *const WaitGroup as *mut WaitGroup;

        let registrations = core::pin::pin!(WaitRegistrations::new(events, group_ptr));

        // Pinning keeps inline registrations stable; large sets use a stable
        // boxed slice. Every node is unlinked before this storage is dropped.
        wait_with_registrations(
            events,
            registrations.as_ref().get_ref(),
            group.as_ref().get_ref(),
        )
    }

    fn register(&self, waiter: &EventWaiter) -> bool {
        let mut waiters_guard = self.waiters.lock();
        if self.state.load(Ordering::Acquire) & STATE_SIGNALED != 0 {
            return false;
        }

        waiters_guard
            .get_or_insert_with(WaitQueue::new)
            .push(waiter);

        // SIGNALLED and QUEUED share one atomic modification order. If signal
        // raced after the recheck but before this RMW, the returned state sees
        // it and this thread removes its own registration instead of sleeping.
        let previous = self.state.fetch_or(STATE_QUEUED, Ordering::AcqRel);
        if previous & STATE_SIGNALED == 0 {
            return true;
        }

        let queue = waiters_guard
            .as_mut()
            .expect("event: registration queue disappeared");
        assert!(
            queue.remove(waiter as *const EventWaiter),
            "event: newly linked registration disappeared"
        );
        if queue.is_empty() {
            *waiters_guard = None;
            self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
        }
        false
    }

    fn unregister(&self, waiter: &EventWaiter) {
        let mut waiters_guard = self.waiters.lock();
        let Some(queue) = waiters_guard.as_mut() else {
            return;
        };
        if !queue.remove(waiter as *const EventWaiter) {
            return;
        }
        if queue.is_empty() {
            *waiters_guard = None;
            self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
        }
    }

    fn claim_one(&self) -> Option<WakeTarget> {
        let mut waiters_guard = self.waiters.lock();

        loop {
            let waiter = waiters_guard.as_mut().and_then(WaitQueue::pop);
            let Some(waiter) = waiter else {
                *waiters_guard = None;
                self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
                return None;
            };

            let group = waiter.group;
            // SAFETY: while this registration is linked or held by the queue
            // lock, its waiting thread cannot finish cleanup and drop the
            // pinned wait group.
            let group_ref = unsafe { &*group };
            if !group_ref.try_claim(waiter.index) {
                continue;
            }

            let target = group_ref.target();
            if waiters_guard.as_ref().is_none_or(WaitQueue::is_empty) {
                *waiters_guard = None;
                self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
            }
            return Some(target);
        }
    }

    fn claim_all(&self) -> WakeBatch {
        let mut batch = WakeBatch::new();
        let mut waiters_guard = self.waiters.lock();

        while let Some(waiter) = waiters_guard.as_mut().and_then(WaitQueue::pop) {
            let group = waiter.group;
            // SAFETY: while this registration is held under the queue lock,
            // the waiting thread cannot complete cleanup and drop its group.
            let group_ref = unsafe { &*group };
            if group_ref.try_claim(waiter.index) {
                batch.push(group);
            }
        }

        *waiters_guard = None;
        self.state.fetch_and(!STATE_QUEUED, Ordering::AcqRel);
        batch
    }
}

impl Default for Event {
    fn default() -> Self {
        Self::new()
    }
}

/// Registers on each event in turn, parks if none claimed the wait, and then
/// unregisters everything before returning the winning index.
fn wait_with_registrations<R: Registrations + ?Sized>(
    events: &[&Event],
    registrations: &R,
    group: &WaitGroup,
) -> usize {
    assert_eq!(
        events.len(),
        registrations.len(),
        "event: registration count mismatch"
    );

    let mut claimed_locally = false;

    for (index, event) in events.iter().enumerate() {
        if group.winner().is_some() {
            break;
        }
        let waiter = registrations.get(index);
        if event.register(waiter) {
            continue;
        }

        if group.try_claim(waiter.index) {
            claimed_locally = true;
            group.target().wake();
        }
        break;
    }

    if !claimed_locally {
        // Keep the park/reschedule handoff non-preemptible. A concurrent wake
        // is still safe before `park_current` because the sequence handshake
        // converts it into a pending wake.
        arch::irqset(false);
        sched::park_current(group.thread, group.park_seq);
    }

    for index in 0..registrations.len() {
        let waiter = registrations.get(index);
        // SAFETY: every registration was constructed from a live borrowed
        // event, which remains borrowed for the duration of this wait.
        unsafe { &*waiter.event }.unregister(waiter);
    }

    group
        .winner()
        .expect("event: wait resumed without a claimed event")
}

#[inline]
fn assert_wait_context() {
    assert!(
        arch::irqstate() && !smp::in_interrupt_context(),
        "event: wait requires thread context with interrupts enabled"
    );
    smp::assert_blockable("event: wait");
}
