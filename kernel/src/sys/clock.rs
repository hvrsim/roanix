//! Monotonic timekeeping, CPU-local callouts, and tickless scheduler deadlines.
//!
//! Each CPU arms its one-shot timer for the earliest scheduler or sleep deadline.
//! Calendar buckets retain full nanosecond deadlines; bucket width never rounds
//! expiration times. Queue changes and hardware writes hold the same IRQ lock.
//! Wakeups run after releasing that lock to preserve scheduler lock ordering.

use alloc::vec::Vec;
use core::{
    hint::spin_loop,
    ptr::NonNull,
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};

use intrusive_collections::{KeyAdapter, RBTree, RBTreeLink, UnsafeRef, intrusive_adapter};
use log::{debug, info};

use crate::{
    arch,
    sys::{
        event::Event,
        sched,
        smp::{self, IrqSpinLock},
        sync::Once,
        thread::Thread,
    },
};

// Timecounter conversion

const CLOCK_SCALE_MAX_SHIFT: u32 = 48;
const NSEC_PER_SEC: u128 = 1_000_000_000;

/// Fixed-point counter conversion using 128-bit intermediates and saturating results.
/// The timekeeping multiplier is rounded to nearest; eventtimer conversion
/// rounds upward so a requested delay never loses a counter cycle.
#[derive(Copy, Clone)]
pub(crate) struct ClockScale {
    ns_mult: u64,
    ns_shift: u32,
    cycles_mult: u64,
    cycles_shift: u32,
}

impl ClockScale {
    /// Builds a conversion scale for `frequency_hz`.
    pub(crate) fn new(frequency_hz: u64) -> Self {
        assert!(frequency_hz != 0, "clock: zero-frequency clocksource");

        let frequency = u128::from(frequency_hz);
        let (ns_mult, ns_shift) = {
            let mut shift = CLOCK_SCALE_MAX_SHIFT;
            loop {
                let unit = 1u128 << shift;
                let multiplier = (NSEC_PER_SEC * unit + frequency / 2) / frequency;
                if (1..=u64::MAX as u128).contains(&multiplier) {
                    break (multiplier as u64, shift);
                }

                assert!(shift != 0, "clock: unable to scale clocksource frequency");
                shift -= 1;
            }
        };
        let (cycles_mult, cycles_shift) = {
            let mut shift = CLOCK_SCALE_MAX_SHIFT;
            loop {
                let unit = 1u128 << shift;
                let multiplier = (frequency * unit).div_ceil(NSEC_PER_SEC);
                if (1..=u64::MAX as u128).contains(&multiplier) {
                    break (multiplier as u64, shift);
                }

                assert!(shift != 0, "clock: unable to scale clocksource frequency");
                shift -= 1;
            }
        };

        Self {
            ns_mult,
            ns_shift,
            cycles_mult,
            cycles_shift,
        }
    }

    #[inline]
    fn cycles_to_ns(self, cycles: u64) -> u64 {
        ((u128::from(cycles) * u128::from(self.ns_mult)) >> self.ns_shift).min(u64::MAX as u128)
            as u64
    }

    /// Converts nanoseconds to counter cycles, rounding up.
    #[inline]
    pub(crate) fn ns_to_cycles(self, ns: u64) -> u64 {
        if ns == 0 {
            return 0;
        }

        let product = u128::from(ns) * u128::from(self.cycles_mult);
        let round = (1u128 << self.cycles_shift) - 1;
        ((product + round) >> self.cycles_shift)
            .max(1)
            .min(u64::MAX as u128) as u64
    }
}

// Calendar callout queue

const BUCKETS: usize = 256;
const BUCKET_WIDTH_NS: u64 = 1_000_000;
const EMPTY: usize = BUCKETS;

intrusive_adapter!(TimerAdapter = UnsafeRef<Timer>: Timer { link: RBTreeLink });

impl<'a> KeyAdapter<'a> for TimerAdapter {
    type Key = (u64, u64);
    fn get_key(&self, timer: &'a Timer) -> Self::Key {
        (timer.deadline_ns, timer.order)
    }
}

/// Exact calendar queue with ordered buckets and a tournament of bucket heads.
///
/// Lookup is O(1); mutations cost O(log m + log BUCKETS) for bucket occupancy m.
/// Full deadlines distinguish calendar turns, and insertion order breaks ties.
/// Fixed storage avoids allocation and redistribution under the IRQ lock.
struct CalendarQueue {
    buckets: [RBTree<TimerAdapter>; BUCKETS],
    // Index 1 is the root; indices BUCKETS..2*BUCKETS are leaves.
    // EMPTY marks a subtree without a timer; index 0 is unused.
    winners: [usize; BUCKETS * 2],
    // Valid only for nonempty buckets; keys are (deadline_ns, insertion_order).
    heads: [(u64, u64); BUCKETS],
    next_order: u64,
}

// SAFETY: queued timers stay pinned and live until removal; all link access
// holds the owning CPU's clock lock. Targets are synchronized event/scheduler
// wakeups, and migration-pinned waiters keep their storage live through dispatch.
unsafe impl Send for CalendarQueue {}

impl CalendarQueue {
    fn new() -> Self {
        Self {
            buckets: core::array::from_fn(|_| RBTree::new(TimerAdapter::new())),
            winners: [EMPTY; BUCKETS * 2],
            heads: [(0, 0); BUCKETS],
            next_order: 0,
        }
    }

    fn bucket(deadline_ns: u64) -> usize {
        (deadline_ns / BUCKET_WIDTH_NS) as usize & (BUCKETS - 1)
    }

    fn key(&self, bucket: usize) -> Option<(u64, u64)> {
        if bucket == EMPTY {
            return None;
        }
        Some(self.heads[bucket])
    }

    /// Refreshes one bucket head and its path to the tournament root.
    fn update(&mut self, bucket: usize) {
        let mut node = BUCKETS + bucket;
        let head = self.buckets[bucket]
            .front()
            .get()
            .map(|e| (e.deadline_ns, e.order));
        if head == self.key(self.winners[node]) {
            return;
        }
        self.winners[node] = if let Some(key) = head {
            self.heads[bucket] = key;
            bucket
        } else {
            EMPTY
        };
        while node > 1 {
            node /= 2;
            let a = self.winners[node * 2];
            let b = self.winners[node * 2 + 1];
            self.winners[node] = match (self.key(a), self.key(b)) {
                (None, _) => b,
                (_, None) => a,
                (Some(ka), Some(kb)) => {
                    if ka <= kb {
                        a
                    } else {
                        b
                    }
                }
            };
        }
    }

    fn next_deadline(&self) -> Option<u64> {
        self.key(self.winners[1]).map(|key| key.0)
    }

    /// # Safety
    /// `timer` must stay at this address and live until removed or popped.
    /// Its deadline must not change while queued. All queue and link access
    /// must be serialized by the owner CPU's clock lock.
    unsafe fn insert(&mut self, timer: &mut Timer) {
        assert!(!timer.link.is_linked(), "clock: callout already armed");
        timer.order = self.next_order;
        self.next_order = self
            .next_order
            .checked_add(1)
            .expect("clock: callout sequence exhausted");
        let bucket = Self::bucket(timer.deadline_ns);
        let changes_head = self
            .key(self.winners[BUCKETS + bucket])
            .is_none_or(|key| (timer.deadline_ns, timer.order) < key);
        // SAFETY: caller guarantees stable storage through removal.
        self.buckets[bucket].insert(unsafe { UnsafeRef::from_raw(timer) });
        if changes_head {
            self.update(bucket);
        }
    }

    /// # Safety
    /// A linked `timer` must belong to this queue, with access serialized by
    /// the owner CPU lock. It must remain live throughout removal.
    unsafe fn remove(&mut self, timer: &Timer) {
        if !timer.link.is_linked() {
            return;
        }
        let bucket = Self::bucket(timer.deadline_ns);
        let changes_head =
            self.key(self.winners[BUCKETS + bucket]) == Some((timer.deadline_ns, timer.order));
        // SAFETY: caller guarantees this timer belongs to the locked queue.
        unsafe { self.buckets[bucket].cursor_mut_from_ptr(timer).remove() }
            .expect("clock: linked callout vanished");
        if changes_head {
            self.update(bucket);
        }
    }

    fn pop_expired(&mut self, now_ns: u64) -> Option<TimerTarget> {
        if self.next_deadline()? > now_ns {
            return None;
        }
        let bucket = self.winners[1];
        let timer = self.buckets[bucket]
            .front_mut()
            .remove()
            .expect("clock: missing calendar head");
        let target = timer.target;
        self.update(bucket);
        Some(target)
    }
}

// Scheduler deadline arbitration

/// A hardware write is distinct from leaving the currently armed timer alone.
enum TimerUpdate {
    Unchanged,
    /// Programs a deadline, or stops delivery when no deadline remains.
    Program(Option<u64>),
}

/// Scheduler, callout, and programmed deadlines protected by the CPU clock lock.
#[derive(Default)]
struct Deadlines {
    scheduler: Option<u64>,
    callout: Option<u64>,
    armed: Option<u64>,
}

impl Deadlines {
    fn set_scheduler(&mut self, deadline: u64) -> TimerUpdate {
        self.scheduler = (deadline != 0).then_some(deadline);
        self.changed()
    }

    fn set_callout(&mut self, deadline: Option<u64>) -> TimerUpdate {
        self.callout = deadline;
        self.changed()
    }

    fn next(&self) -> Option<u64> {
        match (self.scheduler, self.callout) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    fn changed(&mut self) -> TimerUpdate {
        let next = self.next();
        if next == self.armed {
            return TimerUpdate::Unchanged;
        }
        self.armed = next;
        TimerUpdate::Program(next)
    }

    fn finish_interrupt(&mut self, now: u64, callout: Option<u64>) -> Option<u64> {
        self.callout = callout;
        if self.scheduler.is_some_and(|deadline| deadline <= now) {
            self.scheduler = None;
        }
        // Always program/stop after an IRQ, even if the logical deadline has
        // not changed: hardware may have clamped a long delay or be level asserted.
        self.armed = self.next();
        self.armed
    }
}

/// Computes a slice deadline anchored to the last runtime accounting point.
/// Refresh IPIs must not grant extra runtime by restarting the remaining slice.
/// An exhausted slice requests the earliest future interrupt; a due statclock
/// deadline can still take precedence.
pub(crate) fn scheduler_deadline(
    now: u64,
    last_run: u64,
    consumed: u64,
    quantum: u64,
    statclock: u64,
) -> u64 {
    let slice = last_run.saturating_add(quantum.saturating_sub(consumed));
    slice.max(now.saturating_add(1)).min(statclock)
}

static CLOCK_SETUP: Once<()> = Once::new();
static CLOCKSOURCE: Once<RegisteredClockSource> = Once::new();
static EVENT_TIMER: Once<&'static dyn EventTimer> = Once::new();
/// Offset from the monotonic clock to the Unix epoch, or `0` when unknown.
static REALTIME_OFFSET_NS: AtomicU64 = AtomicU64::new(0);

/// Monotonic counter source used for timekeeping and delays.
pub trait ClockSource: Sync {
    /// Human-readable source name.
    fn name(&self) -> &'static str;

    /// Relative quality score. Higher is better.
    fn rating(&self) -> u32;

    /// Counter frequency in Hz.
    fn frequency_hz(&self) -> u64;

    /// Current raw counter value.
    fn counter(&self) -> u64;

    /// Number of significant bits in the raw counter.
    ///
    /// Kernel timekeeping assumes a counter that does not wrap within the
    /// system's lifetime, so only 64-bit sources are accepted. Narrower
    /// hardware must be wrapped in a source that widens the count before it is
    /// registered.
    fn counter_bits(&self) -> u32 {
        64
    }
}

/// One-shot local interrupt source used to deliver deadlines on the current CPU.
pub trait EventTimer: Sync {
    /// Human-readable timer name.
    fn name(&self) -> &'static str;

    /// Minimum armable delay in nanoseconds.
    fn min_period_ns(&self) -> u64;

    /// Maximum armable delay in nanoseconds.
    fn max_period_ns(&self) -> u64;

    /// Programs the next one-shot interrupt after `delay_ns`.
    fn set_oneshot(&self, delay_ns: u64);

    /// Stops timer delivery, if supported.
    fn stop(&self);
}

#[derive(Copy, Clone)]
struct RegisteredClockSource {
    source: &'static dyn ClockSource,
    scale: ClockScale,
    /// Raw counter value observed at registration.
    ///
    /// Subtracting it makes [`monotonic_ns`] start near zero at boot rather
    /// than reporting whatever the hardware counter happened to hold.
    epoch: u64,
}

/// Local timer state stored in [`crate::sys::smp::CoreLocal`].
pub(crate) struct PerCpuClock {
    state: IrqSpinLock<LocalClockState>,
}

impl PerCpuClock {
    fn new() -> Self {
        Self {
            state: IrqSpinLock::new(LocalClockState::new()),
        }
    }
}

/// Stack-pinned callout owned by one CPU until expiry or cancellation.
struct Timer {
    link: RBTreeLink,
    deadline_ns: u64,
    order: u64,
    cpu_id: usize,
    target: TimerTarget,
}

#[derive(Copy, Clone)]
enum TimerTarget {
    Event(NonNull<Event>),
    Thread {
        thread: NonNull<Thread>,
        park_seq: u64,
    },
}

struct LocalClockState {
    deadlines: Deadlines,
    timers: CalendarQueue,
}

impl Timer {
    const fn new(deadline_ns: u64, cpu_id: usize, target: TimerTarget) -> Self {
        Self {
            link: RBTreeLink::new(),
            deadline_ns,
            order: 0,
            cpu_id,
            target,
        }
    }

    fn for_event(deadline_ns: u64, cpu_id: usize, event: &Event) -> Self {
        Self::new(
            deadline_ns,
            cpu_id,
            TimerTarget::Event(NonNull::from(event)),
        )
    }

    fn for_thread(deadline_ns: u64, cpu_id: usize, thread: *mut Thread, park_seq: u64) -> Self {
        Self::new(
            deadline_ns,
            cpu_id,
            TimerTarget::Thread {
                thread: NonNull::new(thread).expect("clock: null sleep thread"),
                park_seq,
            },
        )
    }
}

impl LocalClockState {
    fn new() -> Self {
        Self {
            deadlines: Deadlines::default(),
            timers: CalendarQueue::new(),
        }
    }

    fn insert_timer(&mut self, timer: &mut Timer) -> TimerUpdate {
        // SAFETY: caller pins the timer on its stack until cancellation, with
        // migration disabled, and holds the owner CPU's clock lock.
        unsafe { self.timers.insert(timer) };
        self.deadlines.set_callout(self.timers.next_deadline())
    }

    fn take_expired(&mut self, now_ns: u64) -> Option<TimerTarget> {
        self.timers.pop_expired(now_ns)
    }

    fn remove_timer(&mut self, timer: &Timer) -> TimerUpdate {
        // SAFETY: cancellation verifies the owner CPU and the stack frame stays
        // pinned until this operation has completed under its clock lock.
        unsafe { self.timers.remove(timer) };
        self.deadlines.set_callout(self.timers.next_deadline())
    }
}

/// Registers the active clocksource.
pub fn register_clocksource(clocksource: &'static dyn ClockSource) {
    assert!(
        CLOCKSOURCE.get().is_none(),
        "clock: clocksource already registered"
    );
    let frequency_hz = clocksource.frequency_hz();
    assert_eq!(
        clocksource.counter_bits(),
        64,
        "clock: clocksource {} must present a full 64-bit counter",
        clocksource.name()
    );
    CLOCKSOURCE.call_once(|| RegisteredClockSource {
        source: clocksource,
        scale: ClockScale::new(frequency_hz),
        epoch: clocksource.counter(),
    });

    let (whole, frac, unit) = format_frequency(frequency_hz);
    debug!(
        "clocksource {} registered ({}.{:02} {}, rating {})",
        clocksource.name(),
        whole,
        frac,
        unit,
        clocksource.rating()
    );
}

/// Registers the kernel event timer.
///
/// The event timer is single-assignment on purpose.
pub fn register_event_timer(timer: &'static dyn EventTimer) {
    assert!(
        EVENT_TIMER.get().is_none(),
        "clock: event timer already registered"
    );
    EVENT_TIMER.call_once(|| timer);
    debug!("event timer {} registered", timer.name());
}

/// Starts local timer delivery on the current CPU.
pub(crate) fn start_cpu() {
    CLOCK_SETUP.call_once(bootstrap_clocks);
    // A fresh CPU has no deadlines; explicitly clear any firmware timer.
    apply_deadline(None);
}

/// Clears scheduler delivery on this CPU while preserving armed callouts.
pub fn stop() {
    with_local_clock(|local| local.deadlines.set_scheduler(0));
}

/// Busy-waits for the requested duration using the active local counter.
pub fn delay(duration: Duration) {
    delay_ns(duration_to_ns(duration));
}

/// Puts the current thread to sleep for at least the requested duration.
pub fn sleep(duration: Duration) {
    sleep_ns(duration_to_ns(duration));
}

/// Waits until `event` is notified or `duration` expires.
///
/// Returns `true` when the event won and `false` on timeout.
pub(crate) fn wait_timeout(event: &Event, duration: Duration) -> bool {
    wait_any_timeout(core::slice::from_ref(&event), duration).is_some()
}

/// Waits for any event or a timeout and returns the winning event index.
pub(crate) fn wait_any_timeout(events: &[&Event], duration: Duration) -> Option<usize> {
    assert!(!events.is_empty(), "clock: timed wait requires an event");
    let duration_ns = duration_to_ns(duration);
    if duration_ns == 0 {
        return events.iter().position(|event| event.is_signaled());
    }

    assert!(
        arch::irqstate() && !smp::in_interrupt_context(),
        "clock: timed wait requires thread context with interrupts enabled"
    );
    smp::assert_blockable("clock: timed wait");

    let current = sched::current_thread();
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing.
    let current_ref = unsafe { &*current };
    // Pin so the timer stays associated with the CPU that owns its queue.
    current_ref.pin_migration();
    arch::irqset(false);

    let cpu_id = arch::thiscpu().id;
    let now_ns = monotonic_ns();
    let timeout_event = core::pin::pin!(Event::new());
    let mut timer = core::pin::pin!(Timer::for_event(
        now_ns.saturating_add(duration_ns),
        cpu_id,
        timeout_event.as_ref().get_ref(),
    ));
    with_local_clock(|local| local.insert_timer(timer.as_mut().get_mut()));
    arch::irqset(true);

    let winner = wait_with_timeout_event(events, timeout_event.as_ref().get_ref());

    // Always unlink: on the timeout path the timer already fired, but any
    // other outcome leaves this stack-pinned node in the CPU's timer queue.
    remove_timer(timer.as_ref().get_ref());
    current_ref.unpin_migration();

    (winner != events.len()).then_some(winner)
}

/// Waits on `events` plus a trailing timeout event without heap allocation for
/// the common small-set case.
fn wait_with_timeout_event(events: &[&Event], timeout: &Event) -> usize {
    const INLINE: usize = 8;

    if events.len() < INLINE {
        let mut inline: [&Event; INLINE] = [timeout; INLINE];
        inline[..events.len()].copy_from_slice(events);
        inline[events.len()] = timeout;
        return Event::wait_any(&inline[..events.len() + 1]);
    }

    let mut wait_events = Vec::with_capacity(events.len() + 1);
    wait_events.extend_from_slice(events);
    wait_events.push(timeout);
    Event::wait_any(&wait_events)
}

/// Returns monotonic nanoseconds since the clocksource was registered.
#[inline]
pub fn monotonic_ns() -> u64 {
    let registered = registered_clocksource();
    elapsed_ns(registered)
}

/// Returns monotonic nanoseconds, or zero while timekeeping is unavailable.
///
/// The kernel log timestamps every record, and the first records are written
/// before any clocksource exists. Those callers need a reading that degrades to
/// zero instead of panicking, which would turn a missing timestamp into an
/// unrecoverable early boot failure.
#[inline]
pub fn monotonic_ns_or_zero() -> u64 {
    CLOCKSOURCE.get().map_or(0, elapsed_ns)
}

#[inline]
fn elapsed_ns(registered: &RegisteredClockSource) -> u64 {
    let elapsed = registered.source.counter().wrapping_sub(registered.epoch);
    registered.scale.cycles_to_ns(elapsed)
}

/// Returns wall-clock nanoseconds since the Unix epoch.
///
/// Until a real-time clock driver calls [`set_realtime_offset`] this tracks
/// [`monotonic_ns`], so callers get a consistent but epoch-less timeline
/// rather than silently mislabelled monotonic time.
#[inline]
pub fn realtime_ns() -> u64 {
    monotonic_ns().saturating_add(REALTIME_OFFSET_NS.load(Ordering::Relaxed))
}

/// Returns whether wall-clock time has been established by a driver.
#[inline]
pub fn realtime_is_set() -> bool {
    REALTIME_OFFSET_NS.load(Ordering::Relaxed) != 0
}

/// Records the offset between the monotonic clock and the Unix epoch.
pub fn set_realtime_offset(offset_ns: u64) {
    REALTIME_OFFSET_NS.store(offset_ns, Ordering::Relaxed);
}

/// Updates the current CPU's scheduler deadline and re-arms the local timer if
/// needed. A zero deadline clears scheduler delivery without cancelling callouts.
pub fn set_scheduler_deadline(deadline_ns: u64) {
    with_local_clock(|local| local.deadlines.set_scheduler(deadline_ns));
}

/// Dispatches a bounded batch of expired callouts, then re-arms the local timer.
/// Must run on the owning CPU with interrupts disabled. Migration-pinned waiters
/// cannot resume and destroy their targets before dispatch finishes.
pub fn handle_local_timer_interrupt() {
    let now_ns = monotonic_ns();

    // Bound hard-interrupt work during timeout storms. Remaining overdue
    // callouts re-arm at the hardware minimum, allowing scheduler progress.
    const DISPATCH_BUDGET: usize = 64;
    for _ in 0..DISPATCH_BUDGET {
        let Some(expired) = local_clock().lock().take_expired(now_ns) else {
            break;
        };
        match expired {
            TimerTarget::Event(event) => {
                // SAFETY: the non-null pointer references the pinned timeout
                // event, which remains alive until this signal completes.
                unsafe { event.as_ref() }.signal();
            }
            TimerTarget::Thread { thread, park_seq } => {
                // A stale sequence just means the sleeper was already woken by
                // another path and this expiry lost the race, which is normal.
                let _ = sched::wake(thread.as_ptr(), park_seq);
            }
        }
    }

    with_local_clock(|local| {
        let next = local
            .deadlines
            .finish_interrupt(now_ns, local.timers.next_deadline());
        TimerUpdate::Program(next)
    });
}

/// Runs `f` against the local clock state and programs any resulting deadline
/// before releasing the IRQ lock, keeping cached and hardware deadlines in sync.
fn with_local_clock(f: impl FnOnce(&mut LocalClockState) -> TimerUpdate) {
    let mut local = local_clock().lock();
    if let TimerUpdate::Program(deadline) = f(&mut local) {
        apply_deadline(deadline);
    }
}

fn delay_ns(ns: u64) {
    if ns == 0 {
        return;
    }

    let registered = registered_clocksource();
    let start = registered.source.counter();
    let target = registered.scale.ns_to_cycles(ns);
    while registered.source.counter().wrapping_sub(start) < target {
        spin_loop();
    }
}

fn sleep_ns(ns: u64) {
    if ns == 0 {
        return;
    }

    assert!(
        arch::irqstate() && !smp::in_interrupt_context(),
        "clock: sleep requires thread context with interrupts enabled"
    );
    smp::assert_blockable("clock: sleep");

    // Keep timer setup on one CPU so the queue owner, measured timebase,
    // and programmed local deadline always match.
    arch::irqset(false);

    let current = sched::current_thread();
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing.
    let current_ref = unsafe { &*current };
    // Pin so the sleeper resumes on the CPU that owns its timer, keeping the
    // removal below on the same queue the insertion used.
    current_ref.pin_migration();
    let park_seq = current_ref.prepare_park();
    let cpu_id = arch::thiscpu().id;
    let now_ns = monotonic_ns();
    let mut timer = core::pin::pin!(Timer::for_thread(
        now_ns.saturating_add(ns),
        cpu_id,
        current,
        park_seq,
    ));

    with_local_clock(|local| local.insert_timer(timer.as_mut().get_mut()));

    sched::park_current(current, park_seq);

    // The timer normally fires and removes itself, but any other wake path
    // would leave this stack-pinned node linked in the CPU's timer queue and
    // dangling as soon as this frame returns.
    remove_timer(timer.as_ref().get_ref());
    current_ref.unpin_migration();
}

/// Unlinks `timer` from the queue it was inserted into, if still linked.
fn remove_timer(timer: &Timer) {
    // Migration stays pinned until cancellation completes. Taking the lock
    // also disables IRQs, so a local dispatch cannot outlive this stack frame.
    with_local_clock(|local| {
        assert_eq!(
            timer.cpu_id,
            arch::thiscpu().id,
            "clock: migrated armed callout"
        );
        local.remove_timer(timer)
    });
}

fn bootstrap_clocks() {
    for cpu_id in 0..smp::cpu_count() {
        smp::core_local(cpu_id)
            .unwrap_or_else(|| panic!("clock: missing core-local record for cpu{cpu_id}"))
            .clock
            .call_once(PerCpuClock::new);
    }

    let (whole, frac, unit) = format_frequency(registered_clocksource().source.frequency_hz());
    info!(
        "clocksource {} at {}.{:02} {}, event timer {}",
        registered_clocksource().source.name(),
        whole,
        frac,
        unit,
        event_timer().name(),
    );
}

fn local_clock() -> &'static IrqSpinLock<LocalClockState> {
    clock_for_cpu(arch::thiscpu().id)
}

fn clock_for_cpu(cpu_id: usize) -> &'static IrqSpinLock<LocalClockState> {
    &smp::core_local(cpu_id)
        .unwrap_or_else(|| panic!("clock: missing core-local record for cpu{cpu_id}"))
        .clock
        .get()
        .unwrap_or_else(|| panic!("clock: local clock state not initialized for cpu{cpu_id}"))
        .state
}

fn apply_deadline(deadline_ns: Option<u64>) {
    let timer = event_timer();
    let Some(deadline_ns) = deadline_ns else {
        timer.stop();
        return;
    };

    let now_ns = monotonic_ns();
    let min_ns = timer.min_period_ns();
    let max_ns = timer.max_period_ns();
    let delay_ns = deadline_ns.saturating_sub(now_ns).clamp(min_ns, max_ns);

    timer.set_oneshot(delay_ns);
}

fn format_frequency(freq_hz: u64) -> (u64, u64, &'static str) {
    if freq_hz >= 1_000_000_000 {
        let centi_ghz = ((freq_hz as u128) * 100 + 500_000_000) / 1_000_000_000;
        return ((centi_ghz / 100) as u64, (centi_ghz % 100) as u64, "GHz");
    }

    let centi_khz = ((freq_hz as u128) * 100 + 500) / 1_000;
    ((centi_khz / 100) as u64, (centi_khz % 100) as u64, "KHz")
}

fn duration_to_ns(duration: Duration) -> u64 {
    duration.as_nanos().min(u128::from(u64::MAX)) as u64
}

#[inline]
fn registered_clocksource() -> &'static RegisteredClockSource {
    CLOCKSOURCE
        .get()
        .expect("clock: no active clocksource registered")
}

fn event_timer() -> &'static dyn EventTimer {
    *EVENT_TIMER.get().expect("clock: no event timer registered")
}
