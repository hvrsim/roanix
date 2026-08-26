//!
//! # Kernel Timekeeping
//!
//! Architecture-independent timekeeping, deadline management, and per-CPU
//! sleep queues.
//!

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

static CLOCK_SETUP: Once<()> = Once::new();
static CLOCKSOURCE: Once<RegisteredClockSource> = Once::new();
static EVENT_TIMER: Once<&'static dyn EventTimer> = Once::new();
/// Offset from the monotonic clock to the Unix epoch, or `0` when unknown.
static REALTIME_OFFSET_NS: AtomicU64 = AtomicU64::new(0);

const CLOCK_SCALE_MAX_SHIFT: u32 = 48;
const NSEC_PER_SEC: u128 = 1_000_000_000;

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

/// Precomputed fixed-point conversion between nanoseconds and counter cycles.
#[derive(Copy, Clone)]
pub(crate) struct ClockScale {
    ns_mult: u64,
    ns_shift: u32,
    cycles_mult: u64,
    cycles_shift: u32,
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

/// Intrusive sleep timer pinned on the sleeping thread's stack until expiry.
struct Timer {
    link: RBTreeLink,
    deadline_ns: u64,
    order: u64,
    /// CPU whose timer tree owns this entry.
    ///
    /// A sleeper may resume on a different CPU, so removal has to go back to
    /// the tree the timer was inserted into.
    cpu_id: usize,
    target: TimerTarget,
}

#[derive(Copy, Clone)]
struct ExpiredTimer {
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

// SAFETY: timers are only linked/unlinked while protected by the local timer
// lock for their owning CPU, and every target remains live through removal.
unsafe impl Send for Timer {}
// SAFETY: timer fields are immutable while linked except under the local clock
// lock, and target wakeup synchronization is handled by Event or the scheduler.
unsafe impl Sync for Timer {}

intrusive_adapter!(TimerAdapter = UnsafeRef<Timer>: Timer { link: RBTreeLink });

impl<'a> KeyAdapter<'a> for TimerAdapter {
    type Key = (u64, u64);

    fn get_key(&self, timer: &'a Timer) -> Self::Key {
        (timer.deadline_ns, timer.order)
    }
}

struct LocalClockState {
    next_order: u64,
    next_scheduler_deadline_ns: u64,
    next_timer_deadline_ns: u64,
    armed_deadline_ns: u64,
    timers: RBTree<TimerAdapter>,
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

    fn is_linked(&self) -> bool {
        self.link.is_linked()
    }
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

impl LocalClockState {
    fn new() -> Self {
        Self {
            next_order: 0,
            next_scheduler_deadline_ns: 0,
            next_timer_deadline_ns: 0,
            armed_deadline_ns: 0,
            timers: RBTree::new(TimerAdapter::new()),
        }
    }

    fn start(&mut self) -> Option<u64> {
        self.refresh_programmed_deadline()
    }

    fn stop(&mut self) -> Option<u64> {
        self.next_scheduler_deadline_ns = 0;
        self.refresh_programmed_deadline()
    }

    fn insert_timer(&mut self, timer: &mut Timer) -> Option<u64> {
        timer.order = self.next_order;
        self.next_order = self.next_order.wrapping_add(1);
        // SAFETY: the pinned timer outlives its tree membership and the local
        // clock lock serializes all link manipulation.
        self.timers
            .insert(unsafe { UnsafeRef::from_raw(timer as *const Timer) });
        self.refresh_timer_deadline();
        self.refresh_programmed_deadline()
    }

    fn take_expired(&mut self, now_ns: u64) -> Option<ExpiredTimer> {
        let expired = self.expired_front(now_ns)?;
        // SAFETY: `expired_front` returned the currently linked front element
        // while this tree is exclusively borrowed.
        let timer = unsafe { self.timers.cursor_mut_from_ptr(expired).remove() }
            .expect("clock: timer tree lost armed timer");
        Some(ExpiredTimer {
            target: timer.target,
        })
    }

    fn remove_timer(&mut self, timer: &Timer) -> Option<u64> {
        if !timer.link.is_linked() {
            return None;
        }
        // SAFETY: the timer is linked in this exclusively borrowed tree and
        // remains pinned for the duration of this operation.
        unsafe {
            self.timers
                .cursor_mut_from_ptr(timer as *const Timer)
                .remove()
        }
        .expect("clock: linked timer vanished");
        self.refresh_timer_deadline();
        self.refresh_programmed_deadline()
    }

    fn finish_interrupt(&mut self, now_ns: u64) -> Option<u64> {
        self.refresh_timer_deadline();
        if self.next_scheduler_deadline_ns != 0 && self.next_scheduler_deadline_ns <= now_ns {
            self.next_scheduler_deadline_ns = 0;
        }
        self.armed_deadline_ns = 0;
        self.refresh_programmed_deadline()
    }

    fn set_scheduler_deadline(&mut self, deadline_ns: u64) -> Option<u64> {
        self.next_scheduler_deadline_ns = deadline_ns;
        self.refresh_programmed_deadline()
    }

    fn expired_front(&self, now_ns: u64) -> Option<*const Timer> {
        match self.timers.front().get() {
            Some(timer) if timer.deadline_ns <= now_ns => Some(timer as *const Timer),
            _ => None,
        }
    }

    fn refresh_timer_deadline(&mut self) {
        self.next_timer_deadline_ns = self
            .timers
            .front()
            .get()
            .map(|timer| timer.deadline_ns)
            .unwrap_or(0);
    }

    fn combined_deadline(&self) -> u64 {
        match (self.next_scheduler_deadline_ns, self.next_timer_deadline_ns) {
            (0, 0) => 0,
            (0, deadline) | (deadline, 0) => deadline,
            (a, b) => a.min(b),
        }
    }

    fn refresh_programmed_deadline(&mut self) -> Option<u64> {
        let next = self.combined_deadline();
        if next == self.armed_deadline_ns {
            return None;
        }

        self.armed_deadline_ns = next;
        Some(next)
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
    with_local_clock(|local| (local.start(), ()));
}

/// Stops local timer delivery on the current CPU.
pub fn stop() {
    with_local_clock(|local| (local.stop(), ()));
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
    // Pin so the timer stays associated with the CPU that owns its tree.
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
    with_local_clock(|local| (local.insert_timer(timer.as_mut().get_mut()), ()));
    arch::irqset(true);

    let winner = wait_with_timeout_event(events, timeout_event.as_ref().get_ref());

    // Always unlink: on the timeout path the timer already fired, but any
    // other outcome leaves this stack-pinned node in the CPU's timer tree.
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
/// needed.
pub fn set_scheduler_deadline(deadline_ns: u64) {
    with_local_clock(|local| (local.set_scheduler_deadline(deadline_ns), ()));
}

/// Handles a local timer interrupt on the current CPU.
pub fn handle_local_timer_interrupt() {
    let now_ns = monotonic_ns();

    while let Some(expired) = {
        let mut local = local_clock().lock();
        local.take_expired(now_ns)
    } {
        match expired.target {
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

    with_local_clock(|local| (local.finish_interrupt(now_ns), ()));
}

/// Runs `f` against the local clock state and programs any resulting deadline
/// before releasing the lock.
///
/// Programming the hardware inside the critical section keeps
/// `armed_deadline_ns` and the timer register in agreement. Releasing the lock
/// first would re-enable interrupts and let a timer IRQ reprogram the hardware
/// in between, leaving the two permanently out of sync.
fn with_local_clock<R>(f: impl FnOnce(&mut LocalClockState) -> (Option<u64>, R)) -> R {
    let mut local = local_clock().lock();
    let (arm, result) = f(&mut local);
    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }
    result
}

/// Runs `f` against a specific CPU's clock state.
///
/// Only the owning CPU may program its own timer hardware, so a remote update
/// records the new deadline and relies on that CPU re-arming on its next trap.
fn with_clock_for_cpu<R>(
    cpu_id: usize,
    f: impl FnOnce(&mut LocalClockState) -> (Option<u64>, R),
) -> R {
    let local_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
    let mut local = clock_for_cpu(cpu_id).lock();
    let (arm, result) = f(&mut local);
    if let Some(deadline) = arm
        && local_cpu == Some(cpu_id)
    {
        apply_deadline(deadline);
    }
    result
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
    // removal below on the same tree the insertion used.
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

    with_local_clock(|local| (local.insert_timer(timer.as_mut().get_mut()), ()));

    sched::park_current(current, park_seq);

    // The timer normally fires and removes itself, but any other wake path
    // would leave this stack-pinned node linked in the CPU's timer tree and
    // dangling as soon as this frame returns.
    remove_timer(timer.as_ref().get_ref());
    current_ref.unpin_migration();
}

/// Unlinks `timer` from the tree it was inserted into, if still linked.
fn remove_timer(timer: &Timer) {
    if !timer.is_linked() {
        return;
    }

    with_clock_for_cpu(timer.cpu_id, |local| (local.remove_timer(timer), ()));
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

fn apply_deadline(deadline_ns: u64) {
    let timer = event_timer();
    if deadline_ns == 0 {
        timer.stop();
        return;
    }

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
    let ns = (duration.as_secs() as u128 * 1_000_000_000u128)
        .saturating_add(duration.subsec_nanos() as u128);
    ns.min(u64::MAX as u128) as u64
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
