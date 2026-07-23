//!
//! # Kernel Timekeeping
//!
//! Architecture-independent timekeeping, deadline management, and per-CPU
//! sleep queues.
//!

use core::{hint::spin_loop, ptr::NonNull, time::Duration};

use intrusive_collections::{KeyAdapter, RBTree, RBTreeLink, UnsafeRef, intrusive_adapter};
use log::info;

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
    const fn new(deadline_ns: u64, target: TimerTarget) -> Self {
        Self {
            link: RBTreeLink::new(),
            deadline_ns,
            order: 0,
            target,
        }
    }

    fn for_event(deadline_ns: u64, event: &Event) -> Self {
        Self::new(deadline_ns, TimerTarget::Event(NonNull::from(event)))
    }

    fn for_thread(deadline_ns: u64, thread: *mut Thread, park_seq: u64) -> Self {
        Self::new(
            deadline_ns,
            TimerTarget::Thread {
                thread: NonNull::new(thread).expect("clock: null sleep thread"),
                park_seq,
            },
        )
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
                let multiplier = (frequency * unit + NSEC_PER_SEC - 1) / NSEC_PER_SEC;
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
        ((u128::from(cycles) * u128::from(self.ns_mult)) >> self.ns_shift)
            .min(u64::MAX as u128) as u64
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
    CLOCKSOURCE.call_once(|| RegisteredClockSource {
        source: clocksource,
        scale: ClockScale::new(frequency_hz),
    });

    let (whole, frac, unit) = format_frequency(frequency_hz);
    info!(
        "clock: active clocksource={} ({}.{:02} {}, rating={})",
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
    info!("clock: registered event timer {}", timer.name());
}

/// Starts local timer delivery on the current CPU.
pub(crate) fn start_cpu() {
    CLOCK_SETUP.call_once(bootstrap_clocks);

    let arm = local_clock().lock().start();

    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }
}

/// Stops local timer delivery on the current CPU.
pub fn stop() {
    let arm = local_clock().lock().stop();
    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }
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
    let duration_ns = duration_to_ns(duration);
    if duration_ns == 0 {
        return event.is_signaled();
    }

    assert!(arch::irqstate() && !smp::in_interrupt_context());
    let current = sched::current_thread();
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing.
    unsafe { &*current }.pin_migration();
    arch::irqset(false);

    let now_ns = monotonic_ns();
    let timeout_event = core::pin::pin!(Event::new());
    let mut timer = core::pin::pin!(Timer::for_event(
        now_ns.saturating_add(duration_ns),
        timeout_event.as_ref().get_ref(),
    ));
    let arm = {
        let mut local = local_clock().lock();
        local.insert_timer(timer.as_mut().get_mut())
    };
    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }

    arch::irqset(true);
    let winner = Event::wait_any(&[event, timeout_event.as_ref().get_ref()]);
    if winner == 1 {
        // SAFETY: this balances the pin acquired before timer registration.
        unsafe { &*current }.unpin_migration();
        return false;
    }

    arch::irqset(false);
    let arm = local_clock().lock().remove_timer(timer.as_ref().get_ref());
    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }
    arch::irqset(true);
    // SAFETY: this balances the pin acquired before timer registration.
    unsafe { &*current }.unpin_migration();
    true
}

/// Returns monotonic nanoseconds derived from the active local counter.
#[inline]
pub fn monotonic_ns() -> u64 {
    let registered = registered_clocksource();
    registered.scale.cycles_to_ns(registered.source.counter())
}

/// Updates the current CPU's scheduler deadline and re-arms the local timer if
/// needed.
pub fn set_scheduler_deadline(deadline_ns: u64) {
    let arm = {
        let mut local = local_clock().lock();
        local.set_scheduler_deadline(deadline_ns)
    };

    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }
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
                assert!(
                    sched::wake(thread.as_ptr(), park_seq),
                    "clock: expired sleep timer had a stale park sequence"
                );
            }
        }
    }

    let arm = {
        let mut local = local_clock().lock();
        local.finish_interrupt(now_ns)
    };

    if let Some(deadline) = arm {
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

    assert!(arch::irqstate() && !smp::in_interrupt_context());

    // Keep timer setup on one CPU so the queue owner, measured timebase,
    // and programmed local deadline always match.
    arch::irqset(false);

    let current = sched::current_thread();
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing.
    let park_seq = unsafe { (&*current).prepare_park() };
    let now_ns = monotonic_ns();
    let mut timer = core::pin::pin!(Timer::for_thread(
        now_ns.saturating_add(ns),
        current,
        park_seq,
    ));

    let arm = {
        let mut local = local_clock().lock();
        local.insert_timer(timer.as_mut().get_mut())
    };

    if let Some(deadline) = arm {
        apply_deadline(deadline);
    }

    sched::park_current(current, park_seq);
    debug_assert!(!timer.as_ref().get_ref().link.is_linked());
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
        "clock: source={} ({}.{:02} {}, timer={})",
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
