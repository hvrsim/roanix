//!
//! # Kernel Timekeeping
//!
//! Architecture-independent timekeeping, deadline management, and per-CPU
//! sleep queues.
//!

use core::{hint::spin_loop, time::Duration};

use intrusive_collections::{KeyAdapter, RBTree, RBTreeLink, UnsafeRef, intrusive_adapter};
use log::info;

use crate::{
    arch,
    sys::{
        event::Event,
        smp::{self, IrqSpinLock},
        sync::Once,
    },
};

static CLOCK_SETUP: Once<()> = Once::new();
static CLOCKSOURCE: Once<&'static dyn ClockSource> = Once::new();
static EVENT_TIMER: Once<&'static dyn EventTimer> = Once::new();

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
    event: Event,
}

#[derive(Copy, Clone)]
struct ExpiredTimer {
    event: *const Event,
}

// SAFETY: timers are only linked/unlinked while protected by the local timer
// lock for their owning CPU, and the embedded event synchronizes its waiters.
unsafe impl Send for Timer {}
// SAFETY: timer fields are immutable while linked except under the local clock
// lock, and the embedded event provides its own synchronization.
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
    const fn new(deadline_ns: u64) -> Self {
        Self {
            link: RBTreeLink::new(),
            deadline_ns,
            order: 0,
            event: Event::new(),
        }
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
            event: &timer.event,
        })
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
    CLOCKSOURCE.call_once(|| clocksource);

    let (whole, frac, unit) = format_frequency(clocksource.frequency_hz());
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

    let now_ns = monotonic_ns();
    let arm = local_clock().lock().start();

    if let Some(deadline) = arm {
        apply_deadline(deadline, now_ns);
    }
}

/// Stops local timer delivery on the current CPU.
pub fn stop() {
    let arm = local_clock().lock().stop();
    if let Some(deadline) = arm {
        apply_deadline(deadline, monotonic_ns());
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

/// Returns monotonic nanoseconds derived from the active local counter.
pub fn monotonic_ns() -> u64 {
    let source = clocksource();
    cycles_to_ns(source.frequency_hz(), source.counter())
}

/// Updates the current CPU's scheduler deadline and re-arms the local timer if
/// needed.
pub fn set_scheduler_deadline(deadline_ns: u64) {
    let now_ns = monotonic_ns();
    let arm = {
        let mut local = local_clock().lock();
        local.set_scheduler_deadline(deadline_ns)
    };

    if let Some(deadline) = arm {
        apply_deadline(deadline, now_ns);
    }
}

/// Handles a local timer interrupt on the current CPU.
pub fn handle_local_timer_interrupt() {
    let now_ns = monotonic_ns();

    while let Some(expired) = {
        let mut local = local_clock().lock();
        local.take_expired(now_ns)
    } {
        // SAFETY: the timer remains pinned on the sleeping thread's stack
        // until this signal completes and allows that thread to return.
        unsafe { &*expired.event }.signal();
    }

    let arm = {
        let mut local = local_clock().lock();
        local.finish_interrupt(now_ns)
    };

    if let Some(deadline) = arm {
        apply_deadline(deadline, now_ns);
    }
}

fn delay_ns(ns: u64) {
    if ns == 0 {
        return;
    }

    let source = clocksource();
    let start = source.counter();
    let target = ns_to_cycles(source.frequency_hz(), ns);
    while source.counter().wrapping_sub(start) < target {
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

    let now_ns = monotonic_ns();
    let mut timer = core::pin::pin!(Timer::new(now_ns.saturating_add(ns)));

    let arm = {
        let mut local = local_clock().lock();
        local.insert_timer(timer.as_mut().get_mut())
    };

    if let Some(deadline) = arm {
        apply_deadline(deadline, now_ns);
    }

    // The persistent event signal handles expiry racing with entry into wait.
    arch::irqset(true);
    timer.as_ref().get_ref().event.wait();
}

fn bootstrap_clocks() {
    for cpu_id in 0..smp::cpu_count() {
        smp::core_local(cpu_id)
            .unwrap_or_else(|| panic!("clock: missing core-local record for cpu{cpu_id}"))
            .clock
            .call_once(PerCpuClock::new);
    }

    let (whole, frac, unit) = format_frequency(clocksource().frequency_hz());
    info!(
        "clock: source={} ({}.{:02} {}, timer={})",
        clocksource().name(),
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

fn apply_deadline(deadline_ns: u64, now_ns: u64) {
    let timer = event_timer();
    if deadline_ns == 0 {
        timer.stop();
        return;
    }

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

fn clocksource() -> &'static dyn ClockSource {
    *CLOCKSOURCE
        .get()
        .expect("clock: no active clocksource registered")
}

fn event_timer() -> &'static dyn EventTimer {
    *EVENT_TIMER.get().expect("clock: no event timer registered")
}

fn ns_to_cycles(freq_hz: u64, ns: u64) -> u64 {
    if ns == 0 {
        return 0;
    }

    let cycles = (ns as u128)
        .saturating_mul(freq_hz as u128)
        .saturating_add(999_999_999)
        / 1_000_000_000;
    cycles.max(1).min(u64::MAX as u128) as u64
}

fn cycles_to_ns(freq_hz: u64, cycles: u64) -> u64 {
    ((cycles as u128).saturating_mul(1_000_000_000u128) / freq_hz as u128).min(u64::MAX as u128)
        as u64
}
