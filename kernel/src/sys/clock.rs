//!
//! # Kernel Timekeeping
//!
//! FreeBSD-inspired split between clocksources (timekeeping) and event timers
//! (interrupt delivery).
//!

use core::hint::spin_loop;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use core::time::Duration;

use log::info;

use crate::{arch, sys::smp::IrqSpinLock};

/// Interval for the dummy scheduler statistics callback.
pub const STAT_INTERVAL_NS: u64 = 10_000_000;

/// Global tick counter updated from timer interrupts.
pub static GLOBAL_TICKS: AtomicU64 = AtomicU64::new(0);

/// Whether the periodic statistics timer is armed.
static CLOCK_STARTED: AtomicBool = AtomicBool::new(false);

/// Absolute nanosecond deadline for the next statistics callback.
static NEXT_STAT_DEADLINE_NS: AtomicU64 = AtomicU64::new(0);

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

/// One-shot interrupt source used to deliver deadlines.
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

struct ClockState {
    active_clocksource: Option<&'static dyn ClockSource>,
    event_timer: Option<&'static dyn EventTimer>,
}

static CLOCK_STATE: IrqSpinLock<ClockState> = IrqSpinLock::new(ClockState::new());

impl ClockState {
    const fn new() -> Self {
        Self {
            active_clocksource: None,
            event_timer: None,
        }
    }
}

/// Registers a clocksource, replacing the active source only if it scores better.
pub fn register_clocksource(clocksource: &'static dyn ClockSource) {
    let mut state = CLOCK_STATE.lock();
    let should_switch = state
        .active_clocksource
        .map(|current| {
            clocksource.rating() > current.rating()
                || (clocksource.rating() == current.rating()
                    && clocksource.frequency_hz() > current.frequency_hz())
        })
        .unwrap_or(true);

    if should_switch {
        state.active_clocksource = Some(clocksource);
        info!(
            "clock: active clocksource={} ({} Hz, rating={})",
            clocksource.name(),
            clocksource.frequency_hz(),
            clocksource.rating()
        );
    } else {
        let active = state
            .active_clocksource
            .expect("clock: active clocksource missing after registration");
        info!(
            "clock: ignored clocksource {} (rating={}), active={}",
            clocksource.name(),
            clocksource.rating(),
            active.name()
        );
    }
}

/// Registers the kernel event timer.
///
/// The event timer is single-assignment on purpose.
pub fn register_event_timer(timer: &'static dyn EventTimer) {
    let mut state = CLOCK_STATE.lock();
    assert!(
        state.event_timer.is_none(),
        "clock: event timer already registered"
    );

    state.event_timer = Some(timer);
    info!("clock: registered event timer {}", timer.name());
}

/// Starts periodic statistics delivery using one-shot deadlines.
pub fn start() {
    if CLOCK_STARTED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return;
    }

    let next = monotonic_ns().saturating_add(STAT_INTERVAL_NS);
    NEXT_STAT_DEADLINE_NS.store(next, Ordering::Release);
    program_deadline(next);
}

/// Stops timer delivery if an event timer is active.
pub fn stop() {
    CLOCK_STARTED.store(false, Ordering::Release);
    NEXT_STAT_DEADLINE_NS.store(0, Ordering::Release);
    event_timer().stop();
}

/// Busy-waits for the requested duration using the active clocksource.
pub fn delay(duration: Duration) {
    delay_ns(duration_to_ns(duration));
}

/// Busy-waits for `ns` nanoseconds using the active clocksource.
pub fn delay_ns(ns: u64) {
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

/// Returns the global tick count.
pub fn global_ticks() -> u64 {
    GLOBAL_TICKS.load(Ordering::Relaxed)
}

/// Returns the tick count for the current CPU.
pub fn percpu_ticks() -> u64 {
    arch::thiscpu().ticks
}

/// Returns monotonic nanoseconds derived from the active clocksource.
pub fn monotonic_ns() -> u64 {
    let source = clocksource();
    cycles_to_ns(source.frequency_hz(), source.counter())
}

/// Handles a timer interrupt from the active event timer.
pub fn handle_timer_interrupt() {
    if !CLOCK_STARTED.load(Ordering::Acquire) {
        return;
    }

    let now = monotonic_ns();
    let mut next = NEXT_STAT_DEADLINE_NS.load(Ordering::Acquire);
    let mut fired = 0u64;

    if next == 0 {
        next = now.saturating_add(STAT_INTERVAL_NS);
    } else {
        while next <= now {
            fired += 1;
            next = next.saturating_add(STAT_INTERVAL_NS);
        }
    }

    if fired == 0 {
        fired = 1;
    }

    NEXT_STAT_DEADLINE_NS.store(next, Ordering::Release);

    let cpu = arch::thiscpu();
    cpu.ticks = cpu.ticks.wrapping_add(fired);
    let global = GLOBAL_TICKS.fetch_add(fired, Ordering::Relaxed);

    for idx in 0..fired {
        scheduler_stat_tick(
            global.wrapping_add(idx + 1),
            cpu.ticks.wrapping_sub(fired - idx - 1),
        );
    }

    program_deadline(next);
}

fn scheduler_stat_tick(global: u64, percpu: u64) {
    info!("clock: stat tick global={} cpu={}", global, percpu);
}

fn program_deadline(deadline_ns: u64) {
    let timer = event_timer();
    let now = monotonic_ns();
    let mut delay = deadline_ns.saturating_sub(now);

    delay = delay.max(timer.min_period_ns());
    delay = delay.min(timer.max_period_ns());
    timer.set_oneshot(delay);
}

fn clocksource() -> &'static dyn ClockSource {
    CLOCK_STATE
        .lock()
        .active_clocksource
        .expect("clock: no active clocksource registered")
}

fn event_timer() -> &'static dyn EventTimer {
    CLOCK_STATE
        .lock()
        .event_timer
        .expect("clock: no event timer registered")
}

fn duration_to_ns(duration: Duration) -> u64 {
    let ns = (duration.as_secs() as u128 * 1_000_000_000u128)
        .saturating_add(duration.subsec_nanos() as u128);
    ns.min(u64::MAX as u128) as u64
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
