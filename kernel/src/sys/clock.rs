//!
//! # Kernel Timekeeping
//!
//! Architecture-independent timekeeping, deadline management, and per-CPU
//! sleep queues.
//!

use core::hint::spin_loop;
use core::time::Duration;

use intrusive_collections::{intrusive_adapter, KeyAdapter, RBTree, RBTreeLink, UnsafeRef};
use log::info;
use spin::Once;

use crate::{
    arch,
    sys::{
        sched,
        smp::{self, IrqSpinLock},
        thread::Thread,
    },
};
use core::sync::atomic::{AtomicU64, Ordering};

/// Interval between scheduler accounting ticks.
pub const STAT_INTERVAL_NS: u64 = 10_000_000;

/// Global tick counter updated from local timer interrupts.
static GLOBAL_TICKS: AtomicU64 = AtomicU64::new(0);
static CLOCK_SETUP: Once<()> = Once::new();

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
    thread: *mut Thread,
    park_seq: u64,
}

// SAFETY: timers are only linked/unlinked while protected by the local timer
// lock for their owning CPU.
unsafe impl Send for Timer {}
// SAFETY: shared access is read-only and coordinated by the local timer lock.
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
    ticks: u64,
    next_stat_deadline_ns: u64,
    next_timer_deadline_ns: u64,
    armed_deadline_ns: u64,
    timers: RBTree<TimerAdapter>,
}

impl Timer {
    const fn new(thread: *mut Thread, deadline_ns: u64, park_seq: u64) -> Self {
        Self {
            link: RBTreeLink::new(),
            deadline_ns,
            order: 0,
            thread,
            park_seq,
        }
    }
}

impl LocalClockState {
    fn new() -> Self {
        Self {
            next_order: 0,
            ticks: 0,
            next_stat_deadline_ns: 0,
            next_timer_deadline_ns: 0,
            armed_deadline_ns: 0,
            timers: RBTree::new(TimerAdapter::new()),
        }
    }

    fn start(&mut self, now_ns: u64) -> Option<u64> {
        if self.next_stat_deadline_ns == 0 {
            self.next_stat_deadline_ns = now_ns.saturating_add(STAT_INTERVAL_NS);
        }
        self.refresh_programmed_deadline()
    }

    fn stop(&mut self) -> Option<u64> {
        self.next_stat_deadline_ns = 0;
        self.refresh_programmed_deadline()
    }

    fn insert_timer(&mut self, timer: &mut Timer) -> Option<u64> {
        timer.order = self.next_order;
        self.next_order = self.next_order.wrapping_add(1);
        self.timers
            .insert(unsafe { UnsafeRef::from_raw(timer as *const Timer) });
        self.refresh_timer_deadline();
        self.refresh_programmed_deadline()
    }

    fn cancel_timer(&mut self, timer: *const Timer) -> (bool, Option<u64>) {
        if unsafe { !(*timer).link.is_linked() } {
            return (false, None);
        }

        unsafe {
            self.timers.cursor_mut_from_ptr(timer).remove();
        }
        self.refresh_timer_deadline();
        (true, self.refresh_programmed_deadline())
    }

    fn on_interrupt(&mut self, now_ns: u64) -> (u64, u64, u64) {
        let fired = self.advance_stat_ticks(now_ns);
        let local_ticks = self.ticks;

        while let Some(expired) = self.expired_front(now_ns) {
            let timer = unsafe { self.timers.cursor_mut_from_ptr(expired).remove() }
                .expect("clock: timer tree lost armed timer");
            let _ = sched::wake(timer.thread, timer.park_seq);
        }

        self.refresh_timer_deadline();
        let _ = self.refresh_programmed_deadline();
        (fired, local_ticks, self.armed_deadline_ns)
    }

    fn ticks(&self) -> u64 {
        self.ticks
    }

    fn advance_stat_ticks(&mut self, now_ns: u64) -> u64 {
        let mut next = self.next_stat_deadline_ns;
        if next == 0 {
            self.next_stat_deadline_ns = now_ns.saturating_add(STAT_INTERVAL_NS);
            return 0;
        }

        if now_ns < next {
            return 0;
        }

        let mut fired = 0u64;
        while next <= now_ns {
            fired = fired.wrapping_add(1);
            next = next.saturating_add(STAT_INTERVAL_NS);
        }

        self.next_stat_deadline_ns = next;
        self.ticks = self.ticks.wrapping_add(fired);
        fired
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
        match (self.next_stat_deadline_ns, self.next_timer_deadline_ns) {
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
        let (whole, frac, unit) = format_frequency(clocksource.frequency_hz());
        info!(
            "clock: active clocksource={} ({}.{:02} {}, rating={})",
            clocksource.name(),
            whole,
            frac,
            unit,
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

/// Starts periodic scheduler accounting and local timer delivery.
pub fn start() {
    CLOCK_SETUP.call_once(bootstrap_clocks);

    let now_ns = monotonic_ns();
    let arm = local_clock().lock().start(now_ns);

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

/// Returns the global tick count.
pub fn global_ticks() -> u64 {
    GLOBAL_TICKS.load(Ordering::Relaxed)
}

/// Returns the tick count for the current CPU.
pub fn percpu_ticks() -> u64 {
    local_clock().lock().ticks()
}

/// Returns monotonic nanoseconds derived from the active local counter.
pub fn monotonic_ns() -> u64 {
    cycles_to_ns(clocksource().frequency_hz(), clocksource().counter())
}

/// Handles a local timer interrupt on the current CPU.
pub fn handle_local_timer_interrupt() {
    let now_ns = monotonic_ns();
    let (fired, local_ticks, arm) = {
        let mut local = local_clock().lock();
        local.on_interrupt(now_ns)
    };

    let global_base = GLOBAL_TICKS.fetch_add(fired, Ordering::Relaxed);
    for offset in 0..fired {
        let global = global_base.wrapping_add(offset + 1);
        let local = local_ticks.wrapping_sub(fired - offset - 1);
        sched::stat_tick(global, local);
    }

    apply_deadline(arm, now_ns);
}

fn delay_ns(ns: u64) {
    if ns == 0 {
        return;
    }

    let start = clocksource().counter();
    let target = ns_to_cycles(clocksource().frequency_hz(), ns);
    while clocksource().counter().wrapping_sub(start) < target {
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
    let irq_enabled = arch::irqstate();
    arch::irqset(false);

    let current = sched::current_thread();
    let timer_cpu = arch::thiscpu().id;
    let seq = unsafe { (&*current).prepare_park() };
    let now_ns = monotonic_ns();
    let mut timer = core::pin::pin!(Timer::new(current, now_ns.saturating_add(ns), seq));

    let arm = {
        let mut local = clock_for_cpu(timer_cpu).lock();
        local.insert_timer(timer.as_mut().get_mut())
    };

    if let Some(deadline) = arm {
        apply_deadline(deadline, now_ns);
    }

    if irq_enabled {
        arch::irqset(true);
    }

    sched::park_current(seq);
    let now_ns = monotonic_ns();

    let (removed, arm) = {
        // The scheduler may resume this thread on a different CPU, but the
        // timer remains linked in the source CPU's timer tree until it fires or
        // gets canceled.
        let mut local = clock_for_cpu(timer_cpu).lock();
        local.cancel_timer(timer.as_ref().get_ref() as *const Timer)
    };

    if removed {
        if let Some(deadline) = arm {
            apply_deadline_for_cpu(timer_cpu, deadline, now_ns);
        }
    }
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
    if deadline_ns == 0 {
        event_timer().stop();
        return;
    }

    let min_ns = event_timer().min_period_ns();
    let max_ns = event_timer().max_period_ns();
    let delay_ns = deadline_ns.saturating_sub(now_ns).clamp(min_ns, max_ns);

    event_timer().set_oneshot(delay_ns);
}

fn apply_deadline_for_cpu(cpu_id: usize, deadline_ns: u64, now_ns_hint: u64) {
    let this_cpu = arch::thiscpu().id;
    if this_cpu == cpu_id {
        apply_deadline(deadline_ns, now_ns_hint);
        return;
    }

    let _ = smp::send_ipi(
        move || {
            let now_ns = monotonic_ns();
            apply_deadline(deadline_ns, now_ns);
        },
        smp::IpiTarget::Single(cpu_id),
    );
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
