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
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// Interval between scheduler accounting ticks.
pub const STAT_INTERVAL_NS: u64 = 10_000_000;

/// Global tick counter updated from local timer interrupts.
static GLOBAL_TICKS: AtomicU64 = AtomicU64::new(0);
static CLOCK_STARTED: AtomicBool = AtomicBool::new(false);
static CLOCK_LOGGED: Once<()> = Once::new();

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

/// Starts periodic scheduler accounting and local timer delivery.
pub fn start() {
    bootstrap_local_clocks();
    CLOCK_LOGGED.call_once(log_clock_configuration);
    let _ = CLOCK_STARTED.compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire);
    start_secondary();
}

/// Stops local timer delivery on the current CPU.
pub fn stop() {
    CLOCK_STARTED.store(false, Ordering::Release);
    let arm = local_clock().lock().stop();
    if let Some(deadline) = arm {
        apply_deadline(deadline, monotonic_ns());
    }
}

/// Arms timer delivery for the current CPU when timekeeping is active.
pub fn start_secondary() {
    if !CLOCK_STARTED.load(Ordering::Acquire) {
        return;
    }

    let now_ns = monotonic_ns();
    let arm = local_clock().lock().start(now_ns);
    if let Some(deadline) = arm {
        apply_deadline(deadline, now_ns);
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

/// Busy-waits for `ns` nanoseconds using the active local counter.
pub fn delay_ns(ns: u64) {
    if ns == 0 {
        return;
    }

    let start = arch::timer::counter();
    let target = ns_to_cycles(arch::timer::counter_frequency_hz(), ns);
    while arch::timer::counter().wrapping_sub(start) < target {
        spin_loop();
    }
}

/// Puts the current thread to sleep for at least `ns` nanoseconds.
pub fn sleep_ns(ns: u64) {
    if ns == 0 {
        return;
    }

    if !CLOCK_STARTED.load(Ordering::Acquire) {
        panic!("clock: sleep_ns() called while clock infra down");
        delay_ns(ns);
        return;
    }

    if !arch::irqstate() || smp::in_interrupt_context() {
        panic!("clock: sleep_ns() called from outside thread!");
        delay_ns(ns);
        return;
    }

    // Keep timer setup on one CPU so the queue owner, measured timebase, and
    // programmed local deadline always match even under preemption pressure.
    let irq_enabled = arch::irqstate();
    arch::irqset(false);

    let current = sched::current_thread();
    // Idle must never block through the scheduler sleep path; if a caller
    // reaches here while idle is current, fall back to local delay.
    if unsafe { (&*current).is_idle() } {
        if irq_enabled {
            arch::irqset(true);
        }
        panic!("clock: sleep_ns() called from IDLE thread!");
        delay_ns(ns);
        return;
    }

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
    cycles_to_ns(arch::timer::counter_frequency_hz(), arch::timer::counter())
}

/// Handles a local timer interrupt on the current CPU.
pub fn handle_local_timer_interrupt() {
    if !CLOCK_STARTED.load(Ordering::Acquire) {
        arch::timer::stop();
        return;
    }

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

fn bootstrap_local_clocks() {
    for cpu_id in 0..smp::cpu_count() {
        smp::core_local(cpu_id)
            .unwrap_or_else(|| panic!("clock: missing core-local record for cpu{cpu_id}"))
            .clock
            .call_once(PerCpuClock::new);
    }
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
        arch::timer::stop();
        return;
    }

    let min_ns = arch::timer::min_deadline_ns();
    let max_ns = arch::timer::max_deadline_ns();
    let clamped = deadline_ns.max(now_ns.saturating_add(min_ns));
    let capped = now_ns.saturating_add(max_ns);
    arch::timer::set_deadline(clamped.min(capped), now_ns);
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

fn log_clock_configuration() {
    let (whole, frac, unit) = format_frequency(arch::timer::counter_frequency_hz());
    info!(
        "clock: source={} ({}.{:02} {}, timer={})",
        arch::timer::counter_name(),
        whole,
        frac,
        unit,
        arch::timer::timer_name(),
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
