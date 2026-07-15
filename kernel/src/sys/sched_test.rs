//!
//! # Scheduler Stress Test
//!
//! High-pressure, multicore scheduler exercise that mixes CPU-bound work,
//! blocking wakeups, lock contention, and IPI traffic.
//!
//! This is designed to run on boot once SMP is online so scheduler issues
//! surface quickly with actionable logging.
//!
#![allow(clippy::all)]

use alloc::{boxed::Box, vec::Vec};
use core::{
    hint::spin_loop,
    sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
    time::Duration,
};

use log::{error, info, warn};
use spin::Once;

use crate::{
    arch,
    sys::{clock, sched, smp, sync::Mutex},
};

const HOG_THREADS_PER_CPU: usize = 2;
const INTERACTIVE_THREADS_PER_CPU: usize = 1;
const ITHREAD_THREADS_PER_TWO_CPUS: usize = 1;
const MAX_HOG_WORKERS: usize = 32;
const MAX_INTERACTIVE_WORKERS: usize = 20;
const MAX_ITHREAD_WORKERS: usize = 2;

const STRESS_DURATION_SECS: u64 = 12;
const ITHREAD_PHASE_SECS: u64 = 2;
const WATCHDOG_POLL_MS: u64 = 200;
const WATCHDOG_LOG_MS: u64 = 1_000;
const TOTAL_TIMEOUT_SECS: u64 = 120;
const LOG_WORKER_LIFECYCLE: bool = true;
const ENABLE_IPI_PROBE: bool = false;
const SLEEP_TAIL_GUARD_NS: u64 = 5_000_000;

const HOG_SLEEP_MIN_US: u64 = 20;
const HOG_SLEEP_MAX_US: u64 = 200;
const INTERACTIVE_SLEEP_MIN_US: u64 = 200;
const INTERACTIVE_SLEEP_MAX_US: u64 = 1_600;
#[cfg(not(target_arch = "riscv64"))]
const INTERACTIVE_LAG_WARN_NS: u64 = 30_000_000;
#[cfg(not(target_arch = "riscv64"))]
const INTERACTIVE_LAG_FAIL_NS: u64 = 250_000_000;
#[cfg(target_arch = "riscv64")]
const INTERACTIVE_LAG_WARN_NS: u64 = 500_000_000;
#[cfg(target_arch = "riscv64")]
const INTERACTIVE_LAG_FAIL_NS: u64 = 2_500_000_000;
const TOKEN_MIX: u64 = 0x9E37_79B9_7F4A_7C15;

static SCHED_TEST_STATE: Once<&'static SchedulerStressState> = Once::new();
static SCHED_TEST_QUEUED: AtomicBool = AtomicBool::new(false);

#[derive(Copy, Clone, Eq, PartialEq)]
enum WorkerKind {
    Hog,
    Interactive,
    Ithread,
}

impl WorkerKind {
    fn label(self) -> &'static str {
        match self {
            Self::Hog => "hog",
            Self::Interactive => "interactive",
            Self::Ithread => "ithread",
        }
    }

    fn seed_bias(self) -> u64 {
        match self {
            Self::Hog => 0xA1A1_A1A1_A1A1_A1A1,
            Self::Interactive => 0xB2B2_B2B2_B2B2_B2B2,
            Self::Ithread => 0xC3C3_C3C3_C3C3_C3C3,
        }
    }
}

#[derive(Copy, Clone)]
struct StressConfig {
    cpu_count: usize,
    hog_workers: usize,
    interactive_workers: usize,
    ithread_workers: usize,
    total_workers: usize,
    duration_ns: u64,
    timeout_ns: u64,
}

impl StressConfig {
    fn for_system(cpu_count: usize) -> Self {
        let cpu_count = cpu_count.max(1);
        let hog_workers = (cpu_count * HOG_THREADS_PER_CPU)
            .min(MAX_HOG_WORKERS)
            .max(2);
        let interactive_workers = (cpu_count * INTERACTIVE_THREADS_PER_CPU)
            .min(MAX_INTERACTIVE_WORKERS)
            .max(1);
        let ithread_workers = (((cpu_count + 1) / 2) * ITHREAD_THREADS_PER_TWO_CPUS)
            .min(MAX_ITHREAD_WORKERS)
            .max(1);
        let total_workers = hog_workers + interactive_workers + ithread_workers;

        Self {
            cpu_count,
            hog_workers,
            interactive_workers,
            ithread_workers,
            total_workers,
            duration_ns: duration_to_ns(Duration::from_secs(STRESS_DURATION_SECS)),
            timeout_ns: duration_to_ns(Duration::from_secs(TOTAL_TIMEOUT_SECS)),
        }
    }
}

struct SharedState {
    checksum: u64,
    lock_ops: u64,
    handoffs: u64,
    last_owner: usize,
}

impl SharedState {
    const fn new() -> Self {
        Self {
            checksum: 0,
            lock_ops: 0,
            handoffs: 0,
            last_owner: usize::MAX,
        }
    }
}

struct SchedulerStressState {
    cfg: StressConfig,
    done: AtomicUsize,
    progress: AtomicU64,
    migrations: AtomicU64,
    lock_ops: AtomicU64,
    lock_misses: AtomicU64,
    sleep_ops: AtomicU64,
    ipi_queued: AtomicU64,
    ipi_handled: AtomicU64,
    wake_lag_warn: AtomicU64,
    wake_lag_fail: AtomicU64,
    max_wake_lag_ns: AtomicU64,
    errors: AtomicUsize,
    per_cpu_runs: Box<[AtomicU64]>,
    per_cpu_migrations: Box<[AtomicU64]>,
    shared: Mutex<SharedState>,
}

impl SchedulerStressState {
    fn new(cfg: StressConfig) -> Self {
        Self {
            cfg,
            done: AtomicUsize::new(0),
            progress: AtomicU64::new(0),
            migrations: AtomicU64::new(0),
            lock_ops: AtomicU64::new(0),
            lock_misses: AtomicU64::new(0),
            sleep_ops: AtomicU64::new(0),
            ipi_queued: AtomicU64::new(0),
            ipi_handled: AtomicU64::new(0),
            wake_lag_warn: AtomicU64::new(0),
            wake_lag_fail: AtomicU64::new(0),
            max_wake_lag_ns: AtomicU64::new(0),
            errors: AtomicUsize::new(0),
            per_cpu_runs: boxed_atomic_u64_slice(cfg.cpu_count),
            per_cpu_migrations: boxed_atomic_u64_slice(cfg.cpu_count),
            shared: Mutex::new(SharedState::new()),
        }
    }

    #[inline]
    fn record_progress(&self, cpu_id: usize) {
        self.progress.fetch_add(1, Ordering::Relaxed);
        if cpu_id < self.cfg.cpu_count {
            self.per_cpu_runs[cpu_id].fetch_add(1, Ordering::Relaxed);
        }
    }
}

struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self { state: seed.max(1) }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn in_range_inclusive(&mut self, low: u64, high: u64) -> u64 {
        if high <= low {
            return low;
        }
        low + (self.next_u64() % (high - low + 1))
    }
}

/// Queues the scheduler stress harness to run once scheduling begins.
pub fn schedule() {
    if SCHED_TEST_QUEUED.swap(true, Ordering::AcqRel) {
        return;
    }

    let cfg = StressConfig::for_system(smp::cpu_count());
    let state = Box::leak(Box::new(SchedulerStressState::new(cfg)));
    let state: &'static SchedulerStressState = state;
    SCHED_TEST_STATE.call_once(move || state);

    info!(
        "sched_test: queue stress cpus={} workers={} (hog={}, interactive={}, ithread={}) duration={}s",
        cfg.cpu_count,
        cfg.total_workers,
        cfg.hog_workers,
        cfg.interactive_workers,
        cfg.ithread_workers,
        STRESS_DURATION_SECS,
    );
    info!(
        "sched_test: tuning lag_warn={}ms lag_fail={}ms require_migrations={}",
        ns_to_ms(INTERACTIVE_LAG_WARN_NS),
        ns_to_ms(INTERACTIVE_LAG_FAIL_NS),
        migration_required_for(cfg.cpu_count),
    );

    if cfg.cpu_count < 2 {
        warn!("sched_test: only one CPU online; multicore checks will be limited");
    }

    let tid = sched::run(move || coordinator(state));
    info!("sched_test: coordinator queued as tid={tid}");
}

fn coordinator(state: &'static SchedulerStressState) {
    let coordinator_cpu = arch::thiscpu().id;
    let start_ns = clock::monotonic_ns();
    let phase1_stop_ns = start_ns.saturating_add(state.cfg.duration_ns);
    let phase1_workers = state.cfg.hog_workers + state.cfg.interactive_workers;

    info!(
        "sched_test: coordinator start cpu{} stop_ns={} timeout={}s",
        coordinator_cpu, phase1_stop_ns, TOTAL_TIMEOUT_SECS
    );

    let phase1_workers_spawned = spawn_timeshare_workers(state, phase1_stop_ns);
    info!(
        "sched_test: phase1 started workers={} total={} (cpu{})",
        phase1_workers_spawned, state.cfg.total_workers, coordinator_cpu
    );

    monitor_until_complete(state, start_ns, phase1_workers, "phase1");

    if state.cfg.ithread_workers != 0 {
        let phase2_start_ns = clock::monotonic_ns();
        let phase2_stop_ns =
            phase2_start_ns.saturating_add(duration_to_ns(Duration::from_secs(ITHREAD_PHASE_SECS)));
        info!(
            "sched_test: starting isolated ithread phase workers={} duration={}s",
            state.cfg.ithread_workers, ITHREAD_PHASE_SECS
        );
        spawn_ithread_workers(state, phase2_stop_ns, phase1_workers);
        monitor_until_complete(state, start_ns, state.cfg.total_workers, "phase2");
    }

    finalize(state, start_ns);
}

fn spawn_timeshare_workers(state: &'static SchedulerStressState, stop_ns: u64) -> usize {
    for id in 0..state.cfg.hog_workers {
        let tid = sched::run(move || run_worker(state, id, WorkerKind::Hog, stop_ns));
        if id < 4 || id % 8 == 0 {
            info!("sched_test: spawn worker#{id} kind=hog tid={tid}");
        }
    }

    let interactive_base = state.cfg.hog_workers;
    for offset in 0..state.cfg.interactive_workers {
        let id = interactive_base + offset;
        let tid = sched::run(move || run_worker(state, id, WorkerKind::Interactive, stop_ns));
        if id < 4 || id % 8 == 0 {
            info!("sched_test: spawn worker#{id} kind=interactive tid={tid}");
        }
    }

    let phase1 = state.cfg.hog_workers + state.cfg.interactive_workers;
    assert!(
        phase1 <= state.cfg.total_workers,
        "sched_test: invalid phase1 worker count"
    );
    phase1
}

fn spawn_ithread_workers(state: &'static SchedulerStressState, stop_ns: u64, start_id: usize) {
    for offset in 0..state.cfg.ithread_workers {
        let id = start_id + offset;
        let tid = sched::create_ithread(
            move |arg| run_worker(state, arg as usize, WorkerKind::Ithread, stop_ns),
            id as u64,
        );
        info!("sched_test: spawn worker#{id} kind=ithread tid={tid}");
    }
}

fn monitor_until_complete(
    state: &SchedulerStressState,
    start_ns: u64,
    expected_done: usize,
    phase: &'static str,
) {
    let total = expected_done;
    let mut last_log_ns = start_ns;

    loop {
        let now = clock::monotonic_ns();
        let done = state.done.load(Ordering::Acquire);
        if done >= total {
            return;
        }

        let progress = state.progress.load(Ordering::Acquire);
        if now.saturating_sub(start_ns) > state.cfg.timeout_ns {
            panic!(
                "sched_test: timeout phase={phase} done={done}/{total} elapsed={}ms progress={progress}",
                ns_to_ms(now.saturating_sub(start_ns))
            );
        }

        if now.saturating_sub(last_log_ns) >= duration_to_ns(Duration::from_millis(WATCHDOG_LOG_MS))
        {
            last_log_ns = now;
            log_periodic_snapshot(state, done, total, now.saturating_sub(start_ns));
        }

        // Keep the coordinator out of the sleep/wakeup path so monitor
        // liveness does not depend on timer wakeups under extreme load.
        clock::delay(Duration::from_millis(WATCHDOG_POLL_MS));
    }
}

fn log_periodic_snapshot(state: &SchedulerStressState, done: usize, total: usize, elapsed_ns: u64) {
    info!(
        "sched_test: progress done={done}/{total} elapsed={}ms ops={} lock_ops={} lock_miss={} sleeps={} migrations={} ipi={}/{} lag_max={}us lag_warn={} lag_fail={}",
        ns_to_ms(elapsed_ns),
        state.progress.load(Ordering::Relaxed),
        state.lock_ops.load(Ordering::Relaxed),
        state.lock_misses.load(Ordering::Relaxed),
        state.sleep_ops.load(Ordering::Relaxed),
        state.migrations.load(Ordering::Relaxed),
        state.ipi_handled.load(Ordering::Relaxed),
        state.ipi_queued.load(Ordering::Relaxed),
        state.max_wake_lag_ns.load(Ordering::Relaxed) / 1_000,
        state.wake_lag_warn.load(Ordering::Relaxed),
        state.wake_lag_fail.load(Ordering::Relaxed),
    );
}

fn finalize(state: &SchedulerStressState, start_ns: u64) {
    let elapsed_ns = clock::monotonic_ns().saturating_sub(start_ns);
    let done = state.done.load(Ordering::Acquire);
    let total = state.cfg.total_workers;

    let (shared_ops, shared_handoffs, checksum) = {
        let shared = state.shared.lock();
        (shared.lock_ops, shared.handoffs, shared.checksum)
    };

    let lock_ops = state.lock_ops.load(Ordering::Acquire);
    let ipi_handled = state.ipi_handled.load(Ordering::Acquire);
    let migrations = state.migrations.load(Ordering::Acquire);
    let lag_fail = state.wake_lag_fail.load(Ordering::Acquire);
    let max_lag = state.max_wake_lag_ns.load(Ordering::Acquire);

    let mut active_cpus = 0usize;
    for cpu_id in 0..state.cfg.cpu_count {
        let runs = state.per_cpu_runs[cpu_id].load(Ordering::Relaxed);
        let incoming = state.per_cpu_migrations[cpu_id].load(Ordering::Relaxed);
        if runs != 0 {
            active_cpus += 1;
        }
        info!("sched_test: cpu{cpu_id} runs={runs} incoming_migrations={incoming}");
    }

    info!(
        "sched_test: summary done={done}/{total} elapsed={}ms total_ops={} lock_ops={} lock_miss={} shared_lock_ops={} handoffs={} checksum=0x{:016x} sleeps={} migrations={} ipi={}/{} lag_max={}us",
        ns_to_ms(elapsed_ns),
        state.progress.load(Ordering::Relaxed),
        lock_ops,
        state.lock_misses.load(Ordering::Relaxed),
        shared_ops,
        shared_handoffs,
        checksum,
        state.sleep_ops.load(Ordering::Relaxed),
        migrations,
        ipi_handled,
        state.ipi_queued.load(Ordering::Relaxed),
        max_lag / 1_000,
    );

    if done != total {
        state.errors.fetch_add(1, Ordering::Relaxed);
        error!("sched_test: incomplete completion done={done}/{total}");
    }

    if lock_ops != shared_ops {
        state.errors.fetch_add(1, Ordering::Relaxed);
        error!(
            "sched_test: shared lock accounting mismatch lock_ops={} shared_lock_ops={}",
            lock_ops, shared_ops
        );
    }

    if state.cfg.cpu_count > 1 {
        if active_cpus < 2 {
            state.errors.fetch_add(1, Ordering::Relaxed);
            error!("sched_test: workload did not spread across CPUs (active={active_cpus})");
        }

        let require_migrations = migration_required_for(state.cfg.cpu_count);
        if require_migrations && migrations == 0 {
            state.errors.fetch_add(1, Ordering::Relaxed);
            error!("sched_test: no CPU migrations observed under multicore stress");
        }

        if ENABLE_IPI_PROBE && ipi_handled == 0 {
            state.errors.fetch_add(1, Ordering::Relaxed);
            error!("sched_test: no IPI callbacks were executed");
        }
    }

    if lag_fail != 0 {
        state.errors.fetch_add(1, Ordering::Relaxed);
        error!(
            "sched_test: {} interactive wakeups exceeded {}ms",
            lag_fail,
            ns_to_ms(INTERACTIVE_LAG_FAIL_NS)
        );
    }

    let errors = state.errors.load(Ordering::Acquire);
    if errors != 0 {
        panic!("sched_test: FAILED with {errors} invariant violation(s)");
    }

    info!("sched_test: PASS ULE multicore stress complete");
}

fn run_worker(
    state: &'static SchedulerStressState,
    worker_id: usize,
    kind: WorkerKind,
    stop_ns: u64,
) {
    let start_cpu = arch::thiscpu().id;
    let mut rng = XorShift64::new(
        ((worker_id as u64).wrapping_add(1) << 32) ^ stop_ns ^ kind.seed_bias() ^ start_cpu as u64,
    );

    if LOG_WORKER_LIFECYCLE && (worker_id < 8 || worker_id % 16 == 0 || kind == WorkerKind::Ithread)
    {
        info!(
            "sched_test: worker#{worker_id} kind={} start cpu{}",
            kind.label(),
            start_cpu
        );
    }

    let mut iter = 0u64;
    let mut last_cpu = start_cpu;

    while clock::monotonic_ns() < stop_ns {
        let cpu_id = arch::thiscpu().id;
        if cpu_id != last_cpu {
            state.migrations.fetch_add(1, Ordering::Relaxed);
            if cpu_id < state.cfg.cpu_count {
                state.per_cpu_migrations[cpu_id].fetch_add(1, Ordering::Relaxed);
            }
            last_cpu = cpu_id;
        }

        state.record_progress(cpu_id);

        match kind {
            WorkerKind::Hog => do_hog_step(state, &mut rng, worker_id, iter, stop_ns),
            WorkerKind::Interactive => {
                do_interactive_step(state, &mut rng, worker_id, iter, stop_ns)
            }
            WorkerKind::Ithread => do_ithread_step(state, &mut rng, worker_id, iter),
        }

        iter = iter.wrapping_add(1);
    }

    let final_cpu = arch::thiscpu().id;
    let finished = state.done.fetch_add(1, Ordering::AcqRel) + 1;
    if LOG_WORKER_LIFECYCLE && (worker_id < 8 || worker_id % 16 == 0 || kind == WorkerKind::Ithread)
    {
        info!(
            "sched_test: worker#{worker_id} kind={} finish cpu{} iter={} done={}/{}",
            kind.label(),
            final_cpu,
            iter,
            finished,
            state.cfg.total_workers
        );
    }
}

fn do_hog_step(
    state: &SchedulerStressState,
    rng: &mut XorShift64,
    worker_id: usize,
    iter: u64,
    stop_ns: u64,
) {
    let spins = rng.in_range_inclusive(64, 1024);
    for _ in 0..spins {
        spin_loop();
    }

    if iter & 0x3 == 0 {
        touch_shared_mutex(state, worker_id, iter, WorkerKind::Hog);
    }

    if iter & 0xF == 0 {
        let us = rng.in_range_inclusive(HOG_SLEEP_MIN_US, HOG_SLEEP_MAX_US);
        let req_ns = us.saturating_mul(1_000);
        state.sleep_ops.fetch_add(1, Ordering::Relaxed);
        if should_block_sleep(stop_ns, req_ns) {
            clock::sleep(Duration::from_micros(us));
        } else {
            clock::delay(Duration::from_micros(us));
        }
    }

    if ENABLE_IPI_PROBE && (iter & 0x7F == 0) {
        let target = if state.cfg.cpu_count > 1 {
            smp::IpiTarget::All
        } else {
            smp::IpiTarget::Single(0)
        };
        let queued = smp::send_ipi(ipi_probe_callback, target) as u64;
        if queued != 0 {
            state.ipi_queued.fetch_add(queued, Ordering::Relaxed);
        }
    }
}

fn do_interactive_step(
    state: &SchedulerStressState,
    rng: &mut XorShift64,
    worker_id: usize,
    iter: u64,
    stop_ns: u64,
) {
    let req_us = rng.in_range_inclusive(INTERACTIVE_SLEEP_MIN_US, INTERACTIVE_SLEEP_MAX_US);
    let req_ns = req_us.saturating_mul(1_000);
    let before_ns = clock::monotonic_ns();
    state.sleep_ops.fetch_add(1, Ordering::Relaxed);
    if should_block_sleep(stop_ns, req_ns) {
        clock::sleep(Duration::from_micros(req_us));
    } else {
        clock::delay(Duration::from_micros(req_us));
    }
    let elapsed_ns = clock::monotonic_ns().saturating_sub(before_ns);
    let lag_ns = elapsed_ns.saturating_sub(req_ns);

    update_max_atomic(&state.max_wake_lag_ns, lag_ns);
    if lag_ns > INTERACTIVE_LAG_WARN_NS {
        state.wake_lag_warn.fetch_add(1, Ordering::Relaxed);
    }
    if lag_ns > INTERACTIVE_LAG_FAIL_NS {
        state.wake_lag_fail.fetch_add(1, Ordering::Relaxed);
    }

    touch_shared_mutex(state, worker_id, iter, WorkerKind::Interactive);

    if ENABLE_IPI_PROBE && (iter & 0x3F == 0) {
        let target_cpu = (rng.next_u64() as usize) % state.cfg.cpu_count.max(1);
        let queued = smp::send_ipi(ipi_probe_callback, smp::IpiTarget::Single(target_cpu)) as u64;
        if queued != 0 {
            state.ipi_queued.fetch_add(queued, Ordering::Relaxed);
        }
    }
}

fn do_ithread_step(
    state: &SchedulerStressState,
    rng: &mut XorShift64,
    worker_id: usize,
    iter: u64,
) {
    let spins = rng.in_range_inclusive(32, 256);
    for _ in 0..spins {
        spin_loop();
    }

    touch_shared_mutex(state, worker_id, iter, WorkerKind::Ithread);

    if iter & 0x1F == 0 {
        // Keep ithreads hot to stress class priority interactions without
        // parking this class through timer sleep paths.
        let extra_spins = rng.in_range_inclusive(128, 512);
        for _ in 0..extra_spins {
            spin_loop();
        }
    }
}

fn touch_shared_mutex(state: &SchedulerStressState, worker_id: usize, iter: u64, kind: WorkerKind) {
    let token =
        ((worker_id as u64).wrapping_shl(32)) ^ iter.wrapping_mul(TOKEN_MIX) ^ kind.seed_bias();
    if let Some(mut shared) = state.shared.try_lock() {
        shared.checksum = shared.checksum.rotate_left(7) ^ token;
        shared.lock_ops = shared.lock_ops.wrapping_add(1);
        if shared.last_owner != worker_id {
            shared.handoffs = shared.handoffs.wrapping_add(1);
            shared.last_owner = worker_id;
        }
        state.lock_ops.fetch_add(1, Ordering::Relaxed);
    } else {
        state.lock_misses.fetch_add(1, Ordering::Relaxed);
    }
}

fn ipi_probe_callback() {
    if let Some(state) = SCHED_TEST_STATE.get().copied() {
        state.ipi_handled.fetch_add(1, Ordering::Relaxed);
        state.progress.fetch_add(1, Ordering::Relaxed);
    }
}

fn boxed_atomic_u64_slice(len: usize) -> Box<[AtomicU64]> {
    let mut counters = Vec::with_capacity(len);
    for _ in 0..len {
        counters.push(AtomicU64::new(0));
    }
    counters.into_boxed_slice()
}

fn update_max_atomic(slot: &AtomicU64, candidate: u64) {
    let mut current = slot.load(Ordering::Relaxed);
    while candidate > current {
        match slot.compare_exchange_weak(current, candidate, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return,
            Err(observed) => current = observed,
        }
    }
}

fn duration_to_ns(duration: Duration) -> u64 {
    let ns = (duration.as_secs() as u128)
        .saturating_mul(1_000_000_000u128)
        .saturating_add(duration.subsec_nanos() as u128);
    ns.min(u64::MAX as u128) as u64
}

fn ns_to_ms(ns: u64) -> u64 {
    ns / 1_000_000
}

fn should_block_sleep(stop_ns: u64, req_ns: u64) -> bool {
    let now_ns = clock::monotonic_ns();
    let remaining_ns = stop_ns.saturating_sub(now_ns);
    remaining_ns > req_ns.saturating_add(SLEEP_TAIL_GUARD_NS)
}

fn migration_required_for(cpu_count: usize) -> bool {
    if cfg!(target_arch = "riscv64") {
        // QEMU TCG on RISC-V can run with very limited cross-CPU stealing,
        // especially on low vCPU counts, so only require migration signal
        // once there is enough CPU topology to make balancing meaningful.
        cpu_count >= 4
    } else {
        cpu_count > 1
    }
}
