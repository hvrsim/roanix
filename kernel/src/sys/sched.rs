//!
//! # Scheduler
//!
//! ULE-derived kernel scheduler with FreeBSD-compatible priority bands,
//! interactivity scoring, CPU decay, timeshare queue rotation, load-scaled
//! slices, preemption rules, wake affinity, and load balancing.
//!
//! The scheduler preserves Roanix-specific mechanisms where FreeBSD
//! infrastructure does not map directly: flat CPU topology, Rust-owned thread
//! records, intrusive per-CPU queues, and one-shot deadline multiplexing for
//! the 127 Hz scheduler statclock and sleep timers.
//!

use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::{
    array,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    time::Duration,
};

use intrusive_collections::{LinkedList, UnsafeRef};

use crate::{
    arch,
    mem::VmSpace,
    proc::Process,
    sys::{
        clock,
        smp::{self, IrqSpinLock},
        sync::{Mutex, Once},
        thread::{
            AllThreadAdapter, ExitedThreadAdapter, Thread, ThreadAdapter, ThreadClass, ThreadFlags,
            ThreadState, WakeResult, allocate_forked_user_thread, allocate_thread,
            allocate_user_thread, free_thread, idle_task,
        },
    },
};

type TrapFrame = crate::arch::cpu::TrapFrame;
type ThreadPtr = NonNull<Thread>;

const PRIO_MIN: i32 = -20;
const PRIO_MAX: i32 = 20;

const MAX_ITHD: u8 = 7;
const MIN_KERN: u8 = 40;
const MIN_TIMESHARE: u8 = 56;
const MAX_TIMESHARE: u8 = 223;
const MIN_IDLE: u8 = 224;
const MAX_IDLE: u8 = 255;

const PRI_TIMESHARE_RANGE: u32 = (MAX_TIMESHARE - MIN_TIMESHARE + 1) as u32;
const SCHED_PRI_NRESV: u32 = (((PRIO_MAX - PRIO_MIN) as u32) * 5) / 4;
const PRI_INTERACT_RANGE: u8 = ((PRI_TIMESHARE_RANGE - SCHED_PRI_NRESV) / 2) as u8;
const MIN_INTERACT: u8 = MIN_TIMESHARE;
const MAX_INTERACT: u8 = MIN_INTERACT + PRI_INTERACT_RANGE - 1;
const MIN_BATCH: u8 = MAX_INTERACT + 1;
const MAX_BATCH: u8 = MAX_TIMESHARE;

const SCHED_INTERACT_HALF: u32 = 50;
const SCHED_INTERACT_THRESH: u32 = 30;
const SCHED_TICK_SHIFT: u32 = 10;
const SCHED_CPU_DECAY_NUMER: u64 = 10;
const SCHED_CPU_DECAY_DENOM: u64 = 11;
const SCHED_CPU_DECAY_WINDOW_NS: u64 = 11_000_000_000;
const SCHED_SLP_RUN_MAX_NS: u64 = 5_000_000_000;
const SCHED_SLEEP_TICK_NS: u64 = 1_000_000;

const NSEC_PER_SEC: u64 = 1_000_000_000;
const SCHED_STAT_HZ: u64 = 127;
const SCHED_STAT_INTERVAL_NS: u64 = NSEC_PER_SEC / SCHED_STAT_HZ;
const SCHED_SLICE_DEFAULT_NS: u64 = SCHED_STAT_INTERVAL_NS;
const SCHED_SLICE_MIN_DIVISOR: u32 = 6;
const SCHED_BALANCE_INTERVAL_NS: u64 = NSEC_PER_SEC;
const SCHED_WAKE_AFFINITY_NS: u64 = 2_000_000;
const SCHED_STEAL_THRESHOLD: usize = 2;

/// Maximum threads examined per run-queue bucket when picking a migration
/// candidate. Steal selection runs with two CPU locks held, so the scan is
/// bounded rather than walking an arbitrarily long queue.
const RUNQ_CANDIDATE_SCAN: usize = 16;

const PRI_BATCH_RANGE: usize = (MAX_BATCH - MIN_BATCH + 1) as usize;
const SCHED_PRI_CPU_RANGE: u32 = PRI_BATCH_RANGE as u32 - SCHED_PRI_NRESV;

const _: () = assert!(MAX_INTERACT == 114);
const _: () = assert!(MIN_BATCH == 115);
const _: () = assert!(PRI_BATCH_RANGE == 109);

static SCHEDULER: Once<Scheduler> = Once::new();
static ALL_THREADS: IrqSpinLock<Option<LinkedList<AllThreadAdapter>>> = IrqSpinLock::new(None);

#[derive(Copy, Clone, Eq, PartialEq)]
enum EnqueueKind {
    Normal,
    Preempted,
}

#[derive(Copy, Clone)]
enum CpuKick {
    Resched,
    Deadline,
}

impl EnqueueKind {
    fn push_front(self) -> bool {
        matches!(self, Self::Preempted)
    }

    fn uses_pick_cursor(self) -> bool {
        matches!(self, Self::Preempted)
    }
}

#[derive(Copy, Clone)]
struct SchedulerConfig {
    sched_slice_ns: u64,
    wake_affinity_ns: u64,
    /// Precomputed slice length per runnable-load level, avoiding a 64-bit
    /// division on the trap-return path.
    slice_by_load: [u64; SCHED_SLICE_MIN_DIVISOR as usize + 1],
}

impl SchedulerConfig {
    fn new() -> Self {
        let sched_slice_ns = SCHED_SLICE_DEFAULT_NS;
        let sched_slice_min_ns = (sched_slice_ns / u64::from(SCHED_SLICE_MIN_DIVISOR)).max(1);

        let slice_by_load = array::from_fn(|load| match load as u64 {
            0 | 1 => sched_slice_ns,
            load if load >= u64::from(SCHED_SLICE_MIN_DIVISOR) => sched_slice_min_ns,
            load => (sched_slice_ns / load).max(sched_slice_min_ns),
        });

        Self {
            sched_slice_ns,
            wake_affinity_ns: SCHED_WAKE_AFFINITY_NS,
            slice_by_load,
        }
    }

    #[inline]
    fn slice_for_load(&self, runnable_load: usize) -> u64 {
        self.slice_by_load[runnable_load.min(SCHED_SLICE_MIN_DIVISOR as usize)]
    }
}

struct ExitedThreads {
    list: LinkedList<ExitedThreadAdapter>,
}

// SAFETY: exited-thread lists are only accessed while holding their owning
// per-CPU scheduler lock.
unsafe impl Send for ExitedThreads {}

impl ExitedThreads {
    fn new() -> Self {
        Self {
            list: LinkedList::new(ExitedThreadAdapter::NEW),
        }
    }

    fn push(&mut self, thread: ThreadPtr) {
        // SAFETY: exited threads are scheduler-owned allocations that stay
        // live until the reaper frees them, and the reap link is unused.
        self.list
            .push_back(unsafe { UnsafeRef::from_raw(thread.as_ptr()) });
    }

    fn pop(&mut self) -> Option<ThreadPtr> {
        self.list
            .pop_front()
            .map(|thread| NonNull::new(UnsafeRef::into_raw(thread)).expect("sched: null exit link"))
    }
}

struct RunQueue {
    len: usize,
    bits: [u64; 4],
    queues: [LinkedList<ThreadAdapter>; 256],
}

impl RunQueue {
    fn new() -> Self {
        Self {
            len: 0,
            bits: [0; 4],
            queues: array::from_fn(|_| LinkedList::new(ThreadAdapter::NEW)),
        }
    }

    fn push(&mut self, bucket: u8, thread: ThreadPtr, push_front: bool) {
        let thread_ref = thread_ref(thread);
        assert!(
            !thread_ref.is_runq_linked(),
            "sched: thread {} already linked before enqueue (state={:?} cpu={} rqindex={} priority={})",
            thread_ref.id,
            thread_ref.state,
            thread_ref.cpu(),
            thread_ref.rqindex,
            thread_ref.priority,
        );
        let index = usize::from(bucket);
        self.bits[index / 64] |= 1u64 << (index % 64);
        // SAFETY: run-queue members are scheduler-owned allocations that stay
        // live until the reaper frees them, and this link is currently unused.
        let link = unsafe { UnsafeRef::from_raw(thread.as_ptr()) };
        if push_front {
            self.queues[index].push_front(link);
        } else {
            self.queues[index].push_back(link);
        }
        self.len += 1;
    }

    fn remove(&mut self, bucket: u8, thread: ThreadPtr) -> bool {
        let index = usize::from(bucket);
        let thread_ref = thread_ref(thread);
        assert!(
            thread_ref.is_runq_linked(),
            "sched: thread {} not linked before dequeue (state={:?} cpu={} rqindex={} priority={})",
            thread_ref.id,
            thread_ref.state,
            thread_ref.cpu(),
            thread_ref.rqindex,
            thread_ref.priority,
        );
        // SAFETY: the run-queue link is known to be linked in this exact queue,
        // and the queue is exclusively borrowed.
        unsafe {
            let mut cursor = self.queues[index].cursor_mut_from_ptr(thread.as_ptr());
            let _ = cursor.remove();
        }
        self.len = self.len.saturating_sub(1);

        let empty = self.queues[index].is_empty();
        if empty {
            self.bits[index / 64] &= !(1u64 << (index % 64));
        }
        empty
    }

    fn first_in_range(&self, start: u8, end: u8) -> Option<ThreadPtr> {
        let bucket = self.first_occupied_bucket(start, end)?;
        self.queues[usize::from(bucket)]
            .front()
            .get()
            .map(NonNull::from)
    }

    /// Finds the first non-empty run-queue bucket with at most four bitmap
    /// probes instead of walking every priority.
    fn first_occupied_bucket(&self, start: u8, end: u8) -> Option<u8> {
        debug_assert!(start <= end);

        let first_word = usize::from(start) / 64;
        let last_word = usize::from(end) / 64;

        for word in first_word..=last_word {
            let mut occupied = self.bits[word];

            if word == first_word {
                occupied &= u64::MAX << (usize::from(start) % 64);
            }
            if word == last_word {
                let end_bit = usize::from(end) % 64;
                if end_bit != 63 {
                    occupied &= (1u64 << (end_bit + 1)) - 1;
                }
            }

            if occupied != 0 {
                let bucket = word * 64 + occupied.trailing_zeros() as usize;
                return Some(bucket as u8);
            }
        }

        None
    }

    /// Scans occupied buckets in priority order for the first thread accepted
    /// by `pred`.
    ///
    /// Only occupied buckets are visited, and each bucket's scan is bounded so
    /// one long queue cannot stall a caller holding two CPU locks.
    fn first_in_range_if<F>(&self, start: u8, end: u8, mut pred: F) -> Option<ThreadPtr>
    where
        F: FnMut(ThreadPtr) -> bool,
    {
        let mut cursor = start;
        loop {
            let bucket = self.first_occupied_bucket(cursor, end)?;

            for thread in self.queues[usize::from(bucket)]
                .iter()
                .take(RUNQ_CANDIDATE_SCAN)
            {
                let thread = NonNull::from(thread);
                if pred(thread) {
                    return Some(thread);
                }
            }

            cursor = bucket.checked_add(1)?;
            if cursor > end {
                return None;
            }
        }
    }

    fn first_timeshare(&self, pick_cursor: u8) -> Option<ThreadPtr> {
        let start = MIN_BATCH.saturating_add(pick_cursor);
        self.first_in_range(start, MAX_BATCH).or_else(|| {
            (pick_cursor != 0)
                .then(|| self.first_in_range(MIN_BATCH, start - 1))
                .flatten()
        })
    }

    fn first_timeshare_if<F>(&self, pick_cursor: u8, mut pred: F) -> Option<ThreadPtr>
    where
        F: FnMut(ThreadPtr) -> bool,
    {
        let start = MIN_BATCH.saturating_add(pick_cursor);
        self.first_in_range_if(start, MAX_BATCH, &mut pred)
            .or_else(|| {
                (pick_cursor != 0)
                    .then(|| self.first_in_range_if(MIN_BATCH, start - 1, pred))
                    .flatten()
            })
    }

    fn has_runnable(&self) -> bool {
        self.len != 0
    }
}

/// Per-CPU scheduler state that remote CPUs read without taking the owning
/// CPU's lock.
///
/// Load surveys run on every wake and every balance attempt; publishing these
/// counters atomically keeps those surveys free of cross-CPU lock traffic.
/// The record lives in [`crate::sys::smp::CoreLocal`] so it is cache-line
/// separated from other CPUs' summaries.
pub(crate) struct CpuSummary {
    load: AtomicUsize,
    online: AtomicBool,
}

impl CpuSummary {
    /// Creates an offline summary with no load.
    pub(crate) const fn new() -> Self {
        Self {
            load: AtomicUsize::new(0),
            online: AtomicBool::new(false),
        }
    }

    #[inline]
    fn load(&self) -> usize {
        self.load.load(Ordering::Relaxed)
    }

    #[inline]
    fn is_online(&self) -> bool {
        self.online.load(Ordering::Acquire)
    }
}

fn cpu_summary(cpu_id: usize) -> &'static CpuSummary {
    &smp::core_local(cpu_id)
        .unwrap_or_else(|| panic!("sched: missing core-local record for cpu{cpu_id}"))
        .sched_summary
}

struct CpuState {
    runq: RunQueue,
    current: Option<ThreadPtr>,
    idle: Option<ThreadPtr>,
    exited: ExitedThreads,
    deferred_exit: Option<ThreadPtr>,
    online: bool,
    load: usize,
    sysload: usize,
    ts_insert_cursor: u8,
    ts_pick_cursor: u8,
    ts_ticks: u64,
    next_statclock_ns: u64,
    // The thread this CPU is still unwinding off after selecting a
    // replacement. Remote CPUs must not migrate it until the trap return has
    // switched stacks, or two CPUs would execute the same kernel stack.
    unwinding: Option<ThreadPtr>,
    need_resched: bool,
    summary: &'static CpuSummary,
}

// SAFETY: each `CpuState` is only accessed through its per-CPU
// `IrqSpinLock`, including cross-CPU scheduler operations.
unsafe impl Send for CpuState {}

impl CpuState {
    fn new(cpu_id: usize) -> Self {
        let summary = cpu_summary(cpu_id);
        let online = cpu_id == 0;
        summary.online.store(online, Ordering::Release);
        Self {
            runq: RunQueue::new(),
            current: None,
            idle: None,
            exited: ExitedThreads::new(),
            deferred_exit: None,
            online,
            load: 0,
            sysload: 0,
            ts_insert_cursor: 0,
            ts_pick_cursor: 0,
            ts_ticks: 0,
            next_statclock_ns: 0,
            unwinding: None,
            need_resched: false,
            summary,
        }
    }

    fn set_online(&mut self, online: bool) {
        self.online = online;
        self.summary.online.store(online, Ordering::Release);
    }

    #[inline]
    fn publish_load(&self) {
        self.summary.load.store(self.load, Ordering::Relaxed);
    }

    fn set_idle(&mut self, idle: &'static mut Thread) {
        self.idle = Some(NonNull::from(idle));
    }

    fn idle_thread(&self) -> ThreadPtr {
        self.idle.expect("sched: missing idle thread")
    }

    fn current_thread(&self) -> Option<ThreadPtr> {
        self.current
    }

    fn needs_deadline_refresh(&self, had_runnable: bool) -> bool {
        if had_runnable || self.need_resched {
            return false;
        }

        let Some(current) = self.current else {
            return false;
        };
        let current = thread_ref(current);
        current.state == ThreadState::Running && !current.is_idle()
    }

    fn is_immediately_available(&self) -> bool {
        !self.runq.has_runnable()
            && self
                .current
                .map(|thread| thread_ref(thread).is_idle())
                .unwrap_or(true)
    }

    fn best_runnable(&self) -> Option<ThreadPtr> {
        self.runq
            .first_in_range(0, MAX_INTERACT)
            .or_else(|| self.runq.first_timeshare(self.ts_pick_cursor))
            .or_else(|| self.runq.first_in_range(MIN_IDLE, MAX_IDLE))
    }

    fn take_next_thread(&mut self, cpu_id: usize, now_ns: u64) -> ThreadPtr {
        let previous = self.current;
        let next = self.best_runnable().unwrap_or_else(|| self.idle_thread());
        if !thread_ref(next).is_idle() {
            self.dequeue_thread(next);
        }

        thread_mut(next).mark_running(cpu_id, now_ns);
        let previous_was_idle = previous.map(thread_ref).is_none_or(Thread::is_idle);
        if thread_ref(next).is_idle() {
            self.next_statclock_ns = 0;
        } else if previous_was_idle || self.next_statclock_ns == 0 {
            self.next_statclock_ns = now_ns.saturating_add(SCHED_STAT_INTERVAL_NS);
        }
        self.current = Some(next);
        self.unwinding = previous.filter(|previous| *previous != next);
        next
    }

    fn dequeue_thread(&mut self, thread: ThreadPtr) {
        let thread_ref = thread_ref(thread);
        let emptied = self.runq.remove(thread_ref.rqindex, thread);
        if (MIN_BATCH..=MAX_BATCH).contains(&thread_ref.priority)
            && emptied
            && self.ts_pick_cursor + MIN_BATCH == thread_ref.rqindex
        {
            self.advance_pick_cursor(true);
        }
    }

    fn enqueue_existing(&mut self, thread: &mut Thread, kind: EnqueueKind) {
        if thread.is_runq_linked() {
            assert_eq!(
                thread.state,
                ThreadState::Ready,
                "sched: linked thread {} in unexpected state {:?}",
                thread.id,
                thread.state,
            );
            return;
        }
        let bucket = self.queue_bucket(thread.priority, kind);
        thread.mark_ready(bucket);
        self.runq
            .push(bucket, NonNull::from(thread), kind.push_front());
    }

    fn adopt_thread(&mut self, thread: &mut Thread, kind: EnqueueKind) {
        self.enqueue_existing(thread, kind);
        if thread.counts_towards_load() {
            self.load += 1;
            self.sysload += 1;
            self.publish_load();
        }
    }

    fn release_thread(&mut self, thread: &Thread) {
        if thread.counts_towards_load() {
            self.load = self.load.saturating_sub(1);
            self.sysload = self.sysload.saturating_sub(1);
            self.publish_load();
        }
    }

    fn consider_preemption(&mut self, priority: u8, remote: bool) -> bool {
        let Some(current) = self.current else {
            return false;
        };
        let current_priority = thread_ref(current).priority;
        if priority >= current_priority {
            return false;
        }

        self.need_resched = true;
        should_preempt(priority, current_priority, remote)
    }

    fn consider_wakeup_preemption(
        &mut self,
        thread: &Thread,
        latency_sensitive: bool,
        remote: bool,
    ) -> bool {
        if self.consider_preemption(thread.priority, remote) {
            return true;
        }
        if !latency_sensitive {
            return false;
        }

        let Some(current) = self.current else {
            return false;
        };
        let current = thread_ref(current);
        if thread.priority != current.priority
            || thread.priority > MAX_INTERACT
            || thread.class != ThreadClass::Timeshare
            || current.class != ThreadClass::Timeshare
            || current.state != ThreadState::Running
        {
            return false;
        }

        self.need_resched = true;
        true
    }

    fn queue_bucket(&self, priority: u8, kind: EnqueueKind) -> u8 {
        if !(MIN_BATCH..=MAX_BATCH).contains(&priority) {
            return priority;
        }

        let mut index = if kind.uses_pick_cursor() {
            usize::from(self.ts_pick_cursor)
        } else {
            usize::from(priority - MIN_BATCH) + usize::from(self.ts_insert_cursor)
        } % PRI_BATCH_RANGE;

        if self.ts_pick_cursor != self.ts_insert_cursor && index == usize::from(self.ts_pick_cursor)
        {
            index = (index + PRI_BATCH_RANGE - 1) % PRI_BATCH_RANGE;
        }

        MIN_BATCH + index as u8
    }

    fn advance_pick_cursor(&mut self, mut emptied_current: bool) {
        while self.ts_pick_cursor != self.ts_insert_cursor {
            if emptied_current {
                emptied_current = false;
            } else if self
                .runq
                .first_in_range(
                    MIN_BATCH + self.ts_pick_cursor,
                    MIN_BATCH + self.ts_pick_cursor,
                )
                .is_some()
            {
                break;
            }

            self.ts_pick_cursor = ((usize::from(self.ts_pick_cursor) + 1) % PRI_BATCH_RANGE) as u8;
        }
    }

    fn advance_timeshare_epoch(&mut self, ticks: u64) {
        if ticks == 0 || self.ts_insert_cursor != self.ts_pick_cursor {
            return;
        }

        let total_ticks = self.ts_ticks.saturating_add(ticks);
        let advance = ticks.saturating_mul(2).saturating_sub(total_ticks / 4);
        self.ts_insert_cursor = ((usize::from(self.ts_insert_cursor)
            + (advance as usize % PRI_BATCH_RANGE))
            % PRI_BATCH_RANGE) as u8;
        self.ts_ticks = total_ticks % 4;
        self.advance_pick_cursor(false);
    }

    fn advance_statclock(&mut self, now_ns: u64) -> u64 {
        if self.next_statclock_ns == 0 || now_ns < self.next_statclock_ns {
            return 0;
        }

        let ticks = now_ns
            .saturating_sub(self.next_statclock_ns)
            .saturating_div(SCHED_STAT_INTERVAL_NS)
            .saturating_add(1);
        self.next_statclock_ns = self
            .next_statclock_ns
            .saturating_add(ticks.saturating_mul(SCHED_STAT_INTERVAL_NS));
        ticks
    }

    fn quantum_for(&self, thread: &Thread, config: &SchedulerConfig) -> u64 {
        if thread.class != ThreadClass::Timeshare {
            return config.sched_slice_ns;
        }

        let runnable_load = self.sysload.saturating_sub(1);
        config.slice_for_load(runnable_load)
    }

    fn steal_candidate(&self) -> Option<ThreadPtr> {
        let can_steal = |candidate: ThreadPtr| {
            // During trap return, the outgoing `current` thread can be briefly
            // requeued before the CPU commits to its replacement. Never allow
            // a still-executing stack to migrate to another CPU.
            self.current != Some(candidate)
                && self.unwinding != Some(candidate)
                && thread_ref(candidate).can_migrate()
        };

        self.runq
            .first_in_range_if(0, MAX_INTERACT, &can_steal)
            .or_else(|| self.runq.first_timeshare_if(self.ts_pick_cursor, can_steal))
    }
}

pub(crate) struct PerCpuScheduler {
    state: IrqSpinLock<CpuState>,
}

impl PerCpuScheduler {
    fn new(cpu_id: usize) -> Self {
        Self {
            state: IrqSpinLock::new(CpuState::new(cpu_id)),
        }
    }
}

struct Scheduler {
    cpu_count: usize,
    next_tid: AtomicUsize,
    config: SchedulerConfig,
    /// Direct per-CPU scheduler pointers.
    ///
    /// Resolving these through the SMP registry costs a linear scan plus two
    /// `Once` probes, which the trap-return and wake paths cannot afford.
    cpus: Box<[&'static PerCpuScheduler]>,
    retired_address_spaces: Mutex<Vec<(u64, Arc<VmSpace>)>>,
}

impl Scheduler {
    fn new(cpu_count: usize) -> Self {
        for cpu_id in 0..cpu_count {
            smp::core_local(cpu_id)
                .unwrap_or_else(|| panic!("sched: missing core-local record for cpu{cpu_id}"))
                .scheduler
                .call_once(|| PerCpuScheduler::new(cpu_id));
        }

        let cpus = (0..cpu_count)
            .map(|cpu_id| {
                smp::core_local(cpu_id)
                    .and_then(|core| core.scheduler.get())
                    .unwrap_or_else(|| panic!("sched: cpu{cpu_id} scheduler not initialized"))
            })
            .collect::<Vec<_>>()
            .into_boxed_slice();

        Self {
            cpu_count,
            next_tid: AtomicUsize::new(1),
            config: SchedulerConfig::new(),
            cpus,
            retired_address_spaces: Mutex::new(Vec::new()),
        }
    }

    fn bootstrap<F, R>(&self, init_task: F)
    where
        F: FnOnce() -> R + Send + 'static,
    {
        for cpu_id in 0..self.cpu_count {
            let idle = self.alloc_thread(
                cpu_id,
                ThreadClass::Idle,
                MAX_IDLE,
                ThreadFlags::IDLE | ThreadFlags::NOLOAD | ThreadFlags::NO_MIGRATE,
                || idle_task(),
            );
            let mut cpu = self.cpu(cpu_id).lock();
            cpu.set_idle(idle);
        }

        let init = self.alloc_thread(
            0,
            ThreadClass::Timeshare,
            MIN_INTERACT,
            ThreadFlags::NOLOAD | ThreadFlags::NO_MIGRATE,
            init_task,
        );
        self.cpu(0).lock().adopt_thread(init, EnqueueKind::Normal);

        let maint = self.alloc_thread(
            0,
            ThreadClass::Timeshare,
            MIN_INTERACT,
            ThreadFlags::NOLOAD | ThreadFlags::NO_MIGRATE,
            maintenance_thread,
        );
        self.cpu(0).lock().adopt_thread(maint, EnqueueKind::Normal);
    }

    #[inline]
    fn cpu(&self, cpu_id: usize) -> &'static IrqSpinLock<CpuState> {
        &self.cpu_local(cpu_id).state
    }

    #[inline]
    fn cpu_local(&self, cpu_id: usize) -> &'static PerCpuScheduler {
        self.cpus[cpu_id]
    }

    #[inline]
    fn summary(&self, cpu_id: usize) -> &'static CpuSummary {
        cpu_summary(cpu_id)
    }

    /// Locks two CPU scheduler states in ascending logical CPU order.
    ///
    /// All cross-CPU scheduler operations must use this helper so they share
    /// one deadlock-prevention rule.
    fn with_cpu_pair<R>(
        &self,
        left_id: usize,
        right_id: usize,
        f: impl FnOnce(&mut CpuState, &mut CpuState) -> R,
    ) -> R {
        assert_ne!(left_id, right_id, "sched: duplicate cpu lock request");

        if left_id < right_id {
            let mut left = self.cpu(left_id).lock();
            let mut right = self.cpu(right_id).lock();
            f(&mut left, &mut right)
        } else {
            let mut right = self.cpu(right_id).lock();
            let mut left = self.cpu(left_id).lock();
            f(&mut left, &mut right)
        }
    }

    fn alloc_thread<F, R>(
        &self,
        cpu_id: usize,
        class: ThreadClass,
        priority: u8,
        flags: ThreadFlags,
        task: F,
    ) -> &'static mut Thread
    where
        F: FnOnce() -> R + Send + 'static,
    {
        let tid = self.next_tid.fetch_add(1, Ordering::Relaxed);
        allocate_thread(
            tid,
            cpu_id,
            clock::monotonic_ns(),
            class,
            priority,
            flags,
            task,
        )
    }

    fn spawn<F, R>(&self, task: F) -> usize
    where
        F: FnOnce() -> R + Send + 'static,
    {
        self.spawn_on(
            self.pick_spawn_cpu(),
            ThreadClass::Timeshare,
            MIN_INTERACT,
            task,
        )
    }

    fn spawn_ithread<F, R>(&self, task: F, arg: u64, priority: u8) -> usize
    where
        F: FnOnce(u64) -> R + Send + 'static,
    {
        assert!(
            priority <= MAX_ITHD,
            "sched: ithread priority {priority} exceeds {MAX_ITHD}"
        );
        self.spawn_on(
            self.pick_spawn_cpu(),
            ThreadClass::Ithread,
            priority,
            move || task(arg),
        )
    }

    fn spawn_on<F, R>(&self, cpu_id: usize, class: ThreadClass, priority: u8, task: F) -> usize
    where
        F: FnOnce() -> R + Send + 'static,
    {
        let thread =
            NonNull::from(self.alloc_thread(cpu_id, class, priority, ThreadFlags::empty(), task));
        self.enqueue_new_thread(thread)
    }

    fn spawn_user(&self, process: Arc<Process>, entry: u64, stack: u64) -> usize {
        process.register_thread();
        let cpu_id = self.pick_spawn_cpu();
        let tid = self.next_tid.fetch_add(1, Ordering::Relaxed);
        let thread = allocate_user_thread(
            tid,
            cpu_id,
            clock::monotonic_ns(),
            MIN_INTERACT,
            process.nice(),
            process,
            entry,
            stack,
            0,
        );
        self.enqueue_new_thread(NonNull::from(thread))
    }

    fn spawn_user_thread(
        &self,
        process: Arc<Process>,
        entry: u64,
        stack: u64,
        thread_pointer: u64,
    ) -> usize {
        process.register_thread();
        let cpu_id = self.pick_spawn_cpu();
        let tid = self.next_tid.fetch_add(1, Ordering::Relaxed);
        let thread = allocate_user_thread(
            tid,
            cpu_id,
            clock::monotonic_ns(),
            MIN_INTERACT,
            process.nice(),
            process,
            entry,
            stack,
            thread_pointer,
        );
        self.enqueue_new_thread(NonNull::from(thread))
    }

    fn spawn_forked_user(
        &self,
        process: Arc<Process>,
        parent_frame: &TrapFrame,
        thread_pointer: u64,
    ) -> usize {
        process.register_thread();
        let cpu_id = self.pick_spawn_cpu();
        let tid = self.next_tid.fetch_add(1, Ordering::Relaxed);
        let thread = allocate_forked_user_thread(
            tid,
            cpu_id,
            clock::monotonic_ns(),
            MIN_INTERACT,
            process.nice(),
            process,
            parent_frame,
            thread_pointer,
        );
        self.enqueue_new_thread(NonNull::from(thread))
    }

    fn enqueue_new_thread(&self, thread: ThreadPtr) -> usize {
        register_thread(thread);

        let tid = thread_ref(thread).id;
        let priority = thread_ref(thread).priority;
        let target_cpu = thread_ref(thread).cpu();
        let current_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);

        let kick = {
            let mut cpu = self.cpu(target_cpu).lock();
            let had_runnable = cpu.runq.has_runnable();
            let thread = thread_mut(thread);
            cpu.adopt_thread(thread, EnqueueKind::Normal);
            let should_kick = cpu.consider_preemption(priority, current_cpu != Some(target_cpu));
            if !cpu.online {
                None
            } else if should_kick {
                Some((target_cpu, CpuKick::Resched))
            } else if cpu.needs_deadline_refresh(had_runnable) {
                Some((target_cpu, CpuKick::Deadline))
            } else {
                None
            }
        };
        if let Some((kick_cpu, kind)) = kick {
            self.kick_cpu(kick_cpu, current_cpu, kind);
        }
        tid
    }

    fn pick_spawn_cpu(&self) -> usize {
        if self.cpu_count == 1 {
            return 0;
        }

        let preferred = arch::thiscpu_opt().map(|cpu| cpu.id).unwrap_or(0);
        let mut best_id = 0usize;
        let mut best_load = usize::MAX;
        let mut preferred_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let summary = self.summary(cpu_id);
            if !summary.is_online() {
                continue;
            }
            let load = summary.load();

            if cpu_id == preferred {
                preferred_load = load;
            }
            if load < best_load {
                best_load = load;
                best_id = cpu_id;
            }
        }

        if preferred_load <= best_load.saturating_add(1) {
            preferred
        } else {
            best_id
        }
    }

    fn wake_target_cpu(&self, thread: &Thread, wake_cpu: Option<usize>, now_ns: u64) -> usize {
        let owner_cpu = thread.cpu();
        if self.cpu_count == 1 {
            return owner_cpu;
        }
        if !thread.can_migrate() && smp::is_online(owner_cpu) {
            return owner_cpu;
        }

        if thread.class == ThreadClass::Ithread
            && let Some(cpu_id) = wake_cpu
            && smp::is_online(cpu_id)
        {
            return cpu_id;
        }

        let affine = now_ns.saturating_sub(thread.last_run_ns()) <= self.config.wake_affinity_ns;
        if affine
            && self.summary(owner_cpu).is_online()
            && self
                .cpu(owner_cpu)
                .try_lock()
                .is_some_and(|cpu| cpu.is_immediately_available())
        {
            return owner_cpu;
        }

        let mut best_id = owner_cpu;
        let mut best_load = usize::MAX;
        let mut wake_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let summary = self.summary(cpu_id);
            if !summary.is_online() {
                continue;
            }
            let load = summary.load();
            if Some(cpu_id) == wake_cpu {
                wake_load = load;
            }
            if load < best_load {
                best_load = load;
                best_id = cpu_id;
            }
        }

        if let Some(cpu_id) = wake_cpu
            && smp::is_online(cpu_id)
            && wake_load != usize::MAX
            && wake_load <= best_load.saturating_add(1)
        {
            return cpu_id;
        }

        best_id
    }

    fn account_current(&self, cpu: &mut CpuState, now_ns: u64) {
        let Some(current_ptr) = cpu.current_thread() else {
            return;
        };
        let stat_ticks = cpu.advance_statclock(now_ns);
        cpu.advance_timeshare_epoch(stat_ticks);

        let current = thread_mut(current_ptr);
        let elapsed_ns = now_ns.saturating_sub(current.last_run_ns());

        if !current.is_idle() && current.state != ThreadState::Exited {
            current.set_last_run_ns(now_ns);
            current.slice_ns = current.slice_ns.saturating_add(elapsed_ns);

            if current.class == ThreadClass::Timeshare {
                self.pctcpu_update(current, now_ns, true);

                if stat_ticks != 0 {
                    current.runtime_ns = current
                        .runtime_ns
                        .saturating_add(stat_ticks.saturating_mul(SCHED_STAT_INTERVAL_NS));
                    if current.state == ThreadState::Running {
                        self.interact_update(current);
                        self.priority_update(current);
                    }
                }
            }
        }

        if current.state != ThreadState::Running {
            return;
        }

        if current.is_idle() {
            if cpu.runq.has_runnable() {
                cpu.need_resched = true;
            }
            return;
        }

        if current.slice_ns >= cpu.quantum_for(current, &self.config) {
            current.slice_ns = 0;
            if !cpu.runq.has_runnable() {
                current.flags.remove(ThreadFlags::SLICEEND);
                return;
            }
            current.flags.insert(ThreadFlags::SLICEEND);
            if current.class == ThreadClass::Ithread {
                let demoted = current.base_priority.saturating_add(1);
                if demoted < MAX_ITHD {
                    current.base_priority = demoted;
                    current.refresh_priority();
                }
            }
            cpu.need_resched = true;
            return;
        }

        if let Some(best) = cpu.best_runnable()
            && should_preempt(thread_ref(best).priority, current.priority, false)
        {
            cpu.need_resched = true;
        }
    }

    /// Computes the next local timer deadline from already-locked CPU state.
    fn deadline_for(&self, cpu: &CpuState, now_ns: u64) -> u64 {
        let Some(current_ptr) = cpu.current_thread() else {
            return 0;
        };
        let current = thread_ref(current_ptr);

        if cpu.need_resched || current.state != ThreadState::Running || current.is_idle() {
            return 0;
        }

        let quantum_ns = cpu.quantum_for(current, &self.config);
        let slice_deadline =
            now_ns.saturating_add(quantum_ns.saturating_sub(current.slice_ns).max(1));
        let statclock_deadline = if cpu.next_statclock_ns == 0 {
            now_ns.saturating_add(SCHED_STAT_INTERVAL_NS)
        } else {
            cpu.next_statclock_ns
        };
        statclock_deadline.min(slice_deadline)
    }

    fn current_deadline(&self, cpu_id: usize, now_ns: u64) -> u64 {
        let cpu = self.cpu(cpu_id).lock();
        self.deadline_for(&cpu, now_ns)
    }

    fn trap_return(&self, cpu_id: usize, frame: &mut TrapFrame) -> *mut TrapFrame {
        smp::drain_ipi_queue();
        let now_ns = clock::monotonic_ns();

        // One acquisition covers deferred-exit publication, accounting, the
        // switch decision, and the replacement pick. This lock is taken on
        // every trap, so extra round trips are expensive.
        let (resume_frame, deadline_ns, try_switch_steal, try_idle_pull) = {
            let mut cpu = self.cpu(cpu_id).lock();
            if let Some(thread) = cpu.deferred_exit.take() {
                cpu.exited.push(thread);
            }

            let Some(current_ptr) = cpu.current_thread() else {
                cpu.unwinding = None;
                return frame;
            };
            {
                let current = thread_mut(current_ptr);
                debug_assert!(
                    current.has_valid_stack_canary(),
                    "sched: thread {} stack canary corrupted on cpu{}",
                    current.id,
                    cpu_id,
                );
                current.frame = frame;
            }
            self.account_current(&mut cpu, now_ns);
            let current = thread_mut(current_ptr);

            let should_switch = current.state != ThreadState::Running
                || cpu.need_resched
                || (current.is_idle() && cpu.runq.has_runnable());
            if !should_switch {
                cpu.unwinding = None;
                let idle_pull = current.is_idle() && !cpu.runq.has_runnable();
                let deadline_ns = self.deadline_for(&cpu, now_ns);
                (Some(frame as *mut TrapFrame), deadline_ns, false, idle_pull)
            } else {
                cpu.need_resched = false;

                if current.state == ThreadState::Running && !current.is_idle() {
                    let kind = if current.take_slice_end() {
                        EnqueueKind::Normal
                    } else {
                        EnqueueKind::Preempted
                    };
                    cpu.enqueue_existing(current, kind);
                } else if current.state == ThreadState::Blocked {
                    cpu.release_thread(current);
                }

                let steal = !cpu.runq.has_runnable();
                // More work is queued locally: finish the switch under this
                // same lock instead of dropping it to look elsewhere.
                if !steal {
                    let next = cpu.take_next_thread(cpu_id, now_ns);
                    let next_frame = thread_ref(next).frame;
                    let deadline_ns = self.deadline_for(&cpu, now_ns);
                    drop(cpu);
                    self.activate_current(next);
                    clock::set_scheduler_deadline(deadline_ns);
                    // This CPU still has a backlog, so hand some of it to an
                    // idle CPU rather than waiting for the periodic rebalance.
                    self.kick_idle_cpu(cpu_id);
                    return next_frame;
                }

                // Mark the CPU as mid-switch so a remote steal cannot take the
                // outgoing thread while this CPU drops the lock to look for
                // work elsewhere.
                cpu.unwinding = Some(current_ptr);
                (None, 0, true, false)
            }
        };

        if let Some(frame) = resume_frame {
            if try_idle_pull && self.try_idle_pull(cpu_id) {
                let (next, deadline_ns) = self.pick_next_locked(cpu_id, now_ns);
                self.activate_current(next);
                clock::set_scheduler_deadline(deadline_ns);
                return thread_ref(next).frame;
            }
            clock::set_scheduler_deadline(deadline_ns);
            return frame;
        }

        if try_switch_steal {
            let _ = self.try_switch_steal(cpu_id);
        }

        let (next, deadline_ns) = self.pick_next_locked(cpu_id, now_ns);
        let next_frame = thread_ref(next).frame;
        self.activate_current(next);
        clock::set_scheduler_deadline(deadline_ns);
        next_frame
    }

    /// Nudges one idle CPU when this CPU has surplus runnable work.
    ///
    /// An idle CPU stops its local timer, so without a kick it would sit
    /// waiting for a device interrupt or the next periodic rebalance while
    /// this CPU has a backlog.
    fn kick_idle_cpu(&self, from_cpu: usize) {
        if self.cpu_count == 1 || self.summary(from_cpu).load() < SCHED_STEAL_THRESHOLD {
            return;
        }

        for cpu_id in 0..self.cpu_count {
            if cpu_id == from_cpu {
                continue;
            }
            let summary = self.summary(cpu_id);
            if summary.is_online() && summary.load() == 0 {
                self.kick_cpu(cpu_id, Some(from_cpu), CpuKick::Resched);
                return;
            }
        }
    }

    fn start_current_cpu(&self) -> ! {
        arch::irqset(false);
        clock::start_cpu();

        let cpu_id = arch::thiscpu().id;
        self.cpu(cpu_id).lock().set_online(true);

        let _ = self.try_idle_pull(cpu_id);
        let now_ns = clock::monotonic_ns();
        let (next, deadline_ns) = self.pick_next_locked(cpu_id, now_ns);
        self.activate_current(next);
        clock::set_scheduler_deadline(deadline_ns);
        smp::mark_current_online();

        // SAFETY: `next` is a live scheduler-owned thread with a validated
        // frame and this path never returns after transferring control.
        unsafe {
            crate::arch::cpu::start_first_thread(thread_ref(next).frame);
        }
    }

    fn exit_current(&self, cpu_id: usize) -> ! {
        arch::irqset(false);

        {
            let mut cpu = self.cpu(cpu_id).lock();
            if let Some(thread) = cpu.deferred_exit.take() {
                cpu.exited.push(thread);
            }

            let current = cpu
                .current_thread()
                .expect("sched: no current thread to exit");
            let current_thread = thread_mut(current);
            assert!(!current_thread.is_idle(), "sched: idle thread exited");

            current_thread.mark_exited();
            cpu.release_thread(current_thread);
            assert!(
                cpu.deferred_exit.replace(current).is_none(),
                "sched: deferred exit slot already occupied"
            );
            cpu.current = None;
            cpu.need_resched = false;
        }

        let _ = self.try_idle_pull(cpu_id);
        let now_ns = clock::monotonic_ns();
        let (next, deadline_ns) = self.pick_next_locked(cpu_id, now_ns);
        self.activate_current(next);
        clock::set_scheduler_deadline(deadline_ns);

        // SAFETY: `next` is a live scheduler-owned thread with a validated
        // frame and this path never returns after transferring control.
        unsafe {
            crate::arch::cpu::start_first_thread(thread_ref(next).frame);
        }
    }

    /// Picks the next thread and computes its deadline under one lock.
    fn pick_next_locked(&self, cpu_id: usize, now_ns: u64) -> (ThreadPtr, u64) {
        let mut cpu = self.cpu(cpu_id).lock();
        let next = cpu.take_next_thread(cpu_id, now_ns);
        let deadline_ns = self.deadline_for(&cpu, now_ns);
        (next, deadline_ns)
    }

    fn current_thread(&self, cpu_id: usize) -> *mut Thread {
        self.current_thread_opt(cpu_id)
            .expect("sched: current thread unavailable")
            .as_ptr()
    }

    fn current_thread_opt(&self, cpu_id: usize) -> Option<ThreadPtr> {
        self.cpu(cpu_id).lock().current_thread()
    }

    fn park_current(&self, cpu_id: usize, thread: *mut Thread, seq: u64) {
        let thread_ptr = NonNull::new(thread).expect("sched: park received null thread");
        let irqs_were_enabled = arch::irqstate();
        let current = {
            let mut cpu = self.cpu(cpu_id).lock();
            let current = cpu
                .current_thread()
                .expect("sched: no current thread to park");
            assert_eq!(
                current,
                thread_ptr,
                "sched: thread {} attempted to park on cpu{} while current thread is {}",
                thread_ref(thread_ptr).id,
                cpu_id,
                thread_ref(current).id,
            );

            let current_thread = thread_mut(thread_ptr);
            assert!(!current_thread.is_idle(), "sched: idle thread cannot sleep");

            if !current_thread.mark_parked(seq) {
                if !irqs_were_enabled {
                    arch::irqset(true);
                }
                return;
            }

            current_thread.mark_blocked(clock::monotonic_ns());
            cpu.need_resched = true;
            thread_ptr
        };

        if !irqs_were_enabled {
            arch::irqset(true);
        }

        // A local reschedule request is asynchronous on some platforms. Spin
        // until the scheduler has resumed this thread.
        //
        // RISC-V still must not use `wfi` in this wait loop: the reschedule or
        // wake interrupt can be delivered and cleared before the wait
        // instruction executes, leaving the thread asleep forever with
        // `observed_state == Running`. The spin is intentionally short-lived:
        // `arch::reschedule()` should trap immediately and switch this blocked
        // thread off-CPU instead of burning the full sleep interval.
        arch::reschedule();
        while thread_ref(current).observed_state() != ThreadState::Running {
            core::hint::spin_loop();
        }
    }

    fn kick_cpu(&self, cpu_id: usize, current_cpu: Option<usize>, kind: CpuKick) {
        if current_cpu == Some(cpu_id) {
            if smp::in_interrupt_context() {
                return;
            }

            if matches!(kind, CpuKick::Deadline) {
                clock::set_scheduler_deadline(self.current_deadline(cpu_id, clock::monotonic_ns()));
                return;
            }

            if arch::irqstate() {
                arch::reschedule();
            } else {
                clock::set_scheduler_deadline(clock::monotonic_ns());
            }
            return;
        }

        // A remote deadline refresh only needs the target to re-arm its local
        // timer; forcing a full reschedule would preempt the running thread
        // for no reason.
        let callback = match kind {
            CpuKick::Resched => request_reschedule_ipi as fn(),
            CpuKick::Deadline => request_deadline_refresh_ipi as fn(),
        };
        let _ = smp::send_ipi(callback, smp::IpiTarget::Single(cpu_id));
    }

    fn wake_thread(&self, thread: *mut Thread, seq: u64) -> bool {
        let thread_ptr = NonNull::new(thread).expect("sched: wake received null thread");
        // SAFETY: wake handles only scheduler-owned thread allocations, which
        // remain live until the reaper sees the exited state.
        let wake = unsafe { thread_ptr.as_ref().wake(seq) };
        match wake {
            WakeResult::Stale => return false,
            WakeResult::Pending => return true,
            WakeResult::Parked => {}
        }

        let now_ns = clock::monotonic_ns();
        let wake_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
        // SAFETY: the scheduler owns the live thread record throughout wake.
        let owner_cpu = unsafe { thread_ptr.as_ref().cpu() };
        // SAFETY: same live scheduler-owned thread invariant as above.
        let target_cpu = self.wake_target_cpu(unsafe { thread_ptr.as_ref() }, wake_cpu, now_ns);

        if owner_cpu == target_cpu {
            let kick_local = {
                let mut cpu = self.cpu(target_cpu).lock();
                let thread = thread_mut(thread_ptr);
                let waking_current = cpu.current_thread() == Some(thread_ptr);
                let had_runnable = cpu.runq.has_runnable();
                assert_eq!(
                    thread.state,
                    ThreadState::Blocked,
                    "sched: attempted to wake a non-blocked thread"
                );
                if thread.is_runq_linked() {
                    cpu.dequeue_thread(thread_ptr);
                }
                let latency_sensitive = self.finish_wakeup(thread, now_ns);
                if waking_current {
                    thread.mark_running(target_cpu, now_ns);
                    cpu.need_resched = false;
                    let should_kick = cpu
                        .best_runnable()
                        .map(|best| {
                            cpu.consider_preemption(
                                thread_ref(best).priority,
                                wake_cpu != Some(target_cpu),
                            )
                        })
                        .unwrap_or(false);
                    if should_kick {
                        Some((target_cpu, CpuKick::Resched))
                    } else if wake_cpu != Some(target_cpu) {
                        // A remote wake may need to break the owner CPU out of
                        // a short `wfi` in `park_current`.
                        Some((target_cpu, CpuKick::Resched))
                    } else if cpu.needs_deadline_refresh(had_runnable) {
                        Some((target_cpu, CpuKick::Deadline))
                    } else {
                        None
                    }
                } else {
                    cpu.adopt_thread(thread, EnqueueKind::Normal);
                    let should_kick = cpu.consider_wakeup_preemption(
                        thread,
                        latency_sensitive,
                        wake_cpu != Some(target_cpu),
                    );
                    if should_kick {
                        Some((target_cpu, CpuKick::Resched))
                    } else if cpu.needs_deadline_refresh(had_runnable) {
                        Some((target_cpu, CpuKick::Deadline))
                    } else {
                        None
                    }
                }
            };

            if let Some((kick_cpu, kind)) = kick_local {
                self.kick_cpu(kick_cpu, wake_cpu, kind);
            }
            return true;
        }

        let current_cpu = wake_cpu;
        let mut kick = None;
        self.with_cpu_pair(owner_cpu, target_cpu, |owner, target| {
            let thread = thread_mut(thread_ptr);
            let had_runnable = target.runq.has_runnable();
            assert_eq!(
                thread.state,
                ThreadState::Blocked,
                "sched: attempted to wake a non-blocked thread"
            );
            if thread.is_runq_linked() {
                owner.dequeue_thread(thread_ptr);
            }

            // A blocked thread can still be the owner's current thread in the
            // small `park_current -> int` handoff window. Keep that wake local
            // so we never run the same kernel stack on two CPUs at once.
            if owner.current_thread() == Some(thread_ptr) {
                let _ = self.finish_wakeup(thread, now_ns);
                thread.mark_running(owner_cpu, now_ns);
                owner.need_resched = false;
                let should_kick = owner
                    .best_runnable()
                    .map(|best| {
                        owner.consider_preemption(
                            thread_ref(best).priority,
                            current_cpu != Some(owner_cpu),
                        )
                    })
                    .unwrap_or(false);
                kick = if should_kick || current_cpu != Some(owner_cpu) {
                    Some((owner_cpu, CpuKick::Resched))
                } else if owner.needs_deadline_refresh(false) {
                    Some((owner_cpu, CpuKick::Deadline))
                } else {
                    None
                };
                return;
            }

            // `take_next_thread` publishes the replacement in `current`
            // before the owner CPU has switched RSP away from the outgoing
            // thread's kernel stack. A remote wake of that exact thread must
            // remain on the owner CPU or two CPUs can execute the same stack.
            if owner.unwinding == Some(thread_ptr) {
                let had_owner_runnable = owner.runq.has_runnable();
                let latency_sensitive = self.finish_wakeup(thread, now_ns);
                owner.adopt_thread(thread, EnqueueKind::Normal);
                let should_kick = owner.consider_wakeup_preemption(
                    thread,
                    latency_sensitive,
                    current_cpu != Some(owner_cpu),
                );
                kick = if should_kick || current_cpu != Some(owner_cpu) {
                    Some((owner_cpu, CpuKick::Resched))
                } else if owner.needs_deadline_refresh(had_owner_runnable) {
                    Some((owner_cpu, CpuKick::Deadline))
                } else {
                    None
                };
                return;
            }

            let latency_sensitive = self.finish_wakeup(thread, now_ns);
            thread.set_cpu(target_cpu);
            target.adopt_thread(thread, EnqueueKind::Normal);
            let should_kick = target.consider_wakeup_preemption(
                thread,
                latency_sensitive,
                current_cpu != Some(target_cpu),
            );
            kick = if should_kick {
                Some((target_cpu, CpuKick::Resched))
            } else if target.needs_deadline_refresh(had_runnable) {
                Some((target_cpu, CpuKick::Deadline))
            } else {
                None
            };
        });

        if let Some((kick_cpu, kind)) = kick {
            self.kick_cpu(kick_cpu, current_cpu, kind);
        }
        true
    }

    fn try_idle_pull(&self, dst_cpu: usize) -> bool {
        if self.cpu_count == 1 {
            return false;
        }

        let dst_load = {
            let dst = self.cpu(dst_cpu).lock();
            if !dst.online || !dst.is_immediately_available() {
                return false;
            }
            dst.load
        };

        self.pick_steal_source(dst_cpu, dst_load.max(SCHED_STEAL_THRESHOLD))
            .map(|src_cpu| self.transfer_candidate(src_cpu, dst_cpu, true))
            .unwrap_or(false)
    }

    fn try_switch_steal(&self, dst_cpu: usize) -> bool {
        if self.cpu_count == 1 {
            return false;
        }

        let dst_load = {
            let dst = self.cpu(dst_cpu).lock();
            if !dst.online || dst.runq.has_runnable() {
                return false;
            }
            dst.load
        };

        self.pick_steal_source(dst_cpu, dst_load.max(SCHED_STEAL_THRESHOLD))
            .map(|src_cpu| self.transfer_candidate(src_cpu, dst_cpu, false))
            .unwrap_or(false)
    }

    fn pick_steal_source(&self, dst_cpu: usize, min_load: usize) -> Option<usize> {
        let mut source_cpu = None;
        let mut source_load = min_load.saturating_sub(1);

        for cpu_id in 0..self.cpu_count {
            if cpu_id == dst_cpu {
                continue;
            }

            let summary = self.summary(cpu_id);
            if !summary.is_online() {
                continue;
            }
            let load = summary.load();
            if load < min_load || load <= source_load {
                continue;
            }

            source_cpu = Some(cpu_id);
            source_load = load;
        }

        source_cpu
    }

    fn transfer_candidate(&self, src_cpu: usize, dst_cpu: usize, require_idle_dst: bool) -> bool {
        let current_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
        let mut kick = None;

        let moved = self.with_cpu_pair(src_cpu, dst_cpu, |src, dst| {
            let had_runnable = dst.runq.has_runnable();
            if !src.online || !dst.online || src.load <= dst.load + 1 {
                return false;
            }
            if require_idle_dst && !dst.is_immediately_available() {
                return false;
            }

            let Some(candidate) = src.steal_candidate() else {
                return false;
            };

            src.dequeue_thread(candidate);
            src.release_thread(thread_ref(candidate));

            let thread = thread_mut(candidate);
            thread.set_cpu(dst_cpu);
            dst.adopt_thread(thread, EnqueueKind::Normal);
            let should_kick =
                dst.consider_preemption(thread.priority, current_cpu != Some(dst_cpu));
            kick = if should_kick {
                Some((dst_cpu, CpuKick::Resched))
            } else if dst.needs_deadline_refresh(had_runnable) {
                Some((dst_cpu, CpuKick::Deadline))
            } else {
                None
            };
            true
        });

        if moved && let Some((kick_cpu, kind)) = kick {
            self.kick_cpu(kick_cpu, current_cpu, kind);
        }
        moved
    }

    fn rebalance_once(&self) -> bool {
        if self.cpu_count == 1 {
            return false;
        }

        let mut busiest_cpu = None;
        let mut busiest_load = 0usize;
        let mut idlest_cpu = None;
        let mut idlest_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let summary = self.summary(cpu_id);
            if !summary.is_online() {
                continue;
            }
            let load = summary.load();

            if load > busiest_load {
                busiest_load = load;
                busiest_cpu = Some(cpu_id);
            }
            if load < idlest_load {
                idlest_load = load;
                idlest_cpu = Some(cpu_id);
            }
        }

        let (Some(src_cpu), Some(dst_cpu)) = (busiest_cpu, idlest_cpu) else {
            return false;
        };
        if src_cpu == dst_cpu || busiest_load <= idlest_load + 1 {
            return false;
        }

        self.transfer_candidate(src_cpu, dst_cpu, false)
    }

    fn finish_wakeup(&self, thread: &mut Thread, now_ns: u64) -> bool {
        thread.slice_ns = 0;
        thread.flags.remove(ThreadFlags::SLICEEND);
        let sleep_start_ns = core::mem::replace(&mut thread.sleep_start_ns, 0);

        if thread.class == ThreadClass::Ithread {
            thread.base_priority = thread.ithread_base_priority;
            thread.refresh_priority();
            return false;
        }
        if thread.class != ThreadClass::Timeshare {
            return false;
        }

        if sleep_start_ns == 0 {
            return false;
        }
        let slept_ns = now_ns.saturating_sub(sleep_start_ns);
        if slept_ns >= SCHED_SLEEP_TICK_NS {
            thread.slptime_ns = thread.slptime_ns.saturating_add(slept_ns);
            self.interact_update(thread);
            self.pctcpu_update(thread, now_ns, false);
            self.priority_update(thread);
        }
        slept_ns >= SCHED_SLEEP_TICK_NS && thread.priority <= MAX_INTERACT
    }

    fn activate_current(&self, thread: ThreadPtr) {
        let thread = thread_ref(thread);
        assert!(
            thread.has_valid_frame_ptr(),
            "sched: thread {} has invalid frame pointer {:p}",
            thread.id,
            thread.frame
        );
        assert!(
            thread.has_valid_stack_canary(),
            "sched: thread {} stack canary corrupted before activate on cpu{}",
            thread.id,
            arch::thiscpu().id,
        );

        // SAFETY: activation runs on the local scheduler path with interrupts
        // disabled, giving exclusive access to mutable core-local fields.
        let cpu = unsafe { arch::thiscpu_mut() };
        cpu.kernel_stack = thread.stack_top.as_u64();
        cpu.user_stack = 0;
        cpu.current_thread = thread.id;
        crate::arch::cpu::set_kernel_stack(thread.stack_top.as_u64());
        crate::mem::activate_thread_space(thread.address_space());
        // SAFETY: the frame belongs to `thread`, was validated above, and is
        // exclusively prepared on the local scheduler path.
        unsafe {
            crate::arch::cpu::prepare_thread_frame(thread.frame, thread.thread_pointer());
        }
    }

    /// Computes a thread's ULE interactivity score in `0..=100`.
    ///
    /// Mirrors FreeBSD's `sched_interact_score`. The first branch is the
    /// upstream fast path: with the default threshold below the half point, a
    /// thread that has run at least as long as it slept can never score below
    /// the threshold, so the exact value is not worth computing. Since both
    /// constants are fixed here, that makes the remaining branches
    /// unreachable today; they are kept so the threshold stays tunable.
    fn interact_score(&self, thread: &Thread) -> u64 {
        let half = u64::from(SCHED_INTERACT_HALF);

        if SCHED_INTERACT_THRESH <= SCHED_INTERACT_HALF && thread.runtime_ns >= thread.slptime_ns {
            return half;
        }

        if thread.runtime_ns > thread.slptime_ns {
            let div = (thread.runtime_ns / half).max(1);
            return half + half.saturating_sub(thread.slptime_ns / div);
        }

        if thread.slptime_ns > thread.runtime_ns {
            let div = (thread.slptime_ns / half).max(1);
            return thread.runtime_ns / div;
        }

        if thread.runtime_ns != 0 { half } else { 0 }
    }

    fn priority_update(&self, thread: &mut Thread) {
        if thread.class != ThreadClass::Timeshare {
            return;
        }

        let nice = thread.nice();
        let score = (self.interact_score(thread) as i64 + i64::from(nice)).max(0) as u64;
        let priority = if score < u64::from(SCHED_INTERACT_THRESH) {
            MIN_INTERACT
                + (((MAX_INTERACT - MIN_INTERACT + 1) as u64 * score)
                    / u64::from(SCHED_INTERACT_THRESH)) as u8
        } else {
            let len = window_length_ns(thread);
            let cpu_pri_off =
                ((((SCHED_PRI_CPU_RANGE - 1) as u64 * thread.cpu_estimate) + len / 2) / len
                    + (1u64 << SCHED_TICK_SHIFT) / 2)
                    >> SCHED_TICK_SHIFT;
            let nice_off = (((i32::from(nice) - PRIO_MIN) as u32) * 5) / 4;
            (MIN_BATCH as u32 + cpu_pri_off.min((SCHED_PRI_CPU_RANGE - 1) as u64) as u32 + nice_off)
                .min(MAX_BATCH as u32) as u8
        };

        thread.user_priority = priority;
        thread.base_priority = priority;
        thread.refresh_priority();
    }

    fn interact_update(&self, thread: &mut Thread) {
        let total = thread.runtime_ns.saturating_add(thread.slptime_ns);
        if total < SCHED_SLP_RUN_MAX_NS {
            return;
        }

        if total > SCHED_SLP_RUN_MAX_NS * 2 {
            if thread.runtime_ns > thread.slptime_ns {
                thread.runtime_ns = SCHED_SLP_RUN_MAX_NS;
                thread.slptime_ns = 1;
            } else {
                thread.slptime_ns = SCHED_SLP_RUN_MAX_NS;
                thread.runtime_ns = 1;
            }
            return;
        }

        if total > (SCHED_SLP_RUN_MAX_NS / 5) * 6 {
            thread.runtime_ns /= 2;
            thread.slptime_ns /= 2;
            return;
        }

        thread.runtime_ns = (thread.runtime_ns / 5) * 4;
        thread.slptime_ns = (thread.slptime_ns / 5) * 4;
    }

    fn pctcpu_update(&self, thread: &mut Thread, now_ns: u64, running: bool) {
        let t_max = SCHED_CPU_DECAY_WINDOW_NS;
        let t_tgt = (t_max * SCHED_CPU_DECAY_NUMER) / SCHED_CPU_DECAY_DENOM;
        let elapsed_ns = now_ns.saturating_sub(thread.cpu_last_update_ns);

        if elapsed_ns >= t_tgt {
            thread.cpu_estimate = if running {
                t_tgt.saturating_mul(1u64 << SCHED_TICK_SHIFT)
            } else {
                0
            };
            thread.cpu_window_start_ns = now_ns.saturating_sub(t_tgt);
            thread.cpu_last_update_ns = now_ns;
            return;
        }

        if now_ns.saturating_sub(thread.cpu_window_start_ns) >= t_max {
            // Scale the estimate into the new window with a single widened
            // multiply so the running total does not lose a division's worth
            // of precision every time the window slides.
            let len = u128::from(window_length_ns(thread));
            let scaled = (u128::from(thread.cpu_estimate)
                * u128::from(t_tgt.saturating_sub(elapsed_ns)))
                / len;
            thread.cpu_estimate = scaled.min(u128::from(u64::MAX)) as u64;
            thread.cpu_window_start_ns = now_ns.saturating_sub(t_tgt);
        }

        if running {
            thread.cpu_estimate = thread
                .cpu_estimate
                .saturating_add(elapsed_ns.saturating_mul(1u64 << SCHED_TICK_SHIFT));
        }

        thread.cpu_last_update_ns = now_ns;
    }

    fn reap_exited(&self, cpu_id: usize) {
        loop {
            let exited = {
                let mut cpu = self.cpu(cpu_id).lock();
                cpu.exited.pop()
            };
            let Some(thread) = exited else {
                return;
            };

            if let Some(root) = thread_ref(thread).address_space_root()
                && crate::mem::address_space_root_active(root)
            {
                let space = thread_ref(thread)
                    .take_address_space()
                    .expect("sched: exited user thread lost its address space");
                self.retired_address_spaces.lock().push((root, space));
                crate::mem::release_inactive_address_space_roots();
            }

            unregister_thread(thread);
            // SAFETY: exited threads were removed from all run/current queues
            // and the live-thread registry before reaching the reaper.
            unsafe { free_thread(thread.as_ptr()) };
        }
    }

    fn reap_all_exited(&self) {
        let mut retired = self.retired_address_spaces.lock();
        let mut index = 0;
        while index < retired.len() {
            if crate::mem::address_space_root_active(retired[index].0) {
                index += 1;
            } else {
                retired.swap_remove(index);
            }
        }
        drop(retired);

        for cpu_id in 0..self.cpu_count {
            self.reap_exited(cpu_id);
        }
    }

    fn publish_deferred_exit(&self, cpu_id: usize) {
        let mut cpu = self.cpu(cpu_id).lock();
        if let Some(thread) = cpu.deferred_exit.take() {
            cpu.exited.push(thread);
        }
    }

    /// Applies a new nice value and re-places the thread if it is queued.
    ///
    /// The nice value itself is published atomically, so the recomputed
    /// priority only needs the owning CPU's lock to move the thread between
    /// run-queue buckets.
    fn apply_nice(&self, thread: ThreadPtr, nice: i8) {
        thread_ref(thread).set_nice(nice);
        self.reprioritize(thread, |scheduler, thread| {
            scheduler.priority_update(thread);
        });
    }

    /// Sets or clears the priority lent to `thread` by blocked lock waiters.
    fn adjust_boost(&self, thread: ThreadPtr, boost: Option<u8>) {
        self.reprioritize(thread, |_, thread| {
            match boost {
                Some(priority) => {
                    thread.boost_count = thread.boost_count.saturating_add(1);
                    thread.lent_priority = thread.lent_priority.min(priority);
                }
                None => {
                    thread.boost_count = thread.boost_count.saturating_sub(1);
                    if thread.boost_count == 0 {
                        thread.lent_priority = u8::MAX;
                    }
                }
            }
            thread.refresh_priority();
        });
    }

    /// Recomputes a thread's priority under its owning CPU's lock, moving it
    /// between run-queue buckets when it is queued.
    fn reprioritize(&self, thread: ThreadPtr, update: impl FnOnce(&Self, &mut Thread)) {
        let owner_cpu = thread_ref(thread).cpu();
        let mut cpu = self.cpu(owner_cpu).lock();
        // The owner can change while the lock is being taken; a stale update
        // is harmless because the next statclock tick recomputes anyway.
        if thread_ref(thread).cpu() != owner_cpu {
            return;
        }

        let thread_mut = thread_mut(thread);
        if thread_mut.state == ThreadState::Exited {
            return;
        }

        if !thread_mut.is_runq_linked() {
            update(self, thread_mut);
            if cpu.current_thread() == Some(thread) {
                cpu.need_resched = true;
            }
            return;
        }

        cpu.dequeue_thread(thread);
        update(self, thread_mut);
        cpu.enqueue_existing(thread_mut, EnqueueKind::Normal);
    }
}

/// Registers a newly created thread in the global live-thread list.
fn register_thread(thread: ThreadPtr) {
    let mut all = ALL_THREADS.lock();
    // SAFETY: the thread allocation stays live until the reaper unlinks it in
    // `unregister_thread` and then frees it.
    all.get_or_insert_with(|| LinkedList::new(AllThreadAdapter::NEW))
        .push_back(unsafe { UnsafeRef::from_raw(thread.as_ptr()) });
}

/// Removes a thread from the global live-thread list before it is freed.
fn unregister_thread(thread: ThreadPtr) {
    let mut all = ALL_THREADS.lock();
    let Some(list) = all.as_mut() else {
        return;
    };
    // SAFETY: the thread is either linked in this list or unlinked; both are
    // valid inputs for a cursor created from its own link.
    unsafe {
        if thread.as_ref().all_link.is_linked() {
            list.cursor_mut_from_ptr(thread.as_ptr()).remove();
        }
    }
}

/// Runs `f` for every live thread that belongs to `pid`.
///
/// The registry lock is held across the callback on purpose: membership in
/// `ALL_THREADS` is what keeps a thread allocation alive, because the reaper
/// must acquire this same lock in `unregister_thread` before it can call
/// `free_thread`. Snapshotting pointers and releasing the lock first would let
/// a thread be freed before the callback dereferenced it.
///
/// Lock order is `ALL_THREADS` then the per-CPU scheduler lock; the reaper
/// releases its CPU lock before unregistering, so there is no cycle.
fn for_each_process_thread(pid: usize, mut f: impl FnMut(ThreadPtr)) {
    let all = ALL_THREADS.lock();
    let Some(list) = all.as_ref() else {
        return;
    };

    for thread in list.iter() {
        if thread.belongs_to_process(pid) {
            f(NonNull::from(thread));
        }
    }
}

#[inline]
fn thread_ref(thread: ThreadPtr) -> &'static Thread {
    // SAFETY: scheduler `ThreadPtr` values point to leaked thread allocations
    // that remain live until explicit reaping.
    unsafe { thread.as_ref() }
}

#[inline]
fn thread_mut(thread: ThreadPtr) -> &'static mut Thread {
    // SAFETY: callers hold the scheduler lock that owns this thread's mutable
    // state and therefore have exclusive access.
    unsafe { &mut *thread.as_ptr() }
}

fn window_length_ns(thread: &Thread) -> u64 {
    thread
        .cpu_last_update_ns
        .saturating_sub(thread.cpu_window_start_ns)
        .max(1)
}
fn should_preempt(priority: u8, current: u8, remote: bool) -> bool {
    if priority >= current {
        return false;
    }

    if current >= MIN_IDLE {
        return true;
    }

    if priority < MIN_KERN {
        return true;
    }

    remote && priority <= MAX_INTERACT && current > MAX_INTERACT
}

fn scheduler() -> &'static Scheduler {
    SCHEDULER.get().expect("sched: init before use")
}

pub(crate) fn request_reschedule_ipi() {
    let Some(cpu) = arch::thiscpu_opt() else {
        return;
    };
    let Some(per_cpu) = smp::core_local(cpu.id).and_then(|core| core.scheduler.get()) else {
        return;
    };

    per_cpu.state.lock().need_resched = true;
}

/// Re-arms a remote CPU's local timer without forcing a context switch.
pub(crate) fn request_deadline_refresh_ipi() {
    let Some(cpu) = arch::thiscpu_opt() else {
        return;
    };
    let Some(scheduler) = SCHEDULER.get() else {
        return;
    };

    clock::set_scheduler_deadline(scheduler.current_deadline(cpu.id, clock::monotonic_ns()));
}

// Keep long-term load distribution and thread reaping out of the trap-return
// hot path and in ordinary thread context.
fn maintenance_thread() {
    let mut random = clock::monotonic_ns() ^ 0x9E37_79B9_7F4A_7C15;
    loop {
        scheduler().reap_all_exited();
        clock::sleep(Duration::from_nanos(balance_delay_ns(&mut random)));
        if scheduler().cpu_count > 1 {
            let _ = scheduler().rebalance_once();
        }
    }
}

fn balance_delay_ns(random: &mut u64) -> u64 {
    if *random == 0 {
        *random = 0xA076_1D64_78BD_642F;
    }
    *random ^= *random << 13;
    *random ^= *random >> 7;
    *random ^= *random << 17;

    (SCHED_BALANCE_INTERVAL_NS / 2).saturating_add(*random % SCHED_BALANCE_INTERVAL_NS)
}

/// Builds scheduler state and queues the first CPU0 kernel task.
pub fn bootstrap<F, R>(init_task: F)
where
    F: FnOnce() -> R + Send + 'static,
{
    if SCHEDULER.get().is_some() {
        return;
    }

    SCHEDULER.call_once(|| Scheduler::new(smp::cpu_count()));
    scheduler().bootstrap(init_task);
}

/// Starts scheduling on the current CPU and never returns.
pub fn start() -> ! {
    scheduler().start_current_cpu()
}

/// Spawns a regular timeshare kernel thread and returns its thread ID.
pub fn run<F, R>(task: F) -> usize
where
    F: FnOnce() -> R + Send + 'static,
{
    scheduler().spawn(task)
}

/// Spawns the initial thread of a userspace process.
pub(crate) fn run_user(process: Arc<Process>, entry: u64, stack: u64) -> usize {
    scheduler().spawn_user(process, entry, stack)
}

/// Spawns another userspace thread in an existing process.
pub(crate) fn run_user_thread(
    process: Arc<Process>,
    entry: u64,
    stack: u64,
    thread_pointer: u64,
) -> usize {
    scheduler().spawn_user_thread(process, entry, stack, thread_pointer)
}

/// Spawns a fork child from the current user syscall frame.
pub(crate) fn run_forked_user(
    process: Arc<Process>,
    parent_frame: &TrapFrame,
    thread_pointer: u64,
) -> usize {
    scheduler().spawn_forked_user(process, parent_frame, thread_pointer)
}

/// Spawns an interrupt-thread style task with the supplied argument.
pub fn create_ithread<F, R>(task: F, arg: u64) -> usize
where
    F: FnOnce(u64) -> R + Send + 'static,
{
    scheduler().spawn_ithread(task, arg, MAX_ITHD)
}

/// Spawns an interrupt-thread style task at a ULE ithread priority in `0..=7`.
pub fn create_ithread_with_priority<F, R>(task: F, arg: u64, priority: u8) -> usize
where
    F: FnOnce(u64) -> R + Send + 'static,
{
    scheduler().spawn_ithread(task, arg, priority)
}

/// Lowest nice value, granting the most CPU share.
pub(crate) use crate::sys::thread::NICE_MIN;

/// Highest nice value, granting the least CPU share.
pub(crate) use crate::sys::thread::NICE_MAX;

/// Returns the thread currently executing on this CPU.
pub(crate) fn current_thread() -> *mut Thread {
    scheduler().current_thread(arch::thiscpu().id)
}

/// Returns the current thread on this CPU, if scheduling has started.
pub(crate) fn current_thread_opt() -> Option<*mut Thread> {
    let cpu_id = arch::thiscpu_opt()?.id;
    SCHEDULER
        .get()?
        .current_thread_opt(cpu_id)
        .map(ThreadPtr::as_ptr)
}

/// Applies `nice` to every live thread of `pid`.
///
/// Returns the number of threads updated.
pub(crate) fn set_process_nice(pid: usize, nice: i8) -> usize {
    let nice = nice.clamp(NICE_MIN, NICE_MAX);
    let scheduler = scheduler();
    let mut updated = 0usize;
    for_each_process_thread(pid, |thread| {
        scheduler.apply_nice(thread, nice);
        updated += 1;
    });
    updated
}

/// Returns the nice value of any live thread belonging to `pid`.
pub(crate) fn process_nice(pid: usize) -> Option<i8> {
    let mut nice = None;
    for_each_process_thread(pid, |thread| {
        if nice.is_none() {
            nice = Some(thread_ref(thread).nice());
        }
    });
    nice
}

/// Returns the scheduling priority of a thread, for priority inheritance.
pub(crate) fn thread_priority(thread: *mut Thread) -> u8 {
    let thread = NonNull::new(thread).expect("sched: null thread priority query");
    thread_ref(thread).effective_priority()
}

/// Lends `priority` to `thread` while a waiter blocks on a lock it holds.
///
/// Every call must be paired with exactly one [`unboost_priority`].
pub(crate) fn boost_priority(thread: *mut Thread, priority: u8) {
    let Some(thread) = NonNull::new(thread) else {
        return;
    };
    let Some(scheduler) = SCHEDULER.get() else {
        return;
    };
    scheduler.adjust_boost(thread, Some(priority));
}

/// Releases one priority boost previously applied to `thread`.
pub(crate) fn unboost_priority(thread: *mut Thread) {
    let Some(thread) = NonNull::new(thread) else {
        return;
    };
    let Some(scheduler) = SCHEDULER.get() else {
        return;
    };
    scheduler.adjust_boost(thread, None);
}

/// Publishes a thread whose final stack switch has completed.
pub(crate) fn publish_deferred_exit() {
    scheduler().publish_deferred_exit(arch::thiscpu().id);
}

/// Parks the current thread until a matching wake event occurs.
pub(crate) fn park_current(thread: *mut Thread, seq: u64) {
    scheduler().park_current(arch::thiscpu().id, thread, seq)
}

/// Wakes a previously parked thread and requeues it on a target CPU.
pub(crate) fn wake(thread: *mut Thread, seq: u64) -> bool {
    scheduler().wake_thread(thread, seq)
}

/// Terminates the current thread and immediately schedules a replacement.
pub fn exit_current() -> ! {
    scheduler().exit_current(arch::thiscpu().id)
}

/// Handles reschedule decisions before returning from a trap.
pub fn trap_return(frame: &mut TrapFrame) -> *mut TrapFrame {
    scheduler().trap_return(arch::thiscpu().id, frame)
}
