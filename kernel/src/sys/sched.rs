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

use alloc::boxed::Box;
use core::{
    array,
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};

use intrusive_collections::LinkedList;
use spin::Once;

use crate::{
    arch,
    sys::{
        clock,
        smp::{self, IrqSpinLock},
        thread::{
            ExitedThreadAdapter, Thread, ThreadAdapter, ThreadClass, ThreadFlags, ThreadState,
            WakeResult, allocate_thread, free_thread, idle_task,
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
const SCHED_SLICE_TICKS: u64 = SCHED_STAT_HZ / 10;
const SCHED_SLICE_DEFAULT_NS: u64 = SCHED_STAT_INTERVAL_NS * SCHED_SLICE_TICKS;
const SCHED_SLICE_MIN_DIVISOR: u32 = 6;
const SCHED_BALANCE_INTERVAL_NS: u64 = NSEC_PER_SEC;
const SCHED_WAKE_AFFINITY_NS: u64 = 2_000_000;
const SCHED_STEAL_THRESHOLD: usize = 2;

const PRI_BATCH_RANGE: usize = (MAX_BATCH - MIN_BATCH + 1) as usize;
const SCHED_PRI_CPU_RANGE: u32 = PRI_BATCH_RANGE as u32 - SCHED_PRI_NRESV;

const _: () = assert!(MAX_INTERACT == 114);
const _: () = assert!(MIN_BATCH == 115);
const _: () = assert!(PRI_BATCH_RANGE == 109);

static SCHEDULER: Once<&'static Scheduler> = Once::new();

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
    sched_slice_min_ns: u64,
    wake_affinity_ns: u64,
}

impl SchedulerConfig {
    fn new() -> Self {
        let sched_slice_ns = SCHED_SLICE_DEFAULT_NS;
        let sched_slice_min_ns = (sched_slice_ns / u64::from(SCHED_SLICE_MIN_DIVISOR)).max(1);

        Self {
            sched_slice_ns,
            sched_slice_min_ns,
            wake_affinity_ns: SCHED_WAKE_AFFINITY_NS,
        }
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
        self.list.push_back(thread_ref(thread));
    }

    fn pop(&mut self) -> Option<ThreadPtr> {
        self.list.pop_front().map(NonNull::from)
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
            thread_ref.cpu,
            thread_ref.rqindex,
            thread_ref.priority,
        );
        let index = usize::from(bucket);
        self.bits[index / 64] |= 1u64 << (index % 64);
        if push_front {
            self.queues[index].push_front(thread_ref);
        } else {
            self.queues[index].push_back(thread_ref);
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
            thread_ref.cpu,
            thread_ref.rqindex,
            thread_ref.priority,
        );
        // SAFETY: the run-queue link is known to be linked in this exact queue,
        // and the queue is exclusively borrowed.
        unsafe {
            let mut cursor = self.queues[index].cursor_mut_from_ptr(thread_ref);
            let _ = cursor.remove();
        }
        self.len = self.len.saturating_sub(1);

        let empty = self.queues[index].front().get().is_none();
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

    fn first_in_range_if<F>(&self, start: u8, end: u8, mut pred: F) -> Option<ThreadPtr>
    where
        F: FnMut(ThreadPtr) -> bool,
    {
        for bucket in start..=end {
            let index = usize::from(bucket);
            if self.bits[index / 64] & (1u64 << (index % 64)) == 0 {
                continue;
            }

            for thread in self.queues[index].iter() {
                let thread = NonNull::from(thread);
                if pred(thread) {
                    return Some(thread);
                }
            }
        }

        None
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
        self.bits.iter().any(|bits| *bits != 0)
    }
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
    // Prevent remote steals while this CPU is still unwinding off the old
    // thread's kernel stack after selecting a replacement.
    switching: bool,
    need_resched: bool,
}

// SAFETY: each `CpuState` is only accessed through its per-CPU
// `IrqSpinLock`, including cross-CPU scheduler operations.
unsafe impl Send for CpuState {}

impl CpuState {
    fn new(cpu_id: usize) -> Self {
        Self {
            runq: RunQueue::new(),
            current: None,
            idle: None,
            exited: ExitedThreads::new(),
            deferred_exit: None,
            online: cpu_id == 0,
            load: 0,
            sysload: 0,
            ts_insert_cursor: 0,
            ts_pick_cursor: 0,
            ts_ticks: 0,
            next_statclock_ns: 0,
            switching: false,
            need_resched: false,
        }
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
        self.switching = previous.is_some() && previous != Some(next);
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
        }
    }

    fn release_thread(&mut self, thread: &Thread) {
        if thread.counts_towards_load() {
            self.load = self.load.saturating_sub(1);
            self.sysload = self.sysload.saturating_sub(1);
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

        let runnable_load = self.sysload.saturating_sub(1) as u64;
        if runnable_load >= u64::from(SCHED_SLICE_MIN_DIVISOR) {
            config.sched_slice_min_ns
        } else if runnable_load <= 1 {
            config.sched_slice_ns
        } else {
            config.sched_slice_ns / runnable_load
        }
    }

    fn steal_candidate(&self) -> Option<ThreadPtr> {
        if self.switching {
            return None;
        }

        let can_steal = |candidate: ThreadPtr| {
            // During trap return, the outgoing `current` thread can be briefly
            // requeued before the CPU commits to its replacement. Never allow
            // that still-executing stack to migrate to another CPU.
            self.current != Some(candidate) && thread_ref(candidate).can_migrate()
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

    pub(crate) fn is_online(&self) -> bool {
        self.state.lock().online
    }
}

struct Scheduler {
    cpu_count: usize,
    next_tid: AtomicUsize,
    config: SchedulerConfig,
}

impl Scheduler {
    fn new(cpu_count: usize) -> Self {
        Self {
            cpu_count,
            next_tid: AtomicUsize::new(1),
            config: SchedulerConfig::new(),
        }
    }

    fn bootstrap(&self) {
        for cpu_id in 0..self.cpu_count {
            smp::core_local(cpu_id)
                .unwrap_or_else(|| panic!("sched: missing core-local record for cpu{cpu_id}"))
                .scheduler
                .call_once(|| PerCpuScheduler::new(cpu_id));
        }

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

        let maint = self.alloc_thread(
            0,
            ThreadClass::Timeshare,
            MIN_INTERACT,
            ThreadFlags::NOLOAD | ThreadFlags::NO_MIGRATE,
            maintenance_thread,
        );
        self.cpu(0).lock().adopt_thread(maint, EnqueueKind::Normal);
    }

    fn cpu(&self, cpu_id: usize) -> &IrqSpinLock<CpuState> {
        &self.cpu_local(cpu_id).state
    }

    fn cpu_local(&self, cpu_id: usize) -> &'static PerCpuScheduler {
        smp::core_local(cpu_id)
            .unwrap_or_else(|| panic!("sched: missing core-local record for cpu{cpu_id}"))
            .scheduler
            .get()
            .unwrap_or_else(|| panic!("sched: cpu{cpu_id} scheduler not initialized"))
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
        let tid = thread_ref(thread).id;
        let priority = thread_ref(thread).priority;
        let current_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
        let initial_target = self
            .cpu(cpu_id)
            .try_lock()
            .map(|_| cpu_id)
            .or(current_cpu.filter(|current| smp::is_online(*current)))
            .unwrap_or(cpu_id);
        let fallback_cpu = current_cpu
            .filter(|current| smp::is_online(*current))
            .unwrap_or(initial_target);

        let kick_cpu = {
            let (target_cpu, mut cpu) = if initial_target == fallback_cpu {
                (initial_target, self.cpu(initial_target).lock())
            } else if let Some(cpu) = self.cpu(initial_target).try_lock() {
                (initial_target, cpu)
            } else {
                (fallback_cpu, self.cpu(fallback_cpu).lock())
            };
            let had_runnable = cpu.runq.has_runnable();
            let thread = thread_mut(thread);
            thread.cpu = target_cpu;
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
        if let Some((kick_cpu, kind)) = kick_cpu {
            self.kick_cpu(kick_cpu, current_cpu, kind);
        }
        tid
    }

    fn pick_spawn_cpu(&self) -> usize {
        let preferred = arch::thiscpu_opt().map(|cpu| cpu.id).unwrap_or(0);
        let mut best_id = 0usize;
        let mut best_load = usize::MAX;
        let mut preferred_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let Some(cpu) = self.cpu(cpu_id).try_lock() else {
                continue;
            };
            if !cpu.online {
                continue;
            }

            if cpu_id == preferred {
                preferred_load = cpu.load;
            }
            if cpu.load < best_load {
                best_load = cpu.load;
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
        if !thread.can_migrate() && smp::is_online(thread.cpu) {
            return thread.cpu;
        }

        if thread.class == ThreadClass::Ithread
            && let Some(cpu_id) = wake_cpu
            && smp::is_online(cpu_id)
        {
            return cpu_id;
        }

        let owner_cpu = thread.cpu;
        let mut best_id = owner_cpu;
        let mut best_load = usize::MAX;
        let mut wake_load = usize::MAX;
        let mut owner_idle = false;

        for cpu_id in 0..self.cpu_count {
            let Some(cpu) = self.cpu(cpu_id).try_lock() else {
                continue;
            };
            if !cpu.online {
                continue;
            }
            if cpu_id == owner_cpu {
                owner_idle = cpu.is_immediately_available();
            }
            if Some(cpu_id) == wake_cpu {
                wake_load = cpu.load;
            }
            if cpu.load < best_load {
                best_load = cpu.load;
                best_id = cpu_id;
            }
        }

        if smp::is_online(owner_cpu)
            && owner_idle
            && now_ns.saturating_sub(thread.last_run_ns) <= self.config.wake_affinity_ns
        {
            return owner_cpu;
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
        let elapsed_ns = now_ns.saturating_sub(current.last_run_ns);

        if !current.is_idle() && current.state != ThreadState::Exited {
            current.last_run_ns = now_ns;
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
            current.flags.insert(ThreadFlags::SLICEEND);
            if current.class == ThreadClass::Ithread {
                let demoted = current.base_priority.saturating_add(1);
                if demoted < MAX_ITHD {
                    current.base_priority = demoted;
                    current.priority = demoted;
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

    fn current_deadline(&self, cpu_id: usize, now_ns: u64) -> u64 {
        let cpu = self.cpu(cpu_id).lock();
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

    fn trap_return(&self, cpu_id: usize, frame: &mut TrapFrame) -> *mut TrapFrame {
        self.publish_deferred_exit(cpu_id);
        smp::drain_ipi_queue();
        let now_ns = clock::monotonic_ns();

        let (should_switch, try_switch_steal, try_idle_pull) = {
            let mut cpu = self.cpu(cpu_id).lock();
            let Some(current_ptr) = cpu.current_thread() else {
                cpu.switching = false;
                return frame;
            };
            {
                let current = thread_mut(current_ptr);
                assert!(
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
                cpu.switching = false;
                (false, false, current.is_idle() && !cpu.runq.has_runnable())
            } else {
                cpu.switching = true;
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

                (true, !cpu.runq.has_runnable(), false)
            }
        };

        if !should_switch {
            if try_idle_pull && self.try_idle_pull(cpu_id) {
                let next = self.schedule_next(cpu_id, now_ns);
                let next_frame = thread_ref(next).frame;
                self.activate_current(next);
                clock::set_scheduler_deadline(self.current_deadline(cpu_id, now_ns));
                return next_frame;
            }
            clock::set_scheduler_deadline(self.current_deadline(cpu_id, now_ns));
            return frame;
        }

        if try_switch_steal {
            let _ = self.try_switch_steal(cpu_id);
        }

        let next = self.schedule_next(cpu_id, now_ns);
        let next_frame = thread_ref(next).frame;
        self.activate_current(next);
        clock::set_scheduler_deadline(self.current_deadline(cpu_id, now_ns));
        next_frame
    }

    fn start_cpu(&self, cpu_id: usize) -> ! {
        arch::irqset(false);
        self.cpu(cpu_id).lock().online = true;

        let _ = self.try_idle_pull(cpu_id);
        let now_ns = clock::monotonic_ns();
        let next = self.schedule_next(cpu_id, now_ns);
        self.activate_current(next);
        clock::set_scheduler_deadline(self.current_deadline(cpu_id, now_ns));

        // SAFETY: `next` is a live scheduler-owned thread with a validated
        // frame and this path never returns after transferring control.
        unsafe {
            crate::arch::cpu::start_first_thread(thread_ref(next).frame);
        }
    }

    fn exit_current(&self, cpu_id: usize) -> ! {
        arch::irqset(false);
        self.publish_deferred_exit(cpu_id);

        {
            let mut cpu = self.cpu(cpu_id).lock();
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
        let next = self.schedule_next(cpu_id, now_ns);
        self.activate_current(next);
        clock::set_scheduler_deadline(self.current_deadline(cpu_id, now_ns));

        // SAFETY: `next` is a live scheduler-owned thread with a validated
        // frame and this path never returns after transferring control.
        unsafe {
            crate::arch::cpu::start_first_thread(thread_ref(next).frame);
        }
    }

    fn schedule_next(&self, cpu_id: usize, now_ns: u64) -> ThreadPtr {
        self.cpu(cpu_id).lock().take_next_thread(cpu_id, now_ns)
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
            if matches!(kind, CpuKick::Deadline) {
                if smp::in_interrupt_context() {
                    return;
                }
                clock::set_scheduler_deadline(self.current_deadline(cpu_id, clock::monotonic_ns()));
                return;
            }

            if smp::in_interrupt_context() {
                return;
            }
            if arch::irqstate() {
                arch::reschedule();
            } else {
                clock::set_scheduler_deadline(clock::monotonic_ns());
            }
            return;
        }

        let _ = smp::send_ipi(request_reschedule_ipi, smp::IpiTarget::Single(cpu_id));
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
        let owner_cpu = unsafe { thread_ptr.as_ref().cpu };
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
                self.finish_wakeup(thread, now_ns);
                let priority = thread.priority;
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
                    let should_kick =
                        cpu.consider_preemption(priority, wake_cpu != Some(target_cpu));
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
                self.finish_wakeup(thread, now_ns);
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

            self.finish_wakeup(thread, now_ns);
            thread.cpu = target_cpu;
            let priority = thread.priority;
            target.adopt_thread(thread, EnqueueKind::Normal);
            let should_kick = target.consider_preemption(priority, current_cpu != Some(target_cpu));
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

            let Some(cpu) = self.cpu(cpu_id).try_lock() else {
                continue;
            };
            if !cpu.online || cpu.switching || cpu.load < min_load || cpu.load <= source_load {
                continue;
            }

            source_cpu = Some(cpu_id);
            source_load = cpu.load;
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
            thread.cpu = dst_cpu;
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
        let mut busiest_cpu = None;
        let mut busiest_load = 0usize;
        let mut idlest_cpu = None;
        let mut idlest_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let Some(cpu) = self.cpu(cpu_id).try_lock() else {
                continue;
            };
            if !cpu.online {
                continue;
            }

            if cpu.load > busiest_load {
                busiest_load = cpu.load;
                busiest_cpu = Some(cpu_id);
            }
            if cpu.load < idlest_load {
                idlest_load = cpu.load;
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

    fn finish_wakeup(&self, thread: &mut Thread, now_ns: u64) {
        thread.slice_ns = 0;
        thread.flags.remove(ThreadFlags::SLICEEND);
        let sleep_start_ns = core::mem::replace(&mut thread.sleep_start_ns, 0);

        if thread.class == ThreadClass::Ithread {
            thread.base_priority = thread.ithread_base_priority;
            thread.priority = thread.ithread_base_priority;
            return;
        }
        if thread.class != ThreadClass::Timeshare {
            return;
        }

        let slept_ns = now_ns.saturating_sub(sleep_start_ns);
        if sleep_start_ns != 0 && slept_ns >= SCHED_SLEEP_TICK_NS {
            thread.slptime_ns = thread.slptime_ns.saturating_add(slept_ns);
            self.interact_update(thread);
            self.pctcpu_update(thread, now_ns, false);
            self.priority_update(thread);
        }
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
        crate::mem::activate_thread_space(thread.address_space());
        // SAFETY: the frame belongs to `thread`, was validated above, and is
        // exclusively prepared on the local scheduler path.
        unsafe {
            crate::arch::cpu::prepare_thread_frame(thread.frame);
        }
    }

    fn interact_score(&self, thread: &Thread) -> u64 {
        let half = u64::from(SCHED_INTERACT_HALF);

        if SCHED_INTERACT_THRESH <= SCHED_INTERACT_HALF && thread.runtime_ns >= thread.slptime_ns {
            return half;
        }

        if thread.runtime_ns > thread.slptime_ns {
            let div = (thread.runtime_ns / half).max(1);
            return half + (half - thread.slptime_ns.saturating_div(div));
        }

        if thread.slptime_ns > thread.runtime_ns {
            let div = (thread.slptime_ns / half).max(1);
            return thread.runtime_ns.saturating_div(div);
        }

        if thread.runtime_ns != 0 { half } else { 0 }
    }

    fn priority_update(&self, thread: &mut Thread) {
        if thread.class != ThreadClass::Timeshare {
            return;
        }

        let score = (self.interact_score(thread) as i64 + i64::from(thread.nice)).max(0) as u64;
        let priority = if score < u64::from(SCHED_INTERACT_THRESH) {
            MIN_INTERACT
                + (((MAX_INTERACT - MIN_INTERACT + 1) as u64 * score)
                    / u64::from(SCHED_INTERACT_THRESH)) as u8
        } else {
            let len = window_length_ns(thread).max(1);
            let cpu_pri_off =
                ((((SCHED_PRI_CPU_RANGE - 1) as u64 * thread.cpu_estimate) + len / 2) / len
                    + (1u64 << SCHED_TICK_SHIFT) / 2)
                    >> SCHED_TICK_SHIFT;
            let nice_off = (((thread.nice as i32 - PRIO_MIN) as u32) * 5) / 4;
            (MIN_BATCH as u32 + cpu_pri_off.min((SCHED_PRI_CPU_RANGE - 1) as u64) as u32 + nice_off)
                .min(MAX_BATCH as u32) as u8
        };

        thread.user_priority = priority;
        thread.base_priority = priority;
        thread.priority = priority;
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
            let len = window_length_ns(thread).max(1);
            thread.cpu_estimate =
                (thread.cpu_estimate / len).saturating_mul(t_tgt.saturating_sub(elapsed_ns));
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

            // SAFETY: exited threads were removed from all run/current queues
            // before reaching the reaper.
            unsafe { free_thread(thread.as_ptr()) };
        }
    }

    fn reap_all_exited(&self) {
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
    SCHEDULER.get().copied().expect("sched: init before use")
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

/// Initializes the scheduler and per-CPU idle threads.
pub fn init() {
    if SCHEDULER.get().is_some() {
        return;
    }

    let scheduler = Box::leak(Box::new(Scheduler::new(smp::cpu_count())));
    scheduler.bootstrap();
    SCHEDULER.call_once(|| scheduler);
}

/// Starts scheduling on the bootstrap CPU and never returns.
pub fn start() -> ! {
    scheduler().start_cpu(0)
}

/// Starts scheduling on the current secondary CPU and never returns.
pub fn start_secondary() -> ! {
    scheduler().start_cpu(arch::thiscpu().id)
}

/// Spawns a regular timeshare kernel thread and returns its thread ID.
pub fn run<F, R>(task: F) -> usize
where
    F: FnOnce() -> R + Send + 'static,
{
    scheduler().spawn(task)
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

/// Returns the thread currently executing on this CPU.
pub(crate) fn current_thread() -> *mut Thread {
    scheduler().current_thread(arch::thiscpu().id)
}

/// Returns the current thread on this CPU, if scheduling has started.
pub(crate) fn current_thread_opt() -> Option<*mut Thread> {
    let cpu_id = arch::thiscpu_opt()?.id;
    SCHEDULER
        .get()
        .copied()?
        .current_thread_opt(cpu_id)
        .map(ThreadPtr::as_ptr)
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
