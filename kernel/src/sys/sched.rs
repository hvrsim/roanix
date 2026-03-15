//!
//! # Scheduler
//!
//! ULE-inspired kernel scheduler with per-CPU run queues, wake affinity, idle
//! pulling, and periodic load balancing.
//!

use alloc::boxed::Box;
use core::{
    array,
    ptr::NonNull,
    sync::atomic::{AtomicU64, AtomicUsize, Ordering},
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
            allocate_thread, free_thread, idle_task, ExitedThreadAdapter, Thread, ThreadAdapter,
            ThreadClass, ThreadFlags, ThreadState, WakeResult,
        },
    },
};

type TrapFrame = crate::arch::cpu::TrapFrame;
type ThreadPtr = NonNull<Thread>;

const PRIO_MIN: i32 = -20;
const PRIO_MAX: i32 = 20;

const MAX_ITHD: u8 = 15;
const MIN_KERN: u8 = 48;
const MIN_TIMESHARE: u8 = 88;
const MIN_INTERACT: u8 = MIN_TIMESHARE;
const MIN_BATCH: u8 = MIN_TIMESHARE + 48;
const MIN_IDLE: u8 = 224;

const MAX_INTERACT: u8 = MIN_INTERACT + 48 - 1;
const MAX_BATCH: u8 = MIN_IDLE - 1;
const MAX_IDLE: u8 = 255;

const SCHED_INTERACT_HALF: u32 = 50;
const SCHED_INTERACT_THRESH: u32 = 30;
const SCHED_TICK_SHIFT: u32 = 10;
const SCHED_TICK_SECS: u64 = 11;
const SCHED_CPU_DECAY_NUMER: u64 = 10;
const SCHED_CPU_DECAY_DENOM: u64 = 11;
const SCHED_SLP_RUN_MAX: u32 = 5 * 100 * (1 << SCHED_TICK_SHIFT);

const SCHED_SLICE_DEFAULT_DIVISOR: u32 = 10;
const SCHED_SLICE_MIN_DIVISOR: u32 = 6;
const SCHED_REBALANCE_INTERVAL: u64 = 8;
const SCHED_WAKE_AFFINITY_SLICES: u64 = 2;

const PRI_BATCH_RANGE: usize = (MAX_BATCH - MIN_BATCH + 1) as usize;
const SCHED_PRI_NRESV: u32 = (((PRIO_MAX - PRIO_MIN) as u32) * 5) / 4;
const SCHED_PRI_CPU_RANGE: u32 = PRI_BATCH_RANGE as u32 - SCHED_PRI_NRESV;

const REAPER_INTERVAL_MS: u64 = 10;

static SCHEDULER: Once<&'static Scheduler> = Once::new();

/// Snapshot of aggregate scheduler load.
#[derive(Copy, Clone, Debug, Default)]
pub struct SchedulerStats {
    /// CPUs known to the scheduler.
    pub cpu_count: usize,
    /// CPUs currently online.
    pub online_cpus: usize,
    /// Sum of scheduler load across CPUs.
    pub total_load: usize,
    /// Sum of timeshare/system load counters across CPUs.
    pub total_sysload: usize,
    /// CPUs with runnable work queued.
    pub runnable_cpus: usize,
    /// CPUs with a current thread installed.
    pub active_cpus: usize,
    /// Non-idle threads currently executing.
    pub running_threads: usize,
    /// Runnable threads waiting on per-CPU run queues.
    pub queued_threads: usize,
    /// Owned non-idle threads that are neither running nor queued.
    pub blocked_threads: usize,
    /// Highest per-CPU load observed in the snapshot.
    pub busiest_load: usize,
    /// CPU id carrying `busiest_load`.
    pub busiest_cpu: usize,
    /// Deepest per-CPU run queue observed in the snapshot.
    pub busiest_runq: usize,
    /// CPU id carrying `busiest_runq`.
    pub busiest_runq_cpu: usize,
    /// Monotonic thread ids allocated so far, excluding 0.
    pub total_threads_created: usize,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum EnqueueKind {
    Normal,
    Preempted,
    Stolen,
}

impl EnqueueKind {
    fn push_front(self) -> bool {
        match self {
            // A thread that consumed its slice must yield position to peers at
            // the same priority, otherwise it can livelock at the head of the
            // bucket and starve never-run work.
            Self::Preempted => false,
            Self::Normal | Self::Stolen => false,
        }
    }

    fn uses_pick_cursor(self) -> bool {
        matches!(self, Self::Preempted | Self::Stolen)
    }
}

#[derive(Copy, Clone)]
struct SchedulerConfig {
    realstathz: u32,
    tickincr: u32,
    sched_slice: u32,
    sched_slice_min: u32,
    rebalance_interval: u64,
    wake_affinity: u64,
}

impl SchedulerConfig {
    fn new() -> Self {
        let realstathz = (1_000_000_000u64 / clock::STAT_INTERVAL_NS) as u32;
        let sched_slice = (realstathz / SCHED_SLICE_DEFAULT_DIVISOR).max(1);
        let sched_slice_min = (sched_slice / SCHED_SLICE_MIN_DIVISOR).max(1);

        Self {
            realstathz,
            tickincr: 1 << SCHED_TICK_SHIFT,
            sched_slice,
            sched_slice_min,
            rebalance_interval: SCHED_REBALANCE_INTERVAL,
            wake_affinity: u64::from(sched_slice) * SCHED_WAKE_AFFINITY_SLICES,
        }
    }
}

struct ExitedThreads {
    list: LinkedList<ExitedThreadAdapter>,
}

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
        for bucket in start..=end {
            let index = usize::from(bucket);
            if self.bits[index / 64] & (1u64 << (index % 64)) == 0 {
                continue;
            }

            if let Some(thread) = self.queues[index].front().get() {
                return Some(NonNull::from(thread));
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

    fn has_runnable(&self) -> bool {
        self.bits.iter().any(|bits| *bits != 0)
    }

    fn len(&self) -> usize {
        self.len
    }
}

struct CpuState {
    runq: RunQueue,
    current: Option<ThreadPtr>,
    idle: Option<ThreadPtr>,
    exited: ExitedThreads,
    online: bool,
    load: usize,
    sysload: usize,
    ts_insert_cursor: u8,
    ts_pick_cursor: u8,
    ts_ticks: u8,
    need_resched: bool,
}

unsafe impl Send for CpuState {}

impl CpuState {
    fn new(cpu_id: usize) -> Self {
        Self {
            runq: RunQueue::new(),
            current: None,
            idle: None,
            exited: ExitedThreads::new(),
            online: cpu_id == 0,
            load: 0,
            sysload: 0,
            ts_insert_cursor: 0,
            ts_pick_cursor: 0,
            ts_ticks: 0,
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

    fn take_next_thread(&mut self, cpu_id: usize, global_ticks: u64) -> ThreadPtr {
        let next = self.best_runnable().unwrap_or_else(|| self.idle_thread());
        if !thread_ref(next).is_idle() {
            self.dequeue_thread(next);
        }

        thread_mut(next).mark_running(cpu_id, global_ticks);
        self.current = Some(next);
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

    fn consider_preemption(&mut self, priority: u8) {
        if let Some(current) = self.current {
            if should_preempt(priority, thread_ref(current).priority) {
                self.need_resched = true;
            }
        }
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

    fn advance_timeshare_epoch(&mut self) {
        if self.ts_insert_cursor != self.ts_pick_cursor {
            return;
        }

        self.ts_ticks = self.ts_ticks.wrapping_add(1);
        let advance = 2u16 - u16::from(self.ts_ticks / 4);
        self.ts_insert_cursor =
            ((u16::from(self.ts_insert_cursor) + advance) % PRI_BATCH_RANGE as u16) as u8;
        self.ts_ticks %= 4;
        self.advance_pick_cursor(false);
    }

    fn quantum_for(&self, thread: &Thread, config: &SchedulerConfig) -> u32 {
        if thread.class != ThreadClass::Timeshare {
            return config.sched_slice;
        }

        let runnable_load = self.sysload.saturating_sub(1) as u32;
        if runnable_load >= SCHED_SLICE_MIN_DIVISOR {
            config.sched_slice_min
        } else if runnable_load <= 1 {
            config.sched_slice
        } else {
            config.sched_slice / runnable_load
        }
    }

    fn steal_candidate(&self) -> Option<ThreadPtr> {
        self.runq
            .first_in_range(MIN_INTERACT, MAX_INTERACT)
            .or_else(|| self.runq.first_timeshare(self.ts_pick_cursor))
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
    next_balance: AtomicU64,
    config: SchedulerConfig,
}

impl Scheduler {
    fn new(cpu_count: usize) -> Self {
        let config = SchedulerConfig::new();
        Self {
            cpu_count,
            next_tid: AtomicUsize::new(1),
            next_balance: AtomicU64::new(config.rebalance_interval),
            config,
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
                ThreadFlags::IDLE | ThreadFlags::NOLOAD,
                || idle_task(),
            );
            let mut cpu = self.cpu(cpu_id).lock();
            cpu.set_idle(idle);
        }

        for cpu_id in 0..self.cpu_count {
            let reaper = self.alloc_thread(
                cpu_id,
                ThreadClass::Ithread,
                MAX_ITHD,
                ThreadFlags::NOLOAD,
                move || reaper_thread(cpu_id),
            );
            self.cpu(cpu_id)
                .lock()
                .adopt_thread(reaper, EnqueueKind::Normal);
        }
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
            clock::global_ticks(),
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

    fn spawn_ithread<F, R>(&self, task: F, arg: u64) -> usize
    where
        F: FnOnce(u64) -> R + Send + 'static,
    {
        self.spawn_on(
            self.pick_spawn_cpu(),
            ThreadClass::Ithread,
            MAX_ITHD,
            move || task(arg),
        )
    }

    fn spawn_on<F, R>(&self, cpu_id: usize, class: ThreadClass, priority: u8, task: F) -> usize
    where
        F: FnOnce() -> R + Send + 'static,
    {
        let thread = self.alloc_thread(cpu_id, class, priority, ThreadFlags::empty(), task);
        let tid = thread.id;
        let priority = thread.priority;
        let current_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);

        let kick_remote = {
            let mut cpu = self.cpu(cpu_id).lock();
            cpu.adopt_thread(thread, EnqueueKind::Normal);
            cpu.consider_preemption(priority);
            cpu.online && current_cpu != Some(cpu_id)
        };

        if kick_remote {
            let _ = smp::send_ipi(request_reschedule_ipi, smp::IpiTarget::Single(cpu_id));
        }
        tid
    }

    fn pick_spawn_cpu(&self) -> usize {
        let preferred = arch::thiscpu_opt().map(|cpu| cpu.id).unwrap_or(0);
        let mut best_id = 0usize;
        let mut best_load = usize::MAX;
        let mut preferred_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let cpu = self.cpu(cpu_id).lock();
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

    fn wake_target_cpu(&self, thread: &Thread, wake_cpu: Option<usize>, global: u64) -> usize {
        if thread.class == ThreadClass::Ithread {
            if let Some(cpu_id) = wake_cpu {
                if smp::is_online(cpu_id) {
                    return cpu_id;
                }
            }
        }

        let owner_cpu = thread.cpu;
        let mut best_id = owner_cpu;
        let mut best_load = usize::MAX;
        let mut owner_load = usize::MAX;
        let mut wake_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let cpu = self.cpu(cpu_id).lock();
            if !cpu.online {
                continue;
            }

            if cpu_id == owner_cpu {
                owner_load = cpu.load;
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
            && global.saturating_sub(thread.rltick) <= self.config.wake_affinity
            && owner_load.saturating_sub(usize::from(thread.counts_towards_load()))
                <= best_load.saturating_add(1)
        {
            return owner_cpu;
        }

        if let Some(cpu_id) = wake_cpu {
            if smp::is_online(cpu_id) && wake_load <= best_load {
                return cpu_id;
            }
        }

        best_id
    }

    fn on_tick(&self, cpu_id: usize, global: u64) {
        {
            let mut cpu = self.cpu(cpu_id).lock();
            let Some(current_ptr) = cpu.current_thread() else {
                return;
            };
            let current = thread_mut(current_ptr);

            cpu.advance_timeshare_epoch();

            current.rltick = global;
            self.pctcpu_update(current, global, true);

            if current.class == ThreadClass::Timeshare {
                current.runtime = current.runtime.saturating_add(self.config.tickincr);
                self.interact_update(current);
                self.priority_update(current);
            }

            if current.is_idle() {
                cpu.need_resched = true;
            } else {
                current.slice = current.slice.saturating_add(1);
                if current.slice >= cpu.quantum_for(current, &self.config) {
                    current.slice = 0;
                    current.flags.insert(ThreadFlags::SLICEEND);
                    cpu.need_resched = true;
                }

                if let Some(best) = cpu.best_runnable() {
                    if should_preempt(thread_ref(best).priority, current.priority) {
                        cpu.need_resched = true;
                    }
                }
            }
        }

        self.maybe_rebalance(global);
    }

    fn trap_return(&self, cpu_id: usize, frame: &mut TrapFrame) -> *mut TrapFrame {
        smp::drain_ipi_queue();

        let should_pull = {
            let mut cpu = self.cpu(cpu_id).lock();
            let Some(current_ptr) = cpu.current_thread() else {
                return frame;
            };
            let current = thread_mut(current_ptr);
            assert!(
                current.has_valid_stack_canary(),
                "sched: thread {} stack canary corrupted on cpu{}",
                current.id,
                cpu_id,
            );
            current.frame = frame;

            let should_switch = current.state != ThreadState::Running
                || cpu.need_resched
                || (current.is_idle() && cpu.runq.has_runnable());
            if !should_switch {
                return frame;
            }

            cpu.need_resched = false;

            if current.state == ThreadState::Running && !current.is_idle() {
                let kind = if current.take_slice_end() {
                    EnqueueKind::Preempted
                } else {
                    EnqueueKind::Normal
                };
                cpu.enqueue_existing(current, kind);
            }

            current.is_idle() && !cpu.runq.has_runnable()
        };

        if should_pull {
            let _ = self.try_idle_pull(cpu_id);
        }

        let next = self.schedule_next(cpu_id);
        let next_frame = thread_ref(next).frame;
        self.activate_current(next);
        next_frame
    }

    fn start_cpu(&self, cpu_id: usize) -> ! {
        arch::irqset(false);
        self.cpu(cpu_id).lock().online = true;

        let _ = self.try_idle_pull(cpu_id);
        let next = self.schedule_next(cpu_id);
        self.activate_current(next);

        unsafe {
            crate::arch::cpu::start_first_thread(thread_ref(next).frame);
        }
    }

    fn exit_current(&self, cpu_id: usize) -> ! {
        arch::irqset(false);

        {
            let mut cpu = self.cpu(cpu_id).lock();
            let current = cpu
                .current_thread()
                .expect("sched: no current thread to exit");
            let current_thread = thread_mut(current);
            assert!(!current_thread.is_idle(), "sched: idle thread exited");

            current_thread.mark_exited();
            cpu.release_thread(current_thread);
            cpu.exited.push(current);
            cpu.current = None;
            cpu.need_resched = false;
        }

        let _ = self.try_idle_pull(cpu_id);
        let next = self.schedule_next(cpu_id);
        self.activate_current(next);

        unsafe {
            crate::arch::cpu::start_first_thread(thread_ref(next).frame);
        }
    }

    fn schedule_next(&self, cpu_id: usize) -> ThreadPtr {
        self.cpu(cpu_id)
            .lock()
            .take_next_thread(cpu_id, clock::global_ticks())
    }

    fn current_thread(&self, cpu_id: usize) -> *mut Thread {
        self.current_thread_opt(cpu_id)
            .expect("sched: current thread unavailable")
            .as_ptr()
    }

    fn current_thread_opt(&self, cpu_id: usize) -> Option<ThreadPtr> {
        self.cpu(cpu_id).lock().current_thread()
    }

    fn park_current(&self, cpu_id: usize, seq: u64) {
        {
            let mut cpu = self.cpu(cpu_id).lock();
            let current = cpu
                .current_thread()
                .expect("sched: no current thread to park");
            let current_thread = thread_mut(current);
            assert!(!current_thread.is_idle(), "sched: idle thread cannot sleep");

            if !current_thread.mark_parked(seq) {
                return;
            }

            current_thread.mark_blocked();
            cpu.need_resched = true;
        }

        let _ = smp::send_ipi(request_reschedule_ipi, smp::IpiTarget::Single(cpu_id));
    }

    fn wake_thread(&self, thread: *mut Thread, seq: u64) -> bool {
        let thread_ptr = NonNull::new(thread).expect("sched: wake received null thread");
        let wake = unsafe { thread_ptr.as_ref().wake(seq) };
        match wake {
            WakeResult::Stale => return false,
            WakeResult::Pending => return true,
            WakeResult::Parked => {}
        }

        let global = clock::global_ticks();
        let wake_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
        let owner_cpu = unsafe { thread_ptr.as_ref().cpu };
        let target_cpu = self.wake_target_cpu(unsafe { thread_ptr.as_ref() }, wake_cpu, global);

        self.finish_wakeup(thread_ptr, global);

        if owner_cpu == target_cpu {
            let priority = unsafe { thread_ptr.as_ref().priority };
            let kick_remote = {
                let mut cpu = self.cpu(target_cpu).lock();
                let thread = thread_mut(thread_ptr);
                assert_eq!(
                    thread.state,
                    ThreadState::Blocked,
                    "sched: attempted to wake a non-blocked thread"
                );
                if thread.is_runq_linked() {
                    cpu.dequeue_thread(thread_ptr);
                }
                cpu.enqueue_existing(thread, EnqueueKind::Normal);
                cpu.consider_preemption(priority);
                cpu.online && wake_cpu != Some(target_cpu)
            };

            if kick_remote {
                let _ = smp::send_ipi(request_reschedule_ipi, smp::IpiTarget::Single(target_cpu));
            }
            return true;
        }

        let current_cpu = wake_cpu;
        let mut kick_remote = false;
        self.with_cpu_pair(owner_cpu, target_cpu, |owner, target| {
            let thread = thread_mut(thread_ptr);
            assert_eq!(
                thread.state,
                ThreadState::Blocked,
                "sched: attempted to wake a non-blocked thread"
            );
            if thread.is_runq_linked() {
                owner.dequeue_thread(thread_ptr);
            }

            owner.release_thread(thread);
            thread.cpu = target_cpu;
            target.adopt_thread(thread, EnqueueKind::Normal);
            target.consider_preemption(thread.priority);
            kick_remote = target.online && current_cpu != Some(target_cpu);
        });

        if kick_remote {
            let _ = smp::send_ipi(request_reschedule_ipi, smp::IpiTarget::Single(target_cpu));
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

        let mut source_cpu = None;
        let mut source_load = dst_load.saturating_add(1);

        for cpu_id in 0..self.cpu_count {
            if cpu_id == dst_cpu {
                continue;
            }

            let cpu = self.cpu(cpu_id).lock();
            if !cpu.online || cpu.load <= source_load {
                continue;
            }

            source_cpu = Some(cpu_id);
            source_load = cpu.load;
        }

        source_cpu
            .map(|src_cpu| self.transfer_candidate(src_cpu, dst_cpu, true))
            .unwrap_or(false)
    }

    fn transfer_candidate(&self, src_cpu: usize, dst_cpu: usize, require_idle_dst: bool) -> bool {
        let current_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
        let mut kick_remote = false;

        let moved = self.with_cpu_pair(src_cpu, dst_cpu, |src, dst| {
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
            dst.adopt_thread(thread, EnqueueKind::Stolen);
            dst.consider_preemption(thread.priority);
            kick_remote = dst.online && current_cpu != Some(dst_cpu);
            true
        });

        if moved && kick_remote {
            let _ = smp::send_ipi(request_reschedule_ipi, smp::IpiTarget::Single(dst_cpu));
        }
        moved
    }

    fn maybe_rebalance(&self, global: u64) {
        if self.cpu_count <= 1 {
            return;
        }

        let mut next = self.next_balance.load(Ordering::Acquire);
        loop {
            if global < next {
                return;
            }

            let scheduled = global.saturating_add(self.config.rebalance_interval);
            match self.next_balance.compare_exchange_weak(
                next,
                scheduled,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => break,
                Err(observed) => next = observed,
            }
        }

        let _ = self.rebalance_once();
    }

    fn rebalance_once(&self) -> bool {
        let mut busiest_cpu = None;
        let mut busiest_load = 0usize;
        let mut idlest_cpu = None;
        let mut idlest_load = usize::MAX;

        for cpu_id in 0..self.cpu_count {
            let cpu = self.cpu(cpu_id).lock();
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

    fn finish_wakeup(&self, thread: ThreadPtr, global: u64) {
        let thread = thread_mut(thread);
        thread.slice = 0;
        thread.flags.remove(ThreadFlags::SLICEEND);

        if thread.class != ThreadClass::Timeshare {
            return;
        }

        self.pctcpu_update(thread, global, false);
        let slept = global.saturating_sub(thread.rltick).min(u32::MAX as u64) as u32;
        thread.slptime = thread.slptime.saturating_add(slept);
        self.interact_update(thread);
        self.priority_update(thread);
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

        let cpu = arch::thiscpu();
        cpu.kernel_stack = thread.stack_top.as_u64();
        cpu.user_stack = 0;
        cpu.current_thread = thread.id;
        unsafe {
            crate::arch::cpu::prepare_thread_frame(thread.frame);
        }
    }

    fn interact_score(&self, thread: &Thread) -> u32 {
        if SCHED_INTERACT_THRESH <= SCHED_INTERACT_HALF && thread.runtime >= thread.slptime {
            return SCHED_INTERACT_HALF;
        }

        if thread.runtime > thread.slptime {
            let div = (thread.runtime / SCHED_INTERACT_HALF).max(1);
            return SCHED_INTERACT_HALF
                + (SCHED_INTERACT_HALF - thread.slptime.saturating_div(div));
        }

        if thread.slptime > thread.runtime {
            let div = (thread.slptime / SCHED_INTERACT_HALF).max(1);
            return thread.runtime.saturating_div(div);
        }

        if thread.runtime != 0 {
            SCHED_INTERACT_HALF
        } else {
            0
        }
    }

    fn priority_update(&self, thread: &mut Thread) {
        if thread.class != ThreadClass::Timeshare {
            return;
        }

        let score = self
            .interact_score(thread)
            .saturating_add((thread.nice as i32).max(0) as u32);
        let priority = if score < SCHED_INTERACT_THRESH {
            MIN_INTERACT
                + (((MAX_INTERACT - MIN_INTERACT + 1) as u32 * score) / SCHED_INTERACT_THRESH) as u8
        } else {
            let len = tick_length(thread).max(1);
            let cpu_pri_off =
                ((((SCHED_PRI_CPU_RANGE - 1) as u64 * thread.ticks as u64) + len / 2) / len
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
        let total = thread.runtime.saturating_add(thread.slptime);
        if total < SCHED_SLP_RUN_MAX {
            return;
        }

        if total > SCHED_SLP_RUN_MAX * 2 {
            if thread.runtime > thread.slptime {
                thread.runtime = SCHED_SLP_RUN_MAX;
                thread.slptime = 1;
            } else {
                thread.slptime = SCHED_SLP_RUN_MAX;
                thread.runtime = 1;
            }
            return;
        }

        if total > (SCHED_SLP_RUN_MAX / 5) * 6 {
            thread.runtime /= 2;
            thread.slptime /= 2;
            return;
        }

        thread.runtime = (thread.runtime / 5) * 4;
        thread.slptime = (thread.slptime / 5) * 4;
    }

    fn pctcpu_update(&self, thread: &mut Thread, global: u64, running: bool) {
        let t_max = self.config.realstathz as u64 * SCHED_TICK_SECS;
        let t_tgt = (((t_max << SCHED_TICK_SHIFT) * SCHED_CPU_DECAY_NUMER) / SCHED_CPU_DECAY_DENOM)
            >> SCHED_TICK_SHIFT;
        let lu_span = global.saturating_sub(thread.ltick);

        if lu_span >= t_tgt {
            thread.ticks = if running {
                (t_tgt << SCHED_TICK_SHIFT) as u32
            } else {
                0
            };
            thread.ftick = global.saturating_sub(t_tgt);
            thread.ltick = global;
            return;
        }

        if global.saturating_sub(thread.ftick) >= t_max {
            let len = tick_length(thread).max(1);
            thread.ticks = ((thread.ticks as u64 / len) * t_tgt.saturating_sub(lu_span))
                .min(u32::MAX as u64) as u32;
            thread.ftick = global.saturating_sub(t_tgt);
        }

        if running {
            thread.ticks = thread
                .ticks
                .saturating_add((lu_span << SCHED_TICK_SHIFT).min(u32::MAX as u64) as u32);
        }

        thread.ltick = global;
    }

    fn stats(&self) -> SchedulerStats {
        let mut stats = SchedulerStats {
            cpu_count: self.cpu_count,
            online_cpus: smp::online_cpus(),
            total_threads_created: self.next_tid.load(Ordering::Relaxed).saturating_sub(1),
            ..SchedulerStats::default()
        };

        for cpu_id in 0..self.cpu_count {
            let cpu = self.cpu(cpu_id).lock();
            let queued = cpu.runq.len();
            let running = cpu
                .current
                .map(|thread| usize::from(!thread_ref(thread).is_idle()))
                .unwrap_or(0);
            stats.total_load += cpu.load;
            stats.total_sysload += cpu.sysload;
            stats.active_cpus += usize::from(cpu.current.is_some());
            stats.running_threads += running;
            stats.queued_threads += queued;
            stats.blocked_threads += cpu.load.saturating_sub(queued + running);
            stats.runnable_cpus += usize::from(cpu.runq.has_runnable());
            if cpu.load > stats.busiest_load {
                stats.busiest_load = cpu.load;
                stats.busiest_cpu = cpu_id;
            }
            if queued > stats.busiest_runq {
                stats.busiest_runq = queued;
                stats.busiest_runq_cpu = cpu_id;
            }
        }

        stats
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

            unsafe {
                free_thread(thread.as_ptr());
            }
        }
    }
}

#[inline]
fn thread_ref(thread: ThreadPtr) -> &'static Thread {
    unsafe { thread.as_ref() }
}

#[inline]
fn thread_mut(thread: ThreadPtr) -> &'static mut Thread {
    unsafe { &mut *thread.as_ptr() }
}

fn tick_length(thread: &Thread) -> u64 {
    thread.ltick.saturating_sub(thread.ftick).max(1)
}

fn should_preempt(priority: u8, current: u8) -> bool {
    if priority >= current {
        return false;
    }

    if current >= MIN_IDLE {
        return true;
    }

    if priority <= MIN_KERN {
        return true;
    }

    priority <= MAX_INTERACT && current > MAX_INTERACT
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

fn reaper_thread(cpu_id: usize) {
    loop {
        scheduler().reap_exited(cpu_id);
        clock::sleep(Duration::from_millis(REAPER_INTERVAL_MS));
    }
}

/// Initializes the scheduler and per-CPU idle/reaper threads.
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
    scheduler().spawn_ithread(task, arg)
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

/// Parks the current thread until a matching wake event occurs.
pub(crate) fn park_current(seq: u64) {
    scheduler().park_current(arch::thiscpu().id, seq)
}

/// Wakes a previously parked thread and requeues it on a target CPU.
pub(crate) fn wake(thread: *mut Thread, seq: u64) -> bool {
    scheduler().wake_thread(thread, seq)
}

/// Terminates the current thread and immediately schedules a replacement.
pub fn exit_current() -> ! {
    scheduler().exit_current(arch::thiscpu().id)
}

/// Delivers a scheduler statistics tick for the current CPU.
pub fn stat_tick(global: u64, _percpu: u64) {
    scheduler().on_tick(arch::thiscpu().id, global);
}

/// Handles reschedule decisions before returning from a trap.
pub fn trap_return(frame: &mut TrapFrame) -> *mut TrapFrame {
    scheduler().trap_return(arch::thiscpu().id, frame)
}

/// Returns a point-in-time snapshot of scheduler load and thread creation.
pub fn stats() -> SchedulerStats {
    scheduler().stats()
}
