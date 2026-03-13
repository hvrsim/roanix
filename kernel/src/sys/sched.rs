//!
//! # Scheduler
//!
//! FreeBSD ULE-inspired kernel scheduler with per-CPU run queues and stealing.
//!

use alloc::{boxed::Box, vec::Vec};
use core::{
    array, ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

use intrusive_collections::LinkedList;
use log::info;

use crate::{
    arch,
    sys::{
        clock,
        smp::{self, IrqSpinLock},
        thread::{
            allocate_thread, idle_task, Thread, ThreadAdapter, ThreadClass, ThreadFlags,
            ThreadState,
        },
    },
};

type TrapFrame = crate::arch::cpu::TrapFrame;

const PRIO_MIN: i32 = -20;
const PRIO_MAX: i32 = 20;

const MIN_IDLE: u8 = 224;
const MIN_KERN: u8 = 48;
const MIN_TIMESHARE: u8 = 88;
const MIN_INTERACT: u8 = MIN_TIMESHARE;
const MIN_BATCH: u8 = MIN_TIMESHARE + 48;

const MAX_ITHD: u8 = 15;
const MAX_BATCH: u8 = MIN_IDLE - 1;
const MAX_INTERACT: u8 = MIN_INTERACT + 48 - 1;
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

const PRI_BATCH_RANGE: usize = (MAX_BATCH - MIN_BATCH + 1) as usize;
const SCHED_PRI_NRESV: u32 = (((PRIO_MAX - PRIO_MIN) as u32) * 5) / 4;
const SCHED_PRI_CPU_RANGE: u32 = PRI_BATCH_RANGE as u32 - SCHED_PRI_NRESV;

const SRQ_BORROWING: u32 = 1 << 0;
const SRQ_PREEMPTED: u32 = 1 << 1;

static SCHEDULER: IrqSpinLock<Option<&'static Scheduler>> = IrqSpinLock::new(None);

struct RunQueue {
    bits: [u64; 4],
    queues: [LinkedList<ThreadAdapter>; 256],
}

impl RunQueue {
    fn new() -> Self {
        Self {
            bits: [0; 4],
            queues: array::from_fn(|_| LinkedList::new(ThreadAdapter::NEW)),
        }
    }

    fn add(&mut self, idx: u8, thread: &'static Thread, flags: u32) {
        let idx_usize = idx as usize;
        self.bits[idx_usize / 64] |= 1u64 << (idx_usize % 64);
        if flags & SRQ_PREEMPTED != 0 {
            self.queues[idx_usize].push_front(thread);
        } else {
            self.queues[idx_usize].push_back(thread);
        }
    }

    fn remove(&mut self, idx: u8, thread: &'static Thread) -> bool {
        let idx_usize = idx as usize;
        unsafe {
            let mut cursor = self.queues[idx_usize].cursor_mut_from_ptr(thread);
            let _ = cursor.remove();
        }
        let empty = self.queues[idx_usize].front().get().is_none();
        if empty {
            self.bits[idx_usize / 64] &= !(1u64 << (idx_usize % 64));
        }
        empty
    }

    fn first_in_range(&self, start: u8, end: u8) -> Option<&'static Thread> {
        for idx in start..=end {
            if self.bits[idx as usize / 64] & (1u64 << (idx as usize % 64)) == 0 {
                continue;
            }

            if let Some(thread) = self.queues[idx as usize].front().get() {
                return Some(unsafe { &*(thread as *const Thread) });
            }
        }

        None
    }

    fn first_timeshare(&self, off: u8) -> Option<&'static Thread> {
        let start = MIN_BATCH.saturating_add(off);
        self.first_in_range(start, MAX_BATCH).or_else(|| {
            (off != 0)
                .then(|| self.first_in_range(MIN_BATCH, start - 1))
                .flatten()
        })
    }

    fn has_runnable(&self) -> bool {
        self.bits.iter().any(|bits| *bits != 0)
    }
}

struct CpuQueue {
    runq: RunQueue,
    current: *mut Thread,
    idle: *mut Thread,
    online: bool,
    load: usize,
    sysload: usize,
    lowpri: u8,
    ts_off: u8,
    ts_deq_off: u8,
    ts_ticks: u8,
    need_resched: bool,
}

unsafe impl Send for CpuQueue {}

impl CpuQueue {
    fn new(id: usize) -> Self {
        Self {
            runq: RunQueue::new(),
            current: ptr::null_mut(),
            idle: ptr::null_mut(),
            online: id == 0,
            load: 0,
            sysload: 0,
            lowpri: MAX_IDLE,
            ts_off: 0,
            ts_deq_off: 0,
            ts_ticks: 0,
            need_resched: false,
        }
    }

    fn set_idle(&mut self, idle: &'static mut Thread) {
        self.idle = idle;
    }

    fn choose(&self) -> Option<&'static Thread> {
        self.runq
            .first_in_range(0, MAX_INTERACT)
            .or_else(|| self.runq.first_timeshare(self.ts_deq_off))
            .or_else(|| self.runq.first_in_range(MIN_IDLE, MAX_IDLE))
    }

    fn choose_or_idle(&self) -> *mut Thread {
        self.choose()
            .map(|thread| thread as *const Thread as *mut Thread)
            .unwrap_or(self.idle)
    }

    fn dequeue(&mut self, thread: &'static Thread) {
        let empty = self.runq.remove(thread.rqindex, thread);
        if thread.priority >= MIN_BATCH
            && thread.priority <= MAX_BATCH
            && empty
            && self.ts_deq_off + MIN_BATCH == thread.rqindex
        {
            self.advance_ts_deq_off(true);
        }
    }

    fn enqueue_tracked(&mut self, thread: &'static mut Thread, flags: u32) {
        let counted = thread.counts_towards_load();
        self.enqueue(thread, flags);
        if counted {
            self.load += 1;
            self.sysload += 1;
        }
    }

    fn migrate_out(&mut self, thread: &Thread) {
        if thread.counts_towards_load() {
            self.load = self.load.saturating_sub(1);
            self.sysload = self.sysload.saturating_sub(1);
        }
    }

    fn enqueue(&mut self, thread: &'static mut Thread, flags: u32) {
        let idx = self.queue_index(thread.priority, flags);
        thread.mark_ready(idx);
        self.lowpri = self.lowpri.min(thread.priority);
        self.runq.add(idx, thread, flags);
    }

    fn consider_preemption(&mut self, priority: u8) {
        if let Some(current) = unsafe { self.current.as_ref() } {
            if should_preempt(priority, current.priority) {
                self.need_resched = true;
            }
        }
    }

    fn queue_index(&self, priority: u8, flags: u32) -> u8 {
        if !(MIN_BATCH..=MAX_BATCH).contains(&priority) {
            return priority;
        }

        let mut idx = if flags & (SRQ_BORROWING | SRQ_PREEMPTED) != 0 {
            self.ts_deq_off as usize
        } else {
            (priority - MIN_BATCH) as usize + self.ts_off as usize
        } % PRI_BATCH_RANGE;

        if self.ts_deq_off != self.ts_off && idx == self.ts_deq_off as usize {
            idx = (idx + PRI_BATCH_RANGE - 1) % PRI_BATCH_RANGE;
        }

        MIN_BATCH + idx as u8
    }

    fn advance_ts_deq_off(&mut self, mut current_empty: bool) {
        while self.ts_deq_off != self.ts_off {
            if current_empty {
                current_empty = false;
            } else if self
                .runq
                .first_in_range(MIN_BATCH + self.ts_deq_off, MIN_BATCH + self.ts_deq_off)
                .is_some()
            {
                break;
            }

            self.ts_deq_off = ((self.ts_deq_off as usize + 1) % PRI_BATCH_RANGE) as u8;
        }
    }

    fn slice_for(&self, thread: &Thread, sched_slice: u32, sched_slice_min: u32) -> u32 {
        if thread.class != ThreadClass::Timeshare {
            return sched_slice;
        }

        let load = self.sysload.saturating_sub(1) as u32;
        if load >= SCHED_SLICE_MIN_DIVISOR {
            sched_slice_min
        } else if load <= 1 {
            sched_slice
        } else {
            sched_slice / load
        }
    }

    fn refresh_lowpri(&mut self) {
        let current = unsafe { self.current.as_ref() };
        let candidate = self
            .choose()
            .map(|thread| thread.priority)
            .unwrap_or(MAX_IDLE);
        self.lowpri = current
            .map(|thread| thread.priority.min(candidate))
            .unwrap_or(candidate);
    }

    fn steal_candidate(&self) -> Option<*mut Thread> {
        self.runq
            .first_in_range(MIN_INTERACT, MAX_INTERACT)
            .or_else(|| self.runq.first_timeshare(self.ts_deq_off))
            .map(|thread| thread as *const Thread as *mut Thread)
    }

    fn take_next_thread(&mut self, cpu_id: usize, global_ticks: u64) -> *mut Thread {
        let next_ptr = self.choose_or_idle();
        if !unsafe { &*next_ptr }.is_idle() {
            self.dequeue(unsafe { &*next_ptr });
        }

        unsafe {
            (&mut *next_ptr).mark_running(cpu_id, global_ticks);
        }
        self.current = next_ptr;
        self.refresh_lowpri();
        next_ptr
    }
}

struct Scheduler {
    cpus: Box<[IrqSpinLock<CpuQueue>]>,
    next_tid: AtomicUsize,
    realstathz: u32,
    tickincr: u32,
    sched_slice: u32,
    sched_slice_min: u32,
}

impl Scheduler {
    fn new(cpu_count: usize) -> Self {
        let realstathz = (1_000_000_000u64 / clock::STAT_INTERVAL_NS) as u32;
        let sched_slice = (realstathz / SCHED_SLICE_DEFAULT_DIVISOR).max(1);
        let sched_slice_min = (sched_slice / SCHED_SLICE_MIN_DIVISOR).max(1);
        let tickincr = 1 << SCHED_TICK_SHIFT;
        let mut cpus = Vec::with_capacity(cpu_count);

        for id in 0..cpu_count {
            cpus.push(IrqSpinLock::new(CpuQueue::new(id)));
        }

        Self {
            cpus: cpus.into_boxed_slice(),
            next_tid: AtomicUsize::new(1),
            realstathz,
            tickincr,
            sched_slice,
            sched_slice_min,
        }
    }

    fn bootstrap(&self) {
        for cpu_id in 0..self.cpus.len() {
            let idle = self.alloc_thread(
                cpu_id,
                ThreadClass::Idle,
                MAX_IDLE,
                ThreadFlags::IDLE | ThreadFlags::NOLOAD,
                || idle_task(),
            );
            let mut cpu = self.cpu(cpu_id).lock();
            cpu.set_idle(idle);
            cpu.refresh_lowpri();
        }
    }

    fn cpu(&self, id: usize) -> &IrqSpinLock<CpuQueue> {
        &self.cpus[id]
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

    fn pick_spawn_cpu(&self) -> usize {
        let preferred = arch::thiscpu_opt().map(|cpu| cpu.id).unwrap_or(0);
        let mut best_id = 0usize;
        let mut best_load = usize::MAX;

        for id in 0..self.cpus.len() {
            let cpu = self.cpu(id).lock();
            if !cpu.online {
                continue;
            }

            let better = cpu.load < best_load || (cpu.load == best_load && id == preferred);
            if better {
                best_id = id;
                best_load = cpu.load;
            }
        }

        best_id
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
        let mut kick_remote = false;
        let mut cpu = self.cpu(cpu_id).lock();
        cpu.enqueue_tracked(thread, 0);
        cpu.consider_preemption(priority);
        if cpu.online && current_cpu != Some(cpu_id) {
            kick_remote = true;
        }
        cpu.refresh_lowpri();
        drop(cpu);

        if kick_remote {
            smp::send_ipi(cpu_id);
        }
        tid
    }

    fn on_tick(&self, cpu_id: usize, global: u64) {
        let mut cpu = self.cpu(cpu_id).lock();
        let current = unsafe { cpu.current.as_mut() };
        let Some(current) = current else {
            return;
        };

        if cpu.ts_off == cpu.ts_deq_off {
            cpu.ts_ticks = cpu.ts_ticks.wrapping_add(1);
            let advance = 2u16 - (cpu.ts_ticks / 4) as u16;
            cpu.ts_off = ((cpu.ts_off as u16 + advance) % PRI_BATCH_RANGE as u16) as u8;
            cpu.ts_ticks %= 4;
            cpu.advance_ts_deq_off(false);
        }

        current.rltick = global;
        self.pctcpu_update(current, global, true);

        match current.class {
            ThreadClass::Timeshare => {
                current.runtime = current.runtime.saturating_add(self.tickincr);
                self.interact_update(current);
                self.priority_update(current);
            }
            ThreadClass::Ithread | ThreadClass::Idle => {}
        }

        if current.is_idle() {
            cpu.need_resched = true;
            cpu.refresh_lowpri();
            return;
        }

        current.slice = current.slice.saturating_add(1);
        if current.slice >= cpu.slice_for(current, self.sched_slice, self.sched_slice_min) {
            current.slice = 0;
            current.flags.insert(ThreadFlags::SLICEEND);
            cpu.need_resched = true;
        }

        if let Some(best) = cpu.choose() {
            if should_preempt(best.priority, current.priority) {
                cpu.need_resched = true;
            }
        }

        cpu.refresh_lowpri();
    }

    fn trap_return(&self, cpu_id: usize, frame: &mut TrapFrame) -> *mut TrapFrame {
        {
            let mut cpu = self.cpu(cpu_id).lock();
            let current = unsafe { cpu.current.as_mut() };
            let Some(current) = current else {
                return frame;
            };

            current.frame = frame;

            let must_switch = current.state != ThreadState::Running
                || cpu.need_resched
                || (current.flags.contains(ThreadFlags::IDLE) && cpu.runq.has_runnable());
            if !must_switch {
                cpu.refresh_lowpri();
                return frame;
            }

            cpu.need_resched = false;

            if current.state == ThreadState::Running && !current.is_idle() {
                let flags = if current.take_slice_end() {
                    SRQ_PREEMPTED
                } else {
                    0
                };
                cpu.enqueue(current, flags);
            }
        }

        let need_steal = {
            let cpu = self.cpu(cpu_id).lock();
            let current_idle = unsafe { cpu.current.as_ref() }
                .map(|thread| thread.flags.contains(ThreadFlags::IDLE))
                .unwrap_or(false);
            current_idle && !cpu.runq.has_runnable()
        };
        if need_steal {
            let _ = self.try_steal(cpu_id);
        }

        let next = self.schedule_next(cpu_id);
        let frame = next.frame;
        self.activate_current(next);
        frame
    }

    fn start_cpu(&self, cpu_id: usize) -> ! {
        arch::irqset(false);
        {
            let mut cpu = self.cpu(cpu_id).lock();
            cpu.online = true;
        }

        let _ = self.try_steal(cpu_id);
        let next = self.schedule_next(cpu_id);

        self.activate_current(next);
        unsafe { crate::arch::cpu::start_first_thread(next.frame) }
    }

    fn exit_current(&self, cpu_id: usize) -> ! {
        arch::irqset(false);
        {
            let mut cpu = self.cpu(cpu_id).lock();
            let current =
                unsafe { cpu.current.as_mut() }.expect("sched: no current thread to exit");
            assert!(!current.is_idle(), "sched: idle thread exited");

            current.mark_exited();
            if current.counts_towards_load() {
                cpu.load = cpu.load.saturating_sub(1);
                cpu.sysload = cpu.sysload.saturating_sub(1);
            }
            cpu.current = ptr::null_mut();
            cpu.need_resched = false;
            cpu.refresh_lowpri();
        }

        let _ = self.try_steal(cpu_id);
        let next = self.schedule_next(cpu_id);

        self.activate_current(next);
        unsafe { crate::arch::cpu::start_first_thread(next.frame) }
    }

    fn schedule_next(&self, cpu_id: usize) -> &'static mut Thread {
        let mut cpu = self.cpu(cpu_id).lock();
        let next = cpu.take_next_thread(cpu_id, clock::global_ticks());
        drop(cpu);
        unsafe { &mut *next }
    }

    fn current_thread(&self, cpu_id: usize) -> &'static mut Thread {
        let cpu = self.cpu(cpu_id).lock();
        let current = cpu.current;
        drop(cpu);

        assert!(
            !current.is_null(),
            "sched: no current thread on cpu {cpu_id}"
        );
        unsafe { &mut *current }
    }

    fn park_current(&self, cpu_id: usize) {
        {
            let mut cpu = self.cpu(cpu_id).lock();
            let current =
                unsafe { cpu.current.as_mut() }.expect("sched: no current thread to park");
            assert!(!current.is_idle(), "sched: idle thread cannot sleep");

            if !current.mark_parked() {
                return;
            }

            current.mark_blocked();
            cpu.need_resched = true;
            cpu.refresh_lowpri();
        }

        arch::reschedule();
    }

    fn wake_thread(&self, thread: &'static mut Thread) {
        if !thread.wake() {
            return;
        }

        let cpu_id = thread.cpu;
        let priority = thread.priority;
        let current_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
        let mut kick_remote = false;
        let mut cpu = self.cpu(cpu_id).lock();

        assert_eq!(
            thread.state,
            ThreadState::Blocked,
            "sched: attempted to wake a non-blocked thread"
        );

        cpu.enqueue(thread, 0);
        cpu.consider_preemption(priority);
        if cpu.online && current_cpu != Some(cpu_id) {
            kick_remote = true;
        }
        cpu.refresh_lowpri();
        drop(cpu);

        if kick_remote {
            smp::send_ipi(cpu_id);
        }
    }

    fn try_steal(&self, dst_id: usize) -> bool {
        for src_id in 0..self.cpus.len() {
            if src_id == dst_id {
                continue;
            }

            let (mut a, mut b) = if dst_id < src_id {
                (self.cpu(dst_id).lock(), self.cpu(src_id).lock())
            } else {
                (self.cpu(src_id).lock(), self.cpu(dst_id).lock())
            };

            let (dst, src) = if dst_id < src_id {
                (&mut *a, &mut *b)
            } else {
                (&mut *b, &mut *a)
            };

            if !dst.online || !src.online || src.load <= dst.load + 1 {
                continue;
            }
            if dst.runq.has_runnable() {
                return true;
            }

            let Some(candidate_ptr) = src.steal_candidate() else {
                continue;
            };

            let candidate = unsafe { &*candidate_ptr };
            src.dequeue(candidate);
            src.migrate_out(candidate);

            let thread = unsafe { &mut *candidate_ptr };
            let priority = thread.priority;
            thread.cpu = dst_id;
            dst.enqueue_tracked(thread, SRQ_BORROWING);
            dst.consider_preemption(priority);

            src.refresh_lowpri();
            dst.refresh_lowpri();
            return true;
        }

        false
    }

    fn activate_current(&self, thread: &Thread) {
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
        let sum = thread.runtime.saturating_add(thread.slptime);
        if sum < SCHED_SLP_RUN_MAX {
            return;
        }

        if sum > SCHED_SLP_RUN_MAX * 2 {
            if thread.runtime > thread.slptime {
                thread.runtime = SCHED_SLP_RUN_MAX;
                thread.slptime = 1;
            } else {
                thread.slptime = SCHED_SLP_RUN_MAX;
                thread.runtime = 1;
            }
            return;
        }

        if sum > (SCHED_SLP_RUN_MAX / 5) * 6 {
            thread.runtime /= 2;
            thread.slptime /= 2;
            return;
        }

        thread.runtime = (thread.runtime / 5) * 4;
        thread.slptime = (thread.slptime / 5) * 4;
    }

    fn pctcpu_update(&self, thread: &mut Thread, global: u64, run: bool) {
        let t_max = self.realstathz as u64 * SCHED_TICK_SECS;
        let t_tgt = (((t_max << SCHED_TICK_SHIFT) * SCHED_CPU_DECAY_NUMER) / SCHED_CPU_DECAY_DENOM)
            >> SCHED_TICK_SHIFT;
        let lu_span = global.saturating_sub(thread.ltick);

        if lu_span >= t_tgt {
            thread.ticks = if run {
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

        if run {
            thread.ticks = thread
                .ticks
                .saturating_add((lu_span << SCHED_TICK_SHIFT).min(u32::MAX as u64) as u32);
        }
        thread.ltick = global;
    }
}

fn tick_length(thread: &Thread) -> u64 {
    thread.ltick.saturating_sub(thread.ftick).max(1)
}

fn should_preempt(pri: u8, current: u8) -> bool {
    if pri >= current {
        return false;
    }

    if current >= MIN_IDLE {
        return true;
    }

    if pri <= MIN_KERN {
        return true;
    }

    pri <= MAX_INTERACT && current > MAX_INTERACT
}

fn scheduler() -> &'static Scheduler {
    let guard = SCHEDULER.lock();
    guard.as_ref().copied().expect("sched: init before use")
}

/// Initializes the global scheduler and installs per-CPU idle threads.
pub fn init() {
    let mut guard = SCHEDULER.lock();
    if guard.is_some() {
        return;
    }

    let scheduler = Box::leak(Box::new(Scheduler::new(smp::cpu_count())));
    scheduler.bootstrap();

    info!(
        "sched: ULE ready on {} CPU(s) (slice={} ticks, min_slice={} ticks)",
        scheduler.cpus.len(),
        scheduler.sched_slice,
        scheduler.sched_slice_min
    );
    *guard = Some(scheduler);
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
pub(crate) fn current_thread() -> &'static mut Thread {
    scheduler().current_thread(arch::thiscpu().id)
}

/// Parks the current thread until a matching wake event occurs.
pub(crate) fn park_current() {
    scheduler().park_current(arch::thiscpu().id)
}

/// Wakes a previously parked thread and requeues it on its owner CPU.
pub(crate) fn wake(thread: &'static mut Thread) {
    scheduler().wake_thread(thread)
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
