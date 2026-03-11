//!
//! # Scheduler
//!
//! Early FreeBSD ULE-inspired kernel scheduler for Roanix.
//!

use core::{
    array,
    cell::UnsafeCell,
    mem::{size_of, MaybeUninit},
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};

use bitflags::bitflags;
use intrusive_collections::{intrusive_adapter, LinkedList, LinkedListLink};
use log::info;
use spin::Lazy;

use crate::{
    arch,
    mem::{self, VirtAddr, PAGE_SIZE},
    sys::{clock, smp::IrqSpinLock},
};

type TrapFrame = crate::arch::cpu::TrapFrame;

pub type ThreadEntry = extern "C" fn(usize) -> !;

const MAX_THREADS: usize = 32;
const KSTACK_PAGES: usize = 4;

const PRIO_MIN: i32 = -20;
const PRIO_MAX: i32 = 20;

const MIN_IDLE: u8 = 224;
const MIN_KERN: u8 = 48;
const MIN_TIMESHARE: u8 = 88;
const MIN_INTERACT: u8 = MIN_TIMESHARE;
const MIN_BATCH: u8 = MIN_TIMESHARE + 48;

const MAX_TIMESHARE: u8 = MIN_IDLE - 1;
const MAX_BATCH: u8 = MAX_TIMESHARE;
const MAX_INTERACT: u8 = MIN_INTERACT + 48 - 1;
const MAX_IDLE: u8 = 255;

const SCHED_INTERACT_MAX: u32 = 100;
const SCHED_INTERACT_HALF: u32 = SCHED_INTERACT_MAX / 2;
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

intrusive_adapter!(ThreadAdapter = &'static Thread: Thread { runq_link: LinkedListLink });

static THREAD_SLOTS: Lazy<[ThreadSlot; MAX_THREADS]> =
    Lazy::new(|| array::from_fn(|_| ThreadSlot::new()));

static SCHEDULER: IrqSpinLock<Option<Scheduler>> = IrqSpinLock::new(None);

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum ThreadClass {
    Timeshare,
    Idle,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum ThreadState {
    Ready,
    Running,
    Idle,
}

bitflags! {
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    struct ThreadFlags: u32 {
        const IDLE = 1 << 0;
        const NOLOAD = 1 << 1;
        const SLICEEND = 1 << 2;
    }
}

pub struct Thread {
    runq_link: LinkedListLink,
    _id: usize,
    state: ThreadState,
    class: ThreadClass,
    flags: ThreadFlags,
    priority: u8,
    base_priority: u8,
    user_priority: u8,
    nice: i8,
    cpu: usize,
    rqindex: u8,
    slice: u32,
    ftick: u64,
    ltick: u64,
    rltick: u64,
    slptime: u32,
    runtime: u32,
    ticks: u32,
    _stack_base: VirtAddr,
    stack_top: VirtAddr,
    frame: *mut TrapFrame,
}

unsafe impl Send for Thread {}
unsafe impl Sync for Thread {}

struct ThreadSlot {
    used: AtomicBool,
    thread: UnsafeCell<MaybeUninit<Thread>>,
}

unsafe impl Sync for ThreadSlot {}

impl ThreadSlot {
    const fn new() -> Self {
        Self {
            used: AtomicBool::new(false),
            thread: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    fn try_init(&self, thread: Thread) -> Option<&'static mut Thread> {
        if self
            .used
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }

        unsafe {
            (*self.thread.get()).write(thread);
            Some(&mut *(*self.thread.get()).as_mut_ptr())
        }
    }
}

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
    id: usize,
    runq: RunQueue,
    current: *mut Thread,
    idle: *mut Thread,
    load: usize,
    sysload: usize,
    lowpri: u8,
    ts_off: u8,
    ts_deq_off: u8,
    ts_ticks: u8,
    switchcnt: u16,
    oldswitchcnt: u16,
    need_resched: bool,
}

unsafe impl Send for CpuQueue {}

impl CpuQueue {
    fn new(id: usize) -> Self {
        Self {
            id,
            runq: RunQueue::new(),
            current: ptr::null_mut(),
            idle: ptr::null_mut(),
            load: 0,
            sysload: 0,
            lowpri: MAX_IDLE,
            ts_off: 0,
            ts_deq_off: 0,
            ts_ticks: 0,
            switchcnt: 0,
            oldswitchcnt: 0,
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
        if let Some(thread) = self.choose() {
            thread as *const Thread as *mut Thread
        } else {
            self.idle
        }
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

    fn enqueue_new(&mut self, thread: &'static mut Thread, flags: u32) {
        let counted = !thread.flags.contains(ThreadFlags::NOLOAD);
        self.enqueue(thread, flags);
        if counted {
            self.load += 1;
            self.sysload += 1;
        }
    }

    fn requeue_running(&mut self, thread: &'static mut Thread, flags: u32) {
        self.enqueue(thread, flags);
    }

    fn enqueue(&mut self, thread: &'static mut Thread, flags: u32) {
        let idx = self.queue_index(thread.priority, flags);
        thread.state = ThreadState::Ready;
        thread.rqindex = idx;
        self.lowpri = self.lowpri.min(thread.priority);
        self.runq.add(idx, thread, flags);
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
}

struct Scheduler {
    cpu: CpuQueue,
    next_tid: usize,
    realstathz: u32,
    tickincr: u32,
    sched_slice: u32,
    sched_slice_min: u32,
    global_ticks: u64,
}

unsafe impl Send for Scheduler {}

impl Scheduler {
    fn new() -> Self {
        let realstathz = (1_000_000_000u64 / clock::STAT_INTERVAL_NS) as u32;
        let sched_slice = (realstathz / SCHED_SLICE_DEFAULT_DIVISOR).max(1);
        let sched_slice_min = (sched_slice / SCHED_SLICE_MIN_DIVISOR).max(1);
        let tickincr = 1 << SCHED_TICK_SHIFT;

        Self {
            cpu: CpuQueue::new(0),
            next_tid: 1,
            realstathz,
            tickincr,
            sched_slice,
            sched_slice_min,
            global_ticks: 0,
        }
    }

    fn bootstrap(&mut self) {
        let idle = self.alloc_thread(
            idle_thread,
            0,
            ThreadClass::Idle,
            MAX_IDLE,
            ThreadFlags::IDLE | ThreadFlags::NOLOAD,
        );
        let init = self.alloc_thread(
            init_thread,
            0,
            ThreadClass::Timeshare,
            MIN_INTERACT,
            ThreadFlags::empty(),
        );

        self.cpu.set_idle(idle);
        self.cpu.enqueue_new(init, 0);
        self.cpu.refresh_lowpri();
    }

    fn alloc_thread(
        &mut self,
        entry: ThreadEntry,
        arg: usize,
        class: ThreadClass,
        priority: u8,
        flags: ThreadFlags,
    ) -> &'static mut Thread {
        let (stack_base, stack_top) = allocate_stack();
        let frame_addr = stack_top.as_u64() - size_of::<TrapFrame>() as u64;
        let frame = frame_addr as *mut TrapFrame;

        unsafe {
            crate::arch::cpu::init_kernel_thread_frame(
                frame,
                stack_top.as_u64(),
                thread_trampoline as *const () as usize,
                entry as usize,
                arg,
            );
        }

        let tid = self.next_tid;
        self.next_tid += 1;

        let mut thread = None;
        for slot in THREAD_SLOTS.iter() {
            if let Some(created) = slot.try_init(Thread {
                runq_link: LinkedListLink::new(),
                _id: tid,
                state: if flags.contains(ThreadFlags::IDLE) {
                    ThreadState::Idle
                } else {
                    ThreadState::Ready
                },
                class,
                flags,
                priority,
                base_priority: priority,
                user_priority: priority,
                nice: 0,
                cpu: self.cpu.id,
                rqindex: priority,
                slice: 0,
                ftick: self.global_ticks,
                ltick: self.global_ticks,
                rltick: self.global_ticks,
                slptime: 0,
                runtime: 0,
                ticks: 0,
                _stack_base: stack_base,
                stack_top,
                frame,
            }) {
                thread = Some(created);
                break;
            }
        }

        thread.expect("sched: out of thread slots")
    }

    fn on_tick(&mut self, global: u64) {
        self.global_ticks = global;

        let current = unsafe { self.cpu.current.as_mut() };
        let Some(current) = current else {
            return;
        };

        self.cpu.oldswitchcnt = self.cpu.switchcnt;
        self.cpu.switchcnt = self.cpu.load as u16;

        if self.cpu.ts_off == self.cpu.ts_deq_off {
            self.cpu.ts_ticks = self.cpu.ts_ticks.wrapping_add(1);
            let advance = 2u16 - (self.cpu.ts_ticks / 4) as u16;
            self.cpu.ts_off = ((self.cpu.ts_off as u16 + advance) % PRI_BATCH_RANGE as u16) as u8;
            self.cpu.ts_ticks %= 4;
            self.cpu.advance_ts_deq_off(false);
        }

        current.rltick = global;
        self.pctcpu_update(current, true);

        if current.class == ThreadClass::Timeshare {
            current.runtime = current.runtime.saturating_add(self.tickincr);
            self.interact_update(current);
            self.priority_update(current);
        }

        if current.flags.contains(ThreadFlags::IDLE) {
            if self.cpu.runq.has_runnable() {
                self.cpu.need_resched = true;
            }
            self.cpu.refresh_lowpri();
            return;
        }

        current.slice = current.slice.saturating_add(1);
        if current.slice
            >= self
                .cpu
                .slice_for(current, self.sched_slice, self.sched_slice_min)
        {
            current.slice = 0;
            current.flags.insert(ThreadFlags::SLICEEND);
            self.cpu.need_resched = true;
        }

        if let Some(best) = self.cpu.choose() {
            if should_preempt(best.priority, current.priority) {
                self.cpu.need_resched = true;
            }
        }

        self.cpu.refresh_lowpri();
    }

    fn trap_return(&mut self, frame: &mut TrapFrame) -> *mut TrapFrame {
        let current = unsafe { self.cpu.current.as_mut() };
        let Some(current) = current else {
            return frame;
        };

        current.frame = frame;

        let must_switch = self.cpu.need_resched
            || (current.flags.contains(ThreadFlags::IDLE) && self.cpu.runq.has_runnable());
        if !must_switch {
            self.cpu.refresh_lowpri();
            return frame;
        }

        self.cpu.need_resched = false;

        if !current.flags.contains(ThreadFlags::IDLE) {
            let flags = if current.flags.contains(ThreadFlags::SLICEEND) {
                current.flags.remove(ThreadFlags::SLICEEND);
                SRQ_PREEMPTED
            } else {
                0
            };
            self.cpu.requeue_running(current, flags);
        }

        let next_ptr = self.cpu.choose_or_idle();
        let next = unsafe { &mut *next_ptr };
        if !next.flags.contains(ThreadFlags::IDLE) {
            self.cpu.dequeue(unsafe { &*next_ptr });
        }

        next.state = ThreadState::Running;
        next.cpu = self.cpu.id;
        next.rltick = self.global_ticks;
        self.cpu.current = next;
        self.cpu.refresh_lowpri();
        self.activate_current(next);

        next.frame
    }

    fn start(&mut self) -> ! {
        let next_ptr = self.cpu.choose_or_idle();
        let next = unsafe { &mut *next_ptr };
        if !next.flags.contains(ThreadFlags::IDLE) {
            self.cpu.dequeue(unsafe { &*next_ptr });
        }

        next.state = ThreadState::Running;
        self.cpu.current = next;
        self.cpu.refresh_lowpri();
        self.activate_current(next);

        unsafe { crate::arch::cpu::start_first_thread(next.frame) }
    }

    fn activate_current(&self, thread: &Thread) {
        let cpu = arch::thiscpu();
        cpu.kernel_stack = thread.stack_top.as_u64();
        cpu.user_stack = 0;
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

    fn pctcpu_update(&self, thread: &mut Thread, run: bool) {
        let t = self.global_ticks;
        let t_max = self.realstathz as u64 * SCHED_TICK_SECS;
        let t_tgt = (((t_max << SCHED_TICK_SHIFT) * SCHED_CPU_DECAY_NUMER) / SCHED_CPU_DECAY_DENOM)
            >> SCHED_TICK_SHIFT;
        let lu_span = t.saturating_sub(thread.ltick);

        if lu_span >= t_tgt {
            thread.ticks = if run {
                (t_tgt << SCHED_TICK_SHIFT) as u32
            } else {
                0
            };
            thread.ftick = t.saturating_sub(t_tgt);
            thread.ltick = t;
            return;
        }

        if t.saturating_sub(thread.ftick) >= t_max {
            let len = tick_length(thread).max(1);
            thread.ticks = ((thread.ticks as u64 / len) * t_tgt.saturating_sub(lu_span))
                .min(u32::MAX as u64) as u32;
            thread.ftick = t.saturating_sub(t_tgt);
        }

        if run {
            thread.ticks = thread
                .ticks
                .saturating_add((lu_span << SCHED_TICK_SHIFT).min(u32::MAX as u64) as u32);
        }
        thread.ltick = t;
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

fn allocate_stack() -> (VirtAddr, VirtAddr) {
    let mut base = VirtAddr::zero();

    for page_idx in 0..KSTACK_PAGES {
        let page = mem::phys::alloc_zeroed_page().expect("sched: out of physical memory for stack");
        let virt = mem::phys_to_virt(page.paddr());
        if page_idx == 0 {
            base = virt;
        } else {
            let expected = base
                .checked_add(page_idx as u64 * PAGE_SIZE)
                .expect("sched: stack address overflow");
            assert_eq!(
                virt, expected,
                "sched: expected contiguous stack pages during bootstrap"
            );
        }
    }

    let top = base
        .checked_add(KSTACK_PAGES as u64 * PAGE_SIZE)
        .expect("sched: stack top overflow");
    (base, top)
}

extern "C" fn thread_trampoline(entry: usize, arg: usize) -> ! {
    let entry: ThreadEntry = unsafe { core::mem::transmute(entry) };
    entry(arg)
}

extern "C" fn idle_thread(_: usize) -> ! {
    loop {
        arch::wfi();
    }
}

extern "C" fn init_thread(_: usize) -> ! {
    info!("sched: init thread online");
    loop {
        core::hint::spin_loop();
    }
}

pub fn init() {
    let mut guard = SCHEDULER.lock();
    if guard.is_some() {
        return;
    }

    let mut scheduler = Scheduler::new();
    scheduler.bootstrap();

    info!(
        "sched: ULE bootstrap ready (slice={} ticks, min_slice={} ticks)",
        scheduler.sched_slice, scheduler.sched_slice_min
    );
    *guard = Some(scheduler);
}

pub fn start() -> ! {
    let mut guard = SCHEDULER.lock();
    guard.as_mut().expect("sched: init before start").start()
}

pub fn stat_tick(global: u64, _percpu: u64) {
    let mut guard = SCHEDULER.lock();
    if let Some(scheduler) = guard.as_mut() {
        scheduler.on_tick(global);
    }
}

pub fn trap_return(frame: &mut TrapFrame) -> *mut TrapFrame {
    let mut guard = SCHEDULER.lock();
    match guard.as_mut() {
        Some(scheduler) => scheduler.trap_return(frame),
        None => frame,
    }
}
