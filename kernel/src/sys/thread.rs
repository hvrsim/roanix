//!
//! # Threads
//!
//! Kernel thread storage and bootstrap helpers used by the scheduler.
//!

use alloc::{
    alloc::{alloc_zeroed, handle_alloc_error, Layout},
    boxed::Box,
};
use core::{
    mem::size_of,
    sync::atomic::{AtomicU64, AtomicU8, Ordering},
};

use crate::{
    arch,
    mem::{VirtAddr, PAGE_SIZE},
};
use bitflags::bitflags;
use intrusive_collections::{intrusive_adapter, LinkedListLink};

type TrapFrame = crate::arch::cpu::TrapFrame;

const KSTACK_PAGES: usize = 32;
const KSTACK_SIZE: usize = KSTACK_PAGES * (PAGE_SIZE as usize);
const STACK_CANARY_WORDS: usize = 8;
const STACK_CANARY: u64 = 0xC0DE_CAFE_D15C_A11A;

/// Scheduler class assigned to a kernel thread.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum ThreadClass {
    /// Interrupt handler thread with the highest scheduler priority.
    Ithread,

    /// Regular schedulable kernel work.
    Timeshare,

    /// Per-CPU idle loop.
    Idle,
}

/// High-level lifecycle state tracked by the scheduler.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum ThreadState {
    /// Runnable and queued on a CPU run queue.
    Ready,

    /// Currently executing on a CPU.
    Running,

    /// Sleeping until an external event wakes it.
    Blocked,

    /// Special state for idle threads before first activation.
    Idle,

    /// Permanently terminated and no longer schedulable.
    Exited,
}

bitflags! {
    /// Scheduler-visible flags describing thread behavior.
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    pub(crate) struct ThreadFlags: u32 {
        /// Marks a thread as the per-CPU idle task.
        const IDLE = 1 << 0;

        /// Excludes the thread from scheduler load accounting.
        const NOLOAD = 1 << 1;

        /// Remembers that the thread consumed its timeslice.
        const SLICEEND = 1 << 2;
    }
}

/// Scheduler-owned kernel thread record and execution context.
pub(crate) struct Thread {
    /// Intrusive link used by per-priority run queues.
    pub(crate) runq_link: LinkedListLink,

    /// Intrusive link used by the scheduler reaper queue.
    pub(crate) reap_link: LinkedListLink,

    /// Monotonic thread identifier assigned by the scheduler.
    pub(crate) id: usize,

    /// Current lifecycle state.
    pub(crate) state: ThreadState,

    /// Scheduler class that determines queueing behavior.
    pub(crate) class: ThreadClass,

    /// Miscellaneous scheduler flags.
    pub(crate) flags: ThreadFlags,

    /// Active priority used for run-queue placement.
    pub(crate) priority: u8,

    /// Baseline priority before temporary adjustments.
    pub(crate) base_priority: u8,

    /// User-visible priority computed from interactivity and nice value.
    pub(crate) user_priority: u8,

    /// Nice value in the traditional `[-20, 20]` range.
    pub(crate) nice: i8,

    /// CPU that currently owns the thread.
    pub(crate) cpu: usize,

    /// Run-queue bucket index currently holding the thread.
    pub(crate) rqindex: u8,

    /// Scheduler tick count consumed in the current slice.
    pub(crate) slice: u32,

    /// First tick in the current CPU-usage decay window.
    pub(crate) ftick: u64,

    /// Last tick when CPU usage accounting was refreshed.
    pub(crate) ltick: u64,

    /// Last tick when the thread actually ran.
    pub(crate) rltick: u64,

    /// Accumulated sleep time used by interactivity heuristics.
    pub(crate) slptime: u32,

    /// Accumulated runtime used by interactivity heuristics.
    pub(crate) runtime: u32,

    /// Fixed-point CPU usage estimate.
    pub(crate) ticks: u32,

    /// Backing allocation for the kernel stack.
    _stack: Box<KernelStack>,

    /// Initial top-of-stack address used for context setup.
    pub(crate) stack_top: VirtAddr,

    /// Saved trap frame restored when the thread is resumed.
    pub(crate) frame: *mut TrapFrame,

    /// Parking handshake shared between the scheduler and wait sites.
    park_state: AtomicU8,

    /// Monotonic sequence that tags the current blocking attempt.
    park_seq: AtomicU64,

    /// Heap-allocated entry closure that runs when the thread starts.
    task: Option<Box<dyn KernelTask>>,
}

// SAFETY: `Thread` instances are scheduler-owned, live for the lifetime of the
// kernel, and are only mutated under scheduler-defined synchronization.
unsafe impl Send for Thread {}
// SAFETY: shared access is coordinated by the scheduler and low-level locks.
unsafe impl Sync for Thread {}

// Intrusive list adapter used by scheduler run queues.
intrusive_adapter!(pub(crate) ThreadAdapter = &'static Thread: Thread { runq_link: LinkedListLink });
intrusive_adapter!(pub(crate) ExitedThreadAdapter = &'static Thread: Thread { reap_link: LinkedListLink });

#[repr(align(16))]
struct KernelStack([u8; KSTACK_SIZE]);

const PARK_STATE_IDLE: u8 = 0;
const PARK_STATE_WAITING: u8 = 1;
const PARK_STATE_PARKED: u8 = 2;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum WakeResult {
    Stale,
    Pending,
    Parked,
}

trait KernelTask: Send {
    fn run(self: Box<Self>) -> !;
}

impl<F, R> KernelTask for F
where
    F: FnOnce() -> R + Send + 'static,
{
    fn run(self: Box<Self>) -> ! {
        let _ = (*self)();
        crate::sys::sched::exit_current()
    }
}

impl Thread {
    /// Returns whether this thread is currently linked into a run queue.
    pub(crate) fn is_runq_linked(&self) -> bool {
        self.runq_link.is_linked()
    }

    /// Returns whether this thread is the per-CPU idle task.
    pub(crate) fn is_idle(&self) -> bool {
        self.flags.contains(ThreadFlags::IDLE)
    }

    /// Returns whether this thread contributes to scheduler load.
    pub(crate) fn counts_towards_load(&self) -> bool {
        !self.flags.contains(ThreadFlags::NOLOAD)
    }

    /// Marks the thread runnable on run-queue bucket `rqindex`.
    pub(crate) fn mark_ready(&mut self, rqindex: u8) {
        self.state = ThreadState::Ready;
        self.rqindex = rqindex;
    }

    /// Marks the thread blocked until a wake event arrives.
    pub(crate) fn mark_blocked(&mut self) {
        self.state = ThreadState::Blocked;
    }

    /// Marks the thread as permanently exited.
    pub(crate) fn mark_exited(&mut self) {
        self.state = ThreadState::Exited;
    }

    /// Marks the thread as running on `cpu_id` at `global_ticks`.
    pub(crate) fn mark_running(&mut self, cpu_id: usize, global_ticks: u64) {
        self.state = ThreadState::Running;
        self.cpu = cpu_id;
        self.rltick = global_ticks;
    }

    /// Clears and returns the deferred slice-end flag.
    pub(crate) fn take_slice_end(&mut self) -> bool {
        let exhausted = self.flags.contains(ThreadFlags::SLICEEND);
        if exhausted {
            self.flags.remove(ThreadFlags::SLICEEND);
        }
        exhausted
    }

    /// Publishes an intent to sleep before yielding to the scheduler.
    pub(crate) fn prepare_park(&self) -> u64 {
        let seq = self
            .park_seq
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_add(1);
        self.park_state.store(PARK_STATE_WAITING, Ordering::Release);
        seq
    }

    /// Finalizes the park handshake if no wakeup raced with parking.
    pub(crate) fn mark_parked(&self, seq: u64) -> bool {
        if self.park_seq.load(Ordering::Acquire) != seq {
            return false;
        }

        self.park_state
            .compare_exchange(
                PARK_STATE_WAITING,
                PARK_STATE_PARKED,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    /// Wakes a parked thread and reports whether it must be requeued.
    pub(crate) fn wake(&self, seq: u64) -> WakeResult {
        loop {
            let state = self.park_state.load(Ordering::Acquire);
            match state {
                PARK_STATE_WAITING => {
                    if self.park_seq.load(Ordering::Acquire) != seq {
                        return WakeResult::Stale;
                    }
                    if self
                        .park_state
                        .compare_exchange(
                            PARK_STATE_WAITING,
                            PARK_STATE_IDLE,
                            Ordering::AcqRel,
                            Ordering::Acquire,
                        )
                        .is_ok()
                    {
                        return WakeResult::Pending;
                    }
                }
                PARK_STATE_PARKED => {
                    if self.park_seq.load(Ordering::Acquire) != seq {
                        return WakeResult::Stale;
                    }
                    if self
                        .park_state
                        .compare_exchange(
                            PARK_STATE_PARKED,
                            PARK_STATE_IDLE,
                            Ordering::AcqRel,
                            Ordering::Acquire,
                        )
                        .is_ok()
                    {
                        return WakeResult::Parked;
                    }
                }
                _ => return WakeResult::Stale,
            }
        }
    }

    /// Returns whether the saved trap frame pointer still resides in this
    /// thread's kernel stack allocation.
    pub(crate) fn has_valid_frame_ptr(&self) -> bool {
        let frame = self.frame as usize as u64;
        let stack_top = self.stack_top.as_u64();
        let stack_base = stack_top.saturating_sub(KSTACK_SIZE as u64);

        frame >= stack_base && frame + size_of::<TrapFrame>() as u64 <= stack_top
    }

    /// Returns whether the low-end stack canary is still intact.
    pub(crate) fn has_valid_stack_canary(&self) -> bool {
        self._stack
            .0
            .chunks_exact(size_of::<u64>())
            .take(STACK_CANARY_WORDS)
            .all(|bytes| u64::from_ne_bytes(bytes.try_into().unwrap()) == STACK_CANARY)
    }
}

extern "C" fn thread_entry(thread_ptr: usize) -> ! {
    let thread = unsafe { &mut *(thread_ptr as *mut Thread) };
    let task = thread
        .task
        .take()
        .expect("thread: missing task in thread entry");
    task.run()
}

/// Allocates a kernel thread, stack, and initial trap frame.
pub(crate) fn allocate_thread<F, R>(
    id: usize,
    cpu: usize,
    global_ticks: u64,
    class: ThreadClass,
    priority: u8,
    flags: ThreadFlags,
    task: F,
) -> &'static mut Thread
where
    F: FnOnce() -> R + Send + 'static,
{
    let layout = Layout::new::<KernelStack>();
    let stack_ptr = unsafe { alloc_zeroed(layout) } as *mut KernelStack;
    if stack_ptr.is_null() {
        handle_alloc_error(layout);
    }

    let mut stack = unsafe { Box::from_raw(stack_ptr) };
    for chunk in stack
        .0
        .chunks_exact_mut(size_of::<u64>())
        .take(STACK_CANARY_WORDS)
    {
        chunk.copy_from_slice(&STACK_CANARY.to_ne_bytes());
    }
    let stack_base = VirtAddr::from_ptr(stack.0.as_mut_ptr());
    let stack_top = stack_base
        .checked_add(KSTACK_SIZE as u64)
        .expect("sched: stack top overflow");
    let frame_addr = stack_top.as_u64() - size_of::<TrapFrame>() as u64;
    let frame = frame_addr as *mut TrapFrame;
    let state = if flags.contains(ThreadFlags::IDLE) {
        ThreadState::Idle
    } else {
        ThreadState::Ready
    };
    let thread = Box::leak(Box::new(Thread {
        runq_link: LinkedListLink::new(),
        reap_link: LinkedListLink::new(),
        id,
        state,
        class,
        flags,
        priority,
        base_priority: priority,
        user_priority: priority,
        nice: 0,
        cpu,
        rqindex: priority,
        slice: 0,
        ftick: global_ticks,
        ltick: global_ticks,
        rltick: global_ticks,
        slptime: 0,
        runtime: 0,
        ticks: 0,
        _stack: stack,
        stack_top,
        frame,
        park_state: AtomicU8::new(PARK_STATE_IDLE),
        park_seq: AtomicU64::new(0),
        task: Some(Box::new(task)),
    }));
    unsafe {
        crate::arch::cpu::init_kernel_thread_frame(
            frame,
            stack_top.as_u64(),
            thread_entry as *const () as usize,
            thread as *mut Thread as usize,
            0,
        );
    }
    thread
}

/// Reclaims a thread allocation after it has permanently exited.
///
/// # Safety
///
/// `thread` must point to a thread that is no longer runnable or executing on
/// any CPU.
pub(crate) unsafe fn free_thread(thread: *mut Thread) {
    drop(Box::from_raw(thread));
}

/// Default idle loop that halts until the next interrupt arrives.
pub(crate) extern "C" fn idle_task() -> ! {
    loop {
        arch::wfi();
    }
}
