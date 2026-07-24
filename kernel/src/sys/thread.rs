//!
//! # Threads
//!
//! Kernel thread storage and bootstrap helpers used by the scheduler.
//!

use alloc::{
    alloc::{Layout, alloc_zeroed, handle_alloc_error},
    boxed::Box,
    sync::Arc,
};
use core::{
    mem::size_of,
    sync::atomic::{AtomicU8, AtomicU64, AtomicUsize, Ordering},
};

use crate::{
    arch,
    mem::{PAGE_SIZE, VirtAddr, VmSpace},
    proc::Process,
    sys::smp::IrqSpinLock,
};
use bitflags::bitflags;
use intrusive_collections::{LinkedListLink, intrusive_adapter};

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
#[repr(u8)]
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

        /// Prevents the thread from migrating to another CPU.
        const NO_MIGRATE = 1 << 3;
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

    /// Nominal interrupt-thread priority restored after voluntary sleep.
    pub(crate) ithread_base_priority: u8,

    /// User-visible priority computed from interactivity and nice value.
    pub(crate) user_priority: u8,

    /// Nice value in the traditional `[-20, 20]` range.
    pub(crate) nice: i8,

    /// CPU that currently owns the thread.
    pub(crate) cpu: usize,

    /// Run-queue bucket index currently holding the thread.
    pub(crate) rqindex: u8,

    /// Nanoseconds consumed in the current slice.
    pub(crate) slice_ns: u64,

    /// Start of the current CPU-usage decay window.
    pub(crate) cpu_window_start_ns: u64,

    /// Last point where CPU usage accounting was refreshed.
    pub(crate) cpu_last_update_ns: u64,

    /// Last point where running-time and slice accounting was refreshed.
    pub(crate) last_run_ns: u64,

    /// Start of the current voluntary sleep, or `0` while not blocked.
    pub(crate) sleep_start_ns: u64,

    /// Accumulated sleep time used by interactivity heuristics.
    pub(crate) slptime_ns: u64,

    /// Accumulated runtime used by interactivity heuristics.
    pub(crate) runtime_ns: u64,

    /// Fixed-point CPU usage estimate in nanoseconds.
    pub(crate) cpu_estimate: u64,

    /// Backing allocation for the kernel stack.
    _stack: Box<KernelStack>,

    /// Initial top-of-stack address used for context setup.
    pub(crate) stack_top: VirtAddr,

    /// Saved trap frame restored when the thread is resumed.
    pub(crate) frame: *mut TrapFrame,

    /// Parking handshake shared between the scheduler and wait sites.
    park_state: AtomicU8,

    /// Atomically published lifecycle state used by cross-CPU wake/wait code.
    state_atomic: AtomicU8,

    /// Dynamic scheduler pin count used to keep short critical sections local.
    migration_pins: AtomicUsize,

    /// Monotonic sequence that tags the current blocking attempt.
    park_seq: AtomicU64,

    /// Heap-allocated entry closure that runs when the thread starts.
    task: Option<Box<dyn KernelTask>>,

    /// Process state shared by user threads.
    process: Option<Arc<Process>>,

    /// User address space activated while this thread runs.
    address_space: IrqSpinLock<Option<Arc<VmSpace>>>,

    /// Architecture user thread pointer or TLS base.
    thread_pointer: AtomicU64,
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
    fn publish_state(&mut self, state: ThreadState) {
        self.state = state;
        self.state_atomic.store(state as u8, Ordering::Release);
    }

    /// Returns the last scheduler lifecycle state published for this thread.
    pub(crate) fn observed_state(&self) -> ThreadState {
        match self.state_atomic.load(Ordering::Acquire) {
            x if x == ThreadState::Ready as u8 => ThreadState::Ready,
            x if x == ThreadState::Running as u8 => ThreadState::Running,
            x if x == ThreadState::Blocked as u8 => ThreadState::Blocked,
            x if x == ThreadState::Idle as u8 => ThreadState::Idle,
            x if x == ThreadState::Exited as u8 => ThreadState::Exited,
            value => panic!("thread: invalid published state {value}"),
        }
    }

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

    /// Returns whether the thread may migrate to another CPU.
    pub(crate) fn can_migrate(&self) -> bool {
        !self.flags.contains(ThreadFlags::NO_MIGRATE)
            && self.migration_pins.load(Ordering::Acquire) == 0
    }

    /// Returns the thread's current user address space.
    pub(crate) fn address_space(&self) -> Option<Arc<VmSpace>> {
        self.address_space.lock().clone()
    }

    /// Returns the current user page-table root without cloning its owner.
    pub(crate) fn address_space_root(&self) -> Option<u64> {
        self.address_space
            .lock()
            .as_ref()
            .map(|space| space.pmap().root().as_u64())
    }

    /// Removes the user address-space owner during exited-thread reclamation.
    pub(crate) fn take_address_space(&self) -> Option<Arc<VmSpace>> {
        self.address_space.lock().take()
    }

    /// Returns the process associated with this thread.
    pub(crate) fn process(&self) -> Option<Arc<Process>> {
        self.process.clone()
    }

    /// Replaces the thread's user address space and returns the previous one.
    pub(crate) fn replace_address_space(
        &self,
        space: Option<Arc<VmSpace>>,
    ) -> Option<Arc<VmSpace>> {
        core::mem::replace(&mut *self.address_space.lock(), space)
    }

    /// Returns the userspace thread pointer restored on resume.
    pub(crate) fn thread_pointer(&self) -> u64 {
        self.thread_pointer.load(Ordering::Acquire)
    }

    /// Changes the userspace thread pointer restored on resume.
    pub(crate) fn set_thread_pointer(&self, pointer: u64) {
        self.thread_pointer.store(pointer, Ordering::Release);
    }

    /// Prevents the scheduler from migrating this thread until the matching
    /// unpin completes.
    pub(crate) fn pin_migration(&self) {
        self.migration_pins.fetch_add(1, Ordering::AcqRel);
    }

    /// Releases one dynamic migration pin.
    pub(crate) fn unpin_migration(&self) {
        let previous = self.migration_pins.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "thread: migration pin underflow");
    }

    /// Marks the thread runnable on run-queue bucket `rqindex`.
    pub(crate) fn mark_ready(&mut self, rqindex: u8) {
        self.publish_state(ThreadState::Ready);
        self.rqindex = rqindex;
    }

    /// Marks the thread blocked until a wake event arrives.
    pub(crate) fn mark_blocked(&mut self, now_ns: u64) {
        self.sleep_start_ns = now_ns;
        self.publish_state(ThreadState::Blocked);
    }

    /// Marks the thread as permanently exited.
    pub(crate) fn mark_exited(&mut self) {
        self.publish_state(ThreadState::Exited);
    }

    /// Marks the thread as running on `cpu_id` at `now_ns`.
    pub(crate) fn mark_running(&mut self, cpu_id: usize, now_ns: u64) {
        self.publish_state(ThreadState::Running);
        self.cpu = cpu_id;
        self.last_run_ns = now_ns;
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
            .as_chunks::<{ size_of::<u64>() }>()
            .0
            .iter()
            .take(STACK_CANARY_WORDS)
            .all(|bytes| u64::from_ne_bytes(*bytes) == STACK_CANARY)
    }
}

extern "C" fn thread_entry(thread_ptr: usize) -> ! {
    crate::sys::sched::publish_deferred_exit();
    // SAFETY: the scheduler passes the leaked `Thread` pointer used to create
    // this initial frame and starts it exactly once.
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
    now_ns: u64,
    class: ThreadClass,
    priority: u8,
    flags: ThreadFlags,
    task: F,
) -> &'static mut Thread
where
    F: FnOnce() -> R + Send + 'static,
{
    let thread = allocate_thread_record(
        id,
        cpu,
        now_ns,
        class,
        priority,
        flags,
        Some(Box::new(task)),
        None,
        None,
    );
    // SAFETY: `frame` points inside the exclusively owned stack allocation and
    // is properly aligned for the architecture trap frame.
    unsafe {
        crate::arch::cpu::init_kernel_thread_frame(
            thread.frame,
            thread.stack_top.as_u64(),
            thread_entry as *const () as usize,
            thread as *mut Thread as usize,
            0,
        );
    }
    thread
}

/// Allocates a user thread with a prepared initial userspace frame.
pub(crate) fn allocate_user_thread(
    id: usize,
    cpu: usize,
    now_ns: u64,
    priority: u8,
    process: Arc<Process>,
    entry: u64,
    stack: u64,
    thread_pointer: u64,
) -> &'static mut Thread {
    let address_space = process.address_space();
    let thread = allocate_thread_record(
        id,
        cpu,
        now_ns,
        ThreadClass::Timeshare,
        priority,
        ThreadFlags::empty(),
        None,
        Some(process),
        Some(address_space),
    );
    thread.set_thread_pointer(thread_pointer);
    // SAFETY: `frame` points inside the exclusively owned stack allocation and
    // is properly aligned for the architecture trap frame.
    unsafe {
        crate::arch::cpu::init_user_thread_frame(thread.frame, entry, stack);
    }
    thread
}

/// Allocates a child user thread from a copied parent syscall frame.
pub(crate) fn allocate_forked_user_thread(
    id: usize,
    cpu: usize,
    now_ns: u64,
    priority: u8,
    process: Arc<Process>,
    parent_frame: &TrapFrame,
    thread_pointer: u64,
) -> &'static mut Thread {
    let address_space = process.address_space();
    let thread = allocate_thread_record(
        id,
        cpu,
        now_ns,
        ThreadClass::Timeshare,
        priority,
        ThreadFlags::empty(),
        None,
        Some(process),
        Some(address_space),
    );
    thread.set_thread_pointer(thread_pointer);
    // SAFETY: `frame` points inside the exclusively owned stack allocation,
    // while `parent_frame` remains live for the duration of this copy.
    unsafe {
        crate::arch::cpu::init_forked_user_thread_frame(thread.frame, parent_frame);
    }
    thread
}

#[allow(clippy::too_many_arguments)]
fn allocate_thread_record(
    id: usize,
    cpu: usize,
    now_ns: u64,
    class: ThreadClass,
    priority: u8,
    flags: ThreadFlags,
    task: Option<Box<dyn KernelTask>>,
    process: Option<Arc<Process>>,
    address_space: Option<Arc<VmSpace>>,
) -> &'static mut Thread {
    let layout = Layout::new::<KernelStack>();
    // SAFETY: `layout` describes `KernelStack`; null is handled immediately.
    let stack_ptr = unsafe { alloc_zeroed(layout) } as *mut KernelStack;
    if stack_ptr.is_null() {
        handle_alloc_error(layout);
    }

    // SAFETY: `stack_ptr` is a fresh allocation with the exact `KernelStack`
    // layout and ownership transfers into this box.
    let mut stack = unsafe { Box::from_raw(stack_ptr) };
    for chunk in stack
        .0
        .as_chunks_mut::<{ size_of::<u64>() }>()
        .0
        .iter_mut()
        .take(STACK_CANARY_WORDS)
    {
        *chunk = STACK_CANARY.to_ne_bytes();
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
        ithread_base_priority: priority,
        user_priority: priority,
        nice: 0,
        cpu,
        rqindex: priority,
        slice_ns: 0,
        cpu_window_start_ns: now_ns,
        cpu_last_update_ns: now_ns,
        last_run_ns: now_ns,
        sleep_start_ns: 0,
        slptime_ns: 0,
        runtime_ns: 0,
        cpu_estimate: 0,
        _stack: stack,
        stack_top,
        frame,
        park_state: AtomicU8::new(PARK_STATE_IDLE),
        state_atomic: AtomicU8::new(state as u8),
        migration_pins: AtomicUsize::new(0),
        park_seq: AtomicU64::new(0),
        task,
        process,
        address_space: IrqSpinLock::new(address_space),
        thread_pointer: AtomicU64::new(0),
    }));
    thread
}

/// Reclaims a thread allocation after it has permanently exited.
///
/// # Safety
///
/// `thread` must point to a thread that is no longer runnable or executing on
/// any CPU.
pub(crate) unsafe fn free_thread(thread: *mut Thread) {
    // SAFETY: upheld by this function's ownership contract.
    drop(unsafe { Box::from_raw(thread) });
}

/// Default idle loop that halts until the next interrupt arrives.
pub(crate) extern "C" fn idle_task() -> ! {
    loop {
        arch::wfi();
    }
}
