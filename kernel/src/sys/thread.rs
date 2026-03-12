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
    ptr,
    sync::atomic::{AtomicU8, Ordering},
};

use crate::{
    arch,
    mem::{VirtAddr, PAGE_SIZE},
};
use bitflags::bitflags;
use intrusive_collections::{intrusive_adapter, LinkedListLink};

type TrapFrame = crate::arch::cpu::TrapFrame;

const KSTACK_PAGES: usize = 16;
const KSTACK_SIZE: usize = (KSTACK_PAGES as usize) * (PAGE_SIZE as usize);

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum ThreadClass {
    Ithread,
    Timeshare,
    Idle,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum ThreadState {
    Ready,
    Running,
    Blocked,
    Idle,
    Exited,
}

bitflags! {
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    pub(crate) struct ThreadFlags: u32 {
        const IDLE = 1 << 0;
        const NOLOAD = 1 << 1;
        const SLICEEND = 1 << 2;
    }
}

pub(crate) struct Thread {
    pub(crate) runq_link: LinkedListLink,
    pub(crate) wait_next: *mut Thread,
    pub(crate) id: usize,
    pub(crate) state: ThreadState,
    pub(crate) class: ThreadClass,
    pub(crate) flags: ThreadFlags,
    pub(crate) priority: u8,
    pub(crate) base_priority: u8,
    pub(crate) user_priority: u8,
    pub(crate) nice: i8,
    pub(crate) cpu: usize,
    pub(crate) rqindex: u8,
    pub(crate) slice: u32,
    pub(crate) ftick: u64,
    pub(crate) ltick: u64,
    pub(crate) rltick: u64,
    pub(crate) slptime: u32,
    pub(crate) runtime: u32,
    pub(crate) ticks: u32,
    _stack: Box<KernelStack>,
    pub(crate) stack_top: VirtAddr,
    pub(crate) frame: *mut TrapFrame,
    park_state: AtomicU8,
    task: Box<dyn KernelTask>,
}

unsafe impl Send for Thread {}
unsafe impl Sync for Thread {}

intrusive_adapter!(pub(crate) ThreadAdapter = &'static Thread: Thread { runq_link: LinkedListLink });

#[repr(align(16))]
struct KernelStack([u8; KSTACK_SIZE]);

const PARK_STATE_IDLE: u8 = 0;
const PARK_STATE_WAITING: u8 = 1;
const PARK_STATE_PARKED: u8 = 2;

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
    pub(crate) fn prepare_park(&self) {
        self.park_state.store(PARK_STATE_WAITING, Ordering::Release);
    }

    pub(crate) fn mark_parked(&self) -> bool {
        self.park_state
            .compare_exchange(
                PARK_STATE_WAITING,
                PARK_STATE_PARKED,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    pub(crate) fn wake(&self) -> bool {
        self.park_state.swap(PARK_STATE_IDLE, Ordering::AcqRel) == PARK_STATE_PARKED
    }
}

extern "C" fn thread_entry(thread_ptr: usize) -> ! {
    let thread = unsafe { &mut *(thread_ptr as *mut Thread) };
    let task = unsafe { ptr::read(&thread.task) };
    task.run()
}

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
    let stack_base = VirtAddr::from_ptr(stack.0.as_mut_ptr());
    let stack_top = stack_base
        .checked_add(KSTACK_SIZE as u64)
        .expect("sched: stack top overflow");
    let frame_addr = stack_top.as_u64() - size_of::<TrapFrame>() as u64;
    let frame = frame_addr as *mut TrapFrame;
    let thread = Box::leak(Box::new(Thread {
        runq_link: LinkedListLink::new(),
        wait_next: ptr::null_mut(),
        id,
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
        task: Box::new(task),
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

pub(crate) extern "C" fn idle_task() -> ! {
    loop {
        arch::wfi();
    }
}
