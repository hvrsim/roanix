//!
//! # Threads
//!
//! Kernel thread storage and bootstrap helpers used by the scheduler.
//!

use core::{
    array,
    cell::UnsafeCell,
    mem::{align_of, size_of, MaybeUninit},
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};

use bitflags::bitflags;
use intrusive_collections::{intrusive_adapter, LinkedListLink};
use spin::Lazy;

use crate::{
    arch,
    mem::{self, VirtAddr, PAGE_SIZE},
};

type TrapFrame = crate::arch::cpu::TrapFrame;

const MAX_THREADS: usize = 32;
const KSTACK_PAGES: usize = 4;
const TASK_INLINE_WORDS: usize = 8;

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
    Idle,
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
    pub(crate) _stack_base: VirtAddr,
    pub(crate) stack_top: VirtAddr,
    pub(crate) frame: *mut TrapFrame,
    task: ThreadTask,
}

unsafe impl Send for Thread {}
unsafe impl Sync for Thread {}

intrusive_adapter!(pub(crate) ThreadAdapter = &'static Thread: Thread { runq_link: LinkedListLink });

static THREAD_SLOTS: Lazy<[ThreadSlot; MAX_THREADS]> =
    Lazy::new(|| array::from_fn(|_| ThreadSlot::new()));

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

#[derive(Copy, Clone)]
struct TaskVTable {
    invoke: unsafe fn(*mut u8),
}

struct ThreadTask {
    storage: [usize; TASK_INLINE_WORDS],
    vtable: Option<TaskVTable>,
}

impl ThreadTask {
    const fn empty() -> Self {
        Self {
            storage: [0; TASK_INLINE_WORDS],
            vtable: None,
        }
    }

    fn prepare<F, R>(&mut self, task: F)
    where
        F: FnOnce() -> R + Send + 'static,
    {
        assert!(
            size_of::<F>() <= size_of::<[usize; TASK_INLINE_WORDS]>(),
            "thread task too large for inline storage"
        );
        assert!(
            align_of::<F>() <= align_of::<[usize; TASK_INLINE_WORDS]>(),
            "thread task alignment exceeds inline storage"
        );

        unsafe {
            ptr::write(self.storage.as_mut_ptr().cast::<F>(), task);
        }

        self.vtable = Some(TaskVTable {
            invoke: invoke_task::<F, R>,
        });
    }

    unsafe fn run(&mut self) -> ! {
        let vtable = self.vtable.take().expect("thread task missing");
        (vtable.invoke)(self.storage.as_mut_ptr().cast::<u8>());
        panic!("kernel thread returned");
    }
}

unsafe fn invoke_task<F, R>(raw: *mut u8)
where
    F: FnOnce() -> R + Send + 'static,
{
    let task = raw.cast::<F>().read();
    let _ = task();
}

extern "C" fn thread_entry(thread_ptr: usize) -> ! {
    let thread = unsafe { &mut *(thread_ptr as *mut Thread) };
    unsafe { thread.task.run() }
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
    let (stack_base, stack_top) = allocate_stack();
    let frame_addr = stack_top.as_u64() - size_of::<TrapFrame>() as u64;
    let frame = frame_addr as *mut TrapFrame;

    let mut thread_task = ThreadTask::empty();
    thread_task.prepare(task);

    let slot = THREAD_SLOTS
        .iter()
        .find(|slot| !slot.used.load(Ordering::Acquire))
        .expect("sched: out of thread slots");
    let thread = slot
        .try_init(Thread {
            runq_link: LinkedListLink::new(),
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
            _stack_base: stack_base,
            stack_top,
            frame,
            task: thread_task,
        })
        .expect("sched: failed to claim thread slot");
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

pub(crate) extern "C" fn idle_task() -> ! {
    loop {
        arch::wfi();
    }
}
