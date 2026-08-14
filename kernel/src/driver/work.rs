//! Deferred work and timers.
//!
//! Interrupt handlers must stay short, and bus transactions frequently need to
//! sleep, so drivers need somewhere to run work that cannot happen in interrupt
//! context. A work queue is a kernel thread with a bounded pending list; a
//! timer is a callback scheduled against the monotonic clock.
//!
//! Both are attributed to the module that created them and are torn down when
//! that module unloads, so a driver cannot leave a thread running over code
//! that has been removed.

use alloc::{
    boxed::Box,
    collections::VecDeque,
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
use core::time::Duration;

use crate::sys::{
    clock,
    event::Event,
    sched,
    sync::{Mutex, Once},
};

use super::{
    core::module::Module,
    error::{Error, Result},
    obj::{ObjHeader, ObjKind, framework_object},
};

/// Largest number of items one queue may hold.
pub const MAX_PENDING: usize = 4096;

/// Callback executed by a work queue or timer.
pub type WorkFn = unsafe extern "C" fn(context: *mut c_void, argument: u64);

struct Item {
    callback: WorkFn,
    context: *mut c_void,
    argument: u64,
}

// SAFETY: the callback and context belong to the queue's owning module, which
// cannot unload until the queue is drained and destroyed.
unsafe impl Send for Item {}

struct QueueState {
    pending: VecDeque<Item>,
    running: bool,
}

/// A thread-backed queue of deferred callbacks.
#[repr(C)]
pub struct WorkQueue {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    state: Mutex<QueueState>,
    wake: Event,
    idle: Event,
    stopping: AtomicBool,
    finished: Event,
    processed: AtomicU64,
}

framework_object!(WorkQueue, WorkQueue);

impl WorkQueue {
    /// Returns the queue name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the number of callbacks this queue has run.
    pub fn processed(&self) -> u64 {
        self.processed.load(Ordering::Relaxed)
    }
}

/// A callback scheduled against the monotonic clock.
#[repr(C)]
pub struct Timer {
    header: ObjHeader,
    owner: Option<Arc<Module>>,
    callback: WorkFn,
    context: *mut c_void,
    argument: u64,
    period_ns: AtomicU64,
    deadline_ns: AtomicU64,
    armed: AtomicBool,
    stopping: AtomicBool,
    wake: Event,
    finished: Event,
}

framework_object!(Timer, Timer);

// SAFETY: the callback and context belong to the timer's owning module, which
// cannot unload until the timer is cancelled and its thread has exited.
unsafe impl Send for Timer {}
// SAFETY: arming state is held in atomics and the callback runs on a single
// dedicated thread.
unsafe impl Sync for Timer {}

struct Registry {
    queues: Mutex<Vec<Arc<WorkQueue>>>,
    timers: Mutex<Vec<Arc<Timer>>>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        queues: Mutex::new(Vec::new()),
        timers: Mutex::new(Vec::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// Creates a work queue served by its own kernel thread.
pub fn create_queue(owner: Option<&Arc<Module>>, name: &str) -> Result<Arc<WorkQueue>> {
    if name.is_empty() || name.len() > 64 {
        return Err(Error::InvalidArgument);
    }
    let registry = registry()?;
    let queue = Arc::new(WorkQueue {
        header: ObjHeader::new(ObjKind::WorkQueue),
        name: String::from(name).into_boxed_str(),
        owner: owner.cloned(),
        state: Mutex::new(QueueState {
            pending: VecDeque::new(),
            running: false,
        }),
        wake: Event::new(),
        idle: Event::new(),
        stopping: AtomicBool::new(false),
        finished: Event::new(),
        processed: AtomicU64::new(0),
    });
    registry.queues.lock().push(queue.clone());

    let worker = queue.clone();
    sched::run(move || {
        loop {
            worker.wake.wait();
            worker.wake.reset();
            loop {
                let item = {
                    let mut state = worker.state.lock();
                    let item = state.pending.pop_front();
                    state.running = item.is_some();
                    item
                };
                let Some(item) = item else {
                    break;
                };
                // SAFETY: the queue holds the callback alive, and its owning
                // module cannot unload until the queue is destroyed.
                unsafe { (item.callback)(item.context, item.argument) };
                worker.processed.fetch_add(1, Ordering::Relaxed);
            }
            {
                let mut state = worker.state.lock();
                state.running = false;
                if state.pending.is_empty() {
                    worker.idle.signal();
                }
            }
            if worker.stopping.load(Ordering::Acquire) {
                break;
            }
        }
        worker.finished.signal();
    });
    Ok(queue)
}

/// Appends a callback to a work queue.
///
/// # Safety
///
/// `callback` must follow the work ABI and stay executable until the queue is
/// destroyed.
pub unsafe fn queue_work(
    queue: &Arc<WorkQueue>,
    callback: WorkFn,
    context: *mut c_void,
    argument: u64,
) -> Result<()> {
    if queue.stopping.load(Ordering::Acquire) {
        return Err(Error::NoDevice);
    }
    {
        let mut state = queue.state.lock();
        if state.pending.len() >= MAX_PENDING {
            return Err(Error::NoSpace);
        }
        state.pending.push_back(Item {
            callback,
            context,
            argument,
        });
    }
    queue.idle.reset();
    queue.wake.signal();
    Ok(())
}

/// Waits until a queue has no pending or running callbacks.
pub fn flush_queue(queue: &Arc<WorkQueue>) {
    loop {
        {
            let state = queue.state.lock();
            if state.pending.is_empty() && !state.running {
                return;
            }
        }
        queue.wake.signal();
        queue.idle.wait();
    }
}

/// Destroys a work queue after its worker thread exits.
pub fn destroy_queue(queue: &Arc<WorkQueue>) -> Result<()> {
    if queue.stopping.swap(true, Ordering::AcqRel) {
        return Ok(());
    }
    flush_queue(queue);
    queue.wake.signal();
    queue.finished.wait();
    if let Ok(registry) = registry() {
        registry
            .queues
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, queue));
    }
    queue.header.poison();
    Ok(())
}

/// Creates a timer bound to its own kernel thread.
///
/// # Safety
///
/// `callback` must follow the work ABI and stay executable until the timer is
/// destroyed.
pub unsafe fn create_timer(
    owner: Option<&Arc<Module>>,
    callback: WorkFn,
    context: *mut c_void,
    argument: u64,
) -> Result<Arc<Timer>> {
    let registry = registry()?;
    let timer = Arc::new(Timer {
        header: ObjHeader::new(ObjKind::Timer),
        owner: owner.cloned(),
        callback,
        context,
        argument,
        period_ns: AtomicU64::new(0),
        deadline_ns: AtomicU64::new(0),
        armed: AtomicBool::new(false),
        stopping: AtomicBool::new(false),
        wake: Event::new(),
        finished: Event::new(),
    });
    registry.timers.lock().push(timer.clone());

    let worker = timer.clone();
    sched::run(move || {
        loop {
            if worker.stopping.load(Ordering::Acquire) {
                break;
            }
            if !worker.armed.load(Ordering::Acquire) {
                worker.wake.wait();
                worker.wake.reset();
                continue;
            }
            let now = clock::monotonic_ns();
            let deadline = worker.deadline_ns.load(Ordering::Acquire);
            if now < deadline {
                // Wait on the wake event rather than sleeping outright, so
                // re-arming or cancelling a timer takes effect immediately
                // instead of after the pending deadline.
                clock::wait_timeout(&worker.wake, Duration::from_nanos(deadline - now));
                worker.wake.reset();
                continue;
            }
            if !worker.armed.swap(false, Ordering::AcqRel) {
                continue;
            }
            // SAFETY: the timer holds the callback alive, and its owning module
            // cannot unload until the timer is destroyed.
            unsafe { (worker.callback)(worker.context, worker.argument) };

            let period = worker.period_ns.load(Ordering::Acquire);
            if period != 0 && !worker.stopping.load(Ordering::Acquire) {
                worker
                    .deadline_ns
                    .store(clock::monotonic_ns() + period, Ordering::Release);
                worker.armed.store(true, Ordering::Release);
            }
        }
        worker.finished.signal();
    });
    Ok(timer)
}

/// Arms a timer to fire after `delay_ns`, repeating every `period_ns` when that
/// is non-zero.
pub fn arm_timer(timer: &Arc<Timer>, delay_ns: u64, period_ns: u64) -> Result<()> {
    if timer.stopping.load(Ordering::Acquire) {
        return Err(Error::NoDevice);
    }
    timer.period_ns.store(period_ns, Ordering::Release);
    timer
        .deadline_ns
        .store(clock::monotonic_ns() + delay_ns, Ordering::Release);
    timer.armed.store(true, Ordering::Release);
    timer.wake.signal();
    Ok(())
}

/// Disarms a timer without destroying it.
pub fn cancel_timer(timer: &Arc<Timer>) {
    timer.period_ns.store(0, Ordering::Release);
    timer.armed.store(false, Ordering::Release);
    timer.wake.signal();
}

/// Destroys a timer after its thread exits.
pub fn destroy_timer(timer: &Arc<Timer>) -> Result<()> {
    if timer.stopping.swap(true, Ordering::AcqRel) {
        return Ok(());
    }
    timer.armed.store(false, Ordering::Release);
    timer.period_ns.store(0, Ordering::Release);
    timer.wake.signal();
    timer.finished.wait();
    if let Ok(registry) = registry() {
        registry
            .timers
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, timer));
    }
    timer.header.poison();
    Ok(())
}

pub(crate) fn remove_module_workers(module: &Arc<Module>) {
    let Ok(registry) = registry() else {
        return;
    };
    let timers: Vec<Arc<Timer>> = registry
        .timers
        .lock()
        .iter()
        .filter(|timer| {
            timer
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for timer in timers {
        let _ = destroy_timer(&timer);
    }

    let queues: Vec<Arc<WorkQueue>> = registry
        .queues
        .lock()
        .iter()
        .filter(|queue| {
            queue
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for queue in queues {
        let _ = destroy_queue(&queue);
    }
}

/// Snapshot of one work queue.
#[derive(Clone, Debug)]
pub struct QueueInfo {
    /// Queue name.
    pub name: Box<str>,
    /// Number of callbacks run so far.
    pub processed: u64,
    /// Number of callbacks still pending.
    pub pending: usize,
}

/// Returns a snapshot of every work queue.
pub fn list_queues() -> Result<Vec<QueueInfo>> {
    Ok(registry()?
        .queues
        .lock()
        .iter()
        .map(|queue| QueueInfo {
            name: queue.name.to_string().into_boxed_str(),
            processed: queue.processed(),
            pending: queue.state.lock().pending.len(),
        })
        .collect())
}
