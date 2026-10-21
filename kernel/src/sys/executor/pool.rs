//! Optional shared execution resources for serial state owners.
//!
//! Each executor has at most one queued or running pool job. Its state is moved
//! out while running and returned before rescheduling, without holding a lock
//! across callbacks. A blocked callback occupies one pool worker; interrupt and
//! reclaim progress must use independent execution resources.

use alloc::{boxed::Box, collections::VecDeque, sync::Arc, vec::Vec};
use core::{
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
};

use super::{Operation, SubmitError};
use crate::sys::{
    context,
    event::Event,
    sched,
    service::{Budget, Service},
    sync::Mutex,
    thread::Thread,
};

type Job = Box<dyn FnOnce() + Send>;

struct PoolState {
    jobs: VecDeque<Job>,
    executors: usize,
    closed: bool,
}

/// A fixed pool shared by independently serialized executors.
/// Shut down its executors before joining the pool. Pool callbacks must not
/// synchronously depend on queued work when all workers could be occupied.
pub struct WorkerPool {
    state: Arc<Mutex<PoolState>>,
    workers: Vec<Service>,
}

impl WorkerPool {
    /// Creates `threads` ordinary workers. Zero workers is a programming error.
    pub fn new(threads: usize) -> Arc<Self> {
        context::assert_sleepable("pool: create");
        assert!(threads > 0, "pool: zero workers");
        let state = Arc::new(Mutex::new(PoolState {
            jobs: VecDeque::<Job>::new(),
            executors: 0,
            closed: false,
        }));
        let mut workers = Vec::with_capacity(threads);
        for _ in 0..threads {
            let queue = state.clone();
            workers.push(Service::work(move |budget| {
                while budget.take() {
                    let job = queue.lock().jobs.pop_front();
                    let Some(job) = job else {
                        return false;
                    };
                    job();
                }
                !queue.lock().jobs.is_empty()
            }));
        }
        Arc::new(Self { state, workers })
    }

    /// Creates an independent state owner sharing this pool's execution threads.
    pub fn executor<T: Send + 'static>(
        self: &Arc<Self>,
        value: T,
        capacity: usize,
    ) -> Result<PooledExecutor<T>, SubmitError> {
        context::assert_sleepable("pool: executor");
        assert!(capacity > 0, "pool: zero executor capacity");
        {
            let mut state = self.state.lock();
            if state.closed {
                return Err(SubmitError::Closed);
            }
            state.executors += 1;
        }
        Ok(PooledExecutor {
            inner: Arc::new(Owner {
                pool: self.clone(),
                state: Mutex::new(Pending {
                    operations: VecDeque::new(),
                    value: Some(value),
                    closed: false,
                    scheduled: false,
                }),
                worker: AtomicPtr::new(ptr::null_mut()),
                finished: Event::new(),
                capacity,
            }),
        })
    }

    fn enqueue(&self, job: Job) {
        {
            let mut state = self.state.lock();
            assert!(!state.closed, "pool: live executor submitted after closure");
            state.jobs.push_back(job);
        }
        // Notify all workers: a chosen worker might be blocked inside a job
        // while another worker is idle. Submission is thread-only and may allocate.
        for worker in &self.workers {
            worker.request();
        }
    }

    /// Joins workers, returning false if executors still require the pool.
    /// Every successful concurrent caller waits for all worker completions.
    pub fn shutdown(&self) -> bool {
        context::assert_sleepable("pool: shutdown");
        {
            let mut state = self.state.lock();
            if state.executors != 0 {
                return false;
            }
            state.closed = true;
        }
        for worker in &self.workers {
            worker.shutdown();
        }
        true
    }
}

struct Pending<T> {
    operations: VecDeque<Operation<T>>,
    value: Option<T>,
    closed: bool,
    /// Remains set during final destruction until completion is published.
    scheduled: bool,
}

struct Owner<T: Send + 'static> {
    pool: Arc<WorkerPool>,
    state: Mutex<Pending<T>>,
    worker: AtomicPtr<Thread>,
    finished: Event,
    capacity: usize,
}

/// Serial ownership backed by a shared pool. Close it explicitly before joining
/// the pool or releasing state resources. Clones refer to the same owner.
pub struct PooledExecutor<T: Send + 'static> {
    inner: Arc<Owner<T>>,
}

impl<T: Send + 'static> Clone for PooledExecutor<T> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

impl<T: Send + 'static> Owner<T> {
    fn schedule(self: &Arc<Self>) {
        let owner = self.clone();
        self.pool.enqueue(Box::new(move || owner.run()));
    }

    fn assert_not_current(&self) {
        assert_ne!(
            self.worker.load(Ordering::Acquire),
            sched::current_thread(),
            "pool: callback cannot wait for its own executor"
        );
    }

    fn run(self: Arc<Self>) {
        self.worker
            .store(sched::current_thread(), Ordering::Release);
        let mut value = self
            .state
            .lock()
            .value
            .take()
            .expect("pool: concurrent executor jobs");
        let mut budget = Budget::new();
        while budget.take() {
            let operation = self.state.lock().operations.pop_front();
            let Some(operation) = operation else {
                break;
            };
            operation(&mut value);
        }
        let finish;
        let reschedule;
        {
            let mut state = self.state.lock();
            finish = state.closed && state.operations.is_empty();
            reschedule = !state.operations.is_empty();
            if !finish {
                state.value = Some(value);
                // Clear before publishing idle ownership or enqueuing a new
                // job, so the old worker cannot erase its successor's identity.
                self.worker.store(ptr::null_mut(), Ordering::Release);
                if !reschedule {
                    state.scheduled = false;
                }
                drop(state);
                if reschedule {
                    self.schedule();
                }
                return;
            }
        }
        // Keep scheduled set while destructors run. A concurrent shutdown must
        // join this job, not enqueue another job against an absent value.
        drop(value);
        self.worker.store(ptr::null_mut(), Ordering::Release);
        self.pool.state.lock().executors -= 1;
        self.finished.signal();
    }
}

impl<T: Send + 'static> PooledExecutor<T> {
    /// Appends bounded work in sleepable thread context; may allocate.
    pub fn submit(
        &self,
        operation: impl FnOnce(&mut T) + Send + 'static,
    ) -> Result<(), SubmitError> {
        context::assert_sleepable("pooled executor: submit");
        let schedule = {
            let mut state = self.inner.state.lock();
            if state.closed {
                return Err(SubmitError::Closed);
            }
            if state.operations.len() >= self.inner.capacity {
                return Err(SubmitError::Full);
            }
            state.operations.push_back(Box::new(operation));
            let schedule = !state.scheduled;
            state.scheduled = true;
            schedule
        };
        if schedule {
            self.inner.schedule();
        }
        Ok(())
    }

    /// Waits for a barrier after preceding work. A full queue rejects the barrier.
    pub fn flush(&self) -> Result<(), SubmitError> {
        context::assert_sleepable("pooled executor: flush");
        self.inner.assert_not_current();
        let done = Arc::new(Event::new());
        let signal = done.clone();
        self.submit(move |_| {
            signal.signal();
        })?;
        done.wait();
        Ok(())
    }

    /// Closes admission and joins accepted work and state destruction.
    pub fn shutdown(&self) {
        context::assert_sleepable("pooled executor: shutdown");
        self.inner.assert_not_current();
        let schedule = {
            let mut state = self.inner.state.lock();
            state.closed = true;
            let schedule = !state.scheduled;
            state.scheduled = true;
            schedule
        };
        if schedule {
            self.inner.schedule();
        }
        self.inner.finished.wait();
    }
}
