//! Serial ownership of mutable state, implemented by ordinary thread service.
//!
//! Callbacks run without holding the submission mutex and may sleep. Waiting
//! for later work on the same executor is forbidden. Cross-executor synchronous
//! dependency cycles remain the caller's responsibility.

use alloc::{boxed::Box, collections::VecDeque, sync::Arc};

use super::{context, event::Event, service::Service, sync::Mutex};

mod pool;
pub use pool::{PooledExecutor, WorkerPool};

type Operation<T> = Box<dyn FnOnce(&mut T) + Send>;

struct Pending<T> {
    operations: VecDeque<Operation<T>>,
    closed: bool,
}

/// Submission failure. A rejected callback is dropped in the submitting thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubmitError {
    /// Shutdown has closed submission.
    Closed,
    /// The bounded pending queue is full.
    Full,
}

/// One worker exclusively owns `T`; callers submit operations on that state.
/// Call `shutdown` to drain work and destroy `T` before releasing its resources.
pub struct SerialExecutor<T: Send + 'static> {
    pending: Arc<Mutex<Pending<T>>>,
    service: Service,
    capacity: usize,
}

impl<T: Send + 'static> SerialExecutor<T> {
    /// Creates a bounded executor. Callbacks are serialized even across sleeps.
    pub fn new(mut value: T, capacity: usize) -> Self {
        context::assert_sleepable("executor: create");
        assert!(capacity > 0, "executor: zero capacity");
        let pending = Arc::new(Mutex::new(Pending {
            operations: VecDeque::<Operation<T>>::new(),
            closed: false,
        }));
        let worker = pending.clone();
        let service = Service::work(move |budget| {
            while budget.take() {
                let operation = worker.lock().operations.pop_front();
                let Some(operation) = operation else {
                    return false;
                };
                operation(&mut value);
            }
            !worker.lock().operations.is_empty()
        });
        Self {
            pending,
            service,
            capacity,
        }
    }

    /// Appends work in thread context. This may allocate; hard IRQs use Service.
    pub fn submit(
        &self,
        operation: impl FnOnce(&mut T) + Send + 'static,
    ) -> Result<(), SubmitError> {
        context::assert_sleepable("executor: submit");
        {
            let mut pending = self.pending.lock();
            if pending.closed {
                return Err(SubmitError::Closed);
            }
            if pending.operations.len() >= self.capacity {
                return Err(SubmitError::Full);
            }
            pending.operations.push_back(Box::new(operation));
        }
        // Shutdown requests a final drain, even if it wins this notification.
        self.service.request();
        Ok(())
    }

    /// Returns the number of accepted callbacks that have not started.
    pub fn pending_count(&self) -> usize {
        self.pending.lock().operations.len()
    }

    /// Waits for all operations preceding a barrier. Later submissions may run.
    /// Returns Full if a barrier cannot currently be admitted; callers may retry.
    pub fn flush(&self) -> Result<(), SubmitError> {
        context::assert_sleepable("executor: flush");
        self.service.assert_not_current();
        let done = Arc::new(Event::new());
        let signal = done.clone();
        self.submit(move |_| {
            signal.signal();
        })?;
        done.wait();
        Ok(())
    }

    /// Closes submission and waits for accepted callbacks and state destruction.
    /// Concurrent shutdown callers all wait for the same completed drain.
    pub fn shutdown(&self) {
        context::assert_sleepable("executor: shutdown");
        self.service.assert_not_current();
        self.pending.lock().closed = true;
        self.service.shutdown();
    }
}
