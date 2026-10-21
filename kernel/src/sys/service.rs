//! Preallocated, coalescing requests serviced by a dedicated schedulable thread.
//!
//! A request is a hint to inspect authoritative device state, not a completion
//! count. Requests made during a callback cause another pass. Closing admission
//! and requesting the final drain happen under the same IRQ-safe lock.

use alloc::sync::Arc;
use core::{
    ptr,
    sync::atomic::{AtomicPtr, AtomicU64, Ordering},
    time::Duration,
};

use super::{clock, context, event::Event, sched, smp::IrqSpinLock, thread::Thread};

const ITEMS_PER_PASS: usize = 64;
const PASS_NS: u64 = 1_000_000;

/// Cooperative callback budget. Long callbacks must check it while draining.
pub struct Budget {
    remaining: usize,
    deadline: u64,
}

impl Budget {
    pub(crate) fn new() -> Self {
        Self {
            remaining: ITEMS_PER_PASS,
            deadline: clock::monotonic_ns().saturating_add(PASS_NS),
        }
    }

    /// Grants one more item unless the item or elapsed-time budget is spent.
    pub fn take(&mut self) -> bool {
        if self.remaining == 0 || clock::monotonic_ns() >= self.deadline {
            return false;
        }
        self.remaining -= 1;
        true
    }
}

struct State {
    pending: bool,
    stopping: bool,
    waiter: Option<Wake>,
}

struct Wake {
    thread: *mut Thread,
    sequence: u64,
}
// SAFETY: a published target is the live service worker. Its sequence cannot
// finish parking until the unique claimant performs this matching wake.
unsafe impl Send for Wake {}
impl Wake {
    fn deliver(self) {
        assert!(
            sched::wake(self.thread, self.sequence),
            "service: stale wake sequence"
        );
    }
}

struct Inner {
    state: IrqSpinLock<State>,
    finished: Event,
    worker: AtomicPtr<Thread>,
    requests: AtomicU64,
    passes: AtomicU64,
}

/// Handle to one serial service callback. Cloning does not create a worker.
/// Call `shutdown` before releasing callback resources or unloading their module.
#[derive(Clone)]
pub struct Service {
    inner: Arc<Inner>,
}

impl Service {
    /// Creates interrupt service at scheduler priority `0..=7`.
    /// Return `true` from the callback when authoritative work remains.
    pub fn interrupt(
        callback: impl FnMut(&mut Budget) -> bool + Send + 'static,
        priority: u8,
    ) -> Self {
        assert!(priority <= 7, "service: invalid interrupt priority");
        Self::start(callback, Some(priority))
    }

    /// Creates ordinary thread service, suitable for serial executors.
    pub fn work(callback: impl FnMut(&mut Budget) -> bool + Send + 'static) -> Self {
        Self::start(callback, None)
    }

    fn start(
        mut callback: impl FnMut(&mut Budget) -> bool + Send + 'static,
        priority: Option<u8>,
    ) -> Self {
        context::assert_sleepable("service: create");
        let inner = Arc::new(Inner {
            state: IrqSpinLock::new(State {
                pending: false,
                stopping: false,
                waiter: None,
            }),
            finished: Event::new(),
            worker: AtomicPtr::new(ptr::null_mut()),
            requests: AtomicU64::new(0),
            passes: AtomicU64::new(0),
        });
        let worker = inner.clone();
        let run = move || {
            worker
                .worker
                .store(sched::current_thread(), Ordering::Release);
            let mut next_pass_ns = 0u64;
            loop {
                let mut state = worker.state.lock();
                if !state.pending {
                    if state.stopping {
                        drop(state);
                        // Callback destructors may belong to an unloading module.
                        drop(callback);
                        worker.worker.store(ptr::null_mut(), Ordering::Release);
                        worker.finished.signal();
                        return;
                    }
                    let current = sched::current_thread();
                    // SAFETY: the worker allocation remains live until exit.
                    let sequence = unsafe { &*current }.prepare_park();
                    state.waiter = Some(Wake {
                        thread: current,
                        sequence,
                    });
                    assert!(state.unlock_keep_irqs_disabled());
                    sched::park_current(current, sequence);
                    continue;
                }
                state.pending = false;
                drop(state);
                // Retain the deadline across idle transitions too: repeated IRQs
                // must not bypass throttling by arriving just before parking.
                let delay = next_pass_ns.saturating_sub(clock::monotonic_ns());
                if delay != 0 {
                    clock::sleep(Duration::from_nanos(delay));
                }
                let more = callback(&mut Budget::new());
                next_pass_ns = clock::monotonic_ns().saturating_add(50_000);
                worker.passes.fetch_add(1, Ordering::Relaxed);
                worker.state.lock().pending |= more;
            }
        };
        if let Some(priority) = priority {
            sched::create_ithread_with_priority(move |_| run(), 0, priority);
        } else {
            sched::run(run);
        }
        Self { inner }
    }

    /// Requests inspection without allocation or sleeping. Safe in a hard IRQ.
    /// Returns false once shutdown has closed admission.
    pub fn request(&self) -> bool {
        let target = {
            let mut state = self.inner.state.lock();
            if state.stopping {
                return false;
            }
            state.pending = true;
            self.inner.requests.fetch_add(1, Ordering::Relaxed);
            state.waiter.take()
        };
        // Directly address the sole parked worker. IRQ notification must not
        // scan a parking bucket containing arbitrarily many colliding keys.
        if let Some(target) = target {
            target.deliver();
        }
        true
    }

    /// Rejects waiting for this service from its own callback.
    pub(crate) fn assert_not_current(&self) {
        assert_ne!(
            self.inner.worker.load(Ordering::Acquire),
            sched::current_thread(),
            "service: callback cannot wait for its own executor"
        );
    }

    /// Closes admission, performs a final drain, and joins the callback thread.
    /// Quiesce external producers first. Safe to call concurrently or repeatedly.
    pub fn shutdown(&self) {
        context::assert_sleepable("service: shutdown");
        self.assert_not_current();
        let target = {
            let mut state = self.inner.state.lock();
            if !state.stopping {
                state.stopping = true;
                state.pending = true;
            }
            state.waiter.take()
        };
        if let Some(target) = target {
            target.deliver();
        }
        self.inner.finished.wait();
    }

    /// Returns accepted notifications and completed callback passes.
    pub fn counts(&self) -> (u64, u64) {
        (
            self.inner.requests.load(Ordering::Relaxed),
            self.inner.passes.load(Ordering::Relaxed),
        )
    }
}
