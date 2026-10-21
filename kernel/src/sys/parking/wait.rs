//! Scheduler handshake shared by event, completion, and predicate waits.
//!
//! Registrations are pinned until every source has been unlinked. Selecting a
//! winner transfers wake responsibility, not permission to drop its storage:
//! the waiter must consume the matching scheduler wake before returning.

use super::{Node, queue};
use crate::{
    arch,
    sys::{context, sched, smp, thread::Thread},
};
use alloc::vec::Vec;
use core::{
    ptr,
    sync::atomic::{AtomicPtr, AtomicUsize, Ordering},
};

const NO_WINNER: usize = usize::MAX;
pub(crate) const INLINE_WAITERS: usize = 8;

/// Atomic state operations performed under a parking bucket.
///
/// Methods must not allocate, block, or acquire locks.
///
/// # Safety
/// The key must uniquely identify this source's live, immovable state. Only
/// this module may enqueue payloads under that key: wake interprets them as
/// pinned WaitGroups. The source and its key must outlive every registration.
pub(crate) unsafe trait WaitSource: Sync {
    fn key(&self) -> usize;
    fn should_wait(&self, ticket: u64) -> bool;
    /// Publish/clear the hint under the bucket, preserving notification state.
    /// Publication must be an acquire/release RMW paired with notification.
    fn set_queued(&self, queued: bool);
}

#[derive(Clone, Copy)]
pub(crate) struct Entry<'a> {
    pub(crate) source: &'a dyn WaitSource,
    pub(crate) ticket: u64,
}

struct WaitGroup {
    thread: *mut Thread,
    park_seq: u64,
    winner: AtomicUsize,
    wake_next: AtomicPtr<WaitGroup>,
}

#[derive(Copy, Clone)]
struct WakeTarget {
    thread: *mut Thread,
    park_seq: u64,
}

struct WakeBatch {
    head: *mut WaitGroup,
    tail: *mut WaitGroup,
    count: usize,
}

impl WaitGroup {
    fn new(thread: *mut Thread, park_seq: u64) -> Self {
        Self {
            thread,
            park_seq,
            winner: AtomicUsize::new(NO_WINNER),
            wake_next: AtomicPtr::new(ptr::null_mut()),
        }
    }

    fn try_claim(&self, index: usize) -> bool {
        self.winner
            .compare_exchange(NO_WINNER, index, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    fn winner(&self) -> Option<usize> {
        match self.winner.load(Ordering::Acquire) {
            NO_WINNER => None,
            index => Some(index),
        }
    }

    fn target(&self) -> WakeTarget {
        WakeTarget {
            thread: self.thread,
            park_seq: self.park_seq,
        }
    }
}

impl WakeTarget {
    fn wake(self) {
        assert!(
            sched::wake(self.thread, self.park_seq),
            "parking: claimed waiter had a stale park sequence"
        );
    }
}

impl WakeBatch {
    const fn new() -> Self {
        Self {
            head: ptr::null_mut(),
            tail: ptr::null_mut(),
            count: 0,
        }
    }

    fn push(&mut self, group: *mut WaitGroup) {
        // SAFETY: a successfully claimed wait group remains pinned until this
        // batch invokes its matching scheduler wake.
        let group_ref = unsafe { &*group };
        group_ref
            .wake_next
            .store(ptr::null_mut(), Ordering::Relaxed);

        if self.tail.is_null() {
            self.head = group;
        } else {
            // SAFETY: `tail` is a previously claimed group in this batch and
            // remains live until the batch reaches it.
            unsafe { &*self.tail }
                .wake_next
                .store(group, Ordering::Relaxed);
        }

        self.tail = group;
        self.count += 1;
    }

    fn wake(self) -> usize {
        let mut group = self.head;
        while !group.is_null() {
            // SAFETY: the claimant owns wake responsibility for this pinned
            // group. Read everything needed before waking because the target
            // may immediately run, unlink its registrations, and return.
            let (next, target) = unsafe {
                let group_ref = &*group;
                (
                    group_ref.wake_next.load(Ordering::Relaxed),
                    group_ref.target(),
                )
            };
            target.wake();
            group = next;
        }
        self.count
    }
}

/// Check before constructing a wait set, which may allocate for large sets.
pub(crate) fn assert_wait_context() {
    assert!(
        arch::irqstate() && !smp::in_interrupt_context(),
        "parking: wait requires thread context with interrupts enabled"
    );
    context::assert_sleepable("parking: wait");
}

/// Wait for one source. `release` runs exactly once after registration and
/// outside all buckets, including on the ready fast path. It may unlock the
/// predicate mutex; it must not itself park the current thread.
pub(crate) fn wait(entries: &[Entry<'_>], release: impl FnOnce()) -> usize {
    assert!(!entries.is_empty(), "parking: empty wait set");
    assert_wait_context();
    if let Some(index) = entries
        .iter()
        .position(|entry| !entry.source.should_wait(entry.ticket))
    {
        release();
        return index;
    }

    let current = sched::current_thread();
    // SAFETY: the scheduler keeps its current thread live while it executes.
    let seq = unsafe { (&*current).prepare_park() };
    let group = core::pin::pin!(WaitGroup::new(current, seq));
    let group = group.as_ref().get_ref();
    let pointer = (group as *const WaitGroup).cast();
    let make_node = |index: usize| Node::new(entries[index].source.key(), pointer, index);
    if entries.len() == 1 {
        let node = core::pin::pin!(make_node(0));
        wait_registered(
            entries,
            core::slice::from_ref(node.as_ref().get_ref()),
            group,
            release,
        )
    } else if entries.len() <= INLINE_WAITERS {
        let nodes = core::pin::pin!(core::array::from_fn::<_, INLINE_WAITERS, _>(|index| {
            make_node(index.min(entries.len() - 1))
        }));
        wait_registered(
            entries,
            &nodes.as_ref().get_ref()[..entries.len()],
            group,
            release,
        )
    } else {
        // A boxed slice keeps its nodes stable even if the Box itself moves.
        let nodes = (0..entries.len())
            .map(make_node)
            .collect::<Vec<_>>()
            .into_boxed_slice();
        wait_registered(entries, &nodes, group, release)
    }
}

fn wait_registered(
    entries: &[Entry<'_>],
    nodes: &[Node],
    group: &WaitGroup,
    release: impl FnOnce(),
) -> usize {
    for (entry, node) in entries.iter().zip(nodes) {
        if group.winner().is_some() {
            break;
        }
        let ready = {
            let mut queue = queue(entry.source.key());
            if !entry.source.should_wait(entry.ticket) {
                true
            } else {
                // SAFETY: nodes and group stay pinned through cleanup below;
                // the entries borrow each source for the entire wait.
                unsafe {
                    queue.push(node);
                }
                entry.source.set_queued(true);
                // Notification may have skipped this bucket before QUEUED was
                // published. The RMW above acquires that notification, so this
                // second validation must see its changed state.
                if entry.source.should_wait(entry.ticket) {
                    false
                } else {
                    queue.remove(node);
                    entry.source.set_queued(!queue.is_empty());
                    true
                }
            }
        };
        if ready {
            if group.try_claim(node.token) {
                group.target().wake();
            }
            break;
        }
    }

    // Mutex unlock can wake its own parking bucket. Never call it under an
    // event bucket, even when the two keys hash to different buckets.
    release();
    // Always consume the matching wake, even if a winner is already visible:
    // its claimant may still be using the group to construct a wake batch.
    arch::irqset(false);
    sched::park_current(group.thread, group.park_seq);

    for (entry, node) in entries.iter().zip(nodes) {
        let mut queue = queue(entry.source.key());
        queue.remove(node);
        entry.source.set_queued(!queue.is_empty());
    }
    group.winner().expect("parking: resumed without a winner")
}

pub(crate) fn wake_one(source: &impl WaitSource) -> bool {
    let target = {
        let mut queue = queue(source.key());
        let mut target = None;
        while let Some(node) = queue.pop() {
            // SAFETY: WaitSource owns this key's payload type. Cleanup must
            // acquire this bucket before dropping the pinned group.
            let group = unsafe { &*node.data.cast::<WaitGroup>() };
            if group.try_claim(node.token) {
                target = Some(group.target());
                break;
            }
        }
        source.set_queued(!queue.is_empty());
        target
    };
    if let Some(target) = target {
        target.wake();
        true
    } else {
        false
    }
}

pub(crate) fn wake_all(source: &impl WaitSource) -> usize {
    let batch = {
        let mut batch = WakeBatch::new();
        let mut queue = queue(source.key());
        while let Some(node) = queue.pop() {
            let pointer = node.data.cast::<WaitGroup>().cast_mut();
            // SAFETY: the bucket protects the pinned payload; a successful
            // claim keeps it alive until this batch delivers its wake.
            if unsafe { &*pointer }.try_claim(node.token) {
                batch.push(pointer);
            }
        }
        source.set_queued(false);
        batch
    };
    batch.wake()
}
