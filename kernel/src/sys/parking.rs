//! Shared, address-keyed waiter storage for sleeping synchronization.
//!
//! Buckets contain pinned registrations, not heap-allocated queues. All queue
//! operations match the complete key, so hash collisions only add contention.
//! Primitive state must be rechecked while its queue is locked. Scheduler wakes
//! happen after unlocking; the scheduler's sequence handshake closes the gap
//! between publishing a waiter and actually leaving the CPU.

pub(crate) mod wait;

use intrusive_collections::{LinkedList, LinkedListLink, UnsafeRef, intrusive_adapter};

use super::smp::{IrqSpinLock, IrqSpinLockGuard};

const BUCKETS: usize = 256;
static TABLE: [IrqSpinLock<Bucket>; BUCKETS] = [const { IrqSpinLock::new(Bucket(None)) }; BUCKETS];

struct Bucket(Option<List>);
// SAFETY: intrusive links and payloads are only accessed with this bucket locked.
unsafe impl Send for Bucket {}

type List = LinkedList<NodeAdapter>;
intrusive_adapter!(NodeAdapter = UnsafeRef<Node>: Node { link: LinkedListLink });

/// One registration. Its key and payload remain immutable while queued.
pub(crate) struct Node {
    link: LinkedListLink,
    key: usize,
    /// Primitive-specific pinned state, interpreted only by the key's owner.
    pub(crate) data: *const (),
    /// Primitive-specific discriminator, such as a wait-any index.
    pub(crate) token: usize,
}

impl Node {
    pub(crate) fn new(key: usize, data: *const (), token: usize) -> Self {
        Self {
            link: LinkedListLink::new(),
            key,
            data,
            token,
        }
    }
}

/// Snapshot taken while the bucket protects registration storage.
pub(crate) struct Selected {
    pub(crate) data: *const (),
    pub(crate) token: usize,
}

/// Exclusive access to registrations for one key, with local IRQs masked.
pub(crate) struct Queue<'a> {
    key: usize,
    bucket: IrqSpinLockGuard<'a, Bucket>,
}

pub(crate) fn queue(key: usize) -> Queue<'static> {
    // Mix aligned object addresses before selecting a power-of-two bucket.
    let mixed = key ^ (key >> 8) ^ (key >> 16);
    let mut bucket = TABLE[mixed & (BUCKETS - 1)].lock();
    bucket.0.get_or_insert_with(|| List::new(NodeAdapter::NEW));
    Queue { key, bucket }
}

impl Queue<'_> {
    pub(crate) fn is_empty(&self) -> bool {
        !self
            .bucket
            .0
            .as_ref()
            .unwrap()
            .iter()
            .any(|node| node.key == self.key)
    }

    /// # Safety
    /// `node` and its payload must remain pinned and live until removed under
    /// this same key. Its key must identify a live, immovable primitive.
    pub(crate) unsafe fn push(&mut self, node: &Node) {
        assert_eq!(node.key, self.key);
        // SAFETY: the caller guarantees pinning and lifetime through removal.
        self.bucket
            .0
            .as_mut()
            .unwrap()
            .push_back(unsafe { UnsafeRef::from_raw(node) });
    }

    /// Removes the oldest matching registration. Inspect it before unlocking.
    pub(crate) fn pop(&mut self) -> Option<Selected> {
        let mut cursor = self.bucket.0.as_mut().unwrap().front_mut();
        while let Some(node) = cursor.get() {
            if node.key == self.key {
                return cursor.remove().map(|node| Selected {
                    data: node.data,
                    token: node.token,
                });
            }
            cursor.move_next();
        }
        None
    }

    pub(crate) fn remove(&mut self, node: &Node) -> bool {
        assert_eq!(node.key, self.key);
        if !node.link.is_linked() {
            return false;
        }
        // SAFETY: a linked node for this key belongs to this locked bucket;
        // the enqueue contract keeps it pinned until this removal.
        unsafe {
            self.bucket
                .0
                .as_mut()
                .unwrap()
                .cursor_mut_from_ptr(node)
                .remove();
        }
        true
    }

    pub(crate) fn for_each(&self, mut visit: impl FnMut(&Node)) {
        for node in self
            .bucket
            .0
            .as_ref()
            .unwrap()
            .iter()
            .filter(|node| node.key == self.key)
        {
            visit(node);
        }
    }

    pub(crate) fn irqs_were_enabled(&self) -> bool {
        self.bucket.irqs_were_enabled()
    }

    pub(crate) fn unlock_keep_irqs_disabled(self) -> bool {
        self.bucket.unlock_keep_irqs_disabled()
    }
}
