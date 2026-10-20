//! Ordered, intrusive indexes for memory-management metadata.
//!
//! Nodes come from typed pools and can be reserved before entering critical
//! sections. Publication, rekeying, and range detachment reuse those nodes so
//! structural changes need not allocate through the general kernel heap.
//! External owner locks protect indexes that are shared between threads.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use core::{cell::UnsafeCell, ops::RangeBounds};

use intrusive_collections::{
    Adapter, Bound, KeyAdapter, LinkOps, RBTree, RBTreeLink, UnsafeRef, intrusive_adapter,
};

use super::{
    Error, Result,
    pool::{Pool, Pooled},
};

pub(super) struct Node<T> {
    link: RBTreeLink,
    key: u64,
    value: UnsafeCell<Option<T>>,
}

intrusive_adapter!(Links<T> = UnsafeRef<Node<T>>: Node<T> { link: RBTreeLink });

struct TreeAdapter<T: 'static> {
    links: Links<T>,
    pool: &'static Pool<Node<T>>,
}

// SAFETY: Links supplies the container/link conversions. The owning pointer
// converter returns removed nodes to the same pool that allocated them.
unsafe impl<T> Adapter for TreeAdapter<T> {
    type LinkOps = <Links<T> as Adapter>::LinkOps;
    type PointerOps = &'static Pool<Node<T>>;

    unsafe fn get_value(&self, link: <Self::LinkOps as LinkOps>::LinkPtr) -> *const Node<T> {
        // SAFETY: forwarded from Adapter's link provenance requirement.
        unsafe { self.links.get_value(link) }
    }

    unsafe fn get_link(&self, value: *const Node<T>) -> <Self::LinkOps as LinkOps>::LinkPtr {
        // SAFETY: forwarded from Adapter's valid-container requirement.
        unsafe { self.links.get_link(value) }
    }

    fn link_ops(&self) -> &Self::LinkOps {
        self.links.link_ops()
    }
    fn link_ops_mut(&mut self) -> &mut Self::LinkOps {
        self.links.link_ops_mut()
    }
    fn pointer_ops(&self) -> &Self::PointerOps {
        &self.pool
    }
}

impl<'a, T> KeyAdapter<'a> for TreeAdapter<T> {
    type Key = u64;
    fn get_key(&self, value: &'a Node<T>) -> u64 {
        value.key
    }
}

/// Ordered index with fallible reservation and allocation-free publication.
pub(super) struct Index<T: 'static> {
    tree: RBTree<TreeAdapter<T>>,
    pool: &'static Pool<Node<T>>,
    len: usize,
}

// SAFETY: the index owns its nodes. Links and values are accessed only through
// index borrows; moving the index does not expose its internal shared pointers.
unsafe impl<T: Send> Send for Index<T> {}

// SAFETY: shared operations only traverse links and read payloads. All link
// and payload mutations require an exclusive index borrow; reservation only
// accesses the independently synchronized pool and returns an unpublished node.
unsafe impl<T: Sync> Sync for Index<T> {}

impl<T> Index<T> {
    pub(super) fn new(pool: &'static Pool<Node<T>>) -> Self {
        Self {
            tree: RBTree::new(TreeAdapter {
                links: Links::new(),
                pool,
            }),
            pool,
            len: 0,
        }
    }

    pub(super) fn len(&self) -> usize {
        self.len
    }

    pub(super) fn get(&self, key: &u64) -> Option<&T> {
        self.tree.find(key).get().map(|node| {
            // SAFETY: published nodes contain a value. The shared index borrow
            // excludes mutable access through get_mut or a mutable iterator.
            unsafe { (&*node.value.get()).as_ref().unwrap() }
        })
    }

    pub(super) fn get_mut(&mut self, key: &u64) -> Option<&mut T> {
        self.range_mut(*key..=*key).next().map(|(_, value)| value)
    }
    /// Move an existing tree node without allocating another map entry.
    pub(super) fn rekey(&mut self, old: u64, new: u64) {
        assert!(!self.contains_key(&new));
        let mut node = self
            .tree
            .find_mut(&old)
            .remove()
            .expect("mem: rekey missing entry");
        node.key = new;
        self.tree.insert(node);
    }

    pub(super) fn contains_key(&self, key: &u64) -> bool {
        self.get(key).is_some()
    }

    pub(super) fn reserve(&self, key: u64) -> Result<Option<Pooled<Node<T>>>> {
        if self.contains_key(&key) {
            return Ok(None);
        }
        self.reserve_new(key).map(Some)
    }

    pub(super) fn reserve_new(&self, key: u64) -> Result<Pooled<Node<T>>> {
        self.pool
            .try_alloc(Node {
                link: RBTreeLink::new(),
                key,
                value: UnsafeCell::new(None),
            })
            .map_err(|_| Error::OutOfMemory)
    }

    pub(super) fn insert(
        &mut self,
        key: u64,
        value: T,
        reserved: Option<Pooled<Node<T>>>,
    ) -> Option<T> {
        if let Some(node) = self.tree.find(&key).get() {
            // SAFETY: &mut self excludes outstanding node-value references;
            // changing the payload leaves the tree's links and key untouched.
            return unsafe { (&mut *node.value.get()).replace(value) };
        }
        let mut node = reserved.expect("mem: index insertion without reservation");
        assert_eq!(node.key, key, "mem: index reservation key mismatch");
        *node.value.get_mut() = Some(value);
        self.tree.insert(node);
        self.len += 1;
        None
    }

    pub(super) fn remove(&mut self, key: &u64) -> Option<T> {
        let node = self.tree.find_mut(key).remove()?;
        self.len -= 1;
        Pooled::into_inner(node).value.into_inner()
    }

    pub(super) fn iter(&self) -> impl DoubleEndedIterator<Item = (&u64, &T)> {
        self.tree.iter().map(|node| {
            // SAFETY: shared index access excludes mutation of published values.
            (&node.key, unsafe { (&*node.value.get()).as_ref().unwrap() })
        })
    }

    pub(super) fn values(&self) -> impl Iterator<Item = &T> {
        self.iter().map(|(_, value)| value)
    }

    pub(super) fn range(
        &self,
        range: impl RangeBounds<u64>,
    ) -> impl DoubleEndedIterator<Item = (&u64, &T)> {
        self.tree
            .range(bound(range.start_bound()), bound(range.end_bound()))
            .map(|node| {
                // SAFETY: shared index access excludes mutation of published values.
                (&node.key, unsafe { (&*node.value.get()).as_ref().unwrap() })
            })
    }

    pub(super) fn iter_mut(&mut self) -> impl DoubleEndedIterator<Item = (&u64, &mut T)> {
        self.tree.iter().map(|node| {
            // SAFETY: &mut self excludes other index access. The iterator
            // visits each node once, and values do not overlap links or keys.
            (&node.key, unsafe {
                (&mut *node.value.get()).as_mut().unwrap()
            })
        })
    }

    pub(super) fn range_mut(
        &mut self,
        range: impl RangeBounds<u64>,
    ) -> impl DoubleEndedIterator<Item = (&u64, &mut T)> {
        self.tree
            .range(bound(range.start_bound()), bound(range.end_bound()))
            .map(|node| {
                // SAFETY: exclusive index access and unique node visits make the
                // mutable values disjoint from each other and from keys and links.
                (&node.key, unsafe {
                    (&mut *node.value.get()).as_mut().unwrap()
                })
            })
    }

    pub(super) fn into_entries(self) -> impl Iterator<Item = (u64, T)> {
        self.tree.into_iter().map(|node| {
            let node = Pooled::into_inner(node);
            (node.key, node.value.into_inner().unwrap())
        })
    }

    pub(super) fn split_off(&mut self, key: &u64) -> Self {
        let mut removed = Self::new(self.pool);
        if self.tree.front().get().is_some_and(|node| node.key >= *key) {
            core::mem::swap(self, &mut removed);
            return removed;
        }
        let mut cursor = self.tree.lower_bound_mut(Bound::Included(key));
        while let Some(node) = cursor.remove() {
            removed.tree.insert(node);
            removed.len += 1;
        }
        self.len -= removed.len;
        removed
    }

    /// Transfer existing nodes, preserving their values without allocation.
    pub(super) fn extract_range(&mut self, start: u64, end: u64) -> Self {
        let mut removed = Self::new(self.pool);
        let mut cursor = self.tree.lower_bound_mut(Bound::Included(&start));
        while cursor.get().is_some_and(|node| node.key < end) {
            let node = cursor.remove().expect("mem: missing detached entry");
            removed.tree.insert(node);
            removed.len += 1;
        }
        self.len -= removed.len;
        removed
    }
}

fn bound(value: core::ops::Bound<&u64>) -> Bound<&u64> {
    match value {
        core::ops::Bound::Included(value) => Bound::Included(value),
        core::ops::Bound::Excluded(value) => Bound::Excluded(value),
        core::ops::Bound::Unbounded => Bound::Unbounded,
    }
}
