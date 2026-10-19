//! Memory objects and the unified filesystem page cache.
//!
//! A vnode has one `VmObject`, shared by descriptor I/O and every mapping.
//! Anonymous/tmpfs missing indexes are sparse zeros. Disk-backed objects load
//! missing indexes through a pager, and clean file frames can be discarded
//! and read again instead of occupying swap.
//!
//! # Synchronization invariants
//!
//! * Filesystems serialize writes that change logical length with truncation.
//! * The index and page state use the same owner lock. Transfers release it
//!   before resolving user windows, allocating physical frames, or doing I/O.
//! * Truncation deliberately holds the index lock as a fault gate until old
//!   PTEs are removed and bytes newly outside/inside EOF are sanitized.
//! * Object accounting is charged once when an index is inserted and released
//!   once when that index is removed or the object is destroyed.

use crate::mem::PageRef;
use alloc::sync::Arc;
use core::{
    cmp,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::mem::PAGE_SIZE;

use super::{
    Error, IoSink, IoSource, Result, VmPage,
    index::{Index, Node},
    owner::{Owned, OwnerLock},
    pool::Pool,
};

static PAGE_INDEX_POOL: Pool<Node<PageRef>> = Pool::new();

pub(super) fn drain_pool() {
    PAGE_INDEX_POOL.drain();
}

/// Pages resolved per index acquisition during bulk transfers.
const LOOKUP_BATCH: usize = 32;

/// Immutable backing store for demand-loaded objects. Implementations must
/// initialize the entire output, zeroing holes and bytes past their EOF. They
/// operate on kernel buffers and must not recurse into the object they back.
pub trait VmPager: Send + Sync {
    /// Loads a logical byte range, or returns an error without publishing it.
    fn read(&self, offset: u64, output: &mut [u8]) -> Result<()>;
}

/// Snapshot of one unified page cache.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct PageCacheStats {
    /// Logical file length tracked by the object.
    pub length: u64,
    /// Instantiated logical pages, including zero and swapped pages.
    pub pages: u64,
    /// Pages with resident physical frames.
    pub resident: u64,
    /// Pages represented by compressed or external swap.
    pub swapped: u64,
    /// Instantiated pages that have not materialized a frame.
    pub zero: u64,
    /// Resident pages modified since materialization.
    pub dirty: u64,
    /// Nonresident clean pages that can be loaded again from the file.
    pub file: u64,
}

/// Memory object's semantic owner.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ObjectKind {
    /// Anonymous process or kernel memory.
    Anonymous,
    /// Vnode-backed unified page cache.
    Vnode,
    /// Kernel-private pageable object.
    Kernel,
}

/// Shared page commitment account used by tmpfs mounts and similar owners.
pub struct PageAccount {
    used: AtomicU64,
    limit: Option<u64>,
}

impl PageAccount {
    /// Creates an account that permits at most `limit` allocated object pages.
    pub fn new(limit: u64) -> Arc<Self> {
        Arc::new(Self {
            used: AtomicU64::new(0),
            limit: Some(limit),
        })
    }

    /// Creates an account without an explicit page limit.
    pub fn unlimited() -> Arc<Self> {
        Arc::new(Self {
            used: AtomicU64::new(0),
            limit: None,
        })
    }

    /// Returns the optional page limit.
    pub fn limit(&self) -> Option<u64> {
        self.limit
    }

    /// Returns the current committed page count.
    pub fn used(&self) -> u64 {
        self.used.load(Ordering::Acquire)
    }

    fn reserve(&self) -> Result<()> {
        let mut current = self.used.load(Ordering::Acquire);
        loop {
            let next = current.checked_add(1).ok_or(Error::LimitExceeded)?;
            if self.limit.is_some_and(|limit| next > limit) {
                return Err(Error::LimitExceeded);
            }
            match self.used.compare_exchange_weak(
                current,
                next,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return Ok(()),
                Err(observed) => current = observed,
            }
        }
    }

    fn release(&self, pages: u64) {
        if pages == 0 {
            return;
        }
        let previous = self.used.fetch_sub(pages, Ordering::AcqRel);
        assert!(previous >= pages, "mem/object: page account underflow");
    }
}

/// Object-page membership survives an unlocked pager operation through a
/// weak handle. Clean file eviction removes the index entry under the same
/// owner capability that protects the frame (uvm_pagefree).
pub(super) struct ObjectPages(Owned<Index<PageRef>>);
impl ObjectPages {
    fn new() -> Arc<Self> {
        Arc::new(Self(Owned::new(
            OwnerLock::new().expect("mem: object owner allocation failed"),
            Index::new(&PAGE_INDEX_POOL),
        )))
    }
    pub(super) fn retire(&self, page: &PageRef, owner: &mut super::owner::OwnerWrite<'_>) {
        let pages = self.0.get_locked_mut(owner);
        if pages
            .get(&page.index())
            .is_some_and(|current| Arc::ptr_eq(current, page))
        {
            pages.remove(&page.index());
        }
    }
}
impl core::ops::Deref for ObjectPages {
    type Target = Owned<Index<PageRef>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// Page-cache object indexed by logical page number.
pub struct VmObject {
    id: u64,
    kind: ObjectKind,
    /// The object owner protects both index membership and its pages' state.
    /// Transfers drop it before faulting user memory; truncate holds it while
    /// invalidating mappings and sanitizing an already-pinned tail page.
    pages: Arc<ObjectPages>,
    account: Option<Arc<PageAccount>>,
    length: AtomicU64,
    pager: Option<Arc<dyn VmPager>>,
}

impl VmObject {
    /// Creates an empty object.
    pub fn new(kind: ObjectKind) -> Arc<Self> {
        Self::with_account(kind, None)
    }

    /// Creates an empty object charged to `account`.
    pub fn with_page_account(kind: ObjectKind, account: Arc<PageAccount>) -> Arc<Self> {
        Self::with_account(kind, Some(account))
    }

    fn with_account(kind: ObjectKind, account: Option<Arc<PageAccount>>) -> Arc<Self> {
        Arc::new(Self {
            id: super::allocate_object_id(),
            kind,
            pages: ObjectPages::new(),
            account,
            length: AtomicU64::new(0),
            pager: None,
        })
    }

    /// Creates an immutable file object without loading file contents.
    pub fn with_pager(length: u64, pager: Arc<dyn VmPager>) -> Arc<Self> {
        Arc::new(Self {
            id: super::allocate_object_id(),
            kind: ObjectKind::Vnode,
            pages: ObjectPages::new(),
            account: None,
            length: AtomicU64::new(length),
            pager: Some(pager),
        })
    }

    /// Publish pager pages under the object owner lock before loading them.
    /// Each page's busy/wanted state coalesces overlapping misses; neither
    /// physical allocation nor pager I/O holds the object lock.
    fn populate(&self, index: u64, wanted: usize) -> Result<()> {
        let Some(pager) = &self.pager else {
            return Ok(());
        };
        let end = self.length().div_ceil(PAGE_SIZE);
        if index >= end {
            return Err(Error::InvalidAddress);
        }
        let count = wanted.clamp(1, LOOKUP_BATCH).min((end - index) as usize);
        for next in index..index + count as u64 {
            let result = (|| {
                let page = match self.page(next) {
                    Some(page) => page,
                    None => {
                        let candidate = VmPage::file_in(
                            next,
                            pager.clone(),
                            self.pages.owner().clone(),
                            Some(Arc::downgrade(&self.pages)),
                        )?;
                        let mut pages = self.pages.lock();
                        if next >= self.length().div_ceil(PAGE_SIZE) {
                            return Err(Error::InvalidAddress);
                        }
                        if let Some(page) = pages.get(&next) {
                            page.clone()
                        } else {
                            let reserved = pages.reserve(next)?;
                            pages.insert(next, candidate.clone(), reserved);
                            candidate
                        }
                    }
                };
                page.pin_resident().map(|_| ())
            })();
            // Failure of speculative pages never changes the center's result.
            if next == index {
                result?;
            } else if result.is_err() {
                break;
            }
        }
        Ok(())
    }

    /// Returns the stable object identifier.
    pub fn id(&self) -> u64 {
        self.id
    }

    /// Whether the backing store currently supports only immutable pages.
    pub fn is_read_only(&self) -> bool {
        self.pager.is_some()
    }

    /// Returns the object kind.
    pub fn kind(&self) -> ObjectKind {
        self.kind
    }

    /// Returns the number of instantiated pages, resident or swapped.
    pub fn page_count(&self) -> u64 {
        self.pages.lock().len() as u64
    }

    /// Returns the logical byte length maintained by writes and truncation.
    pub fn length(&self) -> u64 {
        self.length.load(Ordering::Acquire)
    }

    /// Returns cache length and residency statistics.
    pub fn cache_stats(&self) -> PageCacheStats {
        let mut stats = PageCacheStats {
            length: self.length(),
            ..PageCacheStats::default()
        };
        let pages = self.pages.lock();
        stats.pages = pages.len() as u64;
        for page in pages.values() {
            let info = page.info_locked(pages.capability());
            match info.location {
                super::PageLocation::Zero => stats.zero += 1,
                super::PageLocation::File => stats.file += 1,
                super::PageLocation::Resident => stats.resident += 1,
                super::PageLocation::Swapped => stats.swapped += 1,
            }
            stats.dirty += u64::from(info.dirty);
        }
        stats
    }

    /// Looks up an instantiated page.
    pub fn page(&self, index: u64) -> Option<PageRef> {
        self.pages.lock().get(&index).cloned()
    }

    /// Returns an existing page or instantiates a zero-filled page.
    pub fn get_or_create_page(&self, index: u64) -> Result<PageRef> {
        self.populate(index, 1)?;
        self.find_or_create_page(index, false)
    }

    /// Returns the page used for a read or write fault.
    pub fn fault_page(&self, index: u64, write: bool) -> Result<PageRef> {
        if write && self.pager.is_some() {
            return Err(Error::Protection);
        }
        self.populate(index, 1)?;
        self.find_or_create_page(index, true)
    }

    /// Validates object identity and publishes the PTE while truncation is
    /// excluded. The fault must pin the page before taking this index lock,
    /// so installation cannot perform I/O or reclaim behind the truncate gate.
    pub(super) fn with_fault(
        &self,
        index: u64,
        page: &PageRef,
        install: impl FnOnce() -> Result<()>,
    ) -> Result<()> {
        let pages = self.pages.lock();
        if self.kind == ObjectKind::Vnode
            && index >= self.length.load(Ordering::Acquire).div_ceil(PAGE_SIZE)
        {
            return Err(Error::InvalidAddress);
        }
        if !pages
            .get(&index)
            .is_some_and(|current| Arc::ptr_eq(current, page))
        {
            return Err(Error::NotMapped);
        }
        install()
    }

    /// The locked pager lookup used for neighbors. Busy/nonresident pages are
    /// skipped; only the center fault may release locks to perform I/O.
    pub(super) fn with_neighbors(
        &self,
        start: u64,
        count: usize,
        visit: impl FnOnce(&[Option<PageRef>], &mut super::owner::OwnerWrite<'_>),
    ) {
        assert!(count <= 16);
        let mut pages = self.pages.lock();
        let end = self.length().div_ceil(PAGE_SIZE);
        let found: [Option<PageRef>; 16] = core::array::from_fn(|offset| {
            let index = start + offset as u64;
            if offset >= count || (self.kind == ObjectKind::Vnode && index >= end) {
                return None;
            }
            pages.get(&index).cloned()
        });
        visit(&found[..count], pages.capability_mut());
    }

    /// Reads bytes into a sink, returning zeros for holes.
    pub fn read_into(&self, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        let total = if self.pager.is_some() {
            sink.len()
                .min(usize::try_from(self.length().saturating_sub(offset)).unwrap_or(usize::MAX))
        } else {
            sink.len()
        };
        let mut read = 0usize;
        let mut batch: [Option<PageRef>; LOOKUP_BATCH] = [const { None }; LOOKUP_BATCH];
        let mut batch_index = u64::MAX;

        while read < total {
            let position = offset
                .checked_add(read as u64)
                .ok_or(Error::InvalidAddress)?;
            let page_index = position / PAGE_SIZE;
            let page_offset = (position % PAGE_SIZE) as usize;
            let limit = cmp::min(PAGE_SIZE as usize - page_offset, total - read);

            // Pages are resolved a batch at a time so a bulk transfer takes one
            // index acquisition per batch instead of one per page.
            if batch_index == u64::MAX
                || page_index < batch_index
                || page_index - batch_index >= LOOKUP_BATCH as u64
            {
                batch_index = page_index;
                let pages = self.pages.lock();
                for (slot, entry) in batch.iter_mut().enumerate() {
                    *entry = pages.get(&(batch_index + slot as u64)).cloned();
                }
            }

            if self.pager.is_some() && batch[(page_index - batch_index) as usize].is_none() {
                let wanted = (total - read)
                    .saturating_add(page_offset)
                    .div_ceil(PAGE_SIZE as usize);
                if let Err(error) = self.populate(page_index, wanted) {
                    return if read != 0 { Ok(read) } else { Err(error) };
                }
                batch_index = u64::MAX;
                continue;
            }

            // The page reference is taken before the sink window is resolved so
            // that no page backing lock is held across a user page fault.
            let page = batch[(page_index - batch_index) as usize].clone();
            let mut window = match sink.window(read, limit) {
                Ok(window) => window,
                Err(_) if read != 0 => return Ok(read),
                Err(error) => return Err(error),
            };
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            match page {
                Some(page) => {
                    if let Err(error) = window.read_page(&page, page_offset) {
                        return if read != 0 { Ok(read) } else { Err(error) };
                    }
                }
                None => window.fill(0),
            }
            read += count;
        }
        Ok(read)
    }

    /// Reads bytes into a kernel buffer, returning zeros for holes.
    pub fn read_at(&self, offset: u64, output: &mut [u8]) -> Result<usize> {
        self.read_into(offset, &mut IoSink::kernel(output))
    }

    /// Writes bytes from a source and returns the number accepted.
    pub fn write_from(&self, offset: u64, source: &IoSource<'_>) -> Result<usize> {
        if self.pager.is_some() {
            return Err(Error::Protection);
        }
        let total = source.len();
        let mut written = 0usize;
        while written < total {
            let position = offset
                .checked_add(written as u64)
                .ok_or(Error::InvalidAddress)?;
            let page_index = position / PAGE_SIZE;
            let page_offset = (position % PAGE_SIZE) as usize;
            let limit = cmp::min(PAGE_SIZE as usize - page_offset, total - written);

            // The source window is resolved before any page backing lock is
            // taken so that a user page fault never nests inside one.
            let window = match source.window(written, limit) {
                Ok(window) => window,
                Err(_) if written != 0 => return self.finish_write(offset, written),
                Err(error) => return Err(error),
            };
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            position
                .checked_add(count as u64)
                .ok_or(Error::InvalidAddress)?;
            if written == 0 && position > self.length() {
                self.zero_exposed_tail(self.length(), position)?;
            }
            let page = match self.get_or_create_page(page_index) {
                Ok(page) => page,
                Err(error) => {
                    if written != 0 {
                        return self.finish_write(offset, written);
                    }
                    return Err(error);
                }
            };
            if let Err(error) = window.write_page(&page, page_offset) {
                if written != 0 {
                    return self.finish_write(offset, written);
                }
                return Err(error);
            }
            written += count;
        }
        self.finish_write(offset, written)
    }

    /// Writes bytes from a kernel buffer and returns the number accepted.
    pub fn write_at(&self, offset: u64, input: &[u8]) -> Result<usize> {
        self.write_from(offset, &IoSource::kernel(input))
    }

    /// Removes pages beyond `size` and sanitizes a retained tail. Page-in is
    /// performed without the object lock; EOF and identity are revalidated
    /// before any PTE or index change (uvm_vnp_setsize/PG_BUSY protocol).
    pub fn truncate(&self, size: u64) -> Result<u64> {
        if self.pager.is_some() {
            return Err(Error::Protection);
        }
        loop {
            let old_size = self.length();
            if size == old_size {
                return Ok(0);
            }
            let tail_offset = if size > old_size { old_size } else { size };
            let tail = (tail_offset % PAGE_SIZE) as usize;
            let page = if tail != 0 {
                self.page(tail_offset / PAGE_SIZE)
            } else {
                None
            };
            let pin = page
                .as_ref()
                .filter(|page| page.info().location != super::PageLocation::Zero)
                .map(|page| page.pin_resident())
                .transpose()?;
            let mut pages = self.pages.lock();
            if self.length() != old_size
                || (tail != 0 && !same_page(pages.get(&(tail_offset / PAGE_SIZE)), page.as_ref()))
            {
                continue;
            }
            if size > old_size {
                if let Some(page) = &page {
                    super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
                    if page
                        .zero_locked(
                            tail,
                            cmp::min(size - old_size, PAGE_SIZE - tail as u64) as usize,
                            pages.capability_mut(),
                        )
                        .is_err()
                    {
                        continue;
                    }
                }
                self.length.store(size, Ordering::Release);
                return Ok(0);
            }
            let first_removed = size.div_ceil(PAGE_SIZE);
            for page in pages.range(first_removed..).map(|(_, page)| page) {
                super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
            }
            if let Some(page) = &page {
                super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
                if page
                    .zero_locked(tail, PAGE_SIZE as usize - tail, pages.capability_mut())
                    .is_err()
                {
                    continue;
                }
            }
            self.length.store(size, Ordering::Release);
            let removed = pages.split_off(&first_removed);
            let count = removed.len() as u64;
            drop(pages);
            drop(pin);
            drop(removed);
            if let Some(account) = &self.account {
                account.release(count);
            }
            return Ok(count);
        }
    }

    fn zero_exposed_tail(&self, old_size: u64, new_size: u64) -> Result<()> {
        let tail = (old_size % PAGE_SIZE) as usize;
        if tail == 0 || new_size <= old_size {
            return Ok(());
        }
        loop {
            let page = self.page(old_size / PAGE_SIZE);
            let Some(page) = page else {
                return Ok(());
            };
            let _pin = (page.info().location != super::PageLocation::Zero)
                .then(|| page.pin_resident())
                .transpose()?;
            let mut pages = self.pages.lock();
            if self.length() != old_size {
                return Ok(());
            }
            if !same_page(pages.get(&(old_size / PAGE_SIZE)), Some(&page)) {
                continue;
            }
            super::pmap::remove_mappings_for_pages(core::slice::from_ref(&page));
            if page
                .zero_locked(
                    tail,
                    cmp::min(new_size - old_size, PAGE_SIZE - tail as u64) as usize,
                    pages.capability_mut(),
                )
                .is_ok()
            {
                return Ok(());
            }
        }
    }

    fn find_or_create_page(&self, index: u64, check_length: bool) -> Result<PageRef> {
        let validate = || {
            if check_length
                && self.kind == ObjectKind::Vnode
                && index >= self.length.load(Ordering::Acquire).div_ceil(PAGE_SIZE)
            {
                Err(Error::InvalidAddress)
            } else {
                Ok(())
            }
        };
        if self.pager.is_some() {
            loop {
                let pages = self.pages.lock();
                validate()?;
                if let Some(page) = pages.get(&index) {
                    return Ok(page.clone());
                }
                drop(pages);
                self.populate(index, 1)?;
            }
        }
        {
            let pages = self.pages.lock();
            validate()?;
            if let Some(page) = pages.get(&index) {
                return Ok(page.clone());
            }
        }
        let page = VmPage::new_zero_in(
            index,
            // tmpfs exposes a vnode view of its underlying anonymous object;
            // those swap-backed pages count as aobj pages in UVM's clock.
            if self.kind == ObjectKind::Vnode && self.pager.is_none() {
                super::phys::PageOwnerKind::Anonymous
            } else {
                super::page::owner_kind_for_object(self.kind)
            },
            self.pages.owner().clone(),
        )?;
        let mut pages = self.pages.lock();
        // Allocation dropped the index lock. Recheck EOF and identity before
        // charging or publishing, including concurrent truncate and creators.
        validate()?;
        if let Some(existing) = pages.get(&index) {
            return Ok(existing.clone());
        }
        let reserved = pages.reserve(index)?;
        if let Some(account) = &self.account {
            account.reserve()?;
        }
        pages.insert(index, page.clone(), reserved);
        Ok(page)
    }

    /// Publishes one transfer's final length once, keeping the per-page write
    /// path free of contended atomic read-modify-write operations.
    fn finish_write(&self, offset: u64, written: usize) -> Result<usize> {
        if written != 0 {
            let end = offset
                .checked_add(written as u64)
                .ok_or(Error::InvalidAddress)?;
            self.length.fetch_max(end, Ordering::AcqRel);
        }
        Ok(written)
    }
}

impl Drop for VmObject {
    fn drop(&mut self) {
        let pages = self.pages.lock();
        for page in pages.values() {
            super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
        }
        if let Some(account) = &self.account {
            account.release(pages.len() as u64);
        }
    }
}

fn same_page(left: Option<&PageRef>, right: Option<&PageRef>) -> bool {
    match (left, right) {
        (Some(left), Some(right)) => Arc::ptr_eq(left, right),
        (None, None) => true,
        _ => false,
    }
}
