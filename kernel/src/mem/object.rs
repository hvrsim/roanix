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
//! * The page-index lock is released before resolving user windows or taking a
//!   page backing lock on normal transfers.
//! * Truncation deliberately holds the index lock as a fault gate until old
//!   PTEs are removed and bytes newly outside/inside EOF are sanitized.
//! * Object accounting is charged once when an index is inserted and released
//!   once when that index is removed or the object is destroyed.

use alloc::{collections::BTreeMap, sync::Arc, vec::Vec};
use core::{
    cmp,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{mem::PAGE_SIZE, sys::sync::Mutex};

use super::{Error, IoSink, IoSource, Result, VmPage};

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

/// Page-cache object indexed by logical page number.
pub struct VmObject {
    id: u64,
    kind: ObjectKind,
    /// Protects index membership. Ordinary transfers never hold it while
    /// faulting user memory or acquiring a page backing lock. Truncate is the
    /// deliberate exception: it holds the index as a fault gate while it
    /// invalidates mappings and sanitizes the retained tail page.
    pages: Mutex<BTreeMap<u64, Arc<VmPage>>>,
    account: Option<Arc<PageAccount>>,
    length: AtomicU64,
    pager: Option<Arc<dyn VmPager>>,
    fill: [Mutex<()>; 16],
    next_read: AtomicU64,
    next_fault: AtomicU64,
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
            pages: Mutex::new(BTreeMap::new()),
            account,
            length: AtomicU64::new(0),
            pager: None,
            fill: core::array::from_fn(|_| Mutex::new(())),
            next_read: AtomicU64::new(u64::MAX),
            next_fault: AtomicU64::new(u64::MAX),
        })
    }

    /// Creates an immutable file object without loading file contents.
    pub fn with_pager(length: u64, pager: Arc<dyn VmPager>) -> Arc<Self> {
        Arc::new(Self {
            id: super::allocate_object_id(),
            kind: ObjectKind::Vnode,
            pages: Mutex::new(BTreeMap::new()),
            account: None,
            length: AtomicU64::new(length),
            pager: Some(pager),
            fill: core::array::from_fn(|_| Mutex::new(())),
            next_read: AtomicU64::new(u64::MAX),
            next_fault: AtomicU64::new(u64::MAX),
        })
    }

    /// Fill a bounded run of missing pages. The sleepable fill gate coalesces
    /// concurrent overlapping misses; disjoint stripes progress independently.
    /// Cache hits never take a fill gate. No index lock crosses I/O.
    fn populate(&self, index: u64, wanted: usize) -> Result<()> {
        let Some(pager) = &self.pager else {
            return Ok(());
        };
        if self.page(index).is_some() {
            return Ok(());
        }
        let _fill = self.fill[(index / LOOKUP_BATCH as u64) as usize % self.fill.len()].lock();
        if self.page(index).is_some() {
            return Ok(());
        }
        let end = self.length().div_ceil(PAGE_SIZE);
        if index >= end {
            return Err(Error::InvalidAddress);
        }
        let boundary = LOOKUP_BATCH - (index % LOOKUP_BATCH as u64) as usize;
        let mut count = wanted.clamp(1, boundary).min((end - index) as usize);
        {
            let pages = self.pages.lock();
            for next in 1..count {
                if pages.contains_key(&(index + next as u64)) {
                    count = next;
                    break;
                }
            }
        }
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(count * PAGE_SIZE as usize)
            .map_err(|_| Error::OutOfMemory)?;
        bytes.resize(count * PAGE_SIZE as usize, 0);
        if let Err(error) = pager.read(index * PAGE_SIZE, &mut bytes) {
            if count == 1 {
                return Err(error);
            }
            // A speculative neighbour must not make the requested page fail.
            pager.read(index * PAGE_SIZE, &mut bytes[..PAGE_SIZE as usize])?;
            count = 1;
            bytes.truncate(PAGE_SIZE as usize);
        }
        // Prepare every page before making any of this batch visible.
        let mut ready = Vec::new();
        ready
            .try_reserve_exact(count)
            .map_err(|_| Error::OutOfMemory)?;
        for (slot, bytes) in bytes
            .as_chunks::<{ PAGE_SIZE as usize }>()
            .0
            .iter()
            .enumerate()
        {
            ready.push(VmPage::from_pager(
                index + slot as u64,
                pager.clone(),
                bytes,
            )?);
        }
        let mut pages = self.pages.lock();
        for page in ready {
            pages.insert(page.index(), page);
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
            let info = page.info();
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
    pub fn page(&self, index: u64) -> Option<Arc<VmPage>> {
        self.pages.lock().get(&index).cloned()
    }

    /// Returns an existing page or instantiates a zero-filled page.
    pub fn get_or_create_page(&self, index: u64) -> Result<Arc<VmPage>> {
        self.populate(index, 1)?;
        let mut pages = self.pages.lock();
        if let Some(page) = pages.get(&index) {
            return Ok(page.clone());
        }
        self.create_page_locked(&mut pages, index)
    }

    /// Returns the page used for a read or write fault.
    pub fn fault_page(&self, index: u64, write: bool) -> Result<Arc<VmPage>> {
        if write && self.pager.is_some() {
            return Err(Error::Protection);
        }
        let previous = self
            .next_fault
            .swap(index.saturating_add(1), Ordering::Relaxed);
        self.populate(index, if previous == index { LOOKUP_BATCH } else { 1 })?;
        let mut pages = self.pages.lock();
        // The index check and insertion share the same lock as truncate's
        // split, preventing a stale pre-truncate fault from recreating a page
        // beyond the new EOF after truncation has removed it.
        if self.kind == ObjectKind::Vnode
            && index >= self.length.load(Ordering::Acquire).div_ceil(PAGE_SIZE)
        {
            return Err(Error::InvalidAddress);
        }
        if let Some(page) = pages.get(&index) {
            return Ok(page.clone());
        }
        self.create_page_locked(&mut pages, index)
    }

    /// Confirms that a just-installed translation still names a live index.
    ///
    /// Truncate holds the same index lock while removing PTEs. Validation
    /// after `Pmap::enter` closes the remaining race where a fault acquired an
    /// `Arc<VmPage>` immediately before truncate acquired the lock.
    pub(super) fn validates_fault(&self, index: u64, page: &Arc<VmPage>) -> bool {
        let pages = self.pages.lock();
        if self.kind == ObjectKind::Vnode
            && index >= self.length.load(Ordering::Acquire).div_ceil(PAGE_SIZE)
        {
            return false;
        }
        pages
            .get(&index)
            .is_some_and(|current| Arc::ptr_eq(current, page))
    }

    /// Reads bytes into a sink, returning zeros for holes.
    pub fn read_into(&self, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        let total = if self.pager.is_some() {
            sink.len()
                .min(usize::try_from(self.length().saturating_sub(offset)).unwrap_or(usize::MAX))
        } else {
            sink.len()
        };
        let sequential = self
            .next_read
            .swap(offset.saturating_add(total as u64), Ordering::Relaxed)
            == offset;
        let mut read = 0usize;
        let mut batch: [Option<Arc<VmPage>>; LOOKUP_BATCH] = [const { None }; LOOKUP_BATCH];
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
                let wanted = if sequential {
                    wanted.max(LOOKUP_BATCH)
                } else {
                    wanted
                };
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
                    if let Err(error) = page.read(page_offset, &mut window) {
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
            let page = match self.get_or_create_page(page_index) {
                Ok(page) => page,
                Err(error) => {
                    if written != 0 {
                        return self.finish_write(offset, written);
                    }
                    return Err(error);
                }
            };
            if let Err(error) = page.write(page_offset, &window) {
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

    /// Removes pages beyond `size` and zeroes a retained partial tail.
    pub fn truncate(&self, size: u64) -> Result<u64> {
        if self.pager.is_some() {
            return Err(Error::Protection);
        }
        let mut pages = self.pages.lock();
        let old_size = self.length.load(Ordering::Acquire);
        if size == old_size {
            return Ok(0);
        }

        if size > old_size {
            let old_tail = (old_size % PAGE_SIZE) as usize;
            if old_tail != 0
                && let Some(page) = pages.get(&(old_size / PAGE_SIZE))
            {
                // A shared mapping may have written into the rounded portion
                // beyond EOF. Retire its PTE before exposing that range, then
                // restore the filesystem guarantee that a grown hole is zero.
                super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
                let exposed = cmp::min(size - old_size, PAGE_SIZE - old_tail as u64) as usize;
                page.zero(old_tail, exposed)?;
            }
            self.length.store(size, Ordering::Release);
            return Ok(0);
        }

        let first_removed = size.div_ceil(PAGE_SIZE);
        let tail = (size % PAGE_SIZE) as usize;
        let tail_page = if tail == 0 {
            None
        } else {
            pages.get(&(size / PAGE_SIZE)).cloned()
        };

        // Faults cannot pass the index lock while resident translations are
        // retired. Once the shootdown completes, no userspace CPU can race the
        // tail zeroing through an old writable PTE.
        for page in pages.range(first_removed..).map(|(_, page)| page) {
            super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
        }
        if let Some(page) = tail_page.as_ref() {
            super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
        }
        if let Some(page) = tail_page {
            page.zero(tail, PAGE_SIZE as usize - tail)?;
        }

        // Publish EOF under the same lock checked by `fault_page`, then remove
        // the now-unreachable indexes before allowing another fault through.
        self.length.store(size, Ordering::Release);
        let removed = pages.split_off(&first_removed);
        let removed_count = removed.len() as u64;
        drop(pages);
        drop(removed);
        if let Some(account) = &self.account {
            account.release(removed_count);
        }
        Ok(removed_count)
    }

    fn create_page_locked(
        &self,
        pages: &mut BTreeMap<u64, Arc<VmPage>>,
        index: u64,
    ) -> Result<Arc<VmPage>> {
        if let Some(account) = &self.account {
            account.reserve()?;
        }
        let page = VmPage::new_zero(index, super::page::owner_kind_for_object(self.kind));
        pages.insert(index, page.clone());
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
        for page in self.pages.get_mut().values() {
            super::pmap::remove_mappings_for_pages(core::slice::from_ref(page));
        }
        if let Some(account) = &self.account {
            account.release(self.pages.get_mut().len() as u64);
        }
    }
}
