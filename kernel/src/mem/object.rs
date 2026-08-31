//! Memory objects and the unified filesystem page cache.
//!
//! A vnode has one `VmObject`, shared by descriptor I/O and every mapping, so
//! coherence does not require a second buffer or copy. Missing indexes are
//! sparse zeros; instantiated pages use the ordinary VM resident/swap state
//! and therefore participate in the same reclaim policy as anonymous memory.
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

use alloc::{collections::BTreeMap, sync::Arc};
use core::{
    cmp,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{mem::PAGE_SIZE, sys::sync::Mutex};

use super::{Error, IoSink, IoSource, Result, VmPage};

/// Pages resolved per index acquisition during bulk transfers.
const LOOKUP_BATCH: usize = 16;

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
        })
    }

    /// Returns the stable object identifier.
    pub fn id(&self) -> u64 {
        self.id
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
        let mut pages = self.pages.lock();
        if let Some(page) = pages.get(&index) {
            return Ok(page.clone());
        }
        self.create_page_locked(&mut pages, index)
    }

    /// Returns the page used for a read or write fault.
    pub fn fault_page(&self, index: u64, _write: bool) -> Result<Arc<VmPage>> {
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
        let total = sink.len();
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

            // The page reference is taken before the sink window is resolved so
            // that no page backing lock is held across a user page fault.
            let page = batch[(page_index - batch_index) as usize].clone();
            let mut window = sink.window(read, limit)?;
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            match page {
                Some(page) => page.read(page_offset, &mut window)?,
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
