//! Memory objects and unified page-cache implementation.

use alloc::{collections::BTreeMap, sync::Arc};
use core::{
    cmp,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{mem::PAGE_SIZE, sys::sync::Mutex};

use super::{Error, IoSink, IoSource, Result, VmPage};

/// Pages resolved per index acquisition during bulk transfers.
const LOOKUP_BATCH: usize = 16;

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
    pages: Mutex<BTreeMap<u64, Arc<VmPage>>>,
    account: Option<Arc<PageAccount>>,
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

    /// Looks up an instantiated page.
    pub fn page(&self, index: u64) -> Option<Arc<VmPage>> {
        self.pages.lock().get(&index).cloned()
    }

    /// Returns an existing page or instantiates a zero-filled page.
    pub fn get_or_create_page(&self, index: u64) -> Result<Arc<VmPage>> {
        self.get_or_create_page_inner(index).map(|(page, _)| page)
    }

    fn get_or_create_page_inner(&self, index: u64) -> Result<(Arc<VmPage>, bool)> {
        let mut pages = self.pages.lock();
        if let Some(page) = pages.get(&index) {
            return Ok((page.clone(), false));
        }
        if let Some(account) = &self.account {
            account.reserve()?;
        }
        let page = VmPage::new_zero(index, super::page::owner_kind_for_object(self.kind));
        pages.insert(index, page.clone());
        Ok((page, true))
    }

    /// Returns the page used for a read or write fault.
    pub fn fault_page(&self, index: u64, write: bool) -> Result<Arc<VmPage>> {
        if let Some(page) = self.page(index) {
            return Ok(page);
        }
        let _ = write;
        self.get_or_create_page(index)
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
            if batch_index == u64::MAX || page_index < batch_index || page_index - batch_index >= LOOKUP_BATCH as u64
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
            let window = sink.window(read, limit)?;
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            match page {
                Some(page) => page.read(page_offset, window)?,
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
                Err(_) if written != 0 => return Ok(written),
                Err(error) => return Err(error),
            };
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            let existing = self.pages.lock().get(&page_index).cloned();
            if let Some(page) = existing {
                if let Err(error) = page.write(page_offset, window) {
                    if written != 0 {
                        return Ok(written);
                    }
                    return Err(error);
                }
            } else {
                if let Some(account) = &self.account
                    && let Err(error) = account.reserve()
                {
                    if written != 0 {
                        return Ok(written);
                    }
                    return Err(error);
                }
                let page =
                    VmPage::new_zero(page_index, super::page::owner_kind_for_object(self.kind));
                if let Err(error) = page.write(page_offset, window) {
                    if let Some(account) = &self.account {
                        account.release(1);
                    }
                    if written != 0 {
                        return Ok(written);
                    }
                    return Err(error);
                }
                let mut pages = self.pages.lock();
                if let Some(existing) = pages.get(&page_index) {
                    if let Some(account) = &self.account {
                        account.release(1);
                    }
                    existing.write(page_offset, window)?;
                } else {
                    pages.insert(page_index, page);
                }
            }
            written += count;
        }
        Ok(written)
    }

    /// Writes bytes from a kernel buffer and returns the number accepted.
    pub fn write_at(&self, offset: u64, input: &[u8]) -> Result<usize> {
        self.write_from(offset, &IoSource::kernel(input))
    }

    /// Removes pages beyond `size` and zeroes a retained partial tail.
    pub fn truncate(&self, size: u64) -> Result<u64> {
        let first_removed = size.div_ceil(PAGE_SIZE);
        let tail = (size % PAGE_SIZE) as usize;
        let tail_page = if tail == 0 {
            None
        } else {
            self.pages.lock().get(&(size / PAGE_SIZE)).cloned()
        };
        if let Some(page) = tail_page {
            page.zero(tail, PAGE_SIZE as usize - tail)?;
        }

        let mut pages = self.pages.lock();
        let removed = pages.split_off(&first_removed);
        let removed_count = removed.len() as u64;
        drop(pages);
        let removed_pages: alloc::vec::Vec<_> = removed.values().cloned().collect();
        super::pmap::remove_mappings_for_pages(&removed_pages);
        drop(removed);
        if let Some(account) = &self.account {
            account.release(removed_count);
        }
        Ok(removed_count)
    }

    /// Removes one page from the object.
    pub fn remove_page(&self, index: u64) -> Option<Arc<VmPage>> {
        let page = self.pages.lock().remove(&index);
        if let Some(page) = &page {
            super::pmap::remove_all_mappings(page);
        }
        if page.is_some()
            && let Some(account) = &self.account
        {
            account.release(1);
        }
        page
    }
}

impl Drop for VmObject {
    fn drop(&mut self) {
        let pages: alloc::vec::Vec<_> = self.pages.get_mut().values().cloned().collect();
        super::pmap::remove_mappings_for_pages(&pages);
        if let Some(account) = &self.account {
            account.release(self.pages.get_mut().len() as u64);
        }
    }
}
