//! Compressed front-swap and optional external swap backend.

use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};

use lz4_flex::block::{compress_into, decompress_into};

use crate::{
    mem::PAGE_SIZE,
    sys::sync::{Mutex, Once},
};

const PAGE_BYTES: usize = PAGE_SIZE as usize;
const MAX_COMPRESSED_PAGE_BYTES: usize = PAGE_BYTES + PAGE_BYTES / 255 + 16;

// The kernel allocator serves requests above half a page from whole pages.
// Rejecting larger compressed payloads guarantees that front-swap frees more
// physical memory than its payload allocation consumes.
const MAX_USEFUL_COMPRESSED_BYTES: usize = PAGE_BYTES / 2;

/// Opaque slot allocated by an external swap backend.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ExternalSwapSlot(pub u64);

/// Storage location for one evicted page.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SwapHandle {
    /// A page containing one repeated 64-bit value.
    Fill(u64),
    /// LZ4-compressed data held in kernel memory.
    Compressed(u64),
    /// Data held by a registered external backend.
    External(ExternalSwapSlot),
}

/// Failure returned by a swap backend.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SwapError {
    /// The backend has no capacity for another page.
    Full,
    /// The page is not useful to store in the compressed tier.
    Incompressible,
    /// The slot does not exist or its contents are invalid.
    Corrupt,
    /// A low-level backend I/O operation failed.
    Io,
}

/// Disk or device swap implementation registered beneath compressed swap.
pub trait SwapBackend: Send + Sync {
    /// Stores one complete page and returns its backend slot.
    fn store(&self, page: &[u8; PAGE_BYTES]) -> core::result::Result<ExternalSwapSlot, SwapError>;

    /// Restores one complete page from `slot`.
    fn load(
        &self,
        slot: ExternalSwapSlot,
        page: &mut [u8; PAGE_BYTES],
    ) -> core::result::Result<(), SwapError>;

    /// Releases a slot that is no longer referenced.
    fn free(&self, slot: ExternalSwapSlot);
}

struct CompressedSlot {
    /// Reference counted so a page-in can decompress without holding the lock.
    data: Option<Arc<[u8]>>,
    checksum: u64,
}

struct SwapState {
    /// Grown on demand; a full table would otherwise be allocated at boot for
    /// capacity that most systems never use.
    compressed: Vec<CompressedSlot>,
    free_slots: Vec<u32>,
    maximum_slots: usize,
    backend: Option<Arc<dyn SwapBackend>>,
    external_ops: usize,
}

impl SwapState {
    /// Reserves a slot, extending the table only when none is recycled.
    ///
    /// Room to recycle the slot is reserved at the same time. Releasing a slot
    /// therefore never has to allocate, which matters because releases happen
    /// under memory pressure and a failed release would shrink swap capacity
    /// exactly when it is needed most.
    fn take_slot(&mut self) -> Option<u32> {
        if let Some(slot) = self.free_slots.pop() {
            return Some(slot);
        }
        if self.compressed.len() >= self.maximum_slots {
            return None;
        }
        self.compressed.try_reserve(1).ok()?;
        self.free_slots.try_reserve(1).ok()?;
        self.compressed.push(CompressedSlot {
            data: None,
            checksum: 0,
        });
        Some((self.compressed.len() - 1) as u32)
    }
}

struct SwapManager {
    maximum_compressed_bytes: u64,
    compressed_bytes: AtomicU64,
    compressed_pages: AtomicU64,
    same_fill_pages: AtomicU64,
    external_pages: AtomicU64,
    pageins: AtomicU64,
    pageouts: AtomicU64,
    rejected_pages: AtomicU64,
    state: Mutex<SwapState>,
}

/// Snapshot of swap usage and traffic.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct SwapStats {
    /// Maximum compressed payload bytes.
    pub maximum_compressed_bytes: u64,
    /// Bytes currently held by compressed swap.
    pub compressed_bytes: u64,
    /// Pages currently held by compressed swap.
    pub compressed_pages: u64,
    /// Pages represented by one repeated 64-bit value.
    pub same_fill_pages: u64,
    /// Pages currently held by the external backend.
    pub external_pages: u64,
    /// Successful page-ins.
    pub pageins: u64,
    /// Successful page-outs.
    pub pageouts: u64,
    /// Pages rejected by the compressed tier.
    pub rejected_pages: u64,
}

static SWAP: Once<SwapManager> = Once::new();

/// Initializes compressed swap with a payload-byte limit.
pub(super) fn init(maximum_compressed_bytes: u64, maximum_slots: usize) {
    SWAP.call_once(|| {
        let maximum_slots = maximum_slots.clamp(64, u32::MAX as usize);
        SwapManager {
            maximum_compressed_bytes,
            compressed_bytes: AtomicU64::new(0),
            compressed_pages: AtomicU64::new(0),
            same_fill_pages: AtomicU64::new(0),
            external_pages: AtomicU64::new(0),
            pageins: AtomicU64::new(0),
            pageouts: AtomicU64::new(0),
            rejected_pages: AtomicU64::new(0),
            state: Mutex::new(SwapState {
                compressed: Vec::new(),
                free_slots: Vec::new(),
                maximum_slots,
                backend: None,
                external_ops: 0,
            }),
        }
    });
}

/// Registers the external fallback used for incompressible pages.
pub fn register_backend(backend: Arc<dyn SwapBackend>) -> core::result::Result<(), SwapError> {
    let manager = manager();
    let mut state = manager.state.lock();
    if state.backend.is_some() {
        return Err(SwapError::Io);
    }
    state.backend = Some(backend);
    Ok(())
}

/// Removes the external backend after all of its slots have drained.
pub fn unregister_backend() -> core::result::Result<Option<Arc<dyn SwapBackend>>, SwapError> {
    let manager = manager();
    let mut state = manager.state.lock();
    if manager.external_pages.load(Ordering::Acquire) != 0 || state.external_ops != 0 {
        return Err(SwapError::Full);
    }
    Ok(state.backend.take())
}

pub(super) fn store(page: &[u8; PAGE_BYTES]) -> core::result::Result<SwapHandle, SwapError> {
    let manager = manager();
    let fill = u64::from_ne_bytes(page[..8].try_into().expect("mem/swap: page prefix"));
    if page
        .as_chunks::<8>()
        .0
        .iter()
        .all(|word| u64::from_ne_bytes(*word) == fill)
    {
        manager.same_fill_pages.fetch_add(1, Ordering::AcqRel);
        manager.pageouts.fetch_add(1, Ordering::Relaxed);
        return Ok(SwapHandle::Fill(fill));
    }

    let mut output = [0u8; MAX_COMPRESSED_PAGE_BYTES];
    let compressed_len = compress_into(page, &mut output).map_err(|_| SwapError::Incompressible)?;

    if compressed_len <= MAX_USEFUL_COMPRESSED_BYTES {
        let mut payload = Vec::new();
        payload
            .try_reserve_exact(compressed_len)
            .map_err(|_| SwapError::Full)?;
        payload.extend_from_slice(&output[..compressed_len]);
        let checksum = page_checksum(page);
        let payload: Arc<[u8]> = Arc::from(payload.into_boxed_slice());
        let mut state = manager.state.lock();
        let current = manager.compressed_bytes.load(Ordering::Acquire);
        let next = current
            .checked_add(compressed_len as u64)
            .ok_or(SwapError::Full)?;

        if next <= manager.maximum_compressed_bytes
            && let Some(slot) = state.take_slot()
        {
            let entry = &mut state.compressed[slot as usize];
            debug_assert!(entry.data.is_none());
            entry.data = Some(payload);
            entry.checksum = checksum;
            manager.compressed_bytes.store(next, Ordering::Release);
            manager.compressed_pages.fetch_add(1, Ordering::AcqRel);
            manager.pageouts.fetch_add(1, Ordering::Relaxed);
            return Ok(SwapHandle::Compressed(u64::from(slot)));
        }
    }

    manager.rejected_pages.fetch_add(1, Ordering::Relaxed);
    let backend = {
        let mut state = manager.state.lock();
        let Some(backend) = state.backend.clone() else {
            return Err(SwapError::Incompressible);
        };
        state.external_ops += 1;
        backend
    };
    let result = backend.store(page);
    {
        let mut state = manager.state.lock();
        state.external_ops -= 1;
        if result.is_ok() {
            manager.external_pages.fetch_add(1, Ordering::AcqRel);
        }
    }
    let slot = result?;
    manager.pageouts.fetch_add(1, Ordering::Relaxed);
    Ok(SwapHandle::External(slot))
}

pub(super) fn load(
    handle: SwapHandle,
    page: &mut [u8; PAGE_BYTES],
) -> core::result::Result<(), SwapError> {
    let manager = manager();
    match handle {
        SwapHandle::Fill(fill) => {
            for word in page.as_chunks_mut::<8>().0 {
                *word = fill.to_ne_bytes();
            }
        }
        SwapHandle::Compressed(slot) => {
            // Decompression is the expensive part of a page-in, so the payload
            // is claimed by reference and expanded with the lock released.
            let (data, checksum) = {
                let state = manager.state.lock();
                let entry = state
                    .compressed
                    .get(slot as usize)
                    .ok_or(SwapError::Corrupt)?;
                let data = entry.data.clone().ok_or(SwapError::Corrupt)?;
                (data, entry.checksum)
            };
            let written = decompress_into(&data, page).map_err(|_| SwapError::Corrupt)?;
            if written != PAGE_BYTES || page_checksum(page) != checksum {
                return Err(SwapError::Corrupt);
            }
        }
        SwapHandle::External(slot) => {
            let backend = manager.state.lock().backend.clone().ok_or(SwapError::Io)?;
            backend.load(slot, page)?;
        }
    }
    manager.pageins.fetch_add(1, Ordering::Relaxed);
    Ok(())
}

pub(super) fn free(handle: SwapHandle) {
    let manager = manager();
    match handle {
        SwapHandle::Fill(_) => {
            manager.same_fill_pages.fetch_sub(1, Ordering::AcqRel);
        }
        SwapHandle::Compressed(slot) => {
            let mut state = manager.state.lock();
            let Some(entry) = state.compressed.get_mut(slot as usize) else {
                return;
            };
            if let Some(data) = entry.data.take() {
                entry.checksum = 0;
                let length = data.len() as u64;
                drop(data);
                manager.compressed_bytes.fetch_sub(length, Ordering::AcqRel);
                manager.compressed_pages.fetch_sub(1, Ordering::AcqRel);
                // Capacity was reserved when the slot was created, so this
                // cannot allocate and the slot can never be stranded.
                debug_assert!(state.free_slots.len() < state.free_slots.capacity());
                state.free_slots.push(slot as u32);
            }
        }
        SwapHandle::External(slot) => {
            let backend = {
                let mut state = manager.state.lock();
                let Some(backend) = state.backend.clone() else {
                    return;
                };
                state.external_ops += 1;
                backend
            };
            backend.free(slot);
            let mut state = manager.state.lock();
            state.external_ops -= 1;
            manager.external_pages.fetch_sub(1, Ordering::AcqRel);
        }
    }
}

/// Returns current swap capacity and traffic counters.
pub fn stats() -> SwapStats {
    let manager = manager();
    SwapStats {
        maximum_compressed_bytes: manager.maximum_compressed_bytes,
        compressed_bytes: manager.compressed_bytes.load(Ordering::Acquire),
        compressed_pages: manager.compressed_pages.load(Ordering::Acquire),
        same_fill_pages: manager.same_fill_pages.load(Ordering::Acquire),
        external_pages: manager.external_pages.load(Ordering::Acquire),
        pageins: manager.pageins.load(Ordering::Relaxed),
        pageouts: manager.pageouts.load(Ordering::Relaxed),
        rejected_pages: manager.rejected_pages.load(Ordering::Relaxed),
    }
}

fn manager() -> &'static SwapManager {
    SWAP.get().expect("mem/swap: initialized before use")
}

/// Returns an integrity digest over one page.
///
/// The digest runs on every page-out and page-in, so it consumes eight bytes
/// per round rather than one; a byte-at-a-time hash would cost more than the
/// compression it protects.
fn page_checksum(page: &[u8; PAGE_BYTES]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for word in page.as_chunks::<8>().0 {
        hash ^= u64::from_ne_bytes(*word);
        hash = hash.wrapping_mul(0x100_0000_01b3);
        hash ^= hash >> 29;
    }
    hash
}
