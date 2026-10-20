//! Nonblocking allocation classes for small memory-management metadata.
//!
//! The allocator selects fixed, cache-line-aligned storage classes backed by
//! typed pools. Unsupported sizes or alignments fail instead of entering the
//! general heap or waiting for memory. Allocation and release use the same class,
//! and draining returns unused pool storage to physical memory.

use core::{
    alloc::{AllocError, Allocator, Layout},
    mem::MaybeUninit,
    ptr::NonNull,
};
use intrusive_collections::PointerOps;

use super::pool::Pool;

#[repr(align(64))]
struct Block<const N: usize> {
    _bytes: [MaybeUninit<u8>; N],
}

trait Storage: Sync {
    fn allocate(&'static self) -> Result<NonNull<u8>, AllocError>;
    unsafe fn release(&'static self, pointer: NonNull<u8>);
    fn drain(&self);
}

impl<const N: usize> Storage for Pool<Block<N>> {
    fn allocate(&'static self) -> Result<NonNull<u8>, AllocError> {
        let block = self
            .try_alloc(Block {
                _bytes: [MaybeUninit::uninit(); N],
            })
            .map_err(|_| AllocError)?;
        // SAFETY: an owning pool handle always contains a non-null slot.
        Ok(unsafe { NonNull::new_unchecked(self.into_raw(block).cast_mut().cast()) })
    }

    unsafe fn release(&'static self, pointer: NonNull<u8>) {
        // SAFETY: the allocator returns the same pool and base pointer selected
        // for this layout; the raw handle is reconstructed exactly once.
        drop(unsafe { self.from_raw(pointer.as_ptr().cast()) });
    }

    fn drain(&self) {
        Pool::drain(self);
    }
}

static BLOCK_64: Pool<Block<64>> = Pool::new();
static BLOCK_128: Pool<Block<128>> = Pool::new();
static BLOCK_256: Pool<Block<256>> = Pool::new();
static BLOCK_512: Pool<Block<512>> = Pool::new();
static BLOCK_1024: Pool<Block<1024>> = Pool::new();
static BLOCK_2048: Pool<Block<2048>> = Pool::new();
static BLOCK_3072: Pool<Block<3072>> = Pool::new();
static POOLS: [&dyn Storage; 7] = [
    &BLOCK_64,
    &BLOCK_128,
    &BLOCK_256,
    &BLOCK_512,
    &BLOCK_1024,
    &BLOCK_2048,
    &BLOCK_3072,
];

/// Nonblocking allocation policy for small VM metadata. Unsupported layouts
/// fail rather than entering the general heap or waiting for memory.
#[derive(Clone, Copy)]
pub struct NoSleep;

impl NoSleep {
    pub(super) fn size_class(bytes: usize) -> Option<usize> {
        [64, 128, 256, 512, 1024, 2048, 3072]
            .into_iter()
            .find(|&size| bytes <= size)
    }
    fn storage(layout: Layout) -> Result<&'static dyn Storage, AllocError> {
        if layout.align() > 64 {
            return Err(AllocError);
        }
        let class = match layout.size() {
            0..=64 => 0,
            65..=128 => 1,
            129..=256 => 2,
            257..=512 => 3,
            513..=1024 => 4,
            1025..=2048 => 5,
            2049..=3072 => 6,
            _ => return Err(AllocError),
        };
        Ok(POOLS[class])
    }
}

// SAFETY: each layout selects one immutable pool. Its storage satisfies the
// requested size/alignment and stays allocated until returned through that
// same policy. Cloning the zero-sized allocator preserves pool identity.
unsafe impl Allocator for NoSleep {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        let pointer = Self::storage(layout)?.allocate()?;
        Ok(NonNull::slice_from_raw_parts(pointer, layout.size()))
    }

    unsafe fn deallocate(&self, pointer: NonNull<u8>, layout: Layout) {
        // SAFETY: Allocator requires a live allocation and its original layout.
        unsafe {
            Self::storage(layout)
                .expect("mem: invalid metadata layout")
                .release(pointer)
        };
    }
}

pub(super) fn drain() {
    for pool in POOLS {
        pool.drain();
    }
}
