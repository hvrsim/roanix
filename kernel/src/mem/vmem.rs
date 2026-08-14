//!
//! # VMem Resource Allocator
//!
//! General-purpose resource allocator following Bonwick and Adams' vmem
//! design. It allocates arbitrary integer resources — virtual addresses here —
//! in constant time regardless of arena size or fragmentation.
//!
//! An arena describes the same segments through three views:
//!
//! * an address-ordered chain used for coalescing,
//! * per-power-of-two free lists giving constant-time "instant fit", and
//! * a hash table of allocated segments keyed by base, so freeing needs only
//!   an address and never a scan.
//!
//! A sorted index additionally resolves "which segment contains this address"
//! in logarithmic time, which fixed-placement requests need.
//!
//! Segments live in a densely packed vector and are referenced by 32-bit
//! index, avoiding one heap allocation per segment and keeping every link
//! four bytes wide.
//!

use alloc::{collections::BTreeMap, vec::Vec};

/// Sentinel index representing the absence of a segment.
const NIL: u32 = u32::MAX;

/// One free list per power of two, covering the full 64-bit resource space.
const FREELIST_COUNT: usize = 64;

/// Initial number of buckets in the allocated-segment hash table.
const INITIAL_HASH_BUCKETS: usize = 64;

/// Average allocated segments per bucket before the hash table grows.
const HASH_LOAD_FACTOR: usize = 2;

/// Allocation placement policy.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VmemFit {
    /// Constant-time allocation from the first segment guaranteed to fit.
    Instant,
    /// Smallest suitable segment, minimizing fragmentation at higher cost.
    Best,
}

/// Reason an allocation could not be satisfied.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VmemError {
    /// No free segment satisfies the request.
    NoSpace,
    /// The request itself is malformed.
    Invalid,
}

/// Result type produced by arena operations.
pub type Result<T> = core::result::Result<T, VmemError>;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum SegmentKind {
    /// Boundary marker describing an imported resource range.
    Span,
    /// Available resource.
    Free,
    /// Resource handed out to a caller.
    Allocated,
}

#[derive(Copy, Clone)]
struct Segment {
    base: u64,
    size: u64,
    kind: SegmentKind,
    /// Address-ordered neighbours.
    prev: u32,
    next: u32,
    /// Free-list or hash-chain neighbours.
    link_prev: u32,
    link_next: u32,
}

impl Segment {
    const EMPTY: Self = Self {
        base: 0,
        size: 0,
        kind: SegmentKind::Span,
        prev: NIL,
        next: NIL,
        link_prev: NIL,
        link_next: NIL,
    };

    fn end(&self) -> u64 {
        self.base.saturating_add(self.size)
    }
}

/// Constant-time allocator for a contiguous integer resource.
pub struct Vmem {
    segments: Vec<Segment>,
    recycled: Vec<u32>,
    /// Head and tail of the address-ordered chain.
    head: u32,
    tail: u32,
    freelists: [u32; FREELIST_COUNT],
    /// Bitmap of non-empty free lists, reducing instant fit to a bit scan.
    freelist_map: u64,
    /// Base-ordered index over every non-span segment.
    by_base: BTreeMap<u64, u32>,
    hash: Vec<u32>,
    allocated_count: usize,
    /// Smallest allocation unit; every base and size is a multiple of it.
    quantum: u64,
    total: u64,
    used: u64,
}

impl Vmem {
    /// Creates an empty arena whose allocations are multiples of `quantum`.
    ///
    /// # Panics
    ///
    /// Panics if `quantum` is not a power of two.
    pub fn new(quantum: u64) -> Self {
        assert!(
            quantum.is_power_of_two(),
            "mem/vmem: quantum must be a power of two"
        );
        Self {
            segments: Vec::new(),
            recycled: Vec::new(),
            head: NIL,
            tail: NIL,
            freelists: [NIL; FREELIST_COUNT],
            freelist_map: 0,
            by_base: BTreeMap::new(),
            hash: Vec::new(),
            allocated_count: 0,
            quantum,
            total: 0,
            used: 0,
        }
    }

    /// Returns the total resource added to the arena.
    pub fn total(&self) -> u64 {
        self.total
    }

    /// Returns the resource currently allocated.
    pub fn used(&self) -> u64 {
        self.used
    }

    /// Returns the resource currently available.
    pub fn available(&self) -> u64 {
        self.total - self.used
    }

    /// Adds `[base, base + size)` to the arena.
    ///
    /// Spans must not overlap resource already present in the arena.
    pub fn add_span(&mut self, base: u64, size: u64) -> Result<()> {
        if size == 0
            || !base.is_multiple_of(self.quantum)
            || !size.is_multiple_of(self.quantum)
            || base.checked_add(size).is_none()
        {
            return Err(VmemError::Invalid);
        }

        let after = self.tail;
        let span = self.new_segment(base, size, SegmentKind::Span);
        self.link_after(after, span);
        let free = self.new_segment(base, size, SegmentKind::Free);
        self.link_after(span, free);
        self.push_free(free);
        self.by_base.insert(base, free);
        self.total += size;
        Ok(())
    }

    /// Allocates `size` resource honouring `align`.
    pub fn alloc(&mut self, size: u64, align: u64, fit: VmemFit) -> Result<u64> {
        self.xalloc(size, align, 0, u64::MAX, fit)
    }

    /// Allocates `size` resource within `[min, max)` honouring `align`.
    ///
    /// Unconstrained instant-fit requests inspect a single free list and are
    /// therefore constant time. Bounded, over-aligned, or best-fit requests
    /// walk the free lists that can hold a suitable segment.
    pub fn xalloc(
        &mut self,
        size: u64,
        align: u64,
        min: u64,
        max: u64,
        fit: VmemFit,
    ) -> Result<u64> {
        let size = self.round_size(size)?;
        if align == 0 || !align.is_power_of_two() || min >= max {
            return Err(VmemError::Invalid);
        }
        let align = align.max(self.quantum);

        let unconstrained = min == 0 && max == u64::MAX && align == self.quantum;
        let found = if unconstrained && fit == VmemFit::Instant {
            self.instant_fit(size)
        } else {
            self.scan_fit(size, align, min, max, fit == VmemFit::Best)
        };

        let (segment, base) = found.ok_or(VmemError::NoSpace)?;
        self.take(segment, base, size);
        Ok(base)
    }

    /// Allocates `size` resource at a randomized base honouring `align`.
    ///
    /// The offset is drawn from the slack of the selected free segment, which
    /// spreads placements across the arena while remaining constant time.
    pub fn alloc_random(&mut self, size: u64, align: u64, entropy: u64) -> Result<u64> {
        let size = self.round_size(size)?;
        if align == 0 || !align.is_power_of_two() {
            return Err(VmemError::Invalid);
        }
        let align = align.max(self.quantum);
        let (segment, segment_base) = self.instant_fit(size).ok_or(VmemError::NoSpace)?;

        let start = segment_base
            .checked_add(align - 1)
            .ok_or(VmemError::NoSpace)?
            & !(align - 1);
        let end = self.segments[segment as usize].end();
        let slack = end
            .saturating_sub(start)
            .checked_sub(size)
            .ok_or(VmemError::NoSpace)?;
        let base = start + (entropy % (slack / align + 1)) * align;

        self.take(segment, base, size);
        Ok(base)
    }

    /// Allocates exactly `[base, base + size)`.
    pub fn alloc_fixed(&mut self, base: u64, size: u64) -> Result<()> {
        let size = self.round_size(size)?;
        if !base.is_multiple_of(self.quantum) {
            return Err(VmemError::Invalid);
        }
        let end = base.checked_add(size).ok_or(VmemError::Invalid)?;
        let segment = self
            .free_segment_containing(base)
            .filter(|index| self.segments[*index as usize].end() >= end)
            .ok_or(VmemError::NoSpace)?;
        self.take(segment, base, size);
        Ok(())
    }

    /// Returns whether `[base, base + size)` is entirely free.
    pub fn is_free(&self, base: u64, size: u64) -> bool {
        let Some(end) = base.checked_add(size) else {
            return false;
        };
        self.free_segment_containing(base)
            .is_some_and(|index| self.segments[index as usize].end() >= end)
    }

    /// Releases a previously allocated `base` and returns its size.
    ///
    /// # Panics
    ///
    /// Panics if `base` was not returned by this arena.
    pub fn free(&mut self, base: u64) -> u64 {
        let index = self
            .hash_remove(base)
            .expect("mem/vmem: freeing an address this arena never allocated");
        let size = self.segments[index as usize].size;
        self.used -= size;
        self.allocated_count -= 1;
        self.segments[index as usize].kind = SegmentKind::Free;
        self.coalesce(index);
        size
    }

    /// Constant-time selection of a segment guaranteed to fit `size`.
    fn instant_fit(&self, size: u64) -> Option<(u32, u64)> {
        // Free list `n` holds segments sized `[2^n, 2^(n+1))`, so every member
        // of the list whose lower bound reaches `size` is large enough.
        let guaranteed = guaranteed_freelist(size);
        let mask = if guaranteed >= FREELIST_COUNT {
            0
        } else {
            !0u64 << guaranteed
        };
        let candidates = self.freelist_map & mask;
        if candidates != 0 {
            let index = self.freelists[candidates.trailing_zeros() as usize];
            return Some((index, self.segments[index as usize].base));
        }

        // Only the list straddling `size` can still hold a fitting segment.
        let straddling = freelist_index(size);
        if straddling == guaranteed || self.freelist_map & (1u64 << straddling) == 0 {
            return None;
        }
        let mut index = self.freelists[straddling];
        while index != NIL {
            if self.segments[index as usize].size >= size {
                return Some((index, self.segments[index as usize].base));
            }
            index = self.segments[index as usize].link_next;
        }
        None
    }

    /// Free-list walk honouring alignment and bounds.
    fn scan_fit(
        &self,
        size: u64,
        align: u64,
        min: u64,
        max: u64,
        best: bool,
    ) -> Option<(u32, u64)> {
        let mut chosen: Option<(u32, u64, u64)> = None;
        for list in freelist_index(size)..FREELIST_COUNT {
            if self.freelist_map & (1u64 << list) == 0 {
                continue;
            }
            let mut index = self.freelists[list];
            while index != NIL {
                let segment = &self.segments[index as usize];
                if let Some(base) = placement(segment, size, align, min, max) {
                    if !best {
                        return Some((index, base));
                    }
                    if chosen.is_none_or(|(_, _, chosen_size)| segment.size < chosen_size) {
                        chosen = Some((index, base, segment.size));
                    }
                }
                index = segment.link_next;
            }
            // Every segment in a higher list is strictly larger, so the first
            // list yielding a candidate already holds the best fit.
            if let Some((index, base, _)) = chosen {
                return Some((index, base));
            }
        }
        None
    }

    /// Converts free segment `index` into an allocation of `size` at `base`,
    /// returning any leading and trailing remainder to the arena.
    fn take(&mut self, index: u32, base: u64, size: u64) {
        self.pop_free(index);
        let original = self.segments[index as usize];
        debug_assert!(base >= original.base && base + size <= original.end());

        if base > original.base {
            let leading = self.new_segment(original.base, base - original.base, SegmentKind::Free);
            self.link_before(index, leading);
            self.push_free(leading);
            self.by_base.insert(original.base, leading);
            self.segments[index as usize].base = base;
            self.segments[index as usize].size = original.end() - base;
            self.by_base.insert(base, index);
        }

        let remainder = self.segments[index as usize].size - size;
        if remainder > 0 {
            let trailing = self.new_segment(base + size, remainder, SegmentKind::Free);
            self.link_after(index, trailing);
            self.push_free(trailing);
            self.by_base.insert(base + size, trailing);
            self.segments[index as usize].size = size;
        }

        self.segments[index as usize].kind = SegmentKind::Allocated;
        self.hash_insert(index);
        self.used += size;
        self.allocated_count += 1;
    }

    /// Merges free segment `index` with any free neighbours.
    fn coalesce(&mut self, index: u32) {
        let next = self.segments[index as usize].next;
        if next != NIL && self.segments[next as usize].kind == SegmentKind::Free {
            self.pop_free(next);
            self.by_base.remove(&self.segments[next as usize].base);
            self.segments[index as usize].size += self.segments[next as usize].size;
            self.unlink(next);
        }

        let prev = self.segments[index as usize].prev;
        if prev != NIL && self.segments[prev as usize].kind == SegmentKind::Free {
            self.pop_free(prev);
            self.segments[prev as usize].size += self.segments[index as usize].size;
            self.by_base.remove(&self.segments[index as usize].base);
            self.unlink(index);
            self.push_free(prev);
            return;
        }

        self.push_free(index);
    }

    fn new_segment(&mut self, base: u64, size: u64, kind: SegmentKind) -> u32 {
        let segment = Segment {
            base,
            size,
            kind,
            ..Segment::EMPTY
        };
        if let Some(index) = self.recycled.pop() {
            self.segments[index as usize] = segment;
            return index;
        }
        let index = u32::try_from(self.segments.len()).expect("mem/vmem: segment table overflow");
        assert!(index != NIL, "mem/vmem: segment table overflow");
        self.segments.push(segment);
        index
    }

    fn free_segment_containing(&self, address: u64) -> Option<u32> {
        let (_, index) = self.by_base.range(..=address).next_back()?;
        let segment = &self.segments[*index as usize];
        (segment.kind == SegmentKind::Free && address < segment.end()).then_some(*index)
    }

    fn link_after(&mut self, after: u32, index: u32) {
        if after == NIL {
            let old_head = self.head;
            self.segments[index as usize].prev = NIL;
            self.segments[index as usize].next = old_head;
            if old_head != NIL {
                self.segments[old_head as usize].prev = index;
            } else {
                self.tail = index;
            }
            self.head = index;
            return;
        }

        let next = self.segments[after as usize].next;
        self.segments[index as usize].prev = after;
        self.segments[index as usize].next = next;
        self.segments[after as usize].next = index;
        if next != NIL {
            self.segments[next as usize].prev = index;
        } else {
            self.tail = index;
        }
    }

    fn link_before(&mut self, before: u32, index: u32) {
        let prev = self.segments[before as usize].prev;
        self.link_after(prev, index);
    }

    fn unlink(&mut self, index: u32) {
        let Segment { prev, next, .. } = self.segments[index as usize];
        if prev != NIL {
            self.segments[prev as usize].next = next;
        } else {
            self.head = next;
        }
        if next != NIL {
            self.segments[next as usize].prev = prev;
        } else {
            self.tail = prev;
        }
        self.segments[index as usize] = Segment::EMPTY;
        self.recycled.push(index);
    }

    fn push_free(&mut self, index: u32) {
        let list = freelist_index(self.segments[index as usize].size);
        let head = self.freelists[list];
        self.segments[index as usize].link_prev = NIL;
        self.segments[index as usize].link_next = head;
        if head != NIL {
            self.segments[head as usize].link_prev = index;
        }
        self.freelists[list] = index;
        self.freelist_map |= 1u64 << list;
    }

    fn pop_free(&mut self, index: u32) {
        let Segment {
            link_prev,
            link_next,
            size,
            ..
        } = self.segments[index as usize];
        let list = freelist_index(size);
        if link_prev != NIL {
            self.segments[link_prev as usize].link_next = link_next;
        } else {
            self.freelists[list] = link_next;
            if link_next == NIL {
                self.freelist_map &= !(1u64 << list);
            }
        }
        if link_next != NIL {
            self.segments[link_next as usize].link_prev = link_prev;
        }
        self.segments[index as usize].link_prev = NIL;
        self.segments[index as usize].link_next = NIL;
    }

    fn hash_insert(&mut self, index: u32) {
        if self.hash.is_empty() {
            self.hash = alloc::vec![NIL; INITIAL_HASH_BUCKETS];
        } else if self.allocated_count >= self.hash.len() * HASH_LOAD_FACTOR {
            self.rehash();
        }

        let bucket = self.bucket_of(self.segments[index as usize].base);
        let head = self.hash[bucket];
        self.segments[index as usize].link_prev = NIL;
        self.segments[index as usize].link_next = head;
        if head != NIL {
            self.segments[head as usize].link_prev = index;
        }
        self.hash[bucket] = index;
    }

    fn hash_remove(&mut self, base: u64) -> Option<u32> {
        if self.hash.is_empty() {
            return None;
        }
        let bucket = self.bucket_of(base);
        let mut index = self.hash[bucket];
        while index != NIL {
            let segment = self.segments[index as usize];
            if segment.base != base {
                index = segment.link_next;
                continue;
            }
            if segment.link_prev != NIL {
                self.segments[segment.link_prev as usize].link_next = segment.link_next;
            } else {
                self.hash[bucket] = segment.link_next;
            }
            if segment.link_next != NIL {
                self.segments[segment.link_next as usize].link_prev = segment.link_prev;
            }
            self.segments[index as usize].link_prev = NIL;
            self.segments[index as usize].link_next = NIL;
            return Some(index);
        }
        None
    }

    fn rehash(&mut self) {
        let buckets = self.hash.len() * 2;
        let mut fresh = alloc::vec![NIL; buckets];
        for bucket in 0..self.hash.len() {
            let mut index = self.hash[bucket];
            while index != NIL {
                let next = self.segments[index as usize].link_next;
                let target = bucket_index(self.segments[index as usize].base, self.quantum, buckets);
                let head = fresh[target];
                self.segments[index as usize].link_prev = NIL;
                self.segments[index as usize].link_next = head;
                if head != NIL {
                    self.segments[head as usize].link_prev = index;
                }
                fresh[target] = index;
                index = next;
            }
        }
        self.hash = fresh;
    }

    fn bucket_of(&self, base: u64) -> usize {
        bucket_index(base, self.quantum, self.hash.len())
    }

    fn round_size(&self, size: u64) -> Result<u64> {
        if size == 0 {
            return Err(VmemError::Invalid);
        }
        size.checked_add(self.quantum - 1)
            .map(|value| value & !(self.quantum - 1))
            .ok_or(VmemError::Invalid)
    }
}

/// Returns the lowest base within `segment` satisfying every constraint.
fn placement(segment: &Segment, size: u64, align: u64, min: u64, max: u64) -> Option<u64> {
    if segment.kind != SegmentKind::Free {
        return None;
    }
    let start = segment.base.max(min);
    let base = start.checked_add(align - 1)? & !(align - 1);
    let end = base.checked_add(size)?;
    (end <= segment.end() && end <= max).then_some(base)
}

/// Returns the free list holding segments of `size`.
#[inline]
fn freelist_index(size: u64) -> usize {
    (63 - size.max(1).leading_zeros()) as usize
}

/// Returns the lowest free list whose every member satisfies `size`.
#[inline]
fn guaranteed_freelist(size: u64) -> usize {
    let floor = freelist_index(size);
    if size.is_power_of_two() { floor } else { floor + 1 }
}

#[inline]
fn bucket_index(base: u64, quantum: u64, buckets: usize) -> usize {
    // Bases are quantum-aligned, so the low bits carry no information.
    let mixed = (base / quantum).wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 24;
    (mixed & (buckets as u64 - 1)) as usize
}
