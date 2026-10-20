//! Virtual-address arena allocation using boundary tags and size classes.
//!
//! The arena tracks free and allocated ranges without supplying their physical
//! backing. It supports fixed, aligned, best-fit, and randomized placement, splits
//! and coalesces adjacent ranges, and reserves tag storage before committing
//! changes so metadata allocation failure leaves the arena unchanged.

use core::{cell::Cell, ops::Bound, ptr::NonNull};

use super::{
    index::{Index, Node},
    pool::{Pool, Pooled},
};

const FREELIST_COUNT: usize = 64;
static TAG_POOL: Pool<Node<Segment>> = Pool::new();

type Link = Option<NonNull<Segment>>;

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
    /// Boundary-tag storage is unavailable without waiting.
    NoMemory,
    /// The request itself is malformed.
    Invalid,
}

/// Result type produced by arena operations.
pub type Result<T> = core::result::Result<T, VmemError>;

struct Segment {
    base: u64,
    size: Cell<u64>,
    span: u64,
    free: Cell<bool>,
    previous: Cell<Link>,
    next: Cell<Link>,
}

impl Segment {
    fn new(base: u64, size: u64, span: u64, free: bool) -> Self {
        Self {
            base,
            size: Cell::new(size),
            span,
            free: Cell::new(free),
            previous: Cell::new(None),
            next: Cell::new(None),
        }
    }

    fn end(&self) -> u64 {
        self.base + self.size.get()
    }
}

/// Metadata reserved before publishing a fixed-placement map edit.
pub(super) struct Reservation {
    base: u64,
    size: u64,
    start: Option<Pooled<Node<Segment>>>,
    end: Option<Pooled<Node<Segment>>>,
}

/// Address-ordered boundary tags with segregated free lists. All metadata
/// comes from a nonblocking pool; returning space never allocates.
pub struct Vmem {
    segments: Index<Segment>,
    freelists: [Link; FREELIST_COUNT],
    freelist_map: u64,
    quantum: u64,
    total: u64,
    used: u64,
}

// SAFETY: the arena owns every linked tag. Moving it keeps pool addresses
// stable; all tag/link mutation requires exclusive access to the arena.
unsafe impl Send for Vmem {}
// SAFETY: shared operations only read tags. Cells are mutated exclusively by
// methods taking &mut Vmem, and no tag reference escapes this module.
unsafe impl Sync for Vmem {}

pub(super) fn drain_pool() {
    TAG_POOL.drain();
}

impl Vmem {
    pub fn new(quantum: u64) -> Self {
        assert!(quantum.is_power_of_two(), "mem/vmem: invalid quantum");
        Self {
            segments: Index::new(&TAG_POOL),
            freelists: [None; FREELIST_COUNT],
            freelist_map: 0,
            quantum,
            total: 0,
            used: 0,
        }
    }

    pub fn total(&self) -> u64 {
        self.total
    }
    pub fn used(&self) -> u64 {
        self.used
    }
    pub fn available(&self) -> u64 {
        self.total() - self.used()
    }

    pub fn add_span(&mut self, base: u64, size: u64) -> Result<()> {
        let end = base.checked_add(size).ok_or(VmemError::Invalid)?;
        let total = self.total.checked_add(size).ok_or(VmemError::Invalid)?;
        if size == 0
            || !base.is_multiple_of(self.quantum)
            || !size.is_multiple_of(self.quantum)
            || self
                .segments
                .range(..end)
                .next_back()
                .is_some_and(|(_, tag)| tag.end() > base)
        {
            return Err(VmemError::Invalid);
        }
        let tag = self.reserve_tag(base)?;
        self.segments
            .insert(base, Segment::new(base, size, base, true), Some(tag));
        self.push_free(base);
        self.total = total;
        Ok(())
    }

    pub fn alloc(&mut self, size: u64, align: u64, fit: VmemFit) -> Result<u64> {
        self.xalloc(size, align, 0, u64::MAX, fit)
    }

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
        let found =
            if min == 0 && max == u64::MAX && align == self.quantum && fit == VmemFit::Instant {
                self.instant_fit(size)
            } else {
                self.scan_fit(size, align, min, max, fit == VmemFit::Best)
            };
        let (segment, base) = found.ok_or(VmemError::NoSpace)?;
        let reservation = self.prepare_take(segment, base, size)?;
        self.take(segment, reservation);
        Ok(base)
    }

    pub fn alloc_random(&mut self, size: u64, align: u64, entropy: u64) -> Result<u64> {
        let size = self.round_size(size)?;
        if align == 0 || !align.is_power_of_two() {
            return Err(VmemError::Invalid);
        }
        let align = align.max(self.quantum);
        let (segment, start) = if align == self.quantum {
            self.instant_fit(size)
        } else {
            self.scan_fit(size, align, 0, u64::MAX, false)
        }
        .ok_or(VmemError::NoSpace)?;
        let slack = self.tag(segment).end() - start - size;
        let base = start + (entropy % (slack / align + 1)) * align;
        let reservation = self.prepare_take(segment, base, size)?;
        self.take(segment, reservation);
        Ok(base)
    }

    pub fn alloc_fixed(&mut self, base: u64, size: u64) -> Result<()> {
        let size = self.fixed_size(base, size)?;
        let segment = self
            .free_segment_containing(base)
            .filter(|&key| self.tag(key).end() >= base + size)
            .ok_or(VmemError::NoSpace)?;
        let reservation = self.prepare_take(segment, base, size)?;
        self.take(segment, reservation);
        Ok(())
    }

    pub(super) fn reserve_fixed(&self, base: u64, size: u64) -> Result<Reservation> {
        let size = self.fixed_size(base, size)?;
        Ok(Reservation {
            base,
            size,
            start: Some(self.reserve_tag(base)?),
            end: Some(self.reserve_tag(base + size)?),
        })
    }

    pub(super) fn alloc_reserved(&mut self, reservation: Reservation) {
        let segment = self
            .free_segment_containing(reservation.base)
            .filter(|&key| self.tag(key).end() >= reservation.base + reservation.size)
            .expect("mem/vmem: reserved range is not free");
        self.take(segment, reservation);
    }

    pub fn is_free(&self, base: u64, size: u64) -> bool {
        base.checked_add(size).is_some_and(|end| {
            self.free_segment_containing(base)
                .is_some_and(|key| self.tag(key).end() >= end)
        })
    }

    pub fn free(&mut self, base: u64) -> u64 {
        let tag = self.tag(base);
        assert!(!tag.free.get(), "mem/vmem: double free");
        let size = tag.size.get();
        let span = tag.span;
        let next = self
            .segments
            .range((Bound::Excluded(base), Bound::Unbounded))
            .next()
            .filter(|(_, next)| next.free.get() && next.span == span && next.base == tag.end())
            .map(|(&key, _)| key);
        if let Some(next) = next {
            self.pop_free(next);
            self.tag(base)
                .size
                .set(self.tag(base).size.get() + self.tag(next).size.get());
            self.segments.remove(&next);
        }
        let previous = self
            .segments
            .range(..base)
            .next_back()
            .filter(|(_, prev)| prev.free.get() && prev.span == span && prev.end() == base)
            .map(|(&key, _)| key);
        let free = if let Some(previous) = previous {
            self.pop_free(previous);
            self.tag(previous)
                .size
                .set(self.tag(previous).size.get() + self.tag(base).size.get());
            self.segments.remove(&base);
            previous
        } else {
            self.tag(base).free.set(true);
            base
        };
        self.push_free(free);
        self.used -= size;
        size
    }

    fn tag(&self, base: u64) -> &Segment {
        self.segments
            .get(&base)
            .expect("mem/vmem: missing boundary tag")
    }

    fn reserve_tag(&self, base: u64) -> Result<Pooled<Node<Segment>>> {
        self.segments
            .reserve_new(base)
            .map_err(|_| VmemError::NoMemory)
    }

    fn prepare_take(&self, segment: u64, base: u64, size: u64) -> Result<Reservation> {
        let tag = self.tag(segment);
        Ok(Reservation {
            base,
            size,
            start: (base != segment)
                .then(|| self.reserve_tag(base))
                .transpose()?,
            end: (base + size != tag.end())
                .then(|| self.reserve_tag(base + size))
                .transpose()?,
        })
    }

    fn take(&mut self, segment: u64, reservation: Reservation) {
        let Reservation {
            base,
            size,
            start,
            end: trailing,
        } = reservation;
        let original = self.tag(segment);
        let end = original.end();
        let span = original.span;
        self.pop_free(segment);
        if segment < base {
            self.tag(segment).size.set(base - segment);
            self.push_free(segment);
            self.segments
                .insert(base, Segment::new(base, size, span, false), start);
        } else {
            self.tag(segment).size.set(size);
            self.tag(segment).free.set(false);
        }
        if base + size < end {
            self.segments.insert(
                base + size,
                Segment::new(base + size, end - base - size, span, true),
                trailing,
            );
            self.push_free(base + size);
        }
        self.used += size;
    }

    fn free_segment_containing(&self, address: u64) -> Option<u64> {
        self.segments
            .range(..=address)
            .next_back()
            .filter(|(_, tag)| tag.free.get() && address < tag.end())
            .map(|(&base, _)| base)
    }

    fn instant_fit(&self, size: u64) -> Option<(u64, u64)> {
        let guaranteed = guaranteed_freelist(size);
        let candidates = if guaranteed < FREELIST_COUNT {
            self.freelist_map & (!0 << guaranteed)
        } else {
            0
        };
        if candidates != 0 {
            let tag = self.link(self.freelists[candidates.trailing_zeros() as usize].unwrap());
            return Some((tag.base, tag.base));
        }
        let straddling = freelist_index(size);
        let mut next = self.freelists[straddling];
        while let Some(pointer) = next {
            let tag = self.link(pointer);
            if tag.size.get() >= size {
                return Some((tag.base, tag.base));
            }
            next = tag.next.get();
        }
        None
    }

    fn scan_fit(
        &self,
        size: u64,
        align: u64,
        min: u64,
        max: u64,
        best: bool,
    ) -> Option<(u64, u64)> {
        for list in freelist_index(size)..FREELIST_COUNT {
            let mut chosen: Option<(u64, u64, u64)> = None;
            let mut next = self.freelists[list];
            while let Some(pointer) = next {
                let tag = self.link(pointer);
                if let Some(base) = placement(tag, size, align, min, max) {
                    if !best {
                        return Some((tag.base, base));
                    }
                    if chosen.is_none_or(|(_, _, chosen_size)| tag.size.get() < chosen_size) {
                        chosen = Some((tag.base, base, tag.size.get()));
                    }
                }
                next = tag.next.get();
            }
            if let Some((segment, base, _)) = chosen {
                return Some((segment, base));
            }
        }
        None
    }

    fn link(&self, pointer: NonNull<Segment>) -> &Segment {
        // SAFETY: free links only name stable, initialized index payloads.
        // A tag is unlinked before removal, and links cannot escape the arena.
        unsafe { pointer.as_ref() }
    }

    fn push_free(&mut self, base: u64) {
        let tag = self.tag(base);
        let list = freelist_index(tag.size.get());
        let head = self.freelists[list];
        let pointer = NonNull::from(tag);
        tag.previous.set(None);
        tag.next.set(head);
        if let Some(head) = head {
            self.link(head).previous.set(Some(pointer));
        }
        self.freelists[list] = Some(pointer);
        self.freelist_map |= 1 << list;
    }

    fn pop_free(&mut self, base: u64) {
        let tag = self.tag(base);
        let list = freelist_index(tag.size.get());
        let previous = tag.previous.replace(None);
        let next = tag.next.replace(None);
        if let Some(previous) = previous {
            self.link(previous).next.set(next);
        } else {
            self.freelists[list] = next;
            if next.is_none() {
                self.freelist_map &= !(1 << list);
            }
        }
        if let Some(next) = next {
            self.link(next).previous.set(previous);
        }
    }

    fn fixed_size(&self, base: u64, size: u64) -> Result<u64> {
        let size = self.round_size(size)?;
        if !base.is_multiple_of(self.quantum) || base.checked_add(size).is_none() {
            return Err(VmemError::Invalid);
        }
        Ok(size)
    }

    fn round_size(&self, size: u64) -> Result<u64> {
        if size == 0 {
            return Err(VmemError::Invalid);
        }
        size.checked_add(self.quantum - 1)
            .map(|n| n & !(self.quantum - 1))
            .ok_or(VmemError::Invalid)
    }
}

fn placement(segment: &Segment, size: u64, align: u64, min: u64, max: u64) -> Option<u64> {
    let base = segment.base.max(min).checked_add(align - 1)? & !(align - 1);
    let end = base.checked_add(size)?;
    (end <= segment.end() && end <= max).then_some(base)
}

fn freelist_index(size: u64) -> usize {
    (63 - size.max(1).leading_zeros()) as usize
}
fn guaranteed_freelist(size: u64) -> usize {
    freelist_index(size) + usize::from(!size.is_power_of_two())
}
