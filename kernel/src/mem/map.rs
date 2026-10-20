//! Virtual address ranges, mapping attributes, and anonymous overlays.
//!
//! The map indexes entries by their starting address and tracks backing-object
//! and anonymous-slice ownership across insertion, clipping, merging, and unmap.
//! Private mappings share pages until a write fault requires copy-on-write
//! promotion. Reservations prepare metadata before structural changes, and map
//! timestamps let fault handlers detect stale snapshots.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use alloc::sync::Arc;
use bitflags::bitflags;

use crate::mem::{
    PAGE_SIZE, VirtAddr,
    vmem::{Vmem, VmemError, VmemFit},
};

use super::{
    Error, Result, VmObject,
    anon::{self, AnonymousMap, MapRef},
    fault::FaultSnapshot,
    index::{Index, Node},
    pool::Pool,
};

#[cfg(target_arch = "x86_64")]
/// Exclusive upper bound of the architecture user virtual address range.
pub const USER_ADDRESS_MAX: u64 = 0x0000_8000_0000_0000;
#[cfg(target_arch = "riscv64")]
/// Exclusive upper bound of the architecture user virtual address range.
pub const USER_ADDRESS_MAX: u64 = 0x0000_0040_0000_0000;
/// Lowest virtual address accepted for userspace mappings.
pub const USER_ADDRESS_MIN: u64 = 0x1_0000;

bitflags! {
    /// Access permissions attached to a VM map entry.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct VmProtection: u8 {
        /// Permit loads.
        const READ = 1 << 0;
        /// Permit stores.
        const WRITE = 1 << 1;
        /// Permit instruction fetch.
        const EXECUTE = 1 << 2;
    }
}

/// Access that triggered a page fault.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FaultAccess {
    /// Data load.
    Read,
    /// Data store.
    Write,
    /// Instruction fetch.
    Execute,
}

/// Mapping inheritance applied during address-space fork.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VmInheritance {
    /// Child and parent share modifications.
    Share,
    /// Child and parent receive copy-on-write views.
    Copy,
    /// Mapping is omitted from the child.
    None,
}

/// Where a new mapping should be placed.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VmPlacement {
    /// Anywhere with room, chosen at random.
    Any,
    /// At or above the given address, falling back to anywhere.
    Hint(VirtAddr),
    /// Exactly at the given address; the range must already be free.
    Fixed(VirtAddr),
}

/// Physical/logical backing selected for a new mapping.
pub enum VmBacking {
    /// Private zero-fill memory.
    Anonymous,
    /// Pages supplied by a VM object, optionally overlaid for private COW.
    Object {
        /// Object providing base pages.
        object: Arc<VmObject>,
        /// Byte offset into `object`; must be page aligned.
        offset: u64,
        /// Whether writes use a private anonymous overlay.
        private: bool,
    },
}

/// Complete, validated-at-install description of a virtual mapping.
pub struct VmMapping {
    /// Address selection policy.
    pub placement: VmPlacement,
    /// Requested byte length, rounded up to pages during installation.
    pub length: u64,
    /// Initial access permissions.
    pub protection: VmProtection,
    /// Permissions later `mprotect` calls may grant.
    pub maximum_protection: VmProtection,
    /// Fork inheritance policy.
    pub inheritance: VmInheritance,
    /// Source of faulted pages.
    pub backing: VmBacking,
}

/// Access-pattern hint associated with a mapping.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VmAdvice {
    /// No special access pattern.
    Normal,
    /// Accesses are expected to be random.
    Random,
    /// Accesses are expected to be sequential.
    Sequential,
}

/// The first fault uses the advice window clipped to its mapping entry.
/// Subsequent retries are narrow. Sequential backpages are deactivated and
/// excluded from neighbor mapping.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct FaultWindow {
    pub(super) start: u64,
    pub(super) count: usize,
    pub(super) center: usize,
    pub(super) sequential: bool,
}
impl FaultWindow {
    fn new(entry: &VmMapEntry, address: u64, access: FaultAccess) -> Self {
        let (back, forward) =
            if entry.object.is_some() && entry.amap.is_some() && access == FaultAccess::Write {
                (0, 0)
            } else {
                match entry.advice {
                    VmAdvice::Random => (0, 0),
                    VmAdvice::Sequential => (8, 7),
                    VmAdvice::Normal => (3, 4),
                }
            };
        let back = back.min((address - entry.start) / PAGE_SIZE) as usize;
        let forward = forward.min((entry.end - address) / PAGE_SIZE - 1) as usize;
        Self {
            start: address - back as u64 * PAGE_SIZE,
            count: back + forward + 1,
            center: back,
            sequential: entry.advice == VmAdvice::Sequential,
        }
    }
}

/// One contiguous virtual address mapping.
#[derive(Clone)]
pub struct VmMapEntry {
    start: u64,
    end: u64,
    object: Option<Arc<VmObject>>,
    object_offset: u64,
    amap: Option<MapRef>,
    protection: VmProtection,
    maximum_protection: VmProtection,
    inheritance: VmInheritance,
    advice: VmAdvice,
    private: bool,
    needs_copy: bool,
    wired_count: u32,
}

impl VmMapEntry {
    /// Returns the entry's inclusive start address.
    pub fn start(&self) -> VirtAddr {
        VirtAddr::new(self.start)
    }

    /// Returns the entry's exclusive end address.
    pub fn end(&self) -> VirtAddr {
        VirtAddr::new(self.end)
    }

    /// Returns current access permissions.
    pub fn protection(&self) -> VmProtection {
        self.protection
    }

    /// Returns the inheritance policy.
    pub fn inheritance(&self) -> VmInheritance {
        self.inheritance
    }

    /// Returns the access-pattern hint.
    pub fn advice(&self) -> VmAdvice {
        self.advice
    }

    /// Returns whether the entry is wired against reclamation.
    pub fn is_wired(&self) -> bool {
        self.wired_count != 0
    }

    fn contains(&self, address: u64) -> bool {
        address >= self.start && address < self.end
    }

    fn page_offset(&self, address: u64) -> u64 {
        (address - self.start) / PAGE_SIZE
    }

    fn check_access(&self, access: FaultAccess) -> Result<()> {
        let required = match access {
            FaultAccess::Read => VmProtection::READ,
            FaultAccess::Write => VmProtection::WRITE,
            FaultAccess::Execute => VmProtection::EXECUTE,
        };
        if !self.protection.contains(required)
            || (!self.private
                && self.protection.contains(VmProtection::WRITE)
                && self
                    .object
                    .as_ref()
                    .is_some_and(|object| object.is_read_only()))
        {
            return Err(Error::Protection);
        }
        Ok(())
    }

    fn compatible(&self, other: &Self) -> bool {
        self.protection == other.protection
            && self.maximum_protection == other.maximum_protection
            && self.inheritance == other.inheritance
            && self.advice == other.advice
            && self.private == other.private
            && self.needs_copy == other.needs_copy
            && self.wired_count == other.wired_count
            && match (&self.object, &other.object) {
                (None, None) => true,
                (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                _ => false,
            }
    }
    fn contiguous_object(&self, next: &Self) -> bool {
        self.object.is_none()
            || self
                .object_offset
                .checked_add((self.end - self.start) / PAGE_SIZE)
                == Some(next.object_offset)
    }
    fn clipped(&self, start: u64, end: u64) -> Result<Self> {
        let page_delta = (start - self.start) / PAGE_SIZE;
        Ok(Self {
            start,
            end,
            object: self.object.clone(),
            object_offset: self
                .object_offset
                .checked_add(page_delta)
                .ok_or(Error::InvalidAddress)?,
            amap: self
                .amap
                .as_ref()
                .map(|amap| amap.clipped(amap.offset() + page_delta, (end - start) / PAGE_SIZE)),
            protection: self.protection,
            maximum_protection: self.maximum_protection,
            inheritance: self.inheritance,
            advice: self.advice,
            private: self.private,
            needs_copy: self.needs_copy,
            wired_count: self.wired_count,
        })
    }
}

/// Ordered set of virtual mappings for one address space.
///
/// Entries are indexed by address for fault resolution, while free address
/// space is tracked by a vmem arena. The arena answers placement queries in
/// constant time and is the authority on which ranges are available, so the
/// two structures are kept in exact correspondence: every entry owns one arena
/// allocation with the same base and length.
pub struct VmMap {
    entries: Index<VmMapEntry>,
    space: Vmem,
    size: u64,
    timestamp: u64,
}

static ENTRY_POOL: Pool<Node<VmMapEntry>> = Pool::new();
/// Removed entries retained until the caller completes hardware teardown.
#[must_use = "retire hardware mappings before dropping detached map entries"]
pub struct DetachedEntries {
    _entries: Index<VmMapEntry>,
}

pub(super) fn drain_pool() {
    ENTRY_POOL.drain();
    super::vmem::drain_pool();
}

impl VmMap {
    /// Creates an empty user address map.
    pub fn new() -> Result<Self> {
        let mut space = Vmem::new(PAGE_SIZE);
        space
            .add_span(USER_ADDRESS_MIN, USER_ADDRESS_MAX - USER_ADDRESS_MIN)
            .map_err(placement_error)?;
        Ok(Self {
            entries: Index::new(&ENTRY_POOL),
            space,
            size: 0,
            timestamp: 1,
        })
    }

    /// Reserves `length` bytes of address space according to `placement`.
    fn reserve(&mut self, placement: VmPlacement, length: u64) -> Result<u64> {
        debug_assert!(length != 0 && length.is_multiple_of(PAGE_SIZE));
        let result = match placement {
            VmPlacement::Any => {
                let mut entropy = [0u8; 8];
                crate::sys::random::fill_bytes(&mut entropy);
                self.space
                    .alloc_random(length, PAGE_SIZE, u64::from_ne_bytes(entropy))
            }
            VmPlacement::Hint(hint) => {
                let hint = hint.as_u64().max(USER_ADDRESS_MIN);
                let hinted = hint
                    .checked_add(PAGE_SIZE - 1)
                    .map(|value| value & !(PAGE_SIZE - 1))
                    .filter(|hint| *hint < USER_ADDRESS_MAX)
                    .and_then(|hint| {
                        self.space
                            .xalloc(length, PAGE_SIZE, hint, USER_ADDRESS_MAX, VmemFit::Instant)
                            .ok()
                    });
                hinted.map_or_else(|| self.space.alloc(length, PAGE_SIZE, VmemFit::Instant), Ok)
            }
            VmPlacement::Fixed(start) => {
                let end = start
                    .as_u64()
                    .checked_add(length)
                    .ok_or(Error::InvalidAddress)?;
                validate_range(start.as_u64(), end)?;
                self.space
                    .alloc_fixed(start.as_u64(), length)
                    .map(|()| start.as_u64())
            }
        };
        result.map_err(placement_error)
    }

    /// Returns a reservation to the arena.
    fn release(&mut self, start: u64) {
        self.space.free(start);
    }

    /// Returns the address space not covered by any mapping.
    pub fn available(&self) -> u64 {
        self.space.available()
    }

    /// Returns bytes covered by entries.
    pub fn size(&self) -> u64 {
        self.size
    }

    /// Returns the map version incremented by every structural change.
    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    /// Returns whether `[start, start + length)` is entirely unmapped.
    pub fn is_free(&self, start: VirtAddr, length: u64) -> bool {
        checked_page_length(length).is_ok_and(|length| self.space.is_free(start.as_u64(), length))
    }

    /// Reserves and inserts a mapping.
    pub fn map(&mut self, mapping: VmMapping) -> Result<VirtAddr> {
        let VmMapping {
            placement,
            length,
            protection,
            maximum_protection,
            inheritance,
            backing,
        } = mapping;
        let length = checked_page_length(length)?;
        let (object, object_offset, amap, private) = match backing {
            VmBacking::Anonymous if inheritance == VmInheritance::Share => (
                Some(VmObject::new(super::ObjectKind::Anonymous)),
                0,
                None,
                false,
            ),
            VmBacking::Anonymous => (None, 0, None, true),
            VmBacking::Object {
                object,
                offset,
                private,
            } => {
                if !offset.is_multiple_of(PAGE_SIZE) {
                    return Err(Error::InvalidAddress);
                }
                (Some(object), offset / PAGE_SIZE, None, private)
            }
        };
        let start = self.reserve(placement, length)?;
        self.insert(VmMapEntry {
            start,
            end: start + length,
            object,
            object_offset,
            amap,
            protection,
            maximum_protection,
            inheritance,
            advice: VmAdvice::Normal,
            private,
            needs_copy: private,
            wired_count: 0,
        })
    }

    /// Removes all mappings intersecting a range, clipping partial entries.
    pub fn unmap(&mut self, start: VirtAddr, length: u64) -> Result<DetachedEntries> {
        let start = start.as_u64();
        let end = checked_end(start, length)?;
        // Clip only the two boundary entries, then chain
        // the original nodes for deferred destruction. Allocating a change
        // record per entry can make freeing a large map fail under pressure.
        self.clip_at(start)?;
        self.clip_at(end)?;
        let entries = self.entries.extract_range(start, end);
        if entries.len() == 0 {
            return Err(Error::NotMapped);
        }
        for entry in entries.values() {
            self.size -= entry.end - entry.start;
            self.release(entry.start);
        }
        let detached = DetachedEntries { _entries: entries };
        self.bump_timestamp();
        Ok(detached)
    }

    /// Changes protection on a range after clipping entries.
    pub fn protect(
        &mut self,
        start: VirtAddr,
        length: u64,
        protection: VmProtection,
    ) -> Result<()> {
        let start = start.as_u64();
        let end = checked_end(start, length)?;
        self.check_coverage(start, end)?;
        for (_, entry) in self.overlapping(start, end) {
            if !entry.maximum_protection.contains(protection) {
                return Err(Error::Protection);
            }
        }
        self.clip_at(start)?;
        self.clip_at(end)?;
        for (_, entry) in self.entries.range_mut(start..end) {
            entry.protection = protection;
        }
        self.bump_timestamp();
        Ok(())
    }

    /// Updates the access-pattern hint for a range.
    pub fn advise(&mut self, start: VirtAddr, length: u64, advice: VmAdvice) -> Result<()> {
        let start = start.as_u64();
        let end = checked_end(start, length)?;
        self.check_coverage(start, end)?;
        self.clip_at(start)?;
        self.clip_at(end)?;
        for (_, entry) in self.entries.range_mut(start..end) {
            entry.advice = advice;
        }
        self.bump_timestamp();
        Ok(())
    }

    /// Takes a stable fault snapshot, or requests exclusive map access to
    /// detach an inherited anonymous view before touching its slots.
    pub(super) fn fault_snapshot(
        &self,
        address: VirtAddr,
        access: FaultAccess,
    ) -> Result<Option<FaultSnapshot>> {
        let entry = self.entry(address)?;
        entry.check_access(access)?;
        if entry.needs_copy && (access == FaultAccess::Write || entry.object.is_none()) {
            return Ok(None);
        }
        let relative = entry.page_offset(address.align_down().as_u64());
        let index = entry.amap.as_ref().map_or(0, MapRef::offset) + relative;
        Ok(Some(FaultSnapshot {
            anonymous: entry.amap.as_ref().map(MapRef::downgrade),
            index,
            source: entry.amap.as_ref().and_then(|amap| amap.lookup(index)),
            object: entry.object.clone(),
            object_index: entry.object_offset + relative,
            protection: entry.protection,
            private: entry.private,
            needs_copy: entry.needs_copy,
            access,
            window: FaultWindow::new(entry, address.align_down().as_u64(), access),
        }))
    }

    /// Copies slot references only. Physical pages remain shared until a
    /// write fault finds multiple anonymous owners and promotes one page.
    pub(super) fn prepare_copy(&mut self, address: VirtAddr, access: FaultAccess) -> Result<()> {
        let entry = self.entry(address)?;
        entry.check_access(access)?;
        if !entry.needs_copy {
            return Ok(());
        }
        // Lazily create only the chunk containing the fault for
        // a large entry without an anonymous map.
        if entry.amap.is_none() && (entry.end - entry.start) / PAGE_SIZE >= anon::LARGE_PAGES {
            let chunk_bytes = anon::CHUNK_PAGES * PAGE_SIZE;
            let start = (address.as_u64() / chunk_bytes * chunk_bytes).max(entry.start);
            let end = (address.as_u64() / chunk_bytes * chunk_bytes + chunk_bytes).min(entry.end);
            self.clip_at(start)?;
            self.clip_at(end)?;
        }
        let entry = self
            .entries
            .range_mut(..=address.as_u64())
            .next_back()
            .map(|(_, entry)| entry)
            .filter(|entry| entry.contains(address.as_u64()))
            .ok_or(Error::NotMapped)?;
        let pages = (entry.end - entry.start) / PAGE_SIZE;
        if let Some(amap) = &entry.amap {
            if !amap.is_exclusive() {
                entry.amap = Some(amap.slice(amap.offset(), pages)?);
            }
        } else {
            entry.amap = Some(AnonymousMap::allocate(pages)?);
        }
        entry.needs_copy = false;
        self.bump_timestamp();
        Ok(())
    }

    fn entry(&self, address: VirtAddr) -> Result<&VmMapEntry> {
        self.entries
            .range(..=address.as_u64())
            .next_back()
            .map(|(_, entry)| entry)
            .filter(|entry| entry.contains(address.as_u64()))
            .ok_or(Error::NotMapped)
    }

    /// Creates a child map and installs lazy COW state in copy-inherited entries.
    pub fn fork(&mut self, copy_writers: bool) -> Result<Self> {
        let mut child = Self::new()?;
        for entry in self.entries.values() {
            if entry.inheritance == VmInheritance::None {
                continue;
            }
            let mut child_entry = entry.clone();
            if child_entry.inheritance == VmInheritance::Share
                && let Some(amap) = &child_entry.amap
            {
                amap.mark_shared();
            }
            if child_entry.inheritance == VmInheritance::Copy && child_entry.private {
                child_entry.needs_copy = true;
                if copy_writers
                    && let Some(amap) = &entry.amap
                    && let Some(copied) =
                        amap.copy_for_fork(amap.offset(), (entry.end - entry.start) / PAGE_SIZE)?
                {
                    child_entry.amap = Some(copied);
                    child_entry.needs_copy = false;
                }
            }
            let node = child.entries.reserve_new(child_entry.start)?;
            child
                .space
                .alloc_fixed(child_entry.start, child_entry.end - child_entry.start)
                .map_err(placement_error)?;
            child.size += child_entry.end - child_entry.start;
            child
                .entries
                .insert(child_entry.start, child_entry, Some(node));
        }
        for (_, entry) in self.entries.iter_mut() {
            if entry.inheritance == VmInheritance::Copy && entry.private {
                entry.needs_copy = true;
            }
        }
        self.bump_timestamp();
        child.timestamp = self.timestamp;
        Ok(child)
    }

    /// Borrows an entry while the caller retains the map lock. Inspection
    /// does not manufacture an extra logical aref or change COW ownership.
    pub fn lookup(&self, address: VirtAddr) -> Option<&VmMapEntry> {
        self.entry(address).ok()
    }

    /// Publishes an entry whose range is already reserved in the arena.
    ///
    /// The reservation is returned if the entry is rejected, so a failed
    /// mapping never strands address space.
    fn insert(&mut self, entry: VmMapEntry) -> Result<VirtAddr> {
        if let Err(error) = validate_range(entry.start, entry.end) {
            self.release(entry.start);
            return Err(error);
        }
        if !entry.maximum_protection.contains(entry.protection) {
            self.release(entry.start);
            return Err(Error::Protection);
        }
        debug_assert!(
            self.entries
                .range(..entry.end)
                .next_back()
                .is_none_or(|(_, previous)| previous.end <= entry.start),
            "mem/map: the arena handed out a range that is already mapped"
        );

        if self.merge_new(&entry) {
            self.size += entry.end - entry.start;
            self.bump_timestamp();
            return Ok(VirtAddr::new(entry.start));
        }
        let node = match self.entries.reserve_new(entry.start) {
            Ok(node) => node,
            Err(error) => {
                self.release(entry.start);
                return Err(error);
            }
        };
        let start = VirtAddr::new(entry.start);
        self.size += entry.end - entry.start;
        self.entries.insert(entry.start, entry, Some(node));
        self.bump_timestamp();
        Ok(start)
    }

    /// Merge backwards first, then optionally forwards. Only a
    /// sole aref may extend; two populated amaps are never joined.
    fn merge_new(&mut self, entry: &VmMapEntry) -> bool {
        debug_assert!(entry.amap.is_none());
        let previous =
            self.entries
                .range(..entry.start)
                .next_back()
                .and_then(|(&key, previous)| {
                    (previous.end == entry.start
                        && previous.compatible(entry)
                        && previous.contiguous_object(entry)
                        && previous.amap.as_ref().is_none_or(MapRef::is_exclusive))
                    .then_some(key)
                });
        if let Some(key) = previous {
            let Some(reservation) = self.space.reserve_fixed(key, entry.end - key).ok() else {
                return false;
            };
            let previous = self
                .entries
                .get_mut(&key)
                .expect("mem: missing backmerge entry");
            if let Some(amap) = &mut previous.amap
                && amap
                    .extend((entry.end - entry.start) / PAGE_SIZE, false)
                    .is_err()
            {
                return false;
            }
            previous.end = entry.end;
            self.space.free(key);
            self.space.free(entry.start);
            self.space.alloc_reserved(reservation);
            self.merge_following(key);
            return true;
        }
        let Some(next) = self.entries.get(&entry.end) else {
            return false;
        };
        if !entry.compatible(next)
            || !entry.contiguous_object(next)
            || !next.amap.as_ref().is_none_or(MapRef::is_exclusive)
        {
            return false;
        }
        let Some(reservation) = self
            .space
            .reserve_fixed(entry.start, next.end - entry.start)
            .ok()
        else {
            return false;
        };
        let next = self
            .entries
            .get_mut(&entry.end)
            .expect("mem: missing forward merge entry");
        if let Some(amap) = &mut next.amap
            && amap
                .extend((entry.end - entry.start) / PAGE_SIZE, true)
                .is_err()
        {
            return false;
        }
        next.start = entry.start;
        next.object_offset = entry.object_offset;
        self.space.free(entry.end);
        self.space.free(entry.start);
        self.space.alloc_reserved(reservation);
        self.entries.rekey(entry.end, entry.start);
        true
    }

    fn merge_following(&mut self, key: u64) {
        let left = self.entries.get(&key).expect("mem: missing merge entry");
        let end = left.end;
        let Some(right) = self.entries.get(&end) else {
            return;
        };
        if !left.compatible(right)
            || !left.contiguous_object(right)
            || (left.amap.is_some() && right.amap.is_some())
            || !right.amap.as_ref().is_none_or(MapRef::is_exclusive)
        {
            return;
        }
        let right_end = right.end;
        let Some(reservation) = self.space.reserve_fixed(key, right_end - key).ok() else {
            return;
        };
        if left.amap.is_some() {
            if self
                .entries
                .get_mut(&key)
                .expect("mem: merge left")
                .amap
                .as_mut()
                .expect("mem: merge amap")
                .extend((right_end - end) / PAGE_SIZE, false)
                .is_err()
            {
                return;
            }
        } else if right.amap.is_some()
            && self
                .entries
                .get_mut(&end)
                .expect("mem: merge right")
                .amap
                .as_mut()
                .expect("mem: merge amap")
                .extend((end - key) / PAGE_SIZE, true)
                .is_err()
        {
            return;
        }
        let mut right = self
            .entries
            .remove(&end)
            .expect("mem: missing merged entry");
        let left = self
            .entries
            .get_mut(&key)
            .expect("mem: missing merge survivor");
        left.end = right_end;
        if left.amap.is_none() {
            left.amap = right.amap.take();
        }
        self.space.free(key);
        self.space.free(end);
        self.space.alloc_reserved(reservation);
    }

    fn overlapping(&self, start: u64, end: u64) -> impl Iterator<Item = (&u64, &VmMapEntry)> {
        let first = self
            .entries
            .range(..=start)
            .next_back()
            .map_or(start, |(&key, _)| key);
        self.entries
            .range(first..end)
            .filter(move |(_, entry)| entry.end > start)
    }

    fn check_coverage(&self, start: u64, end: u64) -> Result<()> {
        let mut covered = start;
        for (_, entry) in self.overlapping(start, end) {
            if entry.start > covered {
                return Err(Error::NotMapped);
            }
            covered = entry.end;
        }
        if covered < end {
            return Err(Error::NotMapped);
        }
        Ok(())
    }

    /// Allocate only the extra boundary entry when splitting a mapping;
    /// existing interior nodes survive protection/advice and move on unmap.
    fn clip_at(&mut self, boundary: u64) -> Result<()> {
        let Some((&key, entry)) = self.entries.range(..boundary).next_back() else {
            return Ok(());
        };
        if boundary >= entry.end {
            return Ok(());
        }
        let node = self.entries.reserve_new(boundary)?;
        let left_reservation = self
            .space
            .reserve_fixed(key, boundary - key)
            .map_err(placement_error)?;
        let right_reservation = self
            .space
            .reserve_fixed(boundary, entry.end - boundary)
            .map_err(placement_error)?;
        let left = entry.clipped(key, boundary)?;
        let right = entry.clipped(boundary, entry.end)?;
        self.space.free(key);
        self.space.alloc_reserved(left_reservation);
        self.space.alloc_reserved(right_reservation);
        // Both arefs already exist before the full-range aref is released.
        self.entries.insert(key, left, None);
        self.entries.insert(boundary, right, Some(node));
        self.bump_timestamp();
        Ok(())
    }

    pub(super) fn write_locked(&mut self) {
        self.bump_timestamp();
    }

    fn bump_timestamp(&mut self) {
        self.timestamp = self.timestamp.wrapping_add(1);
        if self.timestamp == 0 {
            self.timestamp = 1;
        }
    }
}

fn checked_page_length(length: u64) -> Result<u64> {
    if length == 0 {
        return Err(Error::InvalidAddress);
    }
    let mask = PAGE_SIZE - 1;
    length
        .checked_add(mask)
        .map(|value| value & !mask)
        .ok_or(Error::InvalidAddress)
}

fn checked_end(start: u64, length: u64) -> Result<u64> {
    if !start.is_multiple_of(PAGE_SIZE) {
        return Err(Error::InvalidAddress);
    }
    let end = start
        .checked_add(checked_page_length(length)?)
        .ok_or(Error::InvalidAddress)?;
    validate_range(start, end)?;
    Ok(end)
}

/// Maps an arena failure onto the corresponding mapping error.
fn placement_error(error: VmemError) -> Error {
    match error {
        VmemError::NoSpace | VmemError::NoMemory => Error::OutOfMemory,
        VmemError::Invalid => Error::InvalidAddress,
    }
}

fn validate_range(start: u64, end: u64) -> Result<()> {
    if start < USER_ADDRESS_MIN
        || !start.is_multiple_of(PAGE_SIZE)
        || !end.is_multiple_of(PAGE_SIZE)
        || start >= end
        || end > USER_ADDRESS_MAX
    {
        return Err(Error::InvalidAddress);
    }
    Ok(())
}
