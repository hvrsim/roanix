//! Virtual address maps, anonymous overlays, and copy-on-write resolution.

use alloc::{collections::BTreeMap, sync::Arc};
use bitflags::bitflags;

use crate::mem::{
    PAGE_SIZE, VirtAddr,
    vmem::{Vmem, VmemError, VmemFit},
};

use super::{Error, Result, VmObject, VmPage};

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
    /// Pages are likely to be reused.
    WillNeed,
    /// Pages are unlikely to be reused.
    DontNeed,
}

struct VmAnon {
    page: Arc<VmPage>,
}

struct AnonMap {
    pages: crate::sys::sync::Mutex<BTreeMap<u64, Arc<VmAnon>>>,
}

impl AnonMap {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            pages: crate::sys::sync::Mutex::new(BTreeMap::new()),
        })
    }

    fn private_copy(&self) -> Arc<Self> {
        Arc::new(Self {
            pages: crate::sys::sync::Mutex::new(self.pages.lock().clone()),
        })
    }

    fn slice(&self, start: u64, pages: u64) -> Arc<Self> {
        let end = start.saturating_add(pages);
        let sliced = self
            .pages
            .lock()
            .range(start..end)
            .map(|(index, anon)| (*index, anon.clone()))
            .collect();
        Arc::new(Self {
            pages: crate::sys::sync::Mutex::new(sliced),
        })
    }

    fn lookup(&self, index: u64) -> Option<(Arc<VmPage>, bool)> {
        let pages = self.pages.lock();
        let anon = pages.get(&index)?;
        Some((anon.page.clone(), Arc::strong_count(anon) == 1))
    }

    fn cow_page(&self, index: u64, source: &Arc<VmPage>) -> Result<Arc<VmPage>> {
        let mut pages = self.pages.lock();
        if let Some(anon) = pages.get(&index) {
            if Arc::strong_count(anon) == 1 {
                return Ok(anon.page.clone());
            }
            let page = anon.page.copy_to(index)?;
            pages.insert(index, Arc::new(VmAnon { page: page.clone() }));
            return Ok(page);
        }

        let page = if super::is_shared_zero_page(source) {
            VmPage::new_zero(index, super::phys::PageOwnerKind::Anonymous)
        } else {
            source.copy_to(index)?
        };
        pages.insert(index, Arc::new(VmAnon { page: page.clone() }));
        Ok(page)
    }
}

/// One contiguous virtual address mapping.
#[derive(Clone)]
pub struct VmMapEntry {
    start: u64,
    end: u64,
    object: Option<Arc<VmObject>>,
    object_offset: u64,
    amap: Option<Arc<AnonMap>>,
    amap_offset: u64,
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

    fn clipped(&self, start: u64, end: u64) -> Self {
        let page_delta = (start - self.start) / PAGE_SIZE;
        let mut clipped = self.clone();
        clipped.start = start;
        clipped.end = end;
        clipped.object_offset = clipped.object_offset.saturating_add(page_delta);
        clipped.amap_offset = clipped.amap_offset.saturating_add(page_delta);
        if self.inheritance != VmInheritance::Share
            && let Some(amap) = &self.amap
        {
            clipped.amap = Some(amap.slice(clipped.amap_offset, (end - start) / PAGE_SIZE));
        }
        clipped
    }
}

/// Page and permissions produced by map fault classification.
pub struct ResolvedPage {
    /// Page that should be entered into the pmap.
    pub page: Arc<VmPage>,
    /// Effective mapping permissions.
    pub protection: VmProtection,
    /// Whether this resolution completed a copy-on-write promotion.
    pub promoted: bool,
    /// Object identity to revalidate after the PTE becomes visible.
    pub(super) object_fault: Option<ObjectFault>,
}

pub(super) struct ObjectFault {
    pub(super) object: Arc<VmObject>,
    pub(super) index: u64,
}

/// Ordered set of virtual mappings for one address space.
///
/// Entries are indexed by address for fault resolution, while free address
/// space is tracked by a vmem arena. The arena answers placement queries in
/// constant time and is the authority on which ranges are available, so the
/// two structures are kept in exact correspondence: every entry owns one arena
/// allocation with the same base and length.
pub struct VmMap {
    entries: BTreeMap<u64, VmMapEntry>,
    space: Vmem,
    size: u64,
    timestamp: u64,
}

impl VmMap {
    /// Creates an empty user address map.
    pub fn new() -> Self {
        let mut space = Vmem::new(PAGE_SIZE);
        space
            .add_span(USER_ADDRESS_MIN, USER_ADDRESS_MAX - USER_ADDRESS_MIN)
            .expect("mem/map: user address span is malformed");
        Self {
            entries: BTreeMap::new(),
            space,
            size: 0,
            timestamp: 1,
        }
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
            VmBacking::Anonymous => (None, 0, Some(AnonMap::new()), true),
            VmBacking::Object {
                object,
                offset,
                private,
            } => {
                if !offset.is_multiple_of(PAGE_SIZE) {
                    return Err(Error::InvalidAddress);
                }
                (
                    Some(object),
                    offset / PAGE_SIZE,
                    private.then(AnonMap::new),
                    private,
                )
            }
        };
        let start = self.reserve(placement, length)?;
        self.insert(VmMapEntry {
            start,
            end: start + length,
            object,
            object_offset,
            amap,
            amap_offset: 0,
            protection,
            maximum_protection,
            inheritance,
            advice: VmAdvice::Normal,
            private,
            needs_copy: false,
            wired_count: 0,
        })
    }

    /// Removes all mappings intersecting a range, clipping partial entries.
    pub fn unmap(&mut self, start: VirtAddr, length: u64) -> Result<()> {
        let start = start.as_u64();
        let end = checked_end(start, length)?;
        let keys = self.overlapping_keys(start, end);
        if keys.is_empty() {
            return Err(Error::NotMapped);
        }

        for key in keys {
            let entry = self
                .entries
                .remove(&key)
                .expect("mem/map: overlapping entry vanished");
            self.size -= entry.end - entry.start;

            let mut pieces = [(0u64, 0u64); 2];
            let mut count = 0;
            if entry.start < start {
                pieces[count] = (entry.start, start);
                count += 1;
            }
            if entry.end > end {
                pieces[count] = (end, entry.end);
                count += 1;
            }
            self.reserve_clipped(entry.start, &pieces[..count]);

            if entry.start < start {
                let left = entry.clipped(entry.start, start);
                self.size += left.end - left.start;
                self.entries.insert(left.start, left);
            }
            if entry.end > end {
                let right = entry.clipped(end, entry.end);
                self.size += right.end - right.start;
                self.entries.insert(right.start, right);
            }
        }
        self.bump_timestamp();
        Ok(())
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
        let keys = self.covered_keys(start, end)?;
        for key in &keys {
            let entry = self
                .entries
                .get(key)
                .expect("mem/map: protected entry vanished during validation");
            if !entry.maximum_protection.contains(protection) {
                return Err(Error::Protection);
            }
        }
        self.rewrite_range(start, end, keys, |entry| entry.protection = protection);
        self.bump_timestamp();
        Ok(())
    }

    /// Updates the access-pattern hint for a range.
    pub fn advise(&mut self, start: VirtAddr, length: u64, advice: VmAdvice) -> Result<()> {
        let start = start.as_u64();
        let end = checked_end(start, length)?;
        let keys = self.covered_keys(start, end)?;
        self.rewrite_range(start, end, keys, |entry| entry.advice = advice);
        self.bump_timestamp();
        Ok(())
    }

    /// Resolves a fault using the flat anonymous-overlay/object model.
    pub fn resolve_fault(
        &mut self,
        address: VirtAddr,
        access: FaultAccess,
    ) -> Result<ResolvedPage> {
        let address = address.align_down().as_u64();
        let key = self
            .entries
            .range(..=address)
            .next_back()
            .and_then(|(key, entry)| entry.contains(address).then_some(*key))
            .ok_or(Error::NotMapped)?;
        let entry = self
            .entries
            .get_mut(&key)
            .expect("mem/map: fault entry vanished");

        let required = match access {
            FaultAccess::Read => VmProtection::READ,
            FaultAccess::Write => VmProtection::WRITE,
            FaultAccess::Execute => VmProtection::EXECUTE,
        };
        if !entry.protection.contains(required) {
            return Err(Error::Protection);
        }

        let relative = entry.page_offset(address);
        let anon_index = entry.amap_offset + relative;
        let object_index = entry.object_offset + relative;
        let write = access == FaultAccess::Write;

        if write && entry.private {
            if entry.needs_copy {
                let amap = entry.amap.get_or_insert_with(AnonMap::new);
                if Arc::strong_count(amap) > 1 {
                    entry.amap = Some(amap.private_copy());
                }
                entry.needs_copy = false;
            }
            let source = if let Some(amap) = &entry.amap
                && let Some((page, _)) = amap.lookup(anon_index)
            {
                page
            } else if let Some(object) = &entry.object {
                object.fault_page(object_index, false)?
            } else {
                super::shared_zero_page()
            };
            let amap = entry.amap.get_or_insert_with(AnonMap::new);
            let page = amap.cow_page(anon_index, &source)?;
            return Ok(ResolvedPage {
                page,
                protection: entry.protection,
                promoted: true,
                object_fault: None,
            });
        }

        if let Some(amap) = &entry.amap
            && let Some((page, exclusive)) = amap.lookup(anon_index)
        {
            let mut protection = entry.protection;
            if entry.private && (entry.needs_copy || !exclusive) {
                protection.remove(VmProtection::WRITE);
            }
            return Ok(ResolvedPage {
                page,
                protection,
                promoted: false,
                object_fault: None,
            });
        }

        let (page, object_fault) = if let Some(object) = &entry.object {
            (
                object.fault_page(object_index, write && !entry.private)?,
                Some(ObjectFault {
                    object: object.clone(),
                    index: object_index,
                }),
            )
        } else {
            (super::shared_zero_page(), None)
        };
        let mut protection = entry.protection;
        if entry.private || super::is_shared_zero_page(&page) {
            protection.remove(VmProtection::WRITE);
        }
        Ok(ResolvedPage {
            page,
            protection,
            promoted: false,
            object_fault,
        })
    }

    /// Creates a child map and installs lazy COW state in copy-inherited entries.
    pub fn fork(&mut self) -> Self {
        let mut child = Self::new();
        for entry in self.entries.values_mut() {
            match entry.inheritance {
                VmInheritance::None => continue,
                VmInheritance::Share => {}
                VmInheritance::Copy => {
                    if entry.private {
                        entry.needs_copy = true;
                    }
                }
            }
            let mut child_entry = entry.clone();
            if child_entry.inheritance == VmInheritance::Copy && child_entry.private {
                child_entry.needs_copy = true;
            }
            child
                .space
                .alloc_fixed(child_entry.start, child_entry.end - child_entry.start)
                .expect("mem/map: inherited range is not free in a fresh map");
            child.size += child_entry.end - child_entry.start;
            child.entries.insert(child_entry.start, child_entry);
        }
        self.bump_timestamp();
        child.timestamp = self.timestamp;
        child
    }

    /// Returns a cloned entry containing `address`.
    pub fn lookup(&self, address: VirtAddr) -> Option<VmMapEntry> {
        self.entries
            .range(..=address.as_u64())
            .next_back()
            .and_then(|(_, entry)| entry.contains(address.as_u64()).then(|| entry.clone()))
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

        let start = VirtAddr::new(entry.start);
        self.size += entry.end - entry.start;
        self.entries.insert(entry.start, entry);
        self.bump_timestamp();
        Ok(start)
    }

    /// Replaces one entry's reservation with the ranges it was clipped into.
    ///
    /// The whole original allocation is released first so the surviving pieces
    /// can be reserved back at their exact addresses.
    fn reserve_clipped(&mut self, original: u64, pieces: &[(u64, u64)]) {
        self.release(original);
        for (start, end) in pieces {
            if start == end {
                continue;
            }
            self.space
                .alloc_fixed(*start, end - start)
                .expect("mem/map: reclaiming a just-released range must succeed");
        }
    }

    fn overlapping_keys(&self, start: u64, end: u64) -> alloc::vec::Vec<u64> {
        self.entries
            .range(..end)
            .filter_map(|(key, entry)| (entry.end > start).then_some(*key))
            .collect()
    }

    /// Returns every intersecting entry after proving the range has no holes.
    fn covered_keys(&self, start: u64, end: u64) -> Result<alloc::vec::Vec<u64>> {
        let keys = self.overlapping_keys(start, end);
        let mut covered = start;
        for key in &keys {
            let entry = &self.entries[key];
            if entry.start > covered {
                return Err(Error::NotMapped);
            }
            covered = covered.max(entry.end);
        }
        if covered < end {
            return Err(Error::NotMapped);
        }
        Ok(keys)
    }

    /// Clips each intersecting entry and updates only the requested middle.
    fn rewrite_range(
        &mut self,
        start: u64,
        end: u64,
        keys: alloc::vec::Vec<u64>,
        mut update: impl FnMut(&mut VmMapEntry),
    ) {
        for key in keys {
            let entry = self
                .entries
                .remove(&key)
                .expect("mem/map: rewritten entry vanished");
            let middle_start = entry.start.max(start);
            let middle_end = entry.end.min(end);
            let pieces = [
                (entry.start, middle_start),
                (middle_start, middle_end),
                (middle_end, entry.end),
            ];
            self.reserve_clipped(entry.start, &pieces);

            if entry.start < middle_start {
                let left = entry.clipped(entry.start, middle_start);
                self.entries.insert(left.start, left);
            }
            let mut middle = entry.clipped(middle_start, middle_end);
            update(&mut middle);
            self.entries.insert(middle.start, middle);
            if middle_end < entry.end {
                let right = entry.clipped(middle_end, entry.end);
                self.entries.insert(right.start, right);
            }
        }
    }

    fn bump_timestamp(&mut self) {
        self.timestamp = self.timestamp.wrapping_add(1);
        if self.timestamp == 0 {
            self.timestamp = 1;
        }
    }
}

impl Default for VmMap {
    fn default() -> Self {
        Self::new()
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
        VmemError::NoSpace => Error::OutOfMemory,
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
