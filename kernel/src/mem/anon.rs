//! Anonymous page ownership for private mappings and copy-on-write faults.
//!
//! Mapping slices track which anonymous slots they cover and share an owner lock
//! while their pages remain shared. Partial-reference accounting releases unused
//! slots when possible. Temporary fault snapshots retain weak identities so that
//! observing a page does not create another copy-on-write owner.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use super::{
    Error, Result,
    metadata::NoSleep,
    owner::{Owned, OwnerLock, OwnerRef},
};
use crate::mem::PageRef;
use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{ops::Deref, sync::atomic::Ordering};

pub(super) const LARGE_PAGES: u64 = 256;
pub(super) const CHUNK_PAGES: u64 = 16;

struct AnonymousPage {
    page: PageRef,
}
#[derive(Clone)]
pub(super) struct AnonymousRef {
    pub(super) page: PageRef,
    identity: Weak<AnonymousPage, NoSleep>,
}
impl AnonymousRef {
    pub(super) fn exclusive(&self) -> bool {
        let _owner = self.page.owner().write();
        self.identity.strong_count() == 1
    }
}
#[derive(Clone)]
pub(super) struct Slot(Arc<AnonymousPage, NoSleep>);
impl Slot {
    pub(super) fn new(page: PageRef) -> Result<Self> {
        Arc::try_new_in(AnonymousPage { page }, NoSleep)
            .map(Self)
            .map_err(|_| Error::OutOfMemory)
    }
    fn snapshot(&self) -> AnonymousRef {
        AnonymousRef {
            page: self.0.page.clone(),
            identity: Arc::downgrade(&self.0),
        }
    }
}

/// One logical reference to a page range, distinct from the Arc
/// that keeps the amap allocation alive during an unlocked fault.
pub(super) struct MapRef {
    map: Arc<AnonymousMap, NoSleep>,
    offset: usize,
    length: usize,
}
pub(super) type MapWeak = Weak<AnonymousMap, NoSleep>;
impl MapRef {
    pub(super) fn offset(&self) -> u64 {
        self.offset as u64
    }
    pub(super) fn downgrade(&self) -> MapWeak {
        Arc::downgrade(&self.map)
    }
    pub(super) fn is_exclusive(&self) -> bool {
        self.map.state.read().references == 1
    }
    /// An exclusive reference may extend into unused slots, activate
    /// preallocated slots, or replace the three arrays. Never join two amaps.
    pub(super) fn extend(&mut self, added: u64, backwards: bool) -> Result<()> {
        let added = usize::try_from(added).map_err(|_| Error::OutOfMemory)?;
        let mut state = self.map.state.lock();
        if state.references != 1 {
            return Err(Error::AlreadyMapped);
        }
        let old_end = self.offset + self.length;
        if matches!(state.partial, PartialReferences::Disabled) {
            state.wipe(0, self.offset);
            let tail = state.nslot - old_end;
            state.wipe(old_end, tail);
        }
        let need = if backwards {
            self.length.checked_add(added)
        } else {
            old_end.checked_add(added)
        }
        .ok_or(Error::OutOfMemory)?;
        if !backwards && need <= state.nslot {
            state.adjust(old_end, added, 1);
            self.length += added;
            return Ok(());
        }
        if backwards && added <= self.offset {
            self.offset -= added;
            state.adjust(self.offset, added, 1);
            self.length += added;
            return Ok(());
        }
        if need <= state.slots.len() {
            if backwards {
                let area = state.slots.len() - self.length;
                for index in (0..self.length).rev() {
                    state.slots[area + index] = state.slots[self.offset + index].take();
                }
                state.back.copy_within(self.offset..old_end, area);
                for index in &mut state.used {
                    *index += (area - self.offset) as u32;
                }
                self.offset = area - added;
                state.nslot = state.slots.len();
                self.length += added;
                state.reset_single_coverage(self.offset, self.length);
            } else {
                let old_nslot = state.nslot;
                if old_end < old_nslot {
                    state.adjust(old_end, old_nslot - old_end, 1);
                }
                if let PartialReferences::Counts(counts) = &mut state.partial {
                    set_run(counts, old_nslot, 1, need - old_nslot);
                }
                state.nslot = need;
                self.length += added;
            }
            return Ok(());
        }
        // Keep fresh allocations below the large-map threshold.
        if need >= LARGE_PAGES as usize {
            return Err(Error::OutOfMemory);
        }
        let ppref = matches!(state.partial, PartialReferences::Counts(_));
        let disabled = matches!(state.partial, PartialReferences::Disabled);
        drop(state);
        let mut replacement = AmapState::allocate(need)?;
        if ppref {
            replacement.partial = array(replacement.slots.len())
                .map_or(PartialReferences::Disabled, PartialReferences::Counts);
        } else if disabled {
            replacement.partial = PartialReferences::Disabled;
        }
        let mut state = self.map.state.lock();
        assert_eq!(state.references, 1);
        replacement.shared = state.shared;
        let shift = if backwards {
            replacement.slots.len() - self.length - self.offset
        } else {
            0
        };
        while let Some(index) = state.used.pop() {
            let slot = state.slots[index as usize]
                .take()
                .expect("mem: missing used anon");
            replacement.insert(index as usize + shift, slot);
        }
        if backwards {
            self.offset = replacement.slots.len() - self.length - added;
            replacement.nslot = replacement.slots.len();
        }
        self.length += added;
        replacement.reset_single_coverage(self.offset, self.length);
        let retired = core::mem::replace(&mut *state, replacement);
        drop(state);
        drop(retired);
        Ok(())
    }

    /// Adds a partial reference; clipping does not copy any slots.
    pub(super) fn clipped(&self, offset: u64, count: u64) -> Self {
        let offset = usize::try_from(offset).expect("mem: amap offset overflow");
        let count = usize::try_from(count).expect("mem: amap length overflow");
        assert!(offset >= self.offset && offset + count <= self.offset + self.length);
        self.map.add_reference(self.offset, offset, count);
        Self {
            map: self.map.clone(),
            offset,
            length: count,
        }
    }
    pub(super) fn mark_shared(&self) {
        self.map.state.lock().shared = true;
    }
    pub(super) fn slice(&self, start: u64, count: u64) -> Result<Self> {
        self.map.copy_range(start, count, false)
    }
    pub(super) fn copy_for_fork(&self, start: u64, count: u64) -> Result<Option<Self>> {
        let state = self.map.state.read();
        let start = start as usize;
        let count = count as usize;
        assert!(start + count <= state.nslot);
        let writers = state.slots[start..start + count]
            .iter()
            .flatten()
            .any(|slot| slot.0.page.write_windows.load(Ordering::Acquire) != 0);
        drop(state);
        writers
            .then(|| self.map.copy_range(start as u64, count as u64, true))
            .transpose()
    }
}
impl Clone for MapRef {
    fn clone(&self) -> Self {
        self.map
            .add_reference(self.offset, self.offset, self.length);
        Self {
            map: self.map.clone(),
            offset: self.offset,
            length: self.length,
        }
    }
}
impl Deref for MapRef {
    type Target = AnonymousMap;
    fn deref(&self) -> &AnonymousMap {
        &self.map
    }
}
impl Drop for MapRef {
    fn drop(&mut self) {
        let mut state = self.map.state.lock();
        assert!(state.references != 0, "mem: amap reference underflow");
        if state.references == 1 {
            state.references = 0;
            let length = state.slots.len();
            state.wipe(0, length);
        } else {
            state.establish(self.offset);
            if state.references == 2 {
                state.shared = false;
            }
            state.references -= 1;
            state.adjust(self.offset, self.length, -1);
        }
    }
}

enum PartialReferences {
    Unestablished,
    Disabled,
    Counts(Vec<i32, NoSleep>),
}
struct AmapState {
    nslot: usize,
    references: usize,
    shared: bool,
    slots: Vec<Option<Slot>, NoSleep>,
    used: Vec<u32, NoSleep>,
    back: Vec<u32, NoSleep>,
    partial: PartialReferences,
}
fn array<T: Default>(length: usize) -> Result<Vec<T, NoSleep>> {
    let mut result = Vec::new_in(NoSleep);
    result
        .try_reserve_exact(length)
        .map_err(|_| Error::OutOfMemory)?;
    result.resize_with(length, T::default);
    Ok(result)
}
impl AmapState {
    fn allocate(length: usize) -> Result<Self> {
        let capacity = NoSleep::size_class(
            length
                .checked_mul(size_of::<u32>())
                .ok_or(Error::OutOfMemory)?,
        )
        .ok_or(Error::OutOfMemory)?
            / size_of::<u32>();
        let mut used = Vec::new_in(NoSleep);
        used.try_reserve_exact(capacity)
            .map_err(|_| Error::OutOfMemory)?;
        Ok(Self {
            nslot: length,
            references: 1,
            shared: false,
            slots: array(capacity)?,
            used,
            back: array(capacity)?,
            partial: PartialReferences::Unestablished,
        })
    }
    fn reset_single_coverage(&mut self, offset: usize, length: usize) {
        if let PartialReferences::Counts(counts) = &mut self.partial {
            set_run(counts, 0, 0, offset);
            set_run(counts, offset, 1, length);
            set_run(counts, offset + length, 0, self.nslot - offset - length);
        }
    }
    fn insert(&mut self, index: usize, slot: Slot) {
        if self.slots[index].is_none() {
            self.back[index] = self.used.len() as u32;
            self.used.push(index as u32);
        }
        self.slots[index] = Some(slot);
    }
    /// Swap the last used index into the removed index's place.
    fn remove(&mut self, index: usize) {
        if self.slots[index].is_none() {
            return;
        }
        let position = self.back[index] as usize;
        self.used.swap_remove(position);
        if position < self.used.len() {
            self.back[self.used[position] as usize] = position as u32;
        }
        self.slots[index] = None;
    }
    /// Choose the cheaper of slot and used-index traversal.
    fn wipe(&mut self, start: usize, length: usize) {
        let end = start + length;
        if length < self.used.len() {
            for index in start..end {
                self.remove(index);
            }
        } else {
            let mut used = 0;
            while used < self.used.len() {
                let index = self.used[used] as usize;
                if index >= start && index < end {
                    self.remove(index);
                } else {
                    used += 1;
                }
            }
        }
    }
    fn establish(&mut self, offset: usize) {
        if !matches!(self.partial, PartialReferences::Unestablished) {
            return;
        }
        self.partial = match array(self.slots.len()) {
            Ok(mut counts) => {
                set_run(&mut counts, 0, 0, offset);
                set_run(
                    &mut counts,
                    offset,
                    self.references as i32,
                    self.nslot - offset,
                );
                PartialReferences::Counts(counts)
            }
            // If partial-reference storage is unavailable, retaining unused slots is preferable
            // to making an otherwise valid fork/clip fail for ppref storage.
            Err(_) => PartialReferences::Disabled,
        };
    }
    fn adjust(&mut self, start: usize, length: usize, delta: i32) {
        let end = start + length;
        let mut current = 0;
        let mut previous = 0;
        let PartialReferences::Counts(counts) = &mut self.partial else {
            return;
        };
        while current < start {
            let (refs, mut run) = get_run(counts, current);
            if current + run > start {
                set_run(counts, current, refs, start - current);
                set_run(counts, start, refs, run - (start - current));
                run = start - current;
            }
            previous = current;
            current += run;
        }
        let (previous_refs, previous_len) = if current == 0 {
            (-1, 0)
        } else {
            get_run(counts, previous)
        };
        // Borrow the ppref array only for each update so wiping anons can
        // mutate the other arrays under the same owner capability.
        while current < end {
            let PartialReferences::Counts(counts) = &mut self.partial else {
                unreachable!()
            };
            let (refs, mut run) = get_run(counts, current);
            if current + run > end {
                set_run(counts, current, refs, end - current);
                set_run(counts, end, refs, run - (end - current));
                run = end - current;
            }
            let refs = refs + delta;
            assert!(refs >= 0 && refs as usize <= self.references);
            if current == previous + previous_len && refs == previous_refs {
                set_run(counts, previous, refs, previous_len + run);
            } else {
                set_run(counts, current, refs, run);
            }
            if refs == 0 {
                self.wipe(current, run);
            }
            current += run;
        }
    }
}
fn get_run(counts: &[i32], offset: usize) -> (i32, usize) {
    if counts[offset] > 0 {
        (counts[offset] - 1, 1)
    } else {
        (-counts[offset] - 1, counts[offset + 1] as usize)
    }
}
fn set_run(counts: &mut [i32], offset: usize, refs: i32, length: usize) {
    if length == 0 {
        return;
    }
    if length == 1 {
        counts[offset] = refs + 1;
    } else {
        counts[offset] = -(refs + 1);
        counts[offset + 1] = length as i32;
    }
}

pub(super) struct AnonymousMap {
    state: Owned<AmapState>,
}
impl AnonymousMap {
    pub(super) fn allocate(pages: u64) -> Result<MapRef> {
        Self::with_owner(pages, OwnerLock::new()?)
    }
    fn with_owner(pages: u64, owner: OwnerRef) -> Result<MapRef> {
        let length = usize::try_from(pages).map_err(|_| Error::OutOfMemory)?;
        if length > u32::MAX as usize {
            return Err(Error::OutOfMemory);
        }
        let state = AmapState::allocate(length)?;
        let map = Arc::try_new_in(
            Self {
                state: Owned::new(owner, state),
            },
            NoSleep,
        )
        .map_err(|_| Error::OutOfMemory)?;
        Ok(MapRef {
            map,
            offset: 0,
            length,
        })
    }
    pub(super) fn owner(&self) -> &OwnerRef {
        self.state.owner()
    }
    fn add_reference(&self, original_offset: usize, offset: usize, length: usize) {
        let mut state = self.state.lock();
        assert!(state.references > 0 && offset + length <= state.slots.len());
        state.establish(original_offset);
        state.references = state
            .references
            .checked_add(1)
            .filter(|&refs| refs < (i32::MAX as usize))
            .expect("mem: amap reference overflow");
        state.adjust(offset, length, 1);
    }
    fn copy_range(&self, start: u64, count: u64, copy_writers: bool) -> Result<MapRef> {
        let copy = Self::with_owner(count, self.owner().clone())?;
        let mut owner = self.owner().write();
        let start = start as usize;
        let count = count as usize;
        assert!(start + count <= self.state.get(&owner).nslot);
        for index in 0..count {
            let slot = self.state.get(&owner).slots[start + index].clone();
            if let Some(mut slot) = slot {
                if copy_writers
                    && slot.0.page.write_windows.load(Ordering::Acquire) != 0
                    && let Some(page) = slot.0.page.copy_wired_locked(index as u64, &mut owner)?
                {
                    slot = Slot::new(page)?;
                }
                copy.map
                    .state
                    .get_locked_mut(&mut owner)
                    .insert(index, slot);
            }
        }
        drop(owner);
        Ok(copy)
    }
    pub(super) fn lookup(&self, index: u64) -> Option<AnonymousRef> {
        self.state
            .read()
            .slots
            .get(index as usize)
            .and_then(Option::as_ref)
            .map(Slot::snapshot)
    }
    /// Observe a neighbor under the shared amap/anon owner lock. Temporary
    /// page handles do not change an_ref; only slot ownership controls COW.
    pub(super) fn with_neighbors(
        &self,
        start: u64,
        count: usize,
        visit: impl FnOnce(&[Option<(PageRef, bool)>], &mut super::owner::OwnerWrite<'_>),
    ) {
        assert!(count <= 16);
        let mut state = self.state.lock();
        let pages: [Option<(PageRef, bool)>; 16] = core::array::from_fn(|offset| {
            if offset >= count {
                return None;
            }
            state
                .slots
                .get(start as usize + offset)
                .and_then(Option::as_ref)
                .map(|slot| (slot.0.page.clone(), Arc::strong_count(&slot.0) > 1))
        });
        visit(&pages[..count], state.capability_mut());
    }

    pub(super) fn prepare(&self, index: u64) -> Result<()> {
        let state = self.state.read();
        if state.references == 0 {
            Err(Error::NotMapped)
        } else if index >= state.nslot as u64 {
            Err(Error::InvalidAddress)
        } else {
            Ok(())
        }
    }
    pub(super) fn commit(
        &self,
        index: u64,
        expected: Option<&AnonymousRef>,
        replacement: Option<Slot>,
        install: impl FnOnce() -> Result<()>,
    ) -> Result<bool> {
        let mut state = self.state.lock();
        if state.references == 0 {
            return Ok(false);
        }
        if index >= state.nslot as u64 {
            return Ok(false);
        }
        let Some(slot) = state.slots.get(index as usize) else {
            return Ok(false);
        };
        let matches = match (slot.as_ref(), expected) {
            (Some(current), Some(expected)) => {
                core::ptr::eq(Arc::as_ptr(&current.0), expected.identity.as_ptr())
            }
            (None, None) => true,
            _ => false,
        };
        if !matches {
            return Ok(false);
        }
        if let Some(replacement) = &replacement {
            assert!(
                Arc::ptr_eq(replacement.0.page.owner(), self.owner()),
                "mem: anon has a different owner lock"
            );
            if state.shared
                && let Some(old) = slot
            {
                super::pmap::remove_mappings_for_pages(core::slice::from_ref(&old.0.page));
            }
        }
        install()?;
        if let Some(slot) = replacement {
            state.insert(index as usize, slot);
        }
        Ok(true)
    }
}
// Array storage is returned by NoSleep's RAII allocations; no directory pool
// remains to drain. Kept as the VM metadata drain integration point.
pub(super) fn drain_pool() {}
