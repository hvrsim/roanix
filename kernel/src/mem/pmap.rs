//! Machine-independent pmap ownership and address-space integration.
//!
//! # Lock order
//!
//! Address-space operations acquire `VmSpace::map`, then `PmapInner::mappings`,
//! then a page's reverse-mapping list. Reclaim detaches the reverse list before
//! trying a pmap lock, so it never inverts that order or sleeps behind a fault.

use alloc::{collections::BTreeMap, sync::Arc};
use core::ptr;

use crate::{
    arch,
    mem::{PAGE_SIZE, PhysAddr, VirtAddr, VmFlags},
    sys::sync::Mutex,
};

use super::{
    Error, FaultAccess, ObjectKind, Result, USER_ADDRESS_MAX, USER_ADDRESS_MIN, VmAdvice, VmMap,
    VmMapping, VmObject, VmPage, VmPlacement, VmProtection, tlb::Shootdown,
};

struct Mapping {
    page: Arc<VmPage>,
    protection: VmProtection,
}

/// One hardware mapping of a logical page, recorded on the page itself.
pub(super) struct ReverseMapping {
    pub(super) pmap: alloc::sync::Weak<PmapInner>,
    pub(super) address: u64,
}

/// Hardware page map owned by one virtual address space.
pub struct Pmap {
    inner: Arc<PmapInner>,
}

pub(super) struct PmapInner {
    root: PhysAddr,
    mappings: Mutex<BTreeMap<u64, Mapping>>,
}

impl Pmap {
    /// Allocates a user pmap containing the shared kernel half.
    pub fn new() -> Result<Self> {
        // SAFETY: the captured kernel root is permanent and valid, and the
        // architecture routine only copies shared kernel root entries.
        let root = unsafe { arch::paging::create_user_root(super::kernel_root()) }
            .map_err(|_| Error::OutOfMemory)?;
        Ok(Self {
            inner: Arc::new(PmapInner {
                root,
                mappings: Mutex::new(BTreeMap::new()),
            }),
        })
    }

    /// Returns the root page-table physical address.
    pub fn root(&self) -> PhysAddr {
        self.inner.root
    }

    /// Activates this pmap on the current CPU.
    fn activate(&self) -> Result<()> {
        // SAFETY: `root` remains owned by this pmap and contains the permanent
        // shared kernel mappings required for execution.
        unsafe { arch::paging::activate_root(self.inner.root) }.map_err(|_| Error::Pmap)
    }

    /// Enters or replaces one user mapping.
    ///
    /// Re-entering an identical mapping is the common case when a fault races
    /// another CPU or when a syscall walks user memory, so it is detected
    /// before any page-table write and costs no invalidation.
    pub fn enter(
        &self,
        address: VirtAddr,
        page: Arc<VmPage>,
        protection: VmProtection,
    ) -> Result<()> {
        let address = address.align_down();
        {
            let mappings = self.inner.mappings.lock();
            if let Some(current) = mappings.get(&address.as_u64())
                && current.protection == protection
                && Arc::ptr_eq(&current.page, &page)
            {
                return Ok(());
            }
        }

        let paddr = page.acquire_mapping()?;
        let flags = pmap_flags(protection);
        let mut mappings = self.inner.mappings.lock();

        let existing = mappings.contains_key(&address.as_u64());
        let result = if existing {
            // SAFETY: the pmap lock serializes leaf changes and `paddr` remains
            // wired by the new mapping reference.
            unsafe { arch::paging::remap_page(self.inner.root, address, paddr, flags) }
        } else {
            // SAFETY: the pmap lock serializes page-table construction and the
            // page remains resident through its mapping reference.
            unsafe { arch::paging::map_page(self.inner.root, address, paddr, flags) }
        };
        if result.is_err() {
            drop(mappings);
            page.release_mapping();
            return Err(Error::Pmap);
        }

        let previous = mappings.insert(
            address.as_u64(),
            Mapping {
                page: page.clone(),
                protection,
            },
        );
        let same_page = previous
            .as_ref()
            .is_some_and(|previous| Arc::ptr_eq(&previous.page, &page));
        if !same_page {
            if let Some(previous) = &previous {
                previous
                    .page
                    .remove_reverse_mapping(Arc::as_ptr(&self.inner), address.as_u64());
            }
            page.add_reverse_mapping(&self.inner, address.as_u64());
        }
        drop(mappings);

        let Some(previous) = previous else {
            // A newly present leaf cannot be cached anywhere, so no remote
            // invalidation is required.
            return Ok(());
        };
        let mut shootdown = Shootdown::for_root(self.inner.root);
        shootdown.push(address);
        shootdown.commit();
        if same_page {
            page.release_mapping();
        } else {
            previous.page.release_mapping();
        }
        Ok(())
    }

    /// Removes a virtual range.
    ///
    /// Leaves that were already removed are always retired, even when a later
    /// leaf fails, so no page keeps a mapping reference it no longer has.
    pub fn remove(&self, start: VirtAddr, length: u64) -> Result<()> {
        let start = start.align_down().as_u64();
        let end = checked_page_end(start, length)?;
        let mut failure = None;
        let mut mappings = self.inner.mappings.lock();

        loop {
            let mut retired: [Option<Arc<VmPage>>; Shootdown::INLINE] =
                [const { None }; Shootdown::INLINE];
            let mut count = 0;
            let mut shootdown = Shootdown::for_root(self.inner.root);
            while count < retired.len() {
                let Some(key) = mappings.range(start..end).next().map(|(key, _)| *key) else {
                    break;
                };
                // SAFETY: the pmap lock excludes concurrent changes to this leaf.
                if unsafe { arch::paging::unmap_page(self.inner.root, VirtAddr::new(key)) }.is_err()
                {
                    failure = Some(Error::Pmap);
                    break;
                }
                let mapping = mappings
                    .remove(&key)
                    .expect("mem/pmap: removed mapping vanished");
                mapping
                    .page
                    .remove_reverse_mapping(Arc::as_ptr(&self.inner), key);
                shootdown.push(VirtAddr::new(key));
                retired[count] = Some(mapping.page);
                count += 1;
            }

            shootdown.commit();
            for page in retired[..count].iter_mut().filter_map(Option::take) {
                page.release_mapping();
            }
            if failure.is_some() || count < retired.len() {
                break;
            }
        }
        failure.map_or(Ok(()), Err)
    }

    /// Write-protects every writable mapping before a copy-on-write fork.
    ///
    /// Leaves are downgraded in place, so the pass needs no side table and
    /// cannot fail part way through for want of memory.
    pub fn write_protect_all(&self) -> Result<()> {
        let mut mappings = self.inner.mappings.lock();
        let mut shootdown = Shootdown::for_root(self.inner.root);
        let mut failure = None;

        for (address, mapping) in mappings.iter_mut() {
            if !mapping.protection.contains(VmProtection::WRITE) {
                continue;
            }
            let mut protection = mapping.protection;
            protection.remove(VmProtection::WRITE);
            // SAFETY: the pmap lock serializes this leaf update and the entry
            // keeps its frame pinned through the mapping reference.
            let result = unsafe {
                arch::paging::protect_page(
                    self.inner.root,
                    VirtAddr::new(*address),
                    pmap_flags(protection),
                )
            };
            if result.is_err() {
                failure = Some(Error::Pmap);
                break;
            }
            mapping.protection = protection;
            shootdown.push(VirtAddr::new(*address));
        }
        drop(mappings);
        shootdown.commit();
        failure.map_or(Ok(()), Err)
    }

    /// Returns the logical page and permissions currently mapped at `address`.
    pub(super) fn mapped_page(&self, address: VirtAddr) -> Option<(Arc<VmPage>, VmProtection)> {
        self.inner
            .mappings
            .lock()
            .get(&address.as_u64())
            .map(|mapping| (mapping.page.clone(), mapping.protection))
    }

    /// Returns the physical address currently mapped at `address`.
    pub fn extract(&self, address: VirtAddr) -> Option<PhysAddr> {
        let _mappings = self.inner.mappings.lock();
        // SAFETY: this pmap owns the hierarchy and its mapping lock excludes
        // concurrent page-table writes for the duration of the walk.
        unsafe { arch::paging::translate(self.inner.root, address) }
    }

    /// Returns the number of tracked resident mappings.
    pub fn resident_count(&self) -> usize {
        self.inner.mappings.lock().len()
    }
}

impl Drop for PmapInner {
    fn drop(&mut self) {
        let mappings = core::mem::take(self.mappings.get_mut());
        let mut shootdown = Shootdown::for_root(self.root);
        for (address, mapping) in &mappings {
            // SAFETY: `&mut self` excludes concurrent pmap operations.
            let _ = unsafe { arch::paging::unmap_page(self.root, VirtAddr::new(*address)) };
            mapping
                .page
                .remove_reverse_mapping(ptr::from_ref(self), *address);
            shootdown.push(VirtAddr::new(*address));
        }
        shootdown.commit();
        for (_, mapping) in mappings {
            mapping.page.release_mapping();
        }
        // SAFETY: all user leaves were removed, this root is inactive because
        // every owning `VmSpace` reference has been released, and the shared
        // kernel half is not freed by the architecture routine.
        unsafe { arch::paging::destroy_user_root(self.root) };
    }
}

/// Best-effort teardown used by reclaim, which must not sleep behind a pmap.
pub(super) fn try_remove_all_mappings(page: &Arc<VmPage>) {
    let mut mappings = page.take_reverse_mappings();
    if mappings.is_empty() {
        return;
    }
    let mut shootdown = Shootdown::global();
    let mut removed = 0usize;
    mappings.retain(|mapping| {
        let Some(pmap) = mapping.pmap.upgrade() else {
            return false;
        };
        let Some(mut locked) = pmap.mappings.try_lock() else {
            return true;
        };
        let Some(current) = locked.get(&mapping.address) else {
            return false;
        };
        if !Arc::ptr_eq(&current.page, page) {
            return false;
        }
        // SAFETY: the pmap lock serializes leaf removal.
        if unsafe { arch::paging::unmap_page(pmap.root, VirtAddr::new(mapping.address)) }.is_err() {
            return true;
        }
        locked
            .remove(&mapping.address)
            .expect("mem/pmap: reverse mapping vanished");
        drop(locked);
        shootdown.push(VirtAddr::new(mapping.address));
        removed += 1;
        false
    });

    page.restore_reverse_mappings(mappings);
    if removed != 0 {
        shootdown.commit();
        for _ in 0..removed {
            page.release_mapping();
        }
    }
}

/// Folds hardware accessed and dirty bits back into logical page state.
///
/// The reverse-mapping list is detached once per page rather than sampled per
/// index, so no mapping is skipped when the list is rearranged concurrently.
pub(super) fn harvest_page_states(pages: &[Option<Arc<VmPage>>]) {
    let mut shootdown = Shootdown::global();
    for page in pages.iter().flatten() {
        let mut referenced = false;
        let mut dirty = false;
        let mut mappings = page.take_reverse_mappings();
        mappings.retain(|mapping| {
            let Some(pmap) = mapping.pmap.upgrade() else {
                return false;
            };
            let Some(locked) = pmap.mappings.try_lock() else {
                // Reclaim must never sleep behind a fault or map operation.
                // Treat contention as a recent reference so this page stays
                // resident until a later scan can inspect it safely.
                referenced = true;
                return true;
            };
            let Some(current) = locked.get(&mapping.address) else {
                return false;
            };
            if !Arc::ptr_eq(&current.page, page) {
                return false;
            }
            // SAFETY: the pmap lock serializes this leaf update and the root
            // remains owned by the upgraded pmap.
            if let Ok((was_referenced, was_dirty)) = unsafe {
                arch::paging::take_accessed_dirty(pmap.root, VirtAddr::new(mapping.address))
            } {
                referenced |= was_referenced;
                dirty |= was_dirty;
                if was_referenced || was_dirty {
                    shootdown.push(VirtAddr::new(mapping.address));
                }
            }
            drop(locked);
            true
        });

        page.restore_reverse_mappings(mappings);
        page.record_hardware_state(referenced, dirty);
    }
    shootdown.commit();
}

/// Tears down every hardware mapping before object pages are invalidated.
///
/// Unlike reclaim, truncation and object destruction must wait for pmap locks:
/// returning while an old leaf is reachable could expose data past a new EOF.
pub(super) fn remove_mappings_for_pages(pages: &[Arc<VmPage>]) {
    for page in pages {
        loop {
            let mappings = page.take_reverse_mappings();
            if mappings.is_empty() {
                break;
            }
            let mut shootdown = Shootdown::global();
            let mut removed = 0usize;
            for mapping in mappings {
                let Some(pmap) = mapping.pmap.upgrade() else {
                    continue;
                };
                let mut locked = pmap.mappings.lock();
                let Some(current) = locked.get(&mapping.address) else {
                    continue;
                };
                if !Arc::ptr_eq(&current.page, page) {
                    continue;
                }
                // SAFETY: the pmap lock serializes leaf removal. A tracked
                // mapping must have a leaf; failure is internal corruption.
                unsafe { arch::paging::unmap_page(pmap.root, VirtAddr::new(mapping.address)) }
                    .expect("mem/pmap: tracked mapping has no removable leaf");
                locked
                    .remove(&mapping.address)
                    .expect("mem/pmap: reverse mapping vanished");
                drop(locked);
                shootdown.push(VirtAddr::new(mapping.address));
                removed += 1;
            }
            if removed != 0 {
                shootdown.commit();
                for _ in 0..removed {
                    page.release_mapping();
                }
            }
        }
    }
}

/// Complete virtual address space: MI map plus MD pmap.
pub struct VmSpace {
    map: Mutex<VmMap>,
    pmap: Pmap,
}

impl VmSpace {
    /// Creates an empty user address space.
    pub fn new() -> Result<Arc<Self>> {
        Ok(Arc::new(Self {
            map: Mutex::new(VmMap::new()),
            pmap: Pmap::new()?,
        }))
    }

    /// Activates the address space on the current CPU.
    pub(crate) fn activate(&self) -> Result<()> {
        self.pmap.activate()
    }

    /// Returns the architecture pmap.
    pub fn pmap(&self) -> &Pmap {
        &self.pmap
    }

    /// Returns whether a user range is entirely unmapped.
    pub fn is_free(&self, start: VirtAddr, length: u64) -> bool {
        self.map.lock().is_free(start, length)
    }

    /// Returns the user address space not covered by any mapping.
    pub fn available(&self) -> u64 {
        self.map.lock().available()
    }

    /// Installs a mapping into a currently free range.
    pub fn map(&self, mapping: VmMapping) -> Result<VirtAddr> {
        self.map.lock().map(mapping)
    }

    /// Atomically replaces the fixed range described by `mapping`.
    pub fn replace(&self, mapping: VmMapping) -> Result<VirtAddr> {
        let VmPlacement::Fixed(start) = mapping.placement else {
            return Err(Error::InvalidAddress);
        };
        let mut map = self.map.lock();
        match map.unmap(start, mapping.length) {
            Ok(()) | Err(Error::NotMapped) => {}
            Err(error) => return Err(error),
        }
        // Every tracked leaf must be retired before the old mapping's pages
        // can drop. With the map lock held no fault can repopulate this range.
        self.pmap.remove(start, mapping.length)?;
        map.map(mapping)
    }

    /// Updates the access-pattern hint for a range.
    pub fn advise(&self, start: VirtAddr, length: u64, advice: VmAdvice) -> Result<()> {
        self.map.lock().advise(start, length, advice)
    }

    /// Removes a range from both the map and pmap.
    pub fn unmap(&self, start: VirtAddr, length: u64) -> Result<()> {
        let mut map = self.map.lock();
        map.unmap(start, length)?;
        self.pmap.remove(start, length)
    }

    /// Changes permissions and removes resident translations for reclassification.
    ///
    /// Re-faulting is required when write permission is added: an existing
    /// leaf may still reference a private page shared by a fork, and upgrading
    /// that leaf in place would bypass copy-on-write.
    pub fn protect(&self, start: VirtAddr, length: u64, protection: VmProtection) -> Result<()> {
        let mut map = self.map.lock();
        map.protect(start, length, protection)?;
        self.pmap.remove(start, length)
    }

    /// Resolves and enters one page fault.
    pub fn fault(&self, address: VirtAddr, access: FaultAccess) -> Result<()> {
        let mut map = self.map.lock();
        let resolved = map.resolve_fault(address, access)?;
        let page = resolved.page.clone();
        self.pmap
            .enter(address.align_down(), resolved.page, resolved.protection)?;
        if let Some(fault) = resolved.object_fault
            && !fault.object.validates_fault(fault.index, &page)
        {
            // The map lock prevents another fault from replacing this leaf
            // before cleanup. Truncate either already won (this branch) or
            // will observe the reverse mapping and remove it itself.
            self.pmap.remove(address.align_down(), PAGE_SIZE)?;
            return Err(Error::InvalidAddress);
        }
        super::record_fault(resolved.promoted);
        Ok(())
    }

    /// Validates that a user range is entirely inside the user address window.
    pub fn validate_user(&self, address: VirtAddr, length: usize) -> Result<()> {
        validate_user_range(address, length)
    }

    /// Faults a user address in for `access` and returns its physical address.
    ///
    /// The address-space lock is released before returning, so callers may
    /// safely hold page-cache locks while using the resulting direct-map alias.
    pub fn fault_and_extract(&self, address: VirtAddr, access: FaultAccess) -> Result<PhysAddr> {
        self.fault(address, access)?;
        self.pmap.extract(address).ok_or(Error::NotMapped)
    }

    /// Resolves one user page for `access` and wires its resident frame.
    ///
    /// Returns the logical page and its frame's physical address. The wire
    /// blocks reclamation, which is what makes a direct-map alias of the
    /// returned address safe to hold across an arbitrary-length operation;
    /// [`Self::fault_and_extract`] offers no such guarantee. Release the pin
    /// with `VmPage::release_window` while still holding the page handle.
    pub(super) fn pin_user_page(
        &self,
        address: VirtAddr,
        access: FaultAccess,
    ) -> Result<(Arc<VmPage>, PhysAddr)> {
        // Always fault first, like the copy paths do: a mapping whose
        // protection already grants `access` may still be lazily backed
        // (zero-fill or COW), so its PTE does not necessarily reference the
        // frame the logical page will commit to. Faulting forces that
        // commitment before the alias is taken.
        self.fault(address, access)?;
        let aligned = address.align_down();
        let required = required_protection(access);
        let (page, protection) = self.pmap.mapped_page(aligned).ok_or(Error::NotMapped)?;
        if !protection.contains(required) {
            return Err(Error::Protection);
        }
        // The frame base plus the intra-page offset of `address`: callers
        // expect an alias of exactly this byte onward, matching what a PTE
        // walk of `address` would have produced.
        let physical = page
            .acquire_window()?
            .as_u64()
            .checked_add(address.as_u64() % super::PAGE_SIZE)
            .map(PhysAddr::new)
            .ok_or(Error::InvalidAddress)?;
        Ok((page, physical))
    }
    /// Copies bytes from this address space into a kernel buffer.
    ///
    /// Each page is resolved to its logical page and copied through it, so the
    /// frame stays owned for the whole copy. Reaching for the direct-map alias
    /// of a translation instead would race an unmap that recycles the frame
    /// between the lookup and the copy.
    pub fn read_user(&self, address: VirtAddr, output: &mut [u8]) -> Result<()> {
        self.copy_user(
            address,
            output.len(),
            FaultAccess::Read,
            |page, page_offset, span, copied| {
                page.read(page_offset, &mut output[copied..copied + span])
            },
        )
    }

    /// Copies bytes from a kernel buffer into this address space.
    pub fn write_user(&self, address: VirtAddr, input: &[u8]) -> Result<()> {
        self.copy_user(
            address,
            input.len(),
            FaultAccess::Write,
            |page, page_offset, span, copied| {
                page.write(page_offset, &input[copied..copied + span])
            },
        )
    }

    /// Walks a user range one page at a time, faulting each page in first.
    fn copy_user(
        &self,
        address: VirtAddr,
        length: usize,
        access: FaultAccess,
        mut transfer: impl FnMut(&Arc<VmPage>, usize, usize, usize) -> Result<()>,
    ) -> Result<()> {
        validate_user_range(address, length)?;
        let mut copied = 0usize;
        while copied < length {
            let current = address
                .checked_add(copied as u64)
                .ok_or(Error::InvalidAddress)?;
            let page_offset = (current.as_u64() % PAGE_SIZE) as usize;
            let span = ((PAGE_SIZE as usize) - page_offset).min(length - copied);
            let page = self.resolve_user_page(current, access)?;
            transfer(&page, page_offset, span, copied)?;
            copied += span;
        }
        Ok(())
    }

    /// Resolves one user address to the logical page currently mapped there.
    ///
    /// A mapping already granting `access` is used directly. Anything else is
    /// faulted first, which is what performs copy-on-write promotion; writing
    /// through a mapping that only permits reads would corrupt a page shared
    /// with another address space.
    fn resolve_user_page(&self, address: VirtAddr, access: FaultAccess) -> Result<Arc<VmPage>> {
        let aligned = address.align_down();
        let required = required_protection(access);
        if let Some((page, protection)) = self.pmap.mapped_page(aligned)
            && protection.contains(required)
        {
            return Ok(page);
        }
        self.fault(address, access)?;
        let (page, protection) = self.pmap.mapped_page(aligned).ok_or(Error::NotMapped)?;
        if !protection.contains(required) {
            return Err(Error::Protection);
        }
        Ok(page)
    }

    /// Forks the map with lazy anonymous-overlay COW.
    pub fn fork(&self) -> Result<Arc<Self>> {
        let child_pmap = Pmap::new()?;
        let mut map = self.map.lock();
        self.pmap.write_protect_all()?;
        let child_map = map.fork();
        drop(map);
        Ok(Arc::new(Self {
            map: Mutex::new(child_map),
            pmap: child_pmap,
        }))
    }

    /// Creates a private anonymous object suitable for shared-memory APIs.
    pub fn anonymous_object() -> Arc<VmObject> {
        VmObject::new(ObjectKind::Anonymous)
    }
}

fn validate_user_range(address: VirtAddr, length: usize) -> Result<()> {
    if length == 0 {
        return Ok(());
    }
    let start = address.as_u64();
    let end = start
        .checked_add(length as u64)
        .ok_or(Error::InvalidAddress)?;
    if start < USER_ADDRESS_MIN || end > USER_ADDRESS_MAX || start >= end {
        return Err(Error::InvalidAddress);
    }
    Ok(())
}

fn checked_page_end(start: u64, length: u64) -> Result<u64> {
    let length = length
        .checked_add(PAGE_SIZE - 1)
        .map(|value| value & !(PAGE_SIZE - 1))
        .ok_or(Error::InvalidAddress)?;
    start.checked_add(length).ok_or(Error::InvalidAddress)
}

/// Returns the permission a fault of `access` requires.
fn required_protection(access: FaultAccess) -> VmProtection {
    match access {
        FaultAccess::Read => VmProtection::READ,
        FaultAccess::Write => VmProtection::WRITE,
        FaultAccess::Execute => VmProtection::EXECUTE,
    }
}

fn pmap_flags(protection: VmProtection) -> VmFlags {
    let mut flags = VmFlags::USER;
    if protection.contains(VmProtection::READ) {
        flags |= VmFlags::READ;
    }
    if protection.contains(VmProtection::WRITE) {
        flags |= VmFlags::WRITE;
    }
    if protection.contains(VmProtection::EXECUTE) {
        flags |= VmFlags::EXECUTE;
    }
    flags
}
