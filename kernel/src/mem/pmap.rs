//! Machine-independent pmap ownership and address-space integration.

use alloc::{collections::BTreeMap, sync::Arc, vec::Vec};
use core::ptr;

use crate::{
    arch,
    mem::{PAGE_SIZE, PhysAddr, VirtAddr, VmFlags, align_up},
    sys::sync::Mutex,
};

use super::{
    Error, FaultAccess, ObjectKind, Result, USER_ADDRESS_MAX, USER_ADDRESS_MIN, VmInheritance,
    VmMap, VmObject, VmPage, VmProtection,
};

struct Mapping {
    page: Arc<VmPage>,
    protection: VmProtection,
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
    pub fn enter(
        &self,
        address: VirtAddr,
        page: Arc<VmPage>,
        protection: VmProtection,
    ) -> Result<()> {
        let address = address.align_down();
        let paddr = page.acquire_mapping()?;
        let flags = pmap_flags(protection);
        let mut mappings = self.inner.mappings.lock();

        let result = if mappings.contains_key(&address.as_u64()) {
            // SAFETY: the pmap lock serializes leaf changes and `paddr` remains
            // wired by the new mapping reference.
            unsafe { arch::paging::remap_page(self.inner.root, address, paddr, flags) }
        } else {
            // SAFETY: the pmap lock serializes page-table construction and the
            // page remains resident through its mapping reference.
            unsafe { arch::paging::map_page(self.inner.root, address, paddr, flags) }
        };
        if result.is_err() {
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
        if let Some(previous) = &previous
            && !same_page
        {
            previous
                .page
                .remove_reverse_mapping(Arc::as_ptr(&self.inner), address.as_u64());
            page.add_reverse_mapping(&self.inner, address.as_u64());
        } else if previous.is_none() {
            page.add_reverse_mapping(&self.inner, address.as_u64());
        }
        drop(mappings);

        if let Some(previous) = previous {
            if Arc::ptr_eq(&previous.page, &page) {
                page.release_mapping();
                super::publish_permission_shootdown();
            } else {
                super::retire_mapping(previous.page);
            }
        }
        Ok(())
    }

    /// Removes a virtual range.
    pub fn remove(&self, start: VirtAddr, length: u64) -> Result<()> {
        let start = start.align_down().as_u64();
        let end = start
            .checked_add(align_up(length, PAGE_SIZE))
            .ok_or(Error::InvalidAddress)?;
        let mut mappings = self.inner.mappings.lock();
        let keys: Vec<u64> = mappings.range(start..end).map(|(key, _)| *key).collect();
        let mut retired = Vec::with_capacity(keys.len());
        for key in keys {
            // SAFETY: the pmap lock excludes concurrent changes to this leaf.
            unsafe { arch::paging::unmap_page(self.inner.root, VirtAddr::new(key)) }
                .map_err(|_| Error::Pmap)?;
            let mapping = mappings
                .remove(&key)
                .expect("mem/pmap: removed mapping vanished");
            mapping
                .page
                .remove_reverse_mapping(Arc::as_ptr(&self.inner), key);
            retired.push(mapping.page);
        }
        drop(mappings);
        super::retire_mappings(retired);
        Ok(())
    }

    /// Downgrades mappings in a virtual range.
    pub fn protect(&self, start: VirtAddr, length: u64, protection: VmProtection) -> Result<()> {
        let start = start.align_down().as_u64();
        let end = start
            .checked_add(align_up(length, PAGE_SIZE))
            .ok_or(Error::InvalidAddress)?;
        let mut mappings = self.inner.mappings.lock();
        for (address, mapping) in mappings.range_mut(start..end) {
            if mapping.protection == protection {
                continue;
            }
            let paddr = mapping.page.acquire_mapping()?;
            // SAFETY: the existing mapping pins the page and the pmap lock
            // serializes the leaf update.
            let result = unsafe {
                arch::paging::remap_page(
                    self.inner.root,
                    VirtAddr::new(*address),
                    paddr,
                    pmap_flags(protection),
                )
            };
            mapping.page.release_mapping();
            result.map_err(|_| Error::Pmap)?;
            mapping.protection = protection;
        }
        drop(mappings);
        super::publish_permission_shootdown();
        Ok(())
    }

    /// Write-protects every writable mapping before a COW fork.
    pub fn write_protect_all(&self) -> Result<()> {
        let mut mappings = self.inner.mappings.lock();
        let mut pending: Vec<(u64, Arc<VmPage>, PhysAddr, VmProtection, VmProtection)> =
            Vec::new();
        for (address, mapping) in mappings.iter() {
            if !mapping.protection.contains(VmProtection::WRITE) {
                continue;
            }
            let mut protection = mapping.protection;
            protection.remove(VmProtection::WRITE);
            let paddr = match mapping.page.acquire_mapping() {
                Ok(paddr) => paddr,
                Err(error) => {
                    for (_, page, _, _, _) in pending {
                        page.release_mapping();
                    }
                    return Err(error);
                }
            };
            pending.push((
                *address,
                mapping.page.clone(),
                paddr,
                mapping.protection,
                protection,
            ));
        }

        let mut applied = 0usize;
        for (address, _, paddr, _, protection) in &pending {
            // SAFETY: the existing mapping pins the page and the pmap lock
            // serializes the leaf update.
            let result = unsafe {
                arch::paging::remap_page(
                    self.inner.root,
                    VirtAddr::new(*address),
                    *paddr,
                    pmap_flags(*protection),
                )
            };
            if result.is_err() {
                for (address, _, paddr, original, _) in &pending[..applied] {
                    // SAFETY: these leaves were changed above while this lock
                    // remained held and their pages are still pinned.
                    unsafe {
                        arch::paging::remap_page(
                            self.inner.root,
                            VirtAddr::new(*address),
                            *paddr,
                            pmap_flags(*original),
                        )
                    }
                    .expect("mem/pmap: failed to roll back fork protection");
                    mappings
                        .get_mut(address)
                        .expect("mem/pmap: rollback mapping vanished")
                        .protection = *original;
                }
                for (_, page, _, _, _) in pending {
                    page.release_mapping();
                }
                return Err(Error::Pmap);
            }
            mappings
                .get_mut(address)
                .expect("mem/pmap: fork mapping vanished")
                .protection = *protection;
            applied += 1;
        }
        for (_, page, _, _, _) in pending {
            page.release_mapping();
        }
        drop(mappings);
        if applied != 0 {
            super::publish_permission_shootdown();
        }
        Ok(())
    }

    /// Returns the physical address currently mapped at `address`.
    pub fn extract(&self, address: VirtAddr) -> Option<PhysAddr> {
        // SAFETY: this pmap owns the hierarchy for the duration of the walk.
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
        for (address, mapping) in &mappings {
            // SAFETY: `&mut self` excludes concurrent pmap operations.
            let _ = unsafe { arch::paging::unmap_page(self.root, VirtAddr::new(*address)) };
            mapping
                .page
                .remove_reverse_mapping(self as *const PmapInner, *address);
        }
        if !mappings.is_empty() {
            super::publish_permission_shootdown();
        }
        for (_, mapping) in mappings {
            mapping.page.release_mapping();
        }
        // SAFETY: all user leaves were removed, this root is inactive because
        // every owning `VmSpace` reference has been released, and the shared
        // kernel half is not freed by the architecture routine.
        unsafe { arch::paging::destroy_user_root(self.root) };
    }
}

pub(super) fn remove_all_mappings(page: &Arc<VmPage>) {
    remove_mappings_for_pages(core::slice::from_ref(page));
}

pub(super) fn harvest_page_states(pages: &[Option<Arc<VmPage>>]) {
    let mut cleared = false;
    for page in pages.iter().flatten() {
        let mut referenced = false;
        let mut dirty = false;
        let mut index = 0usize;
        while let Some(mapping) = page.reverse_mapping(index) {
            index += 1;
            let Some(pmap) = mapping.pmap.upgrade() else {
                continue;
            };
            let mappings = pmap.mappings.lock();
            let Some(current) = mappings.get(&mapping.address) else {
                continue;
            };
            if !Arc::ptr_eq(&current.page, page) {
                continue;
            }
            // SAFETY: the pmap lock serializes this leaf update and the root
            // remains owned by the upgraded pmap.
            if let Ok((was_referenced, was_dirty)) = unsafe {
                arch::paging::take_accessed_dirty(pmap.root, VirtAddr::new(mapping.address))
            } {
                referenced |= was_referenced;
                dirty |= was_dirty;
                cleared |= was_referenced || was_dirty;
            }
        }
        page.record_hardware_state(referenced, dirty);
    }
    if cleared {
        super::publish_permission_shootdown();
    }
}

pub(super) fn remove_mappings_for_pages(pages: &[Arc<VmPage>]) {
    for page in pages {
        let mut removed = 0u32;
        while let Some(mapping) = page.first_reverse_mapping() {
            let Some(pmap) = mapping.pmap.upgrade() else {
                page.remove_reverse_mapping(mapping.pmap.as_ptr(), mapping.address);
                continue;
            };
            let mut mappings = pmap.mappings.lock();
            let Some(current) = mappings.get(&mapping.address) else {
                page.remove_reverse_mapping(Arc::as_ptr(&pmap), mapping.address);
                continue;
            };
            if !Arc::ptr_eq(&current.page, page) {
                page.remove_reverse_mapping(Arc::as_ptr(&pmap), mapping.address);
                continue;
            }
            // SAFETY: the pmap lock serializes leaf removal.
            if unsafe { arch::paging::unmap_page(pmap.root, VirtAddr::new(mapping.address)) }
                .is_err()
            {
                break;
            }
            let removed_mapping = mappings
                .remove(&mapping.address)
                .expect("mem/pmap: reverse mapping vanished");
            debug_assert!(Arc::ptr_eq(&removed_mapping.page, page));
            page.remove_reverse_mapping(Arc::as_ptr(&pmap), mapping.address);
            removed = removed.saturating_add(1);
        }
        if removed != 0 {
            super::publish_permission_shootdown();
            for _ in 0..removed {
                page.release_mapping();
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

    /// Finds a free page-aligned user range using first fit.
    pub fn find_space(&self, hint: VirtAddr, length: u64) -> Result<VirtAddr> {
        self.map.lock().find_space(hint, length)
    }

    /// Maps private anonymous zero-fill memory.
    pub fn map_anonymous(
        &self,
        start: VirtAddr,
        length: u64,
        protection: VmProtection,
        maximum_protection: VmProtection,
        inheritance: VmInheritance,
    ) -> Result<()> {
        self.map
            .lock()
            .map_anonymous(start, length, protection, maximum_protection, inheritance)
    }

    /// Maps an object into the address space.
    #[allow(clippy::too_many_arguments)]
    pub fn map_object(
        &self,
        start: VirtAddr,
        length: u64,
        object: Arc<VmObject>,
        object_offset: u64,
        protection: VmProtection,
        maximum_protection: VmProtection,
        inheritance: VmInheritance,
        private: bool,
    ) -> Result<()> {
        self.map.lock().map_object(
            start,
            length,
            object,
            object_offset,
            protection,
            maximum_protection,
            inheritance,
            private,
        )
    }

    /// Removes a range from both the map and pmap.
    pub fn unmap(&self, start: VirtAddr, length: u64) -> Result<()> {
        let mut map = self.map.lock();
        map.unmap(start, length)?;
        self.pmap.remove(start, length)
    }

    /// Changes map and pmap permissions.
    pub fn protect(&self, start: VirtAddr, length: u64, protection: VmProtection) -> Result<()> {
        let mut map = self.map.lock();
        map.protect(start, length, protection)?;
        self.pmap.remove(start, length)
    }

    /// Resolves and enters one page fault.
    pub fn fault(&self, address: VirtAddr, access: FaultAccess) -> Result<()> {
        let mut map = self.map.lock();
        let resolved = map.resolve_fault(address, access)?;
        self.pmap
            .enter(address.align_down(), resolved.page, resolved.protection)?;
        super::record_fault(resolved.promoted);
        Ok(())
    }

    /// Copies bytes from this address space into a kernel buffer.
    pub fn read_user(&self, address: VirtAddr, output: &mut [u8]) -> Result<()> {
        validate_user_range(address, output.len())?;
        let mut copied = 0usize;
        while copied < output.len() {
            let current = address
                .checked_add(copied as u64)
                .ok_or(Error::InvalidAddress)?;
            self.fault(current, FaultAccess::Read)?;
            let physical = self.pmap.extract(current).ok_or(Error::NotMapped)?;
            let count =
                ((PAGE_SIZE - (current.as_u64() % PAGE_SIZE)) as usize).min(output.len() - copied);
            let source = super::phys_to_virt(physical).as_ptr::<u8>();

            // SAFETY: the fault above established a readable resident mapping,
            // the HHDM aliases that physical range, and `count` stays within
            // the current page and the live output slice.
            unsafe {
                ptr::copy_nonoverlapping(source, output.as_mut_ptr().add(copied), count);
            }
            copied += count;
        }
        Ok(())
    }

    /// Copies bytes from a kernel buffer into this address space.
    pub fn write_user(&self, address: VirtAddr, input: &[u8]) -> Result<()> {
        validate_user_range(address, input.len())?;
        let mut copied = 0usize;
        while copied < input.len() {
            let current = address
                .checked_add(copied as u64)
                .ok_or(Error::InvalidAddress)?;
            self.fault(current, FaultAccess::Write)?;
            let physical = self.pmap.extract(current).ok_or(Error::NotMapped)?;
            let count =
                ((PAGE_SIZE - (current.as_u64() % PAGE_SIZE)) as usize).min(input.len() - copied);
            let destination = super::phys_to_virt(physical).as_mut_ptr::<u8>();

            // SAFETY: the write fault above established an exclusively
            // writable resident page for this mapping. The HHDM aliases that
            // physical range and `count` remains within both source and page.
            unsafe {
                ptr::copy_nonoverlapping(input.as_ptr().add(copied), destination, count);
            }
            copied += count;
        }
        Ok(())
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
