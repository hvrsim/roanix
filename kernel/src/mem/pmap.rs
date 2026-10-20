//! Hardware translations, reverse mappings, and address-space integration.
//!
//! Address-space operations acquire the map lock, anonymous index, object index,
//! translation index, and page reverse list in that order when required. Faults
//! release the map before allocation or I/O and revalidate before publication.
//! Reverse-mapping walks try the translation lock and restart on contention,
//! releasing the reverse list before waiting to preserve the normal lock order.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use crate::mem::PageRef;
use alloc::sync::Arc;
use core::{
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

use crate::{
    arch,
    mem::{PAGE_SIZE, PhysAddr, VirtAddr, VmFlags},
};

use super::{
    Error, FaultAccess, ObjectKind, Result, USER_ADDRESS_MAX, USER_ADDRESS_MIN, VmAdvice, VmMap,
    VmMapping, VmObject, VmPlacement, VmProtection,
    index::{Index, Node},
    page::PagePin,
    pool::{Pool, Pooled},
    tlb::Shootdown,
};

#[cfg(target_arch = "x86_64")]
use crate::sys::sync::{Mutex as MappingLock, MutexGuard as MappingGuard};
#[cfg(target_arch = "riscv64")]
use crate::sys::sync::{RwLock as MappingLock, WriteGuard as MappingGuard};
static MAPPING_POOL: Pool<Node<Mapping>> = Pool::new();

struct Mapping {
    page: PageRef,
    protection: VmProtection,
}

/// One hardware mapping of a logical page, recorded on the page itself.
pub(super) struct ReverseMapping {
    pub(super) pmap: alloc::sync::Weak<PmapInner>,
    pub(super) address: u64,
    next: Option<Pooled<Self>>,
}

static REVERSE_POOL: Pool<ReverseMapping> = Pool::new();

pub(super) fn drain_pools() {
    REVERSE_POOL.drain();
    MAPPING_POOL.drain();
}

#[derive(Default)]
pub(super) struct ReverseMappings {
    head: Option<Pooled<ReverseMapping>>,
}

impl ReverseMappings {
    pub(super) fn allocate(pmap: &Arc<PmapInner>, address: u64) -> Result<Pooled<ReverseMapping>> {
        REVERSE_POOL
            .try_alloc(ReverseMapping {
                pmap: Arc::downgrade(pmap),
                address,
                next: None,
            })
            .map_err(|_| Error::OutOfMemory)
    }

    pub(super) fn push(&mut self, mut mapping: Pooled<ReverseMapping>) {
        mapping.next = self.head.take();
        self.head = Some(mapping);
    }

    pub(super) fn is_empty(&self) -> bool {
        self.head.is_none()
    }

    pub(super) fn remove(
        &mut self,
        pmap: *const PmapInner,
        address: u64,
    ) -> Option<Pooled<ReverseMapping>> {
        let mut link = &mut self.head;
        while let Some(mapping) = link.as_ref() {
            if mapping.pmap.as_ptr() == pmap && mapping.address == address {
                let mut removed = link.take().unwrap();
                *link = removed.next.take();
                return Some(removed);
            }
            link = &mut link.as_mut().unwrap().next;
        }
        None
    }

    fn after(
        &self,
        cursor: Option<&(alloc::sync::Weak<PmapInner>, u64)>,
    ) -> Option<&ReverseMapping> {
        if let Some((pmap, address)) = cursor {
            let mut current = self.head.as_deref();
            while let Some(mapping) = current {
                if mapping.address == *address && alloc::sync::Weak::ptr_eq(&mapping.pmap, pmap) {
                    return mapping.next.as_deref();
                }
                current = mapping.next.as_deref();
            }
            // A V->P operation removed the cursor. Restart the remaining PV
            // list so iteration can continue after the lock handoff.
        }
        self.head.as_deref()
    }
}

impl Drop for ReverseMappings {
    fn drop(&mut self) {
        while let Some(mut mapping) = self.head.take() {
            self.head = mapping.next.take();
        }
    }
}

impl Iterator for ReverseMappings {
    type Item = Pooled<ReverseMapping>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut mapping = self.head.take()?;
        self.head = mapping.next.take();
        Some(mapping)
    }
}

/// Hardware page map owned by one virtual address space.
pub struct Pmap {
    inner: Arc<PmapInner>,
}

pub(super) struct PmapInner {
    root: PhysAddr,
    mappings: MappingLock<Index<Mapping>>,
}

impl PmapInner {
    fn lock(&self) -> MappingGuard<'_, Index<Mapping>> {
        #[cfg(target_arch = "x86_64")]
        {
            self.mappings.lock()
        }
        #[cfg(target_arch = "riscv64")]
        {
            self.mappings.write()
        }
    }
    fn try_lock(&self) -> Option<MappingGuard<'_, Index<Mapping>>> {
        #[cfg(target_arch = "x86_64")]
        {
            self.mappings.try_lock()
        }
        #[cfg(target_arch = "riscv64")]
        {
            self.mappings.try_write()
        }
    }
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
                mappings: MappingLock::new(Index::new(&MAPPING_POOL)),
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
    pub(super) fn enter(
        &self,
        address: VirtAddr,
        pin: &PagePin,
        protection: VmProtection,
    ) -> Result<()> {
        let address = address.align_down();
        let page = pin.page();
        if protection.contains(VmProtection::EXECUTE) {
            let frame = super::phys::phys_to_page(pin.address())
                .expect("mem: executable pin lost its frame");
            page.note_executable(frame);
        }
        let mut mappings = self.inner.lock();
        let current = mappings.get(&address.as_u64());
        if let Some(current) = current
            && current.protection == protection
            && Arc::ptr_eq(&current.page, page)
        {
            // RISC-V may cache invalid leaves and old permissions. If a
            // sibling hart installed this PTE, a local fault still needs
            // SFENCE.VMA or it can keep faulting on the stale entry.
            #[cfg(target_arch = "riscv64")]
            arch::paging::flush_all();
            return Ok(());
        }
        let same_page = current.is_some_and(|current| Arc::ptr_eq(&current.page, page));
        let reserved = mappings.reserve(address.as_u64())?;
        if !same_page {
            page.add_reverse_mapping(&self.inner, address.as_u64())?;
        }
        let paddr = pin.acquire_mapping();
        if protection.contains(VmProtection::WRITE) {
            super::phys::phys_to_page(paddr)
                .expect("mem: mapping lost PFN")
                .mark_unknown();
        }
        let flags = pmap_flags(protection);

        let existing = reserved.is_none();
        let result = if existing {
            // SAFETY: the pmap lock serializes leaf changes and `paddr` remains
            // wired by the new mapping reference.
            unsafe { arch::paging::remap_page(self.inner.root, address, paddr, flags) }
        } else {
            // SAFETY: the pmap lock serializes leaf changes; parent tables are
            // published atomically and the pin retains the mapped frame.
            unsafe { arch::paging::map_page(self.inner.root, address, paddr, flags) }
        };
        if let Err(error) = result {
            if !same_page {
                page.remove_reverse_mapping(Arc::as_ptr(&self.inner), address.as_u64());
            }
            drop(mappings);
            page.release_mapping();
            return Err(match error {
                arch::paging::PagingError::OutOfMemory => Error::OutOfMemory,
                _ => Error::Pmap,
            });
        }

        let previous = mappings.insert(
            address.as_u64(),
            Mapping {
                page: page.clone(),
                protection,
            },
            reserved,
        );
        if !same_page && let Some(previous) = &previous {
            previous
                .page
                .remove_reverse_mapping(Arc::as_ptr(&self.inner), address.as_u64());
        }
        drop(mappings);

        let Some(previous) = previous else {
            // No remote present translation needs revoking. RISC-V harts
            // caching an invalid leaf fence lazily on the resulting fault.
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
        {
            let mut mappings = self.inner.lock();

            loop {
                let mut retired: [Option<PageRef>; Shootdown::INLINE] =
                    [const { None }; Shootdown::INLINE];
                let mut count = 0;
                let mut shootdown = Shootdown::for_root(self.inner.root);
                while count < retired.len() {
                    let Some(key) = mappings.range(start..end).next().map(|(key, _)| *key) else {
                        break;
                    };
                    // SAFETY: the pmap lock excludes concurrent changes to this leaf.
                    if unsafe { arch::paging::unmap_page(self.inner.root, VirtAddr::new(key)) }
                        .is_err()
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
        }
        failure.map_or(Ok(()), Err)
    }

    /// Write-protects every writable mapping before a copy-on-write fork.
    ///
    /// Leaves are downgraded in place, so the pass needs no side table and
    /// cannot fail part way through for want of memory.
    pub fn write_protect_all(&self) -> Result<()> {
        let mut shootdown = Shootdown::for_root(self.inner.root);
        let mut failure = None;

        {
            let mut mappings = self.inner.lock();
            for (address, mapping) in mappings.iter_mut() {
                if !mapping.protection.contains(VmProtection::WRITE)
                    || mapping.page.write_windows.load(Ordering::Acquire) != 0
                {
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
        }
        shootdown.commit();
        failure.map_or(Ok(()), Err)
    }

    /// Revokes permissions without discarding resident pages. Permissions
    /// absent from the old PTE remain absent: adding WRITE must still fault
    /// through COW classification before it can expose a writable leaf.
    fn restrict(&self, start: VirtAddr, length: u64, protection: VmProtection) -> Result<()> {
        let end = checked_page_end(start.as_u64(), length)?;
        let remove = self
            .inner
            .lock()
            .range(start.as_u64()..end)
            .any(|(_, mapping)| !(mapping.protection & protection).contains(VmProtection::READ));
        if remove {
            return self.remove(start, length);
        }
        let mut shootdown = Shootdown::for_root(self.inner.root);
        let mut failure = None;
        {
            let mut mappings = self.inner.lock();
            for (address, mapping) in mappings.range_mut(start.as_u64()..end) {
                let restricted = mapping.protection & protection;
                if restricted == mapping.protection {
                    continue;
                }
                // SAFETY: the mapping lock excludes leaf changes; the existing
                // mapping reference pins the frame until this update completes.
                if unsafe {
                    arch::paging::protect_page(
                        self.inner.root,
                        VirtAddr::new(*address),
                        pmap_flags(restricted),
                    )
                }
                .is_err()
                {
                    failure = Some(Error::Pmap);
                    break;
                }
                mapping.protection = restricted;
                shootdown.push(VirtAddr::new(*address));
            }
        }
        shootdown.commit();
        failure.map_or(Ok(()), Err)
    }

    /// Returns the logical page and permissions currently mapped at `address`.
    pub(super) fn mapped_page(&self, address: VirtAddr) -> Option<(PageRef, VmProtection)> {
        self.inner
            .lock()
            .get(&address.as_u64())
            .map(|mapping| (mapping.page.clone(), mapping.protection))
    }

    /// Returns the physical address currently mapped at `address`.
    pub fn extract(&self, address: VirtAddr) -> Option<PhysAddr> {
        let _mappings = self.inner.lock();
        // SAFETY: this pmap owns the hierarchy and its mapping lock excludes
        // concurrent page-table writes for the duration of the walk.
        unsafe { arch::paging::translate(self.inner.root, address) }
    }

    /// Returns the number of tracked resident mappings.
    pub fn resident_count(&self) -> usize {
        self.inner.lock().len()
    }
}

impl Drop for PmapInner {
    fn drop(&mut self) {
        super::retire_address_space_root(self.root);
        let identity = ptr::from_ref(self);
        {
            let mappings = core::mem::replace(self.mappings.get_mut(), Index::new(&MAPPING_POOL));
            let mut shootdown = Shootdown::for_root(self.root);
            for (address, mapping) in mappings.iter() {
                // SAFETY: `&mut self` excludes concurrent pmap operations.
                let _ = unsafe { arch::paging::unmap_page(self.root, VirtAddr::new(*address)) };
                mapping.page.remove_reverse_mapping(identity, *address);
                shootdown.push(VirtAddr::new(*address));
            }
            shootdown.commit();
            for (_, mapping) in mappings.into_entries() {
                mapping.page.release_mapping();
            }
        }
        // SAFETY: all user leaves were removed, the retirement barrier has
        // released every borrowed or outgoing hardware reference, and the shared
        // kernel half is not freed by the architecture routine.
        unsafe { arch::paging::destroy_user_root(self.root) };
    }
}

/// Reference the translation map and
/// try its lock with PV held, release PV, and if contended wait then restart.
/// The Arc outlives both guards, so its final destructor never runs at IPL_VM.
fn visit_reverse_mappings(
    page: &PageRef,
    mut visit: impl FnMut(&PmapInner, &mut Index<Mapping>, u64) -> bool,
) {
    let mut cursor = None;
    loop {
        let pmap;
        let mut reverse = page.reverse_mappings();
        let Some(mapping) = reverse.after(cursor.as_ref()) else {
            break;
        };
        let address = mapping.address;
        let identity = mapping.pmap.as_ptr();
        let next_cursor = (mapping.pmap.clone(), address);
        pmap = match mapping.pmap.upgrade() {
            Some(pmap) => pmap,
            None => {
                let retired = reverse.remove(identity, address);
                drop(reverse);
                drop(retired);
                cursor = None;
                continue;
            }
        };
        let locked = pmap.try_lock();
        drop(reverse);
        let Some(mut locked) = locked else {
            drop(pmap.lock());
            cursor = None;
            continue;
        };
        if !locked
            .get(&address)
            .is_some_and(|mapping| Arc::ptr_eq(&mapping.page, page))
        {
            page.remove_reverse_mapping(identity, address);
            cursor = None;
            continue;
        }
        cursor = visit(&pmap, &mut locked, address).then_some(next_cursor);
    }
}

/// Remove every mapping and complete its TLB visibility before returning.
/// The MI owner stays held across revocation and PG_BUSY/identity changes.
pub(super) fn remove_all_mappings(page: &PageRef) {
    if page.reverse_mappings().is_empty() {
        return;
    }
    visit_reverse_mappings(page, |pmap, mappings, address| {
        // SAFETY: direct PV->pmap handoff retained the mapping identity and
        // this pmap lock exclusively owns its leaf and table hierarchy.
        unsafe { arch::paging::unmap_page(pmap.root, VirtAddr::new(address)) }
            .expect("mem: tracked PV mapping lacks a leaf");
        mappings.remove(&address).expect("mem: PV mapping vanished");
        page.remove_reverse_mapping(ptr::from_ref(pmap), address);
        let mut shootdown = Shootdown::for_root(pmap.root);
        shootdown.push(VirtAddr::new(address));
        shootdown.commit();
        page.release_mapping();
        false
    });
}

pub(super) fn harvest_page_state_locked(page: &PageRef, owner: &mut super::owner::OwnerWrite<'_>) {
    let mut referenced = false;
    let mut dirty = false;
    let mut shootdown = Shootdown::global();
    visit_reverse_mappings(page, |pmap, _, address| {
        // SAFETY: the handoff holds the pmap lock and retains its root; the
        // MI owner keeps this frame's identity stable through the shootdown.
        let hardware =
            unsafe { arch::paging::take_accessed_dirty(pmap.root, VirtAddr::new(address)) };
        if let Ok((was_referenced, was_dirty)) = hardware {
            referenced |= was_referenced;
            dirty |= was_dirty;
            if was_referenced || was_dirty {
                shootdown.push(VirtAddr::new(address));
            }
        }
        true
    });
    shootdown.commit();
    page.record_hardware_state_locked(referenced, dirty, owner);
}

pub(super) fn remove_mappings_for_pages(pages: &[PageRef]) {
    for page in pages {
        remove_all_mappings(page);
    }
}

pub(super) struct UserPagePin<'a> {
    pin: PagePin,
    writers: Option<&'a AtomicUsize>,
}

impl Drop for UserPagePin<'_> {
    fn drop(&mut self) {
        if let Some(writers) = self.writers {
            self.pin
                .page()
                .write_windows
                .fetch_sub(1, Ordering::Release);
            writers.fetch_sub(1, Ordering::Release);
        }
    }
}

/// Complete virtual address space: MI map plus MD pmap.
pub struct VmSpace {
    map: super::map_lock::MapLock,
    pmap: Pmap,
    write_windows: AtomicUsize,
}

impl VmSpace {
    /// Creates an empty user address space.
    pub fn new() -> Result<Arc<Self>> {
        Ok(Arc::new(Self {
            map: super::map_lock::MapLock::new(VmMap::new()?),
            pmap: Pmap::new()?,
            write_windows: AtomicUsize::new(0),
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
        self.map.read().is_free(start, length)
    }

    /// Returns the user address space not covered by any mapping.
    pub fn available(&self) -> u64 {
        self.map.read().available()
    }

    /// Installs a mapping into a currently free range.
    pub fn map(&self, mapping: VmMapping) -> Result<VirtAddr> {
        self.map.write().map(mapping)
    }

    /// Atomically replaces the fixed range described by `mapping`.
    pub fn replace(&self, mapping: VmMapping) -> Result<VirtAddr> {
        let VmPlacement::Fixed(start) = mapping.placement else {
            return Err(Error::InvalidAddress);
        };
        let mut map = self.map.write();
        let detached = match map.unmap(start, mapping.length) {
            Ok(detached) => Some(detached),
            Err(Error::NotMapped) => None,
            Err(error) => return Err(error),
        };
        // Every tracked leaf must be retired before the old mapping's pages
        // can drop. With the map lock held no fault can repopulate this range.
        self.pmap.remove(start, mapping.length)?;
        drop(detached);
        map.map(mapping)
    }

    /// Updates the access-pattern hint for a range.
    pub fn advise(&self, start: VirtAddr, length: u64, advice: VmAdvice) -> Result<()> {
        self.map.write().advise(start, length, advice)
    }

    /// Removes a range from both the map and pmap.
    pub fn unmap(&self, start: VirtAddr, length: u64) -> Result<()> {
        let mut map = self.map.write();
        let detached = map.unmap(start, length)?;
        self.pmap.remove(start, length)?;
        drop(detached);
        Ok(())
    }

    /// Changes permissions, retaining resident translations where possible.
    ///
    /// Re-faulting is required when write permission is added: an existing
    /// leaf may still reference a private page shared by a fork, and upgrading
    /// that leaf in place would bypass copy-on-write.
    pub fn protect(&self, start: VirtAddr, length: u64, protection: VmProtection) -> Result<()> {
        let mut map = self.map.write();
        map.protect(start, length, protection)?;
        self.pmap.restrict(start, length, protection)
    }

    /// Resolves and enters one page fault.
    pub fn fault(&self, address: VirtAddr, access: FaultAccess) -> Result<()> {
        let mut narrow = false;
        loop {
            let ticket = super::prepare_memory_wait();
            let map = self.map.read();
            let Some(snapshot) = map.fault_snapshot(address, access)? else {
                drop(map);
                let result = self.map.write().prepare_copy(address, access);
                match result {
                    Ok(()) => {}
                    Err(Error::OutOfMemory) => super::wait_for_memory(ticket)?,
                    Err(error) => return Err(error),
                }
                continue;
            };
            if !narrow {
                snapshot.enter_neighbors(&self.pmap);
                narrow = true;
            }
            let timestamp = map.timestamp();
            drop(map);
            let prepared = snapshot.resolve();
            let map = self.map.read();
            // Even an I/O error belongs to the old mapping if its version
            // changed. Retry before reporting it or installing any PTE.
            if map.timestamp() != timestamp {
                continue;
            }
            match prepared
                .and_then(|prepared| snapshot.commit(prepared, &self.pmap, address.align_down()))
            {
                Ok(true) => return Ok(()),
                Ok(false) | Err(Error::NotMapped) => {}
                Err(Error::OutOfMemory) => {
                    drop(map);
                    drop(snapshot);
                    super::wait_for_memory(ticket)?;
                }
                Err(error) => return Err(error),
            }
        }
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
    /// [`Self::fault_and_extract`] offers no such guarantee. Dropping the guard
    /// releases the wire and the writable-window ownership registration.
    pub(super) fn pin_user_page(
        &self,
        address: VirtAddr,
        access: FaultAccess,
    ) -> Result<(UserPagePin<'_>, PhysAddr)> {
        loop {
            let page = self.resolve_user_page(address, access)?;
            let pin = page.pin_resident()?;
            let _map = self.map.read();
            if !self
                .pmap
                .mapped_page(address.align_down())
                .is_some_and(|(mapped, protection)| {
                    Arc::ptr_eq(&page, &mapped) && protection.contains(required_protection(access))
                })
            {
                continue;
            }
            let writers = (access == FaultAccess::Write).then(|| {
                page.record_hardware_state(false, true);
                page.write_windows.fetch_add(1, Ordering::Relaxed);
                self.write_windows.fetch_add(1, Ordering::Relaxed);
                &self.write_windows
            });
            let physical = PhysAddr::new(pin.address().as_u64() + address.as_u64() % PAGE_SIZE);
            return Ok((UserPagePin { pin, writers }, physical));
        }
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
        mut transfer: impl FnMut(&PageRef, usize, usize, usize) -> Result<()>,
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
            let _pin = page.pin_resident()?;
            let _map = self.map.read();
            if !self
                .pmap
                .mapped_page(current.align_down())
                .is_some_and(|(mapped, protection)| {
                    Arc::ptr_eq(&page, &mapped) && protection.contains(required_protection(access))
                })
            {
                continue;
            }
            // The pin excludes page-in during the copy; the map guard keeps
            // fork and protection changes from invalidating its COW decision.
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
    fn resolve_user_page(&self, address: VirtAddr, access: FaultAccess) -> Result<PageRef> {
        let aligned = address.align_down();
        let required = required_protection(access);
        loop {
            if let Some((page, protection)) = self.pmap.mapped_page(aligned)
                && protection.contains(required)
            {
                return Ok(page);
            }
            // Fork or reclaim can change the translation immediately after
            // a successful fault. Only the next fault can establish an error.
            self.fault(address, access)?;
        }
    }

    /// Forks the map with lazy anonymous-overlay COW.
    pub fn fork(&self) -> Result<Arc<Self>> {
        let child_pmap = Pmap::new()?;
        let mut map = self.map.write();
        let child_map = map.fork(self.write_windows.load(Ordering::Acquire) != 0)?;
        self.pmap.write_protect_all()?;
        drop(map);
        Ok(Arc::new(Self {
            map: super::map_lock::MapLock::new(child_map),
            pmap: child_pmap,
            write_windows: AtomicUsize::new(0),
        }))
    }

    /// Creates a private anonymous object suitable for shared-memory APIs.
    pub fn anonymous_object() -> Arc<VmObject> {
        VmObject::new(ObjectKind::Anonymous)
    }
}

impl Drop for VmSpace {
    fn drop(&mut self) {
        // Release anonymous references only after hardware mappings are gone.
        // The final VmSpace owner excludes user execution and new faults.
        self.pmap
            .remove(
                VirtAddr::new(USER_ADDRESS_MIN),
                USER_ADDRESS_MAX - USER_ADDRESS_MIN,
            )
            .expect("mem: failed to detach final address-space mappings");
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
