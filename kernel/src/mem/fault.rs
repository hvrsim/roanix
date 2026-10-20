//! Restartable page-fault resolution and translation publication.
//!
//! A fault snapshots its mapping and backing identities under the map lock,
//! resolves allocation or paging work without that lock, then revalidates the
//! snapshot before installing a translation. Neighbor mappings are opportunistic:
//! failure to populate them must not prevent resolving the requested page.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use crate::mem::PageRef;
use alloc::sync::{Arc, Weak};

use super::{
    FaultAccess, Pmap, Result, VirtAddr, VmObject, VmPage, VmProtection,
    anon::{AnonymousRef, MapWeak, Slot},
    page::PagePin,
    phys::PageOwnerKind,
};

pub(super) struct FaultSnapshot {
    pub(super) anonymous: Option<MapWeak>,
    pub(super) index: u64,
    pub(super) source: Option<AnonymousRef>,
    pub(super) object: Option<Arc<VmObject>>,
    pub(super) object_index: u64,
    pub(super) protection: VmProtection,
    pub(super) private: bool,
    pub(super) needs_copy: bool,
    pub(super) access: FaultAccess,
    pub(super) window: super::map::FaultWindow,
}

pub(super) struct PreparedFault {
    slot: Option<Slot>,
    pin: PagePin,
    protection: VmProtection,
    replacement: bool,
    object_source: Option<PageRef>,
}

impl FaultSnapshot {
    /// Enter only resident anonymous or object-backed
    /// neighbors, under their owner lock, while the caller holds the map.
    /// Ignore neighbor allocation failures so the center fault can proceed.
    pub(super) fn enter_neighbors(&self, pmap: &Pmap) {
        if let Some(amap) = self.anonymous.as_ref().and_then(Weak::upgrade) {
            amap.with_neighbors(
                self.index - self.window.center as u64,
                self.window.count,
                |upper, owner| {
                    for (offset, page) in upper.iter().enumerate() {
                        if offset == self.window.center {
                            continue;
                        }
                        let Some((page, shared)) = page else { continue };
                        let address =
                            VirtAddr::new(self.window.start + offset as u64 * super::PAGE_SIZE);
                        if self.window.sequential && offset < self.window.center {
                            page.deactivate_locked(owner);
                        } else if pmap.extract(address).is_none()
                            && let Some(pin) = page.pin_neighbor_locked(owner)
                        {
                            let mut protection = self.protection;
                            if *shared || self.needs_copy {
                                protection.remove(VmProtection::WRITE);
                            }
                            let _ = pmap.enter(address, &pin, protection);
                        }
                    }
                    self.enter_lower_neighbors(pmap, Some(upper));
                },
            );
        } else {
            self.enter_lower_neighbors(pmap, None);
        }
    }

    fn enter_lower_neighbors(&self, pmap: &Pmap, upper: Option<&[Option<(PageRef, bool)>]>) {
        let Some(object) = &self.object else { return };
        let shadowed = upper.is_some_and(|pages| pages[self.window.center].is_some());
        if shadowed && !(self.window.sequential && self.window.center != 0) {
            return;
        }
        object.with_neighbors(
            self.object_index - self.window.center as u64,
            self.window.count,
            |pages, owner| {
                for (offset, page) in pages.iter().enumerate() {
                    if offset == self.window.center {
                        continue;
                    }
                    let Some(page) = page else { continue };
                    if self.window.sequential && offset < self.window.center {
                        page.deactivate_locked(owner);
                        continue;
                    }
                    if shadowed || upper.is_some_and(|pages| pages[offset].is_some()) {
                        continue;
                    }
                    let address =
                        VirtAddr::new(self.window.start + offset as u64 * super::PAGE_SIZE);
                    if pmap.extract(address).is_none()
                        && let Some(pin) = page.pin_neighbor_locked(owner)
                    {
                        let mut protection = self.protection;
                        if self.private {
                            protection.remove(VmProtection::WRITE);
                        }
                        let _ = pmap.enter(address, &pin, protection);
                    }
                }
            },
        );
    }

    /// Allocation, copying and every kind of page-in happen with no map or
    /// anonymous index lock held. The pin carries residency into commit.
    pub(super) fn resolve(&self) -> Result<PreparedFault> {
        let write = self.access == FaultAccess::Write;
        let mut protection = self.protection;
        let mut object_source = None;
        let anonymous_owner = self
            .anonymous
            .as_ref()
            .and_then(Weak::upgrade)
            .map(|amap| amap.owner().clone());
        let (page, replacement) = if let Some(source) = &self.source {
            if write && self.private && !source.exclusive() {
                (
                    source.page.copy_to_in(
                        self.index,
                        anonymous_owner
                            .clone()
                            .expect("mem: private fault lacks amap"),
                    )?,
                    true,
                )
            } else {
                if self.private && (self.needs_copy || !source.exclusive()) {
                    protection.remove(VmProtection::WRITE);
                }
                (source.page.clone(), false)
            }
        } else if let Some(object) = &self.object {
            let source = object.fault_page(self.object_index, write && !self.private)?;
            let page = if write && self.private {
                source.copy_to_in(
                    self.index,
                    anonymous_owner
                        .clone()
                        .expect("mem: private object fault lacks amap"),
                )?
            } else {
                if self.private {
                    protection.remove(VmProtection::WRITE);
                }
                source.clone()
            };
            object_source = Some(source);
            (page, write && self.private)
        } else {
            // A first anonymous read establishes an owned zero-filled page,
            // so a subsequent write needs no second fault or global zero PTE.
            (
                VmPage::new_zero_in(
                    self.index,
                    PageOwnerKind::Anonymous,
                    anonymous_owner.expect("mem: zero fault lacks amap"),
                )?,
                true,
            )
        };
        let pin = page.pin_resident()?;
        // Page-in consumes the reserve. Refill it with no owner/map lock
        // held before allocating PTE metadata; swap backing remains retained.
        super::replenish_reserve();
        let slot = (replacement && self.anonymous.is_some())
            .then(|| Slot::new(page.clone()))
            .transpose()?;
        if let Some(anonymous) = self.anonymous.as_ref().and_then(Weak::upgrade) {
            anonymous.prepare(self.index)?;
        }
        Ok(PreparedFault {
            slot,
            pin,
            protection,
            replacement,
            object_source,
        })
    }

    /// The caller holds the same map version used to take this snapshot.
    pub(super) fn commit(
        &self,
        prepared: PreparedFault,
        pmap: &Pmap,
        address: VirtAddr,
    ) -> Result<bool> {
        let install = || {
            let enter = || pmap.enter(address, &prepared.pin, prepared.protection);
            if let (Some(object), Some(source)) = (&self.object, &prepared.object_source) {
                object.with_fault(self.object_index, source, enter)
            } else {
                enter()
            }
        };
        let committed = if let Some(anonymous) = &self.anonymous {
            let Some(anonymous) = anonymous.upgrade() else {
                return Ok(false);
            };
            anonymous.commit(self.index, self.source.as_ref(), prepared.slot, install)?
        } else {
            install()?;
            true
        };
        if committed {
            super::record_fault(prepared.replacement);
        }
        Ok(committed)
    }
}
