/*
 * Copyright (c) 1997 Charles D. Cranor and Washington University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * from: Id: uvm_fault.c,v 1.1.2.23 1998/02/06 05:29:05 chs Exp
 */

//! Restartable faults: snapshot under the map lock, load without it, then
//! revalidate map and slot ownership before publishing a translation.

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
    /// uvm_fault_upper_lookup / uvm_fault_lower_lookup: enter only resident
    /// neighbors, under their owner lock, while the caller holds the map.
    /// Neighbor allocation failures are deliberately ignored by UVM.
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
