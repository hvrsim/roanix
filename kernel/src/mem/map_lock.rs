/*
 * Copyright (c) 1997 Charles D. Cranor and Washington University.
 * Copyright (c) 1991, 1993, The Regents of the University of California.
 *
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * The Mach Operating System project at Carnegie-Mellon University.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)vm_map.c    8.3 (Berkeley) 1/12/94
 * from: Id: uvm_map.c,v 1.1.2.27 1998/02/07 01:16:54 chs Exp
 *
 *
 * Copyright (c) 1987, 1990 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

//! vm_map_lock / vm_map_busy / vm_map_unbusy (NetBSD uvm_map.c).
//! Readers may enter a busy map. Writers acquire the rwlock and then check
//! the busy owner; the busy owner can drop a read hold and regain a writer
//! hold while every other writer waits on the map condition variable.
use super::VmMap;
use crate::sys::{
    event::SequenceEvent,
    sched,
    sync::{Mutex, ReadGuard, RwLock, WriteGuard},
};
use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

pub(super) struct MapLock {
    entries: RwLock<VmMap>,
    busy: Mutex<Option<usize>>,
    changed: SequenceEvent,
}
fn current() -> usize {
    sched::current_thread_opt().map_or(usize::MAX, |thread| thread as usize)
}
impl MapLock {
    pub(super) fn new(map: VmMap) -> Self {
        Self {
            entries: RwLock::new(map),
            busy: Mutex::new(None),
            changed: SequenceEvent::new(),
        }
    }
    pub(super) fn read(&self) -> ReadGuard<'_, VmMap> {
        self.entries.read()
    }
    pub(super) fn write(&self) -> MapWrite<'_> {
        loop {
            let mut entries = self.entries.write();
            let busy = self.busy.lock();
            if busy.is_none_or(|owner| owner == current()) {
                entries.write_locked();
                return MapWrite {
                    lock: self,
                    entries,
                };
            }
            let ticket = self.changed.observe();
            drop(entries);
            drop(busy);
            self.changed.wait(ticket);
        }
    }
    #[expect(dead_code, reason = "UVM nonblocking map-lock entry point")]
    pub(super) fn try_write(&self) -> Option<MapWrite<'_>> {
        let mut entries = self.entries.try_write()?;
        if self.busy.try_lock()?.is_some() {
            return None;
        }
        entries.write_locked();
        Some(MapWrite {
            lock: self,
            entries,
        })
    }
}
pub(super) struct MapWrite<'a> {
    lock: &'a MapLock,
    entries: WriteGuard<'a, VmMap>,
}
impl<'a> MapWrite<'a> {
    /// The token keeps ownership of VM_MAP_BUSY across releasing the rwlock.
    /// Its borrow keeps the map live; its !Send marker enforces LWP ownership.
    #[expect(dead_code, reason = "vm_map_busy API for pageable/wiring operations")]
    pub(super) fn busy(&self) -> MapBusy<'a> {
        let mut busy = self.lock.busy.lock();
        assert!(busy.is_none(), "uvm: map already busy");
        let owner = current();
        *busy = Some(owner);
        MapBusy {
            lock: self.lock,
            owner,
            _nosend: PhantomData,
        }
    }
}
impl Deref for MapWrite<'_> {
    type Target = VmMap;
    fn deref(&self) -> &VmMap {
        &self.entries
    }
}
impl DerefMut for MapWrite<'_> {
    fn deref_mut(&mut self) -> &mut VmMap {
        &mut self.entries
    }
}
pub(super) struct MapBusy<'a> {
    lock: &'a MapLock,
    owner: usize,
    _nosend: PhantomData<*mut ()>,
}
impl Drop for MapBusy<'_> {
    fn drop(&mut self) {
        assert_eq!(
            self.owner,
            current(),
            "uvm: busy map released by another LWP"
        );
        let mut busy = self.lock.busy.lock();
        assert_eq!(*busy, Some(self.owner));
        *busy = None;
        self.lock.changed.notify();
    }
}
