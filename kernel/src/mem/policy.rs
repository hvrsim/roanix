/*-
 * Copyright (c) 2019, 2020 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Andrew Doran.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
 *	@(#)vm_pageout.c        8.5 (Berkeley) 2/14/94
 * from: Id: uvm_pdaemon.c,v 1.1.2.32 1998/02/06 05:26:30 chs Exp
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

//! NetBSD UVM clock policy (99a671f895b7, uvm_pdpolicy_clock.c).
//! Lock order: page owner -> policy -> PFN interlock. A scan drops policy
//! before trying the owner, then revalidates identity, intent and queue state.
//! Per-CPU pending queues store PFN pointers, whose database lifetime is
//! permanent. Free-list and policy links are independent.
use super::{
    PageRef,
    phys::{Page, PageFlags, PageOwnerKind, PageQueue},
    pmap,
};
use crate::{
    arch,
    sys::{
        clock,
        smp::{self, IrqSpinLock},
        sync::{Mutex, Once},
    },
};
use core::{
    ptr,
    sync::atomic::{AtomicU32, AtomicUsize, Ordering},
};
use intrusive_collections::{LinkedList, LinkedListLink, UnsafeRef, intrusive_adapter};

const QUEUE_SIZE: usize = 128;
const INTENT_MASK: u32 = 3;
const ACTIVATE: u32 = 0;
const DEACTIVATE: u32 = 1;
const ENQUEUE: u32 = 2;
const DEQUEUE: u32 = 3;
const INTENT_SET: u32 = 4;
const INTENT_QUEUED: u32 = 8;
const INACTIVE: u32 = 0x10;
const ACTIVE: u32 = 0x20;
const IDLE_INTERVAL_NS: u64 = 125_000_000;

intrusive_adapter!(PageAdapter = UnsafeRef<Page>: Page { policy_link: LinkedListLink });
struct ClockState {
    active: LinkedList<PageAdapter>,
    inactive: LinkedList<PageAdapter>,
    nactive: usize,
    ninactive: usize,
}
impl ClockState {
    fn new() -> Self {
        Self {
            active: LinkedList::new(PageAdapter::NEW),
            inactive: LinkedList::new(PageAdapter::NEW),
            nactive: 0,
            ninactive: 0,
        }
    }
    fn list(&mut self, active: bool) -> &mut LinkedList<PageAdapter> {
        if active {
            &mut self.active
        } else {
            &mut self.inactive
        }
    }
    fn remove(&mut self, page: &Page, flags: u32) {
        if flags & ACTIVE != 0 {
            // SAFETY: policy lock and actual membership bits prove membership.
            unsafe {
                self.active
                    .cursor_mut_from_ptr(ptr::from_ref(page))
                    .remove();
            }
            self.nactive -= 1;
        } else if flags & INACTIVE != 0 {
            // SAFETY: policy lock and actual membership bits prove membership.
            unsafe {
                self.inactive
                    .cursor_mut_from_ptr(ptr::from_ref(page))
                    .remove();
            }
            self.ninactive -= 1;
        }
    }
    /// Caller holds policy and the page interlock. QUEUED is cleared only by
    /// the CPU that consumes its pending pointer, not by scanner realization.
    fn realize(&mut self, page: &Page) -> bool {
        let flags = page.policy_flags.load(Ordering::Relaxed);
        if flags & INTENT_SET == 0 {
            return false;
        }
        let intent = flags & INTENT_MASK;
        if intent == DEACTIVATE && flags & INACTIVE != 0 {
            page.policy_flags
                .store(flags & !INTENT_SET, Ordering::Relaxed);
            return true;
        }
        self.remove(page, flags);
        let mut flags = flags & !(ACTIVE | INACTIVE | INTENT_SET);
        if intent != DEQUEUE {
            // SAFETY: real PFN entries are permanent; realization exclusively
            // owns the detached policy link until the list takes it.
            let entry = unsafe { UnsafeRef::from_raw(ptr::from_ref(page)) };
            if intent == DEACTIVATE {
                self.inactive.push_back(entry);
                self.ninactive += 1;
                flags |= INACTIVE;
            } else {
                self.active.push_back(entry);
                self.nactive += 1;
                flags |= ACTIVE;
            }
        }
        page.policy_flags.store(flags, Ordering::Release);
        true
    }
    fn target(&self) -> usize {
        ((self.nactive + self.ninactive) * 33 / 100).max(super::free_target() as usize + 1)
    }
}
static POLICY: Once<Mutex<ClockState>> = Once::new();
fn global() -> &'static Mutex<ClockState> {
    POLICY.call_once(|| Mutex::new(ClockState::new()))
}
static ANON_PAGES: AtomicUsize = AtomicUsize::new(0);
static FILE_PAGES: AtomicUsize = AtomicUsize::new(0);
static EXEC_PAGES: AtomicUsize = AtomicUsize::new(0);

struct Pending {
    pages: [Option<&'static Page>; QUEUE_SIZE],
    head: usize,
    last_flush: u64,
}
impl Pending {
    const fn new() -> Self {
        Self {
            pages: [None; QUEUE_SIZE],
            head: QUEUE_SIZE,
            last_flush: 0,
        }
    }
}
static PENDING: [IrqSpinLock<Pending>; super::tlb::MAX_TLB_CPUS] =
    [const { IrqSpinLock::new(Pending::new()) }; super::tlb::MAX_TLB_CPUS];

pub(super) fn account(kind: PageOwnerKind, executable: bool, delta: isize) {
    let counter = if kind == PageOwnerKind::File && executable {
        &EXEC_PAGES
    } else if kind == PageOwnerKind::File {
        &FILE_PAGES
    } else {
        &ANON_PAGES
    };
    if delta > 0 {
        counter.fetch_add(delta as usize, Ordering::Relaxed);
    } else {
        counter.fetch_sub((-delta) as usize, Ordering::Relaxed);
    }
}

pub(super) fn queue(page: &Page) -> PageQueue {
    let flags = page.policy_flags.load(Ordering::Acquire);
    if flags & INTENT_SET != 0 {
        match flags & INTENT_MASK {
            DEQUEUE => PageQueue::None,
            DEACTIVATE => PageQueue::Inactive,
            _ => PageQueue::Active,
        }
    } else if flags & ACTIVE != 0 {
        PageQueue::Active
    } else if flags & INACTIVE != 0 {
        PageQueue::Inactive
    } else {
        PageQueue::None
    }
}

/// Caller holds the PFN interlock. uvm_pageunlock queues the pointer only
/// after releasing that interlock; later intents coalesce until consumption.
fn set_intent(page: &'static Page, intent: u32) -> bool {
    let flags = page.policy_flags.load(Ordering::Relaxed);
    page.policy_flags.store(
        (flags & !INTENT_MASK) | intent | INTENT_SET | INTENT_QUEUED,
        Ordering::Release,
    );
    flags & INTENT_QUEUED == 0
}
fn intend(page: &'static Page, intent: u32) {
    let enqueue = {
        let _interlock = page.owner.lock();
        set_intent(page, intent)
    };
    if enqueue {
        queue_pending(page);
    }
}
pub(super) fn wire(page: &'static Page, count: &AtomicU32) {
    let enqueue = {
        let _interlock = page.owner.lock();
        let previous = count.fetch_add(1, Ordering::AcqRel);
        assert!(previous != u32::MAX, "uvm: wire count overflow");
        previous == 0 && set_intent(page, DEQUEUE)
    };
    if enqueue {
        queue_pending(page);
    }
}
pub(super) fn unwire(page: &'static Page, count: &AtomicU32) {
    let enqueue = {
        let _interlock = page.owner.lock();
        let previous = count.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "uvm: wire count underflow");
        previous == 1 && set_intent(page, ACTIVATE)
    };
    if enqueue {
        queue_pending(page);
    }
}
fn queue_pending(page: &'static Page) {
    {
        let _preempt = smp::disable_preemption();
        let id = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
        let mut pending = PENDING[id].lock();
        if pending.head != 0 {
            pending.head -= 1;
            let head = pending.head;
            pending.pages[head] = Some(page);
            return;
        }
    }
    // The policy mutex may sleep and migrate us. Once acquired, choose and
    // pin the current CPU, drain and enqueue before releasing that pin.
    // Returning to the fast-path loop here would allow migration back to a
    // full queue after repeatedly draining a different CPU's empty queue.
    let mut state = global().lock();
    let _preempt = smp::disable_preemption();
    let id = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
    let mut pending = PENDING[id].lock();
    drain(&mut state, &mut pending, false);
    pending.head -= 1;
    let head = pending.head;
    pending.pages[head] = Some(page);
}
pub(super) fn activate(page: &'static Page) {
    intend(page, ACTIVATE);
}
pub(super) fn deactivate(page: &'static Page) {
    intend(page, DEACTIVATE);
}
pub(super) fn dequeue(page: &'static Page) {
    intend(page, DEQUEUE);
}
pub(super) fn enqueue(page: &'static Page) {
    intend(page, ENQUEUE);
}

fn drain(state: &mut ClockState, pending: &mut Pending, nonblocking: bool) {
    while pending.head != QUEUE_SIZE {
        let page = pending.pages[pending.head].expect("uvm: missing pending page");
        let interlock = if nonblocking {
            page.owner.try_lock()
        } else {
            Some(page.owner.lock())
        };
        let Some(_interlock) = interlock else { break };
        pending.pages[pending.head] = None;
        pending.head += 1;
        page.policy_flags
            .fetch_and(!INTENT_QUEUED, Ordering::Relaxed);
        state.realize(page);
    }
}
/// Acquiring policy can sleep and migrate. Select the current CPU afterwards,
/// then pin only while touching its pending array (uvmpdpol_flush).
pub(super) fn flush() {
    let mut state = global().lock();
    let _preempt = smp::disable_preemption();
    let id = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
    drain(&mut state, &mut PENDING[id].lock(), false);
}

pub(crate) fn idle() {
    let now = clock::monotonic_ns_or_zero();
    let Some(mut state) = global().try_lock() else {
        return;
    };
    let _preempt = smp::disable_preemption();
    let id = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
    let mut pending = PENDING[id].lock();
    if now.wrapping_sub(pending.last_flush) < IDLE_INTERVAL_NS {
        return;
    }
    drain(&mut state, &mut pending, true);
    if pending.head == QUEUE_SIZE {
        pending.last_flush = now;
    }
}

/// A stack marker stays borrowed and stationary until removed by Drop. Every
/// list traversal/change uses policy; real pages may move while it is dropped.
struct Marker<'a> {
    page: &'a Page,
    active: bool,
}
impl<'a> Marker<'a> {
    fn new(page: &'a Page, active: bool) -> Self {
        // SAFETY: the borrow prevents marker movement; Drop removes it before
        // this borrow ends. No marker pointer escapes a locked traversal.
        let entry = unsafe { UnsafeRef::from_raw(ptr::from_ref(page)) };
        global().lock().list(active).push_front(entry);
        Self { page, active }
    }
    fn next(&self, state: &mut ClockState) -> Option<&'static Page> {
        loop {
            // SAFETY: this guard owns the marker and policy protects its link.
            let mut cursor = unsafe {
                state
                    .list(self.active)
                    .cursor_mut_from_ptr(ptr::from_ref(self.page))
            };
            cursor.move_next();
            let next = ptr::from_ref(cursor.get()?);
            // SAFETY: policy keeps both the next page and marker linked. Move
            // the marker after next before dropping policy to acquire owner.
            unsafe {
                state
                    .list(self.active)
                    .cursor_mut_from_ptr(ptr::from_ref(self.page))
                    .remove();
                state
                    .list(self.active)
                    .cursor_mut_from_ptr(next)
                    .insert_after(UnsafeRef::from_raw(ptr::from_ref(self.page)));
            }
            // SAFETY: policy still protects next. Other scans' markers are
            // recognized and skipped while locked; only permanent PFNs escape.
            let page = unsafe { &*next };
            if !page.flags().contains(PageFlags::MARKER) {
                return Some(page);
            }
        }
    }
}
impl Drop for Marker<'_> {
    fn drop(&mut self) {
        // SAFETY: the marker is still linked, stationary and owned by us.
        unsafe {
            global()
                .lock()
                .list(self.active)
                .cursor_mut_from_ptr(ptr::from_ref(self.page))
                .remove();
        }
    }
}

fn same_owner(owner: &PageRef, locked: &Option<super::PageWeak>) -> bool {
    locked
        .as_ref()
        .is_some_and(|current| current.as_ptr() == alloc::sync::Arc::as_ptr(owner))
}

/// Balance the clock hands to 33% inactive, at least freetarg+1. Owner locks
/// are tried with policy dropped and every candidate is revalidated afterwards.
pub(super) fn balance(maximum: usize, mut swap_shortage: usize) {
    flush();
    let marker_page = Page::new_marker();
    let marker = Marker::new(&marker_page, true);
    for _ in 0..maximum {
        let candidate = {
            let mut state = global().lock();
            if state.ninactive >= state.target() && swap_shortage == 0 {
                break;
            }
            let Some(page) = marker.next(&mut state) else {
                break;
            };
            let interlock = page.owner.lock();
            if state.realize(page) {
                continue;
            }
            interlock
                .as_ref()
                .and_then(|owner| owner.upgrade())
                .map(|owner| (page, owner))
        };
        let Some((page, owner)) = candidate else {
            continue;
        };
        let Some(mut locked) = owner.owner().try_write() else {
            continue;
        };
        let deactivate = {
            let mut state = global().lock();
            let interlock = page.owner.lock();
            if state.realize(page)
                || page.policy_flags.load(Ordering::Relaxed) & ACTIVE == 0
                || page.wire_count() != 0
                || !same_owner(&owner, &interlock)
                || !owner
                    .frame_locked(&locked)
                    .is_some_and(|current| ptr::eq(current, page))
            {
                continue;
            }
            state.ninactive < state.target()
        };
        if swap_shortage != 0 && owner.drop_swap_locked(&mut locked) {
            swap_shortage -= 1;
        }
        if deactivate {
            pmap::harvest_page_state_locked(&owner, &mut locked);
            page.take_referenced();
            // uvmpdpol_balancequeue changes its own queues immediately. A
            // deferred intent would leave ninactive stale while deciding how
            // many more pages to age, and can scan the entire active list.
            let mut state = global().lock();
            let _interlock = page.owner.lock();
            let flags = page.policy_flags.load(Ordering::Relaxed);
            page.policy_flags.store(
                (flags & !INTENT_MASK) | DEACTIVATE | INTENT_SET,
                Ordering::Relaxed,
            );
            state.realize(page);
        }
    }
}

/// Select inactive victims using the reference min/max class protection.
/// The owner stays held from revalidation through reclaim's PG_BUSY claim.
pub(super) fn scan(maximum: usize, target: usize) -> usize {
    flush();
    let anon = ANON_PAGES.load(Ordering::Relaxed);
    let file = FILE_PAGES.load(Ordering::Relaxed);
    let exec = EXEC_PAGES.load(Ordering::Relaxed);
    let free = super::phys::stats().map_or(0, |s| s.free_pages);
    let total = {
        let state = global().lock();
        state.nactive + state.ninactive + free
    };
    let mut anonreact = anon <= total * 10 / 100
        || (anon <= total * 80 / 100 && (file > total * 50 / 100 || exec > total * 30 / 100));
    let mut filereact = file <= total * 10 / 100
        || (file <= total * 50 / 100 && (anon > total * 80 / 100 || exec > total * 30 / 100));
    let mut execreact = exec <= total * 5 / 100
        || (exec <= total * 30 / 100 && (anon > total * 80 / 100 || file > total * 50 / 100));
    if filereact && execreact && (anonreact || super::swap::is_full()) {
        anonreact = false;
        filereact = false;
        execreact = false;
    }
    let marker_page = Page::new_marker();
    let marker = Marker::new(&marker_page, false);
    let mut freed = 0;
    let mut dirty_reactivations = 0;
    for _ in 0..maximum {
        if freed >= target {
            break;
        }
        let candidate = {
            let mut state = global().lock();
            let Some(page) = marker.next(&mut state) else {
                break;
            };
            let interlock = page.owner.lock();
            if state.realize(page) {
                continue;
            }
            let protect = if page.owner_kind() == PageOwnerKind::File
                && page.flags().contains(PageFlags::EXECUTABLE)
            {
                execreact
            } else if page.owner_kind() == PageOwnerKind::File {
                filereact
            } else {
                anonreact
            };
            if protect {
                let flags = page.policy_flags.load(Ordering::Relaxed);
                page.policy_flags.store(
                    (flags & !INTENT_MASK) | ACTIVATE | INTENT_SET,
                    Ordering::Relaxed,
                );
                state.realize(page);
                continue;
            }
            interlock
                .as_ref()
                .and_then(|owner| owner.upgrade())
                .map(|owner| (page, owner))
        };
        let Some((page, owner)) = candidate else {
            continue;
        };
        let Some(mut locked) = owner.owner().try_write() else {
            continue;
        };
        {
            let mut state = global().lock();
            let interlock = page.owner.lock();
            if state.realize(page)
                || page.policy_flags.load(Ordering::Relaxed) & INACTIVE == 0
                || page.wire_count() != 0
                || !same_owner(&owner, &interlock)
                || !owner
                    .frame_locked(&locked)
                    .is_some_and(|current| ptr::eq(current, page))
            {
                continue;
            }
        }
        pmap::harvest_page_state_locked(&owner, &mut locked);
        if page.take_referenced() {
            activate(page);
        } else if owner.reclaim_owned(locked) {
            freed += 1;
        } else if page.flags().contains(PageFlags::DIRTY) {
            dirty_reactivations += 1;
            // UVMPD_NUMDIRTYREACTS bounds a fruitless dirty-page pass.
            if dirty_reactivations == 16 {
                break;
            }
        }
    }
    freed
}
pub(super) fn queues_empty() -> bool {
    let state = global().lock();
    state.nactive == 0 && state.ninactive == 0
}
