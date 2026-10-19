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
 *	@(#)vm_page.c   8.3 (Berkeley) 3/21/94
 * from: Id: uvm_page.c,v 1.1.2.18 1998/02/06 05:24:42 chs Exp
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

//!
//! # Physical Memory Manager
//!
//! PFN database, physical-page lifecycle, free/zero queues, per-CPU caches,
//! and managed-page queues.
//!

use crate::mem::{PageRef, PageWeak};
use alloc::sync::Arc;
use core::{
    mem::size_of,
    ptr,
    sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
};

use bitflags::bitflags;
use intrusive_collections::{LinkedList, LinkedListLink, intrusive_adapter};
use limine::memory_map::{Entry, EntryType};
use log::info;

use crate::{
    arch,
    mem::{
        self, PhysAddr, VirtAddr, VmFlags,
        addr::{PAGE_SIZE, align_down, align_up, pages_for_len},
    },
    sys::{smp::IrqSpinLock, sync::Once},
};

/// Virtual base address where the PFN database is mapped.
pub const PAGEDB_ADDR: u64 = 0xFFFF_B000_0000_0000;

const PAGEDB: *mut Page = PAGEDB_ADDR as *mut Page;
const MAX_PAGE_CPUS: usize = 256;
const PAGE_CACHE_CAPACITY: usize = 16;
const PAGE_CACHE_REFILL: usize = 8;

bitflags! {
    /// Allocation requirements for one physical page.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct PageAllocFlags: u8 {
        /// Return a page whose contents are all zero.
        const ZERO = 1 << 0;
        /// Allow the caller to consume the kernel reserve (UVM_PGA_USERESERVE).
        const USERESERVE = 1 << 1;
    }
}

bitflags! {
    /// State flags carried by every PFN database entry.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct PageFlags: u32 {
        /// Page contents differ from their backing store.
        const DIRTY = 1 << 0;
        /// Page was accessed during the current aging interval.
        const REFERENCED = 1 << 1;
        /// Page is exclusively held for an in-progress transition.
        const BUSY = 1 << 2;
        /// A waiter observed the busy page.
        const WANTED = 1 << 3;
        /// Page should be released when its busy transition completes.
        const RELEASED = 1 << 4;
        /// Page is present in an object or anonymous-page index.
        const TABLED = 1 << 5;
        /// Page belongs to anonymous memory.
        const ANON = 1 << 6;
        /// Page belongs to a vnode-backed object.
        const FILE = 1 << 7;
        /// Temporary traversal marker belonging to the clock policy.
        const MARKER = 1 << 8;
        /// Vnode page used by an executable mapping.
        const EXECUTABLE = 1 << 9;
        /// Hardware modified bits have not yet resolved clean/dirty state.
        const DIRTY_UNKNOWN = 1 << 10;
    }
}

/// Allocator-visible page state.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageState {
    /// Memory is not available to the allocator.
    Reserved = 0,
    /// Free page with unspecified contents.
    Free = 1,
    /// Free page whose contents are all zero.
    Zero = 2,
    /// Page is allocated to a subsystem.
    Allocated = 3,
}

/// Current physical-page consumer.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageUse {
    /// No active consumer.
    None = 0,
    /// Pageable object or anonymous memory.
    Managed = 1,
    /// Kernel heap backing.
    KernelHeap = 2,
    /// Architecture page-table page.
    PageTable = 3,
    /// PFN database and bootstrap mappings.
    PfnDatabase = 4,
    /// Firmware, device, or otherwise unavailable memory.
    Reserved = 5,
}

/// Managed-page owner category.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageOwnerKind {
    /// No managed owner.
    None = 0,
    /// Anonymous overlay page.
    Anonymous = 1,
    /// Vnode-backed page-cache page.
    File = 2,
    /// Kernel pageable object.
    Kernel = 3,
}

/// Page-daemon queue membership.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PageQueue {
    /// Not queued for reclamation.
    None = 0,
    /// Recently used page.
    Active = 1,
    /// Reclamation candidate.
    Inactive = 2,
    // The clock policy has only active and inactive queues.
}

/// Authoritative metadata for one physical page frame.
///
/// The database holds one entry per frame of usable memory, so its size is a
/// fixed fraction of RAM. Separate free and replacement links permit deferred
/// policy updates. The entry deliberately holds no owning collections: reverse mappings belong to the
/// logical page. Each entry records its immutable physical frame number;
/// UVM physical segments select its slice of the compact page array.
#[repr(C, align(64))]
pub struct Page {
    /// Physical allocator free-list membership (independent of pdqueue).
    link: LinkedListLink,
    pub(super) policy_link: LinkedListLink,
    frame_number: u32,
    pub(super) policy_flags: AtomicU32,
    flags: AtomicU32,
    /// Packed [`PageState`], [`PageUse`], [`PageQueue`] and [`PageOwnerKind`].
    status: AtomicU32,
    wire_count: AtomicU32,
    loan_count: AtomicU32,
    map_count: AtomicU32,
    pub(super) owner: IrqSpinLock<Option<PageWeak>>,
}

/// Bit position of each packed status field.
const STATE_SHIFT: u32 = 0;
const USAGE_SHIFT: u32 = 8;
const QUEUE_SHIFT: u32 = 16;
const OWNER_KIND_SHIFT: u32 = 24;
const FIELD_MASK: u32 = 0xff;

// Free and replacement queues need distinct links: a freed frame can still
// have a deferred dequeue intent, exactly as in UVM's pageq/pdqueue layout.
const _: () = assert!(
    size_of::<Page>() <= 128 && size_of::<Page>().is_power_of_two(),
    "mem/phys: invalid frame database entry size"
);

const fn pack_status(
    state: PageState,
    usage: PageUse,
    queue: PageQueue,
    owner: PageOwnerKind,
) -> u32 {
    (state as u32) << STATE_SHIFT
        | (usage as u32) << USAGE_SHIFT
        | (queue as u32) << QUEUE_SHIFT
        | (owner as u32) << OWNER_KIND_SHIFT
}

impl Page {
    pub(super) fn new() -> Self {
        Self {
            policy_link: LinkedListLink::new(),
            frame_number: 0,
            policy_flags: AtomicU32::new(0),
            link: LinkedListLink::new(),
            flags: AtomicU32::new(0),
            status: AtomicU32::new(pack_status(
                PageState::Reserved,
                PageUse::Reserved,
                PageQueue::None,
                PageOwnerKind::None,
            )),
            wire_count: AtomicU32::new(0),
            loan_count: AtomicU32::new(0),
            map_count: AtomicU32::new(0),
            owner: IrqSpinLock::new(None),
        }
    }

    pub(super) fn new_marker() -> Self {
        let page = Self::new();
        page.flags
            .store(PageFlags::MARKER.bits(), Ordering::Relaxed);
        page
    }

    /// Returns the physical base address.
    pub fn paddr(&self) -> PhysAddr {
        PhysAddr::new(self.pfn() as u64 * PAGE_SIZE)
    }

    /// Returns the immutable physical frame number, independent of array position.
    pub fn pfn(&self) -> usize {
        self.frame_number as usize
    }

    /// Returns allocator state.
    pub fn state(&self) -> PageState {
        decode_page_state(self.status_field(STATE_SHIFT))
    }

    /// Returns the current allocation consumer.
    pub fn usage(&self) -> PageUse {
        decode_page_use(self.status_field(USAGE_SHIFT))
    }

    /// Returns page-daemon queue membership.
    pub fn queue(&self) -> PageQueue {
        super::policy::queue(self)
    }

    /// Returns managed owner category.
    pub fn owner_kind(&self) -> PageOwnerKind {
        decode_owner_kind(self.status_field(OWNER_KIND_SHIFT))
    }

    /// Returns current page flags.
    pub fn flags(&self) -> PageFlags {
        PageFlags::from_bits_retain(self.flags.load(Ordering::Acquire))
    }

    /// Returns the current wire count.
    pub fn wire_count(&self) -> u32 {
        self.wire_count.load(Ordering::Acquire)
    }

    /// Returns the current loan count.
    pub fn loan_count(&self) -> u32 {
        self.loan_count.load(Ordering::Acquire)
    }

    /// Returns the current hardware mapping count.
    pub fn map_count(&self) -> u32 {
        self.map_count.load(Ordering::Acquire)
    }

    #[inline]
    fn status_field(&self, shift: u32) -> u8 {
        ((self.status.load(Ordering::Acquire) >> shift) & FIELD_MASK) as u8
    }

    fn set_status_field(&self, shift: u32, value: u8) {
        let cleared = !(FIELD_MASK << shift);
        let inserted = (value as u32) << shift;
        let mut current = self.status.load(Ordering::Relaxed);
        loop {
            match self.status.compare_exchange_weak(
                current,
                (current & cleared) | inserted,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    pub(crate) fn bind_owner(&'static self, owner: &PageRef, kind: PageOwnerKind) {
        let mut interlock = self.owner.lock();
        assert_eq!(self.usage(), PageUse::Managed);
        assert_eq!(self.owner_kind(), PageOwnerKind::None);
        *interlock = Some(Arc::downgrade(owner));
        self.set_status_field(OWNER_KIND_SHIFT, kind as u8);
        let executable = kind == PageOwnerKind::File && owner.is_executable();
        let mut owner_flag = match kind {
            PageOwnerKind::Anonymous => PageFlags::ANON,
            PageOwnerKind::File => PageFlags::FILE,
            PageOwnerKind::Kernel | PageOwnerKind::None => PageFlags::empty(),
        };
        if executable {
            owner_flag |= PageFlags::EXECUTABLE;
        }
        self.flags
            .fetch_or((PageFlags::TABLED | owner_flag).bits(), Ordering::AcqRel);
        MANAGED_PAGES.fetch_add(1, Ordering::AcqRel);
        super::policy::account(kind, executable, 1);
    }

    pub(crate) fn clear_owner(&self) {
        let mut interlock = self.owner.lock();
        if self.owner_kind() != PageOwnerKind::None {
            super::policy::account(
                self.owner_kind(),
                self.flags().contains(PageFlags::EXECUTABLE),
                -1,
            );
            MANAGED_PAGES.fetch_sub(1, Ordering::AcqRel);
        }
        *interlock = None;
        self.set_status_field(OWNER_KIND_SHIFT, PageOwnerKind::None as u8);
        self.flags.fetch_and(
            !(PageFlags::TABLED | PageFlags::ANON | PageFlags::FILE | PageFlags::EXECUTABLE).bits(),
            Ordering::AcqRel,
        );
    }

    pub(super) fn note_executable(&self) {
        let _interlock = self.owner.lock();
        if self.owner_kind() == PageOwnerKind::File
            && self
                .flags
                .fetch_or(PageFlags::EXECUTABLE.bits(), Ordering::Relaxed)
                & PageFlags::EXECUTABLE.bits()
                == 0
        {
            super::policy::account(PageOwnerKind::File, false, -1);
            super::policy::account(PageOwnerKind::File, true, 1);
        }
    }

    /// Adds one wire that prevents reclamation.
    pub fn wire(&'static self) {
        super::policy::wire(self, &self.wire_count);
    }

    /// Removes a wire and activates the page after the final wire is released.
    pub fn unwire(&'static self) {
        super::policy::unwire(self, &self.wire_count);
    }

    /// Adds one transient page loan.
    pub fn loan(&self) {
        self.loan_count.fetch_add(1, Ordering::AcqRel);
    }

    /// Removes one transient page loan.
    pub fn unloan(&self) {
        let previous = self.loan_count.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "mem/phys: page loan count underflow");
    }

    pub(crate) fn add_mapping(&self) {
        self.map_count.fetch_add(1, Ordering::AcqRel);
    }

    pub(crate) fn remove_mapping(&self) {
        let previous = self.map_count.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "mem/phys: page mapping count underflow");
    }

    pub(crate) fn mark_dirty(&self) {
        let _ = self
            .flags
            .try_update(Ordering::AcqRel, Ordering::Relaxed, |flags| {
                Some((flags | PageFlags::DIRTY.bits()) & !PageFlags::DIRTY_UNKNOWN.bits())
            });
    }
    pub(crate) fn mark_clean(&self) {
        self.flags.fetch_and(
            !(PageFlags::DIRTY | PageFlags::DIRTY_UNKNOWN).bits(),
            Ordering::Release,
        );
    }
    pub(crate) fn mark_unknown(&self) {
        let _ = self
            .flags
            .try_update(Ordering::AcqRel, Ordering::Relaxed, |flags| {
                Some(if flags & PageFlags::DIRTY.bits() == 0 {
                    flags | PageFlags::DIRTY_UNKNOWN.bits()
                } else {
                    flags
                })
            });
    }
    /// All PTEs have been revoked and their A/D bits folded into this PFN.
    pub(crate) fn resolve_unknown(&self) {
        self.flags
            .fetch_and(!PageFlags::DIRTY_UNKNOWN.bits(), Ordering::Release);
    }

    pub(crate) fn mark_referenced(&self) {
        self.flags
            .fetch_or(PageFlags::REFERENCED.bits(), Ordering::Release);
    }

    pub(crate) fn take_referenced(&self) -> bool {
        self.flags
            .fetch_and(!PageFlags::REFERENCED.bits(), Ordering::AcqRel)
            & PageFlags::REFERENCED.bits()
            != 0
    }

    pub(crate) fn try_busy(&self) -> bool {
        let mut flags = self.flags.load(Ordering::Acquire);
        loop {
            if flags & PageFlags::BUSY.bits() != 0 {
                self.flags
                    .fetch_or(PageFlags::WANTED.bits(), Ordering::AcqRel);
                return false;
            }
            match self.flags.compare_exchange_weak(
                flags,
                flags | PageFlags::BUSY.bits(),
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return true,
                Err(observed) => flags = observed,
            }
        }
    }

    pub(crate) fn unbusy(&self) -> PageFlags {
        let previous = self.flags.fetch_and(
            !(PageFlags::BUSY | PageFlags::WANTED).bits(),
            Ordering::AcqRel,
        );
        PageFlags::from_bits_retain(previous)
    }

    pub(crate) fn reclaimable(&self) -> bool {
        self.wire_count() == 0
            && self.loan_count() == 0
            && self.map_count() == 0
            && !self.flags().contains(PageFlags::BUSY)
    }

    /// Claims a free frame for `usage`.
    ///
    /// The frame is unreachable by any other subsystem at this point, so the
    /// whole status word is published in one store.
    fn prepare_allocation(&self, usage: PageUse) {
        assert!(matches!(self.state(), PageState::Free | PageState::Zero));
        assert!(!self.link.is_linked());
        self.flags.store(0, Ordering::Relaxed);
        self.wire_count.store(0, Ordering::Relaxed);
        self.loan_count.store(0, Ordering::Relaxed);
        self.map_count.store(0, Ordering::Relaxed);
        *self.owner.lock() = None;
        self.status.store(
            pack_status(
                PageState::Allocated,
                usage,
                PageQueue::None,
                PageOwnerKind::None,
            ),
            Ordering::Release,
        );
    }

    fn prepare_free(&self, zeroed: bool) {
        assert_eq!(self.state(), PageState::Allocated);
        assert_eq!(self.owner_kind(), PageOwnerKind::None);
        assert_eq!(self.wire_count(), 0);
        assert_eq!(self.loan_count(), 0);
        assert_eq!(self.map_count(), 0);
        assert!(!self.link.is_linked());
        self.flags.store(0, Ordering::Relaxed);
        self.status.store(
            pack_status(
                if zeroed {
                    PageState::Zero
                } else {
                    PageState::Free
                },
                PageUse::None,
                PageQueue::None,
                PageOwnerKind::None,
            ),
            Ordering::Release,
        );
    }
}

// SAFETY: PFN entries have stable addresses and all interior mutation is
// atomic or protected by locks and allocator queue locks.
unsafe impl Send for Page {}
// SAFETY: shared access observes atomic fields or lock-protected owner state.
unsafe impl Sync for Page {}

// Free lists use their own link; deferred policy updates use policy_link.
intrusive_adapter!(FreePageAdapter = &'static Page: Page { link: LinkedListLink });

struct PmmState {
    free: LinkedList<FreePageAdapter>,
    zero: LinkedList<FreePageAdapter>,
    nfree: usize,
}

#[repr(align(64))]
struct PageCache {
    dirty: [u32; PAGE_CACHE_CAPACITY],
    dirty_len: usize,
    zero: [u32; PAGE_CACHE_CAPACITY],
    zero_len: usize,
}

impl PageCache {
    const fn new() -> Self {
        Self {
            dirty: [0; PAGE_CACHE_CAPACITY],
            dirty_len: 0,
            zero: [0; PAGE_CACHE_CAPACITY],
            zero_len: 0,
        }
    }

    fn pop(&mut self, zero: bool) -> Option<u32> {
        let (slots, len) = if zero {
            (&mut self.zero, &mut self.zero_len)
        } else {
            (&mut self.dirty, &mut self.dirty_len)
        };
        if *len == 0 {
            return None;
        }
        *len -= 1;
        Some(slots[*len])
    }

    fn push(&mut self, pfn: u32, zero: bool) -> bool {
        let (slots, len) = if zero {
            (&mut self.zero, &mut self.zero_len)
        } else {
            (&mut self.dirty, &mut self.dirty_len)
        };
        if *len == PAGE_CACHE_CAPACITY {
            return false;
        }
        slots[*len] = pfn;
        *len += 1;
        true
    }
}

static PMM: IrqSpinLock<Option<PmmState>> = IrqSpinLock::new(None);
static PAGE_CACHES: [IrqSpinLock<PageCache>; MAX_PAGE_CPUS] =
    [const { IrqSpinLock::new(PageCache::new()) }; MAX_PAGE_CPUS];
static READY: AtomicBool = AtomicBool::new(false);
static MAX_PFN: AtomicUsize = AtomicUsize::new(0);
static SEGMENTS: Once<&'static [super::physseg::Segment]> = Once::new();
static TOTAL_PAGES: AtomicUsize = AtomicUsize::new(0);
static USED_PAGES: AtomicUsize = AtomicUsize::new(0);
static FREE_PAGES: AtomicUsize = AtomicUsize::new(0);
static ZERO_PAGES: AtomicUsize = AtomicUsize::new(0);
static MANAGED_PAGES: AtomicUsize = AtomicUsize::new(0);

static mut BOOTSTRAP_NEXT: u64 = 0;
static mut BOOTSTRAP_END: u64 = 0;

/// Snapshot of physical page usage.
#[derive(Copy, Clone, Debug, Default)]
pub struct PhysStats {
    /// Usable physical pages known to the allocator.
    pub total_pages: usize,
    /// Pages currently allocated.
    pub used_pages: usize,
    /// Free pages, including pre-zeroed pages and per-CPU caches.
    pub free_pages: usize,
    /// Free pages known to contain zeros.
    pub zero_pages: usize,
    /// Resident pages owned by anonymous memory or memory objects.
    pub managed_pages: usize,
}

/// Initializes the PFN database and page queues from the boot memory map.
pub fn init() {
    let mmap = mem::memory_map_entries();
    let mut max_usable_end = 0u64;
    let mut usable_pages = 0usize;
    let mut segment_count = 0usize;

    info!("memory map structure:");
    for entry in mmap.iter() {
        let end = entry.base.saturating_add(entry.length);
        info!(
            "\t[{:016x}-{:016x}] {}",
            entry.base,
            end,
            entry_type_name(entry)
        );
        if let Some((start, usable_end)) = usable_page_range(entry) {
            max_usable_end = max_usable_end.max(usable_end);
            usable_pages += ((usable_end - start) / PAGE_SIZE) as usize;
            segment_count += 1;
        }
    }
    assert!(max_usable_end != 0, "mem/phys: no usable memory");

    let maximum_pfn = pages_for_len(max_usable_end) as usize;
    let database_entries = usable_pages;
    assert!(
        maximum_pfn <= u32::MAX as usize,
        "mem/phys: PFN cache encoding supports at most {} pages",
        u32::MAX
    );
    let pagedb_pages = pages_for_len((size_of::<Page>() * database_entries) as u64);
    let segment_pages =
        pages_for_len((segment_count * size_of::<super::physseg::Segment>()) as u64);
    let l1 = pagedb_pages.div_ceil(512);
    let l2 = l1.div_ceil(512);
    let l3 = l2.div_ceil(512);
    let l4 = l3.div_ceil(512);
    let bootstrap_pages = l1 + l2 + l3 + l4 + 8;
    let chunk_size = (pagedb_pages + segment_pages + bootstrap_pages) * PAGE_SIZE;

    let mut pagedb_phys_base = None;
    let mut largest_usable = 0u64;
    for entry in mmap {
        let Some((start, end)) = usable_page_range(entry) else {
            continue;
        };
        let span = end.saturating_sub(start);
        largest_usable = largest_usable.max(span);
        if span >= chunk_size {
            pagedb_phys_base = Some(start);
            break;
        }
    }
    let pagedb_phys_base = pagedb_phys_base.unwrap_or_else(|| {
        panic!(
            "mem/phys: no usable chunk for PFNDB (need {} KiB, largest {} KiB)",
            chunk_size / 1024,
            largest_usable / 1024
        )
    });
    let pagedb_phys_end = pagedb_phys_base + pagedb_pages * PAGE_SIZE;
    let bootstrap_end = pagedb_phys_base + chunk_size;

    // SAFETY: bootstrap allocation is single-threaded and bounded by the
    // reserved PFN database chunk.
    unsafe {
        BOOTSTRAP_NEXT = pagedb_phys_end + segment_pages * PAGE_SIZE;
        BOOTSTRAP_END = bootstrap_end;
    }

    // SAFETY: the PFN database virtual range and selected physical chunk are
    // exclusively reserved during early memory initialization.
    unsafe {
        let root = arch::paging::active_root();
        let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL;
        for i in 0..pagedb_pages {
            let va = VirtAddr::new(PAGEDB_ADDR + i * PAGE_SIZE);
            let pa = PhysAddr::new(pagedb_phys_base + i * PAGE_SIZE);
            arch::paging::map_page(root, va, pa, flags)
                .unwrap_or_else(|error| panic!("mem/phys: map PFNDB page {i}: {error:?}"));
        }
        // Segment descriptors live in the same permanently reserved bootstrap
        // chunk. Build them before publishing the immutable lookup table.
        let segment_pointer = mem::phys_to_virt(PhysAddr::new(pagedb_phys_end))
            .as_mut_ptr::<super::physseg::Segment>();
        let mut index = 0;
        for entry in mmap {
            if let Some((start, end)) = usable_page_range(entry) {
                segment_pointer.add(index).write(super::physseg::Segment {
                    start: (start / PAGE_SIZE) as usize,
                    end: (end / PAGE_SIZE) as usize,
                    first: 0,
                });
                index += 1;
            }
        }
        let segments = core::slice::from_raw_parts_mut(segment_pointer, segment_count);
        assert_eq!(super::physseg::initialize(segments), database_entries);
        for segment in segments.iter() {
            for pfn in segment.start..segment.end {
                let mut page = Page::new();
                page.frame_number = pfn as u32;
                ptr::write(PAGEDB.add(segment.first + pfn - segment.start), page);
            }
        }
        SEGMENTS.call_once(|| segments);
    }

    let mut state = PmmState {
        free: LinkedList::new(FreePageAdapter::NEW),
        zero: LinkedList::new(FreePageAdapter::NEW),
        nfree: 0,
    };
    let mut used_pages = 0usize;
    let mut free_pages = 0usize;

    for entry in mmap {
        let Some((start, end)) = usable_page_range(entry) else {
            continue;
        };
        let mut pa = start;
        while pa < end {
            // SAFETY: every usable page is covered by the initialized PFN DB.
            let index = super::physseg::lookup(SEGMENTS.get().unwrap(), (pa / PAGE_SIZE) as usize)
                .expect("uvm: usable frame missing from physical segments");
            // SAFETY: the initialized segment bounds the compact array index.
            let page = unsafe { &*PAGEDB.add(index) };
            if pa >= pagedb_phys_base && pa < bootstrap_end {
                page.status.store(
                    pack_status(
                        PageState::Allocated,
                        PageUse::PfnDatabase,
                        PageQueue::None,
                        PageOwnerKind::None,
                    ),
                    Ordering::Relaxed,
                );
                used_pages += 1;
            } else {
                page.status.store(
                    pack_status(
                        PageState::Free,
                        PageUse::None,
                        PageQueue::None,
                        PageOwnerKind::None,
                    ),
                    Ordering::Relaxed,
                );
                state.free.push_back(page);
                free_pages += 1;
            }
            pa += PAGE_SIZE;
        }
    }

    MAX_PFN.store(maximum_pfn, Ordering::Release);
    TOTAL_PAGES.store(usable_pages, Ordering::Release);
    USED_PAGES.store(used_pages, Ordering::Release);
    FREE_PAGES.store(free_pages, Ordering::Release);
    ZERO_PAGES.store(0, Ordering::Release);
    MANAGED_PAGES.store(0, Ordering::Release);
    state.nfree = free_pages;
    *PMM.lock() = Some(state);
    READY.store(true, Ordering::Release);

    // SAFETY: publication ends all bootstrap allocations.
    unsafe {
        BOOTSTRAP_NEXT = 0;
        BOOTSTRAP_END = 0;
    }

    info!(
        "page database at [0x{:x}-0x{:x}], {} entries, {} KiB metadata, {} pages used, {} free",
        pagedb_phys_base,
        pagedb_phys_base + pagedb_pages * PAGE_SIZE,
        database_entries,
        pagedb_pages * PAGE_SIZE / 1024,
        used_pages,
        free_pages,
    );
}

/// Converts a page-aligned physical address to its PFN database entry.
pub fn phys_to_page(pa: PhysAddr) -> Option<&'static Page> {
    if !READY.load(Ordering::Acquire) || !pa.is_page_aligned() {
        return None;
    }
    let index = (pa.as_u64() / PAGE_SIZE) as usize;
    page_by_index(index)
}

/// Allocates one physical page for `usage`.
pub fn alloc_page(usage: PageUse, mut flags: PageAllocFlags) -> Option<&'static Page> {
    if !READY.load(Ordering::Acquire) {
        return None;
    }
    assert!(
        !matches!(
            usage,
            PageUse::None | PageUse::Reserved | PageUse::PfnDatabase
        ),
        "mem/phys: invalid dynamic page use {usage:?}"
    );

    if crate::sys::sched::current_thread_opt().is_some_and(|thread| {
        // SAFETY: the scheduler retains the executing thread; process identity
        // is immutable for its lifetime. Kernel LWPs receive USERESERVE in UVM.
        unsafe { &*thread }.is_kernel()
    }) {
        flags.insert(PageAllocFlags::USERESERVE);
    }
    let want_zero = flags.contains(PageAllocFlags::ZERO);
    let (page, was_zero) = take_page(want_zero, flags)?;
    FREE_PAGES.fetch_sub(1, Ordering::AcqRel);
    USED_PAGES.fetch_add(1, Ordering::AcqRel);
    if was_zero {
        ZERO_PAGES.fetch_sub(1, Ordering::AcqRel);
    }
    page.prepare_allocation(usage);
    if want_zero && !was_zero {
        zero_page(page.paddr());
    }
    Some(page)
}

/// Allocates one zeroed physical page for `usage`.
pub fn alloc_zeroed_page(usage: PageUse) -> Option<&'static Page> {
    let reserve = if usage == PageUse::Managed {
        PageAllocFlags::empty()
    } else {
        PageAllocFlags::USERESERVE
    };
    alloc_page(usage, PageAllocFlags::ZERO | reserve)
}

/// Allocates one zeroed page and returns its physical address.
///
/// Before PFN database publication, page-table callers consume the bounded
/// bootstrap range reserved alongside the database.
pub fn alloc_zeroed_phys(usage: PageUse) -> Option<PhysAddr> {
    if let Some(page) = alloc_zeroed_page(usage) {
        return Some(page.paddr());
    }
    if READY.load(Ordering::Acquire) {
        return None;
    }

    // SAFETY: only single-threaded bootstrap page-table construction reaches
    // this path before the PFN database is published.
    let pa = unsafe {
        if BOOTSTRAP_NEXT >= BOOTSTRAP_END {
            return None;
        }
        let pa = BOOTSTRAP_NEXT;
        BOOTSTRAP_NEXT += PAGE_SIZE;
        PhysAddr::new(pa)
    };
    zero_page(pa);
    Some(pa)
}

/// Allocates `count` physically contiguous pages.
///
/// Device engines that walk descriptor rings, command queues, or scatter lists
/// need a run of adjacent frames, and controllers with a narrow addressing
/// window need that run to sit below a limit. `align_pages` constrains the
/// alignment of the first frame in pages, and `max_address` bounds the last
/// addressable byte.
///
/// The run is located by scanning the frame database, so this is a setup-time
/// operation rather than a hot path.
pub fn alloc_contiguous(
    usage: PageUse,
    count: usize,
    align_pages: usize,
    max_address: u64,
) -> Option<&'static Page> {
    if !READY.load(Ordering::Acquire) || count == 0 {
        return None;
    }
    assert!(
        !matches!(
            usage,
            PageUse::None | PageUse::Reserved | PageUse::PfnDatabase
        ),
        "mem/phys: invalid dynamic page use {usage:?}"
    );
    let align = align_pages.max(1);
    if !align.is_power_of_two() {
        return None;
    }

    // Frames parked in per-CPU caches are free but unlinked, so returning them
    // to the shared lists first is what makes a long run findable.
    drain_page_caches();

    let entries = MAX_PFN.load(Ordering::Acquire);
    let limit = ((max_address.saturating_add(1)) / PAGE_SIZE) as usize;
    let entries = entries.min(limit);
    if entries < count {
        return None;
    }

    let mut guard = PMM.lock();
    let state = guard.as_mut()?;
    let minimum = if super::is_page_daemon() {
        0
    } else {
        super::reserve::DAEMON_PAGES
    };
    if state.nfree < count.saturating_add(minimum) {
        return None;
    }

    let mut start = align_index(0, align);
    let mut claimed = None;
    while start + count <= entries {
        start = align_index(super::physseg::next(SEGMENTS.get()?, start)?, align);
        if start + count > entries {
            break;
        }
        let mut index = start;
        let mut run = 0usize;
        while run < count {
            match page_by_index(index) {
                Some(page) if is_linked_free(page) => {
                    index += 1;
                    run += 1;
                }
                _ => break,
            }
        }
        if run == count {
            let mut zeroed = 0usize;
            for offset in 0..count {
                let page = page_by_index(start + offset)?;
                let was_zero = page.state() == PageState::Zero;
                // SAFETY: the page is linked into the list selected by its
                // state, and the PMM lock is held for the whole removal.
                unsafe {
                    let list = if was_zero {
                        &mut state.zero
                    } else {
                        &mut state.free
                    };
                    list.cursor_mut_from_ptr(page).remove();
                }
                if was_zero {
                    zeroed += 1;
                }
            }
            state.nfree -= count;
            claimed = Some((start, zeroed));
            break;
        }
        // `index` is the first frame that broke the run, so resume after it.
        start = align_index(index.max(start) + 1, align);
    }
    drop(guard);

    // Claiming the run happens with the allocator lock released so that the
    // per-frame assertions and the owner lock run without interrupts masked.
    let (base, zeroed) = claimed?;
    for offset in 0..count {
        let page = page_by_index(base + offset)?;
        page.prepare_allocation(usage);
    }
    FREE_PAGES.fetch_sub(count, Ordering::AcqRel);
    USED_PAGES.fetch_add(count, Ordering::AcqRel);
    if zeroed != 0 {
        ZERO_PAGES.fetch_sub(zeroed, Ordering::AcqRel);
    }
    page_by_index(base)
}

/// Releases a run previously produced by [`alloc_contiguous`].
///
/// # Safety
///
/// The same requirements as [`free_page`] apply to every frame in the run, and
/// `count` must match the original allocation.
pub unsafe fn free_contiguous(first: &'static Page, count: usize) {
    let base = first.pfn();
    for offset in 0..count {
        let Some(page) = page_by_index(base + offset) else {
            return;
        };
        // SAFETY: forwarded from this function's caller.
        unsafe { free_page(page) };
    }
}

const fn align_index(index: usize, align: usize) -> usize {
    index.next_multiple_of(align)
}

fn is_linked_free(page: &'static Page) -> bool {
    matches!(page.state(), PageState::Free | PageState::Zero) && page.link.is_linked()
}

/// Returns every per-CPU cached frame to the shared free lists.
fn drain_page_caches() {
    for cache_lock in &PAGE_CACHES {
        loop {
            let entry = {
                let mut cache = cache_lock.lock();
                match cache.pop(false) {
                    Some(pfn) => Some((pfn, false)),
                    None => cache.pop(true).map(|pfn| (pfn, true)),
                }
            };
            let Some((pfn, zeroed)) = entry else {
                break;
            };
            let Some(page) = page_by_index(pfn as usize) else {
                continue;
            };
            let mut guard = PMM.lock();
            let Some(state) = guard.as_mut() else {
                return;
            };
            state.nfree += 1;
            if zeroed {
                state.zero.push_back(page);
            } else {
                state.free.push_back(page);
            }
        }
    }
}

/// Frees an allocated page whose contents should be treated as dirty.
///
/// # Safety
///
/// No virtual mapping, device, owner, loan, or concurrent access may reference
/// `page` after this call.
pub unsafe fn free_page(page: &'static Page) {
    // SAFETY: upheld by the caller; free-page publication happens after every
    // ownership and mapping invariant is checked by `prepare_free`.
    unsafe { free_page_inner(page, false) };
}

/// Frees an allocated page known to contain only zeros.
///
/// # Safety
///
/// The same requirements as [`free_page`] apply, and every byte must be zero.
pub unsafe fn free_zeroed_page(page: &'static Page) {
    // SAFETY: upheld by the caller.
    unsafe { free_page_inner(page, true) };
}

/// Converts up to `maximum` globally queued dirty pages into zero pages.
pub fn zero_free_pages(maximum: usize) -> usize {
    if maximum == 0 || !READY.load(Ordering::Acquire) {
        return 0;
    }
    let mut pages = [None; PAGE_CACHE_REFILL];
    let target = maximum.min(PAGE_CACHE_REFILL);
    let count = {
        let mut guard = PMM.lock();
        let state = guard.as_mut().expect("mem/phys: not initialized");
        let mut count = 0usize;
        while count < target {
            let Some(page) = state.free.pop_front() else {
                break;
            };
            state.nfree -= 1;
            pages[count] = Some(page);
            count += 1;
        }
        count
    };

    for page in pages[..count].iter().flatten() {
        zero_page(page.paddr());
        page.set_status_field(STATE_SHIFT, PageState::Zero as u8);
    }
    if count != 0 {
        let mut guard = PMM.lock();
        let state = guard.as_mut().expect("mem/phys: not initialized");
        for page in pages[..count].iter().flatten() {
            state.nfree += 1;
            state.zero.push_back(*page);
        }
        ZERO_PAGES.fetch_add(count, Ordering::AcqRel);
    }
    count
}

/// Returns current physical page accounting.
pub fn stats() -> Option<PhysStats> {
    READY.load(Ordering::Acquire).then(|| PhysStats {
        total_pages: TOTAL_PAGES.load(Ordering::Acquire),
        used_pages: USED_PAGES.load(Ordering::Acquire),
        free_pages: FREE_PAGES.load(Ordering::Acquire),
        zero_pages: ZERO_PAGES.load(Ordering::Acquire),
        managed_pages: MANAGED_PAGES.load(Ordering::Acquire),
    })
}

pub(super) fn file_reclaim_possible() -> bool {
    if FREE_PAGES.load(Ordering::Acquire) != 0 {
        return true;
    }
    (0..TOTAL_PAGES.load(Ordering::Acquire)).any(|index| {
        // SAFETY: the compact page array contains exactly TOTAL_PAGES entries
        // and remains mapped for the kernel lifetime.
        let page = unsafe { &*PAGEDB.add(index) };
        page.owner_kind() == PageOwnerKind::File && page.wire_count() == 0 && page.loan_count() == 0
    })
}

pub(crate) fn activate_managed(page: &'static Page, _owner: &PageRef) {
    if page.wire_count() == 0 {
        super::policy::activate(page);
    }
}

pub(crate) fn enqueue_managed(page: &'static Page) {
    super::policy::enqueue(page);
}

pub(crate) fn remove_managed(page: &'static Page) {
    super::policy::dequeue(page);
}

pub(crate) fn deactivate_managed(page: &'static Page) {
    let _ = page.take_referenced();
    super::policy::deactivate(page);
}

fn take_page(want_zero: bool, flags: PageAllocFlags) -> Option<(&'static Page, bool)> {
    let cache_id = cache_id();
    if let Some(result) = pop_cache(cache_id, want_zero) {
        return Some(result);
    }
    refill_cache(cache_id, want_zero, flags);
    if let Some(result) = pop_cache(cache_id, want_zero) {
        return Some(result);
    }
    steal_cached_page(cache_id, want_zero)
}

fn pop_cache(cache_id: usize, want_zero: bool) -> Option<(&'static Page, bool)> {
    let mut cache = PAGE_CACHES[cache_id].lock();
    if want_zero {
        if let Some(pfn) = cache.pop(true) {
            return page_by_index(pfn as usize).map(|page| (page, true));
        }
        if let Some(pfn) = cache.pop(false) {
            return page_by_index(pfn as usize).map(|page| (page, false));
        }
    } else {
        if let Some(pfn) = cache.pop(false) {
            return page_by_index(pfn as usize).map(|page| (page, false));
        }
        if let Some(pfn) = cache.pop(true) {
            return page_by_index(pfn as usize).map(|page| (page, true));
        }
    }
    None
}

fn refill_cache(cache_id: usize, want_zero: bool, flags: PageAllocFlags) {
    let cpus = crate::sys::smp::cpu_count();
    let daemon = super::is_page_daemon();
    let mut cache = PAGE_CACHES[cache_id].lock();
    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    for _ in 0..PAGE_CACHE_REFILL {
        let access = super::reserve::access(
            state.nfree,
            cpus,
            flags.contains(PageAllocFlags::USERESERVE),
            daemon,
        );
        if access == super::reserve::Access::Denied {
            return;
        }
        let page = if want_zero {
            state
                .zero
                .pop_front()
                .map(|page| (page, true))
                .or_else(|| state.free.pop_front().map(|page| (page, false)))
        } else {
            state
                .free
                .pop_front()
                .map(|page| (page, false))
                .or_else(|| state.zero.pop_front().map(|page| (page, true)))
        };
        let Some((page, zero)) = page else {
            return;
        };
        state.nfree -= 1;
        if !cache.push(page.pfn() as u32, zero) {
            state.nfree += 1;
            if zero {
                state.zero.push_front(page);
            } else {
                state.free.push_front(page);
            }
            return;
        }
        // Near a reserve, UVM allocates just the requested page and does not
        // speculatively fill the local free-page cache.
        if access == super::reserve::Access::Single {
            return;
        }
    }
}

fn steal_cached_page(local_id: usize, want_zero: bool) -> Option<(&'static Page, bool)> {
    for (cache_id, cache_lock) in PAGE_CACHES.iter().enumerate() {
        if cache_id == local_id {
            continue;
        }
        let mut cache = cache_lock.lock();
        let result = if want_zero {
            cache
                .pop(true)
                .map(|pfn| (pfn, true))
                .or_else(|| cache.pop(false).map(|pfn| (pfn, false)))
        } else {
            cache
                .pop(false)
                .map(|pfn| (pfn, false))
                .or_else(|| cache.pop(true).map(|pfn| (pfn, true)))
        };
        if let Some((pfn, zero)) = result {
            return page_by_index(pfn as usize).map(|page| (page, zero));
        }
    }
    None
}

unsafe fn free_page_inner(page: &'static Page, zeroed: bool) {
    page.prepare_free(zeroed);
    USED_PAGES.fetch_sub(1, Ordering::AcqRel);
    FREE_PAGES.fetch_add(1, Ordering::AcqRel);
    if zeroed {
        ZERO_PAGES.fetch_add(1, Ordering::AcqRel);
    }

    let cache_id = cache_id();
    let mut cache = PAGE_CACHES[cache_id].lock();
    if cache.push(page.pfn() as u32, zeroed) {
        return;
    }

    let mut spill = [0u32; PAGE_CACHE_REFILL];
    let mut count = 0usize;
    while count < PAGE_CACHE_REFILL {
        let Some(pfn) = cache.pop(zeroed) else {
            break;
        };
        spill[count] = pfn;
        count += 1;
    }
    assert!(cache.push(page.pfn() as u32, zeroed));
    drop(cache);

    let mut guard = PMM.lock();
    let state = guard.as_mut().expect("mem/phys: not initialized");
    for pfn in spill[..count].iter().copied() {
        state.nfree += 1;
        let page = page_by_index(pfn as usize).expect("mem/phys: invalid cached PFN");
        if zeroed {
            state.zero.push_back(page);
        } else {
            state.free.push_back(page);
        }
    }
}

fn page_by_index(pfn: usize) -> Option<&'static Page> {
    let index = super::physseg::lookup(SEGMENTS.get()?, pfn)?;
    // SAFETY: immutable physical segments bound this index to their assigned
    // slice of the permanently mapped compact vm_page array.
    Some(unsafe { &*PAGEDB.add(index) })
}

fn cache_id() -> usize {
    let id = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
    assert!(
        id < MAX_PAGE_CPUS,
        "mem/phys: cpu {id} exceeds page-cache capacity"
    );
    id
}

fn entry_type_name(entry: &Entry) -> &'static str {
    match entry.entry_type {
        EntryType::USABLE => "usable",
        EntryType::RESERVED => "reserved",
        EntryType::ACPI_RECLAIMABLE => "acpi_reclaimable",
        EntryType::ACPI_NVS => "acpi_nvs",
        EntryType::BAD_MEMORY => "bad_memory",
        EntryType::BOOTLOADER_RECLAIMABLE => "bootloader_reclaimable",
        EntryType::EXECUTABLE_AND_MODULES => "executable_and_modules",
        EntryType::FRAMEBUFFER => "framebuffer",
        _ => "unknown",
    }
}

fn usable_page_range(entry: &Entry) -> Option<(u64, u64)> {
    if entry.entry_type != EntryType::USABLE {
        return None;
    }
    let start = align_up(entry.base, PAGE_SIZE);
    let end = align_down(entry.base.saturating_add(entry.length), PAGE_SIZE);
    (start < end).then_some((start, end))
}

fn zero_page(pa: PhysAddr) {
    // SAFETY: callers exclusively own `pa`, and HHDM maps the complete frame.
    unsafe {
        ptr::write_bytes(
            mem::phys_to_virt(pa).as_mut_ptr::<u8>(),
            0,
            PAGE_SIZE as usize,
        );
    }
}

fn decode_page_state(value: u8) -> PageState {
    match value {
        0 => PageState::Reserved,
        1 => PageState::Free,
        2 => PageState::Zero,
        3 => PageState::Allocated,
        _ => panic!("mem/phys: invalid page state {value}"),
    }
}

fn decode_page_use(value: u8) -> PageUse {
    match value {
        0 => PageUse::None,
        1 => PageUse::Managed,
        2 => PageUse::KernelHeap,
        3 => PageUse::PageTable,
        4 => PageUse::PfnDatabase,
        5 => PageUse::Reserved,
        _ => panic!("mem/phys: invalid page use {value}"),
    }
}

fn decode_owner_kind(value: u8) -> PageOwnerKind {
    match value {
        0 => PageOwnerKind::None,
        1 => PageOwnerKind::Anonymous,
        2 => PageOwnerKind::File,
        3 => PageOwnerKind::Kernel,
        _ => panic!("mem/phys: invalid page owner {value}"),
    }
}
