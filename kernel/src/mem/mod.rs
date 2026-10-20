//! Memory subsystem initialization and shared kernel memory services.
//!
//! This module connects physical allocation, address spaces, anonymous memory,
//! file-backed objects, swapping, and translation invalidation. It establishes
//! reclamation watermarks, runs the page daemon, coordinates memory waiters,
//! and exposes address conversion and address-space activation helpers.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

use ::alloc::sync::Arc;
use core::{
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    time::Duration,
};

use bitflags::bitflags;
use limine::{
    memory_map::Entry,
    request::{HhdmRequest, MemoryMapRequest},
};
use log::{debug, info};

use crate::{
    arch,
    sys::{
        clock,
        event::{Event, SequenceEvent},
        sched, smp,
        sync::{Mutex, Once},
    },
};

pub mod addr;
pub mod alloc;
mod anon;
mod error;
mod fault;
mod index;
mod io;
pub(crate) mod kstack;
mod map;
mod map_lock;
mod metadata;
mod object;
mod owner;
mod page;
pub mod phys;
mod physseg;
mod pmap;
pub(crate) mod policy;
mod pool;
mod reserve;
pub mod swap;
mod syscall;
mod tlb;
mod vmem;

pub use addr::{PAGE_SIZE, PhysAddr, VirtAddr, align_down, align_up, pages_for_len};
pub(crate) use alloc::HEAP_BASE;
pub use error::{Error, Result};
pub use io::{IoSink, IoSource, SinkWindow, SourceWindow};
pub use map::{
    FaultAccess, USER_ADDRESS_MAX, USER_ADDRESS_MIN, VmAdvice, VmBacking, VmInheritance, VmMap,
    VmMapEntry, VmMapping, VmPlacement, VmProtection,
};
pub use object::{ObjectKind, PageAccount, PageCacheStats, VmObject, VmPager};
pub use page::{PageInfo, PageLocation, PageRef, PageWeak, VmPage};
pub use pmap::{Pmap, VmSpace};
pub use swap::{SwapBackend, SwapError, SwapStats};

const PAGE_SCAN_BATCH: usize = 128;
const PAGE_DAEMON_INTERVAL: Duration = Duration::from_millis(100);
static DAEMON_WAKE: Event = Event::new();
static PAGE_DAEMON_THREAD: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);
static MEMORY_AVAILABLE: SequenceEvent = SequenceEvent::new();

bitflags! {
    /// Virtual memory mapping permissions and attributes used by low-level paging.
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    pub struct VmFlags: u32 {
        /// Permit reads.
        const READ    = 1 << 0;
        /// Permit writes.
        const WRITE   = 1 << 1;
        /// Permit instruction fetch.
        const EXECUTE = 1 << 2;
        /// User-accessible mapping.
        const USER    = 1 << 3;
        /// Global mapping that should survive context switches.
        const GLOBAL  = 1 << 4;
        /// Device/uncached style mapping.
        const DEVICE  = 1 << 5;
        /// Write-combining mapping for large streaming writes such as a
        /// framebuffer.
        const WRITE_COMBINE = 1 << 6;
    }
}

struct MemoryState {
    kernel_root: PhysAddr,
    next_page_id: AtomicU64,
    next_object_id: AtomicU64,
    reclaimed_pages: AtomicU64,
    faults: AtomicU64,
    promotions: AtomicU64,
    reclaiming: Mutex<()>,
    daemon_started: AtomicBool,
}

struct MigrationPin {
    thread: Option<*mut crate::sys::thread::Thread>,
}

impl MigrationPin {
    fn current() -> Self {
        let thread = sched::current_thread_opt();
        if let Some(thread) = thread {
            // SAFETY: the scheduler keeps the current thread live while it is
            // executing on this CPU.
            unsafe { &*thread }.pin_migration();
        }
        Self { thread }
    }
}

impl Drop for MigrationPin {
    fn drop(&mut self) {
        if let Some(thread) = self.thread {
            // SAFETY: the matching pin keeps this current-thread allocation
            // alive and local until the decrement completes.
            unsafe { &*thread }.unpin_migration();
        }
    }
}

/// Global memory accounting snapshot.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct VmStats {
    /// Resident object and anonymous pages.
    pub resident_pages: u64,
    /// Pages successfully reclaimed to swap.
    pub reclaimed_pages: u64,
    /// Faults resolved by the memory subsystem.
    pub faults: u64,
    /// COW promotions completed.
    pub promotions: u64,
    /// Full TLB flushes forced by a CPU falling behind the shootdown ring.
    pub tlb_overflow_flushes: u64,
    /// Current compressed/external swap counters.
    pub swap: SwapStats,
}

static MEMORY: Once<MemoryState> = Once::new();

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static HHDM_REQUEST: HhdmRequest = HhdmRequest::new();

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static MEMORY_MAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

/// Initializes physical memory, the heap, and pageable virtual memory.
pub fn init() {
    debug!("direct map based at 0x{:x}", hhdm_offset());
    phys::init();
    alloc::init();
    init_state();
}

fn init_state() {
    if MEMORY.get().is_some() {
        return;
    }
    let physical = phys::stats().expect("mem: physical memory manager is not initialized");
    let (low_watermark, high_watermark) =
        reserve::watermarks(physical.total_pages, smp::cpu_count());
    let compressed_limit = (physical.total_pages as u64)
        .saturating_mul(PAGE_SIZE)
        .saturating_div(4);

    MEMORY.call_once(|| MemoryState {
        kernel_root: arch::paging::active_root(),
        next_page_id: AtomicU64::new(1),
        next_object_id: AtomicU64::new(1),
        reclaimed_pages: AtomicU64::new(0),
        faults: AtomicU64::new(0),
        promotions: AtomicU64::new(0),
        reclaiming: Mutex::new(()),
        daemon_started: AtomicBool::new(false),
    });
    // Kernel-half tables must exist before any user root copies them, so that
    // later kernel mappings are shared rather than missed.
    // SAFETY: the captured root is the permanent kernel hierarchy and no user
    // address space exists yet.
    unsafe { arch::paging::populate_kernel_tables(state().kernel_root) }
        .expect("mem: failed to reserve kernel page-table entries");
    swap::init(compressed_limit, physical.total_pages / 4);
    register_cpu();

    info!(
        "ready: reclaim watermarks low={} high={}, {} MiB of compressed swap",
        low_watermark,
        high_watermark,
        compressed_limit / (1024 * 1024),
    );
}

/// Starts the page daemon after the scheduler is running.
pub(crate) fn start_page_daemon() {
    let state = state();
    if state
        .daemon_started
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        debug!("starting the page daemon");
        sched::run(page_daemon);
    }
}

/// Returns memory accounting and swap traffic.
pub fn stats() -> VmStats {
    let state = state();
    let physical = phys::stats().expect("mem: physical memory manager is not initialized");
    VmStats {
        resident_pages: physical.managed_pages as u64,
        reclaimed_pages: state.reclaimed_pages.load(Ordering::Relaxed),
        faults: state.faults.load(Ordering::Relaxed),
        promotions: state.promotions.load(Ordering::Relaxed),
        tlb_overflow_flushes: tlb::overflow_flushes() as u64,
        swap: swap::stats(),
    }
}

/// Reclaims up to `target` resident pages synchronously.
pub fn reclaim_now(target: usize) -> usize {
    let Some(_reclaim) = state().reclaiming.try_lock() else {
        return 0;
    };
    reclaim_pages(target)
}

/// Scans under `MemoryState::reclaiming`, which serializes reclaim passes.
fn reclaim_pages(target: usize) -> usize {
    let mut reclaimed = 0usize;
    // One scan marker spans the whole pass, so a contended prefix cannot
    // starve later candidates. Balance follows each scan; the
    // synchronous reclaim API may repeat the daemon cycle once after aging.
    for _ in 0..2 {
        if reclaimed >= target {
            break;
        }
        let progress = policy::scan(usize::MAX, target - reclaimed);
        reclaimed += progress;
        let shortage = if progress == 0 && !swap::has_capacity() && !swap::is_full() {
            free_target().saturating_sub(phys::stats().map_or(0, |s| s.free_pages as u64)) as usize
        } else {
            0
        };
        policy::balance(usize::MAX, shortage);
        if progress == 0 && policy::queues_empty() {
            break;
        }
    }
    if reclaimed >= PAGE_SCAN_BATCH {
        let swap = swap::stats();
        debug!(
            target: "mem::swap",
            "reclaimed {} pages (resident={}, swapped={}, payload={} KiB, in={}, out={}, rejected={})",
            reclaimed,
            phys::stats().map_or(0, |stats| stats.managed_pages),
            swap.compressed_pages + swap.same_fill_pages + swap.external_pages,
            swap.compressed_bytes / 1024,
            swap.pageins,
            swap.pageouts,
            swap.rejected_pages,
        );
    }
    reclaimed
}

/// Registers the current CPU with TLB shootdown tracking.
pub(crate) fn register_cpu() {
    let cpu_id = arch::thiscpu().id;
    arch::thiscpu()
        .active_address_root
        .store(arch::paging::active_root().as_u64(), Ordering::Release);
    tlb::register_cpu(cpu_id);
}

/// Resolves a fault in the current thread's address space.
pub fn handle_current_fault(address: VirtAddr, access: FaultAccess) -> Result<()> {
    let space = current_space().ok_or(Error::NotMapped)?;
    space.fault(address, access).inspect_err(|error| {
        debug!(target: "mem::fault", "root={:#x} address={:#x} access={access:?}: {error}",
            space.pmap().root(), address);
    })
}

/// Installs an address space on the current thread and activates its pmap.
pub fn install_current_space(space: Arc<VmSpace>) {
    let thread = sched::current_thread();
    // SAFETY: the current thread owns its address-space slot while executing;
    // the slot itself is protected by an IRQ-safe spin lock.
    let previous = unsafe { &*thread }.replace_address_space(Some(space.clone()));
    activate_thread_space(Some(space));
    drop(previous);
}

/// Removes the current thread's user address space and restores the kernel pmap.
pub fn clear_current_space() {
    let thread = sched::current_thread();
    // SAFETY: the current thread owns its address-space slot while executing.
    let previous = unsafe { &*thread }.replace_address_space(None);
    activate_kernel_space();
    drop(previous);
}

/// Returns the current thread's address space.
pub fn current_space() -> Option<Arc<VmSpace>> {
    let thread = sched::current_thread_opt()?;
    // SAFETY: scheduler current-thread pointers remain live while executing.
    unsafe { &*thread }.address_space()
}

pub(crate) fn activate_thread_space(space: Option<Arc<VmSpace>>) {
    let Some(space) = space else {
        // Kernel-only threads can safely borrow the previous user pmap because
        // every address space contains the same permanent kernel mappings.
        return;
    };
    let _preempt = smp::disable_preemption();
    let root = space.pmap().root();
    let active = arch::thiscpu().active_address_root.load(Ordering::Acquire);
    if active == root.as_u64() {
        debug_assert_eq!(arch::paging::active_root(), root);
        return;
    }

    // Retain the outgoing root's advertised lifetime until the hardware
    // switch completes. The incoming space is still owned by `space`.
    arch::thiscpu()
        .previous_address_root
        .store(active, Ordering::Release);
    arch::thiscpu()
        .active_address_root
        .store(root.as_u64(), Ordering::Release);
    core::sync::atomic::fence(Ordering::SeqCst);
    if arch::paging::active_root() != root {
        space
            .activate()
            .expect("mem: failed to activate scheduled address space");
    }
    arch::thiscpu()
        .previous_address_root
        .store(0, Ordering::Release);
    // Entries published while this CPU ran another space must be applied
    // before user code observes the new one.
    tlb::poll();
}

/// Returns whether any CPU still has `root` installed.
pub(crate) fn address_space_root_active(root: u64) -> bool {
    (0..smp::cpu_count()).any(|cpu_id| {
        smp::core_local(cpu_id).is_some_and(|cpu| {
            cpu.active_address_root.load(Ordering::Acquire) == root
                || cpu.previous_address_root.load(Ordering::Acquire) == root
        })
    })
}

pub(super) fn retire_address_space_root(root: PhysAddr) {
    // Kernel threads may still borrow a root after its final owner drops it.
    // No new user can activate it once the final pmap reference is gone.
    while address_space_root_active(root.as_u64()) {
        release_inactive_address_space_roots();
        tlb::poll();
        core::hint::spin_loop();
    }
}

/// Requests CPUs running kernel-only threads to release borrowed user roots.
pub(crate) fn release_inactive_address_space_roots() {
    release_inactive_address_space_root();
    let _ = smp::send_ipi(release_inactive_address_space_root, smp::IpiTarget::All);
}

fn release_inactive_address_space_root() {
    // An IPI must not replace a root while its owner is publishing a switch.
    // The reaper retries roots that remain active.
    if arch::thiscpu().preempt_depth.load(Ordering::Relaxed) != 0 {
        return;
    }
    let _preempt = smp::disable_preemption();
    let cpu = arch::thiscpu();
    let active = cpu.active_address_root.load(Ordering::Acquire);
    if active == 0 || active == kernel_root().as_u64() {
        return;
    }
    let current_root = sched::current_thread_opt().and_then(|thread| {
        // SAFETY: scheduler current-thread pointers remain live while executing.
        unsafe { &*thread }.address_space_root()
    });
    if current_root == Some(active) {
        return;
    }
    activate_kernel_space();
}

pub(super) fn allocate_page_id() -> u64 {
    next_id(&state().next_page_id, "page")
}

pub(super) fn allocate_object_id() -> u64 {
    next_id(&state().next_object_id, "object")
}

fn free_minimum() -> u64 {
    reserve::watermarks(phys::stats().map_or(0, |s| s.total_pages), smp::cpu_count()).0
}
fn free_target() -> u64 {
    reserve::watermarks(phys::stats().map_or(0, |s| s.total_pages), smp::cpu_count()).1
}

pub(super) fn replenish_reserve() {
    if let Some(stats) = phys::stats()
        && stats.free_pages as u64 <= free_minimum()
    {
        // A competing pass is work in progress, not failed reclamation.
        // Wait before consuming its reserve and recheck after acquiring it.
        // Callers must not hold a backing or map lock needed by reclamation.
        let _reclaim = state().reclaiming.lock();
        let free = phys::stats().map_or(0, |stats| stats.free_pages as u64);
        if free <= free_minimum() {
            let target = free_target().saturating_sub(free) as usize;
            let _ = reclaim_pages(target.max(1));
        }
    }
}

pub(super) fn allocate_physical_page() -> Result<&'static phys::Page> {
    replenish_reserve();
    if let Some(page) = phys::alloc_zeroed_page(phys::PageUse::Managed) {
        return Ok(page);
    }
    allocate_after_reclaim(PAGE_SCAN_BATCH)
}

/// Allocates the destination of a swap-in without first creating more swap.
///
/// Normal faults refill the free-page high watermark before allocating. A
/// page-in first consumes available free storage without another reclaim
/// pass. Its retained swap slot stays authoritative until the load completes;
/// clean resident swap-backed pages can subsequently be evicted without I/O.
pub(super) fn allocate_pagein_physical_page() -> Result<&'static phys::Page> {
    if let Some(page) = phys::alloc_zeroed_page(phys::PageUse::Managed) {
        return Ok(page);
    }
    allocate_after_reclaim(1)
}

fn allocate_after_reclaim(target: usize) -> Result<&'static phys::Page> {
    let _reclaim = state().reclaiming.lock();
    // The previous owner may already have made enough progress.
    if let Some(page) = phys::alloc_zeroed_page(phys::PageUse::Managed) {
        return Ok(page);
    }
    let _ = reclaim_pages(target);
    phys::alloc_zeroed_page(phys::PageUse::Managed).ok_or(Error::OutOfMemory)
}

pub(super) fn page_reclaimed() {
    state().reclaimed_pages.fetch_add(1, Ordering::Relaxed);
}

pub(super) fn record_fault(promoted: bool) {
    state().faults.fetch_add(1, Ordering::Relaxed);
    if promoted {
        state().promotions.fetch_add(1, Ordering::Relaxed);
    }
}

pub(super) fn kernel_root() -> PhysAddr {
    state().kernel_root
}

/// Returns the page-table root that owns the shared kernel address space.
///
/// Kernel-half mappings must be installed in this root so that every address
/// space created afterwards inherits them.
pub fn kernel_page_root() -> PhysAddr {
    state().kernel_root
}

/// Invalidates `length` bytes of kernel virtual address space on every CPU.
pub fn flush_tlb_range(base: VirtAddr, length: u64) {
    let mut shootdown = tlb::Shootdown::global();
    for index in 0..length.div_ceil(PAGE_SIZE) {
        let address = VirtAddr::new(base.as_u64() + index * PAGE_SIZE);
        arch::paging::flush_page(address);
        shootdown.push(address);
    }
    shootdown.commit();
}

/// Returns whether the boot protocol's direct map covers `pa`.
pub fn is_direct_mapped(pa: PhysAddr) -> bool {
    let address = pa.as_u64();
    memory_map_entries().iter().any(|entry| {
        let end = entry.base.saturating_add(entry.length);
        (entry.base..end).contains(&address)
    })
}

/// Publishes kernel mapping or permission changes to every online CPU.
pub(crate) fn synchronize_kernel_mappings() {
    let mut shootdown = tlb::Shootdown::global();
    shootdown.saturate();
    shootdown.commit();
}

/// Returns Limine memory-map entries by reference.
pub fn memory_map_entries() -> &'static [&'static Entry] {
    MEMORY_MAP_REQUEST
        .get_response()
        .expect("mem: Limine memory map response missing")
        .entries()
}

/// Returns HHDM offset provided by Limine.
pub fn hhdm_offset() -> u64 {
    HHDM_REQUEST
        .get_response()
        .expect("mem: Limine HHDM response missing")
        .offset()
}

/// Converts physical address to HHDM virtual address.
pub fn phys_to_virt(pa: PhysAddr) -> VirtAddr {
    let raw = pa
        .as_u64()
        .checked_add(hhdm_offset())
        .expect("mem: HHDM conversion overflow");
    VirtAddr::new(raw)
}

/// Converts HHDM virtual address back to physical address.
pub fn virt_to_phys_hhdm(va: VirtAddr) -> Option<PhysAddr> {
    let raw = va.as_u64();
    let off = hhdm_offset();
    if raw < off {
        return None;
    }

    Some(PhysAddr::new(raw - off))
}

pub(super) fn is_page_daemon() -> bool {
    sched::current_thread_opt()
        .is_some_and(|thread| PAGE_DAEMON_THREAD.load(Ordering::Relaxed) == thread as usize)
}

fn page_daemon() -> ! {
    PAGE_DAEMON_THREAD.store(sched::current_thread() as usize, Ordering::Release);
    loop {
        clock::wait_timeout(&DAEMON_WAKE, PAGE_DAEMON_INTERVAL);
        DAEMON_WAKE.reset();
        let free = phys::stats().map_or(0, |stats| stats.free_pages as u64);
        if free < free_target() {
            // Drop optional VFS references from the daemon, never from an
            // allocator retry that may already hold filesystem/object locks.
            crate::fs::namecache::reclaim(32);
            let target = free_target().saturating_sub(free) as usize;
            let _ = reclaim_now(target.max(1));
        } else {
            policy::balance(32, 0);
            let _ = phys::zero_free_pages(8);
        }
        MEMORY_AVAILABLE.notify();
    }
}

pub(super) fn prepare_memory_wait() -> u64 {
    MEMORY_AVAILABLE.observe()
}

pub(super) fn wait_for_memory(ticket: u64) -> Result<()> {
    if !state().daemon_started.load(Ordering::Acquire) {
        return Err(Error::OutOfMemory);
    }
    if !swap::has_capacity() && !phys::file_reclaim_possible() {
        return Err(Error::OutOfMemory);
    }
    DAEMON_WAKE.signal();
    MEMORY_AVAILABLE.wait(ticket);
    Ok(())
}

pub(crate) fn activate_kernel_space() {
    let _preempt = smp::disable_preemption();
    if arch::paging::active_root() != kernel_root() {
        // SAFETY: the root was captured from the bootloader-provided kernel
        // page table and remains live for the kernel lifetime.
        unsafe { arch::paging::activate_root(kernel_root()) }
            .expect("mem: failed to restore kernel pmap");
    }
    // Clearing the advertised root after the switch keeps the advertised
    // interval a superset of the real one.
    arch::thiscpu()
        .active_address_root
        .store(kernel_root().as_u64(), Ordering::Release);
    tlb::poll();
}

fn state() -> &'static MemoryState {
    MEMORY.get().expect("mem: initialized before use")
}

fn next_id(counter: &AtomicU64, kind: &str) -> u64 {
    let id = counter.fetch_add(1, Ordering::Relaxed);
    if id == 0 {
        panic!("mem: {kind} identifier wrapped");
    }
    id
}
