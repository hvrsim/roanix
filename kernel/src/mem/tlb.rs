//!
//! # TLB Shootdown
//!
//! Address-granular remote TLB invalidation shared by the kernel heap and the
//! pageable virtual memory system.
//!
//! Invalidations are appended to a lock-free ring keyed by a monotonic
//! sequence number. Each CPU records the highest sequence it has applied, so a
//! responder replays only the entries it has not yet seen instead of
//! discarding its whole TLB. When a CPU falls further behind than the ring can
//! retain, the engine degrades to a single full flush for that CPU rather than
//! stalling producers.
//!
//! Every entry carries the page-table root it belongs to. A CPU skips entries
//! for address spaces it is not running, and initiators correspondingly wait
//! only for the CPUs that could hold the translation.
//!

use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering, fence};

use crate::{
    arch,
    mem::{PhysAddr, VirtAddr},
    sys::{smp, sync::Mutex},
};

/// Maximum CPUs tracked by shootdown acknowledgement state.
pub const MAX_TLB_CPUS: usize = 256;

/// Number of retained invalidation records.
///
/// Producers never block on this bound; exceeding it only forces a lagging
/// responder to take one full flush.
const RING_CAPACITY: usize = 4096;

/// Root tag marking an invalidation that applies to every address space.
const ROOT_GLOBAL: u64 = 0;

/// Cache-line aligned counter, preventing false sharing between CPUs that
/// acknowledge shootdowns concurrently.
#[repr(align(64))]
struct Ack(AtomicU64);

/// One published invalidation record.
struct Record {
    address: AtomicU64,
    root: AtomicU64,
}

struct Engine {
    /// Highest sequence number that is fully written and visible.
    published: AtomicU64,
    /// Sequence at or below which records have been overwritten.
    overflow: AtomicU64,
    /// Serializes producers so `published` advances contiguously.
    producer: Mutex<u64>,
    ring: [Record; RING_CAPACITY],
    acked: [Ack; MAX_TLB_CPUS],
    /// Full flushes taken because a CPU fell behind the ring.
    overflow_flushes: AtomicUsize,
}

static ENGINE: Engine = Engine {
    published: AtomicU64::new(0),
    overflow: AtomicU64::new(0),
    producer: Mutex::new(0),
    ring: [const {
        Record {
            address: AtomicU64::new(0),
            root: AtomicU64::new(ROOT_GLOBAL),
        }
    }; RING_CAPACITY],
    acked: [const { Ack(AtomicU64::new(0)) }; MAX_TLB_CPUS],
    overflow_flushes: AtomicUsize::new(0),
};

/// A set of pending invalidations accumulated before publication.
///
/// Callers batch every address touched by one logical operation so a single
/// round of inter-processor interrupts covers all of them.
pub(super) struct Shootdown {
    root: u64,
    addresses: [u64; Self::INLINE],
    count: usize,
    /// Set when more addresses were added than the batch can hold, which
    /// escalates the operation to a full flush.
    saturated: bool,
}

impl Shootdown {
    /// Addresses recorded before escalating to a full flush.
    pub(super) const INLINE: usize = 64;

    /// Creates a batch targeting one address-space root.
    pub(super) fn for_root(root: PhysAddr) -> Self {
        Self {
            root: root.as_u64(),
            addresses: [0; Self::INLINE],
            count: 0,
            saturated: false,
        }
    }

    /// Creates a batch targeting kernel mappings in every address space.
    pub(super) fn global() -> Self {
        Self {
            root: ROOT_GLOBAL,
            addresses: [0; Self::INLINE],
            count: 0,
            saturated: false,
        }
    }

    /// Returns whether any invalidation has been recorded.
    pub(super) fn is_empty(&self) -> bool {
        self.count == 0 && !self.saturated
    }

    /// Records one page that must be invalidated.
    pub(super) fn push(&mut self, address: VirtAddr) {
        if self.saturated {
            return;
        }
        if self.count == Self::INLINE {
            self.saturated = true;
            return;
        }
        self.addresses[self.count] = address.align_down().as_u64();
        self.count += 1;
    }

    /// Escalates the batch to a full flush on every CPU.
    ///
    /// Used when the affected addresses are not enumerable, such as after a
    /// bulk kernel remapping.
    pub(super) fn saturate(&mut self) {
        self.saturated = true;
    }

    /// Publishes the batch and waits until every relevant CPU has applied it.
    ///
    /// The local CPU is invalidated inline by the architecture routines that
    /// edited the page tables, so only remote CPUs are awaited.
    pub(super) fn commit(self) {
        if self.is_empty() {
            return;
        }
        commit_batch(self.root, &self.addresses[..self.count], self.saturated);
    }
}

/// Applies every invalidation this CPU has not yet observed.
///
/// Runs both from the shootdown inter-processor interrupt and from explicit
/// polling points such as address-space activation.
pub(crate) fn poll() {
    let Some(cpu) = arch::thiscpu_opt() else {
        return;
    };
    if cpu.id >= MAX_TLB_CPUS {
        // Untracked CPUs cannot be awaited, so they conservatively discard
        // every translation they may hold.
        arch::paging::flush_all_global();
        return;
    }

    let published = ENGINE.published.load(Ordering::Acquire);
    let seen = ENGINE.acked[cpu.id].0.load(Ordering::Relaxed);
    if seen >= published {
        return;
    }

    if apply_range(cpu, seen, published) {
        ENGINE.acked[cpu.id].0.store(published, Ordering::Release);
        return;
    }

    ENGINE.overflow_flushes.fetch_add(1, Ordering::Relaxed);
    arch::paging::flush_all_global();
    ENGINE.acked[cpu.id].0.store(published, Ordering::Release);
}

/// Marks `cpu_id` as current with respect to already published invalidations.
///
/// A CPU that has never run cannot hold stale translations, and a CPU that has
/// just reloaded its root has discarded every non-global entry.
pub(crate) fn register_cpu(cpu_id: usize) {
    assert!(
        cpu_id < MAX_TLB_CPUS,
        "mem/tlb: cpu {cpu_id} exceeds shootdown tracking capacity"
    );
    let published = ENGINE.published.load(Ordering::Acquire);
    ENGINE.acked[cpu_id].0.store(published, Ordering::Release);
}

/// Returns the number of full flushes forced by ring overflow.
pub(crate) fn overflow_flushes() -> usize {
    ENGINE.overflow_flushes.load(Ordering::Relaxed)
}

/// Publishes one kernel invalidation without waiting for acknowledgement.
///
/// Callers that quarantine the underlying resource use the returned sequence
/// with [`acknowledged_through`] to learn when releasing it is safe, which
/// keeps the publishing path free of inter-processor round trips.
pub(super) fn publish_async(address: VirtAddr) -> u64 {
    let sequence = {
        let mut next = ENGINE.producer.lock();
        let sequence = next.checked_add(1).expect("mem/tlb: sequence wrapped");
        publish_record(sequence, ROOT_GLOBAL, address.align_down().as_u64());
        ENGINE.overflow.fetch_max(
            sequence.saturating_sub(RING_CAPACITY as u64),
            Ordering::AcqRel,
        );
        *next = sequence;
        ENGINE.published.store(sequence, Ordering::Release);
        sequence
    };

    if let Some(cpu) = arch::thiscpu_opt()
        && cpu.id < MAX_TLB_CPUS
    {
        // The caller invalidated this address locally while unmapping it, but
        // earlier records may still be outstanding here, so only a genuine
        // replay may advance this CPU's acknowledgement.
        poll();
        ENGINE.acked[cpu.id].0.fetch_max(sequence, Ordering::AcqRel);
    }

    fence(Ordering::SeqCst);
    let _ = smp::send_ipi(poll, smp::IpiTarget::All);
    sequence
}

/// Returns the highest sequence every online CPU has applied.
pub(super) fn acknowledged_through() -> u64 {
    let published = ENGINE.published.load(Ordering::Acquire);
    if smp::online_cpus() <= 1 {
        return published;
    }
    let mut applied = published;
    for cpu_id in 0..smp::cpu_count().min(MAX_TLB_CPUS) {
        if smp::is_online(cpu_id) {
            applied = applied.min(ENGINE.acked[cpu_id].0.load(Ordering::Acquire));
        }
    }
    applied
}

/// Forces every CPU to discard its translations and returns the sequence that
/// is then guaranteed to be applied everywhere.
pub(super) fn flush_everything() -> u64 {
    let mut shootdown = Shootdown::global();
    shootdown.saturate();
    shootdown.commit();
    ENGINE.published.load(Ordering::Acquire)
}

/// Replays `(seen, published]` on the current CPU.
///
/// Returns `false` when producers overwrote part of the range, in which case
/// the caller must fall back to a full flush.
fn apply_range(cpu: &smp::CoreLocal, seen: u64, published: u64) -> bool {
    if seen < ENGINE.overflow.load(Ordering::Acquire) {
        return false;
    }

    let active = cpu.active_address_root.load(Ordering::Acquire);
    for sequence in (seen + 1)..=published {
        let record = &ENGINE.ring[slot(sequence)];
        let root = record.root.load(Ordering::Acquire);
        if root != ROOT_GLOBAL && root != active {
            continue;
        }
        arch::paging::flush_page(VirtAddr::new(record.address.load(Ordering::Relaxed)));
    }

    // Records may have been recycled while they were being replayed, so the
    // bound is rechecked before the applied range is acknowledged.
    seen >= ENGINE.overflow.load(Ordering::Acquire)
}

fn commit_batch(root: u64, addresses: &[u64], saturated: bool) {
    if smp::online_cpus() <= 1 {
        return;
    }

    // Migrating between reserving a sequence and waiting for it would leave
    // this CPU's acknowledgement attributed to the wrong core.
    let _pin = super::MigrationPin::current();

    let sequence = {
        let mut next = ENGINE.producer.lock();
        let start = *next;
        let count = if saturated { 1 } else { addresses.len() as u64 };
        let end = start
            .checked_add(count)
            .expect("mem/tlb: shootdown sequence wrapped");

        if saturated {
            // A full flush is expressed as a record no responder can match by
            // address, combined with an overflow bound that forces the
            // fallback path.
            publish_record(end, root, 0);
            ENGINE.overflow.fetch_max(end, Ordering::AcqRel);
        } else {
            for (offset, address) in addresses.iter().enumerate() {
                publish_record(start + offset as u64 + 1, root, *address);
            }
            ENGINE
                .overflow
                .fetch_max(end.saturating_sub(RING_CAPACITY as u64), Ordering::AcqRel);
        }

        *next = end;
        ENGINE.published.store(end, Ordering::Release);
        end
    };

    if let Some(cpu) = arch::thiscpu_opt()
        && cpu.id < MAX_TLB_CPUS
    {
        // The architecture routines already invalidated these addresses
        // locally while editing the leaves.
        ENGINE.acked[cpu.id].0.fetch_max(sequence, Ordering::AcqRel);
    }

    // The page-table edits and the published records must be globally visible
    // before remote address-space roots are sampled. Without this barrier a
    // CPU could be skipped for not running the root while it concurrently
    // installs that root and walks a stale entry.
    fence(Ordering::SeqCst);

    let _ = smp::send_ipi(poll, smp::IpiTarget::All);
    wait_for_acknowledgements(root, sequence);
}

fn publish_record(sequence: u64, root: u64, address: u64) {
    let record = &ENGINE.ring[slot(sequence)];
    record.address.store(address, Ordering::Relaxed);
    record.root.store(root, Ordering::Release);
}

fn wait_for_acknowledgements(root: u64, sequence: u64) {
    let cpu_count = smp::cpu_count().min(MAX_TLB_CPUS);
    loop {
        let mut pending = false;
        for cpu_id in 0..cpu_count {
            if ENGINE.acked[cpu_id].0.load(Ordering::Acquire) >= sequence {
                continue;
            }
            if !smp::is_online(cpu_id) {
                continue;
            }
            // A CPU that is not running this address space cannot hold a
            // translation for it. Roots are published before activation and
            // cleared after deactivation, so this window is a superset of the
            // interval during which the root is truly installed.
            if root != ROOT_GLOBAL
                && let Some(cpu) = smp::core_local(cpu_id)
                && cpu.active_address_root.load(Ordering::Acquire) != root
            {
                continue;
            }
            pending = true;
        }
        if !pending {
            return;
        }
        core::hint::spin_loop();
    }
}

#[inline(always)]
fn slot(sequence: u64) -> usize {
    ((sequence - 1) % RING_CAPACITY as u64) as usize
}
