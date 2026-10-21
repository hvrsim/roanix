//! Execution restrictions shared by interrupts, locks, and blocking operations.
//!
//! Scheduler priority does not grant access to protected data. Local IRQ
//! masking excludes local hard handlers only; an SMP lock is still needed for
//! shared state. These guards are thread-affine and must not cross a wait.

use core::{marker::PhantomData, sync::atomic::Ordering};

use super::{sched, smp};
use crate::arch;

pub use smp::{PreemptGuard, disable_preemption};

/// Masks local maskable interrupts until dropped, restoring the previous state.
/// NMI-like exceptions must not use ordinary kernel locks.
#[must_use]
pub struct IrqGuard {
    restore: bool,
    // Do not retain a shared borrow of the whole CPU record across a scope:
    // trap code may update its other fields while a restriction is active.
    cpu: *const smp::CoreLocal,
    active: bool,
    _nosend: PhantomData<*mut ()>,
}

impl IrqGuard {
    /// Enters a nonblocking region with local maskable interrupts disabled.
    pub fn new() -> Self {
        let restore = arch::irqstate();
        arch::irqset(false);
        let cpu = arch::thiscpu_opt();
        if let Some(cpu) = cpu
            && cpu.irq_guard_depth.fetch_add(1, Ordering::Relaxed) == 0
        {
            cpu.irq_guard_restore.store(restore, Ordering::Relaxed);
            cpu.irq_guard_entries.fetch_add(1, Ordering::Relaxed);
            #[cfg(debug_assertions)]
            cpu.irq_guard_started
                .store(super::clock::monotonic_ns_or_zero(), Ordering::Relaxed);
        }
        Self {
            restore,
            cpu: cpu.map_or(core::ptr::null(), core::ptr::from_ref),
            active: true,
            _nosend: PhantomData,
        }
    }

    /// Transfers final IRQ restoration to the scheduler's park/wake handoff.
    pub(crate) fn leave_masked(mut self) -> bool {
        self.release()
    }

    fn release(&mut self) -> bool {
        if !self.active {
            return false;
        }
        self.active = false;
        if self.cpu.is_null() {
            return self.restore;
        }
        // SAFETY: CPU records are permanent. IRQ masking pins this thread to
        // that CPU, and this thread-affine guard accesses only its atomic state.
        let cpu = unsafe { &*self.cpu };
        let previous = cpu.irq_guard_depth.fetch_sub(1, Ordering::Relaxed);
        assert!(previous > 0, "context: IRQ guard depth underflow");
        if previous != 1 {
            return false;
        }
        #[cfg(debug_assertions)]
        {
            let elapsed = super::clock::monotonic_ns_or_zero()
                .saturating_sub(cpu.irq_guard_started.load(Ordering::Relaxed));
            cpu.irq_guard_max_ns.fetch_max(elapsed, Ordering::Relaxed);
        }
        cpu.irq_guard_restore.load(Ordering::Relaxed)
    }
}

impl Default for IrqGuard {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for IrqGuard {
    fn drop(&mut self) {
        if self.release() {
            arch::irqset(true);
        }
    }
}

/// Checks the complete thread-context contract, including uncontended paths.
/// Early initialization may use uncontended locks before scheduling starts.
pub fn assert_sleepable(operation: &str) {
    smp::assert_blockable(operation);
    assert!(
        arch::thiscpu_opt().is_none_or(|cpu| cpu.irq_guard_depth.load(Ordering::Relaxed) == 0),
        "{operation}: an IRQ guard is held"
    );
    if sched::current_thread_opt().is_some() {
        assert!(arch::irqstate(), "{operation}: local IRQs are masked");
    }
}

/// Per-CPU IRQ restriction measurements. Durations are collected in debug builds.
#[derive(Clone, Copy, Debug)]
pub struct IrqStats {
    /// Number of outermost guarded sections entered on this CPU.
    pub sections: u64,
    /// Longest guarded section, including lock contention, in nanoseconds.
    pub max_masked_ns: u64,
}

/// Returns measurements for a discovered CPU.
pub fn irq_stats(cpu_id: usize) -> Option<IrqStats> {
    let cpu = smp::core_local(cpu_id)?;
    Some(IrqStats {
        sections: cpu.irq_guard_entries.load(Ordering::Relaxed),
        max_masked_ns: cpu.irq_guard_max_ns.load(Ordering::Relaxed),
    })
}
