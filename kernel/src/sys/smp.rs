//!
//! # Multicore Support
//!
//! This module is responsible for managing multiple CPU cores. Main duties
//! include AP bringup and core local data definitions.
//!

use core::ops::{Deref, DerefMut};
use spin::{Mutex, MutexGuard};

use crate::arch;

/// Platform specific core-local fields.
pub struct PlatformFields {
    /// Bitmap of supported x86 extensions.
    #[cfg(target_arch = "x86_64")]
    pub feats: arch::cpu::CpuFeatures,
}

/// Kernel context unique to each CPU core.
pub struct CoreLocal {
    /// Stack used by the kernel on IRQs.
    pub kernel_stack: u64,

    /// Placeholder for user stack on IRQs.
    pub user_stack: u64,

    /// ID of current CPU core.
    pub id: usize,

    /// Per-CPU timer tick counter.
    pub ticks: u64,

    /// Platform specific context.
    #[allow(dead_code)]
    pub platform: PlatformFields,
}

impl PlatformFields {
    /// Creates a new `PlatformFields` instance.
    pub const fn new() -> Self {
        PlatformFields {
            #[cfg(target_arch = "x86_64")]
            feats: arch::cpu::CpuFeatures::empty(),
        }
    }
}

impl CoreLocal {
    /// Creates a new core local context, with CPU ID `cid`.
    pub const fn new(cid: usize) -> Self {
        Self {
            id: cid,
            kernel_stack: 0,
            user_stack: 0,
            ticks: 0,
            platform: PlatformFields::new(),
        }
    }
}

/// `spin::Mutex` wrapper that masks interrupts while holding the lock.
pub struct IrqSpinLock<T> {
    inner: Mutex<T>,
}

/// Guard for [`IrqSpinLock`].
pub struct IrqSpinLockGuard<'a, T> {
    guard: Option<MutexGuard<'a, T>>,
    irq_enabled: bool,
}

impl<T> IrqSpinLock<T> {
    /// Creates an IRQ-safe mutex with initial payload `value`.
    pub const fn new(value: T) -> Self {
        Self {
            inner: Mutex::new(value),
        }
    }

    /// Locks the mutex while interrupts are masked.
    pub fn lock(&self) -> IrqSpinLockGuard<'_, T> {
        let irq_enabled = arch::irqstate();
        arch::irqset(false);

        IrqSpinLockGuard {
            guard: Some(self.inner.lock()),
            irq_enabled,
        }
    }

    /// Attempts to lock the mutex while interrupts are masked.
    pub fn try_lock(&self) -> Option<IrqSpinLockGuard<'_, T>> {
        let irq_enabled = arch::irqstate();
        arch::irqset(false);

        let guard = self.inner.try_lock();
        if guard.is_none() && irq_enabled {
            arch::irqset(true);
        }

        guard.map(|guard| IrqSpinLockGuard {
            guard: Some(guard),
            irq_enabled,
        })
    }
}

impl<T> Deref for IrqSpinLockGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.guard.as_ref().unwrap()
    }
}

impl<T> DerefMut for IrqSpinLockGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.guard.as_mut().unwrap()
    }
}

impl<T> Drop for IrqSpinLockGuard<'_, T> {
    fn drop(&mut self) {
        drop(self.guard.take());

        if self.irq_enabled {
            arch::irqset(true);
        }
    }
}
