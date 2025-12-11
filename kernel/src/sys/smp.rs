//!
//! # Multicore Support
//!
//! This module is responsible for managing multiple CPU cores. Main duties
//! include AP bringup and core local data definitions.
//!

#[cfg(target_arch = "x86_64")]
use crate::arch::cpu::CpuFeatures;

/// Fields available only on x86_64.
#[cfg(target_arch = "x86_64")]
pub struct PlatformFields {
    pub feats: CpuFeatures,
}

/// Empty struct for fields without a platform-specific context.
#[cfg(not(target_arch = "x86_64"))]
pub struct PlatformFields {}

/// Kernel context unique to each CPU core.
pub struct CoreLocal {
    /// Stack used by the kernel on IRQs.
    pub kernel_stack: u64,

    /// Placeholder for user stack on IRQs.
    pub user_stack: u64,

    /// ID of current CPU core.
    pub id: usize,

    /// Platform specific context.
    #[allow(dead_code)]
    pub platform: PlatformFields,
}

impl PlatformFields {
    #[cfg(target_arch = "x86_64")]
    pub const fn new() -> Self {
        return PlatformFields {
            feats: CpuFeatures::empty(),
        };
    }

    #[cfg(not(target_arch = "x86_64"))]
    pub const fn new() -> Self {
        return PlatformFields {};
    }
}

impl CoreLocal {
    /// Creates a new core local context, with CPU ID `cid`.
    pub const fn new(cid: usize) -> Self {
        Self {
            id: cid,
            kernel_stack: 0,
            user_stack: 0,
            platform: PlatformFields::new(),
        }
    }
}
