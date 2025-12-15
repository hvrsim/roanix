//!
//! # Multicore Support
//!
//! This module is responsible for managing multiple CPU cores. Main duties
//! include AP bringup and core local data definitions.
//!

#[cfg(target_arch = "x86_64")]
use crate::arch::cpu::CpuFeatures;

/// Platform specific core-local fields.
pub struct PlatformFields {
    /// Bitmap of supported x86 extensions.
    #[cfg(target_arch = "x86_64")]
    pub feats: CpuFeatures,
}

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
    /// Creates a new `PlatformFields` instance.
    pub const fn new() -> Self {
        PlatformFields {
            #[cfg(target_arch = "x86_64")]
            feats: CpuFeatures::empty(),
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
            platform: PlatformFields::new(),
        }
    }
}
