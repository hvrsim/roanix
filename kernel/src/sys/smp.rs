//!
//! # Multicore Support
//!
//! This module is responsible for managing multiple CPU cores. Main duties
//! include AP bringup and core local data definitions.
//!

use crate::arch::cpu::CpuFeatures;

/// Kernel context unique to each CPU core.
pub struct CoreLocal {
    /// ID of current CPU core.
    pub id: usize,

    /// Supported features for this core.
    pub supported_feats: CpuFeatures,
}

impl CoreLocal {
    /// Creates a new core local context, with CPU ID `cid`.
    pub const fn new(cid: usize) -> Self {
        Self {
            id: cid,
            supported_feats: CpuFeatures::empty(),
        }
    }
}
