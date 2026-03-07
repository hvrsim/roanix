//!
//! # Core kernel modules.
//!
//! Kernel components required by the rest of the kernel.
//! Modules present here should not depend on anything
//! else besides [`arch`](`crate::arch`)!
//!

pub mod debug;
pub mod panic;
pub mod smp;
