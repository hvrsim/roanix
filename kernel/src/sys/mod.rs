//!
//! # Core kernel modules.
//!
//! Kernel components required by the rest of the kernel.
//! Modules present here should not depend on anything
//! else besides [`arch`](`crate::arch`)!
//!

pub mod clock;
pub mod debug;
pub mod event;
pub mod firmware;
pub mod initramfs;
pub mod panic;
pub mod random;
pub mod sched;
pub mod smp;
pub mod sync;
pub mod thread;
