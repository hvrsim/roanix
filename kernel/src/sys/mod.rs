//!
//! # Core kernel modules.
//!
//! Kernel components required by the rest of the kernel.
//! Modules present here should not depend on anything
//! else besides [`arch`](`crate::arch`)!
//!

pub mod clock;
pub mod debug;
pub mod fbcon;
pub mod fireworks;
pub mod framebuffer;
pub mod panic;
pub mod sched;
pub mod smp;
pub mod sync;
pub mod thread;
