//!
//! # Core kernel modules.
//!
//! Kernel components required by the rest of the kernel: scheduling, threads,
//! timekeeping, synchronization, SMP bring-up, logging, and panic handling.
//!

pub mod clock;
pub mod event;
pub mod firmware;
pub mod initramfs;
pub mod klog;
pub mod panic;
pub mod random;
pub mod sched;
pub mod smp;
pub mod sync;
mod syscall;
pub mod thread;
