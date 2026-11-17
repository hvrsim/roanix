//!
//! # Core kernel modules.
//!
//! Kernel components required by the rest of the kernel: scheduling, threads,
//! timekeeping, synchronization, SMP bring-up, logging, and panic handling.
//!

pub mod clock;
pub(crate) mod cmdline;
pub mod context;
pub mod event;
pub mod executor;
pub mod firmware;
pub mod initramfs;
pub mod klog;
pub mod lockdep;
pub mod panic;
mod parking;
mod power;
pub mod random;
pub mod sched;
pub mod service;
pub mod smp;
pub mod sync;
mod syscall;
pub mod thread;
