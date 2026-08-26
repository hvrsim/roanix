//! Kernel-provided device classes.
//!
//! These are the classes the kernel itself needs in order to expose devices to
//! userspace. Everything else - block, network, input, display - is expected to
//! arrive as a driver-registered class, which the framework treats identically.

pub mod chardev;
pub mod console;

/// Initializes the kernel-provided classes.
pub(super) fn init() {
    console::init();
}
