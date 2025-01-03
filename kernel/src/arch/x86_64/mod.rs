//!
//! # x86_64 Subsystem
//!
//! Implements a support interface for the kernel to interact with x86_64 CPU features/hardware. Also
//! provides drivers for on-chip devices, such as the TSC and APIC irqchip.
//!

use core::arch::asm;

mod log;

/// Perform early CPU initialization.
///
/// Registers debug console impl with the `log` crate. Also enumerates and enables CPU features.
pub fn early() {
    log::setup();
}

/// Pause CPU execution and wait for interrupts.
///
/// **WARN: If interrupts are disabled, this will result in an infinite loop.**
pub fn hcf() -> ! {
    loop {
        unsafe {
            asm!("hlt");
        }
    }
}
