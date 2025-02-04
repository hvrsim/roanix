//!
//! # x86_64 Subsystem
//!
//! Implements a support interface for the kernel to interact with the x86_64
//! platform. Drivers for on-chip devices, such as the TSC and APIC are 
//! provided by this module aswell.
//!
//! Code in this module often references the *Intel SDM* for register references 
//! and ISA semantics.
//!
//! *You may download the SDM [here.](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)*
//!

use x86_64::instructions::{hlt, port::PortWriteOnly};

mod cpu;

/// Performs early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {
    cpu::enable_features();
}

///
/// Writes a single character to the emulator debug port.
///
/// Utilizes the bochs/QEMU debug port (port 0xE9) which is unallocated on
/// real hardware. This means writes to the port are ignored if we aren't
/// running on bochs or QEMU.
///
/// *NOTE: to make output from this console visible, pass `-debugcon stdio` to
/// QEMU flags, like so:*
///
/// ```bash
/// $ QEMUFLAGS="... -debugcon stdio" make run-bios
/// ```
///
#[inline]
pub fn debug_putc(byte: u8) {
    let mut port = PortWriteOnly::new(0xE9);
    unsafe { port.write(byte) }
}

/// Pauses CPU execution and waits for interrupts.
///
/// **If interrupts are disabled, this will result in an infinite loop.**
pub fn hcf() -> ! {
    loop {
        hlt();
    }
}
