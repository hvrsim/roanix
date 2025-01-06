//!
//! # x86_64 Subsystem
//!
//! Implements a support interface for the kernel to interact with x86_64 CPU features/hardware. Also
//! provides drivers for on-chip devices, such as the TSC and APIC irqchip.
//!

use x86_64::instructions::{hlt, port::PortWriteOnly};

/// Perform early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {}

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

/// Pause CPU execution and wait for interrupts.
///
/// **WARN: If interrupts are disabled, this will result in an infinite loop.**
pub fn hcf() -> ! {
    loop {
        hlt();
    }
}
