//!
//! Serial port sink for kernel logs.
//!
//! Utilizes the bochs/QEMU debug port (port 0xE9) which is unallocated on 
//! real hardware. This means writes to the port are ignored if we aren't
//! running on bochs or QEMU.
//!
//! *NOTE: to make output from this console visible, pass `-debugcon stdio` to
//! QEMU flags, like so:*
//!
//! ```bash
//! $ QEMUFLAGS="... -debugcon stdio" make run-bios
//! ```
//!

use core::arch::asm;
use core::fmt::{Result, Write};
use log::{LevelFilter, Metadata, Record};

/// Logger implementation for port 0xE9.
struct E9Logger;

/// Global instance of logger, shared between CPU cores.
static LOGGER: E9Logger = E9Logger;

impl Write for E9Logger {
    fn write_str(&mut self, s: &str) -> Result {
        for byte in s.bytes() {
            unsafe {
                asm!("out dx, al", in("dx") 0xE9, in("al") byte, options(nomem, nostack, preserves_flags));
            }
        }

        Ok(())
    }
}

impl log::Log for E9Logger {
    /// We accept all messages, regardless of level.
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        // A write to the debug port can never fail.
        write!(&mut E9Logger, "{}", record.args()).unwrap();
    }

    /// Flush calls are done on a per-character basis, therefore global flush not required.
    fn flush(&self) {}
}

/// Registers debug logger.
pub fn setup() {
    // Kernel logging depends on a valid logger, therefore panic if we are unable to install this logger.
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Info))
        .unwrap();
}
