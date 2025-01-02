use core::arch::asm;
use core::fmt::{Result, Write};
use log::{LevelFilter, Metadata, Record};

struct E9Logger;

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
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        // A write to the debug port can never fail.
        write!(&mut E9Logger, "{}", record.args()).unwrap();
    }

    fn flush(&self) {}
}

pub fn setup() {
    // Kernel logging depends on a valid logger, therefore panic if we are unable to install this logger.
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Info))
        .unwrap();
}
