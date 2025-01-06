//!
//! # Kernel Logging Interface.
//!
//! Responsible for gathering log output from `info!`, `warn!`, `trace!` and the likes.
//!
//! Stores each line of logging output into an internal buffer, then dispatches
//! each character out to the arch-specific debug console (`arch::debug_putc`).
//!

// static mut buffer protected with mutex.
#![allow(static_mut_refs)]

use core::fmt::{Result, Write};
use log::{Level, LevelFilter, Metadata, Record};
use spin::Mutex;

/// Connector between `log` crate and various outputs.
struct KLog;

/// Internal buffer of raw log output, circles back once filled.
struct RingBuffer<const N: usize> {
    data: [u8; N],
    read: usize,
    write: usize,
}

/// Number of characters in the ringbuffer.
const RING_ENTRIES: usize = 4096;

/// Global logger instance, `log` crate invokes this.
static LOGGER: KLog = KLog;

/// Global buffer instance, protected with mutex for SMP contexts.
static mut BUFFER: Mutex<RingBuffer<RING_ENTRIES>> = Mutex::new(RingBuffer {
    data: [0; RING_ENTRIES],
    read: 0,
    write: 0,
});

impl<const T: usize> Write for RingBuffer<T> {
    /// Copies the string into the ringbuffer, and calls `debug_putc` for each byte.
    fn write_str(&mut self, s: &str) -> Result {
        for byte in s.bytes() {
            self.data[self.write] = byte;

            let next = (self.write + 1) % RING_ENTRIES;

            // If we have filled up the write allocation, bump the read pointer.
            if next == self.read {
                self.read = (self.read + 1) % RING_ENTRIES;
            }

            crate::arch::debug_putc(byte);

            self.write = next;
        }

        Ok(())
    }
}

impl log::Log for KLog {
    /// Unused, since we accept all messages regardless of level.
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    /// Pretty-prints log record and sends it through the backend.
    fn log(&self, record: &Record) {
        unsafe {
            let mut buffer = BUFFER.lock();

            match record.level() {
                Level::Error => write!(&mut buffer, "[\x1b[1;31mE\x1b[0m]").unwrap(),
                Level::Warn => write!(&mut buffer, "[\x1b[1;33m!\x1b[0m]").unwrap(),
                Level::Info => write!(&mut buffer, "[\x1b[1;32m*\x1b[0m]").unwrap(),
                Level::Debug => write!(&mut buffer, "[\x1b[1;34mD\x1b[0m]").unwrap(),
                Level::Trace => write!(&mut buffer, "[\x1b[1;35mT\x1b[0m]").unwrap(),
            }

            let path = if let Some(path) = record.file() {
                path
            } else {
                "???"
            };

            let line = if let Some(line) = record.line() {
                line
            } else {
                0
            };

            // A write to the debug port can never fail.
            write!(&mut buffer, " ({}:{}) {}\n", path, line, record.args()).unwrap();
        }
    }

    /// Flush calls are done on a per-character basis, therefore global flush not required.
    fn flush(&self) {}
}

/// Connects kernel logging infra to the log crate.
pub fn register() {
    // Kernel logging depends on a valid logger, therefore panic if we are unable to install this logger.
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Info))
        .unwrap();
}
