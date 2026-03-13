//!
//! # Kernel Debugging Interface
//!
//! Logging backend used by the [`log`] crate.
//!
//! Every log line is captured as a fixed-size [`Record`] and appended to a
//! global ring buffer.
//!

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, Ordering};

use log::Level;

use crate::sys::smp::IrqSpinLock;

/// Connector between [`log`] crate and the kernel logging backend.
struct KLog;

/// Number of entries stored in the global log ring.
pub const RING_CAPACITY: usize = 512;

/// Maximum number of registered sinks.
pub const MAX_SINKS: usize = 8;

/// Callback invoked for every formatted log message.
pub type LogSink = fn(*const u8, usize);

/// Contains data to reconstruct a single kernel log message.
#[derive(Clone, Copy)]
#[allow(dead_code)]
struct Record {
    /// Buffer to store the formatted log message.
    buf: [u8; 256],

    /// Length of formatted log message in bytes.
    buflen: usize,

    /// Log level represented as an integer.
    level: usize,

    /// CPU responsible for the message.
    cpu: usize,
}

/// Fixed-size ring buffer containing the latest kernel logs.
struct LogRing {
    records: [Record; RING_CAPACITY],
    read: usize,
    write: usize,
    len: usize,
}

/// Shared debug subsystem state.
struct DebugState {
    ring: LogRing,
    sinks: [Option<LogSink>; MAX_SINKS],
}

/// Global logger instance, [`log`] crate invokes this.
static LOGGER: KLog = KLog;

/// Global debug state protected by an IRQ-safe spinlock.
static DEBUG_STATE: IrqSpinLock<DebugState> = IrqSpinLock::new(DebugState::new());
static PANIC_MODE: AtomicBool = AtomicBool::new(false);

impl Record {
    /// Creates an empty log record.
    const fn empty() -> Self {
        Self {
            buf: [0; 256],
            buflen: 0,
            level: 0,
            cpu: 0,
        }
    }

    /// Creates a log record from [`log`] metadata and message payload.
    fn from_log_record(record: &log::Record) -> Self {
        let (prefix, level) = match record.level() {
            Level::Error => ("[\x1b[1;31mE\x1b[0m]", 1),
            Level::Warn => ("[\x1b[1;33m!\x1b[0m]", 2),
            Level::Info => ("[\x1b[1;32m*\x1b[0m]", 3),
            Level::Debug => ("[\x1b[1;34mD\x1b[0m]", 4),
            Level::Trace => ("[\x1b[0;35mT\x1b[0m]", 5),
        };

        let mut rec = Self {
            level,
            cpu: crate::arch::thiscpu_opt().map_or(0, |cpu| cpu.id),
            ..Self::empty()
        };

        let path = record.file().map_or("???", |f| f);
        let line = record.line().map_or(0, |l| l);

        write!(
            &mut rec,
            "{} \x1b[2m({}:{})\x1b[0m {}",
            prefix,
            path,
            line,
            record.args()
        )
        .ok();

        rec
    }
}

impl Write for Record {
    /// Copy `s` into the record buffer, truncating on overflow.
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let space = self.buf.len().saturating_sub(self.buflen);
        let count = bytes.len().min(space);

        self.buf[self.buflen..self.buflen + count].copy_from_slice(&bytes[..count]);
        self.buflen += count;

        Ok(())
    }
}

impl LogRing {
    /// Creates an empty log ring.
    const fn new() -> Self {
        Self {
            records: [Record::empty(); RING_CAPACITY],
            read: 0,
            write: 0,
            len: 0,
        }
    }

    /// Pushes a record into the ring, overwriting oldest entries when full.
    fn push(&mut self, rec: Record) {
        self.records[self.write] = rec;

        if self.len == RING_CAPACITY {
            self.read = (self.read + 1) % RING_CAPACITY;
        } else {
            self.len += 1;
        }

        self.write = (self.write + 1) % RING_CAPACITY;
    }
}

impl DebugState {
    /// Creates empty debug state.
    const fn new() -> Self {
        Self {
            ring: LogRing::new(),
            sinks: [const { None }; MAX_SINKS],
        }
    }
}

impl log::Log for KLog {
    /// Unused, since we accept all messages regardless of level.
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    /// Captures a log record into the global ring and dispatches sinks.
    fn log(&self, record: &log::Record) {
        if PANIC_MODE.load(Ordering::Acquire) {
            return;
        }

        let rec = Record::from_log_record(record);
        let mut state = DEBUG_STATE.lock();
        state.ring.push(rec);

        dispatch_sinks_locked(rec.buf.as_ptr(), rec.buflen, &state.sinks);
    }

    fn flush(&self) {}
}

/// Dispatches a preformatted message buffer to all currently registered sinks.
///
/// Expects the caller to hold the debug state lock. We keep the ring update and
/// sink writes under the same lock so concurrent CPUs serialize console output
/// instead of dropping records while a slow sink is draining bytes.
#[inline]
fn dispatch_sinks_locked(buf: *const u8, buflen: usize, sinks: &[Option<LogSink>; MAX_SINKS]) {
    if sinks.iter().all(|slot| slot.is_none()) {
        return;
    }

    for sink in sinks.iter().flatten() {
        sink(buf, buflen);
    }
}

/// Registers a log sink callback.
///
/// Fails silently if a sink is unable to be registered.
pub fn register_sink(sink: LogSink) {
    let mut state = DEBUG_STATE.lock();

    for slot in &mut state.sinks {
        if slot.is_none() {
            *slot = Some(sink);
            let mut idx = state.ring.read;
            for _ in 0..state.ring.len {
                let rec = state.ring.records[idx];
                sink(rec.buf.as_ptr(), rec.buflen);
                idx = (idx + 1) % RING_CAPACITY;
            }

            return;
        }
    }
}

/// Unregisters the first matching sink callback.
pub fn unregister_sink(sink: LogSink) -> bool {
    let mut state = DEBUG_STATE.lock();

    for slot in &mut state.sinks {
        if slot
            .as_ref()
            .map(|registered| core::ptr::fn_addr_eq(*registered, sink))
            .unwrap_or(false)
        {
            *slot = None;
            return true;
        }
    }

    false
}

/// Removes all currently registered sinks.
pub fn clear_sinks() {
    let mut state = DEBUG_STATE.lock();
    for slot in &mut state.sinks {
        *slot = None;
    }
}

/// Prevents regular logs from competing with panic output.
pub(crate) fn enter_panic_mode() {
    PANIC_MODE.store(true, Ordering::Release);
}

/// Force-unlocks the debug sink registry for panic-time recovery.
///
/// # Safety
///
/// This must only be used after all other CPUs have been stopped or when the
/// caller accepts that the previous lock owner will never resume.
pub(crate) unsafe fn force_unlock_for_panic() {
    if DEBUG_STATE.is_locked() {
        DEBUG_STATE.force_unlock();
    }
}

/// Writes a preformatted string buffer directly to all registered sinks
/// without appending a new record to the ring buffer.
pub(crate) fn write_to_sinks(buf: *const u8, buflen: usize) {
    let state = DEBUG_STATE.lock();
    dispatch_sinks_locked(buf, buflen, &state.sinks);
}

/// Connects kernel logging infra to the log crate.
///
/// This function will panic if the kernel logger is unable to be installed,
/// since the log functions depend on a valid kernel logger.
pub fn register() {
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Info))
        .unwrap();
}
