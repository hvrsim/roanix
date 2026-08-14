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

use crate::sys::{event::Event, smp::IrqSpinLock};

/// Connector between [`log`] crate and the kernel logging backend.
struct KLog;

/// Number of entries stored in the global log ring.
pub const RING_CAPACITY: usize = 512;

const RECORD_CAPACITY: usize = 256;

/// Maximum number of registered sinks.
pub const MAX_SINKS: usize = 8;

/// Callback invoked for every formatted log message.
pub type LogSink = fn(&[u8]);

/// Contains data to reconstruct a single kernel log message.
#[derive(Clone, Copy)]
#[allow(dead_code)]
struct Record {
    /// Buffer to store the formatted log message.
    buf: [u8; RECORD_CAPACITY],

    /// Length of formatted log message in bytes.
    buflen: usize,

    /// Log level represented as an integer.
    level: usize,

    /// CPU responsible for the message.
    cpu: usize,

    /// Absolute byte offset of this record in the kmsg stream.
    stream_offset: u64,
}

/// Fixed-size ring buffer containing the latest kernel logs.
struct LogRing {
    records: [Record; RING_CAPACITY],
    read: usize,
    write: usize,
    len: usize,
    next_offset: u64,
}

/// Shared log ring state.
struct DebugState {
    ring: LogRing,
}

/// Registered output sinks.
///
/// Sinks are slow, polled devices (a serial UART, or one SBI call per byte on
/// RISC-V). They get their own lock so a multi-millisecond console write never
/// blocks readers of the log ring or other CPUs appending records.
struct ConsoleState {
    sinks: [Option<LogSink>; MAX_SINKS],
}

/// Global logger instance, [`log`] crate invokes this.
static LOGGER: KLog = KLog;

/// Global log ring protected by an IRQ-safe spinlock.
static DEBUG_STATE: IrqSpinLock<DebugState> = IrqSpinLock::new(DebugState::new());
/// Console sink registry, held only while writing to output devices.
static CONSOLE: IrqSpinLock<ConsoleState> = IrqSpinLock::new(ConsoleState::new());
static PANIC_MODE: AtomicBool = AtomicBool::new(false);
static REGULAR_SINK_OUTPUT: AtomicBool = AtomicBool::new(true);
static LOG_EVENT: Event = Event::new();

/// Failure returned while reading the kernel log stream.
pub(crate) enum LogReadError {
    /// The requested data has already been overwritten.
    Overrun,
    /// A nonblocking read found no new data.
    WouldBlock,
}

impl Record {
    /// Creates an empty log record.
    const fn empty() -> Self {
        Self {
            buf: [0; RECORD_CAPACITY],
            buflen: 0,
            level: 0,
            cpu: 0,
            stream_offset: 0,
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
        let line = record.line().unwrap_or(0);

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

    /// Creates a record from bytes written through `/dev/kmsg`.
    fn from_bytes(bytes: &[u8]) -> Self {
        let mut rec = Self {
            level: 3,
            cpu: crate::arch::thiscpu_opt().map_or(0, |cpu| cpu.id),
            ..Self::empty()
        };
        let count = bytes.len().min(rec.buf.len());
        rec.buf[..count].copy_from_slice(&bytes[..count]);
        rec.buflen = count;
        rec
    }

    fn stream_len(&self) -> u64 {
        self.buflen as u64 + 1
    }

    /// Returns the formatted message bytes for this record.
    fn message(&self) -> &[u8] {
        &self.buf[..self.buflen]
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
            next_offset: 0,
        }
    }

    /// Pushes a record into the ring, overwriting oldest entries when full.
    fn push(&mut self, rec: &Record) {
        let slot = &mut self.records[self.write];
        *slot = *rec;
        slot.stream_offset = self.next_offset;
        self.next_offset = self
            .next_offset
            .checked_add(slot.stream_len())
            .expect("debug: kernel log offset overflow");

        if self.len == RING_CAPACITY {
            self.read = (self.read + 1) % RING_CAPACITY;
        } else {
            self.len += 1;
        }

        self.write = (self.write + 1) % RING_CAPACITY;
    }

    fn oldest_offset(&self) -> u64 {
        if self.len == 0 {
            self.next_offset
        } else {
            self.records[self.read].stream_offset
        }
    }

    fn read_into(&self, offset: u64, output: &mut [u8]) -> usize {
        let mut cursor = offset;
        let mut written = 0;
        let mut index = self.read;

        for _ in 0..self.len {
            let rec = &self.records[index];
            let body_end = rec.stream_offset + rec.buflen as u64;
            let record_end = body_end + 1;
            index = (index + 1) % RING_CAPACITY;

            if cursor >= record_end {
                continue;
            }

            if cursor < body_end {
                let source = (cursor - rec.stream_offset) as usize;
                let count = (rec.buflen - source).min(output.len() - written);
                output[written..written + count]
                    .copy_from_slice(&rec.buf[source..source + count]);
                cursor += count as u64;
                written += count;
            }

            if written == output.len() {
                break;
            }
            if cursor == body_end {
                output[written] = b'\n';
                written += 1;
                cursor += 1;
            }
            if written == output.len() {
                break;
            }
        }

        written
    }
}

impl DebugState {
    /// Creates empty ring state.
    const fn new() -> Self {
        Self {
            ring: LogRing::new(),
        }
    }
}

impl ConsoleState {
    /// Creates an empty sink registry.
    const fn new() -> Self {
        Self {
            sinks: [const { None }; MAX_SINKS],
        }
    }

    fn dispatch(&self, message: &[u8]) {
        for sink in self.sinks.iter().flatten() {
            sink(message);
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
        DEBUG_STATE.lock().ring.push(&rec);
        // Console output happens outside the ring lock so slow serial writes
        // cannot stall readers or other CPUs appending records.
        write_to_sinks(rec.message());
        LOG_EVENT.signal();
    }

    fn flush(&self) {}
}

/// Registers a log sink callback.
///
/// Fails silently if a sink is unable to be registered.
pub fn register_sink(sink: LogSink) {
    let mut console = CONSOLE.lock();

    let Some(slot) = console.sinks.iter_mut().find(|slot| slot.is_none()) else {
        return;
    };
    *slot = Some(sink);

    if !REGULAR_SINK_OUTPUT.load(Ordering::Acquire) {
        return;
    }

    // Replay the backlog under the ring lock so the new sink observes a
    // consistent snapshot. Sinks are registered once during boot, so briefly
    // holding both locks here does not affect steady-state logging.
    let state = DEBUG_STATE.lock();
    let mut index = state.ring.read;
    for _ in 0..state.ring.len {
        sink(state.ring.records[index].message());
        index = (index + 1) % RING_CAPACITY;
    }
}

/// Unregisters the first matching sink callback.
pub fn unregister_sink(sink: LogSink) -> bool {
    let mut console = CONSOLE.lock();

    for slot in &mut console.sinks {
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
    CONSOLE.lock().sinks.fill(None);
}

/// Stops ordinary log records from being mirrored to registered debug sinks.
///
/// Panic-time output still writes directly to the registered sinks.
pub(crate) fn disable_regular_sink_output() {
    let _console = CONSOLE.lock();
    REGULAR_SINK_OUTPUT.store(false, Ordering::Release);
}

/// Restores ordinary log mirroring to registered debug sinks.
pub(crate) fn enable_regular_sink_output() {
    let _console = CONSOLE.lock();
    REGULAR_SINK_OUTPUT.store(true, Ordering::Release);
}

/// Returns the first readable byte offset in the kernel log stream.
pub(crate) fn log_start_offset() -> u64 {
    DEBUG_STATE.lock().ring.oldest_offset()
}

/// Returns the byte offset immediately following the current kernel log.
pub(crate) fn log_end_offset() -> u64 {
    DEBUG_STATE.lock().ring.next_offset
}

/// Reads bytes from the kernel log, waiting for new records when requested.
pub(crate) fn read_log(
    offset: u64,
    output: &mut [u8],
    nonblocking: bool,
) -> Result<usize, LogReadError> {
    if output.is_empty() {
        return Ok(0);
    }

    loop {
        let state = DEBUG_STATE.lock();
        if offset < state.ring.oldest_offset() {
            return Err(LogReadError::Overrun);
        }
        if offset < state.ring.next_offset {
            return Ok(state.ring.read_into(offset, output));
        }
        if nonblocking {
            return Err(LogReadError::WouldBlock);
        }

        LOG_EVENT.reset();
        drop(state);
        LOG_EVENT.wait();
    }
}

/// Appends bytes written through `/dev/kmsg` as kernel log records.
pub(crate) fn append_kernel_message(bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }

    let mut remaining = bytes;
    while !remaining.is_empty() {
        let newline = remaining.iter().position(|byte| *byte == b'\n');
        let line_len = newline.unwrap_or(remaining.len());
        let line = &remaining[..line_len];

        if line.is_empty() {
            append_message(&[]);
        } else {
            for chunk in line.chunks(RECORD_CAPACITY) {
                append_message(chunk);
            }
        }

        remaining = match newline {
            Some(index) => &remaining[index + 1..],
            None => &[],
        };
    }
    LOG_EVENT.signal();
}

fn append_message(bytes: &[u8]) {
    let rec = Record::from_bytes(bytes);
    DEBUG_STATE.lock().ring.push(&rec);
    write_to_sinks(rec.message());
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
        // SAFETY: upheld by this function's panic-only caller contract.
        unsafe { DEBUG_STATE.force_unlock() };
    }
    if CONSOLE.is_locked() {
        // SAFETY: upheld by this function's panic-only caller contract.
        unsafe { CONSOLE.force_unlock() };
    }
}

/// Writes a preformatted message directly to all registered sinks without
/// appending a new record to the ring buffer.
pub(crate) fn write_to_sinks(message: &[u8]) {
    if !REGULAR_SINK_OUTPUT.load(Ordering::Acquire) && !PANIC_MODE.load(Ordering::Acquire) {
        return;
    }

    CONSOLE.lock().dispatch(message);
}

/// Connects kernel logging infra to the log crate.
///
/// This function will panic if the kernel logger is unable to be installed,
/// since the log functions depend on a valid kernel logger.
pub fn register() {
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
}
