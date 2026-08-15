//!
//! # Kernel Logging
//!
//! Packet-oriented kernel log with runtime severity control.
//!
//! A message travels through three stages. The [`log`] facade rejects it
//! outright when its severity is below the current record level, so a filtered
//! call costs one relaxed atomic load and never formats its arguments. A
//! message that survives is rendered into a stack buffer and appended to the
//! wait-free [`packet::Ring`], which is the authoritative history served to
//! `/dev/klog`. Only then, and only when the message also clears the separate
//! console level, is a human-readable line pushed to the registered sinks.
//!
//! Splitting the record level from the console level is what keeps the log
//! both complete and quiet: `dmesg` can show everything the kernel recorded
//! while the serial console stays free for the user's shell.
//!
//! The panic path deliberately bypasses all of this. [`emergency`] formats on
//! the stack and writes straight to the sinks so a crash still reports itself
//! after locks have been force-released and other CPUs stopped.
//!

pub mod dev;
pub mod packet;

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};

use crate::sys::{clock, event::Event, smp::IrqSpinLock};

use packet::{Fetch, PacketBuffer, RecordData, Ring};

/// Severity of a kernel log record.
///
/// Values are ordered by urgency: a smaller number is more severe. Filtering
/// keeps every record whose level is numerically less than or equal to the
/// active threshold.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Level {
    /// The kernel cannot continue. Reserved for the panic path and never
    /// suppressed by either filter.
    Emergency = 0,
    /// An operation failed in a way the kernel could not paper over.
    Error = 1,
    /// Something unexpected happened but the kernel recovered.
    Warn = 2,
    /// A one-off milestone worth reporting on every boot.
    Info = 3,
    /// Detail useful when investigating a specific subsystem.
    Debug = 4,
    /// High-frequency detail that would drown a console.
    Trace = 5,
}

/// Number of distinct severities.
pub const LEVEL_COUNT: u8 = 6;

impl Level {
    /// Converts a raw severity, saturating at [`Level::Trace`].
    pub const fn from_raw(value: u8) -> Self {
        match value {
            0 => Self::Emergency,
            1 => Self::Error,
            2 => Self::Warn,
            3 => Self::Info,
            4 => Self::Debug,
            _ => Self::Trace,
        }
    }

    /// Returns the numeric severity.
    pub const fn as_raw(self) -> u8 {
        self as u8
    }

    /// Returns the lowercase name used by diagnostics.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Emergency => "emerg",
            Self::Error => "error",
            Self::Warn => "warn",
            Self::Info => "info",
            Self::Debug => "debug",
            Self::Trace => "trace",
        }
    }

    /// Returns the single-character console glyph and its SGR attributes.
    const fn glyph(self) -> (char, &'static str) {
        match self {
            Self::Emergency => ('!', "1;41;97"),
            Self::Error => ('E', "1;31"),
            Self::Warn => ('!', "1;33"),
            Self::Info => ('*', "1;32"),
            Self::Debug => ('D', "1;34"),
            Self::Trace => ('T', "0;35"),
        }
    }

    fn from_log(level: log::Level) -> Self {
        match level {
            log::Level::Error => Self::Error,
            log::Level::Warn => Self::Warn,
            log::Level::Info => Self::Info,
            log::Level::Debug => Self::Debug,
            log::Level::Trace => Self::Trace,
        }
    }

    fn to_filter(self) -> log::LevelFilter {
        match self {
            // The facade has no emergency level. Recording only errors is the
            // closest it can express, and the panic path does not use it.
            Self::Emergency | Self::Error => log::LevelFilter::Error,
            Self::Warn => log::LevelFilter::Warn,
            Self::Info => log::LevelFilter::Info,
            Self::Debug => log::LevelFilter::Debug,
            Self::Trace => log::LevelFilter::Trace,
        }
    }
}

/// Callback invoked with one preformatted console line.
pub type Sink = fn(&[u8]);

/// Maximum number of registered console sinks.
pub const MAX_SINKS: usize = 8;

/// Severity recorded into the ring until a boot argument or ioctl says
/// otherwise.
const DEFAULT_RECORD_LEVEL: Level = Level::Debug;

/// Severity mirrored to consoles during boot, before userspace takes over.
const DEFAULT_CONSOLE_LEVEL: Level = Level::Info;

/// Widest console line produced from a single record.
const CONSOLE_LINE_CAPACITY: usize = 512;

/// Widest plain-text line produced from a single record.
pub(crate) const TEXT_LINE_CAPACITY: usize = 448;

/// Snapshot of kernel log statistics.
#[derive(Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct Stats {
    /// Sequence that will be given to the next record.
    pub next_sequence: u64,
    /// Oldest sequence still resident in the ring.
    pub first_sequence: u64,
    /// Records evicted before any reader consumed them.
    pub overwritten: u64,
    /// Records whose message text did not fit in a slot.
    pub truncated: u64,
    /// Number of slots in the ring.
    pub slots: u32,
    /// Bytes of payload each slot can hold.
    pub slot_payload: u32,
    /// Severity currently recorded into the ring.
    pub record_level: u8,
    /// Severity currently mirrored to consoles.
    pub console_level: u8,
    /// Padding reserved for future fields.
    pub reserved: [u8; 6],
}

/// Failure returned while reading kernel log packets.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ReadError {
    /// A nonblocking read found no published records.
    WouldBlock,
    /// The output buffer cannot hold a single packet.
    BufferTooSmall,
    /// A signal became pending while waiting for a record.
    Interrupted,
}

struct Bridge;

static BRIDGE: Bridge = Bridge;
static RING: Ring = Ring::new();
static SINKS: IrqSpinLock<[Option<Sink>; MAX_SINKS]> = IrqSpinLock::new([None; MAX_SINKS]);
static RECORD_LEVEL: AtomicU8 = AtomicU8::new(DEFAULT_RECORD_LEVEL as u8);
static CONSOLE_LEVEL: AtomicU8 = AtomicU8::new(DEFAULT_CONSOLE_LEVEL as u8);
static VISIBLE_FLOOR: AtomicU64 = AtomicU64::new(0);
static PANIC_MODE: AtomicBool = AtomicBool::new(false);
static ARRIVAL: Event = Event::new();

/// Fixed-capacity formatter used for both console lines and message bodies.
struct TextBuffer<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> TextBuffer<N> {
    const fn new() -> Self {
        Self {
            bytes: [0; N],
            length: 0,
        }
    }

    fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.length]
    }
}

impl<const N: usize> Write for TextBuffer<N> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let bytes = text.as_bytes();
        let count = bytes.len().min(self.bytes.len() - self.length);
        self.bytes[self.length..self.length + count].copy_from_slice(&bytes[..count]);
        self.length += count;
        Ok(())
    }
}

impl log::Log for Bridge {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        Level::from_log(metadata.level()) <= record_level()
    }

    fn log(&self, record: &log::Record<'_>) {
        let level = Level::from_log(record.level());
        if level > record_level() {
            return;
        }

        let mut message = TextBuffer::<{ packet::PAYLOAD_CAPACITY }>::new();
        let _ = write!(&mut message, "{}", record.args());

        let mut tag = [0u8; packet::SUBSYSTEM_LIMIT];
        emit(
            level,
            subsystem_of(record.target(), &mut tag),
            trim_source_path(record.file().unwrap_or_default()),
            record.line().unwrap_or(0),
            message.as_slice(),
            0,
        );
    }

    fn flush(&self) {}
}

/// Installs the kernel logger.
///
/// Must run before any other subsystem logs, and panics if the facade already
/// has a logger because losing records silently would hide boot failures.
pub fn init() {
    log::set_logger(&BRIDGE).expect("klog: a logger is already installed");
    log::set_max_level(record_level().to_filter());
}

/// Returns the severity currently recorded into the ring.
#[inline]
pub fn record_level() -> Level {
    Level::from_raw(RECORD_LEVEL.load(Ordering::Relaxed))
}

/// Returns the severity currently mirrored to console sinks.
#[inline]
pub fn console_level() -> Level {
    Level::from_raw(CONSOLE_LEVEL.load(Ordering::Relaxed))
}

/// Changes the severity recorded into the ring and returns the previous value.
///
/// The console filter is clamped to the new value: mirroring a record that was
/// never stored is impossible, and letting the two drift apart would silently
/// discard console output.
pub fn set_record_level(level: Level) -> Level {
    let previous = Level::from_raw(RECORD_LEVEL.swap(level.as_raw(), Ordering::Relaxed));
    log::set_max_level(level.to_filter());
    let _ = CONSOLE_LEVEL.try_update(Ordering::Relaxed, Ordering::Relaxed, |console| {
        (console > level.as_raw()).then_some(level.as_raw())
    });
    previous
}

/// Changes the severity mirrored to console sinks and returns the previous
/// value.
///
/// Raising the console filter above the record filter also raises the record
/// filter, so asking the console for more detail always produces it.
pub fn set_console_level(level: Level) -> Level {
    if level > record_level() {
        set_record_level(level);
    }
    Level::from_raw(CONSOLE_LEVEL.swap(level.as_raw(), Ordering::Relaxed))
}

/// Hides every record recorded so far from new and existing readers.
///
/// Producers are untouched: the floor only moves the point at which readers
/// start, so clearing can never race with a record being published.
pub fn clear() {
    VISIBLE_FLOOR.store(RING.next_sequence(), Ordering::Release);
    ARRIVAL.signal();
}

/// Returns a snapshot of ring occupancy and the active filters.
pub fn stats() -> Stats {
    let next_sequence = RING.next_sequence();
    let first_sequence = first_sequence();
    Stats {
        next_sequence,
        first_sequence,
        overwritten: next_sequence.saturating_sub(packet::RING_SLOTS as u64),
        truncated: RING.truncated(),
        slots: packet::RING_SLOTS as u32,
        slot_payload: packet::PAYLOAD_CAPACITY as u32,
        record_level: record_level().as_raw(),
        console_level: console_level().as_raw(),
        reserved: [0; 6],
    }
}

/// Returns the oldest sequence a reader may observe.
#[inline]
pub fn first_sequence() -> u64 {
    RING.first_sequence().max(VISIBLE_FLOOR.load(Ordering::Acquire))
}

/// Returns the sequence that will be given to the next record.
#[inline]
pub fn next_sequence() -> u64 {
    RING.next_sequence()
}

/// Returns the event signalled whenever a record is published.
#[inline]
pub fn arrival_event() -> &'static Event {
    &ARRIVAL
}

/// Registers a console sink and replays the recorded backlog into it.
///
/// Replay is what makes late console attachment useful: a UART that only comes
/// up after memory management still prints everything that happened first.
pub fn register_sink(sink: Sink) {
    {
        let mut sinks = SINKS.lock();
        let Some(slot) = sinks.iter_mut().find(|slot| slot.is_none()) else {
            return;
        };
        *slot = Some(sink);
    }

    let threshold = console_level();
    let mut buffer = PacketBuffer::new();
    let mut sequence = first_sequence();
    let end = RING.next_sequence();
    while sequence < end {
        match RING.fetch(sequence, &mut buffer) {
            Fetch::Ready(_) => {
                if Level::from_raw(buffer.header().level) <= threshold {
                    let mut line = TextBuffer::<CONSOLE_LINE_CAPACITY>::new();
                    render(&buffer, Style::CONSOLE, &mut line);
                    sink(line.as_slice());
                }
                sequence += 1;
            }
            Fetch::Pending => break,
            Fetch::Lost(resume) => sequence = resume.max(sequence + 1),
        }
    }
}

/// Removes the first registered sink matching `sink`.
pub fn unregister_sink(sink: Sink) -> bool {
    let mut sinks = SINKS.lock();
    for slot in sinks.iter_mut() {
        if slot.is_some_and(|registered| core::ptr::fn_addr_eq(registered, sink)) {
            *slot = None;
            return true;
        }
    }
    false
}

/// Records one message and mirrors it to the console when it passes the
/// console filter.
///
/// This is the single entry point every producer funnels through, including
/// the [`log`] facade and `/dev/kmsg` writes.
pub fn emit(
    level: Level,
    subsystem: &str,
    file: &str,
    line: u32,
    message: &[u8],
    flags: u8,
) {
    if PANIC_MODE.load(Ordering::Relaxed) && flags & packet::FLAG_EMERGENCY == 0 {
        return;
    }

    let (cpu, thread) = origin();
    let record = RecordData {
        level: level.as_raw(),
        flags,
        timestamp_ns: clock::monotonic_ns_or_zero(),
        cpu,
        line: line.min(u16::MAX as u32) as u16,
        thread,
        subsystem: subsystem.as_bytes(),
        file: file.as_bytes(),
        message,
    };

    RING.append(&record);

    if level <= console_level() {
        let mut line = TextBuffer::<CONSOLE_LINE_CAPACITY>::new();
        render_fields(&record, Style::CONSOLE, &mut line);
        write_to_sinks(line.as_slice());
    }

    ARRIVAL.signal();
}

/// Copies the next published packet at or after `*cursor` into `buffer`.
///
/// Advances `*cursor` past the record it returned. A cursor that has fallen out
/// of the ring is snapped forward to the oldest surviving record rather than
/// failing, so a slow reader loses history but never its place in the stream.
///
/// A blocking wait is abandoned when a signal becomes pending, so a program
/// following the log can always be interrupted from its terminal.
pub fn next_packet(
    cursor: &mut u64,
    buffer: &mut PacketBuffer,
    nonblocking: bool,
) -> Result<(), ReadError> {
    loop {
        *cursor = (*cursor).max(first_sequence());
        let observed = RING.next_sequence();

        while *cursor < observed {
            match RING.fetch(*cursor, buffer) {
                Fetch::Ready(_) => {
                    *cursor += 1;
                    return Ok(());
                }
                Fetch::Pending => break,
                Fetch::Lost(resume) => *cursor = resume.max(*cursor + 1),
            }
        }

        if nonblocking {
            return Err(ReadError::WouldBlock);
        }

        let process = crate::proc::current();
        if let Some(process) = process.as_ref()
            && process.prepare_interrupt_wait()
        {
            return Err(ReadError::Interrupted);
        }

        // Clearing the arrival flag can race with another reader clearing it
        // after a producer set it. Re-signalling whenever the ring moved since
        // the scan keeps that race from parking a reader on a record that has
        // already been written.
        ARRIVAL.reset();
        if RING.next_sequence() != observed {
            ARRIVAL.signal();
            continue;
        }

        match process.as_ref() {
            Some(process) => {
                if Event::wait_any(&[&ARRIVAL, process.interrupt_event()]) == 1 {
                    return Err(ReadError::Interrupted);
                }
            }
            None => ARRIVAL.wait(),
        }
    }
}

/// Returns whether a reader positioned at `cursor` has data waiting.
pub fn has_packets(cursor: u64) -> bool {
    cursor.max(first_sequence()) < RING.next_sequence()
}

/// Records a message submitted by userspace through `/dev/kmsg`.
///
/// Each line becomes its own record so a multi-line write stays readable, and
/// oversized lines are split rather than silently cut.
pub fn append_userspace(level: Level, bytes: &[u8]) {
    for line in bytes.split(|byte| *byte == b'\n') {
        if line.is_empty() {
            continue;
        }
        for chunk in line.chunks(packet::MESSAGE_LIMIT) {
            emit(level, "user", "", 0, chunk, packet::FLAG_USERSPACE);
        }
    }
}

/// Writes one preformatted line to every registered sink.
pub(crate) fn write_to_sinks(line: &[u8]) {
    let sinks = SINKS.lock();
    for sink in sinks.iter().flatten() {
        sink(line);
    }
}

/// Stops ordinary records from reaching the ring or the console.
///
/// Called once a panic owns the machine so a stray log from an interrupt that
/// was already in flight cannot interleave with the crash report.
pub(crate) fn enter_panic_mode() {
    PANIC_MODE.store(true, Ordering::Release);
}

/// Formats and writes a panic-time line straight to the console sinks.
///
/// The message also enters the ring so a debugger attached to `/dev/klog` on
/// another machine sees the same report the console printed.
pub(crate) fn emergency(args: fmt::Arguments<'_>) {
    let mut line = TextBuffer::<CONSOLE_LINE_CAPACITY>::new();
    let _ = writeln!(&mut line, "{args}");
    write_to_sinks(line.as_slice());

    let text = line.as_slice();
    let text = text.strip_suffix(b"\n").unwrap_or(text);
    let (cpu, thread) = origin();
    RING.append(&RecordData {
        level: Level::Emergency.as_raw(),
        flags: packet::FLAG_EMERGENCY,
        timestamp_ns: clock::monotonic_ns_or_zero(),
        cpu,
        line: 0,
        thread,
        subsystem: b"panic",
        file: b"",
        message: text,
    });
}

/// Releases the console sink registry after a panic stopped every other CPU.
///
/// # Safety
///
/// Only valid once the caller has stopped all other CPUs or accepts that a
/// previous lock owner will never resume.
pub(crate) unsafe fn force_unlock_for_panic() {
    if SINKS.is_locked() {
        // SAFETY: upheld by this function's panic-only caller contract.
        unsafe { SINKS.force_unlock() };
    }
}

/// Rendering options for one log line.
#[derive(Copy, Clone)]
struct Style {
    /// Emit SGR escape sequences.
    color: bool,
    /// Prefix the line with the record's monotonic timestamp.
    timestamp: bool,
}

impl Style {
    /// Style used for console output, matching the original boot log.
    const CONSOLE: Self = Self {
        color: true,
        timestamp: false,
    };

    /// Style used for the plain-text device, where a timestamp is worth the
    /// width because nothing is scrolling past live.
    const TEXT: Self = Self {
        color: false,
        timestamp: true,
    };
}

/// Renders a stored packet as an uncoloured text line into `output`.
///
/// Returns the number of bytes written, which is never more than
/// [`TEXT_LINE_CAPACITY`].
pub(crate) fn render_text(buffer: &PacketBuffer, output: &mut [u8]) -> usize {
    let mut line = TextBuffer::<TEXT_LINE_CAPACITY>::new();
    render(buffer, Style::TEXT, &mut line);
    let bytes = line.as_slice();
    let count = bytes.len().min(output.len());
    output[..count].copy_from_slice(&bytes[..count]);
    count
}

/// Renders a stored packet into `out`.
fn render<const N: usize>(buffer: &PacketBuffer, style: Style, out: &mut TextBuffer<N>) {
    let header = buffer.header();
    let file_start = packet::HEADER_SIZE + header.subsystem_len as usize;
    let file = &buffer.bytes()[file_start..file_start + header.file_len as usize];

    render_fields(
        &RecordData {
            level: header.level,
            flags: header.flags,
            timestamp_ns: header.timestamp_ns,
            cpu: header.cpu,
            line: header.line,
            thread: header.thread,
            subsystem: buffer.subsystem(),
            file,
            message: buffer.message(),
        },
        style,
        out,
    );
}

/// Renders record fields as a line terminated by a newline.
///
/// The layout is the kernel's long-standing boot format: a severity glyph, the
/// dimmed source location, and then the message tagged with its subsystem.
fn render_fields<const N: usize>(record: &RecordData<'_>, style: Style, out: &mut TextBuffer<N>) {
    let level = Level::from_raw(record.level);
    let (glyph, attributes) = level.glyph();

    if style.timestamp {
        let seconds = record.timestamp_ns / 1_000_000_000;
        let microseconds = (record.timestamp_ns % 1_000_000_000) / 1_000;
        let _ = write!(out, "[{seconds:>5}.{microseconds:06}] ");
    }

    if style.color {
        let _ = write!(out, "[\x1b[{attributes}m{glyph}\x1b[0m]");
    } else {
        let _ = write!(out, "[{glyph}]");
    }

    if !record.file.is_empty() {
        if style.color {
            let _ = write!(
                out,
                " \x1b[2m({}:{})\x1b[0m",
                Utf8(record.file),
                record.line
            );
        } else {
            let _ = write!(out, " ({}:{})", Utf8(record.file), record.line);
        }
    }

    out.write_char(' ').ok();
    if !record.subsystem.is_empty() {
        let _ = write!(out, "{}: ", Utf8(record.subsystem));
    }
    let _ = write!(out, "{}", Utf8(record.message));

    if record.flags & packet::FLAG_TRUNCATED != 0 {
        let _ = out.write_str("...");
    }

    let _ = out.write_str("\n");
}

/// Returns the CPU and thread that a record originated from.
///
/// Both come from core-local state rather than the scheduler so logging stays
/// lock-free and remains usable from interrupt and early-boot contexts.
fn origin() -> (u16, u32) {
    match crate::arch::thiscpu_opt() {
        Some(cpu) => (
            cpu.id.min(u16::MAX as usize) as u16,
            cpu.current_thread.min(u32::MAX as usize) as u32,
        ),
        None => (0, 0),
    }
}

/// Derives a subsystem tag from a [`log`] target.
///
/// Module paths already describe the kernel's structure, so `roanix::mem::phys`
/// becomes `mem/phys` and every record is tagged consistently without callers
/// repeating the name in each message. A `target:` given explicitly at the call
/// site passes through untouched, which is how drivers tag records with their
/// module name.
fn subsystem_of<'a>(target: &str, out: &'a mut [u8; packet::SUBSYSTEM_LIMIT]) -> &'a str {
    let source = target.strip_prefix("roanix::").unwrap_or(target).as_bytes();
    let mut length = 0;
    let mut index = 0;

    while index < source.len() && length < out.len() {
        if source[index..].starts_with(b"::") {
            out[length] = b'/';
            index += 2;
        } else {
            out[length] = source[index];
            index += 1;
        }
        length += 1;
    }

    // Module paths are ASCII, so this only guards against a caller passing a
    // target that was cut mid-sequence.
    core::str::from_utf8(&out[..length]).unwrap_or("kernel")
}

/// Shortens a source path to the part that identifies the file.
fn trim_source_path(path: &str) -> &str {
    path.strip_prefix("kernel/").unwrap_or(path)
}

/// Prints possibly non-UTF-8 bytes, replacing anything unprintable.
struct Utf8<'a>(&'a [u8]);

impl fmt::Display for Utf8<'_> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut rest = self.0;
        while !rest.is_empty() {
            match core::str::from_utf8(rest) {
                Ok(text) => return formatter.write_str(text),
                Err(error) => {
                    let valid = error.valid_up_to();
                    // SAFETY: `from_utf8` reported this prefix as well-formed.
                    formatter.write_str(unsafe { core::str::from_utf8_unchecked(&rest[..valid]) })?;
                    formatter.write_char('\u{fffd}')?;
                    rest = &rest[valid + error.error_len().unwrap_or(1)..];
                }
            }
        }
        Ok(())
    }
}
