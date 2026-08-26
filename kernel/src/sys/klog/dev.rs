//!
//! # Kernel Log Devices
//!
//! `/dev/klog` and `/dev/kmsg`, the two views userspace has of the kernel log.
//!
//! `/dev/klog` streams raw [`packet`] records: one binary packet per message,
//! carrying the level, timestamp, sequence, CPU, thread and source location.
//! `dmesg` reads it and does its own formatting and filtering, and its `ioctl`
//! commands are how the log level is changed at run time.
//!
//! `/dev/kmsg` serves the same records already rendered as plain text lines,
//! which keeps `cat /dev/kmsg` and existing POSIX software working.
//!
//! Both devices are created by the kernel rather than a driver module. The log
//! must be readable even when driver loading is exactly what went wrong.
//!

use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::{
    driver::class::chardev::{self, DeviceNodeOps},
    fs::{Error, OpenFlags, PollEvents, Result, vnode::IoctlContext},
    mem::{IoSink, IoSource},
    sys::{event::Event, sync::Mutex},
};

use super::{Level, ReadError, packet};

/// Builds a Linux-compatible ioctl request code.
const fn ioc(direction: u64, number: u64, size: u64) -> u64 {
    (direction << 30) | (size << 16) | ((b'K' as u64) << 8) | number
}

const READ: u64 = 2;
const WRITE: u64 = 1;
const NONE: u64 = 0;
const U32_SIZE: u64 = size_of::<u32>() as u64;
const STATS_SIZE: u64 = size_of::<super::Stats>() as u64;

/// Reads the severity currently recorded into the log.
pub const KLOG_GET_LEVEL: u64 = ioc(READ, 1, U32_SIZE);
/// Changes the severity recorded into the log.
pub const KLOG_SET_LEVEL: u64 = ioc(WRITE, 2, U32_SIZE);
/// Reads the severity mirrored to the console.
pub const KLOG_GET_CONSOLE_LEVEL: u64 = ioc(READ, 3, U32_SIZE);
/// Changes the severity mirrored to the console.
pub const KLOG_SET_CONSOLE_LEVEL: u64 = ioc(WRITE, 4, U32_SIZE);
/// Hides every record written so far.
pub const KLOG_CLEAR: u64 = ioc(NONE, 5, 0);
/// Reads ring occupancy and the active filters.
pub const KLOG_GET_STATS: u64 = ioc(READ, 6, STATS_SIZE);
/// Rewinds this descriptor to the oldest retained record.
pub const KLOG_SEEK_FIRST: u64 = ioc(NONE, 7, 0);
/// Skips this descriptor forward to the newest record.
pub const KLOG_SEEK_LAST: u64 = ioc(NONE, 8, 0);

const _: () = assert!(STATS_SIZE == 48);

/// How a device renders the records it serves.
#[derive(Copy, Clone, Eq, PartialEq)]
enum Encoding {
    /// Raw binary packets.
    Packets,
    /// Plain text lines.
    Text,
}

/// Per-descriptor read position in the log stream.
///
/// Every open gets its own cursor so two concurrent readers, such as a running
/// `dmesg --follow` and a one-shot dump, never steal records from each other.
struct Reader {
    sequence: AtomicU64,
    /// Rendered text left over from a read whose buffer filled mid-line.
    ///
    /// Text readers are ordinary programs with ordinary buffer sizes, so a line
    /// has to be able to span two reads. Packet readers never need this: a
    /// packet is delivered whole or not at all.
    partial: Mutex<PartialLine>,
}

/// A rendered line being delivered across more than one read.
struct PartialLine {
    bytes: [u8; super::TEXT_LINE_CAPACITY],
    offset: usize,
    length: usize,
}

impl PartialLine {
    const fn new() -> Self {
        Self {
            bytes: [0; super::TEXT_LINE_CAPACITY],
            offset: 0,
            length: 0,
        }
    }

    fn remaining(&self) -> &[u8] {
        &self.bytes[self.offset..self.length]
    }
}

/// Shared implementation behind both log device nodes.
struct LogDevice {
    encoding: Encoding,
}

impl LogDevice {
    /// Recovers the reader owned by an open file description.
    ///
    /// # Safety
    ///
    /// `file_context` must be a value returned by [`DeviceNodeOps::open`] on
    /// this device and not yet passed to [`DeviceNodeOps::close`].
    unsafe fn reader<'a>(&self, file_context: usize) -> Result<&'a Reader> {
        if file_context == 0 {
            return Err(Error::BadFileDescriptor);
        }
        // SAFETY: upheld by this function's contract. The allocation stays live
        // until `close` reclaims it, and the VFS never reuses a context after
        // that.
        Ok(unsafe { &*(file_context as *const Reader) })
    }

    /// Copies whole packets into `sink`.
    ///
    /// A caller whose buffer cannot hold the largest possible packet is
    /// rejected rather than served a fragment, because a partial packet would
    /// desynchronize the reader's framing for good.
    fn read_packets(
        &self,
        reader: &Reader,
        sink: &mut IoSink<'_>,
        nonblocking: bool,
    ) -> Result<usize> {
        let total = sink.len();
        if total < packet::MAX_PACKET_SIZE {
            return Err(Error::InvalidArgument);
        }

        let mut buffer = packet::PacketBuffer::new();
        let mut cursor = reader.sequence.load(Ordering::Acquire);
        let mut written = 0;

        while total - written >= packet::MAX_PACKET_SIZE {
            match super::next_packet(&mut cursor, &mut buffer, nonblocking || written != 0) {
                Ok(()) => {}
                Err(ReadError::WouldBlock) if written != 0 => break,
                Err(ReadError::WouldBlock) => return Err(Error::WouldBlock),
                Err(ReadError::Interrupted) if written != 0 => break,
                Err(ReadError::Interrupted) => return Err(Error::Interrupted),
                Err(ReadError::BufferTooSmall) => return Err(Error::InvalidArgument),
            }

            let bytes = buffer.bytes();
            sink.store(written, bytes)
                .map_err(|_| Error::InvalidArgument)?;
            written += bytes.len();
            // Publishing after the copy keeps a faulting read from consuming
            // records the caller never received.
            reader.sequence.store(cursor, Ordering::Release);
        }

        Ok(written)
    }

    /// Copies rendered text lines into `sink`, resuming any partial line.
    fn read_text(
        &self,
        reader: &Reader,
        sink: &mut IoSink<'_>,
        nonblocking: bool,
    ) -> Result<usize> {
        let total = sink.len();
        let mut partial = reader.partial.lock();
        let mut buffer = packet::PacketBuffer::new();
        let mut cursor = reader.sequence.load(Ordering::Acquire);
        let mut written = 0;

        while written < total {
            if partial.offset < partial.length {
                let available = partial.length - partial.offset;
                let count = available.min(total - written);
                let start = written;
                let offset = partial.offset;
                sink.store(start, &partial.bytes[offset..offset + count])
                    .map_err(|_| Error::InvalidArgument)?;
                partial.offset += count;
                written += count;
                continue;
            }

            match super::next_packet(&mut cursor, &mut buffer, nonblocking || written != 0) {
                Ok(()) => {}
                Err(ReadError::WouldBlock) if written != 0 => break,
                Err(ReadError::WouldBlock) => return Err(Error::WouldBlock),
                Err(ReadError::Interrupted) if written != 0 => break,
                Err(ReadError::Interrupted) => return Err(Error::Interrupted),
                Err(ReadError::BufferTooSmall) => return Err(Error::InvalidArgument),
            }

            partial.length = super::render_text(&buffer, &mut partial.bytes);
            partial.offset = 0;
            reader.sequence.store(cursor, Ordering::Release);
        }

        Ok(written)
    }

    /// Returns whether this reader has anything left to deliver.
    fn readable(&self, reader: &Reader) -> bool {
        if self.encoding == Encoding::Text && !reader.partial.lock().remaining().is_empty() {
            return true;
        }
        super::has_packets(reader.sequence.load(Ordering::Acquire))
    }
}

impl DeviceNodeOps for LogDevice {
    fn open(&self, _flags: u32) -> Result<usize> {
        let reader = Box::new(Reader {
            sequence: AtomicU64::new(super::first_sequence()),
            partial: Mutex::new(PartialLine::new()),
        });
        Ok(Box::into_raw(reader) as usize)
    }

    fn close(&self, file_context: usize, _flags: u32) {
        if file_context == 0 {
            return;
        }
        // SAFETY: the context came from `open`, which leaked exactly one
        // `Reader` allocation, and the VFS calls `close` once per open.
        drop(unsafe { Box::from_raw(file_context as *mut Reader) });
    }

    fn read_at_with_flags(
        &self,
        file_context: usize,
        _offset: u64,
        sink: &mut IoSink<'_>,
        flags: u32,
    ) -> Result<usize> {
        // SAFETY: the VFS only supplies a context this device handed out.
        let reader = unsafe { self.reader(file_context) }?;
        if sink.is_empty() {
            return Ok(0);
        }

        let nonblocking = flags & OpenFlags::NONBLOCK.bits() != 0;
        match self.encoding {
            Encoding::Packets => self.read_packets(reader, sink, nonblocking),
            Encoding::Text => self.read_text(reader, sink, nonblocking),
        }
    }

    fn write_at_with_flags(
        &self,
        _file_context: usize,
        _offset: u64,
        source: &IoSource<'_>,
        _flags: u32,
    ) -> Result<usize> {
        let length = source.len();
        if length == 0 {
            return Ok(0);
        }
        if length > MAX_USER_WRITE {
            return Err(Error::InvalidArgument);
        }

        let mut bytes = alloc::vec![0u8; length];
        source
            .load(0, &mut bytes)
            .map_err(|_| Error::InvalidArgument)?;

        let (level, message) = split_level_prefix(&bytes);
        super::append_userspace(level, message);
        Ok(length)
    }

    fn poll(
        &self,
        file_context: usize,
        _offset: u64,
        events: PollEvents,
        _flags: u32,
    ) -> Result<PollEvents> {
        // SAFETY: the VFS only supplies a context this device handed out.
        let reader = unsafe { self.reader(file_context) }?;
        let mut ready = events & (PollEvents::OUT | PollEvents::WRNORM);
        if self.readable(reader) {
            ready |= events & (PollEvents::IN | PollEvents::RDNORM);
        }
        Ok(ready)
    }

    fn poll_events<'a>(
        &'a self,
        _file_context: usize,
        events: PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        if events.intersects(PollEvents::IN | PollEvents::RDNORM) {
            output.push(super::arrival_event());
        }
        true
    }

    fn ioctl(
        &self,
        file_context: usize,
        _context: IoctlContext,
        request: u64,
        _value: u64,
        argument: &mut [u8],
    ) -> Result<u64> {
        // SAFETY: the VFS only supplies a context this device handed out.
        let reader = unsafe { self.reader(file_context) }?;

        match request {
            KLOG_GET_LEVEL => store_u32(argument, super::record_level().as_raw().into()),
            KLOG_SET_LEVEL => {
                super::set_record_level(load_level(argument)?);
                Ok(0)
            }
            KLOG_GET_CONSOLE_LEVEL => store_u32(argument, super::console_level().as_raw().into()),
            KLOG_SET_CONSOLE_LEVEL => {
                super::set_console_level(load_level(argument)?);
                Ok(0)
            }
            KLOG_CLEAR => {
                super::clear();
                reader
                    .sequence
                    .store(super::first_sequence(), Ordering::Release);
                Ok(0)
            }
            KLOG_GET_STATS => {
                let stats = super::stats();
                if argument.len() != size_of::<super::Stats>() {
                    return Err(Error::InvalidArgument);
                }
                // SAFETY: `Stats` is a `repr(C)` aggregate of integers with no
                // padding, so its bytes are a valid representation to copy out.
                let bytes = unsafe {
                    core::slice::from_raw_parts(
                        (&raw const stats).cast::<u8>(),
                        size_of::<super::Stats>(),
                    )
                };
                argument.copy_from_slice(bytes);
                Ok(0)
            }
            KLOG_SEEK_FIRST => {
                reader
                    .sequence
                    .store(super::first_sequence(), Ordering::Release);
                Ok(0)
            }
            KLOG_SEEK_LAST => {
                reader
                    .sequence
                    .store(super::next_sequence(), Ordering::Release);
                Ok(0)
            }
            _ => Err(Error::InvalidArgument),
        }
    }
}

/// Largest single write accepted through a log device.
const MAX_USER_WRITE: usize = 4096;

/// Publishes `/dev/klog` and `/dev/kmsg`.
pub(crate) fn register() -> Result<()> {
    let root = chardev::root().map_err(Error::from)?;

    for (name, encoding, mode) in [
        (&b"klog"[..], Encoding::Packets, 0o600),
        (&b"kmsg"[..], Encoding::Text, 0o600),
    ] {
        chardev::create_native_node(
            None,
            root,
            core::str::from_utf8(name).map_err(|_| Error::InvalidArgument)?,
            chardev::kind::CHARACTER,
            mode,
            Arc::new(LogDevice { encoding }),
        )
        .map_err(Error::from)?;
    }

    let stats = super::stats();
    log::debug!(
        "/dev/klog and /dev/kmsg ready, {} record buffer, recording at {}",
        stats.slots,
        super::record_level().name()
    );
    Ok(())
}

/// Splits a leading `<level>` prefix from a userspace log write.
///
/// Mirrors the `/dev/kmsg` convention so existing tools can pick a severity
/// instead of having everything land at one level.
fn split_level_prefix(bytes: &[u8]) -> (Level, &[u8]) {
    let default = Level::Info;
    let Some(rest) = bytes.strip_prefix(b"<") else {
        return (default, bytes);
    };
    let Some(end) = rest.iter().position(|byte| *byte == b'>') else {
        return (default, bytes);
    };
    let digits = &rest[..end];
    if digits.is_empty() || digits.len() > 2 || !digits.iter().all(u8::is_ascii_digit) {
        return (default, bytes);
    }

    let value = digits
        .iter()
        .fold(0u32, |total, digit| total * 10 + u32::from(digit - b'0'));
    if value >= u32::from(super::LEVEL_COUNT) {
        return (default, bytes);
    }
    (Level::from_raw(value as u8), &rest[end + 1..])
}

/// Writes a `u32` result into an ioctl argument buffer.
fn store_u32(argument: &mut [u8], value: u32) -> Result<u64> {
    if argument.len() != size_of::<u32>() {
        return Err(Error::InvalidArgument);
    }
    argument.copy_from_slice(&value.to_ne_bytes());
    Ok(0)
}

/// Reads a severity from an ioctl argument buffer.
fn load_level(argument: &[u8]) -> Result<Level> {
    let bytes: [u8; 4] = argument.try_into().map_err(|_| Error::InvalidArgument)?;
    let value = u32::from_ne_bytes(bytes);
    if value >= u32::from(super::LEVEL_COUNT) {
        return Err(Error::InvalidArgument);
    }
    Ok(Level::from_raw(value as u8))
}
