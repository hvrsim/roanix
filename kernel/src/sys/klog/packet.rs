//!
//! # Kernel Log Packets
//!
//! Wire format of a kernel log record and the wait-free ring that stores them.
//!
//! Every message the kernel emits becomes one self-describing *packet*: a
//! fixed header carrying the level, monotonic timestamp, sequence number and
//! origin, followed by the subsystem, source location and message text. The
//! same bytes are handed to userspace through `/dev/klog`, so the layout is
//! part of the kernel ABI and only ever grows at the end of the header.
//!
//! Records live in a power-of-two array of fixed slots. Writers claim a slot
//! with a single atomic increment and publish it with one release store, so a
//! producer never blocks, never allocates and never takes a lock. Readers use
//! the publication word as a sequence lock: they validate it before and after
//! copying, which lets a slow reader be overrun without ever observing a torn
//! record.
//!

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU64, Ordering};

/// Magic word starting every packet, `"KLOG"` in ASCII.
///
/// A reader that loses framing can scan forward for this value to resynchronize
/// on the next packet boundary.
pub const PACKET_MAGIC: u32 = 0x474f_4c4b;

/// Version of the packet layout described by this module.
pub const PACKET_VERSION: u8 = 1;

/// Size in bytes of the fixed packet header.
pub const HEADER_SIZE: usize = 40;

/// Bytes of variable-length payload a single record can carry.
pub const PAYLOAD_CAPACITY: usize = 256;

/// Largest packet any reader can observe.
pub const MAX_PACKET_SIZE: usize = HEADER_SIZE + PAYLOAD_CAPACITY;

/// Number of records the ring retains. Must be a power of two.
pub const RING_SLOTS: usize = 512;

/// Longest subsystem tag stored with a record.
pub const SUBSYSTEM_LIMIT: usize = 32;

/// Longest source path stored with a record.
pub const FILE_LIMIT: usize = 48;

/// Message text a record can always store, whatever its tag and path cost.
pub const MESSAGE_LIMIT: usize = PAYLOAD_CAPACITY - SUBSYSTEM_LIMIT - FILE_LIMIT;

const SLOT_MASK: u64 = RING_SLOTS as u64 - 1;
const COMMITTED: u64 = 1;

/// The message text did not fit and was cut short.
pub const FLAG_TRUNCATED: u8 = 1 << 0;

/// The record was submitted by userspace through `/dev/kmsg`.
pub const FLAG_USERSPACE: u8 = 1 << 1;

/// The record was emitted from the panic path with the log core bypassed.
pub const FLAG_EMERGENCY: u8 = 1 << 2;

/// Fixed-size prefix shared by every kernel log packet.
///
/// The field order is frozen; `/dev/klog` readers decode these bytes directly.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Header {
    /// Always [`PACKET_MAGIC`].
    pub magic: u32,
    /// Total packet size in bytes, header included.
    pub length: u16,
    /// Severity, see [`super::Level`].
    pub level: u8,
    /// Layout version, currently [`PACKET_VERSION`].
    pub version: u8,
    /// Monotonically increasing record number, never reused.
    pub sequence: u64,
    /// Nanoseconds since the clocksource was registered.
    pub timestamp_ns: u64,
    /// CPU that produced the record.
    pub cpu: u16,
    /// Source line, or zero when unknown.
    pub line: u16,
    /// Thread that produced the record, or zero outside thread context.
    pub thread: u32,
    /// Payload bytes holding the subsystem tag.
    pub subsystem_len: u8,
    /// Payload bytes holding the source path.
    pub file_len: u8,
    /// Payload bytes holding the message text.
    pub message_len: u16,
    /// `FLAG_*` bits describing how the record was produced.
    pub flags: u8,
    /// Reserved for future header fields, always zero.
    pub reserved: [u8; 3],
}

impl Header {
    /// Creates a zeroed header.
    const fn empty() -> Self {
        Self {
            magic: PACKET_MAGIC,
            length: HEADER_SIZE as u16,
            level: 0,
            version: PACKET_VERSION,
            sequence: 0,
            timestamp_ns: 0,
            cpu: 0,
            line: 0,
            thread: 0,
            subsystem_len: 0,
            file_len: 0,
            message_len: 0,
            flags: 0,
            reserved: [0; 3],
        }
    }

    /// Serializes the header into the first [`HEADER_SIZE`] bytes of `out`.
    fn encode(&self, out: &mut [u8; HEADER_SIZE]) {
        out[0..4].copy_from_slice(&self.magic.to_ne_bytes());
        out[4..6].copy_from_slice(&self.length.to_ne_bytes());
        out[6] = self.level;
        out[7] = self.version;
        out[8..16].copy_from_slice(&self.sequence.to_ne_bytes());
        out[16..24].copy_from_slice(&self.timestamp_ns.to_ne_bytes());
        out[24..26].copy_from_slice(&self.cpu.to_ne_bytes());
        out[26..28].copy_from_slice(&self.line.to_ne_bytes());
        out[28..32].copy_from_slice(&self.thread.to_ne_bytes());
        out[32] = self.subsystem_len;
        out[33] = self.file_len;
        out[34..36].copy_from_slice(&self.message_len.to_ne_bytes());
        out[36] = self.flags;
        out[37..40].copy_from_slice(&self.reserved);
    }
}

const _: () = assert!(size_of::<Header>() == HEADER_SIZE);

/// Fields supplied by a producer for one log record.
pub struct RecordData<'a> {
    /// Severity of the message.
    pub level: u8,
    /// `FLAG_*` bits describing how the record was produced.
    pub flags: u8,
    /// Nanoseconds since the clocksource was registered.
    pub timestamp_ns: u64,
    /// CPU producing the record.
    pub cpu: u16,
    /// Source line, or zero when unknown.
    pub line: u16,
    /// Producing thread, or zero outside thread context.
    pub thread: u32,
    /// Subsystem tag such as `mem/phys`.
    pub subsystem: &'a [u8],
    /// Source path of the call site.
    pub file: &'a [u8],
    /// Message text without a trailing newline.
    pub message: &'a [u8],
}

/// Outcome of copying one record out of the ring.
pub enum Fetch {
    /// The packet was copied and occupies the returned number of bytes.
    Ready(usize),
    /// The record has been claimed but is not published yet.
    Pending,
    /// The record was overwritten; resume from the returned sequence.
    Lost(u64),
}

/// A decoded packet held in caller-owned storage.
pub struct PacketBuffer {
    bytes: [u8; MAX_PACKET_SIZE],
    length: usize,
    header: Header,
}

impl PacketBuffer {
    /// Creates an empty packet buffer.
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_PACKET_SIZE],
            length: 0,
            header: Header::empty(),
        }
    }

    /// Returns the encoded packet bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }

    /// Returns the header of the buffered packet.
    pub fn header(&self) -> Header {
        self.header
    }

    /// Returns the subsystem tag of the buffered packet.
    pub fn subsystem(&self) -> &[u8] {
        let end = HEADER_SIZE + self.header.subsystem_len as usize;
        &self.bytes[HEADER_SIZE..end.min(self.length)]
    }

    /// Returns the message text of the buffered packet.
    pub fn message(&self) -> &[u8] {
        let start =
            HEADER_SIZE + self.header.subsystem_len as usize + self.header.file_len as usize;
        let end = start + self.header.message_len as usize;
        if start >= self.length {
            return &[];
        }
        &self.bytes[start..end.min(self.length)]
    }
}

impl Default for PacketBuffer {
    fn default() -> Self {
        Self::new()
    }
}

/// One ring entry plus the publication word guarding it.
struct Slot {
    /// `0` while never written, otherwise `(sequence << 1) | committed`.
    state: AtomicU64,
    /// Record metadata, valid only while `state` reports a committed sequence.
    header: UnsafeCell<Header>,
    /// Subsystem, source path and message bytes, in that order.
    payload: UnsafeCell<[u8; PAYLOAD_CAPACITY]>,
}

// SAFETY: slot contents are only reached through the publication word. A
// producer owns a slot exclusively between claiming a sequence and publishing
// it, and consumers copy the contents out under a sequence-lock validation that
// discards anything a concurrent producer touched.
unsafe impl Sync for Slot {}

impl Slot {
    const fn new() -> Self {
        Self {
            state: AtomicU64::new(0),
            header: UnsafeCell::new(Header::empty()),
            payload: UnsafeCell::new([0; PAYLOAD_CAPACITY]),
        }
    }
}

/// Wait-free multi-producer ring of kernel log records.
pub struct Ring {
    slots: [Slot; RING_SLOTS],
    next: AtomicU64,
    truncated: AtomicU64,
}

impl Default for Ring {
    fn default() -> Self {
        Self::new()
    }
}

impl Ring {
    /// Creates an empty ring.
    pub const fn new() -> Self {
        Self {
            slots: [const { Slot::new() }; RING_SLOTS],
            next: AtomicU64::new(0),
            truncated: AtomicU64::new(0),
        }
    }

    /// Sequence that will be assigned to the next record.
    #[inline]
    pub fn next_sequence(&self) -> u64 {
        self.next.load(Ordering::Acquire)
    }

    /// Oldest sequence the ring can still be expected to hold.
    #[inline]
    pub fn first_sequence(&self) -> u64 {
        self.next_sequence().saturating_sub(RING_SLOTS as u64)
    }

    /// Number of records whose message text did not fit in a slot.
    #[inline]
    pub fn truncated(&self) -> u64 {
        self.truncated.load(Ordering::Relaxed)
    }

    /// Stores one record and returns the sequence it was given.
    ///
    /// The claim is a single atomic increment and the publish is a single
    /// release store, so this never blocks and is safe from interrupt context.
    pub fn append(&self, record: &RecordData<'_>) -> u64 {
        let sequence = self.next.fetch_add(1, Ordering::Relaxed);
        let slot = &self.slots[(sequence & SLOT_MASK) as usize];

        // Retire the previous generation before touching its bytes so a reader
        // copying it observes the sequence change and discards the result.
        slot.state.store(sequence << 1, Ordering::Release);

        let subsystem = clamp(record.subsystem, SUBSYSTEM_LIMIT);
        let file = clamp(record.file, FILE_LIMIT);
        let room = PAYLOAD_CAPACITY - subsystem.len() - file.len();
        let message = clamp(record.message, room);

        let mut flags = record.flags;
        if message.len() != record.message.len() {
            flags |= FLAG_TRUNCATED;
            self.truncated.fetch_add(1, Ordering::Relaxed);
        }

        let header = Header {
            length: (HEADER_SIZE + subsystem.len() + file.len() + message.len()) as u16,
            level: record.level,
            sequence,
            timestamp_ns: record.timestamp_ns,
            cpu: record.cpu,
            line: record.line,
            thread: record.thread,
            subsystem_len: subsystem.len() as u8,
            file_len: file.len() as u8,
            message_len: message.len() as u16,
            flags,
            ..Header::empty()
        };

        // SAFETY: this producer owns the slot for the whole window between the
        // reservation store above and the publication store below. Any reader
        // that observes those bytes also observes the state change and throws
        // the copy away.
        unsafe {
            let payload = &mut *slot.payload.get();
            let mut cursor = 0;
            for part in [subsystem, file, message] {
                payload[cursor..cursor + part.len()].copy_from_slice(part);
                cursor += part.len();
            }
            slot.header.get().write(header);
        }

        slot.state
            .store((sequence << 1) | COMMITTED, Ordering::Release);
        sequence
    }

    /// Copies the record at `sequence` into `out`.
    pub fn fetch(&self, sequence: u64, out: &mut PacketBuffer) -> Fetch {
        let slot = &self.slots[(sequence & SLOT_MASK) as usize];

        let before = slot.state.load(Ordering::Acquire);
        if before == 0 {
            return Fetch::Pending;
        }

        let stored = before >> 1;
        if stored > sequence {
            return Fetch::Lost(self.first_sequence());
        }
        if stored < sequence || before & COMMITTED == 0 {
            return Fetch::Pending;
        }

        // SAFETY: the publication word reported this generation as committed.
        // Both reads are volatile so the compiler cannot split, merge or hoist
        // them across the validation load that follows, and the copies are
        // discarded whenever a producer moved the slot on underneath them.
        let (header, payload) = unsafe {
            (
                slot.header.get().read_volatile(),
                slot.payload.get().read_volatile(),
            )
        };

        if slot.state.load(Ordering::Acquire) != before {
            return Fetch::Lost(self.first_sequence());
        }

        // Lengths are re-clamped even though the sequence lock validated the
        // copy, so a corrupt header can never index outside the payload.
        let subsystem_len = (header.subsystem_len as usize).min(PAYLOAD_CAPACITY);
        let file_len = (header.file_len as usize).min(PAYLOAD_CAPACITY - subsystem_len);
        let message_len =
            (header.message_len as usize).min(PAYLOAD_CAPACITY - subsystem_len - file_len);
        let payload_len = subsystem_len + file_len + message_len;

        let header = Header {
            subsystem_len: subsystem_len as u8,
            file_len: file_len as u8,
            message_len: message_len as u16,
            length: (HEADER_SIZE + payload_len) as u16,
            ..header
        };

        let mut encoded = [0u8; HEADER_SIZE];
        header.encode(&mut encoded);
        out.bytes[..HEADER_SIZE].copy_from_slice(&encoded);
        out.bytes[HEADER_SIZE..HEADER_SIZE + payload_len].copy_from_slice(&payload[..payload_len]);
        out.length = HEADER_SIZE + payload_len;
        out.header = header;

        Fetch::Ready(out.length)
    }
}

/// Shortens `bytes` to at most `limit`, splitting on a UTF-8 boundary.
///
/// Cutting mid-sequence would hand `/dev/klog` readers an invalid string, so
/// the split walks back over continuation bytes.
fn clamp(bytes: &[u8], limit: usize) -> &[u8] {
    if bytes.len() <= limit {
        return bytes;
    }

    let mut end = limit;
    while end > 0 && bytes[end] & 0xc0 == 0x80 {
        end -= 1;
    }
    &bytes[..end]
}
