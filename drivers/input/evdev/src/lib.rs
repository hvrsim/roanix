//!
//! # Evdev Input Endpoints
//!
//! Hardware drivers publish a static [`Endpoint`] as a devfs event node and
//! report keyboard or mouse input through it. Each open file has its own record
//! queue and event, so readers consume input independently.
//!
//! The endpoint's IRQ-safe lock protects key and button state, reader queues, and
//! readiness changes. Blocking reads wait outside that lock. On disconnect,
//! readers wake and report hangup while the node stays offline until it can
//! be removed. Drivers serialize publication and removal for each endpoint.
//!

#![no_std]
#![allow(unsafe_code, clippy::cast_possible_truncation, clippy::cast_sign_loss)]

use core::{
    ffi::{CStr, c_void},
    ptr,
};
use ddk::{Error, Event, OwnedEvent, Result, TicketLock, devfs, raw};

const READERS: usize = 8;
const QUEUE: usize = 256;
const RECORD: usize = 24;
const EV_SYN: u16 = 0;
const EV_KEY: u16 = 1;
const EV_REL: u16 = 2;
const REL_X: u16 = 0;
const REL_Y: u16 = 1;
const REL_WHEEL: u16 = 8;
const BTN_LEFT: u16 = 0x110;
const BTN_RIGHT: u16 = 0x111;
const BTN_MIDDLE: u16 = 0x112;
const MOUSE_BUTTONS: [u16; 3] = [BTN_LEFT, BTN_RIGHT, BTN_MIDDLE];
const SYN_REPORT: u16 = 0;
const SYN_DROPPED: u16 = 3;
const EVIOCREVOKE: u64 = 0x4004_4591;

/// Size in bytes of the evdev key capability and pressed-key bitmaps.
pub const KEY_BYTES: usize = 96;

/// Queue and readiness state for one open evdev file.
struct Reader {
    /// Owned event for readable data or hangup. An empty slot has no event.
    event: Option<OwnedEvent>,

    /// Whether this file has been revoked through its ioctl.
    revoked: bool,

    /// Index of the oldest queued record.
    head: usize,

    /// Number of records queued for this file.
    length: usize,

    /// Ring storage for native evdev records.
    records: [[u8; RECORD]; QUEUE],
}

impl Reader {
    const fn new() -> Self {
        Self {
            event: None,
            revoked: false,
            head: 0,
            length: 0,
            records: [[0; RECORD]; QUEUE],
        }
    }

    /// Queues a record for this reader and reports overflow with `SYN_DROPPED`.
    fn push(&mut self, record: [u8; RECORD]) {
        let Some(event) = &self.event else {
            return;
        };

        if self.revoked {
            return;
        }

        let was_empty = self.length == 0;
        if self.length == QUEUE {
            self.head = 0;
            self.length = 1;
            self.records[0] = event_record(EV_SYN, SYN_DROPPED, 0);
        }

        self.records[(self.head + self.length) % QUEUE] = record;
        self.length += 1;
        if was_empty {
            let _ = event.signal();
        }
    }

    /// Copies whole records, advances the queue head, and returns the byte count.
    fn read_records(&mut self, output: &mut [u8]) -> usize {
        let count = self.length.min(output.len() / RECORD);
        for record in output.chunks_exact_mut(RECORD).take(count) {
            record.copy_from_slice(&self.records[self.head]);
            self.head = (self.head + 1) % QUEUE;
        }

        self.length -= count;
        count * RECORD
    }
}

/// Input state shared by input producers and devfs callbacks.
struct State {
    /// Reader slots indexed by the devfs file context.
    readers: [Reader; READERS],

    /// Bitmap of keys currently pressed.
    pressed: [u8; KEY_BYTES],

    /// Bitmap of keys supported by this hardware attachment.
    keys: [u8; KEY_BYTES],

    /// Bitmap of relative axes supported by this hardware attachment.
    relative: [u8; 2],

    /// Whether the endpoint accepts input and new reader operations.
    online: bool,

    /// Devfs node retained until removal succeeds.
    node: Option<u64>,
}

impl State {
    /// Validates an input event and queues it to all active readers.
    fn queue_event(&mut self, kind: u16, code: u16, mut value: i32) {
        if !self.online {
            return;
        }

        if kind == EV_KEY {
            let index = usize::from(code);
            if index >= KEY_BYTES * 8 || !(0..=2).contains(&value) {
                return;
            }

            let mask = 1 << (index % 8);
            let byte = index / 8;
            if self.keys[byte] & mask == 0 {
                return;
            }

            let pressed = &mut self.pressed[byte];
            if value == 0 {
                *pressed &= !mask;
            } else {
                if *pressed & mask != 0 {
                    value = 2;
                }
                *pressed |= mask;
            }
        } else if kind == EV_REL {
            let axis = usize::from(code);
            if value == 0
                || axis >= self.relative.len() * 8
                || self.relative[axis / 8] & (1 << (axis % 8)) == 0
            {
                return;
            }
        } else if kind != EV_SYN {
            return;
        }

        let record = event_record(kind, code, value);
        for reader in &mut self.readers {
            reader.push(record);
        }
    }
}

/// A static evdev endpoint with a stable address for devfs callbacks.
pub struct Endpoint {
    /// State accessed by hardware producers and devfs callbacks.
    state: TicketLock<State>,

    /// Device name reported through evdev.
    name: &'static CStr,

    /// Bus type reported in the evdev device ID.
    bus: u16,
}

impl Endpoint {
    /// Creates an unpublished input endpoint with no pressed keys or open readers.
    #[must_use]
    #[allow(clippy::large_stack_arrays)] // Evaluated in static endpoint initializers.
    pub const fn new(name: &'static CStr, bus: u16) -> Self {
        Self {
            name,
            bus,
            state: TicketLock::new(State {
                readers: [const { Reader::new() }; READERS],
                pressed: [0; KEY_BYTES],
                keys: [0; KEY_BYTES],
                relative: [0; 2],
                online: false,
                node: None,
            }),
        }
    }

    /// Publishes a keyboard with the supplied key bitmap.
    ///
    /// Call from thread context with exclusive access to the endpoint lifecycle.
    ///
    /// # Errors
    /// Returns a devfs error, including busy readers from an earlier attachment.
    pub fn publish(&'static self, keys: [u8; KEY_BYTES]) -> Result<()> {
        self.publish_with_axes(keys, [0; 2])
    }

    /// Publishes a relative mouse with three buttons and optional vertical scroll.
    ///
    /// Call from thread context with exclusive access to the endpoint lifecycle.
    ///
    /// # Errors
    /// Returns a devfs error, including busy readers from an earlier attachment.
    pub fn publish_mouse(&'static self, wheel: bool) -> Result<()> {
        let mut keys = [0; KEY_BYTES];
        for code in MOUSE_BUTTONS {
            let index = usize::from(code);
            keys[index / 8] |= 1 << (index % 8);
        }

        let axes = [
            (1 << REL_X) | (1 << REL_Y),
            u8::from(wheel) << (REL_WHEEL - 8),
        ];
        self.publish_with_axes(keys, axes)
    }

    /// Registers an endpoint and records its capabilities before opening admission.
    #[allow(clippy::missing_panics_doc)] // Generated event names always contain one trailing NUL.
    fn publish_with_axes(&'static self, keys: [u8; KEY_BYTES], relative: [u8; 2]) -> Result<()> {
        if self.state.lock_irqsave().node.is_some() {
            self.unpublish()?;
        }

        let directory = devfs::lookup(c"input")?;
        let mut ops = OPS;
        ops.context = ptr::from_ref(self).cast_mut().cast();

        {
            let mut state = self.state.lock_irqsave();
            state.keys = keys;
            state.relative = relative;
            state.pressed.fill(0);
            state.online = true;
        }

        let node = (0..64)
            .find_map(|index| {
                let mut name = *b"event00\0";
                name[5] = b'0' + index / 10;
                name[6] = b'0' + index % 10;

                // Single-digit names use event0, matching Linux conventions.
                if index < 10 {
                    name[5] = name[6];
                    name[6] = 0;
                }

                let name = CStr::from_bytes_until_nul(&name).expect("event name");

                // SAFETY: this static endpoint outlives the node and its callbacks.
                match unsafe { devfs::create_character(directory, name, 0o600, &ops) } {
                    Err(error) if error.status() == ddk::EEXIST => None,
                    result => Some(result),
                }
            })
            .unwrap_or(Err(Error::from_status(ddk::ENOSPC)));

        let mut state = self.state.lock_irqsave();
        match node {
            Ok(node) => state.node = Some(node),
            Err(error) => {
                state.online = false;
                return Err(error);
            }
        }

        Ok(())
    }

    /// Disconnects readers and removes the device node.
    ///
    /// Call from thread context and serialize with other lifecycle operations
    /// on this endpoint. An open node stays offline so removal can be retried.
    ///
    /// # Errors
    /// Returns a devfs error if removal fails, including while files remain open.
    pub fn unpublish(&self) -> Result<()> {
        let node = {
            let mut state = self.state.lock_irqsave();
            state.online = false;
            for reader in &state.readers {
                if let Some(event) = &reader.event {
                    let _ = event.signal();
                }
            }
            state.node
        };

        if let Some(node) = node {
            devfs::remove(node)?;
            self.state.lock_irqsave().node = None;
        }

        Ok(())
    }

    /// Updates key state and queues a validated key or synchronization event to each reader.
    pub fn event(&self, kind: u16, code: u16, value: i32) {
        self.state.lock_irqsave().queue_event(kind, code, value);
    }

    /// Reports a key press or release followed by a synchronization event.
    pub fn key(&self, code: u16, down: bool) {
        let mut state = self.state.lock_irqsave();
        state.queue_event(EV_KEY, code, i32::from(down));
        state.queue_event(EV_SYN, SYN_REPORT, 0);
    }

    /// Reports one mouse packet with button bits ordered left, right, and middle.
    ///
    /// The endpoint lock keeps button changes and relative axes in the same
    /// evdev frame. Relative displacement uses the Linux screen coordinate
    /// convention with positive Y moving down.
    pub fn mouse(&self, buttons: u8, x: i32, y: i32, wheel: i32) {
        let mut state = self.state.lock_irqsave();
        let mut changed = false;
        for (bit, code) in MOUSE_BUTTONS.into_iter().enumerate() {
            let index = usize::from(code);
            let down = buttons & (1 << bit) != 0;
            let was_down = state.pressed[index / 8] & (1 << (index % 8)) != 0;
            if down != was_down {
                state.queue_event(EV_KEY, code, i32::from(down));
                changed = true;
            }
        }

        for (axis, value) in [(REL_X, x), (REL_Y, y), (REL_WHEEL, wheel)] {
            if value != 0 {
                state.queue_event(EV_REL, axis, value);
                changed = true;
            }
        }

        if changed {
            state.queue_event(EV_SYN, SYN_REPORT, 0);
        }
    }
}

/// Encodes a timestamped event in the native evdev wire layout.
fn event_record(kind: u16, code: u16, value: i32) -> [u8; RECORD] {
    let now = ddk::monotonic_ns();
    let mut bytes = [0; RECORD];
    bytes[..8].copy_from_slice(&(now / 1_000_000_000).to_ne_bytes());
    bytes[8..16].copy_from_slice(&((now / 1000) % 1_000_000).to_ne_bytes());
    bytes[16..18].copy_from_slice(&kind.to_ne_bytes());
    bytes[18..20].copy_from_slice(&code.to_ne_bytes());
    bytes[20..].copy_from_slice(&value.to_ne_bytes());
    bytes
}

/// Recovers the static input endpoint from a devfs callback context.
///
/// # Safety
/// The context must come from a node published by [`Endpoint::publish`] or [`Endpoint::publish_mouse`].
unsafe fn endpoint(context: *mut c_void) -> &'static Endpoint {
    // SAFETY: publication stores the address of a static endpoint in the context.
    unsafe { &*context.cast::<Endpoint>() }
}

/// Reserves a reader slot with its own queue and notification event.
unsafe extern "C" fn open(context: *mut c_void, _: u32, out: *mut usize) -> i32 {
    let event = match OwnedEvent::create() {
        Ok(event) => event,
        Err(error) => return error.status(),
    };

    // SAFETY: devfs passes the publishing endpoint as the context.
    let endpoint = unsafe { endpoint(context) };
    let index = {
        let mut state = endpoint.state.lock_irqsave();
        if !state.online {
            return ddk::ENODEV;
        }

        let Some(index) = state
            .readers
            .iter()
            .position(|reader| reader.event.is_none())
        else {
            return ddk::ENOSPC;
        };

        let reader = &mut state.readers[index];
        reader.revoked = false;
        reader.head = 0;
        reader.length = 0;
        reader.event = Some(event);
        index
    };

    // SAFETY: the node ABI supplies a writable output word.
    unsafe { out.write(index) };
    ddk::OK
}

/// Releases the reader slot and revokes its event.
unsafe extern "C" fn close(context: *mut c_void, index: usize, _: u32) {
    // SAFETY: devfs passes the publishing endpoint as the context.
    let endpoint = unsafe { endpoint(context) };
    let event = {
        endpoint
            .state
            .lock_irqsave()
            .readers
            .get_mut(index)
            .and_then(|r| r.event.take())
    };
    drop(event);
}

/// Copies whole records, waiting for the reader's event when the queue is empty.
unsafe extern "C" fn read(
    context: *mut c_void,
    index: usize,
    _: u64,
    out: *mut u8,
    length: usize,
    flags: u32,
) -> i64 {
    if length == 0 {
        return 0;
    }

    if length < RECORD || index >= READERS {
        return i64::from(ddk::EINVAL);
    }

    // SAFETY: devfs passes the publishing endpoint as the context.
    let endpoint = unsafe { endpoint(context) };

    // SAFETY: devfs supplies an exclusive, initialized kernel buffer.
    let output = match unsafe { ddk::output_bytes(out, length) } {
        Ok(output) => output,
        Err(error) => return i64::from(error.status()),
    };

    loop {
        let event = {
            let mut state = endpoint.state.lock_irqsave();
            if !state.online {
                return i64::from(ddk::ENODEV);
            }

            let reader = &mut state.readers[index];
            if reader.revoked {
                return i64::from(ddk::ENODEV);
            }

            let Some(event) = reader.event.as_ref().map(OwnedEvent::event) else {
                return i64::from(ddk::EINVAL);
            };

            if reader.length != 0 {
                let bytes = reader.read_records(output);
                if reader.length == 0 {
                    let _ = event.reset();
                }

                return i64::from(bytes as u32);
            }

            event
        };

        if flags & ddk::OPEN_NONBLOCK != 0 {
            return i64::from(ddk::EAGAIN);
        }

        if let Err(error) = event.wait() {
            return i64::from(error.status());
        }
    }
}

/// Reports requested readiness flags and disconnection for the reader.
unsafe extern "C" fn poll(
    context: *mut c_void,
    index: usize,
    _: u64,
    requested: u16,
    _: u32,
) -> i64 {
    // SAFETY: devfs passes the publishing endpoint as the context.
    let state = unsafe { endpoint(context) }.state.lock_irqsave();
    let reader = state.readers.get(index);

    if !state.online || reader.is_some_and(|reader| reader.revoked) {
        return i64::from(ddk::POLL_HUP);
    }

    if reader.is_some_and(|reader| reader.length != 0) {
        i64::from(requested & (ddk::POLL_IN | ddk::POLL_RDNORM))
    } else {
        0
    }
}

/// Returns the event ID devfs waits on for read or hangup readiness.
unsafe extern "C" fn reader_event(context: *mut c_void, index: usize) -> usize {
    // SAFETY: devfs passes the publishing endpoint as the context.
    unsafe { endpoint(context) }
        .state
        .lock_irqsave()
        .readers
        .get(index)
        .and_then(|r| r.event.as_ref().map(OwnedEvent::event))
        .map_or(0, Event::id)
}

/// Handles evdev identity, capability, key state, and reader-management requests.
unsafe extern "C" fn ioctl(
    context: *mut c_void,
    index: usize,
    _: *const raw::IoctlIdentity,
    request: u64,
    value: u64,
    buffer: *mut u8,
    length: usize,
) -> i64 {
    if (request >> 8) & 0xff != u64::from(b'E') {
        return i64::from(ddk::ENOTTY);
    }

    // SAFETY: devfs passes the publishing endpoint as the context.
    let endpoint = unsafe { endpoint(context) };
    let mut state = endpoint.state.lock_irqsave();
    if !state.online {
        return i64::from(ddk::ENODEV);
    }

    let Some(reader) = state.readers.get_mut(index) else {
        return i64::from(ddk::EINVAL);
    };

    if reader.revoked {
        return i64::from(ddk::ENODEV);
    }

    if request == EVIOCREVOKE {
        // EVIOCREVOKE takes an immediate argument.
        if value != 0 {
            return i64::from(ddk::EINVAL);
        }
        reader.revoked = true;
        reader.length = 0;
        if let Some(event) = &reader.event {
            let _ = event.signal();
        }

        return 0;
    }

    // SAFETY: the node ABI provides length bytes of initialized kernel storage.
    let output = match unsafe { ddk::output_bytes(buffer, length) } {
        Ok(output) => output,
        Err(error) => return i64::from(error.status()),
    };
    output.fill(0);
    let nr = request & 0xff;
    let mut types = [(1 << EV_SYN) | (1 << EV_KEY)];
    if state.relative != [0; 2] {
        types[0] |= 1 << EV_REL;
    }

    let source: &[u8] = match nr {
        1 if length == 4 => {
            output.copy_from_slice(&0x0001_0001_u32.to_ne_bytes());
            return 0;
        }
        2 if length == 8 => {
            output[..2].copy_from_slice(&endpoint.bus.to_ne_bytes());
            return 0;
        }
        6 => endpoint.name.to_bytes_with_nul(),
        0x18 => &state.pressed,
        0x20 => &types,
        0x21 => &state.keys,
        0x22 => &state.relative,
        0x20..=0x3f | 0x09 | 0x19 => &[],
        _ => return i64::from(ddk::ENOTTY),
    };

    let count = source.len().min(length);
    output[..count].copy_from_slice(&source[..count]);
    i64::from(count as u32)
}

const OPS: raw::NodeOps = raw::NodeOps {
    size: raw::NODE_OPS_SIZE,
    context: ptr::null_mut(),
    open: Some(open),
    close: Some(close),
    initial_offset: None,
    read: Some(read),
    write: None,
    size_bytes: None,
    sync: None,
    poll: Some(poll),
    ioctl: Some(ioctl),
    readable_event: Some(reader_event),
    writable_event: None,
    hangup_event: Some(reader_event),
    terminal_state: None,
};
