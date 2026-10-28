#![no_std]
#![allow(unsafe_code, clippy::cast_possible_truncation, clippy::cast_sign_loss)]
//! Evdev keyboard queues shared by hardware input drivers.

use core::{
    ffi::{CStr, c_void},
    ptr, slice,
};
use ddk::{Error, Event, OwnedEvent, Result, TicketLock, devfs, raw};

const READERS: usize = 8;
const QUEUE: usize = 256;
const RECORD: usize = 24;
pub const KEY_BYTES: usize = 96;

struct Reader {
    event: Option<OwnedEvent>,
    head: usize,
    length: usize,
    records: [[u8; RECORD]; QUEUE],
}
impl Reader {
    const fn new() -> Self {
        Self {
            event: None,
            head: 0,
            length: 0,
            records: [[0; RECORD]; QUEUE],
        }
    }
    fn push(&mut self, record: [u8; RECORD]) {
        if self.length == QUEUE {
            self.head = 0;
            self.length = 1;
            self.records[0] = event_record(0, 3, 0); // SYN_DROPPED
        }
        self.records[(self.head + self.length) % QUEUE] = record;
        self.length += 1;
        if let Some(event) = &self.event {
            let _ = event.signal();
        }
    }
}
struct State {
    readers: [Reader; READERS],
    pressed: [u8; KEY_BYTES],
    keys: [u8; KEY_BYTES],
    online: bool,
    node: Option<u64>,
}

/// The callback address must stay stable until unpublish succeeds and all opens
/// have closed. Static instances also safely retain disconnected reader state.
pub struct Keyboard {
    state: TicketLock<State>,
    name: &'static CStr,
    bus: u16,
}
impl Keyboard {
    #[must_use]
    #[allow(clippy::large_stack_arrays)] // Evaluated in static keyboard initializers.
    pub const fn new(name: &'static CStr, bus: u16) -> Self {
        Self {
            name,
            bus,
            state: TicketLock::new(State {
                readers: [const { Reader::new() }; READERS],
                pressed: [0; KEY_BYTES],
                keys: [0; KEY_BYTES],
                online: false,
                node: None,
            }),
        }
    }

    /// Publish the keyboard under the first available event name.
    ///
    /// # Errors
    /// Returns a devfs error, including busy readers from an earlier attachment.
    #[allow(clippy::missing_panics_doc)] // Generated event names always contain one trailing NUL.
    pub fn publish(&'static self, keys: [u8; KEY_BYTES]) -> Result<()> {
        if self.state.lock_irqsave().node.is_some() {
            self.unpublish()?;
        }
        let directory = devfs::lookup(c"input")?;
        let mut ops = OPS;
        ops.context = ptr::from_ref(self).cast_mut().cast();
        {
            let mut state = self.state.lock_irqsave();
            state.keys = keys;
            state.pressed.fill(0);
            state.online = true;
        }
        let node = (0..64)
            .find_map(|index| {
                let mut name = *b"event00\0";
                name[5] = b'0' + index / 10;
                name[6] = b'0' + index % 10;
                // Single-digit names use event0, matching Linux conventions.
                let name = if index < 10 {
                    name[5] = name[6];
                    name[6] = 0;
                    CStr::from_bytes_with_nul(&name[..7]).expect("event name")
                } else {
                    CStr::from_bytes_with_nul(&name).expect("event name")
                };
                // SAFETY: this static keyboard outlives the node and its callbacks.
                match unsafe { devfs::create_character(directory, name, 0o600, &ops) } {
                    Err(error) if error.status() == ddk::EEXIST => None,
                    result => Some(result),
                }
            })
            .ok_or(Error::from_status(ddk::ENOSPC))
            .and_then(core::convert::identity);
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

    /// Disconnect readers and remove the device node.
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

    pub fn event(&self, kind: u16, code: u16, mut value: i32) {
        let mut state = self.state.lock_irqsave();
        if !state.online {
            return;
        }
        if kind == 1 {
            let index = usize::from(code);
            if index >= KEY_BYTES * 8 || !(0..=2).contains(&value) {
                return;
            }
            let mask = 1 << (index % 8);
            if state.keys[index / 8] & mask == 0 {
                return;
            }
            if value == 0 {
                state.pressed[index / 8] &= !mask;
            } else {
                if state.pressed[index / 8] & mask != 0 {
                    value = 2;
                }
                state.pressed[index / 8] |= mask;
            }
        } else if kind != 0 {
            return;
        }
        let record = event_record(kind, code, value);
        for reader in &mut state.readers {
            if reader.event.is_some() {
                reader.push(record);
            }
        }
    }

    pub fn key(&self, code: u16, down: bool) {
        self.event(1, code, i32::from(down));
        self.event(0, 0, 0);
    }
}

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

fn keyboard<'a>(context: *mut c_void) -> &'a Keyboard {
    // SAFETY: NodeOps.context always points to the static publishing keyboard.
    unsafe { &*context.cast::<Keyboard>() }
}
unsafe extern "C" fn open(context: *mut c_void, _: u32, out: *mut usize) -> i32 {
    let mut event = Some(match OwnedEvent::create() {
        Ok(event) => event,
        Err(error) => return error.status(),
    });
    let index = {
        let mut state = keyboard(context).state.lock_irqsave();
        if !state.online {
            Err(ddk::ENODEV)
        } else if let Some(index) = state
            .readers
            .iter()
            .position(|reader| reader.event.is_none())
        {
            let reader = &mut state.readers[index];
            reader.head = 0;
            reader.length = 0;
            reader.event = event.take();
            Ok(index)
        } else {
            Err(ddk::ENOSPC)
        }
    };
    match index {
        Ok(index) => {
            // SAFETY: the node ABI supplies a writable output word.
            unsafe {
                out.write(index);
            }
            ddk::OK
        }
        Err(error) => error,
    }
}
unsafe extern "C" fn close(context: *mut c_void, index: usize, _: u32) {
    let event = {
        keyboard(context)
            .state
            .lock_irqsave()
            .readers
            .get_mut(index)
            .and_then(|r| r.event.take())
    };
    drop(event);
}
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
    loop {
        let event = {
            let mut state = keyboard(context).state.lock_irqsave();
            if !state.online {
                return i64::from(ddk::ENODEV);
            }
            let reader = &mut state.readers[index];
            let Some(event) = reader.event.as_ref().map(OwnedEvent::event) else {
                return i64::from(ddk::EINVAL);
            };
            let count = reader.length.min(length / RECORD);
            if count != 0 {
                for n in 0..count {
                    // SAFETY: each record fits in the ABI's kernel output buffer.
                    unsafe {
                        ptr::copy_nonoverlapping(
                            reader.records[reader.head].as_ptr(),
                            out.add(n * RECORD),
                            RECORD,
                        );
                    }
                    reader.head = (reader.head + 1) % QUEUE;
                    reader.length -= 1;
                }
                if reader.length == 0 {
                    let _ = event.reset();
                }
                return i64::from((count * RECORD) as u32);
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
unsafe extern "C" fn poll(
    context: *mut c_void,
    index: usize,
    _: u64,
    requested: u16,
    _: u32,
) -> i64 {
    let state = keyboard(context).state.lock_irqsave();
    if !state.online {
        return i64::from(ddk::POLL_HUP);
    }
    if state.readers.get(index).is_some_and(|r| r.length != 0) {
        i64::from(requested & (ddk::POLL_IN | ddk::POLL_RDNORM))
    } else {
        0
    }
}
unsafe extern "C" fn readable(context: *mut c_void, index: usize) -> usize {
    keyboard(context)
        .state
        .lock_irqsave()
        .readers
        .get(index)
        .and_then(|r| r.event.as_ref().map(OwnedEvent::event))
        .map_or(0, Event::id)
}
unsafe extern "C" fn ioctl(
    context: *mut c_void,
    _: usize,
    _: *const raw::IoctlIdentity,
    request: u64,
    _: u64,
    buffer: *mut u8,
    length: usize,
) -> i64 {
    if (request >> 8) & 0xff != u64::from(b'E') {
        return i64::from(ddk::ENOTTY);
    }
    let keyboard = keyboard(context);
    let state = keyboard.state.lock_irqsave();
    if !state.online {
        return i64::from(ddk::ENODEV);
    }
    // SAFETY: the node ABI provides length bytes of initialized kernel storage.
    let output = unsafe { slice::from_raw_parts_mut(buffer, length) };
    output.fill(0);
    let nr = request & 0xff;
    let source: &[u8] = match nr {
        1 if length == 4 => {
            output.copy_from_slice(&0x0001_0001_u32.to_ne_bytes());
            return 0;
        }
        2 if length == 8 => {
            output[..2].copy_from_slice(&keyboard.bus.to_ne_bytes());
            return 0;
        }
        6 => keyboard.name.to_bytes_with_nul(),
        0x18 => &state.pressed,
        0x20 => &[3], // EV_SYN | EV_KEY; repeat is generated by kmscon.
        0x21 => &state.keys,
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
    readable_event: Some(readable),
    writable_event: None,
    hangup_event: Some(readable),
    terminal_state: None,
};
