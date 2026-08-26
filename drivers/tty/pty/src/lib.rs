#![no_std]
#![allow(unsafe_code)]
// Values crossing the module boundary carry the C ABI widths fixed by
// include/roanix/api.h, and both supported targets use 64-bit pointers, so
// casts between them cannot lose information in practice. Large arrays appear
// only inside `const fn` constructors evaluated for statics, never on the
// stack at runtime.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays
)]

//! `/dev/ptmx` and `/dev/pts/*` pseudo terminals with fixed, lock-protected
//! 64 KiB rings in each direction.

use core::{
    ffi::{CStr, c_void},
    ptr,
    ptr::NonNull,
    slice,
};
use ddk::{Devfs, Event, Module, Result, TicketLock, Tty, raw};

const SLOTS: usize = 64;
const CAPACITY: usize = 64 * 1024;
const STORAGE: usize = CAPACITY * 2;
const ALIGN: usize = 64;
const TIOCGPTN: u64 = 0x8004_5430;
const TIOCSPTLCK: u64 = 0x4004_5431;
const FIONREAD: u64 = 0x541b;
const FREE: u8 = 0;
const RESERVED: u8 = 1;
const ACTIVE: u8 = 2;
#[derive(Clone, Copy)]
struct Ring {
    head: usize,
    length: usize,
}
impl Ring {
    const fn new() -> Self {
        Self { head: 0, length: 0 }
    }
    fn available(self) -> usize {
        CAPACITY - self.length
    }
    fn clear(&mut self) {
        self.head = 0;
        self.length = 0;
    }
}
struct Slot {
    state: u8,
    index: u32,
    slave_opens: u32,
    master_open: bool,
    slave_seen: bool,
    locked: bool,
    storage: Option<NonNull<u8>>,
    to_master: Ring,
    to_slave: Ring,
    master_state: Option<Event>,
    master_readable: Option<Event>,
    master_writable: Option<Event>,
    slave_readable: Option<Event>,
    slave_writable: Option<Event>,
    tty: Option<Tty>,
    ops: raw::ConsoleOps,
}
impl Slot {
    const fn new() -> Self {
        Self {
            state: FREE,
            index: 0,
            slave_opens: 0,
            master_open: false,
            slave_seen: false,
            locked: true,
            storage: None,
            to_master: Ring::new(),
            to_slave: Ring::new(),
            master_state: None,
            master_readable: None,
            master_writable: None,
            slave_readable: None,
            slave_writable: None,
            tty: None,
            ops: raw::ConsoleOps {
                size: raw::CONSOLE_OPS_SIZE,
                flags: ddk::CONSOLE_RESET_ON_LAST_CLOSE,
                context: ptr::null_mut(),
                open: Some(slave_open),
                close: Some(slave_close),
                try_read: Some(slave_try_read),
                read: Some(slave_read),
                write: Some(slave_write),
                configure: None,
                flush: None,
                flush_input: Some(slave_flush_input),
                flush_output: Some(slave_flush_output),
                send_break: None,
                writable: Some(slave_writable),
                hung_up: Some(slave_hung_up),
                queued_output: Some(slave_queued),
                destroy: None,
                readable_event: 0,
                writable_event: 0,
                hangup_event: 0,
            },
        }
    }
}
// SAFETY: every field, including the allocation receipt, is accessed only
// while the enclosing `TicketLock` is held with local IRQs disabled.
unsafe impl Send for Slot {}
static SLOTS_STATE: [TicketLock<Slot>; SLOTS] = [const { TicketLock::new(Slot::new()) }; SLOTS];

fn slot(pointer: *mut c_void) -> Option<&'static TicketLock<Slot>> {
    let first = SLOTS_STATE.as_ptr() as usize;
    let end = first + core::mem::size_of_val(&SLOTS_STATE);
    let value = pointer as usize;
    if value < first
        || value >= end
        || !(value - first).is_multiple_of(core::mem::size_of::<TicketLock<Slot>>())
    {
        None
    } else {
        // SAFETY: the range and alignment checks identify one static slot.
        Some(unsafe { &*(pointer.cast()) })
    }
}
fn signal(event: Option<Event>) {
    if let Some(event) = event {
        let _ = event.signal();
    }
}
fn reset(event: Option<Event>) {
    if let Some(event) = event {
        let _ = event.reset();
    }
}
fn read_ring(slot: &mut Slot, master: bool, out: &mut [u8]) -> usize {
    let ring = if master {
        &mut slot.to_master
    } else {
        &mut slot.to_slave
    };
    let Some(base) = slot.storage else { return 0 };
    let offset = if master { 0 } else { CAPACITY };
    let count = out.len().min(ring.length);
    let first = count.min(CAPACITY - ring.head);
    // SAFETY: the slot owns two contiguous `CAPACITY`-byte rings in this
    // allocation, and the lock gives this call exclusive access to them.
    let storage = unsafe { slice::from_raw_parts(base.as_ptr().add(offset), CAPACITY) };
    out[..first].copy_from_slice(&storage[ring.head..ring.head + first]);
    if count > first {
        out[first..count].copy_from_slice(&storage[..count - first]);
    }
    ring.head = (ring.head + count) % CAPACITY;
    ring.length -= count;
    count
}
fn write_ring(slot: &mut Slot, master: bool, input: &[u8]) -> usize {
    let ring = if master {
        &mut slot.to_master
    } else {
        &mut slot.to_slave
    };
    let Some(base) = slot.storage else { return 0 };
    let offset = if master { 0 } else { CAPACITY };
    let count = input.len().min(ring.available());
    let tail = (ring.head + ring.length) % CAPACITY;
    let first = count.min(CAPACITY - tail);
    // SAFETY: the slot owns two contiguous `CAPACITY`-byte rings in this
    // allocation, and the lock gives this call exclusive access to them.
    let storage = unsafe { slice::from_raw_parts_mut(base.as_ptr().add(offset), CAPACITY) };
    storage[tail..tail + first].copy_from_slice(&input[..first]);
    if count > first {
        storage[..count - first].copy_from_slice(&input[first..count]);
    }
    ring.length += count;
    count
}
fn cleanup(slot: &mut Slot) -> Option<NonNull<u8>> {
    if slot.state != ACTIVE || slot.master_open || slot.slave_opens != 0 {
        return None;
    }
    let storage = slot.storage.take();
    slot.state = FREE;
    slot.locked = true;
    slot.to_master = Ring::new();
    slot.to_slave = Ring::new();
    storage
}
fn release(storage: Option<NonNull<u8>>) {
    if let Some(storage) = storage {
        // SAFETY: the slot uniquely owns this allocation and cleanup consumes
        // it with the layout used by `alloc_zeroed`.
        unsafe { ddk::free(storage, STORAGE, ALIGN) }
    }
}

unsafe extern "C" fn master_open(_: *mut c_void, _: u32, out: *mut usize) -> i32 {
    if out.is_null() {
        return ddk::EINVAL;
    }
    for (index, lock) in SLOTS_STATE.iter().enumerate() {
        let reserved = {
            let mut slot = lock.lock_irqsave();
            if slot.state == FREE {
                slot.state = RESERVED;
                true
            } else {
                false
            }
        };
        if !reserved {
            continue;
        }
        let memory = match ddk::alloc_zeroed(STORAGE, ALIGN) {
            Ok(m) => m,
            Err(e) => {
                lock.lock_irqsave().state = FREE;
                return e.status();
            }
        };
        let mut slot = lock.lock_irqsave();
        if slot.state != RESERVED {
            drop(slot);
            release(Some(memory));
            return ddk::ENOMEM;
        }
        slot.state = ACTIVE;
        slot.index = index as u32;
        slot.master_open = true;
        slot.slave_seen = false;
        slot.slave_opens = 0;
        slot.locked = true;
        slot.storage = Some(memory);
        slot.to_master = Ring::new();
        slot.to_slave = Ring::new();
        reset(slot.master_state);
        reset(slot.master_readable);
        reset(slot.slave_readable);
        signal(slot.master_writable);
        signal(slot.slave_writable);
        // SAFETY: `out` was validated above and points to writable ABI output.
        unsafe { *out = core::ptr::from_ref(lock) as usize };
        return ddk::OK;
    }
    ddk::ENOSPC
}
unsafe extern "C" fn master_close(_: *mut c_void, file: usize, _: u32) {
    let Some(lock) = slot(file as *mut c_void) else {
        return;
    };
    let storage = {
        let mut state = lock.lock_irqsave();
        if state.state == ACTIVE && state.master_open {
            state.master_open = false;
            signal(state.master_writable);
            signal(state.slave_readable);
            signal(state.slave_writable);
            cleanup(&mut state)
        } else {
            None
        }
    };
    release(storage);
}
unsafe extern "C" fn master_read(
    _: *mut c_void,
    file: usize,
    _: u64,
    data: *mut u8,
    length: usize,
    flags: u32,
) -> i64 {
    if data.is_null() && length != 0 {
        return ddk::EINVAL.into();
    }
    if length == 0 {
        return 0;
    }
    let Some(lock) = slot(file as *mut c_void) else {
        return ddk::EINVAL.into();
    };
    // SAFETY: the non-null node buffer is writable for `length` bytes.
    let out = unsafe { slice::from_raw_parts_mut(data, length) };
    loop {
        let (mut wait, mut writable) = (None, None);
        let answer = {
            let mut state = lock.lock_irqsave();
            if state.state != ACTIVE || !state.master_open {
                return ddk::EIO.into();
            }
            if state.to_master.length != 0 {
                let count = read_ring(&mut state, true, out);
                writable = state.master_writable;
                if state.to_master.length == 0 {
                    reset(state.master_readable);
                }
                Some(count as i64)
            } else if state.slave_opens == 0 {
                if state.slave_seen {
                    return ddk::EIO.into();
                }
                if flags & ddk::OPEN_NONBLOCK != 0 {
                    return ddk::EAGAIN.into();
                }
                wait = state.master_state;
                reset(wait);
                None
            } else if flags & ddk::OPEN_NONBLOCK != 0 {
                return ddk::EAGAIN.into();
            } else {
                wait = state.master_readable;
                reset(wait);
                None
            }
        };
        if let Some(value) = answer {
            signal(writable);
            return value;
        }
        let _ = wait.map(Event::wait);
    }
}
unsafe extern "C" fn master_write(
    _: *mut c_void,
    file: usize,
    _: u64,
    data: *const u8,
    length: usize,
    flags: u32,
) -> i64 {
    if data.is_null() && length != 0 {
        return ddk::EINVAL.into();
    }
    if length == 0 {
        return 0;
    }
    let Some(lock) = slot(file as *mut c_void) else {
        return ddk::EINVAL.into();
    };
    // SAFETY: the non-null node buffer is readable for `length` bytes.
    let input = unsafe { slice::from_raw_parts(data, length) };
    loop {
        let mut wait = None;
        let answer = {
            let mut state = lock.lock_irqsave();
            if state.state != ACTIVE || !state.master_open {
                return ddk::EIO.into();
            }
            if state.slave_opens == 0 {
                if state.slave_seen {
                    return ddk::EIO.into();
                }
                if flags & ddk::OPEN_NONBLOCK != 0 {
                    return ddk::EAGAIN.into();
                }
                wait = state.master_state;
                reset(wait);
                None
            } else if state.to_slave.available() != 0 {
                let count = write_ring(&mut state, false, input);
                if state.to_slave.available() == 0 {
                    reset(state.slave_writable);
                }
                signal(state.slave_readable);
                Some(count as i64)
            } else if flags & ddk::OPEN_NONBLOCK != 0 {
                return ddk::EAGAIN.into();
            } else {
                wait = state.slave_writable;
                reset(wait);
                None
            }
        };
        if let Some(value) = answer {
            return value;
        }
        let _ = wait.map(Event::wait);
    }
}
unsafe extern "C" fn master_poll(_: *mut c_void, file: usize, _: u64, events: u16, _: u32) -> i64 {
    let Some(lock) = slot(file as *mut c_void) else {
        return ddk::POLL_NVAL.into();
    };
    let state = lock.lock_irqsave();
    let mut ready = 0;
    if state.state != ACTIVE || !state.master_open {
        ready = ddk::POLL_HUP;
    } else {
        if state.to_master.length != 0 {
            ready |= events & (ddk::POLL_IN | ddk::POLL_RDNORM);
        }
        if state.slave_seen && state.slave_opens == 0 {
            ready |= ddk::POLL_HUP;
        } else if state.slave_opens != 0 && state.to_slave.available() != 0 {
            ready |= events & (ddk::POLL_OUT | ddk::POLL_WRNORM);
        }
    }
    ready.into()
}
unsafe extern "C" fn master_ioctl(
    _: *mut c_void,
    file: usize,
    _: *const raw::IoctlIdentity,
    request: u64,
    _: u64,
    argument: *mut u8,
    length: usize,
) -> i64 {
    if argument.is_null() || length != 4 {
        return ddk::EINVAL.into();
    }
    let Some(lock) = slot(file as *mut c_void) else {
        return ddk::EINVAL.into();
    };
    match request {
        TIOCGPTN => {
            let index = lock.lock_irqsave().index;
            // SAFETY: the ioctl buffer is writable for exactly four bytes.
            unsafe { ptr::copy_nonoverlapping((&raw const index).cast(), argument, 4) }
            ddk::OK.into()
        }
        TIOCSPTLCK => {
            let mut value = 0i32;
            // SAFETY: the ioctl buffer is readable for exactly four bytes.
            unsafe { ptr::copy_nonoverlapping(argument, (&raw mut value).cast(), 4) }
            let mut state = lock.lock_irqsave();
            if state.state != ACTIVE || !state.master_open {
                return ddk::EIO.into();
            }
            state.locked = value != 0;
            ddk::OK.into()
        }
        FIONREAD => {
            let count = lock.lock_irqsave().to_master.length.min(i32::MAX as usize) as i32;
            // SAFETY: the ioctl buffer is writable for exactly four bytes.
            unsafe { ptr::copy_nonoverlapping((&raw const count).cast(), argument, 4) }
            ddk::OK.into()
        }
        _ => ddk::ENOTTY.into(),
    }
}
unsafe extern "C" fn master_readable(_: *mut c_void, file: usize) -> usize {
    slot(file as *mut c_void)
        .and_then(|lock| lock.lock_irqsave().master_readable)
        .map_or(0, Event::id)
}
unsafe extern "C" fn master_writable(_: *mut c_void, file: usize) -> usize {
    slot(file as *mut c_void)
        .and_then(|lock| lock.lock_irqsave().slave_writable)
        .map_or(0, Event::id)
}
unsafe extern "C" fn master_hangup(context: *mut c_void, file: usize) -> usize {
    // SAFETY: this forwards the same node context and file receipt.
    unsafe { master_readable(context, file) }
}
static MASTER_OPS: raw::NodeOps = raw::NodeOps {
    size: raw::NODE_OPS_SIZE,
    context: ptr::null_mut(),
    open: Some(master_open),
    close: Some(master_close),
    initial_offset: None,
    read: Some(master_read),
    write: Some(master_write),
    size_bytes: None,
    sync: None,
    poll: Some(master_poll),
    ioctl: Some(master_ioctl),
    readable_event: Some(master_readable),
    writable_event: Some(master_writable),
    hangup_event: Some(master_hangup),
    terminal_state: None,
};

unsafe extern "C" fn slave_open(context: *mut c_void) -> i32 {
    let Some(lock) = slot(context) else {
        return ddk::EINVAL;
    };
    let mut state = lock.lock_irqsave();
    if state.state != ACTIVE || !state.master_open || state.locked {
        return ddk::EIO;
    }
    state.slave_seen = true;
    state.slave_opens += 1;
    if state.to_master.length == 0 {
        reset(state.master_readable);
    }
    signal(state.master_state);
    ddk::OK
}
unsafe extern "C" fn slave_close(context: *mut c_void) {
    let Some(lock) = slot(context) else { return };
    let storage = {
        let mut state = lock.lock_irqsave();
        if state.state != ACTIVE || state.slave_opens == 0 {
            return;
        }
        state.slave_opens -= 1;
        if state.slave_opens == 0 {
            state.to_slave.clear();
            reset(state.slave_readable);
        }
        signal(state.master_state);
        signal(state.master_readable);
        signal(state.slave_writable);
        cleanup(&mut state)
    };
    release(storage);
}
unsafe extern "C" fn slave_read(context: *mut c_void, data: *mut u8, length: usize) -> i64 {
    if data.is_null() && length != 0 {
        return ddk::EINVAL.into();
    }
    let Some(lock) = slot(context) else {
        return ddk::EINVAL.into();
    };
    if length == 0 {
        return 0;
    }
    // SAFETY: the non-null console buffer is writable for `length` bytes.
    let out = unsafe { slice::from_raw_parts_mut(data, length) };
    let (written, event) = {
        let mut state = lock.lock_irqsave();
        if state.state == ACTIVE && state.to_slave.length != 0 {
            let n = read_ring(&mut state, false, out);
            if state.to_slave.length == 0 {
                reset(state.slave_readable);
            }
            (n, state.slave_writable)
        } else {
            (0, None)
        }
    };
    signal(event);
    written as i64
}
unsafe extern "C" fn slave_try_read(context: *mut c_void, data: *mut u8) -> i32 {
    if data.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: the caller supplies one writable byte and `slave_read` preserves
    // that callback contract.
    i32::from(unsafe { slave_read(context, data, 1) } == 1)
}
unsafe extern "C" fn slave_write(
    context: *mut c_void,
    data: *const u8,
    length: usize,
    nonblocking: u8,
) -> i32 {
    if data.is_null() && length != 0 {
        return ddk::EINVAL;
    }
    if length == 0 {
        return ddk::OK;
    }
    let Some(lock) = slot(context) else {
        return ddk::EINVAL;
    };
    // SAFETY: the non-null console buffer is readable for `length` bytes.
    let input = unsafe { slice::from_raw_parts(data, length) };
    let mut offset = 0;
    while offset < length {
        let mut wait = None;
        let progress = {
            let mut state = lock.lock_irqsave();
            if state.state != ACTIVE || !state.master_open {
                return ddk::EIO;
            }
            if nonblocking != 0 && state.to_master.available() < length - offset {
                return ddk::EAGAIN;
            }
            if state.to_master.available() != 0 {
                let n = write_ring(&mut state, true, &input[offset..]);
                if state.to_master.available() == 0 {
                    reset(state.master_writable);
                }
                signal(state.master_readable);
                Some(n)
            } else {
                wait = state.master_writable;
                reset(wait);
                None
            }
        };
        if let Some(n) = progress {
            offset += n;
        } else {
            let _ = wait.map(Event::wait);
        }
    }
    ddk::OK
}
unsafe extern "C" fn slave_hung_up(context: *mut c_void) -> i32 {
    slot(context).map_or(1, |lock| {
        i32::from({
            let state = lock.lock_irqsave();
            state.state != ACTIVE || !state.master_open
        })
    })
}
unsafe extern "C" fn slave_writable(context: *mut c_void) -> i32 {
    slot(context).map_or(0, |lock| {
        i32::from({
            let state = lock.lock_irqsave();
            state.state == ACTIVE && state.master_open && state.to_master.available() != 0
        })
    })
}
unsafe extern "C" fn slave_flush_input(context: *mut c_void) -> i32 {
    let Some(lock) = slot(context) else {
        return ddk::EINVAL;
    };
    let event = {
        let mut state = lock.lock_irqsave();
        if state.state == ACTIVE {
            state.to_slave.clear();
            reset(state.slave_readable);
            state.slave_writable
        } else {
            None
        }
    };
    signal(event);
    ddk::OK
}
unsafe extern "C" fn slave_flush_output(context: *mut c_void) -> i32 {
    let Some(lock) = slot(context) else {
        return ddk::EINVAL;
    };
    let event = {
        let mut state = lock.lock_irqsave();
        if state.state == ACTIVE {
            state.to_master.clear();
            if state.slave_opens != 0 {
                reset(state.master_readable);
            }
            state.master_writable
        } else {
            None
        }
    };
    signal(event);
    ddk::OK
}
unsafe extern "C" fn slave_queued(context: *mut c_void) -> i64 {
    slot(context).map_or(0, |lock| {
        let state = lock.lock_irqsave();
        if state.state == ACTIVE {
            state.to_master.length as i64
        } else {
            0
        }
    })
}

fn decimal(index: u32, buffer: &mut [u8; 11]) -> &CStr {
    let mut digits = [0u8; 10];
    let mut n = 0;
    let mut value = index;
    loop {
        digits[n] = b'0' + (value % 10) as u8;
        n += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    for i in 0..n {
        buffer[i] = digits[n - i - 1];
    }
    buffer[n] = 0;
    // SAFETY: the loop wrote only decimal digits followed by one terminator.
    unsafe { CStr::from_bytes_with_nul_unchecked(&buffer[..=n]) }
}
fn init(_: Module) -> Result<()> {
    let devfs = Devfs::current()?;
    let root = devfs.root()?;
    let pts = devfs.mkdir(root, c"pts", 0o755)?;
    for (index, lock) in SLOTS_STATE.iter().enumerate() {
        let events = (
            Event::create()?,
            Event::create()?,
            Event::create()?,
            Event::create()?,
            Event::create()?,
        );
        let mut state = lock.lock_irqsave();
        state.index = index as u32;
        state.state = FREE;
        state.locked = true;
        state.master_state = Some(events.0);
        state.master_readable = Some(events.1);
        state.master_writable = Some(events.2);
        state.slave_readable = Some(events.3);
        state.slave_writable = Some(events.4);
        state.ops.context = core::ptr::from_ref(lock) as *mut c_void;
        state.ops.readable_event = events.3.id();
        state.ops.writable_event = events.2.id();
        state.ops.hangup_event = events.3.id();
        let mut text = [0; 11];
        // SAFETY: this static slot retains the operation table until terminal
        // removal during module teardown.
        state.tty = Some(unsafe {
            Tty::register(
                None,
                pts,
                decimal(index as u32, &mut text),
                0o620,
                38400,
                &state.ops,
            )
        }?);
    }
    // SAFETY: `MASTER_OPS` is static and validates all slot receipts.
    unsafe {
        let _ = devfs.create_character(root, c"ptmx", 0o666, &MASTER_OPS)?;
    }
    Ok(())
}
fn exit(_: Module) {
    for lock in &SLOTS_STATE {
        let (storage, events, tty) = {
            let mut state = lock.lock_irqsave();
            let storage = state.storage.take();
            let events = [
                state.master_state.take(),
                state.master_readable.take(),
                state.master_writable.take(),
                state.slave_readable.take(),
                state.slave_writable.take(),
            ];
            let tty = state.tty.take();
            *state = Slot::new();
            (storage, events, tty)
        };
        if let Some(mut tty) = tty {
            let _ = tty.unregister();
        }
        for event in events.into_iter().flatten() {
            let _ = event.destroy();
        }
        release(storage);
    }
}
ddk::module!(b"pty\0", b"Pseudo-terminal (ptmx/pts) driver\0", init, exit);
