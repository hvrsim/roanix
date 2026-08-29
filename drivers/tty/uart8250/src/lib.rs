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

//! Interrupt-driven 8250/16550 UARTs.

use core::{
    ffi::{CStr, c_void},
    ptr, slice,
};
use ddk::{
    Bus, Device, DriverRegistration, Event, Irq, Mmio, Module, Result, TicketLock, Tty, raw,
};

const DATA: u32 = 0;
const IER: u32 = 1;
const IID: u32 = 2;
const FCR: u32 = 2;
const LCR: u32 = 3;
const MCR: u32 = 4;
const LSR: u32 = 5;
const MSR: u32 = 6;
const SCR: u32 = 7;
const IER_RX: u8 = 5;
const IER_TX: u8 = 2;
const IER_UUE: u8 = 1 << 6;
const LSR_DR: u8 = 1;
const LSR_THRE: u8 = 1 << 5;
const LCR_DLAB: u8 = 0x80;
const LCR_BREAK: u8 = 1 << 6;
const FIFO: usize = 16;
const RX: usize = 8192;
const TX: usize = 64 * 1024;
const MAX_PORTS: usize = 8;
const MATCH_PXA: usize = 1;
const PXA_CLOCK: u64 = 14_745_600;
const WAKE_RX: u32 = 1;
const WAKE_TX: u32 = 2;
const WAKE_DRAIN: u32 = 4;
const WAKE_SERVICED: u32 = 8;

const fn console_ops() -> raw::ConsoleOps {
    raw::ConsoleOps {
        size: raw::CONSOLE_OPS_SIZE,
        flags: 0,
        context: ptr::null_mut(),
        open: None,
        close: None,
        try_read: Some(try_read),
        read: Some(read),
        write: Some(write),
        configure: Some(configure),
        flush: Some(flush),
        flush_input: Some(flush_input),
        flush_output: Some(flush_output),
        send_break: Some(send_break),
        writable: Some(writable),
        hung_up: None,
        queued_output: Some(queued),
        destroy: None,
        readable_event: 0,
        writable_event: 0,
        hangup_event: 0,
    }
}

#[derive(Clone, Copy)]
struct Ring {
    head: usize,
    length: usize,
}
impl Ring {
    const fn new() -> Self {
        Self { head: 0, length: 0 }
    }
    fn available(self, capacity: usize) -> usize {
        capacity - self.length
    }
    fn push(&mut self, storage: &mut [u8], byte: u8) {
        if self.length == storage.len() {
            self.head = (self.head + 1) % storage.len();
            self.length -= 1;
        }
        let tail = (self.head + self.length) % storage.len();
        storage[tail] = byte;
        self.length += 1;
    }
    fn read(&mut self, storage: &mut [u8], output: &mut [u8]) -> usize {
        let count = output.len().min(self.length);
        let first = count.min(storage.len() - self.head);
        output[..first].copy_from_slice(&storage[self.head..self.head + first]);
        if count > first {
            output[first..count].copy_from_slice(&storage[..count - first]);
        }
        self.head = (self.head + count) % storage.len();
        self.length -= count;
        count
    }
    fn write(&mut self, storage: &mut [u8], input: &[u8]) -> usize {
        let count = input.len().min(self.available(storage.len()));
        let tail = (self.head + self.length) % storage.len();
        let first = count.min(storage.len() - tail);
        storage[tail..tail + first].copy_from_slice(&input[..first]);
        if count > first {
            storage[..count - first].copy_from_slice(&input[first..count]);
        }
        self.length += count;
        count
    }
    fn clear(&mut self) {
        self.head = 0;
        self.length = 0;
    }
}

struct Port {
    mmio: Option<Mmio>,
    io: u16,
    memory: bool,
    shift: u32,
    width: u32,
    clock: u32,
    mapped: bool,
    irq: Option<Irq>,
    tty: Option<Tty>,
    ier: u8,
    required_ier: u8,
    character_ns: u64,
    in_flight: usize,
    rx: Ring,
    tx: Ring,
    rx_store: [u8; RX],
    tx_store: [u8; TX],
    readable: Option<Event>,
    writable: Option<Event>,
    drained: Option<Event>,
    ops: raw::ConsoleOps,
}
impl Port {
    const fn new() -> Self {
        Self {
            mmio: None,
            io: 0,
            memory: false,
            shift: 0,
            width: 1,
            clock: 1_843_200,
            mapped: false,
            irq: None,
            tty: None,
            ier: 0,
            required_ier: 0,
            character_ns: 0,
            in_flight: 0,
            rx: Ring::new(),
            tx: Ring::new(),
            rx_store: [0; RX],
            tx_store: [0; TX],
            readable: None,
            writable: None,
            drained: None,
            ops: console_ops(),
        }
    }

    fn reset(&mut self) {
        self.mmio = None;
        self.io = 0;
        self.memory = false;
        self.shift = 0;
        self.width = 1;
        self.clock = 1_843_200;
        self.mapped = false;
        self.irq = None;
        self.tty = None;
        self.ier = 0;
        self.required_ier = 0;
        self.character_ns = 0;
        self.in_flight = 0;
        self.rx = Ring::new();
        self.tx = Ring::new();
        self.readable = None;
        self.writable = None;
        self.drained = None;
        self.ops = console_ops();
    }
    fn reg_read(&self, register: u32) -> u8 {
        if !self.memory {
            return ddk::port_read8(self.io.wrapping_add(register as u16));
        }
        let offset = (register as usize) << self.shift;
        let mmio = self.mmio.as_ref().expect("mapped UART");
        match self.width {
            4 => mmio.read32(offset) as u8,
            2 => mmio.read16(offset) as u8,
            _ => mmio.read8(offset),
        }
    }
    fn reg_write(&self, register: u32, value: u8) {
        if !self.memory {
            ddk::port_write8(self.io.wrapping_add(register as u16), value);
            return;
        }
        let offset = (register as usize) << self.shift;
        let mmio = self.mmio.as_ref().expect("mapped UART");
        match self.width {
            4 => mmio.write32(offset, value.into()),
            2 => mmio.write16(offset, value.into()),
            _ => mmio.write8(offset, value),
        }
    }
    fn set_ier(&mut self, value: u8) {
        let value = value | self.required_ier;
        if self.ier != value {
            self.ier = value;
            self.reg_write(IER, value);
        }
    }
    fn configure(&mut self, framing: &raw::SerialFraming, clear: bool) -> bool {
        let denominator = 16u64 * u64::from(framing.baud);
        if denominator == 0 || denominator > u64::from(self.clock) {
            return false;
        }
        let divisor = (u64::from(self.clock) + denominator / 2) / denominator;
        if divisor == 0 || divisor > u64::from(u16::MAX) {
            return false;
        }
        let mut line = match framing.data_bits {
            5 => 0,
            6 => 1,
            7 => 2,
            8 => 3,
            _ => return false,
        };
        if framing.stop_bits == 2 {
            line |= 4;
        } else if framing.stop_bits != 1 {
            return false;
        }
        if framing.parity != 0 {
            line |= 8;
            if framing.odd_parity == 0 {
                line |= 16;
            }
        }
        let bits = 1
            + u64::from(framing.data_bits)
            + u64::from(framing.stop_bits)
            + u64::from(framing.parity != 0);
        self.character_ns = (bits * 1_000_000_000).div_ceil(u64::from(framing.baud));
        let ier = self.ier;
        self.reg_write(IER, 0);
        self.reg_write(LCR, LCR_DLAB);
        self.reg_write(DATA, divisor as u8);
        self.reg_write(IER, (divisor >> 8) as u8);
        self.reg_write(LCR, line);
        self.ier = ier;
        self.reg_write(IER, ier);
        self.reg_write(FCR, if clear { 7 } else { 1 });
        self.reg_write(MCR, 0x0b);
        true
    }
    fn drain_rx(&mut self) -> usize {
        let mut count = 0;
        while count < 256 {
            let status = self.reg_read(LSR);
            if status == 0xff || status & LSR_DR == 0 {
                break;
            }
            let byte = self.reg_read(DATA);
            self.rx.push(&mut self.rx_store, byte);
            count += 1;
        }
        count
    }
    fn fill_tx(&mut self) -> u32 {
        let mut wake = 0;
        let was_full = self.tx.available(TX) == 0;
        let had_data = self.tx.length != 0;
        if self.reg_read(LSR) & LSR_THRE != 0 {
            let mut buffer = [0u8; FIFO];
            let count = self
                .tx
                .read(&mut self.tx_store, &mut buffer[..self.tx.length.min(FIFO)]);
            for byte in &buffer[..count] {
                self.reg_write(DATA, *byte);
            }
            self.in_flight = count;
        }
        if was_full && self.tx.available(TX) != 0 {
            wake |= WAKE_TX;
        }
        if self.tx.length == 0 {
            self.set_ier(self.ier & !IER_TX);
            if had_data {
                wake |= WAKE_DRAIN;
            }
        } else {
            self.set_ier(self.ier | IER_TX);
        }
        wake
    }
    fn service(&mut self) -> u32 {
        let mut wake = 0;
        for _ in 0..64 {
            let id = self.reg_read(IID);
            if id == 0xff || id & 1 != 0 {
                break;
            }
            wake |= WAKE_SERVICED;
            match id & 0x0e {
                4 | 12 => {
                    let empty = self.rx.length == 0;
                    if self.drain_rx() != 0 && empty {
                        wake |= WAKE_RX;
                    }
                }
                6 => {
                    if self.reg_read(LSR) & LSR_DR != 0 {
                        let empty = self.rx.length == 0;
                        let byte = self.reg_read(DATA);
                        self.rx.push(&mut self.rx_store, byte);
                        self.drain_rx();
                        if empty {
                            wake |= WAKE_RX;
                        }
                    }
                }
                2 => wake |= self.fill_tx(),
                0 => {
                    self.reg_read(MSR);
                }
                _ => break,
            }
        }
        wake
    }
}

static PORTS: [TicketLock<Port>; MAX_PORTS] = [const { TicketLock::new(Port::new()) }; MAX_PORTS];
static ALLOCATED_PORTS: TicketLock<[bool; MAX_PORTS]> = TicketLock::new([false; MAX_PORTS]);
static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);
static MATCHES: [raw::Match; 8] = [
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"spacemit,k1-uart".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: MATCH_PXA,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"spacemit,pxa-uart".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: MATCH_PXA,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"intel,xscale-uart".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: MATCH_PXA,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"ns16550a".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"ns16550".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"uart8250".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"snps,dw-apb-uart".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"pnp,16550a".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
];
static DRIVER: TicketLock<raw::DriverDef> = TicketLock::new(raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"uart8250".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: Some(remove),
    shutdown: None,
    context: ptr::null_mut(),
});

fn context(pointer: *mut c_void) -> Option<&'static TicketLock<Port>> {
    let first = PORTS.as_ptr() as usize;
    let end = first + core::mem::size_of_val(&PORTS);
    let value = pointer as usize;
    if value < first
        || value >= end
        || !(value - first).is_multiple_of(core::mem::size_of::<TicketLock<Port>>())
    {
        None
    } else {
        // SAFETY: the range and alignment checks identify one static port.
        Some(unsafe { &*(pointer.cast()) })
    }
}
fn signal(event: Option<Event>) {
    if let Some(event) = event {
        let _ = event.signal();
    }
}

fn reserve_port() -> Option<usize> {
    let mut allocated = ALLOCATED_PORTS.lock_irqsave();
    let index = allocated.iter().position(|entry| !*entry)?;
    allocated[index] = true;
    Some(index)
}

fn release_port(index: usize) {
    PORTS[index].lock_irqsave().reset();
    ALLOCATED_PORTS.lock_irqsave()[index] = false;
}

fn cleanup_port(lock: &'static TicketLock<Port>) {
    let tty = { lock.lock_irqsave().tty.take() };
    if let Some(mut tty) = tty {
        let _ = tty.unregister();
    }
    let (irq, events, mapping) = {
        let mut port = lock.lock_irqsave();
        if port.mapped {
            port.reg_write(IER, 0);
        }
        port.ier = 0;
        port.mapped = false;
        (
            port.irq.take(),
            [
                port.readable.take(),
                port.writable.take(),
                port.drained.take(),
            ],
            port.mmio.take(),
        )
    };
    if let Some(mut irq) = irq {
        let _ = irq.release();
    }
    for event in events.into_iter().flatten() {
        let _ = event.destroy();
    }
    drop(mapping);
}

fn create_events() -> Result<(Event, Event, Event)> {
    let readable = Event::create()?;
    let writable = match Event::create() {
        Ok(event) => event,
        Err(error) => {
            let _ = readable.destroy();
            return Err(error);
        }
    };
    let drained = match Event::create() {
        Ok(event) => event,
        Err(error) => {
            let _ = writable.destroy();
            let _ = readable.destroy();
            return Err(error);
        }
    };
    Ok((readable, writable, drained))
}

fn fail_probe(index: usize, status: i32) -> i32 {
    cleanup_port(&PORTS[index]);
    release_port(index);
    status
}

unsafe extern "C" fn irq(context_pointer: *mut c_void, _virq: u32) -> u32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::IRQ_NONE;
    };
    let (wake, events) = {
        let mut port = lock.lock_irqsave();
        let wake = port.service();
        (wake, (port.readable, port.writable, port.drained))
    };
    if wake == 0 {
        return ddk::IRQ_NONE;
    }
    if wake & WAKE_RX != 0 {
        signal(events.0);
    }
    if wake & WAKE_TX != 0 {
        signal(events.1);
    }
    if wake & WAKE_DRAIN != 0 {
        signal(events.2);
    }
    if wake & !WAKE_SERVICED == 0 {
        ddk::IRQ_HANDLED
    } else {
        ddk::IRQ_HANDLED | ddk::IRQ_RESCHEDULE
    }
}
unsafe extern "C" fn read(context_pointer: *mut c_void, out: *mut u8, length: usize) -> i64 {
    if out.is_null() && length != 0 {
        return ddk::EINVAL.into();
    }
    if length == 0 {
        return 0;
    }
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL.into();
    };
    // SAFETY: the non-null console buffer is writable for `length` bytes.
    let output = unsafe { slice::from_raw_parts_mut(out, length) };
    let count = {
        let mut port = lock.lock_irqsave();
        let count = {
            let Port { rx, rx_store, .. } = &mut *port;
            rx.read(rx_store, output)
        };
        if port.rx.length == 0 {
            let _ = port.readable.map(Event::reset);
        }
        count
    };
    count as i64
}
unsafe extern "C" fn try_read(context_pointer: *mut c_void, out: *mut u8) -> i32 {
    if out.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: the caller supplies one writable byte and `read` preserves that
    // callback contract.
    i32::from(unsafe { read(context_pointer, out, 1) } == 1)
}
unsafe extern "C" fn write(
    context_pointer: *mut c_void,
    input: *const u8,
    length: usize,
    nonblocking: u8,
) -> i32 {
    if input.is_null() && length != 0 {
        return ddk::EINVAL;
    }
    if length == 0 {
        return ddk::OK;
    }
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    // SAFETY: the non-null console buffer is readable for `length` bytes.
    let data = unsafe { slice::from_raw_parts(input, length) };
    let mut offset = 0;
    loop {
        let mut wait = None;
        let wake = {
            let mut port = lock.lock_irqsave();
            if nonblocking != 0 && port.tx.available(TX) < data.len() - offset {
                return ddk::EAGAIN;
            }
            if port.tx.available(TX) != 0 {
                let written = {
                    let Port { tx, tx_store, .. } = &mut *port;
                    tx.write(tx_store, &data[offset..])
                };
                offset += written;
                let _ = port.drained.map(Event::reset);
                let wake = port.fill_tx();
                if port.tx.available(TX) == 0 {
                    let _ = port.writable.map(Event::reset);
                }
                wake
            } else {
                wait = port.writable;
                let _ = wait.map(Event::reset);
                0
            }
        };
        if wake != 0 {
            let events = {
                let port = lock.lock_irqsave();
                (port.readable, port.writable, port.drained)
            };
            if wake & WAKE_RX != 0 {
                signal(events.0);
            }
            if wake & WAKE_TX != 0 {
                signal(events.1);
            }
            if wake & WAKE_DRAIN != 0 {
                signal(events.2);
            }
        }
        if offset == data.len() {
            return ddk::OK;
        }
        if let Some(event) = wait {
            if nonblocking != 0 {
                return ddk::EAGAIN;
            }
            let _ = event.wait();
        }
    }
}
unsafe extern "C" fn configure(
    context_pointer: *mut c_void,
    framing: *const raw::SerialFraming,
) -> i32 {
    if framing.is_null() {
        return ddk::EINVAL;
    }
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    // SAFETY: the console ABI keeps this framing record readable for the call.
    if lock.lock_irqsave().configure(unsafe { &*framing }, false) {
        ddk::OK
    } else {
        ddk::EINVAL
    }
}
unsafe extern "C" fn writable(context_pointer: *mut c_void) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return 0;
    };
    let port = lock.lock_irqsave();
    i32::from(port.tx.available(TX) != 0)
}
unsafe extern "C" fn queued(context_pointer: *mut c_void) -> i64 {
    let Some(lock) = context(context_pointer) else {
        return 0;
    };
    let port = lock.lock_irqsave();
    port.tx.length as i64
}
unsafe extern "C" fn flush_input(context_pointer: *mut c_void) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    let event = {
        let mut port = lock.lock_irqsave();
        port.rx.clear();
        port.reg_write(FCR, 3);
        port.readable
    };
    let _ = event.map(Event::reset);
    ddk::OK
}
unsafe extern "C" fn flush_output(context_pointer: *mut c_void) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    let events = {
        let mut port = lock.lock_irqsave();
        port.tx.clear();
        let ier = port.ier & !IER_TX;
        port.set_ier(ier);
        port.reg_write(FCR, 5);
        (port.writable, port.drained)
    };
    signal(events.0);
    signal(events.1);
    ddk::OK
}
unsafe extern "C" fn flush(context_pointer: *mut c_void) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    let delay = loop {
        let event = {
            let port = lock.lock_irqsave();
            if port.tx.length == 0 {
                break port.character_ns.saturating_mul(port.in_flight as u64);
            }
            port.drained
        };
        let _ = event.map(Event::wait);
    };
    ddk::sleep_ns(delay);
    ddk::OK
}
unsafe extern "C" fn send_break(context_pointer: *mut c_void, duration: u64) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    {
        let port = lock.lock_irqsave();
        port.reg_write(LCR, port.reg_read(LCR) | LCR_BREAK);
    }
    ddk::sleep_ns(if duration == 0 {
        250_000_000
    } else {
        duration * 1_000_000
    });
    {
        let port = lock.lock_irqsave();
        port.reg_write(LCR, port.reg_read(LCR) & !LCR_BREAK);
    }
    ddk::OK
}

fn name(index: u32, buffer: &mut [u8; 16]) -> &CStr {
    let mut n = 0;
    for b in b"ttyS" {
        buffer[n] = *b;
        n += 1;
    }
    let mut digits = [0u8; 10];
    let mut value = index;
    let mut count = 0;
    loop {
        digits[count] = b'0' + (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    for i in (0..count).rev() {
        buffer[n] = digits[i];
        n += 1;
    }
    buffer[n] = 0;
    // SAFETY: the buffer contains the fixed prefix, decimal digits, and one
    // terminator, all within its capacity.
    unsafe { CStr::from_bytes_with_nul_unchecked(&buffer[..=n]) }
}
// The probe path maps registers, programs the line, and publishes a
// terminal; its steps share the port lock and ordering.
#[allow(clippy::too_many_lines)]
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, match_data: usize) -> i32 {
    // SAFETY: the driver core supplies a live device for the probe callback.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };
    let Some(index) = reserve_port() else {
        return ddk::ENOSPC;
    };
    let lock = &PORTS[index];
    {
        let mut port = lock.lock_irqsave();
        port.reset();
        let pxa = match_data == MATCH_PXA;
        if pxa {
            port.shift = 2;
            port.width = 4;
            port.required_ier = IER_UUE;
            port.ier = IER_UUE;
        }
        port.clock = device
            .integer(c"clock-frequency")
            .ok()
            .filter(|v| *v != 0)
            .unwrap_or(if pxa { PXA_CLOCK } else { 1_843_200 }) as u32;
        if let Ok(resource) = device.resource(ddk::RESOURCE_MEMORY, 0) {
            port.memory = true;
            port.shift = device.integer(c"reg-shift").unwrap_or(port.shift.into()) as u32;
            port.width = device.integer(c"reg-io-width").unwrap_or(port.width.into()) as u32;
            if port.shift > 8 || !matches!(port.width, 1 | 2 | 4) {
                drop(port);
                return fail_probe(index, ddk::EINVAL);
            }
            match Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE) {
                Ok(map) => {
                    port.mmio = Some(map);
                    port.mapped = true;
                }
                Err(error) => {
                    drop(port);
                    return fail_probe(index, error.status());
                }
            }
        } else if let Ok(resource) = device.resource(ddk::RESOURCE_IO, 0) {
            if resource.start > u64::from(u16::MAX) {
                drop(port);
                return fail_probe(index, ddk::ENOTSUP);
            }
            port.io = resource.start as u16;
            port.mapped = true;
        } else {
            drop(port);
            return fail_probe(index, ddk::ENOENT);
        }
        let original = port.reg_read(SCR);
        port.reg_write(SCR, 0x5a);
        let first = port.reg_read(SCR);
        port.reg_write(SCR, 0xa5);
        let second = port.reg_read(SCR);
        port.reg_write(SCR, original);
        let framing = raw::SerialFraming {
            baud: 115_200,
            data_bits: 8,
            stop_bits: 1,
            parity: 0,
            odd_parity: 0,
        };
        if first != 0x5a
            || second != 0xa5
            || port.reg_read(LSR) == 0xff
            || !port.configure(&framing, true)
        {
            drop(port);
            return fail_probe(index, ddk::ENODEV);
        }
    }
    let (readable, writable, drained) = match create_events() {
        Ok(events) => events,
        Err(error) => return fail_probe(index, error.status()),
    };
    {
        let mut port = lock.lock_irqsave();
        port.readable = Some(readable);
        port.writable = Some(writable);
        port.drained = Some(drained);
    }
    signal(Some(writable));
    signal(Some(drained));
    let virq = match ddk::irq_of_device(device, 0) {
        Ok(value) => value,
        Err(error) => return fail_probe(index, error.status()),
    };
    let context = core::ptr::from_ref(lock) as *mut c_void;
    let irq_receipt = match Irq::request(device, virq, c"uart", ddk::IRQ_SHARED, irq, context) {
        Ok(value) => value,
        Err(error) => return fail_probe(index, error.status()),
    };
    {
        let mut port = lock.lock_irqsave();
        port.irq = Some(irq_receipt);
        let received = port.drain_rx() != 0;
        port.reg_read(LSR);
        port.reg_read(MSR);
        port.set_ier(IER_RX);
        if received {
            signal(port.readable);
        }
    }
    let root = match ddk::Devfs::current().and_then(|devfs| devfs.root()) {
        Ok(root) => root,
        Err(e) => {
            return fail_probe(index, e.status());
        }
    };
    let mut text = [0; 16];
    let serial = device.integer(c"index").unwrap_or(index as u64) as u32;
    {
        let mut port = lock.lock_irqsave();
        port.ops.context = core::ptr::from_ref(lock) as *mut c_void;
        port.ops.readable_event = port.readable.map_or(0, Event::id);
        port.ops.writable_event = port.writable.map_or(0, Event::id);
    }
    let registration = {
        let port = lock.lock_irqsave();
        // SAFETY: this static port retains the operation table until terminal
        // removal unregisters it.
        unsafe {
            Tty::register(
                Some(device),
                root,
                name(serial, &mut text),
                0o660,
                115_200,
                &port.ops,
            )
        }
    };
    match registration {
        Ok(tty) => {
            lock.lock_irqsave().tty = Some(tty);
            // SAFETY: the saved pointer identifies this static port and is
            // cleared before its slot can be reused.
            unsafe {
                let _ = device.set_data(core::ptr::from_ref(lock) as *mut c_void);
            };
            ddk::OK
        }
        Err(e) => fail_probe(index, e.status()),
    }
}
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the driver core supplies the live device previously probed.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };
    // SAFETY: probe stored either a validated static-port pointer or null.
    let saved = unsafe { device.data() };
    if let Some(lock) = context(saved) {
        let index = (core::ptr::from_ref::<TicketLock<Port>>(lock) as usize
            - PORTS.as_ptr() as usize)
            / core::mem::size_of::<TicketLock<Port>>();
        cleanup_port(lock);
        release_port(index);
        // SAFETY: removal clears the opaque pointer before slot reuse.
        unsafe {
            let _ = device.set_data(ptr::null_mut());
        };
    }
}
fn init(_: Module) -> Result<()> {
    let bus = Bus::find(c"platform")?;
    let definition = {
        let mut driver = DRIVER.lock_irqsave();
        driver.bus = bus.as_raw();
        // The static definition outlives the guard; publish it as 'static.
        let pointer = core::ptr::from_ref(&*driver);
        // SAFETY: `DRIVER` is static and locked while registration reads it.
        unsafe { &*pointer }
    };
    // SAFETY: the definition and all referenced match data are static.
    *REGISTRATION.lock_irqsave() = Some(unsafe { DriverRegistration::register(definition) }?);
    Ok(())
}
fn exit(_: Module) {
    if let Some(mut registration) = REGISTRATION.lock_irqsave().take() {
        let _ = registration.unregister();
    }
}
ddk::module!(
    b"uart8250\0",
    b"8250 and 16550 compatible serial ports\0",
    init,
    exit
);
