//!
//! # 8250 and 16550 Serial Ports
//!
//! Each platform UART gets a stable port slot and TTY backend. Register
//! access uses either port I/O or MMIO, and software rings buffer received
//! and outgoing bytes.
//!
//! The hard IRQ handler quiets only its UART. The threaded handler moves
//! bytes between hardware and software buffers, then wakes waiting callers
//! outside the port lock. Checked TTY removal precedes IRQ and MMIO teardown.
//!

#![no_std]
#![no_main]
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

use core::{
    ffi::{CStr, c_void},
    ptr, slice,
};
use ddk::{
    Bus, Device, DriverRegistration, Error, Event, Irq, Mmio, Module, OwnedEvent, Result,
    TicketLock, Tty, raw,
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

/// Builds the TTY operation table installed for each published UART.
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

/// Queue cursors for a UART's separately stored receive or transmit bytes.
#[derive(Clone, Copy)]
struct Ring {
    /// Index of the oldest occupied slot.
    head: usize,

    /// Number of bytes currently queued.
    length: usize,
}

impl Ring {
    const fn new() -> Self {
        Self { head: 0, length: 0 }
    }

    fn available(self, capacity: usize) -> usize {
        capacity - self.length
    }

    /// Appends one received byte, dropping the oldest byte when the ring is full.
    fn push(&mut self, storage: &mut [u8], byte: u8) {
        if self.length == storage.len() {
            self.head = (self.head + 1) % storage.len();
            self.length -= 1;
        }

        let tail = (self.head + self.length) % storage.len();
        storage[tail] = byte;
        self.length += 1;
    }

    /// Copies queued bytes out in order, handling a wrapped ring head.
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

    /// Adds as many bytes as fit without overwriting pending output.
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

/// Register transport, software queues, and kernel receipts for one UART.
struct Port {
    /// Mapped register window for memory-mapped ports.
    mmio: Option<Mmio>,

    /// Base I/O port number for port-mapped devices.
    io: u16,

    /// Shift applied to a logical register index.
    shift: u32,

    /// Register access width in bytes.
    width: u32,

    /// Input clock used to calculate baud divisors.
    clock: u32,

    /// Whether callbacks may access the hardware registers.
    mapped: bool,

    /// Receipt for the hard and threaded interrupt callbacks.
    irq: Option<Irq>,

    /// Receipt retained while the terminal is published.
    tty: Option<Tty>,

    /// Desired interrupt-enable bits in software.
    ier: u8,

    /// Platform bits kept set when the handler quiets the UART.
    required_ier: u8,

    /// Whether the hard handler has handed off to its worker.
    service_pending: bool,

    /// Approximate duration of a transmitted character.
    character_ns: u64,

    /// Bytes handed to the UART but still tracked as pending.
    in_flight: usize,

    /// Cursor for the received-byte ring.
    rx: Ring,

    /// Cursor for the transmit-byte ring.
    tx: Ring,

    /// Backing storage for received bytes.
    rx_store: [u8; RX],

    /// Backing storage for pending transmit bytes.
    tx_store: [u8; TX],

    /// Event signaled on received data.
    readable: Option<OwnedEvent>,

    /// Event signaled when transmit space becomes available.
    writable: Option<OwnedEvent>,

    /// Event signaled when queued output drains.
    drained: Option<OwnedEvent>,
}

impl Port {
    const fn new() -> Self {
        Self {
            mmio: None,
            io: 0,
            shift: 0,
            width: 1,
            clock: 1_843_200,
            mapped: false,
            irq: None,
            tty: None,
            ier: 0,
            required_ier: 0,
            service_pending: false,
            character_ns: 0,
            in_flight: 0,
            rx: Ring::new(),
            tx: Ring::new(),
            rx_store: [0; RX],
            tx_store: [0; TX],
            readable: None,
            writable: None,
            drained: None,
        }
    }

    /// Returns an unused port slot to its initial software state.
    fn reset(&mut self) {
        self.mmio = None;
        self.io = 0;
        self.shift = 0;
        self.width = 1;
        self.clock = 1_843_200;
        self.mapped = false;
        self.irq = None;
        self.tty = None;
        self.ier = 0;
        self.required_ier = 0;
        self.service_pending = false;
        self.character_ns = 0;
        self.in_flight = 0;
        self.rx = Ring::new();
        self.tx = Ring::new();
        self.readable = None;
        self.writable = None;
        self.drained = None;
    }

    /// Reads a UART register using the port's configured I/O transport and register layout.
    fn reg_read(&self, register: u32) -> u8 {
        let Some(mmio) = self.mmio.as_ref() else {
            return ddk::port_read8(self.io.wrapping_add(register as u16));
        };

        let offset = (register as usize) << self.shift;
        match self.width {
            4 => mmio.read32(offset) as u8,
            2 => mmio.read16(offset) as u8,
            _ => mmio.read8(offset),
        }
    }

    /// Writes a UART register using the port's configured I/O transport and register layout.
    fn reg_write(&self, register: u32, value: u8) {
        let Some(mmio) = self.mmio.as_ref() else {
            ddk::port_write8(self.io.wrapping_add(register as u16), value);
            return;
        };

        let offset = (register as usize) << self.shift;
        match self.width {
            4 => mmio.write32(offset, value.into()),
            2 => mmio.write16(offset, value.into()),
            _ => mmio.write8(offset, value),
        }
    }

    /// Updates the interrupt-enable register and its software copy.
    fn set_ier(&mut self, value: u8) {
        let value = value | self.required_ier;
        if self.ier != value {
            self.ier = value;
            if !self.service_pending {
                self.reg_write(IER, value);
            }
        }
    }

    /// Programs the baud divisor, framing, and optional FIFO reset.
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
        self.reg_write(
            IER,
            if self.service_pending {
                self.required_ier
            } else {
                ier
            },
        );
        self.reg_write(FCR, if clear { 7 } else { 1 });
        self.reg_write(MCR, 0x0b);
        true
    }

    /// Moves available received bytes from the UART FIFO into the software ring.
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

    /// Feeds queued output into the UART transmit FIFO.
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
const DRIVER: raw::DriverDef = raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"uart8250".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: None,
    shutdown: None,
    context: ptr::null_mut(),
    try_remove: Some(remove),
};

/// Validates a callback pointer against the static UART slots and recovers the slot lock.
fn context(pointer: *mut c_void) -> Option<&'static TicketLock<Port>> {
    let offset = pointer.addr().checked_sub(PORTS.as_ptr().addr())?;
    let slot_size = core::mem::size_of::<TicketLock<Port>>();
    if !offset.is_multiple_of(slot_size) {
        return None;
    }
    PORTS.get(offset / slot_size)
}

fn signal(event: Option<Event>) {
    if let Some(event) = event {
        let _ = event.signal();
    }
}

/// Claims an unused static UART slot for a probe.
fn reserve_port() -> Option<usize> {
    let mut allocated = ALLOCATED_PORTS.lock_irqsave();
    let index = allocated.iter().position(|entry| !*entry)?;
    allocated[index] = true;
    Some(index)
}

/// Resets the UART slot before making it available to another probe.
fn release_port(index: usize) {
    PORTS[index].lock_irqsave().reset();
    ALLOCATED_PORTS.lock_irqsave()[index] = false;
}

/// Unregisters the TTY, then releases interrupts, events, and MMIO outside the port lock.
fn cleanup_port(lock: &'static TicketLock<Port>) -> Result<()> {
    let tty = { lock.lock_irqsave().tty.take() };
    if let Some(mut tty) = tty {
        if let Err(error) = tty.unregister() {
            lock.lock_irqsave().tty = Some(tty);
            return Err(error);
        }
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
    drop(events);
    drop(mapping);
    Ok(())
}

/// Creates independent notifications for receive, transmit space, and transmit drain.
fn create_events() -> Result<(OwnedEvent, OwnedEvent, OwnedEvent)> {
    Ok((
        OwnedEvent::create()?,
        OwnedEvent::create()?,
        OwnedEvent::create()?,
    ))
}

/// Unwinds an unpublished port reservation and returns the original error.
fn fail_probe(index: usize, status: i32) -> i32 {
    cleanup_port(&PORTS[index]).expect("unpublished UART must be idle");
    release_port(index);
    status
}

// Hard delivery only identifies and quiets this UART. Do not mask a shared
// controller line: another device may still need it while this port is serviced.
/// Acknowledges hard IRQ admission and quiets this UART until the service routine runs.
unsafe extern "C" fn irq(context_pointer: *mut c_void, _virq: u32) -> u32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::IRQ_NONE;
    };

    let mut port = lock.lock_irqsave();
    if !port.mapped || port.service_pending {
        return ddk::IRQ_NONE;
    }

    let id = port.reg_read(IID);
    if id == 0xff || id & 1 != 0 {
        return ddk::IRQ_NONE;
    }
    port.service_pending = true;
    port.reg_write(IER, port.required_ier);
    ddk::IRQ_HANDLED | ddk::IRQ_WAKE_THREAD
}

/// Services RX and TX interrupts and signals waiters after releasing the port lock.
unsafe extern "C" fn irq_service(context_pointer: *mut c_void, _virq: u32) {
    let Some(lock) = context(context_pointer) else {
        return;
    };

    let (wake, events) = {
        let mut port = lock.lock_irqsave();
        if !port.mapped {
            return;
        }
        // Reading IIR in the hard handler can acknowledge a TX interrupt.
        // Inspect RX/TX state directly rather than depending on that cause bit.
        let empty = port.rx.length == 0;
        let received = port.drain_rx();
        let mut wake = if port.tx.length != 0 || port.ier & IER_TX != 0 {
            port.fill_tx()
        } else {
            0
        };

        if received != 0 && empty {
            wake |= WAKE_RX;
        }
        port.reg_read(LSR);
        port.reg_read(MSR);
        port.service_pending = false;
        port.reg_write(IER, port.ier);
        (
            wake,
            (
                port.readable.as_ref().map(OwnedEvent::event),
                port.writable.as_ref().map(OwnedEvent::event),
                port.drained.as_ref().map(OwnedEvent::event),
            ),
        )
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

/// Reads available bytes from the UART's software receive ring.
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
            let _ = port
                .readable
                .as_ref()
                .map(OwnedEvent::event)
                .map(Event::reset);
        }
        count
    };
    count as i64
}

/// Attempts to read a single received byte without waiting.
unsafe extern "C" fn try_read(context_pointer: *mut c_void, out: *mut u8) -> i32 {
    if out.is_null() {
        return ddk::EINVAL;
    }

    // SAFETY: the caller supplies one writable byte and `read` preserves that
    // callback contract.
    i32::from(unsafe { read(context_pointer, out, 1) } == 1)
}

/// Queues transmit bytes and waits for space when the caller permits blocking.
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
                let _ = port
                    .drained
                    .as_ref()
                    .map(OwnedEvent::event)
                    .map(Event::reset);
                let wake = port.fill_tx();
                if port.tx.available(TX) == 0 {
                    let _ = port
                        .writable
                        .as_ref()
                        .map(OwnedEvent::event)
                        .map(Event::reset);
                }
                wake
            } else {
                wait = port.writable.as_ref().map(OwnedEvent::event);
                let _ = wait.map(Event::reset);
                0
            }
        };

        if wake != 0 {
            let events = {
                let port = lock.lock_irqsave();
                (
                    port.readable.as_ref().map(OwnedEvent::event),
                    port.writable.as_ref().map(OwnedEvent::event),
                    port.drained.as_ref().map(OwnedEvent::event),
                )
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

/// Applies the TTY layer's framing request to the UART.
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

/// Reports whether the transmit ring can accept another byte.
unsafe extern "C" fn writable(context_pointer: *mut c_void) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return 0;
    };

    let port = lock.lock_irqsave();
    i32::from(port.tx.available(TX) != 0)
}

/// Returns the bytes still queued for transmission.
unsafe extern "C" fn queued(context_pointer: *mut c_void) -> i64 {
    let Some(lock) = context(context_pointer) else {
        return 0;
    };

    let port = lock.lock_irqsave();
    port.tx.length as i64
}

/// Discards received bytes from the software ring and UART FIFO.
unsafe extern "C" fn flush_input(context_pointer: *mut c_void) -> i32 {
    let Some(lock) = context(context_pointer) else {
        return ddk::EINVAL;
    };
    {
        let mut port = lock.lock_irqsave();
        port.rx.clear();
        port.reg_write(FCR, 3);
        let event = port.readable.as_ref().map(OwnedEvent::event);
        let _ = event.map(Event::reset);
    };
    ddk::OK
}

/// Discards queued transmit data and resets the UART transmit FIFO.
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
        (
            port.writable.as_ref().map(OwnedEvent::event),
            port.drained.as_ref().map(OwnedEvent::event),
        )
    };
    signal(events.0);
    signal(events.1);
    ddk::OK
}

/// Waits until queued output and the hardware transmitter are both drained.
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
            port.drained.as_ref().map(OwnedEvent::event)
        };

        let _ = event.map(Event::wait);
    };
    ddk::sleep_ns(delay);
    ddk::OK
}

/// Asserts break on the line for the requested duration.
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

/// Formats the TTY name from the assigned UART index.
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

fn configure_port(device: Device, lock: &TicketLock<Port>, pxa: bool) -> Result<()> {
    // Probe owns the reserved slot. Query metadata and map registers before
    // taking the raw lock: those framework operations may allocate or sleep.
    let clock = device
        .integer(c"clock-frequency")
        .ok()
        .filter(|&clock| clock != 0)
        .unwrap_or(if pxa { PXA_CLOCK } else { 1_843_200 }) as u32;
    let memory = device.resource(ddk::RESOURCE_MEMORY, 0).ok();
    let (mapping, io, shift, width) = if let Some(resource) = memory {
        let shift = device
            .integer(c"reg-shift")
            .unwrap_or(if pxa { 2 } else { 0 }) as u32;
        let width = device
            .integer(c"reg-io-width")
            .unwrap_or(if pxa { 4 } else { 1 }) as u32;
        if shift > 8 || !matches!(width, 1 | 2 | 4) {
            return Err(Error::from_status(ddk::EINVAL));
        }

        let mapping = Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE)?;
        (Some(mapping), 0, shift, width)
    } else if let Ok(resource) = device.resource(ddk::RESOURCE_IO, 0) {
        if !cfg!(target_arch = "x86_64")
            || resource
                .start
                .checked_add(7)
                .is_none_or(|end| end > u64::from(u16::MAX))
        {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        if resource.length < 8 {
            return Err(Error::from_status(ddk::EINVAL));
        }
        (None, resource.start as u16, 0, 1)
    } else {
        return Err(Error::from_status(ddk::ENOENT));
    };

    let mut port = lock.lock_irqsave();
    port.reset();
    port.mmio = mapping;
    port.io = io;
    port.shift = shift;
    port.width = width;
    port.clock = clock;
    port.mapped = true;
    if pxa {
        port.required_ier = IER_UUE;
        port.ier = IER_UUE;
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
        return Err(Error::from_status(ddk::ENODEV));
    }

    Ok(())
}

fn start_port_interrupts(device: Device, lock: &'static TicketLock<Port>) -> Result<()> {
    let (readable, writable, drained) = create_events()?;
    signal(Some(writable.event()));
    signal(Some(drained.event()));
    {
        let mut port = lock.lock_irqsave();
        port.readable = Some(readable);
        port.writable = Some(writable);
        port.drained = Some(drained);
    }

    let virq = ddk::irq_of_device(device, 0)?;
    let context = core::ptr::from_ref(lock) as *mut c_void;

    // SAFETY: the static port slot remains reserved until the IRQ is released.
    let irq_receipt = unsafe {
        Irq::request_threaded(
            device,
            virq,
            c"uart",
            ddk::IRQ_SHARED,
            irq,
            irq_service,
            context,
        )
    }?;

    let mut port = lock.lock_irqsave();
    port.irq = Some(irq_receipt);
    let received = port.drain_rx() != 0;
    port.reg_read(LSR);
    port.reg_read(MSR);
    port.set_ier(IER_RX);
    if received {
        signal(port.readable.as_ref().map(OwnedEvent::event));
    }

    Ok(())
}

fn publish_port(device: Device, index: usize, lock: &'static TicketLock<Port>) -> Result<()> {
    let context = core::ptr::from_ref(lock) as *mut c_void;
    let root = ddk::devfs::root()?;
    let mut text = [0; 16];
    let serial = device.integer(c"index").unwrap_or(index as u64) as u32;
    let operations = {
        let port = lock.lock_irqsave();
        raw::ConsoleOps {
            context,
            readable_event: port.readable.as_deref().map_or(0, |event| event.id()),
            writable_event: port.writable.as_deref().map_or(0, |event| event.id()),
            ..console_ops()
        }
    };

    // SAFETY: reserve the bound context before publishing a terminal that may
    // immediately acquire external users. No fallible ownership transfer follows.
    unsafe { device.set_data(context)? };
    // SAFETY: registration copies the table. Callback context is this reserved
    // static port and remains live until terminal removal completes.
    let registration =
        unsafe { Tty::register(root, name(serial, &mut text), 0o660, 115_200, &operations) };
    match registration {
        Ok(tty) => {
            lock.lock_irqsave().tty = Some(tty);
            Ok(())
        }

        Err(error) => {
            // SAFETY: the device is still probing and no TTY was published.
            let _ = unsafe { device.set_data(ptr::null_mut()) };
            Err(error)
        }
    }
}

/// Reserves a port, configures its transport and IRQ, and publishes its TTY backend.
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, match_data: usize) -> i32 {
    // SAFETY: the driver core supplies a live device for the probe callback.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };

    let Some(index) = reserve_port() else {
        return ddk::ENOSPC;
    };

    let lock = &PORTS[index];

    configure_port(device, lock, match_data == MATCH_PXA)
        .and_then(|()| start_port_interrupts(device, lock))
        .and_then(|()| publish_port(device, index, lock))
        .map_or_else(|error| fail_probe(index, error.status()), |()| ddk::OK)
}

/// Unregisters and releases the port if no TTY users prevent teardown.
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) -> i32 {
    // SAFETY: the driver core supplies the live device previously probed.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };

    // SAFETY: probe stored either a validated static-port pointer or null.
    let saved = unsafe { device.data() };
    if let Some(lock) = context(saved) {
        let index = (core::ptr::from_ref::<TicketLock<Port>>(lock) as usize
            - PORTS.as_ptr() as usize)
            / core::mem::size_of::<TicketLock<Port>>();
        if let Err(error) = cleanup_port(lock) {
            return error.status();
        }

        // SAFETY: removal clears the opaque pointer before slot reuse.
        unsafe {
            let _ = device.set_data(ptr::null_mut());
        };
        release_port(index);
    }
    ddk::OK
}

/// Registers the 8250 driver and its platform match table.
fn init(_: Module) -> Result<()> {
    let bus = Bus::find(c"platform")?;
    let definition = raw::DriverDef {
        bus: bus.as_raw(),
        ..DRIVER
    };

    // SAFETY: the match table and callbacks have static lifetimes.
    *REGISTRATION.lock_irqsave() = Some(unsafe { DriverRegistration::register(&definition) }?);
    Ok(())
}

/// Unregisters the 8250 driver during module unload.
fn exit(_: Module) {
    let registration = { REGISTRATION.lock_irqsave().take() };
    if let Some(mut registration) = registration {
        let _ = registration.unregister();
    }
}
ddk::module!(
    b"uart8250\0",
    b"8250 and 16550 compatible serial ports\0",
    init,
    exit
);
