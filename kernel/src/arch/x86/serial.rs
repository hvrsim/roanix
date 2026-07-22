//! Interrupt-driven legacy 8250/16550 UART access.

use x86_64::instructions::port::Port;

use crate::{
    dev::{
        self, DeviceNodeId, KERNEL_DRIVER,
        console::SerialSettings,
        interrupt::{InterruptId, ROUTE_ACTIVE_HIGH, ROUTE_EDGE},
    },
    sys::{smp::IrqSpinLock, sync::Once},
};

const BASES: [u16; 4] = [0x3F8, 0x2F8, 0x3E8, 0x2E8];
const IRQ_LINES: [u32; 4] = [4, 3, 4, 3];
const UART_CLOCK: u32 = 1_843_200;
const RX_CAPACITY: usize = 4096;
const MAX_ISR_PASSES: usize = 64;
const MAX_ISR_BYTES: usize = 256;
const IER_RECEIVE: u8 = 1 << 0;
const IER_LINE_STATUS: u8 = 1 << 2;
const IER_RX: u8 = IER_RECEIVE | IER_LINE_STATUS;

static PORTS: [IrqSpinLock<UartState>; BASES.len()] =
    [const { IrqSpinLock::new(UartState::new()) }; BASES.len()];
static IRQ3_ROUTE: Once<InterruptId> = Once::new();
static IRQ4_ROUTE: Once<InterruptId> = Once::new();

struct RxRing {
    bytes: [u8; RX_CAPACITY],
    head: usize,
    len: usize,
}

struct UartState {
    present: bool,
    interrupts_enabled: bool,
    rx: RxRing,
}

impl RxRing {
    const fn new() -> Self {
        Self {
            bytes: [0; RX_CAPACITY],
            head: 0,
            len: 0,
        }
    }

    fn push(&mut self, byte: u8) {
        if self.len == RX_CAPACITY {
            self.head = (self.head + 1) % RX_CAPACITY;
            self.len -= 1;
        }
        let tail = (self.head + self.len) % RX_CAPACITY;
        self.bytes[tail] = byte;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[self.head];
        self.head = (self.head + 1) % RX_CAPACITY;
        self.len -= 1;
        Some(byte)
    }
}

impl UartState {
    const fn new() -> Self {
        Self {
            present: false,
            interrupts_enabled: false,
            rx: RxRing::new(),
        }
    }
}

/// Handle to one legacy PC UART.
#[derive(Copy, Clone)]
pub struct LegacyUart {
    index: usize,
}

impl LegacyUart {
    /// Probes and initializes one conventional COM port.
    pub fn probe(index: usize) -> Option<Self> {
        let base = *BASES.get(index)?;
        let mut state = PORTS[index].lock();
        // SAFETY: the conventional COM-port range is owned by this UART
        // backend, and the per-port IRQ spinlock serializes register access.
        if unsafe { !probe_locked(base) } {
            return None;
        }
        // SAFETY: the probe established an 8250-compatible register block.
        let _ = unsafe { configure_locked(base, default_settings(), true, 0) };
        state.present = true;
        Some(Self { index })
    }

    /// Creates the early COM1 handle without probing.
    pub const fn com1() -> Self {
        Self { index: 0 }
    }

    /// Returns the I/O-port base.
    pub fn base(self) -> u16 {
        BASES[self.index]
    }

    /// Attempts to receive one byte without blocking.
    pub fn try_read(self) -> Option<u8> {
        let mut state = PORTS[self.index].lock();
        if let Some(byte) = state.rx.pop() {
            return Some(byte);
        }
        let base = self.base();
        // SAFETY: the live handle represents an initialized UART and the lock
        // serializes access to its registers. Polling here is a fallback for a
        // delayed or lost hardware IRQ; normal receive still drains in the ISR.
        unsafe { (read(base, 5) & 0x01 != 0).then(|| read(base, 0)) }
    }

    /// Writes a complete byte slice.
    pub fn write(self, bytes: &[u8]) {
        let base = self.base();
        for byte in bytes {
            loop {
                // SAFETY: the live handle represents an initialized UART.
                // Status polling is read-only.
                unsafe {
                    while read(base, 5) & 0x20 == 0 {
                        core::hint::spin_loop();
                    }
                }
                let _lock = PORTS[self.index].lock();
                // SAFETY: the lock serializes the transmit-register write.
                unsafe {
                    if read(base, 5) & 0x20 != 0 {
                        write(base, 0, *byte);
                        break;
                    }
                }
            }
        }
    }

    /// Applies framing and baud settings.
    pub fn configure(self, settings: SerialSettings) -> bool {
        let state = PORTS[self.index].lock();
        let ier = if state.interrupts_enabled { IER_RX } else { 0 };
        // SAFETY: the live handle represents an initialized UART and the lock
        // serializes the complete DLAB programming sequence.
        unsafe { configure_locked(self.base(), settings, false, ier) }
    }

    /// Waits until the transmitter is empty.
    pub fn flush(self) {
        // SAFETY: the live handle represents an initialized UART.
        unsafe {
            while read(self.base(), 5) & 0x40 == 0 {
                core::hint::spin_loop();
            }
        }
    }

    /// Enables or disables the UART break-control bit.
    pub fn set_break(self, enabled: bool) {
        let _lock = PORTS[self.index].lock();
        // SAFETY: the live handle represents an initialized UART.
        unsafe {
            let mut line = read(self.base(), 3);
            if enabled {
                line |= 1 << 6;
            } else {
                line &= !(1 << 6);
            }
            write(self.base(), 3, line);
        }
    }

    /// Routes and enables receive interrupts for this UART.
    pub fn enable_interrupts(self, node: DeviceNodeId) -> dev::Result<()> {
        let irq = IRQ_LINES[self.index];
        let route = match irq {
            3 => &IRQ3_ROUTE,
            4 => &IRQ4_ROUTE,
            _ => return Err(dev::Error::Unsupported),
        };
        if route.get().is_none() {
            let specifier = super::ioapic::isa_specifier(irq);
            let interrupt = dev::interrupt::request_interrupt(
                KERNEL_DRIVER,
                node,
                &specifier,
                ROUTE_EDGE | ROUTE_ACTIVE_HIGH,
                0,
                handle_legacy_irq,
                irq as usize,
            )?;
            route.call_once(|| interrupt);
        }

        let mut state = PORTS[self.index].lock();
        if !state.present {
            return Err(dev::Error::NotFound);
        }
        // SAFETY: the probed UART is owned by this port state and the lock
        // serializes the transition to interrupt-driven receive.
        unsafe {
            drain_receive_locked(self.base(), &mut state);
            let _ = read(self.base(), 5);
            let _ = read(self.base(), 6);
            state.interrupts_enabled = true;
            write(self.base(), 1, IER_RX);
        }
        Ok(())
    }
}

/// Initializes COM1 for early debug output.
pub fn init_debug() {
    let uart = LegacyUart::com1();
    let _lock = PORTS[0].lock();
    // SAFETY: early x86 startup owns COM1 and runs before other UART users.
    let _ = unsafe { configure_locked(uart.base(), default_settings(), true, 0) };
}

fn default_settings() -> SerialSettings {
    SerialSettings {
        baud: 9600,
        data_bits: 8,
        stop_bits: 1,
        parity: false,
        odd_parity: false,
    }
}

unsafe fn probe_locked(base: u16) -> bool {
    // SAFETY: the caller owns the UART register range under its lock.
    let original = unsafe { read(base, 7) };
    // SAFETY: scratch-register writes are non-destructive to UART operation.
    unsafe {
        write(base, 7, 0x5A);
        let first = read(base, 7);
        write(base, 7, 0xA5);
        let second = read(base, 7);
        write(base, 7, original);
        first == 0x5A && second == 0xA5 && read(base, 5) != 0xFF
    }
}

unsafe fn configure_locked(
    base: u16,
    settings: SerialSettings,
    clear_fifos: bool,
    ier: u8,
) -> bool {
    let denominator = 16u64 * u64::from(settings.baud);
    if denominator == 0 || denominator > u64::from(UART_CLOCK) {
        return false;
    }
    let divisor = (u64::from(UART_CLOCK) + denominator / 2) / denominator;
    if divisor == 0 || divisor > u64::from(u16::MAX) {
        return false;
    }
    let divisor = divisor as u16;
    let mut line = match settings.data_bits {
        5 => 0,
        6 => 1,
        7 => 2,
        _ => 3,
    };
    if settings.stop_bits == 2 {
        line |= 1 << 2;
    }
    if settings.parity {
        line |= 1 << 3;
        if !settings.odd_parity {
            line |= 1 << 4;
        }
    }

    // SAFETY: the caller owns the UART register range under its lock.
    unsafe {
        write(base, 1, 0);
        write(base, 3, 0x80);
        write(base, 0, divisor as u8);
        write(base, 1, (divisor >> 8) as u8);
        write(base, 3, line);
        write(base, 1, ier);
        write(base, 2, if clear_fifos { 0x07 } else { 0x01 });
        write(base, 4, 0x0B);
    }
    true
}

unsafe extern "C" fn handle_legacy_irq(context: usize, _interrupt: u64) -> u32 {
    let irq = context as u32;
    for (index, line) in IRQ_LINES.iter().copied().enumerate() {
        if line != irq {
            continue;
        }
        let mut state = PORTS[index].lock();
        if !state.present || !state.interrupts_enabled {
            continue;
        }
        // SAFETY: this handler runs for the routed legacy line and holds the
        // matching UART state lock while inspecting and draining registers.
        unsafe { service_interrupt_locked(BASES[index], &mut state) };
    }
    0
}

unsafe fn service_interrupt_locked(base: u16, state: &mut UartState) {
    for _ in 0..MAX_ISR_PASSES {
        // SAFETY: the caller owns the live UART register range under its lock.
        let iir = unsafe { read(base, 2) };
        if iir == 0xFF || iir & 1 != 0 {
            break;
        }
        match iir & 0x0E {
            0x04 | 0x0C => {
                // SAFETY: the caller owns the receive registers under its lock.
                unsafe { drain_receive_locked(base, state) };
            }
            0x06 => {
                // SAFETY: reading LSR acknowledges line-status conditions.
                let status = unsafe { read(base, 5) };
                if status & 1 != 0 {
                    // SAFETY: LSR reported receive data available.
                    state.rx.push(unsafe { read(base, 0) });
                    // SAFETY: drain any additional FIFO bytes.
                    unsafe { drain_receive_locked(base, state) };
                }
            }
            0x02 => {
                // SAFETY: THRE interrupts are not used; clear the enable bit if
                // firmware or faulty hardware exposed one.
                let ier = unsafe { read(base, 1) };
                unsafe { write(base, 1, ier & !(1 << 1)) };
            }
            0x00 => {
                // SAFETY: reading MSR acknowledges modem-status conditions.
                let _ = unsafe { read(base, 6) };
            }
            _ => break,
        }
    }
}

unsafe fn drain_receive_locked(base: u16, state: &mut UartState) {
    for _ in 0..MAX_ISR_BYTES {
        // SAFETY: the caller owns the live UART register range under its lock.
        let status = unsafe { read(base, 5) };
        if status == 0xFF || status & 1 == 0 {
            break;
        }
        // SAFETY: LSR reported one readable receive byte.
        state.rx.push(unsafe { read(base, 0) });
    }
}

unsafe fn read(base: u16, register: u16) -> u8 {
    let mut port = Port::<u8>::new(base + register);
    // SAFETY: the caller guarantees ownership of this UART register.
    unsafe { port.read() }
}

unsafe fn write(base: u16, register: u16, value: u8) {
    let mut port = Port::<u8>::new(base + register);
    // SAFETY: the caller guarantees ownership of this UART register.
    unsafe { port.write(value) };
}
