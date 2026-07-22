//! Legacy 8250/16550 UART access shared by debug and TTY consoles.

use x86_64::instructions::port::Port;

use crate::{dev::console::SerialSettings, sys::smp::IrqSpinLock};

const BASES: [u16; 4] = [0x3F8, 0x2F8, 0x3E8, 0x2E8];
const UART_CLOCK: u32 = 1_843_200;

static LOCKS: [IrqSpinLock<()>; BASES.len()] = [const { IrqSpinLock::new(()) }; BASES.len()];

/// Handle to one legacy PC UART.
#[derive(Copy, Clone)]
pub struct LegacyUart {
    index: usize,
}

impl LegacyUart {
    /// Probes and initializes one conventional COM port.
    pub fn probe(index: usize) -> Option<Self> {
        let base = *BASES.get(index)?;
        let _lock = LOCKS[index].lock();
        // SAFETY: the conventional COM-port range is owned by this UART
        // backend, and the per-port IRQ spinlock serializes register access.
        if unsafe { !probe_locked(base) } {
            return None;
        }
        // SAFETY: the probe established an 8250-compatible register block.
        let _ = unsafe { configure_locked(base, default_settings(), true) };
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
        let _lock = LOCKS[self.index].lock();
        let base = self.base();
        // SAFETY: the live handle represents an initialized UART and the lock
        // serializes access to its registers.
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
                let _lock = LOCKS[self.index].lock();
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
        let _lock = LOCKS[self.index].lock();
        // SAFETY: the live handle represents an initialized UART and the lock
        // serializes the complete DLAB programming sequence.
        unsafe { configure_locked(self.base(), settings, false) }
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
        let _lock = LOCKS[self.index].lock();
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
}

/// Initializes COM1 for early debug output.
pub fn init_debug() {
    let uart = LegacyUart::com1();
    let _lock = LOCKS[0].lock();
    // SAFETY: early x86 startup owns COM1 and runs before other UART users.
    let _ = unsafe { configure_locked(uart.base(), default_settings(), true) };
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

unsafe fn configure_locked(base: u16, settings: SerialSettings, clear_fifos: bool) -> bool {
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
        write(base, 2, if clear_fifos { 0x07 } else { 0x01 });
        write(base, 4, 0x0B);
    }
    true
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
