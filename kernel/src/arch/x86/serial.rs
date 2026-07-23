//! Minimal early COM1 debug output retained until packaged drivers load.

use x86_64::instructions::port::Port;

const COM1_BASE: u16 = 0x3F8;
const UART_CLOCK: u32 = 1_843_200;
const DEBUG_BAUD: u32 = 9_600;

/// Initializes COM1 for early debug output.
pub fn init_debug() {
    let divisor = (UART_CLOCK / (16 * DEBUG_BAUD)) as u16;
    // SAFETY: early x86 startup owns the conventional COM1 register range.
    unsafe {
        write(1, 0);
        write(3, 0x80);
        write(0, divisor as u8);
        write(1, (divisor >> 8) as u8);
        write(3, 0x03);
        write(2, 0x07);
        write(4, 0x0B);
    }
}

/// Writes bytes to the early COM1 debug console.
pub fn write_debug(bytes: &[u8]) {
    for byte in bytes {
        // SAFETY: COM1 remains reserved for the kernel's early debug sink.
        unsafe {
            while read(5) & 0x20 == 0 {
                core::hint::spin_loop();
            }
            write(0, *byte);
        }
    }
}

unsafe fn read(register: u16) -> u8 {
    let mut port = Port::<u8>::new(COM1_BASE + register);
    // SAFETY: the caller owns the COM1 register range.
    unsafe { port.read() }
}

unsafe fn write(register: u16, value: u8) {
    let mut port = Port::<u8>::new(COM1_BASE + register);
    // SAFETY: the caller owns the COM1 register range.
    unsafe { port.write(value) };
}
