//! Minimal early COM1 debug output retained until packaged drivers load.

use core::sync::atomic::{AtomicBool, Ordering};
use x86_64::instructions::port::Port;

const COM1_BASE: u16 = 0x3F8;
const UART_CLOCK: u32 = 1_843_200;
const DEBUG_BAUD: u32 = 115_200;
const FIFO_CAPACITY: usize = 16;
static FIFO_ENABLED: AtomicBool = AtomicBool::new(false);

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
        FIFO_ENABLED.store(read(2) & 0xC0 == 0xC0, Ordering::Relaxed);
    }
}

/// Writes bytes to the early COM1 debug console.
pub fn write_debug(bytes: &[u8]) {
    let mut output = [0u8; FIFO_CAPACITY];
    let mut length = 0usize;

    for byte in bytes.iter().copied() {
        if byte == b'\n' {
            output[length] = b'\r';
            length += 1;
            if length == output.len() {
                write_fifo(&output);
                length = 0;
            }
        }
        output[length] = byte;
        length += 1;
        if length == output.len() {
            write_fifo(&output);
            length = 0;
        }
    }
    if length != 0 {
        write_fifo(&output[..length]);
    }
}

fn write_fifo(bytes: &[u8]) {
    // SAFETY: COM1 remains reserved for the kernel's early debug sink and
    // THRE guarantees the enabled FIFO can accept one complete burst.
    unsafe {
        while read(5) & 0x20 == 0 {
            core::hint::spin_loop();
        }
        let count = if FIFO_ENABLED.load(Ordering::Relaxed) {
            bytes.len()
        } else {
            1
        };
        for byte in &bytes[..count] {
            write(0, *byte);
        }
    }
    if !FIFO_ENABLED.load(Ordering::Relaxed) && bytes.len() > 1 {
        for byte in &bytes[1..] {
            write_fifo(core::slice::from_ref(byte));
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
