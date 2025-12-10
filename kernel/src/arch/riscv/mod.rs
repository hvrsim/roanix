//!
//! # Riscv64 Subsystem
//!
//! Kernel support layer for the riscv platform, including drivers for
//! the SBI interface, on-chip timer, interrupt controllers and more!
//!
//! For a general riscv reference, I highly recommend the riscv ISA
//! manuals. You can grab the latest copies [here](https://github.com/riscv/riscv-isa-manual/releases/tag/latest).
//!

use core::arch::asm;

///
/// Writes debug messages to the current debug sink.
///
/// On all riscv64 platforms, we use the SBI debug console API.
//
pub struct DebugConsole;

impl DebugConsole {
    /// SBI extension ID for the debug console.
    const DEBUG_EXT_ID: usize = 0x4442434E;

    /// Creates a new instance of [`DebugConsole`].
    pub fn new() -> Self {
        DebugConsole {}
    }

    /// Writes a single character to the SBI debug console.
    #[inline(always)]
    pub fn write(&self, byte: u8) {
        unsafe {
            sbicall(byte.into(), Self::DEBUG_EXT_ID, 2);
        }
    }
}

///
/// Invokes SBI firmware API using the `ecall` instruction.
///
/// **SAFETY:** This function intentionally discards errors returned by the API.
///
#[inline]
unsafe fn sbicall(arg: usize, ext_id: usize, func_id: usize) -> usize {
    let value: usize;

    core::arch::asm!(
        "ecall",
        in("a0") arg,
        in("a6") func_id,
        in("a7") ext_id,
        lateout("a1") value,
    );

    return value;
}

/// Pauses CPU execution and waits for interrupts.
///
/// **If interrupts are disabled, this will result in an infinite loop.**
pub fn wfi() -> ! {
    loop {
        unsafe {
            asm!("wfi", options(nomem, nostack, preserves_flags));
        }
    }
}
