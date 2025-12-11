//!
//! # Riscv64 Subsystem
//!
//! Kernel support layer for the riscv platform, including drivers for
//! the SBI interface, on-chip timer, interrupt controllers and more!
//!
//! For a general riscv reference, I highly recommend the riscv ISA
//! manuals. You can grab the latest copies [here](https://github.com/riscv/riscv-isa-manual/releases/tag/latest).
//!

use crate::sys::smp::CoreLocal;
use core::arch::asm;

pub mod cpu;

/// BSP's core local context.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);

/// Writes debug messages to the current debug sink.
///
/// On all riscv64 platforms, we use the SBI debug console API.
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

/// Invokes SBI firmware API using the `ecall` instruction.
///
/// **SAFETY:** This function intentionally discards errors returned by the API.
#[inline]
unsafe fn sbicall(arg: usize, ext_id: usize, func_id: usize) -> usize {
    let value: usize;

    asm!(
        "ecall",
        in("a0") arg,
        in("a6") func_id,
        in("a7") ext_id,
        lateout("a1") value,
    );

    return value;
}

/// Returns core local context.
///
/// On the riscv64 platform, kernel core local data is
/// stored in the TP register.
///
/// ## Safety
///
/// The kernel thread-local context isn't valid until
/// [`set_core_local`]('set_core_local') is called, which
/// happens very early in boot. If you find yourself requiring
/// thread local context super early in boot, consider moving
/// your init stage into a later part of the boot pipeline.
#[inline(always)]
pub fn thiscpu() -> &'static mut CoreLocal {
    let value: usize;

    unsafe {
        asm!(
            "mv {}, tp",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );

        assert!(value != 0);

        &mut *(value as *mut CoreLocal)
    }
}

/// Sets the core local pointer.
///
/// Writes the provided core local pointer into the TP register.
///
/// **This function can only be called once per core. Further
/// calls may result in a panic!**
pub fn set_core_local(ptr: *const CoreLocal) {
    let value: usize;

    unsafe {
        asm!(
            "mv {}, tp",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );

        assert!(value == 0);

        asm!(
            "mv tp, {addr}",
            addr = in(reg) ptr,
            options(nostack, preserves_flags)
        );
    }
}

/// Performs early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {
    set_core_local(&raw const BSP_CORE_LOCAL);

    cpu::enable_features();
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
