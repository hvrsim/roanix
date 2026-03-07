//!
//! # Riscv64 Subsystem
//!
//! Kernel support layer for the riscv platform, including drivers for
//! the SBI interface, on-chip timer, interrupt controllers and more!
//!
//! For a general riscv reference, I highly recommend the riscv ISA
//! manuals. You can grab the latest copies [here](https://github.com/riscv/riscv-isa-manual/releases/tag/latest).
//!

use crate::sys::{debug, smp::CoreLocal};
use core::arch::asm;

pub mod cpu;

/// BSP's core local context.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);

/// SBI extension ID for the debug console.
const DEBUG_EXT_ID: usize = 0x4442434E;

/// Writes debug messages to the current debug sink.
///
/// On all riscv64 platforms, we use the SBI debug console API.
fn dbgcon_write(buf: *const u8, buflen: usize) {
    let line = unsafe { core::slice::from_raw_parts(buf, buflen) };

    let putc = |byte: u8| {
        let (error, _) = unsafe { sbicall(byte as usize, DEBUG_EXT_ID, 2) };

        // Invoke the legacy SBI v0.1 `console_putchar` extension as a
        // fallback for platforms/firmware that do not implement DBCN.
        if error != 0 {
            let _ = unsafe { sbicall(byte as usize, 0x01, 0) };
        }
    };

    for &byte in line {
        putc(byte);
    }

    putc(b'\n');
}

/// Invokes SBI firmware API using the `ecall` instruction.
///
/// **SAFETY:** This function intentionally discards errors returned by the API.
#[inline]
unsafe fn sbicall(arg: usize, ext_id: usize, func_id: usize) -> (isize, usize) {
    let error: isize;
    let value: usize;

    asm!(
        "ecall",
        inlateout("a0") arg as isize => error,
        in("a6") func_id,
        in("a7") ext_id,
        lateout("a1") value,
    );

    (error, value)
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
    thiscpu_opt().expect("riscv: thiscpu called before TP was initialized")
}

/// Returns core local context if it is initialized.
///
/// **NOTE: This function should only be used by the kernel logger,
/// since it's the only module that runs before corelocal setup.**
#[inline(always)]
pub fn thiscpu_opt() -> Option<&'static mut CoreLocal> {
    let value: usize;

    unsafe {
        asm!(
            "mv {}, tp",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );
    }

    if value == 0 {
        return None;
    }

    Some(unsafe { &mut *(value as *mut CoreLocal) })
}

/// Sets the core local pointer.
///
/// Writes the provided core local pointer into the TP register.
///
/// **NOTE: This function can only be called once per core.
/// Further calls may result in a panic!**
pub fn set_core_local(ptr: *const CoreLocal) {
    let value: usize;

    unsafe {
        asm!(
            "mv {}, tp",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );

        assert_eq!(value, 0);

        asm!(
            "mv tp, {addr}",
            addr = in(reg) ptr,
            options(nostack, preserves_flags)
        );
    }
}

/// Returns whether CPU interrupts are currently enabled.
#[inline(always)]
pub fn irqstate() -> bool {
    let sstatus: usize;

    unsafe {
        asm!(
            "csrr {}, sstatus",
            out(reg) sstatus,
            options(nomem, nostack, preserves_flags)
        );
    }

    sstatus & 0x2 != 0
}

/// Enables or disables CPU interrupts.
#[inline(always)]
pub fn irqset(enable: bool) {
    unsafe {
        if enable {
            asm!(
                "csrsi sstatus, 0x2",
                options(nomem, nostack, preserves_flags)
            );
        } else {
            asm!(
                "csrci sstatus, 0x2",
                options(nomem, nostack, preserves_flags)
            );
        }
    }
}

/// Performs early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {
    set_core_local(&raw const BSP_CORE_LOCAL);
    cpu::enable_features();

    debug::register_sink(dbgcon_write);
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
