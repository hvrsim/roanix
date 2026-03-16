//!
//! # Riscv64 Subsystem
//!
//! Kernel support layer for the riscv64 platform, including drivers for the
//! SBI interface, timer, interrupt delivery, and hart-local state.
//!
//! For a general riscv reference, I highly recommend the riscv ISA
//! manuals. You can grab the latest copies [here](https://github.com/riscv/riscv-isa-manual/releases/tag/latest).
//!

use crate::sys::{debug, smp::CoreLocal};
use core::arch::asm;

pub mod cpu;
pub mod paging;
pub mod timer;

/// BSP's core local context.
// SAFETY: this bootstrap instance is only referenced through a raw pointer
// during early bring-up before hart-local state becomes shared.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);

/// SBI extension ID for the debug console.
const DEBUG_EXT_ID: usize = 0x4442434E;
const SBI_EXT_IPI: usize = 0x735049;
const SBI_EXT_IPI_SEND: usize = 0;
const SBI_LEGACY_SEND_IPI: usize = 0x04;

/// Result tuple returned by an SBI call.
#[derive(Copy, Clone)]
pub(crate) struct SbiRet {
    /// SBI status code, where `0` indicates success.
    pub(crate) error: isize,
}

/// Writes debug messages to the current debug sink.
///
/// On all riscv64 platforms, we use the SBI debug console API.
fn dbgcon_write(buf: *const u8, buflen: usize) {
    // SAFETY: the debug subsystem only calls sinks with a live buffer for the
    // duration of the callback.
    let line = unsafe { core::slice::from_raw_parts(buf, buflen) };

    let putc = |byte: u8| {
        let ret = sbi_call1(byte as usize, DEBUG_EXT_ID, 2);

        // Invoke the legacy SBI v0.1 `console_putchar` extension as a
        // fallback for platforms/firmware that do not implement DBCN.
        if ret.error != 0 {
            let _ = sbi_call1(byte as usize, 0x01, 0);
        }
    };

    for &byte in line {
        putc(byte);
    }

    putc(b'\n');
}

/// Returns core local context.
///
/// On riscv64, kernel core-local data is stored in the TP register.
///
/// ## Safety
///
/// The kernel thread-local context isn't valid until [`set_core_local`] is
/// called, which
/// happens very early in boot. If you find yourself requiring
/// thread-local context super early in boot, consider moving
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

    // SAFETY: TP is only initialized from stable `CoreLocal` allocations.
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

/// Performs post-memory architecture initialization.
pub fn init() {
    timer::init();
}

/// Performs per-hart initialization for a secondary core.
pub fn init_secondary(core_local: *const CoreLocal) {
    set_core_local(core_local);
    cpu::enable_features();
    timer::init_secondary();
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

/// Sends a reschedule IPI to `cpu_id`.
pub fn send_ipi(cpu_id: usize) {
    let hartid = crate::sys::smp::platform_id(cpu_id).expect("riscv: invalid CPU ID for IPI");
    let mask = 1usize;
    if sbi_call3(mask, hartid as usize, 0, SBI_EXT_IPI, SBI_EXT_IPI_SEND).error == 0 {
        return;
    }

    assert!(
        hartid < usize::BITS as u64,
        "riscv: legacy SBI IPI fallback requires hartid < {}",
        usize::BITS
    );
    let legacy_mask = 1usize << hartid;
    let legacy_error = sbi_call1(
        &legacy_mask as *const usize as usize,
        SBI_LEGACY_SEND_IPI,
        0,
    );
    assert_eq!(legacy_error.error, 0, "riscv: SBI send_ipi failed");
}

/// Forces the current CPU through the scheduler trap path.
pub fn reschedule() {
    assert!(
        irqstate(),
        "riscv: local reschedule requires interrupts to be enabled"
    );

    unsafe {
        // Trigger a supervisor-software interrupt and return immediately.
        // Executing `wfi` here can race with immediate trap delivery: SSIP may
        // be serviced and cleared before `wfi`, leaving the hart sleeping
        // unexpectedly until an unrelated interrupt arrives.
        asm!("csrsi sip, 0x2", options(nomem, nostack, preserves_flags));
    }
}

/// Invokes an SBI call that only needs `a0`.
pub(crate) fn sbi_call1(arg0: usize, ext_id: usize, func_id: usize) -> SbiRet {
    unsafe { sbi_call(arg0, 0, 0, ext_id, func_id) }
}

/// Invokes an SBI call that needs `a0`, `a1`, and `a2`.
pub(crate) fn sbi_call3(
    arg0: usize,
    arg1: usize,
    arg2: usize,
    ext_id: usize,
    func_id: usize,
) -> SbiRet {
    unsafe { sbi_call(arg0, arg1, arg2, ext_id, func_id) }
}

/// # Safety
///
/// The caller must ensure the targeted SBI extension/function ID pair is valid
/// for the running firmware and that the register arguments match that call's
/// ABI contract.
unsafe fn sbi_call(arg0: usize, arg1: usize, arg2: usize, ext_id: usize, func_id: usize) -> SbiRet {
    let error: isize;
    let value: usize;

    asm!(
        "ecall",
        inlateout("a0") arg0 as isize => error,
        inlateout("a1") arg1 => value,
        in("a2") arg2,
        in("a6") func_id,
        in("a7") ext_id,
    );

    let _ = value;
    SbiRet { error }
}
