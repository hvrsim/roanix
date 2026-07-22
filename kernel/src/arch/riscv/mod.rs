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
use core::{
    arch::asm,
    sync::atomic::{AtomicBool, Ordering},
};

use crate::sys::smp::{self, IpiTarget};

pub mod cpu;
pub mod paging;
pub mod timer;

/// BSP's core local context.
///
/// # Safety
///
/// This bootstrap instance is only referenced through a raw pointer
/// during early bring-up before hart-local state becomes shared.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);
static EXTERNAL_INTERRUPTS_ENABLED: AtomicBool = AtomicBool::new(false);

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
    console_write(line);
    console_write(b"\n");
}

/// Writes bytes directly to the architecture debug console.
pub(crate) fn console_write(line: &[u8]) {
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
}

/// Returns platform-provided hardware entropy when available.
pub(crate) fn entropy_word() -> Option<u64> {
    None
}

/// Returns core local context.
///
/// On riscv64, kernel core-local data is stored in the TP register.
///
/// The kernel thread-local context isn't valid until [`set_core_local`] is
/// called, which happens very early in boot.
#[inline(always)]
pub fn thiscpu() -> &'static CoreLocal {
    thiscpu_opt().expect("riscv: thiscpu called before TP was initialized")
}

/// Returns core local context if it is initialized.
///
/// **NOTE: This function should only be used by the kernel logger,
/// since it's the only module that runs before corelocal setup.**
#[inline(always)]
pub fn thiscpu_opt() -> Option<&'static CoreLocal> {
    let ptr = thiscpu_ptr()?;

    // SAFETY: TP is only initialized from stable `CoreLocal` allocations.
    Some(unsafe { &*ptr })
}

/// Returns mutable core-local context for the current hart.
///
/// # Safety
///
/// The caller must have exclusive access to the current hart's `CoreLocal` for
/// the duration of the returned borrow. In practice this requires early boot
/// or local interrupts to be disabled.
#[inline(always)]
pub unsafe fn thiscpu_mut() -> &'static mut CoreLocal {
    let ptr = thiscpu_ptr().expect("riscv: thiscpu called before TP was initialized");

    // SAFETY: the caller guarantees exclusive access to this hart's state.
    unsafe { &mut *ptr }
}

#[inline(always)]
fn thiscpu_ptr() -> Option<*mut CoreLocal> {
    let value: usize;

    // SAFETY: reading TP only snapshots the hart-local pointer installed by
    // `set_core_local`.
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

    Some(value as *mut CoreLocal)
}

/// Sets the core local pointer.
///
/// Writes the provided core local pointer into the TP register.
///
/// **NOTE: This function can only be called once per core.
/// Further calls may result in a panic!**
pub fn set_core_local(ptr: *const CoreLocal) {
    let value: usize;

    // SAFETY: this runs once during hart bring-up; `ptr` refers to a stable
    // `CoreLocal` allocation that outlives the hart.
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
    // SAFETY: SSTATUS is readable in supervisor mode.
    unsafe { cpu::rdcsr::<{ cpu::CSR_SSTATUS }>() & cpu::SSTATUS_SIE != 0 }
}

/// Enables or disables CPU interrupts.
#[inline(always)]
pub fn irqset(enable: bool) {
    // SAFETY: toggling SSTATUS.SIE is the architecture-defined local interrupt
    // control operation in supervisor mode.
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

/// Initializes the bootstrap hart and early debug output.
pub fn init_boot_cpu() {
    init_cpu(&raw const BSP_CORE_LOCAL);
    debug::register_sink(dbgcon_write);
}

/// Initializes global RISC-V platform facilities that require memory services.
pub fn init_platform() {
    timer::init();
}

/// Performs per-hart initialization for a secondary core.
pub fn init_secondary_cpu(core_local: *const CoreLocal) {
    init_cpu(core_local);
    timer::init_secondary();
}

fn init_cpu(core_local: *const CoreLocal) {
    set_core_local(core_local);
    cpu::enable_features();
    refresh_external_interrupts();
}

pub(crate) fn set_external_interrupts(enable: bool) {
    EXTERNAL_INTERRUPTS_ENABLED.store(enable, Ordering::Release);
    refresh_external_interrupts();
    let _ = smp::send_ipi(refresh_external_interrupts, IpiTarget::All);
}

fn refresh_external_interrupts() {
    cpu::set_external_interrupts(EXTERNAL_INTERRUPTS_ENABLED.load(Ordering::Acquire));
}

/// Pauses CPU execution and waits for interrupts.
#[inline(always)]
pub fn wfi() {
    // SAFETY: WFI only suspends this hart until an interrupt/event arrives.
    unsafe {
        asm!("wfi", options(nomem, nostack, preserves_flags));
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
    let legacy_mask = 1usize << (hartid as usize);
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

    // A direct local `sip.SSIP` write with SIE already enabled can interrupt
    // the hart at the CSR write itself, leaving `sepc` on that instruction.
    // After trap return the hart can re-execute the same write and livelock in
    // a software-interrupt loop.
    //
    // Raise the pending bit with interrupts masked, then restore SIE. The
    // interrupt is delivered only after reenabling, so `sepc` no longer points
    // at the SSIP source instruction.
    irqset(false);
    // SAFETY: SSIP is the hart-local supervisor software interrupt pending bit.
    unsafe {
        cpu::set_csr_bits::<{ cpu::CSR_SIP }>(cpu::SIE_SSIE);
    }
    irqset(true);
}

/// Invokes an SBI call that only needs `a0`.
pub(crate) fn sbi_call1(arg0: usize, ext_id: usize, func_id: usize) -> SbiRet {
    // SAFETY: this wrapper is only used with fixed SBI calls whose argument
    // layouts are encoded at each call site.
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
    // SAFETY: this wrapper is only used with fixed SBI calls whose argument
    // layouts are encoded at each call site.
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

    // SAFETY: the caller guarantees that the extension/function IDs and
    // register arguments satisfy the selected SBI ABI.
    unsafe {
        asm!(
            "ecall",
            inlateout("a0") arg0 as isize => error,
            inlateout("a1") arg1 => value,
            in("a2") arg2,
            in("a6") func_id,
            in("a7") ext_id,
        );
    }

    let _ = value;
    SbiRet { error }
}
