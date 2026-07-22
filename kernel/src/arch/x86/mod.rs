//!
//! # x86_64 Subsystem
//!
//! Implements the support interface the kernel uses to interact with the
//! x86_64 platform. Drivers for on-chip devices such as the TSC and APIC are
//! provided by this module as well.
//!
//! Code in this module often references the *Intel SDM* for register references
//! and ISA semantics.
//!
//! *You may download the SDM [here.](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)*
//!

use core::arch::asm;

use x86_64::addr::VirtAddr;
use x86_64::instructions::{hlt, interrupts as x86_interrupts, port::*};
use x86_64::registers::model_specific::{GsBase, KernelGsBase};

use crate::sys::{debug, smp::CoreLocal};

pub mod cpu;
pub mod lapic;
pub mod paging;
pub mod serial;
pub mod timer;

/// BSP's core local context.
// SAFETY: this bootstrap instance is only addressed through a raw pointer
// during early bring-up before per-CPU state becomes shared.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);

///
/// Writes debug messages to the current debug sink.
///
/// On QEMU/bochs, the debug sink is port 0xE9 (unallocated on real hardware).
///
/// On real hardware, the debug sink is the PC serial port (COM1).
///
/// For ease of debugging, the debug console writes to both sinks at once.
///
/// *NOTE: to make output from this console visible, pass `-debugcon stdio` to
/// QEMU flags, like so:*
///
/// ```bash
/// $ QEMUFLAGS="... -debugcon stdio" make run-bios
/// ```
///
fn dbgcon_write(buf: *const u8, buflen: usize) {
    // SAFETY: the debug subsystem only calls sinks with a live buffer for the
    // duration of the callback.
    let line = unsafe { core::slice::from_raw_parts(buf, buflen) };
    console_write(line);
    console_write(b"\n");
}

/// Writes bytes directly to the architecture debug console.
pub(crate) fn console_write(line: &[u8]) {
    let mut dbgcon_e9: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0xE9);

    for &byte in line {
        // SAFETY: port 0xE9 is the QEMU/Bochs debug console.
        unsafe {
            dbgcon_e9.write(byte);
        }
        let uart = serial::LegacyUart::com1();
        if byte == b'\n' {
            uart.write(b"\r");
        }
        uart.write(core::slice::from_ref(&byte));
    }
}

/// Initializes the debug console for printing.
///
/// On x86_64, QEMU's debugcon is pre-configured, so we simply
/// set COM1 (16550 UART) to 9600 9600 8N1.
fn dbgcon_init() {
    serial::init_debug();
}

/// Initializes the bootstrap processor and early debug output.
pub fn init_boot_cpu() {
    dbgcon_init();
    init_cpu(&raw const BSP_CORE_LOCAL);
    debug::register_sink(dbgcon_write);
}

/// Initializes global x86 platform facilities that require memory services.
pub fn init_platform() {
    timer::init();
}

/// Performs per-CPU initialization for a secondary core.
pub fn init_secondary_cpu(core_local: *const CoreLocal) {
    init_cpu(core_local);
    timer::init_secondary();
}

fn init_cpu(core_local: *const CoreLocal) {
    set_core_local(core_local);
    let feats = cpu::enable_features(core_local);
    // Loading the GDT refreshes the hidden GS base from its zero-base segment
    // descriptor, so restore both SWAPGS slots afterwards.
    set_core_local(core_local);
    // SAFETY: the current CPU is not shared until its local initialization
    // completes.
    unsafe { thiscpu_mut() }.platform.feats = feats;
}

/// Returns core local context.
///
/// On x86_64, kernel core-local data is addressed through the GS base
/// registers.
///
/// The kernel thread-local context isn't valid until [`set_core_local`] is
/// called, which happens very early in boot.
#[inline(always)]
pub fn thiscpu() -> &'static CoreLocal {
    thiscpu_opt().expect("x86: thiscpu called before GS base was initialized")
}

/// Returns core local context if it is initialized.
#[inline(always)]
pub fn thiscpu_opt() -> Option<&'static CoreLocal> {
    let ptr = thiscpu_ptr()?;

    // SAFETY: GS base is only initialized from stable `CoreLocal` allocations.
    Some(unsafe { &*ptr })
}

/// Returns mutable core-local context for the current CPU.
///
/// # Safety
///
/// The caller must have exclusive access to the current CPU's `CoreLocal` for
/// the duration of the returned borrow. In practice this requires early boot
/// or local interrupts to be disabled.
#[inline(always)]
pub unsafe fn thiscpu_mut() -> &'static mut CoreLocal {
    let ptr = thiscpu_ptr().expect("x86: thiscpu called before GS base was initialized");

    // SAFETY: the caller guarantees exclusive access to this CPU's state.
    unsafe { &mut *ptr }
}

#[inline(always)]
fn thiscpu_ptr() -> Option<*mut CoreLocal> {
    let mut base = GsBase::read();
    if base.is_null() {
        // AP entry may arrive with the kernel value still parked in
        // IA32_KERNEL_GS_BASE until the first SWAPGS path runs.
        base = KernelGsBase::read();
    }

    if base.is_null() {
        return None;
    }

    Some(base.as_mut_ptr::<CoreLocal>())
}

/// Sets the core local pointer.
///
/// Writes the provided core local pointer into both GS base MSRs.
///
pub fn set_core_local(ptr: *const CoreLocal) {
    let ptr = VirtAddr::from_ptr(ptr);

    // Limine may enter different CPUs with either GS slot active, depending on
    // whether SWAPGS has already been used on that path. Keep both MSRs in sync
    // during kernel-only execution so per-CPU state is reachable either way.
    GsBase::write(ptr);
    KernelGsBase::write(ptr);
}

/// Returns whether CPU interrupts are currently enabled.
#[inline(always)]
pub fn irqstate() -> bool {
    x86_interrupts::are_enabled()
}

/// Enables or disables CPU interrupts.
#[inline(always)]
pub fn irqset(enable: bool) {
    if enable {
        x86_interrupts::enable();
    } else {
        x86_interrupts::disable();
    }
}

/// Pauses CPU execution and waits for interrupts.
#[inline(always)]
pub fn wfi() {
    hlt();
}

/// Sends a reschedule IPI to `cpu_id`.
pub fn send_ipi(cpu_id: usize) {
    let lapic_id = crate::sys::smp::platform_id(cpu_id).expect("x86: invalid CPU ID for IPI");
    lapic::send_ipi(lapic_id as u32);
}

/// Forces the current CPU through the scheduler trap path.
pub fn reschedule() {
    // SAFETY: the vector is installed as the kernel's local reschedule trap.
    unsafe {
        asm!("int {vector}", vector = const lapic::SELF_RESCHEDULE_VECTOR);
    }
}
