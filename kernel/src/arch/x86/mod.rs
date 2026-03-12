//!
//! # x86_64 Subsystem
//!
//! Implements a support interface for the kernel to interact with the x86_64
//! platform. Drivers for on-chip devices, such as the TSC and APIC are
//! provided by this module aswell.
//!
//! Code in this module often references the *Intel SDM* for register references
//! and ISA semantics.
//!
//! *You may download the SDM [here.](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)*
//!

use x86_64::addr::VirtAddr;
use x86_64::instructions::interrupts;
use x86_64::instructions::{hlt, interrupts as x86_interrupts, port::*};
use x86_64::registers::model_specific::{GsBase, KernelGsBase};

use crate::sys::{debug, smp::CoreLocal};

pub mod cpu;
pub mod lapic;
pub mod paging;
pub mod timer;

/// BSP's core local context.
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
    let line = unsafe { core::slice::from_raw_parts(buf, buflen) };

    let mut dbgcon_e9: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0xE9);
    let mut status: PortGeneric<u8, ReadOnlyAccess> = PortReadOnly::new(0x3FD);
    let mut com1: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3F8);

    for &byte in line {
        unsafe {
            dbgcon_e9.write(byte);

            while status.read() & 0x20 == 0 {}
            com1.write(byte);
        }
    }

    unsafe {
        dbgcon_e9.write(b'\n');

        while status.read() & 0x20 == 0 {}
        com1.write(b'\n');
    }
}

/// Initializes the debug console for printing.
///
/// On x86_64, QEMU's debugcon is pre-configured, so we simply
/// set COM1 (16550 UART) to 9600 9600 8N1.
fn dbgcon_init() {
    let mut ier: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3F9);
    let mut lcr: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3FB);
    let mut dll: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3F8);
    let mut dlm: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3F9);
    let mut fcr: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3FA);
    let mut mcr: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x3FC);

    unsafe {
        // Disable interrupts.
        ier.write(0x00);
        // Enable DLAB.
        lcr.write(0x80);
        // Divisor = 12 (9600 baud with 1.8432 MHz clock).
        dll.write(0x0C);
        dlm.write(0x00);
        // 8 bits, no parity, one stop bit.
        lcr.write(0x03);
        // Enable FIFO, clear TX/RX queues.
        fcr.write(0x07);
        // DTR | RTS | OUT2.
        mcr.write(0x0B);
    }
}

/// Performs early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {
    dbgcon_init();
    debug::register_sink(dbgcon_write);

    set_core_local(&raw const BSP_CORE_LOCAL);
    let feats = unsafe { cpu::enable_features() };
    thiscpu().platform.feats = feats;
}

/// Performs post-memory architecture initialization.
pub fn init() {
    timer::init();
}

/// Performs per-CPU initialization for a secondary core.
pub fn init_secondary(core_local: *const CoreLocal) {
    set_core_local(core_local);
    let feats = unsafe { cpu::enable_features() };
    thiscpu().platform.feats = feats;
    timer::init_secondary();
}

/// Returns core local context.
///
/// On the x86_64 platform, kernel core-local data is
/// addressed through the GS base registers.
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
    thiscpu_opt().expect("x86: thiscpu called before GS base was initialized")
}

/// Returns core local context if it is initialized.
#[inline(always)]
pub fn thiscpu_opt() -> Option<&'static mut CoreLocal> {
    let mut base = GsBase::read();
    if base.is_null() {
        // AP entry may arrive with the kernel value still parked in
        // IA32_KERNEL_GS_BASE until the first SWAPGS path runs.
        base = KernelGsBase::read();
    }

    if base.is_null() {
        return None;
    }

    Some(unsafe { &mut *base.as_mut_ptr::<CoreLocal>() })
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

///
/// Pauses CPU execution and waits for interrupts.
///
/// **If interrupts are disabled, this will result in an infinite loop.**
///
pub fn wfi() -> ! {
    loop {
        hlt();
    }
}

/// Sends a reschedule IPI to `cpu_id`.
pub fn send_ipi(cpu_id: usize) {
    let lapic_id = crate::sys::smp::platform_id(cpu_id).expect("x86: invalid CPU ID for IPI");
    lapic::send_ipi(lapic_id as u32);
}

/// Forces the current CPU through the scheduler trap path.
pub fn reschedule() {
    interrupts::int3();
}
