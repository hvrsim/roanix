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
use x86_64::instructions::{hlt, port::PortWriteOnly};
use x86_64::registers::segmentation::{Segment64, GS};

use crate::sys::smp::CoreLocal;

pub mod cpu;

/// BSP's core local context.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);

/// Performs early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {
    let feats = cpu::enable_features();

    set_core_local(VirtAddr::from_ptr(&raw const BSP_CORE_LOCAL));
    thiscpu().supported_feats = feats;
}

///
/// Writes a single character to the emulator debug port.
///
/// Utilizes the bochs/QEMU debug port (port 0xE9) which is unallocated on
/// real hardware. This means writes to the port are ignored if we aren't
/// running on bochs or QEMU.
///
/// *NOTE: to make output from this console visible, pass `-debugcon stdio` to
/// QEMU flags, like so:*
///
/// ```bash
/// $ QEMUFLAGS="... -debugcon stdio" make run-bios
/// ```
///
#[inline(always)]
pub fn debug_putc(byte: u8) {
    let mut port = PortWriteOnly::new(0xE9);
    unsafe { port.write(byte) }
}

/// Pauses CPU execution and waits for interrupts.
///
/// **If interrupts are disabled, this will result in an infinite loop.**
pub fn hcf() -> ! {
    loop {
        hlt();
    }
}

/// Returns core local context.
///
/// On the x86_64 platform, kernel core local data is
/// stored in the GS segment register.
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
    let base = GS::read_base();
    assert!(!base.is_null());

    unsafe { &mut *base.as_mut_ptr::<CoreLocal>() }
}

/// Sets the core local pointer.
///
/// Writes the provided core local pointer into the GS
/// segment register.
///
/// **This function can only be called once per core. Further
/// calls may result in a panic!**
pub fn set_core_local(ptr: VirtAddr) {
    assert!(GS::read_base().is_null());

    unsafe {
        GS::write_base(ptr);
    }
}
