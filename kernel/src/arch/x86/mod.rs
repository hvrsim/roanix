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
use x86_64::instructions::{hlt, port::*};
use x86_64::registers::segmentation::{Segment64, GS};

use crate::sys::smp::CoreLocal;

pub mod cpu;

/// BSP's core local context.
static mut BSP_CORE_LOCAL: CoreLocal = CoreLocal::new(0);

///
/// Writes debug messages to the current debug sink.
///
/// On QEMU/bochs, the debug sink is port 0xE9 (unallocated on real hardware).
///
/// On real hardware, the debug sink is the PC serial port (COM1).
///
/// By reading from port `0xE9`, you can test for the presence of the
/// QEMU/bochs debug port. If the debug port is deemed unusable/missing, then
/// the debug sink is set to COM1.
///
/// *NOTE: to make output from this console visible, pass `-debugcon stdio` to
/// QEMU flags, like so:*
///
/// ```bash
/// $ QEMUFLAGS="... -debugcon stdio" make run-bios
/// ```
///
pub struct DebugConsole {
    /// x86 port to write characters to.
    port: u16,

    /// x86 port for checking buffer status.
    status: u16,
}

impl DebugConsole {
    /// Creates a new instance of [`DebugConsole`]. Also selects debug port.
    pub fn new() -> Self {
        let mut e9: PortGeneric<u8, ReadOnlyAccess> = PortReadOnly::new(0xE9);

        if unsafe { e9.read() as u8 } == 0xE9 {
            DebugConsole {
                port: 0xE9,
                status: 0xE9,
            }
        } else {
            DebugConsole {
                port: 0x3F8,
                status: 0x3FD,
            }
        }
    }

    /// Writes a single character to the chosen x86 port.
    #[inline(always)]
    pub fn write(&self, byte: u8) {
        let mut port: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(self.port);
        let mut status: PortGeneric<u8, ReadOnlyAccess> = PortReadOnly::new(self.status);

        unsafe {
            // HACK: port 0xE9 returns a non-zero value, so it passes this check.
            while status.read() & 0x20 == 0 {}

            port.write(byte);
        }
    }
}

/// Performs early CPU initialization.
///
/// Enumerates and enables CPU features, also sets trap handlers for early panic handling.
pub fn early() {
    let feats = unsafe { cpu::enable_features() };

    set_core_local(&raw const BSP_CORE_LOCAL);
    thiscpu().platform.feats = feats;
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
pub fn set_core_local(ptr: *const CoreLocal) {
    assert!(GS::read_base().is_null());

    unsafe {
        GS::write_base(VirtAddr::from_ptr(ptr));
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
