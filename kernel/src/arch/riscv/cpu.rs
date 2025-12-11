//!
//! # CPU Features/Trap Routines
//!
//! This module contains code for setting up the CPU, and handling
//! both hardware/software generated interrupts.
//!
//!

use core::arch::asm;

core::arch::global_asm!(include_str!("trap.S"));

extern "C" {
    static rtrap_entry: u8;
}

pub const SSTATUS: u16 = 0x0100;
pub const SIE: u16 = 0x0104;
pub const STVEC: u16 = 0x0105;

/// Represents the trap frame saved onto the kernel stack during a trap.
///
/// **NOTE:** The layout and offsets MUST exactly match the assembly
/// routine `rtrap_entry`.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
pub struct TrapFrame {
    pub a0: u64,
    pub a1: u64,
    pub a2: u64,
    pub a3: u64,
    pub a4: u64,
    pub a5: u64,
    pub a6: u64,
    pub a7: u64,

    pub t0: u64,
    pub t1: u64,
    pub t2: u64,
    pub t3: u64,
    pub t4: u64,
    pub t5: u64,
    pub t6: u64,

    pub s0: u64,
    pub s1: u64,
    pub s2: u64,
    pub s3: u64,
    pub s4: u64,
    pub s5: u64,
    pub s6: u64,
    pub s7: u64,
    pub s8: u64,
    pub s9: u64,
    pub s10: u64,
    pub s11: u64,

    pub ra: u64,
    pub gp: u64,
    pub prev_sp: u64,
    pub prev_sscratch: u64,

    pub scause: u64,
    pub stval: u64,
    pub ip: u64,
    pub sstatus: u64,
}

/// Returns the contents of the riscv CSR specified in `CSR_ADDR`.
#[inline]
pub unsafe fn rdcsr<const CSR_ADDR: u16>() -> u64 {
    let v: u64;

    asm!(
        "csrr {output}, {csr_addr}",
        output = out(reg) v,
        csr_addr = const CSR_ADDR,
        options(nomem, nostack, preserves_flags)
    );

    v
}

/// Writes `val` to the riscv CSR specified in `CSR_ADDR`.
#[inline]
pub unsafe fn wrcsr<const CSR_ADDR: u16>(val: u64) {
    asm!(
        "csrw {csr_addr}, {input}",
        csr_addr = const CSR_ADDR,
        input = in(reg) val,
        options(nomem, nostack, preserves_flags)
    );
}

/// Configures CPU features and control registers.
pub fn enable_features() {
    //
    // Setup the CPU to a working (and secure) state
    //
    // We accomplish this by doing the following:
    //   - Set the trap vector, and the interrupt mode to
    //     direct delivery.
    //   - Enable software interrupts only in the 'sie' CSR.
    //   - Disable MXR (Make eXecutable Readable)
    //   - Finally, set the SIE bit in the 'sstatus' CSR.
    //
    unsafe {
        wrcsr::<STVEC>((&rtrap_entry as *const u8 as u64) & !0b11u64);
        wrcsr::<SSTATUS>((rdcsr::<SSTATUS>() | 0x2) & !(1 << 19));
        wrcsr::<SIE>(0x202);
    }
}

/// Kernel trap handler.
///
/// All interrupts triggered start their journey here...
#[no_mangle]
extern "C" fn rtrap(frame: &mut TrapFrame) {
    panic!(
        "CPU trap triggered at IP=0x{:X}, cause=0x{:X}",
        frame.ip, frame.scause
    );
}
