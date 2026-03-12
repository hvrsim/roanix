//!
//! # CPU Features/Trap Routines
//!
//! This module contains code for setting up the CPU, and handling
//! both hardware/software generated interrupts.
//!

use core::arch::asm;

core::arch::global_asm!(include_str!("trap.S"));

extern "C" {
    static rtrap_entry: u8;
    fn rthread_resume(frame: *const TrapFrame) -> !;
}

pub const CSR_SSTATUS: u16 = 0x0100;
pub const CSR_SIE: u16 = 0x0104;
pub const CSR_SIP: u16 = 0x0144;
pub const CSR_STVEC: u16 = 0x0105;
const SCAUSE_INTERRUPT: u64 = 1 << 63;
const SCAUSE_BREAKPOINT: u64 = 3;
const SCAUSE_SUPERVISOR_SOFTWARE: u64 = 1;
const SCAUSE_SUPERVISOR_TIMER: u64 = 5;
const SSTATUS_SIE: u64 = 1 << 1;
const SSTATUS_SPIE: u64 = 1 << 5;
const SSTATUS_SPP: u64 = 1 << 8;
const SIE_SSIE: u64 = 1 << 1;
const SIE_STIE: u64 = 1 << 5;
const SIE_SEIE: u64 = 1 << 9;

/// Represents the trap frame saved onto the kernel stack during a trap.
///
/// **NOTE:** The layout and offsets MUST exactly match the assembly
/// routine `rtrap_entry`.
#[repr(C)]
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
    pub reserved: u64,
}

/// Initializes a trap frame for a brand-new kernel thread.
pub unsafe fn init_kernel_thread_frame(
    frame: *mut TrapFrame,
    stack_top: u64,
    ip: usize,
    arg0: usize,
    arg1: usize,
) {
    *frame = TrapFrame {
        a0: arg0 as u64,
        a1: arg1 as u64,
        a2: 0,
        a3: 0,
        a4: 0,
        a5: 0,
        a6: 0,
        a7: 0,
        t0: 0,
        t1: 0,
        t2: 0,
        t3: 0,
        t4: 0,
        t5: 0,
        t6: 0,
        s0: 0,
        s1: 0,
        s2: 0,
        s3: 0,
        s4: 0,
        s5: 0,
        s6: 0,
        s7: 0,
        s8: 0,
        s9: 0,
        s10: 0,
        s11: 0,
        ra: 0,
        gp: read_gp(),
        prev_sp: stack_top,
        prev_sscratch: 0,
        scause: 0,
        stval: 0,
        ip: ip as u64,
        sstatus: SSTATUS_SPP | SSTATUS_SPIE,
        reserved: 0,
    };
}

/// Restores `frame` and enters the first scheduled kernel thread.
pub unsafe fn start_first_thread(frame: *mut TrapFrame) -> ! {
    rthread_resume(frame);
}

/// Refreshes architecture-specific per-CPU state in a thread frame.
pub unsafe fn prepare_thread_frame(frame: *mut TrapFrame) {
    (*frame).prev_sscratch = crate::arch::thiscpu() as *mut crate::sys::smp::CoreLocal as u64;
    (*frame).gp = read_gp();
    (*frame).sstatus |= SSTATUS_SPP | SSTATUS_SPIE;
    (*frame).sstatus &= !SSTATUS_SIE;
}

fn read_gp() -> u64 {
    let value: u64;

    unsafe {
        asm!(
            "mv {}, gp",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );
    }

    value
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
    //   - Set the trap vector address, and the interrupt mode to direct.
    //   - Enable supervisor software and external interrupts in the 'sie' CSR.
    //   - Disable MXR (Make eXecutable Readable).
    //   - Keep SIE masked so bootstrap does not take interrupts until the
    //     scheduler hands control to the first thread context.
    //
    unsafe {
        wrcsr::<CSR_STVEC>((&rtrap_entry as *const u8 as u64) & !0b11);
        wrcsr::<CSR_SSTATUS>(rdcsr::<CSR_SSTATUS>() & !(SSTATUS_SIE | (1 << 19)));
        wrcsr::<CSR_SIE>(SIE_SSIE | SIE_SEIE);
    }
}

/// Enables supervisor timer interrupts once the timer subsystem is ready.
pub fn enable_timer_interrupts() {
    unsafe {
        wrcsr::<CSR_SIE>(rdcsr::<CSR_SIE>() | SIE_STIE);
    }
}

/// Kernel trap handler.
///
/// All interrupts triggered start their journey here...
#[no_mangle]
extern "C" fn rtrap(frame: &mut TrapFrame) -> *mut TrapFrame {
    if frame.scause & SCAUSE_INTERRUPT != 0 {
        match frame.scause & !SCAUSE_INTERRUPT {
            SCAUSE_SUPERVISOR_TIMER => {
                crate::arch::timer::handle_interrupt();
                return crate::sys::sched::trap_return(frame);
            }
            SCAUSE_SUPERVISOR_SOFTWARE => {
                unsafe {
                    wrcsr::<CSR_SIP>(rdcsr::<CSR_SIP>() & !SIE_SSIE);
                }
                return crate::sys::sched::trap_return(frame);
            }
            _ => {}
        }
    }

    if frame.scause == SCAUSE_BREAKPOINT {
        frame.ip = frame.ip.wrapping_add(4);
        return crate::sys::sched::trap_return(frame);
    }

    panic!(
        "CPU trap triggered at IP=0x{:X}, stval=0x{:X}, cause=0x{:X}",
        frame.ip, frame.stval, frame.scause
    );
}
