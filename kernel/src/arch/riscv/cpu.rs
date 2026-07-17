//!
//! # CPU Features/Trap Routines
//!
//! This module contains code for setting up the CPU, and handling
//! both hardware/software generated interrupts.
//!

use core::arch::asm;

core::arch::global_asm!(include_str!("trap.S"));

unsafe extern "C" {
    static rtrap_entry: u8;
    fn rthread_resume(frame: *const TrapFrame) -> !;
}

pub(crate) const CSR_SSTATUS: u16 = 0x0100;
pub(crate) const CSR_SIE: u16 = 0x0104;
pub(crate) const CSR_SIP: u16 = 0x0144;
const CSR_STVEC: u16 = 0x0105;
const SCAUSE_INTERRUPT: u64 = 1 << 63;
const SCAUSE_SUPERVISOR_SOFTWARE: u64 = 1;
const SCAUSE_SUPERVISOR_TIMER: u64 = 5;
const SCAUSE_INSTRUCTION_PAGE_FAULT: u64 = 12;
const SCAUSE_LOAD_PAGE_FAULT: u64 = 13;
const SCAUSE_STORE_PAGE_FAULT: u64 = 15;
pub(crate) const SSTATUS_SIE: u64 = 1 << 1;
const SSTATUS_SPIE: u64 = 1 << 5;
const SSTATUS_SPP: u64 = 1 << 8;
pub(crate) const SIE_SSIE: u64 = 1 << 1;
pub(crate) const SIE_STIE: u64 = 1 << 5;
const SIE_SEIE: u64 = 1 << 9;

/// Represents the trap frame saved onto the kernel stack during a trap.
///
/// **NOTE:** The layout and offsets MUST exactly match the assembly
/// routine `rtrap_entry`.
#[repr(C)]
pub struct TrapFrame {
    a0: u64,
    a1: u64,
    a2: u64,
    a3: u64,
    a4: u64,
    a5: u64,
    a6: u64,
    a7: u64,

    t0: u64,
    t1: u64,
    t2: u64,
    t3: u64,
    t4: u64,
    t5: u64,
    t6: u64,

    s0: u64,
    s1: u64,
    s2: u64,
    s3: u64,
    s4: u64,
    s5: u64,
    s6: u64,
    s7: u64,
    s8: u64,
    s9: u64,
    s10: u64,
    s11: u64,

    ra: u64,
    gp: u64,
    prev_sp: u64,
    prev_sscratch: u64,

    scause: u64,
    stval: u64,
    ip: u64,
    sstatus: u64,
    reserved: u64,
}

/// Initializes a trap frame for a brand-new kernel thread.
///
/// # Safety
///
/// `frame` must be valid for writes, properly aligned for [`TrapFrame`], and
/// point at memory reserved for the new thread's initial register state.
pub unsafe fn init_kernel_thread_frame(
    frame: *mut TrapFrame,
    stack_top: u64,
    ip: usize,
    arg0: usize,
    arg1: usize,
) {
    // SAFETY: the caller guarantees that `frame` is valid and exclusively
    // writable for a complete `TrapFrame`.
    unsafe {
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
}

/// Restores `frame` and enters the first scheduled kernel thread.
///
/// # Safety
///
/// `frame` must contain a valid saved kernel context produced by the trap
/// entry code or by [`init_kernel_thread_frame`].
pub unsafe fn start_first_thread(frame: *mut TrapFrame) -> ! {
    // SAFETY: upheld by this function's caller contract.
    unsafe { rthread_resume(frame) };
}

/// Refreshes architecture-specific per-CPU state in a thread frame.
///
/// # Safety
///
/// `frame` must be the saved context for the thread about to resume on the
/// current hart.
pub unsafe fn prepare_thread_frame(frame: *mut TrapFrame) {
    // SAFETY: the caller guarantees `frame` is valid and exclusively owned.
    unsafe {
        (*frame).prev_sscratch = crate::arch::thiscpu() as *const crate::sys::smp::CoreLocal as u64;
        (*frame).gp = read_gp();
        (*frame).sstatus |= SSTATUS_SPP | SSTATUS_SPIE;
        (*frame).sstatus &= !SSTATUS_SIE;
    }
}

/// Returns the saved instruction pointer from a trap frame.
pub fn trap_frame_ip(frame: &TrapFrame) -> u64 {
    frame.ip
}

fn read_gp() -> u64 {
    let value: u64;

    // SAFETY: this instruction only copies the current global-pointer register
    // into a declared output.
    unsafe {
        asm!(
            "mv {}, gp",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );
    }

    value
}

#[inline(always)]
pub(crate) unsafe fn rdcsr<const CSR_ADDR: u16>() -> u64 {
    let v: u64;

    // SAFETY: the caller chooses a CSR that is readable at supervisor level.
    unsafe {
        asm!(
            "csrr {output}, {csr_addr}",
            output = out(reg) v,
            csr_addr = const CSR_ADDR,
            options(nomem, nostack, preserves_flags)
        );
    }

    v
}

#[inline(always)]
pub(crate) unsafe fn wrcsr<const CSR_ADDR: u16>(val: u64) {
    // SAFETY: the caller chooses a writable CSR and a valid value.
    unsafe {
        asm!(
            "csrw {csr_addr}, {input}",
            csr_addr = const CSR_ADDR,
            input = in(reg) val,
            options(nomem, nostack, preserves_flags)
        );
    }
}

#[inline(always)]
pub(crate) unsafe fn set_csr_bits<const CSR_ADDR: u16>(mask: u64) {
    // SAFETY: the caller chooses a writable CSR and valid set mask.
    unsafe {
        asm!(
            "csrs {csr_addr}, {mask}",
            csr_addr = const CSR_ADDR,
            mask = in(reg) mask,
            options(nomem, nostack, preserves_flags)
        );
    }
}

#[inline(always)]
pub(crate) unsafe fn clear_csr_bits<const CSR_ADDR: u16>(mask: u64) {
    // SAFETY: the caller chooses a writable CSR and valid clear mask.
    unsafe {
        asm!(
            "csrc {csr_addr}, {mask}",
            csr_addr = const CSR_ADDR,
            mask = in(reg) mask,
            options(nomem, nostack, preserves_flags)
        );
    }
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
    // SAFETY: these supervisor CSRs are configured during per-hart bring-up
    // before interrupts or scheduling are enabled.
    unsafe {
        wrcsr::<CSR_STVEC>((&rtrap_entry as *const u8 as u64) & !0b11);
        clear_csr_bits::<CSR_SSTATUS>(SSTATUS_SIE | (1 << 19));
        wrcsr::<CSR_SIE>(SIE_SSIE | SIE_SEIE);
    }
}

/// Enables supervisor timer interrupts once the timer subsystem is ready.
pub fn enable_timer_interrupts() {
    // SAFETY: `sie.STIE` is writable in supervisor mode and the timer backend
    // has been initialized before this call.
    unsafe {
        set_csr_bits::<CSR_SIE>(SIE_STIE);
    }
}

/// Kernel trap handler.
///
/// All interrupts triggered start their journey here...
#[unsafe(no_mangle)]
extern "C" fn rtrap(frame: &mut TrapFrame) -> *mut TrapFrame {
    crate::sys::panic::halt_if_panicking();

    if frame.scause & SCAUSE_INTERRUPT != 0 {
        let _interrupt = crate::sys::smp::enter_interrupt_context();
        match frame.scause & !SCAUSE_INTERRUPT {
            SCAUSE_SUPERVISOR_TIMER => {
                crate::sys::clock::handle_local_timer_interrupt();
                return crate::sys::sched::trap_return(frame);
            }
            SCAUSE_SUPERVISOR_SOFTWARE => {
                // SAFETY: clearing the local SSIP bit acknowledges the
                // supervisor software interrupt currently being handled.
                unsafe {
                    clear_csr_bits::<CSR_SIP>(SIE_SSIE);
                }
                return crate::sys::sched::trap_return(frame);
            }
            _ => {}
        }
    }

    if frame.sstatus & SSTATUS_SPP == 0 {
        let access = match frame.scause {
            SCAUSE_INSTRUCTION_PAGE_FAULT => Some(crate::mem::FaultAccess::Execute),
            SCAUSE_LOAD_PAGE_FAULT => Some(crate::mem::FaultAccess::Read),
            SCAUSE_STORE_PAGE_FAULT => Some(crate::mem::FaultAccess::Write),
            _ => None,
        };
        if let Some(access) = access {
            crate::arch::irqset(true);
            let result =
                crate::mem::handle_current_fault(crate::mem::VirtAddr::new(frame.stval), access);
            crate::arch::irqset(false);
            if result.is_ok() {
                return frame;
            }
        }
    }

    panic!(
        "CPU trap triggered at IP=0x{:X}, stval=0x{:X}, cause=0x{:X}",
        frame.ip, frame.stval, frame.scause
    );
}
