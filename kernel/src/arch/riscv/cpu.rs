//!
//! # CPU Features/Trap Routines
//!
//! This module contains code for setting up the CPU, and handling
//! both hardware/software generated interrupts.
//!

use core::arch::asm;

core::arch::global_asm!(include_str!("trap.S"), include_str!("../../syscall.S"));

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
const SCAUSE_SUPERVISOR_EXTERNAL: u64 = 9;
const SCAUSE_INSTRUCTION_MISALIGNED: u64 = 0;
const SCAUSE_INSTRUCTION_ACCESS_FAULT: u64 = 1;
const SCAUSE_ILLEGAL_INSTRUCTION: u64 = 2;
const SCAUSE_BREAKPOINT: u64 = 3;
const SCAUSE_LOAD_MISALIGNED: u64 = 4;
const SCAUSE_LOAD_ACCESS_FAULT: u64 = 5;
const SCAUSE_STORE_MISALIGNED: u64 = 6;
const SCAUSE_STORE_ACCESS_FAULT: u64 = 7;
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
#[derive(Copy, Clone)]
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

impl TrapFrame {
    pub(crate) fn syscall_argument(&self, index: usize) -> u64 {
        match index {
            0 => self.a0,
            1 => self.a1,
            2 => self.a2,
            3 => self.a3,
            4 => self.a4,
            5 => self.a5,
            _ => panic!("riscv: invalid syscall argument index {index}"),
        }
    }

    pub(crate) fn is_user(&self) -> bool {
        self.sstatus & SSTATUS_SPP == 0
    }

    pub(crate) fn user_stack(&self) -> u64 {
        self.prev_sp
    }

    pub(crate) fn setup_signal_handler(
        &mut self,
        stack: u64,
        handler: u64,
        restorer: u64,
        signal: u64,
        info: u64,
        context: u64,
    ) {
        self.prev_sp = stack;
        self.ip = handler;
        self.a0 = signal;
        self.a1 = info;
        self.a2 = context;
        self.ra = restorer;
    }

    pub(crate) fn restore_signal(&mut self, saved: &Self) -> bool {
        if saved.sstatus & SSTATUS_SPP != 0
            || !(crate::mem::USER_ADDRESS_MIN..crate::mem::USER_ADDRESS_MAX).contains(&saved.ip)
            || !(crate::mem::USER_ADDRESS_MIN..crate::mem::USER_ADDRESS_MAX)
                .contains(&saved.prev_sp)
        {
            return false;
        }
        *self = *saved;
        self.sstatus = SSTATUS_SPIE;
        true
    }

    pub(crate) fn syscall_result(&self) -> i64 {
        self.a0 as i64
    }
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

/// Initializes a trap frame for a brand-new user thread.
///
/// # Safety
///
/// `frame` must be valid for writes, properly aligned for [`TrapFrame`], and
/// point at memory reserved for the new thread's initial register state.
pub unsafe fn init_user_thread_frame(frame: *mut TrapFrame, ip: u64, stack: u64) {
    // SAFETY: the caller guarantees that `frame` is valid and exclusively
    // writable for a complete `TrapFrame`.
    unsafe {
        *frame = TrapFrame {
            a0: 0,
            a1: 0,
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
            gp: 0,
            prev_sp: stack,
            prev_sscratch: 0,
            scause: 0,
            stval: 0,
            ip,
            sstatus: SSTATUS_SPIE,
            reserved: 0,
        };
    }
}

/// Initializes a child user frame from its parent's syscall frame.
///
/// # Safety
///
/// `frame` must be valid for writes and `parent` must be a live user frame
/// whose instruction pointer already advances past the `ecall`.
pub unsafe fn init_forked_user_thread_frame(frame: *mut TrapFrame, parent: &TrapFrame) {
    // SAFETY: the caller guarantees exclusive writable storage for `frame`.
    unsafe {
        *frame = *parent;
        (*frame).a0 = 0;
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
pub unsafe fn prepare_thread_frame(frame: *mut TrapFrame, thread_pointer: u64) {
    // SAFETY: the caller guarantees `frame` is valid and exclusively owned.
    unsafe {
        if (*frame).sstatus & SSTATUS_SPP == 0 {
            (*frame).prev_sscratch = thread_pointer;
        } else {
            (*frame).prev_sscratch =
                crate::arch::thiscpu() as *const crate::sys::smp::CoreLocal as u64;
            (*frame).gp = read_gp();
            (*frame).sstatus |= SSTATUS_SPP;
        }
        (*frame).sstatus |= SSTATUS_SPIE;
        (*frame).sstatus &= !SSTATUS_SIE;
    }
}

/// Updates the ring-0 exception stack for the current hart.
pub fn set_kernel_stack(_stack: u64) {}

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
    //   - Enable supervisor software interrupts in the 'sie' CSR. External
    //     delivery is enabled after a root interrupt controller registers.
    //   - Disable MXR (Make eXecutable Readable).
    //   - Keep SIE masked so bootstrap does not take interrupts until the
    //     scheduler hands control to the first thread context.
    //
    // SAFETY: these supervisor CSRs are configured during per-hart bring-up
    // before interrupts or scheduling are enabled.
    unsafe {
        wrcsr::<CSR_STVEC>((&rtrap_entry as *const u8 as u64) & !0b11);
        clear_csr_bits::<CSR_SSTATUS>(SSTATUS_SIE | (1 << 19));
        wrcsr::<CSR_SIE>(SIE_SSIE);
    }
}

/// Enables or disables supervisor external interrupts on the current hart.
pub(crate) fn set_external_interrupts(enable: bool) {
    // SAFETY: SIE.SEIE is the hart-local supervisor external interrupt enable
    // bit and this operation preserves all other interrupt classes.
    unsafe {
        if enable {
            set_csr_bits::<CSR_SIE>(SIE_SEIE);
        } else {
            clear_csr_bits::<CSR_SIE>(SIE_SEIE);
        }
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
            SCAUSE_SUPERVISOR_EXTERNAL => {
                let cpu = crate::arch::thiscpu();
                let platform_id = crate::sys::smp::platform_id(cpu.id)
                    .expect("riscv: current hart is absent from SMP topology");
                let outcome = crate::driver::irq::dispatch_external(cpu.id as u32, platform_id);
                if outcome.handled {
                    return if outcome.reschedule {
                        crate::sys::sched::trap_return(frame)
                    } else {
                        frame
                    };
                }
                set_external_interrupts(false);
                return frame;
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

        if frame.scause & SCAUSE_INTERRUPT == 0 {
            let signal = match frame.scause {
                SCAUSE_ILLEGAL_INSTRUCTION => crate::proc::signal::SIGILL,
                SCAUSE_BREAKPOINT => crate::proc::signal::SIGTRAP,
                SCAUSE_INSTRUCTION_MISALIGNED
                | SCAUSE_INSTRUCTION_ACCESS_FAULT
                | SCAUSE_LOAD_MISALIGNED
                | SCAUSE_LOAD_ACCESS_FAULT
                | SCAUSE_STORE_MISALIGNED
                | SCAUSE_STORE_ACCESS_FAULT => crate::proc::signal::SIGBUS,
                SCAUSE_INSTRUCTION_PAGE_FAULT
                | SCAUSE_LOAD_PAGE_FAULT
                | SCAUSE_STORE_PAGE_FAULT => crate::proc::signal::SIGSEGV,
                _ => crate::proc::signal::SIGILL,
            };
            crate::proc::signal::send_current(signal);
            return frame;
        }
    }

    let (cpu_id, tid) = crate::arch::thiscpu_opt()
        .map(|cpu| (cpu.id, cpu.current_thread))
        .unwrap_or((usize::MAX, 0));
    crate::mem::kstack::report_guard_fault(frame.stval, cpu_id, tid);

    panic!(
        "CPU trap triggered at IP=0x{:X}, stval=0x{:X}, cause=0x{:X}",
        frame.ip, frame.stval, frame.scause
    );
}

/// Finalizes a syscall after its table-selected handler returns.
#[unsafe(no_mangle)]
extern "C" fn rsyscall_return(frame: &mut TrapFrame) -> *mut TrapFrame {
    // SAFETY: `frame` is the current thread's live user trap frame.
    unsafe {
        prepare_thread_frame(frame, crate::proc::current_thread_pointer());
    }
    crate::sys::sched::trap_return(frame)
}
