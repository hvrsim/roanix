//!
//! # CPU Features/Instructions
//!
//! Code in this module enables and enumerates core CPU features. Documentation
//! for specific features are listed below, along with a reference to the SDM.
//!
//! Note that certain features like syscalls and SSE2 aren't documented because
//! these features are a core part of the x86_64 ISA.
//!
//! ## Global Pages (Required)
//! Global Pages are used for mapping the kernel memmory and HHDM, since both
//! regions of virtual memory are shared between every process. Global pages are
//! not flushed from the translation-lookaside buffer (TLB) on a task switch or
//! a write to register CR3.
//!
//! Reference: *Intel SDM Volume 3A, Section 5.10*
//!
//! ## SMEP
//! SMEP prevents kernel threads from executing code which is user accessible.
//! As you can imagine, this is a pretty helpful security feature and is enabled
//! whenever supported.
//!
//! Reference: *Intel SDM Volume 3A, Section 5.6*
//!
//! ## SMAP
//! When SMAP is enabled, any attempt to access user-space memory while running in
//! a privileged mode will lead to a page fault. If you are wondering how user IO
//! is done with SMAP active, it isn't. SMAP is disabled while user IO is in progress,
//! then enabled once again. State changes here happen through the AC bit in RFLAGS.
//!
//! Reference: *Intel SDM Volume 3A, Section 5.6*
//!
//! ## PCID
//! Basically x86_64's version of a ASID, limited to the lower 12-bits of CR3. ALways
//! useful to have ASIDs on a memory platform, so we enable them here.
//!
//! Reference: *Intel SDM Volume 3A, Section 5.10.1*
//!
//! ## UMIP
//! UMIP prevents userspace code from executing supervisor instructions, such as `sgdt`
//! and `sidt`.
//!
//! Reference: *Intel SDM Volume 3A, Section 2.5*
//!
//! ## Intel CET
//!
//! CET is a new Intel processor feature that blocks return/jump-oriented programming
//! attacks. Roanix enables both the *Shadow Stacks* and *IBRS* features when present.
//!
//! Reference: *Intel SDM Volume 1, Section 18*
//!

use core::{mem::size_of, ptr};

use bitflags::bitflags;
use log::{info, warn};
use raw_cpuid::CpuId;
use x86_64::registers::control::*;
use x86_64::registers::model_specific::{Efer, EferFlags};

use crate::sys::sync::Once;

core::arch::global_asm!(include_str!("trap.S"), options(att_syntax));

unsafe extern "C" {
    fn vstub0();
    fn rthread_resume(frame: *const TrapFrame) -> !;
}

static KERNEL_GDT: Gdt = Gdt::new();
// SAFETY: descriptor tables are initialized during per-CPU early boot before
// concurrent Rust code starts executing on that CPU.
static mut KERNEL_IDT: [Idt; 256] = [Idt::new(); 256];
static BSP_STARTUP: Once<()> = Once::new();

bitflags! {
    /// Bitmap of supported x86 extensions.
    pub struct CpuFeatures: u32 {
        /// Supervisor-mode execution prevention.
        const SMEP = 0b0001;
        /// Supervisor-mode access prevention.
        const SMAP = 0b0010;
        /// Process-context identifiers.
        const PCID = 0b0100;
        /// Control-flow enforcement shadow stacks.
        const CET_SS = 0b1000;
    }
}

/// Structure used by the `lgdt`/`lidt` instructions.
#[repr(C, packed(1))]
struct Descriptor {
    /// The size of the descriptor table minus 1.
    limit: u16,
    /// The linear base address of the GDT or IDT.
    base: u64,
}

/// Representation of the x86_64 Global Descriptor Table.
#[repr(C, packed(1))]
struct Gdt {
    /// GDT entries as raw u64s.
    entries: [u64; 9],
}

/// Representation of the x86_64 Interrupt Descriptor Table.
#[repr(C, packed(1))]
#[derive(Copy, Clone)]
struct Idt {
    /// Low 2 bytes of handler address.
    offset_low: u16,

    /// Segment selector.
    selector: u16,

    /// IST index.
    ist: u8,

    /// Type and attributes (P, DPL, S, Gate Type)
    flags: u8,

    /// Mid 2 bytes of handler address.
    offset_mid: u16,

    /// High 4 bytes of handler address.
    offset_high: u32,

    // IDT reserved field.
    reserved: u32,
}

/// Represents the trap frame saved onto the kernel stack during a trap.
///
/// **NOTE:** The layout and offsets MUST exactly match the assembly
/// routine `rtrap_entry`.
#[repr(C)]
pub struct TrapFrame {
    rax: u64,
    rbx: u64,
    rcx: u64,
    rdx: u64,
    rsi: u64,
    rdi: u64,
    rbp: u64,
    r8: u64,
    r9: u64,
    r10: u64,
    r11: u64,
    r12: u64,
    r13: u64,
    r14: u64,
    r15: u64,

    vec: u64,
    ec: u64,
    ip: u64,
    cs: u64,
    rflags: u64,
    sp: u64,
    ss: u64,
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
        ptr::write(
            frame,
            TrapFrame {
                rax: 0,
                rbx: 0,
                rcx: 0,
                rdx: 0,
                rsi: arg1 as u64,
                rdi: arg0 as u64,
                rbp: 0,
                r8: 0,
                r9: 0,
                r10: 0,
                r11: 0,
                r12: 0,
                r13: 0,
                r14: 0,
                r15: 0,
                vec: 0,
                ec: 0,
                ip: ip as u64,
                cs: 0x28,
                // IF=1 and bit 1 must remain set for architectural validity.
                rflags: (1 << 9) | (1 << 1),
                sp: stack_top,
                ss: 0x30,
            },
        );
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
/// `frame` must point to the current thread's saved trap frame. x86_64 does
/// not currently need to mutate it before resume.
pub unsafe fn prepare_thread_frame(_frame: *mut TrapFrame) {}

/// Returns the saved instruction pointer from a trap frame.
pub fn trap_frame_ip(frame: &TrapFrame) -> u64 {
    frame.ip
}

impl Gdt {
    /// Creates a new GDT structure.
    const fn new() -> Self {
        Self {
            entries: [
                0x0000_0000_0000_0000,
                0x0000_9a00_0000_ffff,
                0x0000_9300_0000_ffff,
                0x00cf_9a00_0000_ffff,
                0x00cf_9300_0000_ffff,
                0x00af_9b00_0000_ffff,
                0x00af_9300_0000_ffff,
                0x00af_fa00_0000_ffff,
                0x008f_f200_0000_ffff,
            ],
        }
    }

    /// Loads the GDT structure into the CPU registers.
    unsafe fn load(&self) {
        let gdtr = Descriptor {
            limit: (size_of::<Self>() - 1) as u16,
            base: (self as *const Self) as u64,
        };

        let gdtr_ptr: u64 = &gdtr as *const Descriptor as u64;

        // SAFETY: `gdtr` points to this static GDT and the selectors match its
        // kernel code/data entries.
        unsafe {
            core::arch::asm!(
                "lgdt ({gdtr})",
                "push $0x28",
                "lea 1f(%rip), %rax",
                "push %rax",
                "lretq",
                "1:",

                "mov $0x30, %eax",
                "mov %eax, %ds",
                "mov %eax, %es",
                "mov %eax, %fs",
                "mov %eax, %gs",
                "mov %eax, %ss",

                gdtr = in(reg) gdtr_ptr,

                // Clobber RAX since it is used to load segment registers.
                out("rax") _,
                options(att_syntax, preserves_flags)
            );
        }
    }
}

impl Idt {
    /// Creates a new IDT entry.
    const fn new() -> Self {
        Idt {
            offset_low: 0,
            selector: 0,
            ist: 0,
            flags: 0,
            offset_mid: 0,
            offset_high: 0,
            reserved: 0,
        }
    }

    /// Creates a new IDT entry given a handler address and IST index.
    const fn from_address(ptr: u64, ist: u8) -> Self {
        Idt {
            offset_low: (ptr & 0xFFFF) as u16,
            offset_mid: ((ptr >> 16) & 0xFFFF) as u16,
            offset_high: (ptr >> 32) as u32,
            selector: 0x28,
            flags: 0x8e,
            ist,
            reserved: 0,
        }
    }
}

/// Checks for and enables the x86 CPU features required by the kernel.
pub fn enable_features() -> CpuFeatures {
    let cpuid = CpuId::new();
    let mut cpufeats = CpuFeatures::empty();

    // Enable the `syscall` and `sysret` instructions, as well as NX bit.
    unsafe {
        // SAFETY: `enable_features` is only called during CPU bring-up before
        // concurrent execution starts on this core.
        Efer::write(
            Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS | EferFlags::NO_EXECUTE_ENABLE,
        );
    }

    // Set the groundwork for SIMD/FP instructions by disabling
    // emulation and activating CR0.MP. Also enable Write Protect
    // because Intel CET requires it.
    unsafe {
        // SAFETY: `enable_features` is only called during CPU bring-up before
        // concurrent execution starts on this core.
        Cr0::write(
            (Cr0::read() & !Cr0Flags::EMULATE_COPROCESSOR)
                | Cr0Flags::MONITOR_COPROCESSOR
                | Cr0Flags::WRITE_PROTECT,
        );
    }

    // Ensure key CR4 features are supported.
    let feats = cpuid
        .get_feature_info()
        .expect("cpu: unable to query for features with CPUID!");

    let ext_feats = cpuid
        .get_extended_feature_info()
        .expect("cpu: unable to query for extended features with CPUID!");

    BSP_STARTUP.call_once(|| {
        if let Some(brand_string) = cpuid.get_processor_brand_string() {
            info!("cpu: model name \"{}\"", brand_string.as_str());
        }

        if !ext_feats.has_fsgsbase() {
            warn!("cpu: {{FS/GS}}BASE instructions are not supported!");
        }

        if !ext_feats.has_smep() {
            warn!("cpu: SMEP not supported!");
        }

        if !ext_feats.has_smap() {
            warn!("cpu: SMAP not supported!");
        }

        // SAFETY: BSP startup is serialized by `BSP_STARTUP`, so the static
        // IDT is initialized exactly once before it is loaded.
        unsafe { init_idt_entries() };
    });

    if !feats.has_pge() {
        panic!("cpu: global pages are not supported!");
    }

    // Enable Global Pages and SSE instructions.
    let mut bits = Cr4Flags::PAGE_GLOBAL | Cr4Flags::OSFXSR | Cr4Flags::OSXMMEXCPT_ENABLE;

    // FSGSBASE is optional in kernel mode, but we enable it when available
    // so userspace can use {RD,WR}{FS,GS}BASE instructions.
    if ext_feats.has_fsgsbase() {
        bits |= Cr4Flags::FSGSBASE;
    }

    // Enable SMEP/SMAP (if supported)
    if ext_feats.has_smep() {
        bits |= Cr4Flags::SUPERVISOR_MODE_EXECUTION_PROTECTION;
        cpufeats |= CpuFeatures::SMEP;
    }

    if ext_feats.has_smap() {
        bits |= Cr4Flags::SUPERVISOR_MODE_ACCESS_PREVENTION;
        cpufeats |= CpuFeatures::SMAP;
    }

    // Only activate PCIDs if the `invpcid` instruction is supported.
    if feats.has_pcid() && ext_feats.has_invpcid() {
        bits |= Cr4Flags::PCID;
        cpufeats |= CpuFeatures::PCID;
    }

    // Enable UMIP (if supported)
    if ext_feats.has_umip() {
        bits |= Cr4Flags::USER_MODE_INSTRUCTION_PREVENTION;
    }

    // // Enable Intel CET (if supported)
    // if ext_feats.has_cet_ss() {
    //     bits |= Cr4Flags::CONTROL_FLOW_ENFORCEMENT;
    //     cpufeats |= CpuFeatures::CET_SS;
    // }

    // SAFETY: `enable_features` is only called during CPU bring-up before
    // concurrent execution starts on this core.
    unsafe {
        Cr4::write(Cr4::read() | bits);
    }
    // SAFETY: both descriptor tables are fully initialized and remain static
    // for the lifetime of the kernel.
    unsafe {
        KERNEL_GDT.load();
        load_idt();
    }

    cpufeats
}

/// Kernel trap handler.
///
/// All interrupts triggered start their journey here...
#[unsafe(no_mangle)]
extern "C" fn rtrap(frame: &mut TrapFrame) -> *mut TrapFrame {
    crate::sys::panic::halt_if_panicking();
    if frame.vec == 14 && frame.cs & 3 == 3 {
        let access = if frame.ec & (1 << 4) != 0 {
            crate::mem::FaultAccess::Execute
        } else if frame.ec & (1 << 1) != 0 {
            crate::mem::FaultAccess::Write
        } else {
            crate::mem::FaultAccess::Read
        };
        let address = crate::mem::VirtAddr::new(Cr2::read_raw());
        crate::arch::irqset(true);
        let result = crate::mem::handle_current_fault(address, access);
        crate::arch::irqset(false);
        if result.is_ok() {
            return frame;
        }
    }
    if frame.vec < 32 {
        let (cpu_id, tid) = crate::arch::thiscpu_opt()
            .map(|cpu| (cpu.id, cpu.current_thread))
            .unwrap_or((usize::MAX, 0));
        log::error!(
            "x86/trap: exception ip=0x{:X} vec=0x{:X} ec=0x{:X} cr2=0x{:X} cs=0x{:X} ss=0x{:X} sp=0x{:X} rflags=0x{:X} cpu={} tid={}",
            frame.ip,
            frame.vec,
            frame.ec,
            Cr2::read_raw(),
            frame.cs,
            frame.ss,
            frame.sp,
            frame.rflags,
            cpu_id,
            tid,
        );
        panic!("x86/trap: exception while returning from kernel trap");
    }

    let _interrupt = crate::sys::smp::enter_interrupt_context();

    let next = match crate::arch::timer::handle_interrupt(frame.vec) {
        crate::arch::timer::InterruptAction::Reschedule => crate::sys::sched::trap_return(frame),
        crate::arch::timer::InterruptAction::Handled => frame,
        crate::arch::timer::InterruptAction::Unhandled => {
            let (cpu_id, tid) = crate::arch::thiscpu_opt()
                .map(|cpu| (cpu.id, cpu.current_thread))
                .unwrap_or((usize::MAX, 0));
            panic!(
                "CPU trap triggered at IP=0x{:X}, vec=0x{:X}, ec=0x{:X}, CR2=0x{:X}, cs=0x{:X}, ss=0x{:X}, sp=0x{:X}, rflags=0x{:X}, cpu={}, tid={}",
                frame.ip,
                frame.vec,
                frame.ec,
                Cr2::read_raw(),
                frame.cs,
                frame.ss,
                frame.sp,
                frame.rflags,
                cpu_id,
                tid,
            );
        }
    };

    // Validate the frame selected for resume before returning to assembly.
    // Under sustained scheduler stress, catching corrupt selectors here
    // provides far better diagnostics than letting `iretq` fault with partial
    // context.
    // SAFETY: trap dispatch returns either the current frame or a scheduler
    // frame that remains live until assembly resumes it.
    let next_ref = unsafe { &*next };
    let next_ip = next_ref.ip;
    let next_cs = next_ref.cs;
    let next_ss = next_ref.ss;
    let next_sp = next_ref.sp;
    let next_rflags = next_ref.rflags;

    if next_cs != 0x28 && next_cs != 0x3B {
        let (cpu_id, tid) = crate::arch::thiscpu_opt()
            .map(|cpu| (cpu.id, cpu.current_thread))
            .unwrap_or((usize::MAX, 0));
        panic!(
            "x86/trap: invalid next frame ptr=0x{:X} ip=0x{:X} cs=0x{:X} ss=0x{:X} sp=0x{:X} rflags=0x{:X} vec=0x{:X} cpu={} tid={}",
            next as usize as u64,
            next_ip,
            next_cs,
            next_ss,
            next_sp,
            next_rflags,
            frame.vec,
            cpu_id,
            tid,
        );
    }
    // For kernel-to-kernel returns (`cs=0x28`), iretq does not consume SS/RSP.
    // Validate SS only for usermode resumes where SS is architecturally used.
    if next_cs == 0x3B && next_ss != 0x43 {
        panic!(
            "x86/trap: invalid ss for cs ptr=0x{:X} ip=0x{:X} cs=0x{:X} ss=0x{:X} sp=0x{:X} rflags=0x{:X} vec=0x{:X} cpu={} tid={}",
            next as usize as u64,
            next_ip,
            next_cs,
            next_ss,
            next_sp,
            next_rflags,
            frame.vec,
            crate::arch::thiscpu().id,
            crate::arch::thiscpu().current_thread
        );
    }
    if !is_canonical_addr(next_ip) || !is_canonical_addr(next_sp) {
        panic!(
            "x86/trap: non-canonical resume frame=0x{:X} ip=0x{:X} sp=0x{:X} cs=0x{:X} ss=0x{:X} vec=0x{:X} cpu={} tid={}",
            next as usize as u64,
            next_ip,
            next_sp,
            next_cs,
            next_ss,
            frame.vec,
            crate::arch::thiscpu().id,
            crate::arch::thiscpu().current_thread
        );
    }
    if next_rflags & (1 << 1) == 0 {
        panic!(
            "x86/trap: invalid rflags frame=0x{:X} ip=0x{:X} rflags=0x{:X} cs=0x{:X} ss=0x{:X} vec=0x{:X} cpu={} tid={}",
            next as usize as u64,
            next_ip,
            next_rflags,
            next_cs,
            next_ss,
            frame.vec,
            crate::arch::thiscpu().id,
            crate::arch::thiscpu().current_thread
        );
    }

    let next_addr = next as usize as u64;
    if next_addr < 0xFFFF_8000_0000_0000 {
        panic!(
            "x86/trap: invalid return frame=0x{:X} from vec=0x{:X} ip=0x{:X} cpu{} tid={}",
            next_addr,
            frame.vec,
            frame.ip,
            crate::arch::thiscpu().id,
            crate::arch::thiscpu().current_thread
        );
    }

    next
}

/// Kernel syscall handler.
#[unsafe(no_mangle)]
extern "C" fn rsyscall(frame: &mut TrapFrame) -> *mut TrapFrame {
    panic!("SYSCALL triggered at IP=0x{:X}", frame.ip);
}

unsafe fn init_idt_entries() {
    let idt = (&raw mut KERNEL_IDT).cast::<Idt>();
    for idx in 0..256 {
        let addr = (vstub0 as *const u8).wrapping_add(idx * 0x10);
        // SAFETY: `idt` points to the 256-element static IDT and each index is
        // initialized exactly once during single-threaded CPU setup.
        unsafe { idt.add(idx).write(Idt::from_address(addr as u64, 0)) };
    }
}

#[inline(always)]
fn is_canonical_addr(addr: u64) -> bool {
    let sign = (addr >> 47) & 1;
    if sign == 0 {
        (addr >> 48) == 0
    } else {
        (addr >> 48) == 0xFFFF
    }
}

unsafe fn load_idt() {
    let idtr = Descriptor {
        limit: (core::mem::size_of::<[Idt; 256]>() - 1) as u16,
        base: (&raw const KERNEL_IDT as *const Idt) as u64,
    };
    let idtr_ptr = &idtr as *const Descriptor as u64;
    // SAFETY: `idtr_ptr` references a live descriptor for the initialized
    // static IDT.
    unsafe { core::arch::asm!("lidt [{idtr}]", idtr = in(reg) idtr_ptr) };
}
