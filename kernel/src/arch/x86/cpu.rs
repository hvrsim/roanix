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

use core::ptr;

use bitflags::bitflags;
use log::{info, warn};
use raw_cpuid::CpuId;
use spin::Once;
use x86_64::registers::control::*;
use x86_64::registers::model_specific::{Efer, EferFlags};

core::arch::global_asm!(include_str!("trap.S"), options(att_syntax));

extern "C" {
    fn vstub0();
    fn rthread_resume(frame: *const TrapFrame) -> !;
}

static KERNEL_GDT: GDT = GDT::new();
// SAFETY: descriptor tables are initialized during per-CPU early boot before
// concurrent Rust code starts executing on that CPU.
static mut KERNEL_IDT: [IDT; 256] = [IDT::new(); 256];
static DESCRIPTORS_READY: Once<()> = Once::new();

bitflags! {
    /// Bitmap of supported x86 extensions.
    pub struct CpuFeatures: u32 {
        const SMEP = 0b0001;
        const SMAP = 0b0010;
        const PCID = 0b0100;
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
struct GDT {
    /// GDT entries as raw u64s.
    entries: [u64; 9],

    /// Low 2 bytes of TSS limit.
    tss_limit_low: u16,

    /// Low 2 bytes of TSS base address.
    tss_base_low: u16,

    /// Mid byte of TSS base address.
    tss_base_mid: u8,

    /// TSS Flags/Access byte.
    tss_access: u8,

    /// High byte of TSS limit.
    tss_limit_high: u8,

    /// High byte of TSS base address.
    tss_base_high: u8,

    /// Extended TSS base address.
    tss_base_ext: u32,

    /// TSS reserved field.
    tss_reserved: u32,
}

/// Representation of the x86_64 Interrupt Descriptor Table.
#[repr(C, packed(1))]
#[derive(Copy, Clone)]
struct IDT {
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
            rflags: 1 << 9,
            sp: stack_top,
            ss: 0x30,
        },
    );
}

/// Restores `frame` and enters the first scheduled kernel thread.
///
/// # Safety
///
/// `frame` must contain a valid saved kernel context produced by the trap
/// entry code or by [`init_kernel_thread_frame`].
pub unsafe fn start_first_thread(frame: *mut TrapFrame) -> ! {
    rthread_resume(frame);
}

/// Refreshes architecture-specific per-CPU state in a thread frame.
///
/// # Safety
///
/// `frame` must point to the current thread's saved trap frame. x86_64 does
/// not currently need to mutate it before resume.
pub unsafe fn prepare_thread_frame(_frame: *mut TrapFrame) {}

impl GDT {
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
            tss_limit_low: 0,
            tss_base_low: 0,
            tss_base_mid: 0,
            tss_access: 0,
            tss_limit_high: 0,
            tss_base_high: 0,
            tss_base_ext: 0,
            tss_reserved: 0,
        }
    }

    /// Loads the GDT structure into the CPU registers.
    unsafe fn load(&self) {
        let gdtr = Descriptor {
            limit: (size_of::<Self>() - 1) as u16,
            base: (self as *const Self) as u64,
        };

        let gdtr_ptr: u64 = &gdtr as *const Descriptor as u64;

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

            // clobber rax since we use it for setting the segement regs.
            out("rax") _,
            options(att_syntax, preserves_flags)
        );
    }
}

impl IDT {
    /// Creates a new IDT entry.
    const fn new() -> Self {
        IDT {
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
        IDT {
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
    DESCRIPTORS_READY.call_once(|| {
        init_idt_entries();
    });

    let cpuid = CpuId::new();
    let mut cpufeats = CpuFeatures::empty();

    if let Some(brand_string) = cpuid.get_processor_brand_string() {
        info!("cpu: model name \"{}\"", brand_string.as_str());
    }

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

    if !feats.has_pge() {
        panic!("cpu: global pages are not supported!");
    }

    // Enable Global Pages and SSE instructions.
    let mut bits = Cr4Flags::PAGE_GLOBAL | Cr4Flags::OSFXSR | Cr4Flags::OSXMMEXCPT_ENABLE;

    // FSGSBASE is optional in kernel mode, but we enable it when available
    // so userspace can use {RD,WR}{FS,GS}BASE instructions.
    if ext_feats.has_fsgsbase() {
        bits |= Cr4Flags::FSGSBASE;
    } else {
        warn!("cpu: {{FS/GS}}BASE instructions are not supported!");
    }

    // Enable SMEP/SMAP (if supported)
    if ext_feats.has_smep() {
        bits |= Cr4Flags::SUPERVISOR_MODE_EXECUTION_PROTECTION;
        cpufeats |= CpuFeatures::SMEP;
    } else {
        warn!("cpu: SMEP not supported!");
    }

    if ext_feats.has_smap() {
        bits |= Cr4Flags::SUPERVISOR_MODE_ACCESS_PREVENTION;
        cpufeats |= CpuFeatures::SMAP;
    } else {
        warn!("cpu: SMAP not supported!");
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

    unsafe {
        // SAFETY: `enable_features` is only called during CPU bring-up before
        // concurrent execution starts on this core.
        Cr4::write(Cr4::read() | bits);
    }
    unsafe {
        KERNEL_GDT.load();
        load_idt();
    }

    cpufeats
}

/// Kernel trap handler.
///
/// All interrupts triggered start their journey here...
#[no_mangle]
extern "C" fn rtrap(frame: &mut TrapFrame) -> *mut TrapFrame {
    if crate::arch::timer::handle_interrupt(frame.vec) || frame.vec == 3 {
        return crate::sys::sched::trap_return(frame);
    }

    panic!(
        "CPU trap triggered at IP=0x{:X}, vec=0x{:X}",
        frame.ip, frame.vec
    );
}

/// Kernel syscall handler.
#[no_mangle]
extern "C" fn rsyscall(frame: &mut TrapFrame) -> *mut TrapFrame {
    panic!("SYSCALL triggered at IP=0x{:X}", frame.ip);
}

fn init_idt_entries() {
    for idx in 0..256 {
        let addr = (vstub0 as *const u8).wrapping_add(idx * 0x10);
        unsafe {
            KERNEL_IDT[idx] = IDT::from_address(addr as u64, 0);
        }
    }
}

unsafe fn load_idt() {
    let idtr = Descriptor {
        limit: (core::mem::size_of::<[IDT; 256]>() - 1) as u16,
        base: (&raw const KERNEL_IDT as *const IDT) as u64,
    };
    let idtr_ptr = &idtr as *const Descriptor as u64;
    core::arch::asm!("lidt [{idtr}]", idtr = in(reg) idtr_ptr);
}
