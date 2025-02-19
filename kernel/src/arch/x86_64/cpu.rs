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

use bitflags::bitflags;
use log::{info, warn};
use raw_cpuid::CpuId;
use x86_64::registers::control::*;
use x86_64::registers::model_specific::{Efer, EferFlags};

bitflags! {
    /// Bitmap of supported x86 extensions.
    pub struct CpuFeatures: u32 {
        const SMEP = 0b0001;
        const SMAP = 0b0010;
        const PCID = 0b0100;
        const CET_SS = 0b1000;
    }
}

/// Checks for (and enables) CPU feature flags.
pub fn enable_features() -> CpuFeatures {
    let cpuid = CpuId::new();
    let mut cpufeats = CpuFeatures::empty();

    if let Some(brand_string) = cpuid.get_processor_brand_string() {
        info!("cpu: model name \"{}\"", brand_string.as_str());
    }

    unsafe {
        // Enable the `syscall` and `sysret` instructions, as well as NX bit.
        Efer::write(
            Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS | EferFlags::NO_EXECUTE_ENABLE,
        );

        // Set the groundwork for SIMD/FP instructions by disabling
        // emulation and activating CR0.MP. Also enable Write Protect
        // becuase Intel CET requires it.
        Cr0::write(
            Cr0::read()
                | !Cr0Flags::EMULATE_COPROCESSOR
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
    if !ext_feats.has_fsgsbase() {
        panic!("cpu: {{FS/GS}}BASE instructions are not supported!");
    }

    // Enable Global Pages and *GSBASE/SSE instructions.
    let mut bits =
        Cr4Flags::PAGE_GLOBAL | Cr4Flags::FSGSBASE | Cr4Flags::OSFXSR | Cr4Flags::OSXMMEXCPT_ENABLE;

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

    // Enable Intel CET (if supportd)
    if ext_feats.has_cet_ss() {
        bits |= Cr4Flags::CONTROL_FLOW_ENFORCEMENT;
        cpufeats |= CpuFeatures::CET_SS;
    }

    unsafe {
        Cr4::write(Cr4::read() | bits);
    }

    return cpufeats;
}
