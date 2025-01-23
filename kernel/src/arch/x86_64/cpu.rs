use x86_64::registers::model_specific::{Efer, EferFlags};
use x86_64::registers::control::*;
use log::{info, warn};
use raw_cpuid::CpuId;

/// Checks for (and enables) CPU feature flags.
pub fn enable_features() {
    let cpuid = CpuId::new();

    if let Some(brand_string) = cpuid.get_processor_brand_string() {
        info!("cpu: model name \"{}\"", brand_string.as_str());
    }

    unsafe {
        // Enable the `syscall` and `sysret` instructions, as well as NX bit.
        Efer::write(Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS | EferFlags::NO_EXECUTE_ENABLE);

        // Set the groundwork for SIMD/FP instructions by disabling
        // emulation and activating CR0.MP
        Cr0::write(Cr0::read() | !Cr0Flags::EMULATE_COPROCESSOR | Cr0Flags::MONITOR_COPROCESSOR | Cr0Flags::WRITE_PROTECT);
    }

    // Ensure key CR4 features are supported.
    let feats = cpuid.get_feature_info().expect("cpu: unable to query for features with CPUID!");

    if !feats.has_pge() {
        panic!("cpu: global pages not supported!");
    }

    let ext_feats = cpuid.get_extended_feature_info().expect("cpu: unable to query for extended features with CPUID!");

    unsafe {
        // Enable Global Pages and *GSBASE/SSE instructions.
        let mut bits = Cr4Flags::PAGE_GLOBAL | Cr4Flags::OSFXSR | Cr4Flags::OSXMMEXCPT_ENABLE;

        // Enable SMEP/SMAP (if supported)
        if ext_feats.has_smep() {
            bits |= Cr4Flags::SUPERVISOR_MODE_EXECUTION_PROTECTION;
        } else {
            warn!("cpu: SMEP not supported!");
        }
        if ext_feats.has_smap() {
            bits |= Cr4Flags::SUPERVISOR_MODE_ACCESS_PREVENTION;
        } else {
            warn!("cpu: SMAP not supported!");
        }

        // Only activate PCIDs if the `invpcid` instruction is supported.
        if feats.has_pcid() && ext_feats.has_invpcid() {
            bits |= Cr4Flags::PCID;
        }

        // Enable UMIP (if supported)
        if ext_feats.has_umip() {
            bits |= Cr4Flags::USER_MODE_INSTRUCTION_PREVENTION;
        }

        // Enable Intel CET (if supportd)
        if ext_feats.has_cet_ss() {
            bits |= Cr4Flags::CONTROL_FLOW_ENFORCEMENT;
        }

        Cr4::write(Cr4::read() | bits);
    }
}
