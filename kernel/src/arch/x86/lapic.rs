//!
//! # Local APIC
//!
//! Local APIC support for per-CPU one-shot timer delivery.
//!

use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

use log::info;
use raw_cpuid::CpuId;
use x86_64::registers::model_specific::Msr;

use crate::{
    arch,
    mem::{self, PhysAddr, VirtAddr, VmFlags},
    sys::{clock::ClockScale, sync::Once},
};

/// Timer interrupt vector used by the local APIC.
pub const TIMER_VECTOR: u8 = 0xE0;

/// Reschedule IPI vector used for cross-core wakeups.
pub const RESCHEDULE_VECTOR: u8 = 0xE1;

/// Software-only reschedule vector used on the local CPU.
pub const SELF_RESCHEDULE_VECTOR: u8 = 0xE2;

/// Spurious interrupt vector used by the local APIC.
pub const SPURIOUS_VECTOR: u8 = 0xFF;

const IA32_APIC_BASE_MSR: u32 = 0x1B;
const IA32_APIC_BASE_X2_ENABLE: u64 = 1 << 10;
const IA32_APIC_BASE_GLOBAL_ENABLE: u64 = 1 << 11;
const IA32_TSC_DEADLINE_MSR: u32 = 0x6E0;
const X2APIC_MSR_BASE: u32 = 0x800;

const LAPIC_TPR: u32 = 0x080;
const LAPIC_EOI: u32 = 0x0B0;
const LAPIC_SVR: u32 = 0x0F0;
const LAPIC_ISR_BASE: u32 = 0x100;
const LAPIC_ISR_END: u32 = 0x170;
const LAPIC_ICR_LOW: u32 = 0x300;
const LAPIC_ICR_HIGH: u32 = 0x310;
const LAPIC_LVT_TIMER: u32 = 0x320;
const LAPIC_INITIAL_COUNT: u32 = 0x380;
const LAPIC_CURRENT_COUNT: u32 = 0x390;
const LAPIC_DIVIDE_CONFIG: u32 = 0x3E0;

const SVR_ENABLE: u32 = 1 << 8;
const LVT_MASKED: u32 = 1 << 16;
const LVT_TIMER_TSC_DEADLINE: u32 = 0b10 << 17;
const DIVIDE_BY_16: u32 = 0b0011;
const LAPIC_TIMER_CALIBRATION_NS: u64 = 10_000_000;

#[derive(Copy, Clone, Eq, PartialEq)]
enum TimerMode {
    TscDeadline,
    LocalOneShot,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum ApicAccess {
    XApic { base: VirtAddr },
    X2Apic,
}

#[derive(Copy, Clone)]
struct LapicState {
    access: ApicAccess,
    timer_mode: TimerMode,
    tsc_hz: u64,
    tsc_scale: ClockScale,
    lapic_timer_hz: u64,
    lapic_timer_scale: Option<ClockScale>,
}

static LAPIC_STATE: Once<LapicState> = Once::new();

impl TimerMode {
    fn name(self) -> &'static str {
        match self {
            Self::TscDeadline => "tsc-deadline",
            Self::LocalOneShot => "lapic-oneshot",
        }
    }
}

impl ApicAccess {
    fn name(self) -> &'static str {
        match self {
            Self::XApic { .. } => "xapic",
            Self::X2Apic => "x2apic",
        }
    }
}

fn lapic_state() -> &'static LapicState {
    LAPIC_STATE
        .get()
        .expect("x86/lapic: init required before use")
}

/// Initializes shared LAPIC state and programs the BSP local timer path.
pub fn init(tsc_hz: u64) {
    if LAPIC_STATE.get().is_some() {
        return;
    }

    let cpuid = CpuId::new();
    let features = cpuid
        .get_feature_info()
        .expect("x86/lapic: CPUID feature leaf missing");
    assert!(features.has_apic(), "x86/lapic: local APIC not supported");

    let access = detect_access_mode(features.has_x2apic());
    let timer_mode = if features.has_tsc_deadline() {
        TimerMode::TscDeadline
    } else {
        TimerMode::LocalOneShot
    };

    mask_legacy_pic();
    init_thiscpu(access, timer_mode);

    let lapic_timer_hz = if timer_mode == TimerMode::LocalOneShot {
        calibrate_lapic_timer(access, tsc_hz)
    } else {
        0
    };

    LAPIC_STATE.call_once(|| LapicState {
        access,
        timer_mode,
        tsc_hz,
        tsc_scale: ClockScale::new(tsc_hz),
        lapic_timer_hz,
        lapic_timer_scale: (lapic_timer_hz != 0).then(|| ClockScale::new(lapic_timer_hz)),
    });

    if timer_mode == TimerMode::LocalOneShot {
        info!(
            "x86/lapic: mode={} timer={} hz={}",
            access.name(),
            timer_mode.name(),
            lapic_timer_hz
        );
    } else {
        info!(
            "x86/lapic: mode={} timer={}",
            access.name(),
            timer_mode.name()
        );
    }
}

/// Programs the local APIC on a secondary CPU using BSP-selected settings.
pub fn init_secondary() {
    let state = lapic_state();
    init_thiscpu(state.access, state.timer_mode);
}

/// Returns the active timer backend name.
pub fn timer_name() -> &'static str {
    let state = lapic_state();
    match state.timer_mode {
        TimerMode::TscDeadline => "lapic-tsc-deadline",
        TimerMode::LocalOneShot => "lapic-oneshot",
    }
}

/// Returns the calibrated TSC frequency shared by all CPUs.
pub fn calibrated_tsc_hz() -> u64 {
    lapic_state().tsc_hz
}

/// Returns the calibrated LAPIC timer frequency shared by all CPUs.
pub fn calibrated_timer_hz() -> u64 {
    lapic_state().lapic_timer_hz
}

/// Arms the local APIC timer in one-shot mode.
pub fn set_oneshot(delay_ns: u64) {
    let state = lapic_state();

    match state.timer_mode {
        TimerMode::TscDeadline => {
            let deadline = rdtsc().wrapping_add(state.tsc_scale.ns_to_cycles(delay_ns));
            write_msr(IA32_TSC_DEADLINE_MSR, deadline);
        }
        TimerMode::LocalOneShot => {
            let count = state
                .lapic_timer_scale
                .expect("x86/lapic: local timer scale missing")
                .ns_to_cycles(delay_ns)
                .min(u64::from(u32::MAX)) as u32;
            write_register(state.access, LAPIC_INITIAL_COUNT, count);
        }
    }
}

/// Stops timer delivery on the current CPU.
pub fn stop_timer() {
    let state = lapic_state();

    match state.timer_mode {
        TimerMode::TscDeadline => write_msr(IA32_TSC_DEADLINE_MSR, 0),
        TimerMode::LocalOneShot => write_register(state.access, LAPIC_INITIAL_COUNT, 0),
    }
}

/// Returns the minimum programmable interval.
pub fn min_period_ns() -> u64 {
    1
}

/// Returns the maximum programmable interval.
pub fn max_period_ns() -> u64 {
    let state = lapic_state();

    match state.timer_mode {
        TimerMode::TscDeadline => u64::MAX,
        TimerMode::LocalOneShot => ((u32::MAX as u128).saturating_mul(1_000_000_000u128)
            / local_lapic_timer_hz() as u128)
            .min(u64::MAX as u128) as u64,
    }
}

/// Issues an end-of-interrupt to the local APIC.
pub fn eoi() {
    write_register(lapic_state().access, LAPIC_EOI, 0);
}

/// Handles LAPIC-delivered timer, IPI, and spurious vectors.
pub fn handle_interrupt(vec: u64) -> bool {
    if vec == SPURIOUS_VECTOR as u64 {
        return true;
    }

    if vec == TIMER_VECTOR as u64 || vec == RESCHEDULE_VECTOR as u64 {
        eoi();
        return true;
    }

    false
}

/// Sends a fixed reschedule IPI to `lapic_id`.
pub fn send_ipi(lapic_id: u32) {
    let state = lapic_state();
    send_fixed_ipi(state.access, lapic_id, RESCHEDULE_VECTOR);
}

fn detect_access_mode(x2apic_supported: bool) -> ApicAccess {
    let apic_base = read_msr(IA32_APIC_BASE_MSR);
    if x2apic_supported && apic_base & IA32_APIC_BASE_X2_ENABLE != 0 {
        return ApicAccess::X2Apic;
    }

    ApicAccess::XApic {
        base: ensure_xapic_mapping(PhysAddr::new(apic_base & !0xFFF)),
    }
}

fn init_thiscpu(access: ApicAccess, timer_mode: TimerMode) {
    enable_access_mode(access);

    // Reset the software-enable bit first so we start from a known state even
    // if firmware or the bootloader left LAPIC state behind.
    write_register(access, LAPIC_SVR, 0);
    clear_in_service(access);
    write_register(access, LAPIC_TPR, 0);
    write_register(access, LAPIC_SVR, SVR_ENABLE | SPURIOUS_VECTOR as u32);
    write_register(access, LAPIC_EOI, 0);

    match timer_mode {
        TimerMode::TscDeadline => {
            write_msr(IA32_TSC_DEADLINE_MSR, 0);
            write_register(
                access,
                LAPIC_LVT_TIMER,
                TIMER_VECTOR as u32 | LVT_TIMER_TSC_DEADLINE,
            );
        }
        TimerMode::LocalOneShot => {
            write_register(access, LAPIC_DIVIDE_CONFIG, DIVIDE_BY_16);
            write_register(access, LAPIC_LVT_TIMER, TIMER_VECTOR as u32);
            write_register(access, LAPIC_INITIAL_COUNT, 0);
        }
    }
}

fn enable_access_mode(access: ApicAccess) {
    let raw = read_msr(IA32_APIC_BASE_MSR);
    match access {
        ApicAccess::XApic { .. } => {
            let enabled = raw | IA32_APIC_BASE_GLOBAL_ENABLE;
            if enabled != raw {
                write_msr(IA32_APIC_BASE_MSR, enabled);
            }
        }
        ApicAccess::X2Apic => {
            let enabled = raw | IA32_APIC_BASE_GLOBAL_ENABLE | IA32_APIC_BASE_X2_ENABLE;
            if enabled != raw {
                write_msr(IA32_APIC_BASE_MSR, enabled);
            }
        }
    }
}

fn clear_in_service(access: ApicAccess) {
    let mut reg = LAPIC_ISR_END;
    loop {
        for _ in 0..32 {
            if read_register(access, reg) == 0 {
                break;
            }
            write_register(access, LAPIC_EOI, 0);
        }

        if reg == LAPIC_ISR_BASE {
            break;
        }
        reg -= 0x10;
    }
}

fn calibrate_lapic_timer(access: ApicAccess, tsc_hz: u64) -> u64 {
    write_register(access, LAPIC_DIVIDE_CONFIG, DIVIDE_BY_16);
    write_register(access, LAPIC_LVT_TIMER, TIMER_VECTOR as u32 | LVT_MASKED);
    write_register(access, LAPIC_INITIAL_COUNT, u32::MAX);

    let start = rdtsc();
    let target = start.wrapping_add(ns_to_cycles(tsc_hz, LAPIC_TIMER_CALIBRATION_NS));
    while rdtsc().wrapping_sub(start) < target.wrapping_sub(start) {
        core::hint::spin_loop();
    }

    let current = read_register(access, LAPIC_CURRENT_COUNT);
    write_register(access, LAPIC_INITIAL_COUNT, 0);
    write_register(access, LAPIC_LVT_TIMER, TIMER_VECTOR as u32);

    let elapsed = u32::MAX.wrapping_sub(current) as u128;
    let hz = elapsed.saturating_mul(1_000_000_000u128) / LAPIC_TIMER_CALIBRATION_NS as u128;
    hz.max(1).min(u64::MAX as u128) as u64
}

fn send_fixed_ipi(access: ApicAccess, lapic_id: u32, vector: u8) {
    match access {
        ApicAccess::XApic { .. } => {
            write_register(access, LAPIC_ICR_HIGH, lapic_id << 24);
            write_register(access, LAPIC_ICR_LOW, vector as u32);
        }
        ApicAccess::X2Apic => {
            let icr = ((lapic_id as u64) << 32) | vector as u64;
            write_register64(access, LAPIC_ICR_LOW, icr);
        }
    }
}

fn ensure_xapic_mapping(base_pa: PhysAddr) -> VirtAddr {
    let base = mem::phys_to_virt(base_pa);
    let root = arch::paging::active_root();

    // SAFETY: we only query the current page tables and conditionally install a
    // single device mapping for the LAPIC page if it is currently absent.
    let mapped = unsafe { arch::paging::translate(root, base).is_some() };
    if !mapped {
        let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL | VmFlags::DEVICE;
        // SAFETY: the LAPIC page is a single architectural MMIO region. The
        // kernel owns the active page tables during early x86 init and this
        // mapping is global, non-executable, and device-typed.
        unsafe {
            arch::paging::map_page(root, base, base_pa, flags)
                .unwrap_or_else(|err| panic!("x86/lapic: failed to map MMIO page: {:?}", err));
        }
    }

    base
}

fn read_register(access: ApicAccess, offset: u32) -> u32 {
    match access {
        ApicAccess::XApic { base } => {
            // SAFETY: `base` is a stable mapping of the architectural LAPIC
            // MMIO page and `offset` is a fixed register offset within it.
            unsafe { read_volatile(register_addr(base, offset).as_ptr::<u32>()) }
        }
        ApicAccess::X2Apic => read_msr(x2apic_msr(offset)) as u32,
    }
}

fn write_register(access: ApicAccess, offset: u32, value: u32) {
    match access {
        ApicAccess::XApic { base } => {
            // SAFETY: `base` is a stable mapping of the architectural LAPIC
            // MMIO page and `offset` resolves to a 32-bit LAPIC register.
            unsafe {
                write_volatile(register_addr(base, offset).as_mut_ptr::<u32>(), value);
                // Flush posted MMIO writes by issuing a dependent LAPIC read.
                let _ = read_volatile(register_addr(base, LAPIC_SVR).as_ptr::<u32>());
            }
        }
        ApicAccess::X2Apic => write_msr(x2apic_msr(offset), value as u64),
    }
}

fn write_register64(access: ApicAccess, offset: u32, value: u64) {
    match access {
        ApicAccess::XApic { .. } => panic!("x86/lapic: 64-bit register write requires x2APIC"),
        ApicAccess::X2Apic => write_msr(x2apic_msr(offset), value),
    }
}

fn x2apic_msr(offset: u32) -> u32 {
    debug_assert_eq!(offset & 0xF, 0);
    X2APIC_MSR_BASE + (offset >> 4)
}

fn local_lapic_timer_hz() -> u64 {
    let hz = arch::thiscpu().platform.lapic_timer_hz;
    debug_assert!(hz != 0, "x86/lapic: local timer frequency not initialized");
    hz
}

fn register_addr(base: VirtAddr, offset: u32) -> VirtAddr {
    base.checked_add(offset as u64)
        .expect("x86/lapic: MMIO register address overflow")
}

fn read_msr(index: u32) -> u64 {
    // SAFETY: callers only pass architectural MSR indexes defined by the
    // x86_64 local-APIC and TSC-deadline interfaces.
    unsafe { Msr::new(index).read() }
}

fn write_msr(index: u32, value: u64) {
    // SAFETY: callers only write architectural MSR indexes with values built
    // according to the local-APIC specification.
    unsafe { Msr::new(index).write(value) }
}

fn mask_legacy_pic() {
    // SAFETY: programming the legacy PIC masks both PIC interrupt lines so the
    // LAPIC becomes the sole interrupt target after x86 bring-up.
    unsafe {
        asm!(
            "out dx, al",
            in("dx") 0x21u16,
            in("al") 0xFFu8,
            options(nomem, nostack, preserves_flags)
        );
        asm!(
            "out dx, al",
            in("dx") 0xA1u16,
            in("al") 0xFFu8,
            options(nomem, nostack, preserves_flags)
        );
    }
}

fn ns_to_cycles(freq_hz: u64, ns: u64) -> u64 {
    ((ns as u128)
        .saturating_mul(freq_hz as u128)
        .saturating_add(999_999_999u128)
        / 1_000_000_000u128)
        .max(1)
        .min(u64::MAX as u128) as u64
}

fn rdtsc() -> u64 {
    let low: u32;
    let high: u32;

    // SAFETY: `rdtsc` is available because x86 timer init requires TSC support.
    unsafe {
        asm!(
            "lfence",
            "rdtsc",
            out("eax") low,
            out("edx") high,
            options(nomem, nostack, preserves_flags)
        );
    }

    ((high as u64) << 32) | low as u64
}
