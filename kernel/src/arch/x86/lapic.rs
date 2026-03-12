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
    sys::smp::IrqSpinLock,
};

/// Timer interrupt vector used by the local APIC.
pub const TIMER_VECTOR: u8 = 0xE0;

/// Reschedule IPI vector used for cross-core wakeups.
pub const RESCHEDULE_VECTOR: u8 = 0xE1;

/// Spurious interrupt vector used by the local APIC.
pub const SPURIOUS_VECTOR: u8 = 0xFF;

const IA32_APIC_BASE_MSR: u32 = 0x1B;
const IA32_APIC_BASE_ENABLE: u64 = 1 << 11;
const IA32_TSC_DEADLINE_MSR: u32 = 0x6E0;

const LAPIC_EOI: u32 = 0x0B0;
const LAPIC_SVR: u32 = 0x0F0;
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
    LapicOneShot,
}

struct LapicState {
    base: VirtAddr,
    tsc_hz: u64,
    lapic_timer_hz: u64,
    timer_mode: TimerMode,
    initialized: bool,
}

static LAPIC_STATE: IrqSpinLock<LapicState> = IrqSpinLock::new(LapicState::new());

impl LapicState {
    const fn new() -> Self {
        Self {
            base: VirtAddr::zero(),
            tsc_hz: 0,
            lapic_timer_hz: 0,
            timer_mode: TimerMode::TscDeadline,
            initialized: false,
        }
    }
}

/// Initializes shared LAPIC state and programs the BSP local timer path.
pub fn init(tsc_hz: u64) {
    let cpuid = CpuId::new();
    let features = cpuid
        .get_feature_info()
        .expect("x86/lapic: CPUID feature leaf missing");
    assert!(features.has_apic(), "x86/lapic: local APIC not supported");

    let mut apic_base = Msr::new(IA32_APIC_BASE_MSR);
    let mut apic_base_raw = unsafe { apic_base.read() };
    if apic_base_raw & IA32_APIC_BASE_ENABLE == 0 {
        apic_base_raw |= IA32_APIC_BASE_ENABLE;
        unsafe { apic_base.write(apic_base_raw) };
    }

    let deadline_capable = features.has_tsc_deadline();
    let base_pa = PhysAddr::new(apic_base_raw & !0xFFF);
    let base = ensure_mmio_mapping(base_pa);
    let timer_mode = if deadline_capable {
        TimerMode::TscDeadline
    } else {
        TimerMode::LapicOneShot
    };

    let mut state = LAPIC_STATE.lock();
    state.base = base;
    state.tsc_hz = tsc_hz;
    state.lapic_timer_hz = if timer_mode == TimerMode::LapicOneShot {
        calibrate_lapic_timer(base, tsc_hz)
    } else {
        0
    };
    state.timer_mode = timer_mode;
    state.initialized = true;

    drop(state);

    mask_legacy_pic();
    init_thiscpu();

    if timer_mode == TimerMode::LapicOneShot {
        info!(
            "x86/lapic: timer mode=lapic-oneshot hz={}",
            LAPIC_STATE.lock().lapic_timer_hz
        );
    } else {
        info!("x86/lapic: timer mode=tsc-deadline");
    }
}

/// Programs the local APIC on a secondary CPU using shared BSP calibration.
pub fn init_secondary() {
    let state = LAPIC_STATE.lock();
    assert!(
        state.initialized,
        "x86/lapic: init required before secondary setup"
    );
    drop(state);
    init_thiscpu();
}

/// Returns the active timer backend name.
pub fn timer_name() -> &'static str {
    let state = LAPIC_STATE.lock();
    match state.timer_mode {
        TimerMode::TscDeadline => "lapic-tsc-deadline",
        TimerMode::LapicOneShot => "lapic-oneshot",
    }
}

/// Arms the local APIC timer in one-shot mode.
pub fn set_oneshot(delay_ns: u64) {
    let state = LAPIC_STATE.lock();
    assert!(state.initialized, "x86/lapic: init required before arm");

    match state.timer_mode {
        TimerMode::TscDeadline => {
            let deadline = rdtsc().wrapping_add(ns_to_cycles(state.tsc_hz, delay_ns));
            unsafe { Msr::new(IA32_TSC_DEADLINE_MSR).write(deadline) };
            write_register(
                state.base,
                LAPIC_LVT_TIMER,
                TIMER_VECTOR as u32 | LVT_TIMER_TSC_DEADLINE,
            );
        }
        TimerMode::LapicOneShot => {
            let count = ns_to_lapic_ticks(state.lapic_timer_hz, delay_ns);
            write_register(state.base, LAPIC_DIVIDE_CONFIG, DIVIDE_BY_16);
            write_register(state.base, LAPIC_LVT_TIMER, TIMER_VECTOR as u32);
            write_register(state.base, LAPIC_INITIAL_COUNT, count);
        }
    }
}

/// Stops timer delivery.
pub fn stop_timer() {
    let state = LAPIC_STATE.lock();
    if !state.initialized {
        return;
    }

    unsafe { Msr::new(IA32_TSC_DEADLINE_MSR).write(0) };
    write_register(state.base, LAPIC_INITIAL_COUNT, 0);
    write_register(
        state.base,
        LAPIC_LVT_TIMER,
        TIMER_VECTOR as u32 | LVT_MASKED,
    );
}

/// Returns the minimum programmable interval.
pub fn min_period_ns() -> u64 {
    1
}

/// Returns the maximum programmable interval.
pub fn max_period_ns() -> u64 {
    let state = LAPIC_STATE.lock();
    if !state.initialized {
        return u64::MAX;
    }

    match state.timer_mode {
        TimerMode::TscDeadline => u64::MAX,
        TimerMode::LapicOneShot => ((u32::MAX as u128).saturating_mul(1_000_000_000u128)
            / state.lapic_timer_hz as u128)
            .min(u64::MAX as u128) as u64,
    }
}

/// Issues an end-of-interrupt to the local APIC.
pub fn eoi() {
    let state = LAPIC_STATE.lock();
    if state.initialized {
        write_register(state.base, LAPIC_EOI, 0);
    }
}

/// Handles LAPIC-delivered interrupts that should just trigger rescheduling.
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

/// Sends a fixed IPI carrying the reschedule vector to `lapic_id`.
pub fn send_ipi(lapic_id: u32) {
    let state = LAPIC_STATE.lock();
    assert!(state.initialized, "x86/lapic: init required before IPI");

    write_register(state.base, LAPIC_ICR_HIGH, lapic_id << 24);
    write_register(state.base, LAPIC_ICR_LOW, RESCHEDULE_VECTOR as u32);
}

fn calibrate_lapic_timer(base: VirtAddr, tsc_hz: u64) -> u64 {
    write_register(base, LAPIC_DIVIDE_CONFIG, DIVIDE_BY_16);
    write_register(base, LAPIC_LVT_TIMER, TIMER_VECTOR as u32 | LVT_MASKED);
    write_register(base, LAPIC_INITIAL_COUNT, u32::MAX);

    let start = rdtsc();
    let target = start.wrapping_add(ns_to_cycles(tsc_hz, LAPIC_TIMER_CALIBRATION_NS));
    while rdtsc().wrapping_sub(start) < target.wrapping_sub(start) {
        core::hint::spin_loop();
    }

    let current = read_register(base, LAPIC_CURRENT_COUNT);
    write_register(base, LAPIC_INITIAL_COUNT, 0);

    let elapsed = u32::MAX.wrapping_sub(current) as u128;
    let hz = elapsed.saturating_mul(1_000_000_000u128) / LAPIC_TIMER_CALIBRATION_NS as u128;
    hz.max(1).min(u64::MAX as u128) as u64
}

fn mask_legacy_pic() {
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

fn read_register(base: VirtAddr, offset: u32) -> u32 {
    unsafe { read_volatile(base.checked_add(offset as u64).unwrap().as_ptr::<u32>()) }
}

fn write_register(base: VirtAddr, offset: u32, value: u32) {
    unsafe {
        write_volatile(
            base.checked_add(offset as u64).unwrap().as_mut_ptr::<u32>(),
            value,
        );
        read_volatile(base.checked_add(LAPIC_SVR as u64).unwrap().as_ptr::<u32>());
    }
}

fn ensure_mmio_mapping(base_pa: PhysAddr) -> VirtAddr {
    let base = mem::phys_to_virt(base_pa);
    let root = arch::paging::active_root();

    let mapped = unsafe { arch::paging::translate(root, base).is_some() };
    if !mapped {
        let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL | VmFlags::DEVICE;
        unsafe {
            arch::paging::map_page(root, base, base_pa, flags)
                .unwrap_or_else(|err| panic!("x86/lapic: failed to map MMIO page: {:?}", err));
        }
    }

    base
}

fn init_thiscpu() {
    let mut apic_base = Msr::new(IA32_APIC_BASE_MSR);
    let mut apic_base_raw = unsafe { apic_base.read() };
    if apic_base_raw & IA32_APIC_BASE_ENABLE == 0 {
        apic_base_raw |= IA32_APIC_BASE_ENABLE;
        unsafe { apic_base.write(apic_base_raw) };
    }

    let state = LAPIC_STATE.lock();
    assert!(
        state.initialized,
        "x86/lapic: init required before local setup"
    );

    write_register(state.base, LAPIC_SVR, SVR_ENABLE | SPURIOUS_VECTOR as u32);
    write_register(state.base, LAPIC_EOI, 0);

    match state.timer_mode {
        TimerMode::TscDeadline => {
            write_register(
                state.base,
                LAPIC_LVT_TIMER,
                TIMER_VECTOR as u32 | LVT_TIMER_TSC_DEADLINE,
            );
            unsafe { Msr::new(IA32_TSC_DEADLINE_MSR).write(0) };
        }
        TimerMode::LapicOneShot => {
            write_register(state.base, LAPIC_DIVIDE_CONFIG, DIVIDE_BY_16);
            write_register(
                state.base,
                LAPIC_LVT_TIMER,
                TIMER_VECTOR as u32 | LVT_MASKED,
            );
            write_register(state.base, LAPIC_INITIAL_COUNT, 0);
        }
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

fn ns_to_lapic_ticks(freq_hz: u64, ns: u64) -> u32 {
    ((ns as u128)
        .saturating_mul(freq_hz as u128)
        .saturating_add(999_999_999u128)
        / 1_000_000_000u128)
        .max(1)
        .min(u32::MAX as u128) as u32
}

fn rdtsc() -> u64 {
    let low: u32;
    let high: u32;

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
