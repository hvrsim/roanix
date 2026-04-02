//!
//! # x86 Platform Timers
//!
//! Timers and counters for the x86 platform, backed by LAPIC/TSC.
//!

use core::arch::asm;

use raw_cpuid::{CpuId, CpuIdReader};
use x86_64::instructions::port::{
    PortGeneric, PortReadOnly, PortWriteOnly, ReadOnlyAccess, WriteOnlyAccess,
};

use crate::sys::clock::{self, ClockSource, EventTimer};

use super::lapic;

const PIT_TICK_RATE: u32 = 1_193_182;
const PIT_TARGET: u32 = 0x3FFF;
const PIT_MAX_COUNT: u32 = 0xFFFF;
const CALIBRATION_MS: u32 = 10;

static TSC_CLOCKSOURCE: TscClockSource = TscClockSource;
static LAPIC_EVENT_TIMER: LapicEventTimer = LapicEventTimer;

struct TscClockSource;
struct LapicEventTimer;

impl ClockSource for TscClockSource {
    fn name(&self) -> &'static str {
        "tsc"
    }

    fn rating(&self) -> u32 {
        4000
    }

    fn frequency_hz(&self) -> u64 {
        super::thiscpu().platform.tsc_hz
    }

    fn counter(&self) -> u64 {
        rdtsc()
    }
}

impl EventTimer for LapicEventTimer {
    fn name(&self) -> &'static str {
        lapic::timer_name()
    }

    fn min_period_ns(&self) -> u64 {
        lapic::min_period_ns()
    }

    fn max_period_ns(&self) -> u64 {
        lapic::max_period_ns()
    }

    fn set_oneshot(&self, delay_ns: u64) {
        lapic::set_oneshot(delay_ns);
    }

    fn stop(&self) {
        lapic::stop_timer();
    }
}

/// Result of dispatching an x86 local interrupt vector through the timer/LAPIC
/// path.
pub enum InterruptAction {
    Unhandled,
    Handled,
    Reschedule,
}

/// Initializes the x86 counter source and local timer hardware.
pub fn init() {
    let cpuid = CpuId::new();
    let feature_info = cpuid
        .get_feature_info()
        .expect("x86/timer: CPUID feature info missing");
    assert!(feature_info.has_tsc(), "x86/timer: TSC not supported");

    let invariant_tsc = cpuid
        .get_advanced_power_mgmt_info()
        .map(|info| info.has_invariant_tsc())
        .unwrap_or(false);
    assert!(invariant_tsc, "x86/timer: invariant TSC required");

    let tsc_hz = calibrate_tsc(&cpuid);
    lapic::init(tsc_hz);
    let platform = &mut super::thiscpu().platform;
    platform.tsc_hz = tsc_hz;
    platform.lapic_timer_hz = lapic::calibrated_timer_hz();

    clock::register_clocksource(&TSC_CLOCKSOURCE);
    clock::register_event_timer(&LAPIC_EVENT_TIMER);
}

/// Programs the local timer backend on a secondary CPU.
pub fn init_secondary() {
    lapic::init_secondary();
    let platform = &mut super::thiscpu().platform;
    platform.tsc_hz = lapic::calibrated_tsc_hz();
    platform.lapic_timer_hz = lapic::calibrated_timer_hz();
}

/// Handles LAPIC timer, reschedule, and spurious vectors.
pub fn handle_interrupt(vec: u64) -> InterruptAction {
    if vec == lapic::SPURIOUS_VECTOR as u64 {
        return InterruptAction::Handled;
    }

    if vec == lapic::TIMER_VECTOR as u64 {
        lapic::eoi();
        crate::sys::clock::handle_local_timer_interrupt();
        return InterruptAction::Reschedule;
    }

    if vec == lapic::RESCHEDULE_VECTOR as u64 {
        lapic::eoi();
        return InterruptAction::Reschedule;
    }

    if vec == lapic::SELF_RESCHEDULE_VECTOR as u64 {
        return InterruptAction::Reschedule;
    }

    InterruptAction::Unhandled
}

fn calibrate_tsc<R: CpuIdReader>(cpuid: &CpuId<R>) -> u64 {
    let tsc_hz = cpuid
        .get_tsc_info()
        .and_then(|info| info.tsc_frequency())
        .unwrap_or(0);
    if tsc_hz != 0 {
        return tsc_hz;
    }

    let reference_hz = cpuid
        .get_processor_frequency_info()
        .map(|info| info.processor_base_frequency() as u64 * 1_000_000)
        .unwrap_or(0);

    pit_calibrate_tsc(reference_hz)
}

fn pit_calibrate_tsc(reference_hz: u64) -> u64 {
    let max_cal_ms = ((PIT_MAX_COUNT - PIT_TARGET) * 1000) / PIT_TICK_RATE;
    let cal_ms = CALIBRATION_MS.min(max_cal_ms);

    let initial_pit = (cal_ms * PIT_TICK_RATE) / 1000 + PIT_TARGET;
    let initial_low = initial_pit as u8;
    let initial_high = (initial_pit >> 8) as u8;

    let irq_enabled = super::irqstate();
    super::irqset(false);

    let mut pit_cmd: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x43);
    let mut pit_ch0_write: PortGeneric<u8, WriteOnlyAccess> = PortWriteOnly::new(0x40);
    let mut pit_ch0_read: PortGeneric<u8, ReadOnlyAccess> = PortReadOnly::new(0x40);

    unsafe {
        pit_cmd.write(0x30);
        pit_ch0_write.write(initial_low);
        pit_ch0_write.write(initial_high);
    }

    let start = rdtsc();
    loop {
        unsafe {
            pit_cmd.write(0x00);
        }

        let low = unsafe { pit_ch0_read.read() };
        let high = unsafe { pit_ch0_read.read() };
        let current = ((high as u16) << 8) | low as u16;
        if current as u32 <= PIT_TARGET {
            break;
        }
    }

    let measured_hz = rdtsc().wrapping_sub(start) / cal_ms as u64 * 1000;

    if irq_enabled {
        super::irqset(true);
    }

    if reference_hz == 0 {
        return measured_hz;
    }

    let delta = measured_hz.saturating_mul(100) / reference_hz;
    if (95..=105).contains(&delta) {
        measured_hz
    } else {
        reference_hz
    }
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
