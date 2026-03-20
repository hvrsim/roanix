//!
//! # riscv64 Timekeeping
//!
//! Architected `time` counter plus SBI timer deadline delivery.
//!

use core::arch::asm;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::dev::dtb;
use crate::sys::clock::{self, ClockSource, EventTimer};

const SBI_EXT_TIME: usize = 0x54494D45;
const SBI_TIME_SET_TIMER: usize = 0;
const SBI_LEGACY_SET_TIMER: usize = 0x00;

static TIMEBASE_HZ: AtomicU64 = AtomicU64::new(0);
static RISCV_CLOCKSOURCE: RiscvClockSource = RiscvClockSource;
static SBI_EVENT_TIMER: SbiEventTimer = SbiEventTimer;

struct RiscvClockSource;
struct SbiEventTimer;

impl ClockSource for RiscvClockSource {
    fn name(&self) -> &'static str {
        "riscv-time"
    }

    fn rating(&self) -> u32 {
        3000
    }

    fn frequency_hz(&self) -> u64 {
        TIMEBASE_HZ.load(Ordering::Relaxed)
    }

    fn counter(&self) -> u64 {
        read_time()
    }
}

impl EventTimer for SbiEventTimer {
    fn name(&self) -> &'static str {
        "sbi-timer"
    }

    fn min_period_ns(&self) -> u64 {
        1
    }

    fn max_period_ns(&self) -> u64 {
        u64::MAX
    }

    fn set_oneshot(&self, delay_ns: u64) {
        let delta = ns_to_cycles(TIMEBASE_HZ.load(Ordering::Relaxed), delay_ns);
        sbi_set_timer(read_time().wrapping_add(delta));
    }

    fn stop(&self) {
        sbi_set_timer(u64::MAX);
    }
}

/// Initializes the riscv counter source and SBI timer delivery.
pub fn init() {
    let timebase_hz =
        dtb::timebase_frequency().expect("riscv/timer: timebase freq missing from DTB!");

    TIMEBASE_HZ.store(timebase_hz, Ordering::Relaxed);
    sbi_set_timer(u64::MAX);
    clock::register_clocksource(&RISCV_CLOCKSOURCE);
    clock::register_event_timer(&SBI_EVENT_TIMER);
    super::cpu::enable_timer_interrupts();
}

/// Enables local timer interrupts on a secondary hart.
pub fn init_secondary() {
    sbi_set_timer(u64::MAX);
    super::cpu::enable_timer_interrupts();
}

fn read_time() -> u64 {
    let value: u64;

    unsafe {
        asm!(
            "rdtime {}",
            out(reg) value,
            options(nomem, nostack, preserves_flags)
        );
    }

    value
}

fn sbi_set_timer(deadline: u64) {
    let ret = super::sbi_call1(deadline as usize, SBI_EXT_TIME, SBI_TIME_SET_TIMER);
    if ret.error == 0 {
        return;
    }

    let legacy = super::sbi_call1(deadline as usize, SBI_LEGACY_SET_TIMER, 0);
    if legacy.error != 0 {
        panic!(
            "riscv/timer: SBI set_timer failed (time ext={}, legacy={})",
            ret.error, legacy.error
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
