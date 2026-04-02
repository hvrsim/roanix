//!
//! # riscv Platform Timers
//!
//! Timers and counters for the riscv platform, backed by SBI/SSTC.
//!

use core::arch::asm;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::dev::dtb;
use crate::sys::clock::{self, ClockSource, EventTimer};

const CSR_STIMECMP: u16 = 0x14D;
const SBI_EXT_TIME: usize = 0x54494D45;
const SBI_TIME_SET_TIMER: usize = 0;
const SBI_LEGACY_SET_TIMER: usize = 0x00;

static TIMEBASE_HZ: AtomicU64 = AtomicU64::new(0);
static USE_CPU_TIMER: AtomicBool = AtomicBool::new(false);
static RISCV_CLOCKSOURCE: RiscvClockSource = RiscvClockSource;
static RISCV_EVENT_TIMER: RiscvEventTimer = RiscvEventTimer;

struct RiscvClockSource;
struct RiscvEventTimer;

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

impl EventTimer for RiscvEventTimer {
    fn name(&self) -> &'static str {
        if use_cpu_timer() {
            "cpu-timer"
        } else {
            "sbi-timer"
        }
    }

    fn min_period_ns(&self) -> u64 {
        1
    }

    fn max_period_ns(&self) -> u64 {
        u64::MAX
    }

    fn set_oneshot(&self, delay_ns: u64) {
        let delta = ns_to_cycles(TIMEBASE_HZ.load(Ordering::Relaxed), delay_ns);
        set_active_timer_deadline(read_time().wrapping_add(delta));
    }

    fn stop(&self) {
        stop_active_timer();
    }
}

/// Initializes the riscv counter source and timer delivery.
pub fn init() {
    let timebase_hz =
        dtb::timebase_frequency().expect("riscv/timer: timebase freq missing from DTB!");
    let use_cpu_timer = dtb::all_cpus_support_sstc() == Some(true);

    TIMEBASE_HZ.store(timebase_hz, Ordering::Relaxed);
    USE_CPU_TIMER.store(use_cpu_timer, Ordering::Relaxed);
    stop_active_timer();
    clock::register_clocksource(&RISCV_CLOCKSOURCE);
    clock::register_event_timer(&RISCV_EVENT_TIMER);
    super::cpu::enable_timer_interrupts();
}

/// Enables local timer interrupts on a secondary hart.
pub fn init_secondary() {
    stop_active_timer();
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

fn write_stimecmp(deadline: u64) {
    unsafe {
        super::cpu::wrcsr::<CSR_STIMECMP>(deadline);
    }
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

fn stop_active_timer() {
    set_active_timer_deadline(u64::MAX);
}

fn set_active_timer_deadline(deadline: u64) {
    if use_cpu_timer() {
        write_stimecmp(deadline);
    } else {
        sbi_set_timer(deadline);
    }
}

fn use_cpu_timer() -> bool {
    USE_CPU_TIMER.load(Ordering::Relaxed)
}

fn ns_to_cycles(freq_hz: u64, ns: u64) -> u64 {
    ((ns as u128)
        .saturating_mul(freq_hz as u128)
        .saturating_add(999_999_999u128)
        / 1_000_000_000u128)
        .max(1)
        .min(u64::MAX as u128) as u64
}
