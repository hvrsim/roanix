//!
//! # riscv64 Timekeeping
//!
//! Architected `time` counter plus SBI timer deadline delivery.
//!

use core::arch::asm;
use core::sync::atomic::{AtomicU64, Ordering};

use log::warn;

use crate::dev::dtb;

const SBI_EXT_TIME: usize = 0x54494D45;
const SBI_TIME_SET_TIMER: usize = 0;
const SBI_LEGACY_SET_TIMER: usize = 0x00;
const QEMU_VIRT_TIMEBASE_HZ: u64 = 10_000_000;

static TIMEBASE_HZ: AtomicU64 = AtomicU64::new(0);

/// Initializes the riscv counter source and SBI timer delivery.
pub fn init() {
    let timebase_hz = dtb::timebase_frequency().unwrap_or_else(|| {
        warn!(
            "riscv/timer: timebase-frequency missing from device tree, falling back to {} Hz",
            QEMU_VIRT_TIMEBASE_HZ
        );
        QEMU_VIRT_TIMEBASE_HZ
    });

    TIMEBASE_HZ.store(timebase_hz, Ordering::Relaxed);
    stop();
    super::cpu::enable_timer_interrupts();
}

/// Enables local timer interrupts on a secondary hart.
pub fn init_secondary() {
    stop();
    super::cpu::enable_timer_interrupts();
}

/// Returns the active counter source name.
pub fn counter_name() -> &'static str {
    "riscv-time"
}

/// Returns the active event timer name.
pub fn timer_name() -> &'static str {
    "sbi-timer"
}

/// Returns the counter frequency in Hz.
pub fn counter_frequency_hz() -> u64 {
    TIMEBASE_HZ.load(Ordering::Relaxed)
}

/// Returns the current raw cycle counter.
pub fn counter() -> u64 {
    read_time()
}

/// Returns the minimum programmable deadline delta in nanoseconds.
pub fn min_deadline_ns() -> u64 {
    1
}

/// Returns the maximum programmable deadline delta in nanoseconds.
pub fn max_deadline_ns() -> u64 {
    u64::MAX
}

/// Programs the next local timer interrupt for `deadline_ns`.
pub fn set_deadline(deadline_ns: u64, _now_ns: u64) {
    let delta = ns_to_cycles(
        counter_frequency_hz(),
        deadline_ns.saturating_sub(monotonic_ns()),
    );
    sbi_set_timer(read_time().wrapping_add(delta));
}

/// Stops local timer delivery.
pub fn stop() {
    sbi_set_timer(u64::MAX);
}

fn monotonic_ns() -> u64 {
    cycles_to_ns(counter_frequency_hz(), read_time())
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

fn cycles_to_ns(freq_hz: u64, cycles: u64) -> u64 {
    ((cycles as u128).saturating_mul(1_000_000_000u128) / freq_hz as u128).min(u64::MAX as u128)
        as u64
}
