//! Final machine transitions requested by the system manager.

use core::sync::atomic::{AtomicUsize, Ordering};

use crate::{
    arch, fs,
    syscall::{Errno, map_fs_error},
};

use super::{clock, smp, sync::Mutex};

const HALT: u64 = 0;
const REBOOT: u64 = 1;
const POWER_OFF: u64 = 2;
const HALT_TIMEOUT_NS: u64 = 1_000_000_000;

static TRANSITION: Mutex<()> = Mutex::new(());
static HALTED_CPUS: AtomicUsize = AtomicUsize::new(0);

pub(super) fn transition(action: u64) -> Result<(), Errno> {
    if !matches!(action, HALT | REBOOT | POWER_OFF) {
        return Err(Errno::Invalid);
    }
    #[cfg(target_arch = "x86_64")]
    if action != HALT {
        return Err(Errno::NotSupported);
    }

    let _transition = TRANSITION.try_lock().ok_or(Errno::Busy)?;
    fs::sync_all().map_err(map_fs_error)?;
    if action == HALT {
        halt_machine();
    }

    #[cfg(target_arch = "riscv64")]
    {
        // SBI SRST operates on the whole system and returns only on failure.
        let status = arch::system_reset(action == REBOOT);
        Err(if status == -2 {
            Errno::NotSupported
        } else {
            Errno::Io
        })
    }
    #[cfg(target_arch = "x86_64")]
    unreachable!("only halt is supported")
}

fn halt_machine() -> ! {
    let _pin = smp::disable_preemption();
    let expected = smp::online_cpus().saturating_sub(1);
    let _ = smp::send_ipi(halt_cpu, smp::IpiTarget::All);
    let start = clock::monotonic_ns();
    let message = loop {
        if HALTED_CPUS.load(Ordering::Acquire) == expected {
            break b"power: system halted\n".as_slice();
        }
        if clock::monotonic_ns().saturating_sub(start) >= HALT_TIMEOUT_NS {
            break b"power: halt timed out waiting for CPUs\n".as_slice();
        }
        core::hint::spin_loop();
    };
    arch::irqset(false);
    arch::console_write(message);
    idle_forever();
}

fn halt_cpu() {
    arch::irqset(false);
    HALTED_CPUS.fetch_add(1, Ordering::Release);
    idle_forever();
}

fn idle_forever() -> ! {
    #[cfg(target_arch = "riscv64")]
    arch::mask_halt_interrupts();
    loop {
        arch::wfi();
    }
}
