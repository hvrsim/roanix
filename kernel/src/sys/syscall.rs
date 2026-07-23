//! Clock and system-information syscall implementations.

use core::time::Duration;

use crate::{
    mem::VirtAddr,
    sys::clock,
    syscall::{Errno, current_process, map_memory_error},
};

const CLOCK_REALTIME: u64 = 0;
const CLOCK_MONOTONIC: u64 = 1;
const CLOCK_MONOTONIC_RAW: u64 = 4;
const CLOCK_BOOTTIME: u64 = 7;
const UTSNAME_FIELD_SIZE: usize = 65;
const UTSNAME_FIELD_COUNT: usize = 6;

crate::syscall_handler! {
    syscall_clock_get(
        _frame,
        clock_id: u64 = 0,
        seconds: u64 = 1,
        nanoseconds: u64 = 2,
    ) {
        if !matches!(
            clock_id,
            CLOCK_REALTIME | CLOCK_MONOTONIC | CLOCK_MONOTONIC_RAW | CLOCK_BOOTTIME
        ) {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let now = clock::monotonic_ns();
        let seconds_value = (now / 1_000_000_000) as i64;
        let nanoseconds_value = (now % 1_000_000_000) as i64;
        process
            .address_space()
            .write_user(VirtAddr::new(seconds), &seconds_value.to_ne_bytes())
            .map_err(map_memory_error)?;
        process
            .address_space()
            .write_user(VirtAddr::new(nanoseconds), &nanoseconds_value.to_ne_bytes())
            .map_err(map_memory_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_clock_sleep(_frame, seconds: u64 = 0, nanoseconds: u64 = 1) {
        if nanoseconds >= 1_000_000_000 {
            return Err(Errno::Invalid);
        }
        clock::sleep(Duration::new(seconds, nanoseconds as u32));
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_system_uname(_frame, output: u64 = 0) {
        #[cfg(target_arch = "x86_64")]
        const MACHINE: &[u8] = b"x86_64";
        #[cfg(target_arch = "riscv64")]
        const MACHINE: &[u8] = b"riscv64";

        let mut record = [0u8; UTSNAME_FIELD_SIZE * UTSNAME_FIELD_COUNT];
        write_utsname_field(&mut record, 0, b"Roanix");
        write_utsname_field(&mut record, 1, b"local");
        write_utsname_field(&mut record, 2, env!("CARGO_PKG_VERSION").as_bytes());
        write_utsname_field(
            &mut record,
            3,
            concat!("#1 ", env!("ROANIX_GIT_HASH")).as_bytes(),
        );
        write_utsname_field(&mut record, 4, MACHINE);
        write_utsname_field(&mut record, 5, b"(none)");

        current_process()?
            .address_space()
            .write_user(VirtAddr::new(output), &record)
            .map_err(map_memory_error)?;
        Ok(0)
    }
}

fn write_utsname_field(record: &mut [u8], index: usize, value: &[u8]) {
    assert!(
        index < UTSNAME_FIELD_COUNT && value.len() < UTSNAME_FIELD_SIZE,
        "uname: invalid field"
    );
    let start = index * UTSNAME_FIELD_SIZE;
    record[start..start + value.len()].copy_from_slice(value);
}
