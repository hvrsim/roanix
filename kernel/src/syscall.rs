//! Shared userspace syscall ABI helpers.

use alloc::{string::String, sync::Arc, vec::Vec};
use core::{mem::size_of, time::Duration};

use crate::{
    fs,
    mem::{self, VirtAddr},
    proc::{self, Process},
};

const MAX_EXEC_ITEMS: usize = 256;
const MAX_EXEC_BYTES: usize = 64 * 1024;

#[repr(i64)]
#[derive(Copy, Clone)]
pub(crate) enum Errno {
    Permission = 1,
    NoEntry = 2,
    NoProcess = 3,
    Interrupted = 4,
    Io = 5,
    BadFileDescriptor = 9,
    NoChild = 10,
    TryAgain = 11,
    OutOfMemory = 12,
    Access = 13,
    Fault = 14,
    Busy = 16,
    Exists = 17,
    CrossDevice = 18,
    NotDirectory = 20,
    IsDirectory = 21,
    Invalid = 22,
    TooManyFiles = 24,
    NotTty = 25,
    FileTooLarge = 27,
    NoSpace = 28,
    IllegalSeek = 29,
    ReadOnly = 30,
    BrokenPipe = 32,
    Range = 34,
    NameTooLong = 36,
    NotImplemented = 38,
    NotEmpty = 39,
    Loop = 40,
    Overflow = 75,
    NotSupported = 95,
    TimedOut = 110,
}

pub(crate) type Result<T> = core::result::Result<T, Errno>;

/// Defines an assembly-table-compatible syscall handler.
///
/// Arguments name their source slot in the six-register userspace ABI. The
/// handler body returns [`crate::syscall::Result<u64>`].
#[macro_export]
macro_rules! syscall_handler {
    (
        $name:ident(
            $frame:ident
            $(, $argument:ident : $argument_type:ty = $argument_index:literal)*
            $(,)?
        ) -> !
        $body:block
    ) => {
        #[unsafe(no_mangle)]
        pub(crate) extern "C" fn $name(
            $frame: &mut $crate::arch::cpu::TrapFrame,
        ) -> i64 {
            $(
                let $argument =
                    $frame.syscall_argument($argument_index) as $argument_type;
            )*
            $body
        }
    };
    (
        $name:ident(
            $frame:ident
            $(, $argument:ident : $argument_type:ty = $argument_index:literal)*
            $(,)?
        )
        $body:block
    ) => {
        #[unsafe(no_mangle)]
        pub(crate) extern "C" fn $name(
            $frame: &mut $crate::arch::cpu::TrapFrame,
        ) -> i64 {
            $(
                let $argument =
                    $frame.syscall_argument($argument_index) as $argument_type;
            )*
            $crate::syscall::raw_result(
                (|| -> $crate::syscall::Result<u64> { $body })()
            )
        }
    };
}

pub(crate) fn raw_result(result: Result<u64>) -> i64 {
    match result {
        Ok(value) => value as i64,
        Err(error) => -(error as i64),
    }
}

crate::syscall_handler! {
    syscall_not_implemented(_frame) {
        Err(Errno::NotImplemented)
    }
}

pub(crate) fn current_process() -> Result<Arc<Process>> {
    proc::current().ok_or(Errno::Invalid)
}

pub(crate) fn read_user_string(process: &Process, address: u64) -> Result<String> {
    let mut bytes = Vec::new();
    for offset in 0..fs::path::MAX_PATH_LEN {
        let mut byte = [0u8; 1];
        let address = address.checked_add(offset as u64).ok_or(Errno::Fault)?;
        process
            .address_space()
            .read_user(VirtAddr::new(address), &mut byte)
            .map_err(map_memory_error)?;
        if byte[0] == 0 {
            return String::from_utf8(bytes).map_err(|_| Errno::Invalid);
        }
        bytes.push(byte[0]);
    }
    Err(Errno::NameTooLong)
}

pub(crate) fn read_user_string_array(process: &Process, address: u64) -> Result<Vec<String>> {
    if address == 0 {
        return Ok(Vec::new());
    }
    let mut strings = Vec::new();
    let mut total_bytes = 0usize;
    for index in 0..MAX_EXEC_ITEMS {
        let pointer_address = address
            .checked_add((index * size_of::<u64>()) as u64)
            .ok_or(Errno::Fault)?;
        let pointer = read_user_u64(process, pointer_address)?;
        if pointer == 0 {
            return Ok(strings);
        }
        let string = read_user_string(process, pointer)?;
        total_bytes = total_bytes
            .checked_add(string.len() + 1)
            .ok_or(Errno::Overflow)?;
        if total_bytes > MAX_EXEC_BYTES {
            return Err(Errno::Invalid);
        }
        strings.push(string);
    }
    Err(Errno::Invalid)
}

pub(crate) fn read_user_u64(process: &Process, address: u64) -> Result<u64> {
    let mut bytes = [0u8; size_of::<u64>()];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(u64::from_ne_bytes(bytes))
}

pub(crate) fn read_user_i32(process: &Process, address: u64) -> Result<i32> {
    let mut bytes = [0u8; size_of::<i32>()];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(i32::from_ne_bytes(bytes))
}

pub(crate) fn read_user_timespec(process: &Process, address: u64) -> Result<Duration> {
    let mut bytes = [0u8; 16];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    let seconds = i64::from_ne_bytes(bytes[..8].try_into().expect("timespec seconds width"));
    let nanoseconds =
        i64::from_ne_bytes(bytes[8..].try_into().expect("timespec nanoseconds width"));
    if seconds < 0 || !(0..1_000_000_000).contains(&nanoseconds) {
        return Err(Errno::Invalid);
    }
    Ok(Duration::new(seconds as u64, nanoseconds as u32))
}

pub(crate) fn map_process_error(error: proc::Error) -> Errno {
    match error {
        proc::Error::Filesystem(error) => map_fs_error(error),
        proc::Error::Memory(error) => map_memory_error(error),
        proc::Error::TooManyFiles => Errno::TooManyFiles,
        proc::Error::BadFileDescriptor => Errno::BadFileDescriptor,
        proc::Error::NoSuchProcess => Errno::NoProcess,
        proc::Error::NoChild => Errno::NoChild,
        proc::Error::PermissionDenied => Errno::Permission,
        proc::Error::InvalidArgument | proc::Error::InvalidElf(_) | proc::Error::UnsupportedElf => {
            Errno::Invalid
        }
    }
}

pub(crate) fn map_memory_error(error: mem::Error) -> Errno {
    match error {
        mem::Error::OutOfMemory | mem::Error::LimitExceeded => Errno::OutOfMemory,
        mem::Error::InvalidAddress
        | mem::Error::NotMapped
        | mem::Error::Protection
        | mem::Error::AlreadyMapped => Errno::Fault,
        mem::Error::SwapUnavailable | mem::Error::CorruptSwap | mem::Error::Pmap => Errno::Io,
    }
}

pub(crate) fn map_fs_error(error: fs::Error) -> Errno {
    match error {
        fs::Error::NotFound => Errno::NoEntry,
        fs::Error::AlreadyExists => Errno::Exists,
        fs::Error::NotDirectory => Errno::NotDirectory,
        fs::Error::IllegalSeek => Errno::IllegalSeek,
        fs::Error::IsDirectory => Errno::IsDirectory,
        fs::Error::NotTty => Errno::NotTty,
        fs::Error::Interrupted => Errno::Interrupted,
        fs::Error::NotEmpty => Errno::NotEmpty,
        fs::Error::CrossDevice => Errno::CrossDevice,
        fs::Error::SymlinkLoop => Errno::Loop,
        fs::Error::NameTooLong => Errno::NameTooLong,
        fs::Error::InvalidArgument => Errno::Invalid,
        fs::Error::Unsupported => Errno::NotSupported,
        fs::Error::Busy => Errno::Busy,
        fs::Error::WouldBlock => Errno::TryAgain,
        fs::Error::NoSpace => Errno::NoSpace,
        fs::Error::FileTooLarge => Errno::FileTooLarge,
        fs::Error::PermissionDenied => Errno::Access,
        fs::Error::ReadOnly => Errno::ReadOnly,
        fs::Error::BadFileDescriptor => Errno::BadFileDescriptor,
        fs::Error::OutOfMemory => Errno::OutOfMemory,
        fs::Error::Io => Errno::Io,
    }
}
