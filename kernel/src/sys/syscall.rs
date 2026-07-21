//! Userspace syscall ABI dispatch and validation.

use alloc::{collections::BTreeMap, string::String, sync::Arc, vec, vec::Vec};
use core::{
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};

use crate::{
    fs::{self, OpenFlags, SeekFrom},
    mem::{self, PAGE_SIZE, USER_ADDRESS_MIN, VirtAddr, VmInheritance, VmProtection},
    proc::{self, Descriptor, Process},
    sys::{
        clock,
        event::Event,
        sync::{Mutex, Once},
    },
};

const SYS_PROCESS_EXIT: u64 = 0;
const SYS_THREAD_SET_POINTER: u64 = 1;
const SYS_FUTEX_WAIT: u64 = 2;
const SYS_FUTEX_WAKE: u64 = 3;
const SYS_FILE_OPEN: u64 = 4;
const SYS_FILE_CLOSE: u64 = 5;
const SYS_FILE_READ: u64 = 6;
const SYS_FILE_WRITE: u64 = 7;
const SYS_FILE_SEEK: u64 = 8;
const SYS_CLOCK_GET: u64 = 9;
const SYS_MEMORY_MAP: u64 = 10;
const SYS_MEMORY_UNMAP: u64 = 11;

const MAX_IO_SIZE: usize = 16 * 1024 * 1024;
const MMAP_BASE: u64 = 0x1000_0000;

const O_ACCMODE: u64 = 0o3;
const O_WRONLY: u64 = 0o1;
const O_RDWR: u64 = 0o2;
const O_CREAT: u64 = 0o100;
const O_EXCL: u64 = 0o200;
const O_NOCTTY: u64 = 0o400;
const O_TRUNC: u64 = 0o1000;
const O_APPEND: u64 = 0o2000;
const O_NONBLOCK: u64 = 0o4000;
const O_LARGEFILE: u64 = 0o100000;
const O_DIRECTORY: u64 = 0o200000;
const O_NOFOLLOW: u64 = 0o400000;
const O_CLOEXEC: u64 = 0o2000000;

const PROT_READ: u64 = 0x01;
const PROT_WRITE: u64 = 0x02;
const PROT_EXEC: u64 = 0x04;

const MAP_SHARED: u64 = 0x01;
const MAP_PRIVATE: u64 = 0x02;
const MAP_FIXED: u64 = 0x10;
const MAP_ANONYMOUS: u64 = 0x20;
const MAP_DENYWRITE: u64 = 0x800;
const MAP_EXECUTABLE: u64 = 0x1000;
const MAP_NORESERVE: u64 = 0x4000;
const MAP_STACK: u64 = 0x20000;

const SEEK_SET: u64 = 0;
const SEEK_CUR: u64 = 1;
const SEEK_END: u64 = 2;

const CLOCK_REALTIME: u64 = 0;
const CLOCK_MONOTONIC: u64 = 1;
const CLOCK_MONOTONIC_RAW: u64 = 4;
const CLOCK_BOOTTIME: u64 = 7;

static FUTEXES: Once<Mutex<BTreeMap<FutexKey, Arc<Futex>>>> = Once::new();

#[derive(Copy, Clone, Eq, Ord, PartialEq, PartialOrd)]
struct FutexKey {
    address_space: usize,
    address: u64,
}

struct Futex {
    generation: AtomicU64,
    event: Event,
}

impl Futex {
    fn new() -> Self {
        Self {
            generation: AtomicU64::new(0),
            event: Event::new(),
        }
    }
}

#[repr(i64)]
#[derive(Copy, Clone)]
enum Errno {
    NoEntry = 2,
    Io = 5,
    BadFileDescriptor = 9,
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
    FileTooLarge = 27,
    NoSpace = 28,
    IllegalSeek = 29,
    ReadOnly = 30,
    NameTooLong = 36,
    NotImplemented = 38,
    NotEmpty = 39,
    Loop = 40,
    Overflow = 75,
    NotSupported = 95,
    TimedOut = 110,
}

type Result<T> = core::result::Result<T, Errno>;

/// Dispatches one syscall and returns either a value or a negative errno.
pub(crate) fn dispatch(number: u64, arguments: [u64; 6]) -> i64 {
    let name = syscall_name(number);
    let (pid, tid) = trace_context();
    if number == SYS_PROCESS_EXIT {
        log::info!("strace[{pid}:{tid}]: {name}({:#x})", arguments[0]);
        proc::exit_current(arguments[0] as i32);
    }

    let result = match number {
        SYS_THREAD_SET_POINTER => sys_thread_set_pointer(arguments[0]),
        SYS_FUTEX_WAIT => sys_futex_wait(arguments[0], arguments[1] as i32, arguments[2]),
        SYS_FUTEX_WAKE => sys_futex_wake(arguments[0], arguments[1]),
        SYS_FILE_OPEN => sys_file_open(arguments[0], arguments[1], arguments[2]),
        SYS_FILE_CLOSE => sys_file_close(arguments[0] as i32),
        SYS_FILE_READ => sys_file_read(arguments[0] as i32, arguments[1], arguments[2]),
        SYS_FILE_WRITE => sys_file_write(arguments[0] as i32, arguments[1], arguments[2]),
        SYS_FILE_SEEK => sys_file_seek(arguments[0] as i32, arguments[1] as i64, arguments[2]),
        SYS_CLOCK_GET => sys_clock_get(arguments[0], arguments[1], arguments[2]),
        SYS_MEMORY_MAP => sys_memory_map(arguments),
        SYS_MEMORY_UNMAP => sys_memory_unmap(arguments[0], arguments[1]),
        _ => Err(Errno::NotImplemented),
    };
    let value = match result {
        Ok(value) => value as i64,
        Err(error) => -(error as i64),
    };
    log::info!(
        "strace[{pid}:{tid}]: {name}({:#x}, {:#x}, {:#x}, {:#x}, {:#x}, {:#x}) = {value}",
        arguments[0],
        arguments[1],
        arguments[2],
        arguments[3],
        arguments[4],
        arguments[5],
    );
    value
}

fn syscall_name(number: u64) -> &'static str {
    match number {
        SYS_PROCESS_EXIT => "exit",
        SYS_THREAD_SET_POINTER => "thread_set_pointer",
        SYS_FUTEX_WAIT => "futex_wait",
        SYS_FUTEX_WAKE => "futex_wake",
        SYS_FILE_OPEN => "open",
        SYS_FILE_CLOSE => "close",
        SYS_FILE_READ => "read",
        SYS_FILE_WRITE => "write",
        SYS_FILE_SEEK => "seek",
        SYS_CLOCK_GET => "clock_get",
        SYS_MEMORY_MAP => "mmap",
        SYS_MEMORY_UNMAP => "munmap",
        _ => "unknown",
    }
}

fn trace_context() -> (usize, usize) {
    let pid = proc::current().map_or(0, |process| process.pid());
    let tid = crate::arch::thiscpu_opt().map_or(0, |cpu| cpu.current_thread);
    (pid, tid)
}

fn sys_thread_set_pointer(pointer: u64) -> Result<u64> {
    proc::set_current_thread_pointer(pointer).map_err(map_process_error)?;
    Ok(0)
}

fn sys_futex_wait(address: u64, expected: i32, timeout: u64) -> Result<u64> {
    if !address.is_multiple_of(core::mem::align_of::<i32>() as u64) {
        return Err(Errno::Invalid);
    }

    let process = current_process()?;
    if read_user_i32(&process, address)? != expected {
        return Err(Errno::TryAgain);
    }
    let key = FutexKey {
        address_space: Arc::as_ptr(&process.address_space()) as usize,
        address,
    };
    let futex = {
        let mut futexes = futexes().lock();
        futexes
            .entry(key)
            .or_insert_with(|| Arc::new(Futex::new()))
            .clone()
    };

    futex.event.reset();
    let generation = futex.generation.load(Ordering::Acquire);
    if read_user_i32(&process, address)? != expected {
        return Err(Errno::TryAgain);
    }
    if futex.generation.load(Ordering::Acquire) != generation {
        return Ok(0);
    }
    if timeout == 0 {
        futex.event.wait();
    } else {
        let duration = read_user_timespec(&process, timeout)?;
        if !clock::wait_timeout(&futex.event, duration) {
            return Err(Errno::TimedOut);
        }
    }
    Ok(0)
}

fn sys_futex_wake(address: u64, maximum: u64) -> Result<u64> {
    if maximum == 0 {
        return Ok(0);
    }
    if !address.is_multiple_of(core::mem::align_of::<i32>() as u64) {
        return Err(Errno::Invalid);
    }
    let process = current_process()?;
    let key = FutexKey {
        address_space: Arc::as_ptr(&process.address_space()) as usize,
        address,
    };
    let Some(futex) = futexes().lock().get(&key).cloned() else {
        return Ok(0);
    };
    futex.generation.fetch_add(1, Ordering::AcqRel);
    let woken = futex.event.signal();
    Ok(woken.min(maximum as usize) as u64)
}

fn sys_file_open(path: u64, flags: u64, mode: u64) -> Result<u64> {
    let process = current_process()?;
    let path = read_user_string(&process, path)?;
    let allowed = O_ACCMODE
        | O_CREAT
        | O_EXCL
        | O_NOCTTY
        | O_TRUNC
        | O_APPEND
        | O_NONBLOCK
        | O_LARGEFILE
        | O_DIRECTORY
        | O_NOFOLLOW
        | O_CLOEXEC;
    if flags & !allowed != 0 {
        return Err(Errno::Invalid);
    }

    let mut open_flags = match flags & O_ACCMODE {
        0 => OpenFlags::READ,
        O_WRONLY => OpenFlags::WRITE,
        O_RDWR => OpenFlags::READ | OpenFlags::WRITE,
        _ => return Err(Errno::Invalid),
    };
    if flags & O_CREAT != 0 {
        open_flags |= OpenFlags::CREATE;
    }
    if flags & O_EXCL != 0 {
        open_flags |= OpenFlags::EXCLUSIVE;
    }
    if flags & O_TRUNC != 0 {
        open_flags |= OpenFlags::TRUNCATE;
    }
    if flags & O_APPEND != 0 {
        open_flags |= OpenFlags::APPEND;
    }
    if flags & O_DIRECTORY != 0 {
        open_flags |= OpenFlags::DIRECTORY;
    }
    if flags & O_NOFOLLOW != 0 {
        open_flags |= OpenFlags::NOFOLLOW;
    }

    let file = fs::open(&path, open_flags, mode as u16).map_err(map_fs_error)?;
    let fd = process.install_file(file).map_err(map_process_error)?;
    Ok(fd as u64)
}

fn sys_file_close(fd: i32) -> Result<u64> {
    if current_process()?.close_descriptor(fd) {
        Ok(0)
    } else {
        Err(Errno::BadFileDescriptor)
    }
}

fn sys_file_read(fd: i32, buffer: u64, size: u64) -> Result<u64> {
    let size = checked_io_size(size)?;
    if size == 0 {
        return Ok(0);
    }
    let process = current_process()?;
    let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
    let mut bytes = vec![0u8; size];
    let read = match descriptor {
        Descriptor::ConsoleInput => 0,
        Descriptor::ConsoleOutput => return Err(Errno::BadFileDescriptor),
        Descriptor::File(file) => file.read(&mut bytes).map_err(map_fs_error)?,
    };
    process
        .address_space()
        .write_user(VirtAddr::new(buffer), &bytes[..read])
        .map_err(map_memory_error)?;
    Ok(read as u64)
}

fn sys_file_write(fd: i32, buffer: u64, size: u64) -> Result<u64> {
    let size = checked_io_size(size)?;
    if size == 0 {
        return Ok(0);
    }
    let process = current_process()?;
    let mut bytes = vec![0u8; size];
    process
        .address_space()
        .read_user(VirtAddr::new(buffer), &mut bytes)
        .map_err(map_memory_error)?;
    let written = match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
        Descriptor::ConsoleInput => return Err(Errno::BadFileDescriptor),
        Descriptor::ConsoleOutput => {
            crate::arch::console_write(&bytes);
            bytes.len()
        }
        Descriptor::File(file) => file.write(&bytes).map_err(map_fs_error)?,
    };
    Ok(written as u64)
}

fn sys_file_seek(fd: i32, offset: i64, whence: u64) -> Result<u64> {
    let process = current_process()?;
    let Descriptor::File(file) = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? else {
        return Err(Errno::IllegalSeek);
    };
    let from = match whence {
        SEEK_SET if offset >= 0 => SeekFrom::Start(offset as u64),
        SEEK_SET => return Err(Errno::Invalid),
        SEEK_CUR => SeekFrom::Current(offset),
        SEEK_END => SeekFrom::End(offset),
        _ => return Err(Errno::Invalid),
    };
    file.seek(from).map_err(map_fs_error)
}

fn sys_clock_get(clock_id: u64, seconds: u64, nanoseconds: u64) -> Result<u64> {
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

fn sys_memory_map(arguments: [u64; 6]) -> Result<u64> {
    let [hint, size, protection, flags, fd, offset] = arguments;
    if size == 0 || protection & !(PROT_READ | PROT_WRITE | PROT_EXEC) != 0 {
        return Err(Errno::Invalid);
    }
    let allowed_flags = MAP_SHARED
        | MAP_PRIVATE
        | MAP_FIXED
        | MAP_ANONYMOUS
        | MAP_DENYWRITE
        | MAP_EXECUTABLE
        | MAP_NORESERVE
        | MAP_STACK;
    if flags & !allowed_flags != 0 || (flags & MAP_SHARED != 0) == (flags & MAP_PRIVATE != 0) {
        return Err(Errno::Invalid);
    }

    let process = current_process()?;
    let space = process.address_space();
    let start = if flags & MAP_FIXED != 0 {
        if hint < USER_ADDRESS_MIN || !hint.is_multiple_of(PAGE_SIZE) {
            return Err(Errno::Invalid);
        }
        match space.unmap(VirtAddr::new(hint), size) {
            Ok(()) | Err(mem::Error::NotMapped) => {}
            Err(error) => return Err(map_memory_error(error)),
        }
        VirtAddr::new(hint)
    } else {
        space
            .find_space(
                VirtAddr::new(if hint == 0 { MMAP_BASE } else { hint }),
                size,
            )
            .map_err(map_memory_error)?
    };
    let protection = vm_protection(protection);
    let inheritance = if flags & MAP_SHARED != 0 {
        VmInheritance::Share
    } else {
        VmInheritance::Copy
    };

    if flags & MAP_ANONYMOUS != 0 {
        space
            .map_anonymous(start, size, protection, protection, inheritance)
            .map_err(map_memory_error)?;
    } else {
        let offset = offset as i64;
        if offset < 0 || !(offset as u64).is_multiple_of(PAGE_SIZE) {
            return Err(Errno::Invalid);
        }
        let fd = fd as i32;
        let Descriptor::File(file) = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? else {
            return Err(Errno::BadFileDescriptor);
        };
        if protection.contains(VmProtection::WRITE)
            && flags & MAP_SHARED != 0
            && !file.flags().contains(OpenFlags::WRITE)
        {
            return Err(Errno::Access);
        }
        let object = file.vnode().memory_object().map_err(map_fs_error)?;
        space
            .map_object(
                start,
                size,
                object,
                offset as u64,
                protection,
                protection,
                inheritance,
                flags & MAP_PRIVATE != 0,
            )
            .map_err(map_memory_error)?;
    }
    Ok(start.as_u64())
}

fn sys_memory_unmap(address: u64, size: u64) -> Result<u64> {
    if size == 0 || !address.is_multiple_of(PAGE_SIZE) {
        return Err(Errno::Invalid);
    }
    current_process()?
        .address_space()
        .unmap(VirtAddr::new(address), size)
        .map_err(map_memory_error)?;
    Ok(0)
}

fn current_process() -> Result<Arc<Process>> {
    proc::current().ok_or(Errno::Invalid)
}

fn read_user_string(process: &Process, address: u64) -> Result<String> {
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

fn read_user_i32(process: &Process, address: u64) -> Result<i32> {
    let mut bytes = [0u8; core::mem::size_of::<i32>()];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(i32::from_ne_bytes(bytes))
}

fn read_user_timespec(process: &Process, address: u64) -> Result<Duration> {
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

fn checked_io_size(size: u64) -> Result<usize> {
    let size = usize::try_from(size).map_err(|_| Errno::Overflow)?;
    if size > MAX_IO_SIZE {
        return Err(Errno::Invalid);
    }
    Ok(size)
}

fn vm_protection(bits: u64) -> VmProtection {
    let mut protection = VmProtection::empty();
    if bits & PROT_READ != 0 {
        protection |= VmProtection::READ;
    }
    if bits & PROT_WRITE != 0 {
        protection |= VmProtection::WRITE;
    }
    if bits & PROT_EXEC != 0 {
        protection |= VmProtection::EXECUTE;
    }
    protection
}

fn futexes() -> &'static Mutex<BTreeMap<FutexKey, Arc<Futex>>> {
    FUTEXES.call_once(|| Mutex::new(BTreeMap::new()))
}

fn map_process_error(error: proc::Error) -> Errno {
    match error {
        proc::Error::Filesystem(error) => map_fs_error(error),
        proc::Error::Memory(error) => map_memory_error(error),
        proc::Error::TooManyFiles => Errno::TooManyFiles,
        proc::Error::InvalidArgument | proc::Error::InvalidElf(_) | proc::Error::UnsupportedElf => {
            Errno::Invalid
        }
    }
}

fn map_memory_error(error: mem::Error) -> Errno {
    match error {
        mem::Error::OutOfMemory | mem::Error::LimitExceeded => Errno::OutOfMemory,
        mem::Error::InvalidAddress
        | mem::Error::NotMapped
        | mem::Error::Protection
        | mem::Error::AlreadyMapped => Errno::Fault,
        mem::Error::SwapUnavailable | mem::Error::CorruptSwap | mem::Error::Pmap => Errno::Io,
    }
}

fn map_fs_error(error: fs::Error) -> Errno {
    match error {
        fs::Error::NotFound => Errno::NoEntry,
        fs::Error::AlreadyExists => Errno::Exists,
        fs::Error::NotDirectory => Errno::NotDirectory,
        fs::Error::IsDirectory => Errno::IsDirectory,
        fs::Error::NotEmpty => Errno::NotEmpty,
        fs::Error::CrossDevice => Errno::CrossDevice,
        fs::Error::SymlinkLoop => Errno::Loop,
        fs::Error::NameTooLong => Errno::NameTooLong,
        fs::Error::InvalidArgument => Errno::Invalid,
        fs::Error::Unsupported => Errno::NotSupported,
        fs::Error::Busy => Errno::Busy,
        fs::Error::NoSpace => Errno::NoSpace,
        fs::Error::FileTooLarge => Errno::FileTooLarge,
        fs::Error::PermissionDenied => Errno::Access,
        fs::Error::ReadOnly => Errno::ReadOnly,
        fs::Error::BadFileDescriptor => Errno::BadFileDescriptor,
        fs::Error::OutOfMemory => Errno::OutOfMemory,
        fs::Error::Io => Errno::Io,
    }
}
