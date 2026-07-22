//! Userspace syscall ABI dispatch and validation.

use alloc::{collections::BTreeMap, string::String, sync::Arc, vec, vec::Vec};
use core::{
    mem::size_of,
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};

use crate::{
    arch::cpu::TrapFrame,
    fs::{self, IoctlContext, OpenFlags, SeekFrom, VnodeAttr, VnodeKind},
    mem::{self, PAGE_SIZE, USER_ADDRESS_MIN, VirtAddr, VmInheritance, VmProtection},
    proc::{self, Descriptor, PipeEnd, PipeError, Process},
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
const SYS_FILE_IOCTL: u64 = 12;
const SYS_FILE_DUP: u64 = 13;
const SYS_FILE_DUP2: u64 = 14;
const SYS_PROCESS_FORK: u64 = 15;
const SYS_PROCESS_EXEC: u64 = 16;
const SYS_PROCESS_WAIT: u64 = 17;
const SYS_PROCESS_GETPID: u64 = 18;
const SYS_PROCESS_GETPPID: u64 = 19;
const SYS_PROCESS_GETPGID: u64 = 20;
const SYS_PROCESS_SETPGID: u64 = 21;
const SYS_PROCESS_GETSID: u64 = 22;
const SYS_PROCESS_SETSID: u64 = 23;
const SYS_PROCESS_KILL: u64 = 24;
const SYS_FILE_CHDIR: u64 = 25;
const SYS_FILE_GETCWD: u64 = 26;
const SYS_FILE_ACCESS: u64 = 27;
const SYS_FILE_STAT: u64 = 28;
const SYS_FILE_FSTAT: u64 = 29;
const SYS_FILE_FCNTL: u64 = 30;
const SYS_FILE_PIPE: u64 = 31;
const SYS_FILE_READDIR: u64 = 32;
const SYS_FILE_READLINK: u64 = 33;
const SYS_FILE_UNLINK: u64 = 34;
const SYS_CLOCK_SLEEP: u64 = 35;
const SYS_FILE_MKDIR: u64 = 36;
const SYS_PROCESS_GETTID: u64 = 37;
const SYS_FILE_FCHMOD: u64 = 38;

const MAX_IO_SIZE: usize = 16 * 1024 * 1024;
const MAX_IOCTL_SIZE: usize = 4096;
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
const FD_CLOEXEC: u64 = 1;

const F_DUPFD: u64 = 0;
const F_GETFD: u64 = 1;
const F_SETFD: u64 = 2;
const F_GETFL: u64 = 3;
const F_SETFL: u64 = 4;
const F_DUPFD_CLOEXEC: u64 = 1030;

const WNOHANG: u64 = 1;
const WUNTRACED: u64 = 2;
const WCONTINUED: u64 = 8;
const PIPE_CLOEXEC: u64 = O_CLOEXEC;
const AT_SYMLINK_NOFOLLOW: u64 = 0x100;
const AT_REMOVEDIR: u64 = 0x200;
const USER_DIRENT_SIZE: usize = 280;
const MAX_EXEC_ITEMS: usize = 256;
const MAX_EXEC_BYTES: usize = 64 * 1024;

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
    Permission = 1,
    NoProcess = 3,
    Interrupted = 4,
    NoEntry = 2,
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

type Result<T> = core::result::Result<T, Errno>;

/// Dispatches one syscall and returns either a value or a negative errno.
pub(crate) fn dispatch(frame: &mut TrapFrame, number: u64, arguments: [u64; 6]) -> i64 {
    let name = syscall_name(number);
    let (pid, tid) = trace_context();
    if number == SYS_PROCESS_EXIT {
        // log::info!("strace[{pid}:{tid}]: {name}({:#x})", arguments[0]);
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
        SYS_FILE_IOCTL => sys_file_ioctl(arguments[0] as i32, arguments[1], arguments[2]),
        SYS_FILE_DUP => sys_file_dup(arguments[0] as i32, arguments[1], arguments[2]),
        SYS_FILE_DUP2 => sys_file_dup2(arguments[0] as i32, arguments[1] as i32, arguments[2]),
        SYS_PROCESS_FORK => sys_process_fork(frame),
        SYS_PROCESS_EXEC => sys_process_exec(frame, arguments[0], arguments[1], arguments[2]),
        SYS_PROCESS_WAIT => sys_process_wait(arguments[0] as i64, arguments[1], arguments[2]),
        SYS_PROCESS_GETPID => sys_process_getpid(),
        SYS_PROCESS_GETPPID => Ok(proc::current_parent_pid() as u64),
        SYS_PROCESS_GETPGID => sys_process_getpgid(arguments[0]),
        SYS_PROCESS_SETPGID => sys_process_setpgid(arguments[0], arguments[1]),
        SYS_PROCESS_GETSID => sys_process_getsid(arguments[0]),
        SYS_PROCESS_SETSID => proc::create_session()
            .map(|value| value as u64)
            .map_err(map_process_error),
        SYS_PROCESS_KILL => sys_process_kill(arguments[0] as i64, arguments[1] as i32),
        SYS_FILE_CHDIR => sys_file_chdir(arguments[0]),
        SYS_FILE_GETCWD => sys_file_getcwd(arguments[0], arguments[1]),
        SYS_FILE_ACCESS => sys_file_access(arguments[0], arguments[1]),
        SYS_FILE_STAT => sys_file_stat(arguments[0], arguments[1], arguments[2]),
        SYS_FILE_FSTAT => sys_file_fstat(arguments[0] as i32, arguments[1]),
        SYS_FILE_FCNTL => sys_file_fcntl(arguments[0] as i32, arguments[1], arguments[2]),
        SYS_FILE_PIPE => sys_file_pipe(arguments[0], arguments[1]),
        SYS_FILE_READDIR => sys_file_readdir(arguments[0] as i32, arguments[1], arguments[2]),
        SYS_FILE_READLINK => sys_file_readlink(arguments[0], arguments[1], arguments[2]),
        SYS_FILE_UNLINK => sys_file_unlink(arguments[0], arguments[1]),
        SYS_CLOCK_SLEEP => sys_clock_sleep(arguments[0], arguments[1]),
        SYS_FILE_MKDIR => sys_file_mkdir(arguments[0], arguments[1]),
        SYS_PROCESS_GETTID => Ok(crate::arch::thiscpu().current_thread as u64),
        SYS_FILE_FCHMOD => sys_file_fchmod(arguments[0] as i32, arguments[1]),
        _ => Err(Errno::NotImplemented),
    };
    let value = match result {
        Ok(value) => value as i64,
        Err(error) => -(error as i64),
    };
    // log::info!(
    //     "strace[{pid}:{tid}]: {name}({:#x}, {:#x}, {:#x}, {:#x}, {:#x}, {:#x}) = {value}",
    //     arguments[0],
    //     arguments[1],
    //     arguments[2],
    //     arguments[3],
    //     arguments[4],
    //     arguments[5],
    // );
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
        SYS_FILE_IOCTL => "ioctl",
        SYS_FILE_DUP => "dup",
        SYS_FILE_DUP2 => "dup2",
        SYS_PROCESS_FORK => "fork",
        SYS_PROCESS_EXEC => "execve",
        SYS_PROCESS_WAIT => "waitpid",
        SYS_PROCESS_GETPID => "getpid",
        SYS_PROCESS_GETPPID => "getppid",
        SYS_PROCESS_GETPGID => "getpgid",
        SYS_PROCESS_SETPGID => "setpgid",
        SYS_PROCESS_GETSID => "getsid",
        SYS_PROCESS_SETSID => "setsid",
        SYS_PROCESS_KILL => "kill",
        SYS_FILE_CHDIR => "chdir",
        SYS_FILE_GETCWD => "getcwd",
        SYS_FILE_ACCESS => "access",
        SYS_FILE_STAT => "stat",
        SYS_FILE_FSTAT => "fstat",
        SYS_FILE_FCNTL => "fcntl",
        SYS_FILE_PIPE => "pipe",
        SYS_FILE_READDIR => "readdir",
        SYS_FILE_READLINK => "readlink",
        SYS_FILE_UNLINK => "unlink",
        SYS_CLOCK_SLEEP => "nanosleep",
        SYS_FILE_MKDIR => "mkdir",
        SYS_PROCESS_GETTID => "gettid",
        SYS_FILE_FCHMOD => "fchmod",
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
    let path = process.resolve_path(&read_user_string(&process, path)?);
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
    if flags & O_NONBLOCK != 0 {
        open_flags |= OpenFlags::NONBLOCK;
    }
    if flags & O_NOCTTY != 0 {
        open_flags |= OpenFlags::NOCTTY;
    }

    let file = fs::open(&path, open_flags, mode as u16).map_err(map_fs_error)?;
    let fd = process
        .install_file(file.clone(), flags & O_CLOEXEC != 0)
        .map_err(map_process_error)?;
    if flags & O_NOCTTY == 0
        && process.session() == process.pid()
        && !process.has_controlling_tty()
        && file
            .ioctl(
                ioctl_context(&process)?,
                crate::dev::console::TIOCSCTTY,
                0,
                &mut [],
            )
            .is_ok()
    {
        process.set_controlling_tty(file.clone());
    }
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
        Descriptor::File(file) => file.read(&mut bytes).map_err(map_fs_error)?,
        Descriptor::Pipe(pipe) => pipe.read(&mut bytes).map_err(map_pipe_error)?,
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
        Descriptor::File(file) => file.write(&bytes).map_err(map_fs_error)?,
        Descriptor::Pipe(pipe) => pipe.write(&bytes).map_err(map_pipe_error)?,
    };
    Ok(written as u64)
}

fn sys_file_seek(fd: i32, offset: i64, whence: u64) -> Result<u64> {
    let process = current_process()?;
    let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
    let Descriptor::File(file) = descriptor else {
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

fn sys_file_ioctl(fd: i32, request: u64, argument: u64) -> Result<u64> {
    let process = current_process()?;
    let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
    let Descriptor::File(file) = descriptor else {
        return Err(Errno::NotTty);
    };
    let spec = crate::dev::console::ioctl_spec(request);
    if spec.size > MAX_IOCTL_SIZE {
        return Err(Errno::Invalid);
    }
    let mut bytes = vec![0u8; spec.size];
    if spec.input && spec.size != 0 {
        process
            .address_space()
            .read_user(VirtAddr::new(argument), &mut bytes)
            .map_err(map_memory_error)?;
    }
    if request == crate::dev::console::TIOCSPGRP {
        let group = i32::from_ne_bytes(bytes.as_slice().try_into().map_err(|_| Errno::Invalid)?);
        if group <= 0 || !proc::process_group_in_session(group as usize, process.session()) {
            return Err(Errno::Permission);
        }
        if request == crate::dev::console::TIOCSCTTY
            && process
                .controlling_tty_key()
                .is_some_and(|key| key != file.vnode().key())
        {
            return Err(Errno::Access);
        }
    }
    let result = file
        .ioctl(ioctl_context(&process)?, request, argument, &mut bytes)
        .map_err(map_fs_error)?;
    match request {
        crate::dev::console::TIOCSCTTY => process.set_controlling_tty(file.clone()),
        crate::dev::console::TIOCNOTTY if process.session() == process.pid() => {
            proc::clear_session_controlling_tty(process.session(), file.vnode().key());
        }
        crate::dev::console::TIOCNOTTY => process.clear_controlling_tty(file.vnode().key()),
        _ => {}
    }
    if spec.output && spec.size != 0 {
        process
            .address_space()
            .write_user(VirtAddr::new(argument), &bytes)
            .map_err(map_memory_error)?;
    }
    Ok(result)
}

fn ioctl_context(process: &Process) -> Result<IoctlContext> {
    Ok(IoctlContext {
        process_id: process.pid(),
        process_group: i32::try_from(process.process_group()).map_err(|_| Errno::Overflow)?,
        session_id: i32::try_from(process.session()).map_err(|_| Errno::Overflow)?,
        is_session_leader: process.session() == process.pid(),
    })
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
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let Descriptor::File(file) = descriptor else {
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

fn sys_file_dup(fd: i32, minimum: u64, flags: u64) -> Result<u64> {
    if flags & !O_CLOEXEC != 0 {
        return Err(Errno::Invalid);
    }
    let minimum = usize::try_from(minimum).map_err(|_| Errno::Invalid)?;
    current_process()?
        .duplicate_descriptor(fd, minimum, flags & O_CLOEXEC != 0)
        .map(|fd| fd as u64)
        .map_err(map_process_error)
}

fn sys_file_dup2(old_fd: i32, new_fd: i32, flags: u64) -> Result<u64> {
    if flags & !O_CLOEXEC != 0 || (old_fd == new_fd && flags != 0) {
        return Err(Errno::Invalid);
    }
    current_process()?
        .duplicate_descriptor_to(old_fd, new_fd, flags & O_CLOEXEC != 0)
        .map(|fd| fd as u64)
        .map_err(map_process_error)
}

fn sys_process_fork(frame: &TrapFrame) -> Result<u64> {
    proc::fork_current(frame)
        .map(|pid| pid as u64)
        .map_err(map_process_error)
}

fn sys_process_exec(
    frame: &mut TrapFrame,
    path: u64,
    arguments: u64,
    environment: u64,
) -> Result<u64> {
    let process = current_process()?;
    let path = read_user_string(&process, path)?;
    let arguments = read_user_string_array(&process, arguments)?;
    let environment = read_user_string_array(&process, environment)?;
    proc::exec_current(frame, &path, &arguments, &environment).map_err(map_process_error)?;
    Ok(0)
}

fn sys_process_wait(selector: i64, status: u64, flags: u64) -> Result<u64> {
    if flags & !(WNOHANG | WUNTRACED | WCONTINUED) != 0 {
        return Err(Errno::Invalid);
    }
    let selector = isize::try_from(selector).map_err(|_| Errno::Invalid)?;
    let Some((pid, encoded_status)) =
        proc::wait_current(selector, flags & WNOHANG != 0).map_err(map_process_error)?
    else {
        return Ok(0);
    };
    if status != 0 {
        current_process()?
            .address_space()
            .write_user(VirtAddr::new(status), &encoded_status.to_ne_bytes())
            .map_err(map_memory_error)?;
    }
    Ok(pid as u64)
}

fn sys_process_getpid() -> Result<u64> {
    Ok(current_process()?.pid() as u64)
}

fn sys_process_getpgid(pid: u64) -> Result<u64> {
    proc::get_process_group(usize::try_from(pid).map_err(|_| Errno::Invalid)?)
        .map(|value| value as u64)
        .map_err(map_process_error)
}

fn sys_process_setpgid(pid: u64, group: u64) -> Result<u64> {
    proc::set_process_group(
        usize::try_from(pid).map_err(|_| Errno::Invalid)?,
        usize::try_from(group).map_err(|_| Errno::Invalid)?,
    )
    .map_err(map_process_error)?;
    Ok(0)
}

fn sys_process_getsid(pid: u64) -> Result<u64> {
    proc::get_session(usize::try_from(pid).map_err(|_| Errno::Invalid)?)
        .map(|value| value as u64)
        .map_err(map_process_error)
}

fn sys_process_kill(pid: i64, signal: i32) -> Result<u64> {
    if signal < 0 {
        return Err(Errno::Invalid);
    }
    if pid == 0 || pid == -1 || pid < -1 {
        return Ok(0);
    }
    let pid = usize::try_from(pid).map_err(|_| Errno::Invalid)?;
    if proc::find(pid).is_none() {
        return Err(Errno::NoProcess);
    }
    if signal == 0 {
        Ok(0)
    } else {
        Err(Errno::NotSupported)
    }
}

fn sys_file_chdir(path: u64) -> Result<u64> {
    let process = current_process()?;
    let path = read_user_string(&process, path)?;
    process.set_cwd(&path).map_err(map_process_error)?;
    Ok(0)
}

fn sys_file_getcwd(buffer: u64, size: u64) -> Result<u64> {
    let size = usize::try_from(size).map_err(|_| Errno::Overflow)?;
    let process = current_process()?;
    let cwd = process.cwd();
    let required = cwd.len().checked_add(1).ok_or(Errno::Overflow)?;
    if size < required {
        return Err(Errno::Range);
    }
    let mut bytes = cwd.into_bytes();
    bytes.push(0);
    process
        .address_space()
        .write_user(VirtAddr::new(buffer), &bytes)
        .map_err(map_memory_error)?;
    Ok(buffer)
}

fn sys_file_access(path: u64, mode: u64) -> Result<u64> {
    if mode & !0o7 != 0 {
        return Err(Errno::Invalid);
    }
    let process = current_process()?;
    let path = process.resolve_path(&read_user_string(&process, path)?);
    let attributes = fs::lookup(&path)
        .map_err(map_fs_error)?
        .getattr()
        .map_err(map_fs_error)?;
    if mode & 0o4 != 0 && attributes.mode & 0o444 == 0 {
        return Err(Errno::Access);
    }
    if mode & 0o2 != 0 && attributes.mode & 0o222 == 0 {
        return Err(Errno::Access);
    }
    if mode & 0o1 != 0 && attributes.mode & 0o111 == 0 {
        return Err(Errno::Access);
    }
    Ok(0)
}

fn sys_file_stat(path: u64, flags: u64, output: u64) -> Result<u64> {
    if flags & !AT_SYMLINK_NOFOLLOW != 0 {
        return Err(Errno::Invalid);
    }
    let process = current_process()?;
    let path = process.resolve_path(&read_user_string(&process, path)?);
    let vnode = if flags & AT_SYMLINK_NOFOLLOW != 0 {
        fs::lookup_nofollow(&path)
    } else {
        fs::lookup(&path)
    }
    .map_err(map_fs_error)?;
    write_user_stat(&process, output, vnode.getattr().map_err(map_fs_error)?)
}

fn sys_file_fstat(fd: i32, output: u64) -> Result<u64> {
    let process = current_process()?;
    let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
    match descriptor {
        Descriptor::File(file) => {
            write_user_stat(&process, output, file.getattr().map_err(map_fs_error)?)
        }
        Descriptor::Pipe(_) => write_user_stat_values(&process, output, 0, 0, 0o666, 6, 1, 0),
    }
}

fn sys_file_fcntl(fd: i32, request: u64, argument: u64) -> Result<u64> {
    let process = current_process()?;
    match request {
        F_DUPFD => sys_file_dup(fd, argument, 0),
        F_DUPFD_CLOEXEC => sys_file_dup(fd, argument, O_CLOEXEC),
        F_GETFD => process
            .descriptor_close_on_exec(fd)
            .map(|value| if value { FD_CLOEXEC } else { 0 })
            .ok_or(Errno::BadFileDescriptor),
        F_SETFD => {
            if argument & !FD_CLOEXEC != 0 {
                return Err(Errno::Invalid);
            }
            process
                .set_descriptor_close_on_exec(fd, argument & FD_CLOEXEC != 0)
                .then_some(0)
                .ok_or(Errno::BadFileDescriptor)
        }
        F_GETFL => match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
            Descriptor::File(file) => Ok(open_flags_to_user(file.flags())),
            Descriptor::Pipe(pipe) => {
                Ok(pipe.access_mode() | if pipe.is_nonblocking() { O_NONBLOCK } else { 0 })
            }
        },
        F_SETFL => {
            if argument & !(O_ACCMODE | O_APPEND | O_NONBLOCK) != 0 {
                return Err(Errno::Invalid);
            }
            match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
                Descriptor::File(file) => {
                    file.set_status_flags(argument & O_APPEND != 0, argument & O_NONBLOCK != 0)
                }
                Descriptor::Pipe(pipe) => pipe.set_nonblocking(argument & O_NONBLOCK != 0),
            }
            Ok(0)
        }
        _ => Err(Errno::Invalid),
    }
}

fn sys_file_pipe(output: u64, flags: u64) -> Result<u64> {
    if flags & !PIPE_CLOEXEC != 0 {
        return Err(Errno::Invalid);
    }
    let process = current_process()?;
    let close_on_exec = flags & PIPE_CLOEXEC != 0;
    let (reader, writer) = PipeEnd::pair();
    let read_fd = process
        .install_descriptor_value(Descriptor::Pipe(reader), close_on_exec)
        .map_err(map_process_error)?;
    let write_fd = match process
        .install_descriptor_value(Descriptor::Pipe(writer), close_on_exec)
        .map_err(map_process_error)
    {
        Ok(fd) => fd,
        Err(error) => {
            process.close_descriptor(read_fd);
            return Err(error);
        }
    };
    let mut bytes = [0u8; size_of::<i32>() * 2];
    bytes[..4].copy_from_slice(&read_fd.to_ne_bytes());
    bytes[4..].copy_from_slice(&write_fd.to_ne_bytes());
    if let Err(error) = process
        .address_space()
        .write_user(VirtAddr::new(output), &bytes)
        .map_err(map_memory_error)
    {
        process.close_descriptor(read_fd);
        process.close_descriptor(write_fd);
        return Err(error);
    }
    Ok(0)
}

fn sys_file_readdir(fd: i32, buffer: u64, size: u64) -> Result<u64> {
    let size = checked_io_size(size)?;
    let maximum = size / USER_DIRENT_SIZE;
    if maximum == 0 {
        return Err(Errno::Invalid);
    }
    let process = current_process()?;
    let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
    let Descriptor::File(file) = descriptor else {
        return Err(Errno::NotDirectory);
    };
    let entries = file.readdir(maximum).map_err(map_fs_error)?;
    let mut output = vec![0u8; entries.len() * USER_DIRENT_SIZE];
    for (index, entry) in entries.iter().enumerate() {
        let record = &mut output[index * USER_DIRENT_SIZE..][..USER_DIRENT_SIZE];
        record[..8].copy_from_slice(&entry.key.node.get().to_ne_bytes());
        record[8..16].copy_from_slice(&0i64.to_ne_bytes());
        record[16..18].copy_from_slice(&(USER_DIRENT_SIZE as u16).to_ne_bytes());
        record[18] = dirent_kind(entry.kind);
        let length = entry.name.len().min(255);
        record[19..19 + length].copy_from_slice(&entry.name[..length]);
        record[19 + length] = 0;
    }
    process
        .address_space()
        .write_user(VirtAddr::new(buffer), &output)
        .map_err(map_memory_error)?;
    Ok(output.len() as u64)
}

fn sys_file_readlink(path: u64, buffer: u64, size: u64) -> Result<u64> {
    let size = checked_io_size(size)?;
    let process = current_process()?;
    let path = process.resolve_path(&read_user_string(&process, path)?);
    let target = fs::lookup_nofollow(&path)
        .map_err(map_fs_error)?
        .readlink()
        .map_err(map_fs_error)?;
    let length = target.len().min(size);
    process
        .address_space()
        .write_user(VirtAddr::new(buffer), &target[..length])
        .map_err(map_memory_error)?;
    Ok(length as u64)
}

fn sys_file_unlink(path: u64, flags: u64) -> Result<u64> {
    if flags & !AT_REMOVEDIR != 0 {
        return Err(Errno::Invalid);
    }
    let process = current_process()?;
    let path = process.resolve_path(&read_user_string(&process, path)?);
    if flags & AT_REMOVEDIR != 0 {
        fs::remove_dir(&path)
    } else {
        fs::unlink(&path)
    }
    .map_err(map_fs_error)?;
    Ok(0)
}

fn sys_clock_sleep(seconds: u64, nanoseconds: u64) -> Result<u64> {
    if nanoseconds >= 1_000_000_000 {
        return Err(Errno::Invalid);
    }
    clock::sleep(Duration::new(seconds, nanoseconds as u32));
    Ok(0)
}

fn sys_file_mkdir(path: u64, mode: u64) -> Result<u64> {
    let process = current_process()?;
    let path = process.resolve_path(&read_user_string(&process, path)?);
    fs::create_dir(&path, mode as u16).map_err(map_fs_error)?;
    Ok(0)
}

fn sys_file_fchmod(fd: i32, mode: u64) -> Result<u64> {
    let process = current_process()?;
    let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
    let Descriptor::File(file) = descriptor else {
        return Err(Errno::BadFileDescriptor);
    };
    file.set_mode(mode as u16).map_err(map_fs_error)?;
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

fn read_user_string_array(process: &Process, address: u64) -> Result<Vec<String>> {
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

fn read_user_u64(process: &Process, address: u64) -> Result<u64> {
    let mut bytes = [0u8; size_of::<u64>()];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(u64::from_ne_bytes(bytes))
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

fn write_user_stat(process: &Process, output: u64, attributes: VnodeAttr) -> Result<u64> {
    write_user_stat_values(
        process,
        output,
        attributes.key.filesystem.get(),
        attributes.key.node.get(),
        u32::from(attributes.mode),
        vnode_kind(attributes.kind),
        attributes.links,
        attributes.size,
    )?;
    let mut times = [0u8; 24];
    times[0..8].copy_from_slice(&attributes.accessed_ns.to_ne_bytes());
    times[8..16].copy_from_slice(&attributes.modified_ns.to_ne_bytes());
    times[16..24].copy_from_slice(&attributes.changed_ns.to_ne_bytes());
    let times_output = output.checked_add(40).ok_or(Errno::Fault)?;
    process
        .address_space()
        .write_user(VirtAddr::new(times_output), &times)
        .map_err(map_memory_error)?;
    Ok(0)
}

#[allow(clippy::too_many_arguments)]
fn write_user_stat_values(
    process: &Process,
    output: u64,
    device: u64,
    inode: u64,
    mode: u32,
    kind: u32,
    links: u64,
    size: u64,
) -> Result<u64> {
    let mut bytes = [0u8; 64];
    bytes[0..8].copy_from_slice(&device.to_ne_bytes());
    bytes[8..16].copy_from_slice(&inode.to_ne_bytes());
    bytes[16..20].copy_from_slice(&mode.to_ne_bytes());
    bytes[20..24].copy_from_slice(&kind.to_ne_bytes());
    bytes[24..32].copy_from_slice(&links.to_ne_bytes());
    bytes[32..40].copy_from_slice(&size.to_ne_bytes());
    process
        .address_space()
        .write_user(VirtAddr::new(output), &bytes)
        .map_err(map_memory_error)?;
    Ok(0)
}

fn vnode_kind(kind: VnodeKind) -> u32 {
    match kind {
        VnodeKind::Regular => 1,
        VnodeKind::Directory => 2,
        VnodeKind::Symlink => 3,
        VnodeKind::CharacterDevice => 4,
        VnodeKind::BlockDevice => 5,
        VnodeKind::Fifo => 6,
        VnodeKind::Socket => 7,
    }
}

fn dirent_kind(kind: VnodeKind) -> u8 {
    match kind {
        VnodeKind::Regular => 8,
        VnodeKind::Directory => 4,
        VnodeKind::Symlink => 10,
        VnodeKind::CharacterDevice => 2,
        VnodeKind::BlockDevice => 6,
        VnodeKind::Fifo => 1,
        VnodeKind::Socket => 12,
    }
}

fn open_flags_to_user(flags: OpenFlags) -> u64 {
    let mut result = match (
        flags.contains(OpenFlags::READ),
        flags.contains(OpenFlags::WRITE),
    ) {
        (true, true) => O_RDWR,
        (false, true) => O_WRONLY,
        _ => 0,
    };
    if flags.contains(OpenFlags::APPEND) {
        result |= O_APPEND;
    }
    if flags.contains(OpenFlags::NONBLOCK) {
        result |= O_NONBLOCK;
    }
    result
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
        proc::Error::BadFileDescriptor => Errno::BadFileDescriptor,
        proc::Error::NoSuchProcess => Errno::NoProcess,
        proc::Error::NoChild => Errno::NoChild,
        proc::Error::PermissionDenied => Errno::Permission,
        proc::Error::InvalidArgument | proc::Error::InvalidElf(_) | proc::Error::UnsupportedElf => {
            Errno::Invalid
        }
    }
}

fn map_pipe_error(error: PipeError) -> Errno {
    match error {
        PipeError::BadDescriptor => Errno::BadFileDescriptor,
        PipeError::TryAgain => Errno::TryAgain,
        PipeError::BrokenPipe => Errno::BrokenPipe,
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
