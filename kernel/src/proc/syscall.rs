//! Process, thread, and futex syscall implementations.

use alloc::{collections::BTreeMap, sync::Arc};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::{
    proc,
    sys::{
        clock,
        event::Event,
        sync::{Mutex, Once},
    },
    syscall::{
        Errno, current_process, map_memory_error, map_process_error, read_user_i32,
        read_user_string, read_user_string_array, read_user_timespec,
    },
};

const WNOHANG: u64 = 1;
const WUNTRACED: u64 = 2;
const WCONTINUED: u64 = 8;

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

crate::syscall_handler! {
    syscall_process_exit(_frame, code: i32 = 0) -> ! {
        proc::exit_current(code)
    }
}

crate::syscall_handler! {
    syscall_thread_set_pointer(_frame, pointer: u64 = 0) {
        proc::set_current_thread_pointer(pointer).map_err(map_process_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_futex_wait(_frame, address: u64 = 0, expected: i32 = 1, timeout: u64 = 2) {
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
}

crate::syscall_handler! {
    syscall_futex_wake(_frame, address: u64 = 0, maximum: u64 = 1) {
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
}

crate::syscall_handler! {
    syscall_process_fork(frame) {
        proc::fork_current(frame)
            .map(|pid| pid as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_process_exec(frame, path: u64 = 0, arguments: u64 = 1, environment: u64 = 2) {
        let process = current_process()?;
        let path = read_user_string(&process, path)?;
        let arguments = read_user_string_array(&process, arguments)?;
        let environment = read_user_string_array(&process, environment)?;
        proc::exec_current(frame, &path, &arguments, &environment).map_err(map_process_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_process_wait(_frame, selector: i64 = 0, status: u64 = 1, flags: u64 = 2) {
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
                .write_user(
                    crate::mem::VirtAddr::new(status),
                    &encoded_status.to_ne_bytes(),
                )
                .map_err(map_memory_error)?;
        }
        Ok(pid as u64)
    }
}

crate::syscall_handler! {
    syscall_process_getpid(_frame) {
        Ok(current_process()?.pid() as u64)
    }
}

crate::syscall_handler! {
    syscall_process_getppid(_frame) {
        Ok(proc::current_parent_pid() as u64)
    }
}

crate::syscall_handler! {
    syscall_process_getpgid(_frame, pid: u64 = 0) {
        proc::get_process_group(usize::try_from(pid).map_err(|_| Errno::Invalid)?)
            .map(|value| value as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_process_setpgid(_frame, pid: u64 = 0, group: u64 = 1) {
        proc::set_process_group(
            usize::try_from(pid).map_err(|_| Errno::Invalid)?,
            usize::try_from(group).map_err(|_| Errno::Invalid)?,
        )
        .map_err(map_process_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_process_getsid(_frame, pid: u64 = 0) {
        proc::get_session(usize::try_from(pid).map_err(|_| Errno::Invalid)?)
            .map(|value| value as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_process_setsid(_frame) {
        proc::create_session()
            .map(|value| value as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_process_kill(_frame, pid: i64 = 0, signal: i32 = 1) {
        proc::signal::kill(pid, signal).map_err(map_process_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_process_gettid(_frame) {
        Ok(crate::arch::thiscpu().current_thread as u64)
    }
}

crate::syscall_handler! {
    syscall_thread_create(
        _frame,
        entry: u64 = 0,
        stack: u64 = 1,
        thread_pointer: u64 = 2,
    ) {
        proc::create_thread(entry, stack, thread_pointer)
            .map(|tid| tid as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_thread_exit(_frame) -> ! {
        proc::exit_current_thread()
    }
}

fn futexes() -> &'static Mutex<BTreeMap<FutexKey, Arc<Futex>>> {
    FUTEXES.call_once(|| Mutex::new(BTreeMap::new()))
}
