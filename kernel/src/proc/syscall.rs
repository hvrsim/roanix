//! Process, thread, and futex syscall implementations.

use alloc::{
    collections::{BTreeMap, VecDeque},
    sync::Arc,
};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::{
    proc,
    sys::{
        clock,
        event::Event,
        sync::{Mutex, Once},
    },
    syscall::{
        Errno, current_process, map_memory_error, map_process_error, read_user_bytes,
        read_user_i32, read_user_path, read_user_path_array, read_user_timespec,
    },
};

const WNOHANG: u64 = 1;
const WUNTRACED: u64 = 2;
const WCONTINUED: u64 = 8;

/// Longest thread name the kernel stores, excluding the terminator.
const THREAD_NAME_CAPACITY: usize = crate::sys::thread::ThreadName::CAPACITY;

static FUTEXES: Once<Mutex<BTreeMap<FutexKey, VecDeque<Arc<FutexWaiter>>>>> = Once::new();

#[derive(Copy, Clone, Eq, Ord, PartialEq, PartialOrd)]
struct FutexKey {
    address_space: usize,
    address: u64,
}

/// One thread parked on a futex address.
///
/// Every waiter owns its own event rather than sharing one per address. The
/// event carries a persistent signaled state, so a wake that lands between the
/// point where the waiter is published on the queue and the point where it
/// actually blocks is still observed instead of being lost.
struct FutexWaiter {
    event: Event,
    woken: AtomicBool,
}

impl FutexWaiter {
    fn new() -> Self {
        Self {
            event: Event::new(),
            woken: AtomicBool::new(false),
        }
    }

    fn was_woken(&self) -> bool {
        self.woken.load(Ordering::Acquire)
    }

    /// Marks this waiter as woken by `futex_wake` and releases it.
    fn wake(&self) {
        self.woken.store(true, Ordering::Release);
        self.event.signal();
    }
}

/// Why a futex wait stopped blocking.
enum FutexOutcome {
    /// The futex event fired.
    Woken,

    /// A signal became deliverable.
    Interrupted,

    /// The caller-supplied deadline elapsed.
    TimedOut,

    /// The word changed, or could not be read, before blocking began.
    Skipped,
}

/// Index of the signal-interruption event in a futex wait set.
const INTERRUPT_EVENT: usize = 1;

/// Publishes `waiter` on the queue for `key` so wakers can find it.
fn futex_enqueue(key: FutexKey, waiter: &Arc<FutexWaiter>) {
    futexes()
        .lock()
        .entry(key)
        .or_default()
        .push_back(waiter.clone());
}

/// Removes `waiter` from the queue for `key`, reclaiming an emptied queue.
///
/// Returns whether the waiter was still queued, which distinguishes a timeout
/// or a signal from a wake that raced with them.
fn futex_dequeue(key: FutexKey, waiter: &Arc<FutexWaiter>) -> bool {
    let mut futexes = futexes().lock();
    let Some(queue) = futexes.get_mut(&key) else {
        return false;
    };
    let removed = match queue.iter().position(|entry| Arc::ptr_eq(entry, waiter)) {
        Some(index) => {
            queue.remove(index);
            true
        }
        None => false,
    };
    if queue.is_empty() {
        futexes.remove(&key);
    }
    removed
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
        let key = FutexKey {
            address_space: Arc::as_ptr(&process.address_space()) as usize,
            address,
        };
        // Fast path, and it also faults the word in before the comparison that
        // decides whether to block is repeated below.
        if read_user_i32(&process, address)? != expected {
            return Err(Errno::TryAgain);
        }

        let duration = if timeout == 0 {
            None
        } else {
            Some(read_user_timespec(&process, timeout)?)
        };

        let waiter = Arc::new(FutexWaiter::new());
        futex_enqueue(key, &waiter);

        // The waiter is visible to wakers from here on, so re-reading the word
        // now closes the race where the value changes and the matching wake
        // runs before this thread manages to block.
        let recheck = read_user_i32(&process, address);
        let outcome = match recheck {
            Ok(value) if value == expected => {
                if process.prepare_interrupt_wait() {
                    FutexOutcome::Interrupted
                } else {
                    let events = [&waiter.event, process.interrupt_event()];
                    match duration {
                        Some(duration) => match clock::wait_any_timeout(&events, duration) {
                            None => FutexOutcome::TimedOut,
                            Some(INTERRUPT_EVENT) => FutexOutcome::Interrupted,
                            Some(_) => FutexOutcome::Woken,
                        },
                        None => match Event::wait_any(&events) {
                            INTERRUPT_EVENT => FutexOutcome::Interrupted,
                            _ => FutexOutcome::Woken,
                        },
                    }
                }
            }
            _ => FutexOutcome::Skipped,
        };

        futex_dequeue(key, &waiter);

        // A wake that raced with a timeout or a signal still counts as a wake:
        // the waiter had already been taken off the queue, so reporting the
        // race instead would drop that wakeup entirely.
        if waiter.was_woken() {
            return Ok(0);
        }

        match outcome {
            // A bare futex wake without the flag is a spurious wakeup, which
            // the futex contract allows callers to see.
            FutexOutcome::Woken => Ok(0),
            FutexOutcome::Interrupted => Err(Errno::Interrupted),
            FutexOutcome::TimedOut => Err(Errno::TimedOut),
            FutexOutcome::Skipped => match recheck? {
                value if value != expected => Err(Errno::TryAgain),
                _ => Ok(0),
            },
        }
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

        // Detach the chosen waiters under the lock but release them outside it,
        // so a woken thread never has to wait on this lock to make progress.
        let woken = {
            let mut futexes = futexes().lock();
            let Some(queue) = futexes.get_mut(&key) else {
                return Ok(0);
            };
            let count = queue.len().min(maximum as usize);
            let woken: alloc::vec::Vec<_> = queue.drain(..count).collect();
            if queue.is_empty() {
                futexes.remove(&key);
            }
            woken
        };

        for waiter in &woken {
            waiter.wake();
        }
        Ok(woken.len() as u64)
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
        let path = read_user_path(&process, path)?;
        let arguments = read_user_path_array(&process, arguments)?;
        let environment = read_user_path_array(&process, environment)?;
        proc::exec_current(frame, &path, &arguments, &environment)
            .map_err(map_process_error)?;
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

crate::syscall_handler! {
    syscall_thread_yield(_frame) {
        crate::sys::sched::yield_current();
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_thread_kill(_frame, pid: u64 = 0, tid: u64 = 1, signal: i32 = 2) {
        let pid = usize::try_from(pid).map_err(|_| Errno::Invalid)?;
        let tid = usize::try_from(tid).map_err(|_| Errno::Invalid)?;
        proc::signal::kill_thread(pid, tid, signal).map_err(map_process_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_thread_set_name(_frame, tid: u64 = 0, name: u64 = 1) {
        let tid = usize::try_from(tid).map_err(|_| Errno::Invalid)?;
        let process = current_process()?;
        // Reject rather than truncate an oversized name, matching
        // `pthread_setname_np`, which reports `ERANGE` instead of silently
        // storing a shortened name.
        let name = read_user_bytes(&process, name, THREAD_NAME_CAPACITY + 1)
            .map_err(|_| Errno::Range)?;
        if name.len() > THREAD_NAME_CAPACITY {
            return Err(Errno::Range);
        }
        if !crate::sys::sched::set_thread_name(tid, process.pid(), &name) {
            return Err(Errno::NoProcess);
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_thread_get_name(_frame, tid: u64 = 0, buffer: u64 = 1, length: u64 = 2) {
        let tid = usize::try_from(tid).map_err(|_| Errno::Invalid)?;
        let process = current_process()?;
        let Some((name, name_length)) = crate::sys::sched::thread_name(tid, process.pid()) else {
            return Err(Errno::NoProcess);
        };
        // Userspace supplies the buffer for the name plus its terminator.
        if length <= name_length as u64 {
            return Err(Errno::Range);
        }
        let mut record = [0u8; THREAD_NAME_CAPACITY + 1];
        record[..name_length].copy_from_slice(&name[..name_length]);
        process
            .address_space()
            .write_user(crate::mem::VirtAddr::new(buffer), &record[..name_length + 1])
            .map_err(map_memory_error)?;
        Ok(0)
    }
}

fn futexes() -> &'static Mutex<BTreeMap<FutexKey, VecDeque<Arc<FutexWaiter>>>> {
    FUTEXES.call_once(|| Mutex::new(BTreeMap::new()))
}
