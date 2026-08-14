//! timerfd clock-backed expiration descriptors.

use alloc::sync::Arc;
use core::{
    mem::size_of,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

use crate::{
    mem::IoSink,
    fs::PollEvents,
    mem::VirtAddr,
    proc::Descriptor,
    sys::{clock, event::Event, sync::Mutex},
    syscall::{Errno, Result, current_process, map_memory_error, map_process_error},
};

const CLOCK_REALTIME: i32 = 0;
const CLOCK_MONOTONIC: i32 = 1;
const CLOCK_BOOTTIME: i32 = 7;

const TFD_NONBLOCK: u64 = 0o4000;
const TFD_CLOEXEC: u64 = 0o2000000;
const TFD_TIMER_ABSTIME: u64 = 1;
const TFD_TIMER_CANCEL_ON_SET: u64 = 1 << 1;
const NANOSECONDS_PER_SECOND: u64 = 1_000_000_000;
const ITIMERSPEC_SIZE: usize = 32;

struct TimerState {
    deadline_ns: u64,
    interval_ns: u64,
    expirations: u64,
}

/// One timerfd open-file description.
pub(crate) struct TimerFd {
    clock_id: i32,
    state: Mutex<TimerState>,
    changed: Event,
    nonblocking: AtomicBool,
}

impl TimerFd {
    fn new(clock_id: i32, nonblocking: bool) -> Arc<Self> {
        Arc::new(Self {
            clock_id,
            state: Mutex::new(TimerState {
                deadline_ns: 0,
                interval_ns: 0,
                expirations: 0,
            }),
            changed: Event::new(),
            nonblocking: AtomicBool::new(nonblocking),
        })
    }

    fn set_time(&self, flags: u64, interval_ns: u64, value_ns: u64) -> Result<TimerState> {
        if flags & !(TFD_TIMER_ABSTIME | TFD_TIMER_CANCEL_ON_SET) != 0
            || flags & TFD_TIMER_CANCEL_ON_SET != 0
                && (self.clock_id != CLOCK_REALTIME || flags & TFD_TIMER_ABSTIME == 0)
        {
            return Err(Errno::Invalid);
        }

        let now = clock::monotonic_ns();
        let mut state = self.state.lock();
        refresh(&mut state, now);
        let previous = TimerState {
            deadline_ns: state.deadline_ns,
            interval_ns: state.interval_ns,
            expirations: state.expirations,
        };
        state.deadline_ns = if value_ns == 0 {
            0
        } else if flags & TFD_TIMER_ABSTIME != 0 {
            value_ns
        } else {
            now.saturating_add(value_ns)
        };
        state.interval_ns = interval_ns;
        state.expirations = 0;
        drop(state);
        self.changed.signal();
        Ok(previous)
    }

    fn current_time(&self) -> TimerState {
        let now = clock::monotonic_ns();
        let mut state = self.state.lock();
        refresh(&mut state, now);
        TimerState {
            deadline_ns: state.deadline_ns,
            interval_ns: state.interval_ns,
            expirations: state.expirations,
        }
    }

    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize> {
        if sink.len() < size_of::<u64>() {
            return Err(Errno::Invalid);
        }

        loop {
            let now = clock::monotonic_ns();
            let deadline = {
                let mut state = self.state.lock();
                refresh(&mut state, now);
                if state.expirations != 0 {
                    // The count is cleared only once it has reached user
                    // memory, so a faulting copy cannot lose expirations.
                    sink.store(0, &state.expirations.to_ne_bytes())
                        .map_err(|_| Errno::Fault)?;
                    state.expirations = 0;
                    return Ok(8);
                }
                if self.nonblocking.load(Ordering::Acquire) {
                    return Err(Errno::TryAgain);
                }
                self.changed.reset();
                state.deadline_ns
            };

            if deadline == 0 {
                self.changed.wait();
                continue;
            }
            let remaining = deadline.saturating_sub(clock::monotonic_ns());
            let _ = clock::wait_timeout(&self.changed, Duration::from_nanos(remaining.max(1)));
        }
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        if !requested.contains(PollEvents::IN) {
            return PollEvents::empty();
        }
        let mut state = self.state.lock();
        refresh(&mut state, clock::monotonic_ns());
        if state.expirations != 0 {
            PollEvents::IN
        } else {
            PollEvents::empty()
        }
    }

    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }

    pub(crate) fn set_nonblocking(&self, nonblocking: bool) {
        self.nonblocking.store(nonblocking, Ordering::Release);
        self.changed.signal();
    }
}

crate::syscall_handler! {
    syscall_timerfd_create(_frame, clock_id: i32 = 0, flags: u64 = 1) {
        if !matches!(clock_id, CLOCK_REALTIME | CLOCK_MONOTONIC | CLOCK_BOOTTIME)
            || flags & !(TFD_NONBLOCK | TFD_CLOEXEC) != 0
        {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let timer = TimerFd::new(clock_id, flags & TFD_NONBLOCK != 0);
        let fd = process
            .install_descriptor_value(
                Descriptor::TimerFd(timer),
                flags & TFD_CLOEXEC != 0,
            )
            .map_err(map_process_error)?;
        Ok(fd as u64)
    }
}

crate::syscall_handler! {
    syscall_timerfd_settime(
        _frame,
        fd: i32 = 0,
        flags: u64 = 1,
        new_value: u64 = 2,
        old_value: u64 = 3,
    ) {
        if new_value == 0 {
            return Err(Errno::Fault);
        }
        let process = current_process()?;
        let Descriptor::TimerFd(timer) =
            process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?
        else {
            return Err(Errno::Invalid);
        };
        let (interval_ns, value_ns) = read_user_itimerspec(&process, new_value)?;
        let previous = timer.set_time(flags, interval_ns, value_ns)?;
        if old_value != 0 {
            write_user_itimerspec(
                &process,
                old_value,
                previous.interval_ns,
                remaining_ns(&previous, clock::monotonic_ns()),
            )?;
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_timerfd_gettime(_frame, fd: i32 = 0, value: u64 = 1) {
        if value == 0 {
            return Err(Errno::Fault);
        }
        let process = current_process()?;
        let Descriptor::TimerFd(timer) =
            process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?
        else {
            return Err(Errno::Invalid);
        };
        let current = timer.current_time();
        write_user_itimerspec(
            &process,
            value,
            current.interval_ns,
            remaining_ns(&current, clock::monotonic_ns()),
        )?;
        Ok(0)
    }
}

fn refresh(state: &mut TimerState, now_ns: u64) {
    if state.deadline_ns == 0 || now_ns < state.deadline_ns {
        return;
    }
    let count = if state.interval_ns == 0 {
        state.deadline_ns = 0;
        1
    } else {
        let count = now_ns
            .saturating_sub(state.deadline_ns)
            .saturating_div(state.interval_ns)
            .saturating_add(1);
        state.deadline_ns = state
            .deadline_ns
            .saturating_add(state.interval_ns.saturating_mul(count));
        count
    };
    state.expirations = state.expirations.saturating_add(count);
}

fn remaining_ns(state: &TimerState, now_ns: u64) -> u64 {
    state.deadline_ns.saturating_sub(now_ns)
}

fn read_user_itimerspec(process: &crate::proc::Process, address: u64) -> Result<(u64, u64)> {
    let mut bytes = [0u8; ITIMERSPEC_SIZE];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok((
        decode_timespec(&bytes[0..16])?,
        decode_timespec(&bytes[16..32])?,
    ))
}

fn write_user_itimerspec(
    process: &crate::proc::Process,
    address: u64,
    interval_ns: u64,
    value_ns: u64,
) -> Result<()> {
    let mut bytes = [0u8; ITIMERSPEC_SIZE];
    encode_timespec(&mut bytes[0..16], interval_ns);
    encode_timespec(&mut bytes[16..32], value_ns);
    process
        .address_space()
        .write_user(VirtAddr::new(address), &bytes)
        .map_err(map_memory_error)
}

fn decode_timespec(bytes: &[u8]) -> Result<u64> {
    let seconds = i64::from_ne_bytes(bytes[0..8].try_into().expect("timespec seconds width"));
    let nanoseconds =
        i64::from_ne_bytes(bytes[8..16].try_into().expect("timespec nanoseconds width"));
    if seconds < 0 || !(0..NANOSECONDS_PER_SECOND as i64).contains(&nanoseconds) {
        return Err(Errno::Invalid);
    }
    (seconds as u64)
        .checked_mul(NANOSECONDS_PER_SECOND)
        .and_then(|value| value.checked_add(nanoseconds as u64))
        .ok_or(Errno::Overflow)
}

fn encode_timespec(bytes: &mut [u8], nanoseconds: u64) {
    let seconds = (nanoseconds / NANOSECONDS_PER_SECOND) as i64;
    let remainder = (nanoseconds % NANOSECONDS_PER_SECOND) as i64;
    bytes[0..8].copy_from_slice(&seconds.to_ne_bytes());
    bytes[8..16].copy_from_slice(&remainder.to_ne_bytes());
}
