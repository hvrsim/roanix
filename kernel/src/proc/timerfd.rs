//! timerfd clock-backed expiration descriptors.

use alloc::{sync::Arc, vec::Vec};
use core::{
    mem::size_of,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

use crate::{
    fs::PollEvents,
    mem::IoSink,
    mem::VirtAddr,
    proc::Descriptor,
    sys::{
        clock::{self, ClockSnapshot},
        event::{Event, EventListener, EventSource},
        sync::Mutex,
    },
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

#[derive(Clone, Copy, Debug)]
struct TimerSetting {
    interval_ns: u64,
    remaining_ns: u64,
}

struct TimerState {
    deadline_ns: u64,
    interval_ns: u64,
    expirations: u64,
    realtime_absolute: bool,
    cancel_generation: Option<u64>,
    canceled: bool,
}

impl TimerState {
    fn empty() -> Self {
        Self {
            deadline_ns: 0,
            interval_ns: 0,
            expirations: 0,
            realtime_absolute: false,
            cancel_generation: None,
            canceled: false,
        }
    }

    fn now(&self, clock: ClockSnapshot) -> u64 {
        if self.realtime_absolute {
            clock.realtime_ns
        } else {
            clock.monotonic_ns
        }
    }

    fn setting(&self, clock: ClockSnapshot) -> TimerSetting {
        TimerSetting {
            interval_ns: self.interval_ns,
            remaining_ns: self.deadline_ns.saturating_sub(self.now(clock)),
        }
    }

    fn refresh(&mut self, clock: ClockSnapshot) {
        if self
            .cancel_generation
            .is_some_and(|epoch| epoch != clock.realtime_generation)
        {
            self.cancel_generation = Some(clock.realtime_generation);
            self.canceled = true;
            self.expirations = 0;
        }
        let now = self.now(clock);
        if self.deadline_ns == 0 || now < self.deadline_ns {
            return;
        }
        // Cancellation consumes pending expirations without restarting a
        // periodic timer that expired during the discontinuity.
        if self.canceled {
            self.deadline_ns = 0;
            return;
        }
        let count = match (now - self.deadline_ns).checked_div(self.interval_ns) {
            None => {
                self.deadline_ns = 0;
                1
            }
            Some(elapsed) => {
                let count = elapsed + 1;
                self.deadline_ns = self
                    .interval_ns
                    .saturating_mul(count)
                    .saturating_add(self.deadline_ns);
                count
            }
        };
        self.expirations = self.expirations.saturating_add(count);
    }
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
            state: Mutex::new(TimerState::empty()),
            changed: Event::new(),
            nonblocking: AtomicBool::new(nonblocking),
        })
    }

    fn set_time(&self, flags: u64, interval_ns: u64, value_ns: u64) -> Result<TimerSetting> {
        if flags & !(TFD_TIMER_ABSTIME | TFD_TIMER_CANCEL_ON_SET) != 0 {
            return Err(Errno::Invalid);
        }
        let mut state = self.state.lock();
        let clock = clock::clock_snapshot();
        let absolute = flags & TFD_TIMER_ABSTIME != 0;
        let realtime_absolute = self.clock_id == CLOCK_REALTIME && absolute;
        let cancel = realtime_absolute && flags & TFD_TIMER_CANCEL_ON_SET != 0;
        let deadline_ns = if value_ns == 0 {
            0
        } else if absolute {
            value_ns
        } else {
            clock.monotonic_ns.saturating_add(value_ns)
        };
        state.refresh(clock);
        let previous = state.setting(clock);
        let was_canceled = state.canceled;
        *state = TimerState {
            deadline_ns,
            interval_ns,
            expirations: 0,
            realtime_absolute,
            cancel_generation: cancel.then_some(clock.realtime_generation),
            canceled: false,
        };
        drop(state);
        self.changed.wake_all();
        // Linux reports an unacknowledged cancellation on rearm while still
        // installing the new cancelable setting. Other settings clear it.
        if was_canceled && cancel {
            Err(Errno::Canceled)
        } else {
            Ok(previous)
        }
    }

    fn current_time(&self) -> TimerSetting {
        let mut state = self.state.lock();
        let clock = clock::clock_snapshot();
        state.refresh(clock);
        state.setting(clock)
    }

    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize> {
        if sink.len() < size_of::<u64>() {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        loop {
            // Capture notifications before sampling state and clock. A settime
            // or discontinuity in the scan/park gap must not be lost.
            let listeners = [
                self.changed.listen(),
                process.interrupt_event().listen(),
                clock::realtime_changed().listen(),
            ];
            let wake_deadline = {
                let mut state = self.state.lock();
                let clock = clock::clock_snapshot();
                state.refresh(clock);
                if state.canceled {
                    state.canceled = false;
                    state.expirations = 0;
                    self.changed.wake_all();
                    return Err(Errno::Canceled);
                }
                if state.expirations != 0 {
                    sink.store(0, &state.expirations.to_ne_bytes())
                        .map_err(|_| Errno::Fault)?;
                    state.expirations = 0;
                    self.changed.wake_all();
                    return Ok(8);
                }
                if self.nonblocking.load(Ordering::Acquire) {
                    return Err(Errno::TryAgain);
                }
                (state.deadline_ns != 0).then(|| {
                    clock
                        .monotonic_ns
                        .saturating_add(state.setting(clock).remaining_ns)
                })
            };
            let listeners = if self.clock_id == CLOCK_REALTIME {
                &listeners[..]
            } else {
                &listeners[..2]
            };
            if process.is_exited() || process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            if let Some(deadline) = wake_deadline {
                let remaining = deadline.saturating_sub(clock::monotonic_ns());
                let _ = clock::wait_listeners_timeout(
                    listeners,
                    Duration::from_nanos(remaining.max(1)),
                );
            } else {
                EventListener::wait_any(listeners);
            }
        }
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        let mut state = self.state.lock();
        state.refresh(clock::clock_snapshot());
        if state.expirations != 0 || state.canceled {
            requested & (PollEvents::IN | PollEvents::RDNORM)
        } else {
            PollEvents::empty()
        }
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        _: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        output.push((&self.changed).into());
        if self.state.lock().cancel_generation.is_some() {
            output.push(clock::realtime_changed().into());
        }
        // Expirations are refreshed lazily; poll/epoll retain their timed scan
        // while notifications cover rearming, consumption and clock changes.
        false
    }

    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }

    pub(crate) fn set_nonblocking(&self, nonblocking: bool) {
        self.nonblocking.store(nonblocking, Ordering::Release);
        self.changed.wake_all();
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
                previous.remaining_ns,
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
            current.remaining_ns,
        )?;
        Ok(0)
    }
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
    // A valid time_t can exceed the clock's nanosecond range. Keep a distant
    // timer armed at the latest representable instant, including systemd's
    // TIME_T_MAX clock-change watcher. Invalid timespecs still fail above.
    Ok((seconds as u64)
        .saturating_mul(NANOSECONDS_PER_SECOND)
        .saturating_add(nanoseconds as u64))
}

fn encode_timespec(bytes: &mut [u8], nanoseconds: u64) {
    let seconds = (nanoseconds / NANOSECONDS_PER_SECOND) as i64;
    let remainder = (nanoseconds % NANOSECONDS_PER_SECOND) as i64;
    bytes[0..8].copy_from_slice(&seconds.to_ne_bytes());
    bytes[8..16].copy_from_slice(&remainder.to_ne_bytes());
}
