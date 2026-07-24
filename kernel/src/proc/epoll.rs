//! epoll descriptor aggregation.

use alloc::{
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::time::Duration;

use crate::{
    fs::PollEvents,
    mem::VirtAddr,
    proc::{Descriptor, DescriptorKey, Process},
    sys::{clock, event::Event, sync::Mutex},
    syscall::{Errno, Result, current_process, map_memory_error, map_process_error},
};

const EPOLL_CLOEXEC: u64 = 0o2000000;
const EPOLL_NONBLOCK: u64 = 0o4000;
const EPOLL_CTL_ADD: i32 = 1;
const EPOLL_CTL_DEL: i32 = 2;
const EPOLL_CTL_MOD: i32 = 3;

const EPOLLERR: u32 = 0x008;
const EPOLLHUP: u32 = 0x010;
const EPOLLRDHUP: u32 = 0x2000;
const EPOLLEXCLUSIVE: u32 = 1 << 28;
const EPOLLWAKEUP: u32 = 1 << 29;
const EPOLLONESHOT: u32 = 1 << 30;
const EPOLLET: u32 = 1 << 31;
const EPOLL_BEHAVIOR_FLAGS: u32 = EPOLLONESHOT | EPOLLET;
const EPOLL_UNSUPPORTED_FLAGS: u32 = EPOLLEXCLUSIVE | EPOLLWAKEUP;
const EPOLL_EVENT_MASK: u32 = 0x07ff | EPOLLRDHUP;
const EPOLL_WAIT_MAX: usize = 4096;
const WAIT_INTERVAL_NS: u64 = 1_000_000;

#[derive(Clone, Copy)]
struct UserEpollEvent {
    events: u32,
    data: u64,
}

struct Watch {
    descriptor: Descriptor,
    events: u32,
    data: u64,
    last_ready: u32,
    disabled: bool,
}

struct EpollState {
    watches: BTreeMap<DescriptorKey, Watch>,
}

/// One epoll open-file description.
pub(crate) struct Epoll {
    process: Weak<Process>,
    state: Mutex<EpollState>,
    changed: Event,
}

impl Epoll {
    fn new(process: &Arc<Process>) -> Arc<Self> {
        Arc::new(Self {
            process: Arc::downgrade(process),
            state: Mutex::new(EpollState {
                watches: BTreeMap::new(),
            }),
            changed: Event::new(),
        })
    }

    fn control(
        &self,
        operation: i32,
        descriptor: Descriptor,
        event: Option<UserEpollEvent>,
    ) -> Result<()> {
        if matches!(descriptor, Descriptor::Epoll(_)) {
            return Err(Errno::Invalid);
        }
        let key = descriptor.key();
        let mut state = self.state.lock();
        let result = match operation {
            EPOLL_CTL_ADD => {
                let event = event.ok_or(Errno::Fault)?;
                validate_events(event.events)?;
                if state.watches.contains_key(&key) {
                    return Err(Errno::Exists);
                }
                state.watches.insert(
                    key,
                    Watch {
                        descriptor,
                        events: event.events,
                        data: event.data,
                        last_ready: 0,
                        disabled: false,
                    },
                );
                Ok(())
            }
            EPOLL_CTL_DEL => state.watches.remove(&key).map(|_| ()).ok_or(Errno::NoEntry),
            EPOLL_CTL_MOD => {
                let event = event.ok_or(Errno::Fault)?;
                validate_events(event.events)?;
                let watch = state.watches.get_mut(&key).ok_or(Errno::NoEntry)?;
                watch.events = event.events;
                watch.data = event.data;
                watch.last_ready = 0;
                watch.disabled = false;
                Ok(())
            }
            _ => Err(Errno::Invalid),
        };
        if result.is_ok() {
            self.changed.signal();
        }
        result
    }

    fn collect_ready(&self, maximum: usize) -> Vec<UserEpollEvent> {
        let Some(process) = self.process.upgrade() else {
            return Vec::new();
        };
        let mut state = self.state.lock();
        state
            .watches
            .retain(|key, _| process.contains_descriptor(*key));

        let mut output = Vec::new();
        for watch in state.watches.values_mut() {
            if output.len() == maximum || watch.disabled {
                continue;
            }
            let requested = poll_mask(watch.events);
            let ready = epoll_mask(watch.descriptor.poll(requested))
                & (watch.events | EPOLLERR | EPOLLHUP | EPOLLRDHUP);
            let report = if watch.events & EPOLLET != 0 {
                ready & !watch.last_ready
            } else {
                ready
            };
            watch.last_ready = ready;
            if report == 0 {
                continue;
            }
            output.push(UserEpollEvent {
                events: report,
                data: watch.data,
            });
            if watch.events & EPOLLONESHOT != 0 {
                watch.disabled = true;
            }
        }
        output
    }

    fn has_ready(&self) -> bool {
        let Some(process) = self.process.upgrade() else {
            return false;
        };
        self.state.lock().watches.iter().any(|(key, watch)| {
            if watch.disabled || !process.contains_descriptor(*key) {
                return false;
            }
            let ready = epoll_mask(watch.descriptor.poll(poll_mask(watch.events)))
                & (watch.events | EPOLLERR | EPOLLHUP | EPOLLRDHUP);
            if watch.events & EPOLLET != 0 {
                ready & !watch.last_ready != 0
            } else {
                ready != 0
            }
        })
    }

    fn wait_descriptors(&self) -> Vec<(Descriptor, PollEvents)> {
        let Some(process) = self.process.upgrade() else {
            return Vec::new();
        };
        let mut state = self.state.lock();
        state
            .watches
            .retain(|key, _| process.contains_descriptor(*key));
        self.changed.reset();
        state
            .watches
            .values()
            .filter(|watch| !watch.disabled)
            .map(|watch| {
                (
                    watch.descriptor.clone(),
                    poll_mask(watch.events) | PollEvents::ERR | PollEvents::HUP,
                )
            })
            .collect()
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        if requested.contains(PollEvents::IN) && self.has_ready() {
            PollEvents::IN
        } else {
            PollEvents::empty()
        }
    }
}

crate::syscall_handler! {
    syscall_epoll_create(_frame, flags: u64 = 0) {
        if flags & !(EPOLL_CLOEXEC | EPOLL_NONBLOCK) != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let epoll = Epoll::new(&process);
        let fd = process
            .install_descriptor_value(
                Descriptor::Epoll(epoll),
                flags & EPOLL_CLOEXEC != 0,
            )
            .map_err(map_process_error)?;
        Ok(fd as u64)
    }
}

crate::syscall_handler! {
    syscall_epoll_ctl(
        _frame,
        epoll_fd: i32 = 0,
        operation: i32 = 1,
        fd: i32 = 2,
        event: u64 = 3,
    ) {
        let process = current_process()?;
        let Descriptor::Epoll(epoll) =
            process.descriptor(epoll_fd).ok_or(Errno::BadFileDescriptor)?
        else {
            return Err(Errno::Invalid);
        };
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let event = if operation == EPOLL_CTL_DEL {
            None
        } else {
            Some(read_user_event(&process, event)?)
        };
        epoll.control(operation, descriptor, event)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_epoll_wait(
        _frame,
        epoll_fd: i32 = 0,
        events: u64 = 1,
        maximum: i32 = 2,
        timeout_ms: i32 = 3,
    ) {
        let maximum = usize::try_from(maximum).map_err(|_| Errno::Invalid)?;
        if maximum == 0 || maximum > EPOLL_WAIT_MAX || timeout_ms < -1 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let Descriptor::Epoll(epoll) =
            process.descriptor(epoll_fd).ok_or(Errno::BadFileDescriptor)?
        else {
            return Err(Errno::Invalid);
        };
        let deadline = (timeout_ms >= 0).then(|| {
            clock::monotonic_ns().saturating_add((timeout_ms as u64).saturating_mul(1_000_000))
        });

        loop {
            let ready = epoll.collect_ready(maximum);
            if !ready.is_empty() {
                write_user_events(&process, events, &ready)?;
                return Ok(ready.len() as u64);
            }
            let now = clock::monotonic_ns();
            if timeout_ms == 0 || deadline.is_some_and(|value| now >= value) {
                return Ok(0);
            }

            let descriptors = epoll.wait_descriptors();
            let mut wait_events = alloc::vec![&epoll.changed];
            let mut event_driven = true;
            for (descriptor, requested) in &descriptors {
                event_driven &= descriptor.poll_events(*requested, &mut wait_events);
            }
            wait_events.sort_unstable_by_key(|event| *event as *const Event as usize);
            wait_events.dedup_by_key(|event| *event as *const Event as usize);

            if event_driven {
                if let Some(deadline) = deadline {
                    let _ = clock::wait_any_timeout(
                        &wait_events,
                        Duration::from_nanos(deadline.saturating_sub(now).max(1)),
                    );
                } else {
                    Event::wait_any(&wait_events);
                }
            } else {
                let sleep_ns = deadline
                    .map(|value| value.saturating_sub(now).min(WAIT_INTERVAL_NS))
                    .unwrap_or(WAIT_INTERVAL_NS);
                clock::sleep(Duration::from_nanos(sleep_ns));
            }
        }
    }
}

fn validate_events(events: u32) -> Result<()> {
    if events & EPOLL_UNSUPPORTED_FLAGS != 0
        || events & !(EPOLL_EVENT_MASK | EPOLL_BEHAVIOR_FLAGS) != 0
    {
        return Err(Errno::Invalid);
    }
    Ok(())
}

fn poll_mask(events: u32) -> PollEvents {
    PollEvents::from_bits_retain(events as u16) | PollEvents::ERR | PollEvents::HUP
}

fn epoll_mask(events: PollEvents) -> u32 {
    u32::from(events.bits())
}

fn read_user_event(process: &Process, address: u64) -> Result<UserEpollEvent> {
    if address == 0 {
        return Err(Errno::Fault);
    }
    let mut bytes = [0u8; user_event_size()];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(UserEpollEvent {
        events: u32::from_ne_bytes(bytes[0..4].try_into().expect("epoll events width")),
        data: u64::from_ne_bytes(
            bytes[user_data_offset()..user_data_offset() + 8]
                .try_into()
                .expect("epoll data width"),
        ),
    })
}

fn write_user_events(process: &Process, address: u64, events: &[UserEpollEvent]) -> Result<()> {
    let event_size = user_event_size();
    let mut bytes = alloc::vec![0u8; events.len() * event_size];
    for (event, record) in events.iter().zip(bytes.chunks_exact_mut(event_size)) {
        record[0..4].copy_from_slice(&event.events.to_ne_bytes());
        record[user_data_offset()..user_data_offset() + 8]
            .copy_from_slice(&event.data.to_ne_bytes());
    }
    process
        .address_space()
        .write_user(VirtAddr::new(address), &bytes)
        .map_err(map_memory_error)
}

#[cfg(target_arch = "x86_64")]
const fn user_event_size() -> usize {
    12
}

#[cfg(target_arch = "riscv64")]
const fn user_event_size() -> usize {
    16
}

#[cfg(target_arch = "x86_64")]
const fn user_data_offset() -> usize {
    4
}

#[cfg(target_arch = "riscv64")]
const fn user_data_offset() -> usize {
    8
}
