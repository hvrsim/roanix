//! epoll descriptor aggregation.

use alloc::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, Weak},
    vec::Vec,
};
use core::time::Duration;

use crate::{
    fs::PollEvents,
    mem::VirtAddr,
    proc::{Descriptor, DescriptorKey, Process, descriptor_life::DescriptorLife},
    sys::{
        clock,
        event::{Event, EventListener},
        sync::Mutex,
    },
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
const MAX_NESTING: usize = 5;

// Serialize graph changes so concurrent additions cannot form a cycle after
// both callers have checked the old topology. Readiness never takes this lock.
static TOPOLOGY: Mutex<Vec<Weak<Epoll>>> = Mutex::new(Vec::new());

#[derive(Clone, Copy)]
struct UserEpollEvent {
    events: u32,
    data: u64,
}

#[derive(Clone, Copy, Eq, Ord, PartialEq, PartialOrd)]
struct WatchKey {
    fd: i32,
    description: DescriptorKey,
}

struct Watch {
    life: Arc<DescriptorLife>,
    descriptor: Descriptor,
    events: u32,
    data: u64,
    last_ready: u32,
    epochs: Vec<(usize, u64)>,
    disabled: bool,
}

struct EpollState {
    watches: BTreeMap<WatchKey, Watch>,
    cursor: Option<WatchKey>,
}

/// One epoll open-file description.
pub(crate) struct Epoll {
    state: Mutex<EpollState>,
    changed: Event,
}

impl Epoll {
    fn new() -> Arc<Self> {
        let epoll = Arc::new(Self {
            state: Mutex::new(EpollState {
                watches: BTreeMap::new(),
                cursor: None,
            }),
            changed: Event::new(),
        });
        let mut topology = TOPOLOGY.lock();
        topology.retain(|entry| entry.strong_count() != 0);
        topology.push(Arc::downgrade(&epoll));
        epoll
    }

    fn control(
        &self,
        operation: i32,
        fd: i32,
        descriptor: Descriptor,
        life: Arc<DescriptorLife>,
        event: Option<UserEpollEvent>,
    ) -> Result<()> {
        let topology = TOPOLOGY.lock();
        if let Descriptor::Epoll(child) = &descriptor {
            if core::ptr::eq(self, child.as_ref()) {
                return Err(Errno::Invalid);
            }
            if operation == EPOLL_CTL_ADD && child.reaches(self) {
                return Err(Errno::Loop);
            }
            if operation == EPOLL_CTL_ADD {
                self.validate_nesting(child, &topology)?;
            }
        }
        let key = WatchKey {
            fd,
            description: descriptor.key(),
        };
        let mut state = self.state.lock();
        state.watches.retain(|_, watch| watch.life.is_open());
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
                        life,
                        descriptor,
                        events: event.events,
                        data: event.data,
                        last_ready: 0,
                        epochs: Vec::new(),
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
                watch.epochs.clear();
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
        let mut state = self.state.lock();
        state.watches.retain(|_, watch| watch.life.is_open());

        let mut output = Vec::new();
        let keys: Vec<_> = state.watches.keys().copied().collect();
        let start = state
            .cursor
            .map_or(0, |cursor| keys.partition_point(|key| *key <= cursor));
        let mut cursor = state.cursor;
        for key in keys.iter().cycle().skip(start).take(keys.len()) {
            if output.len() == maximum {
                break;
            }
            let watch = state.watches.get_mut(key).expect("snapshot watch");
            if watch.disabled {
                continue;
            }
            let requested = poll_mask(watch.events);
            let epochs = if watch.events & EPOLLET != 0 {
                notification_epochs(&watch.descriptor, requested)
            } else {
                Vec::new()
            };
            if epochs != watch.epochs {
                watch.last_ready = 0;
            }
            let ready = epoll_mask(watch.descriptor.poll(requested))
                & (watch.events | EPOLLERR | EPOLLHUP | EPOLLRDHUP);
            let report = if watch.events & EPOLLET != 0 {
                ready & !watch.last_ready
            } else {
                ready
            };
            watch.last_ready = ready;
            watch.epochs = epochs;
            if report == 0 {
                continue;
            }
            cursor = Some(*key);
            output.push(UserEpollEvent {
                events: report,
                data: watch.data,
            });
            if watch.events & EPOLLONESHOT != 0 {
                watch.disabled = true;
            }
        }
        state.cursor = cursor;
        output
    }

    fn children(&self) -> Vec<Arc<Epoll>> {
        let mut state = self.state.lock();
        // Retire dead graph edges before admitting new ones. Merely ignoring
        // them during cycle detection could leave an Arc cycle in storage.
        state.watches.retain(|_, watch| watch.life.is_open());
        state
            .watches
            .values()
            .filter_map(|watch| match &watch.descriptor {
                Descriptor::Epoll(child) => Some(child.clone()),
                _ => None,
            })
            .collect()
    }

    fn reaches(&self, target: &Self) -> bool {
        let mut pending = self.children();
        let mut visited = BTreeSet::new();
        while let Some(child) = pending.pop() {
            if core::ptr::eq(child.as_ref(), target) {
                return true;
            }
            if visited.insert(Arc::as_ptr(&child) as usize) {
                pending.extend(child.children());
            }
        }
        false
    }

    fn validate_nesting(&self, child: &Arc<Self>, topology: &[Weak<Self>]) -> Result<()> {
        // Check ancestors too: adding a shallow subtree under an existing
        // chain can exceed the limit. The cap also bounds recursive Arc drops.
        let mut pending: Vec<_> = topology
            .iter()
            .filter_map(Weak::upgrade)
            .map(|entry| (entry, 1usize))
            .collect();
        let mut depths = BTreeMap::new();
        while let Some((entry, depth)) = pending.pop() {
            if depth > MAX_NESTING {
                return Err(Errno::Loop);
            }
            let pointer = Arc::as_ptr(&entry) as usize;
            if depths
                .get(&pointer)
                .is_some_and(|previous| *previous >= depth)
            {
                continue;
            }
            depths.insert(pointer, depth);
            pending.extend(entry.children().into_iter().map(|child| (child, depth + 1)));
            if core::ptr::eq(entry.as_ref(), self) {
                pending.push((child.clone(), depth + 1));
            }
        }
        Ok(())
    }

    fn ready_sources(&self, children: &mut Vec<Arc<Epoll>>) -> bool {
        self.state.lock().watches.values_mut().any(|watch| {
            if watch.disabled || !watch.life.is_open() {
                return false;
            }
            if watch.events & EPOLLET != 0
                && notification_epochs(&watch.descriptor, poll_mask(watch.events)) != watch.epochs
            {
                watch.last_ready = 0;
            }
            if let Descriptor::Epoll(child) = &watch.descriptor {
                if watch.events & u32::from(PollEvents::IN.bits()) != 0
                    && (watch.events & EPOLLET == 0
                        || watch.last_ready & u32::from(PollEvents::IN.bits()) == 0)
                {
                    children.push(child.clone());
                }
                return false;
            }
            let ready = epoll_mask(watch.descriptor.poll(poll_mask(watch.events)))
                & (watch.events | EPOLLERR | EPOLLHUP | EPOLLRDHUP);
            let report = if watch.events & EPOLLET != 0 {
                ready & !watch.last_ready != 0
            } else {
                ready != 0
            };
            watch.last_ready &= ready;
            report
        })
    }

    fn has_ready(&self) -> bool {
        // Traverse iteratively so userspace cannot exhaust the kernel stack by
        // building a long chain. Never retain a child lock between iterations.
        let mut pending = Vec::new();
        if self.ready_sources(&mut pending) {
            return true;
        }
        let mut visited = BTreeSet::new();
        while let Some(child) = pending.pop() {
            if visited.insert(Arc::as_ptr(&child) as usize) && child.ready_sources(&mut pending) {
                return true;
            }
        }
        false
    }

    fn wait_descriptors(&self) -> Vec<(Descriptor, PollEvents, Arc<DescriptorLife>)> {
        let mut state = self.state.lock();
        state.watches.retain(|_, watch| watch.life.is_open());
        self.changed.reset();
        state
            .watches
            .values()
            .filter(|watch| !watch.disabled)
            .map(|watch| {
                (
                    watch.descriptor.clone(),
                    poll_mask(watch.events) | PollEvents::ERR | PollEvents::HUP,
                    watch.life.clone(),
                )
            })
            .collect()
    }

    fn notification_sources(&self) -> Vec<(Descriptor, PollEvents)> {
        self.state
            .lock()
            .watches
            .iter()
            .filter(|(_, watch)| !watch.disabled && watch.life.is_open())
            .map(|(_, watch)| (watch.descriptor.clone(), poll_mask(watch.events)))
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

// Readiness can become false and true entirely between two epoll calls.
// Preserve endpoint notification epochs so an edge is not lost in that gap,
// including when a parent only polls its child's epoll descriptor.
fn notification_epochs(descriptor: &Descriptor, requested: PollEvents) -> Vec<(usize, u64)> {
    let mut pending = alloc::vec![(descriptor.clone(), requested)];
    let mut visited = BTreeSet::new();
    let mut epochs = Vec::new();
    while let Some((descriptor, requested)) = pending.pop() {
        if let Descriptor::Epoll(child) = &descriptor {
            if visited.insert(Arc::as_ptr(child) as usize) {
                epochs.push((
                    &child.changed as *const Event as usize,
                    child.changed.generation(),
                ));
                pending.extend(child.notification_sources());
            }
        } else {
            let mut events = Vec::new();
            descriptor.poll_events(requested, &mut events);
            epochs.extend(
                events
                    .into_iter()
                    .map(|event| (&*event as *const Event as usize, event.generation())),
            );
        }
    }
    epochs.sort_unstable();
    epochs.dedup();
    epochs
}

crate::syscall_handler! {
    syscall_epoll_create(_frame, flags: u64 = 0) {
        if flags & !(EPOLL_CLOEXEC | EPOLL_NONBLOCK) != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let epoll = Epoll::new();
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
        let (descriptor, life) = process.watch_target(fd).ok_or(Errno::BadFileDescriptor)?;
        if matches!(&descriptor, Descriptor::File(file) if file.is_path()) {
            return Err(Errno::BadFileDescriptor);
        }
        let event = if operation == EPOLL_CTL_DEL {
            None
        } else {
            Some(read_user_event(&process, event)?)
        };
        epoll.control(operation, fd, descriptor, life, event)?;
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
            let descriptors = epoll.wait_descriptors();
            let mut wait_events = alloc::vec![
                crate::sys::event::EventSource::from(&epoll.changed),
                process.interrupt_event().into(),
            ];
            let mut event_driven = true;
            for (descriptor, requested, life) in &descriptors {
                wait_events.push((&life.closed).into());
                event_driven &= descriptor.poll_events(*requested, &mut wait_events);
            }
            wait_events.sort_unstable_by_key(|event| &**event as *const Event as usize);
            wait_events.dedup_by_key(|event| &**event as *const Event as usize);

            let listeners = wait_events.iter().map(|event| event.listen()).collect::<Vec<_>>();
            let ready = epoll.collect_ready(maximum);
            if !ready.is_empty() {
                write_user_events(&process, events, &ready)?;
                return Ok(ready.len() as u64);
            }
            let now = clock::monotonic_ns();
            if timeout_ms == 0 || deadline.is_some_and(|value| now >= value) {
                return Ok(0);
            }

            if process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            if event_driven {
                if let Some(deadline) = deadline {
                    let _ = clock::wait_listeners_timeout(
                        &listeners,
                        Duration::from_nanos(deadline.saturating_sub(now).max(1)),
                    );
                } else {
                    EventListener::wait_any(&listeners);
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
