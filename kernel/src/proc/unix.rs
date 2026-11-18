//! Local stream and datagram endpoints and their filesystem/abstract rendezvous names.
//!
//! Queues have per-endpoint and global bounds. Names retain inode identity,
//! not a pathname cache. Descriptor ownership closes endpoints explicitly;
//! references held by pollers cannot keep a closed socket alive.

mod rights;
mod stream;
mod syscall;

use super::DescriptorEntry;
use rights::RIGHT_CHARGE;
use stream::{Connection, StreamState};

use crate::{
    fs::{PathAnchor, PollEvents, Vnode, VnodeKey, VnodeKind},
    mem::{IoSink, IoSource},
    sys::{
        event::{Event, EventListener, EventSource},
        sync::{Mutex, Once},
    },
    syscall::{Errno, Result, map_fs_error},
};
use alloc::{
    collections::{BTreeMap, VecDeque},
    format,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

pub(crate) const DONTWAIT: u64 = 1;
pub(crate) const PEEK: u64 = 2;
pub(crate) const TRUNC: u64 = 4;
pub(crate) const MAX_MESSAGE: usize = 64 * 1024;
const MAX_QUEUE: usize = 256 * 1024;
const GLOBAL_LIMIT: usize = 16 * 1024 * 1024;
const PACKET_OVERHEAD: usize = 256;
static QUEUED: AtomicUsize = AtomicUsize::new(0);
static SPACE: Event = Event::new();
static NEXT_ID: AtomicUsize = AtomicUsize::new(1);
static NAMES: Once<Mutex<BTreeMap<Name, Weak<UnixSocket>>>> = Once::new();

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    Stream = 1,
    Datagram = 2,
    UeventRaw = 3,
    UeventDatagram = 4,
    SeqPacket = 5,
}
impl TryFrom<u64> for Kind {
    type Error = Errno;
    fn try_from(value: u64) -> Result<Self> {
        match value {
            1 => Ok(Self::Stream),
            2 => Ok(Self::Datagram),
            3 => Ok(Self::UeventRaw),
            4 => Ok(Self::UeventDatagram),
            5 => Ok(Self::SeqPacket),
            _ => Err(Errno::ProtocolNotSupported),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Address {
    Unnamed,
    Path(Vec<u8>),
    Abstract(Vec<u8>),
    Netlink { port: u32, groups: u32 },
}

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Name {
    Netlink(u32),
    Inode(VnodeKey),
    Abstract(Vec<u8>),
}

struct Binding {
    address: Address,
    name: Name,
    // Keep the bound filesystem alive even if the inode is unlinked.
    _location: Option<(Vnode, PathAnchor)>,
}

struct Packet {
    data: Vec<u8>,
    offset: usize,
    address: Address,
    pid: usize,
    rights: Vec<DescriptorEntry>,
}
impl Packet {
    fn weight(&self) -> usize {
        self.data.len() + PACKET_OVERHEAD + self.rights.len() * RIGHT_CHARGE
    }
}

struct State {
    stream: StreamState,
    binding: Option<Binding>,
    peer: Option<Weak<UnixSocket>>,
    queue: VecDeque<Packet>,
    bytes: usize,
    receive_limit: usize,
    send_limit: usize,
    passcred: bool,
    overflow: bool,
    read_shutdown: bool,
    write_shutdown: bool,
    closed: bool,
}

pub(crate) struct UnixSocket {
    pub(crate) id: usize,
    kind: Kind,
    state: Mutex<State>,
    // Serializes bind/connect/close without nesting endpoint state locks.
    naming: Mutex<()>,
    changed: Arc<Event>,
    nonblocking: AtomicBool,
}

fn names() -> &'static Mutex<BTreeMap<Name, Weak<UnixSocket>>> {
    NAMES.call_once(|| Mutex::new(BTreeMap::new()))
}

impl UnixSocket {
    pub(crate) fn new(kind: Kind, nonblocking: bool) -> Arc<Self> {
        Arc::new(Self {
            id: NEXT_ID.fetch_add(1, Ordering::Relaxed),
            kind,
            state: Mutex::new(State {
                stream: StreamState::Idle,
                binding: None,
                peer: None,
                queue: VecDeque::new(),
                bytes: 0,
                receive_limit: MAX_QUEUE,
                send_limit: MAX_MESSAGE,
                passcred: false,
                overflow: false,
                read_shutdown: false,
                write_shutdown: false,
                closed: false,
            }),
            naming: Mutex::new(()),
            changed: Arc::new(Event::new()),
            nonblocking: AtomicBool::new(nonblocking),
        })
    }

    pub(crate) fn pair(kind: Kind, nonblocking: bool, pid: usize) -> (Arc<Self>, Arc<Self>) {
        let a = Self::new(kind, nonblocking);
        let b = Self::new(kind, nonblocking);
        a.state.lock().peer = Some(Arc::downgrade(&b));
        b.state.lock().peer = Some(Arc::downgrade(&a));
        if matches!(kind, Kind::Stream | Kind::SeqPacket) {
            a.state.lock().stream = StreamState::Connected(Connection::unnamed(pid));
            b.state.lock().stream = StreamState::Connected(Connection::unnamed(pid));
        }
        (a, b)
    }

    fn is_uevent(&self) -> bool {
        matches!(self.kind, Kind::UeventRaw | Kind::UeventDatagram)
    }

    pub(crate) fn is_stream(&self) -> bool {
        matches!(self.kind, Kind::Stream | Kind::SeqPacket)
    }

    pub(crate) fn bind(self: &Arc<Self>, address: Address) -> Result<()> {
        let _naming = self.naming.lock();
        {
            let state = self.state.lock();
            if state.closed {
                return Err(Errno::BadFileDescriptor);
            }
            if state.binding.is_some()
                || (self.is_stream() && !matches!(state.stream, StreamState::Idle))
            {
                return Err(Errno::Invalid);
            }
        }
        if self.is_uevent() != matches!(address, Address::Netlink { .. }) {
            return Err(Errno::ProtocolType);
        }
        let address = if let Address::Netlink { port, groups } = address {
            if groups & !3 != 0 {
                return Err(Errno::Invalid);
            }
            Address::Netlink {
                port: if port == 0 {
                    u32::try_from(self.id).map_err(|_| Errno::NoSpace)?
                } else {
                    port
                },
                groups,
            }
        } else if address == Address::Unnamed {
            Address::Abstract(format!("roanix-{:x}", self.id).into_bytes())
        } else {
            address
        };
        let mut registry = names().lock();
        let (name, location) = match &address {
            Address::Netlink { port, .. } => (Name::Netlink(*port), None),
            Address::Abstract(bytes) => (Name::Abstract(bytes.clone()), None),
            Address::Path(path) => {
                let process = super::current().ok_or(Errno::Invalid)?;
                let location = crate::fs::vfs::create_socket_at(
                    &process.cwd_anchor(),
                    path,
                    0o777 & !process.umask(),
                )
                .map_err(|e| {
                    if e == crate::fs::Error::AlreadyExists {
                        Errno::AddressInUse
                    } else {
                        map_fs_error(e)
                    }
                })?;
                (Name::Inode(location.0.key()), Some(location))
            }
            Address::Unnamed => unreachable!(),
        };
        if registry.get(&name).and_then(Weak::upgrade).is_some() {
            return Err(Errno::AddressInUse);
        }
        registry.insert(name.clone(), Arc::downgrade(self));
        self.state.lock().binding = Some(Binding {
            address,
            name,
            _location: location,
        });
        Ok(())
    }

    fn lookup(address: &Address) -> Result<Arc<Self>> {
        let name = match address {
            Address::Netlink { port, .. } => Name::Netlink(*port),
            Address::Abstract(bytes) => Name::Abstract(bytes.clone()),
            Address::Path(path) => {
                let process = super::current().ok_or(Errno::Invalid)?;
                let anchor = crate::fs::resolve_at(&process.cwd_anchor(), path, true)
                    .map_err(map_fs_error)?;
                if anchor.vnode().kind() != VnodeKind::Socket {
                    return Err(Errno::ConnectionRefused);
                }
                if anchor.vnode().getattr().map_err(map_fs_error)?.mode & 0o222 == 0 {
                    return Err(Errno::Access);
                }
                Name::Inode(anchor.vnode().key())
            }
            Address::Unnamed => return Err(Errno::DestinationRequired),
        };
        names()
            .lock()
            .get(&name)
            .and_then(Weak::upgrade)
            .ok_or(Errno::ConnectionRefused)
    }

    pub(crate) fn connect(self: &Arc<Self>, address: Option<&Address>) -> Result<()> {
        if self.is_stream() {
            return self.connect_stream(address.ok_or(Errno::NotSupported)?);
        }
        let _naming = self.naming.lock();
        let peer = address.map(Self::lookup).transpose()?;
        if peer.as_ref().is_some_and(|peer| peer.kind != self.kind) {
            return Err(Errno::ProtocolType);
        }
        if let Some(peer) = &peer
            && peer.state.lock().closed
        {
            return Err(Errno::ConnectionRefused);
        }
        let mut state = self.state.lock();
        if state.closed {
            return Err(Errno::BadFileDescriptor);
        }
        state.peer = peer.as_ref().map(Arc::downgrade);
        drop(state);
        self.changed.wake_all();
        Ok(())
    }

    pub(crate) fn address(&self, peer: bool) -> Result<Address> {
        if self.is_stream() {
            let state = self.state.lock();
            if let StreamState::Connected(connection) = &state.stream {
                return Ok(if peer {
                    connection.remote.clone()
                } else {
                    connection.local.clone()
                });
            }
            if peer {
                return Err(Errno::NotConnected);
            }
        }
        if peer {
            let target = self
                .state
                .lock()
                .peer
                .as_ref()
                .and_then(Weak::upgrade)
                .ok_or(Errno::NotConnected)?;
            return target.address(false);
        }
        Ok(self
            .state
            .lock()
            .binding
            .as_ref()
            .map(|b| b.address.clone())
            .unwrap_or(Address::Unnamed))
    }

    fn send(
        &self,
        data: Vec<u8>,
        destination: Option<&Address>,
        nonblock: bool,
        rights: Vec<DescriptorEntry>,
    ) -> Result<usize> {
        let process = super::current().ok_or(Errno::Invalid)?;
        if self.is_uevent() {
            if !rights.is_empty() {
                return Err(Errno::NotSupported);
            }
            return self.send_uevent(data, destination);
        }
        let connected = self.is_stream();
        let stream = self.kind == Kind::Stream;
        let (peer, address, limit) = {
            let state = self.state.lock();
            if state.closed {
                return Err(Errno::BadFileDescriptor);
            }
            if state.write_shutdown {
                return Err(Errno::BrokenPipe);
            }
            if connected && !matches!(state.stream, StreamState::Connected(_)) {
                return Err(Errno::NotConnected);
            }
            (
                state.peer.clone(),
                state
                    .binding
                    .as_ref()
                    .map(|b| b.address.clone())
                    .unwrap_or(Address::Unnamed),
                state.send_limit,
            )
        };
        if stream && data.is_empty() {
            return if rights.is_empty() {
                Ok(0)
            } else {
                Err(Errno::Invalid)
            };
        }
        let overhead = PACKET_OVERHEAD + rights.len() * RIGHT_CHARGE;
        if !stream && data.len() > limit {
            return Err(Errno::MessageSize);
        }
        if connected && destination.is_some() {
            return Err(Errno::AlreadyConnected);
        }
        let target = if let Some(address) = destination {
            Self::lookup(address)?
        } else {
            peer.ok_or(Errno::DestinationRequired)?
                .upgrade()
                .ok_or(if connected {
                    Errno::BrokenPipe
                } else {
                    Errno::ConnectionRefused
                })?
        };
        if target.kind != self.kind {
            return Err(Errno::ProtocolType);
        }
        loop {
            let listeners = [
                target.changed.listen(),
                SPACE.listen(),
                process.interrupt_event().listen(),
            ];
            {
                let state = self.state.lock();
                if state.closed {
                    return Err(Errno::BadFileDescriptor);
                }
                if state.write_shutdown {
                    return Err(Errno::BrokenPipe);
                }
            }
            {
                let mut state = target.state.lock();
                if state.closed || state.read_shutdown {
                    return Err(if connected {
                        Errno::BrokenPipe
                    } else {
                        Errno::ConnectionRefused
                    });
                }
                if let Some(peer) = &state.peer
                    && !peer.upgrade().is_some_and(|p| p.id == self.id)
                {
                    return Err(Errno::Permission);
                }
                if stream && overhead >= state.receive_limit {
                    return Err(Errno::MessageSize);
                }
                let count = if stream {
                    data.len()
                        .min(limit)
                        .min(state.receive_limit.saturating_sub(state.bytes + overhead))
                        .min(GLOBAL_LIMIT.saturating_sub(QUEUED.load(Ordering::Acquire) + overhead))
                } else {
                    data.len()
                };
                let weight = count + overhead;
                if !stream && weight > state.receive_limit {
                    return Err(Errno::MessageSize);
                }
                if (!stream || count != 0)
                    && state.bytes + weight <= state.receive_limit
                    && QUEUED
                        .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                            n.checked_add(weight).filter(|n| *n <= GLOBAL_LIMIT)
                        })
                        .is_ok()
                {
                    let packet = Packet {
                        data: if count == data.len() {
                            data
                        } else {
                            data[..count].to_vec()
                        },
                        offset: 0,
                        address,
                        pid: process.pid(),
                        rights,
                    };
                    state.bytes += weight;
                    state.queue.push_back(packet);
                    drop(state);
                    target.changed.wake_all();
                    return Ok(count);
                }
            }
            if nonblock || self.is_nonblocking() {
                return Err(Errno::TryAgain);
            }
            if process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            EventListener::wait_any(&listeners);
        }
    }

    fn send_uevent(&self, data: Vec<u8>, destination: Option<&Address>) -> Result<usize> {
        if data.len() > MAX_MESSAGE {
            return Err(Errno::MessageSize);
        }
        let (source, peer) = {
            let state = self.state.lock();
            if state.closed {
                return Err(Errno::BadFileDescriptor);
            }
            if state.write_shutdown {
                return Err(Errno::BrokenPipe);
            }
            let source = state
                .binding
                .as_ref()
                .map(|b| &b.address)
                .ok_or(Errno::NotConnected)?;
            (source.clone(), state.peer.clone())
        };
        let destination = destination
            .cloned()
            .or_else(|| {
                peer.and_then(|peer| peer.upgrade()).and_then(|peer| {
                    peer.state
                        .lock()
                        .binding
                        .as_ref()
                        .map(|b| b.address.clone())
                })
            })
            .ok_or(Errno::DestinationRequired)?;
        let Address::Netlink { port, groups } = destination else {
            return Err(Errno::ProtocolType);
        };
        if groups & !2 != 0 {
            return Err(Errno::Permission);
        }
        let Address::Netlink { port: sender, .. } = source else {
            return Err(Errno::Invalid);
        };
        let pid = super::current().ok_or(Errno::Invalid)?.pid();
        deliver_uevent(&data, port, groups, sender, pid)?;
        Ok(data.len())
    }

    fn receive(
        &self,
        flags: u64,
        capacity: usize,
        mut deliver: impl FnMut(&[u8], &Address, usize, bool, &[DescriptorEntry]) -> Result<usize>,
    ) -> Result<usize> {
        if self.kind == Kind::Stream {
            return self.receive_stream(flags, capacity, deliver);
        }
        let process = super::current().ok_or(Errno::Invalid)?;
        loop {
            let listeners = [self.changed.listen(), process.interrupt_event().listen()];
            {
                let mut state = self.state.lock();
                if state.closed {
                    return Err(Errno::BadFileDescriptor);
                }
                if core::mem::take(&mut state.overflow) {
                    return Err(Errno::NoBuffers);
                }
                if let Some(packet) = state.queue.front() {
                    let count = deliver(
                        &packet.data,
                        &packet.address,
                        packet.pid,
                        state.passcred,
                        &packet.rights,
                    )?;
                    if flags & PEEK == 0 {
                        let packet = state.queue.pop_front().unwrap();
                        state.bytes -= packet.weight();
                        QUEUED.fetch_sub(packet.weight(), Ordering::AcqRel);
                        drop(state);
                        self.changed.wake_all();
                        SPACE.wake_all();
                    }
                    return Ok(count);
                }
                if state.read_shutdown
                    || matches!(&state.stream, StreamState::Connected(connection) if connection.eof)
                {
                    return Ok(0);
                }
            }
            if flags & DONTWAIT != 0 || self.is_nonblocking() {
                return Err(Errno::TryAgain);
            }
            if process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            EventListener::wait_any(&listeners);
        }
    }

    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize> {
        if sink.is_empty() {
            return Ok(0);
        }
        self.receive(0, sink.len(), |data, _, _, _, _| {
            let n = data.len().min(sink.len());
            sink.store(0, &data[..n]).map_err(|_| Errno::Fault)?;
            Ok(n)
        })
    }
    pub(crate) fn write(&self, source: &IoSource<'_>) -> Result<usize> {
        if self.kind != Kind::Stream && source.len() > MAX_MESSAGE {
            return Err(Errno::MessageSize);
        }
        let mut data = alloc::vec![0; source.len().min(MAX_MESSAGE)];
        source.load(0, &mut data).map_err(|_| Errno::Fault)?;
        let result = self.send(data, None, false, Vec::new());
        if matches!(result, Err(Errno::BrokenPipe))
            && let Some(process) = super::current()
        {
            super::signal::send_kernel(&process, super::signal::SIGPIPE);
        }
        result
    }
    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }
    pub(crate) fn set_nonblocking(&self, value: bool) {
        self.nonblocking.store(value, Ordering::Release);
        self.changed.wake_all();
        SPACE.wake_all();
    }

    pub(crate) fn option(&self, option: u64, value: Option<i32>) -> Result<i32> {
        let mut state = self.state.lock();
        let result = match option {
            1 => {
                if let Some(value) = value {
                    state.passcred = value != 0;
                }
                Ok(i32::from(state.passcred))
            }
            2 | 3 => {
                let limit = if option == 2 {
                    &mut state.receive_limit
                } else {
                    &mut state.send_limit
                };
                if let Some(value) = value {
                    if value < 0 {
                        return Err(Errno::Invalid);
                    }
                    *limit = (value as usize)
                        .saturating_mul(2)
                        .clamp(4096, if option == 2 { MAX_QUEUE } else { MAX_MESSAGE });
                }
                Ok(*limit as i32)
            }
            4 if value.is_none() => Ok(0), // SO_ERROR: no deferred connection errors.
            5 if value.is_none() => Ok(if self.kind == Kind::UeventDatagram {
                2
            } else {
                self.kind as i32
            }),
            20 | 21 if self.is_uevent() => {
                let group = value.ok_or(Errno::Invalid)?;
                if !(1..=2).contains(&group) {
                    return Err(Errno::Invalid);
                }
                let binding = state.binding.as_mut().ok_or(Errno::Invalid)?;
                let Address::Netlink { groups, .. } = &mut binding.address else {
                    return Err(Errno::Invalid);
                };
                if option == 20 {
                    *groups |= 1 << (group - 1);
                } else {
                    *groups &= !(1 << (group - 1));
                }
                Ok(0)
            }
            6 if value.is_none() => Ok(i32::from(matches!(
                state.stream,
                StreamState::Listening { .. }
            ))),
            _ => Err(Errno::NotSupported),
        };
        drop(state);
        if value.is_some() {
            self.changed.wake_all();
            SPACE.wake_all();
        }
        result
    }

    pub(crate) fn shutdown(&self, how: u64) -> Result<()> {
        if how > 2 {
            return Err(Errno::Invalid);
        }
        let mut state = self.state.lock();
        if self.is_stream() && !matches!(state.stream, StreamState::Connected(_)) {
            return Err(Errno::NotConnected);
        }
        let peer = state.peer.as_ref().and_then(Weak::upgrade);
        state.read_shutdown |= how != 1;
        state.write_shutdown |= how != 0;
        drop(state);
        if self.is_stream()
            && how != 0
            && let Some(peer) = peer
        {
            peer.stream_eof(false);
        }
        self.changed.wake_all();
        SPACE.wake_all();
        Ok(())
    }
    pub(crate) fn close(&self) {
        let _naming = self.naming.lock();
        let (binding, peer, pending) = {
            let mut state = self.state.lock();
            if state.closed {
                return;
            }
            state.closed = true;
            QUEUED.fetch_sub(state.bytes, Ordering::AcqRel);
            state.bytes = 0;
            state.queue.clear();
            let pending = match &mut state.stream {
                StreamState::Listening { pending, .. } => core::mem::take(pending),
                _ => VecDeque::new(),
            };
            (
                state.binding.take(),
                state.peer.as_ref().and_then(Weak::upgrade),
                pending,
            )
        };
        if let Some(binding) = binding {
            names().lock().remove(&binding.name);
        }
        if self.is_stream()
            && let Some(peer) = peer
        {
            peer.stream_eof(true);
        }
        for child in pending {
            child.close();
        }
        self.changed.wake_all();
        SPACE.wake_all();
    }
    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        if self.is_stream() {
            return self.poll_stream(requested);
        }
        let (mut ready, peer, writable) = {
            let state = self.state.lock();
            if state.closed {
                return PollEvents::ERR | PollEvents::HUP;
            }
            let mut ready = PollEvents::empty();
            if state.overflow {
                ready |= PollEvents::ERR;
            }
            if !state.queue.is_empty() || state.read_shutdown || state.overflow {
                ready |= requested & (PollEvents::IN | PollEvents::RDNORM);
            }
            (ready, state.peer.clone(), !state.write_shutdown)
        };
        let peer_ready = peer
            .map(|peer| {
                peer.upgrade().is_none_or(|peer| {
                    let state = peer.state.lock();
                    state.closed
                        || state.read_shutdown
                        || state.bytes + PACKET_OVERHEAD < state.receive_limit
                })
            })
            .unwrap_or(true);
        if writable && peer_ready && QUEUED.load(Ordering::Acquire) + PACKET_OVERHEAD < GLOBAL_LIMIT
        {
            ready |= requested & (PollEvents::OUT | PollEvents::WRNORM);
        }
        ready
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        requested: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        output.push(self.changed.clone().into());
        if requested.intersects(PollEvents::OUT | PollEvents::WRNORM) {
            output.push((&SPACE).into());
        }
        true
    }
}

/// Broadcasts a kernel device event to NETLINK_KOBJECT_UEVENT subscribers.
pub(crate) fn broadcast_uevent(data: &[u8]) {
    let _ = deliver_uevent(data, 0, 1, 0, 0);
}

fn deliver_uevent(data: &[u8], port: u32, groups: u32, sender: u32, pid: usize) -> Result<()> {
    let targets: Vec<_> = names()
        .lock()
        .iter()
        .filter_map(|(name, endpoint)| {
            if matches!(name, Name::Netlink(id) if groups != 0 || *id == port) {
                endpoint.upgrade()
            } else {
                None
            }
        })
        .collect();
    let mut delivered = false;
    for endpoint in targets {
        let mut state = endpoint.state.lock();
        if state.closed || state.read_shutdown {
            continue;
        }
        if groups != 0 && !state.binding.as_ref().is_some_and(|b| matches!(b.address, Address::Netlink { groups: subscribed, .. } if subscribed & groups != 0)) { continue; }
        delivered = true;
        let weight = data.len() + PACKET_OVERHEAD;
        if state.bytes + weight > state.receive_limit
            || QUEUED
                .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                    n.checked_add(weight).filter(|n| *n <= GLOBAL_LIMIT)
                })
                .is_err()
        {
            state.overflow = true;
        } else {
            state.bytes += weight;
            state.queue.push_back(Packet {
                data: data.to_vec(),
                offset: 0,
                address: Address::Netlink {
                    port: sender,
                    groups,
                },
                pid,
                rights: Vec::new(),
            });
        }
        drop(state);
        endpoint.changed.wake_all();
    }
    if groups == 0 && !delivered {
        Err(Errno::ConnectionRefused)
    } else {
        Ok(())
    }
}
