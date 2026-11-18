//! Stream rendezvous, retained connection identity, and byte queue consumption.

use super::*;

pub(super) struct Connection {
    pub local: Address,
    pub remote: Address,
    pub pid: usize,
    pub eof: bool,
    pub closed: bool,
}
impl Connection {
    pub fn unnamed(pid: usize) -> Self {
        Self {
            local: Address::Unnamed,
            remote: Address::Unnamed,
            pid,
            eof: false,
            closed: false,
        }
    }
}
pub(super) enum StreamState {
    Idle,
    Connecting,
    Listening {
        backlog: usize,
        pending: VecDeque<Arc<UnixSocket>>,
        pid: usize,
    },
    Connected(Connection),
}

// Roll back an unsuccessful attempt without holding the naming lock while
// sleeping. Last-descriptor close must remain able to wake blocked connects.
struct Attempt<'a>(&'a UnixSocket);
impl Drop for Attempt<'_> {
    fn drop(&mut self) {
        let mut state = self.0.state.lock();
        if matches!(state.stream, StreamState::Connecting) {
            state.stream = StreamState::Idle;
            drop(state);
            self.0.changed.wake_all();
        }
    }
}

impl UnixSocket {
    pub(crate) fn listen(&self, backlog: usize) -> Result<()> {
        if !self.is_stream() {
            return Err(Errno::NotSupported);
        }
        let process = super::super::current().ok_or(Errno::Invalid)?;
        let _naming = self.naming.lock();
        let mut state = self.state.lock();
        if state.closed {
            return Err(Errno::BadFileDescriptor);
        }
        if state.binding.is_none() {
            return Err(Errno::Invalid);
        }
        let backlog = backlog.clamp(1, 128);
        match &mut state.stream {
            StreamState::Idle => {
                state.stream = StreamState::Listening {
                    backlog,
                    pending: VecDeque::new(),
                    pid: process.pid(),
                }
            }
            StreamState::Listening {
                backlog: limit,
                pid,
                ..
            } => {
                *limit = backlog;
                *pid = process.pid();
            }
            _ => return Err(Errno::Invalid),
        }
        drop(state);
        self.changed.wake_all();
        Ok(())
    }

    pub(super) fn connect_stream(self: &Arc<Self>, address: &Address) -> Result<()> {
        let process = super::super::current().ok_or(Errno::Invalid)?;
        let listener;
        let local;
        {
            let _naming = self.naming.lock();
            let mut state = self.state.lock();
            if state.closed {
                return Err(Errno::BadFileDescriptor);
            }
            match state.stream {
                StreamState::Idle => {}
                StreamState::Connecting => return Err(Errno::AlreadyInProgress),
                StreamState::Connected(_) => return Err(Errno::AlreadyConnected),
                StreamState::Listening { .. } => return Err(Errno::Invalid),
            }
            listener = Self::lookup(address)?;
            if listener.kind != self.kind {
                return Err(Errno::ProtocolType);
            }
            local = state
                .binding
                .as_ref()
                .map(|b| b.address.clone())
                .unwrap_or(Address::Unnamed);
            state.stream = StreamState::Connecting;
        }
        let _attempt = Attempt(self);
        loop {
            let listeners = [
                listener.changed.listen(),
                self.changed.listen(),
                process.interrupt_event().listen(),
            ];
            {
                let mut server = listener.state.lock();
                if server.closed {
                    return Err(Errno::ConnectionRefused);
                }
                let remote = server
                    .binding
                    .as_ref()
                    .map(|b| b.address.clone())
                    .unwrap_or(Address::Unnamed);
                let (backlog, count, pid) = match &server.stream {
                    StreamState::Listening {
                        backlog,
                        pending,
                        pid,
                    } => (*backlog, pending.len(), *pid),
                    _ => return Err(Errno::ConnectionRefused),
                };
                // A self-connect cannot reach this point: this endpoint is in
                // Connecting state, not Listening. Other endpoint locks are
                // nested only for this initial, unpublished connection setup.
                let mut client = self.state.lock();
                if client.closed {
                    return Err(Errno::BadFileDescriptor);
                }
                if count < backlog {
                    let accepted = Self::new(self.kind, false);
                    {
                        let mut child = accepted.state.lock();
                        child.receive_limit = server.receive_limit;
                        child.send_limit = server.send_limit;
                        child.passcred = server.passcred;
                        child.peer = Some(Arc::downgrade(self));
                        child.stream = StreamState::Connected(Connection {
                            local: remote.clone(),
                            remote: local.clone(),
                            pid: process.pid(),
                            eof: false,
                            closed: false,
                        });
                    }
                    client.peer = Some(Arc::downgrade(&accepted));
                    client.stream = StreamState::Connected(Connection {
                        local,
                        remote,
                        pid,
                        eof: false,
                        closed: false,
                    });
                    if let StreamState::Listening { pending, .. } = &mut server.stream {
                        pending.push_back(accepted);
                    }
                    drop(client);
                    drop(server);
                    listener.changed.wake_all();
                    self.changed.wake_all();
                    return Ok(());
                }
            }
            if self.is_nonblocking() {
                return Err(Errno::TryAgain);
            }
            if process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            EventListener::wait_any(&listeners);
        }
    }

    pub(crate) fn accept<T>(
        &self,
        nonblocking: bool,
        deliver: impl FnOnce(&Arc<Self>) -> Result<T>,
    ) -> Result<T> {
        if !self.is_stream() {
            return Err(Errno::NotSupported);
        }
        let process = super::super::current().ok_or(Errno::Invalid)?;
        loop {
            let listeners = [self.changed.listen(), process.interrupt_event().listen()];
            {
                let mut state = self.state.lock();
                if state.closed {
                    return Err(Errno::BadFileDescriptor);
                }
                let StreamState::Listening { pending, .. } = &mut state.stream else {
                    return Err(Errno::Invalid);
                };
                if let Some(child) = pending.front() {
                    let result = deliver(child)?;
                    pending.pop_front();
                    drop(state);
                    self.changed.wake_all();
                    return Ok(result);
                }
            }
            if nonblocking || self.is_nonblocking() {
                return Err(Errno::TryAgain);
            }
            if process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            EventListener::wait_any(&listeners);
        }
    }

    pub(crate) fn peer_pid(&self) -> Result<usize> {
        match &self.state.lock().stream {
            StreamState::Connected(connection) if self.is_stream() => Ok(connection.pid),
            _ => Err(Errno::NotConnected),
        }
    }

    pub(super) fn stream_eof(&self, closed: bool) {
        let mut state = self.state.lock();
        if let StreamState::Connected(connection) = &mut state.stream {
            connection.eof = true;
            connection.closed |= closed;
        }
        drop(state);
        self.changed.wake_all();
        SPACE.wake_all();
    }

    pub(super) fn receive_stream(
        &self,
        flags: u64,
        capacity: usize,
        mut deliver: impl FnMut(&[u8], &Address, usize, bool, &[DescriptorEntry]) -> Result<usize>,
    ) -> Result<usize> {
        if capacity == 0 {
            return Ok(0);
        }
        let process = super::super::current().ok_or(Errno::Invalid)?;
        loop {
            let listeners = [self.changed.listen(), process.interrupt_event().listen()];
            {
                let mut state = self.state.lock();
                if state.closed {
                    return Err(Errno::BadFileDescriptor);
                }
                let StreamState::Connected(connection) = &state.stream else {
                    return Err(Errno::NotConnected);
                };
                if let Some(first) = state.queue.front() {
                    let pid = first.pid;
                    let limit = capacity.min(MAX_MESSAGE);
                    let first_data = &first.data[first.offset..];
                    let mut joined = Vec::new();
                    let data = if first_data.len() >= limit
                        || state.queue.len() == 1
                        || !first.rights.is_empty()
                    {
                        &first_data[..first_data.len().min(limit)]
                    } else {
                        for packet in &state.queue {
                            if !packet.rights.is_empty() || (state.passcred && packet.pid != pid) {
                                break;
                            }
                            let part = &packet.data[packet.offset..];
                            let n = part.len().min(limit - joined.len());
                            joined.extend_from_slice(&part[..n]);
                            if joined.len() == limit {
                                break;
                            }
                        }
                        &joined
                    };
                    let count =
                        deliver(data, &connection.remote, pid, state.passcred, &first.rights)?;
                    if flags & PEEK == 0 {
                        let mut remaining = count;
                        let mut released = 0;
                        let mut retired = Vec::new();
                        while remaining != 0 {
                            let first = state.queue.front_mut().unwrap();
                            let consumed = remaining.min(first.data.len() - first.offset);
                            if consumed != 0 {
                                released += first.rights.len() * RIGHT_CHARGE;
                                retired.append(&mut first.rights);
                            }
                            first.offset += consumed;
                            remaining -= consumed;
                            if first.offset == first.data.len() {
                                released += state.queue.pop_front().unwrap().weight();
                            }
                        }
                        state.bytes -= released;
                        QUEUED.fetch_sub(released, Ordering::AcqRel);
                        drop(state);
                        drop(retired);
                        self.changed.wake_all();
                        if released != 0 {
                            SPACE.wake_all();
                        }
                    }
                    return Ok(count);
                }
                if state.read_shutdown || connection.eof {
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

    pub(super) fn poll_stream(&self, requested: PollEvents) -> PollEvents {
        let (mut ready, peer, writable) = {
            let state = self.state.lock();
            if state.closed {
                return PollEvents::ERR | PollEvents::HUP;
            }
            let mut ready = PollEvents::empty();
            match &state.stream {
                StreamState::Listening { pending, .. } => {
                    return if pending.is_empty() {
                        ready
                    } else {
                        requested & (PollEvents::IN | PollEvents::RDNORM)
                    };
                }
                StreamState::Idle => {
                    return PollEvents::HUP | (requested & (PollEvents::OUT | PollEvents::WRNORM));
                }
                StreamState::Connecting => return ready,
                StreamState::Connected(connection) => {
                    if !state.queue.is_empty() || state.read_shutdown || connection.eof {
                        ready |= requested & (PollEvents::IN | PollEvents::RDNORM);
                    }
                    if state.read_shutdown || connection.eof {
                        ready |= requested & PollEvents::RDHUP;
                    }
                    if connection.closed {
                        ready |= PollEvents::HUP;
                    }
                }
            }
            (ready, state.peer.clone(), !state.write_shutdown)
        };
        let (peer_ready, failed) =
            peer.and_then(|peer| peer.upgrade())
                .map_or((true, true), |peer| {
                    let state = peer.state.lock();
                    let failed = state.closed || state.read_shutdown;
                    (
                        failed || state.bytes + PACKET_OVERHEAD < state.receive_limit,
                        failed,
                    )
                });
        // A failed write is immediately ready too, even when unrelated sockets
        // have exhausted the global queue budget.
        if !writable
            || failed
            || (peer_ready && QUEUED.load(Ordering::Acquire) + PACKET_OVERHEAD < GLOBAL_LIMIT)
        {
            ready |= requested & (PollEvents::OUT | PollEvents::WRNORM);
        }
        ready
    }
}
