//! Kernel-owned `/dev/ptmx` and `/dev/pts/*` pseudo terminals.

use alloc::{collections::VecDeque, string::ToString, sync::Arc, vec::Vec};

use crate::{
    fs::{
        Error, IoctlContext, OpenFlags, PollEvents, Result,
        devfs::{DeviceNodeOps, read_windows, write_windows},
    },
    mem::{IoSink, IoSource},
    sys::{event::Event, smp::IrqSpinLock},
};

use super::{
    node,
    tty::{self, ConsoleBackend, SerialSettings, Tty},
};

const SLOT_COUNT: usize = 64;
const RING_CAPACITY: usize = 64 * 1024;

struct SlotState {
    reserved: bool,
    master_open: bool,
    slave_seen: bool,
    slave_opens: u32,
    locked: bool,
    to_master: VecDeque<u8>,
    to_slave: VecDeque<u8>,
}

impl SlotState {
    const fn new() -> Self {
        Self {
            reserved: false,
            master_open: false,
            slave_seen: false,
            slave_opens: 0,
            locked: true,
            to_master: VecDeque::new(),
            to_slave: VecDeque::new(),
        }
    }

    fn active(&self) -> bool {
        self.reserved && self.master_open
    }

    fn cleanup(&mut self) {
        if self.reserved && !self.master_open && self.slave_opens == 0 {
            self.reserved = false;
            self.locked = true;
            self.to_master.clear();
            self.to_slave.clear();
        }
    }
}

struct SlotEvents {
    master_state: Event,
    master_readable: Event,
    master_writable: Event,
    slave_readable: Event,
    slave_writable: Event,
}

impl SlotEvents {
    const fn new() -> Self {
        Self {
            master_state: Event::new(),
            master_readable: Event::new(),
            master_writable: Event::new(),
            slave_readable: Event::new(),
            slave_writable: Event::new(),
        }
    }

    fn reset_for_open(&self) {
        self.master_state.reset();
        self.master_readable.reset();
        self.slave_readable.reset();
        self.master_writable.signal();
        self.slave_writable.signal();
    }
}

struct PtySlot {
    state: IrqSpinLock<SlotState>,
    events: SlotEvents,
}

impl PtySlot {
    const fn new() -> Self {
        Self {
            state: IrqSpinLock::new(SlotState::new()),
            events: SlotEvents::new(),
        }
    }
}

static SLOTS: [PtySlot; SLOT_COUNT] = [const { PtySlot::new() }; SLOT_COUNT];

fn slot_for_context(context: usize) -> Result<&'static PtySlot> {
    SLOTS
        .get(context.checked_sub(1).ok_or(Error::InvalidArgument)?)
        .ok_or(Error::InvalidArgument)
}

fn allocated_queue() -> Result<VecDeque<u8>> {
    let mut queue = VecDeque::new();
    queue
        .try_reserve_exact(RING_CAPACITY)
        .map_err(|_| Error::NoSpace)?;
    Ok(queue)
}

fn drain(queue: &mut VecDeque<u8>, output: &mut [u8]) -> usize {
    let count = output.len().min(queue.len());
    for byte in &mut output[..count] {
        *byte = queue.pop_front().expect("pty: counted queue byte vanished");
    }
    count
}

fn append(queue: &mut VecDeque<u8>, input: &[u8]) -> usize {
    let count = input.len().min(RING_CAPACITY - queue.len());
    queue.extend(&input[..count]);
    count
}

struct Master;

impl Master {
    fn read(slot: &'static PtySlot, output: &mut [u8], nonblocking: bool) -> Result<usize> {
        loop {
            let wait = {
                let mut state = slot.state.lock();
                if !state.active() {
                    return Err(Error::Io);
                }
                if !state.to_master.is_empty() {
                    let count = drain(&mut state.to_master, output);
                    if state.to_master.is_empty() {
                        slot.events.master_readable.reset();
                    }
                    slot.events.master_writable.signal();
                    return Ok(count);
                }
                if state.slave_seen && state.slave_opens == 0 {
                    return Err(Error::Io);
                }
                if nonblocking {
                    return Err(Error::WouldBlock);
                }
                if state.slave_opens == 0 {
                    slot.events.master_state.reset();
                    &slot.events.master_state
                } else {
                    slot.events.master_readable.reset();
                    &slot.events.master_readable
                }
            };
            wait.wait();
        }
    }

    fn write(slot: &'static PtySlot, input: &[u8], nonblocking: bool) -> Result<usize> {
        loop {
            let wait = {
                let mut state = slot.state.lock();
                if !state.active() {
                    return Err(Error::Io);
                }
                if state.slave_seen && state.slave_opens == 0 {
                    return Err(Error::Io);
                }
                if state.slave_opens == 0 {
                    if nonblocking {
                        return Err(Error::WouldBlock);
                    }
                    slot.events.master_state.reset();
                    &slot.events.master_state
                } else if state.to_slave.len() < RING_CAPACITY {
                    let count = append(&mut state.to_slave, input);
                    if state.to_slave.len() == RING_CAPACITY {
                        slot.events.slave_writable.reset();
                    }
                    slot.events.slave_readable.signal();
                    return Ok(count);
                } else {
                    if nonblocking {
                        return Err(Error::WouldBlock);
                    }
                    slot.events.slave_writable.reset();
                    &slot.events.slave_writable
                }
            };
            wait.wait();
        }
    }
}

impl DeviceNodeOps for Master {
    fn open(&self, _flags: u32) -> Result<usize> {
        for (index, slot) in SLOTS.iter().enumerate() {
            {
                let mut state = slot.state.lock();
                if state.reserved {
                    continue;
                }
                state.reserved = true;
            }
            let queues = match (allocated_queue(), allocated_queue()) {
                (Ok(to_master), Ok(to_slave)) => (to_master, to_slave),
                (Err(error), _) | (_, Err(error)) => {
                    slot.state.lock().reserved = false;
                    return Err(error);
                }
            };
            let mut state = slot.state.lock();
            state.master_open = true;
            state.slave_seen = false;
            state.slave_opens = 0;
            state.locked = true;
            state.to_master = queues.0;
            state.to_slave = queues.1;
            slot.events.reset_for_open();
            return Ok(index + 1);
        }
        Err(Error::NoSpace)
    }

    fn close(&self, file_context: usize, _flags: u32) {
        let Ok(slot) = slot_for_context(file_context) else {
            return;
        };
        let mut state = slot.state.lock();
        if !state.active() {
            return;
        }
        state.master_open = false;
        slot.events.master_writable.signal();
        slot.events.slave_readable.signal();
        slot.events.slave_writable.signal();
        state.cleanup();
    }

    fn read_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        sink: &mut IoSink<'_>,
        flags: u32,
    ) -> Result<usize> {
        let slot = slot_for_context(file_context)?;
        read_windows(sink, offset, flags, |_, output, flags| {
            Master::read(
                slot,
                output,
                OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK),
            )
        })
    }

    fn write_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        source: &IoSource<'_>,
        flags: u32,
    ) -> Result<usize> {
        let slot = slot_for_context(file_context)?;
        write_windows(source, offset, flags, |_, input, flags| {
            Master::write(
                slot,
                input,
                OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK),
            )
        })
    }

    fn poll(
        &self,
        file_context: usize,
        _offset: u64,
        events: PollEvents,
        _flags: u32,
    ) -> Result<PollEvents> {
        let state = slot_for_context(file_context)?.state.lock();
        if !state.active() {
            return Ok(PollEvents::HUP);
        }
        let mut ready = PollEvents::empty();
        if !state.to_master.is_empty() {
            ready |= events & (PollEvents::IN | PollEvents::RDNORM);
        }
        if state.slave_seen && state.slave_opens == 0 {
            ready |= PollEvents::HUP;
        } else if state.slave_opens != 0 && state.to_slave.len() < RING_CAPACITY {
            ready |= events & (PollEvents::OUT | PollEvents::WRNORM);
        }
        Ok(ready)
    }

    fn poll_events<'a>(
        &'a self,
        file_context: usize,
        events: PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        let Ok(slot) = slot_for_context(file_context) else {
            return false;
        };
        if events.intersects(PollEvents::IN | PollEvents::RDNORM | PollEvents::HUP) {
            output.push(&slot.events.master_readable);
            output.push(&slot.events.master_state);
        }
        if events.intersects(PollEvents::OUT | PollEvents::WRNORM) {
            output.push(&slot.events.slave_writable);
        }
        true
    }

    fn ioctl(
        &self,
        file_context: usize,
        _context: IoctlContext,
        request: u64,
        _value: u64,
        argument: &mut [u8],
    ) -> Result<u64> {
        let bytes: &mut [u8; 4] = argument.try_into().map_err(|_| Error::InvalidArgument)?;
        let slot = slot_for_context(file_context)?;
        match request {
            tty::TIOCGPTN => {
                *bytes = u32::try_from(file_context - 1)
                    .map_err(|_| Error::InvalidArgument)?
                    .to_ne_bytes();
            }
            tty::TIOCSPTLCK => {
                let mut state = slot.state.lock();
                if !state.active() {
                    return Err(Error::Io);
                }
                state.locked = i32::from_ne_bytes(*bytes) != 0;
            }
            tty::FIONREAD => {
                let count = i32::try_from(slot.state.lock().to_master.len()).unwrap_or(i32::MAX);
                *bytes = count.to_ne_bytes();
            }
            _ => return Err(Error::NotTty),
        }
        Ok(0)
    }
}

struct SlaveBackend {
    slot: &'static PtySlot,
}

impl ConsoleBackend for SlaveBackend {
    fn open(&self) -> Result<()> {
        let mut state = self.slot.state.lock();
        if !state.active() || state.locked {
            return Err(Error::Io);
        }
        state.slave_seen = true;
        state.slave_opens = state.slave_opens.checked_add(1).ok_or(Error::Busy)?;
        self.slot.events.master_state.signal();
        Ok(())
    }

    fn close(&self) {
        let mut state = self.slot.state.lock();
        if !state.reserved || state.slave_opens == 0 {
            return;
        }
        state.slave_opens -= 1;
        if state.slave_opens == 0 {
            state.to_slave.clear();
            self.slot.events.slave_readable.reset();
        }
        self.slot.events.master_state.signal();
        self.slot.events.master_readable.signal();
        self.slot.events.slave_writable.signal();
        state.cleanup();
    }

    fn try_read(&self) -> Option<u8> {
        let mut state = self.slot.state.lock();
        let byte = state.to_slave.pop_front()?;
        if state.to_slave.is_empty() {
            self.slot.events.slave_readable.reset();
        }
        self.slot.events.slave_writable.signal();
        Some(byte)
    }

    fn read(&self, output: &mut [u8]) -> Result<usize> {
        let mut state = self.slot.state.lock();
        let count = drain(&mut state.to_slave, output);
        if state.to_slave.is_empty() {
            self.slot.events.slave_readable.reset();
        }
        self.slot.events.slave_writable.signal();
        Ok(count)
    }

    fn write(&self, bytes: &[u8], nonblocking: bool) -> Result<()> {
        let mut offset = 0;
        while offset < bytes.len() {
            let wait = {
                let mut state = self.slot.state.lock();
                if !state.active() {
                    return Err(Error::Io);
                }
                if nonblocking && RING_CAPACITY - state.to_master.len() < bytes.len() - offset {
                    return Err(Error::WouldBlock);
                }
                if state.to_master.len() < RING_CAPACITY {
                    offset += append(&mut state.to_master, &bytes[offset..]);
                    if state.to_master.len() == RING_CAPACITY {
                        self.slot.events.master_writable.reset();
                    }
                    self.slot.events.master_readable.signal();
                    continue;
                }
                self.slot.events.master_writable.reset();
                &self.slot.events.master_writable
            };
            wait.wait();
        }
        Ok(())
    }

    fn configure(&self, _settings: SerialSettings) -> Result<()> {
        Ok(())
    }

    fn flush_input(&self) -> Result<()> {
        self.slot.state.lock().to_slave.clear();
        self.slot.events.slave_readable.reset();
        self.slot.events.slave_writable.signal();
        Ok(())
    }

    fn flush_output(&self) -> Result<()> {
        self.slot.state.lock().to_master.clear();
        self.slot.events.master_readable.reset();
        self.slot.events.master_writable.signal();
        Ok(())
    }

    fn hung_up(&self) -> bool {
        !self.slot.state.lock().active()
    }
    fn writable(&self) -> bool {
        self.slot.state.lock().to_master.len() < RING_CAPACITY
    }
    fn queued_output(&self) -> usize {
        self.slot.state.lock().to_master.len()
    }
    fn readable_event(&self) -> Option<&Event> {
        Some(&self.slot.events.slave_readable)
    }
    fn writable_event(&self) -> Option<&Event> {
        Some(&self.slot.events.master_writable)
    }
    fn hangup_event(&self) -> Option<&Event> {
        Some(&self.slot.events.slave_readable)
    }
    fn reset_on_last_close(&self) -> bool {
        true
    }
}

/// Publishes the multiplexor and its fixed slave namespace.
pub(super) fn init() -> crate::driver::Result<()> {
    let root = node::root()?;
    let pts = node::create_directory(None, root, "pts", 0o755)?;
    for (index, slot) in SLOTS.iter().enumerate() {
        let name = index.to_string();
        let tty = Tty::new(
            Arc::new(SlaveBackend { slot }),
            alloc::format!("/dev/pts/{name}").into_boxed_str(),
            38_400,
        )?;
        node::create_native_node(None, pts, &name, node::kind::CHARACTER, 0o620, tty)?;
    }
    node::create_native_node(
        None,
        root,
        "ptmx",
        node::kind::CHARACTER,
        0o666,
        Arc::new(Master),
    )?;
    Ok(())
}
