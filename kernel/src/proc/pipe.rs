//! Byte pipes for anonymous descriptors and named FIFOs.

use crate::sys::event::{EventListener, EventSource};

use alloc::{collections::VecDeque, sync::Arc};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::{
    fs::PollEvents,
    mem::{IoSink, IoSource},
    sys::{event::Event, sync::Mutex},
};

const PIPE_CAPACITY: usize = 64 * 1024;
const PIPE_ATOMIC_LIMIT: usize = 4096;

/// Pipe operation failure.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum PipeError {
    /// The endpoint does not permit the requested operation.
    BadDescriptor,
    /// A nonblocking operation would have slept.
    TryAgain,
    /// No reader remains attached to the pipe.
    BrokenPipe,
    /// The caller buffer could not be accessed.
    Fault,
    /// A pending signal interrupted the blocking operation.
    Interrupted,
    /// A nonblocking FIFO writer has no reader.
    NoReader,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Direction {
    Read,
    Write,
    Both,
}

struct PipeState {
    buffer: VecDeque<u8>,
    readers: usize,
    writers: usize,
    reader_generation: u64,
    writer_generation: u64,
}

struct Pipe {
    state: Mutex<PipeState>,
    readable: Arc<Event>,
    writable: Arc<Event>,
}

/// One shared open description for a pipe endpoint.
pub(crate) struct PipeEnd {
    pipe: Arc<Pipe>,
    direction: Direction,
    nonblocking: AtomicBool,
    initial_writer_generation: Option<u64>,
}

impl PipeEnd {
    /// Creates the read and write endpoints for one empty pipe.
    pub(crate) fn pair() -> (Arc<Self>, Arc<Self>) {
        let pipe = Arc::new(Pipe {
            state: Mutex::new(PipeState {
                buffer: VecDeque::with_capacity(PIPE_CAPACITY),
                readers: 1,
                writers: 1,
                reader_generation: 1,
                writer_generation: 1,
            }),
            readable: Arc::new(Event::new()),
            writable: Arc::new(Event::new()),
        });
        pipe.writable.signal();
        (
            Arc::new(Self {
                pipe: pipe.clone(),
                direction: Direction::Read,
                nonblocking: AtomicBool::new(false),
                initial_writer_generation: None,
            }),
            Arc::new(Self {
                pipe,
                direction: Direction::Write,
                nonblocking: AtomicBool::new(false),
                initial_writer_generation: None,
            }),
        )
    }

    /// Reads bytes, blocking until data or writer closure is observable.
    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize, PipeError> {
        if self.direction == Direction::Write {
            return Err(PipeError::BadDescriptor);
        }
        if sink.is_empty() {
            return Ok(0);
        }
        loop {
            let mut state = self.pipe.state.lock();
            if !state.buffer.is_empty() {
                let count = sink.len().min(state.buffer.len());
                let copied = {
                    let (front, back) = state.buffer.as_slices();
                    let from_front = front.len().min(count);
                    sink.store(0, &front[..from_front])
                        .map_err(|_| PipeError::Fault)?;
                    if from_front < count {
                        sink.store(from_front, &back[..count - from_front])
                            .map_err(|_| PipeError::Fault)?;
                    }
                    count
                };
                state.buffer.drain(..copied);
                if state.buffer.is_empty() {
                    self.pipe.readable.reset();
                }
                self.pipe.writable.signal();
                return Ok(copied);
            }
            if state.writers == 0 {
                return Ok(0);
            }
            if self.nonblocking.load(Ordering::Acquire) {
                return Err(PipeError::TryAgain);
            }
            self.pipe.readable.reset();
            let ready = self.pipe.readable.listen();
            drop(state);
            wait_interruptibly(ready)?;
        }
    }

    /// Writes bytes, blocking only while the pipe buffer is full.
    pub(crate) fn write(&self, source: &IoSource<'_>) -> Result<usize, PipeError> {
        if self.direction == Direction::Read {
            return Err(PipeError::BadDescriptor);
        }
        if source.is_empty() {
            return Ok(0);
        }
        loop {
            let mut state = self.pipe.state.lock();
            if state.readers == 0 {
                return Err(PipeError::BrokenPipe);
            }
            let available = PIPE_CAPACITY - state.buffer.len();
            let atomic = source.len() <= PIPE_ATOMIC_LIMIT;
            if available != 0 && (!atomic || available >= source.len()) {
                let count = available.min(source.len());
                let mut copied = 0usize;
                let mut buffer = [0u8; 1024];
                while copied < count {
                    let window = match source.window(copied, (count - copied).min(buffer.len())) {
                        Ok(window) => window,
                        Err(_) if copied != 0 => break,
                        Err(_) => return Err(PipeError::Fault),
                    };
                    if window.is_empty() {
                        break;
                    }
                    let length = window.len();
                    window.copy_to_slice(&mut buffer[..length]);
                    state.buffer.extend(&buffer[..length]);
                    copied += length;
                }
                if state.buffer.len() == PIPE_CAPACITY {
                    self.pipe.writable.reset();
                }
                self.pipe.readable.signal();
                return Ok(copied);
            }
            if self.nonblocking.load(Ordering::Acquire) {
                return Err(PipeError::TryAgain);
            }
            self.pipe.writable.reset();
            let ready = self.pipe.writable.listen();
            drop(state);
            wait_interruptibly(ready)?;
        }
    }

    /// Returns the POSIX access mode for this endpoint.
    pub(crate) fn access_mode(&self) -> u64 {
        match self.direction {
            Direction::Read => 0,
            Direction::Write => 1,
            Direction::Both => 2,
        }
    }

    /// Changes whether operations return instead of sleeping.
    pub(crate) fn set_nonblocking(&self, value: bool) {
        self.nonblocking.store(value, Ordering::Release);
    }

    /// Returns whether operations avoid sleeping.
    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }

    /// Returns requested events that are immediately ready.
    pub(crate) fn poll(&self, events: PollEvents) -> PollEvents {
        let state = self.pipe.state.lock();
        if self.direction == Direction::Both {
            let mut ready = PollEvents::empty();
            if !state.buffer.is_empty() {
                ready |= events & (PollEvents::IN | PollEvents::RDNORM);
            }
            if state.buffer.len() < PIPE_CAPACITY {
                ready |= events & (PollEvents::OUT | PollEvents::WRNORM);
            }
            return ready;
        }
        match self.direction {
            Direction::Read => {
                let mut ready = PollEvents::empty();
                if !state.buffer.is_empty() {
                    ready |= events & (PollEvents::IN | PollEvents::RDNORM);
                }
                if state.writers == 0
                    && self.initial_writer_generation != Some(state.writer_generation)
                {
                    ready |= PollEvents::HUP;
                }
                ready
            }
            Direction::Both => unreachable!(),
            Direction::Write => {
                if state.readers == 0 {
                    PollEvents::ERR
                } else if state.buffer.len() < PIPE_CAPACITY {
                    events & (PollEvents::OUT | PollEvents::WRNORM)
                } else {
                    PollEvents::empty()
                }
            }
        }
    }

    pub(crate) fn poll_events<'a>(
        &self,
        events: PollEvents,
        output: &mut alloc::vec::Vec<EventSource<'a>>,
    ) -> bool {
        if self.direction != Direction::Write
            && events.intersects(PollEvents::IN | PollEvents::RDNORM | PollEvents::HUP)
        {
            output.push(self.pipe.readable.clone().into());
        }
        if self.direction != Direction::Read
            && events.intersects(PollEvents::OUT | PollEvents::WRNORM | PollEvents::ERR)
        {
            output.push(self.pipe.writable.clone().into());
        }
        true
    }
}

fn wait_interruptibly(ready: EventListener<'_>) -> Result<(), PipeError> {
    if let Some(process) = super::current() {
        let interrupt = process.interrupt_event().listen();
        if process.is_exited() || process.prepare_interrupt_wait() {
            return Err(PipeError::Interrupted);
        }
        EventListener::wait_any(&[ready, interrupt]);
    } else {
        ready.wait();
    }
    Ok(())
}

impl Drop for PipeEnd {
    fn drop(&mut self) {
        let mut state = self.pipe.state.lock();
        if self.direction != Direction::Write {
            state.readers -= 1;
        }
        if self.direction != Direction::Read {
            state.writers -= 1;
        }
        if state.readers == 0 && state.writers == 0 {
            state.buffer.clear();
        }
        self.pipe.readable.signal();
        self.pipe.writable.signal();
    }
}

/// Reopenable byte pipe backing a FIFO inode.
pub(crate) struct NamedPipe {
    pipe: Arc<Pipe>,
}
impl NamedPipe {
    pub(crate) fn new() -> Self {
        Self {
            pipe: Arc::new(Pipe {
                state: Mutex::new(PipeState {
                    buffer: VecDeque::new(),
                    readers: 0,
                    writers: 0,
                    reader_generation: 0,
                    writer_generation: 0,
                }),
                readable: Arc::new(Event::new()),
                writable: Arc::new(Event::new()),
            }),
        }
    }
    pub(crate) fn open(
        &self,
        read: bool,
        write: bool,
        nonblocking: bool,
    ) -> Result<Arc<PipeEnd>, PipeError> {
        if !read && !write {
            return Err(PipeError::BadDescriptor);
        }
        let direction = if read && write {
            Direction::Both
        } else if read {
            Direction::Read
        } else {
            Direction::Write
        };
        let (reader_generation, writer_generation, peer_present) = {
            let mut state = self.pipe.state.lock();
            if write && !read && nonblocking && state.readers == 0 {
                return Err(PipeError::NoReader);
            }
            if read {
                state.readers += 1;
                state.reader_generation = state.reader_generation.wrapping_add(1);
            }
            if write {
                state.writers += 1;
                state.writer_generation = state.writer_generation.wrapping_add(1);
            }
            (
                state.reader_generation,
                state.writer_generation,
                (read && state.writers != 0) || (write && state.readers != 0),
            )
        };
        let end = Arc::new(PipeEnd {
            pipe: self.pipe.clone(),
            direction,
            nonblocking: AtomicBool::new(nonblocking),
            initial_writer_generation: (nonblocking && read && !peer_present)
                .then_some(writer_generation),
        });
        self.pipe.readable.signal();
        self.pipe.writable.signal();
        if nonblocking || peer_present {
            return Ok(end);
        }
        loop {
            let event = if read {
                &self.pipe.readable
            } else {
                &self.pipe.writable
            };
            let ready = {
                let state = self.pipe.state.lock();
                if (read && (state.writers != 0 || state.writer_generation != writer_generation))
                    || (write
                        && (state.readers != 0 || state.reader_generation != reader_generation))
                {
                    return Ok(end);
                }
                event.reset();
                event.listen()
            };
            wait_interruptibly(ready)?;
        }
    }
}
