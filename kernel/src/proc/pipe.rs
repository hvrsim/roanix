//! Anonymous byte pipes used by process descriptors.

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
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Direction {
    Read,
    Write,
}

struct PipeState {
    buffer: VecDeque<u8>,
    reader_open: bool,
    writer_open: bool,
}

struct Pipe {
    state: Mutex<PipeState>,
    readable: Event,
    writable: Event,
}

/// One shared endpoint of an anonymous pipe.
pub(crate) struct PipeEnd {
    pipe: Arc<Pipe>,
    direction: Direction,
    nonblocking: AtomicBool,
}

impl PipeEnd {
    /// Creates the read and write endpoints for one empty pipe.
    pub(crate) fn pair() -> (Arc<Self>, Arc<Self>) {
        let pipe = Arc::new(Pipe {
            state: Mutex::new(PipeState {
                buffer: VecDeque::with_capacity(PIPE_CAPACITY),
                reader_open: true,
                writer_open: true,
            }),
            readable: Event::new(),
            writable: Event::new(),
        });
        pipe.writable.signal();
        (
            Arc::new(Self {
                pipe: pipe.clone(),
                direction: Direction::Read,
                nonblocking: AtomicBool::new(false),
            }),
            Arc::new(Self {
                pipe,
                direction: Direction::Write,
                nonblocking: AtomicBool::new(false),
            }),
        )
    }

    /// Reads bytes, blocking until data or writer closure is observable.
    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize, PipeError> {
        if self.direction != Direction::Read {
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
            if !state.writer_open {
                return Ok(0);
            }
            if self.nonblocking.load(Ordering::Acquire) {
                return Err(PipeError::TryAgain);
            }
            self.pipe.readable.reset();
            drop(state);
            self.pipe.readable.wait();
        }
    }

    /// Writes bytes, blocking only while the pipe buffer is full.
    pub(crate) fn write(&self, source: &IoSource<'_>) -> Result<usize, PipeError> {
        if self.direction != Direction::Write {
            return Err(PipeError::BadDescriptor);
        }
        if source.is_empty() {
            return Ok(0);
        }
        loop {
            let mut state = self.pipe.state.lock();
            if !state.reader_open {
                return Err(PipeError::BrokenPipe);
            }
            let available = PIPE_CAPACITY - state.buffer.len();
            let atomic = source.len() <= PIPE_ATOMIC_LIMIT;
            if available != 0 && (!atomic || available >= source.len()) {
                let count = available.min(source.len());
                let mut copied = 0usize;
                while copied < count {
                    let window = match source.window(copied, count - copied) {
                        Ok(window) => window,
                        Err(_) if copied != 0 => break,
                        Err(_) => return Err(PipeError::Fault),
                    };
                    if window.is_empty() {
                        break;
                    }
                    state.buffer.extend(&*window);
                    copied += window.len();
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
            drop(state);
            self.pipe.writable.wait();
        }
    }

    /// Returns the POSIX access mode for this endpoint.
    pub(crate) fn access_mode(&self) -> u64 {
        match self.direction {
            Direction::Read => 0,
            Direction::Write => 1,
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
        match self.direction {
            Direction::Read => {
                let mut ready = PollEvents::empty();
                if !state.buffer.is_empty() {
                    ready |= events & (PollEvents::IN | PollEvents::RDNORM);
                }
                if !state.writer_open {
                    ready |= PollEvents::HUP;
                }
                ready
            }
            Direction::Write => {
                if !state.reader_open {
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
        &'a self,
        events: PollEvents,
        output: &mut alloc::vec::Vec<&'a Event>,
    ) -> bool {
        match self.direction {
            Direction::Read
                if events.intersects(PollEvents::IN | PollEvents::RDNORM | PollEvents::HUP) =>
            {
                output.push(&self.pipe.readable);
            }
            Direction::Write
                if events.intersects(PollEvents::OUT | PollEvents::WRNORM | PollEvents::ERR) =>
            {
                output.push(&self.pipe.writable);
            }
            _ => {}
        }
        true
    }
}

impl Drop for PipeEnd {
    fn drop(&mut self) {
        let mut state = self.pipe.state.lock();
        match self.direction {
            Direction::Read => {
                state.reader_open = false;
                self.pipe.writable.signal();
            }
            Direction::Write => {
                state.writer_open = false;
                self.pipe.readable.signal();
            }
        }
    }
}
