//! Named-pipe open descriptions for memory-backed FIFO inodes.
use super::{Error, OpenFlags, PollEvents, Result};
use crate::{
    mem::{IoSink, IoSource},
    proc::pipe::{NamedPipe, PipeEnd, PipeError},
    sys::{event::EventSource, sync::Mutex},
};
use alloc::{collections::BTreeMap, sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};

pub(super) struct Fifo {
    pipe: NamedPipe,
    opens: Mutex<BTreeMap<usize, Arc<PipeEnd>>>,
    next: AtomicUsize,
}
impl Fifo {
    pub fn new() -> Self {
        Self {
            pipe: NamedPipe::new(),
            opens: Mutex::new(BTreeMap::new()),
            next: AtomicUsize::new(1),
        }
    }
    pub fn open(&self, flags: u32) -> Result<usize> {
        let flags = OpenFlags::from_bits_retain(flags);
        let pipe = self
            .pipe
            .open(
                flags.contains(OpenFlags::READ),
                flags.contains(OpenFlags::WRITE),
                flags.contains(OpenFlags::NONBLOCK),
            )
            .map_err(error)?;
        let id = self
            .next
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_add(1))
            .map_err(|_| Error::NoSpace)?;
        self.opens.lock().insert(id, pipe);
        Ok(id)
    }
    pub fn close(&self, id: usize) {
        self.opens.lock().remove(&id);
    }
    fn endpoint(&self, id: usize) -> Result<Arc<PipeEnd>> {
        self.opens
            .lock()
            .get(&id)
            .cloned()
            .ok_or(Error::BadFileDescriptor)
    }
    pub fn read(&self, id: usize, sink: &mut IoSink<'_>, flags: u32) -> Result<usize> {
        let pipe = self.endpoint(id)?;
        pipe.set_nonblocking(OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK));
        pipe.read(sink).map_err(error)
    }
    pub fn write(&self, id: usize, source: &IoSource<'_>, flags: u32) -> Result<usize> {
        let pipe = self.endpoint(id)?;
        pipe.set_nonblocking(OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK));
        pipe.write(source).map_err(error)
    }
    pub fn poll(&self, id: usize, events: PollEvents) -> Result<PollEvents> {
        Ok(self.endpoint(id)?.poll(events))
    }
    pub fn events<'a>(
        &self,
        id: usize,
        events: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        if let Ok(pipe) = self.endpoint(id) {
            pipe.poll_events(events, output);
        }
        true
    }
}
fn error(error: PipeError) -> Error {
    match error {
        PipeError::NoReader => Error::NoDeviceAddress,
        PipeError::BadDescriptor => Error::BadFileDescriptor,
        PipeError::TryAgain => Error::WouldBlock,
        PipeError::BrokenPipe => Error::BrokenPipe,
        PipeError::Fault => Error::InvalidArgument,
        PipeError::Interrupted => Error::Interrupted,
    }
}
