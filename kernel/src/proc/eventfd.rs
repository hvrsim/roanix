//! Shared event counters used for userspace event-loop notifications.

use crate::sys::event::EventSource;

use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::{
    fs::PollEvents,
    mem::{IoSink, IoSource},
    sys::{event::Event, sync::Mutex},
    syscall::{Errno, Result, current_process, map_process_error},
};

use super::Descriptor;

const EFD_SEMAPHORE: u64 = 1;
const EFD_NONBLOCK: u64 = 0o4000;
const EFD_CLOEXEC: u64 = 0o2000000;
const MAX_COUNT: u64 = u64::MAX - 1;

/// One counter shared by duplicated and inherited descriptors.
pub(crate) struct EventFd {
    count: Mutex<u64>,
    semaphore: bool,
    nonblocking: AtomicBool,
    readable: Event,
    writable: Event,
}

impl EventFd {
    fn new(value: u32, flags: u64) -> Arc<Self> {
        Arc::new(Self {
            count: Mutex::new(u64::from(value)),
            semaphore: flags & EFD_SEMAPHORE != 0,
            nonblocking: AtomicBool::new(flags & EFD_NONBLOCK != 0),
            readable: Event::new(),
            writable: Event::new(),
        })
    }

    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize> {
        if sink.len() < 8 {
            return Err(Errno::Invalid);
        }
        loop {
            let wait = self.readable.listen();
            {
                let mut count = self.count.lock();
                if *count != 0 {
                    let value = if self.semaphore { 1 } else { *count };
                    // A failed user copy must leave the counter unchanged.
                    sink.store(0, &value.to_ne_bytes())
                        .map_err(|_| Errno::Fault)?;
                    *count -= value;
                    self.writable.wake_all();
                    return Ok(8);
                }
            }
            if self.is_nonblocking() {
                return Err(Errno::TryAgain);
            }
            wait.wait();
        }
    }

    pub(crate) fn write(&self, source: &IoSource<'_>) -> Result<usize> {
        if source.len() < 8 {
            return Err(Errno::Invalid);
        }
        let mut bytes = [0; 8];
        source.load(0, &mut bytes).map_err(|_| Errno::Fault)?;
        let value = u64::from_ne_bytes(bytes);
        if value == u64::MAX {
            return Err(Errno::Invalid);
        }
        loop {
            let wait = self.writable.listen();
            {
                let mut count = self.count.lock();
                if value <= MAX_COUNT - *count {
                    *count += value;
                    self.readable.wake_all();
                    return Ok(8);
                }
            }
            if self.is_nonblocking() {
                return Err(Errno::TryAgain);
            }
            wait.wait();
        }
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        let count = *self.count.lock();
        let mut ready = PollEvents::empty();
        if count != 0 {
            ready |= requested & (PollEvents::IN | PollEvents::RDNORM);
        }
        if count < MAX_COUNT {
            ready |= requested & (PollEvents::OUT | PollEvents::WRNORM);
        }
        ready
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        events: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        if events.intersects(PollEvents::IN | PollEvents::RDNORM) {
            output.push((&self.readable).into());
        }
        if events.intersects(PollEvents::OUT | PollEvents::WRNORM) {
            output.push((&self.writable).into());
        }
        true
    }

    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }

    pub(crate) fn set_nonblocking(&self, value: bool) {
        self.nonblocking.store(value, Ordering::Release);
        self.readable.wake_all();
        self.writable.wake_all();
    }
}

crate::syscall_handler! {
    syscall_eventfd_create(_frame, value: u64 = 0, flags: u64 = 1) {
        let value = u32::try_from(value).map_err(|_| Errno::Invalid)?;
        if flags & !(EFD_SEMAPHORE | EFD_NONBLOCK | EFD_CLOEXEC) != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let fd = process.install_descriptor_value(
            Descriptor::EventFd(EventFd::new(value, flags)), flags & EFD_CLOEXEC != 0,
        ).map_err(map_process_error)?;
        Ok(fd as u64)
    }
}
