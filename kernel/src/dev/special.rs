//! Kernel-owned null, zero, and random pseudo-devices.

use alloc::sync::Arc;

use crate::{
    fs::{Error, IoctlContext, PollEvents, Result, devfs::DeviceNodeOps},
    mem::{IoSink, IoSource},
};

use super::node;

#[derive(Copy, Clone)]
enum Kind {
    Null,
    Zero,
    Random,
}

struct Special(Kind);

impl DeviceNodeOps for Special {
    fn read_at(&self, _offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        match self.0 {
            Kind::Null => Ok(0),
            Kind::Zero => {
                sink.fill_zero(0, sink.len())
                    .map_err(|_| Error::InvalidArgument)?;
                Ok(sink.len())
            }
            Kind::Random => {
                let length = sink.len();
                let mut done = 0;
                let mut buffer = [0u8; 256];
                while done < length {
                    let mut window = sink
                        .window(done, (length - done).min(buffer.len()))
                        .map_err(|_| Error::InvalidArgument)?;
                    if window.is_empty() {
                        break;
                    }
                    let count = window.len();
                    crate::sys::random::fill_bytes(&mut buffer[..count]);
                    window.copy_from_slice(&buffer[..count]);
                    done += count;
                }
                Ok(done)
            }
        }
    }

    fn write_at(&self, _offset: u64, source: &IoSource<'_>) -> Result<usize> {
        if matches!(self.0, Kind::Random) {
            let length = source.len();
            let mut done = 0;
            let mut buffer = [0u8; 256];
            while done < length {
                let window = source
                    .window(done, (length - done).min(buffer.len()))
                    .map_err(|_| Error::InvalidArgument)?;
                if window.is_empty() {
                    break;
                }
                let count = window.len();
                window.copy_to_slice(&mut buffer[..count]);
                crate::sys::random::mix_bytes(&buffer[..count]);
                done += count;
            }
            return Ok(done);
        }
        Ok(source.len())
    }

    fn poll(
        &self,
        _file_context: usize,
        _offset: u64,
        events: PollEvents,
        _flags: u32,
    ) -> Result<PollEvents> {
        Ok(events & (PollEvents::IN | PollEvents::RDNORM | PollEvents::OUT | PollEvents::WRNORM))
    }

    fn ioctl(
        &self,
        _file_context: usize,
        _context: IoctlContext,
        _request: u64,
        _value: u64,
        _argument: &mut [u8],
    ) -> Result<u64> {
        Err(Error::NotTty)
    }
}

fn publish(name: &str, kind: Kind) -> crate::driver::Result<()> {
    node::create_native_node(
        None,
        node::root()?,
        name,
        node::kind::CHARACTER,
        0o666,
        Arc::new(Special(kind)),
    )?;
    Ok(())
}

pub(super) fn init() -> crate::driver::Result<()> {
    publish("null", Kind::Null)?;
    publish("zero", Kind::Zero)?;
    publish("random", Kind::Random)?;
    publish("urandom", Kind::Random)
}
