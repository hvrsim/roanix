//! Kernel-native special character devices.

use alloc::{sync::Arc, vec::Vec};

use crate::{
    fs::{
        Error as FsError, OpenFlags, Result as FsResult,
        devtempfs::{self, DevNodeId, DeviceNodeKind, DeviceNodeOps},
    },
    sys::{debug, random},
};

use super::{BusId, Error, KERNEL_DRIVER, Result};

struct KmsgDevice;
struct NullDevice;
struct ZeroDevice;
struct RandomDevice;

pub(super) fn start() -> Result<()> {
    random::init();

    let filesystem = devtempfs::global().map_err(|_| Error::Filesystem)?;
    let bus = super::register_bus(KERNEL_DRIVER, super::root_bus()?, "special")?;
    let mut nodes = Vec::new();

    let result = (|| {
        publish(
            filesystem,
            bus,
            &mut nodes,
            "kmsg",
            0o600,
            Arc::new(KmsgDevice),
        )?;
        debug::disable_regular_sink_output();
        publish(
            filesystem,
            bus,
            &mut nodes,
            "random",
            0o666,
            Arc::new(RandomDevice),
        )?;
        publish(
            filesystem,
            bus,
            &mut nodes,
            "urandom",
            0o666,
            Arc::new(RandomDevice),
        )?;
        publish(
            filesystem,
            bus,
            &mut nodes,
            "zero",
            0o666,
            Arc::new(ZeroDevice),
        )?;
        publish(
            filesystem,
            bus,
            &mut nodes,
            "null",
            0o666,
            Arc::new(NullDevice),
        )
    })();

    if let Err(error) = result {
        for node in nodes.into_iter().rev() {
            let _ = filesystem.remove_node(KERNEL_DRIVER, node);
        }
        let _ = super::remove_node(KERNEL_DRIVER, bus.node());
        return Err(error);
    }

    Ok(())
}

fn publish(
    filesystem: &Arc<devtempfs::Devtempfs>,
    bus: BusId,
    nodes: &mut Vec<DevNodeId>,
    name: &str,
    mode: u16,
    operations: Arc<dyn DeviceNodeOps>,
) -> Result<()> {
    let device = super::register_device(KERNEL_DRIVER, bus, name)?;
    match filesystem.create_device(
        KERNEL_DRIVER,
        filesystem.root_id(),
        name.as_bytes(),
        DeviceNodeKind::Character,
        mode,
        device.node(),
        operations,
    ) {
        Ok(node) => {
            nodes.push(node);
            Ok(())
        }
        Err(error) => {
            let _ = super::remove_node(KERNEL_DRIVER, device.node());
            Err(fs_error(error))
        }
    }
}

impl DeviceNodeOps for KmsgDevice {
    fn initial_offset(&self, _flags: u32) -> FsResult<u64> {
        Ok(debug::log_start_offset())
    }

    fn read_at_with_flags(&self, offset: u64, buffer: &mut [u8], flags: u32) -> FsResult<usize> {
        let nonblocking = OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK);
        debug::read_log(offset, buffer, nonblocking).map_err(|error| match error {
            debug::LogReadError::Overrun => FsError::Io,
            debug::LogReadError::WouldBlock => FsError::WouldBlock,
        })
    }

    fn write_at(&self, _offset: u64, buffer: &[u8]) -> FsResult<usize> {
        debug::append_kernel_message(buffer);
        Ok(buffer.len())
    }

    fn size(&self) -> u64 {
        debug::log_end_offset()
    }
}

impl DeviceNodeOps for NullDevice {
    fn read_at(&self, _offset: u64, _buffer: &mut [u8]) -> FsResult<usize> {
        Ok(0)
    }

    fn write_at(&self, _offset: u64, buffer: &[u8]) -> FsResult<usize> {
        Ok(buffer.len())
    }
}

impl DeviceNodeOps for ZeroDevice {
    fn read_at(&self, _offset: u64, buffer: &mut [u8]) -> FsResult<usize> {
        buffer.fill(0);
        Ok(buffer.len())
    }

    fn write_at(&self, _offset: u64, buffer: &[u8]) -> FsResult<usize> {
        Ok(buffer.len())
    }
}

impl DeviceNodeOps for RandomDevice {
    fn read_at(&self, _offset: u64, buffer: &mut [u8]) -> FsResult<usize> {
        random::fill_bytes(buffer);
        Ok(buffer.len())
    }

    fn write_at(&self, _offset: u64, buffer: &[u8]) -> FsResult<usize> {
        random::mix_bytes(buffer);
        Ok(buffer.len())
    }
}

fn fs_error(error: FsError) -> Error {
    match error {
        FsError::NotFound => Error::NotFound,
        FsError::AlreadyExists => Error::AlreadyExists,
        FsError::Busy | FsError::NotEmpty => Error::Busy,
        FsError::PermissionDenied | FsError::ReadOnly => Error::PermissionDenied,
        FsError::NoSpace => Error::NoSpace,
        FsError::OutOfMemory => Error::OutOfMemory,
        FsError::InvalidArgument | FsError::NameTooLong => Error::InvalidArgument,
        _ => Error::Filesystem,
    }
}
