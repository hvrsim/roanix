//!
//! # USB Mass Storage
//!
//! This driver binds SCSI transparent mass-storage interfaces that use USB
//! Bulk-Only Transport. It publishes each supported logical unit as a block
//! device and asks the block layer to discover its partitions.
//!
//! Each USB interface has a sleepable lock for its command, data, and status
//! sequence. Logical units on that interface share the lock because the wire
//! protocol allows one command at a time. Separate interfaces can transfer
//! concurrently. The host stages transfers in reusable endpoint DMA buffers.
//!
//! Removal withdraws block registrations before releasing callback state.
//! The host cancels outstanding transfers on disconnect. Busy block devices
//! retain their registration and context until the host retries removal.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code, clippy::cast_possible_truncation)]

extern crate alloc;

use alloc::{boxed::Box, ffi::CString, format, sync::Arc, vec::Vec};
use core::{ffi::c_void, ptr};
use ddk::{
    BlockRegistration, Bus, Device, DriverRegistration, Error, Module, Mutex, Result, TicketLock,
    raw,
};
use usb::{Bulk, Host};

const CBW_SIGNATURE: [u8; 4] = *b"USBC";
const CSW_SIGNATURE: [u8; 4] = *b"USBS";

/// Data stage of one SCSI command, with direction derived from buffer ownership.
enum Data<'a> {
    /// Command with no data stage.
    None,

    /// Destination for data from the device.
    In(&'a mut [u8]),

    /// Source of data sent to the device.
    Out(&'a [u8]),
}

impl Data<'_> {
    /// Returns the expected wire length of this command data stage.
    fn len(&self) -> usize {
        match self {
            Self::None => 0,
            Self::In(data) => data.len(),
            Self::Out(data) => data.len(),
        }
    }
}

/// Result of a valid Command Status Wrapper.
enum Completion {
    /// Successful command with the number of bytes processed by the device.
    Passed(usize),

    /// SCSI command failure to be followed by REQUEST SENSE.
    Failed,
}

/// Wire state shared by all logical units on one USB interface.
struct Transport {
    /// USB interface device passed to the host controller.
    device: Device,

    /// Interface number used in the class-specific setup packets.
    interface: u16,

    /// Retained owner of the host control pipe.
    host: Host,

    /// Retained owner of the bulk IN and OUT pipes.
    bulk: Bulk,

    /// Tag allocated to the next command.
    next_tag: u32,

    /// A failed reset leaves an interface unsafe to submit to again.
    failed: bool,
}

impl Transport {
    /// Retains both host services and reads the USB interface number.
    fn new(device: Device) -> Result<Self> {
        let interface = u16::try_from(device.integer(c"usb.interface")?)
            .map_err(|_| Error::from_status(ddk::EINVAL))?;
        Ok(Self {
            device,
            interface,
            host: Host::bind()?,
            bulk: Bulk::bind()?,
            next_tag: 0,
            failed: false,
        })
    }

    /// Queries the highest logical unit and handles the single-LUN STALL response.
    fn max_lun(&self) -> Result<u8> {
        let [low, high] = self.interface.to_le_bytes();
        let mut lun = [0];
        match self
            .host
            .control(self.device, [0xa1, 0xfe, 0, 0, low, high, 1, 0], &mut lun)
        {
            Ok(1) if lun[0] <= 15 => Ok(lun[0]),
            Err(error) if error.status() == usb::STALL => Ok(0),
            Err(error) => Err(error),
            Ok(_) => Err(io_error()),
        }
    }

    /// Executes a command and reads sense data before releasing transport admission.
    fn command(&mut self, lun: u8, cdb: &[u8], data: Data<'_>) -> Result<usize> {
        match self.exchange(lun, cdb, data)? {
            Completion::Passed(count) => Ok(count),
            Completion::Failed => Err(self.request_sense(lun)?),
        }
    }

    /// Maps fixed and descriptor-format sense keys to block-layer errors.
    fn request_sense(&mut self, lun: u8) -> Result<Error> {
        let mut sense = [0; 18];
        let cdb = [0x03, 0, 0, 0, 18, 0];
        let result = self.exchange(lun, &cdb, Data::In(&mut sense))?;
        let key = match (result, sense[0] & 0x7f) {
            (Completion::Passed(3..), 0x70 | 0x71) => sense[2] & 15,
            (Completion::Passed(2..), 0x72 | 0x73) => sense[1] & 15,
            _ => return Ok(io_error()),
        };

        Ok(Error::from_status(match key {
            0x02 | 0x06 => ddk::EAGAIN,
            0x05 => ddk::ENOTSUP,
            0x07 => ddk::EROFS,
            _ => ddk::EIO,
        }))
    }

    /// Runs one wire transaction and resets the interface after transport errors.
    fn exchange(&mut self, lun: u8, cdb: &[u8], data: Data<'_>) -> Result<Completion> {
        if self.failed {
            return Err(Error::from_status(ddk::EIO));
        }

        let length = data.len();
        if lun > 15 || cdb.is_empty() || cdb.len() > 16 || length > usb::BULK_MAX_TRANSFER {
            return Err(Error::from_status(ddk::EINVAL));
        }

        let tag = self.next_tag.to_le_bytes();
        self.next_tag = self.next_tag.wrapping_add(1);
        let mut cbw = [0; 31];
        cbw[..4].copy_from_slice(&CBW_SIGNATURE);
        cbw[4..8].copy_from_slice(&tag);
        cbw[8..12].copy_from_slice(&(length as u32).to_le_bytes());
        cbw[12] = if matches!(&data, Data::In(_)) {
            0x80
        } else {
            0
        };
        cbw[13] = lun;
        cbw[14] = cdb.len() as u8;
        cbw[15..15 + cdb.len()].copy_from_slice(cdb);

        let result = (|| {
            if self.bulk.write(self.device, &cbw)? != cbw.len() {
                return Err(io_error());
            }

            let transferred = self.transfer_data(data)?;
            self.status(tag, length, transferred)
        })();
        if result
            .as_ref()
            .is_err_and(|error| error.status() != ddk::ENODEV)
        {
            self.failed = self.reset().is_err();
        }
        result
    }

    /// Performs the data stage and clears STALL before reading command status.
    fn transfer_data(&self, data: Data<'_>) -> Result<Option<usize>> {
        let (input, result) = match data {
            Data::None => return Ok(Some(0)),
            Data::In(buffer) => (true, self.bulk.read(self.device, buffer)),
            Data::Out(buffer) => (false, self.bulk.write(self.device, buffer)),
        };

        match result {
            Err(error) if error.status() == usb::STALL => {
                self.bulk.clear_halt(self.device, input)?;
                // A stalled data stage finishes with a CSW. The host did not
                // deliver data to the caller, so only a failed command is usable.
                Ok(None)
            }
            result => result.map(Some),
        }
    }

    /// Reads and checks the status wrapper, retrying once after an IN halt.
    fn status(
        &self,
        tag: [u8; 4],
        expected: usize,
        transferred: Option<usize>,
    ) -> Result<Completion> {
        let mut csw = [0; 13];
        let received = match self.bulk.read(self.device, &mut csw) {
            Err(error) if error.status() == usb::STALL => {
                self.bulk.clear_halt(self.device, true)?;
                self.bulk.read(self.device, &mut csw)?
            }
            result => result?,
        };

        if received != csw.len() || csw[..4] != CSW_SIGNATURE || csw[4..8] != tag {
            return Err(io_error());
        }

        let residue = u32::from_le_bytes([csw[8], csw[9], csw[10], csw[11]]) as usize;
        let processed = expected.checked_sub(residue).ok_or_else(io_error)?;
        match csw[12] {
            0 if transferred.is_some_and(|bytes| processed <= bytes) => {
                Ok(Completion::Passed(processed))
            }
            1 => Ok(Completion::Failed),
            _ => Err(io_error()),
        }
    }

    /// Resets Bulk-Only Transport and clears the IN and OUT endpoint halts in order.
    fn reset(&self) -> Result<()> {
        let [low, high] = self.interface.to_le_bytes();
        self.host
            .control(self.device, [0x21, 0xff, 0, 0, low, high, 0, 0], &mut [])?;
        self.bulk.clear_halt(self.device, true)?;
        self.bulk.clear_halt(self.device, false)
    }
}

fn io_error() -> Error {
    Error::from_status(ddk::EIO)
}

/// Block geometry returned by READ CAPACITY.
struct Capacity {
    /// Number of addressable logical blocks.
    blocks: u64,

    /// Size of one logical block in bytes.
    block_size: u32,
}

impl Capacity {
    /// Reads capacity and uses the 16-byte command when the 10-byte result saturates.
    fn read(transport: &mut Transport, lun: u8) -> Result<Self> {
        let mut small = [0; 8];
        let cdb = [0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let mut result = transport.command(lun, &cdb, Data::In(&mut small));
        if result
            .as_ref()
            .is_err_and(|error| matches!(error.status(), ddk::EAGAIN | ddk::EIO | ddk::ENOTSUP))
        {
            // A new LUN may report unit attention without returning usable sense data.
            result = transport.command(lun, &cdb, Data::In(&mut small));
        }
        complete(result?, small.len())?;

        let last = u32::from_be_bytes([small[0], small[1], small[2], small[3]]);
        let (last, block_size) = if last == u32::MAX {
            let mut large = [0; 32];
            let mut cdb = [0; 16];
            cdb[0] = 0x9e;
            cdb[1] = 0x10;
            cdb[13] = 32;
            let count = transport.command(lun, &cdb, Data::In(&mut large))?;
            complete(count, large.len())?;
            (
                u64::from_be_bytes(large[..8].try_into().expect("eight LBA bytes")),
                u32::from_be_bytes(large[8..12].try_into().expect("four block size bytes")),
            )
        } else {
            (
                u64::from(last),
                u32::from_be_bytes([small[4], small[5], small[6], small[7]]),
            )
        };

        let blocks = last.checked_add(1).ok_or(Error::from_status(ddk::EIO))?;
        if block_size < 512
            || !block_size.is_power_of_two()
            || block_size as usize > usb::BULK_MAX_TRANSFER
        {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        Ok(Self { blocks, block_size })
    }
}

/// Reads or writes a block range using the smallest applicable command format.
fn transfer_blocks(
    transport: &mut Transport,
    lun: u8,
    lba: u64,
    blocks: u32,
    data: Data<'_>,
) -> Result<()> {
    let bytes = data.len();
    let write = matches!(&data, Data::Out(_));
    let count = if u32::try_from(lba + u64::from(blocks) - 1).is_ok() {
        let mut cdb = [0; 10];
        cdb[0] = if write { 0x2a } else { 0x28 };
        cdb[2..6].copy_from_slice(&(lba as u32).to_be_bytes());
        cdb[7..9].copy_from_slice(&(blocks as u16).to_be_bytes());
        transport.command(lun, &cdb, data)?
    } else {
        let mut cdb = [0; 16];
        cdb[0] = if write { 0x8a } else { 0x88 };
        cdb[2..10].copy_from_slice(&lba.to_be_bytes());
        cdb[10..14].copy_from_slice(&blocks.to_be_bytes());
        transport.command(lun, &cdb, data)?
    };
    complete(count, bytes)
}

/// Commits all outstanding writes for a logical unit before returning.
fn flush_device_cache(transport: &mut Transport, lun: u8) -> Result<()> {
    let cdb = [0x35, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    transport.command(lun, &cdb, Data::None).map(|_| ())
}

/// Rejects short completions for fixed-size responses and block payloads.
fn complete(actual: usize, expected: usize) -> Result<()> {
    if actual == expected {
        Ok(())
    } else {
        Err(Error::from_status(ddk::EIO))
    }
}

static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

/// Stable callback state for a single SCSI logical unit.
struct Disk {
    /// Wire transport shared with other logical units on the interface.
    transport: Arc<Mutex<Transport>>,

    /// Logical unit address encoded in each command wrapper.
    lun: u8,

    /// Validated block count and size for this unit.
    capacity: Capacity,
}

impl Disk {
    /// Checks a request against the disk geometry and the host bulk transfer limit.
    fn request_bytes(&self, lba: u64, blocks: u32) -> Result<usize> {
        let count = (blocks as usize).checked_mul(self.capacity.block_size as usize);
        if blocks == 0
            || lba
                .checked_add(u64::from(blocks))
                .is_none_or(|end| end > self.capacity.blocks)
        {
            return Err(Error::from_status(ddk::EINVAL));
        }
        count
            .filter(|bytes| *bytes <= usb::BULK_MAX_TRANSFER)
            .ok_or(Error::from_status(ddk::EINVAL))
    }
}

/// Owns the block node receipt and keeps its callback address stable.
struct PublishedDisk {
    /// Receipt kept until checked removal succeeds.
    registration: BlockRegistration,

    /// Box passed to the block layer as callback context.
    _disk: Box<Disk>,
}

/// All live disks published by one USB interface.
struct Interface {
    /// Units still registered with the block layer.
    disks: Vec<PublishedDisk>,
}

impl Interface {
    /// Publishes one logical unit and rescans its partitions while its context is live.
    fn publish(&mut self, device: Device, disk: Disk) -> Result<()> {
        let controller = device.integer(c"usb.controller")?;
        let slot = device.integer(c"usb.slot")?;
        let interface = device.integer(c"usb.interface")?;
        let name = CString::new(format!("usb{controller}-{slot}-{interface}-{}", disk.lun))
            .map_err(|_| Error::from_status(ddk::EINVAL))?;
        let disk = Box::new(disk);
        let operations = raw::BlockOps {
            size: raw::BLOCK_OPS_SIZE,
            flags: 0,
            context: ptr::from_ref(disk.as_ref()).cast_mut().cast(),
            logical_block_size: disk.capacity.block_size,
            max_transfer_blocks: (usb::BULK_MAX_TRANSFER / disk.capacity.block_size as usize)
                as u32,
            block_count: disk.capacity.blocks,
            read: Some(read),
            write: Some(write),
            flush: Some(flush),
            discard: None,
        };

        // SAFETY: the box keeps the callback address stable through unregister.
        let registration = unsafe { BlockRegistration::register(&name, &operations) }?;
        if registration.rescan().is_err() {
            ddk::log(ddk::LOG_WARN, c"USB disk partition scan failed");
        }
        self.disks.push(PublishedDisk {
            registration,
            _disk: disk,
        });
        Ok(())
    }

    /// Withdraws every available unit, retaining busy ones for the next attempt.
    fn remove(&mut self) -> Result<()> {
        let mut result = Ok(());
        self.disks
            .retain_mut(|disk| match disk.registration.unregister() {
                Ok(()) => false,
                Err(error) => {
                    result = Err(error);
                    true
                }
            });
        result
    }
}

/// Reads blocks into the caller's geometry-validated buffer.
unsafe extern "C" fn read(context: *mut c_void, lba: u64, blocks: u32, output: *mut u8) -> i32 {
    let result = (|| {
        // SAFETY: block registration retains this Disk through all callbacks.
        let disk = unsafe { &*context.cast::<Disk>() };
        let bytes = disk.request_bytes(lba, blocks)?;

        // SAFETY: the block ABI supplies this number of writable bytes.
        let output = unsafe { ddk::output_bytes(output, bytes) }?;
        transfer_blocks(
            &mut disk.transport.lock(),
            disk.lun,
            lba,
            blocks,
            Data::In(output),
        )
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Writes blocks from the caller's geometry-validated buffer.
unsafe extern "C" fn write(context: *mut c_void, lba: u64, blocks: u32, input: *const u8) -> i32 {
    let result = (|| {
        // SAFETY: block registration retains this Disk through all callbacks.
        let disk = unsafe { &*context.cast::<Disk>() };
        let bytes = disk.request_bytes(lba, blocks)?;

        // SAFETY: the block ABI supplies this number of readable bytes.
        let input = unsafe { ddk::input_bytes(input, bytes) }?;
        transfer_blocks(
            &mut disk.transport.lock(),
            disk.lun,
            lba,
            blocks,
            Data::Out(input),
        )
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Synchronizes the disk cache in the same order as reads and writes.
unsafe extern "C" fn flush(context: *mut c_void) -> i32 {
    // SAFETY: block registration retains this Disk through all callbacks.
    let disk = unsafe { &*context.cast::<Disk>() };
    flush_device_cache(&mut disk.transport.lock(), disk.lun)
        .map_or_else(Error::status, |()| ddk::OK)
}

/// Discovers and publishes usable logical units, keeping partial success bound.
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, _: usize) -> i32 {
    let result = (|| {
        // SAFETY: probe is called with a live USB interface device.
        let device = unsafe { Device::from_raw(pointer) }.ok_or(Error::from_status(ddk::EINVAL))?;
        device.integer(c"usb.bulk-in")?;
        device.integer(c"usb.bulk-out")?;
        let transport = Transport::new(device)?;
        let max_lun = transport.max_lun()?;
        let transport = Arc::new(Mutex::new(transport));
        let mut interface = Box::new(Interface {
            disks: Vec::with_capacity(usize::from(max_lun) + 1),
        });
        // SAFETY: device data refers to this owned box. The core serializes
        // probe with removal, and block callbacks use their own Disk boxes.
        unsafe { device.set_data(ptr::from_mut(interface.as_mut()).cast()) }?;

        let mut last_failure = Error::from_status(ddk::ENODEV);
        for lun in 0..=max_lun {
            let capacity = { Capacity::read(&mut transport.lock(), lun) };
            let publish = capacity.and_then(|capacity| {
                interface.publish(
                    device,
                    Disk {
                        transport: Arc::clone(&transport),
                        lun,
                        capacity,
                    },
                )
            });
            if let Err(error) = publish {
                last_failure = error;
            }
        }

        if interface.disks.is_empty() {
            // SAFETY: no callback context was published.
            let _ = unsafe { device.set_data(ptr::null_mut()) };
            return Err(last_failure);
        }

        let _ = Box::into_raw(interface);
        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Withdraws the disk nodes and frees callback state after checked removal.
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) -> i32 {
    // SAFETY: the driver core supplies the device used for probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::OK;
    };

    // SAFETY: probe installed an Interface box for this binding.
    let interface = unsafe { device.data() }.cast::<Interface>();
    if interface.is_null() {
        return ddk::OK;
    }

    // SAFETY: removal is serialized with probe and other removal attempts.
    // Block callbacks reference only their individual Disk allocations.
    if let Err(error) = unsafe { &mut *interface }.remove() {
        return error.status();
    }

    // SAFETY: all block contexts are withdrawn and the binding owns the box.
    let _ = unsafe { device.set_data(ptr::null_mut()) };
    drop(unsafe { Box::from_raw(interface) });
    ddk::OK
}

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_ID,
    flags: 0,
    key: c"class".as_ptr(),
    value: ptr::null(),
    id0: 0x08_06_50,
    mask0: 0xff_ffff,
    id1: 0,
    mask1: 0,
    data: 0,
    score: 0,
}];

/// Registers the SCSI Bulk-Only class driver after the host has published its bus.
fn init(_: Module) -> Result<()> {
    let bus = Bus::find(c"usb").map_err(|error| {
        if error.status() == ddk::ENOENT {
            Error::from_status(ddk::EDEFERRED)
        } else {
            error
        }
    })?;
    let definition = raw::DriverDef {
        size: raw::DRIVER_DEF_SIZE,
        name: c"usb-storage".as_ptr(),
        bus: bus.as_raw(),
        priority: 0,
        matches: MATCHES.as_ptr(),
        match_count: MATCHES.len(),
        probe: Some(probe),
        remove: None,
        try_remove: Some(remove),
        shutdown: None,
        context: ptr::null_mut(),
    };

    // SAFETY: the callbacks and match table have static lifetimes.
    let registration = unsafe { DriverRegistration::register(&definition) }?;
    *REGISTRATION.lock_irqsave() = Some(registration);
    Ok(())
}

/// Releases the USB class driver registration during unload.
fn exit(_: Module) {
    let registration = { REGISTRATION.lock_irqsave().take() };
    if let Some(mut registration) = registration {
        let _ = registration.unregister();
    }
}

ddk::module!(
    b"usb-storage\0",
    b"USB SCSI Bulk-Only mass storage\0",
    init,
    exit
);
