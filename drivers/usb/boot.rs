//!
//! # USB HID Boot Driver Lifecycle
//!
//! Keyboard and mouse modules supply their own report decoders and evdev
//! capabilities. This module configures boot mode and owns report workers.
//! Each attachment claims a static evdev endpoint so open files can safely
//! outlive a disconnected device.
//!
//! Removal cancels the interrupt transfer and joins the worker before giving
//! up the endpoint claim. A future probe reclaims an offline node after its
//! last reader closes.
//!

use alloc::boxed::Box;
use core::{
    ffi::{CStr, c_void},
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};
use ddk::{Bus, Device, DriverRegistration, Error, Result, TicketLock, Worker, raw};

const MAX_REPORT: usize = 1024;

/// Static endpoint and its attachment admission bit.
pub struct Slot {
    /// Exclusively held from publication through release of the worker.
    claimed: AtomicBool,

    /// Lives as long as the module, including while disconnected readers remain.
    endpoint: evdev::Endpoint,
}

impl Slot {
    /// Creates an unclaimed USB evdev endpoint.
    pub const fn new(name: &'static CStr) -> Self {
        Self {
            claimed: AtomicBool::new(false),
            endpoint: evdev::Endpoint::new(name, 3),
        }
    }
}

/// Report decoder called with the current report and its previous boot state.
type Report = fn(&evdev::Endpoint, &mut [u8; 8], &[u8]);

/// Shared USB boot driver parameters and registration.
pub struct Driver {
    /// Pool of pinned evdev endpoints available to this class.
    slots: &'static [Slot],

    /// Shortest useful report from this boot class.
    minimum_report: usize,

    /// Publishes the evdev capabilities specific to this class.
    publish: fn(&'static evdev::Endpoint) -> Result<()>,

    /// Converts a received report into evdev events.
    report: Report,

    /// Driver receipt kept until module exit.
    registration: TicketLock<Option<DriverRegistration>>,
}

impl Driver {
    /// Creates a USB boot driver for the supplied report format and endpoint pool.
    pub const fn new(
        slots: &'static [Slot],
        minimum_report: usize,
        publish: fn(&'static evdev::Endpoint) -> Result<()>,
        report: Report,
    ) -> Self {
        Self {
            slots,
            minimum_report,
            publish,
            report,
            registration: TicketLock::new(None),
        }
    }

    /// Registers this class on the USB bus and defers until that bus is present.
    pub fn register(&'static self, name: &CStr, matches: &'static [raw::Match]) -> Result<()> {
        let bus = Bus::find(c"usb").map_err(|error| {
            if error.status() == ddk::ENOENT {
                Error::from_status(ddk::EDEFERRED)
            } else {
                error
            }
        })?;
        let definition = raw::DriverDef {
            size: raw::DRIVER_DEF_SIZE,
            name: name.as_ptr(),
            bus: bus.as_raw(),
            priority: 0,
            matches: matches.as_ptr(),
            match_count: matches.len(),
            probe: Some(probe),
            remove: Some(remove),
            try_remove: None,
            shutdown: None,
            context: ptr::from_ref(self).cast_mut().cast(),
        };

        // SAFETY: the match table, callbacks, and driver context live through unregister.
        let registration = unsafe { DriverRegistration::register(&definition) }?;
        *self.registration.lock_irqsave() = Some(registration);
        Ok(())
    }

    /// Releases the class driver's registration in thread context.
    pub fn unregister(&self) {
        let registration = { self.registration.lock_irqsave().take() };
        if let Some(mut registration) = registration {
            let _ = registration.unregister();
        }
    }

    /// Finds a free slot or skips offline nodes that still have open readers.
    fn claim_slot(&self) -> Result<&'static Slot> {
        for slot in self.slots {
            if slot
                .claimed
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
                .is_err()
            {
                continue;
            }

            match (self.publish)(&slot.endpoint) {
                Ok(()) => return Ok(slot),
                Err(error) => {
                    slot.claimed.store(false, Ordering::Release);
                    if error.status() != ddk::EBUSY {
                        return Err(error);
                    }
                }
            }
        }

        Err(Error::from_status(ddk::ENOSPC))
    }
}

/// Pinned USB binding and its interrupt report worker.
struct Attachment {
    /// USB interface used for control and interrupt calls.
    device: Device,

    /// Retains the controller service for the worker and teardown.
    host: usb::Host,

    /// Claimed endpoint for this attachment.
    slot: &'static Slot,

    /// Number of bytes to request on each interrupt transfer.
    packet: usize,

    /// Class-specific report decoder.
    report: Report,

    /// Receipt installed by probe and consumed during removal.
    worker: TicketLock<Option<Worker>>,
}

impl Attachment {
    /// Stops the worker and makes the endpoint available to later attachments.
    fn retire(&self) {
        let worker = { self.worker.lock_irqsave().take() };
        if let Some(mut worker) = worker {
            self.host.cancel(self.device);
            let _ = worker.join();
        }

        let _ = self.slot.endpoint.unpublish();
        self.slot.claimed.store(false, Ordering::Release);
    }
}

/// Reads interrupt reports until the host cancels the endpoint or reports an error.
unsafe extern "C" fn worker(context: *mut c_void) {
    // SAFETY: retirement cancels and joins the worker before freeing the allocation.
    let attachment = unsafe { &*context.cast::<Attachment>() };
    let mut previous = [0; 8];
    let mut buffer = [0; MAX_REPORT];
    while let Ok(length) = attachment
        .host
        .interrupt(attachment.device, &mut buffer[..attachment.packet])
    {
        (attachment.report)(&attachment.slot.endpoint, &mut previous, &buffer[..length]);
    }

    let _ = attachment.slot.endpoint.unpublish();
}

/// Configures boot protocol, publishes the endpoint, and starts report delivery.
unsafe extern "C" fn probe(context: *mut c_void, pointer: *const raw::Device, _: usize) -> i32 {
    let result = (|| {
        // SAFETY: registration stores its static Driver in the callback context.
        let driver = unsafe { &*context.cast::<Driver>() };

        // SAFETY: the USB bus supplies a live interface throughout probe.
        let device = unsafe { Device::from_raw(pointer) }.ok_or(Error::from_status(ddk::EINVAL))?;
        let packet = usize::try_from(device.integer(c"usb.max-packet")?)
            .map_err(|_| Error::from_status(ddk::EINVAL))?;
        if !(driver.minimum_report..=MAX_REPORT).contains(&packet) {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        let host = usb::Host::bind()?;
        let [lo, hi] = u16::try_from(device.integer(c"usb.interface")?)
            .map_err(|_| Error::from_status(ddk::EINVAL))?
            .to_le_bytes();
        host.control(device, [0x21, 0x0b, 0, 0, lo, hi, 0, 0], &mut [])?; // SET_PROTOCOL(Boot)
        let _ = host.control(device, [0x21, 0x0a, 0, 0, lo, hi, 0, 0], &mut []); // SET_IDLE

        let attachment = Box::into_raw(Box::new(Attachment {
            device,
            host,
            slot: driver.claim_slot()?,
            packet,
            report: driver.report,
            worker: TicketLock::new(None),
        }));
        let context = attachment.cast();

        // SAFETY: the probe owns the allocation until removal or rollback.
        let state = unsafe { &*attachment };
        let start = (|| {
            // SAFETY: the allocation stays pinned until retirement joins the worker.
            let worker = unsafe { Worker::spawn(Some(worker), context) }?;
            *state.worker.lock_irqsave() = Some(worker);
            // SAFETY: removal recovers this unique allocation from device data.
            unsafe { device.set_data(context) }
        })();
        if let Err(error) = start {
            state.retire();
            // SAFETY: any worker was joined and no device data was installed.
            drop(unsafe { Box::from_raw(attachment) });
            return Err(error);
        }

        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Cancels report delivery and releases the attachment's pinned context.
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the core keeps the interface live and serializes probe and remove.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };

    // SAFETY: probe stored the unique attachment pointer in this interface.
    let pointer = unsafe { device.data() }.cast::<Attachment>();
    if !pointer.is_null() {
        // SAFETY: probe transferred ownership and the core serializes removal.
        unsafe { &*pointer }.retire();
        // SAFETY: retirement has joined all users of the allocation.
        drop(unsafe { Box::from_raw(pointer) });
    }
}
