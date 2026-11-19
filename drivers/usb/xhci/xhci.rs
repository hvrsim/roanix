//!
//! # xHCI USB Host Driver
//!
//! This module binds PCI xHCI controllers, publishes the USB host service,
//! and exposes discovered USB interfaces to class drivers. x86 uses MSI or
//! MSI-X, while RISC-V uses the firmware-routed PCI interrupt.
//!
//! The IRQ handler completes commands and transfers through controller
//! events. A separate worker handles port changes. Registry lookups retain
//! a controller for each service call without holding the registry lock
//! during transfers or teardown.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code, clippy::cast_possible_truncation, clippy::cast_sign_loss)]
extern crate alloc;
mod controller;
mod pci;
mod port;
mod ring;

use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use controller::Controller;
use core::{
    ffi::c_void,
    mem::size_of,
    ptr,
    sync::atomic::{AtomicU32, Ordering},
};
use ddk::{
    Bus, BusRegistration, Device, DriverRegistration, Error, InterfacePublication, Irq, Module,
    Result, TicketLock, Worker, raw,
};
use port::{EndpointDescriptor, Payload};

static CONTROLLERS: ddk::Mutex<Vec<(u32, Weak<Controller>)>> = ddk::Mutex::new(Vec::new());
static NEXT_ID: AtomicU32 = AtomicU32::new(0);
static DRIVER_RECEIPT: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);
static BUS_RECEIPT: TicketLock<Option<BusRegistration>> = TicketLock::new(None);
static BULK_PUBLICATION: TicketLock<Option<InterfacePublication>> = TicketLock::new(None);
static PUBLICATION: TicketLock<Option<InterfacePublication>> = TicketLock::new(None);

/// Removes the controller from the registry without dropping it under the lock.
fn forget_controller(id: u32) {
    let entry = {
        let mut controllers = CONTROLLERS.lock();
        controllers
            .iter()
            .position(|(controller_id, _)| *controller_id == id)
            .map(|index| controllers.swap_remove(index))
    };
    // A failed probe may leave only this weak reference to the allocation.
    drop(entry);
}

/// Drains event TRBs and signals their command, endpoint, or port waiters.
unsafe extern "C" fn interrupt(context: *mut c_void, _: u32) -> u32 {
    // SAFETY: the controller is Arc-pinned until the IRQ receipt is released.
    let controller = unsafe { &*context.cast::<Controller>() };
    let mut h = controller.hardware.lock_irqsave();
    if h.registers.read32(h.op + 4) & (1 << 3) == 0 {
        return ddk::IRQ_NONE;
    }
    h.registers.write32(h.op + 4, 1 << 3);
    for _ in 0..ring::ENTRIES {
        let Some(event) = h.events.pop() else {
            break;
        };

        match event.kind() {
            33 if event.parameter == h.command_address => {
                h.command_result = Some(event);
                let _ = controller.command_event.signal();
            }
            32 => {
                let slot = (event.control >> 24) as usize;
                let dci = ((event.control >> 16) & 31) as usize;
                if let Some(endpoint) = h
                    .slots
                    .get_mut(slot)
                    .and_then(Option::as_mut)
                    .and_then(|slot| slot.endpoints.get_mut(dci))
                    .and_then(Option::as_mut)
                {
                    endpoint.complete(event, dci == 1);
                }
            }
            34 => {
                let port = (event.parameter >> 24) as usize;
                if port != 0 && port <= h.max_ports {
                    let offset = h.portsc(port - 1);
                    let value = h.registers.read32(offset);
                    if value & 1 == 0 {
                        let slot = usize::from(h.ports[port - 1]);
                        if let Some(state) = &mut h.slots[slot] {
                            state.retire();
                        }
                    }
                    h.registers
                        .write32(offset, value & ((1 << 9) | 0x00fe_0000));
                }

                let _ = controller.port_event.signal();
            }
            37 => {
                controller.failed.store(true, Ordering::Release);
                let _ = controller.command_event.signal();
            }
            _ => {}
        }
    }
    ring::barrier();
    h.registers
        .write64(h.runtime + 0x18, h.events.dequeue() | 8);
    h.registers.write32(h.runtime, 3);
    ddk::IRQ_HANDLED | ddk::IRQ_RESCHEDULE
}

/// Scans ports and waits for further port-change notifications.
unsafe extern "C" fn worker(context: *mut c_void) {
    // SAFETY: the device-owned Arc is retained until this worker is joined.
    let controller = unsafe { &*context.cast::<Controller>() };
    while !controller.stopping.load(Ordering::Acquire) {
        let _ = controller.port_event.reset();
        controller.scan_ports();
        let _ = controller.port_event.wait_timeout(250_000_000);
    }
}

/// Resolves a USB child to its retained controller, device handle, and slot.
fn instance(pointer: *const raw::Device) -> Result<(Arc<Controller>, Device, usize)> {
    // SAFETY: host operations are called with a live USB child device.
    let device = unsafe { Device::from_raw(pointer) }.ok_or(Error::from_status(ddk::EINVAL))?;
    let id = device.integer(c"usb.controller")? as u32;
    let slot = device.integer(c"usb.slot")? as usize;
    let weak = {
        let controllers = CONTROLLERS.lock();
        controllers
            .iter()
            .find(|(controller_id, _)| *controller_id == id)
            .map(|(_, controller)| Weak::clone(controller))
    };
    // The last owning reference may run sleeping teardown when dropped.
    let controller = weak
        .and_then(|controller| controller.upgrade())
        .ok_or(Error::from_status(ddk::ENODEV))?;
    Ok((controller, device, slot))
}

/// Validates the host callback buffer and submits the device's control request.
unsafe extern "C" fn control(
    _: *mut c_void,
    device: *const raw::Device,
    setup: u64,
    buffer: *mut u8,
    length: usize,
) -> i64 {
    let result = (|| -> Result<usize> {
        let (controller, _, slot) = instance(device)?;
        let _guard = controller.transaction.lock()?;

        // SAFETY: USB ABI caller owns length writable bytes for this call.
        let buffer = unsafe { ddk::output_bytes(buffer, length) }?;
        controller.control(slot, setup, buffer)
    })();
    result.map_or_else(|e| i64::from(e.status()), |count| i64::from(count as u32))
}

/// Validates the host callback buffer and receives data from the device's interrupt endpoint.
unsafe extern "C" fn receive(
    _: *mut c_void,
    device: *const raw::Device,
    buffer: *mut u8,
    length: usize,
) -> i64 {
    let result = (|| -> Result<usize> {
        if length == 0 || length > 4096 {
            return Err(Error::from_status(ddk::EINVAL));
        }

        let (controller, device, slot) = instance(device)?;
        let endpoint = EndpointDescriptor {
            address: device.integer(c"usb.endpoint")? as u8,
            packet: device.integer(c"usb.max-packet")? as u16,
            interval: device.integer(c"usb.interval")? as u8,
            burst: 0,
        };

        // SAFETY: USB ABI caller owns length writable bytes for this call.
        let buffer = unsafe { ddk::output_bytes(buffer, length) }?;
        controller.transfer(slot, endpoint, false, Payload::In(buffer))
    })();
    result.map_or_else(|e| i64::from(e.status()), |count| i64::from(count as u32))
}

/// Reads the selected bulk endpoint descriptor from the USB interface.
fn bulk_endpoint(device: Device, input: bool) -> Result<EndpointDescriptor> {
    let key = if input {
        c"usb.bulk-in"
    } else {
        c"usb.bulk-out"
    };

    let bytes = (device.integer(key)? as u32).to_le_bytes();
    Ok(EndpointDescriptor {
        address: bytes[0],
        packet: u16::from_le_bytes([bytes[1], bytes[2]]),
        burst: bytes[3],
        interval: 0,
    })
}

/// Receives a bulk IN transfer after checking the ABI buffer.
unsafe extern "C" fn bulk_read(
    _: *mut c_void,
    pointer: *const raw::Device,
    buffer: *mut u8,
    length: usize,
) -> i64 {
    let result = (|| {
        let (controller, device, slot) = instance(pointer)?;
        let endpoint = bulk_endpoint(device, true)?;

        // SAFETY: the caller supplies length writable bytes for this transfer.
        let buffer = unsafe { ddk::output_bytes(buffer, length) }?;
        controller.transfer(slot, endpoint, true, Payload::In(buffer))
    })();
    result.map_or_else(|e| i64::from(e.status()), |count| i64::from(count as u32))
}

/// Sends a bulk OUT transfer after checking the ABI buffer.
unsafe extern "C" fn bulk_write(
    _: *mut c_void,
    pointer: *const raw::Device,
    buffer: *const u8,
    length: usize,
) -> i64 {
    let result = (|| {
        let (controller, device, slot) = instance(pointer)?;
        let endpoint = bulk_endpoint(device, false)?;

        // SAFETY: the caller supplies length readable bytes for this transfer.
        let buffer = unsafe { ddk::input_bytes(buffer, length) }?;
        controller.transfer(slot, endpoint, true, Payload::Out(buffer))
    })();
    result.map_or_else(|e| i64::from(e.status()), |count| i64::from(count as u32))
}

/// Recovers the selected bulk endpoint after a USB halt.
unsafe extern "C" fn clear_halt(_: *mut c_void, pointer: *const raw::Device, input: bool) -> i32 {
    let result = (|| {
        let (controller, device, slot) = instance(pointer)?;
        controller.clear_bulk_halt(slot, bulk_endpoint(device, input)?)
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Cancels outstanding transfers for the USB child.
unsafe extern "C" fn cancel(_: *mut c_void, device: *const raw::Device) {
    let Ok((controller, device, slot)) = instance(device) else {
        return;
    };

    let interrupt = device
        .integer(c"usb.endpoint")
        .ok()
        .map(|address| EndpointDescriptor {
            address: address as u8,
            ..EndpointDescriptor::default()
        });
    let endpoints = [
        interrupt,
        bulk_endpoint(device, true).ok(),
        bulk_endpoint(device, false).ok(),
    ];
    let mut h = controller.hardware.lock_irqsave();
    if let Some(Some(state)) = h.slots.get_mut(slot) {
        for descriptor in endpoints.into_iter().flatten() {
            let dci = descriptor.dci();
            state.cancelled |= 1 << dci;
            if let Some(endpoint) = &mut state.endpoints[dci] {
                endpoint.cancelled = true;
                let _ = endpoint.event.signal();
            }
        }
    }
}

static BULK_OPS: usb::BulkOps = usb::BulkOps {
    size: size_of::<usb::BulkOps>() as u32,
    read: bulk_read,
    write: bulk_write,
    clear_halt,
};

static HOST_OPS: usb::HostOps = usb::HostOps {
    size: size_of::<usb::HostOps>() as u32,
    control,
    interrupt: receive,
    cancel,
};

/// Registers an xHCI controller, installs its interrupt, and starts the port worker.
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, _: usize) -> i32 {
    let result = (|| -> Result<()> {
        // SAFETY: PCI supplies a live function throughout probe.
        let device = unsafe { Device::from_raw(pointer) }.ok_or(Error::from_status(ddk::EINVAL))?;
        let controller = Controller::new(device, NEXT_ID.fetch_add(1, Ordering::Relaxed))?;
        #[cfg(target_arch = "x86_64")]
        let virq = controller.msi.virq;
        #[cfg(target_arch = "riscv64")]
        let virq = ddk::irq_of_device(device, 0)?;
        let context = Arc::as_ptr(&controller).cast_mut().cast();
        let irq = unsafe {
            Irq::request(
                device,
                virq,
                c"xhci",
                if cfg!(target_arch = "riscv64") {
                    ddk::IRQ_SHARED
                } else {
                    0
                },
                interrupt,
                context,
            )
        }?;
        *controller.irq.lock_irqsave() = Some(irq);
        #[cfg(target_arch = "x86_64")]
        controller.msi.unmask()?;

        // SAFETY: the probe owns this Arc and retains it until removal takes over.
        // Install the context before a worker can publish children.
        unsafe { device.set_data(context) }?;
        CONTROLLERS
            .lock()
            .push((controller.id, Arc::downgrade(&controller)));
        controller.hardware.lock_irqsave().start();

        // SAFETY: the device-owned Arc remains pinned until worker join.
        let worker = match unsafe { Worker::spawn(Some(worker), context) } {
            Ok(worker) => worker,
            Err(error) => {
                forget_controller(controller.id);
                // SAFETY: probe still owns the only controller context.
                let _ = unsafe { device.set_data(ptr::null_mut()) };
                return Err(error);
            }
        };
        *controller.worker.lock_irqsave() = Some(worker);

        // Remove recovers this reference exactly once after successful transfer.
        let _ = Arc::into_raw(controller);
        ddk::log(ddk::LOG_INFO, c"xHCI controller started");
        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

/// Stops port work and withdraws USB children before releasing the controller.
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) -> i32 {
    let result = (|| {
        // SAFETY: the core keeps the PCI function live throughout removal.
        let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
            return Ok(());
        };

        // SAFETY: this driver owns the function's data pointer.
        let pointer = unsafe { device.data() }.cast::<Controller>();
        if pointer.is_null() {
            return Ok(());
        }

        // SAFETY: removal is serialized and the binding retains its owning Arc.
        let controller = unsafe { &*pointer };
        controller.stopping.store(true, Ordering::Release);
        controller.retire_slots();
        let _ = controller.port_event.signal();
        let worker = { controller.worker.lock_irqsave().take() };
        if let Some(mut worker) = worker {
            worker.join()?;
        }

        let count = { controller.hardware.lock_irqsave().max_ports };
        for port in 0..count {
            let slot = { controller.hardware.lock_irqsave().ports[port] };
            if slot != 0 {
                controller.disconnect(port, usize::from(slot))?;
            }
        }

        // SAFETY: all child registrations and the worker are gone.
        unsafe { device.set_data(ptr::null_mut()) }?;
        forget_controller(controller.id);
        // SAFETY: this consumes the unique owning Arc installed at probe.
        drop(unsafe { Arc::from_raw(pointer) });
        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_ID,
    flags: 0,
    key: ptr::null(),
    value: ptr::null(),
    id0: 0,
    mask0: 0,
    id1: 0x0c_03_30,
    mask1: 0xff_ffff,
    data: 0,
    score: 0,
}];
const DRIVER: raw::DriverDef = raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"usb-xhci".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: None,
    shutdown: None,
    context: ptr::null_mut(),
    try_remove: Some(remove),
};

static BUS: raw::BusDef = raw::BusDef {
    size: size_of::<raw::BusDef>() as u32,
    match_device: None,
    prepare: None,
    cleanup: None,
    shutdown: None,
    context: ptr::null_mut(),
};

/// Registers the USB bus, host service, and xHCI PCI driver.
fn init(_: Module) -> Result<()> {
    // SAFETY: BUS, HOST_OPS, and their callback targets are static.
    let bus = unsafe { BusRegistration::register(c"usb", &BUS) }?;
    *BUS_RECEIPT.lock_irqsave() = Some(bus);
    *PUBLICATION.lock_irqsave() = Some(unsafe {
        InterfacePublication::publish(
            usb::HOST_INTERFACE,
            1,
            ddk::IFACE_MAY_SLEEP | ddk::IFACE_CONCURRENT | ddk::IFACE_SINGLETON,
            ptr::from_ref(&HOST_OPS).cast(),
            size_of::<usb::HostOps>(),
            ptr::null_mut(),
        )
    }?);
    // SAFETY: BULK_OPS is static and its callbacks are retained by the service binding.
    *BULK_PUBLICATION.lock_irqsave() = Some(unsafe {
        InterfacePublication::publish(
            usb::BULK_INTERFACE,
            1,
            ddk::IFACE_MAY_SLEEP | ddk::IFACE_CONCURRENT | ddk::IFACE_SINGLETON,
            ptr::from_ref(&BULK_OPS).cast(),
            size_of::<usb::BulkOps>(),
            ptr::null_mut(),
        )
    }?);
    let pci_bus = Bus::find(c"pci")?;
    let definition = raw::DriverDef {
        bus: pci_bus.as_raw(),
        ..DRIVER
    };

    // SAFETY: the match table and callbacks have static lifetimes.
    let registration = unsafe { DriverRegistration::register(&definition) }?;
    *DRIVER_RECEIPT.lock_irqsave() = Some(registration);
    Ok(())
}

/// Withdraws the host service and unregisters the driver and USB bus.
fn exit(_: Module) {
    let driver = { DRIVER_RECEIPT.lock_irqsave().take() };
    if let Some(mut driver) = driver {
        let _ = driver.unregister();
    }

    for receipt in [&BULK_PUBLICATION, &PUBLICATION] {
        let publication = { receipt.lock_irqsave().take() };
        if let Some(mut publication) = publication {
            let _ = publication.withdraw();
        }
    }

    let bus = { BUS_RECEIPT.lock_irqsave().take() };
    if let Some(mut bus) = bus {
        let _ = bus.unregister();
    }
}
ddk::module!(b"usb-xhci\0", b"xHCI USB host controller\0", init, exit);
