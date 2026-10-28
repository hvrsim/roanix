#![no_std]
#![no_main]
#![allow(unsafe_code, clippy::cast_possible_truncation, clippy::cast_sign_loss)]
//! xHCI host controller and root-port USB enumeration.
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
    ptr, slice,
    sync::atomic::{AtomicU32, Ordering},
};
use ddk::{
    Bus, BusRegistration, Device, DriverRegistration, Error, InterfacePublication, Irq, Module,
    Result, TicketLock, Worker, raw,
};

static CONTROLLERS: ddk::Mutex<Vec<(u32, Weak<Controller>)>> = ddk::Mutex::new(Vec::new());
static NEXT_ID: AtomicU32 = AtomicU32::new(0);
static DRIVER_RECEIPT: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);
static BUS_RECEIPT: TicketLock<Option<BusRegistration>> = TicketLock::new(None);
static PUBLICATION: TicketLock<Option<InterfacePublication>> = TicketLock::new(None);

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
                    let code = event.status >> 24;
                    if endpoint.pending != 0 {
                        if code == 13 {
                            endpoint.received = endpoint
                                .requested
                                .saturating_sub((event.status & 0xff_ffff) as usize);
                        }
                        if event.parameter & !15 == endpoint.pending || !matches!(code, 1 | 13) {
                            endpoint.code = code;
                            endpoint.complete = true;
                            if code == 1 && dci != 1 {
                                endpoint.received = endpoint
                                    .requested
                                    .saturating_sub((event.status & 0xff_ffff) as usize);
                            }
                            let _ = endpoint.event.signal();
                        }
                    }
                }
            }
            34 => {
                let port = (event.parameter >> 24) as usize;
                if port != 0 && port <= h.max_ports {
                    let offset = h.portsc(port - 1);
                    let value = h.registers.read32(offset);
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
unsafe extern "C" fn worker(context: *mut c_void) {
    // SAFETY: the device-owned Arc is retained until this worker is joined.
    let controller = unsafe { &*context.cast::<Controller>() };
    while !controller.stopping.load(Ordering::Acquire) {
        let _ = controller.port_event.reset();
        controller.scan_ports();
        let _ = controller.port_event.wait_timeout(250_000_000);
    }
}
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
        controller.control(slot, setup, unsafe {
            slice::from_raw_parts_mut(buffer, length)
        })
    })();
    result.map_or_else(|e| i64::from(e.status()), |count| i64::from(count as u32))
}
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
        let endpoint = device.integer(c"usb.endpoint")? as u8;
        let packet = device.integer(c"usb.max-packet")? as u16;
        let interval = device.integer(c"usb.interval")? as u8;
        // SAFETY: USB ABI caller owns length writable bytes for this call.
        controller.receive_interrupt(slot, endpoint, packet, interval, unsafe {
            slice::from_raw_parts_mut(buffer, length)
        })
    })();
    result.map_or_else(|e| i64::from(e.status()), |count| i64::from(count as u32))
}
unsafe extern "C" fn cancel(_: *mut c_void, device: *const raw::Device) {
    if let Ok((controller, device, slot)) = instance(device) {
        if let Ok(endpoint) = device.integer(c"usb.endpoint") {
            let dci = ((endpoint as usize & 15) * 2) + 1;
            let mut h = controller.hardware.lock_irqsave();
            if let Some(Some(state)) = h.slots.get_mut(slot) {
                state.cancelled |= 1 << dci;
            }
            if let Ok(endpoint) = h.endpoint_mut(slot, dci) {
                endpoint.cancelled = true;
                let _ = endpoint.event.signal();
            }
        }
    }
}
static HOST_OPS: usb::HostOps = usb::HostOps {
    size: size_of::<usb::HostOps>() as u32,
    control,
    interrupt: receive,
    cancel,
};

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
        CONTROLLERS
            .lock()
            .push((controller.id, Arc::downgrade(&controller)));
        {
            controller.hardware.lock_irqsave().start();
        }
        // SAFETY: the device-owned Arc remains pinned until worker join.
        let worker = match unsafe { Worker::spawn(Some(worker), context) } {
            Ok(worker) => worker,
            Err(error) => {
                forget_controller(controller.id);
                return Err(error);
            }
        };
        *controller.worker.lock_irqsave() = Some(worker);
        // SAFETY: retain ownership until the kernel accepts the live identity.
        if let Err(error) = unsafe { device.set_data(context) } {
            forget_controller(controller.id);
            return Err(error);
        }
        // Remove recovers this reference exactly once after successful transfer.
        let _ = Arc::into_raw(controller);
        ddk::log(ddk::LOG_INFO, c"xHCI controller started");
        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the core keeps the PCI function live throughout removal.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };
    // SAFETY: this driver owns the function's data pointer.
    let pointer = unsafe { device.data() }.cast::<Controller>();
    if pointer.is_null() {
        return;
    }
    // SAFETY: probe transferred this unique owning reference to device data.
    let controller = unsafe { Arc::from_raw(pointer) };
    controller.stopping.store(true, Ordering::Release);
    let _ = controller.port_event.signal();
    let worker = { controller.worker.lock_irqsave().take() };
    if let Some(mut worker) = worker {
        let _ = worker.join();
    }
    let count = { controller.hardware.lock_irqsave().max_ports };
    for port in 0..count {
        let slot = { controller.hardware.lock_irqsave().ports[port] };
        if slot != 0 {
            controller.disconnect(port, usize::from(slot));
        }
    }
    // SAFETY: no future callback may acquire this consumed device reference.
    let _ = unsafe { device.set_data(ptr::null_mut()) };
    forget_controller(controller.id);
    drop(controller);
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
static DRIVER: TicketLock<raw::DriverDef> = TicketLock::new(raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"usb-xhci".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: Some(remove),
    shutdown: None,
    context: ptr::null_mut(),
    try_remove: None,
});
static BUS: TicketLock<raw::BusDef> = TicketLock::new(raw::BusDef {
    size: size_of::<raw::BusDef>() as u32,
    match_device: None,
    prepare: None,
    cleanup: None,
    shutdown: None,
    context: ptr::null_mut(),
});
fn init(_: Module) -> Result<()> {
    // SAFETY: BUS, HOST_OPS, and their callback targets are static.
    let bus_definition = {
        let bus = BUS.lock_irqsave();
        // SAFETY: the bus definition is immutable static storage.
        unsafe { &*ptr::from_ref(&*bus) }
    };
    let bus = unsafe { BusRegistration::register(c"usb", bus_definition) }?;
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
    let pci_bus = Bus::find(c"pci")?;
    let definition = {
        let mut driver = DRIVER.lock_irqsave();
        driver.bus = pci_bus.as_raw();
        // SAFETY: publication serializes initialization of this static table.
        unsafe { &*ptr::from_ref(&*driver) }
    };
    // SAFETY: the driver definition remains static until unregister completes.
    let registration = unsafe { DriverRegistration::register(definition) }?;
    *DRIVER_RECEIPT.lock_irqsave() = Some(registration);
    Ok(())
}
fn exit(_: Module) {
    let driver = { DRIVER_RECEIPT.lock_irqsave().take() };
    if let Some(mut driver) = driver {
        let _ = driver.unregister();
    }
    let publication = { PUBLICATION.lock_irqsave().take() };
    if let Some(mut publication) = publication {
        let _ = publication.withdraw();
    }
    let bus = { BUS_RECEIPT.lock_irqsave().take() };
    if let Some(mut bus) = bus {
        let _ = bus.unregister();
    }
}
ddk::module!(b"usb-xhci\0", b"xHCI USB host controller\0", init, exit);
