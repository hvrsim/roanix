#![no_std]
#![no_main]
#![allow(unsafe_code, clippy::cast_possible_truncation)]
//! USB HID boot keyboards, exposed through the shared evdev implementation.
extern crate alloc;
use alloc::boxed::Box;
use core::{
    ffi::c_void,
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};
use ddk::{Bus, Device, DriverRegistration, Error, Module, Result, TicketLock, Worker, raw};

const MAX_KEYBOARDS: usize = 8;
// Static endpoints retain disconnected readers until their final close. A new
// attachment reclaims an idle endpoint; no callback points into freed storage.
static KEYBOARDS: [evdev::Keyboard; MAX_KEYBOARDS] =
    [const { evdev::Keyboard::new(c"USB HID keyboard", 3) }; MAX_KEYBOARDS];
static CLAIMED: [AtomicBool; MAX_KEYBOARDS] = [const { AtomicBool::new(false) }; MAX_KEYBOARDS];
static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);
const MODIFIERS: [u16; 8] = [29, 42, 56, 125, 97, 54, 100, 126];

fn keycode(usage: u8) -> u16 {
    match usage {
        4..=29 => [
            30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38, 50, 49, 24, 25, 16, 19, 31, 20, 22, 47,
            17, 45, 21, 44,
        ][usize::from(usage - 4)],
        30..=39 => u16::from(usage - 28),
        40..=57 => [
            28, 1, 14, 15, 57, 12, 13, 26, 27, 43, 43, 39, 40, 41, 51, 52, 53, 58,
        ][usize::from(usage - 40)],
        58..=69 => [59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 87, 88][usize::from(usage - 58)],
        70..=103 => [
            99, 70, 119, 110, 102, 104, 111, 107, 109, 106, 105, 108, 103, 69, 98, 55, 74, 78, 96,
            79, 80, 81, 75, 76, 77, 71, 72, 73, 82, 83, 86, 127, 116, 117,
        ][usize::from(usage - 70)],
        104..=115 => u16::from(usage) + 79,
        117 => 138,
        118 => 139,
        120 => 128,
        121 => 129,
        122 => 131,
        123 => 137,
        124 => 133,
        125 => 135,
        126 => 136,
        127 => 113,
        128 => 115,
        129 => 114,
        _ => 0,
    }
}
fn capabilities() -> [u8; evdev::KEY_BYTES] {
    let mut keys = [0; evdev::KEY_BYTES];
    for code in (0..=255).map(keycode).chain(MODIFIERS) {
        if code != 0 {
            keys[usize::from(code) / 8] |= 1 << (code % 8);
        }
    }
    keys
}
struct Keyboard {
    device: Device,
    host: usb::Host,
    index: usize,
    stop: AtomicBool,
    worker: TicketLock<Option<Worker>>,
}
impl Drop for Keyboard {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        self.host.cancel(self.device);
        let worker = { self.worker.lock().take() };
        if let Some(mut worker) = worker {
            let _ = worker.join();
        }
        let started = ddk::monotonic_ns();
        loop {
            match KEYBOARDS[self.index].unpublish() {
                Err(error)
                    if error.status() == ddk::EBUSY
                        && ddk::monotonic_ns().wrapping_sub(started) < 2_000_000_000 =>
                {
                    ddk::sleep_ns(1_000_000);
                }
                _ => break,
            }
        }
        // Readers that deliberately keep a disconnected fd open retain this
        // static endpoint; a later attachment reclaims it after their close.
        CLAIMED[self.index].store(false, Ordering::Release);
    }
}
fn report(keyboard: &evdev::Keyboard, previous: &mut [u8; 8], current: [u8; 8]) {
    for (bit, code) in MODIFIERS.into_iter().enumerate() {
        if (previous[0] ^ current[0]) & (1 << bit) != 0 {
            keyboard.event(1, code, i32::from(current[0] & (1 << bit) != 0));
        }
    }
    previous[0] = current[0];
    // ErrorRollOver/POSTFail/Undefined reports do not release held keys.
    if !current[2..].iter().any(|usage| (1..=3).contains(usage)) {
        for usage in &previous[2..] {
            let code = keycode(*usage);
            if code != 0 && !current[2..].contains(usage) {
                keyboard.event(1, code, 0);
            }
        }
        for (index, usage) in current[2..].iter().enumerate() {
            let code = keycode(*usage);
            if code != 0 && !previous[2..].contains(usage) && !current[2..2 + index].contains(usage)
            {
                keyboard.event(1, code, 1);
            }
        }
        *previous = current;
    }
    keyboard.event(0, 0, 0);
}
unsafe extern "C" fn worker(context: *mut c_void) {
    // SAFETY: probe pins the box; remove cancels and joins before freeing it.
    let keyboard = unsafe { &*context.cast::<Keyboard>() };
    let endpoint = &KEYBOARDS[keyboard.index];
    let mut previous = [0; 8];
    while !keyboard.stop.load(Ordering::Acquire) {
        let mut current = [0; 8];
        match keyboard.host.interrupt(keyboard.device, &mut current) {
            Ok(8) => report(endpoint, &mut previous, current),
            Ok(_) => {}
            Err(_) => break,
        }
    }
    report(endpoint, &mut previous, [0; 8]);
    let _ = endpoint.unpublish();
}
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, _: usize) -> i32 {
    let result = (|| -> Result<()> {
        // SAFETY: USB bus supplies a live interface throughout probe.
        let device = unsafe { Device::from_raw(pointer) }.ok_or(Error::from_status(ddk::EINVAL))?;
        let host = usb::Host::bind()?;
        let interface = device.integer(c"usb.interface")? as u16;
        let [lo, hi] = interface.to_le_bytes();
        host.control(device, [0x21, 0x0b, 0, 0, lo, hi, 0, 0], &mut [])?; // SET_PROTOCOL(Boot)
        let _ = host.control(device, [0x21, 0x0a, 0, 0, lo, hi, 0, 0], &mut []); // SET_IDLE
        let mut selected = None;
        for index in 0..MAX_KEYBOARDS {
            if CLAIMED[index]
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
                .is_err()
            {
                continue;
            }
            match KEYBOARDS[index].publish(capabilities()) {
                Ok(()) => {
                    selected = Some(index);
                    break;
                }
                Err(error) => {
                    CLAIMED[index].store(false, Ordering::Release);
                    if error.status() != ddk::EBUSY {
                        return Err(error);
                    }
                }
            }
        }
        let index = selected.ok_or(Error::from_status(ddk::ENOSPC))?;
        let keyboard = Box::new(Keyboard {
            device,
            host,
            index,
            stop: AtomicBool::new(false),
            worker: TicketLock::new(None),
        });
        let context = ptr::from_ref(&*keyboard).cast_mut().cast();
        // SAFETY: the box remains pinned until its worker is joined by Drop.
        let worker = unsafe { Worker::spawn(Some(worker), context) }?;
        *keyboard.worker.lock() = Some(worker);
        // SAFETY: remove recovers this unique allocation exactly once.
        unsafe {
            device.set_data(Box::into_raw(keyboard).cast())?;
        }
        ddk::log(ddk::LOG_INFO, c"USB HID keyboard attached");
        Ok(())
    })();
    result.map_or_else(Error::status, |()| ddk::OK)
}
unsafe extern "C" fn remove(_: *mut c_void, pointer: *const raw::Device) {
    // SAFETY: the core keeps the interface live throughout removal.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return;
    };
    // SAFETY: this driver owns the interface's data pointer.
    let pointer = unsafe { device.data() }.cast::<Keyboard>();
    if !pointer.is_null() {
        // SAFETY: probe transferred this unique box to the device.
        drop(unsafe { Box::from_raw(pointer) });
        // SAFETY: the consumed context must not be returned again.
        let _ = unsafe { device.set_data(ptr::null_mut()) };
    }
}
static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_ID,
    flags: 0,
    key: c"class".as_ptr(),
    value: ptr::null(),
    id0: 0x03_01_01,
    mask0: 0xff_ffff,
    id1: 0,
    mask1: 0,
    data: 0,
    score: 0,
}];
static DRIVER: TicketLock<raw::DriverDef> = TicketLock::new(raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"usb-keyboard".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: MATCHES.as_ptr(),
    match_count: MATCHES.len(),
    probe: Some(probe),
    remove: Some(remove),
    shutdown: None,
    context: ptr::null_mut(),
});
fn init(_: Module) -> Result<()> {
    let bus = Bus::find(c"usb").map_err(|error| {
        if error.status() == ddk::ENOENT {
            Error::from_status(ddk::EDEFERRED)
        } else {
            error
        }
    })?;
    let definition = {
        let mut driver = DRIVER.lock();
        driver.bus = bus.as_raw();
        // SAFETY: this static table is initialized before registration.
        unsafe { &*ptr::from_ref(&*driver) }
    };
    // SAFETY: the definition and callback code remain static until unregister.
    let registration = unsafe { DriverRegistration::register(definition) }?;
    *REGISTRATION.lock() = Some(registration);
    Ok(())
}
fn exit(_: Module) {
    let registration = { REGISTRATION.lock().take() };
    if let Some(mut registration) = registration {
        let _ = registration.unregister();
    }
}
ddk::module!(b"usb-keyboard\0", b"USB HID boot keyboard\0", init, exit);
