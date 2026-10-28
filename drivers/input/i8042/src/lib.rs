#![no_std]
#![no_main]
#![allow(unsafe_code)]
//! PC keyboard controller using the common evdev keyboard endpoint.

use core::{ffi::c_void, ptr};
use ddk::{Bus, Device, DriverRegistration, Error, Irq, Module, Result, TicketLock, raw};

static KEYBOARD: evdev::Keyboard = evdev::Keyboard::new(c"Roanix i8042 keyboard", 0x11);
struct State {
    extended: bool,
    pause: u8,
    irq: Option<Irq>,
    config: u8,
}
static STATE: TicketLock<State> = TicketLock::new(State {
    extended: false,
    pause: 0,
    irq: None,
    config: 0,
});
static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

fn scancode(state: &mut State, byte: u8) {
    if state.pause != 0 {
        state.pause -= 1;
        if state.pause == 0 {
            KEYBOARD.key(119, true);
            KEYBOARD.key(119, false);
        }
        return;
    }
    if byte == 0xe1 {
        state.pause = 5;
        state.extended = false;
        return;
    }
    if byte == 0xe0 {
        state.extended = true;
        return;
    }
    let scan = byte & 0x7f;
    let code = if state.extended {
        state.extended = false;
        match scan {
            0x1c => 96,
            0x1d => 97,
            0x35 => 98,
            0x37 => 99,
            0x38 => 100,
            0x47 => 102,
            0x48 => 103,
            0x49 => 104,
            0x4b => 105,
            0x4d => 106,
            0x4f => 107,
            0x50 => 108,
            0x51 => 109,
            0x52 => 110,
            0x53 => 111,
            0x5b => 125,
            0x5c => 126,
            0x5d => 127,
            _ => return,
        }
    } else {
        match scan {
            1..=83 | 86..=88 => u16::from(scan),
            84 => 99,
            _ => return,
        }
    };
    KEYBOARD.key(code, byte & 0x80 == 0);
}

unsafe extern "C" fn interrupt(_: *mut c_void, _: u32) -> u32 {
    let mut state = STATE.lock_irqsave();
    let mut handled = false;
    for _ in 0..64 {
        let status = ddk::port_read8(0x64);
        if status & 1 == 0 || status & 0x20 != 0 {
            break;
        }
        let byte = ddk::port_read8(0x60);
        handled = true;
        if status & 0xc0 == 0 {
            scancode(&mut state, byte);
        }
    }
    if handled {
        ddk::IRQ_HANDLED
    } else {
        ddk::IRQ_NONE
    }
}

fn wait_input() -> Result<()> {
    for _ in 0..100_000 {
        if ddk::port_read8(0x64) & 2 == 0 {
            return Ok(());
        }
        ddk::cpu_relax();
    }
    Err(Error::from_status(ddk::ENODEV))
}
fn command(value: u8) -> Result<()> {
    wait_input()?;
    ddk::port_write8(0x64, value);
    Ok(())
}
fn data(value: u8) -> Result<()> {
    wait_input()?;
    ddk::port_write8(0x60, value);
    Ok(())
}
fn response() -> Result<u8> {
    for _ in 0..100_000 {
        if ddk::port_read8(0x64) & 1 != 0 {
            return Ok(ddk::port_read8(0x60));
        }
        ddk::cpu_relax();
    }
    Err(Error::from_status(ddk::ENODEV))
}

fn attach(device: Device) -> Result<()> {
    if ddk::port_read8(0x64) == 0xff {
        return Err(Error::from_status(ddk::ENODEV));
    }
    command(0xad)?;
    for _ in 0..64 {
        if ddk::port_read8(0x64) & 1 == 0 {
            break;
        }
        ddk::port_read8(0x60);
    }
    command(0x20)?;
    let original = response()?;
    STATE.lock_irqsave().config = original;
    command(0x60)?;
    data((original | 0x40) & !0x11)?; // translation on; IRQ off until registered
    command(0xae)?;
    data(0xf4)?;
    if response()? != 0xfa {
        return Err(Error::from_status(ddk::ENODEV));
    }
    let virq = ddk::irq_of_device(device, 0)?;
    let irq = unsafe { Irq::request(device, virq, c"i8042", 0, interrupt, ptr::null_mut()) }?;
    STATE.lock_irqsave().irq = Some(irq);
    let mut keys = [0; evdev::KEY_BYTES];
    for code in (1..=83)
        .chain(86..=88)
        .chain(96..=111)
        .chain([119, 125, 126, 127])
    {
        keys[code / 8] |= 1 << (code % 8);
    }
    KEYBOARD.publish(keys)?;
    command(0x60)?;
    data((original | 0x41) & !0x10)?;
    Ok(())
}

fn detach() {
    let _ = command(0xad);
    let irq = { STATE.lock_irqsave().irq.take() };
    if let Some(mut irq) = irq {
        let _ = irq.release();
    }
    let _ = KEYBOARD.unpublish();
    let config = { STATE.lock_irqsave().config };
    let _ = command(0x60).and_then(|()| data(config));
    let _ = command(0xae);
}
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, _: usize) -> i32 {
    // SAFETY: the framework provides a live device throughout probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };
    match attach(device) {
        Ok(()) => ddk::OK,
        Err(error) => {
            detach();
            error.status()
        }
    }
}
unsafe extern "C" fn remove(_: *mut c_void, _: *const raw::Device) {
    detach();
}

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_COMPATIBLE,
    flags: 0,
    key: c"pnp,PNP0303".as_ptr(),
    value: ptr::null(),
    id0: 0,
    mask0: 0,
    id1: 0,
    mask1: 0,
    data: 0,
    score: 0,
}];
static DRIVER: TicketLock<raw::DriverDef> = TicketLock::new(raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"i8042".as_ptr(),
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
fn init(_: Module) -> Result<()> {
    let bus = Bus::find(c"platform")?;
    let definition = {
        let mut driver = DRIVER.lock_irqsave();
        driver.bus = bus.as_raw();
        // SAFETY: DRIVER is static, and its setup is serialized before publication.
        unsafe { &*ptr::from_ref(&*driver) }
    };
    // SAFETY: the definition and all callbacks are static until unregister.
    let registration = unsafe { DriverRegistration::register(definition) }?;
    *REGISTRATION.lock_irqsave() = Some(registration);
    Ok(())
}
fn exit(_: Module) {
    let registration = { REGISTRATION.lock_irqsave().take() };
    if let Some(mut registration) = registration {
        let _ = registration.unregister();
    }
}
ddk::module!(b"i8042\0", b"PC keyboard and evdev input\0", init, exit);
