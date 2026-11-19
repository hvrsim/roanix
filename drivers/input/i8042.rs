//!
//! # i8042 Keyboard and Mouse
//!
//! The primary port uses translated set-1 scancodes. The auxiliary port
//! probes for a PS/2 mouse and negotiates wheel reporting when supported.
//! Both IRQs feed one drain routine because the ports share an output buffer.
//!
//! Probe and removal serialize ownership of the controller in thread context.
//! An IRQ-safe lock keeps status reads, data reads, and decoding in order
//! across cores. Probe keeps both IRQ sources disabled until the handlers
//! and endpoints are ready. Removal inhibits both ports before detaching
//! their handlers and endpoints.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code)]

use core::{ffi::c_void, ptr};
use ddk::{Bus, Device, DriverRegistration, Error, Irq, Module, Mutex, Result, TicketLock, raw};

const DATA_PORT: u16 = 0x60;
const STATUS_PORT: u16 = 0x64;
const COMMAND_PORT: u16 = 0x64;
const OUTPUT_FULL: u8 = 1 << 0;
const INPUT_FULL: u8 = 1 << 1;
const AUX_OUTPUT: u8 = 1 << 5;
const OUTPUT_ERROR: u8 = (1 << 6) | (1 << 7);

const READ_CONFIG: u8 = 0x20;
const WRITE_CONFIG: u8 = 0x60;
const DISABLE_KEYBOARD_PORT: u8 = 0xad;
const ENABLE_KEYBOARD_PORT: u8 = 0xae;
const ENABLE_SCANNING: u8 = 0xf4;
const DISABLE_AUX_PORT: u8 = 0xa7;
const ENABLE_AUX_PORT: u8 = 0xa8;
const WRITE_AUX: u8 = 0xd4;
const SET_DEFAULTS: u8 = 0xf6;
const SET_SAMPLE_RATE: u8 = 0xf3;
const IDENTIFY: u8 = 0xf2;
const ACK: u8 = 0xfa;
const RESEND: u8 = 0xfe;

const CONFIG_KEYBOARD_IRQ: u8 = 1 << 0;
const CONFIG_AUX_IRQ: u8 = 1 << 1;
const CONFIG_KEYBOARD_CLOCK_DISABLED: u8 = 1 << 4;
const CONFIG_AUX_CLOCK_DISABLED: u8 = 1 << 5;
const CONFIG_IRQS: u8 = CONFIG_KEYBOARD_IRQ | CONFIG_AUX_IRQ;
const CONFIG_TRANSLATION: u8 = 1 << 6;
const POLL_ATTEMPTS: usize = 100_000;
const DRAIN_LIMIT: usize = 64;

static MOUSE: evdev::Endpoint = evdev::Endpoint::new(c"Roanix PS/2 mouse", 0x11);
static KEYBOARD: evdev::Endpoint = evdev::Endpoint::new(c"Roanix i8042 keyboard", 0x11);

/// Prefix state for the set-1 scancode stream.
struct KeyboardDecoder {
    /// Whether the next byte belongs to the extended key table.
    extended: bool,

    /// Remaining bytes in the Pause key's multi-byte sequence.
    pause: u8,
}

impl KeyboardDecoder {
    const fn new() -> Self {
        Self {
            extended: false,
            pause: 0,
        }
    }
}

/// Packet decoder owned by the i8042 interrupt handler.
struct MouseDecoder {
    /// Bytes collected for the current packet.
    packet: [u8; 4],

    /// Number of bytes in the current packet.
    received: usize,

    /// Whether the mouse sends a fourth byte for vertical scrolling.
    wheel: bool,
}

impl MouseDecoder {
    /// Starts at a packet boundary in the negotiated reporting mode.
    const fn new(wheel: bool) -> Self {
        Self {
            packet: [0; 4],
            received: 0,
            wheel,
        }
    }

    /// Drops an incomplete packet after a controller parity or timeout error.
    fn reset(&mut self) {
        self.received = 0;
    }

    /// Collects one byte and reports each complete packet through evdev.
    fn byte(&mut self, byte: u8, endpoint: &evdev::Endpoint) {
        if self.received == 0 && byte & 0x08 == 0 {
            return;
        }
        self.packet[self.received] = byte;
        self.received += 1;
        if self.received < 3 + usize::from(self.wheel) {
            return;
        }

        self.reset();
        let [flags, x, y, wheel] = self.packet;
        let x = movement(flags, x, 0x10, 0x40);
        let y = -movement(flags, y, 0x20, 0x80);
        let wheel = if self.wheel {
            -i32::from(i8::from_ne_bytes([wheel]))
        } else {
            0
        };
        endpoint.mouse(flags & 7, x, y, wheel);
    }
}

/// Extends the ninth sign bit and discards a displacement marked as overflow.
fn movement(flags: u8, byte: u8, sign: u8, overflow: u8) -> i32 {
    if flags & overflow != 0 || byte == 0 {
        0
    } else {
        i32::from(byte) - 256 * i32::from(flags & sign != 0)
    }
}

/// Decoders for the two ports sharing the controller output buffer.
struct Input {
    /// Gates interrupt reads while probe or removal uses the shared hardware.
    active: bool,

    /// Set-1 keyboard prefix state.
    keyboard: KeyboardDecoder,

    /// Auxiliary decoder, present only after successful mouse setup.
    mouse: Option<MouseDecoder>,
}

/// Resources acquired while attaching either i8042 port.
#[derive(Default)]
struct Attachment {
    /// Configuration byte observed with both ports inhibited.
    saved_config: Option<u8>,

    /// Keyboard action released before disconnecting evdev.
    keyboard_irq: Option<Irq>,

    /// Mouse action released before disconnecting evdev.
    mouse_irq: Option<Irq>,

    /// Whether this attachment published the keyboard endpoint.
    keyboard_published: bool,

    /// Whether this attachment published the mouse endpoint.
    mouse_published: bool,
}

static INPUT: TicketLock<Input> = TicketLock::new(Input {
    active: false,
    keyboard: KeyboardDecoder::new(),
    mouse: None,
});
static ATTACHMENT: Mutex<Option<Attachment>> = Mutex::new(None);
static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

/// Decodes set-1 prefix, make, and break bytes into evdev key events.
fn scancode(state: &mut KeyboardDecoder, byte: u8) {
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

/// Drains both streams under one lock so IRQ 1 and IRQ 12 can run on different cores.
unsafe extern "C" fn interrupt(_: *mut c_void, _: u32) -> u32 {
    let mut input = INPUT.lock_irqsave();
    if !input.active {
        return ddk::IRQ_NONE;
    }

    let mut handled = false;

    for _ in 0..DRAIN_LIMIT {
        let status = ddk::port_read8(STATUS_PORT);
        if status & OUTPUT_FULL == 0 {
            break;
        }

        let byte = ddk::port_read8(DATA_PORT);
        handled = true;
        let valid = status & OUTPUT_ERROR == 0;
        if status & AUX_OUTPUT != 0 {
            if let Some(mouse) = &mut input.mouse {
                if valid {
                    mouse.byte(byte, &MOUSE);
                } else {
                    mouse.reset();
                }
            }
        } else if valid {
            scancode(&mut input.keyboard, byte);
        } else {
            input.keyboard = KeyboardDecoder::new();
        }
    }

    if handled {
        ddk::IRQ_HANDLED | ddk::IRQ_RESCHEDULE
    } else {
        ddk::IRQ_NONE
    }
}

/// Waits until the selected controller status bits match the expected value.
fn wait_status(mask: u8, expected: u8) -> Result<()> {
    for _ in 0..POLL_ATTEMPTS {
        if ddk::port_read8(STATUS_PORT) & mask == expected {
            return Ok(());
        }
        ddk::cpu_relax();
    }

    Err(Error::from_status(ddk::ENODEV))
}

/// Writes a command or data byte once the controller can accept it.
fn write_port(port: u16, value: u8) -> Result<()> {
    wait_status(INPUT_FULL, 0)?;
    ddk::port_write8(port, value);
    Ok(())
}

/// Waits for a valid response from the selected port while interrupts are disabled.
fn response(aux: bool) -> Result<u8> {
    for _ in 0..POLL_ATTEMPTS {
        let status = ddk::port_read8(STATUS_PORT);
        if status & OUTPUT_FULL != 0 {
            let byte = ddk::port_read8(DATA_PORT);
            if status & OUTPUT_ERROR == 0 && (status & AUX_OUTPUT != 0) == aux {
                return Ok(byte);
            }
        }
        ddk::cpu_relax();
    }

    Err(Error::from_status(ddk::ENODEV))
}

/// Sends the write-configuration command followed by the new configuration byte.
fn write_config(value: u8) -> Result<()> {
    write_port(COMMAND_PORT, WRITE_CONFIG)?;
    write_port(DATA_PORT, value)
}

/// Sends a mouse command or argument and retries a requested resend.
fn aux_command(value: u8) -> Result<()> {
    for _ in 0..3 {
        write_port(COMMAND_PORT, WRITE_AUX)?;
        write_port(DATA_PORT, value)?;
        match response(true)? {
            ACK => return Ok(()),
            RESEND => {}
            _ => break,
        }
    }

    Err(Error::from_status(ddk::ENODEV))
}

/// Attempts `IntelliMouse` wheel mode and restores standard packets on failure.
fn mouse_mode() -> Result<bool> {
    aux_command(SET_DEFAULTS)?;
    let wheel = (|| -> Result<bool> {
        for rate in [200, 100, 80] {
            aux_command(SET_SAMPLE_RATE)?;
            aux_command(rate)?;
        }
        aux_command(IDENTIFY)?;
        Ok(response(true)? == 3)
    })();
    if let Ok(true) = wheel {
        return Ok(true);
    }
    aux_command(SET_DEFAULTS)?;
    Ok(false)
}

/// Configures both ports before enabling either interrupt source.
fn attach(device: Device, slot: &mut Option<Attachment>) -> Result<()> {
    if ddk::port_read8(STATUS_PORT) == 0xff {
        return Err(Error::from_status(ddk::ENODEV));
    }

    let attachment = slot.insert(Attachment::default());
    write_port(COMMAND_PORT, DISABLE_KEYBOARD_PORT)?;
    write_port(COMMAND_PORT, DISABLE_AUX_PORT)?;
    for _ in 0..DRAIN_LIMIT {
        if ddk::port_read8(STATUS_PORT) & OUTPUT_FULL == 0 {
            break;
        }
        ddk::port_read8(DATA_PORT);
    }

    write_port(COMMAND_PORT, READ_CONFIG)?;
    let original = response(false)?;
    attachment.saved_config = Some(original);
    let configured = (original | CONFIG_TRANSLATION | CONFIG_AUX_CLOCK_DISABLED) & !CONFIG_IRQS;
    write_config(configured)?;

    write_port(COMMAND_PORT, ENABLE_KEYBOARD_PORT)?;
    write_port(DATA_PORT, ENABLE_SCANNING)?;
    if response(false)? != ACK {
        return Err(Error::from_status(ddk::ENODEV));
    }
    *INPUT.lock_irqsave() = Input {
        active: false,
        keyboard: KeyboardDecoder::new(),
        mouse: None,
    };

    let virq = ddk::irq_of_device(device, 0)?;

    // SAFETY: both actions access only the controller's static state.
    attachment.keyboard_irq = Some(unsafe {
        Irq::request(
            device,
            virq,
            c"i8042-keyboard",
            0,
            interrupt,
            ptr::null_mut(),
        )
    }?);

    let mut keys = [0; evdev::KEY_BYTES];
    for code in (1..=83)
        .chain(86..=88)
        .chain(96..=111)
        .chain([119, 125, 126, 127])
    {
        keys[code / 8] |= 1 << (code % 8);
    }
    KEYBOARD.publish(keys)?;
    attachment.keyboard_published = true;

    let mut active = (configured & !CONFIG_KEYBOARD_CLOCK_DISABLED) | CONFIG_KEYBOARD_IRQ;
    if let Ok(virq) = ddk::irq_of_device(device, 1) {
        write_port(COMMAND_PORT, DISABLE_KEYBOARD_PORT)?;
        if attach_mouse(device, virq, attachment).is_ok() {
            active = (active & !CONFIG_AUX_CLOCK_DISABLED) | CONFIG_AUX_IRQ;
        } else {
            write_port(COMMAND_PORT, DISABLE_AUX_PORT)?;
            detach_mouse(attachment);
        }
    }

    INPUT.lock_irqsave().active = true;
    write_config(active)
}

/// Probes and publishes the optional mouse while the keyboard port is inhibited.
fn attach_mouse(device: Device, virq: u32, attachment: &mut Attachment) -> Result<()> {
    write_port(COMMAND_PORT, ENABLE_AUX_PORT)?;
    let wheel = mouse_mode()?;

    // SAFETY: the handler uses static decoder and evdev state.
    attachment.mouse_irq =
        Some(unsafe { Irq::request(device, virq, c"i8042-mouse", 0, interrupt, ptr::null_mut()) }?);
    MOUSE.publish_mouse(wheel)?;
    attachment.mouse_published = true;
    INPUT.lock_irqsave().mouse = Some(MouseDecoder::new(wheel));
    aux_command(ENABLE_SCANNING)
}

/// Releases the optional auxiliary action before retiring its endpoint.
fn detach_mouse(attachment: &mut Attachment) {
    if let Some(mut irq) = attachment.mouse_irq.take() {
        let _ = irq.release();
    }
    INPUT.lock_irqsave().mouse = None;
    if core::mem::take(&mut attachment.mouse_published) {
        let _ = MOUSE.unpublish();
    }
}

/// Inhibits the ports, drains both IRQ actions, and restores firmware configuration.
fn detach(mut attachment: Attachment) {
    INPUT.lock_irqsave().active = false;
    let _ = write_port(COMMAND_PORT, DISABLE_KEYBOARD_PORT);
    let _ = write_port(COMMAND_PORT, DISABLE_AUX_PORT);

    if let Some(mut irq) = attachment.keyboard_irq.take() {
        let _ = irq.release();
    }
    detach_mouse(&mut attachment);
    if attachment.keyboard_published {
        let _ = KEYBOARD.unpublish();
    }

    if let Some(config) = attachment.saved_config {
        let _ = write_config(config & !CONFIG_IRQS);
    }

    let _ = write_port(COMMAND_PORT, ENABLE_KEYBOARD_PORT);
}

/// Attaches the controller described by this platform device.
unsafe extern "C" fn probe(_: *mut c_void, pointer: *const raw::Device, _: usize) -> i32 {
    // SAFETY: the framework provides a live device throughout probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };

    let mut slot = ATTACHMENT.lock();
    if slot.is_some() {
        return ddk::EBUSY;
    }

    match attach(device, &mut slot) {
        Ok(()) => ddk::OK,
        Err(error) => {
            if let Some(attachment) = slot.take() {
                detach(attachment);
            }
            error.status()
        }
    }
}

/// Disconnects the i8042 controller from the platform device.
unsafe extern "C" fn remove(_: *mut c_void, _: *const raw::Device) {
    let mut slot = ATTACHMENT.lock();
    if let Some(attachment) = slot.take() {
        detach(attachment);
    }
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

const DRIVER: raw::DriverDef = raw::DriverDef {
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
};

/// Registers the i8042 driver and its platform match table.
fn init(_: Module) -> Result<()> {
    let bus = Bus::find(c"platform")?;
    let definition = raw::DriverDef {
        bus: bus.as_raw(),
        ..DRIVER
    };

    // SAFETY: the match table and callbacks have static lifetimes.
    let registration = unsafe { DriverRegistration::register(&definition) }?;
    *REGISTRATION.lock_irqsave() = Some(registration);
    Ok(())
}

/// Unregisters the platform driver and releases any remaining input resources.
fn exit(_: Module) {
    let registration = { REGISTRATION.lock_irqsave().take() };
    if let Some(mut registration) = registration {
        let _ = registration.unregister();
    }
}

ddk::module!(b"i8042\0", b"PC keyboard and PS/2 mouse\0", init, exit);
