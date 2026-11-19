//!
//! # I/O APIC Interrupt Controller
//!
//! This driver maps platform I/O APICs and routes global system interrupts to
//! x86 interrupt vectors. Each mapping starts masked until a consumer is
//! ready to receive interrupts.
//!
//! The IRQ-safe controller lock serializes the shared register selector and
//! window. Removal refuses to unmap a chip while it owns active routes.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code)]
// Values crossing the module boundary carry the C ABI widths fixed by
// include/roanix/api.h, and both supported targets use 64-bit pointers, so
// casts between them cannot lose information in practice. Large arrays appear
// only inside `const fn` constructors evaluated for statics, never on the
// stack at runtime.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays
)]

use core::{ffi::c_void, ptr};

use ddk::{
    Bus, Device, DriverRegistration, IrqDomain, Mmio, Module, RESOURCE_MEMORY, Result, TicketLock,
    raw,
};

const REGISTER_SELECT: usize = 0x00;
const REGISTER_WINDOW: usize = 0x10;
const REG_VERSION: u8 = 0x01;
const REDIRECTION_BASE: u8 = 0x10;
const REDIRECTION_MASKED: u32 = 1 << 16;
const REDIRECTION_TRIGGER_LEVEL: u32 = 1 << 15;
const REDIRECTION_POLARITY_LOW: u32 = 1 << 13;
const MAX_CHIPS: usize = 8;
const MAX_GSI: usize = 256;
const MAX_REDIRECTIONS: u32 = 120;

/// An I/O APIC mapping and the range of global interrupts it controls.
struct Chip {
    /// Mapped selector and data registers for an occupied slot.
    window: Option<Mmio>,

    /// First global system interrupt served by the chip.
    gsi_base: u32,

    /// Number of hardware redirection entries.
    redirections: u32,
}

impl Chip {
    const fn new() -> Self {
        Self {
            window: None,
            gsi_base: 0,
            redirections: 0,
        }
    }

    /// Selects and reads an I/O APIC register while the controller lock is held.
    fn read(&self, register: u8) -> u32 {
        let window = self.window.as_ref().expect("present I/O APIC is mapped");
        window.write32(REGISTER_SELECT, register.into());
        window.read32(REGISTER_WINDOW)
    }

    /// Selects and writes an I/O APIC register while the controller lock is held.
    fn write(&self, register: u8, value: u32) {
        let window = self.window.as_ref().expect("present I/O APIC is mapped");
        window.write32(REGISTER_SELECT, register.into());
        window.write32(REGISTER_WINDOW, value);
    }

    /// Programs a redirection entry, writing the destination word before the low word.
    fn write_redirection(&self, index: u32, low: u32, high: u32) {
        let register = REDIRECTION_BASE.wrapping_add((index * 2) as u8);
        self.write(register.wrapping_add(1), high);
        self.write(register, low);
    }
}

/// Mapped controllers and their active interrupt vectors, indexed by GSI.
struct State {
    /// Stable chip slots retained while platform devices are bound.
    chips: [Chip; MAX_CHIPS],

    /// Architecture vectors for installed GSIs.
    routes: [Option<u32>; MAX_GSI],
}

impl State {
    const fn new() -> Self {
        Self {
            chips: [const { Chip::new() }; MAX_CHIPS],
            routes: [None; MAX_GSI],
        }
    }

    /// Finds the mapped I/O APIC whose redirection range contains this global interrupt.
    fn chip_index(&self, hwirq: u64) -> Option<usize> {
        self.chips.iter().position(|chip| {
            chip.window.is_some()
                && hwirq >= chip.gsi_base.into()
                && hwirq - u64::from(chip.gsi_base) < chip.redirections.into()
        })
    }
}

static STATE: TicketLock<State> = TicketLock::new(State::new());
static DOMAIN: TicketLock<Option<IrqDomain>> = TicketLock::new(None);
static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_COMPATIBLE,
    flags: 0,
    key: c"intel,ioapic".as_ptr(),
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
    name: c"ioapic".as_ptr(),
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

/// Decodes a global interrupt and optional trigger and polarity flags.
unsafe extern "C" fn translate(
    _context: *mut c_void,
    cells: *const u32,
    count: usize,
    out_hwirq: *mut u64,
    out_flags: *mut u32,
) -> i32 {
    if cells.is_null() || count == 0 || out_hwirq.is_null() || out_flags.is_null() {
        return ddk::EINVAL;
    }

    // SAFETY: the IRQ domain ABI supplies exactly `count` readable cells and
    // writable output words for the callback duration.
    let cells = unsafe { core::slice::from_raw_parts(cells, count) };

    // SAFETY: null was checked above and both output objects are ABI-owned.
    unsafe {
        *out_hwirq = cells[0].into();
        *out_flags = if count > 1 {
            cells[1]
        } else {
            ddk::IRQ_EDGE | ddk::IRQ_ACTIVE_HIGH
        };
    }
    ddk::OK
}

/// Allocates a vector and installs a masked redirection entry for the global interrupt.
unsafe extern "C" fn setup(_context: *mut c_void, hwirq: u64, virq: u32, flags: u32) -> i32 {
    if hwirq as usize >= MAX_GSI {
        return ddk::ENOENT;
    }

    let vector = match ddk::irq_alloc_vector(virq) {
        Ok(vector) => vector,
        Err(error) => return error.status(),
    };

    let destination = ddk::cpu_platform_id(0)
        .ok()
        .filter(|id| *id <= 0xff)
        .unwrap_or(0);
    let mut low = vector & 0xff | REDIRECTION_MASKED;
    if flags & ddk::IRQ_ACTIVE_LOW != 0 {
        low |= REDIRECTION_POLARITY_LOW;
    }

    if flags & ddk::IRQ_LEVEL != 0 {
        low |= REDIRECTION_TRIGGER_LEVEL;
    }

    let mut state = STATE.lock_irqsave();
    let Some(index) = state.chip_index(hwirq) else {
        drop(state);
        let _ = ddk::irq_free_vector(vector);
        return ddk::ENOENT;
    };

    let chip = &state.chips[index];
    chip.write_redirection(
        (hwirq - u64::from(chip.gsi_base)) as u32,
        low,
        (destination as u32) << 24,
    );
    state.routes[hwirq as usize] = Some(vector);
    ddk::OK
}

/// Masks and retires the redirection entry before freeing its interrupt vector.
unsafe extern "C" fn teardown(_context: *mut c_void, hwirq: u64, _virq: u32) {
    if hwirq as usize >= MAX_GSI {
        return;
    }

    let vector = {
        let mut state = STATE.lock_irqsave();
        let Some(index) = state.chip_index(hwirq) else {
            return;
        };

        let Some(vector) = state.routes[hwirq as usize].take() else {
            return;
        };

        let chip = &state.chips[index];
        let register =
            REDIRECTION_BASE.wrapping_add(((hwirq - u64::from(chip.gsi_base)) * 2) as u8);
        chip.write(register, chip.read(register) | REDIRECTION_MASKED);
        vector
    };

    let _ = ddk::irq_free_vector(vector);
}

/// Changes the mask bit on an installed redirection entry.
fn set_mask(hwirq: u64, masked: bool) {
    if hwirq as usize >= MAX_GSI {
        return;
    }

    let state = STATE.lock_irqsave();
    let Some(index) = state.chip_index(hwirq) else {
        return;
    };

    if state.routes[hwirq as usize].is_none() {
        return;
    }

    let chip = &state.chips[index];
    let register = REDIRECTION_BASE.wrapping_add(((hwirq - u64::from(chip.gsi_base)) * 2) as u8);
    let low = chip.read(register);
    chip.write(
        register,
        if masked {
            low | REDIRECTION_MASKED
        } else {
            low & !REDIRECTION_MASKED
        },
    );
}

/// Masks the I/O APIC route for this global interrupt.
unsafe extern "C" fn mask(_context: *mut c_void, hwirq: u64) {
    set_mask(hwirq, true);
}

/// Unmasks the installed I/O APIC route.
unsafe extern "C" fn unmask(_context: *mut c_void, hwirq: u64) {
    set_mask(hwirq, false);
}

/// Retargets an installed redirection entry to the selected CPU's APIC ID.
unsafe extern "C" fn set_affinity(_context: *mut c_void, hwirq: u64, cpu: u32) -> i32 {
    if hwirq as usize >= MAX_GSI {
        return ddk::ENOENT;
    }

    let destination = match ddk::cpu_platform_id(cpu) {
        Ok(value) if value <= 0xff => value,
        Ok(_) => return ddk::ENOTSUP,
        Err(error) => return error.status(),
    };

    let state = STATE.lock_irqsave();
    let Some(index) = state.chip_index(hwirq) else {
        return ddk::ENOENT;
    };

    if state.routes[hwirq as usize].is_none() {
        return ddk::ENOENT;
    }

    let chip = &state.chips[index];
    let register = REDIRECTION_BASE.wrapping_add(((hwirq - u64::from(chip.gsi_base)) * 2) as u8);
    chip.write(register.wrapping_add(1), (destination as u32) << 24);
    ddk::OK
}

static DOMAIN_DEF: raw::IrqDomainDef = raw::IrqDomainDef {
    size: raw::IRQ_DOMAIN_DEF_SIZE,
    translate: Some(translate),
    setup: Some(setup),
    teardown: Some(teardown),
    mask: Some(mask),
    unmask: Some(unmask),
    eoi: None,
    set_affinity: Some(set_affinity),
    claim: None,
    complete: None,
    compose_message: None,
    context: ptr::null_mut(),
};

/// Maps a firmware-described I/O APIC and registers its global interrupt range.
unsafe extern "C" fn probe(
    _context: *mut c_void,
    pointer: *const raw::Device,
    _match_data: usize,
) -> i32 {
    // SAFETY: the driver core calls probe with a live kernel device handle.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };

    let resource = match device.resource(RESOURCE_MEMORY, 0) {
        Ok(value) => value,
        Err(error) => return error.status(),
    };

    if resource.length < (REGISTER_WINDOW + size_of::<u32>()) as u64 {
        return ddk::EINVAL;
    }

    let gsi_base = match device.integer(c"gsi-base") {
        Ok(value) if value < MAX_GSI as u64 => value as u32,
        Ok(_) => return ddk::EINVAL,
        Err(error) => return error.status(),
    };

    let window = match Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE) {
        Ok(window) => window,
        Err(error) => return error.status(),
    };
    // Validate and initialize the unpublished chip before taking the IRQ lock.
    let mut chip = Chip {
        window: Some(window),
        gsi_base,
        redirections: 0,
    };

    let redirections = ((chip.read(REG_VERSION) >> 16) & 0xff) + 1;
    if redirections > MAX_REDIRECTIONS
        || u64::from(gsi_base) + u64::from(redirections) > MAX_GSI as u64
    {
        return ddk::EINVAL;
    }
    chip.redirections = redirections;
    for line in 0..redirections {
        chip.write_redirection(line, REDIRECTION_MASKED, 0);
    }

    let (index, pointer) = {
        let mut state = STATE.lock_irqsave();
        let Some((index, slot)) = state
            .chips
            .iter_mut()
            .enumerate()
            .find(|(_, slot)| slot.window.is_none())
        else {
            return ddk::ENOSPC;
        };
        *slot = chip;
        (index, core::ptr::from_mut(slot))
    };

    // SAFETY: the reserved chip slot has a permanent address and is cleared by remove.
    if let Err(error) = unsafe { device.set_data(pointer.cast()) } {
        let mapping = {
            let mut state = STATE.lock_irqsave();
            state.chips[index].window.take()
        };
        drop(mapping);
        return error.status();
    }
    ddk::OK
}

/// Detaches the I/O APIC after checking that its interrupt range has no active routes.
unsafe extern "C" fn remove(_context: *mut c_void, pointer: *const raw::Device) -> i32 {
    // SAFETY: removal callback receives the same live device handle used at probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::OK;
    };

    // SAFETY: probe stored only a pointer to a static `Chip`. It is validated
    // against the state array below before use.
    let chip_pointer = unsafe { device.data() }.cast::<Chip>();
    let mut state = STATE.lock_irqsave();
    let first = state.chips.as_ptr() as usize;
    let end = first + core::mem::size_of_val(&state.chips);
    if chip_pointer.is_null()
        || (chip_pointer as usize) < first
        || (chip_pointer as usize) >= end
        || !(chip_pointer as usize - first).is_multiple_of(core::mem::size_of::<Chip>())
    {
        return ddk::OK;
    }

    let index = (chip_pointer as usize - first) / core::mem::size_of::<Chip>();
    let chip = &state.chips[index];
    if state.routes.iter().enumerate().any(|(gsi, route)| {
        route.is_some()
            && gsi as u64 >= u64::from(chip.gsi_base)
            && (gsi as u64) < u64::from(chip.gsi_base) + u64::from(chip.redirections)
    }) {
        return ddk::EBUSY;
    }

    let chip = &mut state.chips[index];
    if chip.window.is_some() {
        for line in 0..chip.redirections {
            chip.write_redirection(line, REDIRECTION_MASKED, 0);
        }
    }

    let mapping = chip.window.take();
    drop(state);
    // SAFETY: this exclusively removing device cannot receive new callbacks.
    // Context lookup is a thread-only service, outside the IRQ state guard.
    let _ = unsafe { device.set_data(ptr::null_mut()) };
    drop(mapping);
    ddk::OK
}

/// Registers the I/O APIC interrupt domain and platform driver.
fn init(_module: Module) -> Result<()> {
    let bus = Bus::find(c"platform")?;

    // SAFETY: callbacks are static, synchronize STATE with IRQ guards, and
    // chip removal refuses active mappings before releasing MMIO.
    let domain = unsafe {
        IrqDomain::register(
            c"ioapic",
            ddk::IRQ_DOMAIN_DEFAULT,
            MAX_GSI as u32,
            &DOMAIN_DEF,
        )
    }?;
    *DOMAIN.lock_irqsave() = Some(domain);
    let definition = raw::DriverDef {
        bus: bus.as_raw(),
        ..DRIVER
    };

    // SAFETY: the match table and callbacks have static lifetimes.
    let registration = unsafe { DriverRegistration::register(&definition) }?;
    *REGISTRATION.lock_irqsave() = Some(registration);
    Ok(())
}

/// Unregisters the platform driver and I/O APIC interrupt domain.
fn exit(_module: Module) {
    if let Some(mut registration) = { REGISTRATION.lock_irqsave().take() } {
        let _ = registration.unregister();
    }

    if let Some(mut domain) = { DOMAIN.lock_irqsave().take() } {
        let _ = domain.unregister();
    }
}

ddk::module!(b"ioapic\0", b"I/O APIC interrupt controller\0", init, exit);
