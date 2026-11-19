//!
//! # RISC-V Platform-Level Interrupt Controller
//!
//! The PLIC routes external interrupt sources to supervisor contexts found in
//! firmware. The driver uses hart IDs to locate the context for each CPU and
//! implements claim and completion for the kernel's external interrupt path.
//!
//! The IRQ-safe state lock protects source routing and register updates.
//! Domain registration and teardown run outside that lock. A refused domain
//! unregister keeps the mapping available for the callbacks that still use it.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code)]
// The module ABI fixes integer widths, and all controller indices are checked
// against bounded static arrays before conversion.
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

const PRIORITY_BASE: usize = 0;
const ENABLE_BASE: usize = 0x2000;
const ENABLE_STRIDE: usize = 0x80;
const CONTEXT_BASE: usize = 0x20_0000;
const CONTEXT_STRIDE: usize = 0x1000;
const CONTEXT_THRESHOLD: usize = 0;
const CONTEXT_CLAIM: usize = 4;
const SUPERVISOR_EXTERNAL_INTERRUPT: u64 = 9;
const MAX_SOURCES: usize = 1024;
const MAX_CONTEXTS: usize = 256;

/// A firmware hart and its corresponding supervisor interrupt context.
#[derive(Clone, Copy)]
struct Context {
    /// Firmware hart ID used to find the target CPU.
    platform_id: u64,

    /// Hardware supervisor context number.
    number: u32,
}

const EMPTY_CONTEXT: Context = Context {
    platform_id: u64::MAX,
    number: 0,
};

/// Destination context and enable state of an installed source.
#[derive(Clone, Copy)]
struct Route {
    /// Context currently selected for this source.
    context: u32,

    /// Whether the source is enabled in that context.
    enabled: bool,
}

/// Controller mapping, discovered CPU contexts, and live interrupt routes.
struct State {
    /// Mapped controller registers while the device is attached.
    window: Option<Mmio>,

    /// Exclusive upper bound on valid source numbers.
    source_count: usize,

    /// Storage for discovered CPU supervisor contexts.
    contexts: [Context; MAX_CONTEXTS],

    /// Number of initialized context records.
    context_count: usize,

    /// Installed routes indexed by hardware source.
    routes: [Option<Route>; MAX_SOURCES],
}

impl State {
    const fn new() -> Self {
        Self {
            window: None,
            source_count: 0,
            contexts: [EMPTY_CONTEXT; MAX_CONTEXTS],
            context_count: 0,
            routes: [None; MAX_SOURCES],
        }
    }

    /// Resets the software routing state and returns the controller mapping for teardown.
    fn clear(&mut self) -> Option<Mmio> {
        let window = self.window.take();
        self.source_count = 0;
        self.context_count = 0;
        self.routes.fill(None);
        window
    }

    fn window(&self) -> &Mmio {
        self.window.as_ref().expect("present PLIC is mapped")
    }

    /// Finds the PLIC supervisor context corresponding to a firmware hart ID.
    fn context_for_platform(&self, platform_id: u64) -> Option<u32> {
        self.contexts[..self.context_count]
            .iter()
            .find(|context| context.platform_id == platform_id)
            .map(|context| context.number)
    }

    fn valid_source(&self, hwirq: u64) -> bool {
        hwirq != 0 && hwirq < self.source_count as u64
    }

    fn priority_offset(hwirq: u64) -> usize {
        PRIORITY_BASE + hwirq as usize * 4
    }

    fn enable_offset(context: u32, hwirq: u64) -> usize {
        ENABLE_BASE + context as usize * ENABLE_STRIDE + hwirq as usize / 32 * 4
    }

    fn context_offset(context: u32, register: usize) -> usize {
        CONTEXT_BASE + context as usize * CONTEXT_STRIDE + register
    }

    /// Updates one source's interrupt-enable bit in the selected PLIC context.
    fn set_enabled(&self, context: u32, hwirq: u64, enabled: bool) {
        let offset = Self::enable_offset(context, hwirq);
        let bit = 1u32 << (hwirq as u32 % 32);
        let value = self.window().read32(offset);
        self.window()
            .write32(offset, if enabled { value | bit } else { value & !bit });
    }
}

static STATE: TicketLock<State> = TicketLock::new(State::new());
static DOMAIN: TicketLock<Option<IrqDomain>> = TicketLock::new(None);
static REGISTRATION: TicketLock<Option<DriverRegistration>> = TicketLock::new(None);

static MATCHES: [raw::Match; 3] = [
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"spacemit,k1-plic".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"sifive,plic-1.0.0".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
    raw::Match {
        kind: ddk::MATCH_COMPATIBLE,
        flags: 0,
        key: c"riscv,plic0".as_ptr(),
        value: ptr::null(),
        id0: 0,
        mask0: 0,
        id1: 0,
        mask1: 0,
        data: 0,
        score: 0,
    },
];

const DRIVER: raw::DriverDef = raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"plic".as_ptr(),
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

/// Decodes a firmware interrupt specifier into a PLIC source and trigger flags.
unsafe extern "C" fn translate(
    _context: *mut c_void,
    cells: *const u32,
    count: usize,
    out_hwirq: *mut u64,
    out_flags: *mut u32,
) -> i32 {
    if cells.is_null() || count != 1 || out_hwirq.is_null() || out_flags.is_null() {
        return ddk::EINVAL;
    }

    // SAFETY: all pointers are non-null and the ABI promises one readable cell
    // plus writable output words for this callback.
    let hwirq = u64::from(unsafe { *cells });
    if !STATE.lock_irqsave().valid_source(hwirq) {
        return ddk::ENOENT;
    }

    // SAFETY: the output pointers were validated above.
    unsafe {
        *out_hwirq = hwirq;
        *out_flags = ddk::IRQ_LEVEL | ddk::IRQ_ACTIVE_HIGH;
    }
    ddk::OK
}

/// Installs a disabled source route and sets its initial priority.
unsafe extern "C" fn setup(_context: *mut c_void, hwirq: u64, _virq: u32, _flags: u32) -> i32 {
    let platform_id = match ddk::cpu_platform_id(0) {
        Ok(value) => value,
        Err(error) => return error.status(),
    };

    let mut state = STATE.lock_irqsave();
    if !state.valid_source(hwirq) {
        return ddk::ENOENT;
    }

    let Some(context) = state.context_for_platform(platform_id) else {
        return ddk::ENOENT;
    };
    state.window().write32(State::priority_offset(hwirq), 1);
    state.routes[hwirq as usize] = Some(Route {
        context,
        enabled: false,
    });
    ddk::OK
}

/// Disables the source in its selected context and clears its priority and route.
unsafe extern "C" fn teardown(_context: *mut c_void, hwirq: u64, _virq: u32) {
    let mut state = STATE.lock_irqsave();
    if !state.valid_source(hwirq) {
        return;
    }

    let Some(route) = state.routes[hwirq as usize].take() else {
        return;
    };

    if route.enabled {
        state.set_enabled(route.context, hwirq, false);
    }
    state.window().write32(State::priority_offset(hwirq), 0);
}

/// Enables or disables a configured source in its current context.
fn set_mask(hwirq: u64, masked: bool) {
    let mut state = STATE.lock_irqsave();
    if !state.valid_source(hwirq) {
        return;
    }

    let Some(mut route) = state.routes[hwirq as usize] else {
        return;
    };

    if route.enabled != masked {
        return;
    }
    state.set_enabled(route.context, hwirq, !masked);
    route.enabled = !masked;
    state.routes[hwirq as usize] = Some(route);
}

/// Masks the configured PLIC interrupt source.
unsafe extern "C" fn mask(_context: *mut c_void, hwirq: u64) {
    set_mask(hwirq, true);
}

/// Unmasks the configured PLIC interrupt source.
unsafe extern "C" fn unmask(_context: *mut c_void, hwirq: u64) {
    set_mask(hwirq, false);
}

/// Moves a source to the context of the chosen CPU while preserving its enable state.
unsafe extern "C" fn set_affinity(_context: *mut c_void, hwirq: u64, cpu: u32) -> i32 {
    let platform_id = match ddk::cpu_platform_id(cpu) {
        Ok(value) => value,
        Err(error) => return error.status(),
    };

    let mut state = STATE.lock_irqsave();
    if !state.valid_source(hwirq) {
        return ddk::ENOENT;
    }

    let Some(context) = state.context_for_platform(platform_id) else {
        return ddk::ENOENT;
    };

    let Some(mut route) = state.routes[hwirq as usize] else {
        return ddk::ENOENT;
    };

    if route.context != context && route.enabled {
        state.set_enabled(route.context, hwirq, false);
        state.set_enabled(context, hwirq, true);
    }
    route.context = context;
    state.routes[hwirq as usize] = Some(route);
    ddk::OK
}

/// Reads the highest-priority pending source from the current hart's claim register.
unsafe extern "C" fn claim(
    _context: *mut c_void,
    _cpu: u32,
    platform_id: u64,
    out_hwirq: *mut u64,
) -> i32 {
    if out_hwirq.is_null() {
        return ddk::EINVAL;
    }

    let state = STATE.lock_irqsave();
    let Some(context) = state.context_for_platform(platform_id) else {
        return ddk::ENOENT;
    };

    let hwirq = state
        .window()
        .read32(State::context_offset(context, CONTEXT_CLAIM));
    if hwirq == 0 {
        return 0;
    }

    // SAFETY: the output pointer was validated above.
    unsafe { *out_hwirq = hwirq.into() };
    1
}

/// Acknowledges the claimed source in the current hart's completion register.
unsafe extern "C" fn complete(_context: *mut c_void, _cpu: u32, platform_id: u64, hwirq: u64) {
    let state = STATE.lock_irqsave();
    let Some(context) = state.context_for_platform(platform_id) else {
        return;
    };
    state
        .window()
        .write32(State::context_offset(context, CONTEXT_CLAIM), hwirq as u32);
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
    claim: Some(claim),
    complete: Some(complete),
    compose_message: None,
    context: ptr::null_mut(),
};

/// Registers the PLIC domain and associates it with the firmware device.
fn install_domain(device: Device, source_count: u32) -> Result<()> {
    if DOMAIN.lock_irqsave().is_some() {
        return Err(ddk::Error::from_status(ddk::EBUSY));
    }

    // SAFETY: callbacks are static, synchronize STATE with IRQ guards, and
    // successful domain withdrawal precedes releasing its mapping.
    let mut domain = unsafe {
        IrqDomain::register(
            c"plic",
            ddk::IRQ_DOMAIN_ROOT | ddk::IRQ_DOMAIN_DEFAULT,
            source_count,
            &DOMAIN_DEF,
        )
    }?;

    if let Err(error) = domain.set_device(device) {
        if domain.unregister().is_ok() {
            return Err(error);
        }
        // Legacy/default-domain users may have arrived during registration.
        // Keep their initialized controller and recoverable binding resident.
        ddk::log(
            ddk::LOG_ERROR,
            c"keeping active PLIC domain users after firmware association failed",
        );
    }
    *DOMAIN.lock_irqsave() = Some(domain);
    Ok(())
}

/// Maps the PLIC, identifies supervisor contexts, and initializes source registers.
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

    let ndev = match device.integer(c"riscv,ndev") {
        Ok(value) if value != 0 && value < MAX_SOURCES as u64 => value as usize,
        Ok(_) => return ddk::EINVAL,
        Err(error) => return error.status(),
    };

    let extended_count = device.property_len(c"interrupts-extended");
    let hart_count = device.property_len(c"roanix,interrupt-harts");
    if extended_count == 0 || !extended_count.is_multiple_of(2) || hart_count * 2 != extended_count
    {
        return ddk::EINVAL;
    }

    let mut contexts = [EMPTY_CONTEXT; MAX_CONTEXTS];
    let mut context_count = 0usize;
    for entry in 0..hart_count {
        let interrupt = match device.cell(c"interrupts-extended", entry * 2 + 1) {
            Ok(value) => value,
            Err(error) => return error.status(),
        };

        if interrupt != SUPERVISOR_EXTERNAL_INTERRUPT {
            continue;
        }

        if context_count == MAX_CONTEXTS {
            return ddk::ENOSPC;
        }

        let platform_id = match device.cell(c"roanix,interrupt-harts", entry) {
            Ok(value) => value,
            Err(error) => return error.status(),
        };
        contexts[context_count] = Context {
            platform_id,
            number: entry as u32,
        };
        context_count += 1;
    }

    if context_count == 0 || context_count < ddk::cpu_count() as usize {
        return ddk::EINVAL;
    }

    let last_context = contexts[..context_count]
        .iter()
        .map(|context| context.number)
        .max()
        .unwrap_or(0);
    let required_length = State::context_offset(last_context, CONTEXT_CLAIM) + 4;
    if resource.length < required_length as u64 {
        return ddk::EINVAL;
    }

    let window = match Mmio::map(resource.start, resource.length as usize, ddk::MMIO_DEVICE) {
        Ok(value) => value,
        Err(error) => return error.status(),
    };

    {
        let mut state = STATE.lock_irqsave();
        if state.window.is_some() {
            return ddk::EBUSY;
        }
        state.window = Some(window);
        state.source_count = ndev + 1;
        state.contexts[..context_count].copy_from_slice(&contexts[..context_count]);
        state.context_count = context_count;
        for source in 1..state.source_count {
            state
                .window()
                .write32(State::priority_offset(source as u64), 0);
        }

        let enable_words = state.source_count.div_ceil(32);
        for context in &state.contexts[..state.context_count] {
            for word in 0..enable_words {
                state.window().write32(
                    ENABLE_BASE + context.number as usize * ENABLE_STRIDE + word * 4,
                    0,
                );
            }
            state
                .window()
                .write32(State::context_offset(context.number, CONTEXT_THRESHOLD), 0);
        }
    }

    // Transfer the callback context before publishing a controller that can
    // synchronously admit legacy or firmware-qualified interrupt consumers.
    // SAFETY: STATE is static and fully initialized before this transfer.
    if let Err(error) = unsafe { device.set_data(core::ptr::from_ref(&STATE).cast_mut().cast()) } {
        let mapping = STATE.lock_irqsave().clear();
        drop(mapping);
        return error.status();
    }

    if let Err(error) = install_domain(device, (ndev + 1) as u32) {
        // SAFETY: no registered domain remains on an error return.
        let _ = unsafe { device.set_data(core::ptr::null_mut()) };
        let mapping = STATE.lock_irqsave().clear();
        drop(mapping);
        return error.status();
    }
    ddk::OK
}

/// Unregisters the domain and clears routes before releasing its MMIO mapping.
unsafe extern "C" fn remove(_context: *mut c_void, pointer: *const raw::Device) -> i32 {
    // SAFETY: removal receives the same live device handle used for probe.
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::OK;
    };

    // SAFETY: probe stores only the address of the static state lock.
    if unsafe { device.data() } != core::ptr::from_ref(&STATE).cast_mut().cast() {
        return ddk::OK;
    }

    let domain = { DOMAIN.lock_irqsave().take() };
    if let Some(mut domain) = domain {
        if let Err(error) = domain.unregister() {
            *DOMAIN.lock_irqsave() = Some(domain);
            return error.status();
        }
    }

    let mut state = STATE.lock_irqsave();
    for source in 1..state.source_count {
        let Some(route) = state.routes[source] else {
            continue;
        };

        if route.enabled {
            state.set_enabled(route.context, source as u64, false);
        }
        state
            .window()
            .write32(State::priority_offset(source as u64), 0);
    }

    let mapping = state.clear();
    drop(state);
    // SAFETY: clear the opaque pointer before the controller can be reprobed.
    let _ = unsafe { device.set_data(ptr::null_mut()) };
    drop(mapping);
    ddk::OK
}

/// Registers the PLIC platform driver.
fn init(_module: Module) -> Result<()> {
    let bus = Bus::find(c"platform")?;
    let definition = raw::DriverDef {
        bus: bus.as_raw(),
        ..DRIVER
    };

    // SAFETY: the match table and callbacks have static lifetimes.
    *REGISTRATION.lock_irqsave() = Some(unsafe { DriverRegistration::register(&definition) }?);
    Ok(())
}

/// Unregisters the PLIC platform driver.
fn exit(_module: Module) {
    if let Some(mut registration) = { REGISTRATION.lock_irqsave().take() } {
        let _ = registration.unregister();
    }

    if let Some(mut domain) = { DOMAIN.lock_irqsave().take() } {
        let _ = domain.unregister();
    }
}

ddk::module!(
    b"plic\0",
    b"RISC-V platform-level interrupt controller\0",
    init,
    exit
);
