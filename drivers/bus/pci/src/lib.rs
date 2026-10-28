#![no_std]
#![no_main]
#![allow(unsafe_code)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::doc_markdown,
    clippy::large_stack_arrays,
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::type_complexity
)]

//! ECAM-only PCIe enumeration and MSI/MSI-X configuration.
//! x86 requires ACPI MCFG. RISC-V uses FDT ECAM and controller-qualified INTx.

extern crate alloc;
use alloc::vec::Vec;
#[cfg(target_arch = "x86_64")]
use core::slice;
use core::{
    ffi::{CStr, c_void},
    mem, ptr,
};
#[cfg(target_arch = "riscv64")]
use ddk::{Bus, DriverRegistration};
use ddk::{
    BusRegistration, Device, DeviceBuilder, DeviceRegistration, Error, Mmio, Module, Mutex,
    RESOURCE_MEMORY, Result, TicketLock, raw,
};
#[cfg(target_arch = "x86_64")]
use ddk::{InterfacePublication, IrqDomain};

const COMMAND: u16 = 0x04;
const COMMAND_IO: u16 = 1;
const COMMAND_MEMORY: u16 = 2;
const COMMAND_BUS_MASTER: u16 = 4;
const COMMAND_INTERRUPT_DISABLE: u16 = 1 << 10;
#[cfg(target_arch = "x86_64")]
const STATUS: u16 = 0x06;
#[cfg(target_arch = "x86_64")]
const STATUS_CAPABILITIES: u16 = 1 << 4;
const HEADER_TYPE: u16 = 0x0e;
const BAR0: u16 = 0x10;
#[cfg(target_arch = "x86_64")]
const CAPABILITIES: u16 = 0x34;
#[cfg(target_arch = "x86_64")]
const CAP_MSI: u8 = 0x05;
#[cfg(target_arch = "x86_64")]
const CAP_MSIX: u8 = 0x11;
#[cfg(target_arch = "x86_64")]
const MAX_MESSAGE_IDS: usize = 2048;

#[cfg(target_arch = "x86_64")]
use ddk::pci::MsiOps as PciMsiOps;
use routing::{Address, Lookup as InterruptLookup};
struct Ecam {
    config: config::ConfigRange,
    parent: Option<Device>,
    window: Mmio,
    space: AddressSpace,
}
mod bar;
mod bridge;
mod config;
mod routing;
mod windows;
use bar::{Bar, Kind};
use firmware_tree::InterruptRoute;
use windows::AddressSpace;
#[cfg(target_arch = "x86_64")]
enum MsiMode {
    Msix { table: Mmio },
    Msi,
}
#[cfg(target_arch = "x86_64")]
struct MsiState {
    mode: MsiMode,
    capability: u16,
    hwirqs: Vec<u64>,
    virqs: Vec<u32>,
}
struct Function {
    host: Option<Device>,
    device: Option<DeviceRegistration>,
    address: Address,
    bars: [Bar; 6],
    forwarding: Vec<bridge::Bridge>,
    #[cfg(target_arch = "x86_64")]
    msi: Option<MsiState>,
}
struct State {
    bus: Option<BusRegistration>,
    ecams: Vec<Ecam>,
    functions: Vec<Function>,
    #[cfg(target_arch = "x86_64")]
    domain: Option<IrqDomain>,
    #[cfg(target_arch = "x86_64")]
    publication: Option<InterfacePublication>,
    #[cfg(target_arch = "x86_64")]
    used_ids: [bool; MAX_MESSAGE_IDS],
    #[cfg(target_arch = "riscv64")]
    host_driver: Option<DriverRegistration>,
}
impl State {
    const fn new() -> Self {
        Self {
            bus: None,
            ecams: Vec::new(),
            functions: Vec::new(),
            #[cfg(target_arch = "x86_64")]
            domain: None,
            #[cfg(target_arch = "x86_64")]
            publication: None,
            #[cfg(target_arch = "x86_64")]
            used_ids: [false; MAX_MESSAGE_IDS],
            #[cfg(target_arch = "riscv64")]
            host_driver: None,
        }
    }
    fn ecam(&self, a: Address) -> Option<(&Ecam, usize)> {
        self.ecams
            .iter()
            .find_map(|ecam| ecam.config.offset(a).map(|offset| (ecam, offset)))
    }

    fn bar_address(&self, function: &Function, bar: Bar) -> Option<u64> {
        self.ecam(function.address)?
            .0
            .space
            .translate(bar)
            .filter(|_| {
                function
                    .forwarding
                    .iter()
                    .all(|bridge| bridge.forwards(bar))
            })
    }

    fn read32(&self, a: Address, o: u16) -> u32 {
        self.ecam(a)
            .map_or(u32::MAX, |(e, b)| e.window.read32(b + usize::from(o & !3)))
    }
    fn write32(&self, a: Address, o: u16, v: u32) {
        if let Some((e, b)) = self.ecam(a) {
            e.window.write32(b + usize::from(o & !3), v);
        }
    }
    fn read16(&self, a: Address, o: u16) -> u16 {
        (self.read32(a, o) >> (u32::from(o & 2) * 8)) as u16
    }
    fn write16(&self, a: Address, o: u16, v: u16) {
        // A dword read/modify/write would echo adjacent W1C status bits.
        if let Some((e, base)) = self.ecam(a) {
            e.window.write16(base + usize::from(o), v);
        }
    }
    fn read8(&self, a: Address, o: u16) -> u8 {
        (self.read32(a, o) >> (u32::from(o & 3) * 8)) as u8
    }
    #[cfg(target_arch = "x86_64")]
    fn alloc_id(&mut self) -> Result<u64> {
        let first = if cfg!(target_arch = "riscv64") {
            128
        } else {
            1
        };
        let id = (first..MAX_MESSAGE_IDS)
            .find(|i| !self.used_ids[*i])
            .ok_or(Error::from_status(ddk::ENOSPC))?;
        self.used_ids[id] = true;
        Ok(id as u64)
    }
    #[cfg(target_arch = "x86_64")]
    fn free_id(&mut self, id: u64) {
        if let Some(slot) = self.used_ids.get_mut(id as usize) {
            *slot = false;
        }
    }
}
static STATE: Mutex<State> = Mutex::new(State::new());
fn status(r: Result<()>) -> i32 {
    r.map_or_else(Error::status, |()| ddk::OK)
}

unsafe extern "C" fn prepare(_: *mut c_void, p: *const raw::Device) -> i32 {
    let Some(d) = (unsafe { Device::from_raw(p) }) else {
        return ddk::EINVAL;
    };
    status((|| {
        let a = device_address(d)?;
        let s = STATE.lock();
        let function = s
            .functions
            .iter()
            .find(|function| function.address == a)
            .ok_or(Error::from_status(ddk::ENODEV))?;
        let decode = function
            .bars
            .iter()
            .filter(|bar| s.bar_address(function, **bar).is_some())
            .fold(0, |flags, bar| {
                flags
                    | if bar.kind == Kind::Io {
                        COMMAND_IO
                    } else {
                        COMMAND_MEMORY
                    }
            });
        for bridge in function.forwarding.iter().rev() {
            let command = s.read16(bridge.address, COMMAND);
            s.write16(
                bridge.address,
                COMMAND,
                command | decode | COMMAND_BUS_MASTER,
            );
        }
        let c = s.read16(a, COMMAND);
        s.write16(
            a,
            COMMAND,
            (c | decode | COMMAND_BUS_MASTER) & !COMMAND_INTERRUPT_DISABLE,
        );
        Ok(())
    })())
}
static BUS_DEFINITION: raw::BusDef = raw::BusDef {
    size: raw::BUS_DEF_SIZE,
    match_device: None,
    prepare: Some(prepare),
    cleanup: None,
    shutdown: None,
    context: ptr::null_mut(),
};
fn device_address(d: Device) -> Result<Address> {
    Ok(Address {
        segment: d.integer(c"pci.segment")? as u16,
        bus: d.integer(c"pci.bus")? as u8,
        device: d.integer(c"pci.device")? as u8,
        function: d.integer(c"pci.function")? as u8,
    })
}
fn cstr(b: &[u8]) -> &CStr {
    unsafe { CStr::from_bytes_with_nul_unchecked(b) }
}
fn hex(v: u8) -> [u8; 2] {
    const D: &[u8; 16] = b"0123456789abcdef";
    [D[usize::from(v >> 4)], D[usize::from(v & 15)]]
}
fn function_name(a: Address, o: &mut [u8; 24]) -> &CStr {
    let segment = a.segment.to_be_bytes();
    o[..4].copy_from_slice(b"pci@");
    o[4..6].copy_from_slice(&hex(segment[0]));
    o[6..8].copy_from_slice(&hex(segment[1]));
    o[8] = b':';
    o[9..11].copy_from_slice(&hex(a.bus));
    o[11] = b':';
    o[12..14].copy_from_slice(&hex(a.device));
    o[14] = b'.';
    o[15] = b'0' + a.function;
    o[16] = 0;
    cstr(&o[..17])
}
fn bar_name(i: usize, o: &mut [u8; 8]) -> &CStr {
    o[..3].copy_from_slice(b"bar");
    o[3] = b'0' + i as u8;
    o[4] = 0;
    cstr(&o[..5])
}
struct Configuration<'a> {
    state: &'a State,
    address: Address,
}
impl bar::Config for Configuration<'_> {
    fn read(&self, index: usize) -> u32 {
        self.state.read32(self.address, BAR0 + index as u16 * 4)
    }
    fn write(&mut self, index: usize, value: u32) {
        self.state
            .write32(self.address, BAR0 + index as u16 * 4, value);
    }
}
impl bridge::Config for Configuration<'_> {
    fn read32(&self, offset: u16) -> u32 {
        self.state.read32(self.address, offset)
    }
    fn write32(&mut self, offset: u16, value: u32) {
        self.state.write32(self.address, offset, value);
    }
    fn write16(&mut self, offset: u16, value: u16) {
        self.state.write16(self.address, offset, value);
    }
}
fn probe_bar(s: &State, a: Address, i: usize, count: usize) -> bar::Probe {
    bar::probe(
        &mut Configuration {
            state: s,
            address: a,
        },
        i,
        count,
    )
}
fn bar_key(a: Address, index: usize) -> u64 {
    (u64::from(a.segment) << 24)
        | (u64::from(a.bus) << 16)
        | (u64::from(a.device) << 11)
        | (u64::from(a.function) << 8)
        | index as u64
}

#[cfg(target_arch = "riscv64")]
fn firmware_error(error: firmware_tree::Error) -> Error {
    Error::from_status(match error {
        firmware_tree::Error::Malformed => ddk::EINVAL,
        firmware_tree::Error::Unsupported => ddk::ENOTSUP,
    })
}

fn publish_function(
    parent: Option<Device>,
    a: Address,
    bars: [Bar; 6],
    interrupt: Option<InterruptRoute>,
) -> Result<()> {
    let s = STATE.lock();
    let identity = s.read32(a, 0);
    if identity as u16 == u16::MAX {
        return Ok(());
    }
    let cr = s.read32(a, 8);
    let class = cr >> 8;
    let function = s
        .functions
        .iter()
        .find(|function| function.address == a)
        .ok_or(Error::from_status(ddk::ENODEV))?;
    let physical = bars.map(|bar| s.bar_address(function, bar));
    let parent = if let Some(bridge) = function.forwarding.first() {
        Some(
            s.functions
                .iter()
                .find(|function| function.address == bridge.address)
                .and_then(|function| function.device.as_ref())
                .ok_or(Error::from_status(ddk::ENODEV))?
                .device()?,
        )
    } else {
        parent
    };
    let bus = s
        .bus
        .as_ref()
        .ok_or(Error::from_status(ddk::EINVAL))?
        .bus()?;
    drop(s);
    // Construction and publication can allocate and synchronously probe. Do
    // not retain the configuration lock through either operation.
    let mut n = [0u8; 24];
    let mut b = DeviceBuilder::new(function_name(a, &mut n))?;
    b.set_bus(bus)?;
    if let Some(p) = parent {
        b.set_parent(p)?;
    }
    b.set_dma_mask(u64::MAX)?;
    b.add_u32(c"id", identity)?;
    b.add_u32(c"class", class)?;
    b.add_u32(c"pci.segment", u32::from(a.segment))?;
    b.add_u32(c"pci.bus", u32::from(a.bus))?;
    b.add_u32(c"pci.device", u32::from(a.device))?;
    b.add_u32(c"pci.function", u32::from(a.function))?;
    b.add_u32(c"pci.revision", cr & 0xff)?;
    if let Some(route) = interrupt {
        b.add_firmware_irq(route.phandle, &route.cells)?;
    }
    for i in 0..6 {
        if let Some(cpu) = physical[i] {
            let mut rn = [0u8; 8];
            b.add_resource(
                if bars[i].kind == Kind::Io {
                    ddk::RESOURCE_IO
                } else {
                    RESOURCE_MEMORY
                },
                bars[i].flags(),
                if bars[i].kind == Kind::Io {
                    bars[i].base
                } else {
                    cpu
                },
                bars[i].size,
                bar_name(i, &mut rn),
            )?;
        }
    }
    let unassigned = bars.iter().enumerate().fold(0u32, |mask, (index, bar)| {
        mask | if bar.size != 0 && bar.base == 0 {
            1 << index
        } else {
            0
        }
    });
    b.add_u32(c"pci.unassigned-bars", unassigned)?;
    let unmapped = bars.iter().enumerate().fold(0u32, |mask, (index, bar)| {
        mask | if bar.size != 0 && bar.base != 0 && physical[index].is_none() {
            1 << index
        } else {
            0
        }
    });
    b.add_u32(c"pci.unmapped-bars", unmapped)?;
    match b.publish() {
        Ok(device) => {
            let mut s = STATE.lock();
            let function = s
                .functions
                .iter_mut()
                .find(|f| f.address == a)
                .ok_or(Error::from_status(ddk::ENODEV))?;
            function.device = Some(device);
            Ok(())
        }
        Err(error) => Err(error),
    }
}
fn same_parent(first: Option<Device>, second: Option<Device>) -> bool {
    first.map(Device::as_raw) == second.map(Device::as_raw)
}

fn enumerate(
    parent: Option<Device>,
    route: impl Fn(Address, u8, Option<Address>) -> Result<InterruptLookup<InterruptRoute>>,
) -> Result<()> {
    let mut state = STATE.lock();
    let ranges: Vec<_> = state
        .ecams
        .iter()
        .enumerate()
        .filter(|(_, e)| same_parent(e.parent, parent))
        .map(|(index, e)| (index, e.config))
        .collect();
    let mut discovered = Vec::new();
    let mut bridges = Vec::new();
    for (root, config) in ranges.iter().copied() {
        for bus in config.start_bus..=config.end_bus {
            for device in 0..32 {
                let first = Address {
                    segment: config.segment,
                    bus,
                    device,
                    function: 0,
                };
                if state.read16(first, 0) == u16::MAX {
                    continue;
                }
                let functions = if state.read8(first, HEADER_TYPE) & 0x80 != 0 {
                    8
                } else {
                    1
                };
                for function in 0..functions {
                    let address = Address { function, ..first };
                    if state.read16(address, 0) != u16::MAX
                        && state.read8(address, HEADER_TYPE) & 0x7f == 1
                    {
                        bridges.push(
                            bridge::Bridge::probe(
                                address,
                                &mut Configuration {
                                    state: &state,
                                    address,
                                },
                            )
                            .map_err(|error| {
                                Error::from_status(match error {
                                    bridge::Error::Malformed => ddk::EINVAL,
                                    bridge::Error::Unsupported => ddk::ENOTSUP,
                                })
                            })?,
                        );
                    }
                    if state.read16(address, 0) == u16::MAX
                        || state.functions.iter().any(|f| f.address == address)
                    {
                        continue;
                    }
                    let count = match state.read8(address, HEADER_TYPE) & 0x7f {
                        0 => 6,
                        1 => 2,
                        _ => 0,
                    };
                    let saved = state.read16(address, COMMAND);
                    state.write16(address, COMMAND, saved & !(COMMAND_IO | COMMAND_MEMORY));
                    let mut bars = [Bar::default(); 6];
                    let mut index = 0;
                    while index < count {
                        let found = probe_bar(&state, address, index, count);
                        if let Some(bar) = found.bar {
                            bars[index] = bar;
                        }
                        index += found.slots;
                    }
                    state.write16(address, COMMAND, saved);
                    discovered.push((root, address, bars));
                }
            }
        }
    }
    bridge::validate(&bridges).map_err(|_| Error::from_status(ddk::EINVAL))?;
    let paths: Vec<_> = discovered
        .iter()
        .map(|(root, address, _)| {
            let root_bus =
                cfg!(target_arch = "riscv64").then_some(state.ecams[*root].config.start_bus);
            bridge::path(root_bus, *address, &bridges).map_err(|_| Error::from_status(ddk::EINVAL))
        })
        .collect::<Result<_>>()?;
    let irq_bridges: Vec<_> = bridges
        .iter()
        .map(|bridge| (bridge.address, bridge.secondary))
        .collect();
    let interrupts: Vec<_> = discovered
        .iter()
        .map(|(root, address, _)| {
            routing::resolve(
                state.ecams[*root].config.start_bus,
                *address,
                state.read8(*address, 0x3d),
                &irq_bridges,
                &route,
            )
            .map_err(|error| match error {
                routing::Error::InvalidTopology => Error::from_status(ddk::EINVAL),
                routing::Error::Firmware(error) => error,
            })
        })
        .collect::<Result<_>>()?;
    // Reserve every firmware assignment before choosing any new address. A
    // later BDF can otherwise own the range allocated to an earlier one.
    let mut spaces: Vec<_> = state.ecams.iter().map(|e| e.space.clone()).collect();
    for (root, address, bars) in &discovered {
        for (index, bar) in bars.iter().copied().enumerate() {
            if bar.size != 0 && bar.base != 0 {
                spaces[*root]
                    .reserve(bar_key(*address, index), bar)
                    .map_err(|_| Error::from_status(ddk::EINVAL))?;
            }
        }
    }
    for (index, space) in spaces.iter().enumerate() {
        if spaces[..index].iter().any(|other| space.conflicts(other)) {
            return Err(Error::from_status(ddk::EINVAL));
        }
    }
    for ((root, address, bars), path) in discovered.iter_mut().zip(&paths) {
        for (index, bar) in bars.iter_mut().enumerate() {
            if bar.size == 0 || bar.base != 0 {
                continue;
            }
            if let Some(base) = spaces[*root].allocate_within(
                bar_key(*address, index),
                *bar,
                &bridge::limits(path, *bar),
            ) {
                let saved = state.read16(*address, COMMAND);
                state.write16(*address, COMMAND, saved & !(COMMAND_IO | COMMAND_MEMORY));
                let offset = BAR0 + index as u16 * 4;
                let flags =
                    state.read32(*address, offset) & if bar.kind == Kind::Io { 3 } else { 15 };
                state.write32(*address, offset, base as u32 | flags);
                if bar.address_bits == 64 {
                    state.write32(*address, offset + 4, (base >> 32) as u32);
                }
                state.write16(*address, COMMAND, saved);
                bar.base = base;
            }
        }
    }
    for (ecam, space) in state.ecams.iter_mut().zip(spaces) {
        ecam.space = space;
    }
    // Keep configuration/allocation receipts even if constructing or publishing
    // one device fails. Consumer probes can also reenter PCI during publication.
    let depths: Vec<_> = paths.iter().map(Vec::len).collect();
    for ((_, address, bars), path) in discovered.iter().zip(paths) {
        state.functions.push(Function {
            host: parent,
            device: None,
            address: *address,
            bars: *bars,
            forwarding: path,
            #[cfg(target_arch = "x86_64")]
            msi: None,
        });
    }
    drop(state);
    let mut publications: Vec<_> = discovered.into_iter().zip(interrupts).zip(depths).collect();
    // Parent receipts must exist before publishing children, including when
    // one hierarchy spans several ACPI MCFG entries.
    publications.sort_by_key(|(_, depth)| *depth);
    for (((_, address, bars), interrupt), _) in publications {
        if publish_function(parent, address, bars, interrupt).is_err() {
            ddk::log(
                ddk::LOG_ERROR,
                c"PCI function publication failed; host retains configuration for removal",
            );
        }
    }
    Ok(())
}

#[cfg(target_arch = "x86_64")]
fn capabilities(s: &State, a: Address) -> (Option<u16>, Option<u16>) {
    if s.read16(a, STATUS) & STATUS_CAPABILITIES == 0 {
        return (None, None);
    }
    let mut next = u16::from(s.read8(a, CAPABILITIES) & !3);
    let (mut msi, mut msix) = (None, None);
    for _ in 0..48 {
        if !(0x40..0x100).contains(&next) {
            break;
        }
        match s.read8(a, next) {
            CAP_MSI => msi = Some(next),
            CAP_MSIX => msix = Some(next),
            _ => {}
        }
        next = u16::from(s.read8(a, next + 1) & !3);
        if next == 0 {
            break;
        }
    }
    (msix, msi)
}
#[cfg(target_arch = "x86_64")]
fn rollback(s: &mut State, ids: &mut Vec<u64>) {
    while let Some(id) = ids.pop() {
        if let Some(d) = s.domain.as_ref() {
            let _ = d.unmap(id);
        }
        s.free_id(id);
    }
}
#[cfg(target_arch = "x86_64")]
fn map_messages(s: &mut State, count: usize) -> Result<(Vec<u64>, Vec<u32>, Vec<(u64, u32)>)> {
    let (mut ids, mut virqs, mut msgs) = (Vec::new(), Vec::new(), Vec::new());
    for index in 0..count {
        let id = s.alloc_id()?;
        let d = s.domain.as_ref().ok_or(Error::from_status(ddk::ENOTSUP))?;
        let mapped = match d.map(id, ddk::IRQ_EDGE) {
            Ok(v) => {
                let configured = ddk::irq_set_affinity(v, index as u32 % ddk::cpu_count().max(1))
                    .and_then(|()| d.compose_message(id).map(|message| (v, message)));
                if configured.is_err() {
                    let _ = d.unmap(id);
                }
                configured
            }
            Err(error) => Err(error),
        };
        match mapped {
            Ok((v, m)) => {
                ids.push(id);
                virqs.push(v);
                msgs.push(m);
            }
            Err(e) => {
                s.free_id(id);
                rollback(s, &mut ids);
                return Err(e);
            }
        }
    }
    Ok((ids, virqs, msgs))
}

#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn msi_enable(
    _: *mut c_void,
    p: *const raw::Device,
    requested: u32,
    out: *mut u32,
    actual: *mut u32,
) -> i32 {
    let Some(d) = (unsafe { Device::from_raw(p) }) else {
        return ddk::EINVAL;
    };
    status((|| {
        if requested == 0 || out.is_null() || actual.is_null() {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let address = device_address(d)?;
        let mut s = STATE.lock();
        let i = s
            .functions
            .iter()
            .position(|f| f.address == address)
            .ok_or(Error::from_status(ddk::ENODEV))?;
        if s.functions[i].msi.is_some() {
            return Err(Error::from_status(ddk::EBUSY));
        }
        let a = s.functions[i].address;
        let (msix, msi) = capabilities(&s, a);
        let configured = if let Some(cap) = msix {
            let ctl = s.read16(a, cap + 2);
            let count = (requested as usize).min(usize::from((ctl & 0x7ff) + 1));
            let info = s.read32(a, cap + 4);
            let bir = (info & 7) as usize;
            let offset = u64::from(info & !7);
            let bar = s.functions[i]
                .bars
                .get(bir)
                .copied()
                .filter(|b| b.size != 0 && b.base != 0 && b.kind == Kind::Memory)
                .ok_or(Error::from_status(ddk::EINVAL))?;
            let bytes = count
                .checked_mul(16)
                .ok_or(Error::from_status(ddk::EINVAL))?;
            if offset
                .checked_add(bytes as u64)
                .is_none_or(|end| end > bar.size)
            {
                return Err(Error::from_status(ddk::EINVAL));
            }
            let cpu = s
                .bar_address(&s.functions[i], bar)
                .ok_or(Error::from_status(ddk::ENOTSUP))?;
            let table = Mmio::map(cpu + offset, bytes, ddk::MMIO_DEVICE)?;
            let (ids, virqs, msgs) = map_messages(&mut s, count)?;
            s.write16(a, cap + 2, (ctl | 1 << 14) & !(1 << 15));
            for (j, (addr, data)) in msgs.iter().copied().enumerate() {
                let base = j * 16;
                table.write32(base, addr as u32);
                table.write32(base + 4, (addr >> 32) as u32);
                table.write32(base + 8, data);
                table.write32(base + 12, 1);
            }
            MsiState {
                mode: MsiMode::Msix { table },
                capability: cap,
                hwirqs: ids,
                virqs,
            }
        } else if let Some(cap) = msi {
            let ctl = s.read16(a, cap + 2);
            let (ids, virqs, msgs) = map_messages(&mut s, 1)?;
            let (addr, data) = msgs[0];
            s.write32(a, cap + 4, addr as u32);
            let data_offset = if ctl & (1 << 7) != 0 {
                s.write32(a, cap + 8, (addr >> 32) as u32);
                cap + 12
            } else {
                cap + 8
            };
            s.write16(a, data_offset, data as u16);
            s.write16(a, cap + 2, ctl & !1);
            MsiState {
                mode: MsiMode::Msi,
                capability: cap,
                hwirqs: ids,
                virqs,
            }
        } else {
            ddk::log(
                ddk::LOG_ERROR,
                c"PCI function has no usable MSI-X or MSI capability; refusing INTx fallback",
            );
            return Err(Error::from_status(ddk::ENOTSUP));
        };
        unsafe {
            ptr::copy_nonoverlapping(configured.virqs.as_ptr(), out, configured.virqs.len());
            ptr::write(actual, configured.virqs.len() as u32);
        }
        s.functions[i].msi = Some(configured);
        Ok(())
    })())
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn msi_unmask(_: *mut c_void, p: *const raw::Device) -> i32 {
    let Some(d) = (unsafe { Device::from_raw(p) }) else {
        return ddk::EINVAL;
    };
    status((|| {
        let address = device_address(d)?;
        let s = STATE.lock();
        let f = s
            .functions
            .iter()
            .find(|f| f.address == address)
            .ok_or(Error::from_status(ddk::ENODEV))?;
        let m = f.msi.as_ref().ok_or(Error::from_status(ddk::EINVAL))?;
        match &m.mode {
            MsiMode::Msix { table } => {
                for i in 0..m.virqs.len() {
                    table.write32(i * 16 + 12, 0);
                }
                core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
                let ctl = s.read16(f.address, m.capability + 2);
                s.write16(f.address, m.capability + 2, (ctl | 1 << 15) & !(1 << 14));
            }
            MsiMode::Msi => {
                let ctl = s.read16(f.address, m.capability + 2);
                s.write16(f.address, m.capability + 2, ctl | 1);
            }
        }
        Ok(())
    })())
}
#[cfg(target_arch = "x86_64")]
fn disable_index(s: &mut State, i: usize) {
    let Some(mut m) = s.functions[i].msi.take() else {
        return;
    };
    let a = s.functions[i].address;
    let ctl = s.read16(a, m.capability + 2);
    match &m.mode {
        MsiMode::Msix { table } => {
            s.write16(a, m.capability + 2, (ctl | 1 << 14) & !(1 << 15));
            for j in 0..m.virqs.len() {
                table.write32(j * 16 + 12, 1);
            }
        }
        MsiMode::Msi => s.write16(a, m.capability + 2, ctl & !1),
    }
    while let Some(id) = m.hwirqs.pop() {
        if let Some(d) = s.domain.as_ref() {
            let _ = d.unmap(id);
        }
        s.free_id(id);
    }
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn msi_disable(_: *mut c_void, p: *const raw::Device) {
    if let Some(d) = unsafe { Device::from_raw(p) } {
        let Ok(address) = device_address(d) else {
            return;
        };
        let mut s = STATE.lock();
        if let Some(i) = s.functions.iter().position(|f| f.address == address) {
            disable_index(&mut s, i);
        }
    }
}
#[cfg(target_arch = "x86_64")]
static MSI_OPS: PciMsiOps = PciMsiOps {
    size: mem::size_of::<PciMsiOps>() as u32,
    enable: Some(msi_enable),
    unmask: Some(msi_unmask),
    disable: Some(msi_disable),
};
fn remove_devices(parent: Option<Device>) -> Result<()> {
    let devices: Vec<_> = STATE
        .lock()
        .functions
        .iter()
        .filter(|f| parent.is_none() || same_parent(f.host, parent))
        .filter_map(|f| f.device.as_ref().and_then(|entry| entry.device().ok()))
        .collect();
    for device in devices.into_iter().rev() {
        match device.remove() {
            Ok(()) => {}
            // Parent removal may already have completed the claimed children.
            Err(error) if error.status() == ddk::ENODEV => {}
            Err(error) => return Err(error),
        }
        let mut state = STATE.lock();
        if let Some(index) = state.functions.iter().position(|f| {
            f.device
                .as_ref()
                .and_then(|entry| entry.device().ok())
                .is_some_and(|entry| entry.as_raw() == device.as_raw())
        }) {
            #[cfg(target_arch = "x86_64")]
            disable_index(&mut state, index);
            let function = state.functions.remove(index);
            for ecam in &mut state.ecams {
                if ecam.config.offset(function.address).is_some() {
                    for bar in 0..6 {
                        ecam.space.release(bar_key(function.address, bar));
                    }
                }
            }
        }
    }
    // Unpublished records own BAR reservations as well; they are safe to
    // retire once every published child has completed checked removal.
    let mut state = STATE.lock();
    while let Some(index) = state
        .functions
        .iter()
        .position(|f| f.device.is_none() && (parent.is_none() || same_parent(f.host, parent)))
    {
        #[cfg(target_arch = "x86_64")]
        disable_index(&mut state, index);
        let function = state.functions.remove(index);
        for ecam in &mut state.ecams {
            for bar in 0..6 {
                ecam.space.release(bar_key(function.address, bar));
            }
        }
    }
    Ok(())
}

#[cfg(target_arch = "x86_64")]
#[derive(Clone, Copy)]
struct Route {
    vector: u32,
    cpu: u32,
}
#[cfg(target_arch = "x86_64")]
static ROUTES: TicketLock<[Option<Route>; MAX_MESSAGE_IDS]> =
    TicketLock::new([None; MAX_MESSAGE_IDS]);
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn message_setup(_: *mut c_void, h: u64, v: u32, _: u32) -> i32 {
    status((|| {
        let vector = ddk::irq_alloc_vector(v)?;
        let installed = {
            let mut routes = ROUTES.lock_irqsave();
            if let Some(slot) = routes.get_mut(h as usize) {
                *slot = Some(Route {
                    vector,
                    cpu: ddk::cpu_current(),
                });
                true
            } else {
                false
            }
        };
        if !installed {
            let _ = ddk::irq_free_vector(vector);
            return Err(Error::from_status(ddk::EINVAL));
        }
        Ok(())
    })())
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn message_teardown(_: *mut c_void, h: u64, _: u32) {
    let route = {
        ROUTES
            .lock_irqsave()
            .get_mut(h as usize)
            .and_then(Option::take)
    };
    if let Some(route) = route {
        let _ = ddk::irq_free_vector(route.vector);
    }
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn message_affinity(_: *mut c_void, h: u64, cpu: u32) -> i32 {
    status((|| {
        if ddk::cpu_platform_id(cpu)? > u64::from(u8::MAX) {
            return Err(Error::from_status(ddk::ENOTSUP));
        }
        let mut r = ROUTES.lock_irqsave();
        r.get_mut(h as usize)
            .and_then(Option::as_mut)
            .ok_or(Error::from_status(ddk::ENOENT))?
            .cpu = cpu;
        Ok(())
    })())
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn message_compose(
    _: *mut c_void,
    h: u64,
    address: *mut u64,
    data: *mut u32,
) -> i32 {
    status((|| {
        if address.is_null() || data.is_null() {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let route = ROUTES
            .lock_irqsave()
            .get(h as usize)
            .copied()
            .flatten()
            .ok_or(Error::from_status(ddk::ENOENT))?;
        let apic = ddk::cpu_platform_id(route.cpu)?;
        if apic > u64::from(u8::MAX) {
            return Err(Error::from_status(ddk::ENOTSUP));
        }
        unsafe {
            ptr::write(address, 0xfee0_0000 | (apic << 12));
            ptr::write(data, route.vector);
        }
        Ok(())
    })())
}

#[cfg(target_arch = "x86_64")]
static MESSAGE_DOMAIN: raw::IrqDomainDef = raw::IrqDomainDef {
    size: raw::IRQ_DOMAIN_DEF_SIZE,
    translate: None,
    setup: Some(message_setup),
    teardown: Some(message_teardown),
    mask: None,
    unmask: None,
    eoi: None,
    set_affinity: Some(message_affinity),
    claim: None,
    complete: None,
    compose_message: Some(message_compose),
    context: ptr::null_mut(),
};
#[cfg(target_arch = "riscv64")]
unsafe extern "C" fn host_probe(_: *mut c_void, p: *const raw::Device, _: usize) -> i32 {
    let Some(d) = (unsafe { Device::from_raw(p) }) else {
        return ddk::EINVAL;
    };
    status((|| {
        let r = d.resource(RESOURCE_MEMORY, 0)?;
        if r.length == 0 || !r.length.is_multiple_of(1 << 20) || r.length > (256 << 20) {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let buses = r.length >> 20;
        let (start_bus, end_bus) = match d.property_len(c"bus-range") {
            0 => (0, (buses - 1) as u8),
            2 => {
                let start = d.cell(c"bus-range", 0)?;
                let end = d.cell(c"bus-range", 1)?;
                if end > 255 || start > end || end - start + 1 > buses {
                    return Err(Error::from_status(ddk::EINVAL));
                }
                (start as u8, end as u8)
            }
            _ => return Err(Error::from_status(ddk::EINVAL)),
        };
        let segment = match d.integer(c"linux,pci-domain") {
            Ok(value) => u16::try_from(value).map_err(|_| Error::from_status(ddk::EINVAL))?,
            Err(error) if error.status() == ddk::ENOENT => 0,
            Err(error) => return Err(error),
        };
        let config = config::ConfigRange::new(r.start, r.length, segment, start_bus, end_bus)
            .ok_or(Error::from_status(ddk::EINVAL))?;
        let window = Mmio::map(config.physical, config.length, ddk::MMIO_DEVICE)?;
        let cells = d.property_len(c"roanix,bus-ranges");
        if !cells.is_multiple_of(5) {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let mut windows = Vec::new();
        for row in (0..cells).step_by(5) {
            let kind = match d.cell(c"roanix,bus-ranges", row)? {
                value if value == u64::from(ddk::RESOURCE_MEMORY) => Kind::Memory,
                value if value == u64::from(ddk::RESOURCE_IO) => Kind::Io,
                _ => return Err(Error::from_status(ddk::EINVAL)),
            };
            windows.push(
                windows::Window::new(
                    kind,
                    d.cell(c"roanix,bus-ranges", row + 1)? as u32,
                    d.cell(c"roanix,bus-ranges", row + 2)?,
                    d.cell(c"roanix,bus-ranges", row + 3)?,
                    d.cell(c"roanix,bus-ranges", row + 4)?,
                )
                .map_err(|_| Error::from_status(ddk::EINVAL))?,
            );
        }
        let space = if d.integer(c"roanix,ranges-identity").unwrap_or(0) != 0 {
            AddressSpace::identity()
        } else {
            AddressSpace::new(windows).map_err(|_| Error::from_status(ddk::EINVAL))?
        };
        let mut blob = alloc::vec![0u8; 2 * 1024 * 1024];
        let length = ddk::firmware_devicetree(&mut blob)?;
        let tree = firmware_tree::Tree::parse(&blob[..length]).map_err(firmware_error)?;
        let node = usize::try_from(d.integer(c"roanix,node-index")?)
            .map_err(|_| Error::from_status(ddk::EINVAL))?;
        if tree.nodes().get(node).is_none() {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let mut s = STATE.lock();
        if s.ecams
            .iter()
            .any(|ecam| ecam.config.conflicts(config) || ecam.space.conflicts(&space))
        {
            return Err(Error::from_status(ddk::EINVAL));
        }
        s.ecams.push(Ecam {
            config,
            parent: Some(d),
            window,
            space,
        });
        drop(s);
        if let Err(error) = enumerate(Some(d), |address, pin, bridge| {
            let unit_address = |address: Address| {
                (u32::from(address.bus) << 16)
                    | (u32::from(address.device) << 11)
                    | (u32::from(address.function) << 8)
            };
            let nexus = if let Some(bridge) = bridge {
                let Some(index) = tree
                    .pci_node(node, unit_address(bridge))
                    .map_err(firmware_error)?
                else {
                    return Ok(InterruptLookup::Inherited);
                };
                if tree.nodes()[index].bytes(c"interrupt-map").is_none() {
                    return Ok(InterruptLookup::Inherited);
                }
                index
            } else {
                node
            };
            tree.resolve_interrupt(nexus, &[unit_address(address), 0, 0], &[u32::from(pin)])
                .map(InterruptLookup::Mapped)
                .map_err(firmware_error)
        }) {
            let ecam = {
                let mut state = STATE.lock();
                state
                    .ecams
                    .iter()
                    .position(|ecam| same_parent(ecam.parent, Some(d)))
                    .map(|index| state.ecams.remove(index))
            };
            drop(ecam);
            return Err(error);
        }
        Ok(())
    })())
}
#[cfg(target_arch = "riscv64")]
unsafe extern "C" fn host_remove(_: *mut c_void, pointer: *const raw::Device) -> i32 {
    let Some(device) = (unsafe { Device::from_raw(pointer) }) else {
        return ddk::EINVAL;
    };
    status((|| {
        remove_devices(Some(device))?;
        let ecam = {
            let mut state = STATE.lock();
            state
                .ecams
                .iter()
                .position(|ecam| same_parent(ecam.parent, Some(device)))
                .map(|index| state.ecams.remove(index))
        };
        drop(ecam);
        Ok(())
    })())
}
#[cfg(target_arch = "riscv64")]
static HOST_MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_COMPATIBLE,
    flags: 0,
    key: c"pci-host-ecam-generic".as_ptr(),
    value: ptr::null(),
    id0: 0,
    mask0: 0,
    id1: 0,
    mask1: 0,
    data: 0,
    score: 0,
}];
#[cfg(target_arch = "riscv64")]
static HOST_DRIVER: TicketLock<raw::DriverDef> = TicketLock::new(raw::DriverDef {
    size: raw::DRIVER_DEF_SIZE,
    name: c"pci-ecam-host".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: HOST_MATCHES.as_ptr(),
    match_count: 1,
    probe: Some(host_probe),
    remove: None,
    shutdown: None,
    context: ptr::null_mut(),
    try_remove: Some(host_remove),
});

#[cfg(target_arch = "x86_64")]
fn le32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}
#[cfg(target_arch = "x86_64")]
fn le64(b: &[u8]) -> u64 {
    u64::from_le_bytes(b[..8].try_into().unwrap_or([0; 8]))
}
#[cfg(target_arch = "x86_64")]
fn checksum(b: &[u8]) -> u8 {
    b.iter().fold(0u8, |s, v| s.wrapping_add(*v))
}
#[cfg(target_arch = "x86_64")]
fn acpi_table(physical: u64) -> Result<&'static [u8]> {
    const H: usize = 36;
    let address = Mmio::direct(physical)?;
    let header = unsafe { slice::from_raw_parts(address, H) };
    let length = le32(&header[4..8]) as usize;
    if !(H..=4 * 1024 * 1024).contains(&length) {
        return Err(Error::from_status(ddk::EINVAL));
    }
    let table = unsafe { slice::from_raw_parts(address, length) };
    if checksum(table) != 0 {
        return Err(Error::from_status(ddk::EINVAL));
    }
    Ok(table)
}
#[cfg(target_arch = "x86_64")]
fn discover_mcfg(s: &mut State) -> Result<()> {
    let mut rsdp = [0u8; 4096];
    let length = ddk::firmware_acpi(&mut rsdp)?;
    if length < 20 || &rsdp[..8] != b"RSD PTR " || checksum(&rsdp[..20]) != 0 {
        return Err(Error::from_status(ddk::EINVAL));
    }
    let wide = rsdp[15] >= 2 && length >= 36;
    let root = if wide {
        le64(&rsdp[24..32])
    } else {
        u64::from(le32(&rsdp[16..20]))
    };
    let root = acpi_table(root)?;
    let stride = if wide { 8 } else { 4 };
    for slot in root[36..].chunks_exact(stride) {
        let physical = if wide {
            le64(slot)
        } else {
            u64::from(le32(slot))
        };
        let Ok(table) = acpi_table(physical) else {
            continue;
        };
        if table.len() < 44 || &table[..4] != b"MCFG" {
            continue;
        }
        if !(table.len() - 44).is_multiple_of(16) {
            return Err(Error::from_status(ddk::EINVAL));
        }
        for entry in table[44..].chunks_exact(16) {
            let base = le64(entry);
            let segment = u16::from_le_bytes([entry[8], entry[9]]);
            let start = entry[10];
            let end = entry[11];
            let config = config::ConfigRange::mcfg(base, segment, start, end)
                .ok_or(Error::from_status(ddk::EINVAL))?;
            if s.ecams.iter().any(|ecam| ecam.config.conflicts(config)) {
                return Err(Error::from_status(ddk::EINVAL));
            }
            s.ecams.push(Ecam {
                config,
                parent: None,
                window: Mmio::map(config.physical, config.length, ddk::MMIO_DEVICE)?,
                space: AddressSpace::identity(),
            });
        }
        break;
    }
    if s.ecams.is_empty() {
        Err(Error::from_status(ddk::ENOTSUP))
    } else {
        Ok(())
    }
}

fn init_inner() -> Result<()> {
    let bus = unsafe { BusRegistration::register(c"pci", &BUS_DEFINITION) }?;
    STATE.lock().bus = Some(bus);
    #[cfg(target_arch = "x86_64")]
    {
        // SAFETY: callbacks and ROUTES are static, with short synchronized
        // controller operations. All message mappings retire before withdrawal.
        let domain = unsafe {
            IrqDomain::register(c"x86-pci-msi", 0, MAX_MESSAGE_IDS as u32, &MESSAGE_DOMAIN)
        }?;
        STATE.lock().domain = Some(domain);
        let mut s = STATE.lock();
        if let Err(e) = discover_mcfg(&mut s) {
            ddk::log(
                ddk::LOG_ERROR,
                c"PCIe requires ACPI MCFG ECAM; refusing legacy CF8/CFC configuration access",
            );
            return Err(e);
        }
        drop(s);
        enumerate(None, |_, _, bridge| {
            Ok(if bridge.is_some() {
                InterruptLookup::Inherited
            } else {
                InterruptLookup::Mapped(None)
            })
        })?;
    }
    #[cfg(target_arch = "riscv64")]
    {
        let platform = Bus::find(c"platform")?;
        let host_def = {
            let mut d = HOST_DRIVER.lock_irqsave();
            d.bus = platform.as_raw();
            let p = ptr::from_ref(&*d);
            unsafe { &*p }
        };
        STATE.lock().host_driver = Some(unsafe { DriverRegistration::register(host_def) }?);
    }
    #[cfg(target_arch = "x86_64")]
    {
        STATE.lock().publication = Some(unsafe {
            InterfacePublication::publish(
                c"pci.msi",
                1,
                ddk::IFACE_MAY_SLEEP | ddk::IFACE_CONCURRENT | ddk::IFACE_SINGLETON,
                ptr::from_ref(&MSI_OPS).cast(),
                mem::size_of::<PciMsiOps>(),
                ptr::null_mut(),
            )
        }?);
    }
    Ok(())
}
fn init(module: Module) -> Result<()> {
    let result = init_inner();
    if result.is_err() {
        exit(module);
    }
    result
}
fn exit(_module: Module) {
    #[cfg(target_arch = "x86_64")]
    if let Some(mut p) = { STATE.lock().publication.take() } {
        let _ = p.withdraw();
    }
    #[cfg(target_arch = "riscv64")]
    if let Some(mut d) = { STATE.lock().host_driver.take() } {
        let _ = d.unregister();
    }
    if remove_devices(None).is_err() {
        return;
    }
    let (ecams, bus) = {
        let mut s = STATE.lock();
        (mem::take(&mut s.ecams), s.bus.take())
    };
    drop(ecams);
    #[cfg(target_arch = "x86_64")]
    if let Some(mut d) = { STATE.lock().domain.take() } {
        let _ = d.unregister();
    }
    if let Some(mut bus) = bus {
        let _ = bus.unregister();
    }
}
ddk::module!(
    b"pci\0",
    b"ECAM PCIe bus with MSI/MSI-X and PLIC INTx services\0",
    init,
    exit
);
