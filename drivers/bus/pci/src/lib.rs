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
//! x86 requires ACPI MCFG. RISC-V supports QEMU `virt` ECAM and PLIC INTx.

extern crate alloc;
use alloc::vec::Vec;
#[cfg(target_arch = "x86_64")]
use core::slice;
use core::{
    ffi::{CStr, c_void},
    mem, ptr,
};
#[cfg(target_arch = "riscv64")]
use ddk::DriverRegistration;
use ddk::{
    Bus, Device, DeviceBuilder, Error, Mmio, Module, RESOURCE_MEMORY, Result, TicketLock, raw,
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
const QEMU_VIRT_MMIO_START: u64 = 0x4000_0000;
const QEMU_VIRT_MMIO_END: u64 = 0x8000_0000;
#[cfg(target_arch = "x86_64")]
const MAX_MESSAGE_IDS: usize = 2048;

#[repr(C)]
#[cfg(target_arch = "x86_64")]
pub struct PciMsiOps {
    pub size: u32,
    pub enable: Option<
        unsafe extern "C" fn(*mut c_void, *const raw::Device, u32, *mut u32, *mut u32) -> i32,
    >,
    pub unmask: Option<unsafe extern "C" fn(*mut c_void, *const raw::Device) -> i32>,
    pub disable: Option<unsafe extern "C" fn(*mut c_void, *const raw::Device)>,
}
#[derive(Clone, Copy)]
struct Address {
    segment: u16,
    bus: u8,
    device: u8,
    function: u8,
}
struct Ecam {
    segment: u16,
    start_bus: u8,
    end_bus: u8,
    window: Mmio,
}
#[derive(Clone, Copy, Default)]
struct Bar {
    base: u64,
    size: u64,
}
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
    device: Device,
    #[cfg(target_arch = "x86_64")]
    address: Address,
    #[cfg(target_arch = "x86_64")]
    bars: [Bar; 6],
    #[cfg(target_arch = "x86_64")]
    msi: Option<MsiState>,
}
struct State {
    bus: Option<Bus>,
    ecams: Vec<Ecam>,
    functions: Vec<Function>,
    #[cfg(target_arch = "x86_64")]
    domain: Option<IrqDomain>,
    #[cfg(target_arch = "x86_64")]
    publication: Option<InterfacePublication>,
    #[cfg(target_arch = "x86_64")]
    used_ids: [bool; MAX_MESSAGE_IDS],
    next_mmio: u64,
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
            next_mmio: QEMU_VIRT_MMIO_START,
            #[cfg(target_arch = "riscv64")]
            host_driver: None,
        }
    }
    fn ecam(&self, a: Address) -> Option<(&Ecam, usize)> {
        let e = self
            .ecams
            .iter()
            .find(|e| e.segment == a.segment && (e.start_bus..=e.end_bus).contains(&a.bus))?;
        Some((
            e,
            (usize::from(a.bus - e.start_bus) << 20)
                | (usize::from(a.device) << 15)
                | (usize::from(a.function) << 12),
        ))
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
        let s = u32::from(o & 2) * 8;
        let c = self.read32(a, o);
        self.write32(a, o, (c & !(0xffff << s)) | (u32::from(v) << s));
    }
    fn read8(&self, a: Address, o: u16) -> u8 {
        (self.read32(a, o) >> (u32::from(o & 3) * 8)) as u8
    }
    fn allocate_mmio(&mut self, size: u64) -> Option<u64> {
        if !cfg!(target_arch = "riscv64") || size == 0 || !size.is_power_of_two() {
            return None;
        }
        let start = self.next_mmio.checked_add(size - 1)? & !(size - 1);
        let end = start.checked_add(size)?;
        if end > QEMU_VIRT_MMIO_END {
            return None;
        }
        self.next_mmio = end;
        Some(start)
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
static STATE: TicketLock<State> = TicketLock::new(State::new());
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
        let c = s.read16(a, COMMAND);
        s.write16(
            a,
            COMMAND,
            (c | COMMAND_MEMORY | COMMAND_BUS_MASTER) & !COMMAND_INTERRUPT_DISABLE,
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
    let b = hex(a.bus);
    let d = hex(a.device);
    o[..4].copy_from_slice(b"pci@");
    o[4..6].copy_from_slice(&b);
    o[6] = b':';
    o[7..9].copy_from_slice(&d);
    o[9] = b'.';
    o[10] = b'0' + a.function;
    o[11] = 0;
    cstr(&o[..12])
}
fn bar_name(i: usize, o: &mut [u8; 8]) -> &CStr {
    o[..3].copy_from_slice(b"bar");
    o[3] = b'0' + i as u8;
    o[4] = 0;
    cstr(&o[..5])
}
fn probe_bar(s: &mut State, a: Address, i: usize) -> Option<(Bar, u32, bool)> {
    let o = BAR0 + i as u16 * 4;
    let low = s.read32(a, o);
    if low & 1 != 0 {
        return None;
    }
    let wide = low & 6 == 4;
    let high = wide.then(|| s.read32(a, o + 4));
    s.write32(a, o, u32::MAX);
    if wide {
        s.write32(a, o + 4, u32::MAX);
    }
    let ml = s.read32(a, o);
    let mh = wide.then(|| s.read32(a, o + 4));
    if let Some(v) = high {
        s.write32(a, o + 4, v);
    }
    s.write32(a, o, low);
    let (mut base, mask) = if wide {
        (
            (u64::from(high?) << 32) | u64::from(low & !15),
            (u64::from(mh?) << 32) | u64::from(ml & !15),
        )
    } else {
        (
            u64::from(low & !15),
            u64::from(ml & !15) | 0xffff_ffff_0000_0000,
        )
    };
    let size = (!mask).wrapping_add(1);
    if size == 0 || !size.is_power_of_two() {
        return None;
    }
    if base == 0 {
        base = s.allocate_mmio(size)?;
        s.write32(a, o, base as u32 | (low & 15));
        if wide {
            s.write32(a, o + 4, (base >> 32) as u32);
        }
    } else if cfg!(target_arch = "riscv64")
        && (QEMU_VIRT_MMIO_START..QEMU_VIRT_MMIO_END).contains(&base)
    {
        s.next_mmio = s.next_mmio.max(base.saturating_add(size));
    }
    Some((Bar { base, size }, if low & 8 != 0 { 2 } else { 0 }, wide))
}
fn publish_function(s: &mut State, parent: Option<Device>, a: Address) -> Result<()> {
    let identity = s.read32(a, 0);
    if identity as u16 == u16::MAX {
        return Ok(());
    }
    let cr = s.read32(a, 8);
    let class = cr >> 8;
    let h = s.read16(a, HEADER_TYPE) as u8 & 0x7f;
    let count = if h == 0 {
        6
    } else if h == 1 {
        2
    } else {
        0
    };
    let saved = s.read16(a, COMMAND);
    s.write16(a, COMMAND, saved & !(COMMAND_IO | COMMAND_MEMORY));
    let mut bars = [Bar::default(); 6];
    let mut flags = [0u32; 6];
    let mut present = [false; 6];
    let mut i = 0;
    while i < count {
        if let Some((bar, f, wide)) = probe_bar(s, a, i) {
            bars[i] = bar;
            flags[i] = f;
            present[i] = true;
            if wide {
                i += 1;
            }
        }
        i += 1;
    }
    s.write16(a, COMMAND, saved | COMMAND_MEMORY | COMMAND_BUS_MASTER);
    let mut n = [0u8; 24];
    let mut b = DeviceBuilder::new(function_name(a, &mut n))?;
    b.set_bus(s.bus.ok_or(Error::from_status(ddk::EINVAL))?)?;
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
    #[cfg(target_arch = "riscv64")]
    {
        let pin = s.read8(a, 0x3d);
        if (1..=4).contains(&pin) {
            // QEMU virt's PCI interrupt-map swizzles INTA-D onto PLIC 32-35.
            let irq = 32 + (u32::from(a.device) + u32::from(pin) - 1) % 4;
            b.add_irq(&[irq])?;
        }
    }
    for i in 0..count {
        if present[i] {
            let mut rn = [0u8; 8];
            b.add_resource(
                RESOURCE_MEMORY,
                flags[i],
                bars[i].base,
                bars[i].size,
                bar_name(i, &mut rn),
            )?;
        }
    }
    let device = b.publish()?;
    s.functions.push(Function {
        device,
        #[cfg(target_arch = "x86_64")]
        address: a,
        #[cfg(target_arch = "x86_64")]
        bars,
        #[cfg(target_arch = "x86_64")]
        msi: None,
    });
    Ok(())
}
fn enumerate(s: &mut State, parent: Option<Device>) -> Result<()> {
    let ranges: Vec<_> = s
        .ecams
        .iter()
        .map(|e| (e.segment, e.start_bus, e.end_bus))
        .collect();
    for (segment, start, end) in ranges {
        for bus in start..=end {
            for device in 0..32 {
                let first = Address {
                    segment,
                    bus,
                    device,
                    function: 0,
                };
                if s.read16(first, 0) == u16::MAX {
                    continue;
                }
                let fs = if s.read16(first, HEADER_TYPE) & 0x80 != 0 {
                    8
                } else {
                    1
                };
                for function in 0..fs {
                    publish_function(s, parent, Address { function, ..first })?;
                }
            }
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
        let mut s = STATE.lock();
        let i = s
            .functions
            .iter()
            .position(|f| f.device.as_raw() == d.as_raw())
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
                .filter(|b| b.size != 0)
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
            s.write16(a, cap + 2, (ctl | 1 << 14) & !(1 << 15));
            let table = Mmio::map(bar.base + offset, bytes, ddk::MMIO_DEVICE)?;
            let (ids, virqs, msgs) = map_messages(&mut s, count)?;
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
        let s = STATE.lock();
        let f = s
            .functions
            .iter()
            .find(|f| f.device.as_raw() == d.as_raw())
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
        let mut s = STATE.lock();
        if let Some(i) = s
            .functions
            .iter()
            .position(|f| f.device.as_raw() == d.as_raw())
        {
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
fn remove_devices() {
    let devices = {
        let mut s = STATE.lock();
        #[cfg(target_arch = "x86_64")]
        for i in 0..s.functions.len() {
            disable_index(&mut s, i);
        }
        mem::take(&mut s.functions)
            .into_iter()
            .map(|f| f.device)
            .collect::<Vec<_>>()
    };
    for d in devices.into_iter().rev() {
        let _ = d.remove();
    }
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
        let mut routes = ROUTES.lock();
        let Some(slot) = routes.get_mut(h as usize) else {
            ddk::irq_free_vector(vector);
            return Err(Error::from_status(ddk::EINVAL));
        };
        *slot = Some(Route {
            vector,
            cpu: ddk::cpu_current(),
        });
        Ok(())
    })())
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn message_teardown(_: *mut c_void, h: u64, _: u32) {
    if let Some(slot) = ROUTES.lock().get_mut(h as usize) {
        if let Some(route) = slot.take() {
            ddk::irq_free_vector(route.vector);
        }
    }
}
#[cfg(target_arch = "x86_64")]
unsafe extern "C" fn message_affinity(_: *mut c_void, h: u64, cpu: u32) -> i32 {
    status((|| {
        if ddk::cpu_platform_id(cpu)? > u64::from(u8::MAX) {
            return Err(Error::from_status(ddk::ENOTSUP));
        }
        let mut r = ROUTES.lock();
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
            .lock()
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
        let buses = (r.length >> 20).clamp(1, 256);
        let window = Mmio::map(r.start, r.length as usize, ddk::MMIO_DEVICE)?;
        let mut s = STATE.lock();
        if !s.ecams.is_empty() {
            return Err(Error::from_status(ddk::EBUSY));
        }
        s.ecams.push(Ecam {
            segment: 0,
            start_bus: 0,
            end_bus: (buses - 1) as u8,
            window,
        });
        enumerate(&mut s, Some(d))
    })())
}
#[cfg(target_arch = "riscv64")]
unsafe extern "C" fn host_remove(_: *mut c_void, _: *const raw::Device) {
    remove_devices();
    STATE.lock().ecams.clear();
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
    name: c"qemu-virt-pcie".as_ptr(),
    bus: ptr::null(),
    priority: 0,
    matches: HOST_MATCHES.as_ptr(),
    match_count: 1,
    probe: Some(host_probe),
    remove: Some(host_remove),
    shutdown: None,
    context: ptr::null_mut(),
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
        for entry in table[44..].chunks_exact(16) {
            let base = le64(entry);
            let segment = u16::from_le_bytes([entry[8], entry[9]]);
            let start = entry[10];
            let end = entry[11];
            if base == 0 || end < start {
                continue;
            }
            let bytes = (usize::from(end - start) + 1) << 20;
            s.ecams.push(Ecam {
                segment,
                start_bus: start,
                end_bus: end,
                window: Mmio::map(base, bytes, ddk::MMIO_DEVICE)?,
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
    let bus = unsafe { Bus::register(c"pci", &BUS_DEFINITION) }?;
    STATE.lock().bus = Some(bus);
    #[cfg(target_arch = "x86_64")]
    {
        STATE.lock().domain = Some(IrqDomain::register(
            c"x86-pci-msi",
            0,
            MAX_MESSAGE_IDS as u32,
            &MESSAGE_DOMAIN,
        )?);
        let mut s = STATE.lock();
        if let Err(e) = discover_mcfg(&mut s) {
            ddk::log(
                ddk::LOG_ERROR,
                c"PCIe requires ACPI MCFG ECAM; refusing legacy CF8/CFC configuration access",
            );
            return Err(e);
        }
        enumerate(&mut s, None)?;
    }
    #[cfg(target_arch = "riscv64")]
    {
        let platform = Bus::find(c"platform")?;
        let host_def = {
            let mut d = HOST_DRIVER.lock();
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
    if let Some(mut p) = STATE.lock().publication.take() {
        let _ = p.withdraw();
    }
    #[cfg(target_arch = "riscv64")]
    if let Some(mut d) = STATE.lock().host_driver.take() {
        let _ = d.unregister();
    }
    remove_devices();
    let (ecams, bus) = {
        let mut s = STATE.lock();
        (mem::take(&mut s.ecams), s.bus.take())
    };
    drop(ecams);
    #[cfg(target_arch = "x86_64")]
    if let Some(mut d) = STATE.lock().domain.take() {
        let _ = d.unregister();
    }
    if let Some(bus) = bus {
        let _ = bus.unregister();
    }
}
ddk::module!(
    b"pci\0",
    b"ECAM PCIe bus (mandatory MSI/MSI-X on x86_64, PLIC INTx on RISC-V)\0",
    init,
    exit
);
