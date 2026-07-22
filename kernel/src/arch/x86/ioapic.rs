//! ACPI MADT discovery and IOAPIC interrupt-controller integration.

use alloc::{collections::BTreeSet, vec::Vec};
use core::{mem, ptr, slice};

use log::info;

use crate::{
    dev::{
        self, Error, KERNEL_DRIVER, Result,
        abi::{
            STATUS_ABI_MISMATCH, STATUS_ALREADY_EXISTS, STATUS_BUSY, STATUS_INVALID_ARGUMENT,
            STATUS_IO, STATUS_NOT_FOUND, STATUS_OK, STATUS_OUT_OF_MEMORY, STATUS_PERMISSION_DENIED,
            STATUS_UNSUPPORTED,
        },
        interrupt::{
            INTERRUPT_ABI_V1, InterruptControllerV1, InterruptRouteV1, ROUTE_ACTIVE_HIGH,
            ROUTE_ACTIVE_LOW, ROUTE_EDGE, ROUTE_LEVEL,
        },
    },
    mem::{PhysAddr, VmFlags},
    sys::{
        smp::IrqSpinLock,
        sync::{Mutex, Once},
    },
};

const SDT_HEADER_SIZE: usize = 36;
const MADT_FIXED_SIZE: usize = 44;
const MAX_ACPI_TABLE_SIZE: usize = 1024 * 1024;

const MADT_IOAPIC: u8 = 1;
const MADT_INTERRUPT_OVERRIDE: u8 = 2;

const IOAPIC_REGISTER_SELECT: usize = 0x00;
const IOAPIC_REGISTER_WINDOW: usize = 0x10;
const IOAPIC_VERSION: u8 = 0x01;
const IOAPIC_REDIRECTION_BASE: u8 = 0x10;

const REDIRECTION_POLARITY_LOW: u32 = 1 << 13;
const REDIRECTION_TRIGGER_LEVEL: u32 = 1 << 15;
const REDIRECTION_MASKED: u32 = 1 << 16;

const SPECIFIER_ISA_IRQ: u32 = 1;
const SPECIFIER_GSI: u32 = 2;

static CONTROLLER: Once<IoApicController> = Once::new();

#[derive(Copy, Clone)]
struct InterruptOverride {
    gsi: u32,
    flags: u16,
}

struct IoApic {
    base: usize,
    gsi_base: u32,
    redirection_count: u32,
    lock: IrqSpinLock<()>,
}

struct IoApicController {
    ioapics: Vec<IoApic>,
    isa_overrides: [Option<InterruptOverride>; 16],
    routes: Mutex<BTreeSet<u32>>,
}

/// Initializes the built-in x86 IOAPIC controller and firmware interrupt domain.
pub fn init() -> Result<()> {
    if CONTROLLER.get().is_some() {
        return Ok(());
    }
    let controller = discover_controller()?;
    let count = controller.ioapics.len();
    let controller = CONTROLLER.call_once(|| controller);
    let operations = InterruptControllerV1 {
        size: mem::size_of::<InterruptControllerV1>() as u32,
        abi_version: INTERRUPT_ABI_V1,
        flags: 0,
        context: controller as *const IoApicController as usize,
        connect: Some(connect),
        disconnect: Some(disconnect),
        mask: Some(mask),
        unmask: Some(unmask),
        set_affinity: Some(set_affinity),
        claim: None,
        complete: None,
    };
    // SAFETY: the callback table references a permanent Once allocation and
    // all callbacks serialize IOAPIC register-window access.
    unsafe {
        dev::interrupt::register_controller(
            KERNEL_DRIVER,
            dev::platform_buses()?.firmware,
            operations,
        )?;
    }
    info!("x86/ioapic: registered {count} controller(s)");
    Ok(())
}

/// Encodes one legacy ISA IRQ for the built-in IOAPIC domain.
pub(crate) fn isa_specifier(irq: u32) -> [u8; 8] {
    let mut bytes = [0u8; 8];
    bytes[..4].copy_from_slice(&SPECIFIER_ISA_IRQ.to_le_bytes());
    bytes[4..].copy_from_slice(&irq.to_le_bytes());
    bytes
}

impl IoApicController {
    fn connect(
        &self,
        kind: u32,
        source: u32,
        vector: u8,
        target_platform_id: u64,
        route_flags: u64,
    ) -> Result<u64> {
        let (gsi, polarity_low, level_triggered) =
            self.resolve_source(kind, source, route_flags)?;
        let destination = u8::try_from(target_platform_id).map_err(|_| Error::Unsupported)?;
        let ioapic = self.ioapic_for_gsi(gsi)?;
        let mut routes = self.routes.lock();
        if routes.contains(&gsi) {
            return Err(Error::AlreadyExists);
        }
        let index = gsi - ioapic.gsi_base;
        let mut low = u32::from(vector) | REDIRECTION_MASKED;
        if polarity_low {
            low |= REDIRECTION_POLARITY_LOW;
        }
        if level_triggered {
            low |= REDIRECTION_TRIGGER_LEVEL;
        }
        ioapic.write_redirection(index, low, u32::from(destination) << 24);
        routes.insert(gsi);
        Ok(u64::from(gsi))
    }

    fn disconnect(&self, cookie: u64) -> Result<()> {
        let gsi = u32::try_from(cookie).map_err(|_| Error::InvalidArgument)?;
        let mut routes = self.routes.lock();
        if !routes.contains(&gsi) {
            return Err(Error::NotFound);
        }
        self.ioapic_for_gsi(gsi)?.mask(gsi)?;
        routes.remove(&gsi);
        Ok(())
    }

    fn mask(&self, cookie: u64) -> Result<()> {
        let gsi = u32::try_from(cookie).map_err(|_| Error::InvalidArgument)?;
        let routes = self.routes.lock();
        if !routes.contains(&gsi) {
            return Err(Error::NotFound);
        }
        self.ioapic_for_gsi(gsi)?.mask(gsi)
    }

    fn unmask(&self, cookie: u64) -> Result<()> {
        let gsi = u32::try_from(cookie).map_err(|_| Error::InvalidArgument)?;
        let routes = self.routes.lock();
        if !routes.contains(&gsi) {
            return Err(Error::NotFound);
        }
        self.ioapic_for_gsi(gsi)?.unmask(gsi)
    }

    fn set_affinity(&self, cookie: u64, target_platform_id: u64) -> Result<()> {
        let gsi = u32::try_from(cookie).map_err(|_| Error::InvalidArgument)?;
        let destination = u8::try_from(target_platform_id).map_err(|_| Error::Unsupported)?;
        let routes = self.routes.lock();
        if !routes.contains(&gsi) {
            return Err(Error::NotFound);
        }
        self.ioapic_for_gsi(gsi)?.set_destination(gsi, destination);
        Ok(())
    }

    fn resolve_source(
        &self,
        kind: u32,
        source: u32,
        route_flags: u64,
    ) -> Result<(u32, bool, bool)> {
        match kind {
            SPECIFIER_ISA_IRQ => {
                let source_index = usize::try_from(source).map_err(|_| Error::InvalidArgument)?;
                if source_index >= self.isa_overrides.len() {
                    return Err(Error::InvalidArgument);
                }
                if let Some(override_) = self.isa_overrides[source_index] {
                    let polarity_low = match override_.flags & 0b11 {
                        0 | 1 => false,
                        3 => true,
                        _ => return Err(Error::InvalidArgument),
                    };
                    let level_triggered = match (override_.flags >> 2) & 0b11 {
                        0 | 1 => false,
                        3 => true,
                        _ => return Err(Error::InvalidArgument),
                    };
                    Ok((override_.gsi, polarity_low, level_triggered))
                } else {
                    let (polarity_low, level_triggered) = route_modes(route_flags, false, false)?;
                    Ok((source, polarity_low, level_triggered))
                }
            }
            SPECIFIER_GSI => {
                let (polarity_low, level_triggered) = route_modes(route_flags, false, false)?;
                Ok((source, polarity_low, level_triggered))
            }
            _ => Err(Error::InvalidArgument),
        }
    }

    fn ioapic_for_gsi(&self, gsi: u32) -> Result<&IoApic> {
        self.ioapics
            .iter()
            .find(|ioapic| {
                gsi >= ioapic.gsi_base && gsi - ioapic.gsi_base < ioapic.redirection_count
            })
            .ok_or(Error::NotFound)
    }
}

impl IoApic {
    fn register(&self, register: u8) -> u32 {
        let _lock = self.lock.lock();
        // SAFETY: this object owns a mapped IOAPIC register-select/window pair.
        unsafe { self.read_locked(register) }
    }

    fn write_redirection(&self, index: u32, low: u32, high: u32) {
        let _lock = self.lock.lock();
        let register = IOAPIC_REDIRECTION_BASE + (index as u8) * 2;
        // SAFETY: discovery bounded `index` by the hardware redirection count
        // and the lock serializes the shared register window.
        unsafe {
            self.write_locked(register + 1, high);
            self.write_locked(register, low);
        }
    }

    fn mask(&self, gsi: u32) -> Result<()> {
        self.update_low(gsi, |low| low | REDIRECTION_MASKED)
    }

    fn unmask(&self, gsi: u32) -> Result<()> {
        self.update_low(gsi, |low| low & !REDIRECTION_MASKED)
    }

    fn set_destination(&self, gsi: u32, destination: u8) {
        let index = gsi - self.gsi_base;
        let _lock = self.lock.lock();
        let register = IOAPIC_REDIRECTION_BASE + (index as u8) * 2;
        // SAFETY: the GSI was resolved to this controller and the lock
        // serializes the register window.
        unsafe { self.write_locked(register + 1, u32::from(destination) << 24) };
    }

    fn update_low(&self, gsi: u32, update: impl FnOnce(u32) -> u32) -> Result<()> {
        if gsi < self.gsi_base || gsi - self.gsi_base >= self.redirection_count {
            return Err(Error::NotFound);
        }
        let index = gsi - self.gsi_base;
        let _lock = self.lock.lock();
        let register = IOAPIC_REDIRECTION_BASE + (index as u8) * 2;
        // SAFETY: the GSI was resolved to this controller and the lock
        // serializes the register window.
        unsafe {
            let low = self.read_locked(register);
            self.write_locked(register, update(low));
        }
        Ok(())
    }

    unsafe fn read_locked(&self, register: u8) -> u32 {
        // SAFETY: the caller holds the IOAPIC window lock and both addresses
        // lie within the mapped architectural register page.
        unsafe {
            ptr::write_volatile(
                (self.base + IOAPIC_REGISTER_SELECT) as *mut u32,
                u32::from(register),
            );
            ptr::read_volatile((self.base + IOAPIC_REGISTER_WINDOW) as *const u32)
        }
    }

    unsafe fn write_locked(&self, register: u8, value: u32) {
        // SAFETY: the caller holds the IOAPIC window lock and both addresses
        // lie within the mapped architectural register page.
        unsafe {
            ptr::write_volatile(
                (self.base + IOAPIC_REGISTER_SELECT) as *mut u32,
                u32::from(register),
            );
            ptr::write_volatile((self.base + IOAPIC_REGISTER_WINDOW) as *mut u32, value);
        }
    }
}

unsafe extern "C" fn connect(
    context: usize,
    route: *const InterruptRouteV1,
    out_cookie: *mut u64,
) -> i32 {
    if context == 0
        || route.is_null()
        || !(route as usize).is_multiple_of(mem::align_of::<InterruptRouteV1>())
        || out_cookie.is_null()
        || !(out_cookie as usize).is_multiple_of(mem::align_of::<u64>())
    {
        return STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: pointer validity and alignment are required by the callback ABI.
    let route = unsafe { &*route };
    if route.abi_version != INTERRUPT_ABI_V1
        || (route.size as usize) < mem::size_of::<InterruptRouteV1>()
        || route.vector > u8::MAX.into()
        || route.specifier.len != 8
        || route.specifier.data.is_null()
    {
        return STATUS_ABI_MISMATCH;
    }
    // SAFETY: the route ABI provides an eight-byte readable specifier.
    let specifier = unsafe { slice::from_raw_parts(route.specifier.data, 8) };
    let kind = u32::from_le_bytes(specifier[..4].try_into().expect("IOAPIC kind width"));
    let source = u32::from_le_bytes(specifier[4..].try_into().expect("IOAPIC source width"));
    // SAFETY: registration stores a permanent Once-backed controller pointer.
    let controller = unsafe { &*(context as *const IoApicController) };
    match controller.connect(
        kind,
        source,
        route.vector as u8,
        route.target_platform_id,
        route.flags,
    ) {
        Ok(cookie) => {
            // SAFETY: the callback ABI requires a writable aligned output.
            unsafe { out_cookie.write(cookie) };
            STATUS_OK
        }
        Err(error) => status(error),
    }
}

unsafe extern "C" fn disconnect(context: usize, cookie: u64) -> i32 {
    with_controller(context, |controller| controller.disconnect(cookie))
}

unsafe extern "C" fn mask(context: usize, cookie: u64) -> i32 {
    with_controller(context, |controller| controller.mask(cookie))
}

unsafe extern "C" fn unmask(context: usize, cookie: u64) -> i32 {
    with_controller(context, |controller| controller.unmask(cookie))
}

unsafe extern "C" fn set_affinity(
    context: usize,
    cookie: u64,
    route: *const InterruptRouteV1,
) -> i32 {
    if route.is_null() || !(route as usize).is_multiple_of(mem::align_of::<InterruptRouteV1>()) {
        return STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: pointer validity and alignment are required by the callback ABI.
    let route = unsafe { &*route };
    if route.abi_version != INTERRUPT_ABI_V1
        || (route.size as usize) < mem::size_of::<InterruptRouteV1>()
    {
        return STATUS_ABI_MISMATCH;
    }
    with_controller(context, |controller| {
        controller.set_affinity(cookie, route.target_platform_id)
    })
}

fn with_controller(context: usize, operation: impl FnOnce(&IoApicController) -> Result<()>) -> i32 {
    if context == 0 {
        return STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: registration stores a permanent Once-backed controller pointer.
    let controller = unsafe { &*(context as *const IoApicController) };
    operation(controller).map_or_else(status, |_| STATUS_OK)
}

fn discover_controller() -> Result<IoApicController> {
    let rsdp_address = dev::acpi_rsdp_address().ok_or(Error::NotFound)?;
    // SAFETY: the ACPI subsystem validated and retained this Limine mapping.
    let rsdp_v1 = unsafe { slice::from_raw_parts(rsdp_address as *const u8, 20) };
    let revision = rsdp_v1[15];
    let rsdt = u64::from(u32::from_le_bytes(
        rsdp_v1[16..20].try_into().expect("RSDT address width"),
    ));
    let xsdt = if revision >= 2 {
        // SAFETY: ACPI revision 2 guarantees the complete extended RSDP.
        let rsdp_v2 = unsafe { slice::from_raw_parts(rsdp_address as *const u8, 36) };
        u64::from_le_bytes(rsdp_v2[24..32].try_into().expect("XSDT address width"))
    } else {
        0
    };
    let madt = if xsdt != 0 {
        find_table(xsdt, 8, b"XSDT", b"APIC").or_else(|_| find_table(rsdt, 4, b"RSDT", b"APIC"))?
    } else {
        find_table(rsdt, 4, b"RSDT", b"APIC")?
    };
    parse_madt(madt)
}

fn find_table(
    root_address: u64,
    entry_size: usize,
    root_signature: &[u8; 4],
    wanted_signature: &[u8; 4],
) -> Result<&'static [u8]> {
    if root_address == 0 {
        return Err(Error::NotFound);
    }
    let root = table_at(root_address)?;
    if &root[..4] != root_signature || !(root.len() - SDT_HEADER_SIZE).is_multiple_of(entry_size) {
        return Err(Error::InvalidArgument);
    }
    for entry in root[SDT_HEADER_SIZE..].chunks_exact(entry_size) {
        let address = if entry_size == 8 {
            u64::from_le_bytes(entry.try_into().expect("XSDT entry width"))
        } else {
            u64::from(u32::from_le_bytes(
                entry.try_into().expect("RSDT entry width"),
            ))
        };
        if address == 0 {
            continue;
        }
        let table = table_at(address)?;
        if &table[..4] == wanted_signature {
            return Ok(table);
        }
    }
    Err(Error::NotFound)
}

fn table_at(physical: u64) -> Result<&'static [u8]> {
    let address = crate::mem::phys_to_virt(PhysAddr::new(physical)).as_u64() as usize;
    // SAFETY: the HHDM maps physical firmware tables for the kernel lifetime.
    let header = unsafe { slice::from_raw_parts(address as *const u8, SDT_HEADER_SIZE) };
    let length = u32::from_le_bytes(header[4..8].try_into().expect("SDT length width")) as usize;
    if !(SDT_HEADER_SIZE..=MAX_ACPI_TABLE_SIZE).contains(&length) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: ACPI supplies a complete table at this physical address and the
    // validated length is bounded before constructing the slice.
    let table = unsafe { slice::from_raw_parts(address as *const u8, length) };
    if checksum(table) != 0 {
        return Err(Error::InvalidArgument);
    }
    Ok(table)
}

fn parse_madt(madt: &[u8]) -> Result<IoApicController> {
    if madt.len() < MADT_FIXED_SIZE || &madt[..4] != b"APIC" {
        return Err(Error::InvalidArgument);
    }
    let mut ioapics = Vec::new();
    let mut overrides = [None; 16];
    let mut offset = MADT_FIXED_SIZE;
    while offset < madt.len() {
        if madt.len() - offset < 2 {
            return Err(Error::InvalidArgument);
        }
        let entry_type = madt[offset];
        let length = madt[offset + 1] as usize;
        if length < 2 || offset + length > madt.len() {
            return Err(Error::InvalidArgument);
        }
        let entry = &madt[offset..offset + length];
        match entry_type {
            MADT_IOAPIC if length >= 12 => {
                let physical = u64::from(u32::from_le_bytes(
                    entry[4..8].try_into().expect("IOAPIC address width"),
                ));
                let gsi_base =
                    u32::from_le_bytes(entry[8..12].try_into().expect("IOAPIC GSI base width"));
                let base = map_ioapic(physical)?;
                let mut ioapic = IoApic {
                    base,
                    gsi_base,
                    redirection_count: 0,
                    lock: IrqSpinLock::new(()),
                };
                let version = ioapic.register(IOAPIC_VERSION);
                ioapic.redirection_count = ((version >> 16) & 0xFF) + 1;
                if ioapic.redirection_count > 120 {
                    return Err(Error::Unsupported);
                }
                ioapics.push(ioapic);
            }
            MADT_INTERRUPT_OVERRIDE if length >= 10 && entry[2] == 0 => {
                let source = entry[3] as usize;
                if source < overrides.len() {
                    if overrides[source].is_some() {
                        return Err(Error::AlreadyExists);
                    }
                    overrides[source] = Some(InterruptOverride {
                        gsi: u32::from_le_bytes(
                            entry[4..8].try_into().expect("override GSI width"),
                        ),
                        flags: u16::from_le_bytes(
                            entry[8..10].try_into().expect("override flags width"),
                        ),
                    });
                }
            }
            _ => {}
        }
        offset += length;
    }
    if ioapics.is_empty() {
        return Err(Error::NotFound);
    }
    ioapics.sort_by_key(|ioapic| ioapic.gsi_base);
    for pair in ioapics.windows(2) {
        let end = pair[0]
            .gsi_base
            .checked_add(pair[0].redirection_count)
            .ok_or(Error::InvalidArgument)?;
        if end > pair[1].gsi_base {
            return Err(Error::InvalidArgument);
        }
    }
    Ok(IoApicController {
        ioapics,
        isa_overrides: overrides,
        routes: Mutex::new(BTreeSet::new()),
    })
}

fn map_ioapic(physical: u64) -> Result<usize> {
    if physical == 0 || physical & 0xFFF != 0 {
        return Err(Error::InvalidArgument);
    }
    let physical = PhysAddr::new(physical);
    let virtual_address = crate::mem::phys_to_virt(physical);
    let root = super::paging::active_root();
    // SAFETY: this only queries the active kernel page tables for the IOAPIC
    // page discovered in the validated MADT.
    if unsafe { super::paging::translate(root, virtual_address).is_none() } {
        let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL | VmFlags::DEVICE;
        // SAFETY: the MADT identifies this aligned page as IOAPIC MMIO and
        // platform initialization is the sole mapper for the range.
        unsafe { super::paging::map_page(root, virtual_address, physical, flags) }
            .map_err(|_| Error::OutOfMemory)?;
    }
    Ok(virtual_address.as_u64() as usize)
}

fn route_modes(flags: u64, default_low: bool, default_level: bool) -> Result<(bool, bool)> {
    let polarity_low = if flags & ROUTE_ACTIVE_LOW != 0 {
        true
    } else if flags & ROUTE_ACTIVE_HIGH != 0 {
        false
    } else {
        default_low
    };
    let level = if flags & ROUTE_LEVEL != 0 {
        true
    } else if flags & ROUTE_EDGE != 0 {
        false
    } else {
        default_level
    };
    Ok((polarity_low, level))
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

fn status(error: Error) -> i32 {
    match error {
        Error::InvalidArgument => STATUS_INVALID_ARGUMENT,
        Error::NotFound => STATUS_NOT_FOUND,
        Error::AlreadyExists => STATUS_ALREADY_EXISTS,
        Error::PermissionDenied => STATUS_PERMISSION_DENIED,
        Error::Busy => STATUS_BUSY,
        Error::AbiMismatch => STATUS_ABI_MISMATCH,
        Error::Unsupported => STATUS_UNSUPPORTED,
        Error::OutOfMemory => STATUS_OUT_OF_MEMORY,
        _ => STATUS_IO,
    }
}
