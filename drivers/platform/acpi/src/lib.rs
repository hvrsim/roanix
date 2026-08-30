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

//! ACPI platform enumeration. This is intentionally limited to the MADT
//! records consumed by the previous C driver.

use core::{ffi::CStr, slice};

use ddk::{
    Bus, Device, DeviceBuilder, Error, Module, RESOURCE_IO, RESOURCE_MEMORY, Result, TicketLock,
};

const RSDP_V1_SIZE: usize = 20;
const RSDP_V2_MIN_SIZE: usize = 36;
const RSDP_MAX_SIZE: usize = 4096;
const SDT_HEADER_SIZE: usize = 36;
const SDT_MAX_LENGTH: usize = 4 * 1024 * 1024;
const MADT_FIXED_SIZE: usize = 44;
const MADT_IOAPIC: u8 = 1;
const MADT_INTERRUPT_OVERRIDE: u8 = 2;
const MAX_IOAPICS: usize = 8;
const MAX_OVERRIDES: usize = 16;
const MAX_LEGACY_UARTS: usize = 4;

const LEGACY_PORTS: [u16; MAX_LEGACY_UARTS] = [0x3f8, 0x2f8, 0x3e8, 0x2e8];
const LEGACY_IRQS: [u32; MAX_LEGACY_UARTS] = [4, 3, 4, 3];

#[derive(Clone, Copy)]
struct InterruptOverride {
    gsi: u32,
    flags: u16,
    present: bool,
}

const EMPTY_OVERRIDE: InterruptOverride = InterruptOverride {
    gsi: 0,
    flags: 0,
    present: false,
};

struct State {
    overrides: [InterruptOverride; MAX_OVERRIDES],
    devices: [Option<Device>; MAX_IOAPICS + MAX_LEGACY_UARTS],
    count: usize,
    bus: Option<Bus>,
}

impl State {
    const fn new() -> Self {
        Self {
            overrides: [EMPTY_OVERRIDE; MAX_OVERRIDES],
            devices: [None; MAX_IOAPICS + MAX_LEGACY_UARTS],
            count: 0,
            bus: None,
        }
    }

    fn remember(&mut self, device: Device) -> Result<()> {
        if self.count == self.devices.len() {
            return Err(Error::from_status(ddk::ENOSPC));
        }
        self.devices[self.count] = Some(device);
        self.count += 1;
        Ok(())
    }

    fn platform_bus(&self) -> Result<Bus> {
        self.bus.ok_or(Error::from_status(ddk::EINVAL))
    }
}

static STATE: TicketLock<State> = TicketLock::new(State::new());

fn le16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}
fn le32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}
fn le64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes([
        bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
    ])
}
fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

fn cstr(bytes: &[u8]) -> &CStr {
    // SAFETY: every caller writes exactly one trailing NUL and the supplied
    // slice ends at it.
    unsafe { CStr::from_bytes_with_nul_unchecked(bytes) }
}

fn hex_name<'a>(prefix: &[u8], value: u32, output: &'a mut [u8; 32]) -> &'a CStr {
    let mut length = prefix.len();
    output[..length].copy_from_slice(prefix);
    let mut shift: u32 = 28;
    let mut started = false;
    loop {
        let digit = ((value >> shift) & 0xf) as u8;
        if digit != 0 || started || shift == 0 {
            output[length] = if digit < 10 {
                b'0' + digit
            } else {
                b'a' + digit - 10
            };
            length += 1;
            started = true;
        }
        if shift == 0 {
            break;
        }
        shift -= 4;
    }
    output[length] = 0;
    cstr(&output[..=length])
}

fn record_override(state: &mut State, entry: &[u8]) {
    if entry.len() < 10 || entry[3] as usize >= MAX_OVERRIDES {
        return;
    }
    let source = entry[3] as usize;
    state.overrides[source] = InterruptOverride {
        gsi: le32(&entry[4..8]),
        flags: le16(&entry[8..10]),
        present: true,
    };
}

fn resolve_isa(state: &State, isa: u32) -> (u32, u32) {
    let mut gsi = isa;
    let mut flags = ddk::IRQ_EDGE | ddk::IRQ_ACTIVE_HIGH;
    if let Some(override_) = state
        .overrides
        .get(isa as usize)
        .filter(|value| value.present)
    {
        gsi = override_.gsi;
        flags = if override_.flags & 3 == 3 {
            ddk::IRQ_ACTIVE_LOW
        } else {
            ddk::IRQ_ACTIVE_HIGH
        };
        flags |= if (override_.flags >> 2) & 3 == 3 {
            ddk::IRQ_LEVEL
        } else {
            ddk::IRQ_EDGE
        };
    }
    (gsi, flags)
}

fn create_ioapic(state: &mut State, id: u8, address: u64, gsi_base: u32) -> Result<()> {
    let mut name = [0u8; 32];
    let mut builder = DeviceBuilder::new(hex_name(b"ioapic@", address as u32, &mut name))?;
    let compatible = [c"intel,ioapic".as_ptr()];
    builder.set_bus(state.platform_bus()?)?;
    builder.add_strings(c"compatible", &compatible)?;
    builder.add_u32(c"acpi.id", id.into())?;
    builder.add_u32(c"gsi-base", gsi_base)?;
    builder.add_resource(RESOURCE_MEMORY, 0, address, 0x20, c"regs")?;
    state.remember(builder.publish()?)
}

fn create_serial(state: &mut State, index: usize) -> Result<()> {
    let mut name = [0u8; 32];
    let mut builder =
        DeviceBuilder::new(hex_name(b"serial@", LEGACY_PORTS[index].into(), &mut name))?;
    let compatible = [c"pnp,16550a".as_ptr(), c"ns16550a".as_ptr()];
    let (gsi, flags) = resolve_isa(state, LEGACY_IRQS[index]);
    builder.set_bus(state.platform_bus()?)?;
    builder.add_strings(c"compatible", &compatible)?;
    builder.add_u32(c"index", index as u32)?;
    builder.add_u32(c"clock-frequency", 1_843_200)?;
    builder.add_resource(RESOURCE_IO, 0, LEGACY_PORTS[index].into(), 8, c"regs")?;
    builder.add_irq(&[gsi, flags])?;
    state.remember(builder.publish()?)
}

fn map_table(physical: u64) -> Result<&'static [u8]> {
    let address = ddk::Mmio::direct(physical)?;
    // SAFETY: `mmio_direct` returns a valid boot direct-map address. The first
    // SDT header is fixed-size and the C implementation reads the same bytes.
    let header = unsafe { slice::from_raw_parts(address, SDT_HEADER_SIZE) };
    let length = le32(&header[4..8]) as usize;
    if !(SDT_HEADER_SIZE..=SDT_MAX_LENGTH).contains(&length) {
        return Err(Error::from_status(ddk::EINVAL));
    }
    // SAFETY: ACPI SDTs are physically contiguous and `length` was validated
    // against the same bounded maximum used by the legacy implementation.
    let table = unsafe { slice::from_raw_parts(address, length) };
    if checksum(table) != 0 {
        return Err(Error::from_status(ddk::EINVAL));
    }
    Ok(table)
}

fn parse_madt(state: &mut State, madt: &[u8]) -> Result<()> {
    let mut offset = MADT_FIXED_SIZE;
    while offset + 2 <= madt.len() {
        let length = madt[offset + 1] as usize;
        if length < 2 || offset + length > madt.len() {
            break;
        }
        if madt[offset] == MADT_INTERRUPT_OVERRIDE {
            record_override(state, &madt[offset..offset + length]);
        }
        offset += length;
    }

    offset = MADT_FIXED_SIZE;
    let mut created = 0;
    while offset + 2 <= madt.len() {
        let length = madt[offset + 1] as usize;
        if length < 2 || offset + length > madt.len() {
            break;
        }
        let entry = &madt[offset..offset + length];
        if entry[0] == MADT_IOAPIC && entry.len() >= 12 && created < MAX_IOAPICS {
            let address = u64::from(le32(&entry[4..8]));
            if address != 0 {
                create_ioapic(state, entry[2], address, le32(&entry[8..12]))?;
                created += 1;
            }
        }
        offset += length;
    }
    Ok(())
}

fn walk_tables(state: &mut State, root: u64, wide: bool) -> Result<()> {
    let table = map_table(root)?;
    let stride = if wide { 8 } else { 4 };
    for slot in table[SDT_HEADER_SIZE..].chunks_exact(stride) {
        let address = if wide {
            le64(slot)
        } else {
            u64::from(le32(slot))
        };
        if address == 0 {
            continue;
        }
        let Ok(entry) = map_table(address) else {
            continue;
        };
        if entry.len() >= MADT_FIXED_SIZE && &entry[..4] == b"APIC" {
            parse_madt(state, entry)?;
        }
    }
    Ok(())
}

fn init(_module: Module) -> Result<()> {
    let mut rsdp = [0u8; RSDP_MAX_SIZE];
    let length = match ddk::firmware_acpi(&mut rsdp) {
        Ok(length) => length,
        Err(error) if error.status() == ddk::ENOENT => return Ok(()),
        Err(error) => return Err(error),
    };
    if length < RSDP_V1_SIZE || &rsdp[..8] != b"RSD PTR " || checksum(&rsdp[..RSDP_V1_SIZE]) != 0 {
        return Err(Error::from_status(ddk::EINVAL));
    }
    let (root, wide) = if rsdp[15] >= 2 && length >= RSDP_V2_MIN_SIZE {
        let extended = le32(&rsdp[20..24]) as usize;
        if extended < RSDP_V2_MIN_SIZE || extended > length || checksum(&rsdp[..extended]) != 0 {
            return Err(Error::from_status(ddk::EINVAL));
        }
        (le64(&rsdp[24..32]), true)
    } else {
        (u64::from(le32(&rsdp[16..20])), false)
    };
    if root == 0 {
        return Err(Error::from_status(ddk::EINVAL));
    }

    let mut state = STATE.lock_irqsave();
    *state = State::new();
    state.bus = Some(Bus::find(c"platform")?);
    walk_tables(&mut state, root, wide)?;
    for index in 0..MAX_LEGACY_UARTS {
        create_serial(&mut state, index)?;
    }
    Ok(())
}

fn exit(_module: Module) {
    let devices = {
        let mut state = STATE.lock_irqsave();
        let devices = state.devices;
        *state = State::new();
        devices
    };
    for device in devices.into_iter().flatten().rev() {
        let _ = device.remove();
    }
}

ddk::module!(b"acpi\0", b"ACPI platform enumerator\0", init, exit);
