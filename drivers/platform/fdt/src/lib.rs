#![no_std]
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

//! Flattened device-tree platform enumeration without a parser allocation.

use core::{
    ffi::{CStr, c_char},
    ptr,
};

use ddk::{Bus, Device, DeviceBuilder, Error, Module, RESOURCE_MEMORY, Result, TicketLock};

const FDT_MAGIC: u32 = 0xd00d_feed;
const FDT_HEADER_SIZE: usize = 40;
const FDT_BEGIN_NODE: u32 = 1;
const FDT_END_NODE: u32 = 2;
const FDT_PROP: u32 = 3;
const FDT_NOP: u32 = 4;
const FDT_END: u32 = 9;
const MAX_DEPTH: usize = 32;
const MAX_DEVICES: usize = 64;
const MAX_STRINGS: usize = 16;
const MAX_CELLS: usize = 32;
const MAX_BLOB: usize = 2 * 1024 * 1024;

#[derive(Clone, Copy)]
struct Level {
    address_cells: u32,
    size_cells: u32,
}

#[derive(Clone, Copy)]
struct Node {
    name: usize,
    reg: Option<(usize, usize)>,
    compatible: Option<(usize, usize)>,
    interrupts: Option<(usize, usize)>,
    reg_shift: u32,
    reg_width: u32,
    clock: u32,
    interrupt_cells: u32,
    enabled: bool,
}

impl Node {
    const fn empty() -> Self {
        Self {
            name: 0,
            reg: None,
            compatible: None,
            interrupts: None,
            reg_shift: 0,
            reg_width: 0,
            clock: 0,
            interrupt_cells: 0,
            enabled: true,
        }
    }
}

struct State {
    blob: [u8; MAX_BLOB],
    length: usize,
    bus: Option<Bus>,
    devices: [Option<Device>; MAX_DEVICES],
    count: usize,
}

impl State {
    const fn new() -> Self {
        Self {
            blob: [0; MAX_BLOB],
            length: 0,
            bus: None,
            devices: [None; MAX_DEVICES],
            count: 0,
        }
    }

    fn reset(&mut self) {
        self.length = 0;
        self.bus = None;
        self.devices = [None; MAX_DEVICES];
        self.count = 0;
    }

    fn be32(&self, offset: usize) -> Result<u32> {
        let bytes = self
            .blob
            .get(offset..offset + 4)
            .ok_or(Error::from_status(ddk::EINVAL))?;
        Ok(u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn cstr(&self, offset: usize) -> Result<&CStr> {
        let bytes = self
            .blob
            .get(offset..self.length)
            .ok_or(Error::from_status(ddk::EINVAL))?;
        let length = bytes
            .iter()
            .position(|byte| *byte == 0)
            .ok_or(Error::from_status(ddk::EINVAL))?;
        CStr::from_bytes_with_nul(&bytes[..=length]).map_err(|_| Error::from_status(ddk::EINVAL))
    }

    fn remember(&mut self, device: Device) -> Result<()> {
        if self.count == MAX_DEVICES {
            return Err(Error::from_status(ddk::ENOSPC));
        }
        self.devices[self.count] = Some(device);
        self.count += 1;
        Ok(())
    }
}

static STATE: TicketLock<State> = TicketLock::new(State::new());

const fn align4(value: usize) -> usize {
    (value + 3) & !3
}

fn string_at(state: &State, strings: usize, strings_size: usize, offset: u32) -> Option<usize> {
    let offset = offset as usize;
    if offset >= strings_size {
        return None;
    }
    let start = strings.checked_add(offset)?;
    let end = strings.checked_add(strings_size)?;
    state
        .blob
        .get(start..end)?
        .iter()
        .position(|byte| *byte == 0)
        .map(|_| start)
}

fn property_is(state: &State, name: Option<usize>, expected: &[u8]) -> bool {
    name.and_then(|offset| state.cstr(offset).ok())
        .is_some_and(|name| name.to_bytes() == expected)
}

fn cell_or(state: &State, offset: usize, length: usize, fallback: u32) -> u32 {
    if length < 4 {
        fallback
    } else {
        state.be32(offset).unwrap_or(fallback)
    }
}

fn status_enabled(state: &State, offset: usize, length: usize) -> bool {
    let Some(bytes) = state.blob.get(offset..offset.saturating_add(length)) else {
        return false;
    };
    let length = bytes
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(bytes.len());
    matches!(&bytes[..length], b"ok" | b"okay")
}

fn read_cells(state: &State, offset: usize, length: usize, cells: u32) -> Option<u64> {
    if cells == 0 || cells > 2 || length < cells as usize * 4 {
        return None;
    }
    let mut value = 0u64;
    for index in 0..cells as usize {
        value = (value << 32) | u64::from(state.be32(offset + index * 4).ok()?);
    }
    Some(value)
}

fn publish(state: &mut State, node: Node, parent: Level) -> Result<()> {
    let Some((reg_offset, reg_length)) = node.reg else {
        return Ok(());
    };
    let Some((compatible_offset, compatible_length)) = node.compatible else {
        return Ok(());
    };
    if !node.enabled {
        return Ok(());
    }
    if state.count == MAX_DEVICES {
        return Err(Error::from_status(ddk::ENOSPC));
    }

    let mut strings = [ptr::null::<c_char>(); MAX_STRINGS];
    let mut string_count = 0;
    let mut offset = compatible_offset;
    let end = compatible_offset.saturating_add(compatible_length);
    while offset < end && string_count < MAX_STRINGS {
        let Some(cstr) = state.cstr(offset).ok() else {
            break;
        };
        if cstr.is_empty() {
            break;
        }
        strings[string_count] = cstr.as_ptr();
        string_count += 1;
        offset = offset.saturating_add(cstr.to_bytes_with_nul().len());
    }
    if string_count == 0 {
        return Ok(());
    }

    let address_bytes = parent.address_cells as usize * 4;
    let Some(address) = read_cells(state, reg_offset, reg_length, parent.address_cells) else {
        return Ok(());
    };
    let Some(size) = read_cells(
        state,
        reg_offset + address_bytes,
        reg_length.saturating_sub(address_bytes),
        parent.size_cells,
    ) else {
        return Ok(());
    };
    if size == 0 {
        return Ok(());
    }

    let name = state.cstr(node.name)?;
    let mut builder = DeviceBuilder::new(name)?;
    builder.set_bus(state.bus.ok_or(Error::from_status(ddk::EINVAL))?)?;
    builder.add_strings(c"compatible", &strings[..string_count])?;
    builder.add_resource(RESOURCE_MEMORY, 0, address, size, c"regs")?;
    if node.reg_shift != 0 {
        builder.add_u32(c"reg-shift", node.reg_shift)?;
    }
    if node.reg_width != 0 {
        builder.add_u32(c"reg-io-width", node.reg_width)?;
    }
    if node.clock != 0 {
        builder.add_u32(c"clock-frequency", node.clock)?;
    }

    if let Some((interrupt_offset, interrupt_length)) =
        node.interrupts.filter(|(_, length)| *length >= 4)
    {
        let stride = node.interrupt_cells.max(1) as usize;
        let count = (interrupt_length / 4).min(MAX_CELLS);
        let mut values = [0u64; MAX_CELLS];
        let mut cells = [0u32; MAX_CELLS];
        for index in 0..count {
            cells[index] = state.be32(interrupt_offset + index * 4)?;
            values[index] = cells[index].into();
        }
        for index in (0..count).step_by(stride) {
            if index + stride > count {
                break;
            }
            builder.add_irq(&cells[index..index + stride])?;
        }
        builder.add_u32_list(c"interrupts", &values[..count])?;
    }
    state.remember(builder.publish()?)
}

fn walk(
    state: &mut State,
    structure: usize,
    structure_size: usize,
    strings: usize,
    strings_size: usize,
) -> Result<()> {
    let end = structure
        .checked_add(structure_size)
        .filter(|end| *end <= state.length)
        .ok_or(Error::from_status(ddk::EINVAL))?;
    let mut levels = [Level {
        address_cells: 0,
        size_cells: 0,
    }; MAX_DEPTH];
    levels[0] = Level {
        address_cells: 2,
        size_cells: 1,
    };
    let mut current = Node::empty();
    let mut depth = 0usize;
    let mut offset = structure;

    while offset + 4 <= end {
        let token = state.be32(offset)?;
        offset += 4;
        if token == FDT_END {
            break;
        }
        if token == FDT_NOP {
            continue;
        }
        if token == FDT_BEGIN_NODE {
            let start = offset;
            while offset < end && state.blob[offset] != 0 {
                offset += 1;
            }
            if offset == end {
                return Err(Error::from_status(ddk::EINVAL));
            }
            offset = align4(offset + 1);
            if offset > end || depth + 1 >= MAX_DEPTH {
                return Err(Error::from_status(ddk::EINVAL));
            }
            depth += 1;
            levels[depth] = levels[depth - 1];
            current = Node::empty();
            current.name = start;
            continue;
        }
        if token == FDT_END_NODE {
            if depth == 0 {
                return Err(Error::from_status(ddk::EINVAL));
            }
            publish(state, current, levels[depth - 1])?;
            current = Node::empty();
            depth -= 1;
            continue;
        }
        if token != FDT_PROP || offset + 8 > end {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let length = state.be32(offset)? as usize;
        let name_offset = state.be32(offset + 4)?;
        offset += 8;
        if offset.checked_add(length).is_none_or(|value| value > end) {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let value = offset;
        let name = string_at(state, strings, strings_size, name_offset);
        offset = align4(offset + length);
        if offset > end {
            return Err(Error::from_status(ddk::EINVAL));
        }
        if depth == 0 {
            continue;
        }

        if property_is(state, name, b"#address-cells") {
            levels[depth].address_cells = cell_or(state, value, length, 2);
        } else if property_is(state, name, b"#size-cells") {
            levels[depth].size_cells = cell_or(state, value, length, 1);
        } else if property_is(state, name, b"#interrupt-cells") {
            current.interrupt_cells = cell_or(state, value, length, 1);
        } else if property_is(state, name, b"reg") {
            current.reg = Some((value, length));
        } else if property_is(state, name, b"compatible") {
            current.compatible = Some((value, length));
        } else if property_is(state, name, b"interrupts") {
            current.interrupts = Some((value, length));
        } else if property_is(state, name, b"reg-shift") {
            current.reg_shift = cell_or(state, value, length, 0);
        } else if property_is(state, name, b"reg-io-width") {
            current.reg_width = cell_or(state, value, length, 1);
        } else if property_is(state, name, b"clock-frequency") {
            current.clock = cell_or(state, value, length, 0);
        } else if property_is(state, name, b"status") {
            current.enabled = status_enabled(state, value, length);
        }
    }
    Ok(())
}

fn init(_module: Module) -> Result<()> {
    let mut state = STATE.lock_irqsave();
    state.reset();
    let length = match ddk::firmware_devicetree(&mut state.blob) {
        Ok(length) => length,
        Err(error) if error.status() == ddk::ENOENT => return Ok(()),
        Err(error) => return Err(error),
    };
    if !(FDT_HEADER_SIZE..=MAX_BLOB).contains(&length) || state.be32(0)? != FDT_MAGIC {
        return Err(Error::from_status(ddk::EINVAL));
    }
    state.length = length;
    let total = state.be32(4)? as usize;
    let structure = state.be32(8)? as usize;
    let strings = state.be32(12)? as usize;
    let strings_size = state.be32(32)? as usize;
    let structure_size = state.be32(36)? as usize;
    if total < FDT_HEADER_SIZE
        || total > length
        || structure
            .checked_add(structure_size)
            .is_none_or(|end| end > total)
        || strings
            .checked_add(strings_size)
            .is_none_or(|end| end > total)
    {
        return Err(Error::from_status(ddk::EINVAL));
    }
    state.bus = Some(Bus::find(c"platform")?);
    walk(&mut state, structure, structure_size, strings, strings_size)
}

fn exit(_module: Module) {
    let devices = {
        let mut state = STATE.lock_irqsave();
        let devices = state.devices;
        state.reset();
        devices
    };
    for device in devices.into_iter().flatten().rev() {
        let _ = device.remove();
    }
}

ddk::module!(
    b"fdt\0",
    b"Flattened device-tree platform enumerator\0",
    init,
    exit
);
