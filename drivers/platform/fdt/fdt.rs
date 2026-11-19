//!
//! # Device Tree Platform Enumeration
//!
//! This module converts the firmware tree into Roanix platform devices. It
//! publishes parents first, translates register and bus windows through the
//! tree, and attaches firmware-qualified interrupt specifiers.
//!
//! The parsed tree borrows the firmware buffer during enumeration. Only
//! device registration receipts survive publication and they are removed
//! in reverse order.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss
)]
extern crate alloc;
use alloc::{ffi::CString, vec::Vec};
use ddk::{Bus, DeviceBuilder, DeviceRegistration, Error, Module, Mutex, Result};
use firmware_tree as tree;
use tree::{Node, Space, Tree};
const MAX_BLOB: usize = 2 * 1024 * 1024;
const MAX_CELLS: usize = 1024;
static DEVICES: Mutex<Vec<DeviceRegistration>> = Mutex::new(Vec::new());
/// Converts a device-tree decode failure to the corresponding driver ABI status.
fn error(error: tree::Error) -> Error {
    Error::from_status(match error {
        tree::Error::Malformed => ddk::EINVAL,
        tree::Error::Unsupported => ddk::ENOTSUP,
    })
}

/// Decodes a bounded property into big-endian 32-bit cells in the DDK's input format.
fn cells(bytes: &[u8]) -> Result<Vec<u64>> {
    if !bytes.len().is_multiple_of(4) || bytes.len() / 4 > MAX_CELLS {
        return Err(Error::from_status(ddk::EINVAL));
    }

    Ok(bytes
        .chunks_exact(4)
        .map(|part| u64::from(u32::from_be_bytes(part.try_into().expect("cell width"))))
        .collect())
}

/// Firmware interrupt controller indexed by its phandle.
struct Controller {
    /// Firmware identity used in interrupt specifiers.
    phandle: u32,

    /// Number of interrupt specifier cells for this controller.
    cells: usize,

    /// CPU hart ID when the controller belongs to a CPU node.
    hart: Option<u64>,
}

/// Indexes interrupt phandles, specifier widths, and parent CPU hart IDs.
fn controllers(tree: &Tree<'_>) -> tree::Result<Vec<Controller>> {
    let mut controllers = Vec::new();
    for node in tree.nodes() {
        if let (Some(phandle), Some(count)) = (node.phandle()?, node.cell(c"#interrupt-cells")?) {
            let hart = if let Some(parent) = node
                .parent
                .filter(|p| tree.nodes()[*p].bytes(c"device_type") == Some(b"cpu\0"))
            {
                tree.registers(parent)?.first().map(|reg| reg.address.value)
            } else {
                None
            };
            controllers.push(Controller {
                phandle,
                cells: count as usize,
                hart,
            });
        }
    }

    Ok(controllers)
}

/// Publishes parallel hart IDs for an interrupt list with CPU-parented controllers.
fn interrupt_harts(
    builder: &mut DeviceBuilder,
    values: &[u64],
    controllers: &[Controller],
) -> Result<()> {
    let mut harts = Vec::new();
    let mut index = 0;
    while index < values.len() {
        let Some(controller) = controllers
            .iter()
            .find(|c| u64::from(c.phandle) == values[index])
        else {
            return Ok(());
        };

        let Some(hart) = controller.hart else {
            return Ok(());
        };

        let end = index
            .checked_add(1 + controller.cells)
            .filter(|end| *end <= values.len())
            .ok_or(Error::from_status(ddk::EINVAL))?;
        harts.push(hart);
        index = end;
    }
    builder.add_u64_list(c"roanix,interrupt-harts", &harts)
}

/// Adds CPU-visible MMIO resources and records how many registers failed translation.
fn add_registers(
    tree: &Tree<'_>,
    index: usize,
    node: &Node<'_>,
    builder: &mut DeviceBuilder,
) -> Result<()> {
    let mut unmapped = 0u32;
    for (reg_index, reg) in tree.registers(index).map_err(error)?.iter().enumerate() {
        if reg.size == 0 || reg.address.space == Space::Configuration {
            continue;
        }

        let Some(physical) = tree
            .translate(node.parent.expect("register bus"), reg.address, reg.size)
            .map_err(error)?
        else {
            unmapped += 1;
            continue;
        };

        let fallback;
        let name = match reg.name {
            Some(name) => name,
            None if reg_index == 0 => c"regs",
            None => {
                fallback = CString::new(alloc::format!("regs{reg_index}")).expect("register name");
                &fallback
            }
        };
        builder.add_resource(ddk::RESOURCE_MEMORY, 0, physical, reg.size, name)?;
    }

    if unmapped != 0 {
        builder.add_u32(c"roanix,unmapped-reg-count", unmapped)?;
    }

    Ok(())
}

/// Publishes each bus window that translates to a CPU address.
fn add_bus_windows(
    tree: &Tree<'_>,
    index: usize,
    node: &Node<'_>,
    builder: &mut DeviceBuilder,
) -> Result<()> {
    let Some(parent) = node.parent else {
        return Ok(());
    };

    let Some(ranges) = tree.ranges(index).map_err(error)? else {
        return Ok(());
    };

    if ranges.is_empty() {
        builder.add_u32(c"roanix,ranges-identity", 1)?;
    }

    let mut windows = Vec::new();
    for range in ranges {
        let kind = match range.child.space {
            Space::Configuration => continue,
            Space::Io => ddk::RESOURCE_IO,
            Space::Unspecified | Space::Native | Space::Memory => ddk::RESOURCE_MEMORY,
        };

        if range.size == 0 {
            continue;
        }

        if let Some(physical) = tree
            .translate(parent, range.parent, range.size)
            .map_err(error)?
        {
            windows.extend_from_slice(&[
                u64::from(kind),
                u64::from(range.flags),
                range.child.value,
                physical,
                range.size,
            ]);
        }
    }
    builder.add_u64_list(c"roanix,bus-ranges", &windows)
}

/// Publishes enabled firmware nodes parent-first with translated resources and properties.
fn publish(tree: &Tree<'_>, bus: Bus, devices: &mut Vec<DeviceRegistration>) -> Result<()> {
    let controllers = controllers(tree).map_err(error)?;
    let mut published = alloc::vec![None; tree.nodes().len()];
    for (index, node) in tree.nodes().iter().enumerate() {
        if !node.enabled {
            continue;
        }

        let name = if node.parent.is_none() {
            c"devicetree"
        } else {
            node.name
        };

        let mut builder = DeviceBuilder::new(name)?;
        builder.set_bus(bus)?;
        builder.add_u32(c"roanix,node-index", index as u32)?;
        if let Some(parent) = node.parent {
            builder.set_parent(published[parent].expect("enabled firmware parent"))?;
        }

        for key in [c"compatible", c"device_type", c"reg-names"] {
            let strings = node.strings(key).map_err(error)?;
            if !strings.is_empty() {
                builder.add_strings(key, &strings)?;
            }
        }

        for key in [
            c"reg-shift",
            c"reg-io-width",
            c"#interrupt-cells",
            c"riscv,ndev",
            c"#address-cells",
            c"#size-cells",
        ] {
            if let Some(value) = node.cell(key).map_err(error)? {
                builder.add_u32(key, value)?;
            }
        }

        let clock = node
            .cell(c"clock-frequency")
            .map_err(error)?
            .filter(|value| *value != 0)
            .or(node.cell(c"clk-fpga").map_err(error)?);
        if let Some(clock) = clock {
            builder.add_u32(c"clock-frequency", clock)?;
        }

        if let Some(phandle) = node.phandle().map_err(error)? {
            builder.add_u32(c"phandle", phandle)?;
        }

        let parent_irq = tree.interrupt_parent(index).map_err(error)?;
        if parent_irq != 0 {
            builder.add_u32(c"interrupt-parent", parent_irq)?;
        }

        if let Some(domain) = node.cell(c"linux,pci-domain").map_err(error)? {
            builder.add_u32(c"linux,pci-domain", domain)?;
        }

        for key in [
            c"reg",
            c"ranges",
            c"interrupt-map",
            c"interrupt-map-mask",
            c"bus-range",
        ] {
            if let Some(bytes) = node.bytes(key) {
                builder.add_u32_list(key, &cells(bytes)?)?;
            }
        }

        if let Some(bytes) = node.bytes(c"interrupts-extended") {
            let values = cells(bytes)?;
            builder.add_u32_list(c"interrupts-extended", &values)?;
            interrupt_harts(&mut builder, &values, &controllers)?;
        }

        if let Some(bytes) = node.bytes(c"interrupts") {
            builder.add_u32_list(c"interrupts", &cells(bytes)?)?;
        }

        for route in tree.device_interrupts(index).map_err(error)? {
            builder.add_firmware_irq(route.phandle, &route.cells)?;
        }
        add_registers(tree, index, node, &mut builder)?;
        add_bus_windows(tree, index, node, &mut builder)?;
        let device = builder.publish()?;
        published[index] = Some(device.device()?);
        devices.push(device);
    }

    Ok(())
}

/// Loads the firmware device-tree blob, decodes it, and publishes its enabled devices.
fn init(_: Module) -> Result<()> {
    let mut blob = Vec::new();
    blob.try_reserve_exact(MAX_BLOB)
        .map_err(|_| Error::from_status(ddk::ENOMEM))?;
    blob.resize(MAX_BLOB, 0);
    let length = match ddk::firmware_devicetree(&mut blob) {
        Ok(length) => length,
        Err(error) if error.status() == ddk::ENOENT => return Ok(()),
        Err(error) => return Err(error),
    };

    let bus = Bus::find(c"platform")?;
    let tree =
        Tree::parse(blob.get(..length).ok_or(Error::from_status(ddk::EINVAL))?).map_err(error)?;
    let mut devices = Vec::new();
    let result = publish(&tree, bus, &mut devices);

    // Keep receipts even when a later node fails to publish.
    *DEVICES.lock() = devices;
    result
}

/// Removes published firmware devices in reverse order.
fn exit(_: Module) {
    let mut devices = { core::mem::take(&mut *DEVICES.lock()) };
    while let Some(mut device) = devices.pop() {
        if device.remove().is_err() {
            devices.push(device);
            DEVICES.lock().append(&mut devices);
            ddk::log(
                ddk::LOG_ERROR,
                c"keeping publication receipts after firmware device removal was refused",
            );
            return;
        }
    }
}

ddk::module!(
    b"fdt\0",
    b"Flattened device-tree platform enumerator\0",
    init,
    exit
);
