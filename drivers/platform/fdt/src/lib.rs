#![no_std]
#![no_main]
#![allow(unsafe_code)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays
)]
//! Firmware topology with parent-first publication and translated resources.
extern crate alloc;
use alloc::{ffi::CString, vec::Vec};
use ddk::{Bus, DeviceBuilder, DeviceRegistration, Error, Module, Mutex, Result};
use firmware_tree as tree;
use tree::{Space, Tree};
const MAX_BLOB: usize = 2 * 1024 * 1024;
const MAX_CELLS: usize = 1024;
struct State {
    blob: [u8; MAX_BLOB],
    devices: Vec<DeviceRegistration>,
}
static STATE: Mutex<State> = Mutex::new(State {
    blob: [0; MAX_BLOB],
    devices: Vec::new(),
});
fn error(error: tree::Error) -> Error {
    Error::from_status(match error {
        tree::Error::Malformed => ddk::EINVAL,
        tree::Error::Unsupported => ddk::ENOTSUP,
    })
}
fn cells(bytes: &[u8]) -> Result<Vec<u64>> {
    if !bytes.len().is_multiple_of(4) || bytes.len() / 4 > MAX_CELLS {
        return Err(Error::from_status(ddk::EINVAL));
    }
    Ok(bytes
        .chunks_exact(4)
        .map(|part| u64::from(u32::from_be_bytes(part.try_into().expect("cell width"))))
        .collect())
}
struct Controller {
    phandle: u32,
    cells: usize,
    hart: Option<u64>,
}
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
#[allow(clippy::too_many_lines)]
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
            let fallback = CString::new(alloc::format!("regs{reg_index}")).expect("register name");
            let name = reg
                .name
                .unwrap_or(if reg_index == 0 { c"regs" } else { &fallback });
            builder.add_resource(ddk::RESOURCE_MEMORY, 0, physical, reg.size, name)?;
        }
        if unmapped != 0 {
            builder.add_u32(c"roanix,unmapped-reg-count", unmapped)?;
        }
        if let Some(bus_parent) = node.parent {
            if let Some(ranges) = tree.ranges(index).map_err(error)? {
                if ranges.is_empty() {
                    builder.add_u32(c"roanix,ranges-identity", 1)?;
                }
                let mut windows = Vec::new();
                for range in ranges {
                    if range.size == 0 || range.child.space == Space::Configuration {
                        continue;
                    }
                    if let Some(physical) = tree
                        .translate(bus_parent, range.parent, range.size)
                        .map_err(error)?
                    {
                        windows.extend_from_slice(&[
                            u64::from(if range.child.space == Space::Io {
                                ddk::RESOURCE_IO
                            } else {
                                ddk::RESOURCE_MEMORY
                            }),
                            u64::from(range.flags),
                            range.child.value,
                            physical,
                            range.size,
                        ]);
                    }
                }
                builder.add_u64_list(c"roanix,bus-ranges", &windows)?;
            }
        }
        let device = builder.publish()?;
        published[index] = Some(device.device()?);
        devices.push(device);
    }
    Ok(())
}
fn init(_: Module) -> Result<()> {
    let mut state = STATE.lock();
    if !state.devices.is_empty() {
        return Err(Error::from_status(ddk::EBUSY));
    }
    let length = match ddk::firmware_devicetree(&mut state.blob) {
        Ok(length) => length,
        Err(error) if error.status() == ddk::ENOENT => return Ok(()),
        Err(error) => return Err(error),
    };
    let bus = Bus::find(c"platform")?;
    let State { blob, devices } = &mut *state;
    let tree =
        Tree::parse(blob.get(..length).ok_or(Error::from_status(ddk::EINVAL))?).map_err(error)?;
    publish(&tree, bus, devices)
}
fn exit(_: Module) {
    let mut devices = { core::mem::take(&mut STATE.lock().devices) };
    while let Some(mut device) = devices.pop() {
        if device.remove().is_err() {
            devices.push(device);
            STATE.lock().devices.append(&mut devices);
            ddk::log(
                ddk::LOG_ERROR,
                c"firmware device removal refused; retaining publication receipts",
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
