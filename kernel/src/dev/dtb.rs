//!
//! # Flattened Device Tree
//!
//! Minimal DTB accessors used during early platform discovery.
//!

use core::slice;

use limine::request::DeviceTreeBlobRequest;

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static DEVICE_TREE_BLOB_REQUEST: DeviceTreeBlobRequest = DeviceTreeBlobRequest::new();

/// Returns the raw DTB bytes provided by Limine.
pub fn blob() -> Option<&'static [u8]> {
    let response = DEVICE_TREE_BLOB_REQUEST.get_response()?;
    let ptr = response.dtb_ptr() as *const u8;
    let len = dtb_len(ptr)?;
    Some(unsafe { slice::from_raw_parts(ptr, len) })
}

/// Parses the Limine-provided DTB.
pub fn parse() -> Option<fdt::Fdt<'static>> {
    let data = blob()?;
    fdt::Fdt::new(data).ok()
}

/// Returns the platform `timebase-frequency` from `/cpus`, if present.
pub fn timebase_frequency() -> Option<u64> {
    let fdt = parse()?;
    let cpus = fdt.find_node("/cpus")?;
    let prop = cpus.property("timebase-frequency")?;

    match prop.value.len() {
        4 => Some(u32::from_be_bytes(prop.value.try_into().ok()?) as u64),
        8 => Some(u64::from_be_bytes(prop.value.try_into().ok()?)),
        _ => None,
    }
}

/// Returns whether every CPU node advertises the `sstc` extension.
///
/// This checks the preferred `riscv,isa-extensions` string array first, then
/// falls back to the deprecated `riscv,isa` property when needed.
pub fn all_cpus_support_sstc() -> Option<bool> {
    let fdt = parse()?;
    let mut any = false;
    for cpu in fdt.cpus() {
        any = true;
        if !cpu_supports_sstc(cpu) {
            return Some(false);
        }
    }
    Some(any)
}

fn cpu_supports_sstc(cpu: fdt::standard_nodes::Cpu<'_, '_>) -> bool {
    cpu.property("riscv,isa-extensions")
        .map(|p| core::str::from_utf8(p.value).is_ok_and(|s| s.split('\0').any(|x| x == "sstc")))
        .or_else(|| {
            cpu.property("riscv,isa")
                .and_then(|p| p.as_str())
                .map(|s| s.split('_').any(|x| x == "sstc"))
        })
        .unwrap_or(false)
}

fn dtb_len(ptr: *const u8) -> Option<usize> {
    if ptr.is_null() {
        return None;
    }

    let raw_len = unsafe { ptr.add(4).cast::<u32>().read_unaligned() };
    Some(u32::from_be(raw_len) as usize)
}
