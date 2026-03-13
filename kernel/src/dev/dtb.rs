//!
//! # Flattened Device Tree
//!
//! Minimal DTB accessors used during early platform discovery.
//!

use limine::request::DeviceTreeBlobRequest;

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static DEVICE_TREE_BLOB_REQUEST: DeviceTreeBlobRequest = DeviceTreeBlobRequest::new();

/// Returns the raw DTB bytes provided by Limine.
pub fn blob() -> Option<&'static [u8]> {
    let response = DEVICE_TREE_BLOB_REQUEST.get_response()?;
    let ptr = response.dtb_ptr() as *const u8;
    if ptr.is_null() {
        return None;
    }

    let rawsize = unsafe { ptr.wrapping_add(4).cast::<u32>().read_unaligned() };
    Some(unsafe { core::slice::from_raw_parts(ptr, u32::from_be(size_raw) as usize) })
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
