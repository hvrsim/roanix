//!
//! # Flattened Device Tree
//!
//! Minimal DTB accessors used during early platform discovery.
//!

use core::slice;

use limine::request::DeviceTreeBlobRequest;

use super::ResourceKey;

const FDT_MAGIC: u32 = 0xD00D_FEED;
const FDT_HEADER_LEN: usize = 40;

/// Resource containing the validated raw flattened device tree.
pub const DTB_RESOURCE: ResourceKey = ResourceKey::new(0x524F_414E_4958_4657, 2);

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static DEVICE_TREE_BLOB_REQUEST: DeviceTreeBlobRequest = DeviceTreeBlobRequest::new();

/// Returns the raw DTB bytes provided by Limine.
///
/// Limine owns the blob for the kernel's lifetime. The header is checked before
/// the firmware-provided total size is used to construct the slice.
pub fn blob() -> Option<&'static [u8]> {
    let response = DEVICE_TREE_BLOB_REQUEST.get_response()?;
    let ptr = response.dtb_ptr() as *const u8;
    // SAFETY: the Limine response guarantees that `dtb_ptr` addresses a
    // persistent flattened-device-tree header.
    let len = unsafe { dtb_len(ptr)? };
    // SAFETY: Limine guarantees the complete DTB remains mapped, and `dtb_len`
    // rejected invalid magic and undersized headers.
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

/// Reads and validates the fixed portion of a flattened-device-tree header.
///
/// # Safety
///
/// `ptr` must be readable for at least eight bytes.
unsafe fn dtb_len(ptr: *const u8) -> Option<usize> {
    if ptr.is_null() {
        return None;
    }

    // SAFETY: guaranteed by the caller; unaligned reads avoid imposing extra
    // alignment requirements on the firmware blob.
    let (raw_magic, raw_len) = unsafe {
        (
            ptr.cast::<u32>().read_unaligned(),
            ptr.add(4).cast::<u32>().read_unaligned(),
        )
    };
    if u32::from_be(raw_magic) != FDT_MAGIC {
        return None;
    }

    let len = u32::from_be(raw_len) as usize;
    (len >= FDT_HEADER_LEN).then_some(len)
}
