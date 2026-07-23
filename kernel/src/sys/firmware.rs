//! Minimal bootloader firmware-data access used before drivers are loaded.

use core::slice;

#[cfg(target_arch = "riscv64")]
use limine::request::DeviceTreeBlobRequest;
#[cfg(target_arch = "x86_64")]
use limine::request::RsdpRequest;

#[cfg(target_arch = "x86_64")]
const RSDP_V1_SIZE: usize = 20;
#[cfg(target_arch = "x86_64")]
const RSDP_V2_MIN_SIZE: usize = 36;
#[cfg(target_arch = "x86_64")]
const RSDP_MAX_SIZE: usize = 4096;

#[cfg(target_arch = "riscv64")]
const FDT_MAGIC: u32 = 0xD00D_FEED;
#[cfg(target_arch = "riscv64")]
const FDT_HEADER_LEN: usize = 40;

#[cfg(target_arch = "x86_64")]
#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static RSDP_REQUEST: RsdpRequest = RsdpRequest::new();

#[cfg(target_arch = "riscv64")]
#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static DEVICE_TREE_BLOB_REQUEST: DeviceTreeBlobRequest = DeviceTreeBlobRequest::new();

/// Returns the validated ACPI RSDP supplied by the bootloader.
#[cfg(target_arch = "x86_64")]
pub fn acpi_rsdp() -> Option<&'static [u8]> {
    let response = RSDP_REQUEST.get_response()?;
    let pointer = response.address() as *const u8;
    // SAFETY: Limine guarantees a persistent RSDP mapping; `rsdp_length`
    // validates the fixed header before trusting the complete length.
    let length = unsafe { rsdp_length(pointer)? };
    // SAFETY: the validated complete RSDP remains mapped for kernel lifetime.
    Some(unsafe { slice::from_raw_parts(pointer, length) })
}

/// Returns the validated flattened device tree supplied by the bootloader.
#[cfg(target_arch = "riscv64")]
pub fn dtb() -> Option<&'static [u8]> {
    let response = DEVICE_TREE_BLOB_REQUEST.get_response()?;
    let pointer = response.dtb_ptr() as *const u8;
    // SAFETY: Limine guarantees a persistent FDT header at this address.
    let length = unsafe { dtb_length(pointer)? };
    // SAFETY: the validated complete FDT remains mapped for kernel lifetime.
    Some(unsafe { slice::from_raw_parts(pointer, length) })
}

/// Returns the platform counter frequency from the boot device tree.
#[cfg(target_arch = "riscv64")]
pub fn timebase_frequency() -> Option<u64> {
    let fdt = fdt::Fdt::new(dtb()?).ok()?;
    let cpus = fdt.find_node("/cpus")?;
    let property = cpus.property("timebase-frequency")?;
    match property.value.len() {
        4 => Some(u32::from_be_bytes(property.value.try_into().ok()?) as u64),
        8 => Some(u64::from_be_bytes(property.value.try_into().ok()?)),
        _ => None,
    }
}

/// Returns whether every boot CPU advertises the `sstc` extension.
#[cfg(target_arch = "riscv64")]
pub fn all_cpus_support_sstc() -> Option<bool> {
    let fdt = fdt::Fdt::new(dtb()?).ok()?;
    let mut any = false;
    for cpu in fdt.cpus() {
        any = true;
        if !cpu_supports_sstc(cpu) {
            return Some(false);
        }
    }
    Some(any)
}

#[cfg(target_arch = "riscv64")]
fn cpu_supports_sstc(cpu: fdt::standard_nodes::Cpu<'_, '_>) -> bool {
    cpu.property("riscv,isa-extensions")
        .map(|property| {
            core::str::from_utf8(property.value)
                .is_ok_and(|value| value.split('\0').any(|extension| extension == "sstc"))
        })
        .or_else(|| {
            cpu.property("riscv,isa")
                .and_then(|property| property.as_str())
                .map(|value| value.split('_').any(|extension| extension == "sstc"))
        })
        .unwrap_or(false)
}

#[cfg(target_arch = "x86_64")]
unsafe fn rsdp_length(pointer: *const u8) -> Option<usize> {
    if pointer.is_null() {
        return None;
    }
    // SAFETY: the bootloader response guarantees the fixed ACPI 1.0 header.
    let v1 = unsafe { slice::from_raw_parts(pointer, RSDP_V1_SIZE) };
    if &v1[..8] != b"RSD PTR " || checksum(v1) != 0 {
        return None;
    }
    if v1[15] < 2 {
        return Some(RSDP_V1_SIZE);
    }
    // SAFETY: ACPI revision 2 guarantees the complete fixed extension.
    let fixed = unsafe { slice::from_raw_parts(pointer, RSDP_V2_MIN_SIZE) };
    let length = u32::from_le_bytes(fixed[20..24].try_into().ok()?) as usize;
    if !(RSDP_V2_MIN_SIZE..=RSDP_MAX_SIZE).contains(&length) {
        return None;
    }
    // SAFETY: the bootloader exposes the full firmware-reported structure.
    let complete = unsafe { slice::from_raw_parts(pointer, length) };
    (checksum(complete) == 0).then_some(length)
}

#[cfg(target_arch = "riscv64")]
unsafe fn dtb_length(pointer: *const u8) -> Option<usize> {
    if pointer.is_null() {
        return None;
    }
    // SAFETY: the bootloader response guarantees at least the fixed FDT prefix.
    let (magic, length) = unsafe {
        (
            pointer.cast::<u32>().read_unaligned(),
            pointer.add(4).cast::<u32>().read_unaligned(),
        )
    };
    if u32::from_be(magic) != FDT_MAGIC {
        return None;
    }
    let length = u32::from_be(length) as usize;
    (length >= FDT_HEADER_LEN).then_some(length)
}

#[cfg(target_arch = "x86_64")]
fn checksum(bytes: &[u8]) -> u8 {
    bytes
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}
