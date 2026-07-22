//! ACPI firmware discovery for x86_64.

use alloc::sync::Arc;
use core::slice;

use limine::request::RsdpRequest;

use super::{
    BusId, Error, KERNEL_DRIVER, Result,
    resource::{ResourceFlags, ResourceKey, ResourceValue},
};

const RSDP_V1_SIZE: usize = 20;
const RSDP_V2_MIN_SIZE: usize = 36;
const RSDP_MAX_SIZE: usize = 4096;

/// Resource containing the validated raw ACPI RSDP bytes.
pub const RSDP_RESOURCE: ResourceKey = ResourceKey::new(0x524F_414E_4958_4657, 1);

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static RSDP_REQUEST: RsdpRequest = RsdpRequest::new();

/// Publishes the bootloader-provided RSDP on the ACPI bus.
pub(super) fn publish_rsdp(bus: BusId) -> Result<()> {
    let response = RSDP_REQUEST.get_response().ok_or(Error::NotFound)?;
    let pointer = response.address() as *const u8;
    // SAFETY: Limine guarantees that the returned RSDP address remains mapped
    // for the kernel lifetime. `rsdp_length` validates the fixed header before
    // the computed length is used.
    let length = unsafe { rsdp_length(pointer)? };
    // SAFETY: the validated RSDP length is contained in the persistent ACPI
    // root structure supplied by Limine.
    let bytes: Arc<[u8]> = Arc::from(unsafe { slice::from_raw_parts(pointer, length) });
    super::publish_resource(
        KERNEL_DRIVER,
        bus,
        RSDP_RESOURCE,
        ResourceFlags::CACHEABLE,
        ResourceValue::Data(bytes),
    )?;
    Ok(())
}

/// Validates the RSDP signature and checksums and returns its complete length.
///
/// # Safety
///
/// `pointer` must address at least the ACPI 1.0 RSDP and, for revision 2 or
/// newer, the length reported by its extended header.
unsafe fn rsdp_length(pointer: *const u8) -> Result<usize> {
    if pointer.is_null() {
        return Err(Error::NotFound);
    }
    // SAFETY: guaranteed by the caller for the fixed ACPI 1.0 header.
    let v1 = unsafe { slice::from_raw_parts(pointer, RSDP_V1_SIZE) };
    if &v1[..8] != b"RSD PTR " || checksum(v1) != 0 {
        return Err(Error::InvalidArgument);
    }
    if v1[15] < 2 {
        return Ok(RSDP_V1_SIZE);
    }

    // SAFETY: ACPI revision 2 guarantees the extended fixed header.
    let fixed = unsafe { slice::from_raw_parts(pointer, RSDP_V2_MIN_SIZE) };
    let length = u32::from_le_bytes(
        fixed[20..24]
            .try_into()
            .expect("ACPI RSDP length field width"),
    ) as usize;
    if !(RSDP_V2_MIN_SIZE..=RSDP_MAX_SIZE).contains(&length) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the caller guarantees the complete reported RSDP is readable.
    let complete = unsafe { slice::from_raw_parts(pointer, length) };
    if checksum(complete) != 0 {
        return Err(Error::InvalidArgument);
    }
    Ok(length)
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes
        .iter()
        .fold(0u8, |checksum, byte| checksum.wrapping_add(*byte))
}
