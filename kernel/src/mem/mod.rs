//!
//! # Memory subsystem
//!
//! This module implements the kernel memory manager, responsible
//! for managing memory resources, swapping to disk and virtual 
//! page mapping.
//!

use limine::{memory_map::EntryType, request::MemoryMapRequest};
use log::{info, trace};

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static MEMORY_MAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

/// Helper to convert [`EntryType`](`EntryType`) to its string representation.
fn etype_to_str(t: EntryType) -> &'static str {
    return match t {
        EntryType::USABLE => "Usable",
        EntryType::RESERVED => "Reserved",
        EntryType::ACPI_RECLAIMABLE => "ACPI (reclaim)",
        EntryType::ACPI_NVS => "ACPI NVS",
        EntryType::BAD_MEMORY => "Bad Memory",
        EntryType::BOOTLOADER_RECLAIMABLE => "Bootloader (reclaim)",
        EntryType::EXECUTABLE_AND_MODULES => "Kernel & Modules",
        EntryType::FRAMEBUFFER => "Framebuffer",
        _ => "???",
    };
}

/// Performs early initalization of the memory subsystem.
///
/// This routine only sets up  components which relate to
/// the mapping of device memory. The rest of the mem module
/// is set up after the kernel scheduler is online.
pub fn early() {
    let resp = MEMORY_MAP_REQUEST.get_response().unwrap();

    let mut usable_mem = 0;

    trace!("mem: memory map structure:");
    for entry in resp.entries() {
        trace!(
            "\t[{:016x}-{:016x}] {}",
            entry.base,
            entry.base + entry.length,
            etype_to_str(entry.entry_type)
        );

        if entry.entry_type == EntryType::USABLE {
            usable_mem += entry.length;
        }
    }

    info!(
        "mem: {} MB of usable RAM detected!",
        usable_mem / 1024 / 1024
    );
}
