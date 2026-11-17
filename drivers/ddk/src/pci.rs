//! PCI MSI service ABI shared by the bus provider and its consumers.
use crate::{Error, Result, TypedInterface, raw};
use core::ffi::c_void;

pub const MSI_INTERFACE: &core::ffi::CStr = c"pci.msi";

#[repr(C)]
pub struct MsiOps {
    /// Provider table extent, including any compatible appended fields.
    pub size: u32,
    /// Allocates up to `requested` masked routes. The output array must have
    /// `requested` entries; `actual` receives the number successfully enabled.
    pub enable: Option<
        unsafe extern "C" fn(*mut c_void, *const raw::Device, u32, *mut u32, *mut u32) -> i32,
    >,
    /// Enables delivery after the consumer has installed its IRQ handlers.
    pub unmask: Option<unsafe extern "C" fn(*mut c_void, *const raw::Device) -> i32>,
    /// Disables MSI resources. Consumers must first stop interrupt generation
    /// and release their IRQ actions. The device and binding must remain live.
    pub disable: Option<unsafe extern "C" fn(*mut c_void, *const raw::Device)>,
}

impl MsiOps {
    /// Retains a v1 provider with the execution/concurrency contract used by
    /// in-tree consumers. Calls can block and must run in thread context.
    pub fn bind() -> Result<TypedInterface<Self>> {
        // SAFETY: pci.msi revision 1 is defined by this shared C-layout table.
        let binding = unsafe {
            TypedInterface::<Self>::bind(
                MSI_INTERFACE,
                1,
                1,
                crate::IFACE_MAY_SLEEP | crate::IFACE_CONCURRENT,
            )
        }?;
        let ops = binding.ops();
        if (ops.size as usize) < size_of::<Self>()
            || ops.enable.is_none()
            || ops.unmask.is_none()
            || ops.disable.is_none()
        {
            return Err(Error::from_status(crate::ENOTSUP));
        }
        Ok(binding)
    }
}

/// Resolves the specified PCI BAR for a consumer requiring MMIO. I/O BARs and
/// implemented BARs without a CPU mapping or assigned address report `Unsupported`; absent BARs
/// report `NotFound`. A later memory BAR can never substitute for `index`.
pub fn memory_bar(device: crate::Device, index: usize) -> Result<crate::Resource> {
    if index >= 6 {
        return Err(Error::from_status(crate::EINVAL));
    }
    let mut bytes = *b"bar0\0";
    bytes[3] += u8::try_from(index).expect("validated BAR index");
    let name = core::ffi::CStr::from_bytes_with_nul(&bytes).expect("BAR name");
    match device.resource_named(crate::RESOURCE_MEMORY, name) {
        Ok(resource) => Ok(resource),
        Err(error) if error.status() == crate::ENOENT => {
            match device.resource_named(crate::RESOURCE_IO, name) {
                Ok(_) => Err(Error::from_status(crate::ENOTSUP)),
                Err(io_error) if io_error.status() == crate::ENOENT => {
                    for property in [c"pci.unassigned-bars", c"pci.unmapped-bars"] {
                        match device.integer(property) {
                            Ok(mask) if mask & (1 << index) != 0 => {
                                return Err(Error::from_status(crate::ENOTSUP));
                            }
                            Ok(_) => {}
                            Err(property_error) if property_error.status() == crate::ENOENT => {}
                            Err(property_error) => return Err(property_error),
                        }
                    }
                    Err(error)
                }
                Err(error) => Err(error),
            }
        }
        Err(error) => Err(error),
    }
}
