//! PCI MSI service ABI shared by the bus provider and its consumers.
use crate::{Error, Result, TypedInterface, raw};
use core::ffi::c_void;

pub const MSI_INTERFACE: &core::ffi::CStr = c"pci.msi";

/// Version-one PCI message interrupt service shared by the bus and its consumers.
#[repr(C)]
pub struct MsiOps {
    /// Provider table extent, including any compatible appended fields.
    pub size: u32,
    /// Allocates up to `requested` masked routes. The output array must have
    /// `requested` entries. `actual` receives the number successfully enabled.
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

        if !binding.ops().is_complete() {
            return Err(Error::from_status(crate::ENOTSUP));
        }

        Ok(binding)
    }

    fn is_complete(&self) -> bool {
        self.size as usize >= size_of::<Self>()
            && self.enable.is_some()
            && self.unmask.is_some()
            && self.disable.is_some()
    }
}

/// Optional affinity extension to the version-one MSI service.
#[repr(C)]
pub struct MsiAffinityOps {
    /// Original table prefix, including the extent of the full table.
    pub base: MsiOps,

    /// Allocates masked routes with one requested logical CPU per vector.
    ///
    /// The CPU and IRQ arrays must each have `requested` entries and must be
    /// disjoint. The provider can return fewer routes, in the supplied order.
    pub enable_with_affinity: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *const raw::Device,
            u32,
            *const u32,
            *mut u32,
            *mut u32,
        ) -> i32,
    >,
}

impl MsiAffinityOps {
    /// Retains a provider that can program per-vector CPU affinity during setup.
    pub fn bind() -> Result<TypedInterface<Self>> {
        // SAFETY: this C-layout extension uses the same immutable v1 prefix.
        let binding = unsafe {
            TypedInterface::<Self>::bind(
                MSI_INTERFACE,
                1,
                1,
                crate::IFACE_MAY_SLEEP | crate::IFACE_CONCURRENT,
            )
        }?;

        let ops = binding.ops();
        if !ops.base.is_complete()
            || (ops.base.size as usize) < size_of::<Self>()
            || ops.enable_with_affinity.is_none()
        {
            return Err(Error::from_status(crate::ENOTSUP));
        }

        Ok(binding)
    }
}

/// Resolves the memory resource for exactly this PCI BAR index.
///
/// I/O BARs and implemented BARs without a CPU mapping or assigned address
/// report `ENOTSUP`. Absent BARs report `ENOENT`.
pub fn memory_bar(device: crate::Device, index: usize) -> Result<crate::Resource> {
    if index >= 6 {
        return Err(Error::from_status(crate::EINVAL));
    }

    let mut bytes = *b"bar0\0";
    bytes[3] += u8::try_from(index).expect("validated BAR index");
    let name = core::ffi::CStr::from_bytes_with_nul(&bytes).expect("BAR name");
    let absent = match device.resource_named(crate::RESOURCE_MEMORY, name) {
        Ok(resource) => return Ok(resource),
        Err(error) if error.status() == crate::ENOENT => error,
        Err(error) => return Err(error),
    };

    match device.resource_named(crate::RESOURCE_IO, name) {
        Ok(_) => return Err(Error::from_status(crate::ENOTSUP)),
        Err(error) if error.status() == crate::ENOENT => {}
        Err(error) => return Err(error),
    }

    for property in [c"pci.unassigned-bars", c"pci.unmapped-bars"] {
        match device.integer(property) {
            Ok(mask) if mask & (1 << index) != 0 => {
                return Err(Error::from_status(crate::ENOTSUP));
            }

            Ok(_) => {}
            Err(error) if error.status() == crate::ENOENT => {}
            Err(error) => return Err(error),
        }
    }

    Err(absent)
}
