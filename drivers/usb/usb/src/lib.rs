//! Shares the USB host service contract and retained bindings with USB class drivers.

#![no_std]
use core::{ffi::c_void, mem::size_of};
use ddk::{Device, Error, Result, TypedInterface, raw};

/// Service name for control and interrupt transfers.
pub const HOST_INTERFACE: &core::ffi::CStr = c"usb.host";

/// Version-one operation table published by a USB host controller.
#[repr(C)]
pub struct HostOps {
    /// Published byte extent of the service table.
    pub size: u32,

    /// Submits a control transfer described by its packed setup packet.
    pub control: unsafe extern "C" fn(*mut c_void, *const raw::Device, u64, *mut u8, usize) -> i64,

    /// Receives one interrupt IN transfer into the supplied buffer.
    pub interrupt: unsafe extern "C" fn(*mut c_void, *const raw::Device, *mut u8, usize) -> i64,

    /// Cancels pending transfers for the supplied interface.
    pub cancel: unsafe extern "C" fn(*mut c_void, *const raw::Device),
}

/// Retained binding that keeps the USB host provider and its operations live.
pub struct Host(TypedInterface<HostOps>);
impl Host {
    /// Retains a USB host provider whose operations may sleep and run concurrently.
    /// # Errors
    /// Returns a binding error or rejects a provider table shorter than the supported ABI.
    pub fn bind() -> Result<Self> {
        // SAFETY: usb.host revision 1 is defined by this shared HostOps table.
        let binding = unsafe {
            TypedInterface::<HostOps>::bind(
                HOST_INTERFACE,
                1,
                1,
                ddk::IFACE_MAY_SLEEP | ddk::IFACE_CONCURRENT,
            )
        }?;

        if binding.ops().size < size_of::<HostOps>() as u32 {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        Ok(Self(binding))
    }

    /// Submits a synchronous USB control transfer with the supplied setup packet.
    /// # Errors
    /// Returns the provider's transfer error or an error for an invalid byte count.
    pub fn control(&self, device: Device, setup: [u8; 8], buffer: &mut [u8]) -> Result<usize> {
        // SAFETY: the device and buffer remain live for the synchronous call.
        let result = unsafe {
            (self.0.ops().control)(
                self.0.context(),
                device.as_raw(),
                u64::from_le_bytes(setup),
                buffer.as_mut_ptr(),
                buffer.len(),
            )
        };
        count(result, buffer.len())
    }

    /// Receives one interrupt IN report, retaining caller buffer ownership until
    /// completion or cancellation. Calls may run concurrently. The xHCI
    /// provider admits one transfer per endpoint through its final buffer copy.
    /// An overlapping call on that endpoint returns `EBUSY`. Cancellation or
    /// disconnection returns `ENODEV`. Different endpoints progress independently.
    pub fn interrupt(&self, device: Device, buffer: &mut [u8]) -> Result<usize> {
        // SAFETY: the device and buffer remain live until completion/cancellation.
        let result = unsafe {
            (self.0.ops().interrupt)(
                self.0.context(),
                device.as_raw(),
                buffer.as_mut_ptr(),
                buffer.len(),
            )
        };
        count(result, buffer.len())
    }

    /// Asks the host to cancel pending transfers for the device.
    pub fn cancel(&self, device: Device) {
        // SAFETY: the provider retains pending endpoint state until cancellation.
        unsafe { (self.0.ops().cancel)(self.0.context(), device.as_raw()) };
    }
}

/// Converts the provider's signed completion value to a bounded byte count.
fn count(value: i64, capacity: usize) -> Result<usize> {
    if value < 0 {
        return Err(Error::from_status(i32::try_from(value).unwrap_or(ddk::EIO)));
    }
    usize::try_from(value)
        .ok()
        .filter(|count| *count <= capacity)
        .ok_or(Error::from_status(ddk::EIO))
}

/// Service name for bulk transfers and halt recovery.
pub const BULK_INTERFACE: &core::ffi::CStr = c"usb.bulk";

/// Maximum payload for one bulk service call.
pub const BULK_MAX_TRANSFER: usize = 64 * 1024;

/// Transfer stopped by the device with a USB STALL handshake.
pub const STALL: i32 = -100;

/// Version-one operations for the bulk endpoint pair published on a USB interface.
#[repr(C)]
pub struct BulkOps {
    /// Published byte extent of the service table.
    pub size: u32,

    /// Receives data from the bulk IN endpoint.
    pub read: unsafe extern "C" fn(*mut c_void, *const raw::Device, *mut u8, usize) -> i64,

    /// Sends data to the bulk OUT endpoint.
    pub write: unsafe extern "C" fn(*mut c_void, *const raw::Device, *const u8, usize) -> i64,

    /// Clears the endpoint halt and resets the host transfer ring.
    pub clear_halt: unsafe extern "C" fn(*mut c_void, *const raw::Device, bool) -> i32,
}

/// Retains the host provider while a class driver uses its paired bulk endpoints.
pub struct Bulk(TypedInterface<BulkOps>);

impl Bulk {
    /// Binds the host bulk service for use in thread context.
    ///
    /// # Errors
    /// Returns the binding error or rejects a truncated service table.
    pub fn bind() -> Result<Self> {
        // SAFETY: usb.bulk revision 1 publishes this shared BulkOps layout.
        let binding = unsafe {
            TypedInterface::<BulkOps>::bind(
                BULK_INTERFACE,
                1,
                1,
                ddk::IFACE_MAY_SLEEP | ddk::IFACE_CONCURRENT,
            )
        }?;

        if binding.ops().size < size_of::<BulkOps>() as u32 {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        Ok(Self(binding))
    }

    /// Receives a bulk payload and returns the actual length, including short packets.
    ///
    /// # Errors
    /// Returns the transfer error or rejects an invalid byte count.
    pub fn read(&self, device: Device, buffer: &mut [u8]) -> Result<usize> {
        // SAFETY: the device and writable buffer stay live for the synchronous call.
        let value = unsafe {
            (self.0.ops().read)(
                self.0.context(),
                device.as_raw(),
                buffer.as_mut_ptr(),
                buffer.len(),
            )
        };
        count(value, buffer.len())
    }

    /// Sends a bulk payload and returns the number of bytes transferred.
    ///
    /// # Errors
    /// Returns the transfer error or rejects an invalid byte count.
    pub fn write(&self, device: Device, buffer: &[u8]) -> Result<usize> {
        // SAFETY: the device and readable buffer stay live for the synchronous call.
        let value = unsafe {
            (self.0.ops().write)(
                self.0.context(),
                device.as_raw(),
                buffer.as_ptr(),
                buffer.len(),
            )
        };
        count(value, buffer.len())
    }

    /// Clears the halt on the bulk IN endpoint when input is true or OUT otherwise.
    ///
    /// # Errors
    /// Returns the host or device recovery error.
    pub fn clear_halt(&self, device: Device, input: bool) -> Result<()> {
        // SAFETY: the binding and device remain live for the synchronous call.
        let status = unsafe { (self.0.ops().clear_halt)(self.0.context(), device.as_raw(), input) };
        if status == ddk::OK {
            Ok(())
        } else {
            Err(Error::from_status(status))
        }
    }
}
