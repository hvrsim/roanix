#![no_std]
//! USB host service ABI shared by host controllers and USB class drivers.
use core::{ffi::c_void, mem::size_of};
use ddk::{Device, Error, Result, TypedInterface, raw};

pub const HOST_INTERFACE: &core::ffi::CStr = c"usb.host";
#[repr(C)]
pub struct HostOps {
    pub size: u32,
    pub control: unsafe extern "C" fn(*mut c_void, *const raw::Device, u64, *mut u8, usize) -> i64,
    pub interrupt: unsafe extern "C" fn(*mut c_void, *const raw::Device, *mut u8, usize) -> i64,
    pub cancel: unsafe extern "C" fn(*mut c_void, *const raw::Device),
}

pub struct Host(TypedInterface<HostOps>);
impl Host {
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
        let ops = binding.ops();
        if ops.size < size_of::<HostOps>() as u32 {
            return Err(Error::from_status(ddk::ENOTSUP));
        }
        Ok(Self(binding))
    }
    fn ops(&self) -> &HostOps {
        self.0.ops()
    }
    pub fn control(&self, device: Device, setup: [u8; 8], buffer: &mut [u8]) -> Result<usize> {
        // SAFETY: the device and buffer remain live for the synchronous call.
        let result = unsafe {
            (self.ops().control)(
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
    /// provider admits one transfer per endpoint through its final buffer copy;
    /// an overlapping call on that endpoint returns `EBUSY`. Cancellation or
    /// disconnection returns `ENODEV`; different endpoints progress independently.
    pub fn interrupt(&self, device: Device, buffer: &mut [u8]) -> Result<usize> {
        // SAFETY: the device and buffer remain live until completion/cancellation.
        let result = unsafe {
            (self.ops().interrupt)(
                self.0.context(),
                device.as_raw(),
                buffer.as_mut_ptr(),
                buffer.len(),
            )
        };
        count(result, buffer.len())
    }
    pub fn cancel(&self, device: Device) {
        // SAFETY: the provider retains pending endpoint state until cancellation.
        unsafe { (self.ops().cancel)(self.0.context(), device.as_raw()) };
    }
}
fn count(value: i64, capacity: usize) -> Result<usize> {
    if value < 0 {
        return Err(Error::from_status(value as i32));
    }
    usize::try_from(value)
        .ok()
        .filter(|count| *count <= capacity)
        .ok_or(Error::from_status(ddk::EIO))
}
