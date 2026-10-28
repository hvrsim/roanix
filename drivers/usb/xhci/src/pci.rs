//! A single MSI/MSI-X interrupter on x86; PLIC-routed `INTx` on RISC-V.
#[cfg(target_arch = "x86_64")]
use ddk::{Device, Error, Result, TypedInterface, pci::MsiOps};

#[cfg(target_arch = "x86_64")]
pub struct Msi {
    binding: TypedInterface<MsiOps>,
    device: Device,
    pub virq: u32,
}
// SAFETY: pci.msi is a concurrent interface; this wrapper shares only its
// immutable binding and device handle. Disabling requires exclusive Drop.
#[cfg(target_arch = "x86_64")]
unsafe impl Sync for Msi {}
#[cfg(target_arch = "x86_64")]
impl Msi {
    pub fn new(device: Device) -> Result<Self> {
        let binding = MsiOps::bind()?;
        let ops = binding.ops();
        let mut virq = 0;
        let mut count = 0;
        // SAFETY: the table is retained by binding and outputs have one entry.
        let status = unsafe {
            (ops.enable.expect("checked MSI operation"))(
                binding.context(),
                device.as_raw(),
                1,
                &raw mut virq,
                &raw mut count,
            )
        };
        if status != ddk::OK {
            return Err(Error::from_status(status));
        }
        let result = Self {
            binding,
            device,
            virq,
        };
        if count != 1 {
            return Err(Error::from_status(ddk::EIO));
        }
        Ok(result)
    }
    pub fn unmask(&self) -> Result<()> {
        // SAFETY: binding and device outlive this checked immediate operation.
        let status = unsafe {
            (self.binding.ops().unmask.expect("checked MSI operation"))(
                self.binding.context(),
                self.device.as_raw(),
            )
        };
        crate::controller::status(status)
    }
}
#[cfg(target_arch = "x86_64")]
impl Drop for Msi {
    fn drop(&mut self) {
        // SAFETY: disable is idempotent and the binding still owns the provider.
        unsafe {
            (self.binding.ops().disable.expect("checked MSI operation"))(
                self.binding.context(),
                self.device.as_raw(),
            );
        }
    }
}
