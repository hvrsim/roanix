//! Owns the x86 MSI or MSI-X interrupter for the lifetime of an xHCI controller.

#[cfg(target_arch = "x86_64")]
use ddk::{Device, Error, Result, TypedInterface, pci::MsiOps};

/// One masked or enabled message route retained through its PCI provider binding.
#[cfg(target_arch = "x86_64")]
pub struct Msi {
    /// Retained reference to the PCI MSI provider.
    binding: TypedInterface<MsiOps>,

    /// Function that owns the configured message.
    device: Device,

    /// Virtual IRQ used to register the xHCI handler.
    pub virq: u32,
}

// SAFETY: pci.msi is a concurrent interface. This wrapper shares only its
// immutable binding and device handle. Disabling requires exclusive Drop.
#[cfg(target_arch = "x86_64")]
unsafe impl Sync for Msi {}
#[cfg(target_arch = "x86_64")]
impl Msi {
    /// Requests one masked message interrupt and retains the PCI provider binding.
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

    /// Enables delivery after the xHCI handler has been installed.
    pub fn unmask(&self) -> Result<()> {
        // SAFETY: binding and device outlive this checked immediate operation.
        let status = unsafe {
            (self.binding.ops().unmask.expect("checked MSI operation"))(
                self.binding.context(),
                self.device.as_raw(),
            )
        };

        if status == ddk::OK {
            Ok(())
        } else {
            Err(Error::from_status(status))
        }
    }
}

#[cfg(target_arch = "x86_64")]
impl Drop for Msi {
    /// Disables the controller's message interrupt before giving up the PCI binding.
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
