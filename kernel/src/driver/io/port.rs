//! Architectural port I/O.
//!
//! Port access exists only on x86. On other architectures the entry points
//! report [`Error::Unsupported`] so a driver that is compiled for both can
//! detect the difference at run time instead of failing to build.

#[cfg(not(target_arch = "x86_64"))]
use super::super::error::Error;
use super::super::error::Result;

/// Reads one byte from an I/O port.
pub fn read8(port: u16) -> Result<u8> {
    #[cfg(target_arch = "x86_64")]
    {
        let value: u8;
        // SAFETY: port reads have no memory effects, and driver access to a
        // port range is the caller's responsibility.
        unsafe {
            core::arch::asm!("in al, dx", out("al") value, in("dx") port,
                options(nomem, nostack, preserves_flags));
        }
        Ok(value)
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = port;
        Err(Error::Unsupported)
    }
}

/// Reads two bytes from an I/O port.
pub fn read16(port: u16) -> Result<u16> {
    #[cfg(target_arch = "x86_64")]
    {
        let value: u16;
        // SAFETY: see [`read8`].
        unsafe {
            core::arch::asm!("in ax, dx", out("ax") value, in("dx") port,
                options(nomem, nostack, preserves_flags));
        }
        Ok(value)
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = port;
        Err(Error::Unsupported)
    }
}

/// Reads four bytes from an I/O port.
pub fn read32(port: u16) -> Result<u32> {
    #[cfg(target_arch = "x86_64")]
    {
        let value: u32;
        // SAFETY: see [`read8`].
        unsafe {
            core::arch::asm!("in eax, dx", out("eax") value, in("dx") port,
                options(nomem, nostack, preserves_flags));
        }
        Ok(value)
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = port;
        Err(Error::Unsupported)
    }
}

/// Writes one byte to an I/O port.
pub fn write8(port: u16, value: u8) -> Result<()> {
    #[cfg(target_arch = "x86_64")]
    {
        // SAFETY: the caller is responsible for the effect of the write on the
        // addressed device.
        unsafe {
            core::arch::asm!("out dx, al", in("dx") port, in("al") value,
                options(nomem, nostack, preserves_flags));
        }
        Ok(())
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (port, value);
        Err(Error::Unsupported)
    }
}

/// Writes two bytes to an I/O port.
pub fn write16(port: u16, value: u16) -> Result<()> {
    #[cfg(target_arch = "x86_64")]
    {
        // SAFETY: see [`write8`].
        unsafe {
            core::arch::asm!("out dx, ax", in("dx") port, in("ax") value,
                options(nomem, nostack, preserves_flags));
        }
        Ok(())
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (port, value);
        Err(Error::Unsupported)
    }
}

/// Writes four bytes to an I/O port.
pub fn write32(port: u16, value: u32) -> Result<()> {
    #[cfg(target_arch = "x86_64")]
    {
        // SAFETY: see [`write8`].
        unsafe {
            core::arch::asm!("out dx, eax", in("dx") port, in("eax") value,
                options(nomem, nostack, preserves_flags));
        }
        Ok(())
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (port, value);
        Err(Error::Unsupported)
    }
}

/// Returns whether this architecture has an I/O port space.
pub const fn available() -> bool {
    cfg!(target_arch = "x86_64")
}
