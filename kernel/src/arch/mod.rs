//!
//! Architecture specific modules.
//!

#[cfg(target_arch = "x86_64")]
mod x86;

#[cfg(target_arch = "x86_64")]
pub use self::x86::*;
