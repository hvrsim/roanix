//! Names that cross the C boundary.
//!
//! A `Box<str>` holds exactly its own bytes with no terminator, so handing its
//! pointer to C and letting the module call `strlen` reads past the end of the
//! allocation. Every name the framework exposes through the ABI is stored here
//! instead, with a trailing NUL, so it is simultaneously a valid Rust string
//! and a valid C string.

use alloc::{boxed::Box, vec::Vec};
use core::ffi::c_char;

/// A name usable from both Rust and C.
pub struct Name {
    /// UTF-8 bytes followed by one NUL terminator.
    bytes: Box<[u8]>,
}

impl Name {
    /// Creates a name from `text`.
    pub fn new(text: &str) -> Self {
        let mut bytes = Vec::with_capacity(text.len() + 1);
        bytes.extend_from_slice(text.as_bytes());
        bytes.push(0);
        Self {
            bytes: bytes.into_boxed_slice(),
        }
    }

    /// Returns the name without its terminator.
    pub fn as_str(&self) -> &str {
        // SAFETY: the buffer was built from a `&str` plus one trailing NUL, so
        // everything before the terminator is the original UTF-8.
        unsafe { core::str::from_utf8_unchecked(&self.bytes[..self.bytes.len() - 1]) }
    }

    /// Returns a NUL-terminated pointer safe to hand to a module.
    pub fn as_c_ptr(&self) -> *const c_char {
        self.bytes.as_ptr().cast()
    }

    /// Returns the name length in bytes, excluding the terminator.
    pub fn len(&self) -> usize {
        self.bytes.len() - 1
    }

    /// Returns whether the name is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl PartialEq<str> for Name {
    fn eq(&self, other: &str) -> bool {
        self.as_str() == other
    }
}

impl core::fmt::Display for Name {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(self.as_str())
    }
}

impl core::fmt::Debug for Name {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.as_str(), formatter)
    }
}
