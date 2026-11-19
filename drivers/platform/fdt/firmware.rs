//! Exports the shared device tree parser used by the platform and PCI drivers.

#![no_std]
extern crate alloc;
mod tree;
pub use tree::*;
