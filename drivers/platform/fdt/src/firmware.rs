//! Shared, allocation-backed firmware decoding without kernel calls.
#![no_std]
extern crate alloc;
mod tree;
pub use tree::*;
