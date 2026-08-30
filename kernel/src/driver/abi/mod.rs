//! The C application binary interface.
//!
//! Modules enter through [`types`] after the loader resolves the versioned C
//! ABI shims defined by [`api`]. No internal Rust symbol is exposed.

pub mod api;
pub mod events;
pub mod loader;
pub mod types;
