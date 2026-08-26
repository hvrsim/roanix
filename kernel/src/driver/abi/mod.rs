//! The C application binary interface.
//!
//! Modules enter through [`types`] and receive the C-compatible service table
//! in [`api`], which is also the contract documented by the C headers in
//! `drivers/include/roanix/`. Rust modules additionally import a small,
//! versioned set of C ABI shims selected by [`api`]'s curated export resolver;
//! no internal Rust symbol is exposed.

pub mod api;
pub mod events;
pub mod loader;
pub mod types;
