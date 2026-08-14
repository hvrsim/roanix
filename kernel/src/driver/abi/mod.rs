//! The C application binary interface.
//!
//! Modules see the kernel through exactly two things: the entry point signature
//! in [`types`] and the service table in [`api`]. Everything else a driver uses
//! is either inlined by its own compiler or reached through that table, which
//! is why the framework ships no driver support library.

pub mod api;
pub mod events;
pub mod loader;
pub mod types;
