#![no_std]
#![no_main]

//!
//! # Roanix Kernel Documentation
//!
//! This is the top level of the rustdoc generated kernel documentation. For
//! a more general guide to Roanix, check out [Roanix Internals](https://example.com).
//!
//! # Project Orginization
//!
//! Roanix code is split into modules, each representing a logical kernel subsystem.
//! Currently implemented top level modules are as follows:
//!
//! - `arch` - CPU architecture specific code.
//! - `sys` - core kernel components, basis for the rest of the kernel.
//!

use limine::request::{RequestsEndMarker, RequestsStartMarker};
use limine::BaseRevision;
use log::info;

pub mod arch;
pub mod sys;

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static BASE_REVISION: BaseRevision = BaseRevision::with_revision(3);

#[used]
#[doc(hidden)]
#[link_section = ".requests_start_marker"]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();

#[used]
#[doc(hidden)]
#[link_section = ".requests_end_marker"]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

/// Kernel Entrypoint.
///
/// Ensures that the limine protocol matches the requested version (3), then
/// calls initializers for the various kernel subsystems. Finishes with a
/// polling loop, waiting for the scheduler to active and switch to the
/// init thread.
#[no_mangle]
unsafe extern "C" fn rmain() -> ! {
    assert!(BASE_REVISION.is_supported());

    sys::debug::register();
    info!("main: welcome to roanix!");

    arch::early();
    arch::hcf();
}
