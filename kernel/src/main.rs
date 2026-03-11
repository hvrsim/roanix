#![no_std]
#![no_main]
#![doc = include_str!("../../README.md")]

use limine::{
    request::{RequestsEndMarker, RequestsStartMarker},
    BaseRevision,
};
use log::info;

pub mod arch;
pub mod dev;
pub mod mem;
pub mod sys;

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static BASE_REVISION: BaseRevision = BaseRevision::with_revision(4);

#[used]
#[doc(hidden)]
#[link_section = ".requests_start_marker"]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();

#[used]
#[doc(hidden)]
#[link_section = ".requests_end_marker"]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

/// Kernel entrypoint.
///
/// Ensures that the limine protocol matches the requested version (4), then
/// calls initializers for the various kernel subsystems. Finishes with a
/// polling loop, waiting for the scheduler to activate and switch to the
/// init thread.
#[no_mangle]
unsafe extern "C" fn rmain() -> ! {
    assert!(BASE_REVISION.is_supported());

    sys::debug::register();
    info!("welcome to roanix!");

    arch::early();
    mem::early();
    arch::init();

    sys::clock::start();
    arch::irqset(true);
    arch::wfi();
}
