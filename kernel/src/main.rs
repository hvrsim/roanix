#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![doc = include_str!("../../README.md")]

extern crate alloc;

use limine::{
    BaseRevision,
    request::{RequestsEndMarker, RequestsStartMarker},
};
use log::info;

pub mod arch;
pub mod dev;
pub mod fs;
pub mod mem;
pub mod proc;
pub mod sys;

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static BASE_REVISION: BaseRevision = BaseRevision::with_revision(5);

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests_start_marker")]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests_end_marker")]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

/// Kernel entrypoint.
///
/// Validates the boot protocol and enters early kernel initialization.
#[unsafe(no_mangle)]
unsafe extern "C" fn rmain() -> ! {
    assert!(BASE_REVISION.is_supported());
    early_init()
}

/// Initializes the facilities required to start the first kernel thread.
fn early_init() -> ! {
    sys::debug::register();
    arch::init_boot_cpu();
    info!("welcome to roanix!");
    sys::initramfs::init();

    mem::init();
    arch::init_platform();
    sys::smp::discover();
    sys::sched::bootstrap(init_thread);
    sys::sched::start()
}

/// Completes kernel initialization in scheduled CPU0 thread context.
fn init_thread() {
    sys::smp::start_secondary_cpus();
    mem::start_page_daemon();
    fs::init();
    sys::initramfs::populate().expect("boot: failed to import initramfs");
    proc::spawn_init().expect("boot: failed to start /sbin/init");
    info!("boot: initialization complete");
}
