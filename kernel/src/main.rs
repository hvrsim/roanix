#![no_std]
#![no_main]
#![feature(alloc_error_handler, allocator_api, arbitrary_self_types)]
#![doc = include_str!("../../README.md")]

extern crate alloc;

use core::time::Duration;
use limine::{
    BaseRevision,
    request::{RequestsEndMarker, RequestsStartMarker},
};
use log::{error, info};

pub mod arch;
pub mod dev;
pub mod driver;
pub mod fs;
pub mod mem;
pub mod proc;
pub mod sys;
mod syscall;

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
    sys::klog::init();
    arch::init_boot_cpu();
    info!(target: "boot", "welcome to roanix!");
    sys::cmdline::init();
    sys::initramfs::init();

    mem::init();
    driver::init();
    dev::init();
    driver::load_boot_modules().expect("boot: failed to load required boot modules");
    arch::init_platform();
    #[cfg(target_arch = "riscv64")]
    if let Some(source) = sys::firmware::dtb_source() {
        info!(target: "boot", "device tree source: {source}");
    }
    sys::smp::discover();
    sys::sched::bootstrap(init_thread);
    sys::sched::start()
}

/// Completes kernel initialization in scheduled CPU0 thread context.
fn init_thread() {
    sys::smp::start_secondary_cpus();
    mem::start_page_daemon();
    fs::init();
    sys::random::init();
    dev::publish();
    sys::initramfs::populate().expect("boot: failed to import initramfs");
    driver::load_packaged_modules().expect("boot: failed to load packaged driver modules");

    info!(
        target: "boot",
        "initialization complete in {}ms",
        sys::clock::monotonic_ns() / 1_000_000
    );

    match mount_boot_root() {
        Ok(()) => {
            if let Err(error) = proc::spawn_init(false) {
                error!(target: "boot", "cannot start /sbin/init: {error}");
                proc::spawn_init(true).expect("boot: failed to start the serial rescue shell");
            }
        }
        Err(error) => {
            error!(target: "boot", "cannot mount the system root: {error}");
            proc::spawn_init(true).expect("boot: failed to start the serial rescue shell");
        }
    }

    // The console is the user's terminal from here on. Routine kernel chatter
    // would fight with the shell for it, so only failures stay on screen while
    // the full log remains available through /dev/klog and dmesg.
    sys::klog::set_console_level(sys::klog::Level::Error);
}

/// Waits for the configured block device before replacing the bootstrap root.
fn mount_boot_root() -> fs::Result<()> {
    let source = sys::cmdline::get()
        .split(u8::is_ascii_whitespace)
        .rev()
        .find_map(|arg| arg.strip_prefix(b"root="))
        .and_then(|value| core::str::from_utf8(value).ok())
        .filter(|value| !value.is_empty())
        .ok_or(fs::Error::InvalidArgument)?;
    info!(target: "boot", "mounting read-only ext4 root from {source}");
    let name = source.strip_prefix("/dev/").unwrap_or(source);
    for _ in 0..9 {
        match dev::block::find(name) {
            Ok(_) => break,
            Err(driver::Error::NotFound) => sys::clock::sleep(Duration::from_secs(1)),
            Err(error) => return Err(error.into()),
        }
    }
    fs::mount_block(source, b"/", "ext4")
}
