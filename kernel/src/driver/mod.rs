//!
//! # Driver Framework
//!
//! Roanix drivers are modules that run in the kernel and may implement any
//! kernel-visible functionality, not just device access. The framework is
//! organized around five ideas:
//!
//! * A **device** is a uniform node in one tree. There is no separate bus node
//!   type, so a bridge, a hub, or a controller is an ordinary device that
//!   happens to have children.
//! * A **bus** describes how a family of devices is enumerated, matched, and
//!   addressed. Buses are registered by modules, so PCI, USB, or I2C support is
//!   added without touching the framework.
//! * A **driver** binds to devices through a match table that understands
//!   device-tree compatible strings, ACPI identifiers, and masked numeric
//!   identifier tables of the kind PCI and USB use.
//! * An **interface** is a versioned operation table published under a name.
//!   It is the only mechanism for one driver to call another, and it has no
//!   ancestry requirement, so any driver can consume any service. A driver that
//!   needs a service which has not appeared yet defers, and the probe engine
//!   retries it when the topology changes.
//! * A **class** groups devices that share a software contract and notifies its
//!   owner as members come and go, which is what allows an entire subsystem to
//!   be implemented as a driver.
//!
//! Handles crossing the C boundary are pointers to reference-counted objects
//! rather than identifiers resolved through a table, so no lock is taken and no
//! lookup is performed to make a framework call. Register access, after the
//! window is mapped, involves no framework call at all.
//!

pub mod abi;
pub mod class;
pub mod core;
pub mod error;
pub mod io;
pub mod irq;
pub mod obj;
pub mod platform;

pub use core::{
    bus::Bus,
    class::{Class, ClassDevice},
    device::{Device, DeviceBuilder},
    driver::Driver,
    iface::{Interface, InterfaceRef},
    module::{Module, ModuleId},
};
pub use error::{Error, Result};

pub mod work;

struct BootModuleSpec {
    name: &'static str,
    path: &'static [u8],
}

// Keep this list deliberately small: these modules must be usable before the
// VFS, scheduler, and architecture platform bring-up are complete.
#[cfg(target_arch = "x86_64")]
const BOOT_MODULES: &[BootModuleSpec] = &[
    BootModuleSpec {
        name: "acpi",
        path: b"usr/lib/roanix/drivers/acpi.ko",
    },
    BootModuleSpec {
        name: "ioapic",
        path: b"usr/lib/roanix/drivers/ioapic.ko",
    },
    BootModuleSpec {
        name: "tmpfs",
        path: b"usr/lib/roanix/drivers/tmpfs.ko",
    },
    BootModuleSpec {
        name: "devfs",
        path: b"usr/lib/roanix/drivers/devfs.ko",
    },
];

#[cfg(target_arch = "riscv64")]
const BOOT_MODULES: &[BootModuleSpec] = &[
    BootModuleSpec {
        name: "fdt",
        path: b"usr/lib/roanix/drivers/fdt.ko",
    },
    BootModuleSpec {
        name: "plic",
        path: b"usr/lib/roanix/drivers/plic.ko",
    },
    BootModuleSpec {
        name: "tmpfs",
        path: b"usr/lib/roanix/drivers/tmpfs.ko",
    },
    BootModuleSpec {
        name: "devfs",
        path: b"usr/lib/roanix/drivers/devfs.ko",
    },
];

/// Initializes the driver framework.
///
/// Runs after memory services are available and before any address space other
/// than the kernel's exists, which is what lets the register window's page
/// tables be shared by every process created later.
pub fn init() {
    obj::init();
    core::init();
    irq::init();
    io::init();
    work::init();
    class::init();
    platform::init();
    // Filesystem providers register through the driver ABI during the early
    // boot-module phase, before VFS mounts its root.
    crate::fs::provider::init();
    crate::fs::provider::init_broker();
}

/// Loads the architecture-specific modules needed during early platform boot.
///
/// The archive is scanned directly, so this path does not depend on the VFS or
/// scheduler. Every listed module is required; a missing or broken image is a
/// boot failure.
pub fn load_boot_modules() -> Result<usize> {
    let Some(files) = crate::sys::initramfs::regular_files() else {
        log::error!("boot: no initramfs archive; cannot load boot modules");
        return Err(Error::NotFound);
    };

    let mut images: [Option<&[u8]>; BOOT_MODULES.len()] = [None; BOOT_MODULES.len()];
    for file in files {
        let (path, bytes) = match file {
            Ok(file) => file,
            Err(error) => {
                log::error!("boot: failed to scan initramfs for boot modules: {error}");
                return Err(Error::InvalidArgument);
            }
        };

        for (index, spec) in BOOT_MODULES.iter().enumerate() {
            if path.as_bytes() != spec.path {
                continue;
            }
            if images[index].replace(bytes).is_some() {
                log::error!("boot: duplicate required boot module {}", spec.name);
                return Err(Error::InvalidArgument);
            }
        }
    }

    let mut loaded = 0usize;
    for (index, spec) in BOOT_MODULES.iter().enumerate() {
        let Some(bytes) = images[index] else {
            log::error!(
                "boot: required {} boot module is absent from initramfs",
                spec.name
            );
            return Err(Error::NotFound);
        };
        match abi::loader::load_bytes(bytes) {
            Ok(module) => {
                log::info!("boot: loaded required module {}", module.name());
                loaded += 1;
            }
            Err(error) => {
                log::error!(
                    "boot: failed to load required {} module: {error:?}",
                    spec.name
                );
                return Err(error);
            }
        }
    }

    Ok(loaded)
}

/// Loads the modules packaged in the initial filesystem.
pub fn load_packaged_modules() -> Result<usize> {
    let loaded = abi::loader::load_directory(b"/usr/lib/roanix/drivers")?;
    log::info!("loaded {loaded} driver module(s)");
    core::probe::report_unbound();
    Ok(loaded)
}
