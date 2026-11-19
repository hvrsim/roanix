//!
//! # Rust Driver Development Kit
//!
//! The DDK wraps the kernel services used by loadable Rust drivers, including
//! device registration, interrupts, MMIO, DMA, and worker threads. Only the
//! [`raw`] records and callbacks cross the module boundary.
//!
//! Registration receipts keep track of resources that need checked cleanup.
//! A non-owning handle identifies a kernel object without extending its
//! lifetime. Keep callback context valid until its registration is withdrawn
//! and in-flight work has drained.
//!
//! Use [`Mutex`] for thread-context operations that can sleep. Use
//! [`TicketLock`] for state shared with hard interrupts, and take owned
//! resources out of a ticket lock before operations that can block.
//!

#![no_std]
#![allow(unsafe_code)]
#![deny(unsafe_op_in_unsafe_fn)]
// The DDK intentionally mirrors a C ABI: many tiny status-returning wrappers
// and checked pointer adapters would otherwise repeat boilerplate docs.
#![allow(
    clippy::cast_possible_truncation,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::not_unsafe_ptr_arg_deref
)]

mod mutex;
pub use mutex::{Mutex, MutexGuard};
pub mod pci;

extern crate alloc;

#[cfg(feature = "module-build")]
extern crate std;

use core::{
    alloc::{GlobalAlloc, Layout},
    cell::UnsafeCell,
    ffi::{CStr, c_void},
    marker::PhantomData,
    ops::{Deref, DerefMut},
    ptr::{self, NonNull},
    sync::atomic::{AtomicPtr, AtomicU32, AtomicU64, Ordering},
};

/// Configures Cargo's final link for a loadable Roanix driver module.
#[cfg(feature = "module-build")]
pub fn configure_module_link() {
    let name = std::env::var("CARGO_PKG_NAME").expect("Cargo must provide the package name");
    configure_module_linker_flags();
    std::println!("cargo::rustc-link-arg-bins=-soname={name}.ko");
}

/// Configures Cargo's final links for multiple loadable driver binaries in one package.
#[cfg(feature = "module-build")]
pub fn configure_module_links(binary_names: &[&str]) {
    configure_module_linker_flags();
    for name in binary_names {
        std::println!("cargo::rustc-link-arg-bin={name}=-soname={name}.ko");
    }
}

/// Emits the linker options common to all loadable driver binaries.
#[cfg(feature = "module-build")]
fn configure_module_linker_flags() {
    for argument in [
        "-no-pie",
        "-shared",
        "-Bsymbolic",
        "--allow-shlib-undefined",
        "--gc-sections",
        "--build-id=none",
        "--strip-debug",
        "--hash-style=sysv",
        "-znow",
        "-zrelro",
        "-zseparate-code",
        "-znoexecstack",
        "-zmax-page-size=4096",
        "-erdf_module_entry",
    ] {
        std::println!("cargo::rustc-link-arg-bins={argument}");
    }
}

mod imports {
    //! Kernel ABI imports. The wrappers in this crate supply their contracts.

    use core::ffi::{c_char, c_void};

    use crate::raw::{self, BlockOps, Module, NodeOps};

    macro_rules! kernel_imports {
        ($(fn $name:ident($($argument:ty),*) $(-> $result:ty)?;)*) => {
            unsafe extern "C" {
                $(
                    #[link_name = concat!("rdf_api_v2_", stringify!($name))]
                    pub fn $name($(_: $argument),*) $(-> $result)?;
                )*
            }
        };
    }

    kernel_imports! {
        fn alloc(usize, usize) -> *mut c_void;
        fn alloc_zeroed(usize, usize) -> *mut c_void;
        fn bus_find(*const c_char, *mut *const raw::Bus) -> i32;
        fn bus_register(*const raw::Module, *const c_char, *const raw::BusDef, *mut *const raw::Bus) -> i32;
        fn bus_unregister(*const raw::Bus) -> i32;
        fn block_register(*const Module, *const c_char, *const BlockOps, *mut *const raw::BlockDevice) -> i32;
        fn block_unregister(*const raw::BlockDevice) -> i32;
        fn block_rescan(*const raw::BlockDevice) -> i32;
        fn cpu_count() -> u32;
        fn cpu_current() -> u32;
        fn cpu_platform_id(u32, *mut u64) -> i32;
        fn devfs_create(*const Module, u64, *const c_char, u32, u16, *const NodeOps, *mut u64) -> i32;
        fn devfs_mkdir(*const raw::Module, u64, *const c_char, u16, *mut u64) -> i32;
        fn devfs_lookup(*const c_char, *mut u64) -> i32;
        fn devfs_remove(*const Module, u64) -> i32;
        fn devfs_root(*mut u64) -> i32;
        fn device_add(*mut raw::DeviceBuilder, *mut *const raw::Device) -> i32;
        fn device_add_cells(*mut raw::DeviceBuilder, *const c_char, *const u64, usize, u32) -> i32;
        fn device_add_int(*mut raw::DeviceBuilder, *const c_char, u64, u32) -> i32;
        fn device_add_firmware_irq(*mut raw::DeviceBuilder, u32, *const u32, usize) -> i32;
        fn device_add_irq(*mut raw::DeviceBuilder, *const raw::IrqDomain, *const u32, usize) -> i32;
        fn device_add_resource(*mut raw::DeviceBuilder, u32, u32, u64, u64, *const c_char) -> i32;
        fn device_add_strings(*mut raw::DeviceBuilder, *const c_char, *const *const c_char, usize) -> i32;
        fn device_cell(*const raw::Device, *const c_char, usize, *mut u64) -> i32;
        fn device_data(*const raw::Device) -> *mut c_void;
        fn device_discard(*mut raw::DeviceBuilder);
        fn device_int(*const raw::Device, *const c_char, *mut u64) -> i32;
        fn device_new(*const raw::Module, *const c_char, *mut *mut raw::DeviceBuilder) -> i32;
        fn device_property_len(*const raw::Device, *const c_char) -> usize;
        fn device_remove(*const raw::Device) -> i32;
        fn device_resource(*const raw::Device, u32, usize, *mut u64, *mut u64, *mut u32) -> i32;
        fn device_resource_named(*const raw::Device, u32, *const c_char, *mut u64, *mut u64, *mut u32) -> i32;
        fn device_set_bus(*mut raw::DeviceBuilder, *const raw::Bus) -> i32;
        fn device_set_data_checked(*const raw::Device, *mut c_void) -> i32;
        fn device_set_dma_mask(*mut raw::DeviceBuilder, u64) -> i32;
        fn device_set_parent(*mut raw::DeviceBuilder, *const raw::Device) -> i32;
        fn driver_register(*const raw::Module, *const raw::DriverDef, *mut *const raw::Driver) -> i32;
        fn driver_unregister(*const raw::Driver) -> i32;
        fn mutex_create(*const raw::Module, *mut usize) -> i32;
        fn mutex_lock(usize) -> i32;
        fn mutex_try_lock(usize) -> i32;
        fn mutex_unlock(usize) -> i32;
        fn mutex_destroy(usize) -> i32;
        fn event_create(*const raw::Module, *mut usize) -> i32;
        fn event_destroy(usize) -> i32;
        fn event_reset(usize) -> i32;
        fn event_signal(usize) -> i32;
        fn event_wait(usize) -> i32;
        fn event_wait_any(*const usize, usize, *mut usize) -> i32;
        fn event_wait_timeout(usize, u64, *mut u8) -> i32;
        fn free(*mut c_void, usize, usize);
        fn dma_alloc(*const raw::Module, *const raw::Device, usize, usize, u32, *mut raw::Dma) -> i32;
        fn dma_free(*mut raw::Dma) -> i32;
        fn firmware_acpi(*mut u8, usize, *mut usize) -> i32;
        fn firmware_devicetree(*mut u8, usize, *mut usize) -> i32;
        fn iface_publish(*const raw::Module, *const raw::Device, *const c_char, u32, u32, *const c_void, usize, *mut c_void, *mut *const raw::Interface) -> i32;
        fn iface_withdraw(*const raw::Interface) -> i32;
        fn iface_bind_global(*const raw::Module, *const c_char, u32, u32, *mut *mut raw::InterfaceBinding, *mut raw::InterfaceBindingInfo) -> i32;
        fn iface_unbind(*mut raw::InterfaceBinding);
        fn irq_alloc_vector(u32, *mut u32) -> i32;
        fn irq_domain_register(*const raw::Module, *const c_char, u32, u32, *const raw::IrqDomainDef, *mut *const raw::IrqDomain) -> i32;
        fn irq_domain_set_device(*const raw::IrqDomain, *const raw::Device) -> i32;
        fn irq_domain_unregister(*const raw::IrqDomain) -> i32;
        fn irq_map(*const raw::IrqDomain, u64, u32, *mut u32) -> i32;
        fn irq_unmap(*const raw::IrqDomain, u64) -> i32;
        fn irq_free_vector(u32) -> i32;
        fn irq_set_affinity(u32, u32) -> i32;
        fn irq_compose_message(*const raw::IrqDomain, u64, *mut u64, *mut u32) -> i32;
        fn irq_of_device(*const raw::Device, usize, *mut u32) -> i32;
        fn irq_release(*mut raw::Irq) -> i32;
        fn irq_request(*const raw::Module, *const raw::Device, u32, *const c_char, u32, Option<unsafe extern "C" fn(*mut c_void, u32) -> u32>, Option<unsafe extern "C" fn(*mut c_void, u32)>, *mut c_void, *mut *mut raw::Irq) -> i32;
        fn log(u32, *const c_char, *const c_char);
        fn module_name(*const raw::Module) -> *const c_char;
        fn mmio_direct(u64, *mut *mut c_void) -> i32;
        fn mmio_map(*const raw::Module, u64, usize, u32, *mut raw::Mmio) -> i32;
        fn mmio_unmap(*mut raw::Mmio) -> i32;
        fn random_fill(*mut u8, usize);
        fn random_mix(*const u8, usize);
        fn time_monotonic() -> u64;
        fn time_sleep(u64);
        fn tty_register(*const raw::Module, u64, *const c_char, u16, u32, *const raw::ConsoleOps, *mut *mut raw::Tty) -> i32;
        fn tty_unregister(*mut raw::Tty) -> i32;
        fn worker_join(*mut raw::Worker) -> i32;
        fn worker_spawn(*const Module, Option<unsafe extern "C" fn(*mut c_void)>, *mut c_void, *mut *mut raw::Worker) -> i32;
    }
}

/// C-compatible records and opaque handles. These definitions intentionally
/// contain no Rust references, enums, trait objects, or ownership semantics.
/// C-compatible records, callbacks, and opaque identities used by the driver ABI.
pub mod raw {
    use core::ffi::{c_char, c_void};

    pub const ABI_MAJOR: u16 = 2;
    pub const MODULE_DEF_SIZE: u32 = 48;
    /// The original 112-byte prefix remains accepted by the kernel.
    pub const NODE_OPS_LEGACY_SIZE: u32 = 112;
    pub const NODE_OPS_SIZE: u32 = 120;
    pub const CONSOLE_OPS_SIZE: u32 = 160;
    pub const BUS_DEF_SIZE: u32 = 48;
    pub const DRIVER_DEF_SIZE: u32 = 88;
    pub const IRQ_DOMAIN_DEF_SIZE: u32 = 96;
    pub const BLOCK_OPS_SIZE: u32 = 64;

    macro_rules! opaque {
        ($($name:ident),+ $(,)?) => {$(
            #[doc = concat!("Opaque kernel ", stringify!($name), " identity.")]
            #[repr(C)]
            pub struct $name {
                _private: [u8; 0],
            }
        )+};
    }
    opaque!(
        Module,
        Device,
        DeviceBuilder,
        Bus,
        Driver,
        Interface,
        InterfaceBinding,
        IrqDomain,
        Irq,
        Tty,
        BlockDevice,
        Worker,
    );

    /// Optional entry point for a kernel-owned worker thread.
    pub type WorkerEntry = Option<unsafe extern "C" fn(*mut c_void)>;

    /// Metadata returned when a driver retains an interface provider.
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct InterfaceBindingInfo {
        /// Address of the provider's operation table.
        pub ops: *const c_void,

        /// Opaque pointer passed to provider callbacks.
        pub context: *mut c_void,

        /// Published table extent in bytes.
        pub ops_size: usize,

        /// Selected interface revision.
        pub version: u32,

        /// Provider execution and concurrency flags.
        pub flags: u32,
    }

    /// Raw MMIO mapping issued by the kernel.
    #[repr(C)]
    pub struct Mmio {
        /// CPU address of the mapped register window.
        pub base: *mut c_void,

        /// Mapped byte span.
        pub length: usize,

        /// Kernel receipt required to unmap the window.
        pub token: *mut c_void,
    }

    /// Raw coherent DMA allocation issued by the kernel.
    #[repr(C)]
    pub struct Dma {
        /// CPU address of the allocation.
        pub cpu: *mut c_void,

        /// Bus address programmed into the device.
        pub device: u64,

        /// Allocated bytes.
        pub length: usize,

        /// Kernel receipt required to release the allocation.
        pub token: *mut c_void,
    }

    /// A match criterion and the data passed to probe when it matches.
    #[repr(C)]
    pub struct Match {
        /// Selector that determines how the entry matches.
        pub kind: u32,

        /// Matching modifiers interpreted for this selector.
        pub flags: u32,

        /// Property name or other selector-specific string.
        pub key: *const c_char,

        /// String value matched for the relevant selector.
        pub value: *const c_char,

        /// First numeric match value.
        pub id0: u64,

        /// Bits compared in the first numeric value.
        pub mask0: u64,

        /// Second numeric match value.
        pub id1: u64,

        /// Bits compared in the second numeric value.
        pub mask1: u64,

        /// Opaque match data passed to probe.
        pub data: usize,

        /// Priority contribution when this entry matches.
        pub score: i32,
    }

    /// C ABI description copied into a kernel driver registration.
    #[repr(C)]
    pub struct DriverDef {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// NUL-terminated driver name.
        pub name: *const c_char,

        /// Bus the driver matches on, or null for any bus.
        pub bus: *const Bus,

        /// Priority used when ranking otherwise compatible drivers.
        pub priority: i32,

        /// Pointer to the match table copied during registration.
        pub matches: *const Match,

        /// Number of entries at the match table pointer.
        pub match_count: usize,

        /// Attempts to bind a matching device.
        pub probe: Option<unsafe extern "C" fn(*mut c_void, *const Device, usize) -> i32>,

        /// Legacy removal callback for drivers that cannot refuse teardown.
        pub remove: Option<unsafe extern "C" fn(*mut c_void, *const Device)>,

        /// Quiesces a bound device during shutdown.
        pub shutdown: Option<unsafe extern "C" fn(*mut c_void, *const Device)>,

        /// Opaque driver context passed to callbacks.
        pub context: *mut c_void,

        /// Checked removal callback that may refuse to detach a busy device.
        pub try_remove: Option<unsafe extern "C" fn(*mut c_void, *const Device) -> i32>,
    }

    /// Callbacks and context for a registered device bus.
    #[repr(C)]
    pub struct BusDef {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// Optional bus-specific match callback.
        pub match_device:
            Option<unsafe extern "C" fn(*mut c_void, *const Device, *const Driver) -> i32>,

        /// Prepares a device for driver probe.
        pub prepare: Option<unsafe extern "C" fn(*mut c_void, *const Device) -> i32>,

        /// Releases bus-side state after driver use.
        pub cleanup: Option<unsafe extern "C" fn(*mut c_void, *const Device)>,

        /// Quiesces bus-side state during shutdown.
        pub shutdown: Option<unsafe extern "C" fn(*mut c_void, *const Device)>,

        /// Opaque bus context passed to callbacks.
        pub context: *mut c_void,
    }

    /// Controller callbacks used by the kernel to route and deliver interrupts.
    #[repr(C)]
    pub struct IrqDomainDef {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// Converts firmware cells to a hardware interrupt and flags.
        pub translate:
            Option<unsafe extern "C" fn(*mut c_void, *const u32, usize, *mut u64, *mut u32) -> i32>,

        /// Installs controller state for a new virtual IRQ mapping.
        pub setup: Option<unsafe extern "C" fn(*mut c_void, u64, u32, u32) -> i32>,

        /// Releases controller state when the mapping is removed.
        pub teardown: Option<unsafe extern "C" fn(*mut c_void, u64, u32)>,

        /// Disables delivery of the specified hardware source.
        pub mask: Option<unsafe extern "C" fn(*mut c_void, u64)>,

        /// Enables delivery of the specified hardware source.
        pub unmask: Option<unsafe extern "C" fn(*mut c_void, u64)>,

        /// Notifies the controller that handling has completed.
        pub eoi: Option<unsafe extern "C" fn(*mut c_void, u64)>,

        /// Changes the destination CPU of a source.
        pub set_affinity: Option<unsafe extern "C" fn(*mut c_void, u64, u32) -> i32>,

        /// Claims the next interrupt at a root domain.
        pub claim: Option<unsafe extern "C" fn(*mut c_void, u32, u64, *mut u64) -> i32>,

        /// Completes an interrupt claimed by the root domain.
        pub complete: Option<unsafe extern "C" fn(*mut c_void, u32, u64, u64)>,

        /// Returns the address and data for a message interrupt.
        pub compose_message:
            Option<unsafe extern "C" fn(*mut c_void, u64, *mut u64, *mut u32) -> i32>,

        /// Opaque controller context passed to callbacks.
        pub context: *mut c_void,
    }

    /// Process and job-control identity supplied to a device ioctl.
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct IoctlIdentity {
        /// ID of the calling process.
        pub process: u64,

        /// Process group of the caller.
        pub group: i32,

        /// Session of the caller.
        pub session: i32,

        /// Nonzero when the caller leads its session.
        pub session_leader: u8,
    }

    /// Terminal state used by VFS job-control checks.
    ///
    /// This record is intentionally C-compatible and contains no Rust bools.
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct TerminalState {
        /// Session that owns the controlling terminal.
        pub session: i32,

        /// Process group receiving terminal input and signals.
        pub foreground_group: i32,

        /// Nonzero when background output is stopped.
        pub stop_background_output: u8,

        /// Reserved bytes for the public ABI.
        pub reserved: [u8; 3],
    }

    /// Callbacks for a devfs node and readiness events for its open files.
    #[derive(Clone, Copy)]
    #[repr(C)]
    pub struct NodeOps {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// Opaque node context passed to callbacks.
        pub context: *mut c_void,

        /// Opens a file and optionally sets its file context.
        pub open: Option<unsafe extern "C" fn(*mut c_void, u32, *mut usize) -> i32>,

        /// Closes a file with the context supplied by open.
        pub close: Option<unsafe extern "C" fn(*mut c_void, usize, u32)>,

        /// Returns the initial position for a new file.
        pub initial_offset: Option<unsafe extern "C" fn(*mut c_void, usize, u32) -> i64>,

        /// Copies bytes to the supplied kernel buffer.
        pub read: Option<unsafe extern "C" fn(*mut c_void, usize, u64, *mut u8, usize, u32) -> i64>,

        /// Consumes bytes from the supplied kernel buffer.
        pub write:
            Option<unsafe extern "C" fn(*mut c_void, usize, u64, *const u8, usize, u32) -> i64>,

        /// Returns the size reported by this node.
        pub size_bytes: Option<unsafe extern "C" fn(*mut c_void) -> u64>,

        /// Requests that buffered writes be made durable.
        pub sync: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Reports ready events for an open file.
        pub poll: Option<unsafe extern "C" fn(*mut c_void, usize, u64, u16, u32) -> i64>,

        /// Handles a device-specific control operation.
        pub ioctl: Option<
            unsafe extern "C" fn(
                *mut c_void,
                usize,
                *const IoctlIdentity,
                u64,
                u64,
                *mut u8,
                usize,
            ) -> i64,
        >,

        /// Returns an event ID suitable for waiting on read readiness.
        pub readable_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,

        /// Returns an event ID suitable for waiting on write readiness.
        pub writable_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,

        /// Returns an event ID suitable for waiting on disconnection.
        pub hangup_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
        /// Appended in ABI minor 1. Old 112-byte tables leave this absent.
        pub terminal_state: Option<unsafe extern "C" fn(*mut c_void, *mut TerminalState) -> i32>,
    }

    /// Logical-block callbacks supplied by a hardware driver.
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct BlockOps {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// Block device flags understood by the kernel.
        pub flags: u32,

        /// Opaque namespace context passed to callbacks.
        pub context: *mut c_void,

        /// Bytes per logical block.
        pub logical_block_size: u32,

        /// Maximum logical blocks per request.
        pub max_transfer_blocks: u32,

        /// Total addressable logical blocks.
        pub block_count: u64,

        /// Reads consecutive logical blocks into caller storage.
        pub read: Option<unsafe extern "C" fn(*mut c_void, u64, u32, *mut u8) -> i32>,

        /// Writes consecutive logical blocks from caller storage.
        pub write: Option<unsafe extern "C" fn(*mut c_void, u64, u32, *const u8) -> i32>,

        /// Commits outstanding device writes.
        pub flush: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Notifies the device that a block range is no longer needed.
        pub discard: Option<unsafe extern "C" fn(*mut c_void, u64, u32) -> i32>,
    }

    /// TTY settings passed to a serial backend.
    #[repr(C)]
    pub struct SerialFraming {
        /// Serial line speed in symbols per second.
        pub baud: u32,

        /// Number of data bits in each character.
        pub data_bits: u8,

        /// Number of stop bits in each character.
        pub stop_bits: u8,

        /// Nonzero when parity is enabled.
        pub parity: u8,

        /// Nonzero when enabled parity is odd.
        pub odd_parity: u8,
    }

    /// Callbacks and readiness events provided by a TTY backend.
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct ConsoleOps {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// TTY behavior flags.
        pub flags: u64,

        /// Opaque port context passed to callbacks.
        pub context: *mut c_void,

        /// Activates the underlying backend for a TTY open.
        pub open: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Releases backend state for a TTY close.
        pub close: Option<unsafe extern "C" fn(*mut c_void)>,

        /// Attempts to retrieve one byte without waiting.
        pub try_read: Option<unsafe extern "C" fn(*mut c_void, *mut u8) -> i32>,

        /// Retrieves available input bytes.
        pub read: Option<unsafe extern "C" fn(*mut c_void, *mut u8, usize) -> i64>,

        /// Submits output bytes in the requested blocking mode.
        pub write: Option<unsafe extern "C" fn(*mut c_void, *const u8, usize, u8) -> i32>,

        /// Applies new serial framing.
        pub configure: Option<unsafe extern "C" fn(*mut c_void, *const SerialFraming) -> i32>,

        /// Waits for pending output to drain.
        pub flush: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Discards pending input.
        pub flush_input: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Discards pending output.
        pub flush_output: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Asserts a serial break for the requested duration.
        pub send_break: Option<unsafe extern "C" fn(*mut c_void, u64) -> i32>,

        /// Reports whether the backend accepts more output.
        pub writable: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Reports whether the underlying connection is closed.
        pub hung_up: Option<unsafe extern "C" fn(*mut c_void) -> i32>,

        /// Reports output bytes still queued.
        pub queued_output: Option<unsafe extern "C" fn(*mut c_void) -> i64>,

        /// Releases the backend context at final teardown.
        pub destroy: Option<unsafe extern "C" fn(*mut c_void)>,

        /// Event ID for newly readable input.
        pub readable_event: usize,

        /// Event ID for newly available output space.
        pub writable_event: usize,

        /// Event ID for disconnection.
        pub hangup_event: usize,
    }

    /// Descriptor returned to the loader by a loadable module.
    #[repr(C)]
    pub struct ModuleDef {
        /// Declared extent of the ABI structure in bytes.
        pub size: u32,

        /// Required major driver ABI revision.
        pub abi_major: u16,

        /// Reserved version field.
        pub abi_reserved: u16,

        /// Module flags understood by the loader.
        pub flags: u32,

        /// NUL-terminated module name.
        pub name: *const c_char,

        /// NUL-terminated module description.
        pub description: *const c_char,

        /// Runs after the loader establishes the module identity.
        pub init: Option<unsafe extern "C" fn(*const Module) -> i32>,

        /// Releases driver resources during unload.
        pub exit: Option<unsafe extern "C" fn(*const Module)>,
    }

    // SAFETY: all callback tables and module descriptors used by the DDK are
    // immutable statics. Their callback synchronization is explicit.
    unsafe impl Sync for NodeOps {}
    // SAFETY: block operation tables are immutable after registration and the
    // driver synchronizes their opaque context.
    unsafe impl Sync for BlockOps {}
    // SAFETY: the immutable table delegates synchronization of its context to
    // the owning console backend.
    unsafe impl Sync for ConsoleOps {}
    // SAFETY: console tables are immutable after registration. Their opaque
    // context is synchronized by the owning driver.
    unsafe impl Send for ConsoleOps {}
    // SAFETY: driver definitions are immutable after publication and their
    // callback contexts are synchronized by the driver.
    unsafe impl Sync for DriverDef {}
    // SAFETY: bus definitions are immutable after registration and the bus
    // owner synchronizes every callback context.
    unsafe impl Sync for BusDef {}
    // SAFETY: IRQ domain definitions are immutable after registration and the
    // controller synchronizes its callback context.
    unsafe impl Sync for IrqDomainDef {}
    // SAFETY: module descriptors are immutable statics for the image lifetime.
    unsafe impl Sync for ModuleDef {}
    // SAFETY: match entries contain immutable values and static string pointers.
    unsafe impl Sync for Match {}
    // SAFETY: moving a driver definition does not access its opaque context.
    // Registration requires the driver to uphold callback lifetime and concurrency.
    unsafe impl Send for DriverDef {}
    // SAFETY: see `Sync`. Registration handles callback concurrency.
    unsafe impl Send for BusDef {}

    const _: () = assert!(core::mem::size_of::<InterfaceBindingInfo>() == 32);
    const _: () = assert!(core::mem::size_of::<Mmio>() == 24);
    const _: () = assert!(core::mem::size_of::<Dma>() == 32);
    const _: () = assert!(core::mem::size_of::<Match>() == 72);
    const _: () = assert!(core::mem::size_of::<BusDef>() == BUS_DEF_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<DriverDef>() == DRIVER_DEF_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<IrqDomainDef>() == IRQ_DOMAIN_DEF_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<NodeOps>() == NODE_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<BlockOps>() == BLOCK_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<ConsoleOps>() == CONSOLE_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<SerialFraming>() == 8);
    const _: () = assert!(core::mem::size_of::<IoctlIdentity>() == 24);
    const _: () = assert!(core::mem::size_of::<TerminalState>() == 12);
    const _: () = assert!(core::mem::size_of::<ModuleDef>() == MODULE_DEF_SIZE as usize);
}

pub const OK: i32 = 0;
pub const EINVAL: i32 = -1;
pub const ENOENT: i32 = -2;
pub const EEXIST: i32 = -3;
pub const EKIND: i32 = -4;
pub const EPERM: i32 = -5;
pub const EBUSY: i32 = -6;
pub const ENOTSUP: i32 = -7;
pub const ENOSPC: i32 = -8;
pub const ENOMEM: i32 = -9;
pub const EIO: i32 = -10;
pub const EAGAIN: i32 = -11;
pub const EDEFERRED: i32 = -14;
pub const EINTR: i32 = -12;
pub const ENODEV: i32 = -17;
pub const ENOTTY: i32 = -19;
pub const ENOTDIR: i32 = -22;
pub const EISDIR: i32 = -23;
pub const ENOTEMPTY: i32 = -24;
pub const EXDEV: i32 = -25;
pub const ELOOP: i32 = -26;
pub const ENAMETOOLONG: i32 = -27;
pub const EFBIG: i32 = -28;
pub const EROFS: i32 = -29;
pub const EBADF: i32 = -30;

pub const RESOURCE_MEMORY: u32 = 1;
pub const RESOURCE_IO: u32 = 2;
pub const MMIO_DEVICE: u32 = 0;
pub const IRQ_SHARED: u32 = 1 << 4;
pub const IRQ_EDGE: u32 = 1;
pub const IRQ_LEVEL: u32 = 1 << 1;
pub const IRQ_ACTIVE_HIGH: u32 = 1 << 2;
pub const IRQ_ACTIVE_LOW: u32 = 1 << 3;
pub const IRQ_DOMAIN_ROOT: u32 = 1 << 0;
pub const IRQ_DOMAIN_DEFAULT: u32 = 1 << 2;
pub const IRQ_NONE: u32 = 0;
pub const IRQ_HANDLED: u32 = 1;
/// Ask the kernel to run this action's registered service callback.
pub const IRQ_WAKE_THREAD: u32 = 1 << 1;
pub const IRQ_RESCHEDULE: u32 = 1 << 2;
pub const LOG_ERROR: u32 = 1;
pub const LOG_WARN: u32 = 2;
pub const LOG_INFO: u32 = 3;
pub const IFACE_MAY_SLEEP: u32 = 1 << 1;
pub const IFACE_CONCURRENT: u32 = 1 << 2;
pub const IFACE_SINGLETON: u32 = 1 << 3;
pub const MATCH_COMPATIBLE: u32 = 1;
pub const MATCH_ID: u32 = 5;
pub const NODE_CHARACTER: u32 = 1;
pub const DMA_ZERO: u32 = 1;
pub const DMA_ADDRESS32: u32 = 1 << 1;
pub const OPEN_NONBLOCK: u32 = 1 << 8;
pub const POLL_IN: u16 = 0x0001;
pub const POLL_OUT: u16 = 0x0004;
pub const POLL_HUP: u16 = 0x0010;
pub const POLL_NVAL: u16 = 0x0020;
pub const POLL_RDNORM: u16 = 0x0040;
pub const POLL_WRNORM: u16 = 0x0100;
pub const CONSOLE_RESET_ON_LAST_CLOSE: u64 = 1;

/// A status code returned by the driver ABI.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Error(i32);

impl Error {
    /// Wraps the status returned by a kernel or driver callback.
    #[must_use]
    pub const fn from_status(status: i32) -> Self {
        Self(status)
    }

    /// Returns the ABI status code carried by this error.
    #[must_use]
    pub const fn status(self) -> i32 {
        self.0
    }
}

/// The result type used by DDK operations.
pub type Result<T> = core::result::Result<T, Error>;

/// Borrows a nullable ABI input range.
///
/// # Safety
/// A nonempty `pointer` range must remain readable and unaliased from mutable
/// access for the returned lifetime.
pub unsafe fn input_bytes<'a>(pointer: *const u8, length: usize) -> Result<&'a [u8]> {
    if length == 0 {
        return Ok(&[]);
    }

    if pointer.is_null() {
        return Err(Error::from_status(EINVAL));
    }

    // SAFETY: forwarded from this function's contract.
    Ok(unsafe { core::slice::from_raw_parts(pointer, length) })
}

/// Borrows a nullable ABI output range.
///
/// # Safety
/// A nonempty `pointer` range must remain writable and exclusively borrowed
/// for the returned lifetime.
pub unsafe fn output_bytes<'a>(pointer: *mut u8, length: usize) -> Result<&'a mut [u8]> {
    if length == 0 {
        return Ok(&mut []);
    }

    if pointer.is_null() {
        return Err(Error::from_status(EINVAL));
    }

    // SAFETY: forwarded from this function's contract.
    Ok(unsafe { core::slice::from_raw_parts_mut(pointer, length) })
}

/// Allocates a nonzero, monotonically increasing identifier.
///
/// `u64::MAX` is reserved so callers can safely use `id + 1` as an iteration
/// cursor. Once exhausted, the counter remains exhausted and IDs never wrap.
pub fn allocate_monotonic_id(next: &AtomicU64) -> Result<u64> {
    let mut id = next.load(Ordering::Relaxed);
    loop {
        let successor = id
            .checked_add(1)
            .filter(|_| id != 0)
            .ok_or(Error::from_status(ENOSPC))?;
        match next.compare_exchange_weak(id, successor, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return Ok(id),
            Err(observed) => id = observed,
        }
    }
}

fn result(status: i32) -> Result<()> {
    if status == OK {
        Ok(())
    } else {
        Err(Error::from_status(status))
    }
}

static SELF: AtomicPtr<raw::Module> = AtomicPtr::new(ptr::null_mut());

/// The allocator used by loadable Rust drivers.
///
/// Memory always comes from the kernel's driver allocation service. This keeps
/// `alloc` objects inside the module on the same ownership domain as C driver
/// allocations and avoids a Rust runtime allocator dependency.
pub struct KernelAllocator;

// SAFETY: allocations and deallocations are forwarded with the original
// `Layout` to the kernel allocator service, which serializes its own state.
unsafe impl GlobalAlloc for KernelAllocator {
    /// Allocates kernel memory with the global allocator's layout.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let size = layout.size().max(1);
        let align = layout.align();

        // SAFETY: `Layout` supplies a non-zero power-of-two alignment.
        unsafe { imports::alloc(size, align).cast() }
    }

    /// Allocates zeroed kernel memory with the global allocator's layout.
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let size = layout.size().max(1);
        let align = layout.align();

        // SAFETY: `Layout` supplies a non-zero power-of-two alignment.
        unsafe { imports::alloc_zeroed(size, align).cast() }
    }

    /// Frees a global allocation through the kernel using its original layout.
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if pointer.is_null() {
            return;
        }

        let size = layout.size().max(1);
        let align = layout.align();

        // SAFETY: the global allocator receives exactly the layout used for
        // the prior allocation.
        unsafe { imports::free(pointer.cast(), size, align) };
    }
}

#[cfg(target_os = "none")]
#[global_allocator]
static KERNEL_ALLOCATOR: KernelAllocator = KernelAllocator;

/// Opaque identity of a loaded driver module.
#[derive(Clone, Copy)]
pub struct Module(NonNull<raw::Module>);

impl Module {
    /// # Safety
    /// `pointer` must be the live module callback argument supplied by the
    /// loader. This wrapper uses it as an opaque identity.
    pub unsafe fn from_raw(pointer: *const raw::Module) -> Option<Self> {
        NonNull::new(pointer.cast_mut()).map(Self)
    }

    const fn as_ptr(self) -> *const raw::Module {
        self.0.as_ptr().cast_const()
    }
}

/// Returns the driver identity installed by the loader's init trampoline.
fn module() -> Result<Module> {
    let pointer = SELF.load(Ordering::Acquire);

    // SAFETY: `module!` stores only the live loader callback argument.
    unsafe { Module::from_raw(pointer) }.ok_or(Error::from_status(EINVAL))
}

/// Writes one already-formatted message under this module's log name.
pub fn log(level: u32, message: &CStr) {
    let Ok(owner) = module() else {
        return;
    };

    // SAFETY: the module and both C strings remain valid for the call.
    unsafe {
        imports::log(
            level,
            imports::module_name(owner.as_ptr()),
            message.as_ptr(),
        );
    }
}

/// Non-owning identity of a registered device bus.
#[derive(Clone, Copy)]
pub struct Bus(*const raw::Bus);

// SAFETY: this is an opaque, immutable kernel registration handle. The wrapper
// never dereferences it. Synchronization belongs to the kernel bus registry.
unsafe impl Send for Bus {}
// SAFETY: see `Send`. Copying the handle does not grant mutable access.
unsafe impl Sync for Bus {}

impl Bus {
    /// Looks up a bus by name and returns a non-owning handle.
    pub fn find(name: &CStr) -> Result<Self> {
        let mut bus = ptr::null();

        // SAFETY: `name` and `bus` remain valid for this immediate C ABI call.
        unsafe { result(imports::bus_find(name.as_ptr(), &raw mut bus))? };
        NonNull::new(bus.cast_mut())
            .map(|value| Self(value.as_ptr().cast_const()))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Returns the opaque bus identity for an ABI call.
    #[must_use]
    pub const fn as_raw(self) -> *const raw::Bus {
        self.0
    }
}

/// Unique receipt for a bus registered by this module.
#[must_use = "retain this registration and its context until checked cleanup succeeds"]
pub struct BusRegistration(Option<NonNull<raw::Bus>>);

// SAFETY: the receipt is opaque and registration teardown is externally
// serialized by the owning module.
unsafe impl Send for BusRegistration {}

impl BusRegistration {
    /// Registers a bus owned by the current module.
    ///
    /// # Safety
    /// `definition` and all callback targets it contains must remain valid and
    /// immutable until [`BusRegistration::unregister`] completes.
    pub unsafe fn register(name: &CStr, definition: &'static raw::BusDef) -> Result<Self> {
        let mut bus = ptr::null();

        // SAFETY: upheld by this method's contract. The output pointer is
        // writable local storage and the current module is live.
        unsafe {
            result(imports::bus_register(
                module()?.as_ptr(),
                name.as_ptr(),
                definition,
                &raw mut bus,
            ))?;
        }
        NonNull::new(bus.cast_mut())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Returns a non-owning view while this registration receipt is live.
    pub fn bus(&self) -> Result<Bus> {
        self.0
            .map(|value| Bus(value.as_ptr().cast_const()))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Unregisters the bus, keeping the receipt if the kernel refuses.
    pub fn unregister(&mut self) -> Result<()> {
        let Some(bus) = self.0 else {
            return Ok(());
        };

        // SAFETY: the receipt stays owned until unregistration succeeds.
        unsafe { result(imports::bus_unregister(bus.as_ptr().cast_const()))? };
        self.0 = None;
        Ok(())
    }
}

/// Non-owning, revocable device view. Copying it retains neither the device
/// registration nor callback context. Kernel operations validate the identity
/// each time. A retired device reports `ENODEV`.
#[derive(Clone, Copy)]
pub struct Device(*const raw::Device);

// SAFETY: this is an opaque kernel-owned handle. The wrapper only passes it
// back through documented kernel calls, which synchronize device state.
unsafe impl Send for Device {}
// SAFETY: see `Send`. The wrapper creates no Rust reference to kernel device memory.
unsafe impl Sync for Device {}

impl Device {
    /// # Safety
    /// `pointer` must be an identity issued by the kernel. Probe and remove
    /// callback arguments meet this requirement. Keeping the view does not
    /// extend callback context lifetime or prevent identity revocation.
    pub unsafe fn from_raw(pointer: *const raw::Device) -> Option<Self> {
        NonNull::new(pointer.cast_mut()).map(|value| Self(value.as_ptr().cast_const()))
    }

    /// Returns the opaque device identity for an ABI call.
    #[must_use]
    pub const fn as_raw(self) -> *const raw::Device {
        self.0
    }

    /// Reads a named scalar property as a 64-bit unsigned value.
    pub fn integer(self, name: &CStr) -> Result<u64> {
        let mut value = 0;

        // SAFETY: device and property name are kernel/static handles, and
        // `value` is writable for this immediate call.
        unsafe { result(imports::device_int(self.0, name.as_ptr(), &raw mut value))? };
        Ok(value)
    }

    /// Reads one entry of a named numeric property.
    pub fn cell(self, name: &CStr, index: usize) -> Result<u64> {
        let mut value = 0;

        // SAFETY: device and property name are live kernel handles, and
        // `value` is writable for this immediate call.
        unsafe {
            result(imports::device_cell(
                self.0,
                name.as_ptr(),
                index,
                &raw mut value,
            ))?;
        };

        Ok(value)
    }

    /// Returns the length reported by the kernel for a named property.
    #[must_use]
    pub fn property_len(self, name: &CStr) -> usize {
        // SAFETY: device and property name remain valid for the call.
        unsafe { imports::device_property_len(self.0, name.as_ptr()) }
    }

    /// Looks up an indexed resource of the requested kind.
    pub fn resource(self, kind: u32, index: usize) -> Result<Resource> {
        let mut start = 0;
        let mut length = 0;
        let mut flags = 0;

        // SAFETY: all output pointers refer to initialized local storage.
        unsafe {
            result(imports::device_resource(
                self.0,
                kind,
                index,
                &raw mut start,
                &raw mut length,
                &raw mut flags,
            ))?;
        };

        Ok(Resource {
            start,
            length,
            flags,
        })
    }

    /// # Safety
    /// The pointer must be valid until the driver's matching remove callback
    /// clears it and must be synchronized with every callback that reads it.
    pub unsafe fn set_data(self, value: *mut c_void) -> Result<()> {
        // SAFETY: upheld by the caller. The kernel only stores this opaque
        // driver-private pointer.
        unsafe { result(imports::device_set_data_checked(self.0, value)) }
    }

    /// # Safety
    /// The caller must validate the returned opaque pointer before use.
    pub unsafe fn data(self) -> *mut c_void {
        // SAFETY: the kernel returns an opaque pointer without dereferencing it.
        unsafe { imports::device_data(self.0) }
    }

    /// Requests removal through a non-owning view. This can revoke other
    /// views. Publication owners retain their unique cleanup receipt until
    /// removal succeeds or an already-retired identity is acknowledged.
    pub fn remove(self) -> Result<()> {
        // SAFETY: the kernel validates this opaque identity before removal.
        unsafe { result(imports::device_remove(self.0)) }
    }

    /// Finds a resource by its bus/firmware name, preserving indices across kinds.
    pub fn resource_named(self, kind: u32, name: &CStr) -> Result<Resource> {
        let mut resource = Resource {
            start: 0,
            length: 0,
            flags: 0,
        };

        // SAFETY: name is NUL-terminated and output fields are writable.
        unsafe {
            result(imports::device_resource_named(
                self.0,
                kind,
                name.as_ptr(),
                &raw mut resource.start,
                &raw mut resource.length,
                &raw mut resource.flags,
            ))?;
        }

        Ok(resource)
    }
}

/// A named or indexed range published by a bus or firmware enumerator.
pub struct Resource {
    /// First address or index in the range.
    pub start: u64,

    /// Number of bytes or units in the range.
    pub length: u64,

    /// Bus-specific resource attributes.
    pub flags: u32,
}

/// Unique cleanup receipt for a device published by this module.
///
/// Cleanup is explicit: `remove` preserves this receipt on refusal. Dropping a
/// live receipt leaves the registration in the module's kernel registry. This
/// does not prove that associated callback context can be freed.
///
#[must_use = "retain the publication receipt until checked removal succeeds"]
pub struct DeviceRegistration(Option<Device>);

impl DeviceRegistration {
    /// Returns a non-owning view, which may already have been revoked by a
    /// parent removal or module sweep.
    pub fn device(&self) -> Result<Device> {
        self.0.ok_or(Error::from_status(ENODEV))
    }

    /// Removes this publication. Busy/error results retain the receipt, while
    /// an already-revoked identity acknowledges completed ancestor removal.
    pub fn remove(&mut self) -> Result<()> {
        let Some(device) = self.0 else {
            return Ok(());
        };

        match device.remove() {
            Ok(()) => {}
            Err(error) if error.status() == ENODEV => {}
            Err(error) => return Err(error),
        }
        self.0 = None;
        Ok(())
    }
}

/// Unpublished device properties and resources owned until publication.
pub struct DeviceBuilder {
    raw: Option<NonNull<raw::DeviceBuilder>>,
}

impl DeviceBuilder {
    /// Starts building an unpublished device owned by this module.
    pub fn new(name: &CStr) -> Result<Self> {
        let mut builder = ptr::null_mut();

        // SAFETY: module is live, `name` is NUL-terminated, and the output is
        // valid local storage for this immediate ABI call.
        unsafe {
            result(imports::device_new(
                module()?.as_ptr(),
                name.as_ptr(),
                &raw mut builder,
            ))?;
        };
        NonNull::new(builder)
            .map(|raw| Self { raw: Some(raw) })
            .ok_or(Error::from_status(EINVAL))
    }

    fn pointer(&self) -> Result<*mut raw::DeviceBuilder> {
        self.raw
            .map(NonNull::as_ptr)
            .ok_or(Error::from_status(EINVAL))
    }

    /// Selects the bus on which the device will be published.
    pub fn set_bus(&mut self, bus: Bus) -> Result<()> {
        // SAFETY: both opaque handles are valid during the call.
        unsafe { result(imports::device_set_bus(self.pointer()?, bus.as_raw())) }
    }

    /// Sets the device's parent before publication.
    pub fn set_parent(&mut self, parent: Device) -> Result<()> {
        // SAFETY: both opaque handles remain live for this immediate call.
        unsafe { result(imports::device_set_parent(self.pointer()?, parent.as_raw())) }
    }

    /// Sets the DMA address mask the device can accept.
    pub fn set_dma_mask(&mut self, mask: u64) -> Result<()> {
        // SAFETY: the builder is live and the mask is copied by value.
        unsafe { result(imports::device_set_dma_mask(self.pointer()?, mask)) }
    }

    /// Adds a named 32-bit integer property.
    pub fn add_u32(&mut self, name: &CStr, value: u32) -> Result<()> {
        self.add_integer(name, value.into(), false)
    }

    /// Adds a named 64-bit integer property.
    pub fn add_u64(&mut self, name: &CStr, value: u64) -> Result<()> {
        self.add_integer(name, value, true)
    }

    /// Encodes a scalar using the kernel ABI's 32-bit or 64-bit property selector.
    fn add_integer(&mut self, name: &CStr, value: u64, wide: bool) -> Result<()> {
        // SAFETY: builder and property name remain valid for the call. Width
        // one is the public ABI selector for a 64-bit property.
        unsafe {
            result(imports::device_add_int(
                self.pointer()?,
                name.as_ptr(),
                value,
                u32::from(wide),
            ))
        }
    }

    /// Copies the input strings into a named list property.
    pub fn add_strings(&mut self, name: &CStr, values: &[&CStr]) -> Result<()> {
        let pointers: alloc::vec::Vec<_> = values.iter().map(|value| value.as_ptr()).collect();

        // SAFETY: every borrowed C string and the pointer array outlive the call.
        unsafe {
            result(imports::device_add_strings(
                self.pointer()?,
                name.as_ptr(),
                pointers.as_ptr(),
                pointers.len(),
            ))
        }
    }

    /// Adds a list of 32-bit cells using the ABI's 64-bit input slots.
    pub fn add_u32_list(&mut self, name: &CStr, values: &[u64]) -> Result<()> {
        // SAFETY: values has the documented u64 C-ABI representation.
        unsafe {
            result(imports::device_add_cells(
                self.pointer()?,
                name.as_ptr(),
                values.as_ptr(),
                values.len(),
                0,
            ))
        }
    }

    /// Adds a copied list of 64-bit cells, preserving bus/CPU address widths.
    pub fn add_u64_list(&mut self, name: &CStr, values: &[u64]) -> Result<()> {
        // SAFETY: the kernel copies the slice during this immediate call.
        unsafe {
            result(imports::device_add_cells(
                self.pointer()?,
                name.as_ptr(),
                values.as_ptr(),
                values.len(),
                1,
            ))
        }
    }

    /// Adds a named resource range for a matching driver to claim.
    pub fn add_resource(
        &mut self,
        kind: u32,
        flags: u32,
        start: u64,
        length: u64,
        name: &CStr,
    ) -> Result<()> {
        // SAFETY: builder and resource name are valid during this immediate call.
        unsafe {
            result(imports::device_add_resource(
                self.pointer()?,
                kind,
                flags,
                start,
                length,
                name.as_ptr(),
            ))
        }
    }

    /// Adds an interrupt specifier for the default domain to resolve.
    pub fn add_irq(&mut self, cells: &[u32]) -> Result<()> {
        // SAFETY: the kernel copies this firmware specifier before returning.
        unsafe {
            result(imports::device_add_irq(
                self.pointer()?,
                ptr::null(),
                cells.as_ptr(),
                cells.len(),
            ))
        }
    }

    /// Retains a firmware controller identity until that domain is available.
    pub fn add_firmware_irq(&mut self, phandle: u32, cells: &[u32]) -> Result<()> {
        // SAFETY: the builder is live and the kernel copies these cells.
        unsafe {
            result(imports::device_add_firmware_irq(
                self.pointer()?,
                phandle,
                cells.as_ptr(),
                cells.len(),
            ))
        }
    }

    /// Consumes the builder and returns the receipt for the published device.
    pub fn publish(mut self) -> Result<DeviceRegistration> {
        let mut device = ptr::null();
        let builder = self.raw.take().ok_or(Error::from_status(EINVAL))?;

        // SAFETY: the kernel consumes this receipt on success AND failure.
        unsafe { result(imports::device_add(builder.as_ptr(), &raw mut device))? };
        // SAFETY: the service returns a live, kernel-owned device handle.
        unsafe { Device::from_raw(device) }
            .map(|device| DeviceRegistration(Some(device)))
            .ok_or(Error::from_status(EINVAL))
    }
}

impl Drop for DeviceBuilder {
    /// Discards an unpublished builder.
    fn drop(&mut self) {
        let Some(builder) = self.raw.take() else {
            return;
        };

        // SAFETY: this is the unique, unconsumed builder receipt.
        unsafe { imports::device_discard(builder.as_ptr()) };
    }
}

/// Receipt for a published service until it has been withdrawn.
#[must_use = "retain this registration and its context until checked cleanup succeeds"]
pub struct InterfacePublication(Option<NonNull<raw::Interface>>);

// SAFETY: withdrawal is externally serialized and the opaque handle is never
// dereferenced by this wrapper.
unsafe impl Send for InterfacePublication {}

impl InterfacePublication {
    /// Publishes a global interface owned by this module.
    ///
    /// # Safety
    /// `operations` and `context` must remain valid until withdrawal succeeds.
    pub unsafe fn publish(
        name: &CStr,
        version: u32,
        flags: u32,
        operations: *const c_void,
        operations_size: usize,
        context: *mut c_void,
    ) -> Result<Self> {
        let mut interface = ptr::null();

        // SAFETY: the operation table lifetime is guaranteed by the caller and
        // the remaining pointers are valid for this immediate ABI call.
        unsafe {
            result(imports::iface_publish(
                module()?.as_ptr(),
                ptr::null(),
                name.as_ptr(),
                version,
                flags,
                operations,
                operations_size,
                context,
                &raw mut interface,
            ))?;
        }
        NonNull::new(interface.cast_mut())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Withdraws the interface and preserves its receipt if withdrawal fails.
    pub fn withdraw(&mut self) -> Result<()> {
        let Some(interface) = self.0.take() else {
            return Ok(());
        };

        // SAFETY: this uniquely consumes the publication receipt.
        let outcome = unsafe { result(imports::iface_withdraw(interface.as_ptr().cast_const())) };
        if outcome.is_err() {
            self.0 = Some(interface);
        }
        outcome
    }
}

/// A consumer reference that keeps an interface provider loaded.
pub struct InterfaceBinding {
    raw: Option<NonNull<raw::InterfaceBinding>>,
    operations: NonNull<c_void>,
    context: *mut c_void,
    operations_size: usize,
    version: u32,
    flags: u32,
}

// SAFETY: the binding is immutable and unbinding is externally serialized.
unsafe impl Send for InterfaceBinding {}

impl InterfaceBinding {
    /// Legacy selection: revisions are assumed backward compatible through
    /// `u32::MAX`. Prefer a bounded range or `TypedInterface` for new services.
    pub fn bind(name: &CStr, minimum_version: u32) -> Result<Self> {
        Self::bind_versioned(name, minimum_version, u32::MAX)
    }

    /// Selects the newest service revision within an inclusive supported range.
    pub fn bind_versioned(name: &CStr, minimum_version: u32, maximum_version: u32) -> Result<Self> {
        let mut binding = ptr::null_mut();
        let mut info = raw::InterfaceBindingInfo {
            ops: ptr::null(),
            context: ptr::null_mut(),
            ops_size: 0,
            version: 0,
            flags: 0,
        };

        // SAFETY: name is NUL-terminated and both outputs are writable.
        unsafe {
            result(imports::iface_bind_global(
                module()?.as_ptr(),
                name.as_ptr(),
                minimum_version,
                maximum_version,
                &raw mut binding,
                &raw mut info,
            ))?;
        }

        let binding = NonNull::new(binding).ok_or(Error::from_status(EINVAL))?;
        let Some(operations) = NonNull::new(info.ops.cast_mut()) else {
            // SAFETY: release the receipt accompanying an unusable table.
            unsafe { imports::iface_unbind(binding.as_ptr()) };
            return Err(Error::from_status(EINVAL));
        };

        Ok(Self {
            raw: Some(binding),
            operations,
            context: info.context,
            operations_size: info.ops_size,
            version: info.version,
            flags: info.flags,
        })
    }

    /// Returns the published operation table's size in bytes.
    #[must_use]
    pub const fn operations_size(&self) -> usize {
        self.operations_size
    }

    /// Returns the interface revision selected at bind time.
    #[must_use]
    pub const fn version(&self) -> u32 {
        self.version
    }

    /// Returns the provider's execution and concurrency flags.
    #[must_use]
    pub const fn flags(&self) -> u32 {
        self.flags
    }

    /// Returns the raw table address. It may be used only while the binding is active.
    #[must_use]
    pub fn operations(&self) -> *const c_void {
        self.operations.as_ptr().cast_const()
    }

    /// Returns the provider's opaque context for calls through the retained table.
    #[must_use]
    pub fn context(&self) -> *mut c_void {
        self.context
    }

    /// Releases the provider reference. Table and context pointers must no longer be used.
    pub fn unbind(&mut self) {
        if let Some(binding) = self.raw.take() {
            // SAFETY: this uniquely consumes the binding receipt.
            unsafe { imports::iface_unbind(binding.as_ptr()) };
        }
    }
}

impl Drop for InterfaceBinding {
    fn drop(&mut self) {
        self.unbind();
    }
}

/// A retained operation table whose published extent and behavior were checked
/// before constructing a typed reference. Operations may be called only in the
/// execution contexts permitted by the service contract and its published flags.
pub struct TypedInterface<T> {
    binding: InterfaceBinding,
    _table: PhantomData<T>,
}

impl<T> TypedInterface<T> {
    /// # Safety
    /// `name` and the supported revision range must identify immutable tables
    /// with the layout and valid values of `T`, including callback signatures,
    /// whenever the reported byte extent and alignment suffice for `T`.
    /// Metadata checks do not validate arbitrary function pointers or context.
    pub unsafe fn bind(
        name: &CStr,
        minimum: u32,
        maximum: u32,
        required_flags: u32,
    ) -> Result<Self> {
        let binding = InterfaceBinding::bind_versioned(name, minimum, maximum)?;
        if binding.operations_size() < size_of::<T>()
            || !binding.operations().addr().is_multiple_of(align_of::<T>())
            || binding.flags() & required_flags != required_flags
            || !(minimum..=maximum).contains(&binding.version())
        {
            return Err(Error::from_status(ENOTSUP));
        }

        Ok(Self {
            binding,
            _table: PhantomData,
        })
    }

    /// Borrows the checked operation table for the lifetime of the binding.
    #[must_use]
    pub fn ops(&self) -> &T {
        // SAFETY: bind validated extent and alignment. Its unsafe contract validates
        // the table's layout and contents. This borrow retains the provider.
        unsafe { &*self.binding.operations().cast::<T>() }
    }

    /// Returns the provider context to pass to operations on this binding.
    #[must_use]
    pub fn context(&self) -> *mut c_void {
        self.binding.context()
    }

    /// Returns the revision of the operation table.
    #[must_use]
    pub fn version(&self) -> u32 {
        self.binding.version()
    }

    /// Returns the flags advertised by the bound provider.
    #[must_use]
    pub fn flags(&self) -> u32 {
        self.binding.flags()
    }
}

/// Receipt returned by registering a driver with a bus.
#[must_use = "retain this registration and its context until checked cleanup succeeds"]
pub struct DriverRegistration(Option<NonNull<raw::Driver>>);

// SAFETY: registration teardown is externally serialized. This opaque
// receipt never dereferences kernel memory.
unsafe impl Send for DriverRegistration {}

impl DriverRegistration {
    /// Registers a driver and owns the returned receipt.
    ///
    /// The kernel copies the definition and match table before returning.
    ///
    /// # Safety
    /// The definition and its match data must be valid for this call.
    /// Callbacks and their context must remain valid until unregistration
    /// completes.
    pub unsafe fn register(definition: &raw::DriverDef) -> Result<Self> {
        let mut driver = ptr::null();

        // SAFETY: upheld by this method's contract.
        unsafe {
            result(imports::driver_register(
                module()?.as_ptr(),
                definition,
                &raw mut driver,
            ))?;
        };
        NonNull::new(driver.cast_mut())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Unregisters the driver and retains the receipt if cleanup is refused.
    pub fn unregister(&mut self) -> Result<()> {
        let Some(driver) = self.0 else {
            return Ok(());
        };

        // SAFETY: the receipt stays owned until unregistration succeeds.
        unsafe { result(imports::driver_unregister(driver.as_ptr().cast_const()))? };
        self.0 = None;
        Ok(())
    }
}

/// Owned MMIO mapping with checked register bounds and access widths.
pub struct Mmio {
    raw: raw::Mmio,
}

// SAFETY: a mapping receipt owns no Rust references. Moving it between CPUs is
// safe when the driver serializes device register access, as all DDK drivers do.
unsafe impl Send for Mmio {}

impl Mmio {
    /// Maps a physical register range and retains its unmapping receipt.
    pub fn map(physical: u64, length: usize, flags: u32) -> Result<Self> {
        let mut raw = raw::Mmio {
            base: ptr::null_mut(),
            length: 0,
            token: ptr::null_mut(),
        };

        // SAFETY: `raw` is writable output storage and module is live.
        unsafe {
            result(imports::mmio_map(
                module()?.as_ptr(),
                physical,
                length,
                flags,
                &raw mut raw,
            ))?;
        };

        if raw.base.is_null() || raw.token.is_null() || raw.length < length {
            if !raw.token.is_null() {
                // SAFETY: this is the mapping receipt just returned above.
                let _ = unsafe { imports::mmio_unmap(&raw mut raw) };
            }

            return Err(Error::from_status(EIO));
        }

        Ok(Self { raw })
    }

    /// Resolves a physical address in the direct map. Dereferencing it requires a valid range.
    pub fn direct(physical: u64) -> Result<*const u8> {
        let mut output = ptr::null_mut();

        // SAFETY: `output` is writable local storage.
        unsafe { result(imports::mmio_direct(physical, &raw mut output))? };
        NonNull::new(output)
            .map(|value| value.as_ptr().cast_const().cast())
            .ok_or(Error::from_status(EINVAL))
    }

    /// Returns the mapped register span in bytes.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.raw.length
    }

    /// Reports whether the mapping has zero length.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.raw.length == 0
    }

    /// Checks the register's width, bounds, and alignment before volatile access.
    fn address(&self, offset: usize, width: usize) -> *mut u8 {
        assert!(
            offset
                .checked_add(width)
                .is_some_and(|end| end <= self.raw.length)
        );
        // SAFETY: the range assertion bounds this pointer within the mapping.
        let address = unsafe { self.raw.base.cast::<u8>().add(offset) };
        assert!(
            (address as usize).is_multiple_of(width),
            "unaligned MMIO register"
        );
        address
    }

    /// Reads an 8-bit register using a volatile load.
    #[must_use]
    pub fn read8(&self, offset: usize) -> u8 {
        // SAFETY: `address` validates bounds and MMIO must use volatile access.
        unsafe { ptr::read_volatile(self.address(offset, 1)) }
    }

    /// Reads an aligned 16-bit register using a volatile load.
    #[must_use]
    pub fn read16(&self, offset: usize) -> u16 {
        // SAFETY: resource alignment is a hardware contract established by the
        // platform description. The volatile access matches the C driver.
        unsafe { ptr::read_volatile(self.address(offset, 2).cast()) }
    }

    /// Reads an aligned 32-bit register using a volatile load.
    #[must_use]
    pub fn read32(&self, offset: usize) -> u32 {
        // SAFETY: see `read16`. The access width matches the caller's device.
        unsafe { ptr::read_volatile(self.address(offset, 4).cast()) }
    }

    /// Reads an aligned 64-bit register using a volatile load.
    #[must_use]
    pub fn read64(&self, offset: usize) -> u64 {
        // SAFETY: the device resource establishes eight-byte alignment and
        // `address` verifies that the access remains within the mapping.
        unsafe { ptr::read_volatile(self.address(offset, 8).cast()) }
    }

    /// Writes an 8-bit register using a volatile store.
    pub fn write8(&self, offset: usize, value: u8) {
        // SAFETY: `address` validates bounds and MMIO must use volatile access.
        unsafe { ptr::write_volatile(self.address(offset, 1), value) };
    }

    /// Writes an aligned 16-bit register using a volatile store.
    pub fn write16(&self, offset: usize, value: u16) {
        // SAFETY: see `read16`. The access width matches the caller's device.
        unsafe { ptr::write_volatile(self.address(offset, 2).cast(), value) };
    }

    /// Writes an aligned 32-bit register using a volatile store.
    pub fn write32(&self, offset: usize, value: u32) {
        // SAFETY: see `read16`. The access width matches the caller's device.
        unsafe { ptr::write_volatile(self.address(offset, 4).cast(), value) };
    }

    /// Writes an aligned 64-bit register using a volatile store.
    pub fn write64(&self, offset: usize, value: u64) {
        // SAFETY: see `read64`. Volatile access is required for MMIO.
        unsafe { ptr::write_volatile(self.address(offset, 8).cast(), value) };
    }
}

impl Drop for Mmio {
    /// Releases the kernel mapping receipt.
    fn drop(&mut self) {
        if self.raw.token.is_null() {
            return;
        }

        // SAFETY: this mapping receipt is owned uniquely by `self`.
        let _ = unsafe { imports::mmio_unmap(&raw mut self.raw) };
        self.raw.token = ptr::null_mut();
    }
}

/// An owned coherent DMA allocation.
pub struct Dma {
    raw: raw::Dma,
}

// SAFETY: the receipt owns its allocation. Moving it does not create an alias.
// drivers must still synchronize CPU and device access to the contents.
unsafe impl Send for Dma {}

impl Dma {
    /// Allocates a DMA buffer owned by this module with the device's addressing constraints.
    pub fn allocate(device: Option<Device>, size: usize, align: usize, flags: u32) -> Result<Self> {
        if size == 0 || !align.is_power_of_two() {
            return Err(Error::from_status(EINVAL));
        }

        let mut raw = raw::Dma {
            cpu: ptr::null_mut(),
            device: 0,
            length: 0,
            token: ptr::null_mut(),
        };

        // SAFETY: all handles are live and `raw` is writable output storage.
        unsafe {
            result(imports::dma_alloc(
                module()?.as_ptr(),
                device.map_or(ptr::null(), Device::as_raw),
                size,
                align,
                flags,
                &raw mut raw,
            ))?;
        }

        if raw.cpu.is_null() || raw.token.is_null() || raw.length < size {
            if !raw.token.is_null() {
                // SAFETY: this is the receipt just returned above.
                let _ = unsafe { imports::dma_free(&raw mut raw) };
            }

            return Err(Error::from_status(EIO));
        }

        Ok(Self { raw })
    }

    /// Returns the CPU-side address of the DMA buffer.
    #[must_use]
    pub const fn as_ptr(&self) -> *mut u8 {
        self.raw.cpu.cast()
    }

    /// Returns the DMA address to program into the device.
    #[must_use]
    pub const fn device_address(&self) -> u64 {
        self.raw.device
    }

    /// Returns the DMA buffer's length in bytes.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.raw.length
    }

    /// Reports whether the DMA buffer has zero length.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.raw.length == 0
    }

    /// Clears the full DMA buffer. Callers must coordinate with the device before reuse.
    pub fn fill_zero(&mut self) {
        // SAFETY: the allocation is uniquely borrowed and valid for `length`
        // bytes for the lifetime of this receipt.
        unsafe { ptr::write_bytes(self.as_ptr(), 0, self.raw.length) };
    }
}

impl Drop for Dma {
    /// Returns the DMA allocation to the kernel.
    fn drop(&mut self) {
        if self.raw.token.is_null() {
            return;
        }

        // SAFETY: this uniquely consumes the live DMA allocation receipt.
        let _ = unsafe { imports::dma_free(&raw mut self.raw) };
        self.raw.token = ptr::null_mut();
    }
}

/// FIFO spin lock for short critical sections shared with hard interrupts.
pub struct TicketLock<T> {
    /// Ticket number handed to the next waiter.
    next: AtomicU32,

    /// Ticket number admitted to the protected value.
    serving: AtomicU32,

    /// State accessed only by the current ticket holder.
    value: UnsafeCell<T>,
}

// SAFETY: the ticket protocol permits access only through its exclusive guards.
// `IrqGuard` keeps local interrupts disabled. Longer thread operations use Mutex.
unsafe impl<T: Send> Sync for TicketLock<T> {}

impl<T> TicketLock<T> {
    /// Exclusive access before publication or after all users have stopped.
    pub fn get_mut(&mut self) -> &mut T {
        self.value.get_mut()
    }

    /// Creates an unlocked ticket lock around the given value.
    #[must_use]
    pub const fn new(value: T) -> Self {
        Self {
            next: AtomicU32::new(0),
            serving: AtomicU32::new(0),
            value: UnsafeCell::new(value),
        }
    }

    /// Disables local interrupts before taking the lock. Dropping the guard restores them.
    pub fn lock_irqsave(&self) -> IrqGuard<'_, T> {
        let flags = irq_save();
        self.lock_ticket();
        IrqGuard {
            lock: self,
            flags,
            marker: PhantomData,
        }
    }

    /// Waits for this CPU's ticket to become the next one served.
    fn lock_ticket(&self) {
        let ticket = self.next.fetch_add(1, Ordering::Relaxed);
        while self.serving.load(Ordering::Acquire) != ticket {
            cpu_relax();
        }
    }
}

/// Local interrupt state must be restored on the acquiring thread.
///
pub struct IrqGuard<'a, T> {
    /// Ticket lock held by this guard.
    lock: &'a TicketLock<T>,

    /// Interrupt state restored after releasing the ticket.
    flags: usize,

    /// Keeps guard ownership on the acquiring thread.
    marker: PhantomData<(&'a mut T, *mut ())>,
}

impl<T> Deref for IrqGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        // SAFETY: the ticket lock is held by this guard, giving unique mutable
        // access. Reborrowing it immutably is therefore valid.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for IrqGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: the ticket lock is held by this guard and its lifetime
        // prevents another guard from accessing the value concurrently.
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for IrqGuard<'_, T> {
    /// Releases the ticket before restoring the saved interrupt state.
    fn drop(&mut self) {
        self.lock.serving.fetch_add(1, Ordering::Release);
        irq_restore(self.flags);
    }
}

/// Hints that the current CPU is in a short spin loop.
#[inline]
pub fn cpu_relax() {
    #[cfg(target_arch = "x86_64")]
    // SAFETY: `pause` has no operands or memory side effects and is the
    // documented x86 spin-wait hint.
    unsafe {
        core::arch::asm!("pause", options(nomem, nostack, preserves_flags));
    };

    #[cfg(target_arch = "riscv64")]
    // SAFETY: `nop` is a RISC-V spin-wait hint with no memory side effects.
    unsafe {
        core::arch::asm!("nop", options(nomem, nostack, preserves_flags));
    };
}

/// Disables local interrupts and captures their previous enable state.
///
/// The assembly acts as a compiler barrier so lock acquisition stays after
/// interrupt disable and lock release stays before interrupt restore.
#[inline]
fn irq_save() -> usize {
    #[cfg(target_arch = "x86_64")]
    {
        let flags: usize;

        // SAFETY: this is the exact interrupt-state sequence used by the C
        // DDK. It returns the prior flags and does not touch Rust stack data.
        unsafe { core::arch::asm!("pushfq", "pop {}", "cli", out(reg) flags) };
        flags
    }

    #[cfg(target_arch = "riscv64")]
    {
        let previous: usize;

        // SAFETY: clearing SIE is the RISC-V local interrupt primitive.
        unsafe {
            core::arch::asm!(
                "csrrc {}, sstatus, {}",
                out(reg) previous,
                in(reg) 2usize,
                options(nostack)
            );
        };
        previous
    }
}

/// Restores the local interrupt state saved by `irq_save`.
#[inline]
fn irq_restore(state: usize) {
    #[cfg(target_arch = "x86_64")]
    if state & (1 << 9) != 0 {
        // SAFETY: restores only the interrupt-enable state captured by
        // `irq_save`, matching the C DDK implementation.
        unsafe { core::arch::asm!("sti", options(nostack)) };
    }

    #[cfg(target_arch = "riscv64")]
    if state & (1 << 1) != 0 {
        // SAFETY: restores SIE only if it was set by the saved status word.
        unsafe {
            core::arch::asm!("csrs sstatus, {}", in(reg) 2usize, options(nostack));
        };
    }
}

/// Unique owner of a module event. Dropping it revokes the identity and wakes
/// existing waiters. Kernel operations already in flight retain their storage.
/// Destruction is a thread-context operation, so move owners out of IRQ locks
/// before dropping them. Module teardown revokes any deliberately leaked owner.
///
pub struct OwnedEvent(Event);

impl OwnedEvent {
    /// Creates an initially unsignalled, module-owned event.
    pub fn create() -> Result<Self> {
        let mut event = 0;

        // SAFETY: module is live and event is writable output storage.
        unsafe { result(imports::event_create(module()?.as_ptr(), &raw mut event))? };
        Event::from_id(event)
            .map(Self)
            .ok_or_else(|| Error::from_status(EINVAL))
    }

    /// Returns a non-owning, revocable handle, suitable for an IRQ or wait.
    #[must_use]
    pub const fn event(&self) -> Event {
        self.0
    }
}

impl Deref for OwnedEvent {
    type Target = Event;

    fn deref(&self) -> &Event {
        &self.0
    }
}

impl Drop for OwnedEvent {
    /// Revokes the event identity and wakes any admitted waiters.
    fn drop(&mut self) {
        // Destruction cannot be busy. An already swept identity is harmless.
        let _ = event_call(imports::event_destroy, self.0.id());
    }
}

/// Non-owning event identity. Copying it does not extend the owner's lifetime.
/// Revoked identities fail lookup. Waits already admitted are woken on revoke.
#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct Event(usize);

impl Event {
    /// Returns the non-owning identity used by the ABI.
    #[must_use]
    pub const fn id(self) -> usize {
        self.0
    }

    /// Wraps a nonzero identity without checking whether it is still registered.
    #[must_use]
    pub const fn from_id(id: usize) -> Option<Self> {
        if id == 0 { None } else { Some(Self(id)) }
    }

    /// Sets the event and wakes waiters.
    pub fn signal(self) -> Result<()> {
        event_call(imports::event_signal, self.0)
    }

    /// Clears the signalled state for a later wait.
    pub fn reset(self) -> Result<()> {
        event_call(imports::event_reset, self.0)
    }

    /// Waits in thread context until the event is signalled or revoked.
    pub fn wait(self) -> Result<()> {
        event_call(imports::event_wait, self.0)
    }

    /// Waits until any event in `events` is signalled and returns its index.
    pub fn wait_any(events: &[Self]) -> Result<usize> {
        if events.is_empty() {
            return Err(Error::from_status(EINVAL));
        }

        let mut winner = 0usize;

        // SAFETY: the slice contains kernel-issued event handles and `winner`
        // is writable local output storage.
        let status = unsafe {
            imports::event_wait_any(events.as_ptr().cast(), events.len(), &raw mut winner)
        };
        result(status)?;
        if winner >= events.len() {
            return Err(Error::from_status(EIO));
        }

        Ok(winner)
    }

    /// Waits for this event until `nanoseconds` elapse.
    ///
    /// Returns `true` when the event was signalled and `false` on timeout.
    pub fn wait_timeout(self, nanoseconds: u64) -> Result<bool> {
        let mut signalled = 0u8;

        // SAFETY: this is a kernel-issued event handle and `signalled` is
        // writable local output storage.
        let status =
            unsafe { imports::event_wait_timeout(self.0, nanoseconds, &raw mut signalled) };
        result(status)?;
        Ok(signalled != 0)
    }
}

/// Receipt for a kernel thread executing a module callback.
#[must_use = "retain this callback receipt until release or join completes"]
pub struct Worker(Option<NonNull<raw::Worker>>);

// SAFETY: worker ownership is transferred between terminal lifecycle code. The
// opaque kernel receipt is never dereferenced in Rust.
unsafe impl Send for Worker {}

impl Worker {
    /// Starts a worker owned by the current module.
    ///
    /// # Safety
    /// `entry` and `context` must remain valid until `join` returns.
    pub unsafe fn spawn(entry: raw::WorkerEntry, context: *mut c_void) -> Result<Self> {
        let mut worker = ptr::null_mut::<raw::Worker>();

        // SAFETY: the callback and context meet this method's contract and
        // `worker` is writable output storage.
        let status =
            unsafe { imports::worker_spawn(module()?.as_ptr(), entry, context, &raw mut worker) };
        result(status)?;
        NonNull::new(worker)
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Waits for the worker to finish and consumes its kernel receipt, even
    /// when the ABI reports an invalid/stale receipt. Never join from that
    /// worker's own callback.
    pub fn join(&mut self) -> Result<()> {
        let Some(worker) = self.0.take() else {
            return Ok(());
        };

        // SAFETY: this uniquely consumes the worker receipt.
        let status = unsafe { imports::worker_join(worker.as_ptr()) };
        result(status)
    }
}

fn event_call(function: unsafe extern "C" fn(usize) -> i32, event: usize) -> Result<()> {
    // SAFETY: event IDs are issued by the kernel and the import has the
    // matching versioned C ABI signature.
    unsafe { result(function(event)) }
}

/// Registration receipt for a kernel IRQ domain.
#[must_use = "retain this registration and its context until checked cleanup succeeds"]
pub struct IrqDomain(Option<NonNull<raw::IrqDomain>>);

// SAFETY: registration lifetime is externally synchronized by its owner.
unsafe impl Send for IrqDomain {}

impl IrqDomain {
    /// Registers controller callbacks owned by the current module.
    ///
    /// # Safety
    /// Callback targets and context must remain valid until unregister succeeds.
    /// Mask, unmask, EOI, claim and complete must be IRQ-safe. Callbacks must
    /// tolerate concurrent calls. Do not withdraw the domain or the current
    /// mapping from its own callback, or recursively register from setup.
    pub unsafe fn register(
        name: &CStr,
        flags: u32,
        count: u32,
        definition: &'static raw::IrqDomainDef,
    ) -> Result<Self> {
        let mut domain = ptr::null();

        // SAFETY: `definition` is immutable static C ABI data.
        unsafe {
            result(imports::irq_domain_register(
                module()?.as_ptr(),
                name.as_ptr(),
                flags,
                count,
                definition,
                &raw mut domain,
            ))?;
        };
        NonNull::new(domain.cast_mut())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Unregisters the IRQ domain and keeps its receipt if the kernel refuses.
    pub fn unregister(&mut self) -> Result<()> {
        let Some(domain) = self.0 else {
            return Ok(());
        };

        // SAFETY: the receipt stays owned until unregistration succeeds.
        unsafe { result(imports::irq_domain_unregister(domain.as_ptr().cast_const()))? };
        self.0 = None;
        Ok(())
    }

    /// Publishes the identity of a fully initialized firmware controller.
    /// Call from thread context. This can synchronously retry deferred consumers,
    /// so callbacks and hardware state must already be ready.
    pub fn set_device(&self, device: Device) -> Result<()> {
        let domain = self.0.ok_or(Error::from_status(EINVAL))?;

        // SAFETY: both opaque identities are validated by the kernel.
        unsafe {
            result(imports::irq_domain_set_device(
                domain.as_ptr(),
                device.as_raw(),
            ))
        }
    }

    /// Maps a hardware interrupt to a kernel virtual IRQ in this domain.
    pub fn map(&self, hwirq: u64, flags: u32) -> Result<u32> {
        let mut virq = 0;
        let domain = self.0.ok_or(Error::from_status(EINVAL))?;

        // SAFETY: the domain is live and `virq` is writable output storage.
        unsafe {
            result(imports::irq_map(
                domain.as_ptr().cast_const(),
                hwirq,
                flags,
                &raw mut virq,
            ))?;
        }

        Ok(virq)
    }

    /// Removes the domain mapping for the hardware interrupt.
    pub fn unmap(&self, hwirq: u64) -> Result<()> {
        let domain = self.0.ok_or(Error::from_status(EINVAL))?;

        // SAFETY: this domain owns the mapped hardware interrupt.
        unsafe { result(imports::irq_unmap(domain.as_ptr().cast_const(), hwirq)) }
    }

    /// Returns the address and data programmed into a device's message interrupt.
    pub fn compose_message(&self, hwirq: u64) -> Result<(u64, u32)> {
        let domain = self.0.ok_or(Error::from_status(EINVAL))?;
        let mut address = 0;
        let mut data = 0;

        // SAFETY: the domain is live and both outputs are writable.
        unsafe {
            result(imports::irq_compose_message(
                domain.as_ptr().cast_const(),
                hwirq,
                &raw mut address,
                &raw mut data,
            ))?;
        }

        Ok((address, data))
    }
}

/// Receipt for an installed hard IRQ action and its optional worker.
#[must_use = "retain this callback receipt until release or join completes"]
pub struct Irq(Option<NonNull<raw::Irq>>);

// SAFETY: IRQ release is synchronized by the kernel and the wrapper never
// dereferences the opaque action pointer.
unsafe impl Send for Irq {}

impl Irq {
    /// Registers a hard interrupt handler.
    ///
    /// # Safety
    /// The handler and context must remain valid until release completes, and
    /// tolerate concurrent hard-interrupt invocation. The context must not be
    /// freed while a registered threaded callback can still access it.
    pub unsafe fn request(
        device: Device,
        virq: u32,
        name: &CStr,
        flags: u32,
        handler: unsafe extern "C" fn(*mut c_void, u32) -> u32,
        context: *mut c_void,
    ) -> Result<Self> {
        Self::request_with_thread(device, virq, name, flags, handler, None, context)
    }

    /// Registers a short hard handler and a sleepable, serialized service callback.
    /// The hard handler quiets its source and returns `IRQ_WAKE_THREAD`. The service
    /// callback drains authoritative state and rearms it. Both callbacks and their
    /// context must stay live until release. Release the action from another thread.
    ///
    /// # Safety
    /// Both callbacks and the context must remain valid until release completes.
    /// They must synchronize shared state and the hard handler must not sleep.
    #[allow(clippy::too_many_arguments)]
    pub unsafe fn request_threaded(
        device: Device,
        virq: u32,
        name: &CStr,
        flags: u32,
        handler: unsafe extern "C" fn(*mut c_void, u32) -> u32,
        thread: unsafe extern "C" fn(*mut c_void, u32),
        context: *mut c_void,
    ) -> Result<Self> {
        Self::request_with_thread(device, virq, name, flags, handler, Some(thread), context)
    }

    /// Registers a hard IRQ handler and an optional thread callback under one receipt.
    #[allow(clippy::too_many_arguments)]
    fn request_with_thread(
        device: Device,
        virq: u32,
        name: &CStr,
        flags: u32,
        handler: unsafe extern "C" fn(*mut c_void, u32) -> u32,
        thread: Option<unsafe extern "C" fn(*mut c_void, u32)>,
        context: *mut c_void,
    ) -> Result<Self> {
        let mut irq = ptr::null_mut();

        // SAFETY: handler and context remain valid until `release`.
        unsafe {
            result(imports::irq_request(
                module()?.as_ptr(),
                device.as_raw(),
                virq,
                name.as_ptr(),
                flags,
                Some(handler),
                thread,
                context,
                &raw mut irq,
            ))?;
        };
        NonNull::new(irq)
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Consumes the action receipt and drains callbacks. This is a consuming
    /// ABI operation even on stale-receipt errors. Never call it from this
    /// action's own hard or threaded callback.
    pub fn release(&mut self) -> Result<()> {
        let Some(irq) = self.0.take() else {
            return Ok(());
        };

        // SAFETY: this uniquely consumes the request receipt.
        unsafe { result(imports::irq_release(irq.as_ptr())) }
    }
}

/// Resolves the device's indexed interrupt specifier to a virtual IRQ.
pub fn irq_of_device(device: Device, index: usize) -> Result<u32> {
    let mut virq = 0;

    // SAFETY: `virq` is writable local output storage.
    unsafe {
        result(imports::irq_of_device(
            device.as_raw(),
            index,
            &raw mut virq,
        ))?;
    };

    Ok(virq)
}

/// Allocates an architecture interrupt vector for the virtual IRQ.
pub fn irq_alloc_vector(virq: u32) -> Result<u32> {
    let mut vector = 0;

    // SAFETY: vector is writable local output storage.
    unsafe { result(imports::irq_alloc_vector(virq, &raw mut vector))? };
    Ok(vector)
}

/// Releases an architecture interrupt vector.
pub fn irq_free_vector(vector: u32) -> Result<()> {
    // SAFETY: vector came from `irq_alloc_vector` in this module.
    unsafe { result(imports::irq_free_vector(vector)) }
}

/// Routes a virtual IRQ to the specified logical CPU.
pub fn irq_set_affinity(virq: u32, cpu: u32) -> Result<()> {
    // SAFETY: the kernel validates the virtual interrupt and CPU.
    unsafe { result(imports::irq_set_affinity(virq, cpu)) }
}

/// Resolves a logical CPU index to its firmware or hardware identifier.
pub fn cpu_platform_id(cpu: u32) -> Result<u64> {
    let mut id = 0;

    // SAFETY: id is writable local output storage.
    unsafe { result(imports::cpu_platform_id(cpu, &raw mut id))? };
    Ok(id)
}

/// Returns the kernel's logical CPU count.
#[must_use]
pub fn cpu_count() -> u32 {
    // SAFETY: the service takes no pointers and returns the current topology size.
    unsafe { imports::cpu_count() }
}

/// Copies the firmware ACPI root pointer data into the caller's buffer.
pub fn firmware_acpi(bytes: &mut [u8]) -> Result<usize> {
    firmware(imports::firmware_acpi, bytes)
}

/// Copies the firmware device tree into the caller's buffer.
pub fn firmware_devicetree(bytes: &mut [u8]) -> Result<usize> {
    firmware(imports::firmware_devicetree, bytes)
}

/// Passes buffer capacity to a firmware accessor and returns its reported byte count.
fn firmware(
    function: unsafe extern "C" fn(*mut u8, usize, *mut usize) -> i32,
    bytes: &mut [u8],
) -> Result<usize> {
    let mut written = 0;

    // SAFETY: the mutable slice supplies exactly this writable range.
    unsafe { result(function(bytes.as_mut_ptr(), bytes.len(), &raw mut written))? };
    if written > bytes.len() {
        Err(Error::from_status(EINVAL))
    } else {
        Ok(written)
    }
}

/// Unique receipt for a hardware block device owned by this module.
#[must_use = "retain this registration and its context until checked cleanup succeeds"]
pub struct BlockRegistration(Option<NonNull<raw::BlockDevice>>);

// SAFETY: the receipt is opaque. Drivers must serialize access to their own
// callback context, while the kernel serializes registration lifetime.
unsafe impl Send for BlockRegistration {}

impl BlockRegistration {
    /// Registers logical-block callbacks with the kernel block layer.
    ///
    /// # Safety
    /// Callback targets and `operations.context` must remain valid until this
    /// receipt has been unregistered.
    pub unsafe fn register(name: &CStr, operations: &raw::BlockOps) -> Result<Self> {
        let mut device = ptr::null();

        // SAFETY: upheld by this method's contract. Both input records remain
        // readable and output storage is writable for the immediate call.
        unsafe {
            result(imports::block_register(
                module()?.as_ptr(),
                name.as_ptr(),
                operations,
                &raw mut device,
            ))?;
        }
        NonNull::new(device.cast_mut())
            .map(|device| Self(Some(device)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Unregisters the disk after all mounts and open device nodes are gone.
    pub fn unregister(&mut self) -> Result<()> {
        let Some(device) = self.0 else {
            return Ok(());
        };

        // SAFETY: this is the live receipt returned by `block_register`.
        unsafe { result(imports::block_unregister(device.as_ptr()))? };
        self.0 = None;
        Ok(())
    }

    /// Runs partition discovery after the controller and callback context have
    /// reached their final published locations.
    pub fn rescan(&self) -> Result<()> {
        let device = self.0.ok_or(Error::from_status(EINVAL))?;

        // SAFETY: the receipt remains live for this shared borrow.
        unsafe { result(imports::block_rescan(device.as_ptr())) }
    }
}

/// Device-filesystem operations. Registration and removal resolve the calling
/// module at the operation that needs it. Lookup and root queries need no module receipt.
pub mod devfs {
    use super::{CStr, NODE_CHARACTER, Result, imports, module, raw, result};
    /// Looks up a path relative to the devfs root, such as `input/event0`.
    pub fn lookup(path: &CStr) -> Result<u64> {
        let mut node = 0;

        // SAFETY: path and output storage remain live for this lookup.
        unsafe {
            result(imports::devfs_lookup(path.as_ptr(), &raw mut node))?;
        }

        Ok(node)
    }

    /// Returns the root node identity for devfs.
    pub fn root() -> Result<u64> {
        let mut node = 0;

        // SAFETY: the versioned import has the C ABI checked in kernel exports.
        let status = unsafe { imports::devfs_root(&raw mut node) };
        result(status)?;
        Ok(node)
    }

    /// Creates a module-owned devfs directory beneath the specified parent.
    pub fn mkdir(parent: u64, name: &CStr, mode: u16) -> Result<u64> {
        let mut node = 0;

        // SAFETY: name and output storage remain valid during this call.
        unsafe {
            result(imports::devfs_mkdir(
                module()?.as_ptr(),
                parent,
                name.as_ptr(),
                mode,
                &raw mut node,
            ))?;
        };

        Ok(node)
    }

    /// # Safety
    /// `operations` must describe valid C ABI callbacks. The kernel copies the
    /// size-prefixed table during this call. Its context and callback targets
    /// must remain valid until the node is removed.
    pub unsafe fn create_character(
        parent: u64,
        name: &CStr,
        mode: u16,
        operations: &raw::NodeOps,
    ) -> Result<u64> {
        let mut node = 0;

        // SAFETY: upheld by this method's contract and kernel's closed import.
        let status = unsafe {
            imports::devfs_create(
                module()?.as_ptr(),
                parent,
                name.as_ptr(),
                NODE_CHARACTER,
                mode,
                operations,
                &raw mut node,
            )
        };
        result(status)?;
        Ok(node)
    }

    /// Removes a devfs node owned by this module.
    pub fn remove(node: u64) -> Result<()> {
        // SAFETY: the node receipt belongs to this module and is consumed by
        // the kernel's device-filesystem removal path.
        let status = unsafe { imports::devfs_remove(module()?.as_ptr(), node) };
        result(status)
    }
}

/// Receipt for a TTY backend whose callbacks remain live until unregistration.
#[must_use = "retain this registration and its context until checked cleanup succeeds"]
pub struct Tty(Option<NonNull<raw::Tty>>);

// SAFETY: terminal receipt use is serialized by its driver state lock.
unsafe impl Send for Tty {}

impl Tty {
    /// # Safety
    /// Registration copies `operations`. Its callback code and context must
    /// remain valid until the terminal is unregistered.
    pub unsafe fn register(
        parent: u64,
        name: &CStr,
        mode: u16,
        baud: u32,
        operations: &raw::ConsoleOps,
    ) -> Result<Self> {
        let mut tty = ptr::null_mut();

        // SAFETY: operation table and C string remain valid after registration.
        unsafe {
            result(imports::tty_register(
                module()?.as_ptr(),
                parent,
                name.as_ptr(),
                mode,
                baud,
                operations,
                &raw mut tty,
            ))?;
        };
        NonNull::new(tty)
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Unregisters the TTY backend and retains its handle if teardown fails.
    pub fn unregister(&mut self) -> Result<()> {
        let Some(tty) = self.0 else {
            return Ok(());
        };

        // SAFETY: the receipt remains valid until the kernel reports success.
        unsafe { result(imports::tty_unregister(tty.as_ptr())) }?;
        self.0 = None;
        Ok(())
    }
}

/// Sleeps in thread context for the requested interval in nanoseconds.
pub fn sleep_ns(nanoseconds: u64) {
    // SAFETY: the versioned import takes only a scalar duration.
    unsafe {
        imports::time_sleep(nanoseconds);
    }
}

/// Returns monotonic nanoseconds since boot.
pub fn monotonic_ns() -> u64 {
    // SAFETY: the versioned import has no arguments or pointer requirements.
    unsafe { imports::time_monotonic() }
}

/// Fills the provided buffer using the kernel random service.
pub fn random_fill(bytes: &mut [u8]) {
    if bytes.is_empty() {
        return;
    }

    // SAFETY: the slice is writable for exactly its length.
    unsafe {
        imports::random_fill(bytes.as_mut_ptr(), bytes.len());
    };
}

/// Mixes the provided bytes into the kernel random pool.
pub fn random_mix(bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }

    // SAFETY: the slice is readable for exactly its length.
    unsafe {
        imports::random_mix(bytes.as_ptr(), bytes.len());
    };
}

/// Allocates a zeroed kernel buffer that must be freed with the same size and alignment.
pub fn alloc_zeroed(size: usize, align: usize) -> Result<NonNull<u8>> {
    // SAFETY: size and alignment are provided by the fixed driver allocation.
    let pointer = unsafe { imports::alloc_zeroed(size, align).cast() };
    NonNull::new(pointer).ok_or(Error::from_status(ENOMEM))
}

/// # Safety
/// `pointer`, `size`, and `align` must exactly match an allocation returned by
/// `alloc_zeroed` or the corresponding kernel allocation service.
pub unsafe fn free(pointer: NonNull<u8>, size: usize, align: usize) {
    // SAFETY: upheld by this function's safety contract.
    unsafe {
        imports::free(pointer.as_ptr().cast(), size, align);
    };
}

/// Orders coherent DMA memory and device register accesses.
/// Use before publishing a descriptor through a doorbell and after observing
/// device completion before reading the associated DMA payload.
#[inline]
pub fn device_barrier() {
    #[cfg(target_arch = "riscv64")]
    // SAFETY: orders memory and device I/O without changing execution state.
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack));
    }

    #[cfg(not(target_arch = "riscv64"))]
    core::sync::atomic::fence(Ordering::SeqCst);
}

/// Reads an 8-bit x86 I/O port. Other targets return all ones.
pub fn port_read8(port: u16) -> u8 {
    #[cfg(target_arch = "x86_64")]
    {
        let value: u8;

        // SAFETY: the caller selected this port from a device resource. The
        // instruction is the complete x86 port-I/O operation.
        unsafe {
            core::arch::asm!(
                "in al, dx",
                out("al") value,
                in("dx") port,
                options(nostack, preserves_flags),
            );
        }
        value
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = port;
        u8::MAX
    }
}

/// Writes an 8-bit x86 I/O port. Other targets ignore the write.
pub fn port_write8(port: u16, value: u8) {
    #[cfg(target_arch = "x86_64")]
    {
        // SAFETY: the caller selected this port from a device resource. The
        // instruction is the complete x86 port-I/O operation.
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") port,
                in("al") value,
                options(nostack, preserves_flags),
            );
        }
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (port, value);
    }
}

/// Reads a 16-bit x86 I/O port. Other targets return all ones.
pub fn port_read16(port: u16) -> u16 {
    #[cfg(target_arch = "x86_64")]
    {
        let value: u16;

        // SAFETY: this is one complete x86 port-I/O instruction.
        unsafe {
            core::arch::asm!(
                "in ax, dx",
                out("ax") value,
                in("dx") port,
                options(nostack, preserves_flags),
            );
        }
        value
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = port;
        u16::MAX
    }
}

/// Writes a 16-bit x86 I/O port. Other targets ignore the write.
pub fn port_write16(port: u16, value: u16) {
    #[cfg(target_arch = "x86_64")]
    // SAFETY: this is one complete x86 port-I/O instruction.
    unsafe {
        core::arch::asm!(
            "out dx, ax",
            in("dx") port,
            in("ax") value,
            options(nostack, preserves_flags),
        );
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (port, value);
    }
}

/// Reads a 32-bit x86 I/O port. Other targets return all ones.
pub fn port_read32(port: u16) -> u32 {
    #[cfg(target_arch = "x86_64")]
    {
        let value: u32;

        // SAFETY: this is one complete x86 port-I/O instruction.
        unsafe {
            core::arch::asm!(
                "in eax, dx",
                out("eax") value,
                in("dx") port,
                options(nostack, preserves_flags),
            );
        }
        value
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = port;
        u32::MAX
    }
}

/// Writes a 32-bit x86 I/O port. Other targets ignore the write.
pub fn port_write32(port: u16, value: u32) {
    #[cfg(target_arch = "x86_64")]
    // SAFETY: this is one complete x86 port-I/O instruction.
    unsafe {
        core::arch::asm!(
            "out dx, eax",
            in("dx") port,
            in("eax") value,
            options(nostack, preserves_flags),
        );
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (port, value);
    }
}

/// Returns the current logical CPU index.
#[must_use]
pub fn cpu_current() -> u32 {
    // SAFETY: the service takes no pointers and returns the current logical CPU.
    unsafe { imports::cpu_current() }
}

/// Publishes the loader-provided module identity before driver initialization.
#[doc(hidden)]
pub fn install_module(module: Module) {
    SELF.store(module.0.as_ptr(), Ordering::Release);
}

/// Terminates execution with an architecture trap.
#[inline(never)]
pub fn abort() -> ! {
    #[cfg(target_arch = "x86_64")]
    // SAFETY: `ud2` terminates an aborting freestanding x86 image.
    unsafe {
        core::arch::asm!("ud2", options(noreturn))
    };

    #[cfg(target_arch = "riscv64")]
    // SAFETY: `unimp` terminates an aborting freestanding RISC-V image.
    unsafe {
        core::arch::asm!("unimp", options(noreturn))
    };
}

#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    abort()
}

/// Defines the sole C ABI module entry point and descriptor.
#[macro_export]
macro_rules! module {
    ($name:expr, $description:expr, $init:path, $exit:path $(,)?) => {
        static __RDF_MODULE_NAME: &[u8] = $name;
        static __RDF_MODULE_DESCRIPTION: &[u8] = $description;

        /// Installs the module identity and forwards initialization to the driver.
        unsafe extern "C" fn __rdf_module_init(pointer: *const $crate::raw::Module) -> i32 {
            // SAFETY: the loader supplies the live module callback handle.
            let Some(module) = (unsafe { $crate::Module::from_raw(pointer) }) else {
                return $crate::EINVAL;
            };
            $crate::install_module(module);
            match ($init)(module) {
                Ok(()) => $crate::OK,
                Err(error) => error.status(),
            }
        }

        /// Forwards unload to the driver when the loader supplies a module identity.
        unsafe extern "C" fn __rdf_module_exit(pointer: *const $crate::raw::Module) {
            // SAFETY: the loader supplies the live module callback handle.
            if let Some(module) = unsafe { $crate::Module::from_raw(pointer) } {
                ($exit)(module);
            }
        }

        #[used]
        static __RDF_MODULE_DEFINITION: $crate::raw::ModuleDef = $crate::raw::ModuleDef {
            size: $crate::raw::MODULE_DEF_SIZE,
            abi_major: $crate::raw::ABI_MAJOR,
            abi_reserved: 0,
            flags: 0,
            name: __RDF_MODULE_NAME.as_ptr().cast(),
            description: __RDF_MODULE_DESCRIPTION.as_ptr().cast(),
            init: Some(__rdf_module_init),
            exit: Some(__rdf_module_exit),
        };

        /// Returns the static module descriptor consumed by the Roanix loader.
        /// # Safety
        /// Call using the module-entry ABI. The reserved argument is ignored.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn rdf_module_entry(
            _reserved: *const core::ffi::c_void,
        ) -> *const $crate::raw::ModuleDef {
            core::ptr::addr_of!(__RDF_MODULE_DEFINITION)
        }
    };
}
