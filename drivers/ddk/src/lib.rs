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

//! The Roanix Rust driver development kit.
//!
//! `raw` is the only ABI surface: all of its records are `repr(C)`, opaque
//! kernel objects are never dereferenced, and every callback is `extern "C"`.
//! The remaining types are small ownership and synchronization wrappers used
//! inside a Rust module; none are passed over a module boundary.

extern crate alloc;

#[cfg(feature = "module-build")]
extern crate std;

use core::{
    alloc::{GlobalAlloc, Layout},
    cell::UnsafeCell,
    ffi::{CStr, c_char, c_void},
    marker::PhantomData,
    ops::{Deref, DerefMut},
    ptr::{self, NonNull},
    sync::atomic::{AtomicPtr, AtomicU32, Ordering},
};

/// Configures Cargo's final link for a loadable Roanix driver module.
#[cfg(feature = "module-build")]
pub fn configure_module_link() {
    let name = std::env::var("CARGO_PKG_NAME").expect("Cargo must provide the package name");
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
    std::println!("cargo::rustc-link-arg-bins=-soname={name}.ko");
}

#[allow(dead_code)]
mod imports {
    use core::ffi::{c_char, c_void};

    use crate::raw::{
        self, DevfsEndpoint, FsMemoryObject, FsPageAccount, Module, NodeOps, TtyProviderOps,
    };

    macro_rules! kernel_imports {
        ($(fn $name:ident($($argument:ty),*) $(-> $result:ty)?;)*) => {
            unsafe extern "C" {
                $(
                    #[link_name = concat!("rdf_api_v1_", stringify!($name))]
                    pub fn $name($(_: $argument),*) $(-> $result)?;
                )*
            }
        };
    }

    kernel_imports! {
        fn alloc(usize, usize) -> *mut c_void;
        fn alloc_zeroed(usize, usize) -> *mut c_void;
        fn bus_find(*const c_char, *mut *const raw::Bus) -> i32;
        fn class_add(*const raw::Module, *const raw::Class, *const raw::Device, *const c_char, *const c_void, usize, *mut c_void, *mut *const raw::ClassDevice) -> i32;
        fn class_register(*const raw::Module, *const c_char, *const raw::ClassDef, *mut *const raw::Class) -> i32;
        fn class_remove(*const raw::ClassDevice);
        fn class_unregister(*const raw::Class) -> i32;
        fn cpu_count() -> u32;
        fn cpu_platform_id(u32, *mut u64) -> i32;
        fn devfs_broker_register(*const raw::Module, *const raw::DevfsBrokerOps, *mut *mut raw::DevfsBroker) -> i32;
        fn devfs_broker_unregister(*mut raw::DevfsBroker) -> i32;
        fn devfs_create(*const Module, *const c_void, u64, *const c_char, u32, u16, *const NodeOps, *mut u64) -> i32;
        fn devfs_endpoint_close(*mut DevfsEndpoint, usize, u32);
        fn devfs_endpoint_event(*mut DevfsEndpoint, usize, u32) -> usize;
        fn devfs_endpoint_initial_offset(*mut DevfsEndpoint, usize, u32, *mut u64) -> i32;
        fn devfs_endpoint_ioctl(*mut DevfsEndpoint, usize, u64, i32, i32, u8, u64, u64, *mut u8, usize) -> i64;
        fn devfs_endpoint_open(*mut DevfsEndpoint, u32, *mut usize) -> i32;
        fn devfs_endpoint_poll(*mut DevfsEndpoint, usize, u64, u16, u32) -> i64;
        fn devfs_endpoint_read(*mut DevfsEndpoint, usize, u64, *mut u8, usize, u32) -> i64;
        fn devfs_endpoint_release(*mut DevfsEndpoint);
        fn devfs_endpoint_size(*mut DevfsEndpoint) -> u64;
        fn devfs_endpoint_sync(*mut DevfsEndpoint) -> i32;
        fn devfs_endpoint_terminal_state(*mut DevfsEndpoint, *mut crate::raw::FsTerminalState) -> i32;
        fn devfs_endpoint_write(*mut DevfsEndpoint, usize, u64, *const u8, usize, u32) -> i64;
        fn devfs_mkdir(*const raw::Module, u64, *const c_char, u16, *mut u64) -> i32;
        fn devfs_remove(*const Module, u64) -> i32;
        fn devfs_root(*mut u64) -> i32;
        fn device_add(*mut raw::DeviceBuilder, *mut *const raw::Device) -> i32;
        fn device_add_cells(*mut raw::DeviceBuilder, *const c_char, *const u64, usize, u32) -> i32;
        fn device_add_int(*mut raw::DeviceBuilder, *const c_char, u64, u32) -> i32;
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
        fn device_set_bus(*mut raw::DeviceBuilder, *const raw::Bus) -> i32;
        fn device_set_data(*const raw::Device, *mut c_void);
        fn driver_register(*const raw::Module, *const raw::DriverDef, *mut *const raw::Driver) -> i32;
        fn driver_unregister(*const raw::Driver) -> i32;
        fn event_create(*const raw::Module, *mut usize) -> i32;
        fn event_destroy(usize) -> i32;
        fn event_reset(usize) -> i32;
        fn event_signal(usize) -> i32;
        fn event_wait(usize) -> i32;
        fn event_wait_any(*const usize, usize, *mut usize) -> i32;
        fn event_wait_timeout(usize, u64, *mut u8) -> i32;
        fn free(*mut c_void, usize, usize);
        fn firmware_acpi(*mut u8, usize, *mut usize) -> i32;
        fn firmware_devicetree(*mut u8, usize, *mut usize) -> i32;
        fn fs_memory_object_create(*mut FsPageAccount, *mut *mut FsMemoryObject) -> i32;
        fn fs_memory_object_page_count(*mut FsMemoryObject, *mut u64) -> i32;
        fn fs_memory_object_read(*mut FsMemoryObject, u64, *mut u8, usize) -> i64;
        fn fs_memory_object_release(*mut FsMemoryObject);
        fn fs_memory_object_retain(*mut FsMemoryObject) -> i32;
        fn fs_memory_object_truncate(*mut FsMemoryObject, u64, *mut u64) -> i32;
        fn fs_memory_object_write(*mut FsMemoryObject, u64, *const u8, usize) -> i64;
        fn fs_page_account_create(u64, *mut *mut FsPageAccount) -> i32;
        fn fs_page_account_limit(*mut FsPageAccount, *mut u64) -> i32;
        fn fs_page_account_release(*mut FsPageAccount);
        fn fs_page_account_used(*mut FsPageAccount, *mut u64) -> i32;
        fn fs_provider_register(*const raw::Module, *const c_char, *const raw::FsProviderOps, *mut *mut raw::FsProvider) -> i32;
        fn fs_provider_unregister(*mut raw::FsProvider) -> i32;
        fn fs_total_physical_pages() -> u64;
        fn irq_alloc_vector(u32, *mut u32) -> i32;
        fn irq_domain_register(*const raw::Module, *const c_char, u32, u32, *const raw::IrqDomainDef, *mut *const raw::IrqDomain) -> i32;
        fn irq_domain_unregister(*const raw::IrqDomain) -> i32;
        fn irq_free_vector(u32);
        fn irq_of_device(*const raw::Device, usize, *mut u32) -> i32;
        fn irq_release(*mut raw::Irq) -> i32;
        fn irq_request(*const raw::Module, *const raw::Device, u32, *const c_char, u32, Option<unsafe extern "C" fn(*mut c_void, u32) -> u32>, Option<unsafe extern "C" fn(*mut c_void, u32)>, *mut c_void, *mut *mut raw::Irq) -> i32;
        fn mmio_direct(u64, *mut *mut c_void) -> i32;
        fn mmio_map(*const raw::Module, u64, usize, u32, *mut raw::Mmio) -> i32;
        fn mmio_unmap(*mut raw::Mmio) -> i32;
        fn port_read8(u16) -> u32;
        fn port_write8(u16, u8);
        fn process_group_signal(i32, u8) -> i32;
        fn random_fill(*mut u8, usize);
        fn random_mix(*const u8, usize);
        fn time_monotonic() -> u64;
        fn time_sleep(u64);
        fn tty_provider_register(*const Module, *const TtyProviderOps, *mut *mut c_void) -> i32;
        fn tty_provider_unregister(*mut c_void) -> i32;
        fn tty_register(*const raw::Module, *const raw::Device, u64, *const c_char, u16, u32, *const raw::ConsoleOps, *mut *mut raw::Tty) -> i32;
        fn tty_unregister(*mut raw::Tty) -> i32;
        fn worker_join(*mut c_void) -> i32;
        fn worker_spawn(*const Module, Option<unsafe extern "C" fn(*mut c_void)>, *mut c_void, *mut *mut c_void) -> i32;
    }
}

/// C-compatible records and opaque handles. These definitions intentionally
/// contain no Rust references, enums, trait objects, or ownership semantics.
pub mod raw {
    use core::ffi::{c_char, c_void};

    pub const ABI_MAJOR: u16 = 1;
    pub const ABI_MINOR: u16 = 3;
    pub const MODULE_DEF_SIZE: u32 = 48;
    /// The original 112-byte prefix remains accepted by the kernel.
    pub const NODE_OPS_LEGACY_SIZE: u32 = 112;
    pub const NODE_OPS_SIZE: u32 = 120;
    pub const CONSOLE_OPS_SIZE: u32 = 160;
    pub const TTY_PROVIDER_OPS_SIZE: u32 = 32;
    pub const FS_PROVIDER_OPS_SIZE: u32 = 248;
    pub const FS_PROVIDER_REQUIRED_OPS_SIZE: u32 = 144;
    pub const DEVFS_BROKER_OPS_SIZE: u32 = 72;
    pub const DEVFS_BROKER_REQUIRED_OPS_SIZE: u32 = 64;
    pub const FS_DIRECTORY_NAME_MAX: usize = 255;
    pub const CLASS_DEF_SIZE: u32 = 32;
    pub const DRIVER_DEF_SIZE: u32 = 80;
    pub const IRQ_DOMAIN_DEF_SIZE: u32 = 96;

    macro_rules! opaque {
        ($($name:ident),+ $(,)?) => {$(
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
        Class,
        ClassDevice,
        IrqDomain,
        Irq,
        Tty,
        TtyProvider,
        Worker,
        FsProvider,
        FsPageAccount,
        FsMemoryObject,
        DevfsBroker,
        DevfsEndpoint,
    );

    pub type WorkerEntry = Option<unsafe extern "C" fn(*mut c_void)>;

    #[repr(C)]
    pub struct Mmio {
        pub base: *mut c_void,
        pub length: usize,
        pub token: *mut c_void,
    }

    #[repr(C)]
    pub struct Match {
        pub kind: u32,
        pub flags: u32,
        pub key: *const c_char,
        pub value: *const c_char,
        pub id0: u64,
        pub mask0: u64,
        pub id1: u64,
        pub mask1: u64,
        pub data: usize,
        pub score: i32,
    }

    #[repr(C)]
    pub struct DriverDef {
        pub size: u32,
        pub name: *const c_char,
        pub bus: *const Bus,
        pub priority: i32,
        pub matches: *const Match,
        pub match_count: usize,
        pub probe: Option<unsafe extern "C" fn(*mut c_void, *const Device, usize) -> i32>,
        pub remove: Option<unsafe extern "C" fn(*mut c_void, *const Device)>,
        pub shutdown: Option<unsafe extern "C" fn(*mut c_void, *const Device)>,
        pub context: *mut c_void,
    }

    #[repr(C)]
    pub struct IrqDomainDef {
        pub size: u32,
        pub translate:
            Option<unsafe extern "C" fn(*mut c_void, *const u32, usize, *mut u64, *mut u32) -> i32>,
        pub setup: Option<unsafe extern "C" fn(*mut c_void, u64, u32, u32) -> i32>,
        pub teardown: Option<unsafe extern "C" fn(*mut c_void, u64, u32)>,
        pub mask: Option<unsafe extern "C" fn(*mut c_void, u64)>,
        pub unmask: Option<unsafe extern "C" fn(*mut c_void, u64)>,
        pub eoi: Option<unsafe extern "C" fn(*mut c_void, u64)>,
        pub set_affinity: Option<unsafe extern "C" fn(*mut c_void, u64, u32) -> i32>,
        pub claim: Option<unsafe extern "C" fn(*mut c_void, u32, u64, *mut u64) -> i32>,
        pub complete: Option<unsafe extern "C" fn(*mut c_void, u32, u64, u64)>,
        pub compose_message:
            Option<unsafe extern "C" fn(*mut c_void, u64, *mut u64, *mut u32) -> i32>,
        pub context: *mut c_void,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct IoctlIdentity {
        pub process: u64,
        pub group: i32,
        pub session: i32,
        pub session_leader: u8,
    }

    /// Terminal state used by VFS job-control checks.
    ///
    /// This record is intentionally C-compatible and contains no Rust bools.
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct TerminalState {
        pub session: i32,
        pub foreground_group: i32,
        pub stop_background_output: u8,
        pub reserved: [u8; 3],
    }

    #[derive(Clone, Copy)]
    #[repr(C)]
    pub struct NodeOps {
        pub size: u32,
        pub context: *mut c_void,
        pub open: Option<unsafe extern "C" fn(*mut c_void, u32, *mut usize) -> i32>,
        pub close: Option<unsafe extern "C" fn(*mut c_void, usize, u32)>,
        pub initial_offset: Option<unsafe extern "C" fn(*mut c_void, usize, u32) -> i64>,
        pub read: Option<unsafe extern "C" fn(*mut c_void, usize, u64, *mut u8, usize, u32) -> i64>,
        pub write:
            Option<unsafe extern "C" fn(*mut c_void, usize, u64, *const u8, usize, u32) -> i64>,
        pub size_bytes: Option<unsafe extern "C" fn(*mut c_void) -> u64>,
        pub sync: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub poll: Option<unsafe extern "C" fn(*mut c_void, usize, u64, u16, u32) -> i64>,
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
        pub readable_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
        pub writable_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
        pub hangup_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
        /// Appended in ABI minor 1. Old 112-byte tables leave this absent.
        pub terminal_state: Option<unsafe extern "C" fn(*mut c_void, *mut TerminalState) -> i32>,
    }

    #[repr(C)]
    pub struct SerialFraming {
        pub baud: u32,
        pub data_bits: u8,
        pub stop_bits: u8,
        pub parity: u8,
        pub odd_parity: u8,
    }

    #[repr(C)]
    pub struct ConsoleOps {
        pub size: u32,
        pub flags: u64,
        pub context: *mut c_void,
        pub open: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub close: Option<unsafe extern "C" fn(*mut c_void)>,
        pub try_read: Option<unsafe extern "C" fn(*mut c_void, *mut u8) -> i32>,
        pub read: Option<unsafe extern "C" fn(*mut c_void, *mut u8, usize) -> i64>,
        pub write: Option<unsafe extern "C" fn(*mut c_void, *const u8, usize, u8) -> i32>,
        pub configure: Option<unsafe extern "C" fn(*mut c_void, *const SerialFraming) -> i32>,
        pub flush: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub flush_input: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub flush_output: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub send_break: Option<unsafe extern "C" fn(*mut c_void, u64) -> i32>,
        pub writable: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub hung_up: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        pub queued_output: Option<unsafe extern "C" fn(*mut c_void) -> i64>,
        pub destroy: Option<unsafe extern "C" fn(*mut c_void)>,
        pub readable_event: usize,
        pub writable_event: usize,
        pub hangup_event: usize,
    }

    /// Provider callbacks registered by the modular terminal subsystem.
    #[repr(C)]
    pub struct TtyProviderOps {
        pub size: u32,
        pub context: *mut c_void,
        pub register: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *const Module,
                *const Device,
                u64,
                *const c_char,
                u16,
                u32,
                *const ConsoleOps,
                *mut *mut c_void,
            ) -> i32,
        >,
        pub unregister: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsMountOptions {
        pub size: u32,
        pub flags: u32,
        pub page_limit: u64,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsVnode {
        pub receipt: *mut c_void,
        pub node_id: u64,
        pub kind: u32,
        pub reserved: u32,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsAttr {
        pub size: u64,
        pub links: u64,
        pub accessed_ns: u64,
        pub modified_ns: u64,
        pub changed_ns: u64,
        pub mode: u16,
        pub kind: u32,
        pub reserved: u16,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsSetAttr {
        pub valid: u32,
        pub reserved: u32,
        pub size: u64,
        pub mode: u16,
        pub reserved2: [u8; 6],
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsIoVec {
        pub data: *const u8,
        pub length: usize,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsDirEntry {
        pub vnode: FsVnode,
        pub name_length: u16,
        pub reserved: [u8; 6],
        pub offset: u64,
        pub name: [u8; FS_DIRECTORY_NAME_MAX],
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct DevfsBrokerEntry {
        pub node: u64,
        pub name_length: u16,
        pub reserved: [u8; 6],
        pub name: [u8; FS_DIRECTORY_NAME_MAX],
    }

    #[repr(C)]
    #[derive(Clone, Copy, Default)]
    pub struct FsStat {
        pub total_bytes: u64,
        pub used_bytes: u64,
        pub total_nodes: u64,
        pub used_nodes: u64,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct FsTerminalState {
        pub session: i32,
        pub foreground_group: i32,
        pub stop_background_output: u8,
        pub reserved: [u8; 3],
    }

    /// The device-node ioctl entry of [`FsProviderOps`]. Factored out only to
    /// keep the record readable; the C layout is unchanged.
    pub type FsProviderIoctl = unsafe extern "C" fn(
        *mut c_void,
        *mut c_void,
        *mut c_void,
        usize,
        u64,
        i32,
        i32,
        u8,
        u64,
        u64,
        *mut u8,
        usize,
        *mut u64,
    ) -> i32;

    #[repr(C)]
    pub struct FsProviderOps {
        pub size: u32,
        pub context: *mut c_void,
        pub mount: Option<
            unsafe extern "C" fn(*mut c_void, *const FsMountOptions, *mut *mut c_void) -> i32,
        >,
        pub unmount: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
        pub root: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut FsVnode) -> i32>,
        pub statfs: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut FsStat) -> i32>,
        pub sync: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
        pub vnode_release: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void)>,
        pub getattr:
            Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut FsAttr) -> i32>,
        pub setattr: Option<
            unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const FsSetAttr) -> i32,
        >,
        pub lookup: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *const u8,
                usize,
                *mut FsVnode,
            ) -> i32,
        >,
        pub parent: Option<
            unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut FsVnode) -> i32,
        >,
        pub create: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *const u8,
                usize,
                u32,
                *const u8,
                usize,
                u16,
                *mut FsVnode,
            ) -> i32,
        >,
        pub link: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *const u8,
                usize,
                *mut c_void,
            ) -> i32,
        >,
        pub unlink: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *const u8,
                usize,
                u8,
            ) -> i32,
        >,
        pub rename: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *const u8,
                usize,
                *mut c_void,
                *const u8,
                usize,
            ) -> i32,
        >,
        pub open: Option<
            unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, u32, *mut usize) -> i32,
        >,
        pub close: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, usize, u32)>,
        pub initial_offset: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                usize,
                u32,
                *mut u64,
            ) -> i32,
        >,
        pub read: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                usize,
                u64,
                *mut u8,
                usize,
                u32,
            ) -> i64,
        >,
        pub write: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                usize,
                u64,
                *const u8,
                usize,
                u32,
            ) -> i64,
        >,
        pub append: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *const FsIoVec,
                usize,
                *mut u64,
            ) -> i64,
        >,
        pub truncate:
            Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, u64) -> i32>,
        pub memory_object: Option<
            unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut *mut c_void) -> i32,
        >,
        pub readlink: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *mut u8,
                usize,
                *mut usize,
            ) -> i32,
        >,
        pub readdir: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                u64,
                *mut FsDirEntry,
                usize,
                *mut usize,
                *mut u64,
            ) -> i32,
        >,
        pub fsync: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void) -> i32>,
        pub poll: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                usize,
                u64,
                u16,
                u32,
                *mut u16,
            ) -> i32,
        >,
        pub poll_events: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                usize,
                u16,
                *mut usize,
                usize,
                *mut usize,
            ) -> i32,
        >,
        pub terminal_state: Option<
            unsafe extern "C" fn(
                *mut c_void,
                *mut c_void,
                *mut c_void,
                *mut FsTerminalState,
            ) -> i32,
        >,
        pub ioctl: Option<FsProviderIoctl>,
    }

    #[repr(C)]
    pub struct DevfsBrokerOps {
        pub size: u32,
        pub context: *mut c_void,
        pub root: Option<unsafe extern "C" fn(*mut c_void, *mut u64) -> i32>,
        pub mkdir: Option<
            unsafe extern "C" fn(*mut c_void, u64, u64, *const u8, usize, u16, *mut u64) -> i32,
        >,
        pub create: Option<
            unsafe extern "C" fn(
                *mut c_void,
                u64,
                u64,
                *const u8,
                usize,
                u32,
                u16,
                *mut DevfsEndpoint,
                *mut u64,
            ) -> i32,
        >,
        pub remove: Option<unsafe extern "C" fn(*mut c_void, u64, u64) -> i32>,
        pub remove_owner: Option<unsafe extern "C" fn(*mut c_void, u64, u8) -> i32>,
        pub lookup: Option<unsafe extern "C" fn(*mut c_void, *const u8, usize, *mut u64) -> i32>,
        pub children: Option<
            unsafe extern "C" fn(*mut c_void, u64, *mut DevfsBrokerEntry, usize, *mut usize) -> i32,
        >,
    }

    /// Class callbacks supplied through the C ABI.
    #[repr(C)]
    pub struct ClassDef {
        pub size: u32,
        pub attach: Option<unsafe extern "C" fn(*mut c_void, *const ClassDevice) -> i32>,
        pub detach: Option<unsafe extern "C" fn(*mut c_void, *const ClassDevice)>,
        pub context: *mut c_void,
    }

    #[repr(C)]
    pub struct ModuleDef {
        pub size: u32,
        pub abi_major: u16,
        pub abi_minor: u16,
        pub flags: u32,
        pub name: *const c_char,
        pub description: *const c_char,
        pub init: Option<unsafe extern "C" fn(*mut Module) -> i32>,
        pub exit: Option<unsafe extern "C" fn(*mut Module)>,
    }

    // SAFETY: all callback tables and module descriptors used by the DDK are
    // immutable statics. Their callback synchronization is explicit.
    unsafe impl Sync for NodeOps {}
    // SAFETY: the immutable table delegates synchronization of its context to
    // the owning console backend.
    unsafe impl Sync for ConsoleOps {}
    // SAFETY: provider operation tables are immutable after publication and
    // their contexts are synchronized by the provider.
    unsafe impl Sync for TtyProviderOps {}
    // SAFETY: provider operation tables are immutable after publication and
    // their contexts are synchronized by the provider.
    unsafe impl Sync for FsProviderOps {}
    // SAFETY: broker operation tables are immutable after publication and the
    // broker synchronizes its context.
    unsafe impl Sync for DevfsBrokerOps {}
    // SAFETY: class definitions are immutable after registration and their
    // callback context is synchronized by the class owner.
    unsafe impl Sync for ClassDef {}
    // SAFETY: console tables are immutable after registration; their opaque
    // context is synchronized by the owning driver.
    unsafe impl Send for ConsoleOps {}
    // SAFETY: driver definitions are immutable after publication and their
    // callback contexts are synchronized by the driver.
    unsafe impl Sync for DriverDef {}
    // SAFETY: IRQ domain definitions are immutable after registration and the
    // controller synchronizes its callback context.
    unsafe impl Sync for IrqDomainDef {}
    // SAFETY: module descriptors are immutable statics for the image lifetime.
    unsafe impl Sync for ModuleDef {}
    // SAFETY: match entries contain immutable values and static string pointers.
    unsafe impl Sync for Match {}
    // SAFETY: definitions are only accepted by registration APIs as immutable
    // module statics. Drivers that update a setup-time bus pointer must hold
    // their own `TicketLock` before publishing the definition.
    unsafe impl Send for DriverDef {}

    const _: () = assert!(core::mem::size_of::<Mmio>() == 24);
    const _: () = assert!(core::mem::size_of::<Match>() == 72);
    const _: () = assert!(core::mem::size_of::<DriverDef>() == DRIVER_DEF_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<IrqDomainDef>() == IRQ_DOMAIN_DEF_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<NodeOps>() == NODE_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<ConsoleOps>() == CONSOLE_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<TtyProviderOps>() == TTY_PROVIDER_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<FsMountOptions>() == 16);
    const _: () = assert!(core::mem::size_of::<FsVnode>() == 24);
    const _: () = assert!(core::mem::size_of::<FsAttr>() == 56);
    const _: () = assert!(core::mem::size_of::<FsSetAttr>() == 24);
    const _: () = assert!(core::mem::size_of::<FsIoVec>() == 16);
    const _: () = assert!(core::mem::size_of::<FsDirEntry>() == 296);
    const _: () = assert!(core::mem::size_of::<DevfsBrokerEntry>() == 272);
    const _: () = assert!(core::mem::size_of::<FsTerminalState>() == 12);
    const _: () = assert!(core::mem::size_of::<FsProviderOps>() == FS_PROVIDER_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<DevfsBrokerOps>() == DEVFS_BROKER_OPS_SIZE as usize);
    const _: () = assert!(core::mem::size_of::<ClassDef>() == CLASS_DEF_SIZE as usize);
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
pub const IRQ_RESCHEDULE: u32 = 1 << 2;
pub const MATCH_COMPATIBLE: u32 = 1;
pub const NODE_CHARACTER: u32 = 1;
pub const OPEN_NONBLOCK: u32 = 1 << 8;
pub const POLL_IN: u16 = 0x0001;
pub const POLL_OUT: u16 = 0x0004;
pub const POLL_HUP: u16 = 0x0010;
pub const POLL_NVAL: u16 = 0x0020;
pub const POLL_RDNORM: u16 = 0x0040;
pub const POLL_WRNORM: u16 = 0x0100;
pub const CONSOLE_RESET_ON_LAST_CLOSE: u64 = 1;
pub const FS_KIND_REGULAR: u32 = 1;
pub const FS_KIND_DIRECTORY: u32 = 2;
pub const FS_KIND_SYMLINK: u32 = 3;
pub const FS_KIND_CHARACTER_DEVICE: u32 = 4;
pub const FS_KIND_BLOCK_DEVICE: u32 = 5;
pub const FS_CREATE_REGULAR: u32 = 1;
pub const FS_CREATE_DIRECTORY: u32 = 2;
pub const FS_CREATE_SYMLINK: u32 = 3;
pub const FS_SETATTR_SIZE: u32 = 1;
pub const FS_SETATTR_MODE: u32 = 1 << 1;
pub const DEVFS_EVENT_READABLE: u32 = 1;
pub const DEVFS_EVENT_WRITABLE: u32 = 2;
pub const DEVFS_EVENT_HANGUP: u32 = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Error(i32);

impl Error {
    #[must_use]
    pub const fn from_status(status: i32) -> Self {
        Self(status)
    }

    #[must_use]
    pub const fn status(self) -> i32 {
        self.0
    }
}

pub type Result<T> = core::result::Result<T, Error>;

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
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let size = layout.size().max(1);
        let align = layout.align();
        // SAFETY: `Layout` supplies a non-zero power-of-two alignment.
        unsafe { imports::alloc(size, align).cast() }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let size = layout.size().max(1);
        let align = layout.align();
        // SAFETY: `Layout` supplies a non-zero power-of-two alignment.
        unsafe { imports::alloc_zeroed(size, align).cast() }
    }

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

#[derive(Clone, Copy)]
pub struct Module(NonNull<raw::Module>);

impl Module {
    /// # Safety
    /// `pointer` must be the live module callback argument supplied by the
    /// loader; it is never dereferenced by this wrapper.
    pub unsafe fn from_raw(pointer: *mut raw::Module) -> Option<Self> {
        NonNull::new(pointer).map(Self)
    }

    const fn as_ptr(self) -> *const raw::Module {
        self.0.as_ptr().cast_const()
    }
}

fn module() -> Result<Module> {
    let pointer = SELF.load(Ordering::Acquire);
    // SAFETY: `module!` stores only the live loader callback argument.
    unsafe { Module::from_raw(pointer) }.ok_or(Error::from_status(EINVAL))
}

#[derive(Clone, Copy)]
pub struct Bus(*const raw::Bus);

// SAFETY: this is an opaque, immutable kernel registration handle. The wrapper
// never dereferences it; synchronization belongs to the kernel bus registry.
unsafe impl Send for Bus {}
// SAFETY: see `Send`; copying the handle does not grant mutable access.
unsafe impl Sync for Bus {}

impl Bus {
    pub fn find(name: &CStr) -> Result<Self> {
        let mut bus = ptr::null();
        // SAFETY: `name` and `bus` remain valid for this immediate C ABI call.
        unsafe { result(imports::bus_find(name.as_ptr(), &raw mut bus))? };
        NonNull::new(bus.cast_mut())
            .map(|value| Self(value.as_ptr().cast_const()))
            .ok_or(Error::from_status(EINVAL))
    }

    #[must_use]
    pub const fn as_raw(self) -> *const raw::Bus {
        self.0
    }
}

#[derive(Clone, Copy)]
pub struct Device(*const raw::Device);

// SAFETY: this is an opaque kernel-owned handle. The wrapper only passes it
// back through documented kernel calls, which synchronize device state.
unsafe impl Send for Device {}
// SAFETY: see `Send`; no Rust reference to kernel device memory is created.
unsafe impl Sync for Device {}

impl Device {
    /// # Safety
    /// `pointer` must remain a kernel-owned device handle for the wrapper's
    /// use. Probe and remove callbacks meet this requirement.
    pub unsafe fn from_raw(pointer: *const raw::Device) -> Option<Self> {
        NonNull::new(pointer.cast_mut()).map(|value| Self(value.as_ptr().cast_const()))
    }

    #[must_use]
    pub const fn as_raw(self) -> *const raw::Device {
        self.0
    }

    pub fn integer(self, name: &CStr) -> Result<u64> {
        let mut value = 0;
        // SAFETY: device and property name are kernel/static handles, and
        // `value` is writable for this immediate call.
        unsafe { result(imports::device_int(self.0, name.as_ptr(), &raw mut value))? };
        Ok(value)
    }

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

    #[must_use]
    pub fn property_len(self, name: &CStr) -> usize {
        // SAFETY: device and property name remain valid for the call.
        unsafe { imports::device_property_len(self.0, name.as_ptr()) }
    }

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
        // SAFETY: upheld by the caller; the kernel only stores this opaque
        // driver-private pointer.
        unsafe { imports::device_set_data(self.0, value) };
        Ok(())
    }

    /// # Safety
    /// The caller must validate the returned opaque pointer before use.
    pub unsafe fn data(self) -> *mut c_void {
        // SAFETY: the kernel returns an opaque pointer without dereferencing it.
        unsafe { imports::device_data(self.0) }
    }

    pub fn remove(self) -> Result<()> {
        // SAFETY: the device is kernel-owned and valid until removal returns.
        unsafe { result(imports::device_remove(self.0)) }
    }
}

pub struct Resource {
    pub start: u64,
    pub length: u64,
    pub flags: u32,
}

pub struct DeviceBuilder {
    raw: Option<NonNull<raw::DeviceBuilder>>,
}

impl DeviceBuilder {
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

    pub fn set_bus(&mut self, bus: Bus) -> Result<()> {
        // SAFETY: both opaque handles are valid during the call.
        unsafe { result(imports::device_set_bus(self.pointer()?, bus.as_raw())) }
    }

    pub fn add_u32(&mut self, name: &CStr, value: u32) -> Result<()> {
        // SAFETY: builder and property name are valid during the immediate call.
        unsafe {
            result(imports::device_add_int(
                self.pointer()?,
                name.as_ptr(),
                value.into(),
                0,
            ))
        }
    }

    pub fn add_strings(&mut self, name: &CStr, values: &[*const c_char]) -> Result<()> {
        // SAFETY: `values` holds NUL-terminated static strings for this call.
        unsafe {
            result(imports::device_add_strings(
                self.pointer()?,
                name.as_ptr(),
                values.as_ptr(),
                values.len(),
            ))
        }
    }

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

    pub fn publish(mut self) -> Result<Device> {
        let mut device = ptr::null();
        // SAFETY: this transfers the builder receipt to the kernel exactly once.
        unsafe { result(imports::device_add(self.pointer()?, &raw mut device))? };
        self.raw = None;
        // SAFETY: the service returns a live, kernel-owned device handle.
        unsafe { Device::from_raw(device) }.ok_or(Error::from_status(EINVAL))
    }
}

impl Drop for DeviceBuilder {
    fn drop(&mut self) {
        let Some(builder) = self.raw.take() else {
            return;
        };
        // SAFETY: this is the unique, unconsumed builder receipt.
        unsafe { imports::device_discard(builder.as_ptr()) };
    }
}

/// Receipt for a registered device class.
pub struct Class(Option<NonNull<raw::Class>>);

// SAFETY: class registration ownership is serialized by the provider that
// stores this receipt; the opaque pointer is never dereferenced in Rust.
unsafe impl Send for Class {}

impl Class {
    /// Registers a class owned by this module.
    ///
    /// # Safety
    /// `definition` must remain immutable and valid until `unregister`
    /// completes.
    pub unsafe fn register(name: &CStr, definition: &'static raw::ClassDef) -> Result<Self> {
        let mut class = ptr::null();
        // SAFETY: module, name, immutable definition, and output storage are
        // valid for this immediate ABI call.
        unsafe {
            result(imports::class_register(
                module()?.as_ptr(),
                name.as_ptr(),
                definition,
                &raw mut class,
            ))?;
        }
        NonNull::new(class.cast_mut())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    fn pointer(&self) -> Result<*const raw::Class> {
        self.0
            .map(|value| value.as_ptr().cast_const())
            .ok_or(Error::from_status(EINVAL))
    }

    /// Adds a class membership for `device`.
    ///
    /// # Safety
    /// `operations` and `context` must remain valid until the returned
    /// membership is removed.
    pub unsafe fn add(
        &self,
        device: Option<Device>,
        name: &CStr,
        operations: *const c_void,
        operations_size: usize,
        context: *mut c_void,
    ) -> Result<ClassDevice> {
        let mut member = ptr::null();
        // SAFETY: all opaque handles and the membership table meet this
        // method's contract, and `member` is writable local output storage.
        unsafe {
            result(imports::class_add(
                module()?.as_ptr(),
                self.pointer()?,
                device.map_or(ptr::null(), Device::as_raw),
                name.as_ptr(),
                operations,
                operations_size,
                context,
                &raw mut member,
            ))?;
        }
        NonNull::new(member.cast_mut())
            .map(|value| ClassDevice(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    pub fn unregister(&mut self) -> Result<()> {
        let Some(class) = self.0.take() else {
            return Ok(());
        };
        // SAFETY: this uniquely consumes the class registration receipt.
        unsafe { result(imports::class_unregister(class.as_ptr().cast_const())) }
    }
}

/// Receipt for one module-owned class membership.
pub struct ClassDevice(Option<NonNull<raw::ClassDevice>>);

// SAFETY: membership removal is serialized by its terminal owner and the
// opaque pointer is never dereferenced by Rust.
unsafe impl Send for ClassDevice {}

impl ClassDevice {
    pub fn remove(&mut self) {
        let Some(member) = self.0.take() else {
            return;
        };
        // SAFETY: this uniquely consumes the membership receipt.
        unsafe { imports::class_remove(member.as_ptr().cast_const()) };
    }
}

/// Receipt returned by registering a driver with a bus.
pub struct DriverRegistration(Option<NonNull<raw::Driver>>);

// SAFETY: the receipt is consumed only while held in a `TicketLock`; it is an
// opaque kernel handle and this wrapper never dereferences it.
unsafe impl Send for DriverRegistration {}

impl DriverRegistration {
    /// Registers an immutable C ABI definition and owns the returned receipt.
    ///
    /// # Safety
    /// The match table, definition, callbacks, and callback context must stay
    /// valid until `unregister` completes.
    pub unsafe fn register(definition: &'static raw::DriverDef) -> Result<Self> {
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

    pub fn unregister(&mut self) -> Result<()> {
        let Some(driver) = self.0.take() else {
            return Ok(());
        };
        // SAFETY: this uniquely consumes the registration receipt.
        unsafe { result(imports::driver_unregister(driver.as_ptr().cast_const())) }
    }
}

pub struct Mmio {
    raw: raw::Mmio,
}

// SAFETY: a mapping receipt owns no Rust references. Moving it between CPUs is
// safe when the driver serializes device register access, as all DDK drivers do.
unsafe impl Send for Mmio {}

impl Mmio {
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
        if raw.base.is_null() || raw.length < length {
            return Err(Error::from_status(EINVAL));
        }
        Ok(Self { raw })
    }

    pub fn direct(physical: u64) -> Result<*const u8> {
        let mut output = ptr::null_mut();
        // SAFETY: `output` is writable local storage.
        unsafe { result(imports::mmio_direct(physical, &raw mut output))? };
        NonNull::new(output)
            .map(|value| value.as_ptr().cast_const().cast())
            .ok_or(Error::from_status(EINVAL))
    }

    fn address(&self, offset: usize, width: usize) -> *mut u8 {
        assert!(
            offset
                .checked_add(width)
                .is_some_and(|end| end <= self.raw.length)
        );
        // SAFETY: the range assertion proves this pointer remains within the
        // kernel mapping, which owns the full `raw.length` extent.
        unsafe { self.raw.base.cast::<u8>().add(offset) }
    }

    #[must_use]
    pub fn read8(&self, offset: usize) -> u8 {
        // SAFETY: `address` validates bounds and MMIO must use volatile access.
        unsafe { ptr::read_volatile(self.address(offset, 1)) }
    }

    #[must_use]
    pub fn read16(&self, offset: usize) -> u16 {
        // SAFETY: resource alignment is a hardware contract established by the
        // platform description; the volatile access matches the C driver.
        unsafe { ptr::read_volatile(self.address(offset, 2).cast()) }
    }

    #[must_use]
    pub fn read32(&self, offset: usize) -> u32 {
        // SAFETY: see `read16`; the access width matches the caller's device.
        unsafe { ptr::read_volatile(self.address(offset, 4).cast()) }
    }

    pub fn write8(&self, offset: usize, value: u8) {
        // SAFETY: `address` validates bounds and MMIO must use volatile access.
        unsafe { ptr::write_volatile(self.address(offset, 1), value) };
    }

    pub fn write16(&self, offset: usize, value: u16) {
        // SAFETY: see `read16`; the access width matches the caller's device.
        unsafe { ptr::write_volatile(self.address(offset, 2).cast(), value) };
    }

    pub fn write32(&self, offset: usize, value: u32) {
        // SAFETY: see `read16`; the access width matches the caller's device.
        unsafe { ptr::write_volatile(self.address(offset, 4).cast(), value) };
    }
}

impl Drop for Mmio {
    fn drop(&mut self) {
        if self.raw.token.is_null() {
            return;
        }
        // SAFETY: this mapping receipt is owned uniquely by `self`.
        let _ = unsafe { imports::mmio_unmap(&raw mut self.raw) };
        self.raw.token = ptr::null_mut();
    }
}

pub struct TicketLock<T> {
    next: AtomicU32,
    serving: AtomicU32,
    value: UnsafeCell<T>,
}

// SAFETY: the ticket protocol permits access only through its exclusive guards.
// `SpinGuard` is limited to thread-context state; interrupt-shared state uses
// `IrqGuard` and keeps local interrupts disabled.
unsafe impl<T: Send> Sync for TicketLock<T> {}

impl<T> TicketLock<T> {
    #[must_use]
    pub const fn new(value: T) -> Self {
        Self {
            next: AtomicU32::new(0),
            serving: AtomicU32::new(0),
            value: UnsafeCell::new(value),
        }
    }

    pub fn lock_irqsave(&self) -> IrqGuard<'_, T> {
        let flags = irq_save();
        self.lock_ticket();
        IrqGuard {
            lock: self,
            flags,
            marker: PhantomData,
        }
    }

    /// Acquires a ticket lock without changing interrupt state.
    ///
    /// This variant is for thread-context state that is never touched by an
    /// interrupt handler. Callers must not sleep or invoke a blocking backend
    /// operation while the returned guard is live.
    pub fn lock(&self) -> SpinGuard<'_, T> {
        self.lock_ticket();
        SpinGuard {
            lock: self,
            marker: PhantomData,
        }
    }

    /// Attempts to acquire an uncontended non-IRQ ticket lock.
    pub fn try_lock(&self) -> Option<SpinGuard<'_, T>> {
        let serving = self.serving.load(Ordering::Acquire);
        self.next
            .compare_exchange(
                serving,
                serving.wrapping_add(1),
                Ordering::Acquire,
                Ordering::Relaxed,
            )
            .ok()
            .map(|_| SpinGuard {
                lock: self,
                marker: PhantomData,
            })
    }

    fn lock_ticket(&self) {
        let ticket = self.next.fetch_add(1, Ordering::Relaxed);
        while self.serving.load(Ordering::Acquire) != ticket {
            cpu_relax();
        }
    }
}

/// Guard for [`TicketLock::lock`].
pub struct SpinGuard<'a, T> {
    lock: &'a TicketLock<T>,
    marker: PhantomData<&'a mut T>,
}

impl<T> Deref for SpinGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        // SAFETY: the ticket lock is held by this guard, giving unique mutable
        // access; reborrowing it immutably is therefore valid.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for SpinGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: the ticket lock is held by this guard and its lifetime
        // prevents another guard from accessing the value concurrently.
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for SpinGuard<'_, T> {
    fn drop(&mut self) {
        self.lock.serving.fetch_add(1, Ordering::Release);
    }
}

pub struct IrqGuard<'a, T> {
    lock: &'a TicketLock<T>,
    flags: usize,
    marker: PhantomData<&'a mut T>,
}

impl<T> Deref for IrqGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        // SAFETY: the ticket lock is held by this guard, giving unique mutable
        // access; reborrowing it immutably is therefore valid.
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
    fn drop(&mut self) {
        self.lock.serving.fetch_add(1, Ordering::Release);
        irq_restore(self.flags);
    }
}

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

#[inline]
fn irq_save() -> usize {
    #[cfg(target_arch = "x86_64")]
    {
        let flags: usize;
        // SAFETY: this is the exact interrupt-state sequence used by the C
        // DDK. It returns the prior flags and does not touch Rust stack data.
        unsafe { core::arch::asm!("pushfq", "pop {}", "cli", out(reg) flags, options(nomem)) };
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
                options(nomem)
            );
        };
        previous
    }
}

#[inline]
fn irq_restore(state: usize) {
    #[cfg(target_arch = "x86_64")]
    if state & (1 << 9) != 0 {
        // SAFETY: restores only the interrupt-enable state captured by
        // `irq_save`, matching the C DDK implementation.
        unsafe { core::arch::asm!("sti", options(nomem, nostack)) };
    }
    #[cfg(target_arch = "riscv64")]
    if state & (1 << 1) != 0 {
        // SAFETY: restores SIE only if it was set by the saved status word.
        unsafe { core::arch::asm!("csrs sstatus, {}", in(reg) 2usize, options(nomem, nostack)) };
    }
}

#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct Event(usize);

impl Event {
    pub fn create() -> Result<Self> {
        let mut event = 0;
        // SAFETY: module is live and event is writable output storage.
        unsafe { result(imports::event_create(module()?.as_ptr(), &raw mut event))? };
        if event == 0 {
            Err(Error::from_status(EINVAL))
        } else {
            Ok(Self(event))
        }
    }

    #[must_use]
    pub const fn id(self) -> usize {
        self.0
    }

    #[must_use]
    pub const fn from_id(id: usize) -> Option<Self> {
        if id == 0 { None } else { Some(Self(id)) }
    }

    pub fn signal(self) -> Result<()> {
        event_call(imports::event_signal, self.0)
    }
    pub fn reset(self) -> Result<()> {
        event_call(imports::event_reset, self.0)
    }
    pub fn wait(self) -> Result<()> {
        event_call(imports::event_wait, self.0)
    }
    pub fn destroy(self) -> Result<()> {
        event_call(imports::event_destroy, self.0)
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
pub struct Worker(Option<NonNull<raw::Worker>>);

// SAFETY: worker ownership is transferred between terminal lifecycle code; the
// opaque kernel receipt is never dereferenced in Rust.
unsafe impl Send for Worker {}

impl Worker {
    /// Starts a worker owned by the current module.
    ///
    /// # Safety
    /// `entry` and `context` must remain valid until `join` returns.
    pub unsafe fn spawn(entry: raw::WorkerEntry, context: *mut c_void) -> Result<Self> {
        let mut worker = ptr::null_mut::<c_void>();
        // SAFETY: the callback and context meet this method's contract and
        // `worker` is writable output storage.
        let status =
            unsafe { imports::worker_spawn(module()?.as_ptr(), entry, context, &raw mut worker) };
        result(status)?;
        NonNull::new(worker.cast())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Waits for the worker to finish and releases its kernel receipt.
    pub fn join(&mut self) -> Result<()> {
        let Some(worker) = self.0.take() else {
            return Ok(());
        };
        // SAFETY: this uniquely consumes the worker receipt.
        let status = unsafe { imports::worker_join(worker.as_ptr().cast()) };
        result(status)
    }
}

fn event_call(function: unsafe extern "C" fn(usize) -> i32, event: usize) -> Result<()> {
    // SAFETY: event IDs are issued by the kernel and the import has the
    // matching versioned C ABI signature.
    unsafe { result(function(event)) }
}

pub struct IrqDomain(Option<NonNull<raw::IrqDomain>>);

// SAFETY: registration lifetime is externally synchronized by its owner.
unsafe impl Send for IrqDomain {}

impl IrqDomain {
    pub fn register(
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

    pub fn unregister(&mut self) -> Result<()> {
        let Some(domain) = self.0.take() else {
            return Ok(());
        };
        // SAFETY: this consumes this unique kernel-issued domain receipt.
        unsafe { result(imports::irq_domain_unregister(domain.as_ptr().cast_const())) }
    }
}

pub struct Irq(Option<NonNull<raw::Irq>>);

// SAFETY: IRQ release is synchronized by the kernel and the wrapper never
// dereferences the opaque action pointer.
unsafe impl Send for Irq {}

impl Irq {
    pub fn request(
        device: Device,
        virq: u32,
        name: &CStr,
        flags: u32,
        handler: unsafe extern "C" fn(*mut c_void, u32) -> u32,
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
                None,
                context,
                &raw mut irq,
            ))?;
        };
        NonNull::new(irq)
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    pub fn release(&mut self) -> Result<()> {
        let Some(irq) = self.0.take() else {
            return Ok(());
        };
        // SAFETY: this uniquely consumes the request receipt.
        unsafe { result(imports::irq_release(irq.as_ptr())) }
    }
}

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

pub fn irq_alloc_vector(virq: u32) -> Result<u32> {
    let mut vector = 0;
    // SAFETY: vector is writable local output storage.
    unsafe { result(imports::irq_alloc_vector(virq, &raw mut vector))? };
    Ok(vector)
}

pub fn irq_free_vector(vector: u32) {
    // SAFETY: vector came from `irq_alloc_vector` in this module.
    unsafe { imports::irq_free_vector(vector) };
}

pub fn cpu_platform_id(cpu: u32) -> Result<u64> {
    let mut id = 0;
    // SAFETY: id is writable local output storage.
    unsafe { result(imports::cpu_platform_id(cpu, &raw mut id))? };
    Ok(id)
}

#[must_use]
pub fn cpu_count() -> u32 {
    // SAFETY: the service takes no pointers and returns the current topology size.
    unsafe { imports::cpu_count() }
}

pub fn firmware_acpi(bytes: &mut [u8]) -> Result<usize> {
    firmware(imports::firmware_acpi, bytes)
}
pub fn firmware_devicetree(bytes: &mut [u8]) -> Result<usize> {
    firmware(imports::firmware_devicetree, bytes)
}

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

pub struct Devfs;

impl Devfs {
    pub fn current() -> Result<Self> {
        let _ = module()?;
        Ok(Self)
    }

    pub fn root(&self) -> Result<u64> {
        let mut node = 0;
        // SAFETY: the versioned import has the C ABI checked in kernel exports.
        let status = unsafe { imports::devfs_root(&raw mut node) };
        result(status)?;
        Ok(node)
    }

    pub fn mkdir(&self, parent: u64, name: &CStr, mode: u16) -> Result<u64> {
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
    /// size-prefixed table during this call; its context and callback targets
    /// must remain valid until the node is removed.
    pub unsafe fn create_character(
        &self,
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
                ptr::null(),
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

    pub fn remove(&self, node: u64) -> Result<()> {
        // SAFETY: the node receipt belongs to this module and is consumed by
        // the kernel's device-filesystem removal path.
        let status = unsafe { imports::devfs_remove(module()?.as_ptr(), node) };
        result(status)
    }
}

/// Registers a filesystem-provider table and returns its opaque registration
/// receipt.  The table must remain immutable until it is unregistered.
pub fn fs_provider_register(
    name: &CStr,
    operations: &'static raw::FsProviderOps,
) -> Result<NonNull<raw::FsProvider>> {
    let mut provider = ptr::null_mut();
    // SAFETY: the static table, C string, and local output remain valid for
    // this immediate C ABI registration call.
    unsafe {
        result(imports::fs_provider_register(
            module()?.as_ptr(),
            name.as_ptr(),
            operations,
            &raw mut provider,
        ))?;
    }
    NonNull::new(provider).ok_or(Error::from_status(EINVAL))
}

/// Removes a filesystem-provider registration.
///
/// # Safety
/// `provider` must be the live receipt returned by [`fs_provider_register`].
pub unsafe fn fs_provider_unregister(provider: NonNull<raw::FsProvider>) -> Result<()> {
    // SAFETY: upheld by this function's safety contract.
    unsafe { result(imports::fs_provider_unregister(provider.as_ptr())) }
}

pub fn fs_page_account_create(limit: u64) -> Result<NonNull<raw::FsPageAccount>> {
    let mut account = ptr::null_mut();
    // SAFETY: account is writable local output storage.
    let status = unsafe { imports::fs_page_account_create(limit, &raw mut account) };
    result(status)?;
    NonNull::new(account).ok_or(Error::from_status(EINVAL))
}

/// # Safety
/// `account` must be an owned account receipt.
pub unsafe fn fs_page_account_release(account: NonNull<raw::FsPageAccount>) {
    // SAFETY: upheld by this function's safety contract.
    unsafe { imports::fs_page_account_release(account.as_ptr()) };
}

pub fn fs_page_account_limit(account: NonNull<raw::FsPageAccount>) -> Result<u64> {
    let mut value = 0;
    // SAFETY: account is opaque and value is writable local output storage.
    let status = unsafe { imports::fs_page_account_limit(account.as_ptr(), &raw mut value) };
    result(status)?;
    Ok(value)
}

pub fn fs_page_account_used(account: NonNull<raw::FsPageAccount>) -> Result<u64> {
    let mut value = 0;
    // SAFETY: account is opaque and value is writable local output storage.
    let status = unsafe { imports::fs_page_account_used(account.as_ptr(), &raw mut value) };
    result(status)?;
    Ok(value)
}

pub fn fs_memory_object_create(
    account: NonNull<raw::FsPageAccount>,
) -> Result<NonNull<raw::FsMemoryObject>> {
    let mut object = ptr::null_mut();
    // SAFETY: account is opaque and object is writable local output storage.
    let status = unsafe { imports::fs_memory_object_create(account.as_ptr(), &raw mut object) };
    result(status)?;
    NonNull::new(object).ok_or(Error::from_status(EINVAL))
}

/// # Safety
/// `object` must be a live memory-object receipt.
pub unsafe fn fs_memory_object_retain(object: NonNull<raw::FsMemoryObject>) -> Result<()> {
    // SAFETY: upheld by this function's safety contract.
    unsafe { result(imports::fs_memory_object_retain(object.as_ptr())) }
}

/// # Safety
/// `object` must be an owned memory-object receipt.
pub unsafe fn fs_memory_object_release(object: NonNull<raw::FsMemoryObject>) {
    // SAFETY: upheld by this function's safety contract.
    unsafe { imports::fs_memory_object_release(object.as_ptr()) };
}

pub fn fs_memory_object_read(
    object: NonNull<raw::FsMemoryObject>,
    offset: u64,
    output: &mut [u8],
) -> Result<usize> {
    // SAFETY: output is writable for exactly its slice length.
    let value = unsafe {
        imports::fs_memory_object_read(object.as_ptr(), offset, output.as_mut_ptr(), output.len())
    };
    signed_count(value, output.len())
}

pub fn fs_memory_object_write(
    object: NonNull<raw::FsMemoryObject>,
    offset: u64,
    input: &[u8],
) -> Result<usize> {
    // SAFETY: input is readable for exactly its slice length.
    let value = unsafe {
        imports::fs_memory_object_write(object.as_ptr(), offset, input.as_ptr(), input.len())
    };
    signed_count(value, input.len())
}

pub fn fs_memory_object_truncate(object: NonNull<raw::FsMemoryObject>, size: u64) -> Result<u64> {
    let mut removed = 0;
    // SAFETY: object is opaque and removed is writable local output storage.
    let status =
        unsafe { imports::fs_memory_object_truncate(object.as_ptr(), size, &raw mut removed) };
    result(status)?;
    Ok(removed)
}

pub fn fs_memory_object_page_count(object: NonNull<raw::FsMemoryObject>) -> Result<u64> {
    let mut count = 0;
    // SAFETY: object is opaque and count is writable local output storage.
    let status = unsafe { imports::fs_memory_object_page_count(object.as_ptr(), &raw mut count) };
    result(status)?;
    Ok(count)
}

pub fn fs_total_physical_pages() -> u64 {
    // SAFETY: this versioned import takes no arguments.
    unsafe { imports::fs_total_physical_pages() }
}

fn signed_count(value: i64, capacity: usize) -> Result<usize> {
    if value < 0 {
        return Err(Error::from_status(value as i32));
    }
    let count = usize::try_from(value).map_err(|_| Error::from_status(EINVAL))?;
    if count > capacity {
        Err(Error::from_status(EINVAL))
    } else {
        Ok(count)
    }
}

pub fn devfs_broker_register(
    operations: &'static raw::DevfsBrokerOps,
) -> Result<NonNull<raw::DevfsBroker>> {
    let mut broker = ptr::null_mut();
    // SAFETY: table is static and broker is local writable output storage.
    unsafe {
        result(imports::devfs_broker_register(
            module()?.as_ptr(),
            operations,
            &raw mut broker,
        ))?;
    };
    NonNull::new(broker).ok_or(Error::from_status(EINVAL))
}

/// # Safety
/// `broker` must be an owned devfs broker registration receipt.
pub unsafe fn devfs_broker_unregister(broker: NonNull<raw::DevfsBroker>) -> Result<()> {
    // SAFETY: upheld by this function's safety contract.
    unsafe { result(imports::devfs_broker_unregister(broker.as_ptr())) }
}

/// Endpoint operations used by the devfs provider.  Each method borrows an
/// endpoint held in a devfs vnode receipt; `release` consumes that receipt.
pub struct DevfsEndpoint;

impl DevfsEndpoint {
    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn open(endpoint: *mut raw::DevfsEndpoint, flags: u32) -> Result<usize> {
        let mut file = 0;
        // SAFETY: upheld by this function's safety contract.
        let status = unsafe { imports::devfs_endpoint_open(endpoint, flags, &raw mut file) };
        result(status)?;
        Ok(file)
    }

    /// # Safety
    /// `endpoint` and `file` must be the matching live endpoint/open receipt.
    pub unsafe fn close(endpoint: *mut raw::DevfsEndpoint, file: usize, flags: u32) {
        // SAFETY: upheld by this function's safety contract.
        unsafe { imports::devfs_endpoint_close(endpoint, file, flags) };
    }

    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn initial_offset(
        endpoint: *mut raw::DevfsEndpoint,
        file: usize,
        flags: u32,
    ) -> Result<u64> {
        let mut offset = 0;
        // SAFETY: upheld by this function's safety contract.
        let status = unsafe {
            imports::devfs_endpoint_initial_offset(endpoint, file, flags, &raw mut offset)
        };
        result(status)?;
        Ok(offset)
    }

    /// # Safety
    /// `endpoint` must be live and `output` writable for its slice length.
    pub unsafe fn read(
        endpoint: *mut raw::DevfsEndpoint,
        file: usize,
        offset: u64,
        output: &mut [u8],
        flags: u32,
    ) -> Result<usize> {
        // SAFETY: upheld by contract and output is writable for its length.
        let value = unsafe {
            imports::devfs_endpoint_read(
                endpoint,
                file,
                offset,
                output.as_mut_ptr(),
                output.len(),
                flags,
            )
        };
        signed_count(value, output.len())
    }

    /// # Safety
    /// `endpoint` must be live and `input` readable for its slice length.
    pub unsafe fn write(
        endpoint: *mut raw::DevfsEndpoint,
        file: usize,
        offset: u64,
        input: &[u8],
        flags: u32,
    ) -> Result<usize> {
        // SAFETY: upheld by contract and input is readable for its length.
        let value = unsafe {
            imports::devfs_endpoint_write(
                endpoint,
                file,
                offset,
                input.as_ptr(),
                input.len(),
                flags,
            )
        };
        signed_count(value, input.len())
    }

    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn size(endpoint: *mut raw::DevfsEndpoint) -> u64 {
        // SAFETY: upheld by this function's safety contract.
        unsafe { imports::devfs_endpoint_size(endpoint) }
    }

    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn sync(endpoint: *mut raw::DevfsEndpoint) -> Result<()> {
        // SAFETY: upheld by this function's safety contract.
        unsafe { result(imports::devfs_endpoint_sync(endpoint)) }
    }

    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn poll(
        endpoint: *mut raw::DevfsEndpoint,
        file: usize,
        offset: u64,
        events: u16,
        flags: u32,
    ) -> Result<u16> {
        // SAFETY: upheld by this function's safety contract.
        let value = unsafe { imports::devfs_endpoint_poll(endpoint, file, offset, events, flags) };
        if value < 0 {
            Err(Error::from_status(value as i32))
        } else {
            u16::try_from(value).map_err(|_| Error::from_status(EINVAL))
        }
    }

    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn event(endpoint: *mut raw::DevfsEndpoint, file: usize, selector: u32) -> usize {
        // SAFETY: upheld by this function's safety contract.
        unsafe { imports::devfs_endpoint_event(endpoint, file, selector) }
    }

    /// # Safety
    /// `endpoint` must be a live kernel endpoint receipt.
    pub unsafe fn terminal_state(
        endpoint: *mut raw::DevfsEndpoint,
        state: &mut raw::FsTerminalState,
    ) -> Result<()> {
        // SAFETY: upheld by this function's safety contract.
        unsafe { result(imports::devfs_endpoint_terminal_state(endpoint, state)) }
    }

    /// # Safety
    /// `endpoint` must be live and `argument` writable for its slice length.
    #[allow(clippy::too_many_arguments)]
    pub unsafe fn ioctl(
        endpoint: *mut raw::DevfsEndpoint,
        file: usize,
        process: u64,
        group: i32,
        session: i32,
        session_leader: bool,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> Result<u64> {
        // SAFETY: upheld by contract and argument is writable for its length.
        let result = unsafe {
            imports::devfs_endpoint_ioctl(
                endpoint,
                file,
                process,
                group,
                session,
                u8::from(session_leader),
                request,
                value,
                argument.as_mut_ptr(),
                argument.len(),
            )
        };
        if result < 0 {
            Err(Error::from_status(result as i32))
        } else {
            u64::try_from(result).map_err(|_| Error::from_status(EINVAL))
        }
    }

    /// # Safety
    /// `endpoint` must be the owned endpoint receipt passed to devfs create.
    pub unsafe fn release(endpoint: *mut raw::DevfsEndpoint) {
        // SAFETY: upheld by this function's safety contract.
        unsafe { imports::devfs_endpoint_release(endpoint) };
    }
}

pub struct Tty(Option<NonNull<raw::Tty>>);

// SAFETY: terminal receipt use is serialized by its driver state lock.
unsafe impl Send for Tty {}

impl Tty {
    /// # Safety
    /// `operations` must remain immutable and valid for the terminal's
    /// lifetime. Its context ownership stays with the caller.
    pub unsafe fn register(
        device: Option<Device>,
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
                device.map_or(ptr::null(), Device::as_raw),
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

/// Receipt for the single registered terminal provider.
pub struct TtyProvider(Option<NonNull<raw::TtyProvider>>);

// SAFETY: provider lifecycle is serialized by the console module's global
// state and the opaque receipt is never dereferenced by Rust.
unsafe impl Send for TtyProvider {}

impl TtyProvider {
    /// Registers the terminal-semantics provider with the kernel broker.
    ///
    /// # Safety
    /// `operations` must remain immutable until `unregister` returns.
    pub unsafe fn register(operations: &'static raw::TtyProviderOps) -> Result<Self> {
        let mut provider = ptr::null_mut::<c_void>();
        // SAFETY: upheld by this method's contract and `provider` is writable
        // local output storage.
        let status = unsafe {
            imports::tty_provider_register(module()?.as_ptr(), operations, &raw mut provider)
        };
        result(status)?;
        NonNull::new(provider.cast())
            .map(|value| Self(Some(value)))
            .ok_or(Error::from_status(EINVAL))
    }

    /// Unregisters the provider after all backend terminal receipts are gone.
    pub fn unregister(&mut self) -> Result<()> {
        let Some(provider) = self.0 else {
            return Ok(());
        };
        // SAFETY: the receipt remains valid until the kernel reports success.
        let status = unsafe { imports::tty_provider_unregister(provider.as_ptr().cast()) };
        result(status)?;
        self.0 = None;
        Ok(())
    }
}

/// Delivers a POSIX signal to every process in `process_group`.
pub fn signal_process_group(process_group: i32, signal: u8) -> Result<()> {
    // SAFETY: both arguments are scalar process and signal identifiers.
    let status = unsafe { imports::process_group_signal(process_group, signal) };
    result(status)
}

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

pub fn random_fill(bytes: &mut [u8]) -> Result<()> {
    if bytes.is_empty() {
        return Ok(());
    }
    // SAFETY: the slice is writable for exactly its length.
    unsafe {
        imports::random_fill(bytes.as_mut_ptr(), bytes.len());
    };
    Ok(())
}

pub fn random_mix(bytes: &[u8]) -> Result<()> {
    if bytes.is_empty() {
        return Ok(());
    }
    // SAFETY: the slice is readable for exactly its length.
    unsafe {
        imports::random_mix(bytes.as_ptr(), bytes.len());
    };
    Ok(())
}

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

pub fn port_read8(port: u16) -> u8 {
    // SAFETY: the caller selected this I/O port from a firmware resource.
    unsafe { imports::port_read8(port) as u8 }
}

pub fn port_write8(port: u16, value: u8) {
    // SAFETY: the caller selected this I/O port from a firmware resource.
    unsafe {
        imports::port_write8(port, value);
    }
}

#[doc(hidden)]
pub fn install_module(module: Module) {
    SELF.store(module.0.as_ptr(), Ordering::Release);
}

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

        unsafe extern "C" fn __rdf_module_init(pointer: *mut $crate::raw::Module) -> i32 {
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

        unsafe extern "C" fn __rdf_module_exit(pointer: *mut $crate::raw::Module) {
            // SAFETY: the loader supplies the live module callback handle.
            if let Some(module) = unsafe { $crate::Module::from_raw(pointer) } {
                ($exit)(module);
            }
        }

        #[used]
        static __RDF_MODULE_DEFINITION: $crate::raw::ModuleDef = $crate::raw::ModuleDef {
            size: $crate::raw::MODULE_DEF_SIZE,
            abi_major: $crate::raw::ABI_MAJOR,
            abi_minor: $crate::raw::ABI_MINOR,
            flags: 0,
            name: __RDF_MODULE_NAME.as_ptr().cast(),
            description: __RDF_MODULE_DESCRIPTION.as_ptr().cast(),
            init: Some(__rdf_module_init),
            exit: Some(__rdf_module_exit),
        };

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn rdf_module_entry(
            _reserved: *const core::ffi::c_void,
        ) -> *const $crate::raw::ModuleDef {
            core::ptr::addr_of!(__RDF_MODULE_DEFINITION)
        }
    };
}
