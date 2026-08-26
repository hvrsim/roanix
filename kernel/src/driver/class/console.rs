//! Terminal-provider broker.
//!
//! The kernel deliberately owns only this C-compatible registration ABI. The
//! line discipline, terminal nodes, class membership, and backend invocation
//! live in the loadable `console.ko` provider. A terminal receipt leases both
//! the provider and its backend module, so neither callback table can vanish
//! while the provider owns the terminal.

#![allow(missing_docs)]

use alloc::{sync::Arc, vec::Vec};
use core::{
    ffi::{c_char, c_void},
    mem::size_of,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::sys::sync::{Mutex, Once};

use super::super::{
    core::{
        device::Device,
        module::{self, Module, ModuleLease},
    },
    error::{Error, Result},
    obj,
};
use super::chardev::DevNodeId;

/// Console behaviour flags supplied by a byte-stream backend.
pub mod flags {
    /// Reset terminal settings after the final close.
    pub const RESET_ON_LAST_CLOSE: u64 = 1 << 0;
}

/// Terminal ioctl copy metadata retained by the language-neutral broker.
#[derive(Copy, Clone)]
pub struct IoctlSpec {
    /// Bytes copied in from userspace before dispatch.
    pub input: bool,
    /// Bytes copied back to userspace after dispatch.
    pub output: bool,
    /// Argument-buffer size.
    pub size: usize,
}

pub const TCGETS: u64 = 0x5401;
pub const TCSETS: u64 = 0x5402;
pub const TCSETSW: u64 = 0x5403;
pub const TCSETSF: u64 = 0x5404;
pub const TCSBRK: u64 = 0x5409;
pub const TCXONC: u64 = 0x540A;
pub const TCFLSH: u64 = 0x540B;
pub const TIOCEXCL: u64 = 0x540C;
pub const TIOCNXCL: u64 = 0x540D;
pub const TIOCSCTTY: u64 = 0x540E;
pub const TIOCGPGRP: u64 = 0x540F;
pub const TIOCSPGRP: u64 = 0x5410;
pub const TIOCOUTQ: u64 = 0x5411;
pub const TIOCSTI: u64 = 0x5412;
pub const TIOCGWINSZ: u64 = 0x5413;
pub const TIOCSWINSZ: u64 = 0x5414;
pub const TIOCGSOFTCAR: u64 = 0x5419;
pub const TIOCSSOFTCAR: u64 = 0x541A;
pub const FIONREAD: u64 = 0x541B;
pub const TIOCNOTTY: u64 = 0x5422;
pub const TIOCGSID: u64 = 0x5429;
pub const TIOCGPTN: u64 = 0x8004_5430;
pub const TIOCSPTLCK: u64 = 0x4004_5431;
pub const TIOCGPATH: u64 = 0x5254_0001;

const TERMIOS_SIZE: usize = 60;
const WINSIZE_SIZE: usize = 8;
const TTY_PATH_SIZE: usize = 32;

/// Returns the userspace-copy requirements for a terminal ioctl.
pub fn ioctl_spec(request: u64) -> IoctlSpec {
    match request {
        TCGETS => IoctlSpec {
            input: false,
            output: true,
            size: TERMIOS_SIZE,
        },
        TCSETS | TCSETSW | TCSETSF => IoctlSpec {
            input: true,
            output: false,
            size: TERMIOS_SIZE,
        },
        TIOCGPGRP | TIOCOUTQ | TIOCGSOFTCAR | FIONREAD | TIOCGSID | TIOCGPTN => IoctlSpec {
            input: false,
            output: true,
            size: 4,
        },
        TIOCSPGRP | TIOCSSOFTCAR | TIOCSPTLCK => IoctlSpec {
            input: true,
            output: false,
            size: 4,
        },
        TIOCSTI => IoctlSpec {
            input: true,
            output: false,
            size: 1,
        },
        TIOCGWINSZ => IoctlSpec {
            input: false,
            output: true,
            size: WINSIZE_SIZE,
        },
        TIOCSWINSZ => IoctlSpec {
            input: true,
            output: false,
            size: WINSIZE_SIZE,
        },
        TIOCGPATH => IoctlSpec {
            input: false,
            output: true,
            size: TTY_PATH_SIZE,
        },
        _ => {
            let direction = (request >> 30) & 0x3;
            IoctlSpec {
                input: direction & 1 != 0,
                output: direction & 2 != 0,
                size: ((request >> 16) & 0x3fff) as usize,
            }
        }
    }
}

/// Byte-level operations a UART, PTY, or other terminal backend implements.
///
/// This legacy ABI remains unchanged so C backends continue to register
/// through `tty_register`.
#[repr(C)]
pub struct ConsoleOps {
    /// Size of this table, allowing later revisions to append entries.
    pub size: u32,
    /// Behaviour flags drawn from [`flags`].
    pub flags: u64,
    /// Context passed to every callback.
    pub context: *mut c_void,
    /// Prepares the backend for a new file description.
    pub open: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Releases a file description.
    pub close: Option<unsafe extern "C" fn(*mut c_void)>,
    /// Returns one immediately available byte, or a negative status.
    pub try_read: Option<unsafe extern "C" fn(*mut c_void, *mut u8) -> i32>,
    /// Reads available bytes in bulk. Negative results are status codes.
    pub read: Option<unsafe extern "C" fn(*mut c_void, *mut u8, usize) -> i64>,
    /// Writes bytes, honouring the non-blocking request.
    pub write: Option<unsafe extern "C" fn(*mut c_void, *const u8, usize, u8) -> i32>,
    /// Applies serial framing settings.
    pub configure: Option<unsafe extern "C" fn(*mut c_void, *const SerialFraming) -> i32>,
    /// Waits for queued output to reach the hardware.
    pub flush: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Discards queued input.
    pub flush_input: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Discards queued output.
    pub flush_output: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Generates a break condition for the requested duration.
    pub send_break: Option<unsafe extern "C" fn(*mut c_void, u64) -> i32>,
    /// Returns whether one byte can be written without waiting.
    pub writable: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Returns whether the endpoint has disconnected.
    pub hung_up: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Returns the number of bytes still queued for output.
    pub queued_output: Option<unsafe extern "C" fn(*mut c_void) -> i64>,
    /// Releases backend state when the terminal is removed.
    pub destroy: Option<unsafe extern "C" fn(*mut c_void)>,
    /// Event signalled while input is available.
    pub readable_event: usize,
    /// Event signalled while output space is available.
    pub writable_event: usize,
    /// Event signalled after the endpoint disconnects.
    pub hangup_event: usize,
}

/// Serial framing parameters handed to a terminal backend.
#[repr(C)]
pub struct SerialFraming {
    /// Line rate in bits per second.
    pub baud: u32,
    /// Data bits per character.
    pub data_bits: u8,
    /// Stop bits per character.
    pub stop_bits: u8,
    /// Whether parity generation is enabled.
    pub parity: u8,
    /// Whether enabled parity is odd rather than even.
    pub odd_parity: u8,
}

/// Operations supplied by the modular terminal implementation.
///
/// The provider receives the original backend table unchanged. It may copy
/// that table and call it directly: the broker's backend lease keeps its code
/// and static callback table resident for the whole terminal lifetime.
#[repr(C)]
pub struct TtyProviderOps {
    /// Size of this table.
    pub size: u32,
    /// Context passed to provider callbacks.
    pub context: *mut c_void,
    /// Creates a terminal and returns a provider-owned opaque receipt.
    pub register: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *const c_void,
            *const c_void,
            u64,
            *const c_char,
            u16,
            u32,
            *const ConsoleOps,
            *mut *mut c_void,
        ) -> i32,
    >,
    /// Stops a terminal worker and releases provider-owned terminal state.
    pub unregister: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
}

// SAFETY: provider tables are immutable after registration and their context
// synchronization is the responsibility of the module that supplies them.
unsafe impl Send for TtyProviderOps {}
// SAFETY: as above.
unsafe impl Sync for TtyProviderOps {}

/// Opaque receipt for the registered terminal provider.
pub struct Provider {
    ops: TtyProviderOps,
    owner: Option<Arc<Module>>,
    terminals: AtomicU64,
}

// SAFETY: callback table execution is serialized by provider-defined state;
// the broker only reads immutable table entries.
unsafe impl Send for Provider {}
// SAFETY: as above.
unsafe impl Sync for Provider {}

/// Terminal receipt returned to a backend through the preserved ABI.
pub struct Terminal {
    provider: Arc<Provider>,
    receipt: Mutex<TerminalReceipt>,
    backend_owner: Option<Arc<Module>>,
    _provider_lease: ModuleLease,
    _backend_lease: ModuleLease,
}

enum TerminalReceipt {
    Live(*mut c_void),
    Removing,
    Removed,
}

// SAFETY: receipt teardown is serialized by `receipt`, and callback
// concurrency is handled by the provider.
unsafe impl Send for Terminal {}
// SAFETY: the immutable receipt may be retained in the broker registry.
unsafe impl Sync for Terminal {}

struct State {
    provider: Mutex<Option<Arc<Provider>>>,
    terminals: Mutex<Vec<Arc<Terminal>>>,
}

static STATE: Once<State> = Once::new();

pub(super) fn init() {
    STATE.call_once(|| State {
        provider: Mutex::new(None),
        terminals: Mutex::new(Vec::new()),
    });
}

fn state() -> Result<&'static State> {
    STATE.get().ok_or(Error::NotInitialized)
}

/// Registers the one modular terminal provider.
///
/// # Safety
///
/// `ops` must be immutable and executable until [`unregister_provider`]
/// returns.
pub unsafe fn register_provider(
    owner: Option<&Arc<Module>>,
    ops: TtyProviderOps,
) -> Result<Arc<Provider>> {
    if (ops.size as usize) < size_of::<TtyProviderOps>()
        || ops.register.is_none()
        || ops.unregister.is_none()
    {
        return Err(Error::InvalidArgument);
    }
    let state = state()?;
    let mut registered = state.provider.lock();
    if registered.is_some() {
        return Err(Error::AlreadyExists);
    }
    let provider = Arc::new(Provider {
        ops,
        owner: owner.cloned(),
        terminals: AtomicU64::new(0),
    });
    *registered = Some(provider.clone());
    Ok(provider)
}

/// Removes the provider once every backend terminal has gone away.
pub fn unregister_provider(provider: &Arc<Provider>) -> Result<()> {
    if provider.terminals.load(Ordering::Acquire) != 0 {
        return Err(Error::Busy);
    }
    let state = state()?;
    let mut registered = state.provider.lock();
    let Some(current) = registered.as_ref() else {
        return Err(Error::NotFound);
    };
    if !Arc::ptr_eq(current, provider) {
        return Err(Error::InvalidArgument);
    }
    // Backend registration increments the terminal count while holding this
    // same lock, so recheck after acquiring it before withdrawing callbacks.
    if provider.terminals.load(Ordering::Acquire) != 0 {
        return Err(Error::Busy);
    }
    *registered = None;
    Ok(())
}

/// Forwards a legacy backend registration to the active provider.
///
/// # Safety
///
/// `ops` must remain valid for the returned terminal's lifetime.
#[allow(clippy::too_many_arguments)]
pub unsafe fn register(
    backend_owner: Option<&Arc<Module>>,
    device: Option<&Arc<Device>>,
    parent: DevNodeId,
    name: *const c_char,
    mode: u16,
    baud: u32,
    ops: *const ConsoleOps,
) -> Result<Arc<Terminal>> {
    if ops.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the backend registration ABI requires a readable operation
    // table. The provider receives this same immutable table.
    //
    // Unlike `NodeOps`, which still accepts legacy 112-byte prefixes so old
    // modules keep loading, the console table is required at its full current
    // size: the console ABI froze before any module shipped against a shorter
    // prefix, so there is nothing to stay compatible with.
    if unsafe { (*ops).size as usize } < size_of::<ConsoleOps>() {
        return Err(Error::InvalidArgument);
    }
    let state = state()?;
    let (provider, provider_lease, backend_lease) = {
        let registered = state.provider.lock();
        let provider = registered.clone().ok_or(Error::NoDevice)?;
        let provider_lease = module::lease_owner(provider.owner.as_ref())?;
        let backend_lease = module::lease_owner(backend_owner)?;
        provider.terminals.fetch_add(1, Ordering::AcqRel);
        (provider, provider_lease, backend_lease)
    };

    let mut provider_receipt = core::ptr::null_mut();
    let callback = provider.ops.register.expect("validated provider callback");
    // SAFETY: table lifetimes are protected by the leases acquired above; all
    // remaining pointers come from validated framework handles or the caller.
    let status = unsafe {
        callback(
            provider.ops.context,
            backend_owner.map_or(core::ptr::null(), obj::handle),
            device.map_or(core::ptr::null(), obj::handle),
            parent.get(),
            name,
            mode,
            baud,
            ops,
            &raw mut provider_receipt,
        )
    };
    if status < 0 || provider_receipt.is_null() {
        provider.terminals.fetch_sub(1, Ordering::Release);
        return Err(if status < 0 {
            Error::from_status(status)
        } else {
            Error::InvalidArgument
        });
    }

    let terminal = Arc::new(Terminal {
        provider,
        receipt: Mutex::new(TerminalReceipt::Live(provider_receipt)),
        backend_owner: backend_owner.cloned(),
        _provider_lease: provider_lease,
        _backend_lease: backend_lease,
    });
    state.terminals.lock().push(terminal.clone());
    Ok(terminal)
}

/// Releases a terminal through its provider before either module lease drops.
pub fn unregister(terminal: &Arc<Terminal>) -> Result<()> {
    let receipt = {
        let mut receipt = terminal.receipt.lock();
        match *receipt {
            TerminalReceipt::Live(value) => {
                *receipt = TerminalReceipt::Removing;
                value
            }
            TerminalReceipt::Removing => return Err(Error::Busy),
            TerminalReceipt::Removed => return Ok(()),
        }
    };
    let callback = terminal
        .provider
        .ops
        .unregister
        .expect("validated provider callback");
    // SAFETY: the terminal keeps both provider and backend callback tables
    // resident until this callback has completed.
    let status = unsafe { callback(terminal.provider.ops.context, receipt) };
    if status < 0 {
        *terminal.receipt.lock() = TerminalReceipt::Live(receipt);
        return Err(Error::from_status(status));
    }
    *terminal.receipt.lock() = TerminalReceipt::Removed;
    terminal.provider.terminals.fetch_sub(1, Ordering::Release);
    if let Ok(state) = state() {
        state
            .terminals
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, terminal));
    }
    Ok(())
}

/// Releases terminals owned by an unloading backend module.
///
/// This is a last-resort lifecycle hook. Normal backend removal should invoke
/// `tty_unregister` itself; the broker path guarantees workers stop before its
/// callback tables and module leases are released.
pub(in super::super) fn remove_module_terminals(module: &Arc<Module>) {
    let Ok(state) = state() else {
        return;
    };
    let terminals: Vec<Arc<Terminal>> = state
        .terminals
        .lock()
        .iter()
        .filter(|terminal| {
            terminal
                .backend_owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for terminal in terminals {
        let _ = unregister(&terminal);
    }
}
