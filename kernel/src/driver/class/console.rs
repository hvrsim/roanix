//! Terminal registration.
//!
//! A driver that owns a byte-oriented console - a UART, a pseudo-terminal
//! endpoint, or later a display console - supplies only the operations in
//! [`ConsoleOps`]. Everything a terminal owes userspace, from canonical-mode
//! editing to job control, comes from the shared line discipline in
//! [`super::tty`].
//!
//! Registering a terminal joins the `tty` class, so a driver or a future
//! subsystem can enumerate every terminal in the system without knowing which
//! hardware backs it.

use alloc::{format, string::String, sync::Arc, vec::Vec};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{
    fs::{Error as FsError, Result as FsResult, devtempfs::DevNodeId},
    sys::{event::Event, sync::Mutex},
};

use super::{
    super::{
        core::{
            class::{self, Class, ClassDevice, Membership},
            device::Device,
            module::{self, Module},
        },
        error::{self, Error, Result},
    },
    chardev,
    tty::{ConsoleBackend, SerialSettings, Tty},
};

/// Name of the class every terminal joins.
pub const CLASS_NAME: &str = "tty";

/// Console behaviour flags.
pub mod flags {
    /// Reset terminal settings when the last file description closes.
    pub const RESET_ON_LAST_CLOSE: u64 = 1 << 0;
}

/// Byte-level operations a console backend implements.
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

/// Serial framing parameters handed to a backend.
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
    /// Whether parity is odd rather than even.
    pub odd_parity: u8,
}

struct Backend {
    ops: ConsoleOps,
    owner: Option<Arc<Module>>,
}

// SAFETY: the operation table and context belong to the owning module, which
// cannot unload until the terminal is removed.
unsafe impl Send for Backend {}
// SAFETY: the ABI requires console operations to tolerate concurrent use by the
// line discipline's reader and writer paths.
unsafe impl Sync for Backend {}

impl Backend {
    fn pin(&self) -> FsResult<module::ModuleGuard> {
        module::pin_owner(self.owner.as_ref(), false).map_err(|_| FsError::Io)
    }

    fn event(pointer: usize) -> Option<&'static Event> {
        super::super::abi::events::resolve(pointer)
    }
}

impl ConsoleBackend for Backend {
    fn open(&self) -> FsResult<()> {
        let Some(callback) = self.ops.open else {
            return Ok(());
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback.
        error::from_status(unsafe { callback(self.ops.context) }).map_err(FsError::from)
    }

    fn close(&self) {
        let Some(callback) = self.ops.close else {
            return;
        };
        let Ok(_pin) = module::pin_owner(self.owner.as_ref(), true) else {
            return;
        };
        // SAFETY: registration validated the callback.
        unsafe { callback(self.ops.context) };
    }

    fn try_read(&self) -> Option<u8> {
        let callback = self.ops.try_read?;
        let _pin = self.pin().ok()?;
        let mut byte = 0u8;
        // SAFETY: registration validated the callback and the output pointer
        // addresses local storage that outlives the call.
        let status = unsafe { callback(self.ops.context, &raw mut byte) };
        (status > 0).then_some(byte)
    }

    fn read(&self, output: &mut [u8]) -> FsResult<usize> {
        let Some(callback) = self.ops.read else {
            return self.read_fallback(output);
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback, and the buffer describes
        // memory owned by the caller for this call.
        let value = unsafe { callback(self.ops.context, output.as_mut_ptr(), output.len()) };
        if value < 0 {
            return Err(Error::from_status(value as i32).into());
        }
        let read = value as usize;
        if read > output.len() {
            return Err(FsError::Io);
        }
        Ok(read)
    }

    fn write(&self, bytes: &[u8], nonblocking: bool) -> FsResult<()> {
        let callback = self.ops.write.ok_or(FsError::Unsupported)?;
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback, and the buffer describes
        // memory owned by the caller for this call.
        let status = unsafe {
            callback(
                self.ops.context,
                bytes.as_ptr(),
                bytes.len(),
                u8::from(nonblocking),
            )
        };
        error::from_status(status).map_err(FsError::from)
    }

    fn configure(&self, settings: SerialSettings) -> FsResult<()> {
        let Some(callback) = self.ops.configure else {
            return Ok(());
        };
        let _pin = self.pin()?;
        let framing = SerialFraming {
            baud: settings.baud,
            data_bits: settings.data_bits,
            stop_bits: settings.stop_bits,
            parity: u8::from(settings.parity),
            odd_parity: u8::from(settings.odd_parity),
        };
        // SAFETY: registration validated the callback and `framing` outlives
        // the call.
        error::from_status(unsafe { callback(self.ops.context, &raw const framing) })
            .map_err(FsError::from)
    }

    fn flush(&self) -> FsResult<()> {
        self.simple(self.ops.flush)
    }

    fn flush_input(&self) -> FsResult<()> {
        match self.ops.flush_input {
            Some(callback) => self.simple(Some(callback)),
            None => {
                let mut bytes = [0u8; 512];
                while self.read(&mut bytes)? != 0 {}
                Ok(())
            }
        }
    }

    fn flush_output(&self) -> FsResult<()> {
        self.simple(self.ops.flush_output)
    }

    fn send_break(&self, duration: u64) -> FsResult<()> {
        let Some(callback) = self.ops.send_break else {
            return Ok(());
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback.
        error::from_status(unsafe { callback(self.ops.context, duration) })
            .map_err(FsError::from)
    }

    fn hung_up(&self) -> bool {
        self.predicate(self.ops.hung_up, false)
    }

    fn writable(&self) -> bool {
        self.predicate(self.ops.writable, true)
    }

    fn queued_output(&self) -> usize {
        let Some(callback) = self.ops.queued_output else {
            return 0;
        };
        let Ok(_pin) = self.pin() else {
            return 0;
        };
        // SAFETY: registration validated the callback.
        let value = unsafe { callback(self.ops.context) };
        if value < 0 { 0 } else { value as usize }
    }

    fn readable_event(&self) -> Option<&Event> {
        Self::event(self.ops.readable_event)
    }

    fn writable_event(&self) -> Option<&Event> {
        Self::event(self.ops.writable_event)
    }

    fn hangup_event(&self) -> Option<&Event> {
        Self::event(self.ops.hangup_event)
    }

    fn reset_on_last_close(&self) -> bool {
        self.ops.flags & flags::RESET_ON_LAST_CLOSE != 0
    }
}

impl Backend {
    fn simple(&self, callback: Option<unsafe extern "C" fn(*mut c_void) -> i32>) -> FsResult<()> {
        let Some(callback) = callback else {
            return Ok(());
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback.
        error::from_status(unsafe { callback(self.ops.context) }).map_err(FsError::from)
    }

    fn predicate(
        &self,
        callback: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        default: bool,
    ) -> bool {
        let Some(callback) = callback else {
            return default;
        };
        let Ok(_pin) = self.pin() else {
            return default;
        };
        // SAFETY: registration validated the callback.
        unsafe { callback(self.ops.context) > 0 }
    }
}

impl Drop for Backend {
    fn drop(&mut self) {
        let Some(callback) = self.ops.destroy else {
            return;
        };
        let Ok(_pin) = module::pin_owner(self.owner.as_ref(), true) else {
            return;
        };
        // SAFETY: registration validated the callback and no further console
        // operation can start once the backend is being dropped.
        unsafe { callback(self.ops.context) };
    }
}

/// A registered terminal.
pub struct Terminal {
    tty: Arc<Tty>,
    node: DevNodeId,
    member: Mutex<Option<Arc<ClassDevice>>>,
    owner: Option<Arc<Module>>,
}

impl Terminal {
    /// Returns the line discipline instance.
    pub fn tty(&self) -> &Arc<Tty> {
        &self.tty
    }

    /// Returns the device-filesystem node identifier.
    pub const fn node(&self) -> DevNodeId {
        self.node
    }
}

struct State {
    class: Arc<Class>,
    terminals: Mutex<Vec<Arc<Terminal>>>,
    next_index: AtomicU64,
}

static STATE: crate::sys::sync::Once<State> = crate::sys::sync::Once::new();

pub(super) fn init() {
    // SAFETY: the class is kernel-owned and installs no callbacks.
    let class = unsafe { class::register(None, CLASS_NAME, class::ClassOps::default()) }
        .expect("driver/tty: failed to register the terminal class");
    STATE.call_once(|| State {
        class,
        terminals: Mutex::new(Vec::new()),
        next_index: AtomicU64::new(0),
    });
}

fn state() -> Result<&'static State> {
    STATE.get().ok_or(Error::NotInitialized)
}

/// Registers a terminal backed by a driver's console operations.
///
/// # Safety
///
/// Every callback in `ops` must follow the console ABI and stay executable
/// until the terminal is removed.
pub unsafe fn register(
    owner: Option<&Arc<Module>>,
    device: Option<&Arc<Device>>,
    parent: DevNodeId,
    name: &str,
    mode: u16,
    baud: u32,
    ops: ConsoleOps,
) -> Result<Arc<Terminal>> {
    if (ops.size as usize) < size_of::<ConsoleOps>() {
        return Err(Error::InvalidArgument);
    }
    if name.is_empty() {
        return Err(Error::InvalidArgument);
    }
    let state = state()?;

    let path = terminal_path(parent, name)?;
    let backend = Arc::new(Backend {
        ops,
        owner: owner.cloned(),
    });
    let tty = Tty::new(backend, path.into_boxed_str(), baud)?;
    let node = chardev::create_native_node(
        owner,
        parent,
        name,
        chardev::kind::CHARACTER,
        mode,
        tty.clone(),
    )?;

    let terminal = Arc::new(Terminal {
        tty,
        node,
        member: Mutex::new(None),
        owner: owner.cloned(),
    });

    let membership = Membership {
        name: String::from(name).into_boxed_str(),
        ops: Arc::as_ptr(&terminal).cast(),
        ops_size: size_of::<Terminal>(),
        context: core::ptr::null_mut(),
    };
    // SAFETY: the membership table is the terminal itself, which outlives the
    // membership because the terminal list holds a reference.
    match unsafe { class::add_device(owner, &state.class, device, membership) } {
        Ok(member) => *terminal.member.lock() = Some(member),
        Err(error) => {
            let _ = chardev::remove(owner, node);
            return Err(error);
        }
    }
    state.next_index.fetch_add(1, Ordering::Relaxed);
    state.terminals.lock().push(terminal.clone());
    Ok(terminal)
}

fn terminal_path(parent: DevNodeId, name: &str) -> Result<String> {
    if parent == chardev::root()? {
        return Ok(format!("/dev/{name}"));
    }
    // Nested terminal directories are rare; the only current case is the
    // pseudo-terminal slave directory.
    Ok(format!("/dev/pts/{name}"))
}

/// Removes a registered terminal.
pub fn unregister(terminal: &Arc<Terminal>) -> Result<()> {
    let state = state()?;
    terminal.tty.shutdown();
    if let Some(member) = terminal.member.lock().take() {
        class::remove_device(&member);
    }
    chardev::remove(terminal.owner.as_ref(), terminal.node)?;
    state
        .terminals
        .lock()
        .retain(|entry| !Arc::ptr_eq(entry, terminal));
    Ok(())
}

/// Returns every registered terminal.
pub fn list() -> Vec<Arc<Terminal>> {
    STATE
        .get()
        .map(|state| state.terminals.lock().clone())
        .unwrap_or_default()
}

pub(in super::super) fn remove_module_terminals(module: &Arc<Module>) {
    let Ok(state) = state() else {
        return;
    };
    let owned: Vec<Arc<Terminal>> = state
        .terminals
        .lock()
        .iter()
        .filter(|terminal| {
            terminal
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for terminal in owned {
        let _ = unregister(&terminal);
    }
}

/// Returns the class every terminal joins.
pub fn class() -> Result<Arc<Class>> {
    Ok(state()?.class.clone())
}
