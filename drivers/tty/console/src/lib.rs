#![no_std]
#![allow(unsafe_code)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::explicit_auto_deref,
    clippy::ignored_unit_patterns,
    clippy::must_use_candidate,
    clippy::obfuscated_if_else,
    clippy::redundant_closure_for_method_calls,
    clippy::struct_excessive_bools,
    clippy::too_many_lines,
    clippy::trivially_copy_pass_by_ref,
    clippy::unreadable_literal,
    clippy::unused_self
)]

//! Shared terminal line discipline loadable module.

extern crate alloc;

use alloc::{boxed::Box, collections::VecDeque, format};
use core::{
    ffi::{CStr, c_char, c_void},
    ptr,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    time::Duration,
};

use ddk::{
    self, Class, ClassDevice, Devfs, Device, Event, Module, Result as DdkResult, TicketLock,
    TtyProvider, Worker, raw,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FsError {
    Unsupported,
    WouldBlock,
    Interrupted,
    Io,
    Busy,
    InvalidArgument,
    PermissionDenied,
    NotTty,
}

type FsResult<T> = core::result::Result<T, FsError>;

impl FsError {
    const fn status(self) -> i32 {
        match self {
            Self::Unsupported => ddk::ENOTSUP,
            Self::WouldBlock => ddk::EAGAIN,
            Self::Interrupted => ddk::EINTR,
            Self::Io => ddk::EIO,
            Self::Busy => ddk::EBUSY,
            Self::InvalidArgument => ddk::EINVAL,
            Self::PermissionDenied => ddk::EPERM,
            Self::NotTty => ddk::ENOTTY,
        }
    }

    const fn from_status(status: i32) -> Self {
        match status {
            ddk::EAGAIN => Self::WouldBlock,
            ddk::EINTR => Self::Interrupted,
            ddk::EBUSY => Self::Busy,
            ddk::EINVAL => Self::InvalidArgument,
            ddk::EPERM => Self::PermissionDenied,
            ddk::ENOTTY => Self::NotTty,
            ddk::ENOTSUP => Self::Unsupported,
            _ => Self::Io,
        }
    }
}

impl From<ddk::Error> for FsError {
    fn from(error: ddk::Error) -> Self {
        Self::from_status(error.status())
    }
}

const SIGHUP: u8 = 1;
const SIGINT: u8 = 2;
const SIGQUIT: u8 = 3;
const SIGCONT: u8 = 18;
const SIGTSTP: u8 = 20;
const SIGWINCH: u8 = 28;

fn signal_group(group: i32, signal: u8) {
    if group > 0 {
        let _ = ddk::signal_process_group(group, signal);
    }
}

mod clock {
    use core::time::Duration;

    use ddk::Event;

    pub fn sleep(duration: Duration) {
        ddk::sleep_ns(duration.as_nanos().min(u128::from(u64::MAX)) as u64);
    }

    pub fn monotonic_ns() -> u64 {
        ddk::monotonic_ns()
    }

    pub fn wait_timeout(event: &Event, duration: Duration) -> bool {
        event
            .wait_timeout(duration.as_nanos().min(u128::from(u64::MAX)) as u64)
            .unwrap_or(false)
    }
}

const INPUT_CAPACITY: usize = 4096;
const ECHO_CAPACITY: usize = 4096;
const INPUT_BATCH_SIZE: usize = 512;
const OUTPUT_BATCH_SIZE: usize = 512;
const FALLBACK_POLL_INTERVAL: Duration = Duration::from_micros(100);
const NCCS: usize = 32;

const VINTR: usize = 0;
const VQUIT: usize = 1;
const VERASE: usize = 2;
const VKILL: usize = 3;
const VEOF: usize = 4;
const VTIME: usize = 5;
const VMIN: usize = 6;
const VSTART: usize = 8;
const VSTOP: usize = 9;
const VSUSP: usize = 10;
const VEOL: usize = 11;
const VREPRINT: usize = 12;
const VDISCARD: usize = 13;
const VWERASE: usize = 14;
const VLNEXT: usize = 15;
const VEOL2: usize = 16;

const ISTRIP: u32 = 0o000040;
const INLCR: u32 = 0o000100;
const IGNCR: u32 = 0o000200;
const ICRNL: u32 = 0o000400;
const IXON: u32 = 0o002000;
const IXANY: u32 = 0o004000;
const IMAXBEL: u32 = 0o020000;
const IUTF8: u32 = 0o040000;

const OPOST: u32 = 0o000001;
const OLCUC: u32 = 0o000002;
const ONLCR: u32 = 0o000004;
const OCRNL: u32 = 0o000010;
const ONOCR: u32 = 0o000020;
const ONLRET: u32 = 0o000040;
const TABDLY: u32 = 0o014000;
const TAB3: u32 = 0o014000;

const CSIZE: u32 = 0o000060;
const CS5: u32 = 0o000000;
const CS6: u32 = 0o000020;
const CS7: u32 = 0o000040;
const CS8: u32 = 0o000060;
const CSTOPB: u32 = 0o000100;
const CREAD: u32 = 0o000200;
const PARENB: u32 = 0o000400;
const PARODD: u32 = 0o001000;
const CLOCAL: u32 = 0o004000;
const CBAUD: u32 = 0o010017;
const B9600: u32 = 13;

const ISIG: u32 = 0o000001;
const ICANON: u32 = 0o000002;
const ECHO: u32 = 0o000010;
const ECHOE: u32 = 0o000020;
const ECHOK: u32 = 0o000040;
const ECHONL: u32 = 0o000100;
const NOFLSH: u32 = 0o000200;
const TOSTOP: u32 = 0o000400;
const ECHOCTL: u32 = 0o001000;
const ECHOKE: u32 = 0o004000;
const FLUSHO: u32 = 0o010000;
const IEXTEN: u32 = 0o100000;

/// Linux-compatible terminal settings request.
pub const TCGETS: u64 = 0x5401;
/// Apply terminal settings immediately.
pub const TCSETS: u64 = 0x5402;
/// Apply terminal settings after pending output drains.
pub const TCSETSW: u64 = 0x5403;
/// Drain output, flush input, and apply terminal settings.
pub const TCSETSF: u64 = 0x5404;
/// Send a break or drain output.
pub const TCSBRK: u64 = 0x5409;
/// Suspend/resume terminal flow.
pub const TCXONC: u64 = 0x540A;
/// Flush terminal queues.
pub const TCFLSH: u64 = 0x540B;
/// Enable exclusive opens.
pub const TIOCEXCL: u64 = 0x540C;
/// Disable exclusive opens.
pub const TIOCNXCL: u64 = 0x540D;
/// Acquire a controlling terminal.
pub const TIOCSCTTY: u64 = 0x540E;
/// Return the foreground process group.
pub const TIOCGPGRP: u64 = 0x540F;
/// Set the foreground process group.
pub const TIOCSPGRP: u64 = 0x5410;
/// Return pending output bytes.
pub const TIOCOUTQ: u64 = 0x5411;
/// Inject one terminal input byte.
pub const TIOCSTI: u64 = 0x5412;
/// Return terminal window size.
pub const TIOCGWINSZ: u64 = 0x5413;
/// Set terminal window size.
pub const TIOCSWINSZ: u64 = 0x5414;
/// Return software-carrier state.
pub const TIOCGSOFTCAR: u64 = 0x5419;
/// Set software-carrier state.
pub const TIOCSSOFTCAR: u64 = 0x541A;
/// Return queued input bytes.
pub const FIONREAD: u64 = 0x541B;
/// Release a controlling terminal.
pub const TIOCNOTTY: u64 = 0x5422;
/// Return the controlling session.
pub const TIOCGSID: u64 = 0x5429;
/// Return the PTY slave number.
pub const TIOCGPTN: u64 = 0x8004_5430;
/// Lock or unlock a PTY slave.
pub const TIOCSPTLCK: u64 = 0x4004_5431;
/// Returns the canonical Roanix devtempfs path for this TTY.
pub const TIOCGPATH: u64 = 0x5254_0001;

/// Size of the Linux ABI `termios` structure used by mlibc.
pub const TERMIOS_SIZE: usize = 60;
/// Size of `struct winsize`.
pub const WINSIZE_SIZE: usize = 8;
/// Fixed buffer size used by [`TIOCGPATH`].
pub const TTY_PATH_SIZE: usize = 32;

/// Direction and size of an ioctl argument buffer.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct IoctlSpec {
    /// Bytes copied from userspace before dispatch.
    pub input: bool,
    /// Bytes copied back to userspace after dispatch.
    pub output: bool,
    /// Argument-buffer length.
    pub size: usize,
}

/// UART framing derived from terminal settings.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SerialSettings {
    /// Requested baud rate.
    pub baud: u32,
    /// Data bits, from five through eight.
    pub data_bits: u8,
    /// Stop bits, one or two.
    pub stop_bits: u8,
    /// Whether parity generation/checking is enabled.
    pub parity: bool,
    /// Whether enabled parity is odd.
    pub odd_parity: bool,
}

/// Direct adapter over a backend's stable C operation table.
struct Backend {
    ops: raw::ConsoleOps,
    destroyed: AtomicBool,
}

// SAFETY: the broker holds a module lease for this terminal, and backend
// callback synchronization is defined by the backend ABI.
unsafe impl Send for Backend {}
// SAFETY: as above.
unsafe impl Sync for Backend {}

impl Backend {
    unsafe fn copy(ops: *const raw::ConsoleOps) -> FsResult<Self> {
        if ops.is_null() {
            return Err(FsError::InvalidArgument);
        }
        // SAFETY: the provider callback receives a readable backend table that
        // remains resident under the broker's backend lease.
        let ops = unsafe { ptr::read(ops) };
        if (ops.size as usize) < core::mem::size_of::<raw::ConsoleOps>() {
            return Err(FsError::InvalidArgument);
        }
        Ok(Self {
            ops,
            destroyed: AtomicBool::new(false),
        })
    }

    fn open(&self) -> FsResult<()> {
        self.ops.open.map_or(Ok(()), |callback| {
            // SAFETY: the broker keeps this immutable callback table resident.
            let status = unsafe { callback(self.ops.context) };
            (status >= 0)
                .then_some(())
                .ok_or(FsError::from_status(status))
        })
    }

    fn close(&self) {
        if let Some(callback) = self.ops.close {
            // SAFETY: the broker keeps this immutable callback table resident.
            unsafe { callback(self.ops.context) };
        }
    }

    fn try_read(&self) -> Option<u8> {
        let callback = self.ops.try_read?;
        let mut byte = 0;
        // SAFETY: `byte` is writable output storage for this callback.
        (unsafe { callback(self.ops.context, &raw mut byte) } > 0).then_some(byte)
    }

    fn read(&self, output: &mut [u8]) -> FsResult<usize> {
        if let Some(callback) = self.ops.read {
            // SAFETY: `output` is writable for exactly its length.
            let value = unsafe { callback(self.ops.context, output.as_mut_ptr(), output.len()) };
            if value < 0 {
                return Err(FsError::from_status(value as i32));
            }
            let count = value as usize;
            return (count <= output.len()).then_some(count).ok_or(FsError::Io);
        }
        let mut count = 0;
        while count < output.len() {
            let Some(byte) = self.try_read() else { break };
            output[count] = byte;
            count += 1;
        }
        Ok(count)
    }

    fn write(&self, bytes: &[u8], nonblocking: bool) -> FsResult<()> {
        let callback = self.ops.write.ok_or(FsError::Unsupported)?;
        // SAFETY: `bytes` is readable for exactly its length.
        let status = unsafe {
            callback(
                self.ops.context,
                bytes.as_ptr(),
                bytes.len(),
                u8::from(nonblocking),
            )
        };
        (status >= 0)
            .then_some(())
            .ok_or(FsError::from_status(status))
    }

    fn configure(&self, settings: SerialSettings) -> FsResult<()> {
        let Some(callback) = self.ops.configure else {
            return Ok(());
        };
        let framing = raw::SerialFraming {
            baud: settings.baud,
            data_bits: settings.data_bits,
            stop_bits: settings.stop_bits,
            parity: u8::from(settings.parity),
            odd_parity: u8::from(settings.odd_parity),
        };
        // SAFETY: `framing` remains valid through the immediate callback.
        let status = unsafe { callback(self.ops.context, &raw const framing) };
        (status >= 0)
            .then_some(())
            .ok_or(FsError::from_status(status))
    }

    fn simple(&self, callback: Option<unsafe extern "C" fn(*mut c_void) -> i32>) -> FsResult<()> {
        let Some(callback) = callback else {
            return Ok(());
        };
        // SAFETY: the broker keeps this immutable callback table resident.
        let status = unsafe { callback(self.ops.context) };
        (status >= 0)
            .then_some(())
            .ok_or(FsError::from_status(status))
    }

    fn flush(&self) -> FsResult<()> {
        self.simple(self.ops.flush)
    }
    fn flush_output(&self) -> FsResult<()> {
        self.simple(self.ops.flush_output)
    }

    fn flush_input(&self) -> FsResult<()> {
        if self.ops.flush_input.is_some() {
            return self.simple(self.ops.flush_input);
        }
        let mut bytes = [0; INPUT_BATCH_SIZE];
        while self.read(&mut bytes)? != 0 {}
        Ok(())
    }

    fn send_break(&self, duration: u64) -> FsResult<()> {
        let Some(callback) = self.ops.send_break else {
            return Ok(());
        };
        // SAFETY: the broker keeps this immutable callback table resident.
        let status = unsafe { callback(self.ops.context, duration) };
        (status >= 0)
            .then_some(())
            .ok_or(FsError::from_status(status))
    }

    fn predicate(
        &self,
        callback: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
        default: bool,
    ) -> bool {
        callback.map_or(default, |callback| {
            // SAFETY: the broker keeps this immutable callback table resident.
            unsafe { callback(self.ops.context) > 0 }
        })
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
        // SAFETY: the broker keeps this immutable callback table resident.
        let value = unsafe { callback(self.ops.context) };
        (value.max(0)) as usize
    }

    fn readable_event(&self) -> Option<Event> {
        Event::from_id(self.ops.readable_event)
    }
    fn writable_event(&self) -> Option<Event> {
        Event::from_id(self.ops.writable_event)
    }
    fn hangup_event(&self) -> Option<Event> {
        Event::from_id(self.ops.hangup_event)
    }
    fn reset_on_last_close(&self) -> bool {
        self.ops.flags & ddk::CONSOLE_RESET_ON_LAST_CLOSE != 0
    }

    fn destroy(&self) {
        if self.destroyed.swap(true, Ordering::AcqRel) {
            return;
        }
        if let Some(callback) = self.ops.destroy {
            // SAFETY: the broker holds the backend module lease until terminal
            // teardown completes, after all terminal workers have stopped.
            unsafe { callback(self.ops.context) };
        }
    }
}

impl Drop for Backend {
    fn drop(&mut self) {
        self.destroy();
    }
}

/// Linux-compatible terminal settings.
#[derive(Copy, Clone)]
struct Termios {
    input_flags: u32,
    output_flags: u32,
    control_flags: u32,
    local_flags: u32,
    line: u8,
    control: [u8; NCCS],
    input_baud: u32,
    output_baud: u32,
}

#[derive(Copy, Clone, Default, Eq, PartialEq)]
struct WinSize {
    rows: u16,
    columns: u16,
    x_pixels: u16,
    y_pixels: u16,
}

#[derive(Copy, Clone)]
enum InputItem {
    Byte(u8),
    Eof,
}

struct TtyState {
    termios: Termios,
    winsize: WinSize,
    input: VecDeque<InputItem>,
    echo: VecDeque<u8>,
    canonical_ready: usize,
    input_bytes: usize,
    literal_next: bool,
    output_stopped: bool,
    interrupted: bool,
    exclusive: bool,
    soft_carrier: bool,
    foreground_group: i32,
    session: i32,
    hung_up: bool,
}

struct InputWorker {
    control: Box<InputControl>,
    worker: Worker,
}

struct InputControl {
    tty: *const Tty,
    stop: Event,
}

unsafe extern "C" fn input_worker_entry(context: *mut c_void) {
    if context.is_null() {
        return;
    }
    // SAFETY: `context` points to an `InputControl` owned by `InputWorker`
    // until `Worker::join` has observed this callback return.
    let control = unsafe { &*context.cast::<InputControl>() };
    // SAFETY: the terminal allocation outlives the joined worker.
    let tty = unsafe { &*control.tty };
    tty.input_worker_loop(control.stop);
}

/// Character terminal layered over a platform console backend.
pub struct Tty {
    backend: Backend,
    path: Box<str>,
    state: TicketLock<TtyState>,
    lifecycle_lock: TicketLock<()>,
    output_lock: TicketLock<usize>,
    input_flushing: AtomicBool,
    input_generation: AtomicU64,
    input_ready: Event,
    output_resumed: Event,
    input_worker: TicketLock<Option<InputWorker>>,
    node: TicketLock<Option<u64>>,
    member: TicketLock<Option<ClassDevice>>,
    opens: AtomicU64,
    initial_baud: u32,
}

// SAFETY: all mutable terminal state is protected by ticket locks; backend
// callback synchronization is part of the backend ABI.
unsafe impl Send for Tty {}
// SAFETY: as above.
unsafe impl Sync for Tty {}

impl Tty {
    /// Creates a terminal with cooked defaults at the hardware's initial baud.
    fn new(backend: Backend, path: Box<str>, baud: u32) -> DdkResult<Box<Self>> {
        let termios = Termios::with_baud(baud).ok_or(ddk::Error::from_status(ddk::EINVAL))?;
        let input_ready = Event::create()?;
        let output_resumed = match Event::create() {
            Ok(event) => event,
            Err(error) => {
                let _ = input_ready.destroy();
                return Err(error);
            }
        };
        Ok(Box::new(Self {
            backend,
            path,
            state: TicketLock::new(TtyState {
                termios,
                winsize: WinSize::default(),
                input: VecDeque::with_capacity(INPUT_CAPACITY),
                echo: VecDeque::with_capacity(ECHO_CAPACITY),
                canonical_ready: 0,
                input_bytes: 0,
                literal_next: false,
                output_stopped: false,
                interrupted: false,
                exclusive: false,
                soft_carrier: true,
                foreground_group: 0,
                session: 0,
                hung_up: false,
            }),
            lifecycle_lock: TicketLock::new(()),
            output_lock: TicketLock::new(0),
            input_flushing: AtomicBool::new(false),
            input_generation: AtomicU64::new(0),
            input_ready,
            output_resumed,
            input_worker: TicketLock::new(None),
            node: TicketLock::new(None),
            member: TicketLock::new(None),
            opens: AtomicU64::new(0),
            initial_baud: baud,
        }))
    }

    fn start_input_worker(&self) {
        if self.backend.readable_event().is_none() && self.backend.hangup_event().is_none() {
            return;
        }
        let mut worker = self.input_worker.lock();
        if worker.is_some() {
            return;
        }
        let control = Box::new(InputControl {
            tty: self,
            stop: match Event::create() {
                Ok(event) => event,
                Err(_) => return,
            },
        });
        let context = (&raw const *control).cast_mut().cast();
        // SAFETY: `control` remains owned by `input_worker` until its worker
        // has stopped and joined.
        let Ok(worker_receipt) = (unsafe { Worker::spawn(Some(input_worker_entry), context) })
        else {
            let _ = control.stop.destroy();
            return;
        };
        *worker = Some(InputWorker {
            control,
            worker: worker_receipt,
        });
    }

    /// Marks the terminal disconnected and stops its input worker.
    ///
    /// Called when the owning driver removes the terminal so that readers wake
    /// up and the worker thread stops executing driver code.
    pub fn shutdown(&self) {
        self.state.lock().hung_up = true;
        let _ = self.input_ready.signal();
        let _ = self.output_resumed.signal();
        self.stop_input_worker();
    }

    fn stop_input_worker(&self) {
        let Some(mut worker) = self.input_worker.lock().take() else {
            return;
        };
        let _ = worker.control.stop.signal();
        let _ = worker.worker.join();
        let _ = worker.control.stop.destroy();
    }

    fn input_worker_loop(&self, stop: Event) {
        loop {
            self.pump_input(true);
            if self.observe_hangup() {
                break;
            }

            let readable = self.backend.readable_event();
            let hangup = self
                .backend
                .hangup_event()
                .filter(|hangup| readable.is_none_or(|readable| readable.id() != hangup.id()));
            let writable = if self.state.lock().echo.is_empty() {
                None
            } else {
                self.backend.writable_event()
            };
            let stopped = match (readable, hangup, writable) {
                (Some(readable), Some(hangup), Some(writable)) => {
                    Event::wait_any(&[stop, readable, hangup, writable])
                        .map_or(true, |index| index == 0)
                }
                (Some(readable), Some(hangup), None) => {
                    Event::wait_any(&[stop, readable, hangup]).map_or(true, |index| index == 0)
                }
                (Some(readable), None, Some(writable)) => {
                    Event::wait_any(&[stop, readable, writable]).map_or(true, |index| index == 0)
                }
                (None, Some(hangup), Some(writable)) => {
                    Event::wait_any(&[stop, hangup, writable]).map_or(true, |index| index == 0)
                }
                (Some(readable), None, None) => {
                    Event::wait_any(&[stop, readable]).map_or(true, |index| index == 0)
                }
                (None, Some(hangup), None) => {
                    Event::wait_any(&[stop, hangup]).map_or(true, |index| index == 0)
                }
                (None, None, Some(writable)) => {
                    Event::wait_any(&[stop, writable]).map_or(true, |index| index == 0)
                }
                (None, None, None) => true,
            };
            if stopped {
                break;
            }
        }
    }

    fn observe_hangup(&self) -> bool {
        if !self.backend.hung_up() {
            return false;
        }
        let foreground_group = {
            let mut state = self.state.lock();
            if state.hung_up {
                return true;
            }
            state.hung_up = true;
            state.output_stopped = false;
            state.foreground_group
        };
        let _ = self.input_ready.signal();
        let _ = self.output_resumed.signal();
        if foreground_group > 0 {
            signal_group(foreground_group, SIGHUP);
            signal_group(foreground_group, SIGCONT);
        }
        true
    }

    fn pump_input(&self, nonblocking_echo: bool) -> usize {
        if self.input_flushing.load(Ordering::Acquire) {
            return 0;
        }
        let mut input = [0u8; INPUT_BATCH_SIZE];
        let mut count = 0usize;
        while count < INPUT_CAPACITY {
            let generation = self.input_generation.load(Ordering::Acquire);
            let Ok(read) = self.backend.read(&mut input) else {
                break;
            };
            if read == 0 {
                break;
            }
            if self.input_flushing.load(Ordering::Acquire)
                || self.input_generation.load(Ordering::Acquire) != generation
            {
                continue;
            }
            self.process_input_generation(&input[..read], generation, nonblocking_echo);
            count += read;
            if read < input.len() {
                break;
            }
        }
        count
    }

    fn process_input(&self, byte: u8) {
        let generation = self.input_generation.load(Ordering::Acquire);
        self.process_input_generation(core::slice::from_ref(&byte), generation, false);
    }

    fn process_input_generation(&self, input: &[u8], generation: u64, nonblocking_echo: bool) {
        let mut state = self.state.lock();
        if self.input_flushing.load(Ordering::Acquire)
            || self.input_generation.load(Ordering::Acquire) != generation
        {
            return;
        }
        let termios = state.termios;
        let raw_input_flags = ISTRIP | INLCR | IGNCR | ICRNL | IXON;
        let raw_local_flags = ISIG | ICANON | ECHO | ECHONL | IEXTEN;
        if termios.control_flags & CREAD != 0
            && termios.input_flags & raw_input_flags == 0
            && termios.local_flags & raw_local_flags == 0
            && !state.literal_next
        {
            let accepted = input.len().min(INPUT_CAPACITY - state.input.len());
            state
                .input
                .extend(input[..accepted].iter().copied().map(InputItem::Byte));
            state.input_bytes += accepted;
            state.canonical_ready = state.input.len();
            drop(state);
            if accepted != 0 {
                let _ = self.input_ready.signal();
            }
            return;
        }
        let mut signals = 0u64;
        let mut flush_output = false;
        for byte in input.iter().copied() {
            let termios = state.termios;
            self.process_input_byte(&mut state, termios, byte, &mut signals, &mut flush_output);
        }
        let foreground_group = state.foreground_group;
        let wake = state.interrupted || input_ready(&state);
        drop(state);

        if flush_output {
            let _output = self.output_lock.lock();
            let _ = self.backend.flush_output();
        }
        self.flush_echo(nonblocking_echo);
        if wake {
            let _ = self.input_ready.signal();
        }
        if foreground_group > 0 {
            for signal in [SIGINT, SIGQUIT, SIGTSTP] {
                if signals & (1u64 << signal) != 0 {
                    signal_group(foreground_group, signal);
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn process_input_byte(
        &self,
        state: &mut TtyState,
        termios: Termios,
        mut byte: u8,
        signals: &mut u64,
        flush_output: &mut bool,
    ) {
        if termios.control_flags & CREAD == 0 {
            return;
        }
        if termios.input_flags & ISTRIP != 0 {
            byte &= 0x7f;
        }
        match byte {
            b'\r' if termios.input_flags & IGNCR != 0 => return,
            b'\r' if termios.input_flags & ICRNL != 0 => byte = b'\n',
            b'\n' if termios.input_flags & INLCR != 0 => byte = b'\r',
            _ => {}
        }

        if termios.input_flags & IXON != 0 {
            if control_matches(&termios, VSTOP, byte) {
                if !state.output_stopped {
                    state.output_stopped = true;
                    let _ = self.output_resumed.reset();
                    return;
                }
                if !control_matches(&termios, VSTART, byte) {
                    return;
                }
            }
            if control_matches(&termios, VSTART, byte)
                || (state.output_stopped && termios.input_flags & IXANY != 0)
            {
                state.output_stopped = false;
                let _ = self.output_resumed.signal();
                if control_matches(&termios, VSTART, byte) {
                    return;
                }
            }
        }

        if state.literal_next {
            state.literal_next = false;
            self.enqueue(state, InputItem::Byte(byte));
            echo_input(state, &termios, byte);
            return;
        }
        if termios.local_flags & IEXTEN != 0 && control_matches(&termios, VLNEXT, byte) {
            state.literal_next = true;
            if termios.local_flags & ECHO != 0 {
                if termios.local_flags & ECHOE != 0 {
                    queue_echo(state, b"^\x08");
                } else {
                    echo_input(state, &termios, byte);
                }
            }
            return;
        }
        if termios.local_flags & IEXTEN != 0 && control_matches(&termios, VDISCARD, byte) {
            if state.termios.local_flags & FLUSHO == 0 {
                state.echo.clear();
                if termios.local_flags & ECHO != 0 {
                    echo_input(state, &termios, byte);
                }
                state.termios.local_flags |= FLUSHO;
                *flush_output = true;
            } else {
                state.termios.local_flags &= !FLUSHO;
                if termios.local_flags & ECHO != 0 {
                    echo_input(state, &termios, byte);
                }
            }
            return;
        }

        let signal = if termios.local_flags & ISIG == 0 {
            None
        } else if control_matches(&termios, VINTR, byte) {
            Some(SIGINT)
        } else if control_matches(&termios, VQUIT, byte) {
            Some(SIGQUIT)
        } else if control_matches(&termios, VSUSP, byte) {
            Some(SIGTSTP)
        } else {
            None
        };
        if let Some(signal) = signal {
            *signals |= 1u64 << signal;
            if termios.local_flags & NOFLSH == 0 {
                clear_input(state);
                state.echo.clear();
                *flush_output = true;
            }
            state.output_stopped = false;
            let _ = self.output_resumed.signal();
            state.interrupted = true;
            if termios.local_flags & ECHO != 0 {
                echo_input(state, &termios, byte);
            }
            return;
        }

        if termios.local_flags & ICANON != 0 {
            if control_matches(&termios, VERASE, byte) {
                let (erased, columns) = erase_last_character(&mut state.input, &termios);
                if erased != 0 {
                    state.input_bytes = state.input_bytes.saturating_sub(erased);
                    if termios.local_flags & ECHOE != 0 {
                        for _ in 0..columns {
                            queue_echo(state, b"\x08 \x08");
                        }
                    } else if termios.local_flags & ECHO != 0 {
                        echo_input(state, &termios, byte);
                    }
                }
                return;
            }
            if control_matches(&termios, VKILL, byte) {
                let erased = discard_current_line(&mut state.input, &termios);
                state.input_bytes = state.input_bytes.saturating_sub(erased);
                if termios.local_flags & (ECHO | ECHOE | ECHOKE) == (ECHO | ECHOE | ECHOKE) {
                    for _ in 0..erased {
                        queue_echo(state, b"\x08 \x08");
                    }
                } else if termios.local_flags & (ECHO | ECHOK) == (ECHO | ECHOK) {
                    echo_input(state, &termios, b'\n');
                }
                return;
            }
            if termios.local_flags & IEXTEN != 0 && control_matches(&termios, VWERASE, byte) {
                let erased = erase_word(&mut state.input, &termios);
                state.input_bytes = state.input_bytes.saturating_sub(erased);
                if termios.local_flags & ECHOE != 0 {
                    for _ in 0..erased {
                        queue_echo(state, b"\x08 \x08");
                    }
                }
                return;
            }
            if termios.local_flags & (IEXTEN | ECHO) == (IEXTEN | ECHO)
                && control_matches(&termios, VREPRINT, byte)
            {
                echo_input(state, &termios, byte);
                echo_input(state, &termios, b'\n');
                let start = current_line_start(&state.input, &termios);
                for index in start..state.input.len() {
                    if let Some(InputItem::Byte(byte)) = state.input.get(index).copied() {
                        echo_input(state, &termios, byte);
                    }
                }
                return;
            }
            if control_matches(&termios, VEOF, byte) {
                self.enqueue(state, InputItem::Eof);
                return;
            }
        }

        self.enqueue(state, InputItem::Byte(byte));
        if termios.local_flags & ECHO != 0 || (byte == b'\n' && termios.local_flags & ECHONL != 0) {
            echo_input(state, &termios, byte);
        }
    }

    fn enqueue(&self, state: &mut TtyState, item: InputItem) {
        if state.input.len() >= INPUT_CAPACITY {
            let delimiter = is_delimiter(item, &state.termios);
            if delimiter && state.termios.local_flags & ICANON != 0 {
                if matches!(state.input.pop_back(), Some(InputItem::Byte(_))) {
                    state.input_bytes = state.input_bytes.saturating_sub(1);
                }
                push_input(state, item);
                state.canonical_ready = state.input.len();
                return;
            }
            if state.termios.input_flags & IMAXBEL != 0 {
                queue_echo(state, b"\x07");
            }
            return;
        }
        let delimiter = is_delimiter(item, &state.termios);
        push_input(state, item);
        if state.termios.local_flags & ICANON == 0
            || delimiter
            || state.input.len() == INPUT_CAPACITY
        {
            state.canonical_ready = state.input.len();
        }
    }

    fn flush_echo(&self, nonblocking: bool) {
        let mut column = self.output_lock.lock();
        let mut output = [0u8; OUTPUT_BATCH_SIZE];
        loop {
            let mut count = {
                let state = self.state.lock();
                output
                    .iter_mut()
                    .zip(state.echo.iter())
                    .map(|(output, input)| *output = *input)
                    .count()
            };
            if count == 0 {
                return;
            }
            loop {
                match self.backend.write(&output[..count], nonblocking) {
                    Ok(()) => break,
                    Err(FsError::WouldBlock) if nonblocking && count > 1 => count /= 2,
                    Err(_) => return,
                }
            }
            for byte in &output[..count] {
                update_output_column(&mut *column, *byte);
            }
            let mut state = self.state.lock();
            for _ in 0..count {
                state.echo.pop_front();
            }
        }
    }

    fn read_canonical(&self, buffer: &mut [u8], nonblocking: bool) -> FsResult<usize> {
        loop {
            if self.input_worker.lock().is_none() {
                self.pump_input(nonblocking);
                self.observe_hangup();
            }
            {
                let mut state = self.state.lock();
                if state.interrupted {
                    state.interrupted = false;
                    refresh_input_event(&state, &self.input_ready);
                    return Err(FsError::Interrupted);
                }
                if state.canonical_ready != 0 {
                    let mut read = 0;
                    while read < buffer.len() {
                        match pop_input(&mut state) {
                            Some(InputItem::Byte(byte)) => {
                                buffer[read] = byte;
                                read += 1;
                                if is_delimiter(InputItem::Byte(byte), &state.termios) {
                                    break;
                                }
                            }
                            Some(InputItem::Eof) | None => break,
                        }
                    }
                    if read == buffer.len()
                        && matches!(state.input.front(), Some(InputItem::Eof))
                        && state.canonical_ready != 0
                    {
                        let _ = pop_input(&mut state);
                    }
                    refresh_input_event(&state, &self.input_ready);
                    return Ok(read);
                }
                if state.hung_up {
                    return Ok(0);
                }
                if nonblocking {
                    return Err(FsError::WouldBlock);
                }
                let _ = self.input_ready.reset();
            }
            if self.input_worker.lock().is_some() {
                let _ = self.input_ready.wait();
            } else {
                clock::sleep(FALLBACK_POLL_INTERVAL);
            }
        }
    }

    fn read_raw(&self, buffer: &mut [u8], nonblocking: bool) -> FsResult<usize> {
        let (minimum, timeout_ns) = {
            let state = self.state.lock();
            (
                usize::from(state.termios.control[VMIN]).min(buffer.len()),
                u64::from(state.termios.control[VTIME]) * 100_000_000,
            )
        };
        let mut read = 0usize;
        let mut deadline = (minimum == 0 && timeout_ns != 0)
            .then(|| clock::monotonic_ns().saturating_add(timeout_ns));

        loop {
            if self.input_worker.lock().is_none() {
                self.pump_input(nonblocking);
                self.observe_hangup();
            }
            let before = read;
            {
                let mut state = self.state.lock();
                if state.interrupted {
                    if read != 0 {
                        refresh_input_event(&state, &self.input_ready);
                        return Ok(read);
                    }
                    state.interrupted = false;
                    refresh_input_event(&state, &self.input_ready);
                    return Err(FsError::Interrupted);
                }
                while read < buffer.len() {
                    match pop_input(&mut state) {
                        Some(InputItem::Byte(byte)) => {
                            buffer[read] = byte;
                            read += 1;
                        }
                        Some(InputItem::Eof) => {}
                        None => break,
                    }
                }
                if read != before && minimum != 0 && timeout_ns != 0 {
                    deadline = Some(clock::monotonic_ns().saturating_add(timeout_ns));
                }

                if read == buffer.len()
                    || (minimum != 0 && read >= minimum)
                    || (minimum == 0 && read != 0)
                    || (state.hung_up && state.input_bytes == 0)
                {
                    refresh_input_event(&state, &self.input_ready);
                    return Ok(read);
                }
                if nonblocking {
                    refresh_input_event(&state, &self.input_ready);
                    return if read == 0 {
                        Err(FsError::WouldBlock)
                    } else {
                        Ok(read)
                    };
                }
                if minimum == 0 && timeout_ns == 0 {
                    refresh_input_event(&state, &self.input_ready);
                    return Ok(read);
                }
                let _ = self.input_ready.reset();
            }

            let now = clock::monotonic_ns();
            if let Some(deadline) = deadline {
                if now >= deadline {
                    return Ok(read);
                }
                if self.input_worker.lock().is_some() {
                    if !clock::wait_timeout(
                        &self.input_ready,
                        Duration::from_nanos(deadline.saturating_sub(now).max(1)),
                    ) {
                        return Ok(read);
                    }
                } else {
                    clock::sleep(Duration::from_nanos(
                        deadline
                            .saturating_sub(now)
                            .min(FALLBACK_POLL_INTERVAL.as_nanos() as u64)
                            .max(1),
                    ));
                }
            } else if self.input_worker.lock().is_some() {
                let _ = self.input_ready.wait();
            } else {
                clock::sleep(FALLBACK_POLL_INTERVAL);
            }
        }
    }

    fn write_transformed(&self, input: &[u8], nonblocking: bool) -> FsResult<usize> {
        self.observe_hangup();
        if self.state.lock().hung_up {
            return Err(FsError::Io);
        }
        loop {
            let stopped = self.state.lock().output_stopped;
            if !stopped {
                break;
            }
            if nonblocking {
                return Err(FsError::WouldBlock);
            }
            if self.input_worker.lock().is_none() {
                self.pump_input(false);
            } else {
                let _ = self.output_resumed.wait();
            }
        }
        let (output_flags, discard_output) = {
            let state = self.state.lock();
            (
                state.termios.output_flags,
                state.termios.local_flags & FLUSHO != 0,
            )
        };
        if discard_output {
            return Ok(input.len());
        }
        let mut column = if nonblocking {
            self.output_lock.try_lock().ok_or(FsError::WouldBlock)?
        } else {
            self.output_lock.lock()
        };
        let transform_flags = OLCUC | ONLCR | OCRNL | ONOCR | ONLRET | TABDLY;
        if output_flags & OPOST == 0 || output_flags & transform_flags == 0 {
            let result = if nonblocking {
                let mut consumed = 0usize;
                for chunk in input.chunks(OUTPUT_BATCH_SIZE) {
                    if let Err(error) = self.backend.write(chunk, true) {
                        if consumed == 0 {
                            drop(column);
                            self.flush_echo(true);
                            return Err(error);
                        }
                        break;
                    }
                    consumed += chunk.len();
                }
                Ok(consumed)
            } else {
                self.backend.write(input, false).map(|()| input.len())
            };
            drop(column);
            self.flush_echo(true);
            return result;
        }

        let mut output = [0u8; OUTPUT_BATCH_SIZE];
        let mut output_len = 0usize;
        let mut buffered_input = 0usize;
        let mut consumed = 0usize;
        let mut current_column = *column;
        let mut sent_column = *column;

        for mut byte in input.iter().copied() {
            let mut transformed = [0u8; 8];
            let mut transformed_len = 0usize;
            let mut next_column = current_column;

            if output_flags & OLCUC != 0 {
                byte = byte.to_ascii_uppercase();
            }
            match byte {
                b'\r' if output_flags & ONOCR != 0 && current_column == 0 => {}
                b'\r' if output_flags & OCRNL != 0 => {
                    transformed[0] = b'\n';
                    transformed_len = 1;
                    if output_flags & ONLRET != 0 {
                        next_column = 0;
                    }
                }
                b'\n' if output_flags & ONLCR != 0 => {
                    transformed[..2].copy_from_slice(b"\r\n");
                    transformed_len = 2;
                    next_column = 0;
                }
                b'\t' if output_flags & TABDLY == TAB3 => {
                    transformed_len = 8 - (current_column & 7);
                    transformed[..transformed_len].fill(b' ');
                    next_column = current_column + transformed_len;
                }
                b'\r' => {
                    transformed[0] = byte;
                    transformed_len = 1;
                    next_column = 0;
                }
                b'\n' => {
                    transformed[0] = byte;
                    transformed_len = 1;
                    if output_flags & ONLRET != 0 {
                        next_column = 0;
                    }
                }
                b'\x08' => {
                    transformed[0] = byte;
                    transformed_len = 1;
                    next_column = current_column.saturating_sub(1);
                }
                _ => {
                    transformed[0] = byte;
                    transformed_len = 1;
                    if !is_control(byte) {
                        next_column = current_column.saturating_add(1);
                    }
                }
            }

            if output_len + transformed_len > output.len() {
                if let Err(error) = self.backend.write(&output[..output_len], nonblocking) {
                    *column = sent_column;
                    drop(column);
                    self.flush_echo(true);
                    return if consumed == 0 {
                        Err(error)
                    } else {
                        Ok(consumed)
                    };
                }
                consumed += buffered_input;
                buffered_input = 0;
                output_len = 0;
                sent_column = current_column;
            }
            output[output_len..output_len + transformed_len]
                .copy_from_slice(&transformed[..transformed_len]);
            output_len += transformed_len;
            buffered_input += 1;
            current_column = next_column;
        }

        if output_len != 0 {
            if let Err(error) = self.backend.write(&output[..output_len], nonblocking) {
                *column = sent_column;
                drop(column);
                self.flush_echo(true);
                return if consumed == 0 {
                    Err(error)
                } else {
                    Ok(consumed)
                };
            }
            consumed += buffered_input;
            sent_column = current_column;
        } else {
            consumed += buffered_input;
            sent_column = current_column;
        }
        *column = sent_column;
        drop(column);
        self.flush_echo(true);
        Ok(consumed)
    }

    fn apply_termios(&self, termios: Termios, drain: bool, flush_input: bool) -> FsResult<()> {
        let settings = termios.serial_settings()?;
        if drain {
            self.flush_echo(false);
        }
        {
            let _output = self.output_lock.lock();
            if drain {
                self.backend.flush()?;
            }
            self.backend.configure(settings)?;
        }
        if flush_input {
            self.flush_input();
        }
        let mut state = self.state.lock();
        state.termios = termios;
        recalculate_canonical_ready(&mut state);
        refresh_input_event(&state, &self.input_ready);
        Ok(())
    }

    fn flush_input(&self) {
        self.input_flushing.store(true, Ordering::Release);
        self.input_generation.fetch_add(1, Ordering::AcqRel);
        {
            let mut state = self.state.lock();
            clear_input(&mut state);
            state.interrupted = false;
        }
        let _ = self.backend.flush_input();
        let _ = self.input_ready.reset();
        self.input_flushing.store(false, Ordering::Release);
    }

    fn queued_input(&self) -> usize {
        let state = self.state.lock();
        if state.termios.local_flags & ICANON == 0 {
            return state.input_bytes;
        }
        state
            .input
            .iter()
            .take(state.canonical_ready)
            .filter(|item| matches!(item, InputItem::Byte(_)))
            .count()
    }
}

impl Tty {
    fn open(&self, _flags: u32) -> FsResult<usize> {
        let _lifecycle = self.lifecycle_lock.lock();
        if self.state.lock().exclusive && self.opens.load(Ordering::Acquire) != 0 {
            return Err(FsError::Busy);
        }
        self.backend.open()?;
        let previous = self.opens.fetch_add(1, Ordering::AcqRel);
        if previous == 0 {
            self.state.lock().hung_up = false;
            self.start_input_worker();
        }
        Ok(0)
    }

    fn close(&self, _file_context: usize, _flags: u32) {
        let _lifecycle = self.lifecycle_lock.lock();
        let previous = self.opens.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "console: TTY open count underflow");
        if previous == 1 {
            self.stop_input_worker();
            self.flush_echo(false);
            let _ = self.backend.flush();
        }
        self.backend.close();
        if previous == 1 && self.backend.reset_on_last_close() {
            let termios = Termios::with_baud(self.initial_baud)
                .expect("console: initial baud stopped being supported");
            *self.state.lock() = TtyState {
                termios,
                winsize: WinSize::default(),
                input: VecDeque::with_capacity(INPUT_CAPACITY),
                echo: VecDeque::with_capacity(ECHO_CAPACITY),
                canonical_ready: 0,
                input_bytes: 0,
                literal_next: false,
                output_stopped: false,
                interrupted: false,
                exclusive: false,
                soft_carrier: true,
                foreground_group: 0,
                session: 0,
                hung_up: false,
            };
            *self.output_lock.lock() = 0;
            let _ = self.input_ready.reset();
            let _ = self.output_resumed.signal();
        }
    }

    fn read(&self, buffer: &mut [u8], flags: u32) -> FsResult<usize> {
        if buffer.is_empty() {
            return Ok(0);
        }
        let canonical = self.state.lock().termios.local_flags & ICANON != 0;
        let nonblocking = flags & ddk::OPEN_NONBLOCK != 0;
        if canonical {
            self.read_canonical(buffer, nonblocking)
        } else {
            self.read_raw(buffer, nonblocking)
        }
    }

    fn write(&self, input: &[u8], flags: u32) -> FsResult<usize> {
        self.write_transformed(input, flags & ddk::OPEN_NONBLOCK != 0)
    }

    fn poll(&self, events: u16, flags: u32) -> u16 {
        if self.input_worker.lock().is_none() {
            self.pump_input(true);
            self.observe_hangup();
        }
        let (interrupted, readable, hung_up, output_stopped) = {
            let state = self.state.lock();
            (
                state.interrupted,
                if state.termios.local_flags & ICANON != 0 {
                    state.canonical_ready != 0
                } else {
                    state.input_bytes != 0
                },
                state.hung_up,
                state.output_stopped,
            )
        };
        let mut ready = 0u16;
        let read_events = if flags & 1 != 0 {
            events & (ddk::POLL_IN | ddk::POLL_RDNORM)
        } else {
            0
        };
        if interrupted || readable {
            ready |= read_events;
        }
        if hung_up {
            ready |= ddk::POLL_HUP;
        }
        if flags & 2 != 0 && !output_stopped && self.backend.writable() {
            ready |= events & (ddk::POLL_OUT | ddk::POLL_WRNORM);
        }
        ready
    }

    fn readable_event(&self) -> usize {
        self.input_worker
            .lock()
            .is_some()
            .then_some(self.input_ready.id())
            .unwrap_or(0)
    }

    fn writable_event(&self) -> usize {
        let state = self.state.lock();
        if state.output_stopped {
            self.output_resumed.id()
        } else {
            self.backend.writable_event().map_or(0, Event::id)
        }
    }

    fn terminal_state(&self) -> raw::TerminalState {
        let state = self.state.lock();
        raw::TerminalState {
            session: state.session,
            foreground_group: state.foreground_group,
            stop_background_output: u8::from(state.termios.local_flags & TOSTOP != 0),
            reserved: [0; 3],
        }
    }

    fn sync(&self) -> FsResult<()> {
        self.flush_echo(false);
        self.backend.flush()
    }

    fn ioctl(
        &self,
        context: raw::IoctlIdentity,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> FsResult<u64> {
        match request {
            TCGETS => {
                self.state.lock().termios.encode(argument)?;
            }
            TCSETS | TCSETSW | TCSETSF => {
                let termios = Termios::decode(argument)?;
                self.apply_termios(termios, request != TCSETS, request == TCSETSF)?;
            }
            TCSBRK => {
                self.flush_echo(false);
                let _output = self.output_lock.lock();
                if value == 0 {
                    self.backend.flush()?;
                    self.backend.send_break(250)?;
                } else {
                    self.backend.flush()?;
                }
            }
            TCXONC => match value {
                0 => {
                    self.state.lock().output_stopped = true;
                    let _ = self.output_resumed.reset();
                }
                1 => {
                    self.state.lock().output_stopped = false;
                    let _ = self.output_resumed.signal();
                }
                2 => {
                    let byte = self.state.lock().termios.control[VSTOP];
                    let _output = self.output_lock.lock();
                    self.backend.write(core::slice::from_ref(&byte), false)?;
                }
                3 => {
                    let byte = self.state.lock().termios.control[VSTART];
                    let _output = self.output_lock.lock();
                    self.backend.write(core::slice::from_ref(&byte), false)?;
                }
                _ => return Err(FsError::InvalidArgument),
            },
            TCFLSH => match value {
                0 => self.flush_input(),
                1 => {
                    self.state.lock().echo.clear();
                    self.backend.flush_output()?;
                }
                2 => {
                    self.flush_input();
                    self.state.lock().echo.clear();
                    self.backend.flush_output()?;
                }
                _ => return Err(FsError::InvalidArgument),
            },
            TIOCEXCL => self.state.lock().exclusive = true,
            TIOCNXCL => self.state.lock().exclusive = false,
            TIOCSCTTY => {
                if context.session_leader == 0 {
                    return Err(FsError::PermissionDenied);
                }
                let mut state = self.state.lock();
                if state.session != 0 && state.session != context.session && value == 0 {
                    return Err(FsError::PermissionDenied);
                }
                state.session = context.session;
                if state.foreground_group == 0 {
                    state.foreground_group = context.group;
                }
            }
            TIOCNOTTY => {
                let mut state = self.state.lock();
                if state.session != context.session {
                    return Err(FsError::PermissionDenied);
                }
                if context.session_leader != 0 {
                    let foreground_group = state.foreground_group;
                    state.session = 0;
                    state.foreground_group = 0;
                    drop(state);
                    if foreground_group > 0 {
                        signal_group(foreground_group, SIGHUP);
                        signal_group(foreground_group, SIGCONT);
                    }
                }
            }
            TIOCGPGRP => {
                let state = self.state.lock();
                if state.session != context.session {
                    return Err(FsError::NotTty);
                }
                put_i32(argument, state.foreground_group)?;
            }
            TIOCSPGRP => {
                let group = get_i32(argument)?;
                if group <= 0 {
                    return Err(FsError::InvalidArgument);
                }
                let mut state = self.state.lock();
                if state.session != context.session {
                    return Err(FsError::NotTty);
                }
                state.foreground_group = group;
            }
            TIOCOUTQ => put_i32(
                argument,
                i32::try_from(self.backend.queued_output()).unwrap_or(i32::MAX),
            )?,
            TIOCSTI => {
                let byte = *argument.first().ok_or(FsError::InvalidArgument)?;
                self.process_input(byte);
            }
            TIOCGWINSZ => self.state.lock().winsize.encode(argument)?,
            TIOCSWINSZ => {
                let winsize = WinSize::decode(argument)?;
                let foreground_group = {
                    let mut state = self.state.lock();
                    if state.winsize == winsize {
                        0
                    } else {
                        state.winsize = winsize;
                        state.foreground_group
                    }
                };
                if foreground_group > 0 {
                    signal_group(foreground_group, SIGWINCH);
                }
            }
            TIOCGSOFTCAR => put_i32(argument, i32::from(self.state.lock().soft_carrier))?,
            TIOCSSOFTCAR => self.state.lock().soft_carrier = get_i32(argument)? != 0,
            FIONREAD => {
                if self.input_worker.lock().is_none() {
                    self.pump_input(true);
                    self.observe_hangup();
                }
                put_i32(
                    argument,
                    i32::try_from(self.queued_input()).unwrap_or(i32::MAX),
                )?;
            }
            TIOCGSID => put_i32(argument, self.state.lock().session)?,
            TIOCGPATH => {
                if argument.len() != TTY_PATH_SIZE || self.path.len() + 1 > argument.len() {
                    return Err(FsError::InvalidArgument);
                }
                argument.fill(0);
                argument[..self.path.len()].copy_from_slice(self.path.as_bytes());
            }
            _ => return Err(FsError::NotTty),
        }
        Ok(0)
    }

    fn destroy(&self) {
        self.shutdown();
        self.backend.destroy();
        let _ = self.input_ready.destroy();
        let _ = self.output_resumed.destroy();
    }
}

impl Default for Termios {
    fn default() -> Self {
        let mut control = [0u8; NCCS];
        control[VINTR] = 3;
        control[VQUIT] = 28;
        control[VERASE] = 0x7f;
        control[VKILL] = 21;
        control[VEOF] = 4;
        control[VMIN] = 1;
        control[VSTART] = 17;
        control[VSTOP] = 19;
        control[VSUSP] = 26;
        control[VREPRINT] = 18;
        control[VDISCARD] = 15;
        control[VWERASE] = 23;
        control[VLNEXT] = 22;
        Self {
            input_flags: ICRNL | IXON,
            output_flags: OPOST | ONLCR,
            control_flags: CREAD | CS8 | CLOCAL | B9600,
            local_flags: ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN,
            line: 0,
            control,
            input_baud: 9600,
            output_baud: 9600,
        }
    }
}

impl Termios {
    fn with_baud(baud: u32) -> Option<Self> {
        let mut termios = Self::default();
        termios.control_flags &= !CBAUD;
        termios.control_flags |= baud_code(baud)?;
        termios.input_baud = baud;
        termios.output_baud = baud;
        Some(termios)
    }

    fn serial_settings(&self) -> FsResult<SerialSettings> {
        let data_bits = match self.control_flags & CSIZE {
            CS5 => 5,
            CS6 => 6,
            CS7 => 7,
            CS8 => 8,
            _ => return Err(FsError::InvalidArgument),
        };
        let baud = baud_rate(self.control_flags & CBAUD).ok_or(FsError::InvalidArgument)?;
        Ok(SerialSettings {
            baud,
            data_bits,
            stop_bits: if self.control_flags & CSTOPB != 0 {
                2
            } else {
                1
            },
            parity: self.control_flags & PARENB != 0,
            odd_parity: self.control_flags & PARODD != 0,
        })
    }

    fn encode(&self, output: &mut [u8]) -> FsResult<()> {
        if output.len() != TERMIOS_SIZE {
            return Err(FsError::InvalidArgument);
        }
        output.fill(0);
        output[0..4].copy_from_slice(&self.input_flags.to_ne_bytes());
        output[4..8].copy_from_slice(&self.output_flags.to_ne_bytes());
        output[8..12].copy_from_slice(&self.control_flags.to_ne_bytes());
        output[12..16].copy_from_slice(&self.local_flags.to_ne_bytes());
        output[16] = self.line;
        output[17..49].copy_from_slice(&self.control);
        output[52..56].copy_from_slice(&self.input_baud.to_ne_bytes());
        output[56..60].copy_from_slice(&self.output_baud.to_ne_bytes());
        Ok(())
    }

    fn decode(input: &[u8]) -> FsResult<Self> {
        if input.len() != TERMIOS_SIZE {
            return Err(FsError::InvalidArgument);
        }
        let mut control = [0u8; NCCS];
        control.copy_from_slice(&input[17..49]);
        Ok(Self {
            input_flags: u32::from_ne_bytes(input[0..4].try_into().expect("termios iflag width")),
            output_flags: u32::from_ne_bytes(input[4..8].try_into().expect("termios oflag width")),
            control_flags: u32::from_ne_bytes(
                input[8..12].try_into().expect("termios cflag width"),
            ),
            local_flags: u32::from_ne_bytes(input[12..16].try_into().expect("termios lflag width")),
            line: input[16],
            control,
            input_baud: u32::from_ne_bytes(input[52..56].try_into().expect("termios ibaud width")),
            output_baud: u32::from_ne_bytes(input[56..60].try_into().expect("termios obaud width")),
        })
    }
}

impl WinSize {
    fn encode(self, output: &mut [u8]) -> FsResult<()> {
        if output.len() != WINSIZE_SIZE {
            return Err(FsError::InvalidArgument);
        }
        output[0..2].copy_from_slice(&self.rows.to_ne_bytes());
        output[2..4].copy_from_slice(&self.columns.to_ne_bytes());
        output[4..6].copy_from_slice(&self.x_pixels.to_ne_bytes());
        output[6..8].copy_from_slice(&self.y_pixels.to_ne_bytes());
        Ok(())
    }

    fn decode(input: &[u8]) -> FsResult<Self> {
        if input.len() != WINSIZE_SIZE {
            return Err(FsError::InvalidArgument);
        }
        Ok(Self {
            rows: u16::from_ne_bytes(input[0..2].try_into().expect("winsize row width")),
            columns: u16::from_ne_bytes(input[2..4].try_into().expect("winsize column width")),
            x_pixels: u16::from_ne_bytes(input[4..6].try_into().expect("winsize x width")),
            y_pixels: u16::from_ne_bytes(input[6..8].try_into().expect("winsize y width")),
        })
    }
}

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

fn input_ready(state: &TtyState) -> bool {
    if state.termios.local_flags & ICANON != 0 {
        state.canonical_ready != 0
    } else {
        state.input_bytes != 0
    }
}

fn refresh_input_event(state: &TtyState, event: &Event) {
    if state.interrupted || state.hung_up || input_ready(state) {
        let _ = event.signal();
    } else {
        let _ = event.reset();
    }
}

fn push_input(state: &mut TtyState, item: InputItem) {
    if matches!(item, InputItem::Byte(_)) {
        state.input_bytes += 1;
    }
    state.input.push_back(item);
}

fn pop_input(state: &mut TtyState) -> Option<InputItem> {
    let item = state.input.pop_front()?;
    if state.canonical_ready != 0 {
        state.canonical_ready -= 1;
    }
    if matches!(item, InputItem::Byte(_)) {
        state.input_bytes = state.input_bytes.saturating_sub(1);
    }
    Some(item)
}

fn clear_input(state: &mut TtyState) {
    state.input.clear();
    state.canonical_ready = 0;
    state.input_bytes = 0;
    state.literal_next = false;
}

fn recalculate_canonical_ready(state: &mut TtyState) {
    if state.termios.local_flags & ICANON == 0 {
        state.canonical_ready = state.input.len();
        return;
    }
    state.canonical_ready = state
        .input
        .iter()
        .enumerate()
        .filter_map(|(index, item)| is_delimiter(*item, &state.termios).then_some(index + 1))
        .next_back()
        .unwrap_or(0);
}

fn is_delimiter(item: InputItem, termios: &Termios) -> bool {
    match item {
        InputItem::Eof | InputItem::Byte(b'\n') => true,
        InputItem::Byte(byte) => {
            (termios.control[VEOL] != 0 && byte == termios.control[VEOL])
                || (termios.local_flags & IEXTEN != 0
                    && termios.control[VEOL2] != 0
                    && byte == termios.control[VEOL2])
        }
    }
}

fn discard_current_line(input: &mut VecDeque<InputItem>, termios: &Termios) -> usize {
    let mut removed = 0;
    while let Some(item) = input.back() {
        if is_delimiter(*item, termios) {
            break;
        }
        if matches!(input.pop_back(), Some(InputItem::Byte(_))) {
            removed += 1;
        }
    }
    removed
}

fn erase_last_character(input: &mut VecDeque<InputItem>, termios: &Termios) -> (usize, usize) {
    let Some(item) = input.back().copied() else {
        return (0, 0);
    };
    if is_delimiter(item, termios) {
        return (0, 0);
    }
    let Some(InputItem::Byte(mut byte)) = input.pop_back() else {
        return (0, 0);
    };
    let mut erased = 1;
    if termios.input_flags & IUTF8 != 0 && byte & 0xc0 == 0x80 {
        while let Some(InputItem::Byte(previous)) = input.back().copied() {
            if is_delimiter(InputItem::Byte(previous), termios) {
                break;
            }
            byte = previous;
            input.pop_back();
            erased += 1;
            if byte & 0xc0 != 0x80 {
                break;
            }
        }
    }
    let columns = if is_control(byte) && termios.local_flags & ECHOCTL != 0 {
        2
    } else {
        1
    };
    (erased, columns)
}

fn erase_word(input: &mut VecDeque<InputItem>, termios: &Termios) -> usize {
    let mut erased = 0;
    while input.back().is_some_and(|item| {
        !is_delimiter(*item, termios)
            && matches!(item, InputItem::Byte(byte) if byte.is_ascii_whitespace())
    }) {
        input.pop_back();
        erased += 1;
    }
    while input.back().is_some_and(|item| {
        !is_delimiter(*item, termios)
            && matches!(item, InputItem::Byte(byte) if !byte.is_ascii_whitespace())
    }) {
        input.pop_back();
        erased += 1;
    }
    erased
}

fn current_line_start(input: &VecDeque<InputItem>, termios: &Termios) -> usize {
    input
        .iter()
        .rposition(|item| is_delimiter(*item, termios))
        .map_or(0, |index| index + 1)
}

fn is_control(byte: u8) -> bool {
    byte < b' ' && !matches!(byte, b'\n' | b'\t') || byte == 0x7f
}

fn queue_echo(state: &mut TtyState, bytes: &[u8]) {
    if state.termios.local_flags & FLUSHO != 0 {
        return;
    }
    let overflow = state
        .echo
        .len()
        .saturating_add(bytes.len())
        .saturating_sub(ECHO_CAPACITY);
    for _ in 0..overflow {
        state.echo.pop_front();
    }
    state.echo.extend(bytes.iter().copied());
}

fn echo_input(state: &mut TtyState, termios: &Termios, byte: u8) {
    if byte == b'\n' {
        if termios.output_flags & (OPOST | ONLCR) == (OPOST | ONLCR) {
            queue_echo(state, b"\r\n");
        } else {
            queue_echo(state, b"\n");
        }
    } else if is_control(byte) && termios.local_flags & ECHOCTL != 0 {
        echo_control(state, byte);
    } else {
        queue_echo(state, core::slice::from_ref(&byte));
    }
}

fn echo_control(state: &mut TtyState, byte: u8) {
    let shown = if byte == 0x7f { b'?' } else { byte ^ 0x40 };
    queue_echo(state, &[b'^', shown]);
}

fn update_output_column(column: &mut usize, byte: u8) {
    match byte {
        b'\r' | b'\n' => *column = 0,
        b'\x08' => *column = column.saturating_sub(1),
        b'\t' => *column += 8 - (*column & 7),
        _ if !is_control(byte) => *column = column.saturating_add(1),
        _ => {}
    }
}

fn control_matches(termios: &Termios, index: usize, byte: u8) -> bool {
    termios.control[index] != 0 && byte == termios.control[index]
}

fn get_i32(input: &[u8]) -> FsResult<i32> {
    if input.len() != 4 {
        return Err(FsError::InvalidArgument);
    }
    Ok(i32::from_ne_bytes(
        input.try_into().expect("ioctl integer width"),
    ))
}

fn put_i32(output: &mut [u8], value: i32) -> FsResult<()> {
    if output.len() != 4 {
        return Err(FsError::InvalidArgument);
    }
    output.copy_from_slice(&value.to_ne_bytes());
    Ok(())
}

fn baud_rate(code: u32) -> Option<u32> {
    Some(match code {
        1 => 50,
        2 => 75,
        3 => 110,
        4 => 134,
        5 => 150,
        6 => 200,
        7 => 300,
        8 => 600,
        9 => 1200,
        10 => 1800,
        11 => 2400,
        12 => 4800,
        13 => 9600,
        14 => 19_200,
        15 => 38_400,
        0o10001 => 57_600,
        0o10002 => 115_200,
        0o10003 => 230_400,
        0o10004 => 460_800,
        0o10005 => 500_000,
        0o10006 => 576_000,
        0o10007 => 921_600,
        0o10010 => 1_000_000,
        0o10011 => 1_152_000,
        0o10012 => 1_500_000,
        0o10013 => 2_000_000,
        0o10014 => 2_500_000,
        0o10015 => 3_000_000,
        0o10016 => 3_500_000,
        0o10017 => 4_000_000,
        _ => return None,
    })
}

fn baud_code(baud: u32) -> Option<u32> {
    Some(match baud {
        50 => 1,
        75 => 2,
        110 => 3,
        134 => 4,
        150 => 5,
        200 => 6,
        300 => 7,
        600 => 8,
        1200 => 9,
        1800 => 10,
        2400 => 11,
        4800 => 12,
        9600 => 13,
        19_200 => 14,
        38_400 => 15,
        57_600 => 0o10001,
        115_200 => 0o10002,
        230_400 => 0o10003,
        460_800 => 0o10004,
        500_000 => 0o10005,
        576_000 => 0o10006,
        921_600 => 0o10007,
        1_000_000 => 0o10010,
        1_152_000 => 0o10011,
        1_500_000 => 0o10012,
        2_000_000 => 0o10013,
        2_500_000 => 0o10014,
        3_000_000 => 0o10015,
        3_500_000 => 0o10016,
        4_000_000 => 0o10017,
        _ => return None,
    })
}

fn callback_status(result: FsResult<()>) -> i32 {
    result.map_or_else(|error| error.status(), |_| ddk::OK)
}

fn callback_count(result: FsResult<usize>) -> i64 {
    result.map_or_else(
        |error| i64::from(error.status()),
        |count| i64::try_from(count).unwrap_or(i64::MAX),
    )
}

unsafe fn terminal_from_context<'terminal>(context: *mut c_void) -> Option<&'terminal Tty> {
    if context.is_null() {
        return None;
    }
    // SAFETY: provider registration stores a `Box<Tty>` address in the copied
    // node table and frees it only after devfs has revoked the node.
    Some(unsafe { &*context.cast::<Tty>() })
}

unsafe extern "C" fn node_open(context: *mut c_void, flags: u32, out: *mut usize) -> i32 {
    if out.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return ddk::EINVAL;
    };
    match terminal.open(flags) {
        Ok(file) => {
            // SAFETY: checked non-null above and points to caller output.
            unsafe { out.write(file) };
            ddk::OK
        }
        Err(error) => error.status(),
    }
}

unsafe extern "C" fn node_close(context: *mut c_void, file: usize, flags: u32) {
    let _ = (file, flags);
    // SAFETY: the node table supplies a live terminal context.
    if let Some(terminal) = unsafe { terminal_from_context(context) } {
        terminal.close(file, flags);
    }
}

unsafe extern "C" fn node_read(
    context: *mut c_void,
    _file: usize,
    _offset: u64,
    data: *mut u8,
    length: usize,
    flags: u32,
) -> i64 {
    if length != 0 && data.is_null() {
        return i64::from(ddk::EINVAL);
    }
    let output = if length == 0 {
        &mut []
    } else {
        // SAFETY: the node ABI guarantees writable `length` bytes.
        unsafe { core::slice::from_raw_parts_mut(data, length) }
    };
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return i64::from(ddk::EINVAL);
    };
    callback_count(terminal.read(output, flags))
}

unsafe extern "C" fn node_write(
    context: *mut c_void,
    _file: usize,
    _offset: u64,
    data: *const u8,
    length: usize,
    flags: u32,
) -> i64 {
    if length != 0 && data.is_null() {
        return i64::from(ddk::EINVAL);
    }
    let input = if length == 0 {
        &[]
    } else {
        // SAFETY: the node ABI guarantees readable `length` bytes.
        unsafe { core::slice::from_raw_parts(data, length) }
    };
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return i64::from(ddk::EINVAL);
    };
    callback_count(terminal.write(input, flags))
}

unsafe extern "C" fn node_sync(context: *mut c_void) -> i32 {
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return ddk::EINVAL;
    };
    callback_status(terminal.sync())
}

unsafe extern "C" fn node_poll(
    context: *mut c_void,
    _file: usize,
    _offset: u64,
    events: u16,
    flags: u32,
) -> i64 {
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return i64::from(ddk::EINVAL);
    };
    i64::from(terminal.poll(events, flags))
}

unsafe extern "C" fn node_ioctl(
    context: *mut c_void,
    _file: usize,
    identity: *const raw::IoctlIdentity,
    request: u64,
    value: u64,
    argument: *mut u8,
    argument_length: usize,
) -> i64 {
    if identity.is_null() || (argument_length != 0 && argument.is_null()) {
        return i64::from(ddk::EINVAL);
    }
    // SAFETY: the node ABI guarantees these argument records for the callback.
    let identity = unsafe { *identity };
    // SAFETY: the node ABI guarantees writable `argument_length` bytes when
    // nonzero; null is accepted for the empty slice.
    let argument = if argument_length == 0 {
        &mut []
    } else {
        // SAFETY: validated non-null above and writable for this exact range.
        unsafe { core::slice::from_raw_parts_mut(argument, argument_length) }
    };
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return i64::from(ddk::EINVAL);
    };
    terminal
        .ioctl(identity, request, value, argument)
        .map_or_else(|error| i64::from(error.status()), |result| result as i64)
}

unsafe extern "C" fn node_readable_event(context: *mut c_void, _file: usize) -> usize {
    // SAFETY: the node table supplies a live terminal context.
    unsafe { terminal_from_context(context) }.map_or(0, Tty::readable_event)
}

unsafe extern "C" fn node_writable_event(context: *mut c_void, _file: usize) -> usize {
    // SAFETY: the node table supplies a live terminal context.
    unsafe { terminal_from_context(context) }.map_or(0, Tty::writable_event)
}

unsafe extern "C" fn node_hangup_event(context: *mut c_void, _file: usize) -> usize {
    // SAFETY: the node table supplies a live terminal context.
    unsafe { terminal_from_context(context) }.map_or(0, Tty::readable_event)
}

unsafe extern "C" fn node_terminal_state(
    context: *mut c_void,
    out: *mut raw::TerminalState,
) -> i32 {
    if out.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: the node table supplies a live terminal context.
    let Some(terminal) = (unsafe { terminal_from_context(context) }) else {
        return ddk::EINVAL;
    };
    // SAFETY: checked non-null above and points to caller output.
    unsafe { out.write(terminal.terminal_state()) };
    ddk::OK
}

fn terminal_node_ops(context: *mut c_void) -> raw::NodeOps {
    raw::NodeOps {
        size: raw::NODE_OPS_SIZE,
        context,
        open: Some(node_open),
        close: Some(node_close),
        initial_offset: None,
        read: Some(node_read),
        write: Some(node_write),
        size_bytes: None,
        sync: Some(node_sync),
        poll: Some(node_poll),
        ioctl: Some(node_ioctl),
        readable_event: Some(node_readable_event),
        writable_event: Some(node_writable_event),
        hangup_event: Some(node_hangup_event),
        terminal_state: Some(node_terminal_state),
    }
}

struct ProviderState {
    class: Class,
    provider: TtyProvider,
}

static PROVIDER: TicketLock<Option<ProviderState>> = TicketLock::new(None);

static CLASS_DEFINITION: raw::ClassDef = raw::ClassDef {
    size: raw::CLASS_DEF_SIZE,
    attach: None,
    detach: None,
    context: ptr::null_mut(),
};

static PROVIDER_OPERATIONS: raw::TtyProviderOps = raw::TtyProviderOps {
    size: raw::TTY_PROVIDER_OPS_SIZE,
    context: ptr::null_mut(),
    register: Some(provider_register),
    unregister: Some(provider_unregister),
};

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn provider_register(
    _context: *mut c_void,
    _backend_module: *const raw::Module,
    device: *const raw::Device,
    parent: u64,
    name: *const c_char,
    mode: u16,
    baud: u32,
    operations: *const raw::ConsoleOps,
    out: *mut *mut c_void,
) -> i32 {
    if name.is_null() || out.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: the broker validates the C string before invoking this callback.
    let c_name = unsafe { CStr::from_ptr(name) };
    let Ok(name) = c_name.to_str() else {
        return ddk::EINVAL;
    };
    // SAFETY: the provider ABI keeps the operation prefix readable for this
    // callback; `copy` validates its size and required entries.
    let Ok(backend) = (unsafe { Backend::copy(operations) }) else {
        return ddk::EINVAL;
    };
    let Ok(devfs) = Devfs::current() else {
        return ddk::ENODEV;
    };
    let Ok(root) = devfs.root() else {
        return ddk::ENODEV;
    };
    let path: Box<str> = if parent == root {
        format!("/dev/{name}").into_boxed_str()
    } else {
        format!("/dev/pts/{name}").into_boxed_str()
    };
    let Ok(mut terminal) = Tty::new(backend, path, baud) else {
        return ddk::EINVAL;
    };
    let context = (&raw mut *terminal).cast::<c_void>();
    let node_ops = terminal_node_ops(context);
    // SAFETY: `node_ops` refers to the boxed terminal, which remains live until
    // node removal joins its worker.
    let Ok(node) = (unsafe { devfs.create_character(parent, c_name, mode, &node_ops) }) else {
        terminal.destroy();
        return ddk::EIO;
    };
    // SAFETY: the optional device pointer comes from the kernel registration
    // callback and stays live for its duration.
    let device = unsafe { Device::from_raw(device) };
    let member = {
        let provider = PROVIDER.lock();
        let Some(provider) = provider.as_ref() else {
            let _ = devfs.remove(node);
            terminal.destroy();
            return ddk::ENODEV;
        };
        // SAFETY: no class member operations are exposed; the terminal
        // allocation remains valid until the membership is removed.
        unsafe { provider.class.add(device, c_name, ptr::null(), 0, context) }
    };
    let Ok(member) = member else {
        let _ = devfs.remove(node);
        terminal.destroy();
        return ddk::EIO;
    };
    *terminal.node.lock() = Some(node);
    *terminal.member.lock() = Some(member);
    // SAFETY: checked non-null above and the broker consumes this receipt
    // through `provider_unregister`.
    unsafe { out.write(Box::into_raw(terminal).cast()) };
    ddk::OK
}

unsafe extern "C" fn provider_unregister(_context: *mut c_void, receipt: *mut c_void) -> i32 {
    if receipt.is_null() {
        return ddk::EINVAL;
    }
    // SAFETY: the broker calls this at most once for the receipt returned from
    // `provider_register`; devfs has no active callbacks after removal.
    let terminal = unsafe { &mut *receipt.cast::<Tty>() };
    let Some(node) = terminal.node.lock().take() else {
        return ddk::EINVAL;
    };
    let Ok(devfs) = Devfs::current() else {
        *terminal.node.lock() = Some(node);
        return ddk::ENODEV;
    };
    if devfs.remove(node).is_err() {
        *terminal.node.lock() = Some(node);
        return ddk::EBUSY;
    }
    if let Some(mut member) = terminal.member.lock().take() {
        member.remove();
    }
    terminal.destroy();
    // SAFETY: the devfs node is gone and its worker was joined by `destroy`.
    drop(unsafe { Box::from_raw(receipt.cast::<Tty>()) });
    ddk::OK
}

fn console_init(_module: Module) -> DdkResult<()> {
    // SAFETY: the class definition and its callback table are static.
    let class = unsafe { Class::register(c"tty", &CLASS_DEFINITION) }?;
    // SAFETY: the provider definition and its callback table are static.
    let provider = unsafe { TtyProvider::register(&PROVIDER_OPERATIONS) }?;
    *PROVIDER.lock() = Some(ProviderState { class, provider });
    Ok(())
}

fn console_exit(_module: Module) {
    let Some(mut state) = PROVIDER.lock().take() else {
        return;
    };
    if state.provider.unregister().is_err() {
        *PROVIDER.lock() = Some(state);
        return;
    }
    let _ = state.class.unregister();
}

ddk::module!(
    b"console\0",
    b"Shared terminal line discipline\0",
    console_init,
    console_exit,
);
