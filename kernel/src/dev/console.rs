//! Reusable character-console and TTY line discipline.

use alloc::{
    boxed::Box,
    collections::VecDeque,
    sync::{Arc, Weak},
};
use core::{
    ptr,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    time::Duration,
};

use crate::{
    fs::{
        Error as FsError, IoctlContext, OpenFlags, PollEvents, Result as FsResult,
        devtempfs::DeviceNodeOps, vnode::TerminalState,
    },
    proc,
    sys::{clock, event::Event, sched, sync::Mutex},
};

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

/// Hardware backend consumed by the shared TTY line discipline.
pub trait ConsoleBackend: Send + Sync {
    /// Opens one TTY file description.
    fn open(&self) -> FsResult<()> {
        Ok(())
    }

    /// Closes one TTY file description.
    fn close(&self) {}

    /// Returns one immediately available byte.
    fn try_read(&self) -> Option<u8>;

    /// Reads immediately available input bytes.
    fn read(&self, output: &mut [u8]) -> FsResult<usize> {
        self.read_fallback(output)
    }

    /// Compatibility implementation for byte-oriented backends.
    fn read_fallback(&self, output: &mut [u8]) -> FsResult<usize> {
        let mut read = 0;
        while read < output.len() {
            let Some(byte) = self.try_read() else {
                break;
            };
            output[read] = byte;
            read += 1;
        }
        Ok(read)
    }

    /// Writes every supplied byte or reports why no progress was possible.
    fn write(&self, bytes: &[u8], nonblocking: bool) -> FsResult<()>;

    /// Applies UART framing settings.
    fn configure(&self, _settings: SerialSettings) -> FsResult<()> {
        Ok(())
    }

    /// Waits for pending output to reach the hardware.
    fn flush(&self) -> FsResult<()> {
        Ok(())
    }

    /// Discards queued hardware or backend input.
    fn flush_input(&self) -> FsResult<()> {
        let mut bytes = [0u8; INPUT_BATCH_SIZE];
        while self.read(&mut bytes)? != 0 {}
        Ok(())
    }

    /// Discards queued hardware or backend output.
    fn flush_output(&self) -> FsResult<()> {
        Ok(())
    }

    /// Generates a serial break condition.
    fn send_break(&self, _duration: u64) -> FsResult<()> {
        Ok(())
    }

    /// Returns whether the peer or hardware endpoint has disconnected.
    fn hung_up(&self) -> bool {
        false
    }

    /// Returns whether one output byte can be accepted without waiting.
    fn writable(&self) -> bool {
        true
    }

    /// Returns output bytes still queued in the backend.
    fn queued_output(&self) -> usize {
        0
    }

    /// Persistent event signaled while input may be read.
    fn readable_event(&self) -> Option<&Event> {
        None
    }

    /// Persistent event signaled while output queue space is available.
    fn writable_event(&self) -> Option<&Event> {
        None
    }

    /// Persistent event signaled after endpoint disconnection.
    fn hangup_event(&self) -> Option<&Event> {
        None
    }

    /// Returns whether terminal state should reset after the final close.
    fn reset_on_last_close(&self) -> bool {
        false
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
    stop: Event,
    exited: Event,
}

/// Character terminal layered over a platform console backend.
pub struct Tty {
    backend: Arc<dyn ConsoleBackend>,
    path: Box<str>,
    state: Mutex<TtyState>,
    lifecycle_lock: Mutex<()>,
    output_lock: Mutex<usize>,
    input_flushing: AtomicBool,
    input_generation: AtomicU64,
    input_ready: Event,
    output_resumed: Event,
    self_ref: Weak<Tty>,
    input_worker: Mutex<Option<Arc<InputWorker>>>,
    opens: AtomicU64,
    initial_baud: u32,
}

impl Tty {
    /// Creates a terminal with cooked defaults at the hardware's initial baud.
    pub fn new(
        backend: Arc<dyn ConsoleBackend>,
        path: Box<str>,
        baud: u32,
    ) -> super::Result<Arc<Self>> {
        let termios = Termios::with_baud(baud).ok_or(super::Error::Unsupported)?;
        Ok(Arc::new_cyclic(|weak| Self {
            backend,
            path,
            state: Mutex::new(TtyState {
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
            lifecycle_lock: Mutex::new(()),
            output_lock: Mutex::new(0),
            input_flushing: AtomicBool::new(false),
            input_generation: AtomicU64::new(0),
            input_ready: Event::new(),
            output_resumed: Event::new(),
            self_ref: weak.clone(),
            input_worker: Mutex::new(None),
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
        let control = Arc::new(InputWorker {
            stop: Event::new(),
            exited: Event::new(),
        });
        let tty = self
            .self_ref
            .upgrade()
            .expect("console: live TTY lost its self reference");
        let task_control = control.clone();
        sched::run(move || tty.input_worker_loop(task_control));
        *worker = Some(control);
    }

    fn stop_input_worker(&self) {
        let Some(worker) = self.input_worker.lock().take() else {
            return;
        };
        worker.stop.signal();
        worker.exited.wait();
    }

    fn input_worker_loop(&self, worker: Arc<InputWorker>) {
        loop {
            self.pump_input(true);
            if self.observe_hangup() {
                break;
            }

            let readable = self.backend.readable_event();
            let hangup = self
                .backend
                .hangup_event()
                .filter(|hangup| readable.is_none_or(|readable| !ptr::eq(readable, *hangup)));
            let writable = if self.state.lock().echo.is_empty() {
                None
            } else {
                self.backend.writable_event()
            };
            let stopped = match (readable, hangup, writable) {
                (Some(readable), Some(hangup), Some(writable)) => {
                    Event::wait_any(&[&worker.stop, readable, hangup, writable]) == 0
                }
                (Some(readable), Some(hangup), None) => {
                    Event::wait_any(&[&worker.stop, readable, hangup]) == 0
                }
                (Some(readable), None, Some(writable)) => {
                    Event::wait_any(&[&worker.stop, readable, writable]) == 0
                }
                (None, Some(hangup), Some(writable)) => {
                    Event::wait_any(&[&worker.stop, hangup, writable]) == 0
                }
                (Some(readable), None, None) => Event::wait_any(&[&worker.stop, readable]) == 0,
                (None, Some(hangup), None) => Event::wait_any(&[&worker.stop, hangup]) == 0,
                (None, None, Some(writable)) => {
                    Event::wait_any(&[&worker.stop, writable]) == 0
                }
                (None, None, None) => true,
            };
            if stopped {
                break;
            }
        }
        worker.exited.signal();
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
        self.input_ready.signal();
        self.output_resumed.signal();
        if foreground_group > 0 {
            proc::signal::send_kernel_process_group(
                foreground_group as usize,
                proc::signal::SIGHUP,
            );
            proc::signal::send_kernel_process_group(
                foreground_group as usize,
                proc::signal::SIGCONT,
            );
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
                self.input_ready.signal();
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
            self.input_ready.signal();
        }
        if foreground_group > 0 {
            for signal in [
                proc::signal::SIGINT,
                proc::signal::SIGQUIT,
                proc::signal::SIGTSTP,
            ] {
                if signals & (1u64 << signal) != 0 {
                    proc::signal::send_kernel_process_group(foreground_group as usize, signal);
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
                    self.output_resumed.reset();
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
                self.output_resumed.signal();
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
            Some(proc::signal::SIGINT)
        } else if control_matches(&termios, VQUIT, byte) {
            Some(proc::signal::SIGQUIT)
        } else if control_matches(&termios, VSUSP, byte) {
            Some(proc::signal::SIGTSTP)
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
            self.output_resumed.signal();
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
                self.input_ready.reset();
            }
            if self.input_worker.lock().is_some() {
                self.input_ready.wait();
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
                self.input_ready.reset();
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
                self.input_ready.wait();
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
                self.output_resumed.wait();
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
        self.input_ready.reset();
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

impl DeviceNodeOps for Tty {
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
            self.input_ready.reset();
            self.output_resumed.signal();
        }
    }

    fn read_at_with_flags(
        &self,
        _file_context: usize,
        _offset: u64,
        buffer: &mut [u8],
        flags: u32,
    ) -> FsResult<usize> {
        if buffer.is_empty() {
            return Ok(0);
        }
        let nonblocking = OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK);
        if self.state.lock().termios.local_flags & ICANON != 0 {
            self.read_canonical(buffer, nonblocking)
        } else {
            self.read_raw(buffer, nonblocking)
        }
    }

    fn read_at(&self, offset: u64, buffer: &mut [u8]) -> FsResult<usize> {
        self.read_at_with_flags(0, offset, buffer, 0)
    }

    fn write_at(&self, _offset: u64, buffer: &[u8]) -> FsResult<usize> {
        self.write_transformed(buffer, false)
    }

    fn write_at_with_flags(
        &self,
        _file_context: usize,
        _offset: u64,
        buffer: &[u8],
        flags: u32,
    ) -> FsResult<usize> {
        let nonblocking = OpenFlags::from_bits_retain(flags).contains(OpenFlags::NONBLOCK);
        self.write_transformed(buffer, nonblocking)
    }

    fn poll(
        &self,
        _file_context: usize,
        _offset: u64,
        events: PollEvents,
        flags: u32,
    ) -> FsResult<PollEvents> {
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
        let mut ready = PollEvents::empty();
        let flags = OpenFlags::from_bits_retain(flags);
        let read_events = if flags.contains(OpenFlags::READ) {
            events & (PollEvents::IN | PollEvents::RDNORM)
        } else {
            PollEvents::empty()
        };
        if interrupted || readable {
            ready |= read_events;
        }
        if hung_up {
            ready |= PollEvents::HUP;
        }
        if flags.contains(OpenFlags::WRITE) && !output_stopped && self.backend.writable() {
            ready |= events & (PollEvents::OUT | PollEvents::WRNORM);
        }
        Ok(ready)
    }

    fn poll_events<'a>(
        &'a self,
        _file_context: usize,
        events: PollEvents,
        output: &mut alloc::vec::Vec<&'a Event>,
    ) -> bool {
        let state = self.state.lock();
        let mut complete = true;
        if events.intersects(PollEvents::IN | PollEvents::RDNORM | PollEvents::HUP) {
            if self.input_worker.lock().is_some() {
                output.push(&self.input_ready);
            } else {
                complete = false;
            }
        }
        if events.intersects(PollEvents::OUT | PollEvents::WRNORM) {
            if state.output_stopped {
                output.push(&self.output_resumed);
            } else if let Some(event) = self.backend.writable_event() {
                output.push(event);
            } else {
                complete = false;
            }
        }
        complete
    }

    fn terminal_state(&self) -> Option<TerminalState> {
        let state = self.state.lock();
        Some(TerminalState {
            session: state.session,
            foreground_group: state.foreground_group,
            stop_background_output: state.termios.local_flags & TOSTOP != 0,
        })
    }

    fn sync(&self) -> FsResult<()> {
        self.flush_echo(false);
        self.backend.flush()
    }

    fn ioctl(
        &self,
        _file_context: usize,
        context: IoctlContext,
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
                    self.output_resumed.reset();
                }
                1 => {
                    self.state.lock().output_stopped = false;
                    self.output_resumed.signal();
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
                if !context.is_session_leader {
                    return Err(FsError::PermissionDenied);
                }
                let mut state = self.state.lock();
                if state.session != 0 && state.session != context.session_id && value == 0 {
                    return Err(FsError::PermissionDenied);
                }
                state.session = context.session_id;
                if state.foreground_group == 0 {
                    state.foreground_group = context.process_group;
                }
            }
            TIOCNOTTY => {
                let mut state = self.state.lock();
                if state.session != context.session_id {
                    return Err(FsError::PermissionDenied);
                }
                if context.is_session_leader {
                    let foreground_group = state.foreground_group;
                    state.session = 0;
                    state.foreground_group = 0;
                    drop(state);
                    if foreground_group > 0 {
                        proc::signal::send_kernel_process_group(
                            foreground_group as usize,
                            proc::signal::SIGHUP,
                        );
                        proc::signal::send_kernel_process_group(
                            foreground_group as usize,
                            proc::signal::SIGCONT,
                        );
                    }
                }
            }
            TIOCGPGRP => {
                let state = self.state.lock();
                if state.session != context.session_id {
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
                if state.session != context.session_id {
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
                    proc::signal::send_kernel_process_group(
                        foreground_group as usize,
                        proc::signal::SIGWINCH,
                    );
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
        event.signal();
    } else {
        event.reset();
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

fn erase_last_character(
    input: &mut VecDeque<InputItem>,
    termios: &Termios,
) -> (usize, usize) {
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
