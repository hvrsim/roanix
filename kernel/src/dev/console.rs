//! Reusable character-console and TTY line discipline.

use alloc::{boxed::Box, collections::VecDeque, sync::Arc, vec::Vec};
use core::{
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    time::Duration,
};

use crate::{
    fs::{
        Error as FsError, IoctlContext, OpenFlags, PollEvents, Result as FsResult,
        devtempfs::DeviceNodeOps,
    },
    sys::{clock, sync::Mutex},
};

const INPUT_CAPACITY: usize = 4096;
const POLL_INTERVAL: Duration = Duration::from_millis(1);
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

const OPOST: u32 = 0o000001;
const OLCUC: u32 = 0o000002;
const ONLCR: u32 = 0o000004;
const OCRNL: u32 = 0o000010;

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
const ECHOCTL: u32 = 0o001000;
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

    /// Returns whether terminal state should reset after the final close.
    fn reset_on_last_close(&self) -> bool {
        false
    }
}

/// Linux-compatible terminal settings.
#[derive(Clone)]
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

#[derive(Copy, Clone, Default)]
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
    literal_next: bool,
    output_stopped: bool,
    interrupted: bool,
    exclusive: bool,
    soft_carrier: bool,
    foreground_group: i32,
    session: i32,
}

/// Character terminal layered over a platform console backend.
pub struct Tty {
    backend: Arc<dyn ConsoleBackend>,
    path: Box<str>,
    state: Mutex<TtyState>,
    lifecycle_lock: Mutex<()>,
    output_lock: Mutex<()>,
    input_flushing: AtomicBool,
    input_generation: AtomicU64,
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
        Ok(Arc::new(Self {
            backend,
            path,
            state: Mutex::new(TtyState {
                termios,
                winsize: WinSize::default(),
                input: VecDeque::with_capacity(INPUT_CAPACITY),
                literal_next: false,
                output_stopped: false,
                interrupted: false,
                exclusive: false,
                soft_carrier: true,
                foreground_group: 0,
                session: 0,
            }),
            lifecycle_lock: Mutex::new(()),
            output_lock: Mutex::new(()),
            input_flushing: AtomicBool::new(false),
            input_generation: AtomicU64::new(0),
            opens: AtomicU64::new(0),
            initial_baud: baud,
        }))
    }

    fn pump_input(&self, nonblocking_echo: bool) -> usize {
        if self.input_flushing.load(Ordering::Acquire) {
            return 0;
        }
        let mut count = 0;
        while count < 256 {
            let generation = self.input_generation.load(Ordering::Acquire);
            let Some(byte) = self.backend.try_read() else {
                break;
            };
            if self.input_flushing.load(Ordering::Acquire)
                || self.input_generation.load(Ordering::Acquire) != generation
            {
                continue;
            }
            self.process_input_generation(byte, generation, nonblocking_echo);
            count += 1;
        }
        count
    }

    fn process_input(&self, byte: u8) {
        let generation = self.input_generation.load(Ordering::Acquire);
        self.process_input_generation(byte, generation, false);
    }

    fn process_input_generation(
        &self,
        mut byte: u8,
        generation: u64,
        nonblocking_echo: bool,
    ) {
        let mut state = self.state.lock();
        if self.input_flushing.load(Ordering::Acquire)
            || self.input_generation.load(Ordering::Acquire) != generation
        {
            return;
        }
        let termios = state.termios.clone();
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
                state.output_stopped = true;
                return;
            }
            if control_matches(&termios, VSTART, byte)
                || (state.output_stopped && termios.input_flags & IXANY != 0)
            {
                state.output_stopped = false;
                if control_matches(&termios, VSTART, byte) {
                    return;
                }
            }
        }

        if state.literal_next {
            state.literal_next = false;
            self.enqueue(&mut state, InputItem::Byte(byte), nonblocking_echo);
            self.echo_input(&termios, byte, nonblocking_echo);
            return;
        }
        if termios.local_flags & IEXTEN != 0 && control_matches(&termios, VLNEXT, byte) {
            state.literal_next = true;
            if termios.local_flags & ECHO != 0 {
                self.echo_bytes(b"^", nonblocking_echo);
            }
            return;
        }

        if termios.local_flags & ISIG != 0
            && (control_matches(&termios, VINTR, byte)
                || control_matches(&termios, VQUIT, byte)
                || control_matches(&termios, VSUSP, byte))
        {
            if termios.local_flags & NOFLSH == 0 {
                state.input.clear();
            }
            state.interrupted = true;
            self.echo_control(&termios, byte, nonblocking_echo);
            self.echo_bytes(b"\r\n", nonblocking_echo);
            return;
        }

        if termios.local_flags & ICANON != 0 {
            if control_matches(&termios, VERASE, byte) {
                if erase_last_byte(&mut state.input) && termios.local_flags & ECHOE != 0 {
                    self.echo_bytes(b"\x08 \x08", nonblocking_echo);
                }
                return;
            }
            if control_matches(&termios, VKILL, byte) {
                discard_current_line(&mut state.input);
                if termios.local_flags & ECHOK != 0 {
                    self.echo_bytes(b"\r\n", nonblocking_echo);
                }
                return;
            }
            if termios.local_flags & IEXTEN != 0 && control_matches(&termios, VWERASE, byte) {
                let erased = erase_word(&mut state.input);
                if termios.local_flags & ECHOE != 0 {
                    for _ in 0..erased {
                        self.echo_bytes(b"\x08 \x08", nonblocking_echo);
                    }
                }
                return;
            }
            if termios.local_flags & IEXTEN != 0 && control_matches(&termios, VREPRINT, byte) {
                self.echo_bytes(b"^R\r\n", nonblocking_echo);
                for item in current_line(&state.input) {
                    if let InputItem::Byte(byte) = item {
                        self.echo_input(&termios, *byte, nonblocking_echo);
                    }
                }
                return;
            }
            if control_matches(&termios, VEOF, byte) {
                self.enqueue(&mut state, InputItem::Eof, nonblocking_echo);
                return;
            }
        }

        self.enqueue(&mut state, InputItem::Byte(byte), nonblocking_echo);
        if termios.local_flags & ECHO != 0 || (byte == b'\n' && termios.local_flags & ECHONL != 0) {
            self.echo_input(&termios, byte, nonblocking_echo);
        }
    }

    fn enqueue(&self, state: &mut TtyState, item: InputItem, nonblocking_echo: bool) {
        if state.input.len() >= INPUT_CAPACITY {
            let delimiter = match item {
                InputItem::Eof => true,
                InputItem::Byte(byte) => {
                    byte == b'\n'
                        || (state.termios.control[VEOL] != 0 && byte == state.termios.control[VEOL])
                        || (state.termios.control[VEOL2] != 0
                            && byte == state.termios.control[VEOL2])
                }
            };
            if delimiter && state.termios.local_flags & ICANON != 0 {
                state.input.pop_back();
                state.input.push_back(item);
                return;
            }
            if state.termios.input_flags & IMAXBEL != 0 {
                self.echo_bytes(b"\x07", nonblocking_echo);
            }
            return;
        }
        state.input.push_back(item);
    }

    fn echo_input(&self, termios: &Termios, byte: u8, nonblocking: bool) {
        if byte == b'\n' {
            self.echo_bytes(b"\r\n", nonblocking);
        } else if is_control(byte) && termios.local_flags & ECHOCTL != 0 {
            self.echo_control(termios, byte, nonblocking);
        } else {
            self.echo_bytes(core::slice::from_ref(&byte), nonblocking);
        }
    }

    fn echo_control(&self, _termios: &Termios, byte: u8, nonblocking: bool) {
        let shown = if byte == 0x7f { b'?' } else { byte ^ 0x40 };
        self.echo_bytes(&[b'^', shown], nonblocking);
    }

    fn echo_bytes(&self, bytes: &[u8], nonblocking: bool) {
        let _output = if nonblocking {
            let Some(output) = self.output_lock.try_lock() else {
                return;
            };
            output
        } else {
            self.output_lock.lock()
        };
        let _ = self.backend.write(bytes, nonblocking);
    }

    fn read_canonical(&self, buffer: &mut [u8], nonblocking: bool) -> FsResult<usize> {
        loop {
            self.pump_input(nonblocking);
            {
                let mut state = self.state.lock();
                if state.interrupted {
                    state.interrupted = false;
                    return Err(FsError::Interrupted);
                }
                if canonical_ready(&state.input, &state.termios) {
                    let mut read = 0;
                    while read < buffer.len() {
                        match state.input.pop_front() {
                            Some(InputItem::Byte(byte)) => {
                                buffer[read] = byte;
                                read += 1;
                                if byte == b'\n'
                                    || (state.termios.control[VEOL] != 0
                                        && byte == state.termios.control[VEOL])
                                    || (state.termios.control[VEOL2] != 0
                                        && byte == state.termios.control[VEOL2])
                                {
                                    break;
                                }
                            }
                            Some(InputItem::Eof) | None => break,
                        }
                    }
                    if read == buffer.len() && matches!(state.input.front(), Some(InputItem::Eof)) {
                        state.input.pop_front();
                    }
                    return Ok(read);
                }
            }
            if self.backend.hung_up() {
                return Ok(0);
            }
            if nonblocking {
                return Err(FsError::WouldBlock);
            }
            clock::sleep(POLL_INTERVAL);
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
        let mut deadline = None;

        loop {
            self.pump_input(nonblocking);
            {
                let mut state = self.state.lock();
                if state.interrupted {
                    if read != 0 {
                        return Ok(read);
                    }
                    state.interrupted = false;
                    return Err(FsError::Interrupted);
                }
                while read < buffer.len() {
                    match state.input.pop_front() {
                        Some(InputItem::Byte(byte)) => {
                            buffer[read] = byte;
                            read += 1;
                            if timeout_ns != 0 {
                                deadline = Some(clock::monotonic_ns().saturating_add(timeout_ns));
                            }
                        }
                        Some(InputItem::Eof) => {}
                        None => break,
                    }
                }
            }

            if read == buffer.len()
                || (minimum != 0 && read >= minimum)
                || (minimum == 0 && read != 0)
            {
                return Ok(read);
            }
            if read == 0 && self.backend.hung_up() {
                return Ok(0);
            }
            if nonblocking {
                return if read == 0 {
                    Err(FsError::WouldBlock)
                } else {
                    Ok(read)
                };
            }
            if minimum == 0 && timeout_ns == 0 {
                return Ok(read);
            }
            if minimum == 0 && deadline.is_none() {
                deadline = Some(clock::monotonic_ns().saturating_add(timeout_ns));
            }
            if deadline.is_some_and(|deadline| clock::monotonic_ns() >= deadline) {
                return Ok(read);
            }
            clock::sleep(POLL_INTERVAL);
        }
    }

    fn write_transformed(&self, input: &[u8], nonblocking: bool) -> FsResult<usize> {
        if self.backend.hung_up() {
            return Err(FsError::Io);
        }
        while self.state.lock().output_stopped {
            self.pump_input(nonblocking);
            if nonblocking && self.state.lock().output_stopped {
                return Err(FsError::WouldBlock);
            }
            clock::sleep(POLL_INTERVAL);
        }
        let output_flags = self.state.lock().termios.output_flags;
        let _output = if nonblocking {
            self.output_lock.try_lock().ok_or(FsError::WouldBlock)?
        } else {
            self.output_lock.lock()
        };
        if output_flags & OPOST == 0 {
            self.backend.write(input, nonblocking)?;
            return Ok(input.len());
        }

        let mut transformed = Vec::with_capacity(input.len());
        for mut byte in input.iter().copied() {
            if output_flags & OLCUC != 0 {
                byte = byte.to_ascii_uppercase();
            }
            if byte == b'\r' && output_flags & OCRNL != 0 {
                byte = b'\n';
            }
            if byte == b'\n' && output_flags & ONLCR != 0 {
                transformed.push(b'\r');
            }
            transformed.push(byte);
        }
        self.backend.write(&transformed, nonblocking)?;
        Ok(input.len())
    }

    fn apply_termios(&self, termios: Termios, drain: bool, flush_input: bool) -> FsResult<()> {
        let settings = termios.serial_settings()?;
        let mut state = self.state.lock();
        let _output = self.output_lock.lock();
        if drain {
            self.backend.flush()?;
        }
        self.backend.configure(settings)?;
        if flush_input {
            self.input_flushing.store(true, Ordering::Release);
            self.input_generation.fetch_add(1, Ordering::AcqRel);
            state.input.clear();
            for _ in 0..INPUT_CAPACITY {
                if self.backend.try_read().is_none() {
                    break;
                }
            }
            self.input_flushing.store(false, Ordering::Release);
        }
        state.termios = termios;
        Ok(())
    }

    fn flush_input(&self) {
        self.input_flushing.store(true, Ordering::Release);
        self.input_generation.fetch_add(1, Ordering::AcqRel);
        self.state.lock().input.clear();
        for _ in 0..INPUT_CAPACITY {
            if self.backend.try_read().is_none() {
                break;
            }
        }
        self.input_flushing.store(false, Ordering::Release);
    }

    fn queued_input(&self) -> usize {
        let state = self.state.lock();
        if state.termios.local_flags & ICANON == 0 {
            return state
                .input
                .iter()
                .filter(|item| matches!(item, InputItem::Byte(_)))
                .count();
        }
        let mut count = 0;
        let mut ready = 0;
        for item in &state.input {
            match item {
                InputItem::Byte(byte) => {
                    count += 1;
                    if *byte == b'\n'
                        || (state.termios.control[VEOL] != 0
                            && *byte == state.termios.control[VEOL])
                        || (state.termios.control[VEOL2] != 0
                            && *byte == state.termios.control[VEOL2])
                    {
                        ready = count;
                    }
                }
                InputItem::Eof => ready = count,
            }
        }
        ready
    }
}

impl DeviceNodeOps for Tty {
    fn open(&self, _flags: u32) -> FsResult<usize> {
        let _lifecycle = self.lifecycle_lock.lock();
        let state = self.state.lock();
        if state.exclusive && self.opens.load(Ordering::Acquire) != 0 {
            return Err(FsError::Busy);
        }
        self.backend.open()?;
        self.opens.fetch_add(1, Ordering::AcqRel);
        Ok(0)
    }

    fn close(&self, _file_context: usize, _flags: u32) {
        let _lifecycle = self.lifecycle_lock.lock();
        let previous = self.opens.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "console: TTY open count underflow");
        self.backend.close();
        if previous == 1 && self.backend.reset_on_last_close() {
            let termios = Termios::with_baud(self.initial_baud)
                .expect("console: initial baud stopped being supported");
            *self.state.lock() = TtyState {
                termios,
                winsize: WinSize::default(),
                input: VecDeque::with_capacity(INPUT_CAPACITY),
                literal_next: false,
                output_stopped: false,
                interrupted: false,
                exclusive: false,
                soft_carrier: true,
                foreground_group: 0,
                session: 0,
            };
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
        _flags: u32,
    ) -> FsResult<PollEvents> {
        self.pump_input(true);
        let state = self.state.lock();
        let mut ready = PollEvents::empty();
        let read_events = events & (PollEvents::IN | PollEvents::RDNORM);
        let readable = if state.termios.local_flags & ICANON != 0 {
            canonical_ready(&state.input, &state.termios)
        } else {
            state
                .input
                .iter()
                .any(|item| matches!(item, InputItem::Byte(_)))
        };
        if state.interrupted || readable {
            ready |= read_events;
        }
        if self.backend.hung_up() {
            ready |= PollEvents::HUP;
        }
        if !state.output_stopped && self.backend.writable() {
            ready |= events & (PollEvents::OUT | PollEvents::WRNORM);
        }
        Ok(ready)
    }

    fn sync(&self) -> FsResult<()> {
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
                if value == 0 {
                    self.backend.send_break(value)?;
                } else {
                    self.backend.flush()?;
                }
            }
            TCXONC => match value {
                0 => self.state.lock().output_stopped = true,
                1 => self.state.lock().output_stopped = false,
                2 => {
                    let byte = self.state.lock().termios.control[VSTOP];
                    self.echo_bytes(core::slice::from_ref(&byte), false);
                }
                3 => {
                    let byte = self.state.lock().termios.control[VSTART];
                    self.echo_bytes(core::slice::from_ref(&byte), false);
                }
                _ => return Err(FsError::InvalidArgument),
            },
            TCFLSH => match value {
                0 => self.flush_input(),
                1 => self.backend.flush()?,
                2 => {
                    self.flush_input();
                    self.backend.flush()?;
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
                    state.session = 0;
                    state.foreground_group = 0;
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
            TIOCOUTQ => put_i32(argument, 0)?,
            TIOCSTI => {
                let byte = *argument.first().ok_or(FsError::InvalidArgument)?;
                self.process_input(byte);
            }
            TIOCGWINSZ => self.state.lock().winsize.encode(argument)?,
            TIOCSWINSZ => self.state.lock().winsize = WinSize::decode(argument)?,
            TIOCGSOFTCAR => put_i32(argument, i32::from(self.state.lock().soft_carrier))?,
            TIOCSSOFTCAR => self.state.lock().soft_carrier = get_i32(argument)? != 0,
            FIONREAD => {
                self.pump_input(true);
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
        control[VWERASE] = 23;
        control[VLNEXT] = 22;
        Self {
            input_flags: ICRNL | IXON,
            output_flags: OPOST | ONLCR,
            control_flags: CREAD | CS8 | CLOCAL | B9600,
            local_flags: ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | IEXTEN,
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

fn canonical_ready(input: &VecDeque<InputItem>, termios: &Termios) -> bool {
    input.iter().any(|item| match item {
        InputItem::Eof | InputItem::Byte(b'\n') => true,
        InputItem::Byte(byte) => {
            (termios.control[VEOL] != 0 && *byte == termios.control[VEOL])
                || (termios.control[VEOL2] != 0 && *byte == termios.control[VEOL2])
        }
    })
}

fn discard_current_line(input: &mut VecDeque<InputItem>) {
    while let Some(item) = input.back() {
        if matches!(item, InputItem::Byte(b'\n') | InputItem::Eof) {
            break;
        }
        input.pop_back();
    }
}

fn erase_last_byte(input: &mut VecDeque<InputItem>) -> bool {
    match input.back() {
        Some(InputItem::Byte(b'\n') | InputItem::Eof) | None => false,
        Some(InputItem::Byte(_)) => {
            input.pop_back();
            true
        }
    }
}

fn erase_word(input: &mut VecDeque<InputItem>) -> usize {
    let mut erased = 0;
    while input
        .back()
        .is_some_and(|item| matches!(item, InputItem::Byte(byte) if byte.is_ascii_whitespace() && *byte != b'\n'))
    {
        input.pop_back();
        erased += 1;
    }
    while input
        .back()
        .is_some_and(|item| matches!(item, InputItem::Byte(byte) if !byte.is_ascii_whitespace()))
    {
        input.pop_back();
        erased += 1;
    }
    erased
}

fn current_line(input: &VecDeque<InputItem>) -> impl Iterator<Item = &InputItem> {
    let start = input
        .iter()
        .rposition(|item| matches!(item, InputItem::Byte(b'\n') | InputItem::Eof))
        .map_or(0, |index| index + 1);
    input.iter().skip(start)
}

fn is_control(byte: u8) -> bool {
    byte < b' ' && !matches!(byte, b'\n' | b'\t') || byte == 0x7f
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
