//!
//! # Kernel Debugging Interface
//!
//! Responsible for gathering log output from [`log`] crate.
//!
//! Stores each line of logging output into an internal ring-buffer, then
//! dispatches each character out to the arch-specific debug
//! console ([`arch::DebugConsole`][crate::arch::DebugConsole]).
//!
//! The kernel panic handler is also implemented in this module.
//!

use core::fmt::{self, Write};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::arch;

use limine::request::ExecutableFileRequest;
use log::Level;
use spin::Once;
use xmas_elf::{sections::*, symbol_table::*, ElfFile};

/// Connector between [`log`] crate and various outputs.
struct KLog;

/// Interface that lets us use [`write!`] with [`arch::DebugConsole`].
struct DebugWriter;

/// Contains data to reconstruct a single kernel log message.
struct Record {
    /// Buffer to store the formatted log message.
    buf: [u8; 256],

    /// Length of formatted log message in bytes.
    buflen: usize,

    /// Time when log message was sent (represented as UNIX epoch).
    timestamp: u64,
}

/// Helper macro for printing to the debug console.
macro_rules! dprint {
    ($($arg:tt)*) => (write!(&mut DebugWriter, "{}", format_args!($($arg)*)).unwrap());
}

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static KERNEL_FILE: ExecutableFileRequest = ExecutableFileRequest::new();

/// Global logger instance, [`log`] crate invokes this.
static LOGGER: KLog = KLog;

/// Atomic flag to indicate a kernel panic is active.
static IN_PANIC: AtomicBool = AtomicBool::new(false);

/// Instance of architecture specific debug console
static DBGCON: Once<arch::DebugConsole> = Once::new();

impl Record {
    /// Creates a log Record using metadata from the [`log`] crate.
    pub fn new(record: &log::Record) -> Self {
        let mut rec = Self {
            buf: [0; 256],
            buflen: 0,
            timestamp: 0,
        };

        match record.level() {
            Level::Error => write!(&mut rec, "[\x1b[1;31mE\x1b[0m]").unwrap(),
            Level::Warn => write!(&mut rec, "[\x1b[1;33m!\x1b[0m]").unwrap(),
            Level::Info => write!(&mut rec, "[\x1b[1;32m*\x1b[0m]").unwrap(),
            Level::Debug => write!(&mut rec, "[\x1b[1;34mD\x1b[0m]").unwrap(),
            Level::Trace => write!(&mut rec, "[\x1b[1;35mT\x1b[0m]").unwrap(),
        }

        let path = record.file().map_or("???", |f| f);
        let line = record.line().map_or(0, |l| l);

        // A write to the debug port can never fail.
        write!(
            &mut rec,
            " \x1b[2m({}:{})\x1b[0m {}\n",
            path,
            line,
            record.args()
        )
        .unwrap();

        rec
    }

    /// Prints the record to the debug console.
    pub fn debug_print(&self) {
        for i in 0..self.buflen {
            DBGCON.get().unwrap().write(self.buf[i]);
        }
    }
}

impl Write for DebugWriter {
    /// Calls [`arch::DebugConsole::write`] for each character of `s`.
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            DBGCON.get().unwrap().write(byte);
        }

        Ok(())
    }
}
impl Write for Record {
    /// Copy the string `s` into the record's buffer, silently
    /// dropping bytes on overflow.
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        if self.buflen + bytes.len() > self.buf.len() {
            return Ok(());
        }

        self.buf[self.buflen..self.buflen + bytes.len()].copy_from_slice(bytes);
        self.buflen += bytes.len();
        Ok(())
    }
}

impl log::Log for KLog {
    /// Unused, since we accept all messages regardless of level.
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    /// Pretty-prints `record` and sends it through the logging backend.
    fn log(&self, record: &log::Record) {
        let rec = Record::new(record);
        rec.debug_print();
    }

    /// Flush calls are done on a per-character basis, therefore global flush not required.
    fn flush(&self) {}
}

/// x86_64-specific unwind function.
///
/// Iterates backwards through the stack, printing the symbol at each level.
/// *Max backtrace depth is currently set to 32 function calls.*
#[cfg(target_arch = "x86_64")]
fn perform_bt(symtab: Option<&[Entry64]>, kfile: Option<&ElfFile>) {
    let mut rbp: usize;

    unsafe {
        core::arch::asm!("mov {}, rbp", out(reg) rbp);
    }

    if rbp == 0 {
        return;
    }

    dprint!("\n<{:-^40}>\n\n", " BACKTRACE ");

    for depth in 0..32 {
        let rip = if let Some(r) = rbp.checked_add(core::mem::size_of::<usize>()) {
            unsafe { *(r as *const usize) }
        } else {
            0
        };

        if rip == 0 {
            break;
        }

        unsafe {
            rbp = *(rbp as *const usize);
        }

        let name = if let (Some(symtab), Some(kfile)) = (symtab, kfile) {
            symtab
                .iter()
                .find(|data| {
                    let value = data.value() as usize;
                    let size = data.size() as usize;
                    rip >= value && rip < value.saturating_add(size)
                })
                .and_then(|data| data.get_name(kfile).ok())
                .map(|raw| rustc_demangle::demangle(raw))
        } else {
            None
        };

        if let Some(name) = name {
            dprint!("{:>2}: 0x{:016x} - {:#}\n", depth, rip, name);
        } else {
            dprint!("{depth:>2}: 0x{rip:016x} - <unknown>\n");
        }
    }
}

/// riscv64-specific unwind function.
///
/// Iterates backwards through the stack, printing the symbol at each level.
/// *Max backtrace depth is currently set to 32 function calls.*
#[cfg(target_arch = "riscv64")]
fn perform_bt(symtab: Option<&[Entry64]>, kfile: Option<&ElfFile>) {
    let mut fp: usize;

    unsafe {
        core::arch::asm!("mv {}, fp", out(reg) fp);
    }

    if fp == 0 {
        return;
    }

    dprint!("\n<{:-^40}>\n\n", " BACKTRACE ");

    for depth in 0..32 {
        let rip = if let Some(r) = fp.checked_sub(core::mem::size_of::<usize>()) {
            unsafe { *(r as *const usize) }
        } else {
            0
        };

        if rip == 0 {
            break;
        }

        unsafe {
            let prev_fp_addr = if let Some(addr) = fp.checked_sub(2 * core::mem::size_of::<usize>())
            {
                addr
            } else {
                break;
            };
            fp = *(prev_fp_addr as *const usize);
        }

        let name = if let (Some(symtab), Some(kfile)) = (symtab, kfile) {
            symtab
                .iter()
                .find(|data| {
                    let value = data.value() as usize;
                    let size = data.size() as usize;
                    rip >= value && rip < value.saturating_add(size)
                })
                .and_then(|data| data.get_name(kfile).ok())
                .map(|raw| rustc_demangle::demangle(raw))
        } else {
            None
        };

        if let Some(name) = name {
            dprint!("{:>2}: 0x{:016x} - {:#}\n", depth, rip, name);
        } else {
            dprint!("{depth:>2}: 0x{rip:016x} - <unknown>\n");
        }
    }
}

#[doc(hidden)]
#[panic_handler]
fn rust_panic(info: &PanicInfo) -> ! {
    if IN_PANIC.swap(true, Ordering::Acquire) {
        arch::wfi();
    }

    // setup the dbgcon here incase it wasn't prepared before
    DBGCON.call_once(|| arch::DebugConsole::new());

    dprint!("\n  _________________________  \n");
    dprint!("< uh oh, kernel panicked... >\n");
    dprint!("  -------------------------  \n");
    dprint!("          \\   ^__^          \n");
    dprint!("           \\  (oo)\\_______  \n");
    dprint!("              (__)\\       )\\/\\\\\n");
    dprint!("                  ||----w |  \n");
    dprint!("                  ||     ||  \n\n\n");
    dprint!("\x1b[31m{}\x1b[0m\n", info.message());

    if let Some(loc) = info.location() {
        dprint!(
            "panic occurred in file '{}' at line {}.\n",
            loc.file(),
            loc.line()
        );
    }

    if let Some(resp) = KERNEL_FILE.get_response() {
        let file = resp.file();
        let slice = unsafe { core::slice::from_raw_parts(file.addr(), file.size() as usize) };
        if let Ok(efile) = ElfFile::new(slice) {
            let symtab = efile
                .section_iter()
                .find(|s| s.get_type() == Ok(ShType::SymTab))
                .and_then(|s| s.get_data(&efile).ok())
                .and_then(|d| match d {
                    SectionData::SymbolTable64(st) => Some(st),
                    _ => None,
                });

            perform_bt(symtab, Some(&efile));
        } else {
            perform_bt(None, None);
        }
    } else {
        perform_bt(None, None);
    }

    arch::wfi();
}

/// Connects kernel logging infra to the log crate.
///
/// This function will panic if the kernel logger is unable to be installed,
/// since the log functions depend on a valid kernel logger.
pub fn register() {
    DBGCON.call_once(|| arch::DebugConsole::new());

    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
}
