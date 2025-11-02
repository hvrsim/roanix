//!
//! # Kernel Debugging Interface
//!
//! Responsible for gathering log output from [`log`] crate.
//!
//! Stores each line of logging output into an internal ring-buffer, then
//! dispatches each character out to the arch-specific debug
//! console ([`arch::debug_putc`][crate::arch::debug_putc]).
//!
//! The kernel panic handler is also implemented in this module.
//!

use core::fmt::{self, Write};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::arch;

use limine::request::ExecutableFileRequest;
use log::Level;
use xmas_elf::{sections::*, symbol_table::*, ElfFile};

/// Connector between `log` crate and various outputs.
struct KLog;

/// Interface that lets us use [`write!`] with [`arch::debug_putc`][crate::arch::debug_putc].
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

/// Global logger instance, `log` crate invokes this.
static LOGGER: KLog = KLog;

/// Atomic flag to indicate a kernel panic is active.
static IN_PANIC: AtomicBool = AtomicBool::new(false);

impl Write for DebugWriter {
    /// Calls [`arch::debug_putc`][crate::arch::debug_putc] for each character of `s`.
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            arch::debug_putc(byte);
        }

        Ok(())
    }
}

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

        let path = if let Some(path) = record.file() {
            path
        } else {
            "???"
        };

        let line = if let Some(line) = record.line() {
            line
        } else {
            0
        };

        // A write to the debug port can never fail.
        write!(
            &mut rec,
            " \x1b[2m({}:{})\x1b[0m {}\n",
            path,
            line,
            record.args()
        )
        .unwrap();

        return rec;
    }

    /// Prints the record to the debug console.
    pub fn debug_print(&self) {
        for i in 0..self.buflen {
            arch::debug_putc(self.buf[i]);
        }
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

        let mut name = None;

        if symtab.is_some() && kfile.is_some() {
            for data in symtab.unwrap() {
                let value = data.value() as usize;
                let size = data.size() as usize;

                if rip >= value && rip < (value + size) {
                    let raw = data.get_name(kfile.unwrap()).unwrap_or("<unknown>");
                    name = Some(rustc_demangle::demangle(raw));
                }
            }
        }

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
    if IN_PANIC.load(Ordering::Acquire) == true {
        arch::hcf();
    }

    IN_PANIC.store(true, Ordering::Release);

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

    let efile = if let Some(resp) = KERNEL_FILE.get_response() {
        let file = resp.file();
        let slice = unsafe { core::slice::from_raw_parts(file.addr(), file.size() as usize) };
        let efile = ElfFile::new(slice);

        if efile.is_err() {
            None
        } else {
            Some(&efile.unwrap())
        }
    } else {
        None
    };

    let mut symtab = None;

    if let Some(elf) = efile {
        for section in elf.section_iter() {
            if section.get_type() == Ok(ShType::SymTab) {
                let section_data = section.get_data(&elf).unwrap();

                if let SectionData::SymbolTable64(st) = section_data {
                    symtab = Some(st);
                }
            }
        }
    }

    perform_bt(symtab, efile);

    arch::hcf();
}

/// Connects kernel logging infra to the log crate.
///
/// This function will panic if the kernel logger is unable to be installed,
/// since the log functions depend on a valid kernel logger.
pub fn register() {
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Trace))
        .unwrap();
}
