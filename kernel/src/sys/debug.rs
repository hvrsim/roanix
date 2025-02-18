//!
//! # Kernel Debugging Interface
//!
//! Responsible for gathering log output from [`log::info!`], [`log::warn!`], [`log::trace!`] and the likes.
//!
//! Stores each line of logging output into an internal buffer, then dispatches
//! each character out to the arch-specific debug console ([`arch::debug_putc`][crate::arch::debug_putc]).
//!
//! The kernel panic handler is also implemented in this module.
//!

// static mut buffer is protected with mutex.
#![allow(static_mut_refs)]

use core::fmt::{Result, Write};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

use limine::request::ExecutableFileRequest;
use log::{Level, LevelFilter, Metadata, Record};
use spin::{Mutex, Once};
use xmas_elf::{
    sections::SectionData, sections::ShType, symbol_table::Entry, symbol_table::Entry64, ElfFile,
};

/// Connector between `log` crate and various outputs.
struct KLog;

/// Interface that lets us use [`write!`] with [`arch::debug_putc`][crate::arch::debug_putc].
struct PanicWriter;

/// Internal buffer of raw log output, circles back once filled.
struct RingBuffer<const N: usize> {
    data: [u8; N],
    read: usize,
    write: usize,
}

/// Wrapper struct for [`ElfFile`]('ElfFile'), used for parsing the kernel symbol table.
struct KernelElf {
    pub file: ElfFile<'static>,
}

/// Helper macro for printing with the panic handler.
macro_rules! eprint {
    ($($arg:tt)*) => (write!(&mut PanicWriter, "{}", format_args!($($arg)*)).unwrap());
}

/// Number of characters in the ringbuffer.
const RING_ENTRIES: usize = 4096;

/// Global logger instance, `log` crate invokes this.
static LOGGER: KLog = KLog;

/// Atomic flag to indicate a kernel panic is active.
static IN_PANIC: AtomicBool = AtomicBool::new(false);

/// Instance of kernel file parser for panic unwinding.
static KERNEL_ELF: Once<KernelElf> = Once::new();

/// Global buffer instance, protected with mutex for SMP contexts.
static BUFFER: Mutex<RingBuffer<RING_ENTRIES>> = Mutex::new(RingBuffer {
    data: [0; RING_ENTRIES],
    read: 0,
    write: 0,
});

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static KERNEL_FILE: ExecutableFileRequest = ExecutableFileRequest::new();

impl KernelElf {
    fn new(elf: ElfFile<'static>) -> Self {
        Self { file: elf }
    }
}

impl Write for PanicWriter {
    /// Calls [`arch::debug_putc`][crate::arch::debug_putc] for each character of `s`.
    fn write_str(&mut self, s: &str) -> Result {
        for byte in s.bytes() {
            crate::arch::debug_putc(byte);
        }

        Ok(())
    }
}

impl<const T: usize> Write for RingBuffer<T> {
    /// Copies the string into the ringbuffer and passes it to [`PanicWriter`].
    fn write_str(&mut self, s: &str) -> Result {
        for byte in s.bytes() {
            self.data[self.write] = byte;

            let next = (self.write + 1) % RING_ENTRIES;

            // If we have filled up the write allocation, bump the read pointer.
            if next == self.read {
                self.read = (self.read + 1) % RING_ENTRIES;
            }

            crate::arch::debug_putc(byte);

            self.write = next;
        }

        Ok(())
    }
}

impl log::Log for KLog {
    /// Unused, since we accept all messages regardless of level.
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    /// Pretty-prints `record` and sends it through the backend.
    fn log(&self, record: &Record) {
        let mut buffer = BUFFER.lock();

        match record.level() {
            Level::Error => write!(&mut buffer, "[\x1b[1;31mE\x1b[0m]").unwrap(),
            Level::Warn => write!(&mut buffer, "[\x1b[1;33m!\x1b[0m]").unwrap(),
            Level::Info => write!(&mut buffer, "[\x1b[1;32m*\x1b[0m]").unwrap(),
            Level::Debug => write!(&mut buffer, "[\x1b[1;34mD\x1b[0m]").unwrap(),
            Level::Trace => write!(&mut buffer, "[\x1b[1;35mT\x1b[0m]").unwrap(),
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
            &mut buffer,
            " \x1b[2m({}:{})\x1b[0m {}\n",
            path,
            line,
            record.args()
        )
        .unwrap();
    }

    /// Flush calls are done on a per-character basis, therefore global flush not required.
    fn flush(&self) {}
}

/// Connects kernel logging infra to the log crate. Also parses the kernel ELF for panic unwinding.
pub fn register() {
    let kfile_resp = KERNEL_FILE
        .get_response()
        .expect("debug: limine kernel file response missing!");

    KERNEL_ELF.call_once(|| {
        let file = kfile_resp.file();

        let slice = unsafe { core::slice::from_raw_parts(file.addr(), file.size() as usize) };
        let elf = ElfFile::new(slice).expect("debug: unable to parse kernel file!");

        KernelElf::new(elf)
    });

    // Kernel logging depends on a valid logger, therefore panic if we are unable to install this logger.
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Trace))
        .unwrap();
}

/// Target-specific unwind function.
///
/// Iterates backwards through the stack, printing the symbol at each level.
/// *Max backtrace depth is currently set to 32 function calls.*
#[cfg(target_arch = "x86_64")]
fn perform_bt(symtab: &[Entry64], kfile: &ElfFile) {
    let mut rbp: usize;

    unsafe {
        core::arch::asm!("mov {}, rbp", out(reg) rbp);
    }

    if rbp == 0 {
        return;
    }

    eprint!("\n<{:-^40}>\n\n", " BACKTRACE ");

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

        for data in symtab {
            let value = data.value() as usize;
            let size = data.size() as usize;

            if rip >= value && rip < (value + size) {
                let raw = data.get_name(kfile).unwrap_or("<unknown>");
                name = Some(rustc_demangle::demangle(raw));
            }
        }

        if let Some(name) = name {
            eprint!("{:>2}: 0x{:016x} - {:#}\n", depth, rip, name);
        } else {
            eprint!("{depth:>2}: 0x{rip:016x} - <unknown>\n");
        }
    }
}

#[doc(hidden)]
#[panic_handler]
fn rust_panic(info: &PanicInfo) -> ! {
    if IN_PANIC.load(Ordering::Acquire) == true {
        crate::arch::hcf();
    }

    IN_PANIC.store(true, Ordering::Release);

    eprint!("\n  _________________________  \n");
    eprint!("< uh oh, kernel panicked... >\n");
    eprint!("  -------------------------  \n");
    eprint!("          \\   ^__^          \n");
    eprint!("           \\  (oo)\\_______  \n");
    eprint!("              (__)\\       )\\/\\\\\n");
    eprint!("                  ||----w |  \n");
    eprint!("                  ||     ||  \n\n\n");
    eprint!("\x1b[31m{}\x1b[0m\n", info.message());

    if let Some(loc) = info.location() {
        eprint!(
            "panic occurred in file '{}' at line {}.\n",
            loc.file(),
            loc.line()
        );
    }

    let elf_file = &KERNEL_ELF.get().unwrap().file;
    let mut symtab = None;

    for section in elf_file.section_iter() {
        if section.get_type() == Ok(ShType::SymTab) {
            let section_data = section.get_data(elf_file).unwrap();

            if let SectionData::SymbolTable64(st) = section_data {
                symtab = Some(st);
            }
        }
    }

    let symtab = symtab.unwrap();
    perform_bt(symtab, elf_file);

    crate::arch::hcf();
}
