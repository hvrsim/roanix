//!
//! # Kernel Panic Handling
//!
//! Captures panic details and writes them directly to registered log sinks.
//!

use core::fmt::{self, Write};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

use limine::request::ExecutableFileRequest;
use xmas_elf::{sections::*, symbol_table::*, ElfFile};

use crate::{arch, sys::debug};

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static KERNEL_FILE: ExecutableFileRequest = ExecutableFileRequest::new();

/// Atomic flag to indicate a kernel panic is active.
static IN_PANIC: AtomicBool = AtomicBool::new(false);

/// Fixed-size formatter for panic log lines.
struct PanicWriter {
    buf: [u8; 256],
    buflen: usize,
}

impl PanicWriter {
    /// Create a new `PanicWriter` with an empty internal buffer.
    #[inline(always)]
    const fn new() -> Self {
        Self {
            buf: [0; 256],
            buflen: 0,
        }
    }
}

impl Write for PanicWriter {
    /// Copy `s` into the internal buffer, truncating on overflow.
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let space = self.buf.len().saturating_sub(self.buflen);
        let count = bytes.len().min(space);

        self.buf[self.buflen..self.buflen + count].copy_from_slice(&bytes[..count]);
        self.buflen += count;

        Ok(())
    }
}

/// Helper function to bridge `plog!` to `debug::write_to_sinks`
#[inline(always)]
fn panic_log(args: core::fmt::Arguments) {
    let mut writer = PanicWriter::new();
    write!(&mut writer, "{}", args).ok();
    debug::write_to_sinks(writer.buf.as_ptr(), writer.buflen);
}

macro_rules! plog {
    ($($arg:tt)*) => {
        panic_log(format_args!($($arg)*))
    };
}

/// x86_64-specific unwind function.
///
/// Iterates backwards through the stack, logging the symbol at each level.
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

    plog!("BACKTRACE:");

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
            plog!("{:>2}: 0x{:016x} - {:#}", depth, rip, name);
        } else {
            plog!("{depth:>2}: 0x{rip:016x} - <unknown>");
        }
    }
}

/// riscv64-specific unwind function.
///
/// Iterates backwards through the stack, logging the symbol at each level.
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

    plog!("BACKTRACE:");

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
            plog!("{:>2}: 0x{:016x} - {:#}", depth, rip, name);
        } else {
            plog!("{depth:>2}: 0x{rip:016x} - <unknown>");
        }
    }
}

#[inline(always)]
fn panic_loop() -> ! {
    arch::irqset(false);

    loop {
        core::hint::spin_loop();
    }
}

#[doc(hidden)]
#[panic_handler]
fn rust_panic(info: &PanicInfo) -> ! {
    if IN_PANIC.swap(true, Ordering::Acquire) {
        panic_loop();
    }

    plog!("\n  _________________________  ");
    plog!("< uh oh, kernel panicked... >");
    plog!("  -------------------------  ");
    plog!("          \\   ^__^          ");
    plog!("           \\  (oo)\\_______  ");
    plog!("              (__)\\       )\\/\\\\");
    plog!("                  ||----w |  ");
    plog!("                  ||     ||  \n\n");
    plog!("\x1b[31m{}\x1b[0m", info.message());

    if let Some(loc) = info.location() {
        plog!(
            "panic occurred in file '{}' at line {}.",
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

    panic_loop()
}
