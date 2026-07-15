//!
//! # Kernel Panic Handling
//!
//! Captures panic details and writes them directly to registered log sinks.
//!

use core::fmt::{self, Write};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use limine::request::ExecutableFileRequest;
use xmas_elf::{ElfFile, sections::*, symbol_table::*};

use crate::{
    arch,
    sys::{debug, fbcon, smp},
};

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static KERNEL_FILE: ExecutableFileRequest = ExecutableFileRequest::new();

/// Atomic flag to indicate a kernel panic is active.
static IN_PANIC: AtomicBool = AtomicBool::new(false);
static PANIC_OWNER: AtomicUsize = AtomicUsize::new(usize::MAX);
static PANIC_STOPPED_CPUS: AtomicUsize = AtomicUsize::new(0);
const BACKTRACE_STACK_WINDOW: usize = 256 * 1024;
const PANIC_SHOOTDOWN_SPINS: usize = 1_000_000;

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
    let rsp: usize;

    // SAFETY: these instructions only snapshot the current frame and stack
    // pointers into declared output registers.
    unsafe {
        core::arch::asm!("mov {}, rbp", out(reg) rbp);
        core::arch::asm!("mov {}, rsp", out(reg) rsp);
    }

    if !frame_ptr_in_bounds(rbp, rsp) {
        return;
    }

    plog!("BACKTRACE:");

    for depth in 0..32 {
        let next_rbp_addr = rbp;
        let rip_addr = match rbp.checked_add(core::mem::size_of::<usize>()) {
            Some(addr) if frame_ptr_in_bounds(addr, rsp) => addr,
            _ => break,
        };

        // SAFETY: both addresses passed `frame_ptr_in_bounds` for the current
        // stack window and are naturally aligned frame slots.
        let next_rbp = unsafe { *(next_rbp_addr as *const usize) };
        // SAFETY: `rip_addr` was separately bounds-checked above.
        let rip = unsafe { *(rip_addr as *const usize) };

        if rip == 0 {
            break;
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

        if !frame_ptr_in_bounds(next_rbp, rsp) || next_rbp <= rbp {
            break;
        }
        rbp = next_rbp;
    }
}

/// riscv64-specific unwind function.
///
/// Iterates backwards through the stack, logging the symbol at each level.
/// *Max backtrace depth is currently set to 32 function calls.*
#[cfg(target_arch = "riscv64")]
fn perform_bt(symtab: Option<&[Entry64]>, kfile: Option<&ElfFile>) {
    let mut fp: usize;
    let sp: usize;

    // SAFETY: these instructions only snapshot the current frame and stack
    // pointers into declared output registers.
    unsafe {
        core::arch::asm!("mv {}, fp", out(reg) fp);
        core::arch::asm!("mv {}, sp", out(reg) sp);
    }

    if !frame_ptr_in_bounds(fp, sp) {
        return;
    }

    plog!("BACKTRACE:");

    for depth in 0..32 {
        let rip_addr = match fp.checked_sub(core::mem::size_of::<usize>()) {
            Some(addr) if frame_ptr_in_bounds(addr, sp) => addr,
            _ => break,
        };
        let prev_fp_addr = match fp.checked_sub(2 * core::mem::size_of::<usize>()) {
            Some(addr) if frame_ptr_in_bounds(addr, sp) => addr,
            _ => break,
        };
        // SAFETY: `rip_addr` passed the current stack-window bounds check.
        let rip = unsafe { *(rip_addr as *const usize) };

        if rip == 0 {
            break;
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

        // SAFETY: `prev_fp_addr` passed the current stack-window bounds check.
        let next_fp = unsafe { *(prev_fp_addr as *const usize) };
        if !frame_ptr_in_bounds(next_fp, sp) || next_fp <= fp {
            break;
        }
        fp = next_fp;
    }
}

#[inline(always)]
fn frame_ptr_in_bounds(ptr: usize, stack_ptr: usize) -> bool {
    ptr != 0
        && ptr.is_multiple_of(core::mem::align_of::<usize>())
        && ptr >= stack_ptr
        && ptr.saturating_sub(stack_ptr) < BACKTRACE_STACK_WINDOW
}

#[inline(always)]
fn panic_loop() -> ! {
    arch::irqset(false);

    loop {
        core::hint::spin_loop();
    }
}

pub(crate) fn halt_if_panicking() {
    if !IN_PANIC.load(Ordering::Acquire) {
        return;
    }

    let cpu_id = arch::thiscpu_opt().map(|cpu| cpu.id).unwrap_or(0);
    if cpu_id == PANIC_OWNER.load(Ordering::Acquire) {
        return;
    }

    PANIC_STOPPED_CPUS.fetch_add(1, Ordering::AcqRel);
    panic_loop();
}

fn shoot_down_other_cpus(owner: usize) {
    let total = smp::cpu_count();
    if total <= 1 {
        return;
    }

    for cpu_id in 0..total {
        if cpu_id != owner {
            let _ = smp::send_ipi(halt_if_panicking, smp::IpiTarget::Single(cpu_id));
        }
    }
}

fn wait_for_other_cpus() {
    let expected = smp::online_cpus().saturating_sub(1);
    for _ in 0..PANIC_SHOOTDOWN_SPINS {
        if PANIC_STOPPED_CPUS.load(Ordering::Acquire) >= expected {
            break;
        }

        core::hint::spin_loop();
    }
}

fn prepare_panic_output() {
    let owner = arch::thiscpu_opt().map(|cpu| cpu.id).unwrap_or(0);
    PANIC_OWNER.store(owner, Ordering::Release);
    PANIC_STOPPED_CPUS.store(0, Ordering::Release);

    debug::enter_panic_mode();
    shoot_down_other_cpus(owner);
    wait_for_other_cpus();

    // SAFETY: every other CPU has stopped or been abandoned, so stale lock
    // owners cannot resume after panic recovery force-unlocks them.
    unsafe {
        debug::force_unlock_for_panic();
        fbcon::force_unlock_for_panic();
    }
}

#[doc(hidden)]
#[panic_handler]
fn rust_panic(info: &PanicInfo) -> ! {
    if IN_PANIC.swap(true, Ordering::AcqRel) {
        halt_if_panicking();
        panic_loop();
    }

    prepare_panic_output();

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
        // SAFETY: Limine keeps the kernel image mapped and reports its exact
        // byte length for the kernel lifetime.
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
