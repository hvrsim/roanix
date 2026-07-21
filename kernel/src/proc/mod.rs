//! Process ownership and executable startup.

mod elf;

use alloc::{sync::Arc, vec, vec::Vec};
use core::{
    fmt,
    sync::atomic::{AtomicBool, AtomicI32, AtomicUsize, Ordering},
};

use log::info;

use crate::{
    fs::FileRef,
    mem::{self, USER_ADDRESS_MAX, VmSpace},
    sys::{sched, sync::Mutex},
};

static NEXT_PID: AtomicUsize = AtomicUsize::new(1);
const MAX_FILES: usize = 1024;

/// Process creation or executable loading failure.
#[derive(Debug)]
pub enum Error {
    /// A filesystem operation failed.
    Filesystem(crate::fs::Error),
    /// A virtual memory operation failed.
    Memory(mem::Error),
    /// An ELF structure failed validation.
    InvalidElf(&'static str),
    /// The executable uses an unsupported ELF feature.
    UnsupportedElf,
    /// A required process argument was invalid.
    InvalidArgument,
    /// The process descriptor table is full.
    TooManyFiles,
}

impl From<crate::fs::Error> for Error {
    fn from(error: crate::fs::Error) -> Self {
        Self::Filesystem(error)
    }
}

impl From<mem::Error> for Error {
    fn from(error: mem::Error) -> Self {
        Self::Memory(error)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Filesystem(error) => write!(formatter, "filesystem error: {error}"),
            Self::Memory(error) => write!(formatter, "memory error: {error}"),
            Self::InvalidElf(error) => write!(formatter, "invalid ELF: {error}"),
            Self::UnsupportedElf => formatter.write_str("unsupported ELF image"),
            Self::InvalidArgument => formatter.write_str("invalid process argument"),
            Self::TooManyFiles => formatter.write_str("process file table is full"),
        }
    }
}

/// Result type used by process and executable loading operations.
pub type Result<T> = core::result::Result<T, Error>;

/// One process descriptor table entry.
#[derive(Clone)]
pub(crate) enum Descriptor {
    /// Empty console input stream.
    ConsoleInput,
    /// Architecture debug console output stream.
    ConsoleOutput,
    /// VFS-backed open file description.
    File(FileRef),
}

/// Shared resources owned by one userspace process.
pub(crate) struct Process {
    pid: usize,
    address_space: Arc<VmSpace>,
    files: Mutex<Vec<Option<Descriptor>>>,
    exited: AtomicBool,
    exit_status: AtomicI32,
}

impl Process {
    fn new(address_space: Arc<VmSpace>) -> Arc<Self> {
        Arc::new(Self {
            pid: NEXT_PID.fetch_add(1, Ordering::Relaxed),
            address_space,
            files: Mutex::new(vec![
                Some(Descriptor::ConsoleInput),
                Some(Descriptor::ConsoleOutput),
                Some(Descriptor::ConsoleOutput),
            ]),
            exited: AtomicBool::new(false),
            exit_status: AtomicI32::new(0),
        })
    }

    /// Returns this process's identifier.
    pub(crate) fn pid(&self) -> usize {
        self.pid
    }

    /// Returns the process address space.
    pub(crate) fn address_space(&self) -> Arc<VmSpace> {
        self.address_space.clone()
    }

    /// Installs an open file and returns its new descriptor number.
    pub(crate) fn install_file(&self, file: FileRef) -> Result<i32> {
        let mut files = self.files.lock();
        if let Some((index, slot)) = files
            .iter_mut()
            .enumerate()
            .skip(3)
            .find(|(_, slot)| slot.is_none())
        {
            *slot = Some(Descriptor::File(file));
            return i32::try_from(index).map_err(|_| Error::TooManyFiles);
        }
        if files.len() >= MAX_FILES {
            return Err(Error::TooManyFiles);
        }
        let fd = i32::try_from(files.len()).map_err(|_| Error::TooManyFiles)?;
        files.push(Some(Descriptor::File(file)));
        Ok(fd)
    }

    /// Returns a cloned descriptor table entry.
    pub(crate) fn descriptor(&self, fd: i32) -> Option<Descriptor> {
        let index = usize::try_from(fd).ok()?;
        self.files.lock().get(index)?.clone()
    }

    /// Removes a descriptor table entry.
    pub(crate) fn close_descriptor(&self, fd: i32) -> bool {
        let Ok(index) = usize::try_from(fd) else {
            return false;
        };
        let mut files = self.files.lock();
        let Some(slot) = files.get_mut(index) else {
            return false;
        };
        slot.take().is_some()
    }

    fn mark_exited(&self, status: i32) {
        self.exit_status.store(status, Ordering::Release);
        self.exited.store(true, Ordering::Release);
        self.files.lock().clear();
    }
}

/// Loads and schedules `/sbin/init`.
pub fn spawn_init() -> Result<usize> {
    let image = elf::load(
        "/sbin/init",
        &["/sbin/init"],
        &["PATH=/usr/bin:/bin", "HOME=/"],
    )?;
    let process = Process::new(image.address_space);
    let pid = process.pid();
    let tid = sched::run_user(process, image.entry, image.stack);
    info!("proc: started /sbin/init pid={pid} tid={tid}");
    Ok(pid)
}

/// Returns the process associated with the current thread.
pub(crate) fn current() -> Option<Arc<Process>> {
    let thread = sched::current_thread_opt()?;
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing on this CPU.
    unsafe { &*thread }.process()
}

/// Returns the current userspace thread pointer.
pub(crate) fn current_thread_pointer() -> u64 {
    let Some(thread) = sched::current_thread_opt() else {
        return 0;
    };
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing on this CPU.
    unsafe { &*thread }.thread_pointer()
}

/// Changes the current userspace thread pointer.
pub(crate) fn set_current_thread_pointer(pointer: u64) -> Result<()> {
    if pointer >= USER_ADDRESS_MAX {
        return Err(Error::InvalidArgument);
    }
    let thread = sched::current_thread_opt().ok_or(Error::InvalidArgument)?;
    // SAFETY: the scheduler keeps the current thread allocation live while it
    // is executing on this CPU.
    unsafe { &*thread }.set_thread_pointer(pointer);
    Ok(())
}

/// Terminates the current process and thread.
pub(crate) fn exit_current(status: i32) -> ! {
    if let Some(process) = current() {
        info!("proc: pid={} exited with status {status}", process.pid());
        process.mark_exited(status);
    }
    sched::exit_current()
}
