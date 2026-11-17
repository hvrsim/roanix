//! Process ownership and executable startup.

pub(crate) mod cgroup;
mod descriptor_life;
mod elf;
pub(crate) mod epoll;
pub(crate) mod eventfd;
pub(crate) mod inotify;
mod pipe;
pub(crate) mod signal;
pub(crate) mod syscall;
pub(crate) mod timerfd;
pub(crate) mod unix;

use crate::sys::event::EventSource;
use descriptor_life::{DescriptorLife, OpenHandle};

use alloc::{
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    fmt,
    sync::atomic::{AtomicBool, AtomicI8, AtomicI32, AtomicU8, AtomicU16, AtomicUsize, Ordering},
};

use log::{debug, info, trace};

use crate::{
    arch::cpu::TrapFrame,
    fs::{FileRef, IoctlContext, PathAnchor, PollEvents, VnodeKey, VnodeKind},
    mem::{self, USER_ADDRESS_MAX, USER_ADDRESS_MIN, VmSpace},
    sys::{
        event::{Event, EventListener},
        sched,
        smp::IrqSpinLock,
        sync::{Mutex, Once},
    },
};

pub(crate) use epoll::Epoll;
pub(crate) use eventfd::EventFd;
pub(crate) use inotify::Inotify;
pub(crate) use pipe::{PipeEnd, PipeError};
pub(crate) use signal::SignalFd;
pub(crate) use timerfd::TimerFd;

static NEXT_PID: AtomicUsize = AtomicUsize::new(1);
static PROCESSES: Once<Mutex<ProcessRegistry>> = Once::new();
pub(crate) const MAX_FILES: usize = 1024;

/// Permission bits cleared from newly created filesystem objects by default.
const DEFAULT_UMASK: u16 = 0o022;

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
    /// A descriptor number does not name an open descriptor.
    BadFileDescriptor,
    /// The selected process does not exist.
    NoSuchProcess,
    /// The caller has no matching child process.
    NoChild,
    /// A pending signal interrupted the operation.
    Interrupted,
    /// The requested process operation is not permitted.
    PermissionDenied,
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
            Self::BadFileDescriptor => formatter.write_str("bad file descriptor"),
            Self::NoSuchProcess => formatter.write_str("process does not exist"),
            Self::NoChild => formatter.write_str("no matching child process"),
            Self::Interrupted => formatter.write_str("process operation interrupted"),
            Self::PermissionDenied => formatter.write_str("process operation is not permitted"),
        }
    }
}

/// Result type used by process and executable loading operations.
pub type Result<T> = core::result::Result<T, Error>;

/// One process descriptor table entry.
#[derive(Clone)]
pub(crate) enum Descriptor {
    /// VFS-backed open file description.
    File(FileRef),
    /// Anonymous pipe endpoint.
    Pipe(Arc<PipeEnd>),
    /// Pending-signal stream.
    SignalFd(Arc<SignalFd>),
    /// Set of descriptor readiness watches.
    Epoll(Arc<Epoll>),
    /// Filesystem notification queue.
    Inotify(Arc<Inotify>),
    /// Clock expiration counter.
    TimerFd(Arc<TimerFd>),
    /// Userspace notification counter.
    EventFd(Arc<EventFd>),
    /// Local socket endpoint.
    Unix(Arc<unix::UnixSocket>),
}

/// Stable identity of one open descriptor description.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub(crate) struct DescriptorKey {
    kind: u8,
    pointer: usize,
}

impl Descriptor {
    pub(crate) fn key(&self) -> DescriptorKey {
        match self {
            Self::Unix(value) => DescriptorKey {
                kind: 7,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::File(value) => DescriptorKey {
                kind: 0,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::Pipe(value) => DescriptorKey {
                kind: 1,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::SignalFd(value) => DescriptorKey {
                kind: 2,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::Epoll(value) => DescriptorKey {
                kind: 3,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::Inotify(value) => DescriptorKey {
                kind: 4,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::TimerFd(value) => DescriptorKey {
                kind: 5,
                pointer: Arc::as_ptr(value) as usize,
            },
            Self::EventFd(value) => DescriptorKey {
                kind: 6,
                pointer: Arc::as_ptr(value) as usize,
            },
        }
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        match self {
            Self::Unix(socket) => socket.poll(requested),
            Self::File(file) => file.poll(requested).unwrap_or(PollEvents::ERR),
            Self::Pipe(pipe) => pipe.poll(requested),
            Self::SignalFd(signal_fd) => signal_fd.poll(requested),
            Self::Epoll(epoll) => epoll.poll(requested),
            Self::Inotify(inotify) => inotify.poll(requested),
            Self::TimerFd(timer) => timer.poll(requested),
            Self::EventFd(counter) => counter.poll(requested),
        }
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        requested: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        match self {
            Self::Unix(socket) => socket.poll_events(requested, output),
            Self::File(file) => file.poll_events(requested, output),
            Self::Pipe(pipe) => pipe.poll_events(requested, output),
            Self::SignalFd(signal_fd) => signal_fd.poll_events(requested, output),
            Self::Epoll(_) => false,
            Self::TimerFd(timer) => timer.poll_events(requested, output),
            Self::Inotify(inotify) => inotify.poll_events(requested, output),
            Self::EventFd(counter) => counter.poll_events(requested, output),
        }
    }
}

#[derive(Clone)]
struct DescriptorEntry {
    handle: OpenHandle,
    descriptor: Descriptor,
    close_on_exec: bool,
}

#[derive(Clone)]
struct WorkingDirectory {
    path: Vec<u8>,
    anchor: PathAnchor,
}

struct ProcessRegistry {
    processes: BTreeMap<usize, Weak<Process>>,
    init: Weak<Process>,
}

#[derive(Clone, Copy)]
pub(crate) struct DescriptorLimit {
    pub(crate) soft: usize,
    pub(crate) hard: usize,
}

#[derive(Clone)]
struct DescriptorTable {
    slots: Vec<Option<DescriptorEntry>>,
    limit: DescriptorLimit,
}

impl DescriptorTable {
    fn available(&self, minimum: usize) -> impl Iterator<Item = usize> + '_ {
        (minimum..self.limit.soft).filter(|&fd| self.slots.get(fd).is_none_or(Option::is_none))
    }

    fn insert(&mut self, index: usize, entry: DescriptorEntry) {
        if self.slots.len() <= index {
            self.slots.resize_with(index + 1, || None);
        }
        self.slots[index] = Some(entry);
    }
}

/// Shared resources owned by one userspace process.
pub(crate) struct Process {
    pid: usize,
    address_space: IrqSpinLock<Arc<VmSpace>>,
    executable: Mutex<Option<FileRef>>,
    files: Mutex<DescriptorTable>,
    cwd: Mutex<WorkingDirectory>,
    parent: Mutex<Weak<Process>>,
    parent_death_signal: AtomicU8,
    children: Mutex<BTreeMap<usize, Arc<Process>>>,
    child_event: Event,
    session: AtomicUsize,
    process_group: AtomicUsize,
    controlling_tty: Mutex<Option<FileRef>>,
    signals: signal::SignalManager,
    umask: AtomicU16,
    nice: AtomicI8,
    active_threads: AtomicUsize,
    main_thread: AtomicUsize,
    exited: AtomicBool,
    exit_status: AtomicI32,
    wait_status: AtomicI32,
}

impl ProcessRegistry {
    fn new() -> Self {
        Self {
            processes: BTreeMap::new(),
            init: Weak::new(),
        }
    }
}

impl Process {
    fn new_root(address_space: Arc<VmSpace>, executable: FileRef) -> Arc<Self> {
        let pid = NEXT_PID.fetch_add(1, Ordering::Relaxed);
        let cwd = WorkingDirectory {
            path: b"/".to_vec(),
            anchor: crate::fs::root_anchor().expect("proc: root filesystem unavailable"),
        };
        let process = Arc::new(Self {
            pid,
            address_space: IrqSpinLock::new(address_space),
            executable: Mutex::new(Some(executable)),
            files: Mutex::new(DescriptorTable {
                slots: Vec::new(),
                limit: DescriptorLimit {
                    soft: MAX_FILES,
                    hard: MAX_FILES,
                },
            }),
            cwd: Mutex::new(cwd),
            parent: Mutex::new(Weak::new()),
            parent_death_signal: AtomicU8::new(0),
            children: Mutex::new(BTreeMap::new()),
            child_event: Event::new(),
            session: AtomicUsize::new(pid),
            process_group: AtomicUsize::new(pid),
            controlling_tty: Mutex::new(None),
            signals: signal::SignalManager::new(),
            umask: AtomicU16::new(DEFAULT_UMASK),
            nice: AtomicI8::new(0),
            active_threads: AtomicUsize::new(0),
            main_thread: AtomicUsize::new(0),
            exited: AtomicBool::new(false),
            exit_status: AtomicI32::new(0),
            wait_status: AtomicI32::new(0),
        });
        cgroup::register(&process, None).expect("proc: register root cgroup");
        let mut registry = process_registry().lock();
        registry.init = Arc::downgrade(&process);
        registry.processes.insert(pid, Arc::downgrade(&process));
        process
    }

    fn fork_from(parent: &Arc<Self>, address_space: Arc<VmSpace>) -> Result<Arc<Self>> {
        let pid = NEXT_PID.fetch_add(1, Ordering::Relaxed);
        let process = Arc::new(Self {
            pid,
            address_space: IrqSpinLock::new(address_space),
            executable: Mutex::new(parent.executable()),
            files: Mutex::new(parent.files.lock().clone()),
            cwd: Mutex::new(parent.cwd.lock().clone()),
            parent: Mutex::new(Arc::downgrade(parent)),
            parent_death_signal: AtomicU8::new(0),
            children: Mutex::new(BTreeMap::new()),
            child_event: Event::new(),
            session: AtomicUsize::new(parent.session()),
            process_group: AtomicUsize::new(parent.process_group()),
            controlling_tty: Mutex::new(parent.controlling_tty.lock().clone()),
            signals: signal::SignalManager::fork_from(&parent.signals),
            umask: AtomicU16::new(parent.umask()),
            nice: AtomicI8::new(parent.nice()),
            active_threads: AtomicUsize::new(0),
            main_thread: AtomicUsize::new(0),
            exited: AtomicBool::new(false),
            exit_status: AtomicI32::new(0),
            wait_status: AtomicI32::new(0),
        });
        cgroup::register(&process, Some(parent.pid()))?;
        parent.children.lock().insert(pid, process.clone());
        process_registry()
            .lock()
            .processes
            .insert(pid, Arc::downgrade(&process));
        Ok(process)
    }

    /// Returns this process's scheduler nice value.
    pub(crate) fn nice(&self) -> i8 {
        self.nice.load(Ordering::Relaxed)
    }

    /// Records a new scheduler nice value for this process.
    pub(crate) fn set_nice(&self, nice: i8) {
        self.nice.store(nice, Ordering::Relaxed);
    }

    /// Returns this process's identifier.
    pub(crate) fn pid(&self) -> usize {
        self.pid
    }

    /// Returns the current parent, or zero when there is none.
    pub(crate) fn parent_pid(&self) -> usize {
        self.parent
            .lock()
            .upgrade()
            .map_or(0, |parent| parent.pid())
    }

    /// Changes the signal sent when this process's parent exits. Fork clears
    /// the setting, while exec preserves it under the native fixed-root model.
    pub(crate) fn set_parent_death_signal(&self, signal: u8) {
        // Serialize with adoption so a setting applies either before or after
        // the parent changes. Signal delivery happens outside this lock.
        let _parent = self.parent.lock();
        self.parent_death_signal.store(signal, Ordering::Relaxed);
    }

    pub(crate) fn parent_death_signal(&self) -> u8 {
        self.parent_death_signal.load(Ordering::Relaxed)
    }

    /// Snapshots occupied descriptor numbers without creating descriptor owners.
    pub(crate) fn descriptor_numbers(&self) -> Vec<i32> {
        self.files
            .lock()
            .slots
            .iter()
            .enumerate()
            .filter_map(|(fd, entry)| entry.as_ref().map(|_| fd as i32))
            .collect()
    }

    /// Reads a description and its close-on-exec bit under one table lock.
    pub(crate) fn descriptor_info(&self, fd: i32) -> Option<(Descriptor, bool)> {
        let index = usize::try_from(fd).ok()?;
        let files = self.files.lock();
        let entry = files.slots.get(index)?.as_ref()?;
        Some((entry.descriptor.clone(), entry.close_on_exec))
    }

    /// Returns the process address space.
    pub(crate) fn address_space(&self) -> Arc<VmSpace> {
        self.address_space.lock().clone()
    }

    /// Returns the file used to load the current executable image.
    pub(crate) fn executable(&self) -> Option<FileRef> {
        self.executable.lock().clone()
    }

    fn replace_address_space(&self, space: Arc<VmSpace>) -> Arc<VmSpace> {
        core::mem::replace(&mut *self.address_space.lock(), space)
    }

    /// Resolves a process-relative path into the global VFS namespace.
    pub(crate) fn resolve_path(&self, path: &[u8]) -> Vec<u8> {
        let mut combined = Vec::with_capacity(path.len() + 1);
        if path.first() == Some(&b'/') {
            combined.extend_from_slice(path);
        } else {
            let cwd = self.cwd.lock();
            combined.extend_from_slice(&cwd.path);
            if cwd.path.as_slice() != b"/" {
                combined.push(b'/');
            }
            drop(cwd);
            combined.extend_from_slice(path);
        }

        let mut components: Vec<&[u8]> = Vec::new();
        for component in combined.split(|byte| *byte == b'/') {
            match component {
                b"" | b"." => {}
                b".." => {
                    components.pop();
                }
                value => components.push(value),
            }
        }
        if components.is_empty() {
            return b"/".to_vec();
        }
        let mut resolved = Vec::new();
        for component in components {
            resolved.push(b'/');
            resolved.extend_from_slice(component);
        }
        resolved
    }

    /// Changes the process working directory.
    pub(crate) fn set_cwd(&self, path: &[u8]) -> Result<()> {
        let current = self.cwd.lock().clone();
        let anchor = crate::fs::resolve_at(&current.anchor, path, true)?;
        if anchor.vnode().kind() != VnodeKind::Directory {
            return Err(crate::fs::Error::NotDirectory.into());
        }
        let resolved_path = self.resolve_path(path);
        *self.cwd.lock() = WorkingDirectory {
            path: resolved_path,
            anchor,
        };
        Ok(())
    }

    /// Reanchors this process at a newly installed global root filesystem.
    pub(crate) fn reset_cwd(&self) -> Result<()> {
        *self.cwd.lock() = WorkingDirectory {
            path: b"/".to_vec(),
            anchor: crate::fs::root_anchor()?,
        };
        Ok(())
    }

    /// Returns the process working directory.
    pub(crate) fn cwd(&self) -> Vec<u8> {
        self.cwd.lock().path.clone()
    }

    /// Returns the file mode creation mask.
    pub(crate) fn umask(&self) -> u16 {
        self.umask.load(Ordering::Acquire)
    }

    /// Replaces the file mode creation mask and returns the previous value.
    pub(crate) fn set_umask(&self, mask: u16) -> u16 {
        self.umask.swap(mask & 0o777, Ordering::AcqRel)
    }

    /// Applies the creation mask to a requested permission mode.
    pub(crate) fn apply_umask(&self, mode: u16) -> u16 {
        mode & 0o7777 & !self.umask()
    }

    /// Arms signal interruption for a blocking syscall.
    ///
    /// Returns whether a pending signal should abort the operation now.
    pub(crate) fn prepare_interrupt_wait(&self) -> bool {
        self.signals.prepare_interrupt_wait()
    }

    /// Returns the event signalled when a signal becomes pending.
    pub(crate) fn interrupt_event(&self) -> &crate::sys::event::Event {
        self.signals.interrupt_event()
    }

    /// Returns the stable namespace location of the process working directory.
    pub(crate) fn cwd_anchor(&self) -> PathAnchor {
        self.cwd.lock().anchor.clone()
    }

    /// Snapshots the old limit and commits the new one only after successful copyout.
    pub(crate) fn update_descriptor_limit(
        &self,
        new: Option<DescriptorLimit>,
        publish: impl FnOnce(DescriptorLimit) -> crate::syscall::Result<()>,
    ) -> crate::syscall::Result<()> {
        use crate::syscall::Errno;
        if let Some(limit) = new {
            if limit.soft > limit.hard {
                return Err(Errno::Invalid);
            }
            if limit.hard > MAX_FILES {
                return Err(Errno::Permission);
            }
        }
        let mut files = self.files.lock();
        publish(files.limit)?;
        if let Some(limit) = new {
            files.limit = limit;
        }
        Ok(())
    }

    fn install_descriptor(&self, entry: DescriptorEntry, minimum: usize) -> Result<i32> {
        self.install_descriptor_from(minimum, || entry)
    }

    // Construct ownership only after a slot is available. A failed installation
    // must not close a socket still owned by a listener's pending queue.
    fn install_descriptor_from(
        &self,
        minimum: usize,
        make: impl FnOnce() -> DescriptorEntry,
    ) -> Result<i32> {
        let mut files = self.files.lock();
        let index = files.available(minimum).next().ok_or(Error::TooManyFiles)?;
        files.insert(index, make());
        Ok(index as i32)
    }

    /// Installs an open file and returns its new descriptor number.
    pub(crate) fn install_file(&self, file: FileRef, close_on_exec: bool) -> Result<i32> {
        self.install_descriptor(
            DescriptorEntry {
                handle: OpenHandle::new(),
                descriptor: Descriptor::File(file),
                close_on_exec,
            },
            0,
        )
    }

    /// Installs an arbitrary process descriptor.
    pub(crate) fn install_descriptor_value(
        &self,
        descriptor: Descriptor,
        close_on_exec: bool,
    ) -> Result<i32> {
        self.install_descriptor_from(0, || DescriptorEntry {
            handle: OpenHandle::for_descriptor(&descriptor),
            descriptor,
            close_on_exec,
        })
    }

    /// Returns a cloned descriptor table entry.
    pub(crate) fn descriptor(&self, fd: i32) -> Option<Descriptor> {
        let index = usize::try_from(fd).ok()?;
        self.files
            .lock()
            .slots
            .get(index)?
            .as_ref()
            .map(|entry| entry.descriptor.clone())
    }

    /// Resolves several descriptors under one descriptor-table lock.
    ///
    /// Readiness scans revisit the whole set on every pass, so acquiring the
    /// lock once per pass avoids one acquisition per descriptor.
    pub(crate) fn descriptors(&self, fds: &[i32]) -> Vec<Option<Descriptor>> {
        let files = self.files.lock();
        fds.iter()
            .map(|fd| {
                let index = usize::try_from(*fd).ok()?;
                files
                    .slots
                    .get(index)?
                    .as_ref()
                    .map(|entry| entry.descriptor.clone())
            })
            .collect()
    }

    /// Resolves a watch without counting the observer as an open descriptor.
    pub(crate) fn watch_target(&self, fd: i32) -> Option<(Descriptor, Arc<DescriptorLife>)> {
        let index = usize::try_from(fd).ok()?;
        let files = self.files.lock();
        let entry = files.slots.get(index)?.as_ref()?;
        Some((entry.descriptor.clone(), entry.handle.life()))
    }

    /// Duplicates `fd` into the first slot at or above `minimum`.
    pub(crate) fn duplicate_descriptor(
        &self,
        fd: i32,
        minimum: usize,
        close_on_exec: bool,
    ) -> Result<i32> {
        let index = usize::try_from(fd).map_err(|_| Error::BadFileDescriptor)?;
        let mut entry = self
            .files
            .lock()
            .slots
            .get(index)
            .and_then(Option::as_ref)
            .cloned()
            .ok_or(Error::BadFileDescriptor)?;
        entry.close_on_exec = close_on_exec;
        self.install_descriptor(entry, minimum)
    }

    /// Duplicates `old_fd` onto `new_fd`.
    pub(crate) fn duplicate_descriptor_to(
        &self,
        old_fd: i32,
        new_fd: i32,
        close_on_exec: bool,
    ) -> Result<i32> {
        let old_index = usize::try_from(old_fd).map_err(|_| Error::BadFileDescriptor)?;
        let new_index = usize::try_from(new_fd).map_err(|_| Error::InvalidArgument)?;
        let mut files = self.files.lock();
        let mut entry = files
            .slots
            .get(old_index)
            .and_then(Option::as_ref)
            .cloned()
            .ok_or(Error::BadFileDescriptor)?;
        if old_index == new_index {
            return Ok(new_fd);
        }
        if new_index >= files.limit.soft {
            return Err(Error::TooManyFiles);
        }
        entry.close_on_exec = close_on_exec;
        files.insert(new_index, entry);
        Ok(new_fd)
    }

    /// Returns whether a descriptor is marked close-on-exec.
    pub(crate) fn descriptor_close_on_exec(&self, fd: i32) -> Option<bool> {
        let index = usize::try_from(fd).ok()?;
        self.files
            .lock()
            .slots
            .get(index)?
            .as_ref()
            .map(|entry| entry.close_on_exec)
    }

    /// Changes a descriptor's close-on-exec flag.
    pub(crate) fn set_descriptor_close_on_exec(&self, fd: i32, value: bool) -> bool {
        let Ok(index) = usize::try_from(fd) else {
            return false;
        };
        let mut files = self.files.lock();
        let Some(Some(entry)) = files.slots.get_mut(index) else {
            return false;
        };
        entry.close_on_exec = value;
        true
    }

    /// Removes a descriptor table entry.
    pub(crate) fn close_descriptor(&self, fd: i32) -> bool {
        let Ok(index) = usize::try_from(fd) else {
            return false;
        };
        let mut files = self.files.lock();
        let Some(slot) = files.slots.get_mut(index) else {
            return false;
        };
        slot.take().is_some()
    }

    fn close_exec_descriptors(&self) {
        for slot in self.files.lock().slots.iter_mut() {
            if slot.as_ref().is_some_and(|entry| entry.close_on_exec) {
                *slot = None;
            }
        }
    }

    /// Returns the process session identifier.
    pub(crate) fn session(&self) -> usize {
        self.session.load(Ordering::Acquire)
    }

    /// Returns the process-group identifier.
    pub(crate) fn process_group(&self) -> usize {
        self.process_group.load(Ordering::Acquire)
    }

    /// Returns whether this process has a controlling terminal.
    pub(crate) fn has_controlling_tty(&self) -> bool {
        self.controlling_tty.lock().is_some()
    }

    /// Returns the controlling terminal vnode identity.
    pub(crate) fn controlling_tty_key(&self) -> Option<VnodeKey> {
        self.controlling_tty
            .lock()
            .as_ref()
            .map(|file| file.vnode().key())
    }

    /// Records the process controlling terminal.
    pub(crate) fn set_controlling_tty(&self, file: FileRef) {
        *self.controlling_tty.lock() = Some(file);
    }

    /// Relinquishes a matching controlling terminal.
    pub(crate) fn clear_controlling_tty(&self, key: VnodeKey) {
        let mut terminal = self.controlling_tty.lock();
        if terminal
            .as_ref()
            .is_some_and(|file| file.vnode().key() == key)
        {
            *terminal = None;
        }
    }

    fn set_process_group(&self, group: usize) {
        self.process_group.store(group, Ordering::Release);
    }

    fn create_session(&self) -> Result<usize> {
        if self.process_group() == self.pid {
            return Err(Error::PermissionDenied);
        }
        self.session.store(self.pid, Ordering::Release);
        self.process_group.store(self.pid, Ordering::Release);
        *self.controlling_tty.lock() = None;
        Ok(self.pid)
    }

    pub(crate) fn is_exited(&self) -> bool {
        self.exited.load(Ordering::Acquire)
    }

    pub(crate) fn register_thread(&self, tid: usize) {
        let _ = self
            .main_thread
            .compare_exchange(0, tid, Ordering::AcqRel, Ordering::Acquire);
        self.active_threads.fetch_add(1, Ordering::AcqRel);
    }

    pub(crate) fn main_thread(&self) -> usize {
        self.main_thread.load(Ordering::Acquire)
    }

    fn unregister_thread(&self) -> bool {
        let previous = self.active_threads.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "proc: active thread count underflow");
        previous == 1
    }

    pub(crate) fn thread_count(&self) -> usize {
        self.active_threads.load(Ordering::Acquire)
    }

    fn encoded_exit_status(&self) -> i32 {
        self.exit_status.load(Ordering::Acquire)
    }

    fn matches_wait(&self, selector: WaitSelector) -> bool {
        match selector {
            WaitSelector::All => true,
            WaitSelector::Process(pid) => self.pid == pid,
            WaitSelector::Group(group) => self.process_group() == group,
        }
    }

    fn mark_exited(self: &Arc<Self>, encoded_status: i32) {
        let terminal = if self.session() == self.pid {
            self.controlling_tty.lock().take()
        } else {
            None
        };
        if let Some(terminal) = terminal {
            let _ = terminal.ioctl(
                IoctlContext {
                    process_id: self.pid,
                    process_group: self.process_group() as i32,
                    session_id: self.session() as i32,
                    is_session_leader: true,
                },
                crate::dev::tty::TIOCNOTTY,
                0,
                &mut [],
            );
            clear_session_controlling_tty(self.session(), terminal.vnode().key());
        }
        self.reparent_children();

        let parent = self.parent.lock().upgrade();
        let executable = self.executable.lock().take();
        drop(executable);
        self.files.lock().slots.clear();
        cgroup::exited(self.pid);
        if let Some(parent) = &parent {
            let _children = parent.children.lock();
            self.exit_status.store(encoded_status, Ordering::Release);
            self.exited.store(true, Ordering::Release);
        } else {
            self.exit_status.store(encoded_status, Ordering::Release);
            self.exited.store(true, Ordering::Release);
        }

        // Waking a local waiter can synchronously reschedule on x86. Keep all
        // process locks out of that path so the waiter can inspect children.
        if let Some(parent) = parent {
            signal::send_kernel(&parent, signal::SIGCHLD);
            parent.child_event.signal();
        }
    }

    fn reparent_children(self: &Arc<Self>) {
        let children = core::mem::take(&mut *self.children.lock());
        if children.is_empty() {
            return;
        }
        let init = process_registry().lock().init.upgrade();
        let init = init.filter(|init| !Arc::ptr_eq(init, self));
        for (pid, child) in children {
            let death_signal = {
                let mut parent = child.parent.lock();
                *parent = init.as_ref().map_or_else(Weak::new, Arc::downgrade);
                if let Some(init) = &init {
                    init.children.lock().insert(pid, child.clone());
                }
                child.parent_death_signal()
            };
            // Both wakeups can reschedule immediately. Release process locks
            // before a waiter or signal handler can inspect the child.
            if let Some(init) = &init
                && child.is_exited()
            {
                init.child_event.signal();
            }
            if death_signal != 0 && !child.is_exited() {
                signal::send_kernel(&child, death_signal);
            }
        }
    }

    fn notify_wait_change(&self) {
        let Some(parent) = self.parent.lock().upgrade() else {
            return;
        };
        // Serialize with a waiter's scan/reset, but wake outside process locks.
        drop(parent.children.lock());
        parent.child_event.signal();
        signal::send_kernel(&parent, signal::SIGCHLD);
    }
}

fn process_registry() -> &'static Mutex<ProcessRegistry> {
    PROCESSES.call_once(|| Mutex::new(ProcessRegistry::new()))
}

fn unregister_process(pid: usize) {
    process_registry().lock().processes.remove(&pid);
    cgroup::reaped(pid);
}

/// Finds one live or zombie process by identifier.
pub(crate) fn find(pid: usize) -> Option<Arc<Process>> {
    process_registry().lock().processes.get(&pid)?.upgrade()
}

/// Snapshots registered process numbers. A process may exit after this returns.
pub(crate) fn process_ids() -> Vec<usize> {
    process_registry()
        .lock()
        .processes
        .iter()
        .filter_map(|(pid, process)| (process.strong_count() != 0).then_some(*pid))
        .collect()
}

/// Loads and schedules `/sbin/init`.
pub fn spawn_init() -> Result<usize> {
    let image = elf::load(
        b"/sbin/init",
        &[b"/sbin/init".as_slice()],
        &[b"PATH=/usr/bin:/bin".as_slice(), b"HOME=/".as_slice()],
    )?;
    let process = Process::new_root(image.address_space, image.executable);
    let pid = process.pid();
    let tid = sched::run_user(process, image.entry, image.stack);
    info!("started /sbin/init as pid {pid}, tid {tid}");
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

/// Forks the current process with a lazy-COW address space.
pub(crate) fn fork_current(frame: &TrapFrame) -> Result<usize> {
    let parent = current().ok_or(Error::InvalidArgument)?;
    let child_space = parent.address_space().fork()?;
    let child = Process::fork_from(&parent, child_space)?;
    let pid = child.pid();
    sched::run_forked_user(child, frame, current_thread_pointer());
    Ok(pid)
}

/// Creates another userspace thread in the current process.
pub(crate) fn create_thread(entry: u64, stack: u64, thread_pointer: u64) -> Result<usize> {
    if !(USER_ADDRESS_MIN..USER_ADDRESS_MAX).contains(&entry)
        || !(USER_ADDRESS_MIN..USER_ADDRESS_MAX).contains(&stack)
        || thread_pointer >= USER_ADDRESS_MAX
    {
        return Err(Error::InvalidArgument);
    }
    let process = current().ok_or(Error::InvalidArgument)?;
    if process.is_exited() {
        return Err(Error::InvalidArgument);
    }
    Ok(sched::run_user_thread(
        process,
        entry,
        stack,
        thread_pointer,
    ))
}

/// Replaces the current process image.
pub(crate) fn exec_current(
    frame: &mut TrapFrame,
    path: &[u8],
    arguments: &[Vec<u8>],
    environment: &[Vec<u8>],
) -> Result<()> {
    let process = current().ok_or(Error::InvalidArgument)?;
    if process.thread_count() != 1 {
        return Err(Error::InvalidArgument);
    }
    let path = process.resolve_path(path);
    let image = elf::load(&path, arguments, environment)?;
    let previous = process.replace_address_space(image.address_space.clone());
    mem::install_current_space(image.address_space);
    drop(previous);
    let previous = process.executable.lock().replace(image.executable);
    drop(previous);
    process.close_exec_descriptors();
    process.signals.reset_for_exec();
    process
        .main_thread
        .store(crate::arch::thiscpu().current_thread, Ordering::Release);
    set_current_thread_pointer(0)?;
    // SAFETY: `frame` is the current thread's live syscall frame.
    unsafe {
        crate::arch::cpu::init_user_thread_frame(frame, image.entry, image.stack);
        #[cfg(target_arch = "x86_64")]
        crate::arch::cpu::FpuState::new().restore();
    }
    Ok(())
}

/// Process selection shared by waitpid and waitid. Group 1 is distinct from all.
#[derive(Clone, Copy)]
pub(crate) enum WaitSelector {
    All,
    Process(usize),
    Group(usize),
}

/// Child events to report and whether observing them consumes their status.
pub(crate) struct WaitOptions {
    pub(crate) nohang: bool,
    pub(crate) exited: bool,
    pub(crate) stopped: bool,
    pub(crate) continued: bool,
    pub(crate) nowait: bool,
}

/// Waits for a matching child event. Delivery precedes consumption under the
/// children lock, so a failed user copy does not lose a zombie or stop report.
/// The callback must not enter process lifecycle operations.
pub(crate) fn wait_current(
    selector: WaitSelector,
    options: WaitOptions,
    mut deliver: impl FnMut(Option<(usize, i32)>) -> Result<()>,
) -> Result<Option<(usize, i32)>> {
    let parent = current().ok_or(Error::InvalidArgument)?;
    loop {
        // Preserve notifications through the scan/park gap even if another
        // waiter resets the shared level while waiting for different events.
        let listeners = [
            parent.child_event.listen(),
            parent.interrupt_event().listen(),
        ];
        let reaped = {
            let mut children = parent.children.lock();
            let mut matching = false;
            let mut exited = None;
            for (pid, child) in children.iter() {
                if !child.matches_wait(selector) {
                    continue;
                }
                matching = true;
                if child.is_exited() {
                    if options.exited {
                        exited = Some(*pid);
                        break;
                    }
                    continue;
                }
                let status = child.wait_status.load(Ordering::Acquire);
                if (options.stopped && status & 0xff == 0x7f)
                    || (options.continued && status == 0xffff)
                {
                    deliver(Some((*pid, status)))?;
                    if options.nowait
                        || child
                            .wait_status
                            .compare_exchange(status, 0, Ordering::AcqRel, Ordering::Acquire)
                            .is_ok()
                    {
                        return Ok(Some((*pid, status)));
                    }
                }
            }
            if !matching {
                return Err(Error::NoChild);
            }
            if let Some(pid) = exited {
                let status = children[&pid].encoded_exit_status();
                deliver(Some((pid, status)))?;
                if options.nowait {
                    return Ok(Some((pid, status)));
                }
                children
                    .remove(&pid)
                    .expect("proc: selected child vanished before reap");
                if !children.values().any(|child| child.is_exited()) {
                    parent.child_event.reset();
                }
                Some((pid, status))
            } else if options.nohang {
                deliver(None)?;
                return Ok(None);
            } else {
                parent.child_event.reset();
                None
            }
        };
        if let Some((pid, status)) = reaped {
            unregister_process(pid);
            return Ok(Some((pid, status)));
        }
        if parent.prepare_interrupt_wait() {
            return Err(Error::Interrupted);
        }
        EventListener::wait_any(&listeners);
    }
}

/// Returns the current process's parent identifier.
pub(crate) fn current_parent_pid() -> usize {
    current()
        .and_then(|process| process.parent.lock().upgrade())
        .map_or(0, |parent| parent.pid())
}

/// Returns a process-group identifier.
pub(crate) fn get_process_group(pid: usize) -> Result<usize> {
    let process = if pid == 0 {
        current().ok_or(Error::InvalidArgument)?
    } else {
        find(pid).ok_or(Error::NoSuchProcess)?
    };
    Ok(process.process_group())
}

/// Returns a process session identifier.
pub(crate) fn get_session(pid: usize) -> Result<usize> {
    let process = if pid == 0 {
        current().ok_or(Error::InvalidArgument)?
    } else {
        find(pid).ok_or(Error::NoSuchProcess)?
    };
    Ok(process.session())
}

/// Changes a process group for the current process or one of its children.
pub(crate) fn set_process_group(pid: usize, group: usize) -> Result<()> {
    let current = current().ok_or(Error::InvalidArgument)?;
    let target = if pid == 0 || pid == current.pid() {
        current.clone()
    } else {
        current
            .children
            .lock()
            .get(&pid)
            .cloned()
            .ok_or(Error::NoSuchProcess)?
    };
    if target.session() != current.session() {
        return Err(Error::PermissionDenied);
    }
    let group = if group == 0 { target.pid() } else { group };
    target.set_process_group(group);
    Ok(())
}

/// Selector for the `setpriority`/`getpriority` target set.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum PriorityWhich {
    /// A single process.
    Process,
    /// Every process in a process group.
    ProcessGroup,
    /// Every process owned by a user.
    User,
}

/// Collects the live processes selected by `which` and `who`.
///
/// A `who` of zero selects the caller's own process, group, or user, matching
/// POSIX. The kernel is single-user, so the only valid user id is zero.
fn priority_targets(which: PriorityWhich, who: u64) -> Result<Vec<Arc<Process>>> {
    let current = current().ok_or(Error::InvalidArgument)?;

    let targets: Vec<Arc<Process>> = match which {
        PriorityWhich::Process => {
            let pid = if who == 0 {
                current.pid()
            } else {
                usize::try_from(who).map_err(|_| Error::NoSuchProcess)?
            };
            find(pid).into_iter().collect()
        }
        PriorityWhich::ProcessGroup => {
            let group = if who == 0 {
                current.process_group()
            } else {
                usize::try_from(who).map_err(|_| Error::NoSuchProcess)?
            };
            process_registry()
                .lock()
                .processes
                .values()
                .filter_map(Weak::upgrade)
                .filter(|process| process.process_group() == group)
                .collect()
        }
        PriorityWhich::User => {
            if who != 0 {
                return Err(Error::NoSuchProcess);
            }
            process_registry()
                .lock()
                .processes
                .values()
                .filter_map(Weak::upgrade)
                .collect()
        }
    };

    let live: Vec<Arc<Process>> = targets
        .into_iter()
        .filter(|process| !process.is_exited())
        .collect();
    if live.is_empty() {
        return Err(Error::NoSuchProcess);
    }
    Ok(live)
}

/// Applies `nice` to every process selected by `which` and `who`.
pub(crate) fn set_priority(which: PriorityWhich, who: u64, nice: i8) -> Result<()> {
    let nice = nice.clamp(sched::NICE_MIN, sched::NICE_MAX);
    for process in priority_targets(which, who)? {
        process.set_nice(nice);
        sched::set_process_nice(process.pid(), nice);
    }
    Ok(())
}

/// Returns the most favourable nice value among the selected processes.
///
/// POSIX specifies the lowest (most favourable) value when several processes
/// match.
pub(crate) fn get_priority(which: PriorityWhich, who: u64) -> Result<i8> {
    let mut best = sched::NICE_MAX;
    for process in priority_targets(which, who)? {
        // Prefer the live scheduler value; fall back to the process record for
        // a process whose threads have not started yet.
        let nice = sched::process_nice(process.pid()).unwrap_or_else(|| process.nice());
        best = best.min(nice);
    }
    Ok(best)
}

/// Returns whether a live process group belongs to `session`.
pub(crate) fn process_group_in_session(group: usize, session: usize) -> bool {
    process_registry().lock().processes.values().any(|process| {
        process.upgrade().is_some_and(|process| {
            !process.is_exited() && process.process_group() == group && process.session() == session
        })
    })
}

/// Clears a controlling terminal from every process in a session.
pub(crate) fn clear_session_controlling_tty(session: usize, key: VnodeKey) {
    for process in process_registry()
        .lock()
        .processes
        .values()
        .filter_map(Weak::upgrade)
        .filter(|process| process.session() == session)
    {
        process.clear_controlling_tty(key);
    }
}

/// Creates a new session for the current process.
pub(crate) fn create_session() -> Result<usize> {
    current().ok_or(Error::InvalidArgument)?.create_session()
}

/// Terminates the current process and thread.
pub(crate) fn exit_current(status: i32) -> ! {
    if let Some(process) = current() {
        trace!("pid {} exited with status {status}", process.pid());
        if process.pid() == 1 {
            panic!("init exited with status {status}");
        }
        process.mark_exited((status & 0xff) << 8);
        finish_current_process_thread_exit(process);
    }
    sched::exit_current()
}

/// Terminates the current process as the result of an uncaught signal.
pub(crate) fn exit_current_signal(signal: u8) -> ! {
    if let Some(process) = current() {
        // A process dying on a signal it never handled is almost always a bug
        // in that process, and it is invisible from userspace once the shell
        // has reported the exit status.
        debug!("pid {} killed by signal {signal}", process.pid());
        if process.pid() == 1 {
            panic!("init killed by signal {signal}");
        }
        process.mark_exited(i32::from(signal & 0x7f));
        finish_current_process_thread_exit(process);
    }
    sched::exit_current()
}

/// Terminates only the current userspace thread.
pub(crate) fn exit_current_thread() -> ! {
    if let Some(process) = current()
        && process.unregister_thread()
        && !process.is_exited()
    {
        process.mark_exited(0);
    }
    sched::exit_current()
}

/// Terminates a peer thread after another thread exited the process.
pub(crate) fn exit_current_thread_if_process_exited() {
    let Some(process) = current() else {
        return;
    };
    if process.is_exited() {
        finish_current_process_thread_exit(process);
    }
}

fn finish_current_process_thread_exit(process: Arc<Process>) -> ! {
    // The active-thread decrement and scheduler removal must be atomic with
    // trap return, which also retires threads belonging to an exited process.
    crate::arch::irqset(false);
    process.unregister_thread();
    // Context switching abandons this stack without running destructors. The
    // thread record retains the process until reaping; release our temporary
    // owner now so its address space can eventually be destroyed.
    drop(process);
    sched::exit_current()
}
