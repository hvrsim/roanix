//! inotify filesystem notification descriptors.

use alloc::{
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicUsize, Ordering};

use crate::{
    fs::{self, PollEvents, VnodeKey, VnodeKind},
    mem::IoSink,
    proc::Descriptor,
    sys::{
        event::Event,
        sync::{Mutex, Once},
    },
    syscall::{Errno, Result, current_process, map_fs_error, map_process_error, read_user_path},
};

pub(crate) const IN_ACCESS: u32 = 0x0000_0001;
pub(crate) const IN_MODIFY: u32 = 0x0000_0002;
pub(crate) const IN_ATTRIB: u32 = 0x0000_0004;
pub(crate) const IN_CLOSE_WRITE: u32 = 0x0000_0008;
pub(crate) const IN_CLOSE_NOWRITE: u32 = 0x0000_0010;
pub(crate) const IN_OPEN: u32 = 0x0000_0020;
pub(crate) const IN_MOVED_FROM: u32 = 0x0000_0040;
pub(crate) const IN_MOVED_TO: u32 = 0x0000_0080;
pub(crate) const IN_CREATE: u32 = 0x0000_0100;
pub(crate) const IN_DELETE: u32 = 0x0000_0200;
pub(crate) const IN_DELETE_SELF: u32 = 0x0000_0400;
pub(crate) const IN_MOVE_SELF: u32 = 0x0000_0800;
const IN_Q_OVERFLOW: u32 = 0x0000_4000;
const IN_IGNORED: u32 = 0x0000_8000;
const IN_ONLYDIR: u32 = 0x0100_0000;
const IN_DONT_FOLLOW: u32 = 0x0200_0000;
const IN_EXCL_UNLINK: u32 = 0x0400_0000;
const IN_MASK_ADD: u32 = 0x2000_0000;
pub(crate) const IN_ISDIR: u32 = 0x4000_0000;
const IN_ONESHOT: u32 = 0x8000_0000;
const IN_ALL_EVENTS: u32 = IN_ACCESS
    | IN_MODIFY
    | IN_ATTRIB
    | IN_CLOSE_WRITE
    | IN_CLOSE_NOWRITE
    | IN_OPEN
    | IN_MOVED_FROM
    | IN_MOVED_TO
    | IN_CREATE
    | IN_DELETE
    | IN_DELETE_SELF
    | IN_MOVE_SELF;
const IN_CONTROL_FLAGS: u32 =
    IN_ONLYDIR | IN_DONT_FOLLOW | IN_EXCL_UNLINK | IN_MASK_ADD | IN_ONESHOT;

const IN_CLOEXEC: u64 = 0o2000000;
const IN_NONBLOCK: u64 = 0o4000;
const EVENT_HEADER_SIZE: usize = 16;
const MAX_QUEUE_BYTES: usize = 1024 * 1024;

static WATCHERS: Once<Mutex<BTreeMap<VnodeKey, Vec<Weak<Inotify>>>>> = Once::new();
/// Number of vnodes with at least one registered watcher.
///
/// Every VFS data and metadata operation reports to [`notify`], so the common
/// case of no inotify watches at all must not touch the global registry lock.
static WATCHED_KEYS: AtomicUsize = AtomicUsize::new(0);
static NEXT_COOKIE: AtomicU32 = AtomicU32::new(1);

#[derive(Clone, Copy)]
struct Watch {
    key: VnodeKey,
    mask: u32,
}

struct InotifyState {
    watches: BTreeMap<i32, Watch>,
    by_key: BTreeMap<VnodeKey, i32>,
    queue: VecDeque<Box<[u8]>>,
    queued_bytes: usize,
    overflowed: bool,
}

/// One inotify open-file description.
pub(crate) struct Inotify {
    state: Mutex<InotifyState>,
    event: Event,
    next_watch: AtomicI32,
    nonblocking: AtomicBool,
}

impl Inotify {
    fn new(nonblocking: bool) -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(InotifyState {
                watches: BTreeMap::new(),
                by_key: BTreeMap::new(),
                queue: VecDeque::new(),
                queued_bytes: 0,
                overflowed: false,
            }),
            event: Event::new(),
            next_watch: AtomicI32::new(1),
            nonblocking: AtomicBool::new(nonblocking),
        })
    }

    fn add_watch(self: &Arc<Self>, key: VnodeKey, mask: u32) -> Result<i32> {
        validate_mask(mask)?;
        let mut state = self.state.lock();
        if let Some(watch_descriptor) = state.by_key.get(&key).copied() {
            let watch = state
                .watches
                .get_mut(&watch_descriptor)
                .expect("inotify: key index lost watch");
            watch.mask = if mask & IN_MASK_ADD != 0 {
                watch.mask | (mask & !IN_MASK_ADD)
            } else {
                mask
            };
            return Ok(watch_descriptor);
        }

        let watch_descriptor = self.next_watch.fetch_add(1, Ordering::Relaxed);
        if watch_descriptor <= 0 {
            return Err(Errno::Overflow);
        }
        state.watches.insert(
            watch_descriptor,
            Watch {
                key,
                mask: mask & !IN_MASK_ADD,
            },
        );
        state.by_key.insert(key, watch_descriptor);
        drop(state);

        let weak = Arc::downgrade(self);
        let mut watchers = watchers().lock();
        let entries = watchers.entry(key).or_default();
        entries.retain(|entry| entry.strong_count() != 0);
        if !entries.iter().any(|entry| entry.ptr_eq(&weak)) {
            entries.push(weak);
        }
        WATCHED_KEYS.store(watchers.len(), Ordering::Release);
        Ok(watch_descriptor)
    }

    fn remove_watch(&self, watch_descriptor: i32) -> Result<()> {
        let mut state = self.state.lock();
        let watch = state
            .watches
            .remove(&watch_descriptor)
            .ok_or(Errno::Invalid)?;
        state.by_key.remove(&watch.key);
        queue_record(&mut state, watch_descriptor, IN_IGNORED, 0, None);
        self.event.signal();
        Ok(())
    }

    fn queue_for_key(&self, key: VnodeKey, mask: u32, cookie: u32, name: Option<&[u8]>) {
        let mut state = self.state.lock();
        let Some(watch_descriptor) = state.by_key.get(&key).copied() else {
            return;
        };
        let watch = state
            .watches
            .get(&watch_descriptor)
            .copied()
            .expect("inotify: key index lost watch");
        if watch.mask & mask & IN_ALL_EVENTS == 0 {
            return;
        }

        queue_record(&mut state, watch_descriptor, mask, cookie, name);
        let remove = watch.mask & IN_ONESHOT != 0 || mask & IN_DELETE_SELF != 0;
        if remove {
            state.watches.remove(&watch_descriptor);
            state.by_key.remove(&key);
            queue_record(&mut state, watch_descriptor, IN_IGNORED, 0, None);
        }
        self.event.signal();
    }

    pub(crate) fn read(&self, sink: &mut IoSink<'_>) -> Result<usize> {
        let capacity = sink.len();
        loop {
            let mut state = self.state.lock();
            if let Some(first) = state.queue.front() {
                if first.len() > capacity {
                    return Err(Errno::Invalid);
                }
                let mut written = 0usize;
                while let Some(record) = state.queue.front() {
                    if written + record.len() > capacity {
                        break;
                    }
                    // The event is dequeued only after it has reached user
                    // memory, so a faulting copy cannot drop an event.
                    if sink.store(written, record).is_err() {
                        if written != 0 {
                            return Ok(written);
                        }
                        return Err(Errno::Fault);
                    }
                    let record = state.queue.pop_front().expect("inotify: event vanished");
                    let length = record.len();
                    written += length;
                    state.queued_bytes = state.queued_bytes.saturating_sub(length);
                    if event_mask(&record) & IN_Q_OVERFLOW != 0 {
                        state.overflowed = false;
                    }
                }
                if state.queue.is_empty() {
                    self.event.reset();
                }
                return Ok(written);
            }
            if self.nonblocking.load(Ordering::Acquire) {
                return Err(Errno::TryAgain);
            }
            self.event.reset();
            drop(state);
            self.event.wait();
        }
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        if requested.contains(PollEvents::IN) && !self.state.lock().queue.is_empty() {
            PollEvents::IN
        } else {
            PollEvents::empty()
        }
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        requested: PollEvents,
        output: &mut alloc::vec::Vec<&'a Event>,
    ) -> bool {
        if requested.contains(PollEvents::IN) {
            output.push(&self.event);
        }
        true
    }

    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }

    pub(crate) fn set_nonblocking(&self, nonblocking: bool) {
        self.nonblocking.store(nonblocking, Ordering::Release);
    }
}

/// Allocates a nonzero rename cookie.
pub(crate) fn next_cookie() -> u32 {
    loop {
        let cookie = NEXT_COOKIE.fetch_add(1, Ordering::Relaxed);
        if cookie != 0 {
            return cookie;
        }
    }
}

/// Returns whether any vnode currently has a watcher.
///
/// Callers use this to skip work that only exists to describe an event.
pub(crate) fn is_watching() -> bool {
    WATCHED_KEYS.load(Ordering::Acquire) != 0
}

/// Emits an inotify event for one watched vnode.
pub(crate) fn notify(key: VnodeKey, mask: u32, cookie: u32, name: Option<&[u8]>) {
    if WATCHED_KEYS.load(Ordering::Acquire) == 0 {
        return;
    }
    let instances = {
        let mut watchers = watchers().lock();
        let Some(entries) = watchers.get_mut(&key) else {
            return;
        };
        let instances = entries.iter().filter_map(Weak::upgrade).collect::<Vec<_>>();
        entries.retain(|entry| entry.strong_count() != 0);
        if entries.is_empty() {
            watchers.remove(&key);
        }
        WATCHED_KEYS.store(watchers.len(), Ordering::Release);
        instances
    };
    for instance in instances {
        instance.queue_for_key(key, mask, cookie, name);
    }
}

crate::syscall_handler! {
    syscall_inotify_init(_frame, flags: u64 = 0) {
        if flags & !(IN_CLOEXEC | IN_NONBLOCK) != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let inotify = Inotify::new(flags & IN_NONBLOCK != 0);
        let fd = process
            .install_descriptor_value(
                Descriptor::Inotify(inotify),
                flags & IN_CLOEXEC != 0,
            )
            .map_err(map_process_error)?;
        Ok(fd as u64)
    }
}

crate::syscall_handler! {
    syscall_inotify_add(
        _frame,
        inotify_fd: i32 = 0,
        path: u64 = 1,
        mask: u32 = 2,
    ) {
        let process = current_process()?;
        let Descriptor::Inotify(inotify) =
            process.descriptor(inotify_fd).ok_or(Errno::BadFileDescriptor)?
        else {
            return Err(Errno::Invalid);
        };
        validate_mask(mask)?;
        let path = read_user_path(&process, path)?;
        let anchor = fs::resolve_at(
            &process.cwd_anchor(),
            &path,
            mask & IN_DONT_FOLLOW == 0,
        )
        .map_err(map_fs_error)?;
        if mask & IN_ONLYDIR != 0 && anchor.vnode().kind() != VnodeKind::Directory {
            return Err(Errno::NotDirectory);
        }
        Ok(inotify.add_watch(anchor.vnode().key(), mask)? as u64)
    }
}

crate::syscall_handler! {
    syscall_inotify_remove(_frame, inotify_fd: i32 = 0, watch_descriptor: i32 = 1) {
        let process = current_process()?;
        let Descriptor::Inotify(inotify) =
            process.descriptor(inotify_fd).ok_or(Errno::BadFileDescriptor)?
        else {
            return Err(Errno::Invalid);
        };
        inotify.remove_watch(watch_descriptor)?;
        Ok(0)
    }
}

fn watchers() -> &'static Mutex<BTreeMap<VnodeKey, Vec<Weak<Inotify>>>> {
    WATCHERS.call_once(|| Mutex::new(BTreeMap::new()))
}

fn validate_mask(mask: u32) -> Result<()> {
    if mask & IN_ALL_EVENTS == 0 || mask & !(IN_ALL_EVENTS | IN_CONTROL_FLAGS) != 0 {
        return Err(Errno::Invalid);
    }
    Ok(())
}

fn queue_record(
    state: &mut InotifyState,
    watch_descriptor: i32,
    mask: u32,
    cookie: u32,
    name: Option<&[u8]>,
) {
    let name_length = name.map_or(0, |name| name.len().saturating_add(1));
    let padded_name_length = name_length.saturating_add(3) & !3;
    let record_length = EVENT_HEADER_SIZE.saturating_add(padded_name_length);
    if state.queued_bytes.saturating_add(record_length) > MAX_QUEUE_BYTES {
        if state.overflowed {
            return;
        }
        state.overflowed = true;
        let overflow = encode_record(-1, IN_Q_OVERFLOW, 0, None);
        state.queued_bytes = state.queued_bytes.saturating_add(overflow.len());
        state.queue.push_back(overflow);
        return;
    }
    let record = encode_record(watch_descriptor, mask, cookie, name);
    state.queued_bytes += record.len();
    state.queue.push_back(record);
}

fn encode_record(watch_descriptor: i32, mask: u32, cookie: u32, name: Option<&[u8]>) -> Box<[u8]> {
    let name_length = name.map_or(0, |name| name.len().saturating_add(1));
    let padded_name_length = name_length.saturating_add(3) & !3;
    let mut record = alloc::vec![0u8; EVENT_HEADER_SIZE + padded_name_length];
    record[0..4].copy_from_slice(&watch_descriptor.to_ne_bytes());
    record[4..8].copy_from_slice(&mask.to_ne_bytes());
    record[8..12].copy_from_slice(&cookie.to_ne_bytes());
    record[12..16].copy_from_slice(&(padded_name_length as u32).to_ne_bytes());
    if let Some(name) = name {
        record[16..16 + name.len()].copy_from_slice(name);
    }
    record.into_boxed_slice()
}

fn event_mask(record: &[u8]) -> u32 {
    u32::from_ne_bytes(record[4..8].try_into().expect("inotify mask width"))
}
