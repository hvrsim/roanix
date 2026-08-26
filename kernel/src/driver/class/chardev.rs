//! Character and block device nodes.
//!
//! This is the bridge between a driver's C operation table and the device
//! filesystem. It owns the translation in both directions: kernel buffers and
//! results become pointers and status codes on the way in, and driver status
//! codes become filesystem errors on the way out.
//!
//! Each node keeps a reference to the module that created it and pins that
//! module for the duration of every callback, so a driver's code cannot be
//! unloaded while userspace is inside one of its operations.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use core::{
    ffi::c_void,
    mem::{MaybeUninit, size_of},
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{
    fs::{
        self, Error as FsError, IoctlContext, PollEvents, Result as FsResult,
        provider::{self, FsTerminalState},
    },
    mem::{IoSink, IoSource},
    sys::{event::Event, sync::Mutex},
};

use super::super::{
    core::{
        device::Device,
        module::{self, Module, ModuleLease},
    },
    error::{self, Error, Result},
    obj::{ObjHeader, ObjKind},
};

/// Node kinds a driver may create.
pub mod kind {
    /// Byte-stream character device.
    pub const CHARACTER: u32 = 1;
    /// Random-access block device.
    pub const BLOCK: u32 = 2;
}

/// Stable identifier for a node within the devfs namespace.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DevNodeId(u64);

impl DevNodeId {
    pub(crate) const fn from_raw(value: u64) -> Self {
        Self(value)
    }

    /// Returns the raw namespace identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// The operation surface a device node implementation provides.
///
/// Kernel-native nodes implement this trait directly; loadable modules
/// instead supply a [`NodeOps`] C table, which the adapter below dispatches.
pub trait DeviceNodeOps: Send + Sync {
    /// Byte offset reads start from when the node has no stored offset.
    fn initial_offset(&self, _file_context: usize, _flags: u32) -> FsResult<u64> {
        Ok(0)
    }

    /// Creates per-open state and returns its opaque identifier.
    fn open(&self, _flags: u32) -> FsResult<usize> {
        Ok(0)
    }

    /// Releases state returned by [`Self::open`].
    fn close(&self, _file_context: usize, _flags: u32) {}

    /// Reads bytes without per-open state or flags.
    fn read_at(&self, _offset: u64, _sink: &mut IoSink<'_>) -> FsResult<usize> {
        Err(FsError::Unsupported)
    }

    /// Writes bytes without per-open state or flags.
    fn write_at(&self, _offset: u64, _source: &IoSource<'_>) -> FsResult<usize> {
        Err(FsError::Unsupported)
    }

    /// Reads bytes using per-open state and open flags.
    fn read_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        sink: &mut IoSink<'_>,
        flags: u32,
    ) -> FsResult<usize>;

    /// Writes bytes using per-open state and open flags.
    fn write_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        source: &IoSource<'_>,
        flags: u32,
    ) -> FsResult<usize>;

    /// Reports the subset of requested events that are immediately ready.
    fn poll(
        &self,
        _file_context: usize,
        _offset: u64,
        _events: PollEvents,
        _flags: u32,
    ) -> FsResult<PollEvents> {
        Ok(PollEvents::empty())
    }

    /// Appends waitable events and returns whether the request is supported.
    fn poll_events<'a>(
        &'a self,
        _file_context: usize,
        _events: PollEvents,
        _output: &mut Vec<&'a Event>,
    ) -> bool {
        false
    }

    /// Returns terminal job-control state when this node is a terminal.
    fn terminal_state(&self) -> Option<crate::fs::vnode::TerminalState> {
        None
    }

    /// Returns the logical size in bytes.
    fn size(&self) -> u64 {
        0
    }

    /// Flushes pending state to the backing device.
    fn sync(&self) -> FsResult<()> {
        Ok(())
    }

    /// Performs a device-specific control operation.
    fn ioctl(
        &self,
        file_context: usize,
        context: IoctlContext,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> FsResult<u64>;
}

/// Operations a driver implements for a device node.
///
/// Every entry is optional except that a node with neither `read` nor `write`
/// is of little use. Unimplemented operations report "unsupported" to
/// userspace.
#[repr(C)]
pub struct NodeOps {
    /// Size of this table, allowing later revisions to append entries.
    pub size: u32,
    /// Context passed to every callback.
    pub context: *mut c_void,
    /// Creates per-open state and returns an opaque handle.
    pub open: Option<unsafe extern "C" fn(*mut c_void, u32, *mut usize) -> i32>,
    /// Releases per-open state.
    pub close: Option<unsafe extern "C" fn(*mut c_void, usize, u32)>,
    /// Returns the initial file offset for a new open.
    pub initial_offset: Option<unsafe extern "C" fn(*mut c_void, usize, u32) -> i64>,
    /// Reads bytes. Negative results are status codes.
    pub read: Option<unsafe extern "C" fn(*mut c_void, usize, u64, *mut u8, usize, u32) -> i64>,
    /// Writes bytes. Negative results are status codes.
    pub write: Option<unsafe extern "C" fn(*mut c_void, usize, u64, *const u8, usize, u32) -> i64>,
    /// Returns the device's logical size in bytes.
    pub size_bytes: Option<unsafe extern "C" fn(*mut c_void) -> u64>,
    /// Flushes pending state to hardware.
    pub sync: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Reports which of the requested events are ready.
    pub poll: Option<unsafe extern "C" fn(*mut c_void, usize, u64, u16, u32) -> i64>,
    /// Performs a device-specific control operation.
    pub ioctl: Option<
        unsafe extern "C" fn(
            *mut c_void,
            usize,
            *const IoctlIdentity,
            u64,
            u64,
            *mut u8,
            usize,
        ) -> i64,
    >,
    /// Returns an event signalled while the device is readable.
    pub readable_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
    /// Returns an event signalled while the device is writable.
    pub writable_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
    /// Returns an event signalled after the device hangs up.
    pub hangup_event: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
    /// Appended in ABI minor 1; legacy tables leave it absent.
    pub terminal_state: Option<unsafe extern "C" fn(*mut c_void, *mut FsTerminalState) -> i32>,
}

/// Caller identity supplied to a control operation.
#[repr(C)]
pub struct IoctlIdentity {
    /// Calling process identifier.
    pub process: u64,
    /// Calling process group.
    pub group: i32,
    /// Calling session.
    pub session: i32,
    /// Whether the caller leads its session.
    pub session_leader: u8,
}

/// A device node backed by a driver's operation table.
pub struct Node {
    ops: NodeOps,
    owner: Option<Arc<Module>>,
    name: Box<str>,
    devfs: Mutex<Option<DevNodeId>>,
    opens: AtomicU64,
}

// SAFETY: the operation table and context belong to the owning module, which
// cannot unload until the node is removed.
unsafe impl Send for Node {}
// SAFETY: the ABI requires node operations to tolerate concurrent invocation
// from several file descriptions.
unsafe impl Sync for Node {}

impl Node {
    /// Returns the node name within its directory.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the number of open file descriptions.
    pub fn opens(&self) -> u64 {
        self.opens.load(Ordering::Relaxed)
    }

    fn pin(&self) -> FsResult<module::ModuleGuard> {
        module::pin_owner(self.owner.as_ref(), false).map_err(|_| FsError::Io)
    }

    fn pin_cleanup(&self) -> Option<module::ModuleGuard> {
        module::pin_owner(self.owner.as_ref(), true).ok()
    }

    fn event(
        &self,
        callback: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
        file_context: usize,
    ) -> Option<&'static Event> {
        let callback = callback?;
        let _pin = self.pin().ok()?;
        // SAFETY: registration validated the callback.
        let pointer = unsafe { callback(self.ops.context, file_context) };
        super::super::abi::events::resolve(pointer)
    }

    /// The terminal job-control state of the node, when it implements the request.
    fn terminal_state_value(&self) -> Option<crate::fs::vnode::TerminalState> {
        let callback = self.ops.terminal_state?;
        let mut state = FsTerminalState {
            session: 0,
            foreground_group: 0,
            stop_background_output: 0,
            reserved: [0; 3],
        };
        // SAFETY: the record outlives this immediate call.
        let status = unsafe { callback(self.ops.context, &raw mut state) };
        if status != error::STATUS_OK {
            return None;
        }
        Some(crate::fs::vnode::TerminalState {
            session: state.session,
            foreground_group: state.foreground_group,
            stop_background_output: state.stop_background_output != 0,
        })
    }
}

pub(crate) const NODE_OPS_LEGACY_SIZE: usize = 112;

fn signed_result(value: i64) -> FsResult<usize> {
    if value < 0 {
        Err(Error::from_status(value as i32).into())
    } else {
        Ok(value as usize)
    }
}

/// Runs `transfer` once per page-bounded window of `sink`.
fn read_windows<F>(
    sink: &mut IoSink<'_>,
    offset: u64,
    flags: u32,
    mut transfer: F,
) -> FsResult<usize>
where
    F: FnMut(u64, &mut [u8], u32) -> FsResult<usize>,
{
    let total = sink.len();
    let mut done = 0;
    while done < total {
        let window_flags = if done == 0 {
            flags
        } else {
            flags | fs::OpenFlags::NONBLOCK.bits()
        };
        let mut window = sink
            .window(done, total - done)
            .map_err(|_| FsError::InvalidArgument)?;
        if window.is_empty() {
            break;
        }
        let capacity = window.len();
        let count = match transfer(
            offset.saturating_add(done as u64),
            &mut window,
            window_flags,
        ) {
            Ok(count) => count,
            Err(_) if done != 0 => break,
            Err(error) => return Err(error),
        };
        if count > capacity {
            return Err(FsError::Io);
        }
        done += count;
        if count < capacity {
            break;
        }
    }
    Ok(done)
}

/// Runs `transfer` once per page-bounded window of `source`.
fn write_windows<F>(
    source: &IoSource<'_>,
    offset: u64,
    flags: u32,
    mut transfer: F,
) -> FsResult<usize>
where
    F: FnMut(u64, &[u8], u32) -> FsResult<usize>,
{
    let total = source.len();
    let mut done = 0;
    while done < total {
        let window_flags = if done == 0 {
            flags
        } else {
            flags | fs::OpenFlags::NONBLOCK.bits()
        };
        let window = source
            .window(done, total - done)
            .map_err(|_| FsError::InvalidArgument)?;
        if window.is_empty() {
            break;
        }
        let capacity = window.len();
        let count = match transfer(offset.saturating_add(done as u64), &window, window_flags) {
            Ok(count) => count,
            Err(_) if done != 0 => break,
            Err(error) => return Err(error),
        };
        if count > capacity {
            return Err(FsError::Io);
        }
        done += count;
        if count < capacity {
            break;
        }
    }
    Ok(done)
}

impl DeviceNodeOps for Node {
    fn initial_offset(&self, file_context: usize, flags: u32) -> FsResult<u64> {
        let Some(callback) = self.ops.initial_offset else {
            return Ok(0);
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback.
        let value = unsafe { callback(self.ops.context, file_context, flags) };
        if value < 0 {
            return Err(Error::from_status(value as i32).into());
        }
        Ok(value as u64)
    }

    fn open(&self, flags: u32) -> FsResult<usize> {
        let Some(callback) = self.ops.open else {
            self.opens.fetch_add(1, Ordering::Relaxed);
            return Ok(0);
        };
        let _pin = self.pin()?;
        let mut file_context = 0usize;
        // SAFETY: registration validated the callback and the output pointer
        // addresses local storage that outlives the call.
        let status = unsafe { callback(self.ops.context, flags, &raw mut file_context) };
        error::from_status(status).map_err(FsError::from)?;
        self.opens.fetch_add(1, Ordering::Relaxed);
        Ok(file_context)
    }

    fn close(&self, file_context: usize, flags: u32) {
        self.opens.fetch_sub(1, Ordering::Relaxed);
        let Some(callback) = self.ops.close else {
            return;
        };
        let Some(_pin) = self.pin_cleanup() else {
            return;
        };
        // SAFETY: registration validated the callback and `file_context` came
        // from the matching open.
        unsafe { callback(self.ops.context, file_context, flags) };
    }

    fn read_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        sink: &mut IoSink<'_>,
        flags: u32,
    ) -> FsResult<usize> {
        let callback = self.ops.read.ok_or(FsError::Unsupported)?;
        let _pin = self.pin()?;
        read_windows(sink, offset, flags, |offset, window, flags| {
            // SAFETY: registration validated the callback, and the window
            // pointer and length describe memory owned by the caller for the
            // duration of this call.
            let value = unsafe {
                callback(
                    self.ops.context,
                    file_context,
                    offset,
                    window.as_mut_ptr(),
                    window.len(),
                    flags,
                )
            };
            signed_result(value)
        })
    }

    fn read_at(&self, offset: u64, sink: &mut IoSink<'_>) -> FsResult<usize> {
        self.read_at_with_flags(0, offset, sink, 0)
    }

    fn write_at_with_flags(
        &self,
        file_context: usize,
        offset: u64,
        source: &IoSource<'_>,
        flags: u32,
    ) -> FsResult<usize> {
        let callback = self.ops.write.ok_or(FsError::Unsupported)?;
        let _pin = self.pin()?;
        write_windows(source, offset, flags, |offset, window, flags| {
            // SAFETY: registration validated the callback, and the window
            // pointer and length describe memory owned by the caller for the
            // duration of this call.
            let value = unsafe {
                callback(
                    self.ops.context,
                    file_context,
                    offset,
                    window.as_ptr(),
                    window.len(),
                    flags,
                )
            };
            signed_result(value)
        })
    }

    fn write_at(&self, offset: u64, source: &IoSource<'_>) -> FsResult<usize> {
        self.write_at_with_flags(0, offset, source, 0)
    }

    fn poll(
        &self,
        file_context: usize,
        offset: u64,
        events: PollEvents,
        flags: u32,
    ) -> FsResult<PollEvents> {
        let Some(callback) = self.ops.poll else {
            return Ok(PollEvents::empty());
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback.
        let value =
            unsafe { callback(self.ops.context, file_context, offset, events.bits(), flags) };
        if value < 0 {
            return Err(Error::from_status(value as i32).into());
        }
        Ok(PollEvents::from_bits_truncate(value as u16))
    }

    fn poll_events<'a>(
        &'a self,
        file_context: usize,
        events: PollEvents,
        output: &mut Vec<&'a Event>,
    ) -> bool {
        let mut registered = false;
        if events.intersects(PollEvents::IN | PollEvents::RDNORM)
            && let Some(event) = self.event(self.ops.readable_event, file_context)
        {
            output.push(event);
            registered = true;
        }
        if events.intersects(PollEvents::OUT | PollEvents::WRNORM)
            && let Some(event) = self.event(self.ops.writable_event, file_context)
        {
            output.push(event);
            registered = true;
        }
        if let Some(event) = self.event(self.ops.hangup_event, file_context) {
            output.push(event);
            registered = true;
        }
        registered
    }

    fn size(&self) -> u64 {
        let Some(callback) = self.ops.size_bytes else {
            return 0;
        };
        let Ok(_pin) = self.pin() else {
            return 0;
        };
        // SAFETY: registration validated the callback.
        unsafe { callback(self.ops.context) }
    }

    fn sync(&self) -> FsResult<()> {
        let Some(callback) = self.ops.sync else {
            return Ok(());
        };
        let _pin = self.pin()?;
        // SAFETY: registration validated the callback.
        error::from_status(unsafe { callback(self.ops.context) }).map_err(FsError::from)
    }

    fn ioctl(
        &self,
        file_context: usize,
        context: IoctlContext,
        request: u64,
        value: u64,
        argument: &mut [u8],
    ) -> FsResult<u64> {
        let callback = self.ops.ioctl.ok_or(FsError::NotTty)?;
        let _pin = self.pin()?;
        let identity = IoctlIdentity {
            process: context.process_id as u64,
            group: context.process_group,
            session: context.session_id,
            session_leader: u8::from(context.is_session_leader),
        };
        // SAFETY: registration validated the callback, and both the identity
        // and argument buffer outlive the call.
        let result = unsafe {
            callback(
                self.ops.context,
                file_context,
                &raw const identity,
                request,
                value,
                argument.as_mut_ptr(),
                argument.len(),
            )
        };
        if result < 0 {
            return Err(Error::from_status(result as i32).into());
        }
        Ok(result as u64)
    }

    fn terminal_state(&self) -> Option<crate::fs::vnode::TerminalState> {
        self.terminal_state_value()
    }
}

/// One driver-backed device endpoint retained by the devfs provider.
///
/// The receipt owns a module lease rather than taking transient callback
/// pins. The provider retains this receipt for every live vnode and open file,
/// which keeps the hardware driver's code resident through all callbacks.
struct Endpoint {
    header: ObjHeader,
    operations: Arc<dyn DeviceNodeOps>,
    _lease: ModuleLease,
}

// SAFETY: `DeviceNodeOps` is `Send + Sync`, and the immutable lease has no
// interior mutability.
unsafe impl Send for Endpoint {}
// SAFETY: the same immutable fields make shared endpoint references safe.
unsafe impl Sync for Endpoint {}

pub(crate) fn endpoint_receipt(
    owner: Option<&Arc<Module>>,
    name: &str,
    operations: Arc<dyn DeviceNodeOps>,
) -> Result<*mut c_void> {
    let header = ObjHeader::new_with(
        ObjKind::DeviceNode,
        Some(name),
        owner.map(|module| module.id().get()),
        None,
    );
    let lease = match owner {
        Some(module) => module::lease(module)?,
        None => ModuleLease::none(),
    };
    let endpoint = Arc::new(Endpoint {
        header,
        operations,
        _lease: lease,
    });
    endpoint.header.register()?;
    // SAFETY: the receipt leaks one strong reference; release consumes it.
    Ok(Arc::into_raw(endpoint).cast_mut().cast())
}

unsafe fn borrow_endpoint<'a>(receipt: *mut c_void) -> Result<&'a Endpoint> {
    if receipt.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the caller guarantees a live endpoint receipt for 'a.
    Ok(unsafe { &*(receipt.cast::<Endpoint>()) })
}

pub(crate) unsafe fn endpoint_open(receipt: *mut c_void, flags: u32) -> Result<usize> {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    endpoint.operations.open(flags).map_err(Error::from)
}

pub(crate) unsafe fn endpoint_close(receipt: *mut c_void, file: usize, flags: u32) {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = match unsafe { borrow_endpoint(receipt) } {
        Ok(endpoint) => endpoint,
        Err(_) => return,
    };
    endpoint.operations.close(file, flags);
}

pub(crate) unsafe fn endpoint_initial_offset(
    receipt: *mut c_void,
    file: usize,
    flags: u32,
) -> Result<u64> {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    endpoint
        .operations
        .initial_offset(file, flags)
        .map_err(Error::from)
}

pub(crate) unsafe fn endpoint_read(
    receipt: *mut c_void,
    file: usize,
    offset: u64,
    buffer: *mut u8,
    length: usize,
    flags: u32,
) -> Result<i64> {
    use core::slice;
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    let data = if length == 0 {
        &mut []
    } else {
        if buffer.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the non-null ABI buffer is writable for `length` bytes.
        unsafe { slice::from_raw_parts_mut(buffer, length) }
    };
    let read = endpoint
        .operations
        .read_at_with_flags(file, offset, &mut IoSink::kernel(data), flags)
        .map_err(Error::from)?;
    i64::try_from(read).map_err(|_| Error::InvalidArgument)
}

pub(crate) unsafe fn endpoint_write(
    receipt: *mut c_void,
    file: usize,
    offset: u64,
    buffer: *const u8,
    length: usize,
    flags: u32,
) -> Result<i64> {
    use core::slice;
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    let data = if length == 0 {
        &[]
    } else {
        if buffer.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the non-null ABI buffer is readable for `length` bytes.
        unsafe { slice::from_raw_parts(buffer, length) }
    };
    let written = endpoint
        .operations
        .write_at_with_flags(file, offset, &IoSource::kernel(data), flags)
        .map_err(Error::from)?;
    i64::try_from(written).map_err(|_| Error::InvalidArgument)
}

pub(crate) unsafe fn endpoint_size(receipt: *mut c_void) -> u64 {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = match unsafe { borrow_endpoint(receipt) } {
        Ok(endpoint) => endpoint,
        Err(_) => return 0,
    };
    endpoint.operations.size()
}

pub(crate) unsafe fn endpoint_sync(receipt: *mut c_void) -> Result<()> {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    endpoint.operations.sync().map_err(Error::from)
}

pub(crate) unsafe fn endpoint_poll(
    receipt: *mut c_void,
    file: usize,
    offset: u64,
    events: u16,
    flags: u32,
) -> Result<u16> {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    Ok(endpoint
        .operations
        .poll(file, offset, PollEvents::from_bits_truncate(events), flags)
        .map_err(Error::from)?
        .bits())
}

pub(crate) unsafe fn endpoint_event(receipt: *mut c_void, file: usize, selector: u32) -> usize {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = match unsafe { borrow_endpoint(receipt) } {
        Ok(endpoint) => endpoint,
        Err(_) => return 0,
    };
    let requested = match selector {
        provider::DEVFS_EVENT_READABLE => PollEvents::IN | PollEvents::RDNORM,
        provider::DEVFS_EVENT_WRITABLE => PollEvents::OUT | PollEvents::WRNORM,
        provider::DEVFS_EVENT_HANGUP => PollEvents::HUP,
        _ => return 0,
    };
    let mut events = Vec::new();
    if !endpoint
        .operations
        .poll_events(file, requested, &mut events)
    {
        return 0;
    }
    events
        .first()
        .map_or(0, |event| (*event as *const Event).cast::<()>() as usize)
}

pub(crate) unsafe fn endpoint_terminal_state(receipt: *mut c_void) -> Result<FsTerminalState> {
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    let state = endpoint.operations.terminal_state().ok_or(Error::NotTty)?;
    Ok(FsTerminalState {
        session: state.session,
        foreground_group: state.foreground_group,
        stop_background_output: state.stop_background_output as u8,
        reserved: [0; 3],
    })
}

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe fn endpoint_ioctl(
    receipt: *mut c_void,
    file: usize,
    process: u64,
    group: i32,
    session: i32,
    session_leader: bool,
    request: u64,
    value: u64,
    argument: *mut u8,
    length: usize,
) -> Result<u64> {
    use core::slice;
    // SAFETY: the caller guarantees a live endpoint receipt.
    let endpoint = unsafe { borrow_endpoint(receipt) }?;
    let data = if length == 0 {
        &mut []
    } else {
        if argument.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the non-null ABI buffer is writable for `length` bytes.
        unsafe { slice::from_raw_parts_mut(argument, length) }
    };
    let context = IoctlContext {
        process_id: process as usize,
        process_group: group,
        session_id: session,
        is_session_leader: session_leader,
    };
    endpoint
        .operations
        .ioctl(file, context, request, value, data)
        .map_err(Error::from)
}

/// Releases one endpoint receipt consumed by devfs.
///
/// # Safety
///
/// `receipt` must be an owned endpoint receipt consumed exactly once.
pub unsafe fn endpoint_release(receipt: *mut c_void) {
    if !receipt.is_null() {
        // SAFETY: forwarded from this function's contract.
        drop(unsafe { Arc::from_raw(receipt.cast::<Endpoint>()) });
    }
}

/// Copies a size-prefixed node table without reading past an old ABI table.
///
/// # Safety
///
/// `operations` must point to a readable table containing at least its
/// size-prefixed legacy prefix.
pub(crate) unsafe fn copy_node_ops(operations: *const NodeOps) -> Result<NodeOps> {
    if operations.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: only the always-present prefix field is read before checking.
    let declared = unsafe { (*operations).size as usize };
    if declared < NODE_OPS_LEGACY_SIZE {
        return Err(Error::InvalidArgument);
    }
    let copied = declared.min(size_of::<NodeOps>());
    let mut table = MaybeUninit::<NodeOps>::zeroed();
    // SAFETY: `copied` is no larger than one full destination table and no
    // larger than what the caller declares readable.
    unsafe {
        core::ptr::copy_nonoverlapping(
            operations.cast::<u8>(),
            table.as_mut_ptr().cast::<u8>(),
            copied,
        );
        Ok(table.assume_init())
    }
}

/// Returns the identifier of the devfs root.
pub fn root() -> Result<DevNodeId> {
    Ok(DevNodeId::from_raw(provider::devfs_root()?))
}

fn owner_id(owner: Option<&Arc<Module>>) -> u64 {
    owner.map_or(0, |module| module.id().get())
}

/// Creates a directory inside the devfs namespace.
pub fn create_directory(
    owner: Option<&Arc<Module>>,
    parent: DevNodeId,
    name: &str,
    mode: u16,
) -> Result<DevNodeId> {
    match provider::devfs_mkdir(owner_id(owner), parent.get(), name.as_bytes(), mode) {
        Ok(id) => Ok(DevNodeId::from_raw(id)),
        Err(error) => Err(error),
    }
}

/// Creates a device node backed by a driver's operation table.
///
/// # Safety
///
/// Every callback in `ops` must follow the node ABI and stay executable until
/// the node is removed.
pub unsafe fn create_node(
    owner: Option<&Arc<Module>>,
    _device: Option<&Arc<Device>>,
    parent: DevNodeId,
    name: &str,
    node_kind: u32,
    mode: u16,
    ops: NodeOps,
) -> Result<(DevNodeId, Arc<Node>)> {
    if (ops.size as usize) < NODE_OPS_LEGACY_SIZE {
        return Err(Error::InvalidArgument);
    }

    let devfs_kind = match node_kind {
        kind::CHARACTER => provider::FS_KIND_CHARACTER_DEVICE,
        kind::BLOCK => provider::FS_KIND_BLOCK_DEVICE,
        _ => return Err(Error::InvalidArgument),
    };
    let node = Arc::new(Node {
        ops,
        owner: owner.cloned(),
        name: String::from(name).into_boxed_str(),
        devfs: Mutex::new(None),
        opens: AtomicU64::new(0),
    });
    let operations: Arc<dyn DeviceNodeOps> = node.clone();
    let endpoint = endpoint_receipt(owner, name, operations)?;
    let id = match provider::devfs_create(
        owner_id(owner),
        parent.get(),
        name.as_bytes(),
        devfs_kind,
        mode,
        endpoint,
    ) {
        Ok(id) => id,
        Err(error) => {
            // SAFETY: creation failed and the broker did not consume the
            // endpoint receipt.
            unsafe { endpoint_release(endpoint) };
            return Err(error);
        }
    };
    let id = DevNodeId::from_raw(id);
    *node.devfs.lock() = Some(id);
    Ok((id, node))
}

/// Publishes an existing filesystem-level device implementation.
///
/// Used by kernel-side classes such as the terminal layer, which supply a
/// Rust implementation rather than a C operation table.
pub fn create_native_node(
    owner: Option<&Arc<Module>>,
    parent: DevNodeId,
    name: &str,
    node_kind: u32,
    mode: u16,
    ops: Arc<dyn DeviceNodeOps>,
) -> Result<DevNodeId> {
    let devfs_kind = match node_kind {
        kind::CHARACTER => provider::FS_KIND_CHARACTER_DEVICE,
        kind::BLOCK => provider::FS_KIND_BLOCK_DEVICE,
        _ => return Err(Error::InvalidArgument),
    };
    let endpoint = endpoint_receipt(owner, name, ops)?;
    match provider::devfs_create(
        owner_id(owner),
        parent.get(),
        name.as_bytes(),
        devfs_kind,
        mode,
        endpoint,
    ) {
        Ok(id) => Ok(DevNodeId::from_raw(id)),
        Err(error) => {
            // SAFETY: a failed create leaves ownership with the caller.
            unsafe { endpoint_release(endpoint) };
            Err(error)
        }
    }
}

/// Removes a node created by this module.
pub fn remove(owner: Option<&Arc<Module>>, node: DevNodeId) -> Result<()> {
    provider::devfs_remove(owner_id(owner), node.get())
}

/// Removes every node created by `module`.
pub fn remove_module_nodes(module: &Arc<Module>) {
    let _ = provider::devfs_remove_owner(module.id().get(), true);
}

/// Removes nodes only if none are busy; returns whether any remain.
pub fn try_remove_module_nodes(module: &Arc<Module>) -> Result<()> {
    provider::devfs_remove_owner(module.id().get(), true)
}

/// Resolves an absolute devfs path to a node identifier.
pub fn lookup(path: &str) -> Result<DevNodeId> {
    Ok(DevNodeId::from_raw(provider::devfs_lookup(
        path.as_bytes(),
    )?))
}

/// Returns whether the device filesystem is mounted.
pub fn available() -> bool {
    provider::devfs_root().is_ok()
}

/// Returns the names of the entries directly below `parent`.
pub fn children(parent: DevNodeId) -> Result<Vec<Box<str>>> {
    Ok(provider::devfs_children(parent.get())?
        .into_iter()
        .map(|entry| {
            String::from_utf8_lossy(&entry.name[..entry.name_length as usize])
                .to_string()
                .into_boxed_str()
        })
        .collect())
}
