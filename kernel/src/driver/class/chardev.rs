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
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{
    fs::{
        self, Error as FsError, IoctlContext, PollEvents, Result as FsResult,
        devtempfs::{
            self, DevNodeId, DeviceNodeKind, DeviceNodeOps, Devtempfs, OwnerId, read_windows,
            write_windows,
        },
    },
    mem::{IoSink, IoSource},
    sys::{event::Event, sync::Mutex},
};

use super::super::{
    core::{
        device::Device,
        module::{self, Module},
    },
    error::{self, Error, Result},
};

/// Node kinds a driver may create.
pub mod kind {
    /// Byte-stream character device.
    pub const CHARACTER: u32 = 1;
    /// Random-access block device.
    pub const BLOCK: u32 = 2;
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

    fn event(&self, callback: Option<unsafe extern "C" fn(*mut c_void, usize) -> usize>,
        file_context: usize) -> Option<&'static Event> {
        let callback = callback?;
        let _pin = self.pin().ok()?;
        // SAFETY: registration validated the callback.
        let pointer = unsafe { callback(self.ops.context, file_context) };
        super::super::abi::events::resolve(pointer)
    }
}

fn signed_result(value: i64) -> FsResult<usize> {
    if value < 0 {
        Err(Error::from_status(value as i32).into())
    } else {
        Ok(value as usize)
    }
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
        let value = unsafe {
            callback(
                self.ops.context,
                file_context,
                offset,
                events.bits(),
                flags,
            )
        };
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
}

fn filesystem() -> Result<&'static Arc<Devtempfs>> {
    devtempfs::global().map_err(Error::from)
}

/// Returns the identifier of the device filesystem root.
pub fn root() -> Result<DevNodeId> {
    Ok(filesystem()?.root_id())
}

fn owner_token(owner: Option<&Arc<Module>>) -> OwnerId {
    owner.map_or(OwnerId::KERNEL, |module| OwnerId::new(module.id().get()))
}

/// Creates a directory below the device filesystem root.
pub fn create_directory(
    owner: Option<&Arc<Module>>,
    parent: DevNodeId,
    name: &str,
    mode: u16,
) -> Result<DevNodeId> {
    Ok(filesystem()?.create_dir(owner_token(owner), parent, name.as_bytes(), mode)?)
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
    if (ops.size as usize) < size_of::<NodeOps>() {
        return Err(Error::InvalidArgument);
    }
    let devfs_kind = match node_kind {
        kind::CHARACTER => DeviceNodeKind::Character,
        kind::BLOCK => DeviceNodeKind::Block,
        _ => return Err(Error::InvalidArgument),
    };
    let node = Arc::new(Node {
        ops,
        owner: owner.cloned(),
        name: String::from(name).into_boxed_str(),
        devfs: Mutex::new(None),
        opens: AtomicU64::new(0),
    });
    let id = filesystem()?.create_device(
        owner_token(owner),
        parent,
        name.as_bytes(),
        devfs_kind,
        mode,
        node.clone(),
    )?;
    *node.devfs.lock() = Some(id);
    Ok((id, node))
}

/// Publishes an existing filesystem-level device implementation.
///
/// Used by kernel-side classes such as the terminal layer, which supply a Rust
/// implementation rather than a C operation table.
pub fn create_native_node(
    owner: Option<&Arc<Module>>,
    parent: DevNodeId,
    name: &str,
    node_kind: u32,
    mode: u16,
    ops: Arc<dyn DeviceNodeOps>,
) -> Result<DevNodeId> {
    let devfs_kind = match node_kind {
        kind::CHARACTER => DeviceNodeKind::Character,
        kind::BLOCK => DeviceNodeKind::Block,
        _ => return Err(Error::InvalidArgument),
    };
    Ok(filesystem()?.create_device(
        owner_token(owner),
        parent,
        name.as_bytes(),
        devfs_kind,
        mode,
        ops,
    )?)
}

/// Removes a node created by this module.
pub fn remove(owner: Option<&Arc<Module>>, node: DevNodeId) -> Result<()> {
    Ok(filesystem()?.remove_node(owner_token(owner), node)?)
}

/// Removes every node created by `module`.
pub fn remove_module_nodes(module: &Arc<Module>) {
    let Ok(filesystem) = filesystem() else {
        return;
    };
    let _ = filesystem.force_remove_owner(OwnerId::new(module.id().get()));
}

/// Resolves an absolute device-filesystem path to a node identifier.
pub fn lookup(path: &str) -> Result<DevNodeId> {
    let mut current = root()?;
    for component in path.split('/').filter(|part| !part.is_empty()) {
        current = filesystem()?.lookup_child(current, component.as_bytes())?;
    }
    Ok(current)
}

/// Returns whether the device filesystem is mounted.
pub fn available() -> bool {
    fs::devtempfs::global().is_ok()
}

/// Returns the names of the entries directly below `parent`.
pub fn children(parent: DevNodeId) -> Result<Vec<Box<str>>> {
    Ok(filesystem()?
        .child_names(parent)?
        .into_iter()
        .map(|name| String::from_utf8_lossy(&name).to_string().into_boxed_str())
        .collect())
}
