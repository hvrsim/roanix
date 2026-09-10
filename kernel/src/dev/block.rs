//! Kernel-owned block layer.
//!
//! Hardware drivers expose only logical-block operations.  This layer owns
//! byte-range adaptation, devfs publication, partition children and lifetime
//! management, keeping filesystem policy out of loadable modules.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    string::ToString,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    ffi::c_void,
    mem::{MaybeUninit, size_of},
    sync::atomic::{AtomicU32, AtomicU64, Ordering},
};

use crate::{
    driver::{
        core::module::{self, Module},
        error::{self, Error, Result},
        obj::{ObjHeader, ObjState, Object, framework_object},
    },
    fs::{Error as FsError, PollEvents, Result as FsResult, devfs::DeviceNodeOps},
    mem::{IoSink, IoSource},
    sys::sync::{Mutex, Once},
};

use super::node::{self, DevNodeId};

const IO_BATCH_BYTES: usize = 128 * 1024;

const BLOCK_OPS_SIZE: usize = 64;
const MAX_LOGICAL_BLOCK_SIZE: u32 = 1024 * 1024;

/// Size-prefixed hardware block operation table used by ABI v2 modules.
#[repr(C)]
#[derive(Clone, Copy)]
#[allow(missing_docs)]
pub struct BlockOps {
    pub size: u32,
    pub flags: u32,
    pub context: *mut c_void,
    pub logical_block_size: u32,
    pub max_transfer_blocks: u32,
    pub block_count: u64,
    pub read: Option<unsafe extern "C" fn(*mut c_void, u64, u32, *mut u8) -> i32>,
    pub write: Option<unsafe extern "C" fn(*mut c_void, u64, u32, *const u8) -> i32>,
    pub flush: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    pub discard: Option<unsafe extern "C" fn(*mut c_void, u64, u32) -> i32>,
}

/// Exact-block backend implemented by hardware or a block transform.
#[allow(missing_docs)]
pub trait BlockBackend: Send + Sync {
    fn read_blocks(&self, lba: u64, blocks: u32, buffer: &mut [u8]) -> Result<()>;
    fn write_blocks(&self, _lba: u64, _blocks: u32, _buffer: &[u8]) -> Result<()> {
        Err(Error::ReadOnly)
    }
    fn flush(&self) -> Result<()> {
        Ok(())
    }
    fn discard(&self, _lba: u64, _blocks: u32) -> Result<()> {
        Err(Error::Unsupported)
    }
    fn writable(&self) -> bool {
        false
    }
}

struct ModuleBackend {
    owner: Arc<Module>,
    ops: BlockOps,
}

// SAFETY: the module ABI requires its opaque callback context to support
// concurrent calls. The retained module and callback pins keep code and data
// resident for every invocation.
unsafe impl Send for ModuleBackend {}
// SAFETY: as above.
unsafe impl Sync for ModuleBackend {}

impl ModuleBackend {
    fn pin(&self) -> Result<module::ModuleGuard> {
        module::pin_owner(Some(&self.owner), false)
    }
}

impl BlockBackend for ModuleBackend {
    fn read_blocks(&self, lba: u64, blocks: u32, buffer: &mut [u8]) -> Result<()> {
        let callback = self.ops.read.ok_or(Error::Unsupported)?;
        let _pin = self.pin()?;
        // SAFETY: registration copied and validated the callback table, and
        // `BlockDevice` verified that this slice has exactly the transfer size.
        error::from_status(unsafe { callback(self.ops.context, lba, blocks, buffer.as_mut_ptr()) })
    }

    fn write_blocks(&self, lba: u64, blocks: u32, buffer: &[u8]) -> Result<()> {
        let callback = self.ops.write.ok_or(Error::ReadOnly)?;
        let _pin = self.pin()?;
        // SAFETY: the callback and exact-size readable slice satisfy the ABI.
        error::from_status(unsafe { callback(self.ops.context, lba, blocks, buffer.as_ptr()) })
    }

    fn flush(&self) -> Result<()> {
        let Some(callback) = self.ops.flush else {
            return Ok(());
        };
        let _pin = self.pin()?;
        // SAFETY: the callback table remains live under the module pin.
        error::from_status(unsafe { callback(self.ops.context) })
    }

    fn discard(&self, lba: u64, blocks: u32) -> Result<()> {
        let callback = self.ops.discard.ok_or(Error::Unsupported)?;
        let _pin = self.pin()?;
        // SAFETY: the callback table remains live under the module pin.
        error::from_status(unsafe { callback(self.ops.context, lba, blocks) })
    }

    fn writable(&self) -> bool {
        self.ops.write.is_some()
    }
}

struct Registry {
    devices: Mutex<BTreeMap<Box<str>, Arc<BlockDevice>>>,
}

static REGISTRY: Once<Registry> = Once::new();

/// Initializes the block registry before driver modules can register devices.
pub fn init() {
    REGISTRY.call_once(|| Registry {
        devices: Mutex::new(BTreeMap::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// One kernel-managed block address space.
#[repr(C)]
pub struct BlockDevice {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    parent: Option<Arc<BlockDevice>>,
    children: Mutex<Vec<Weak<BlockDevice>>>,
    backend: Arc<dyn BlockBackend>,
    logical_block_size: u32,
    max_transfer_blocks: u32,
    block_count: u64,
    devfs_node: Mutex<Option<DevNodeId>>,
    claims: AtomicU64,
    scanned: AtomicU32,
}

framework_object!(BlockDevice, BlockDevice);

#[allow(missing_docs)]
impl BlockDevice {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub const fn logical_block_size(&self) -> u32 {
        self.logical_block_size
    }

    pub const fn block_count(&self) -> u64 {
        self.block_count
    }

    pub fn size_bytes(&self) -> u64 {
        self.block_count
            .saturating_mul(u64::from(self.logical_block_size))
    }

    pub fn writable(&self) -> bool {
        self.backend.writable()
    }

    fn check_transfer(&self, lba: u64, blocks: u32, length: usize) -> Result<()> {
        if blocks == 0 {
            return (length == 0).then_some(()).ok_or(Error::InvalidArgument);
        }
        let end = lba
            .checked_add(u64::from(blocks))
            .ok_or(Error::InvalidArgument)?;
        let expected = usize::try_from(u64::from(blocks) * u64::from(self.logical_block_size))
            .map_err(|_| Error::InvalidArgument)?;
        if end > self.block_count || length != expected {
            return Err(Error::InvalidArgument);
        }
        Ok(())
    }

    pub fn read_blocks(&self, lba: u64, blocks: u32, buffer: &mut [u8]) -> Result<()> {
        self.check_transfer(lba, blocks, buffer.len())?;
        self.transfer_chunks(lba, blocks, buffer, |backend, at, count, bytes| {
            backend.read_blocks(at, count, bytes)
        })
    }

    fn transfer_chunks<F>(
        &self,
        mut lba: u64,
        mut blocks: u32,
        buffer: &mut [u8],
        mut transfer: F,
    ) -> Result<()>
    where
        F: FnMut(&dyn BlockBackend, u64, u32, &mut [u8]) -> Result<()>,
    {
        let limit = if self.max_transfer_blocks == 0 {
            u32::MAX
        } else {
            self.max_transfer_blocks
        };
        let block_size = self.logical_block_size as usize;
        let mut offset = 0usize;
        while blocks != 0 {
            let count = blocks.min(limit);
            let bytes = count as usize * block_size;
            transfer(
                self.backend.as_ref(),
                lba,
                count,
                &mut buffer[offset..offset + bytes],
            )?;
            lba += u64::from(count);
            blocks -= count;
            offset += bytes;
        }
        Ok(())
    }

    pub fn write_blocks(&self, mut lba: u64, mut blocks: u32, buffer: &[u8]) -> Result<()> {
        self.check_transfer(lba, blocks, buffer.len())?;
        let limit = if self.max_transfer_blocks == 0 {
            u32::MAX
        } else {
            self.max_transfer_blocks
        };
        let block_size = self.logical_block_size as usize;
        let mut offset = 0usize;
        while blocks != 0 {
            let count = blocks.min(limit);
            let bytes = count as usize * block_size;
            self.backend
                .write_blocks(lba, count, &buffer[offset..offset + bytes])?;
            lba += u64::from(count);
            blocks -= count;
            offset += bytes;
        }
        Ok(())
    }

    pub fn flush(&self) -> Result<()> {
        self.backend.flush()
    }

    pub fn discard(&self, mut lba: u64, mut blocks: u32) -> Result<()> {
        self.check_transfer(
            lba,
            blocks,
            blocks as usize * self.logical_block_size as usize,
        )?;
        let limit = if self.max_transfer_blocks == 0 {
            u32::MAX
        } else {
            self.max_transfer_blocks
        };
        while blocks != 0 {
            let count = blocks.min(limit);
            self.backend.discard(lba, count)?;
            lba += u64::from(count);
            blocks -= count;
        }
        Ok(())
    }

    pub fn read_bytes(&self, offset: u64, buffer: &mut [u8]) -> Result<usize> {
        let available = self.size_bytes().saturating_sub(offset);
        let length = buffer
            .len()
            .min(usize::try_from(available).unwrap_or(usize::MAX));
        if length == 0 {
            return Ok(0);
        }
        let bs = self.logical_block_size as usize;
        let first = offset / bs as u64;
        let leading = offset as usize % bs;
        if leading == 0 && length % bs == 0 {
            self.read_blocks(first, (length / bs) as u32, &mut buffer[..length])?;
            return Ok(length);
        }
        let mut bounce = block_buffer(bs)?;
        let mut done = 0usize;
        let mut lba = first;
        if leading != 0 {
            self.read_blocks(lba, 1, &mut bounce)?;
            let take = length.min(bs - leading);
            buffer[..take].copy_from_slice(&bounce[leading..leading + take]);
            done += take;
            lba += 1;
        }
        let middle = (length - done) / bs * bs;
        if middle != 0 {
            self.read_blocks(lba, (middle / bs) as u32, &mut buffer[done..done + middle])?;
            done += middle;
            lba += (middle / bs) as u64;
        }
        if done != length {
            self.read_blocks(lba, 1, &mut bounce)?;
            let tail = length - done;
            buffer[done..length].copy_from_slice(&bounce[..tail]);
        }
        Ok(length)
    }

    /// Reads through a bounded transfer buffer, independent of user-page
    /// boundaries. This is transient transport storage, not a second cache.
    pub fn read_into(&self, offset: u64, sink: &mut IoSink<'_>) -> FsResult<usize> {
        let length = sink
            .len()
            .min(usize::try_from(self.size_bytes().saturating_sub(offset)).unwrap_or(usize::MAX));
        if length == 0 {
            return Ok(0);
        }
        let mut buffer = block_buffer(length.min(IO_BATCH_BYTES)).map_err(FsError::from)?;
        let mut done = 0;
        while done < length {
            let take = buffer.len().min(length - done);
            match self.read_bytes(offset + done as u64, &mut buffer[..take]) {
                Ok(count) if count == take => {}
                _ if done != 0 => return Ok(done),
                Ok(_) => return Err(FsError::Io),
                Err(error) => return Err(FsError::from(error)),
            }
            let mut copied = 0;
            while copied < take {
                let mut window = match sink.window(done, take - copied) {
                    Ok(window) => window,
                    Err(_) if done != 0 => return Ok(done),
                    Err(_) => return Err(FsError::InvalidArgument),
                };
                let count = window.len();
                if count == 0 {
                    return Err(FsError::Io);
                }
                window.copy_from_slice(&buffer[copied..copied + count]);
                copied += count;
                done += count;
            }
        }
        Ok(done)
    }

    /// Gathers page windows into a bounded batch before submitting a write.
    /// A user fault cannot leave a partially initialized device request.
    pub fn write_from(&self, offset: u64, source: &IoSource<'_>) -> FsResult<usize> {
        if source.is_empty() {
            return Ok(0);
        }
        let mut buffer = block_buffer(source.len().min(IO_BATCH_BYTES)).map_err(FsError::from)?;
        let mut done = 0;
        while done < source.len() {
            let take = buffer.len().min(source.len() - done);
            let at = offset
                .checked_add(done as u64)
                .ok_or(FsError::InvalidArgument)?;
            let result = source
                .load(done, &mut buffer[..take])
                .map_err(|_| FsError::InvalidArgument)
                .and_then(|()| self.write_bytes(at, &buffer[..take]).map_err(FsError::from));
            match result {
                Ok(count) => {
                    done += count;
                    if count < take {
                        break;
                    }
                }
                Err(_) if done != 0 => break,
                Err(error) => return Err(error),
            }
        }
        Ok(done)
    }

    pub fn write_bytes(&self, offset: u64, buffer: &[u8]) -> Result<usize> {
        let end = offset
            .checked_add(buffer.len() as u64)
            .ok_or(Error::InvalidArgument)?;
        if end > self.size_bytes() {
            return Err(Error::NoSpace);
        }
        if buffer.is_empty() {
            return Ok(0);
        }
        let bs = self.logical_block_size as usize;
        let first = offset / bs as u64;
        let leading = offset as usize % bs;
        if leading == 0 && buffer.len().is_multiple_of(bs) {
            self.write_blocks(first, (buffer.len() / bs) as u32, buffer)?;
            return Ok(buffer.len());
        }
        let mut bounce = block_buffer(bs)?;
        let mut done = 0usize;
        let mut lba = first;
        if leading != 0 {
            self.read_blocks(lba, 1, &mut bounce)?;
            let take = buffer.len().min(bs - leading);
            bounce[leading..leading + take].copy_from_slice(&buffer[..take]);
            self.write_blocks(lba, 1, &bounce)?;
            done += take;
            lba += 1;
        }
        let middle = (buffer.len() - done) / bs * bs;
        if middle != 0 {
            self.write_blocks(lba, (middle / bs) as u32, &buffer[done..done + middle])?;
            done += middle;
            lba += (middle / bs) as u64;
        }
        if done != buffer.len() {
            self.read_blocks(lba, 1, &mut bounce)?;
            let tail = buffer.len() - done;
            bounce[..tail].copy_from_slice(&buffer[done..]);
            self.write_blocks(lba, 1, &bounce)?;
        }
        Ok(buffer.len())
    }

    pub fn claim(self: &Arc<Self>) -> Result<BlockClaim> {
        let _pin = module::pin_owner(self.owner.as_ref(), false)?;
        if !self.header.is_live() {
            return Err(Error::NoDevice);
        }
        self.claims.fetch_add(1, Ordering::AcqRel);
        if !self.header.is_live() {
            self.claims.fetch_sub(1, Ordering::AcqRel);
            return Err(Error::NoDevice);
        }
        Ok(BlockClaim(self.clone()))
    }
}

fn block_buffer(size: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(size)
        .map_err(|_| Error::OutOfMemory)?;
    bytes.resize(size, 0);
    Ok(bytes)
}

impl DeviceNodeOps for BlockDevice {
    fn open(&self, _flags: u32) -> FsResult<usize> {
        // The short-lived pin is a state gate: once module unload begins, no
        // new devfs open can race the final idle check and callback teardown.
        let _pin = module::pin_owner(self.owner.as_ref(), false).map_err(|_| FsError::Io)?;
        Ok(0)
    }

    fn read_at(&self, offset: u64, sink: &mut IoSink<'_>) -> FsResult<usize> {
        self.read_into(offset, sink)
    }

    fn write_at(&self, offset: u64, source: &IoSource<'_>) -> FsResult<usize> {
        self.write_from(offset, source)
    }

    fn poll(
        &self,
        _context: usize,
        _offset: u64,
        events: PollEvents,
        _flags: u32,
    ) -> FsResult<PollEvents> {
        let ready = PollEvents::IN
            | PollEvents::RDNORM
            | if self.writable() {
                PollEvents::OUT | PollEvents::WRNORM
            } else {
                PollEvents::empty()
            };
        Ok(events & ready)
    }

    fn size(&self) -> u64 {
        self.size_bytes()
    }

    fn sync(&self) -> FsResult<()> {
        self.flush().map_err(FsError::from)
    }
}

/// A mount/consumer lease preventing removal of its block device.
pub struct BlockClaim(Arc<BlockDevice>);

#[allow(missing_docs)]
impl BlockClaim {
    pub fn device(&self) -> &Arc<BlockDevice> {
        &self.0
    }
}

impl Drop for BlockClaim {
    fn drop(&mut self) {
        self.0.claims.fetch_sub(1, Ordering::AcqRel);
    }
}

/// Copies and validates a module's size-prefixed operation table.
///
/// # Safety
/// `operations` must address its declared readable prefix.
pub unsafe fn copy_ops(operations: *const BlockOps) -> Result<BlockOps> {
    if operations.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: size is the first mandatory field of the ABI record.
    let declared = unsafe { (*operations).size as usize };
    if declared < BLOCK_OPS_SIZE {
        return Err(Error::InvalidArgument);
    }
    let mut copied = MaybeUninit::<BlockOps>::zeroed();
    // SAFETY: both records contain at least `BLOCK_OPS_SIZE` readable bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(
            operations.cast::<u8>(),
            copied.as_mut_ptr().cast::<u8>(),
            size_of::<BlockOps>().min(declared),
        );
        Ok(copied.assume_init())
    }
}

/// Registers a module-backed disk and scans it for partitions.
pub fn register_module(owner: Arc<Module>, name: &str, ops: BlockOps) -> Result<Arc<BlockDevice>> {
    if ops.read.is_none()
        || ops.logical_block_size < 512
        || !ops.logical_block_size.is_power_of_two()
        || ops.logical_block_size > MAX_LOGICAL_BLOCK_SIZE
        || ops.block_count == 0
        || ops
            .block_count
            .checked_mul(u64::from(ops.logical_block_size))
            .is_none()
    {
        return Err(Error::InvalidArgument);
    }
    let block_size = ops.logical_block_size;
    let max_transfer = ops.max_transfer_blocks;
    let count = ops.block_count;
    register(
        name,
        Some(owner.clone()),
        None,
        Arc::new(ModuleBackend { owner, ops }),
        block_size,
        max_transfer,
        count,
    )
}

/// Registers a derived block address space such as a partition.
pub fn register_child(
    name: &str,
    parent: Arc<BlockDevice>,
    backend: Arc<dyn BlockBackend>,
    block_count: u64,
) -> Result<Arc<BlockDevice>> {
    register(
        name,
        parent.owner.clone(),
        Some(parent.clone()),
        backend,
        parent.logical_block_size,
        parent.max_transfer_blocks,
        block_count,
    )
}

#[allow(clippy::too_many_arguments)]
fn register(
    name: &str,
    owner: Option<Arc<Module>>,
    parent: Option<Arc<BlockDevice>>,
    backend: Arc<dyn BlockBackend>,
    logical_block_size: u32,
    max_transfer_blocks: u32,
    block_count: u64,
) -> Result<Arc<BlockDevice>> {
    if name.is_empty() || name.len() > 255 || name.bytes().any(|byte| byte == b'/') {
        return Err(Error::InvalidArgument);
    }
    let registry = registry()?;
    if registry.devices.lock().contains_key(name) {
        return Err(Error::AlreadyExists);
    }
    let device = Arc::new(BlockDevice {
        header: ObjHeader::new_with(
            crate::driver::obj::ObjKind::BlockDevice,
            Some(name),
            owner.as_ref().map(|module| module.id().get()),
            parent.as_ref().map(|device| device.object_id()),
        ),
        name: name.to_string().into_boxed_str(),
        owner,
        parent: parent.clone(),
        children: Mutex::new(Vec::new()),
        backend,
        logical_block_size,
        max_transfer_blocks,
        block_count,
        devfs_node: Mutex::new(None),
        claims: AtomicU64::new(0),
        scanned: AtomicU32::new(0),
    });
    device.header.register()?;
    let node_id = match node::create_native_node(
        device.owner.as_ref(),
        node::root()?,
        name,
        node::kind::BLOCK,
        0o660,
        device.clone(),
    ) {
        Ok(id) => id,
        Err(error) => {
            device.header.poison();
            return Err(error);
        }
    };
    *device.devfs_node.lock() = Some(node_id);
    registry
        .devices
        .lock()
        .insert(device.name.clone(), device.clone());
    if let Some(parent) = parent {
        parent.children.lock().push(Arc::downgrade(&device));
    }
    Ok(device)
}

/// Scans a fully published whole disk exactly once.
pub fn rescan(device: &Arc<BlockDevice>) -> Result<()> {
    if device.parent.is_some() {
        return Err(Error::WrongKind);
    }
    if device
        .scanned
        .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        crate::fs::partition::scan(device);
        device.scanned.store(2, Ordering::Release);
    }
    Ok(())
}

/// Finds a registered disk or partition by its devfs basename.
pub fn find(name: &str) -> Result<Arc<BlockDevice>> {
    registry()?
        .devices
        .lock()
        .get(name)
        .cloned()
        .ok_or(Error::NotFound)
}

/// Removes a device and all unclaimed partition children.
pub fn unregister(device: &Arc<BlockDevice>) -> Result<()> {
    if device.claims.load(Ordering::Acquire) != 0 {
        return Err(Error::Busy);
    }
    let children: Vec<_> = device
        .children
        .lock()
        .iter()
        .filter_map(Weak::upgrade)
        .collect();
    if children
        .iter()
        .any(|child| child.claims.load(Ordering::Acquire) != 0)
    {
        return Err(Error::Busy);
    }
    for child in children.into_iter().rev() {
        unregister(&child)?;
    }
    device.header.set_state(ObjState::Removing);
    if let Some(node_id) = device.devfs_node.lock().take()
        && let Err(error) = node::remove(device.owner.as_ref(), node_id)
    {
        device.header.set_state(ObjState::Live);
        *device.devfs_node.lock() = Some(node_id);
        return Err(error);
    }
    registry()?.devices.lock().remove(device.name());
    device.header.poison();
    Ok(())
}

/// Checks whether all block devices owned by a module are idle.
pub fn try_remove_module_devices(module: &Arc<Module>) -> Result<()> {
    let devices: Vec<_> = registry()?
        .devices
        .lock()
        .values()
        .filter(|device| {
            device
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    if devices
        .iter()
        .any(|device| device.claims.load(Ordering::Acquire) != 0)
    {
        Err(Error::Busy)
    } else {
        Ok(())
    }
}

/// Sweeps module-owned root devices after its exit callback.
pub fn remove_module_devices(module: &Arc<Module>) {
    let Ok(registry) = registry() else { return };
    let roots: Vec<_> = registry
        .devices
        .lock()
        .values()
        .filter(|device| {
            device.parent.is_none()
                && device
                    .owner
                    .as_ref()
                    .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for device in roots {
        let _ = unregister(&device);
    }
}

const _: () = assert!(size_of::<BlockOps>() == BLOCK_OPS_SIZE);
