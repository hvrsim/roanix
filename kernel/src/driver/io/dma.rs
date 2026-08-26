//! Direct memory access buffers.
//!
//! Controllers that move data themselves - storage queues, network rings, USB
//! transfer descriptors - need memory the device can address and that the CPU
//! never sees through a stale cache line. This module supplies coherent
//! allocations backed by physically contiguous frames, honours the addressing
//! limit a device advertises, and lets a bus substitute its own translation so
//! an IOMMU or an offset-mapped bus can be added later without changing any
//! driver.

use alloc::{sync::Arc, vec::Vec};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{
    mem::{PAGE_SIZE, PhysAddr, VirtAddr, phys},
    sys::sync::{Mutex, Once},
};

use super::super::{
    core::{device::Device, module::Module},
    error::{Error, Result},
};

/// Transfer directions.
pub mod direction {
    /// Memory is written by the device and read by the CPU.
    pub const FROM_DEVICE: u32 = 1;
    /// Memory is written by the CPU and read by the device.
    pub const TO_DEVICE: u32 = 2;
    /// Memory is accessed in both directions.
    pub const BIDIRECTIONAL: u32 = 3;
}

/// Allocation attribute flags.
pub mod flags {
    /// Zero the buffer before returning it.
    pub const ZERO: u32 = 1 << 0;
    /// The device cannot address memory above 32 bits.
    pub const ADDRESS32: u32 = 1 << 1;
}

/// Translates CPU physical addresses to bus addresses for one bus.
#[repr(C)]
pub struct DmaOps {
    /// Size of this table, for forward-compatible extension.
    pub size: u32,
    /// Converts a CPU physical address into a device-visible bus address.
    pub to_device: Option<unsafe extern "C" fn(context: *mut c_void, physical: u64) -> u64>,
    /// Converts a device-visible bus address back into a CPU physical address.
    pub from_device: Option<unsafe extern "C" fn(context: *mut c_void, bus: u64) -> u64>,
    /// Context passed to every operation.
    pub context: *mut c_void,
}

/// A coherent buffer shared between the CPU and a device.
pub struct DmaBuffer {
    virt: VirtAddr,
    physical: PhysAddr,
    device_address: u64,
    size: usize,
    pages: usize,
    owner: Option<Arc<Module>>,
}

impl DmaBuffer {
    /// Returns the CPU address of the buffer.
    pub const fn address(&self) -> VirtAddr {
        self.virt
    }

    /// Returns the CPU physical address of the buffer.
    pub const fn physical(&self) -> PhysAddr {
        self.physical
    }

    /// Returns the address the device must be programmed with.
    pub const fn device_address(&self) -> u64 {
        self.device_address
    }

    /// Returns the usable size in bytes.
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns whether the buffer is empty.
    pub const fn is_empty(&self) -> bool {
        self.size == 0
    }
}

struct State {
    buffers: Mutex<Vec<Arc<DmaBuffer>>>,
    allocated: AtomicU64,
}

static STATE: Once<State> = Once::new();

pub(crate) fn init() {
    STATE.call_once(|| State {
        buffers: Mutex::new(Vec::new()),
        allocated: AtomicU64::new(0),
    });
}

fn state() -> Result<&'static State> {
    STATE.get().ok_or(Error::NotInitialized)
}

/// Returns the addressing limit that applies to `device`.
fn address_limit(device: Option<&Arc<Device>>, attributes: u32) -> u64 {
    let mut limit = device.map_or(u64::MAX, |device| device.dma_mask());
    if attributes & flags::ADDRESS32 != 0 {
        limit = limit.min(u32::MAX as u64);
    }
    limit
}

/// Applies the bus translation, if the device's bus installed one.
fn to_device_address(device: Option<&Arc<Device>>, physical: u64) -> u64 {
    let Some(device) = device else {
        return physical;
    };
    let Some(bus) = device.bus() else {
        return physical;
    };
    let ops = bus.dma_ops();
    if ops.is_null() {
        return physical;
    }
    // SAFETY: `set_dma_ops` requires an immutable table that stays valid while
    // the bus is registered, and the bus is alive because the device is.
    let table = unsafe { &*ops.cast::<DmaOps>() };
    if (table.size as usize) < size_of::<DmaOps>() {
        return physical;
    }
    match table.to_device {
        // SAFETY: the bus published this callback with the table above.
        Some(translate) => unsafe { translate(table.context, physical) },
        None => physical,
    }
}

/// Allocates a coherent buffer of `size` bytes.
pub fn alloc_coherent(
    owner: Option<&Arc<Module>>,
    device: Option<&Arc<Device>>,
    size: usize,
    align: usize,
    attributes: u32,
) -> Result<Arc<DmaBuffer>> {
    if size == 0 {
        return Err(Error::InvalidArgument);
    }
    let state = state()?;
    let pages = (size as u64).div_ceil(PAGE_SIZE) as usize;
    let align_pages = align
        .div_ceil(PAGE_SIZE as usize)
        .max(1)
        .next_power_of_two();
    let limit = address_limit(device, attributes);

    let first = phys::alloc_contiguous(phys::PageUse::KernelHeap, pages, align_pages, limit)
        .ok_or(Error::OutOfMemory)?;
    let physical = first.paddr();
    let virt = crate::mem::phys_to_virt(physical);

    if attributes & flags::ZERO != 0 {
        // SAFETY: the run was just allocated exclusively to this buffer and the
        // direct map covers every usable frame.
        unsafe {
            core::ptr::write_bytes(virt.as_mut_ptr::<u8>(), 0, pages * PAGE_SIZE as usize);
        }
    }

    let buffer = Arc::new(DmaBuffer {
        virt,
        physical,
        device_address: to_device_address(device, physical.as_u64()),
        size,
        pages,
        owner: owner.cloned(),
    });
    state.buffers.lock().push(buffer.clone());
    state
        .allocated
        .fetch_add((pages as u64) * PAGE_SIZE, Ordering::Relaxed);
    Ok(buffer)
}

/// Releases a coherent buffer.
pub fn free_coherent(buffer: &Arc<DmaBuffer>) -> Result<()> {
    let state = state()?;
    let removed = {
        let mut buffers = state.buffers.lock();
        let position = buffers
            .iter()
            .position(|entry| Arc::ptr_eq(entry, buffer))
            .ok_or(Error::NotFound)?;
        buffers.remove(position)
    };
    release(&removed);
    Ok(())
}

fn release(buffer: &Arc<DmaBuffer>) {
    let Some(first) = phys::phys_to_page(buffer.physical) else {
        return;
    };
    // SAFETY: the buffer is no longer registered, so no device or driver may
    // reference the run after this point.
    unsafe { phys::free_contiguous(first, buffer.pages) };
    if let Ok(state) = state() {
        state
            .allocated
            .fetch_sub((buffer.pages as u64) * PAGE_SIZE, Ordering::Relaxed);
    }
}

/// Maps an existing kernel buffer for device access.
///
/// The direct map is coherent on both supported architectures, so this resolves
/// the bus address without copying. It fails when the buffer does not satisfy
/// the device's addressing limit, which is where a bounce buffer or an IOMMU
/// mapping would be introduced.
pub fn map_single(
    device: Option<&Arc<Device>>,
    address: VirtAddr,
    size: usize,
    _direction: u32,
) -> Result<u64> {
    if size == 0 {
        return Err(Error::InvalidArgument);
    }
    let physical = crate::mem::virt_to_phys_hhdm(address).ok_or(Error::InvalidArgument)?;
    let limit = address_limit(device, 0);
    let end = physical
        .as_u64()
        .checked_add(size as u64 - 1)
        .ok_or(Error::InvalidArgument)?;
    if end > limit {
        return Err(Error::Unsupported);
    }
    Ok(to_device_address(device, physical.as_u64()))
}

/// Releases a mapping created by [`map_single`].
pub fn unmap_single(_device: Option<&Arc<Device>>, _address: u64, _size: usize, _direction: u32) {}

/// Orders CPU and device views of a buffer.
///
/// Both supported architectures keep the direct map coherent with device
/// traffic, so only a compiler and memory barrier is required.
pub fn sync(_address: VirtAddr, _size: usize, _direction: u32) {
    core::sync::atomic::fence(Ordering::SeqCst);
}

pub(in super::super) fn remove_module_buffers(module: &Arc<Module>) {
    let Ok(state) = state() else {
        return;
    };
    let owned: Vec<Arc<DmaBuffer>> = {
        let mut buffers = state.buffers.lock();
        let mut owned = Vec::new();
        buffers.retain(|buffer| {
            let matches = buffer
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module));
            if matches {
                owned.push(buffer.clone());
            }
            !matches
        });
        owned
    };
    for buffer in owned {
        release(&buffer);
    }
}

/// Returns the number of bytes currently held in coherent buffers.
pub fn allocated_bytes() -> u64 {
    STATE
        .get()
        .map_or(0, |state| state.allocated.load(Ordering::Relaxed))
}
