//! VM-backed metadata reads for immutable disk filesystems.
//!
//! Only metadata is addressed through this object; regular file data lives in
//! its file object. Both use the VM's physical-page accounting and reclamation.

use super::{Error, Result};
use crate::{
    dev::block::{BlockClaim, BlockDevice},
    mem::{self, VmObject, VmPager},
};
use alloc::sync::Arc;

struct MetadataPager(BlockClaim);
impl VmPager for MetadataPager {
    fn read(&self, offset: u64, output: &mut [u8]) -> mem::Result<()> {
        let count = self
            .0
            .device()
            .read_bytes(offset, output)
            .map_err(|_| mem::Error::Io)?;
        output[count..].fill(0);
        Ok(())
    }
}

pub(super) fn metadata_object(device: &Arc<BlockDevice>) -> Result<Arc<VmObject>> {
    Ok(VmObject::with_pager(
        device.size_bytes(),
        Arc::new(MetadataPager(device.claim().map_err(Error::from)?)),
    ))
}

pub(super) fn pager_error(error: Error) -> mem::Error {
    match error {
        Error::OutOfMemory => mem::Error::OutOfMemory,
        _ => mem::Error::Io,
    }
}

/// One initialized byte extent. Gaps in the mapping are sparse zeros.
pub(super) struct FileExtent {
    pub logical: u64,
    pub physical: u64,
    pub length: u64,
}

/// Filesystem-independent immutable extent pager. It owns a device claim, so
/// a mapping remains usable after its mount and last descriptor are gone.
pub(super) struct ExtentPager {
    claim: BlockClaim,
    size: u64,
    extents: alloc::vec::Vec<FileExtent>,
}

impl ExtentPager {
    pub fn object(
        device: &Arc<BlockDevice>,
        size: u64,
        extents: alloc::vec::Vec<FileExtent>,
    ) -> Result<Arc<VmObject>> {
        let mut mapped = alloc::vec::Vec::<FileExtent>::new();
        mapped
            .try_reserve_exact(extents.len())
            .map_err(|_| Error::OutOfMemory)?;
        for extent in extents {
            let end = extent.logical.checked_add(extent.length).ok_or(Error::Io)?;
            let disk_end = extent
                .physical
                .checked_add(extent.length)
                .ok_or(Error::Io)?;
            if extent.length == 0 || disk_end > device.size_bytes() {
                return Err(Error::Io);
            }
            if let Some(last) = mapped.last_mut() {
                let last_end = last.logical + last.length;
                if extent.logical < last_end {
                    return Err(Error::Io);
                }
                if extent.logical == last_end && extent.physical == last.physical + last.length {
                    last.length = end - last.logical;
                    continue;
                }
            }
            mapped.push(extent);
        }
        Ok(VmObject::with_pager(
            size,
            Arc::new(Self {
                claim: device.claim().map_err(Error::from)?,
                size,
                extents: mapped,
            }),
        ))
    }
}

impl VmPager for ExtentPager {
    fn read(&self, offset: u64, output: &mut [u8]) -> mem::Result<()> {
        output.fill(0);
        let length = output
            .len()
            .min(usize::try_from(self.size.saturating_sub(offset)).unwrap_or(usize::MAX));
        if length == 0 {
            return Ok(());
        }
        let end = offset + length as u64;
        let first = self
            .extents
            .partition_point(|extent| extent.logical + extent.length <= offset);
        for extent in &self.extents[first..] {
            if extent.logical >= end {
                break;
            }
            let start = offset.max(extent.logical);
            let stop = end.min(extent.logical + extent.length);
            let within = (start - offset) as usize;
            let count = (stop - start) as usize;
            let physical = extent.physical + start - extent.logical;
            if self
                .claim
                .device()
                .read_bytes(physical, &mut output[within..within + count])
                .map_err(|_| mem::Error::Io)?
                != count
            {
                return Err(mem::Error::Io);
            }
        }
        Ok(())
    }
}
