//! Checked MBR and GPT partition discovery.
//!
//! On-disk records are decoded from byte slices instead of packed Rust
//! structs, so malformed media cannot cause unaligned loads or out-of-bounds
//! access. Discovered ranges become ordinary block devices layered over their
//! parent disk.

use alloc::{format, sync::Arc, vec::Vec};

use crate::{
    dev::block::{self, BlockBackend, BlockDevice},
    driver::error::{Error, Result},
};

const GPT_SIGNATURE: &[u8; 8] = b"EFI PART";
const GPT_MIN_HEADER: usize = 92;
const GPT_MIN_ENTRY: usize = 128;
const MAX_GPT_ENTRIES: usize = 4096;
const MAX_GPT_BYTES: usize = 16 * 1024 * 1024;
const MAX_EBR_LINKS: usize = 128;

#[derive(Clone, Copy, Debug)]
struct Partition {
    number: usize,
    first: u64,
    blocks: u64,
}

struct PartitionBackend {
    parent: Arc<BlockDevice>,
    first: u64,
    blocks: u64,
}

impl PartitionBackend {
    fn map(&self, lba: u64, blocks: u32) -> Result<u64> {
        let end = lba
            .checked_add(u64::from(blocks))
            .ok_or(Error::InvalidArgument)?;
        if end > self.blocks {
            return Err(Error::InvalidArgument);
        }
        self.first.checked_add(lba).ok_or(Error::InvalidArgument)
    }
}

impl BlockBackend for PartitionBackend {
    fn read_blocks(&self, lba: u64, blocks: u32, buffer: &mut [u8]) -> Result<()> {
        self.parent
            .read_blocks(self.map(lba, blocks)?, blocks, buffer)
    }

    fn write_blocks(&self, lba: u64, blocks: u32, buffer: &[u8]) -> Result<()> {
        self.parent
            .write_blocks(self.map(lba, blocks)?, blocks, buffer)
    }

    fn flush(&self) -> Result<()> {
        self.parent.flush()
    }

    fn discard(&self, lba: u64, blocks: u32) -> Result<()> {
        self.parent.discard(self.map(lba, blocks)?, blocks)
    }

    fn writable(&self) -> bool {
        self.parent.writable()
    }
}

/// Scans a newly registered whole disk. Corrupt or absent tables leave the
/// disk usable and are reported without failing hardware probe.
pub fn scan(device: &Arc<BlockDevice>) {
    match discover(device) {
        Ok(partitions) => {
            for partition in partitions {
                let name = format!("{}p{}", device.name(), partition.number);
                let backend = Arc::new(PartitionBackend {
                    parent: device.clone(),
                    first: partition.first,
                    blocks: partition.blocks,
                });
                match block::register_child(&name, device.clone(), backend, partition.blocks) {
                    Ok(child) => {
                        log::info!(
                            target: "partition",
                            "{}: blocks {}..{} ({})",
                            child.name(),
                            partition.first,
                            partition.first + partition.blocks - 1,
                            partition.blocks,
                        );
                    }
                    Err(error) => {
                        log::warn!(target: "partition", "{name}: registration failed: {error:?}")
                    }
                }
            }
        }
        Err(Error::NotFound) => {}
        Err(error) => {
            log::warn!(target: "partition", "{}: invalid partition table: {error:?}", device.name())
        }
    }
}

fn discover(device: &BlockDevice) -> Result<Vec<Partition>> {
    let block_size = device.logical_block_size() as usize;
    if block_size < 512 {
        return Err(Error::InvalidArgument);
    }
    let mut sector = zeroed(block_size)?;
    device.read_blocks(0, 1, &mut sector)?;
    if sector.get(510..512) != Some(&[0x55, 0xaa]) {
        return Err(Error::NotFound);
    }
    let protective = (0..4).any(|index| sector[446 + index * 16 + 4] == 0xee);
    if protective {
        return discover_gpt(device, block_size);
    }
    discover_mbr(device, block_size, &sector)
}

fn discover_gpt(device: &BlockDevice, block_size: usize) -> Result<Vec<Partition>> {
    if device.block_count() < 2 {
        return Err(Error::InvalidArgument);
    }
    let mut header = zeroed(block_size)?;
    device.read_blocks(1, 1, &mut header)?;
    if header.get(..8) != Some(GPT_SIGNATURE) {
        return Err(Error::InvalidArgument);
    }
    let header_size = le32(&header, 12)? as usize;
    if !(GPT_MIN_HEADER..=block_size).contains(&header_size) {
        return Err(Error::InvalidArgument);
    }
    let stored_header_crc = le32(&header, 16)?;
    let mut checked_header = header[..header_size].to_vec();
    checked_header[16..20].fill(0);
    if crc32(&checked_header) != stored_header_crc {
        return Err(Error::Io);
    }
    if le64(&header, 24)? != 1 || le64(&header, 32)? >= device.block_count() {
        return Err(Error::InvalidArgument);
    }
    let first_usable = le64(&header, 40)?;
    let last_usable = le64(&header, 48)?;
    if first_usable > last_usable || last_usable >= device.block_count() {
        return Err(Error::InvalidArgument);
    }
    let entries_lba = le64(&header, 72)?;
    let entry_count = le32(&header, 80)? as usize;
    let entry_size = le32(&header, 84)? as usize;
    if entry_count == 0
        || entry_count > MAX_GPT_ENTRIES
        || entry_size < GPT_MIN_ENTRY
        || !entry_size.is_multiple_of(8)
    {
        return Err(Error::InvalidArgument);
    }
    let bytes = entry_count
        .checked_mul(entry_size)
        .ok_or(Error::InvalidArgument)?;
    if bytes > MAX_GPT_BYTES {
        return Err(Error::InvalidArgument);
    }
    let blocks = bytes.div_ceil(block_size);
    let end = entries_lba
        .checked_add(blocks as u64)
        .ok_or(Error::InvalidArgument)?;
    if entries_lba == 0 || end > device.block_count() {
        return Err(Error::InvalidArgument);
    }
    let mut entries = zeroed(blocks * block_size)?;
    device.read_blocks(
        entries_lba,
        u32::try_from(blocks).map_err(|_| Error::InvalidArgument)?,
        &mut entries,
    )?;
    if crc32(&entries[..bytes]) != le32(&header, 88)? {
        return Err(Error::Io);
    }
    let mut output = Vec::new();
    for index in 0..entry_count {
        let entry = &entries[index * entry_size..(index + 1) * entry_size];
        if entry[..16].iter().all(|byte| *byte == 0) {
            continue;
        }
        let first = le64(entry, 32)?;
        let last = le64(entry, 40)?;
        if first < first_usable || last > last_usable || first > last {
            return Err(Error::InvalidArgument);
        }
        output.push(Partition {
            number: index + 1,
            first,
            blocks: last - first + 1,
        });
    }
    validate_ranges(output, device.block_count())
}

fn discover_mbr(device: &BlockDevice, block_size: usize, sector: &[u8]) -> Result<Vec<Partition>> {
    let mut output = Vec::new();
    let mut logical_number = 5usize;
    for index in 0..4 {
        let entry = &sector[446 + index * 16..462 + index * 16];
        let kind = entry[4];
        let first = u64::from(le32(entry, 8)?);
        let blocks = u64::from(le32(entry, 12)?);
        if kind == 0 || blocks == 0 {
            continue;
        }
        if is_extended(kind) {
            discover_ebr(
                device,
                block_size,
                first,
                blocks,
                &mut logical_number,
                &mut output,
            )?;
        } else if kind != 0xee {
            output.push(Partition {
                number: index + 1,
                first,
                blocks,
            });
        }
    }
    if output.is_empty() {
        return Err(Error::NotFound);
    }
    validate_ranges(output, device.block_count())
}

fn discover_ebr(
    device: &BlockDevice,
    block_size: usize,
    base: u64,
    container_blocks: u64,
    number: &mut usize,
    output: &mut Vec<Partition>,
) -> Result<()> {
    let container_end = base
        .checked_add(container_blocks)
        .filter(|end| *end <= device.block_count())
        .ok_or(Error::InvalidArgument)?;
    let mut relative = 0u64;
    let mut visited = Vec::new();
    for _ in 0..MAX_EBR_LINKS {
        let ebr_lba = base.checked_add(relative).ok_or(Error::InvalidArgument)?;
        if ebr_lba < base || ebr_lba >= container_end || visited.contains(&ebr_lba) {
            return Err(Error::InvalidArgument);
        }
        visited.push(ebr_lba);
        let mut sector = zeroed(block_size)?;
        device.read_blocks(ebr_lba, 1, &mut sector)?;
        if sector.get(510..512) != Some(&[0x55, 0xaa]) {
            return Err(Error::InvalidArgument);
        }
        let logical = &sector[446..462];
        let kind = logical[4];
        let first_relative = u64::from(le32(logical, 8)?);
        let blocks = u64::from(le32(logical, 12)?);
        if kind != 0 && blocks != 0 && !is_extended(kind) {
            let first = ebr_lba
                .checked_add(first_relative)
                .ok_or(Error::InvalidArgument)?;
            if first < base
                || first
                    .checked_add(blocks)
                    .is_none_or(|end| end > container_end)
            {
                return Err(Error::InvalidArgument);
            }
            output.push(Partition {
                number: *number,
                first,
                blocks,
            });
            *number += 1;
        }
        let link = &sector[462..478];
        if !is_extended(link[4]) || le32(link, 12)? == 0 {
            return Ok(());
        }
        relative = u64::from(le32(link, 8)?);
    }
    Err(Error::InvalidArgument)
}

fn is_extended(kind: u8) -> bool {
    matches!(kind, 0x05 | 0x0f | 0x85)
}

fn validate_ranges(mut partitions: Vec<Partition>, disk_blocks: u64) -> Result<Vec<Partition>> {
    partitions.sort_unstable_by_key(|partition| partition.first);
    let mut previous_end = 0u64;
    for partition in &partitions {
        let end = partition
            .first
            .checked_add(partition.blocks)
            .ok_or(Error::InvalidArgument)?;
        if partition.blocks == 0
            || partition.first == 0
            || partition.first >= disk_blocks
            || end > disk_blocks
            || partition.first < previous_end
        {
            return Err(Error::InvalidArgument);
        }
        previous_end = end;
    }
    partitions.sort_unstable_by_key(|partition| partition.number);
    Ok(partitions)
}

fn le32(bytes: &[u8], offset: usize) -> Result<u32> {
    let value = bytes
        .get(offset..offset + 4)
        .ok_or(Error::InvalidArgument)?;
    Ok(u32::from_le_bytes(
        value.try_into().map_err(|_| Error::InvalidArgument)?,
    ))
}

fn le64(bytes: &[u8], offset: usize) -> Result<u64> {
    let value = bytes
        .get(offset..offset + 8)
        .ok_or(Error::InvalidArgument)?;
    Ok(u64::from_le_bytes(
        value.try_into().map_err(|_| Error::InvalidArgument)?,
    ))
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}

fn zeroed(length: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| Error::OutOfMemory)?;
    bytes.resize(length, 0);
    Ok(bytes)
}
