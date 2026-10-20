//! Read-only FAT32/VFAT filesystem driver.
//!
//! FAT is decoded from byte slices, never packed structures. The mount owns a
//! block claim, validates every cluster transition, and caches active vnodes by
//! their physical directory-entry address. VFAT long names are checksum-checked
//! and decoded as UTF-16; malformed chains fall back to their 8.3 alias.

use alloc::{
    boxed::Box,
    collections::BTreeSet,
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{any::Any, char};

use crate::{
    dev::block::{BlockClaim, BlockDevice},
    mem::{self, VmObject, VmPager},
    sys::sync::{Mutex, Once},
};

use super::{
    Error, Result,
    vnode::{
        CreateKind, DirEntry, FileSystem, FilesystemId, NodeId, SetAttr, StatFs, Vnode, VnodeAttr,
        VnodeCache, VnodeKey, VnodeKind, VnodeOps,
    },
};

const FAT32_NAME: &str = "fat32";
const ATTR_DIRECTORY: u8 = 0x10;
const ATTR_VOLUME_ID: u8 = 0x08;
const ATTR_LONG_NAME: u8 = 0x0f;
const FAT_ENTRY_MASK: u32 = 0x0fff_ffff;
const FAT_BAD_CLUSTER: u32 = 0x0fff_fff7;
const FAT_END_MIN: u32 = 0x0fff_fff8;
const ROOT_NODE: u64 = 1;
const DIRECTORY_CURSOR_BASE: u64 = 2;

struct Geometry {
    bytes_per_cluster: u32,
    fat_offset: u64,
    fat_bytes: u64,
    data_offset: u64,
    total_bytes: u64,
    cluster_count: u32,
    root_cluster: u32,
    active_fat: u8,
    fs_info_offset: Option<u64>,
    free_clusters: Option<u32>,
}

/// One mounted FAT32 volume.
pub struct Fat32 {
    id: FilesystemId,
    volume: Arc<FatVolume>,
    root: Once<Vnode>,
    cache: VnodeCache,
    objects: super::cache::ObjectCache,
}

struct FatVolume {
    claim: BlockClaim,
    geometry: Geometry,
    metadata: Arc<VmObject>,
}

struct FatNode {
    filesystem: Weak<Fat32>,
    kind: VnodeKind,
    first_cluster: u32,
    size: u64,
    entry_offset: Option<u64>,
    parent: Option<Vnode>,
}

#[derive(Clone)]
struct Record {
    name: Arc<[u8]>,
    first_cluster: u32,
    size: u64,
    attributes: u8,
    entry_offset: u64,
    /// Logical byte position after the short entry, including its LFN slots.
    next_offset: u64,
}

impl Record {
    fn key(&self, filesystem: FilesystemId) -> VnodeKey {
        // Division by a 32-byte slot leaves room for the two reserved IDs.
        VnodeKey {
            filesystem,
            node: NodeId::new(self.entry_offset / 32 + 2),
        }
    }

    fn kind(&self) -> VnodeKind {
        if self.attributes & ATTR_DIRECTORY != 0 {
            VnodeKind::Directory
        } else {
            VnodeKind::Regular
        }
    }
}

#[derive(Default)]
struct ClusterChain {
    clusters: Vec<u32>,
    seen: BTreeSet<u32>,
}

struct FatPager {
    volume: Arc<FatVolume>,
    first_cluster: u32,
    size: u64,
    chain: Mutex<ClusterChain>,
}

impl VmPager for FatPager {
    fn read(&self, offset: u64, output: &mut [u8]) -> mem::Result<()> {
        self.read_file(offset, output)
            .map_err(super::io::pager_error)
    }
}

impl FatPager {
    fn read_file(&self, offset: u64, output: &mut [u8]) -> Result<()> {
        let length = output
            .len()
            .min(usize::try_from(self.size.saturating_sub(offset)).unwrap_or(usize::MAX));
        output[length..].fill(0);
        if length == 0 {
            return Ok(());
        }
        let fs = &self.volume;
        let cluster_bytes = u64::from(fs.geometry.bytes_per_cluster);
        let end = (offset + length as u64).div_ceil(cluster_bytes);
        if end > u64::from(fs.geometry.cluster_count) {
            return Err(Error::Io);
        }
        // Persist the index across reads and page faults. Metadata I/O may
        // sleep here, but no user windows or VM page-index locks are held.
        let mut chain = self.chain.lock();
        let additional = (end as usize).saturating_sub(chain.clusters.len());
        chain
            .clusters
            .try_reserve(additional)
            .map_err(|_| Error::OutOfMemory)?;
        while chain.clusters.len() < end as usize {
            let next = match chain.clusters.last() {
                Some(&last) => fs.next_cluster(last)?.ok_or(Error::Io)?,
                None => self.first_cluster,
            };
            fs.validate_cluster(next)?;
            if !chain.seen.insert(next) {
                return Err(Error::Io);
            }
            chain.clusters.push(next);
        }
        let first = (offset / cluster_bytes) as usize;
        let mut runs = Vec::<(u64, usize)>::new();
        runs.try_reserve(end as usize - first)
            .map_err(|_| Error::OutOfMemory)?;
        let mut remaining = length;
        let mut within = (offset % cluster_bytes) as usize;
        for &cluster in &chain.clusters[first..end as usize] {
            let at = fs.cluster_offset(cluster)? + within as u64;
            let take = remaining.min(cluster_bytes as usize - within);
            if let Some((start, count)) = runs.last_mut()
                && *start + *count as u64 == at
            {
                *count += take;
            } else {
                runs.push((at, take));
            }
            remaining -= take;
            within = 0;
        }
        drop(chain);
        let mut done = 0;
        for (at, count) in runs {
            if fs
                .device()
                .read_bytes(at, &mut output[done..done + count])
                .map_err(Error::from)?
                != count
            {
                return Err(Error::Io);
            }
            done += count;
        }
        Ok(())
    }
}

impl FatVolume {
    fn device(&self) -> &Arc<BlockDevice> {
        self.claim.device()
    }

    fn validate_cluster(&self, cluster: u32) -> Result<()> {
        if cluster < 2 || cluster >= self.geometry.cluster_count.saturating_add(2) {
            Err(Error::Io)
        } else {
            Ok(())
        }
    }

    fn cluster_offset(&self, cluster: u32) -> Result<u64> {
        self.validate_cluster(cluster)?;
        self.geometry
            .data_offset
            .checked_add(u64::from(cluster - 2) * u64::from(self.geometry.bytes_per_cluster))
            .filter(|offset| *offset < self.geometry.total_bytes)
            .ok_or(Error::Io)
    }

    fn fat_entry(&self, cluster: u32) -> Result<u32> {
        self.validate_cluster(cluster)?;
        let fat = self
            .geometry
            .fat_offset
            .checked_add(u64::from(self.geometry.active_fat) * self.geometry.fat_bytes)
            .ok_or(Error::Io)?;
        let offset = fat
            .checked_add(u64::from(cluster) * 4)
            .filter(|offset| *offset + 4 <= fat + self.geometry.fat_bytes)
            .ok_or(Error::Io)?;
        let mut bytes = [0u8; 4];
        if self
            .metadata
            .read_at(offset, &mut bytes)
            .map_err(Error::from)?
            != 4
        {
            return Err(Error::Io);
        }
        Ok(u32::from_le_bytes(bytes) & FAT_ENTRY_MASK)
    }

    fn next_cluster(&self, cluster: u32) -> Result<Option<u32>> {
        let next = self.fat_entry(cluster)?;
        if next >= FAT_END_MIN {
            return Ok(None);
        }
        if next == 0 || next == 1 || next == FAT_BAD_CLUSTER {
            return Err(Error::Io);
        }
        self.validate_cluster(next)?;
        Ok(Some(next))
    }
}

impl Fat32 {
    /// Probes and mounts a FAT32 volume on `device`.
    pub fn new(device: Arc<BlockDevice>) -> Result<Arc<Self>> {
        let claim = device.claim().map_err(Error::from)?;
        let mut boot = [0u8; 512];
        if device.read_bytes(0, &mut boot).map_err(Error::from)? != boot.len() {
            return Err(Error::Io);
        }
        let mut geometry = parse_boot(&boot, device.size_bytes())?;
        if let Some(offset) = geometry.fs_info_offset {
            let mut info = [0u8; 512];
            if device.read_bytes(offset, &mut info).map_err(Error::from)? == info.len()
                && le32(&info, 0) == 0x4161_5252
                && le32(&info, 484) == 0x6141_7272
                && le32(&info, 508) == 0xaa55_0000
            {
                let free = le32(&info, 488);
                if free != u32::MAX && free <= geometry.cluster_count {
                    geometry.free_clusters = Some(free);
                }
            }
        }
        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            volume: Arc::new(FatVolume {
                claim,
                geometry,
                metadata: super::io::metadata_object(&device)?,
            }),
            root: Once::new(),
            cache: VnodeCache::new(),
            objects: super::cache::ObjectCache::new(),
        });
        filesystem
            .volume
            .validate_cluster(filesystem.volume.geometry.root_cluster)?;
        let root = Vnode::new(
            VnodeKey {
                filesystem: filesystem.id,
                node: NodeId::new(ROOT_NODE),
            },
            VnodeKind::Directory,
            Box::new(FatNode {
                filesystem: Arc::downgrade(&filesystem),
                kind: VnodeKind::Directory,
                first_cluster: filesystem.volume.geometry.root_cluster,
                size: 0,
                entry_offset: None,
                parent: None,
            }),
        );
        filesystem.root.call_once(|| root);
        Ok(filesystem)
    }

    /// Decode one cluster at a time and stop as soon as the visitor is done.
    fn visit_directory(
        &self,
        first_cluster: u32,
        start: u64,
        mut visit: impl FnMut(Record) -> Result<bool>,
    ) -> Result<()> {
        self.volume.validate_cluster(first_cluster)?;
        let cluster_bytes = self.volume.geometry.bytes_per_cluster as usize;
        let mut cluster = first_cluster;
        let mut traversed = 0u32;
        if !start.is_multiple_of(32) {
            return Err(Error::InvalidArgument);
        }
        let skip = start / cluster_bytes as u64;
        if skip >= u64::from(self.volume.geometry.cluster_count) {
            return Ok(());
        }
        // Follow only FAT metadata over the prefix. Do not reread directory
        // data or allocate/decode old names when resuming a listing.
        for _ in 0..skip {
            let Some(next) = self.volume.next_cluster(cluster)? else {
                return Ok(());
            };
            cluster = next;
            traversed += 1;
        }
        let mut logical = skip * cluster_bytes as u64;
        let mut within = (start % cluster_bytes as u64) as usize / 32;
        let mut lfn = Vec::<[u8; 32]>::new();
        let mut bytes = zeroed(cluster_bytes)?;
        loop {
            traversed = traversed.checked_add(1).ok_or(Error::Io)?;
            if traversed > self.volume.geometry.cluster_count {
                return Err(Error::Io);
            }
            let absolute = self.volume.cluster_offset(cluster)?;
            if self
                .volume
                .metadata
                .read_at(absolute, &mut bytes)
                .map_err(Error::from)?
                != bytes.len()
            {
                return Err(Error::Io);
            }
            let (entries, _) = bytes.as_chunks::<32>();
            for (index, entry) in entries.iter().enumerate().skip(within) {
                match entry[0] {
                    0x00 => return Ok(()),
                    0xe5 => {
                        lfn.clear();
                        continue;
                    }
                    _ => {}
                }
                if entry[11] == ATTR_LONG_NAME {
                    let mut part = [0u8; 32];
                    part.copy_from_slice(entry);
                    // VFAT permits at most 20 slots (255 UTF-16 code units).
                    // Discard malformed prefixes instead of growing without bound.
                    if lfn.len() == 20 {
                        lfn.clear();
                    }
                    lfn.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
                    lfn.push(part);
                    continue;
                }
                if entry[11] & ATTR_VOLUME_ID != 0 {
                    lfn.clear();
                    continue;
                }
                let short: [u8; 11] = entry[..11].try_into().map_err(|_| Error::Io)?;
                let name =
                    decode_lfn(&lfn, &short).unwrap_or_else(|| decode_short(&short, entry[12]));
                lfn.clear();
                if name.as_ref() == b"." || name.as_ref() == b".." || name.is_empty() {
                    continue;
                }
                let first = u32::from(le16(entry, 26)) | (u32::from(le16(entry, 20)) << 16);
                let attributes = entry[11];
                if attributes & ATTR_DIRECTORY != 0 && first == 0 {
                    continue;
                }
                if !visit(Record {
                    name,
                    first_cluster: first,
                    size: u64::from(le32(entry, 28)),
                    attributes,
                    entry_offset: absolute + (index * 32) as u64,
                    next_offset: logical + ((index + 1) * 32) as u64,
                })? {
                    return Ok(());
                }
            }
            let Some(next) = self.volume.next_cluster(cluster)? else {
                return Ok(());
            };
            cluster = next;
            logical += cluster_bytes as u64;
            within = 0;
        }
    }

    fn vnode(self: &Arc<Self>, record: &Record, parent: &Vnode) -> Result<Vnode> {
        self.cache.get_or_create(record.entry_offset, || {
            Ok(Vnode::new(
                record.key(self.id),
                record.kind(),
                Box::new(FatNode {
                    filesystem: Arc::downgrade(self),
                    kind: record.kind(),
                    first_cluster: record.first_cluster,
                    size: record.size,
                    entry_offset: Some(record.entry_offset),
                    parent: Some(parent.clone()),
                }),
            ))
        })
    }
}

impl FileSystem for Fat32 {
    fn id(&self) -> FilesystemId {
        self.id
    }

    fn name(&self) -> &'static str {
        FAT32_NAME
    }

    fn root(&self) -> Vnode {
        self.root.get().cloned().expect("fat32: root initialized")
    }

    fn statfs(&self) -> StatFs {
        let total_bytes = u64::from(self.volume.geometry.cluster_count)
            * u64::from(self.volume.geometry.bytes_per_cluster);
        let used_bytes = self.volume.geometry.free_clusters.map_or(0, |free| {
            u64::from(self.volume.geometry.cluster_count - free)
                * u64::from(self.volume.geometry.bytes_per_cluster)
        });
        StatFs {
            total_bytes,
            used_bytes,
            total_nodes: u64::from(self.volume.geometry.cluster_count),
            used_nodes: 0,
        }
    }

    fn sync(&self) -> Result<()> {
        self.volume.device().flush().map_err(Error::from)
    }
}

impl FatNode {
    fn filesystem(&self) -> Result<Arc<Fat32>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }

    fn directory_cluster(&self) -> Result<u32> {
        if self.kind == VnodeKind::Directory {
            Ok(self.first_cluster)
        } else {
            Err(Error::NotDirectory)
        }
    }
}

impl VnodeOps for FatNode {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        Ok(VnodeAttr {
            key: vnode.key(),
            kind: self.kind,
            size: self.size,
            links: if self.kind == VnodeKind::Directory {
                2
            } else {
                1
            },
            // FAT has no Unix execute bits. Treat all visible files as
            // executable/readable, as other Unix FAT implementations do when
            // no per-file mode metadata exists.
            mode: 0o555,
            accessed_ns: 0,
            modified_ns: 0,
            changed_ns: 0,
        })
    }

    fn setattr(&self, _vnode: &Vnode, _attr: SetAttr) -> Result<()> {
        Err(Error::ReadOnly)
    }

    fn lookup(&self, directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        let filesystem = self.filesystem()?;
        let mut found = None;
        filesystem.visit_directory(self.directory_cluster()?, 0, |record| {
            if names_equal(&record.name, name) {
                found = Some(record);
            }
            Ok(found.is_none())
        })?;
        filesystem.vnode(&found.ok_or(Error::NotFound)?, directory)
    }

    fn parent(&self, directory: &Vnode) -> Result<Vnode> {
        if self.entry_offset.is_none() {
            Ok(directory.clone())
        } else {
            self.parent.clone().ok_or(Error::Io)
        }
    }

    fn create(
        &self,
        _directory: &Vnode,
        _name: &[u8],
        _kind: CreateKind,
        _mode: u16,
    ) -> Result<Vnode> {
        Err(Error::ReadOnly)
    }

    fn unlink(&self, _directory: &Vnode, _name: &[u8], _remove_directory: bool) -> Result<()> {
        Err(Error::ReadOnly)
    }

    fn rename(
        &self,
        _source: &Vnode,
        _source_name: &[u8],
        _target: &Vnode,
        _target_name: &[u8],
    ) -> Result<()> {
        Err(Error::ReadOnly)
    }

    fn truncate(&self, _vnode: &Vnode, _size: u64) -> Result<()> {
        Err(Error::ReadOnly)
    }

    fn memory_object(&self, _vnode: &Vnode) -> Result<Arc<VmObject>> {
        if self.kind == VnodeKind::Directory {
            return Err(Error::IsDirectory);
        }

        let filesystem = self.filesystem()?;
        let key = self.entry_offset.ok_or(Error::Io)?;
        filesystem.objects.get_or_create(key, || {
            Ok(VmObject::with_pager(
                self.size,
                Arc::new(FatPager {
                    volume: filesystem.volume.clone(),
                    first_cluster: self.first_cluster,
                    size: self.size,
                    chain: Mutex::new(ClusterChain::default()),
                }),
            ))
        })
    }

    fn readdir(
        &self,
        directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        let mut result = Vec::new();
        if maximum == 0 {
            return Ok((result, cursor));
        }
        let mut next = cursor;
        if next == 0 {
            result.push(DirEntry {
                name: Arc::from(&b"."[..]),
                key: directory.key(),
                kind: VnodeKind::Directory,
                offset: 1,
            });
            next = 1;
        }
        if next == 1 && result.len() < maximum {
            let parent = self.parent(directory)?;
            result.push(DirEntry {
                name: Arc::from(&b".."[..]),
                key: parent.key(),
                kind: VnodeKind::Directory,
                offset: DIRECTORY_CURSOR_BASE,
            });
            next = DIRECTORY_CURSOR_BASE;
        }
        let filesystem = self.filesystem()?;
        if result.len() == maximum {
            return Ok((result, next));
        }
        let start = next.saturating_sub(DIRECTORY_CURSOR_BASE);
        filesystem.visit_directory(self.directory_cluster()?, start, |record| {
            result.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
            let key = record.key(filesystem.id);
            let kind = record.kind();
            next = record
                .next_offset
                .checked_add(DIRECTORY_CURSOR_BASE)
                .ok_or(Error::Io)?;
            result.push(DirEntry {
                name: record.name,
                key,
                kind,
                offset: next,
            });
            Ok(result.len() < maximum)
        })?;
        Ok((result, next))
    }

    fn fsync(&self, _vnode: &Vnode) -> Result<()> {
        self.filesystem()?.sync()
    }
}

fn parse_boot(boot: &[u8; 512], device_bytes: u64) -> Result<Geometry> {
    if boot[510..] != [0x55, 0xaa] {
        return Err(Error::InvalidArgument);
    }
    let bytes_per_sector = u32::from(le16(boot, 11));
    let sectors_per_cluster = u32::from(boot[13]);
    let reserved = u32::from(le16(boot, 14));
    let fats = boot[16];
    let root_entries = le16(boot, 17);
    let total16 = le16(boot, 19);
    let fat16 = le16(boot, 22);
    let total_sectors = le32(boot, 32);
    let fat_sectors = le32(boot, 36);
    let flags = le16(boot, 40);
    let version = le16(boot, 42);
    let root_cluster = le32(boot, 44) & FAT_ENTRY_MASK;
    let fs_info_sector = u32::from(le16(boot, 48));
    if !matches!(bytes_per_sector, 512 | 1024 | 2048 | 4096)
        || sectors_per_cluster == 0
        || !sectors_per_cluster.is_power_of_two()
        || sectors_per_cluster > 128
        || reserved == 0
        || fats == 0
        || root_entries != 0
        || total16 != 0
        || fat16 != 0
        || total_sectors == 0
        || fat_sectors == 0
        || version != 0
    {
        return Err(Error::InvalidArgument);
    }
    let first_data_sector = reserved
        .checked_add(
            u32::from(fats)
                .checked_mul(fat_sectors)
                .ok_or(Error::InvalidArgument)?,
        )
        .ok_or(Error::InvalidArgument)?;
    if first_data_sector >= total_sectors {
        return Err(Error::InvalidArgument);
    }
    let cluster_count = (total_sectors - first_data_sector) / sectors_per_cluster;
    if cluster_count < 65_525 || root_cluster < 2 || root_cluster >= cluster_count + 2 {
        return Err(Error::InvalidArgument);
    }
    let total_bytes = u64::from(total_sectors)
        .checked_mul(u64::from(bytes_per_sector))
        .filter(|bytes| *bytes <= device_bytes)
        .ok_or(Error::InvalidArgument)?;
    let fat_bytes = u64::from(fat_sectors) * u64::from(bytes_per_sector);
    if u64::from(cluster_count + 2) * 4 > fat_bytes {
        return Err(Error::InvalidArgument);
    }
    let active_fat = if flags & 0x80 != 0 {
        (flags & 0x0f) as u8
    } else {
        0
    };
    if active_fat >= fats {
        return Err(Error::InvalidArgument);
    }
    Ok(Geometry {
        bytes_per_cluster: bytes_per_sector * sectors_per_cluster,
        fat_offset: u64::from(reserved) * u64::from(bytes_per_sector),
        fat_bytes,
        data_offset: u64::from(first_data_sector) * u64::from(bytes_per_sector),
        total_bytes,
        cluster_count,
        root_cluster,
        active_fat,
        fs_info_offset: (fs_info_sector != 0 && fs_info_sector < reserved)
            .then_some(u64::from(fs_info_sector) * u64::from(bytes_per_sector)),
        free_clusters: None,
    })
}

fn decode_short(short: &[u8; 11], case: u8) -> Arc<[u8]> {
    let mut output = Vec::with_capacity(12);
    let mut base = short[..8]
        .iter()
        .copied()
        .take_while(|byte| *byte != b' ')
        .collect::<Vec<_>>();
    if base.first() == Some(&0x05) {
        base[0] = 0xe5;
    }
    if case & 0x08 != 0 {
        base.make_ascii_lowercase();
    }
    output.extend_from_slice(&base);
    let mut extension = short[8..]
        .iter()
        .copied()
        .take_while(|byte| *byte != b' ')
        .collect::<Vec<_>>();
    if !extension.is_empty() {
        if case & 0x10 != 0 {
            extension.make_ascii_lowercase();
        }
        output.push(b'.');
        output.extend_from_slice(&extension);
    }
    Arc::from(output.into_boxed_slice())
}

fn decode_lfn(parts: &[[u8; 32]], short: &[u8; 11]) -> Option<Arc<[u8]>> {
    let first = parts.first()?;
    let count = usize::from(first[0] & 0x1f);
    if first[0] & 0xe0 != 0x40 || count == 0 || count > 20 || count != parts.len() {
        return None;
    }
    let checksum = short_checksum(short);
    for (index, part) in parts.iter().enumerate() {
        let expected = (count - index) as u8 | if index == 0 { 0x40 } else { 0 };
        if part[0] != expected || part[13] != checksum || part[12] != 0 || le16(part, 26) != 0 {
            return None;
        }
    }
    let mut units = Vec::with_capacity(count * 13);
    for part in parts.iter().rev() {
        for offset in [1usize, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30] {
            let unit = le16(part, offset);
            if unit == 0 {
                return utf16_name(&units);
            }
            if unit != 0xffff {
                units.push(unit);
            }
        }
    }
    utf16_name(&units)
}

fn utf16_name(units: &[u16]) -> Option<Arc<[u8]>> {
    let mut name = String::new();
    for decoded in char::decode_utf16(units.iter().copied()) {
        name.push(decoded.ok()?);
    }
    if name.is_empty()
        || name.len() > super::path::MAX_NAME_LEN
        || name.as_bytes().contains(&b'/')
        || name.as_bytes().contains(&0)
    {
        None
    } else {
        Some(Arc::from(name.into_bytes().into_boxed_slice()))
    }
}

fn short_checksum(short: &[u8; 11]) -> u8 {
    short
        .iter()
        .fold(0u8, |sum, byte| sum.rotate_right(1).wrapping_add(*byte))
}

fn names_equal(left: &[u8], right: &[u8]) -> bool {
    left.len() == right.len()
        && left
            .iter()
            .zip(right)
            .all(|(left, right)| left.eq_ignore_ascii_case(right))
}

fn le16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn le32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap_or([0; 4]))
}

fn zeroed(length: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| Error::OutOfMemory)?;
    bytes.resize(length, 0);
    Ok(bytes)
}
