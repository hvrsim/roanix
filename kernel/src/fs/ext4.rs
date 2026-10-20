//! Read-only ext4 filesystem driver.
//!
//! The first implementation deliberately accepts a narrow, explicit feature
//! set: extent-mapped files, typed directory entries, ordinary sparse files,
//! and fast or extent-backed symbolic links. Journals, indexed directories,
//! bigalloc, encryption, inline data, and metadata checksums are rejected at
//! mount time instead of being interpreted partially.

use alloc::{
    boxed::Box,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::any::Any;

use crate::{
    dev::block::{BlockClaim, BlockDevice},
    mem::{IoSink, IoSource, VmObject},
    sys::sync::Once,
};

use super::{
    Error, Result,
    vnode::{
        CreateKind, DirEntry, FileSystem, FilesystemId, NodeId, SetAttr, StatFs, Vnode, VnodeAttr,
        VnodeCache, VnodeKey, VnodeKind, VnodeOps,
    },
};

const EXT4_NAME: &str = "ext4";
const SUPER_OFFSET: u64 = 1024;
const SUPER_SIZE: usize = 1024;
const SUPER_MAGIC: u16 = 0xef53;
const ROOT_INODE: u64 = 2;
const EXTENT_MAGIC: u16 = 0xf30a;
const EXTENTS_FLAG: u32 = 0x0008_0000;
const DIRECTORY_CURSOR_BASE: u64 = 2;

const MODE_TYPE: u16 = 0xf000;
const MODE_FIFO: u16 = 0x1000;
const MODE_CHARACTER: u16 = 0x2000;
const MODE_DIRECTORY: u16 = 0x4000;
const MODE_BLOCK: u16 = 0x6000;
const MODE_REGULAR: u16 = 0x8000;
const MODE_SYMLINK: u16 = 0xa000;
const MODE_SOCKET: u16 = 0xc000;

const COMPAT_DIRECTORY_INDEX: u32 = 0x0020;
const INCOMPAT_FILETYPE: u32 = 0x0002;
const INCOMPAT_RECOVER: u32 = 0x0004;
const INCOMPAT_EXTENTS: u32 = 0x0040;
const INCOMPAT_64BIT: u32 = 0x0080;
const INCOMPAT_FLEX_BG: u32 = 0x0200;
const SUPPORTED_INCOMPAT: u32 =
    INCOMPAT_FILETYPE | INCOMPAT_EXTENTS | INCOMPAT_64BIT | INCOMPAT_FLEX_BG;
const RO_COMPAT_SPARSE_SUPER: u32 = 0x0001;
const RO_COMPAT_LARGE_FILE: u32 = 0x0002;
const RO_COMPAT_HUGE_FILE: u32 = 0x0008;
const RO_COMPAT_DIR_NLINK: u32 = 0x0020;
const RO_COMPAT_EXTRA_ISIZE: u32 = 0x0040;
const SUPPORTED_RO_COMPAT: u32 = RO_COMPAT_SPARSE_SUPER
    | RO_COMPAT_LARGE_FILE
    | RO_COMPAT_HUGE_FILE
    | RO_COMPAT_DIR_NLINK
    | RO_COMPAT_EXTRA_ISIZE;

struct Geometry {
    block_size: u32,
    block_count: u64,
    free_blocks: u64,
    inode_count: u64,
    free_inodes: u64,
    inodes_per_group: u32,
    inode_size: u16,
    group_count: usize,
    inode_tables: Vec<u64>,
}

#[derive(Clone, Copy)]
struct Extent {
    logical: u32,
    physical: u64,
    length: u32,
    initialized: bool,
}

struct Inode {
    number: u64,
    kind: VnodeKind,
    mode: u16,
    size: u64,
    links: u16,
    accessed_ns: u64,
    modified_ns: u64,
    changed_ns: u64,
    extents: Vec<Extent>,
    inline_symlink: Option<Box<[u8]>>,
}

/// One mounted ext4 volume.
pub struct Ext4 {
    id: FilesystemId,
    claim: BlockClaim,
    geometry: Geometry,
    root: Once<Vnode>,
    cache: VnodeCache,
    metadata: Arc<VmObject>,
    objects: super::cache::ObjectCache,
}

struct Ext4Node {
    filesystem: Weak<Ext4>,
    inode: Inode,
    parent: Option<Vnode>,
}

impl Ext4 {
    /// Probes and mounts a compatible ext4 volume on `device`.
    pub fn new(device: Arc<BlockDevice>) -> Result<Arc<Self>> {
        let claim = device.claim().map_err(Error::from)?;
        let mut superblock = [0u8; SUPER_SIZE];
        read_exact(&device, SUPER_OFFSET, &mut superblock)?;
        let mut geometry = parse_superblock(&superblock, device.size_bytes())?;
        geometry.inode_tables = read_inode_tables(&device, &superblock, &geometry)?;

        let filesystem = Arc::new(Self {
            id: FilesystemId::allocate(),
            metadata: super::io::metadata_object(&device)?,
            claim,
            geometry,
            root: Once::new(),
            cache: VnodeCache::new(),
            objects: super::cache::ObjectCache::new(),
        });
        let inode = filesystem.read_inode(ROOT_INODE)?;
        if inode.kind != VnodeKind::Directory {
            return Err(Error::InvalidArgument);
        }
        let root = Vnode::new(
            VnodeKey {
                filesystem: filesystem.id,
                node: NodeId::new(ROOT_INODE),
            },
            VnodeKind::Directory,
            Box::new(Ext4Node {
                filesystem: Arc::downgrade(&filesystem),
                inode,
                parent: None,
            }),
        );
        filesystem.root.call_once(|| root);
        Ok(filesystem)
    }

    fn device(&self) -> &Arc<BlockDevice> {
        self.claim.device()
    }

    fn block_offset(&self, block: u64) -> Result<u64> {
        if block >= self.geometry.block_count {
            return Err(Error::Io);
        }
        block
            .checked_mul(u64::from(self.geometry.block_size))
            .ok_or(Error::Io)
    }

    fn read_block(&self, block: u64) -> Result<Vec<u8>> {
        let mut bytes = zeroed(self.geometry.block_size as usize)?;
        self.metadata
            .read_at(self.block_offset(block)?, &mut bytes)
            .map_err(Error::from)?;
        Ok(bytes)
    }

    fn read_inode(&self, number: u64) -> Result<Inode> {
        if number == 0 || number > self.geometry.inode_count {
            return Err(Error::NotFound);
        }
        let index = number - 1;
        let group = index / u64::from(self.geometry.inodes_per_group);
        let within = index % u64::from(self.geometry.inodes_per_group);
        let table = *self
            .geometry
            .inode_tables
            .get(usize::try_from(group).map_err(|_| Error::Io)?)
            .ok_or(Error::Io)?;
        let offset = self
            .block_offset(table)?
            .checked_add(within * u64::from(self.geometry.inode_size))
            .ok_or(Error::Io)?;
        let mut bytes = zeroed(self.geometry.inode_size.into())?;
        self.metadata
            .read_at(offset, &mut bytes)
            .map_err(Error::from)?;

        let mode = le16(&bytes, 0)?;
        let kind = mode_kind(mode)?;
        let links = le16(&bytes, 26)?;
        if links == 0 {
            return Err(Error::NotFound);
        }
        let size = u64::from(le32(&bytes, 4)?) | (u64::from(le32(&bytes, 108)?) << 32);
        let flags = le32(&bytes, 32)?;
        let blocks = le32(&bytes, 28)?;
        let block_bytes = bytes.get(40..100).ok_or(Error::Io)?;
        let fast_symlink = kind == VnodeKind::Symlink && size <= 60 && blocks == 0;
        let inline_symlink = if fast_symlink {
            let length = usize::try_from(size).map_err(|_| Error::FileTooLarge)?;
            Some(Box::from(block_bytes.get(..length).ok_or(Error::Io)?))
        } else {
            None
        };

        let mut extents = Vec::new();
        if size != 0 && !fast_symlink {
            if flags & EXTENTS_FLAG == 0 {
                return Err(Error::Unsupported);
            }
            self.collect_extents(block_bytes, None, &mut extents)?;
            extents.sort_unstable_by_key(|extent| extent.logical);
            validate_extents(&extents, self.geometry.block_count)?;
        }

        Ok(Inode {
            number,
            kind,
            mode: mode & 0o7777,
            size,
            links,
            accessed_ns: seconds_ns(le32(&bytes, 8)?),
            changed_ns: seconds_ns(le32(&bytes, 12)?),
            modified_ns: seconds_ns(le32(&bytes, 16)?),
            extents,
            inline_symlink,
        })
    }

    fn collect_extents(
        &self,
        bytes: &[u8],
        expected_depth: Option<u16>,
        output: &mut Vec<Extent>,
    ) -> Result<()> {
        if le16(bytes, 0)? != EXTENT_MAGIC {
            return Err(Error::Io);
        }
        let entries = usize::from(le16(bytes, 2)?);
        let maximum = usize::from(le16(bytes, 4)?);
        let depth = le16(bytes, 6)?;
        if expected_depth.is_some_and(|expected| expected != depth) || depth > 5 {
            return Err(Error::Io);
        }
        let capacity = bytes.len().saturating_sub(12) / 12;
        if entries > maximum || maximum > capacity {
            return Err(Error::Io);
        }
        output
            .try_reserve(entries)
            .map_err(|_| Error::OutOfMemory)?;

        let mut previous = None;
        for index in 0..entries {
            let offset = 12 + index * 12;
            let logical = le32(bytes, offset)?;
            if previous.is_some_and(|value| logical <= value) {
                return Err(Error::Io);
            }
            previous = Some(logical);
            if depth == 0 {
                let raw_length = le16(bytes, offset + 4)?;
                if raw_length == 0 {
                    return Err(Error::Io);
                }
                let initialized = raw_length <= 0x8000;
                let length = if raw_length > 0x8000 {
                    u32::from(raw_length - 0x8000)
                } else {
                    u32::from(raw_length)
                };
                let physical = u64::from(le32(bytes, offset + 8)?)
                    | (u64::from(le16(bytes, offset + 6)?) << 32);
                output.push(Extent {
                    logical,
                    physical,
                    length,
                    initialized,
                });
            } else {
                let child = u64::from(le32(bytes, offset + 4)?)
                    | (u64::from(le16(bytes, offset + 8)?) << 32);
                let child_bytes = self.read_block(child)?;
                self.collect_extents(&child_bytes, Some(depth - 1), output)?;
            }
        }
        Ok(())
    }

    fn vnode(self: &Arc<Self>, number: u64, parent: &Vnode) -> Result<Vnode> {
        self.cache.get_or_create(number, || {
            let inode = self.read_inode(number)?;
            Ok(Vnode::new(
                VnodeKey {
                    filesystem: self.id,
                    node: NodeId::new(number),
                },
                inode.kind,
                Box::new(Ext4Node {
                    filesystem: Arc::downgrade(self),
                    inode,
                    parent: Some(parent.clone()),
                }),
            ))
        })
    }

    fn extent_at<'a>(&self, extents: &'a [Extent], logical: u32) -> Option<&'a Extent> {
        let index = extents.partition_point(|extent| extent.logical <= logical);
        let extent = extents.get(index.checked_sub(1)?)?;
        (logical < extent.logical.checked_add(extent.length)?).then_some(extent)
    }

    fn next_extent(&self, extents: &[Extent], logical: u32) -> Option<u32> {
        extents
            .get(extents.partition_point(|extent| extent.logical <= logical))
            .map(|extent| extent.logical)
    }

    fn read_data(&self, inode: &Inode, offset: u64, sink: &mut IoSink<'_>) -> Result<usize> {
        if offset >= inode.size || sink.is_empty() {
            return Ok(0);
        }
        let length = sink
            .len()
            .min(usize::try_from(inode.size - offset).unwrap_or(usize::MAX));
        let block_size = u64::from(self.geometry.block_size);
        let mut done = 0usize;
        while done < length {
            let at = offset.checked_add(done as u64).ok_or(Error::Io)?;
            let logical = u32::try_from(at / block_size).map_err(|_| Error::FileTooLarge)?;
            let within = at % block_size;
            if let Some(extent) = self.extent_at(&inode.extents, logical) {
                let available = u64::from(extent.logical + extent.length - logical)
                    .checked_mul(block_size)
                    .and_then(|bytes| bytes.checked_sub(within))
                    .ok_or(Error::Io)?;
                let take = (length - done).min(usize::try_from(available).unwrap_or(usize::MAX));
                if extent.initialized {
                    let physical = extent
                        .physical
                        .checked_add(u64::from(logical - extent.logical))
                        .ok_or(Error::Io)?;
                    let disk_offset = self
                        .block_offset(physical)?
                        .checked_add(within)
                        .ok_or(Error::Io)?;
                    if inode.kind == VnodeKind::Directory {
                        let mut target = sink.slice(done, take).map_err(Error::from)?;
                        if self
                            .metadata
                            .read_into(disk_offset, &mut target)
                            .map_err(Error::from)?
                            != take
                        {
                            return Err(Error::Io);
                        }
                    } else {
                        self.read_into_sink(disk_offset, sink, done, take)?;
                    }
                } else {
                    sink.fill_zero(done, take).map_err(Error::from)?;
                }
                done += take;
            } else {
                let next = self.next_extent(&inode.extents, logical);
                let hole_end = next.map_or(inode.size, |block| u64::from(block) * block_size);
                let take = (length - done)
                    .min(usize::try_from(hole_end.saturating_sub(at)).unwrap_or(usize::MAX));
                if take == 0 {
                    return Err(Error::Io);
                }
                sink.fill_zero(done, take).map_err(Error::from)?;
                done += take;
            }
        }
        Ok(done)
    }

    fn read_into_sink(
        &self,
        disk_offset: u64,
        sink: &mut IoSink<'_>,
        sink_offset: usize,
        length: usize,
    ) -> Result<()> {
        let mut copied = 0usize;
        let mut buffer = [0u8; 1024];
        while copied < length {
            let mut window = sink
                .window(sink_offset + copied, (length - copied).min(buffer.len()))
                .map_err(Error::from)?;
            if window.is_empty() {
                return Err(Error::InvalidArgument);
            }
            let count = window.len();
            read_exact(
                self.device(),
                disk_offset.checked_add(copied as u64).ok_or(Error::Io)?,
                &mut buffer[..count],
            )?;
            window.copy_from_slice(&buffer[..count]);
            copied += count;
        }
        Ok(())
    }

    /// Walk borrowed names in disk order. A false visitor result stops I/O.
    fn visit_directory(
        &self,
        inode: &Inode,
        start: u64,
        mut visit: impl FnMut(&[u8], u64, Option<VnodeKind>, u64) -> Result<bool>,
    ) -> Result<()> {
        if inode.kind != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
        if start >= inode.size {
            return Ok(());
        }
        let block_size = self.geometry.block_size as usize;
        let mut bytes = zeroed(block_size)?;
        // Cookies are byte offsets. Resume at the containing block rather
        // than scanning the directory prefix on every getdents batch.
        let mut offset = start / block_size as u64 * block_size as u64;
        while offset < inode.size {
            let length = block_size.min(usize::try_from(inode.size - offset).unwrap_or(usize::MAX));
            if self.read_data(inode, offset, &mut IoSink::kernel(&mut bytes[..length]))? != length {
                return Err(Error::Io);
            }
            let mut position = 0usize;
            while position < length {
                let record = bytes[..length].get(position..).ok_or(Error::Io)?;
                let number = u64::from(le32(record, 0)?);
                let record_length = usize::from(le16(record, 4)?);
                let name_length = usize::from(*record.get(6).ok_or(Error::Io)?);
                if record_length < 8
                    || !record_length.is_multiple_of(4)
                    || position + record_length > length
                    || name_length > record_length - 8
                {
                    return Err(Error::Io);
                }
                if number != 0 {
                    if number > self.geometry.inode_count {
                        return Err(Error::Io);
                    }
                    let name = record.get(8..8 + name_length).ok_or(Error::Io)?;
                    if name.is_empty() || name.contains(&b'/') || name.contains(&0) {
                        return Err(Error::Io);
                    }
                    let entry_offset = offset + position as u64;
                    if entry_offset >= start && name != b"." && name != b".." {
                        let kind = match record[7] {
                            0 => None,
                            1 => Some(VnodeKind::Regular),
                            2 => Some(VnodeKind::Directory),
                            3 => Some(VnodeKind::CharacterDevice),
                            4 => Some(VnodeKind::BlockDevice),
                            5 => Some(VnodeKind::Fifo),
                            6 => Some(VnodeKind::Socket),
                            7 => Some(VnodeKind::Symlink),
                            _ => return Err(Error::Io),
                        };
                        let next = entry_offset
                            .checked_add(record_length as u64)
                            .ok_or(Error::Io)?;
                        if !visit(name, number, kind, next)? {
                            return Ok(());
                        }
                    }
                }
                position += record_length;
            }
            offset = offset.checked_add(length as u64).ok_or(Error::Io)?;
        }
        Ok(())
    }
}

impl FileSystem for Ext4 {
    fn id(&self) -> FilesystemId {
        self.id
    }

    fn name(&self) -> &'static str {
        EXT4_NAME
    }

    fn root(&self) -> Vnode {
        self.root.get().cloned().expect("ext4: root initialized")
    }

    fn statfs(&self) -> StatFs {
        let block_size = u64::from(self.geometry.block_size);
        StatFs {
            total_bytes: self.geometry.block_count.saturating_mul(block_size),
            used_bytes: self
                .geometry
                .block_count
                .saturating_sub(self.geometry.free_blocks)
                .saturating_mul(block_size),
            total_nodes: self.geometry.inode_count,
            used_nodes: self
                .geometry
                .inode_count
                .saturating_sub(self.geometry.free_inodes),
        }
    }

    fn sync(&self) -> Result<()> {
        self.device().flush().map_err(Error::from)
    }
}

impl Ext4Node {
    fn filesystem(&self) -> Result<Arc<Ext4>> {
        self.filesystem.upgrade().ok_or(Error::Io)
    }

    fn require_directory(&self) -> Result<()> {
        if self.inode.kind == VnodeKind::Directory {
            Ok(())
        } else {
            Err(Error::NotDirectory)
        }
    }
}

impl VnodeOps for Ext4Node {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn getattr(&self, vnode: &Vnode) -> Result<VnodeAttr> {
        Ok(VnodeAttr {
            key: vnode.key(),
            kind: self.inode.kind,
            size: self.inode.size,
            links: u64::from(self.inode.links),
            mode: self.inode.mode,
            accessed_ns: self.inode.accessed_ns,
            modified_ns: self.inode.modified_ns,
            changed_ns: self.inode.changed_ns,
        })
    }

    fn setattr(&self, _vnode: &Vnode, _attr: SetAttr) -> Result<()> {
        Err(Error::ReadOnly)
    }

    fn lookup(&self, directory: &Vnode, name: &[u8]) -> Result<Vnode> {
        self.require_directory()?;
        let filesystem = self.filesystem()?;
        let mut found = None;
        filesystem.visit_directory(&self.inode, 0, |candidate, number, _, _| {
            if candidate == name {
                found = Some(number);
            }
            Ok(found.is_none())
        })?;
        filesystem.vnode(found.ok_or(Error::NotFound)?, directory)
    }

    fn parent(&self, directory: &Vnode) -> Result<Vnode> {
        self.require_directory()?;
        if self.inode.number == ROOT_INODE {
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

    fn write_at(&self, _vnode: &Vnode, _offset: u64, _source: &IoSource<'_>) -> Result<usize> {
        Err(Error::ReadOnly)
    }

    fn truncate(&self, _vnode: &Vnode, _size: u64) -> Result<()> {
        Err(Error::ReadOnly)
    }

    fn memory_object(&self, _vnode: &Vnode) -> Result<Arc<VmObject>> {
        if self.inode.kind != VnodeKind::Regular {
            return Err(if self.inode.kind == VnodeKind::Directory {
                Error::IsDirectory
            } else {
                Error::Unsupported
            });
        }
        let filesystem = self.filesystem()?;
        filesystem.objects.get_or_create(self.inode.number, || {
            let bs = u64::from(filesystem.geometry.block_size);
            let mut extents = Vec::new();
            extents
                .try_reserve_exact(self.inode.extents.len())
                .map_err(|_| Error::OutOfMemory)?;
            for extent in &self.inode.extents {
                if extent.initialized {
                    extents.push(super::io::FileExtent {
                        logical: u64::from(extent.logical) * bs,
                        physical: extent.physical * bs,
                        length: u64::from(extent.length) * bs,
                    });
                }
            }
            super::io::ExtentPager::object(filesystem.device(), self.inode.size, extents)
        })
    }

    fn readlink(&self, _vnode: &Vnode) -> Result<Box<[u8]>> {
        if self.inode.kind != VnodeKind::Symlink {
            return Err(Error::InvalidArgument);
        }
        if let Some(target) = &self.inode.inline_symlink {
            return Ok(target.clone());
        }
        let size = usize::try_from(self.inode.size).map_err(|_| Error::FileTooLarge)?;
        let mut target = zeroed(size)?;
        let read =
            self.filesystem()?
                .read_data(&self.inode, 0, &mut IoSink::kernel(&mut target))?;
        if read != size || target.contains(&0) {
            return Err(Error::Io);
        }
        Ok(target.into_boxed_slice())
    }

    fn readdir(
        &self,
        directory: &Vnode,
        cursor: u64,
        maximum: usize,
    ) -> Result<(Vec<DirEntry>, u64)> {
        self.require_directory()?;
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
        filesystem.visit_directory(&self.inode, start, |name, number, kind, offset| {
            // Typed ext4 entries already provide identity and type. Listing
            // them need not read an inode or activate a vnode for each name.
            let kind = match kind {
                Some(kind) => kind,
                None => filesystem.read_inode(number)?.kind,
            };
            result.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
            next = offset.checked_add(DIRECTORY_CURSOR_BASE).ok_or(Error::Io)?;
            result.push(DirEntry {
                name: Arc::from(name),
                key: VnodeKey {
                    filesystem: filesystem.id,
                    node: NodeId::new(number),
                },
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

fn parse_superblock(bytes: &[u8; SUPER_SIZE], device_bytes: u64) -> Result<Geometry> {
    if le16(bytes, 56)? != SUPER_MAGIC {
        return Err(Error::InvalidArgument);
    }
    let log_block_size = le32(bytes, 24)?;
    if log_block_size > 2 || le32(bytes, 28)? != log_block_size {
        return Err(Error::Unsupported);
    }
    let block_size = 1024u32
        .checked_shl(log_block_size)
        .ok_or(Error::InvalidArgument)?;
    let incompat = le32(bytes, 96)?;
    let ro_compat = le32(bytes, 100)?;
    let compat = le32(bytes, 92)?;
    if incompat & !SUPPORTED_INCOMPAT != 0
        || incompat & INCOMPAT_EXTENTS == 0
        || incompat & INCOMPAT_FILETYPE == 0
        || incompat & INCOMPAT_RECOVER != 0
        || ro_compat & !SUPPORTED_RO_COMPAT != 0
        || compat & COMPAT_DIRECTORY_INDEX != 0
    {
        return Err(Error::Unsupported);
    }
    let has_64bit = incompat & INCOMPAT_64BIT != 0;
    let block_count = u64::from(le32(bytes, 4)?)
        | if has_64bit {
            u64::from(le32(bytes, 336)?) << 32
        } else {
            0
        };
    let free_blocks = u64::from(le32(bytes, 12)?)
        | if has_64bit {
            u64::from(le32(bytes, 344)?) << 32
        } else {
            0
        };
    let inode_count = u64::from(le32(bytes, 0)?);
    let free_inodes = u64::from(le32(bytes, 16)?);
    let first_data_block = u64::from(le32(bytes, 20)?);
    let blocks_per_group = le32(bytes, 32)?;
    let inodes_per_group = le32(bytes, 40)?;
    let inode_size = le16(bytes, 88)?;
    if block_count <= first_data_block
        || block_count.checked_mul(u64::from(block_size)).is_none()
        || block_count * u64::from(block_size) > device_bytes
        || free_blocks > block_count
        || inode_count < ROOT_INODE
        || free_inodes > inode_count
        || blocks_per_group == 0
        || inodes_per_group == 0
        || inode_size < 128
        || u32::from(inode_size) > block_size
        || !inode_size.is_multiple_of(4)
    {
        return Err(Error::InvalidArgument);
    }
    let groups = block_count
        .saturating_sub(first_data_block)
        .div_ceil(u64::from(blocks_per_group));
    let group_count = usize::try_from(groups).map_err(|_| Error::FileTooLarge)?;
    Ok(Geometry {
        block_size,
        block_count,
        free_blocks,
        inode_count,
        free_inodes,
        inodes_per_group,
        inode_size,
        group_count,
        inode_tables: Vec::new(),
    })
}

fn read_inode_tables(
    device: &BlockDevice,
    superblock: &[u8; SUPER_SIZE],
    geometry: &Geometry,
) -> Result<Vec<u64>> {
    let has_64bit = le32(superblock, 96)? & INCOMPAT_64BIT != 0;
    let descriptor_size = if has_64bit {
        usize::from(le16(superblock, 254)?).max(64)
    } else {
        32
    };
    if descriptor_size > geometry.block_size as usize || !descriptor_size.is_multiple_of(4) {
        return Err(Error::InvalidArgument);
    }
    let table_block = if geometry.block_size == 1024 {
        2u64
    } else {
        1u64
    };
    let table_offset = table_block * u64::from(geometry.block_size);
    let mut output = Vec::new();
    output
        .try_reserve_exact(geometry.group_count)
        .map_err(|_| Error::OutOfMemory)?;
    for group in 0..geometry.group_count {
        let mut descriptor = [0u8; 64];
        let read_length = descriptor_size.min(descriptor.len());
        read_exact(
            device,
            table_offset
                .checked_add((group * descriptor_size) as u64)
                .ok_or(Error::Io)?,
            &mut descriptor[..read_length],
        )?;
        let table = u64::from(le32(&descriptor, 8)?)
            | if has_64bit {
                u64::from(le32(&descriptor, 40)?) << 32
            } else {
                0
            };
        let table_bytes = u64::from(geometry.inodes_per_group)
            .checked_mul(u64::from(geometry.inode_size))
            .ok_or(Error::Io)?;
        let table_blocks = table_bytes.div_ceil(u64::from(geometry.block_size));
        if table == 0
            || table
                .checked_add(table_blocks)
                .is_none_or(|end| end > geometry.block_count)
        {
            return Err(Error::InvalidArgument);
        }
        output.push(table);
    }
    Ok(output)
}

fn validate_extents(extents: &[Extent], block_count: u64) -> Result<()> {
    let mut logical_end = 0u32;
    for (index, extent) in extents.iter().enumerate() {
        if extent.length == 0
            || (index != 0 && extent.logical < logical_end)
            || extent
                .physical
                .checked_add(u64::from(extent.length))
                .is_none_or(|end| end > block_count)
        {
            return Err(Error::Io);
        }
        logical_end = extent.logical.checked_add(extent.length).ok_or(Error::Io)?;
    }
    Ok(())
}

fn mode_kind(mode: u16) -> Result<VnodeKind> {
    match mode & MODE_TYPE {
        MODE_REGULAR => Ok(VnodeKind::Regular),
        MODE_DIRECTORY => Ok(VnodeKind::Directory),
        MODE_SYMLINK => Ok(VnodeKind::Symlink),
        MODE_CHARACTER => Ok(VnodeKind::CharacterDevice),
        MODE_BLOCK => Ok(VnodeKind::BlockDevice),
        MODE_FIFO => Ok(VnodeKind::Fifo),
        MODE_SOCKET => Ok(VnodeKind::Socket),
        _ => Err(Error::Unsupported),
    }
}

fn seconds_ns(seconds: u32) -> u64 {
    u64::from(seconds).saturating_mul(1_000_000_000)
}

fn read_exact(device: &BlockDevice, offset: u64, bytes: &mut [u8]) -> Result<()> {
    if device.read_bytes(offset, bytes).map_err(Error::from)? == bytes.len() {
        Ok(())
    } else {
        Err(Error::Io)
    }
}

fn le16(bytes: &[u8], offset: usize) -> Result<u16> {
    let value: [u8; 2] = bytes
        .get(offset..offset + 2)
        .ok_or(Error::Io)?
        .try_into()
        .map_err(|_| Error::Io)?;
    Ok(u16::from_le_bytes(value))
}

fn le32(bytes: &[u8], offset: usize) -> Result<u32> {
    let value: [u8; 4] = bytes
        .get(offset..offset + 4)
        .ok_or(Error::Io)?
        .try_into()
        .map_err(|_| Error::Io)?;
    Ok(u32::from_le_bytes(value))
}

fn zeroed(length: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| Error::OutOfMemory)?;
    bytes.resize(length, 0);
    Ok(bytes)
}
