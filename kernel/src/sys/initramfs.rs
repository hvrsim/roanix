//!
//! # Initial RAM Filesystem
//!
//! Access to and extraction of the userspace archive loaded as a Limine module.
//!

use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet},
    string::String,
    vec::Vec,
};
use core::{fmt, slice, str};

use limine::request::ModuleRequest;
use log::{debug, error, info};

use crate::fs::{
    self,
    vfs::{self as vfs_mod, PathAnchor},
};

const TAR_BLOCK_SIZE: usize = 512;
const TAR_NAME_RANGE: core::ops::Range<usize> = 0..100;
const TAR_MODE_RANGE: core::ops::Range<usize> = 100..108;
const TAR_SIZE_RANGE: core::ops::Range<usize> = 124..136;
const TAR_CHECKSUM_RANGE: core::ops::Range<usize> = 148..156;
const TAR_TYPE_OFFSET: usize = 156;
const TAR_LINK_RANGE: core::ops::Range<usize> = 157..257;
const TAR_MAGIC_RANGE: core::ops::Range<usize> = 257..263;
const TAR_PREFIX_RANGE: core::ops::Range<usize> = 345..500;

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
static MODULE_REQUEST: ModuleRequest = ModuleRequest::new();

/// Failure while parsing or importing the initial RAM filesystem.
#[derive(Debug)]
pub enum Error {
    /// The archive ended before a complete header or payload was available.
    Truncated,
    /// The archive did not contain a valid USTAR header.
    InvalidHeader,
    /// A header checksum did not match its contents.
    InvalidChecksum,
    /// A numeric header field was malformed or too large.
    InvalidNumber,
    /// An archive path was malformed, non-UTF-8, or escaped the archive root.
    InvalidPath,
    /// The archive contained the same path more than once.
    DuplicatePath,
    /// The archive ended without the required two zero blocks.
    MissingEndMarker,
    /// Non-zero bytes followed the archive end marker.
    TrailingData,
    /// The archive contains an entry type the importer does not materialize.
    UnsupportedEntry(u8),
    /// A hard-link target was not present in the imported archive.
    UnresolvedHardLink,
    /// The VFS rejected an import operation.
    Filesystem(fs::Error),
}

impl From<fs::Error> for Error {
    fn from(error: fs::Error) -> Self {
        Self::Filesystem(error)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated => formatter.write_str("truncated archive"),
            Self::InvalidHeader => formatter.write_str("invalid USTAR header"),
            Self::InvalidChecksum => formatter.write_str("invalid USTAR checksum"),
            Self::InvalidNumber => formatter.write_str("invalid USTAR numeric field"),
            Self::InvalidPath => formatter.write_str("invalid archive path"),
            Self::DuplicatePath => formatter.write_str("duplicate archive path"),
            Self::MissingEndMarker => formatter.write_str("missing USTAR end marker"),
            Self::TrailingData => formatter.write_str("non-zero data after USTAR end marker"),
            Self::UnsupportedEntry(kind) => {
                write!(formatter, "unsupported USTAR entry type {kind:#x}")
            }
            Self::UnresolvedHardLink => formatter.write_str("unresolved archive hard link"),
            Self::Filesystem(error) => write!(formatter, "filesystem error: {error}"),
        }
    }
}

type Result<T> = core::result::Result<T, Error>;

enum EntryKind<'a> {
    Regular(&'a [u8]),
    Directory,
    Symlink(String),
    HardLink(String),
}

struct Entry<'a> {
    path: String,
    mode: u16,
    kind: EntryKind<'a>,
}

struct TarArchive<'a> {
    bytes: &'a [u8],
    offset: usize,
    finished: bool,
}

impl<'a> TarArchive<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            offset: 0,
            finished: false,
        }
    }

    fn next(&mut self) -> Result<Option<Entry<'a>>> {
        if self.finished {
            return Ok(None);
        }

        let header_end = self
            .offset
            .checked_add(TAR_BLOCK_SIZE)
            .ok_or(Error::Truncated)?;
        let header = self
            .bytes
            .get(self.offset..header_end)
            .ok_or(Error::Truncated)?;

        if header.iter().all(|byte| *byte == 0) {
            let second_end = header_end
                .checked_add(TAR_BLOCK_SIZE)
                .ok_or(Error::MissingEndMarker)?;
            let second = self
                .bytes
                .get(header_end..second_end)
                .ok_or(Error::MissingEndMarker)?;
            if !second.iter().all(|byte| *byte == 0) {
                return Err(Error::MissingEndMarker);
            }
            if self.bytes[second_end..].iter().any(|byte| *byte != 0) {
                return Err(Error::TrailingData);
            }
            self.finished = true;
            self.offset = self.bytes.len();
            return Ok(None);
        }

        let magic = &header[TAR_MAGIC_RANGE];
        if magic != b"ustar\0" && magic != b"ustar " {
            return Err(Error::InvalidHeader);
        }

        let expected_checksum = parse_octal(&header[TAR_CHECKSUM_RANGE])?;
        let actual_checksum = header
            .iter()
            .enumerate()
            .map(|(index, byte)| {
                if TAR_CHECKSUM_RANGE.contains(&index) {
                    u64::from(b' ')
                } else {
                    u64::from(*byte)
                }
            })
            .sum::<u64>();
        if expected_checksum != actual_checksum {
            return Err(Error::InvalidChecksum);
        }

        let path = parse_entry_path(
            header_field(&header[TAR_PREFIX_RANGE])?,
            header_field(&header[TAR_NAME_RANGE])?,
        )?;
        let mode = u16::try_from(parse_octal(&header[TAR_MODE_RANGE])? & 0o7777)
            .map_err(|_| Error::InvalidNumber)?;
        let size = usize::try_from(parse_octal(&header[TAR_SIZE_RANGE])?)
            .map_err(|_| Error::InvalidNumber)?;
        let data_end = header_end.checked_add(size).ok_or(Error::Truncated)?;
        let data = self
            .bytes
            .get(header_end..data_end)
            .ok_or(Error::Truncated)?;
        let padded_size = size
            .checked_add(TAR_BLOCK_SIZE - 1)
            .ok_or(Error::Truncated)?
            & !(TAR_BLOCK_SIZE - 1);
        self.offset = header_end
            .checked_add(padded_size)
            .filter(|end| *end <= self.bytes.len())
            .ok_or(Error::Truncated)?;

        let kind = match header[TAR_TYPE_OFFSET] {
            0 | b'0' => EntryKind::Regular(data),
            b'1' => EntryKind::HardLink(parse_link_path(header_field(&header[TAR_LINK_RANGE])?)?),
            b'2' => EntryKind::Symlink(parse_symlink_target(header_field(
                &header[TAR_LINK_RANGE],
            )?)?),
            b'5' => EntryKind::Directory,
            kind => return Err(Error::UnsupportedEntry(kind)),
        };

        Ok(Some(Entry { path, mode, kind }))
    }
}

/// Iterator over regular files in the Limine initramfs.
///
/// Archive parsing remains private to this module; callers only see validated
/// archive paths and their payload bytes.
pub(crate) struct RegularFileIter<'a> {
    archive: TarArchive<'a>,
    failed: bool,
}

impl<'a> RegularFileIter<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self {
            archive: TarArchive::new(bytes),
            failed: false,
        }
    }
}

impl<'a> Iterator for RegularFileIter<'a> {
    type Item = Result<(String, &'a [u8])>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.failed {
            return None;
        }

        loop {
            match self.archive.next() {
                Ok(Some(entry)) => match entry.kind {
                    EntryKind::Regular(bytes) => return Some(Ok((entry.path, bytes))),
                    EntryKind::Directory | EntryKind::Symlink(_) | EntryKind::HardLink(_) => {}
                },
                Ok(None) => return None,
                Err(error) => {
                    self.failed = true;
                    return Some(Err(error));
                }
            }
        }
    }
}

/// Returns the initramfs archive decompressed by Limine.
pub fn archive() -> Option<&'static [u8]> {
    let response = MODULE_REQUEST.get_response()?;
    let module = response
        .modules()
        .iter()
        .copied()
        .find(|module| module.string().to_bytes() == b"initramfs")?;
    let len = usize::try_from(module.size()).ok()?;

    // SAFETY: Limine keeps module data mapped for the kernel lifetime and
    // reports the exact byte length in the module response.
    Some(unsafe { slice::from_raw_parts(module.addr(), len) })
}

/// Returns an iterator over regular files in the Limine initramfs.
pub(crate) fn regular_files() -> Option<RegularFileIter<'static>> {
    archive().map(RegularFileIter::new)
}

/// Imports the Limine initramfs into the mounted root filesystem.
pub(crate) fn populate() -> Result<usize> {
    let Some(bytes) = archive() else {
        return Ok(0);
    };

    let root = vfs_mod::root_anchor().map_err(Error::from)?;

    let mut archive = TarArchive::new(bytes);
    let mut imported = BTreeSet::new();
    // Anchors for directories whose VFS identity is already resolved. A
    // typical initramfs has thousands of files sharing a few hundred parent
    // directories, so caching turns per-entry path walks into a hash lookup.
    let mut dir_cache: BTreeMap<Box<str>, PathAnchor> = BTreeMap::new();
    dir_cache.insert(Box::from("/"), root.clone());
    let mut pending_links = Vec::new();
    let mut count = 0usize;

    while let Some(entry) = archive.next()? {
        if !imported.insert(entry.path.clone()) {
            return Err(Error::DuplicatePath);
        }

        let absolute = absolute_path(&entry.path);

        match entry.kind {
            EntryKind::Regular(data) => {
                let (dir_anchor, name) = cached_parent(&root, &mut dir_cache, &absolute)?;
                let vnode = vfs_mod::create_file_at(&dir_anchor, name.as_bytes(), entry.mode)?;
                write_all(&vnode, data)?;
            }
            EntryKind::Directory => {
                let (dir_anchor, name) = cached_parent(&root, &mut dir_cache, &absolute)?;
                match vfs_mod::create_dir_at(&dir_anchor, name.as_bytes(), entry.mode) {
                    Ok(_) | Err(fs::Error::AlreadyExists) => {
                        let anchor = vfs_mod::resolve_at(&root, absolute.as_bytes(), true)?;
                        dir_cache.insert(Box::from(absolute.as_str()), anchor);
                    }
                    Err(error) => return Err(error.into()),
                }
            }
            EntryKind::Symlink(target) => {
                let (dir_anchor, name) = cached_parent(&root, &mut dir_cache, &absolute)?;
                vfs_mod::symlink_at(target.as_bytes(), &dir_anchor, name.as_bytes())?;
            }
            EntryKind::HardLink(target) => {
                pending_links.push((absolute_path(&target), absolute));
            }
        }
        count = count.checked_add(1).ok_or(Error::InvalidNumber)?;
    }

    resolve_hard_links(pending_links)?;
    info!("imported {count} initramfs entries");
    Ok(count)
}

/// Splits `path` into a resolved parent anchor and final component name.
///
/// Walks only uncached prefix components; once a cached anchor covers the
/// parent, no VFS lookup is performed at all.
fn cached_parent<'a>(
    root: &PathAnchor,
    cache: &mut BTreeMap<Box<str>, PathAnchor>,
    path: &'a str,
) -> Result<(PathAnchor, &'a str)> {
    // A leading slash means the parent is the root; map "" to "/".
    let parent = if path.starts_with('/') {
        match path.rsplit_once('/') {
            Some(("", _)) => "/",
            Some((parent, _)) => parent,
            None => "/",
        }
    } else {
        match path.rsplit_once('/') {
            Some((parent, _)) => parent,
            None => "/",
        }
    };

    let name = path.rsplit_once('/').map(|(_, n)| n).unwrap_or(path);

    if let Some(anchor) = cache.get(parent) {
        return Ok((anchor.clone(), name));
    }

    // Walk components from the root, caching each newly seen directory.
    let mut current = String::new();
    for component in parent.split('/') {
        current.push('/');
        current.push_str(component);
        if !cache.contains_key(current.as_str()) {
            let anchor = vfs_mod::resolve_at(root, current.as_bytes(), true).or_else(|_| {
                vfs_mod::create_dir_at(root, current.as_bytes(), 0o755)?;
                vfs_mod::resolve_at(root, current.as_bytes(), true)
            })?;
            cache.insert(Box::from(current.as_str()), anchor);
        }
    }

    match cache.get(parent) {
        Some(anchor) => Ok((anchor.clone(), name)),
        None => Err(Error::InvalidPath),
    }
}

/// Reports whether Limine supplied an initramfs module.
pub(crate) fn init() {
    match archive() {
        Some(archive) => debug!("initramfs module is {} KiB", archive.len() / 1024),
        // Without an initramfs there is no /sbin/init to hand control to, so
        // this is a boot failure that has simply not happened yet.
        None => error!("the bootloader supplied no initramfs module"),
    }
}

fn header_field(field: &[u8]) -> Result<&str> {
    let end = field
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(field.len());
    str::from_utf8(&field[..end]).map_err(|_| Error::InvalidPath)
}

fn parse_octal(field: &[u8]) -> Result<u64> {
    if field.first().is_some_and(|byte| byte & 0x80 != 0) {
        return Err(Error::InvalidNumber);
    }

    let mut value = 0u64;
    let mut saw_digit = false;
    let mut finished = false;
    for byte in field {
        match *byte {
            b' ' | 0 if !saw_digit => {}
            b'0'..=b'7' if !finished => {
                saw_digit = true;
                value = value
                    .checked_mul(8)
                    .and_then(|value| value.checked_add(u64::from(*byte - b'0')))
                    .ok_or(Error::InvalidNumber)?;
            }
            b' ' | 0 => finished = true,
            _ => return Err(Error::InvalidNumber),
        }
    }
    Ok(value)
}

fn parse_entry_path(prefix: &str, name: &str) -> Result<String> {
    if name.is_empty() {
        return Err(Error::InvalidPath);
    }
    let mut path =
        String::with_capacity(prefix.len() + usize::from(!prefix.is_empty()) + name.len());
    if !prefix.is_empty() {
        path.push_str(prefix);
        path.push('/');
    }
    path.push_str(name);
    normalize_path(&path)
}

fn parse_link_path(target: &str) -> Result<String> {
    normalize_path(target)
}

fn parse_symlink_target(target: &str) -> Result<String> {
    if target.is_empty() || target.as_bytes().contains(&0) {
        return Err(Error::InvalidPath);
    }
    Ok(String::from(target))
}

fn normalize_path(path: &str) -> Result<String> {
    if path.is_empty() || path.starts_with('/') || path.as_bytes().contains(&0) {
        return Err(Error::InvalidPath);
    }

    let path = path.trim_end_matches('/');
    if path.is_empty() {
        return Err(Error::InvalidPath);
    }

    let mut normalized = String::with_capacity(path.len());
    for component in path.split('/') {
        if component.is_empty() || component == "." || component == ".." {
            return Err(Error::InvalidPath);
        }
        if !normalized.is_empty() {
            normalized.push('/');
        }
        normalized.push_str(component);
    }
    Ok(normalized)
}

fn absolute_path(path: &str) -> String {
    let mut absolute = String::with_capacity(path.len() + 1);
    absolute.push('/');
    absolute.push_str(path);
    absolute
}

fn write_all(vnode: &fs::Vnode, mut data: &[u8]) -> Result<()> {
    let mut offset = 0u64;
    while !data.is_empty() {
        let written = vnode.write_at(offset, &crate::mem::IoSource::kernel(data))?;
        if written == 0 {
            return Err(fs::Error::Io.into());
        }
        offset = offset
            .checked_add(written as u64)
            .ok_or(Error::InvalidNumber)?;
        data = &data[written..];
    }
    Ok(())
}

fn resolve_hard_links(mut pending: Vec<(String, String)>) -> Result<()> {
    while !pending.is_empty() {
        let mut unresolved = Vec::new();
        let mut progress = false;

        for (target, path) in pending {
            match fs::link(target.as_bytes(), path.as_bytes()) {
                Ok(()) => progress = true,
                Err(fs::Error::NotFound) => unresolved.push((target, path)),
                Err(error) => return Err(error.into()),
            }
        }

        if !progress && !unresolved.is_empty() {
            return Err(Error::UnresolvedHardLink);
        }
        pending = unresolved;
    }
    Ok(())
}
