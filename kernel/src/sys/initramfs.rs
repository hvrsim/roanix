//!
//! # Initial RAM Filesystem
//!
//! Access to and extraction of the userspace archive loaded as a Limine module.
//!

use alloc::{collections::BTreeSet, string::String, vec::Vec};
use core::{fmt, slice, str};

use limine::request::ModuleRequest;
use log::info;

use crate::fs::{self, OpenFlags, SetAttr};

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

/// Imports the Limine initramfs into the mounted root filesystem.
pub(crate) fn populate() -> Result<usize> {
    let Some(bytes) = archive() else {
        return Ok(0);
    };

    let mut archive = TarArchive::new(bytes);
    let mut imported = BTreeSet::new();
    let mut pending_links = Vec::new();
    let mut count = 0usize;

    while let Some(entry) = archive.next()? {
        if !imported.insert(entry.path.clone()) {
            return Err(Error::DuplicatePath);
        }
        ensure_parent_directories(&entry.path)?;
        let path = absolute_path(&entry.path);

        match entry.kind {
            EntryKind::Regular(data) => {
                let vnode = fs::create_file(&path, entry.mode)?;
                write_all(&vnode, data)?;
                vnode.setattr(SetAttr {
                    size: None,
                    mode: Some(entry.mode),
                })?;
            }
            EntryKind::Directory => {
                let vnode = match fs::create_dir(&path, entry.mode) {
                    Ok(vnode) => vnode,
                    Err(fs::Error::AlreadyExists) => {
                        let file = fs::open(
                            &path,
                            OpenFlags::READ | OpenFlags::DIRECTORY | OpenFlags::NOFOLLOW,
                            0,
                        )?;
                        file.vnode().clone()
                    }
                    Err(error) => return Err(error.into()),
                };
                vnode.setattr(SetAttr {
                    size: None,
                    mode: Some(entry.mode),
                })?;
            }
            EntryKind::Symlink(target) => {
                fs::symlink(&target, &path)?;
            }
            EntryKind::HardLink(target) => {
                pending_links.push((absolute_path(&target), path));
            }
        }
        count = count.checked_add(1).ok_or(Error::InvalidNumber)?;
    }

    resolve_hard_links(pending_links)?;
    info!("boot: imported {count} initramfs entries");
    Ok(count)
}

/// Reports whether Limine supplied an initramfs module.
pub(crate) fn init() {
    match archive() {
        Some(archive) => info!("boot: initramfs module loaded ({} bytes)", archive.len()),
        None => info!("boot: no initramfs module"),
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

fn ensure_parent_directories(path: &str) -> Result<()> {
    let Some((parent, _)) = path.rsplit_once('/') else {
        return Ok(());
    };

    let mut current = String::new();
    for component in parent.split('/') {
        current.push('/');
        current.push_str(component);
        match fs::open(
            &current,
            OpenFlags::READ | OpenFlags::DIRECTORY | OpenFlags::NOFOLLOW,
            0,
        ) {
            Ok(_) => {}
            Err(fs::Error::NotFound) => {
                fs::create_dir(&current, 0o755)?;
            }
            Err(error) => return Err(error.into()),
        }
    }
    Ok(())
}

fn write_all(vnode: &fs::Vnode, mut data: &[u8]) -> Result<()> {
    let mut offset = 0u64;
    while !data.is_empty() {
        let written = vnode.write_at(offset, data)?;
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
            match fs::link(&target, &path) {
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
