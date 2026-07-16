//! Byte-oriented VFS path validation and component parsing.

use alloc::{boxed::Box, collections::VecDeque};

use super::error::{Error, Result};

/// Maximum complete path length accepted by the VFS.
pub const MAX_PATH_LEN: usize = 4096;

/// Maximum single-component length accepted by the VFS.
pub const MAX_NAME_LEN: usize = 255;

/// Maximum number of symbolic links followed by one lookup.
pub const MAX_SYMLINK_DEPTH: usize = 8;

/// Parsed path components and whether the source path was absolute.
pub(crate) struct ParsedPath {
    /// Whether the path begins at the namespace root.
    pub absolute: bool,
    /// Normal, `.` and `..` components in traversal order.
    pub components: VecDeque<Box<[u8]>>,
}

/// Validates and splits a byte path without imposing UTF-8 semantics.
pub(crate) fn parse(path: &[u8]) -> Result<ParsedPath> {
    if path.is_empty() || path.len() > MAX_PATH_LEN || path.contains(&0) {
        return Err(if path.len() > MAX_PATH_LEN {
            Error::NameTooLong
        } else {
            Error::InvalidArgument
        });
    }

    let absolute = path.first() == Some(&b'/');
    let mut components = VecDeque::new();
    for component in path.split(|byte| *byte == b'/') {
        if component.is_empty() {
            continue;
        }
        validate_name(component)?;
        components.push_back(Box::<[u8]>::from(component));
    }

    Ok(ParsedPath {
        absolute,
        components,
    })
}

/// Splits a path into its parent path and final component.
pub(crate) fn split_parent(path: &[u8]) -> Result<(&[u8], &[u8])> {
    if path.is_empty() || path.len() > MAX_PATH_LEN || path.contains(&0) {
        return Err(if path.len() > MAX_PATH_LEN {
            Error::NameTooLong
        } else {
            Error::InvalidArgument
        });
    }

    let mut end = path.len();
    while end > 1 && path[end - 1] == b'/' {
        end -= 1;
    }
    let path = &path[..end];
    let split = path.iter().rposition(|byte| *byte == b'/');
    let (parent, name) = match split {
        Some(0) => (&path[..1], &path[1..]),
        Some(index) => (&path[..index], &path[index + 1..]),
        None => (&b"."[..], path),
    };

    validate_leaf_name(name)?;
    Ok((parent, name))
}

/// Validates a directory-entry name.
pub(crate) fn validate_leaf_name(name: &[u8]) -> Result<()> {
    validate_name(name)?;
    if name == b"." || name == b".." {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

/// Validates a symbolic-link target while preserving its exact bytes.
pub(crate) fn validate_symlink_target(target: &[u8]) -> Result<()> {
    if target.is_empty() || target.contains(&0) {
        return Err(Error::InvalidArgument);
    }
    if target.len() > MAX_PATH_LEN {
        return Err(Error::NameTooLong);
    }
    Ok(())
}

fn validate_name(name: &[u8]) -> Result<()> {
    if name.is_empty() || name.contains(&0) || name.contains(&b'/') {
        return Err(Error::InvalidArgument);
    }
    if name.len() > MAX_NAME_LEN {
        return Err(Error::NameTooLong);
    }
    Ok(())
}
