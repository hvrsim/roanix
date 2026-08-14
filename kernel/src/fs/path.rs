//! Byte-oriented VFS path validation and component parsing.

use super::error::{Error, Result};

/// Maximum complete path length accepted by the VFS.
pub const MAX_PATH_LEN: usize = 4096;

/// Maximum single-component length accepted by the VFS.
pub const MAX_NAME_LEN: usize = 255;

/// Maximum number of symbolic links followed by one lookup.
pub const MAX_SYMLINK_DEPTH: usize = 8;

/// Borrowing iterator over the non-empty components of a path.
///
/// Traversal allocates nothing, which keeps the common lookup path free of
/// heap traffic. Only symbolic-link expansion needs owned components.
#[derive(Clone)]
pub(crate) struct Components<'a> {
    path: &'a [u8],
    position: usize,
}

impl<'a> Components<'a> {
    fn new(path: &'a [u8]) -> Self {
        Self { path, position: 0 }
    }

    /// Returns the next component without consuming it.
    pub(crate) fn peek(&self) -> Option<&'a [u8]> {
        self.clone().next()
    }
}

impl<'a> Iterator for Components<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<&'a [u8]> {
        while self.position < self.path.len() && self.path[self.position] == b'/' {
            self.position += 1;
        }
        if self.position == self.path.len() {
            return None;
        }
        let start = self.position;
        while self.position < self.path.len() && self.path[self.position] != b'/' {
            self.position += 1;
        }
        Some(&self.path[start..self.position])
    }
}

/// Parsed path components and the syntactic properties of the source path.
pub(crate) struct ParsedPath<'a> {
    /// Whether the path begins at the namespace root.
    pub absolute: bool,
    /// Whether the path ends with a separator and therefore names a directory.
    pub trailing_slash: bool,
    /// Normal, `.` and `..` components in traversal order.
    pub components: Components<'a>,
}

/// Validates and splits a byte path without imposing UTF-8 semantics.
pub(crate) fn parse(path: &[u8]) -> Result<ParsedPath<'_>> {
    validate_path(path)?;
    for component in Components::new(path) {
        validate_name(component)?;
    }

    Ok(ParsedPath {
        absolute: path.first() == Some(&b'/'),
        trailing_slash: path.last() == Some(&b'/'),
        components: Components::new(path),
    })
}

/// Parent path, final component, and whether the path named a directory.
pub(crate) struct ParentPath<'a> {
    /// Path of the directory holding the final component.
    pub parent: &'a [u8],
    /// Final path component.
    pub name: &'a [u8],
    /// Whether the source path ended with a separator.
    pub trailing_slash: bool,
}

/// Splits a path into its parent path and final component.
pub(crate) fn split_parent(path: &[u8]) -> Result<ParentPath<'_>> {
    validate_path(path)?;

    let trailing_slash = path.last() == Some(&b'/');
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
    Ok(ParentPath {
        parent,
        name,
        trailing_slash,
    })
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

fn validate_path(path: &[u8]) -> Result<()> {
    if path.len() > MAX_PATH_LEN {
        return Err(Error::NameTooLong);
    }
    if path.is_empty() || path.contains(&0) {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

fn validate_name(name: &[u8]) -> Result<()> {
    if name.len() > MAX_NAME_LEN {
        return Err(Error::NameTooLong);
    }
    if name.is_empty() || name.contains(&0) || name.contains(&b'/') {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}
