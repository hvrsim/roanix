//! Typed device properties.
//!
//! Properties model firmware and bus metadata such as device-tree cells, ACPI
//! identifiers, and PCI configuration attributes. They are populated while a
//! device is being built and frozen when it is published, so lookups on a live
//! device need no locking.

use alloc::{boxed::Box, string::String, vec::Vec};

use super::super::error::{Error, Result};

/// Value stored under a property name.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PropValue {
    /// A present property with no payload, such as a device-tree flag.
    Empty,
    /// A single unsigned 32-bit cell.
    U32(u32),
    /// A single unsigned 64-bit cell.
    U64(u64),
    /// A UTF-8 string.
    Str(Box<str>),
    /// An ordered list of UTF-8 strings, such as `compatible`.
    StrList(Box<[Box<str>]>),
    /// An ordered list of 32-bit cells.
    U32List(Box<[u32]>),
    /// An ordered list of 64-bit cells.
    U64List(Box<[u64]>),
    /// Opaque bytes preserved verbatim.
    Bytes(Box<[u8]>),
}

/// Stable ABI discriminant for a [`PropValue`].
pub mod kind {
    /// [`super::PropValue::Empty`].
    pub const EMPTY: u32 = 0;
    /// [`super::PropValue::U32`].
    pub const U32: u32 = 1;
    /// [`super::PropValue::U64`].
    pub const U64: u32 = 2;
    /// [`super::PropValue::Str`].
    pub const STR: u32 = 3;
    /// [`super::PropValue::StrList`].
    pub const STR_LIST: u32 = 4;
    /// [`super::PropValue::U32List`].
    pub const U32_LIST: u32 = 5;
    /// [`super::PropValue::U64List`].
    pub const U64_LIST: u32 = 6;
    /// [`super::PropValue::Bytes`].
    pub const BYTES: u32 = 7;
}

impl PropValue {
    /// Returns this value's ABI discriminant.
    pub const fn kind(&self) -> u32 {
        match self {
            Self::Empty => kind::EMPTY,
            Self::U32(_) => kind::U32,
            Self::U64(_) => kind::U64,
            Self::Str(_) => kind::STR,
            Self::StrList(_) => kind::STR_LIST,
            Self::U32List(_) => kind::U32_LIST,
            Self::U64List(_) => kind::U64_LIST,
            Self::Bytes(_) => kind::BYTES,
        }
    }

    /// Returns this value as an unsigned integer when it holds a single cell.
    pub fn as_u64(&self) -> Option<u64> {
        match self {
            Self::U32(value) => Some(u64::from(*value)),
            Self::U64(value) => Some(*value),
            Self::U32List(values) => values.first().map(|value| u64::from(*value)),
            Self::U64List(values) => values.first().copied(),
            _ => None,
        }
    }

    /// Returns the string at `index` for string-valued properties.
    pub fn string_at(&self, index: usize) -> Option<&str> {
        match self {
            Self::Str(value) if index == 0 => Some(value),
            Self::StrList(values) => values.get(index).map(|value| &**value),
            _ => None,
        }
    }

    /// Returns the number of elements in a list-valued property.
    pub fn len(&self) -> usize {
        match self {
            Self::Empty => 0,
            Self::U32(_) | Self::U64(_) | Self::Str(_) => 1,
            Self::StrList(values) => values.len(),
            Self::U32List(values) => values.len(),
            Self::U64List(values) => values.len(),
            Self::Bytes(values) => values.len(),
        }
    }

    /// Returns whether any string in this property equals `text`.
    pub fn contains_string(&self, text: &str) -> bool {
        match self {
            Self::Str(value) => &**value == text,
            Self::StrList(values) => values.iter().any(|value| &**value == text),
            _ => false,
        }
    }

    /// Returns the cell at `index` for integer-list properties.
    pub fn cell_at(&self, index: usize) -> Option<u64> {
        match self {
            Self::U32(value) if index == 0 => Some(u64::from(*value)),
            Self::U64(value) if index == 0 => Some(*value),
            Self::U32List(values) => values.get(index).map(|value| u64::from(*value)),
            Self::U64List(values) => values.get(index).copied(),
            _ => None,
        }
    }

    /// Returns raw bytes for opaque properties.
    pub fn bytes(&self) -> Option<&[u8]> {
        match self {
            Self::Bytes(value) => Some(value),
            _ => None,
        }
    }
}

/// Mutable property collection used while a device is being built.
#[derive(Default)]
pub struct PropertyBuilder {
    entries: Vec<(Box<str>, PropValue)>,
}

impl PropertyBuilder {
    /// Creates an empty builder.
    pub const fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Inserts or replaces `name`.
    pub fn set(&mut self, name: &str, value: PropValue) -> Result<()> {
        if name.is_empty() || name.len() > MAX_NAME {
            return Err(Error::InvalidArgument);
        }
        match self.entries.iter_mut().find(|(key, _)| &**key == name) {
            Some(entry) => entry.1 = value,
            None => {
                if self.entries.len() >= MAX_PROPERTIES {
                    return Err(Error::NoSpace);
                }
                self.entries.push((String::from(name).into_boxed_str(), value));
            }
        }
        Ok(())
    }

    /// Removes `name` if present.
    pub fn remove(&mut self, name: &str) {
        self.entries.retain(|(key, _)| &**key != name);
    }

    /// Freezes the collection into its published form.
    pub fn build(mut self) -> Properties {
        self.entries.sort_unstable_by(|left, right| left.0.cmp(&right.0));
        Properties {
            entries: self.entries.into_boxed_slice(),
        }
    }
}

/// Maximum number of properties on one device.
pub const MAX_PROPERTIES: usize = 256;
/// Maximum property-name length in bytes.
pub const MAX_NAME: usize = 128;

/// Immutable property collection attached to a published device.
#[derive(Default)]
pub struct Properties {
    entries: Box<[(Box<str>, PropValue)]>,
}

impl Properties {
    /// Creates an empty collection.
    pub fn empty() -> Self {
        Self::default()
    }

    /// Looks up a property by name.
    pub fn get(&self, name: &str) -> Option<&PropValue> {
        self.entries
            .binary_search_by(|(key, _)| (**key).cmp(name))
            .ok()
            .map(|index| &self.entries[index].1)
    }

    /// Returns whether `name` is present.
    pub fn contains(&self, name: &str) -> bool {
        self.get(name).is_some()
    }

    /// Returns the number of stored properties.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns whether the collection is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Returns the name and value at `index` in sorted order.
    pub fn entry(&self, index: usize) -> Option<(&str, &PropValue)> {
        self.entries
            .get(index)
            .map(|(name, value)| (&**name, value))
    }

    /// Returns whether the property `name` lists `text`.
    pub fn has_string(&self, name: &str, text: &str) -> bool {
        self.get(name)
            .is_some_and(|value| value.contains_string(text))
    }
}
