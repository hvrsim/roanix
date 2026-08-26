//! Descriptive hardware resources attached to a device.
//!
//! Resources describe what a device physically occupies: register windows, I/O
//! port ranges, bus-address windows, and interrupt lines. They deliberately
//! carry no capability or dependency semantics; software relationships between
//! drivers are expressed with interfaces instead. Keeping the two concerns
//! apart is what lets a driver consume a service that is not an ancestor in the
//! device tree.

use alloc::{boxed::Box, string::String, vec::Vec};

use super::super::error::{Error, Result};

/// Resource category.
pub mod kind {
    /// Memory-mapped register or buffer window, described by physical address.
    pub const MEM: u32 = 1;
    /// Architectural I/O port range.
    pub const IO: u32 = 2;
    /// Interrupt line already resolved to a virtual interrupt number.
    pub const IRQ: u32 = 3;
    /// Bus-address window usable for DMA.
    pub const DMA: u32 = 4;
    /// Numbering window delegated to a child bus, such as PCI bus numbers.
    pub const BUS: u32 = 5;
}

/// Resource attribute flags.
pub mod flags {
    /// The range is read-only.
    pub const READONLY: u32 = 1 << 0;
    /// The range may be mapped write-combining, such as a framebuffer.
    pub const PREFETCHABLE: u32 = 1 << 1;
    /// The range is cacheable normal memory rather than device memory.
    pub const CACHEABLE: u32 = 1 << 2;
    /// The window is 64-bit addressable.
    pub const WIDE: u32 = 1 << 3;
    /// The resource is shared with other devices and must not be claimed
    /// exclusively.
    pub const SHARED: u32 = 1 << 4;
}

/// Maximum number of resources on one device.
pub const MAX_RESOURCES: usize = 64;

/// One physical range or line occupied by a device.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Resource {
    /// Resource category, one of the [`kind`] constants.
    pub kind: u32,
    /// Attribute bits drawn from [`flags`].
    pub flags: u32,
    /// First address, port, or interrupt number in the range.
    pub start: u64,
    /// Number of bytes, ports, or lines covered.
    pub size: u64,
    /// Optional bus-visible alias of `start` used for DMA translation.
    pub bus_start: u64,
    /// Optional human-readable label such as a device-tree `reg-names` entry.
    pub name: Option<Box<str>>,
}

impl Resource {
    /// Creates a resource covering `size` units starting at `start`.
    pub fn new(kind: u32, flags: u32, start: u64, size: u64) -> Result<Self> {
        if !(kind::MEM..=kind::BUS).contains(&kind) {
            return Err(Error::InvalidArgument);
        }
        if size == 0 || start.checked_add(size - 1).is_none() {
            return Err(Error::InvalidArgument);
        }
        Ok(Self {
            kind,
            flags,
            start,
            size,
            bus_start: start,
            name: None,
        })
    }

    /// Re-checks the construction invariants.
    ///
    /// Fields are public for ABI-friendly construction, so the builder runs
    /// this before publishing a device; a hand-built resource with a zero
    /// size, an out-of-range kind, or an overflowing range is rejected there
    /// instead of corrupting later arithmetic such as [`Self::end`].
    pub fn validate(&self) -> Result<()> {
        if !(kind::MEM..=kind::BUS).contains(&self.kind) {
            return Err(Error::InvalidArgument);
        }
        if self.size == 0 || self.start.checked_add(self.size - 1).is_none() {
            return Err(Error::InvalidArgument);
        }
        Ok(())
    }

    /// Returns the last address, port, or line in the range.
    ///
    /// Callers must have passed the resource through [`Self::validate`].
    pub const fn end(&self) -> u64 {
        self.start + self.size - 1
    }

    /// Returns whether this resource fully contains `offset..offset + len`.
    pub fn covers(&self, offset: u64, len: u64) -> bool {
        len != 0 && offset.checked_add(len).is_some_and(|end| end <= self.size)
    }

    /// Attaches a label to this resource.
    pub fn with_name(mut self, name: &str) -> Self {
        if !name.is_empty() {
            self.name = Some(String::from(name).into_boxed_str());
        }
        self
    }

    /// Overrides the bus-visible base address used for DMA translation.
    pub const fn with_bus_start(mut self, bus_start: u64) -> Self {
        self.bus_start = bus_start;
        self
    }
}

/// Mutable resource list used while a device is being built.
#[derive(Default)]
pub struct ResourceBuilder {
    entries: Vec<Resource>,
}

impl ResourceBuilder {
    /// Creates an empty builder.
    pub const fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Appends a resource.
    pub fn push(&mut self, resource: Resource) -> Result<usize> {
        if self.entries.len() >= MAX_RESOURCES {
            return Err(Error::NoSpace);
        }
        let index = self.entries.len();
        self.entries.push(resource);
        Ok(index)
    }

    /// Freezes the list into its published form.
    pub fn build(self) -> Box<[Resource]> {
        self.entries.into_boxed_slice()
    }
}

/// Returns the `index`-th resource of `kind` within `resources`.
pub fn find(resources: &[Resource], kind: u32, index: usize) -> Option<&Resource> {
    resources
        .iter()
        .filter(|resource| resource.kind == kind)
        .nth(index)
}

/// Returns the resource of `kind` labelled `name`.
pub fn find_named<'a>(resources: &'a [Resource], kind: u32, name: &str) -> Option<&'a Resource> {
    resources.iter().find(|resource| {
        resource.kind == kind && resource.name.as_ref().is_some_and(|label| &**label == name)
    })
}
