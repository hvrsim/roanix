//! Firmware description nodes.
//!
//! A firmware node is an opaque back-reference from a device to the entry that
//! described it: a device-tree offset, an ACPI namespace handle, or a synthetic
//! token invented by a bus enumerator.
//!
//! The kernel deliberately understands no firmware format. Enumerators such as
//! the device-tree or ACPI drivers translate firmware data into device
//! properties and resources when they create a device, and publish an interface
//! for the rarer cases that need to walk the original description, such as
//! resolving a phandle to another node. Keeping format knowledge in drivers is
//! what lets a device-tree platform and an ACPI platform share the same core.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::Arc,
};

use super::super::{
    error::{Error, Result},
    obj::{ObjHeader, ObjKind, framework_object},
};

/// Firmware description formats.
pub mod kind {
    /// A flattened or unflattened device-tree node.
    pub const DEVICE_TREE: u32 = 1;
    /// An ACPI namespace object.
    pub const ACPI: u32 = 2;
    /// A token invented by a bus enumerator, such as a PCI address.
    pub const SYNTHETIC: u32 = 3;
}

/// An opaque reference to the firmware entry describing a device.
#[repr(C)]
pub struct Fwnode {
    header: ObjHeader,
    kind: u32,
    token: u64,
    /// Interface name a driver binds to in order to query this node further.
    provider: Option<Box<str>>,
    path: Option<Box<str>>,
}

framework_object!(Fwnode, Fwnode);

impl Fwnode {
    /// Returns the firmware format, one of the [`kind`] constants.
    pub const fn kind(&self) -> u32 {
        self.kind
    }

    /// Returns the provider-defined token identifying this node.
    pub const fn token(&self) -> u64 {
        self.token
    }

    /// Returns the interface name that can decode this node.
    pub fn provider(&self) -> Option<&str> {
        self.provider.as_deref()
    }

    /// Returns the firmware path, when the enumerator recorded one.
    pub fn path(&self) -> Option<&str> {
        self.path.as_deref()
    }
}

/// Creates a firmware node reference.
pub fn create(
    kind: u32,
    token: u64,
    provider: Option<&str>,
    path: Option<&str>,
) -> Result<Arc<Fwnode>> {
    if !(kind::DEVICE_TREE..=kind::SYNTHETIC).contains(&kind) {
        return Err(Error::InvalidArgument);
    }
    let header = ObjHeader::new_with(ObjKind::Fwnode, path.or(provider), None, None);
    let node = Arc::new(Fwnode {
        header,
        kind,
        token,
        provider: provider.map(|name| String::from(name).into_boxed_str()),
        path: path.map(|value| value.to_string().into_boxed_str()),
    });
    node.header.register()?;
    Ok(node)
}
