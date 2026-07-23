//! Typed, inheritable device resources.

use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, Ordering};

use super::{
    error::{Error, Result},
    tree::DriverId,
};

/// Stable identifier for one published resource.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ResourceId(u64);

impl ResourceId {
    pub(crate) const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Stable identifier for one acquired resource lease.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ResourceLeaseId(u64);

impl ResourceLeaseId {
    pub(crate) const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Extensible 128-bit resource type key.
///
/// The high half identifies a vendor, subsystem, or standardized namespace.
/// The low half identifies a resource within that namespace. Incompatible
/// protocol revisions use a new key; compatible additions extend the
/// size-prefixed operation table.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ResourceKey {
    /// Namespace identifier.
    pub namespace: u64,
    /// Resource identifier within the namespace.
    pub resource: u64,
}

impl ResourceKey {
    /// Creates a resource key.
    pub const fn new(namespace: u64, resource: u64) -> Self {
        Self {
            namespace,
            resource,
        }
    }
}

/// Namespace used for device-manager and future sysfs metadata.
pub const PROPERTY_NAMESPACE: u64 = 0x524F_414E_4958_5052;
/// Device subsystem name such as `pci`, `usb`, `block`, or `tty`.
pub const PROPERTY_SUBSYSTEM: ResourceKey = ResourceKey::new(PROPERTY_NAMESPACE, 1);
/// Subsystem-specific device type.
pub const PROPERTY_DEVICE_TYPE: ResourceKey = ResourceKey::new(PROPERTY_NAMESPACE, 2);
/// Driver-autoload and userspace matching alias.
pub const PROPERTY_MODALIAS: ResourceKey = ResourceKey::new(PROPERTY_NAMESPACE, 3);
/// Firmware or protocol compatibility string list.
pub const PROPERTY_COMPATIBLE: ResourceKey = ResourceKey::new(PROPERTY_NAMESPACE, 4);
/// Numeric or textual device class.
pub const PROPERTY_CLASS: ResourceKey = ResourceKey::new(PROPERTY_NAMESPACE, 5);

/// Resource access and behavior flags.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ResourceFlags(u64);

impl ResourceFlags {
    /// No special behavior.
    pub const EMPTY: Self = Self(0);
    /// Resource data or operations may be cached by consumers.
    pub const CACHEABLE: Self = Self(1 << 0);
    /// Resource describes memory-mapped I/O.
    pub const MMIO: Self = Self(1 << 1);
    /// Protocol operations may block the current thread.
    pub const MAY_BLOCK: Self = Self(1 << 2);
    /// Protocol operations are safe to call from interrupt context.
    pub const IRQ_SAFE: Self = Self(1 << 3);
    /// Provider cannot disappear while a consumer holds a lease.
    pub const ORDERLY: Self = Self(1 << 4);
    /// Protocol operations may be called concurrently.
    pub const CONCURRENT: Self = Self(1 << 5);
    /// Protocol operations may re-enter the provider.
    pub const REENTRANT: Self = Self(1 << 6);

    /// Creates flags from their raw representation.
    pub const fn from_bits(bits: u64) -> Self {
        Self(bits)
    }

    /// Returns the raw representation.
    pub const fn bits(self) -> u64 {
        self.0
    }

    /// Returns whether all supplied flags are present.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
}

/// Direct-call protocol published by a resource provider.
#[derive(Copy, Clone)]
pub struct ResourceProtocol {
    revision: u32,
    context: usize,
    operations: *const u8,
    operations_size: usize,
}

// SAFETY: publication requires the operation table and every callback it
// contains to remain immutable and executable until the resource is removed.
// Resource leases prevent provider removal while a consumer can use the table.
unsafe impl Send for ResourceProtocol {}
// SAFETY: protocol concurrency is described by resource flags and enforced by
// the provider. The operation-table pointer itself is immutable.
unsafe impl Sync for ResourceProtocol {}

impl ResourceProtocol {
    /// Creates a direct-call protocol descriptor.
    ///
    /// # Safety
    ///
    /// `operations` must address an immutable table of `operations_size` bytes
    /// that remains live until the resource is removed. Function pointers in
    /// the table must follow the protocol ABI represented by the resource key.
    pub const unsafe fn new(
        revision: u32,
        context: usize,
        operations: *const u8,
        operations_size: usize,
    ) -> Self {
        Self {
            revision,
            context,
            operations,
            operations_size,
        }
    }

    /// Returns the compatible protocol revision.
    pub const fn revision(self) -> u32 {
        self.revision
    }

    /// Returns the provider-defined call context.
    pub const fn context(self) -> usize {
        self.context
    }

    /// Returns the immutable operation table.
    pub const fn operations(self) -> *const u8 {
        self.operations
    }

    /// Returns the operation-table size.
    pub const fn operations_size(self) -> usize {
        self.operations_size
    }
}

/// Physical address range delegated by a bus or firmware provider.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct MemoryRegion {
    physical: u64,
    size: u64,
}

impl MemoryRegion {
    /// Creates a non-empty physical range.
    pub fn new(physical: u64, size: u64) -> Result<Self> {
        if size == 0 || physical.checked_add(size).is_none() {
            return Err(Error::InvalidArgument);
        }
        Ok(Self { physical, size })
    }

    /// Returns the first physical byte.
    pub const fn physical(self) -> u64 {
        self.physical
    }

    /// Returns the number of bytes in the range.
    pub const fn size(self) -> u64 {
        self.size
    }
}

/// Resource payload published by a provider.
pub enum ResourceValue {
    /// Immutable bytes borrowed through a lease.
    Data(Arc<[u8]>),
    /// Delegated physical range mapped through the memory service.
    Memory(MemoryRegion),
    /// Versioned direct-call protocol borrowed through a lease.
    Protocol(ResourceProtocol),
}

/// Resource inherited by descendants of its publishing bus.
pub struct Resource {
    id: ResourceId,
    owner: DriverId,
    key: ResourceKey,
    flags: ResourceFlags,
    revoked: AtomicBool,
    value: ResourceValue,
}

impl Resource {
    pub(crate) fn new(
        id: ResourceId,
        owner: DriverId,
        key: ResourceKey,
        flags: ResourceFlags,
        value: ResourceValue,
    ) -> Self {
        Self {
            id,
            owner,
            key,
            flags,
            revoked: AtomicBool::new(false),
            value,
        }
    }

    /// Returns this resource's stable identifier.
    pub const fn id(&self) -> ResourceId {
        self.id
    }

    /// Returns the publishing driver.
    pub const fn owner(&self) -> DriverId {
        self.owner
    }

    /// Returns this resource's type key.
    pub const fn key(&self) -> ResourceKey {
        self.key
    }

    /// Returns this resource's behavior flags.
    pub const fn flags(&self) -> ResourceFlags {
        self.flags
    }

    /// Returns shared data, if this is a data resource.
    pub fn data(&self) -> Result<Arc<[u8]>> {
        self.ensure_live()?;
        match &self.value {
            ResourceValue::Data(data) => Ok(data.clone()),
            ResourceValue::Memory(_) | ResourceValue::Protocol(_) => Err(Error::WrongKind),
        }
    }

    /// Returns a delegated physical memory range.
    pub fn memory(&self) -> Result<MemoryRegion> {
        self.ensure_live()?;
        match &self.value {
            ResourceValue::Memory(region) => Ok(*region),
            ResourceValue::Data(_) | ResourceValue::Protocol(_) => Err(Error::WrongKind),
        }
    }

    /// Returns a compatible direct-call protocol.
    pub fn protocol(&self, minimum_revision: u32) -> Result<ResourceProtocol> {
        self.ensure_live()?;
        match &self.value {
            ResourceValue::Protocol(protocol) if protocol.revision() >= minimum_revision => {
                Ok(*protocol)
            }
            ResourceValue::Protocol(_) => Err(Error::Unsupported),
            ResourceValue::Data(_) | ResourceValue::Memory(_) => Err(Error::WrongKind),
        }
    }

    pub(crate) fn try_revoke(&self) -> Result<()> {
        match self
            .revoked
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) | Err(true) => Ok(()),
            Err(false) => Err(Error::Busy),
        }
    }

    pub(crate) fn restore(&self) {
        self.revoked.store(false, Ordering::Release);
    }

    pub(crate) fn ensure_live(&self) -> Result<()> {
        if self.revoked.load(Ordering::Acquire) {
            Err(Error::Busy)
        } else {
            Ok(())
        }
    }
}
