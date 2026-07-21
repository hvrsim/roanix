//! Inheritable bus resources.

use alloc::sync::Arc;
use core::{
    ptr,
    sync::atomic::{AtomicU64, Ordering},
};

use super::{
    error::{Error, Result},
    tree::DriverId,
};

const RESOURCE_REVOKED: u64 = 1 << 63;
const RESOURCE_CALLS_MASK: u64 = !RESOURCE_REVOKED;

struct ResourceCallGuard<'a> {
    state: &'a AtomicU64,
}

impl Drop for ResourceCallGuard<'_> {
    fn drop(&mut self) {
        let previous = self.state.fetch_sub(1, Ordering::Release);
        assert!(
            previous & RESOURCE_CALLS_MASK != 0,
            "dev: resource callback count underflow"
        );
    }
}

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

/// Extensible 128-bit resource type key.
///
/// The high half should identify a vendor, subsystem, or standardized
/// namespace. The low half identifies a resource within that namespace.
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

/// Resource access and behavior flags.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ResourceFlags(u64);

impl ResourceFlags {
    /// No special behavior.
    pub const EMPTY: Self = Self(0);
    /// Resource data may be cached by consumers.
    pub const CACHEABLE: Self = Self(1 << 0);
    /// Resource data describes memory-mapped I/O.
    pub const MMIO: Self = Self(1 << 1);
    /// Resource method may block the current thread.
    pub const MAY_BLOCK: Self = Self(1 << 2);
    /// Resource is safe to use from interrupt context.
    pub const IRQ_SAFE: Self = Self(1 << 3);

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

/// Foreign-callable resource method.
pub type ResourceCallback = unsafe extern "C" fn(
    context: usize,
    input: *const u8,
    input_len: usize,
    output: *mut u8,
    output_len: usize,
    written: *mut usize,
) -> i32;

/// Callable operation published by a bus.
#[derive(Copy, Clone)]
pub struct ResourceMethod {
    owner: Option<DriverId>,
    context: usize,
    callback: ResourceCallback,
}

impl ResourceMethod {
    /// Creates a callable resource.
    ///
    /// # Safety
    ///
    /// `callback` must remain executable while this value exists, accept the
    /// supplied byte ranges, initialize `written` on success, and be safe for
    /// concurrent calls when the resource is shared between devices.
    pub const unsafe fn new(context: usize, callback: ResourceCallback) -> Self {
        Self {
            owner: None,
            context,
            callback,
        }
    }

    pub(crate) const unsafe fn new_driver(
        owner: DriverId,
        context: usize,
        callback: ResourceCallback,
    ) -> Self {
        Self {
            owner: Some(owner),
            context,
            callback,
        }
    }

    pub(crate) fn bind_owner(&mut self, owner: DriverId) {
        self.owner = Some(owner);
    }

    /// Invokes the resource without holding device-tree locks.
    pub fn invoke(&self, input: &[u8], output: &mut [u8]) -> Result<usize> {
        let _callback = match self.owner {
            Some(owner) => Some(super::driver::callback_guard(owner)?),
            None => None,
        };
        let mut written = 0usize;
        let input_ptr = if input.is_empty() {
            ptr::null()
        } else {
            input.as_ptr()
        };
        let output_ptr = if output.is_empty() {
            ptr::null_mut()
        } else {
            output.as_mut_ptr()
        };

        // SAFETY: the constructor contract guarantees the callback accepts
        // these ranges and remains live. The slices provide valid buffers for
        // the duration of the call.
        let status = unsafe {
            (self.callback)(
                self.context,
                input_ptr,
                input.len(),
                output_ptr,
                output.len(),
                &mut written,
            )
        };
        if status != 0 {
            return Err(Error::CallbackFailed(status));
        }
        if written > output.len() {
            return Err(Error::CallbackFailed(-1));
        }
        Ok(written)
    }
}

/// Resource payload published by a bus.
pub enum ResourceValue {
    /// Immutable, shareable bytes.
    Data(Arc<[u8]>),
    /// Driver-provided operation.
    Method(ResourceMethod),
}

/// Resource inherited by descendants of its owning bus.
pub struct Resource {
    id: ResourceId,
    key: ResourceKey,
    flags: ResourceFlags,
    call_state: AtomicU64,
    value: ResourceValue,
}

impl Resource {
    pub(crate) fn new(
        id: ResourceId,
        key: ResourceKey,
        flags: ResourceFlags,
        value: ResourceValue,
    ) -> Self {
        Self {
            id,
            key,
            flags,
            call_state: AtomicU64::new(0),
            value,
        }
    }

    /// Returns this resource's stable identifier.
    pub const fn id(&self) -> ResourceId {
        self.id
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
        if self.call_state.load(Ordering::Acquire) & RESOURCE_REVOKED != 0 {
            return Err(Error::Busy);
        }
        match &self.value {
            ResourceValue::Data(data) => Ok(data.clone()),
            ResourceValue::Method(_) => Err(Error::WrongKind),
        }
    }

    /// Invokes this resource, if it is a method resource.
    pub fn invoke(&self, input: &[u8], output: &mut [u8]) -> Result<usize> {
        let _call = self.acquire_call()?;
        let result = match &self.value {
            ResourceValue::Method(method) => method.invoke(input, output),
            ResourceValue::Data(_) => Err(Error::WrongKind),
        };
        result
    }

    pub(crate) fn try_revoke(&self) -> Result<()> {
        loop {
            let state = self.call_state.load(Ordering::Acquire);
            if state == RESOURCE_REVOKED {
                return Ok(());
            }
            if state & RESOURCE_CALLS_MASK != 0 {
                return Err(Error::Busy);
            }
            if self
                .call_state
                .compare_exchange(0, RESOURCE_REVOKED, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return Ok(());
            }
        }
    }

    pub(crate) fn restore(&self) {
        let _ = self.call_state.compare_exchange(
            RESOURCE_REVOKED,
            0,
            Ordering::Release,
            Ordering::Relaxed,
        );
    }

    fn acquire_call(&self) -> Result<ResourceCallGuard<'_>> {
        loop {
            let state = self.call_state.load(Ordering::Acquire);
            if state & RESOURCE_REVOKED != 0 || state & RESOURCE_CALLS_MASK == RESOURCE_CALLS_MASK {
                return Err(Error::Busy);
            }
            if self
                .call_state
                .compare_exchange_weak(state, state + 1, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return Ok(ResourceCallGuard {
                    state: &self.call_state,
                });
            }
        }
    }
}
