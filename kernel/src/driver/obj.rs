//! Reference-counted framework objects addressed by stable pointers.
//!
//! The previous framework translated every driver call through an identifier
//! map guarded by one global lock, so each call paid a lock acquisition and a
//! logarithmic lookup. This module replaces that with direct pointers into
//! reference-counted allocations: handle validation is a constant-time tag
//! check and no lock is taken.
//!
//! Every object begins with an [`ObjHeader`] carrying a type-mixed magic word.
//! A handle is the address of that header, so a driver-supplied pointer is
//! rejected unless it addresses a live object of the expected type.

use alloc::sync::Arc;
use core::{
    ffi::c_void,
    sync::atomic::{AtomicU32, Ordering},
};

use super::error::{Error, Result};

const MAGIC_BASE: u32 = 0x5244_4600;
const MAGIC_DEAD: u32 = 0xDEAD_0000;

/// Framework object type tag.
#[repr(u32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ObjKind {
    /// A loadable or resident driver module.
    Module = 1,
    /// A node in the device tree.
    Device = 2,
    /// A driver registered against a bus.
    Driver = 3,
    /// A bus type.
    Bus = 4,
    /// A device class.
    Class = 5,
    /// A published interface.
    Interface = 6,
    /// A class membership record.
    ClassDevice = 7,
    /// An interrupt controller domain.
    IrqDomain = 8,
    /// A work queue.
    WorkQueue = 9,
    /// A deferrable timer.
    Timer = 10,
    /// A firmware description node.
    Fwnode = 11,
}

impl ObjKind {
    const fn magic(self) -> u32 {
        MAGIC_BASE | self as u32
    }
}

/// Common prefix of every framework object.
#[repr(C)]
pub struct ObjHeader {
    magic: AtomicU32,
    kind: u32,
}

impl ObjHeader {
    /// Creates a live header for `kind`.
    pub fn new(kind: ObjKind) -> Self {
        Self {
            magic: AtomicU32::new(kind.magic()),
            kind: kind as u32,
        }
    }

    /// Returns this object's type tag.
    pub const fn kind(&self) -> u32 {
        self.kind
    }

    /// Marks the object invalid so later handle lookups fail.
    ///
    /// The allocation stays alive until the last reference drops; poisoning
    /// only revokes handle-based access.
    pub fn poison(&self) {
        self.magic.store(MAGIC_DEAD | self.kind, Ordering::Release);
    }

    /// Returns whether this header is still a valid handle target.
    pub fn is_live(&self) -> bool {
        self.magic.load(Ordering::Acquire) == (MAGIC_BASE | self.kind)
    }
}

/// A framework type reachable through a driver-visible handle.
///
/// # Safety
///
/// Implementors must store [`Object::HEADER`] as the first field of a
/// `#[repr(C)]` type so that the object address and the header address are
/// identical.
pub unsafe trait Object: Sized {
    /// Type tag validated on every handle conversion.
    const KIND: ObjKind;

    /// Returns this object's embedded header.
    fn header(&self) -> &ObjHeader;
}

/// Borrows an object from a driver-supplied handle.
///
/// The returned reference is only valid while the caller keeps the object
/// alive, which callbacks guarantee by holding a reference across the call.
///
/// # Safety
///
/// `handle` must be null or address a framework object allocation.
pub unsafe fn borrow<'a, T: Object>(handle: *const c_void) -> Result<&'a T> {
    if handle.is_null() || (handle as usize) & (align_of::<ObjHeader>() - 1) != 0 {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the caller guarantees `handle` addresses a framework object, so
    // its first field is a header whose tag distinguishes the concrete type.
    let header = unsafe { &*handle.cast::<ObjHeader>() };
    if header.kind != T::KIND as u32 {
        return Err(Error::WrongKind);
    }
    if !header.is_live() {
        return Err(Error::NoDevice);
    }
    // SAFETY: the tag matched `T::KIND`, and `Object` requires the header to be
    // the first field of a `#[repr(C)]` layout, so the addresses coincide.
    Ok(unsafe { &*handle.cast::<T>() })
}

/// Borrows an object and clones a counted reference to it.
///
/// # Safety
///
/// `handle` must satisfy the [`borrow`] contract and must have originated from
/// an [`Arc`] allocation of `T`.
pub unsafe fn upgrade<T: Object>(handle: *const c_void) -> Result<Arc<T>> {
    // SAFETY: forwarded from this function's caller.
    let object = unsafe { borrow::<T>(handle) }?;
    // SAFETY: the caller guarantees the object lives inside an `Arc<T>`, and
    // the borrow above proves the allocation is still live.
    let arc = unsafe { Arc::from_raw(object as *const T) };
    let clone = arc.clone();
    let _ = Arc::into_raw(arc);
    Ok(clone)
}

/// Returns the driver-visible handle for an object.
pub fn handle<T: Object>(object: &Arc<T>) -> *const c_void {
    Arc::as_ptr(object).cast()
}

/// Converts an owned reference into a handle that keeps the object alive.
///
/// The caller becomes responsible for balancing this with [`release`].
pub fn into_handle<T: Object>(object: Arc<T>) -> *const c_void {
    Arc::into_raw(object).cast()
}

/// Drops a reference previously produced by [`into_handle`].
///
/// # Safety
///
/// `handle` must have come from [`into_handle`] for the same type and must not
/// be released twice.
pub unsafe fn release<T: Object>(handle: *const c_void) {
    if handle.is_null() {
        return;
    }
    // SAFETY: forwarded from this function's caller.
    drop(unsafe { Arc::from_raw(handle.cast::<T>()) });
}

/// Declares a framework object type with an embedded header.
macro_rules! framework_object {
    ($type:ty, $kind:ident) => {
        // SAFETY: the type stores `header` as the first field of a `#[repr(C)]`
        // layout, so the object and header addresses coincide.
        unsafe impl $crate::driver::obj::Object for $type {
            const KIND: $crate::driver::obj::ObjKind = $crate::driver::obj::ObjKind::$kind;

            fn header(&self) -> &$crate::driver::obj::ObjHeader {
                &self.header
            }
        }
    };
}

pub(crate) use framework_object;
