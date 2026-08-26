//! Shared C ABI types and helpers.

use alloc::sync::Arc;
use core::{
    ffi::{c_char, c_void},
    mem::ManuallyDrop,
    slice, str,
};

use super::super::error::{Error, Result};

/// Incompatible ABI generation. A module built against a different major
/// version is rejected.
pub const ABI_MAJOR: u16 = 1;
/// Append-only ABI revision. A module may request at most the kernel's value.
pub const ABI_MINOR: u16 = 3;

/// Longest string accepted across the boundary.
pub const MAX_STRING: usize = 4096;

/// Describes a module image to the framework.
#[repr(C)]
pub struct ModuleDef {
    /// Size of this record.
    pub size: u32,
    /// Incompatible ABI generation the module was built against.
    pub abi_major: u16,
    /// Append-only ABI revision the module requires.
    pub abi_minor: u16,
    /// Reserved for future use and required to be zero.
    pub flags: u32,
    /// Module name, unique among loaded modules.
    pub name: *const c_char,
    /// Optional human-readable description.
    pub description: *const c_char,
    /// Required initialization callback.
    pub init: Option<unsafe extern "C" fn(module: *const c_void) -> i32>,
    /// Optional teardown callback.
    pub exit: Option<unsafe extern "C" fn(module: *const c_void)>,
}

/// A module image's entry point.
///
/// The kernel calls this once with the service table so the module can record
/// it before any framework call is made, then uses the returned descriptor to
/// load the module.
pub type ModuleEntryFn = unsafe extern "C" fn(api: *const super::api::Api) -> *const ModuleDef;

/// Borrows a NUL-terminated string supplied by a module.
///
/// # Safety
///
/// `pointer` must be null or address a NUL-terminated byte string that stays
/// valid for the duration of the call.
pub unsafe fn borrow_str<'a>(pointer: *const c_char) -> Result<&'a str> {
    if pointer.is_null() {
        return Err(Error::InvalidArgument);
    }
    let mut length = 0usize;
    while length < MAX_STRING {
        // SAFETY: the caller guarantees the string is NUL-terminated within
        // `MAX_STRING` bytes, so this read stays inside the allocation.
        if unsafe { *pointer.add(length) } == 0 {
            break;
        }
        length += 1;
    }
    if length == MAX_STRING {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: `length` bytes before the terminator are readable.
    let bytes = unsafe { slice::from_raw_parts(pointer.cast::<u8>(), length) };
    str::from_utf8(bytes).map_err(|_| Error::InvalidArgument)
}

/// Borrows an optional NUL-terminated string.
///
/// # Safety
///
/// The same requirements as [`borrow_str`] apply when `pointer` is non-null.
pub unsafe fn borrow_opt_str<'a>(pointer: *const c_char) -> Result<Option<&'a str>> {
    if pointer.is_null() {
        return Ok(None);
    }
    // SAFETY: forwarded from this function's caller.
    unsafe { borrow_str(pointer) }.map(Some)
}

/// Borrows a caller-supplied slice.
///
/// # Safety
///
/// `pointer` must address `count` readable elements for the duration of the
/// call, or `count` must be zero.
pub unsafe fn borrow_slice<'a, T>(pointer: *const T, count: usize) -> Result<&'a [T]> {
    if count == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: forwarded from this function's caller.
    Ok(unsafe { slice::from_raw_parts(pointer, count) })
}

/// Writes `value` through a caller-supplied output pointer.
///
/// # Safety
///
/// `pointer` must be null or address writable storage of the right type.
pub unsafe fn write_out<T>(pointer: *mut T, value: T) {
    if pointer.is_null() {
        return;
    }
    // SAFETY: forwarded from this function's caller.
    unsafe { pointer.write(value) };
}

/// Copies `source` into a caller buffer and reports the full length.
///
/// # Safety
///
/// `buffer` must address `capacity` writable bytes, and `written` must be null
/// or address writable storage.
pub unsafe fn copy_out(
    source: &[u8],
    buffer: *mut u8,
    capacity: usize,
    written: *mut usize,
) -> Result<()> {
    // SAFETY: forwarded from this function's caller.
    unsafe { write_out(written, source.len()) };
    if buffer.is_null() || capacity == 0 {
        return if source.is_empty() {
            Ok(())
        } else {
            Err(Error::BufferTooSmall)
        };
    }
    if source.len() > capacity {
        return Err(Error::BufferTooSmall);
    }
    // SAFETY: the caller guarantees `capacity` writable bytes and the length
    // was just checked against it.
    unsafe { core::ptr::copy_nonoverlapping(source.as_ptr(), buffer, source.len()) };
    Ok(())
}

/// Hands a counted reference to a module as an opaque receipt.
///
/// Receipts identify resources a module owns, such as a register window or an
/// interrupt registration. They are returned by the call that creates the
/// resource and consumed by the matching release call.
pub fn receipt<T>(value: Arc<T>) -> *mut c_void {
    Arc::into_raw(value).cast_mut().cast()
}

/// Borrows the resource behind a receipt without consuming it.
///
/// # Safety
///
/// `handle` must be a receipt produced by [`receipt`] for the same type and
/// must not have been released.
pub unsafe fn borrow_receipt<T>(handle: *mut c_void) -> Result<ManuallyDrop<Arc<T>>> {
    if handle.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: forwarded from this function's caller. Wrapping the reconstructed
    // reference in `ManuallyDrop` leaves the receipt's count untouched.
    Ok(ManuallyDrop::new(unsafe {
        Arc::from_raw(handle.cast::<T>())
    }))
}

/// Reclaims the reference behind a receipt.
///
/// # Safety
///
/// The same requirements as [`borrow_receipt`] apply, and the receipt must not
/// be used again.
pub unsafe fn claim_receipt<T>(handle: *mut c_void) -> Result<Arc<T>> {
    if handle.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: forwarded from this function's caller.
    Ok(unsafe { Arc::from_raw(handle.cast::<T>()) })
}
