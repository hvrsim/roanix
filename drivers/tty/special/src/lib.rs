#![no_std]
// Device-node callbacks are C ABI entry points and document each raw access.
#![allow(unsafe_code)]
// Values crossing the module boundary carry the C ABI widths fixed by
// include/roanix/api.h, and both supported targets use 64-bit pointers, so
// casts between them cannot lose information in practice. Large arrays appear
// only inside `const fn` constructors evaluated for statics, never on the
// stack at runtime.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::large_stack_arrays
)]

//! Null, zero, and random pseudo-devices.

use core::{ffi::c_void, ptr, slice};

use ddk::{Devfs, Module, Result, raw};

#[repr(usize)]
enum SpecialKind {
    Null = 0,
    Zero = 1,
    Random = 2,
}

impl SpecialKind {
    fn from_context(context: *mut c_void) -> Option<Self> {
        match context as usize {
            0 => Some(Self::Null),
            1 => Some(Self::Zero),
            2 => Some(Self::Random),
            _ => None,
        }
    }
}

const fn context(kind: SpecialKind) -> *mut c_void {
    kind as usize as *mut c_void
}

fn byte_count(length: usize) -> i64 {
    i64::try_from(length).unwrap_or(i64::MAX)
}

unsafe extern "C" fn special_read(
    context: *mut c_void,
    _file_context: usize,
    _offset: u64,
    data: *mut u8,
    length: usize,
    _flags: u32,
) -> i64 {
    let Some(kind) = SpecialKind::from_context(context) else {
        return ddk::EINVAL.into();
    };
    match kind {
        SpecialKind::Null => 0,
        SpecialKind::Zero => {
            if length != 0 {
                if data.is_null() {
                    return ddk::EINVAL.into();
                }
                // SAFETY: the node ABI guarantees `length` writable bytes
                // whenever the buffer is non-null, checked just above.
                unsafe { ptr::write_bytes(data, 0, length) };
            }
            byte_count(length)
        }
        SpecialKind::Random => {
            if length != 0 {
                if data.is_null() {
                    return ddk::EINVAL.into();
                }
                // SAFETY: the node ABI guarantees this buffer is writable for
                // `length` bytes while the callback runs.
                let output = unsafe { slice::from_raw_parts_mut(data, length) };
                if let Err(error) = ddk::random_fill(output) {
                    return i64::from(error.status());
                }
            }
            byte_count(length)
        }
    }
}

unsafe extern "C" fn special_write(
    context: *mut c_void,
    _file_context: usize,
    _offset: u64,
    data: *const u8,
    length: usize,
    _flags: u32,
) -> i64 {
    let Some(kind) = SpecialKind::from_context(context) else {
        return ddk::EINVAL.into();
    };
    if matches!(kind, SpecialKind::Random) && length != 0 {
        if data.is_null() {
            return ddk::EINVAL.into();
        }
        // SAFETY: the node ABI guarantees this buffer is readable for
        // `length` bytes while the callback runs.
        let input = unsafe { slice::from_raw_parts(data, length) };
        if let Err(error) = ddk::random_mix(input) {
            return i64::from(error.status());
        }
    }
    byte_count(length)
}

unsafe extern "C" fn special_poll(
    _context: *mut c_void,
    _file_context: usize,
    _offset: u64,
    events: u16,
    _flags: u32,
) -> i64 {
    i64::from(events & (ddk::POLL_IN | ddk::POLL_RDNORM | ddk::POLL_OUT | ddk::POLL_WRNORM))
}

static NULL_OPERATIONS: raw::NodeOps = raw::NodeOps {
    size: raw::NODE_OPS_SIZE,
    context: context(SpecialKind::Null),
    open: None,
    close: None,
    initial_offset: None,
    read: Some(special_read),
    write: Some(special_write),
    size_bytes: None,
    sync: None,
    poll: Some(special_poll),
    ioctl: None,
    readable_event: None,
    writable_event: None,
    hangup_event: None,
    terminal_state: None,
};

static ZERO_OPERATIONS: raw::NodeOps = raw::NodeOps {
    context: context(SpecialKind::Zero),
    ..NULL_OPERATIONS
};

static RANDOM_OPERATIONS: raw::NodeOps = raw::NodeOps {
    context: context(SpecialKind::Random),
    ..NULL_OPERATIONS
};

fn publish(
    devfs: &Devfs,
    parent: u64,
    name: &'static core::ffi::CStr,
    operations: &'static raw::NodeOps,
) -> Result<()> {
    // SAFETY: these immutable callback tables and C strings live for the
    // module's lifetime, as required by the device-node ABI.
    let _node = unsafe { devfs.create_character(parent, name, 0o666, operations) }?;
    Ok(())
}

fn special_init(_module: Module) -> Result<()> {
    let devfs = Devfs::current()?;
    let root = devfs.root()?;
    publish(&devfs, root, c"null", &NULL_OPERATIONS)?;
    publish(&devfs, root, c"zero", &ZERO_OPERATIONS)?;
    publish(&devfs, root, c"random", &RANDOM_OPERATIONS)?;
    publish(&devfs, root, c"urandom", &RANDOM_OPERATIONS)
}

fn special_exit(_module: Module) {}

ddk::module!(
    b"special\0",
    b"Null, zero, and random pseudo-devices\0",
    special_init,
    special_exit,
);
