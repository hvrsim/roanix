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

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicU32, AtomicU64, Ordering},
};

use crate::sys::sync::{Mutex, Once};

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
    /// A registered filesystem provider.
    Filesystem = 12,
    /// One mounted filesystem instance.
    Mount = 13,
    /// A live filesystem vnode adapter.
    Vnode = 14,
    /// A device endpoint published through devfs.
    DeviceNode = 15,
}

impl ObjKind {
    const fn magic(self) -> u32 {
        MAGIC_BASE | self as u32
    }
}

/// Common lifecycle state recorded for every framework object.
#[repr(u32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ObjState {
    /// The allocation exists but has not been published.
    New = 0,
    /// The object is published and may be used.
    Live = 1,
    /// The object is being withdrawn.
    Removing = 2,
    /// The object has been withdrawn permanently.
    Dead = 3,
}

impl ObjState {
    const fn from_raw(value: u32) -> Self {
        match value {
            0 => Self::New,
            1 => Self::Live,
            2 => Self::Removing,
            _ => Self::Dead,
        }
    }
}

/// Stable identity assigned to a framework object.
pub type ObjId = u64;

struct ObjMetadata {
    kind: ObjKind,
    name: Option<Box<str>>,
    owner: Option<ObjId>,
    parent: Option<ObjId>,
    state: AtomicU32,
}

struct Registry {
    entries: Mutex<BTreeMap<ObjId, Weak<ObjMetadata>>>,
}

static NEXT_ID: AtomicU64 = AtomicU64::new(1);
static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        entries: Mutex::new(BTreeMap::new()),
    });
}

fn registry() -> Option<&'static Registry> {
    REGISTRY.get()
}

/// A snapshot of one registered framework object.
#[derive(Clone, Debug)]
pub struct ObjInfo {
    /// Stable object identity.
    pub id: ObjId,
    /// Framework object kind.
    pub kind: ObjKind,
    /// Common lifecycle state.
    pub state: ObjState,
    /// Object name, when the object has one.
    pub name: Option<Box<str>>,
    /// Owning module identity, or `None` for kernel-owned objects.
    pub owner: Option<ObjId>,
    /// Parent object identity, when the object has a natural parent.
    pub parent: Option<ObjId>,
}

/// Common prefix of every framework object.
#[repr(C)]
pub struct ObjHeader {
    magic: AtomicU32,
    kind: u32,
    id: ObjId,
    metadata: Arc<ObjMetadata>,
}

impl ObjHeader {
    /// Creates a live header for `kind`.
    pub fn new(kind: ObjKind) -> Self {
        let header = Self::new_with(kind, None, None, None);
        header.set_state(ObjState::Live);
        header
    }

    /// Creates a header with descriptive object metadata.
    pub fn new_with(
        kind: ObjKind,
        name: Option<&str>,
        owner: Option<ObjId>,
        parent: Option<ObjId>,
    ) -> Self {
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        Self {
            magic: AtomicU32::new(kind.magic()),
            kind: kind as u32,
            id,
            metadata: Arc::new(ObjMetadata {
                kind,
                name: name.map(|value| String::from(value).into_boxed_str()),
                owner,
                parent,
                state: AtomicU32::new(ObjState::New as u32),
            }),
        }
    }

    /// Returns this object's type tag.
    pub const fn kind(&self) -> u32 {
        self.kind
    }

    /// Returns this object's stable numeric identity.
    pub const fn id(&self) -> ObjId {
        self.id
    }

    /// Returns this object's common lifecycle state.
    pub fn state(&self) -> ObjState {
        ObjState::from_raw(self.metadata.state.load(Ordering::Acquire))
    }

    /// Returns this object's name, when it has one.
    pub fn name(&self) -> Option<&str> {
        self.metadata.name.as_deref()
    }

    /// Returns the owning module identity, when the object is module-owned.
    pub fn owner(&self) -> Option<ObjId> {
        self.metadata.owner
    }

    /// Returns the parent object's identity, when one exists.
    pub fn parent(&self) -> Option<ObjId> {
        self.metadata.parent
    }

    /// Marks the object as published and adds it to the object registry.
    pub fn register(&self) -> Result<()> {
        let registry = registry().ok_or(Error::NotInitialized)?;
        self.metadata
            .state
            .store(ObjState::Live as u32, Ordering::Release);
        let mut entries = registry.entries.lock();
        if entries
            .insert(self.id, Arc::downgrade(&self.metadata))
            .is_some()
        {
            return Err(Error::AlreadyExists);
        }
        Ok(())
    }

    /// Records a lifecycle transition without changing handle validity.
    pub fn set_state(&self, state: ObjState) {
        self.metadata.state.store(state as u32, Ordering::Release);
    }

    /// Marks the object invalid so later handle lookups fail.
    ///
    /// The allocation stays alive until the last reference drops; poisoning
    /// only revokes handle-based access.
    pub fn poison(&self) {
        self.metadata
            .state
            .store(ObjState::Dead as u32, Ordering::Release);
        self.magic.store(MAGIC_DEAD | self.kind, Ordering::Release);
        if let Some(registry) = registry() {
            registry.entries.lock().remove(&self.id);
        }
    }

    /// Returns whether this header is still a valid handle target.
    pub fn is_live(&self) -> bool {
        self.magic.load(Ordering::Acquire) == (MAGIC_BASE | self.kind)
            && matches!(self.state(), ObjState::Live | ObjState::Removing)
    }
}

/// Returns a snapshot of all currently registered framework objects.
///
/// The registry stores weak metadata only. It therefore cannot keep an object
/// alive, and stale entries from an exceptional drop are removed while taking
/// the snapshot.
pub fn snapshot() -> Vec<ObjInfo> {
    let Some(registry) = registry() else {
        return Vec::new();
    };
    let mut entries = registry.entries.lock();
    let mut stale = Vec::new();
    let mut snapshot = Vec::new();
    for (&id, metadata) in entries.iter() {
        let Some(metadata) = metadata.upgrade() else {
            stale.push(id);
            continue;
        };
        let state = ObjState::from_raw(metadata.state.load(Ordering::Acquire));
        if state == ObjState::Dead {
            stale.push(id);
            continue;
        }
        snapshot.push(ObjInfo {
            id,
            kind: metadata.kind,
            state,
            name: metadata.name.clone(),
            owner: metadata.owner,
            parent: metadata.parent,
        });
    }
    for id in stale {
        entries.remove(&id);
    }
    snapshot
}

/// Enumerates the current object snapshot.
pub fn enumerate() -> Vec<ObjInfo> {
    snapshot()
}

/// A framework type reachable through a driver-visible handle.
///
/// # Safety
///
/// Implementors must store the header returned by [`Object::header`] as the
/// first field of a `#[repr(C)]` type so that the object address and the header
/// address are identical.
pub unsafe trait Object: Sized {
    /// Type tag validated on every handle conversion.
    const KIND: ObjKind;

    /// Returns this object's embedded header.
    fn header(&self) -> &ObjHeader;

    /// Returns this object's stable numeric identity.
    fn object_id(&self) -> ObjId {
        self.header().id()
    }

    /// Returns this object's kind.
    fn object_kind(&self) -> ObjKind {
        Self::KIND
    }

    /// Returns this object's common lifecycle state.
    fn object_state(&self) -> ObjState {
        self.header().state()
    }

    /// Returns this object's name, when it has one.
    fn object_name(&self) -> Option<&str> {
        self.header().name()
    }

    /// Returns the owning module identity, when there is one.
    fn object_owner(&self) -> Option<ObjId> {
        self.header().owner()
    }

    /// Returns the parent object's identity, when there is one.
    fn object_parent(&self) -> Option<ObjId> {
        self.header().parent()
    }

    /// Returns whether the object can still be used through a handle.
    fn object_is_live(&self) -> bool {
        self.header().is_live()
    }
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
