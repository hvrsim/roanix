//! Reference-counted framework objects addressed through opaque identities.
//!
//! Handle lookup retains an Arc before accessing an object. Retired identities
//! cannot alias a future allocation; neither stale pointers nor type tags are
//! read from memory supplied by a module. Metadata remains separately indexed
//! for diagnostics.

use alloc::{
    boxed::Box,
    collections::BTreeMap,
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    any::Any,
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
    /// A published interface.
    Interface = 6,
    /// An interrupt controller domain.
    IrqDomain = 8,
    /// A work queue.
    WorkQueue = 9,
    /// A deferrable timer.
    Timer = 10,
    /// A firmware description node.
    Fwnode = 11,
    /// A kernel-managed block device.
    BlockDevice = 12,
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
    handles: super::handle::HandleTable<Arc<dyn Any + Send + Sync>>,
}

static NEXT_ID: AtomicU64 = AtomicU64::new(1);
static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        entries: Mutex::new(BTreeMap::new()),
        handles: super::handle::HandleTable::new(),
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
        let id = NEXT_ID
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .expect("framework object identities exhausted");
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
            registry.handles.retain(|id, _| id != self.id as usize);
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

/// A framework type reachable through an opaque, typed driver identity.
pub trait Object: Sized + Any + Send + Sync {
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

/// Resolves an opaque identity and retains the object for the operation.
/// Stale or fabricated identities are rejected without accessing caller memory.
pub fn upgrade<T: Object>(handle: *const c_void) -> Result<Arc<T>> {
    let registry = registry().ok_or(Error::NotInitialized)?;
    let object = registry
        .handles
        .get(handle as usize)
        .ok_or(Error::NoDevice)?;
    let object = object.downcast::<T>().map_err(|_| Error::WrongKind)?;
    if !object.object_is_live() {
        return Err(Error::NoDevice);
    }
    Ok(object)
}

/// Borrows an object for an IRQ-safe, nonblocking scalar operation. The caller
/// must not return borrowed Rust references or reenter handle lookup in `action`.
pub fn with<T: Object, R>(handle: *const c_void, action: impl FnOnce(&T) -> R) -> Result<R> {
    registry()
        .ok_or(Error::NotInitialized)?
        .handles
        .with(handle as usize, |object| {
            let object = object.downcast_ref::<T>().ok_or(Error::WrongKind)?;
            if !object.object_is_live() {
                return Err(Error::NoDevice);
            }
            Ok(action(object))
        })
        .ok_or(Error::NoDevice)?
}

/// Returns a non-dereferenceable identity for a live framework object.
pub fn handle<T: Object>(object: &Arc<T>) -> *const c_void {
    let id = object.object_id() as usize;
    let registry = registry().expect("driver object registry initialized");
    if registry.handles.get(id).is_none() {
        let erased: Arc<dyn Any + Send + Sync> = object.clone();
        registry.handles.insert(id, erased);
    }
    id as *const c_void
}

/// Declares a framework object type with an embedded header.
macro_rules! framework_object {
    ($type:ty, $kind:ident) => {
        impl $crate::driver::obj::Object for $type {
            const KIND: $crate::driver::obj::ObjKind = $crate::driver::obj::ObjKind::$kind;

            fn header(&self) -> &$crate::driver::obj::ObjHeader {
                &self.header
            }
        }
    };
}

pub(crate) use framework_object;
