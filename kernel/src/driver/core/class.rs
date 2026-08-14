//! Device classes.
//!
//! A class is a named registry of devices that share a software contract, such
//! as `tty`, `block`, `input`, or `drm`. Classes carry no policy of their own:
//! the module that registers a class defines what the member operation table
//! means and is notified whenever a member joins or leaves.
//!
//! That is deliberately minimal, and it is what allows an entire subsystem to
//! be written as an ordinary driver. A display core registers the `drm` class,
//! receives an attach callback for every graphics device that joins, and
//! decides for itself how those devices appear to userspace.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};

use crate::sys::sync::{Mutex, Once};

use super::{
    super::{
        error::{self, Error, Result},
        obj::{ObjHeader, ObjKind, framework_object},
    },
    device::Device,
    module::{self, Module},
    name::Name,
};

/// Maximum class-name length in bytes.
pub const MAX_NAME: usize = 64;

/// Accepts a device joining a class.
pub type AttachFn = unsafe extern "C" fn(context: *mut c_void, member: *const c_void) -> i32;
/// Releases a device leaving a class.
pub type DetachFn = unsafe extern "C" fn(context: *mut c_void, member: *const c_void);

/// Callbacks a class implements.
#[derive(Default)]
pub struct ClassOps {
    /// Optional callback invoked when a member joins.
    pub attach: Option<AttachFn>,
    /// Optional callback invoked when a member leaves.
    pub detach: Option<DetachFn>,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// A registered device class.
#[repr(C)]
pub struct Class {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    ops: ClassOps,
    members: Mutex<Vec<Arc<ClassDevice>>>,
    next_index: AtomicU64,
}

framework_object!(Class, Class);

// SAFETY: class callbacks and context stay valid until the owning module is
// unloaded, and unload detaches every member first.
unsafe impl Send for Class {}
// SAFETY: membership changes are serialised by the members lock, and the ABI
// requires attach and detach to tolerate concurrent invocation.
unsafe impl Sync for Class {}

impl Class {
    /// Returns the class name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the module that registered this class.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns every current member.
    pub fn members(&self) -> Vec<Arc<ClassDevice>> {
        self.members.lock().clone()
    }

    /// Returns the member named `name`.
    pub fn find(&self, name: &str) -> Option<Arc<ClassDevice>> {
        self.members
            .lock()
            .iter()
            .find(|member| member.name.as_str() == name)
            .cloned()
    }
}

/// One device's membership in a class.
#[repr(C)]
pub struct ClassDevice {
    header: ObjHeader,
    class: Arc<Class>,
    device: Option<Weak<Device>>,
    owner: Option<Arc<Module>>,
    name: Name,
    index: u64,
    ops: *const c_void,
    ops_size: usize,
    context: *mut c_void,
    class_data: Mutex<usize>,
    live: AtomicBool,
}

framework_object!(ClassDevice, ClassDevice);

// SAFETY: the member operation table is immutable and outlives the membership,
// which is dropped before the providing module is unloaded.
unsafe impl Send for ClassDevice {}
// SAFETY: the table pointer is immutable after registration and the contract
// for its contents is defined by the owning class.
unsafe impl Sync for ClassDevice {}

impl ClassDevice {
    /// Returns the class this membership belongs to.
    pub fn class(&self) -> &Arc<Class> {
        &self.class
    }

    /// Returns the underlying device, if it still exists.
    pub fn device(&self) -> Option<Arc<Device>> {
        self.device.as_ref().and_then(Weak::upgrade)
    }

    /// Returns the module that added this member.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns the member name within the class.
    pub fn name(&self) -> &str {
        self.name.as_str()
    }

    /// Returns the member name as a NUL-terminated pointer for the ABI.
    pub fn name_c_ptr(&self) -> *const core::ffi::c_char {
        self.name.as_c_ptr()
    }

    /// Returns the index assigned when the member joined.
    pub const fn index(&self) -> u64 {
        self.index
    }

    /// Returns the member operation table defined by the class contract.
    pub const fn ops(&self) -> *const c_void {
        self.ops
    }

    /// Returns the member operation-table size.
    pub const fn ops_size(&self) -> usize {
        self.ops_size
    }

    /// Returns the member context passed to every operation.
    pub const fn context(&self) -> *mut c_void {
        self.context
    }

    /// Returns the private value the class stored on this membership.
    pub fn class_data(&self) -> usize {
        *self.class_data.lock()
    }

    /// Stores a private value on behalf of the owning class.
    pub fn set_class_data(&self, value: usize) {
        *self.class_data.lock() = value;
    }

    /// Returns whether the membership is still active.
    pub fn is_live(&self) -> bool {
        self.live.load(Ordering::Acquire)
    }
}

/// Description supplied when a device joins a class.
pub struct Membership {
    /// Member name within the class, such as `ttyS0`.
    pub name: Box<str>,
    /// Member operation table interpreted by the class contract.
    pub ops: *const c_void,
    /// Member operation-table size.
    pub ops_size: usize,
    /// Member context passed back to every operation.
    pub context: *mut c_void,
}

struct Registry {
    classes: Mutex<Vec<Arc<Class>>>,
}

static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        classes: Mutex::new(Vec::new()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

/// Registers a class.
///
/// # Safety
///
/// Every callback in `ops` must follow the class ABI and stay executable until
/// the class is unregistered.
pub unsafe fn register(
    owner: Option<&Arc<Module>>,
    name: &str,
    ops: ClassOps,
) -> Result<Arc<Class>> {
    if name.is_empty() || name.len() > MAX_NAME {
        return Err(Error::InvalidArgument);
    }
    let registry = registry()?;
    let mut classes = registry.classes.lock();
    if classes.iter().any(|class| &*class.name == name) {
        return Err(Error::AlreadyExists);
    }
    let class = Arc::new(Class {
        header: ObjHeader::new(ObjKind::Class),
        name: String::from(name).into_boxed_str(),
        owner: owner.cloned(),
        ops,
        members: Mutex::new(Vec::new()),
        next_index: AtomicU64::new(0),
    });
    classes.push(class.clone());
    drop(classes);
    super::probe::retrigger();
    Ok(class)
}

/// Unregisters a class once it has no members.
pub fn unregister(class: &Arc<Class>) -> Result<()> {
    if !class.members.lock().is_empty() {
        return Err(Error::Busy);
    }
    force_unregister(class);
    Ok(())
}

fn force_unregister(class: &Arc<Class>) {
    for member in class.members() {
        remove_device(&member);
    }
    if let Ok(registry) = registry() {
        registry
            .classes
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, class));
    }
    class.header.poison();
}

/// Returns a registered class by name.
pub fn find(name: &str) -> Result<Arc<Class>> {
    registry()?
        .classes
        .lock()
        .iter()
        .find(|class| &*class.name == name)
        .cloned()
        .ok_or(Error::NotFound)
}

/// Adds `device` to `class`.
///
/// # Safety
///
/// `membership.ops` must address an immutable table matching the class
/// contract and stay valid until the member is removed.
pub unsafe fn add_device(
    owner: Option<&Arc<Module>>,
    class: &Arc<Class>,
    device: Option<&Arc<Device>>,
    membership: Membership,
) -> Result<Arc<ClassDevice>> {
    if membership.name.is_empty() || membership.name.len() > MAX_NAME {
        return Err(Error::InvalidArgument);
    }
    let _class_pin = module::pin_owner(class.owner.as_ref(), false)?;

    let mut members = class.members.lock();
    if members
        .iter()
        .any(|member| member.name.as_str() == &*membership.name && member.is_live())
    {
        return Err(Error::AlreadyExists);
    }
    let member = Arc::new(ClassDevice {
        header: ObjHeader::new(ObjKind::ClassDevice),
        class: class.clone(),
        device: device.map(Arc::downgrade),
        owner: owner.cloned(),
        name: Name::new(&membership.name),
        index: class.next_index.fetch_add(1, Ordering::Relaxed),
        ops: membership.ops,
        ops_size: membership.ops_size,
        context: membership.context,
        class_data: Mutex::new(0),
        live: AtomicBool::new(true),
    });
    members.push(member.clone());
    drop(members);

    // The member's module now depends on the class provider staying loaded.
    if let Err(error) = module::link(owner, class.owner.as_ref()) {
        detach_member(class, &member);
        return Err(error);
    }

    if let Some(attach) = class.ops.attach {
        let handle = super::super::obj::handle(&member);
        // SAFETY: registration validated the callback, and the handle addresses
        // a live membership for the duration of the call.
        let status = unsafe { attach(class.ops.context, handle) };
        if let Err(error) = error::from_status(status) {
            module::unlink(owner, class.owner.as_ref());
            detach_member(class, &member);
            return Err(error);
        }
    }

    if let Some(device) = device {
        device.attach_class(member.clone());
    }
    Ok(member)
}

/// Removes a class membership.
pub fn remove_device(member: &Arc<ClassDevice>) {
    if !member.live.swap(false, Ordering::AcqRel) {
        return;
    }
    let class = member.class.clone();
    if let Some(detach) = class.ops.detach
        && let Ok(_pin) = module::pin_owner(class.owner.as_ref(), true)
    {
        let handle = super::super::obj::handle(member);
        // SAFETY: registration validated the callback, and the handle addresses
        // a live membership for the duration of the call.
        unsafe { detach(class.ops.context, handle) };
    }
    module::unlink(member.owner.as_ref(), class.owner.as_ref());
    if let Some(device) = member.device() {
        device.detach_class(member);
    }
    detach_member(&class, member);
    member.header.poison();
}

fn detach_member(class: &Arc<Class>, member: &Arc<ClassDevice>) {
    class
        .members
        .lock()
        .retain(|entry| !Arc::ptr_eq(entry, member));
}

pub(super) fn remove_module_classes(module: &Arc<Module>) {
    let Ok(registry) = registry() else {
        return;
    };
    // Members contributed by this module must leave classes owned by others.
    let all_classes: Vec<Arc<Class>> = registry.classes.lock().clone();
    for class in &all_classes {
        for member in class.members() {
            if member
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
            {
                remove_device(&member);
            }
        }
    }
    for class in all_classes {
        if class
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module))
        {
            force_unregister(&class);
        }
    }
}

/// Snapshot of one registered class.
#[derive(Clone, Debug)]
pub struct ClassInfo {
    /// Class name.
    pub name: Box<str>,
    /// Number of current members.
    pub members: usize,
    /// Registering module name.
    pub owner: Option<Box<str>>,
}

/// Returns a snapshot of every registered class.
pub fn list() -> Result<Vec<ClassInfo>> {
    Ok(registry()?
        .classes
        .lock()
        .iter()
        .map(|class| ClassInfo {
            name: class.name.to_string().into_boxed_str(),
            members: class.members.lock().len(),
            owner: class
                .owner
                .as_ref()
                .map(|owner| owner.name().to_string().into_boxed_str()),
        })
        .collect())
}
