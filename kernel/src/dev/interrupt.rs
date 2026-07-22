//! Interrupt-controller domains, routing, and lock-free handler dispatch.

use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
    vec,
    vec::Vec,
};
use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    mem,
    sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
};

use log::error;

use crate::sys::{
    smp,
    sync::{Mutex, Once},
};

use super::{
    BusId, DeviceNodeId, DriverId, Error, ResourceFlags, ResourceKey, ResourceValue, Result,
    abi::{AbiSlice, STATUS_NOT_FOUND, STATUS_OK},
    driver::{self, CallbackOwner},
};

/// Version of interrupt-controller and route ABI records.
pub const INTERRUPT_ABI_V1: u32 = 1;

/// Inherited resource identifying the interrupt domain for a bus subtree.
pub const INTERRUPT_DOMAIN_RESOURCE: ResourceKey = ResourceKey::new(0x524F_414E_4958_4952, 1);

/// Controller handles the architecture's root external-interrupt input.
pub const CONTROLLER_ROOT: u64 = 1 << 0;

/// Route is edge triggered.
pub const ROUTE_EDGE: u64 = 1 << 0;
/// Route is level triggered.
pub const ROUTE_LEVEL: u64 = 1 << 1;
/// Route is active high.
pub const ROUTE_ACTIVE_HIGH: u64 = 1 << 2;
/// Route is active low.
pub const ROUTE_ACTIVE_LOW: u64 = 1 << 3;
/// Leave the route masked after installing its handler.
pub const ROUTE_START_MASKED: u64 = 1 << 4;

/// Interrupt handler requests a scheduler trap return.
pub const HANDLER_RESCHEDULE: u32 = 1 << 0;

const CONTROLLER_FLAGS: u64 = CONTROLLER_ROOT;
const ROUTE_FLAGS: u64 =
    ROUTE_EDGE | ROUTE_LEVEL | ROUTE_ACTIVE_HIGH | ROUTE_ACTIVE_LOW | ROUTE_START_MASKED;
const DOMAIN_RECORD_SIZE: usize = 16;
const MAX_INTERRUPT_SLOTS: usize = 1024;
const INTERRUPT_SLOT_BITS: u32 = 16;
const INTERRUPT_SLOT_MASK: u64 = (1 << INTERRUPT_SLOT_BITS) - 1;
const MAX_SPECIFIER_SIZE: usize = 4096;
const MAX_CLAIMS_PER_TRAP: usize = 256;

#[cfg(target_arch = "x86_64")]
const FIRST_EXTERNAL_VECTOR: u16 = 0x40;
#[cfg(target_arch = "x86_64")]
const LAST_EXTERNAL_VECTOR: u16 = 0xDF;
#[cfg(target_arch = "x86_64")]
const VECTOR_EMPTY: u64 = 0;
#[cfg(target_arch = "x86_64")]
const VECTOR_QUIESCED: u64 = u64::MAX;

/// Stable handle for one registered interrupt controller.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct InterruptControllerId(u64);

impl InterruptControllerId {
    /// Creates a handle from its ABI representation.
    pub const fn from_raw(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric controller identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Generation-tagged handle for one routed interrupt.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct InterruptId(u64);

impl InterruptId {
    /// Creates a handle from its ABI representation.
    pub const fn from_raw(value: u64) -> Self {
        Self(value)
    }

    /// Returns the ABI representation.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Immutable route description passed to controller callbacks.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct InterruptRouteV1 {
    /// Size of this record.
    pub size: u32,
    /// Must equal [`INTERRUPT_ABI_V1`].
    pub abi_version: u32,
    /// Logical interrupt assigned by the kernel.
    pub interrupt: u64,
    /// Architecture vector, or `u32::MAX` when the architecture has none.
    pub vector: u32,
    /// Logical target CPU.
    pub target_cpu: u32,
    /// Architecture platform identifier for `target_cpu`.
    pub target_platform_id: u64,
    /// Route electrical and behavior flags.
    pub flags: u64,
    /// Controller-specific firmware interrupt specifier.
    pub specifier: AbiSlice,
}

/// Installs one controller route in a masked state.
pub type InterruptConnectFn = unsafe extern "C" fn(
    context: usize,
    route: *const InterruptRouteV1,
    out_cookie: *mut u64,
) -> i32;
/// Removes one masked controller route.
pub type InterruptDisconnectFn = unsafe extern "C" fn(context: usize, cookie: u64) -> i32;
/// Masks or unmasks one controller route.
pub type InterruptLineFn = unsafe extern "C" fn(context: usize, cookie: u64) -> i32;
/// Retargets one masked or live route.
pub type InterruptSetAffinityFn =
    unsafe extern "C" fn(context: usize, cookie: u64, route: *const InterruptRouteV1) -> i32;
/// Claims one pending interrupt from a root controller.
pub type InterruptClaimFn = unsafe extern "C" fn(
    context: usize,
    cpu: u32,
    platform_id: u64,
    out_interrupt: *mut u64,
    out_cookie: *mut u64,
) -> i32;
/// Completes one interrupt claimed from a root controller.
pub type InterruptCompleteFn =
    unsafe extern "C" fn(context: usize, cpu: u32, platform_id: u64, interrupt: u64, cookie: u64);
/// Device interrupt handler.
pub type InterruptHandlerFn = unsafe extern "C" fn(context: usize, interrupt: u64) -> u32;

/// Versioned controller callback table copied by the kernel.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct InterruptControllerV1 {
    /// Size of this record.
    pub size: u32,
    /// Must equal [`INTERRUPT_ABI_V1`].
    pub abi_version: u32,
    /// Controller capability flags.
    pub flags: u64,
    /// Opaque controller context.
    pub context: usize,
    /// Required route-install callback.
    pub connect: Option<InterruptConnectFn>,
    /// Required route-removal callback.
    pub disconnect: Option<InterruptDisconnectFn>,
    /// Required route-mask callback.
    pub mask: Option<InterruptLineFn>,
    /// Required route-unmask callback.
    pub unmask: Option<InterruptLineFn>,
    /// Optional route-affinity callback.
    pub set_affinity: Option<InterruptSetAffinityFn>,
    /// Required for root controllers.
    pub claim: Option<InterruptClaimFn>,
    /// Required for root controllers.
    pub complete: Option<InterruptCompleteFn>,
}

/// Result of one architecture interrupt dispatch.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct DispatchOutcome {
    /// Hardware delivery belonged to the interrupt subsystem.
    pub handled: bool,
    /// One handler requested scheduler trap return.
    pub reschedule: bool,
}

impl DispatchOutcome {
    const fn unhandled() -> Self {
        Self {
            handled: false,
            reschedule: false,
        }
    }

    const fn handled() -> Self {
        Self {
            handled: true,
            reschedule: false,
        }
    }

    fn merge(&mut self, other: Self) {
        self.handled |= other.handled;
        self.reschedule |= other.reschedule;
    }
}

struct ControllerRecord {
    id: InterruptControllerId,
    owner: DriverId,
    bus: BusId,
    callback_owner: CallbackOwner,
    operations: InterruptControllerV1,
    available: AtomicBool,
}

#[derive(Clone, Eq, Ord, PartialEq, PartialOrd)]
struct RouteKey {
    controller: InterruptControllerId,
    specifier: Box<[u8]>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum RouteState {
    Connecting,
    Ready,
    Updating,
    Prepared { was_masked: bool },
}

struct RouteRecord {
    owner: DriverId,
    controller: Arc<ControllerRecord>,
    key: RouteKey,
    flags: u64,
    vector: Option<u8>,
    cookie: u64,
    masked: bool,
    state: RouteState,
}

#[derive(Clone)]
struct RouteSnapshot {
    id: InterruptId,
    controller: Arc<ControllerRecord>,
    key: RouteKey,
    flags: u64,
    vector: Option<u8>,
    cookie: u64,
    masked: bool,
}

struct RegistryState {
    next_controller: u64,
    controllers: BTreeMap<InterruptControllerId, Arc<ControllerRecord>>,
    root_controller: Option<InterruptControllerId>,
    routes: BTreeMap<InterruptId, RouteRecord>,
    route_keys: BTreeMap<RouteKey, InterruptId>,
    used_slots: Box<[bool]>,
    slot_generations: Box<[u64]>,
    pending_drivers: BTreeSet<DriverId>,
    #[cfg(target_arch = "x86_64")]
    next_vector: u16,
}

struct InterruptRegistry {
    state: Mutex<RegistryState>,
}

struct InterruptSlotData {
    owner: CallbackOwner,
    controller: InterruptControllerId,
    handler: InterruptHandlerFn,
    context: usize,
}

struct InterruptSlot {
    published_id: AtomicU64,
    active: AtomicUsize,
    data: UnsafeCell<Option<InterruptSlotData>>,
}

struct RootControllerSlot {
    published_id: AtomicU64,
    active: AtomicUsize,
    controller: UnsafeCell<Option<Arc<ControllerRecord>>>,
}

static REGISTRY: Once<InterruptRegistry> = Once::new();
static INTERRUPT_SLOTS: [InterruptSlot; MAX_INTERRUPT_SLOTS] =
    [const { InterruptSlot::new() }; MAX_INTERRUPT_SLOTS];
static ROOT_CONTROLLER: RootControllerSlot = RootControllerSlot::new();

#[cfg(target_arch = "x86_64")]
static VECTOR_MAP: [AtomicU64; 256] = [const { AtomicU64::new(VECTOR_EMPTY) }; 256];

// SAFETY: control-plane mutation only occurs while the slot is unpublished and
// its active count is zero. Dispatch increments the count before observing the
// published identifier and reads data only after an exact identifier match.
unsafe impl Sync for InterruptSlot {}

// SAFETY: the root controller uses the same publish/drain protocol as an
// interrupt slot, and the stored Arc is immutable while published.
unsafe impl Sync for RootControllerSlot {}

impl InterruptRegistry {
    fn new() -> Self {
        Self {
            state: Mutex::new(RegistryState {
                next_controller: 1,
                controllers: BTreeMap::new(),
                root_controller: None,
                routes: BTreeMap::new(),
                route_keys: BTreeMap::new(),
                used_slots: vec![false; MAX_INTERRUPT_SLOTS].into_boxed_slice(),
                slot_generations: vec![0; MAX_INTERRUPT_SLOTS].into_boxed_slice(),
                pending_drivers: BTreeSet::new(),
                #[cfg(target_arch = "x86_64")]
                next_vector: FIRST_EXTERNAL_VECTOR,
            }),
        }
    }
}

impl ControllerRecord {
    fn route_descriptor(
        &self,
        id: InterruptId,
        key: &RouteKey,
        vector: Option<u8>,
        flags: u64,
        target_cpu: u32,
        target_platform_id: u64,
    ) -> InterruptRouteV1 {
        InterruptRouteV1 {
            size: mem::size_of::<InterruptRouteV1>() as u32,
            abi_version: INTERRUPT_ABI_V1,
            interrupt: id.get(),
            vector: vector.map_or(u32::MAX, u32::from),
            target_cpu,
            target_platform_id,
            flags,
            specifier: AbiSlice {
                data: key.specifier.as_ptr(),
                len: key.specifier.len(),
            },
        }
    }

    fn connect(&self, route: &InterruptRouteV1) -> Result<u64> {
        let _callback = self.callback_owner.acquire_control()?;
        let callback = self.operations.connect.ok_or(Error::Unsupported)?;
        let mut cookie = 0u64;
        // SAFETY: controller registration validated this callback and the route
        // record and output remain live for the duration of the call.
        callback_result(unsafe { callback(self.operations.context, route, &mut cookie) })?;
        Ok(cookie)
    }

    fn disconnect(&self, cookie: u64, cleanup: bool) -> Result<()> {
        let _callback = if cleanup {
            self.callback_owner.acquire_cleanup()?
        } else {
            self.callback_owner.acquire_control()?
        };
        let callback = self.operations.disconnect.ok_or(Error::Unsupported)?;
        // SAFETY: controller registration validated this callback and `cookie`
        // was returned by its connect callback.
        callback_result(unsafe { callback(self.operations.context, cookie) })
    }

    fn mask(&self, cookie: u64, cleanup: bool) -> Result<()> {
        let _callback = if cleanup {
            self.callback_owner.acquire_cleanup()?
        } else {
            self.callback_owner.acquire_control()?
        };
        let callback = self.operations.mask.ok_or(Error::Unsupported)?;
        // SAFETY: controller registration validated this callback and `cookie`
        // identifies one live route.
        callback_result(unsafe { callback(self.operations.context, cookie) })
    }

    fn unmask(&self, cookie: u64) -> Result<()> {
        let _callback = self.callback_owner.acquire_control()?;
        let callback = self.operations.unmask.ok_or(Error::Unsupported)?;
        // SAFETY: controller registration validated this callback and `cookie`
        // identifies one live route.
        callback_result(unsafe { callback(self.operations.context, cookie) })
    }

    fn set_affinity(&self, cookie: u64, route: &InterruptRouteV1) -> Result<()> {
        let _callback = self.callback_owner.acquire_control()?;
        let callback = self.operations.set_affinity.ok_or(Error::Unsupported)?;
        // SAFETY: controller registration validated this callback and both
        // records remain live for the duration of the call.
        callback_result(unsafe { callback(self.operations.context, cookie, route) })
    }
}

impl InterruptSlot {
    const fn new() -> Self {
        Self {
            published_id: AtomicU64::new(0),
            active: AtomicUsize::new(0),
            data: UnsafeCell::new(None),
        }
    }

    fn install(&self, id: InterruptId, data: InterruptSlotData) {
        assert_eq!(
            self.published_id.load(Ordering::SeqCst),
            0,
            "dev/interrupt: installing a published slot"
        );
        assert_eq!(
            self.active.load(Ordering::SeqCst),
            0,
            "dev/interrupt: installing an active slot"
        );
        // SAFETY: the slot is unpublished, inactive, and exclusively reserved
        // by the control-plane registry.
        unsafe { *self.data.get() = Some(data) };
        self.published_id.store(id.get(), Ordering::SeqCst);
    }

    fn dispatch(
        &self,
        id: InterruptId,
        controller: Option<InterruptControllerId>,
    ) -> DispatchOutcome {
        self.active.fetch_add(1, Ordering::SeqCst);
        if self.published_id.load(Ordering::SeqCst) != id.get() {
            self.finish_dispatch();
            return DispatchOutcome::unhandled();
        }

        // SAFETY: an exact published-ID match observed after incrementing the
        // active count prevents control-plane mutation until dispatch exits.
        let data = unsafe { (&*self.data.get()).as_ref() }
            .expect("dev/interrupt: published slot without handler data");
        if controller.is_some_and(|controller| controller != data.controller) {
            self.finish_dispatch();
            return DispatchOutcome::unhandled();
        }
        let result = match data.owner.acquire_irq() {
            Ok(_callback) => {
                // SAFETY: the callback owner pins the driver code and the
                // registration contract requires an IRQ-safe callback.
                unsafe { (data.handler)(data.context, id.get()) }
            }
            Err(_) => 0,
        };
        self.finish_dispatch();
        DispatchOutcome {
            handled: true,
            reschedule: result & HANDLER_RESCHEDULE != 0,
        }
    }

    fn suspend(&self, id: InterruptId) {
        let previous = self.published_id.swap(0, Ordering::SeqCst);
        assert_eq!(
            previous,
            id.get(),
            "dev/interrupt: suspending the wrong slot generation"
        );
        self.wait_inactive();
    }

    fn resume(&self, id: InterruptId) {
        assert_eq!(
            self.active.load(Ordering::SeqCst),
            0,
            "dev/interrupt: resuming an active slot"
        );
        assert_eq!(
            self.published_id.swap(id.get(), Ordering::SeqCst),
            0,
            "dev/interrupt: resuming a published slot"
        );
    }

    fn remove_suspended(&self) {
        assert_eq!(
            self.published_id.load(Ordering::SeqCst),
            0,
            "dev/interrupt: removing a published slot"
        );
        self.wait_inactive();
        // SAFETY: the slot is unpublished, inactive, and exclusively owned by
        // the control plane.
        let data = unsafe { (&mut *self.data.get()).take() };
        assert!(
            data.is_some(),
            "dev/interrupt: removing an empty interrupt slot"
        );
    }

    fn finish_dispatch(&self) {
        let previous = self.active.fetch_sub(1, Ordering::SeqCst);
        assert!(previous != 0, "dev/interrupt: slot activity underflow");
    }

    fn wait_inactive(&self) {
        while self.active.load(Ordering::SeqCst) != 0 {
            spin_loop();
        }
    }
}

impl RootControllerSlot {
    const fn new() -> Self {
        Self {
            published_id: AtomicU64::new(0),
            active: AtomicUsize::new(0),
            controller: UnsafeCell::new(None),
        }
    }

    fn install(&self, controller: Arc<ControllerRecord>) {
        assert_eq!(
            self.published_id.load(Ordering::SeqCst),
            0,
            "dev/interrupt: root controller already published"
        );
        assert_eq!(
            self.active.load(Ordering::SeqCst),
            0,
            "dev/interrupt: root controller active during install"
        );
        let id = controller.id.get();
        // SAFETY: the root slot is unpublished, inactive, and exclusively
        // reserved by the control-plane registry.
        unsafe { *self.controller.get() = Some(controller) };
        self.published_id.store(id, Ordering::SeqCst);
    }

    fn dispatch(&self, cpu: u32, platform_id: u64) -> DispatchOutcome {
        self.active.fetch_add(1, Ordering::SeqCst);
        let id = self.published_id.load(Ordering::SeqCst);
        if id == 0 {
            self.finish_dispatch();
            return DispatchOutcome::unhandled();
        }

        // SAFETY: the published-ID check observed after incrementing activity
        // prevents the stored Arc from being removed during this dispatch.
        let controller = unsafe { (&*self.controller.get()).as_ref() }
            .expect("dev/interrupt: published root controller without data");
        let outcome = dispatch_root_controller(controller, cpu, platform_id);
        self.finish_dispatch();
        outcome
    }

    fn suspend(&self, id: InterruptControllerId) {
        let previous = self.published_id.swap(0, Ordering::SeqCst);
        assert_eq!(
            previous,
            id.get(),
            "dev/interrupt: suspending the wrong root controller"
        );
        self.wait_inactive();
    }

    fn resume(&self, id: InterruptControllerId) {
        assert_eq!(
            self.active.load(Ordering::SeqCst),
            0,
            "dev/interrupt: resuming an active root controller"
        );
        assert_eq!(
            self.published_id.swap(id.get(), Ordering::SeqCst),
            0,
            "dev/interrupt: resuming a published root controller"
        );
    }

    fn remove_suspended(&self) {
        assert_eq!(
            self.published_id.load(Ordering::SeqCst),
            0,
            "dev/interrupt: removing a published root controller"
        );
        self.wait_inactive();
        // SAFETY: the root slot is unpublished, inactive, and exclusively
        // owned by the control plane.
        let controller = unsafe { (&mut *self.controller.get()).take() };
        assert!(
            controller.is_some(),
            "dev/interrupt: removing an empty root-controller slot"
        );
    }

    fn finish_dispatch(&self) {
        let previous = self.active.fetch_sub(1, Ordering::SeqCst);
        assert!(
            previous != 0,
            "dev/interrupt: root-controller activity underflow"
        );
    }

    fn wait_inactive(&self) {
        while self.active.load(Ordering::SeqCst) != 0 {
            spin_loop();
        }
    }
}

/// Initializes interrupt routing state.
pub(crate) fn init() {
    REGISTRY.call_once(InterruptRegistry::new);
}

/// Registers a controller and publishes its inherited interrupt domain.
///
/// # Safety
///
/// Every callback must follow the version-1 ABI and remain executable until
/// controller removal. `claim`, `complete`, and interrupt handlers must be
/// allocation-free, non-blocking, and safe in interrupt context.
pub unsafe fn register_controller(
    owner: DriverId,
    bus: BusId,
    operations: InterruptControllerV1,
) -> Result<InterruptControllerId> {
    let _owner = driver::mutation_guard(owner)?;
    validate_controller(&operations)?;
    let callback_owner = driver::callback_owner(owner)?;
    let registry = registry()?;
    let is_root = operations.flags & CONTROLLER_ROOT != 0;

    let controller = {
        let mut state = registry.state.lock();
        ensure_driver_available(&state, owner)?;
        if is_root && state.root_controller.is_some() {
            return Err(Error::AlreadyExists);
        }
        let raw_id = state.next_controller;
        state.next_controller = state
            .next_controller
            .checked_add(1)
            .filter(|value| *value != 0)
            .expect("dev/interrupt: controller identifier wrapped");
        let id = InterruptControllerId::from_raw(raw_id);
        let controller = Arc::new(ControllerRecord {
            id,
            owner,
            bus,
            callback_owner,
            operations,
            available: AtomicBool::new(false),
        });
        state.controllers.insert(id, controller.clone());
        if is_root {
            state.root_controller = Some(id);
        }
        controller
    };

    let resource = domain_record(controller.id);
    if let Err(error) = super::publish_resource(
        owner,
        bus,
        INTERRUPT_DOMAIN_RESOURCE,
        ResourceFlags::CACHEABLE,
        ResourceValue::Data(resource),
    ) {
        rollback_controller_registration(controller.id);
        return Err(error);
    }

    if is_root {
        ROOT_CONTROLLER.install(controller.clone());
    }
    controller.available.store(true, Ordering::Release);
    #[cfg(target_arch = "riscv64")]
    if is_root {
        crate::arch::set_external_interrupts(true);
    }
    Ok(controller.id)
}

/// Unregisters an idle controller and removes its interrupt domain resource.
pub fn unregister_controller(owner: DriverId, id: InterruptControllerId) -> Result<()> {
    let _owner = driver::mutation_guard(owner)?;
    let registry = registry()?;
    let controller = {
        let state = registry.state.lock();
        ensure_driver_available(&state, owner)?;
        let controller = state.controllers.get(&id).cloned().ok_or(Error::NotFound)?;
        if controller.owner != owner {
            return Err(Error::PermissionDenied);
        }
        if state.routes.values().any(|route| route.controller.id == id) {
            return Err(Error::Busy);
        }
        controller.available.store(false, Ordering::Release);
        controller
    };

    let is_root = controller.operations.flags & CONTROLLER_ROOT != 0;
    if is_root {
        #[cfg(target_arch = "riscv64")]
        crate::arch::set_external_interrupts(false);
        ROOT_CONTROLLER.suspend(id);
    }
    if let Err(error) = super::remove_resource(owner, controller.bus, INTERRUPT_DOMAIN_RESOURCE) {
        if is_root {
            ROOT_CONTROLLER.resume(id);
            #[cfg(target_arch = "riscv64")]
            crate::arch::set_external_interrupts(true);
        }
        controller.available.store(true, Ordering::Release);
        return Err(error);
    }

    {
        let mut state = registry.state.lock();
        state.controllers.remove(&id);
        if state.root_controller == Some(id) {
            state.root_controller = None;
        }
    }
    if is_root {
        ROOT_CONTROLLER.remove_suspended();
    }
    Ok(())
}

/// Routes a device interrupt through the nearest inherited controller domain.
///
/// The controller connect callback must install the hardware route masked.
pub fn request_interrupt(
    owner: DriverId,
    node: DeviceNodeId,
    specifier: &[u8],
    flags: u64,
    target_cpu: u32,
    handler: InterruptHandlerFn,
    context: usize,
) -> Result<InterruptId> {
    let _owner = driver::mutation_guard(owner)?;
    validate_route_flags(flags)?;
    if specifier.len() > MAX_SPECIFIER_SIZE {
        return Err(Error::InvalidArgument);
    }
    if super::node_info(node)?.owner != owner {
        return Err(Error::PermissionDenied);
    }
    let target_cpu_usize = usize::try_from(target_cpu).map_err(|_| Error::InvalidArgument)?;
    let target_platform_id = smp::platform_id(target_cpu_usize).ok_or(Error::NotFound)?;
    if target_cpu != 0 && !smp::is_online(target_cpu_usize) {
        return Err(Error::Busy);
    }

    let controller_id = resolve_domain(node)?;
    let callback_owner = driver::callback_owner(owner)?;
    let key = RouteKey {
        controller: controller_id,
        specifier: Box::from(specifier),
    };
    let registry = registry()?;

    let (id, controller, vector) = {
        let mut state = registry.state.lock();
        ensure_driver_available(&state, owner)?;
        let controller = state
            .controllers
            .get(&controller_id)
            .cloned()
            .ok_or(Error::NotFound)?;
        ensure_driver_available(&state, controller.owner)?;
        if !controller.available.load(Ordering::Acquire) || !controller.callback_owner.is_loaded() {
            return Err(Error::Busy);
        }
        if state.route_keys.contains_key(&key) {
            return Err(Error::AlreadyExists);
        }

        let id = allocate_interrupt_id(&mut state)?;
        #[cfg(target_arch = "x86_64")]
        let vector = match allocate_vector(&mut state) {
            Ok(vector) => Some(vector),
            Err(error) => {
                release_interrupt_id(&mut state, id);
                return Err(error);
            }
        };
        #[cfg(target_arch = "riscv64")]
        let vector = None;

        let record = RouteRecord {
            owner,
            controller: controller.clone(),
            key: key.clone(),
            flags,
            vector,
            cookie: 0,
            masked: true,
            state: RouteState::Connecting,
        };
        state.route_keys.insert(key.clone(), id);
        state.routes.insert(id, record);
        (id, controller, vector)
    };

    let route =
        controller.route_descriptor(id, &key, vector, flags, target_cpu, target_platform_id);
    let cookie = match controller.connect(&route) {
        Ok(cookie) => cookie,
        Err(error) => {
            abandon_connecting_route(id);
            return Err(error);
        }
    };

    interrupt_slot(id).install(
        id,
        InterruptSlotData {
            owner: callback_owner,
            controller: controller.id,
            handler,
            context,
        },
    );
    #[cfg(target_arch = "x86_64")]
    VECTOR_MAP[vector.expect("x86 interrupt route without vector") as usize]
        .store(id.get(), Ordering::Release);

    {
        let mut state = registry.state.lock();
        let record = state
            .routes
            .get_mut(&id)
            .expect("dev/interrupt: connecting route disappeared");
        record.cookie = cookie;
    }

    let start_masked = flags & ROUTE_START_MASKED != 0;
    if !start_masked && let Err(error) = controller.unmask(cookie) {
        rollback_connected_route(id, &controller, cookie);
        return Err(error);
    }

    {
        let mut state = registry.state.lock();
        let record = state
            .routes
            .get_mut(&id)
            .expect("dev/interrupt: connected route disappeared");
        record.masked = start_masked;
        record.state = RouteState::Ready;
    }
    Ok(id)
}

/// Releases one routed interrupt owned by `owner`.
pub fn release_interrupt(owner: DriverId, id: InterruptId) -> Result<()> {
    let _owner = driver::mutation_guard(owner)?;
    let snapshot = begin_route_update(owner, id)?;
    if !snapshot.masked
        && let Err(error) = snapshot.controller.mask(snapshot.cookie, false)
    {
        finish_route_update(id, snapshot.masked);
        return Err(error);
    }

    interrupt_slot(id).suspend(id);
    quiesce_vector(snapshot.vector);
    if let Err(error) = snapshot.controller.disconnect(snapshot.cookie, false) {
        restore_vector(snapshot.vector, id);
        interrupt_slot(id).resume(id);
        let masked = if snapshot.masked {
            true
        } else {
            snapshot.controller.unmask(snapshot.cookie).is_err()
        };
        finish_route_update(id, masked);
        return Err(error);
    }

    interrupt_slot(id).remove_suspended();
    remove_route_record(id);
    Ok(())
}

/// Masks one routed interrupt.
pub fn mask_interrupt(owner: DriverId, id: InterruptId) -> Result<()> {
    let _owner = driver::mutation_guard(owner)?;
    let snapshot = begin_route_update(owner, id)?;
    if snapshot.masked {
        finish_route_update(id, true);
        return Ok(());
    }
    match snapshot.controller.mask(snapshot.cookie, false) {
        Ok(()) => {
            finish_route_update(id, true);
            Ok(())
        }
        Err(error) => {
            finish_route_update(id, false);
            Err(error)
        }
    }
}

/// Unmasks one routed interrupt.
pub fn unmask_interrupt(owner: DriverId, id: InterruptId) -> Result<()> {
    let _owner = driver::mutation_guard(owner)?;
    let snapshot = begin_route_update(owner, id)?;
    if !snapshot.masked {
        finish_route_update(id, false);
        return Ok(());
    }
    match snapshot.controller.unmask(snapshot.cookie) {
        Ok(()) => {
            finish_route_update(id, false);
            Ok(())
        }
        Err(error) => {
            finish_route_update(id, true);
            Err(error)
        }
    }
}

/// Retargets one routed interrupt to an online CPU.
pub fn set_interrupt_affinity(owner: DriverId, id: InterruptId, target_cpu: u32) -> Result<()> {
    let _owner = driver::mutation_guard(owner)?;
    let target_cpu_usize = usize::try_from(target_cpu).map_err(|_| Error::InvalidArgument)?;
    let target_platform_id = smp::platform_id(target_cpu_usize).ok_or(Error::NotFound)?;
    if target_cpu != 0 && !smp::is_online(target_cpu_usize) {
        return Err(Error::Busy);
    }

    let snapshot = begin_route_update(owner, id)?;
    let route = snapshot.controller.route_descriptor(
        id,
        &snapshot.key,
        snapshot.vector,
        snapshot.flags,
        target_cpu,
        target_platform_id,
    );
    match snapshot.controller.set_affinity(snapshot.cookie, &route) {
        Ok(()) => {
            let mut state = registry()?.state.lock();
            let record = state
                .routes
                .get_mut(&id)
                .expect("dev/interrupt: affinity route disappeared");
            record.state = RouteState::Ready;
            Ok(())
        }
        Err(error) => {
            finish_route_update(id, snapshot.masked);
            Err(error)
        }
    }
}

/// Dispatches an x86 hardware vector through the lock-free vector map.
#[cfg(target_arch = "x86_64")]
pub(crate) fn dispatch_vector(vector: u8) -> DispatchOutcome {
    let raw = VECTOR_MAP[vector as usize].load(Ordering::Acquire);
    if raw == VECTOR_EMPTY {
        return DispatchOutcome::unhandled();
    }
    if raw == VECTOR_QUIESCED {
        return DispatchOutcome::handled();
    }
    let mut outcome =
        interrupt_slot(InterruptId::from_raw(raw)).dispatch(InterruptId::from_raw(raw), None);
    outcome.handled = true;
    outcome
}

/// Dispatches one architecture root external interrupt.
#[cfg(target_arch = "riscv64")]
pub(crate) fn dispatch_external(cpu: u32, platform_id: u64) -> DispatchOutcome {
    ROOT_CONTROLLER.dispatch(cpu, platform_id)
}

pub(crate) fn prepare_remove_driver(owner: DriverId) -> Result<()> {
    let registry = match REGISTRY.get() {
        Some(registry) => registry,
        None => return Ok(()),
    };
    let (routes, root, controllers) = {
        let mut state = registry.state.lock();
        if !state.pending_drivers.insert(owner) {
            return Err(Error::Busy);
        }

        let owned_controllers: Vec<_> = state
            .controllers
            .values()
            .filter(|controller| controller.owner == owner)
            .cloned()
            .collect();
        if owned_controllers.iter().any(|controller| {
            state
                .routes
                .values()
                .any(|route| route.controller.id == controller.id && route.owner != owner)
        }) {
            state.pending_drivers.remove(&owner);
            return Err(Error::Busy);
        }

        let route_ids: Vec<_> = state
            .routes
            .iter()
            .filter(|(_, route)| route.owner == owner || route.controller.owner == owner)
            .map(|(id, _)| *id)
            .collect();
        if route_ids.iter().any(|id| {
            state
                .routes
                .get(id)
                .is_none_or(|route| route.state != RouteState::Ready)
        }) {
            state.pending_drivers.remove(&owner);
            return Err(Error::Busy);
        }

        let mut routes = Vec::with_capacity(route_ids.len());
        for id in route_ids {
            let route = state
                .routes
                .get_mut(&id)
                .expect("dev/interrupt: removal route disappeared");
            let was_masked = route.masked;
            let route_snapshot = snapshot(id, route);
            route.masked = true;
            route.state = RouteState::Prepared { was_masked };
            routes.push(route_snapshot);
        }
        for controller in &owned_controllers {
            controller.available.store(false, Ordering::Release);
        }
        let root = state.root_controller.and_then(|id| {
            state
                .controllers
                .get(&id)
                .filter(|controller| controller.owner == owner)
                .map(|_| id)
        });
        (routes, root, owned_controllers)
    };

    let mut masked: Vec<RouteSnapshot> = Vec::new();
    for route in routes.iter().filter(|route| !route.masked) {
        if let Err(error) = route.controller.mask(route.cookie, false) {
            let mut still_masked = BTreeSet::new();
            for previous in masked.into_iter().rev() {
                if previous.controller.unmask(previous.cookie).is_err() {
                    still_masked.insert(previous.id);
                }
            }
            rollback_preparation(owner, &controllers, &still_masked);
            return Err(error);
        }
        masked.push(route.clone());
    }

    for route in &routes {
        interrupt_slot(route.id).suspend(route.id);
    }
    if let Some(id) = root {
        #[cfg(target_arch = "riscv64")]
        crate::arch::set_external_interrupts(false);
        ROOT_CONTROLLER.suspend(id);
    }
    Ok(())
}

pub(crate) fn restore_driver(owner: DriverId) -> Result<()> {
    let registry = match REGISTRY.get() {
        Some(registry) => registry,
        None => return Ok(()),
    };
    let (routes, root, controllers) = {
        let state = registry.state.lock();
        if !state.pending_drivers.contains(&owner) {
            return Ok(());
        }
        let routes: Vec<_> = state
            .routes
            .iter()
            .filter_map(|(id, route)| match route.state {
                RouteState::Prepared { .. }
                    if route.owner == owner || route.controller.owner == owner =>
                {
                    Some(snapshot(*id, route))
                }
                _ => None,
            })
            .collect();
        let controllers: Vec<_> = state
            .controllers
            .values()
            .filter(|controller| controller.owner == owner)
            .cloned()
            .collect();
        let root = state.root_controller.and_then(|id| {
            state
                .controllers
                .get(&id)
                .filter(|controller| controller.owner == owner)
                .map(|_| id)
        });
        (routes, root, controllers)
    };

    if let Some(id) = root {
        ROOT_CONTROLLER.resume(id);
        #[cfg(target_arch = "riscv64")]
        crate::arch::set_external_interrupts(true);
    }
    for route in &routes {
        interrupt_slot(route.id).resume(route.id);
    }

    let mut first_error = None;
    let mut unmasked = BTreeSet::new();
    for route in &routes {
        if !route.masked {
            match route.controller.unmask(route.cookie) {
                Ok(()) => {
                    unmasked.insert(route.id);
                }
                Err(error) => {
                    first_error.get_or_insert(error);
                }
            }
        }
    }

    {
        let mut state = registry.state.lock();
        for route in &routes {
            let record = state
                .routes
                .get_mut(&route.id)
                .expect("dev/interrupt: restored route disappeared");
            let was_masked = match record.state {
                RouteState::Prepared { was_masked } => was_masked,
                _ => panic!("dev/interrupt: restoring an unprepared route"),
            };
            record.masked = was_masked || !unmasked.contains(&route.id);
            record.state = RouteState::Ready;
        }
        for controller in controllers {
            controller.available.store(true, Ordering::Release);
        }
        state.pending_drivers.remove(&owner);
    }

    first_error.map_or(Ok(()), Err)
}

pub(crate) fn remove_driver(owner: DriverId) {
    let Some(registry) = REGISTRY.get() else {
        return;
    };
    let (routes, root, controllers) = {
        let state = registry.state.lock();
        let routes: Vec<_> = state
            .routes
            .iter()
            .filter_map(|(id, route)| match route.state {
                RouteState::Prepared { .. }
                    if route.owner == owner || route.controller.owner == owner =>
                {
                    Some(snapshot(*id, route))
                }
                _ => None,
            })
            .collect();
        let controllers: Vec<_> = state
            .controllers
            .values()
            .filter(|controller| controller.owner == owner)
            .map(|controller| controller.id)
            .collect();
        let root = state.root_controller.filter(|id| controllers.contains(id));
        (routes, root, controllers)
    };

    for route in routes {
        quiesce_vector(route.vector);
        if let Err(error) = route.controller.disconnect(route.cookie, true) {
            error!(
                "dev/interrupt: failed to disconnect interrupt {} for driver {}: {error}",
                route.id.get(),
                owner.get()
            );
        }
        interrupt_slot(route.id).remove_suspended();
        remove_route_record(route.id);
    }
    if root.is_some() {
        ROOT_CONTROLLER.remove_suspended();
    }

    let mut state = registry.state.lock();
    for id in controllers {
        state.controllers.remove(&id);
        if state.root_controller == Some(id) {
            state.root_controller = None;
        }
    }
    state.pending_drivers.remove(&owner);
}

pub(crate) fn cleanup_failed_load(owner: DriverId) {
    if let Err(error) = prepare_remove_driver(owner) {
        error!(
            "dev/interrupt: failed to quiesce objects for failed driver {}: {error}",
            owner.get()
        );
        return;
    }
    remove_driver(owner);
}

fn dispatch_root_controller(
    controller: &ControllerRecord,
    cpu: u32,
    platform_id: u64,
) -> DispatchOutcome {
    let Ok(_callback) = controller.callback_owner.acquire_irq() else {
        return DispatchOutcome::unhandled();
    };
    let Some(claim) = controller.operations.claim else {
        return DispatchOutcome::unhandled();
    };
    let complete = controller
        .operations
        .complete
        .expect("dev/interrupt: root controller missing completion callback");
    let mut outcome = DispatchOutcome::handled();

    for _ in 0..MAX_CLAIMS_PER_TRAP {
        let mut interrupt = 0u64;
        let mut cookie = 0u64;
        // SAFETY: root-controller registration validated the IRQ-safe callback
        // contract and both output pointers remain live for this call.
        let status = unsafe {
            claim(
                controller.operations.context,
                cpu,
                platform_id,
                &mut interrupt,
                &mut cookie,
            )
        };
        if status == STATUS_NOT_FOUND {
            break;
        }
        if status != STATUS_OK {
            break;
        }

        let id = InterruptId::from_raw(interrupt);
        if slot_index(id).is_some() {
            outcome.merge(interrupt_slot(id).dispatch(id, Some(controller.id)));
        }
        // SAFETY: the claim callback returned this interrupt and cookie, and
        // the same pinned controller callback owner spans completion.
        unsafe {
            complete(
                controller.operations.context,
                cpu,
                platform_id,
                interrupt,
                cookie,
            );
        }
    }

    outcome
}

fn begin_route_update(owner: DriverId, id: InterruptId) -> Result<RouteSnapshot> {
    let registry = registry()?;
    let mut state = registry.state.lock();
    ensure_driver_available(&state, owner)?;
    let controller_owner = state
        .routes
        .get(&id)
        .ok_or(Error::NotFound)?
        .controller
        .owner;
    ensure_driver_available(&state, controller_owner)?;
    let route = state.routes.get_mut(&id).ok_or(Error::NotFound)?;
    if route.owner != owner {
        return Err(Error::PermissionDenied);
    }
    if route.state != RouteState::Ready {
        return Err(Error::Busy);
    }
    route.state = RouteState::Updating;
    Ok(snapshot(id, route))
}

fn finish_route_update(id: InterruptId, masked: bool) {
    let mut state = registry()
        .expect("dev/interrupt: registry disappeared")
        .state
        .lock();
    let route = state
        .routes
        .get_mut(&id)
        .expect("dev/interrupt: updated route disappeared");
    route.masked = masked;
    route.state = RouteState::Ready;
}

fn snapshot(id: InterruptId, route: &RouteRecord) -> RouteSnapshot {
    let masked = match route.state {
        RouteState::Prepared { was_masked } => was_masked,
        _ => route.masked,
    };
    RouteSnapshot {
        id,
        controller: route.controller.clone(),
        key: route.key.clone(),
        flags: route.flags,
        vector: route.vector,
        cookie: route.cookie,
        masked,
    }
}

fn rollback_preparation(
    owner: DriverId,
    controllers: &[Arc<ControllerRecord>],
    still_masked: &BTreeSet<InterruptId>,
) {
    let mut state = registry()
        .expect("dev/interrupt: registry disappeared")
        .state
        .lock();
    for (id, route) in state
        .routes
        .iter_mut()
        .filter(|(_, route)| route.owner == owner || route.controller.owner == owner)
    {
        if let RouteState::Prepared { was_masked } = route.state {
            route.masked = was_masked || still_masked.contains(id);
            route.state = RouteState::Ready;
        }
    }
    for controller in controllers {
        controller.available.store(true, Ordering::Release);
    }
    state.pending_drivers.remove(&owner);
}

fn abandon_connecting_route(id: InterruptId) {
    let mut state = registry()
        .expect("dev/interrupt: registry disappeared")
        .state
        .lock();
    let route = state
        .routes
        .remove(&id)
        .expect("dev/interrupt: connecting route disappeared");
    state.route_keys.remove(&route.key);
    release_interrupt_id(&mut state, id);
}

fn rollback_connected_route(id: InterruptId, controller: &ControllerRecord, cookie: u64) {
    let vector = registry()
        .expect("dev/interrupt: registry disappeared")
        .state
        .lock()
        .routes
        .get(&id)
        .and_then(|route| route.vector);
    quiesce_vector(vector);
    interrupt_slot(id).suspend(id);
    if let Err(error) = controller.disconnect(cookie, false) {
        error!(
            "dev/interrupt: failed to roll back interrupt {}: {error}",
            id.get()
        );
    }
    interrupt_slot(id).remove_suspended();
    remove_route_record(id);
}

fn remove_route_record(id: InterruptId) {
    let mut state = registry()
        .expect("dev/interrupt: registry disappeared")
        .state
        .lock();
    let route = state
        .routes
        .remove(&id)
        .expect("dev/interrupt: removing an unknown route");
    state.route_keys.remove(&route.key);
    release_interrupt_id(&mut state, id);
}

fn rollback_controller_registration(id: InterruptControllerId) {
    let mut state = registry()
        .expect("dev/interrupt: registry disappeared")
        .state
        .lock();
    state.controllers.remove(&id);
    if state.root_controller == Some(id) {
        state.root_controller = None;
    }
}

fn resolve_domain(node: DeviceNodeId) -> Result<InterruptControllerId> {
    let data = super::resolve_resource(node, INTERRUPT_DOMAIN_RESOURCE)?.data()?;
    if data.len() != DOMAIN_RECORD_SIZE {
        return Err(Error::AbiMismatch);
    }
    let size = u32::from_le_bytes(data[..4].try_into().expect("interrupt domain size width"));
    let version = u32::from_le_bytes(
        data[4..8]
            .try_into()
            .expect("interrupt domain version width"),
    );
    if size as usize != DOMAIN_RECORD_SIZE || version != INTERRUPT_ABI_V1 {
        return Err(Error::AbiMismatch);
    }
    let id = u64::from_le_bytes(
        data[8..16]
            .try_into()
            .expect("interrupt domain identifier width"),
    );
    if id == 0 {
        return Err(Error::InvalidArgument);
    }
    Ok(InterruptControllerId::from_raw(id))
}

fn domain_record(id: InterruptControllerId) -> Arc<[u8]> {
    let mut bytes = [0u8; DOMAIN_RECORD_SIZE];
    bytes[..4].copy_from_slice(&(DOMAIN_RECORD_SIZE as u32).to_le_bytes());
    bytes[4..8].copy_from_slice(&INTERRUPT_ABI_V1.to_le_bytes());
    bytes[8..].copy_from_slice(&id.get().to_le_bytes());
    Arc::from(bytes)
}

fn validate_controller(operations: &InterruptControllerV1) -> Result<()> {
    if operations.abi_version != INTERRUPT_ABI_V1
        || (operations.size as usize) < mem::size_of::<InterruptControllerV1>()
        || operations.flags & !CONTROLLER_FLAGS != 0
        || operations.connect.is_none()
        || operations.disconnect.is_none()
        || operations.mask.is_none()
        || operations.unmask.is_none()
    {
        return Err(Error::AbiMismatch);
    }
    let root = operations.flags & CONTROLLER_ROOT != 0;
    if operations.claim.is_some() != operations.complete.is_some()
        || root != operations.claim.is_some()
    {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

fn validate_route_flags(flags: u64) -> Result<()> {
    if flags & !ROUTE_FLAGS != 0
        || flags & ROUTE_EDGE != 0 && flags & ROUTE_LEVEL != 0
        || flags & ROUTE_ACTIVE_HIGH != 0 && flags & ROUTE_ACTIVE_LOW != 0
    {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

fn callback_result(status: i32) -> Result<()> {
    if status == STATUS_OK {
        Ok(())
    } else {
        Err(Error::CallbackFailed(status))
    }
}

fn registry() -> Result<&'static InterruptRegistry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

fn ensure_driver_available(state: &RegistryState, owner: DriverId) -> Result<()> {
    if state.pending_drivers.contains(&owner) {
        Err(Error::Busy)
    } else {
        Ok(())
    }
}

fn allocate_interrupt_id(state: &mut RegistryState) -> Result<InterruptId> {
    let Some(index) = state.used_slots.iter().position(|used| !*used) else {
        return Err(Error::NoSpace);
    };
    state.used_slots[index] = true;
    let generation =
        state.slot_generations[index].wrapping_add(1).max(1) & (u64::MAX >> INTERRUPT_SLOT_BITS);
    let generation = generation.max(1);
    state.slot_generations[index] = generation;
    Ok(InterruptId(
        (generation << INTERRUPT_SLOT_BITS) | (index as u64 + 1),
    ))
}

fn release_interrupt_id(state: &mut RegistryState, id: InterruptId) {
    let index = slot_index(id).expect("dev/interrupt: invalid allocated interrupt ID");
    assert!(
        state.used_slots[index],
        "dev/interrupt: interrupt slot double free"
    );
    state.used_slots[index] = false;
}

fn slot_index(id: InterruptId) -> Option<usize> {
    let raw = id.get();
    if raw == 0 || raw == u64::MAX {
        return None;
    }
    let encoded = raw & INTERRUPT_SLOT_MASK;
    if encoded == 0 || encoded > MAX_INTERRUPT_SLOTS as u64 || raw >> INTERRUPT_SLOT_BITS == 0 {
        return None;
    }
    Some(encoded as usize - 1)
}

fn interrupt_slot(id: InterruptId) -> &'static InterruptSlot {
    &INTERRUPT_SLOTS[slot_index(id).expect("dev/interrupt: invalid interrupt ID")]
}

#[cfg(target_arch = "x86_64")]
fn allocate_vector(state: &mut RegistryState) -> Result<u8> {
    if state.next_vector > LAST_EXTERNAL_VECTOR {
        return Err(Error::NoSpace);
    }
    let vector = state.next_vector as u8;
    state.next_vector += 1;
    Ok(vector)
}

#[cfg(target_arch = "x86_64")]
fn quiesce_vector(vector: Option<u8>) {
    if let Some(vector) = vector {
        VECTOR_MAP[vector as usize].store(VECTOR_QUIESCED, Ordering::Release);
    }
}

#[cfg(target_arch = "riscv64")]
fn quiesce_vector(_vector: Option<u8>) {}

#[cfg(target_arch = "x86_64")]
fn restore_vector(vector: Option<u8>, id: InterruptId) {
    if let Some(vector) = vector {
        VECTOR_MAP[vector as usize].store(id.get(), Ordering::Release);
    }
}

#[cfg(target_arch = "riscv64")]
fn restore_vector(_vector: Option<u8>, _id: InterruptId) {}
