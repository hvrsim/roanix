//! Interrupt routing and delivery.
//!
//! Interrupts are described by two layers. A *domain* is a controller's view of
//! its own hardware interrupt numbers, and a *virtual interrupt* is the dense
//! system-wide number the framework hands to drivers. Domains are registered by
//! ordinary driver modules, so an I/O APIC, a PLIC, or a PCI message-signalled
//! block is added without changing the framework.
//!
//! Delivery is designed for a short path. The architecture trap handler
//! resolves a virtual interrupt with one array index, and the descriptor's
//! handler list is guarded by an interrupt-safe spin lock that is essentially
//! never contended. There is no map lookup and no global lock on the delivery
//! path.
//!
//! A handler that must do lengthy work returns [`outcome::WAKE_THREAD`] and the
//! rest runs on a dedicated thread, which is what a storage or host-controller
//! driver needs in order to complete requests without blocking delivery. On a
//! level-triggered line the top half must also mask (or otherwise quiet) the
//! device before returning: until the thread services it, each re-delivery
//! re-runs every handler and re-raises the wake.

use alloc::{
    boxed::Box,
    string::{String, ToString},
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    ffi::c_void,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, Ordering},
};

use log::error;

use crate::sys::{
    event::Event,
    sched,
    smp::IrqSpinLock,
    sync::{Mutex, Once},
};

use super::{
    core::{device::Device, module::Module},
    error::{self, Error, Result},
    obj::{ObjHeader, ObjKind, framework_object},
};

/// Largest number of virtual interrupts the system can route.
pub const MAX_VIRQ: usize = 1024;
/// Largest hardware interrupt space one domain may declare.
pub const MAX_HWIRQ: u32 = 4096;

/// First architecture vector the framework may allocate on x86.
const FIRST_VECTOR: usize = 0x30;
/// One past the last allocatable architecture vector on x86.
const LAST_VECTOR: usize = 0xE0;
/// Number of architecture vectors tracked for dispatch.
const VECTOR_COUNT: usize = 256;

/// Values a handler may return.
pub mod outcome {
    /// The interrupt did not belong to this handler.
    pub const NONE: u32 = 0;
    /// The handler serviced the interrupt.
    pub const HANDLED: u32 = 1 << 0;
    /// The handler serviced the interrupt and needs its thread to run.
    pub const WAKE_THREAD: u32 = 1 << 1;
    /// The handler made a thread runnable and asks for a reschedule.
    pub const RESCHEDULE: u32 = 1 << 2;
}

/// Interrupt trigger and polarity flags.
pub mod trigger {
    /// Edge triggered.
    pub const EDGE: u32 = 1 << 0;
    /// Level triggered.
    pub const LEVEL: u32 = 1 << 1;
    /// Asserted high.
    pub const ACTIVE_HIGH: u32 = 1 << 2;
    /// Asserted low.
    pub const ACTIVE_LOW: u32 = 1 << 3;
    /// The line may be shared with other devices.
    pub const SHARED: u32 = 1 << 4;
    /// Leave the line masked after it is requested.
    pub const START_MASKED: u32 = 1 << 5;
}

/// Domain capability flags.
pub mod domain_flags {
    /// The domain is the architecture's root controller and must be consulted
    /// to identify an incoming interrupt.
    pub const ROOT: u32 = 1 << 0;
    /// The domain issues message-signalled interrupts.
    pub const MESSAGE: u32 = 1 << 1;
    /// The domain decodes interrupt specifiers for devices that do not name a
    /// controller of their own.
    ///
    /// Firmware on most platforms describes a device's interrupt as a bare
    /// number relative to the system controller, so one domain has to claim
    /// that role.
    pub const DEFAULT: u32 = 1 << 2;
}

/// Top-half handler.
pub type HandlerFn = unsafe extern "C" fn(context: *mut c_void, virq: u32) -> u32;
/// Threaded bottom-half handler.
pub type ThreadFn = unsafe extern "C" fn(context: *mut c_void, virq: u32);

/// Callbacks an interrupt controller implements.
#[derive(Default)]
pub struct DomainOps {
    /// Decodes firmware specifier cells into a hardware interrupt number.
    pub translate: Option<
        unsafe extern "C" fn(
            context: *mut c_void,
            cells: *const u32,
            count: usize,
            out_hwirq: *mut u64,
            out_flags: *mut u32,
        ) -> i32,
    >,
    /// Programs routing for a newly mapped interrupt.
    pub setup: Option<
        unsafe extern "C" fn(context: *mut c_void, hwirq: u64, virq: u32, flags: u32) -> i32,
    >,
    /// Releases routing for an unmapped interrupt.
    pub teardown: Option<unsafe extern "C" fn(context: *mut c_void, hwirq: u64, virq: u32)>,
    /// Masks a line.
    pub mask: Option<unsafe extern "C" fn(context: *mut c_void, hwirq: u64)>,
    /// Unmasks a line.
    pub unmask: Option<unsafe extern "C" fn(context: *mut c_void, hwirq: u64)>,
    /// Signals end of interrupt.
    pub eoi: Option<unsafe extern "C" fn(context: *mut c_void, hwirq: u64)>,
    /// Redirects a line to another CPU.
    pub set_affinity:
        Option<unsafe extern "C" fn(context: *mut c_void, hwirq: u64, cpu: u32) -> i32>,
    /// Identifies the interrupt currently pending on this CPU.
    pub claim: Option<
        unsafe extern "C" fn(
            context: *mut c_void,
            cpu: u32,
            platform_id: u64,
            out_hwirq: *mut u64,
        ) -> i32,
    >,
    /// Completes an interrupt previously reported by `claim`.
    pub complete:
        Option<unsafe extern "C" fn(context: *mut c_void, cpu: u32, platform_id: u64, hwirq: u64)>,
    /// Reports the address and payload a device must write to raise `hwirq`.
    pub compose_message: Option<
        unsafe extern "C" fn(
            context: *mut c_void,
            hwirq: u64,
            out_address: *mut u64,
            out_data: *mut u32,
        ) -> i32,
    >,
    /// Context passed to every callback.
    pub context: *mut c_void,
}

/// An interrupt controller's hardware interrupt space.
#[repr(C)]
pub struct IrqDomain {
    header: ObjHeader,
    name: Box<str>,
    owner: Option<Arc<Module>>,
    flags: u32,
    ops: DomainOps,
    hwirq_count: u32,
    map: Box<[AtomicU32]>,
}

framework_object!(IrqDomain, IrqDomain);

// SAFETY: domain callbacks and context stay valid until the domain is
// unregistered, which requires every mapping it owns to be gone first.
unsafe impl Send for IrqDomain {}
// SAFETY: the callback table is immutable after registration and the ABI
// requires the callbacks to tolerate concurrent invocation.
unsafe impl Sync for IrqDomain {}

impl IrqDomain {
    /// Returns the domain name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the module that registered this domain.
    pub fn owner(&self) -> Option<&Arc<Module>> {
        self.owner.as_ref()
    }

    /// Returns the number of hardware interrupts this domain describes.
    pub const fn hwirq_count(&self) -> u32 {
        self.hwirq_count
    }

    /// Returns whether this domain is the architecture's root controller.
    pub const fn is_root(&self) -> bool {
        self.flags & domain_flags::ROOT != 0
    }

    fn lookup(&self, hwirq: u64) -> Option<u32> {
        let index = usize::try_from(hwirq).ok()?;
        let slot = self.map.get(index)?;
        match slot.load(Ordering::Acquire) {
            0 => None,
            virq => Some(virq),
        }
    }

    fn translate(&self, cells: &[u32]) -> Result<(u64, u32)> {
        let Some(translate) = self.ops.translate else {
            let hwirq = cells.first().copied().unwrap_or(0);
            return Ok((u64::from(hwirq), 0));
        };
        let mut hwirq = 0u64;
        let mut flags = 0u32;
        // SAFETY: registration validated the callback, and the output pointers
        // address local storage that outlives the call.
        let status = unsafe {
            translate(
                self.ops.context,
                cells.as_ptr(),
                cells.len(),
                &raw mut hwirq,
                &raw mut flags,
            )
        };
        error::from_status(status)?;
        Ok((hwirq, flags))
    }

    fn mask(&self, hwirq: u64) {
        if let Some(mask) = self.ops.mask {
            // SAFETY: registration validated the callback.
            unsafe { mask(self.ops.context, hwirq) };
        }
    }

    fn unmask(&self, hwirq: u64) {
        if let Some(unmask) = self.ops.unmask {
            // SAFETY: registration validated the callback.
            unsafe { unmask(self.ops.context, hwirq) };
        }
    }

    fn eoi(&self, hwirq: u64) {
        if let Some(eoi) = self.ops.eoi {
            // SAFETY: registration validated the callback.
            unsafe { eoi(self.ops.context, hwirq) };
        }
    }

    /// Returns the message address and payload that raise `hwirq`.
    pub fn compose_message(&self, hwirq: u64) -> Result<(u64, u32)> {
        let compose = self.ops.compose_message.ok_or(Error::Unsupported)?;
        let mut address = 0u64;
        let mut data = 0u32;
        // SAFETY: registration validated the callback, and the output pointers
        // address local storage that outlives the call.
        let status = unsafe { compose(self.ops.context, hwirq, &raw mut address, &raw mut data) };
        error::from_status(status)?;
        Ok((address, data))
    }
}

/// One registered handler on a virtual interrupt.
pub struct IrqAction {
    name: Box<str>,
    virq: u32,
    handler: HandlerFn,
    thread: Option<ThreadFn>,
    context: *mut c_void,
    owner: Option<Arc<Module>>,
    device: Option<Weak<Device>>,
    wake: Event,
    finished: Event,
    stopping: AtomicBool,
    threaded: AtomicBool,
    calls: AtomicU64,
}

// SAFETY: the handler, thread callback, and context outlive the action, which
// is released before its owning module is unloaded.
unsafe impl Send for IrqAction {}
// SAFETY: the callbacks are invoked from interrupt context on any CPU and the
// ABI requires them to tolerate that.
unsafe impl Sync for IrqAction {}

impl IrqAction {
    /// Returns the virtual interrupt this handler is attached to.
    pub const fn virq(&self) -> u32 {
        self.virq
    }

    /// Returns the number of times this handler ran.
    pub fn calls(&self) -> u64 {
        self.calls.load(Ordering::Relaxed)
    }

    /// Returns the handler name.
    pub fn name(&self) -> &str {
        &self.name
    }
}

struct DescBinding {
    domain: Option<Arc<IrqDomain>>,
    hwirq: u64,
    flags: u32,
    masked: bool,
    /// Immutable handler list. Edits publish a whole new slice, so dispatch can
    /// take a counted reference to it and release the lock before running any
    /// driver code.
    actions: Arc<[Arc<IrqAction>]>,
}

struct IrqDesc {
    virq: u32,
    inner: IrqSpinLock<DescBinding>,
    /// Number of dispatches currently executing handlers from this descriptor.
    ///
    /// Detaching a handler waits for this to drain, which is what guarantees no
    /// driver code is still running when a module is unloaded.
    in_flight: AtomicU64,
    count: AtomicU64,
    spurious: AtomicU64,
}

struct Registry {
    descs: Mutex<Vec<Arc<IrqDesc>>>,
    domains: Mutex<Vec<Arc<IrqDomain>>>,
    vectors: Mutex<[bool; VECTOR_COUNT]>,
    /// Serialises handler-list edits.
    ///
    /// The list itself is guarded by an interrupt-safe lock so delivery can
    /// read it, but a replacement list has to be built with allocation
    /// allowed, which means outside that lock. This mutex makes the
    /// read-modify-write atomic with respect to other registrations.
    registration: Mutex<()>,
}

/// Lock-free descriptor table indexed by virtual interrupt number.
static DESCS: [AtomicPtr<IrqDesc>; MAX_VIRQ] =
    [const { AtomicPtr::new(core::ptr::null_mut()) }; MAX_VIRQ];
/// Lock-free architecture vector table used by the x86 delivery path.
static VECTOR_MAP: [AtomicU32; VECTOR_COUNT] = [const { AtomicU32::new(0) }; VECTOR_COUNT];
/// Root controller consulted by architectures that report a generic external
/// interrupt.
static ROOT_DOMAIN: AtomicPtr<IrqDomain> = AtomicPtr::new(core::ptr::null_mut());
/// Controller that decodes specifiers for devices without an explicit domain.
static DEFAULT_DOMAIN: AtomicPtr<IrqDomain> = AtomicPtr::new(core::ptr::null_mut());
static NEXT_VIRQ: AtomicU32 = AtomicU32::new(1);
static REGISTRY: Once<Registry> = Once::new();

pub(crate) fn init() {
    REGISTRY.call_once(|| Registry {
        descs: Mutex::new(Vec::new()),
        domains: Mutex::new(Vec::new()),
        vectors: Mutex::new([false; VECTOR_COUNT]),
        registration: Mutex::new(()),
    });
}

fn registry() -> Result<&'static Registry> {
    REGISTRY.get().ok_or(Error::NotInitialized)
}

fn desc(virq: u32) -> Option<&'static IrqDesc> {
    let slot = DESCS.get(virq as usize)?;
    let pointer = slot.load(Ordering::Acquire);
    if pointer.is_null() {
        return None;
    }
    // SAFETY: descriptors are allocated once, published here, and never freed,
    // so a non-null slot always addresses a live descriptor.
    Some(unsafe { &*pointer })
}

fn allocate_desc() -> Result<&'static IrqDesc> {
    let registry = registry()?;
    let virq = NEXT_VIRQ.fetch_add(1, Ordering::Relaxed);
    if virq as usize >= MAX_VIRQ {
        return Err(Error::NoSpace);
    }
    let record = Arc::new(IrqDesc {
        virq,
        inner: IrqSpinLock::new(DescBinding {
            domain: None,
            hwirq: 0,
            flags: 0,
            masked: true,
            actions: Arc::from([]),
        }),
        in_flight: AtomicU64::new(0),
        count: AtomicU64::new(0),
        spurious: AtomicU64::new(0),
    });
    // Descriptors are immortal by design: the delivery path reads them with a
    // single atomic load and no lifetime bookkeeping. A failed mapping attempt
    // therefore consumes one virtual interrupt number even when nothing binds
    // to it - failure paths clear the domain map so nothing stale is reachable,
    // and the 1024-number space is sized to make that acceptable. Reclaiming
    // numbers would need generation-tagged handles; revisit only if a system
    // ever exhausts the space.
    let pointer = Arc::into_raw(record.clone()).cast_mut();
    registry.descs.lock().push(record);
    DESCS[virq as usize].store(pointer, Ordering::Release);
    // SAFETY: the pointer was just published from a leaked reference.
    Ok(unsafe { &*pointer })
}

/// Registers an interrupt controller domain.
///
/// # Safety
///
/// Every callback in `ops` must follow the domain ABI and stay executable until
/// the domain is unregistered.
pub unsafe fn register_domain(
    owner: Option<&Arc<Module>>,
    name: &str,
    flags: u32,
    hwirq_count: u32,
    ops: DomainOps,
) -> Result<Arc<IrqDomain>> {
    if name.is_empty() || hwirq_count == 0 || hwirq_count > MAX_HWIRQ {
        return Err(Error::InvalidArgument);
    }
    if flags & domain_flags::ROOT != 0 && ops.claim.is_none() {
        return Err(Error::InvalidArgument);
    }
    let registry = registry()?;
    let mut domains = registry.domains.lock();
    if domains.iter().any(|domain| &*domain.name == name) {
        return Err(Error::AlreadyExists);
    }

    let mut map = Vec::new();
    map.resize_with(hwirq_count as usize, || AtomicU32::new(0));
    let header = ObjHeader::new_with(
        ObjKind::IrqDomain,
        Some(name),
        owner.map(|module| module.id().get()),
        None,
    );
    let domain = Arc::new(IrqDomain {
        header,
        name: String::from(name).into_boxed_str(),
        owner: owner.cloned(),
        flags,
        ops,
        hwirq_count,
        map: map.into_boxed_slice(),
    });
    domain.header.register()?;
    domains.push(domain.clone());
    drop(domains);

    if domain.is_root() && ROOT_DOMAIN.load(Ordering::Acquire).is_null() {
        ROOT_DOMAIN.store(Arc::as_ptr(&domain).cast_mut(), Ordering::Release);
        #[cfg(target_arch = "riscv64")]
        crate::arch::set_external_interrupts(true);
    }
    if flags & domain_flags::DEFAULT != 0 && DEFAULT_DOMAIN.load(Ordering::Acquire).is_null() {
        DEFAULT_DOMAIN.store(Arc::as_ptr(&domain).cast_mut(), Ordering::Release);
    }
    super::core::probe::retrigger();
    Ok(domain)
}

/// Unregisters a domain once none of its interrupts are in use.
pub fn unregister_domain(domain: &Arc<IrqDomain>) -> Result<()> {
    for index in 0..domain.hwirq_count as usize {
        if domain.map[index].load(Ordering::Acquire) != 0 {
            return Err(Error::Busy);
        }
    }
    force_unregister_domain(domain);
    Ok(())
}

fn force_unregister_domain(domain: &Arc<IrqDomain>) {
    domain.header.set_state(super::obj::ObjState::Removing);
    for index in 0..domain.hwirq_count as usize {
        let virq = domain.map[index].swap(0, Ordering::AcqRel);
        if virq != 0 {
            unbind_desc(virq);
        }
    }
    if ROOT_DOMAIN.load(Ordering::Acquire) == Arc::as_ptr(domain).cast_mut() {
        ROOT_DOMAIN.store(core::ptr::null_mut(), Ordering::Release);
        #[cfg(target_arch = "riscv64")]
        crate::arch::set_external_interrupts(false);
    }
    if DEFAULT_DOMAIN.load(Ordering::Acquire) == Arc::as_ptr(domain).cast_mut() {
        DEFAULT_DOMAIN.store(core::ptr::null_mut(), Ordering::Release);
    }
    if let Ok(registry) = registry() {
        registry
            .domains
            .lock()
            .retain(|entry| !Arc::ptr_eq(entry, domain));
    }
    domain.header.poison();
}

fn unbind_desc(virq: u32) {
    let Some(record) = desc(virq) else {
        return;
    };
    let previous = {
        let mut inner = record.inner.lock();
        inner.domain = None;
        inner.hwirq = 0;
        inner.masked = true;
        core::mem::replace(&mut inner.actions, Arc::from([]))
    };
    drop(previous);
}

/// Maps a hardware interrupt into the virtual interrupt space.
pub fn create_mapping(domain: &Arc<IrqDomain>, hwirq: u64, flags: u32) -> Result<u32> {
    let index = usize::try_from(hwirq).map_err(|_| Error::InvalidArgument)?;
    if index >= domain.hwirq_count as usize {
        return Err(Error::InvalidArgument);
    }
    let _registration = registry()?.registration.lock();
    if let Some(existing) = domain.lookup(hwirq) {
        return Ok(existing);
    }

    let record = allocate_desc()?;
    if domain.map[index]
        .compare_exchange(0, record.virq, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return domain.lookup(hwirq).ok_or(Error::Busy);
    }

    {
        let mut inner = record.inner.lock();
        inner.domain = Some(domain.clone());
        inner.hwirq = hwirq;
        inner.flags = flags;
        inner.masked = true;
    }

    if let Some(setup) = domain.ops.setup {
        // SAFETY: registration validated the callback and the mapping is
        // published so the controller may reference it immediately.
        let status = unsafe { setup(domain.ops.context, hwirq, record.virq, flags) };
        if let Err(error) = error::from_status(status) {
            domain.map[index].store(0, Ordering::Release);
            unbind_desc(record.virq);
            return Err(error);
        }
    }
    Ok(record.virq)
}

/// Removes a hardware interrupt mapping after every handler has been released.
pub fn destroy_mapping(domain: &Arc<IrqDomain>, hwirq: u64) -> Result<()> {
    let index = usize::try_from(hwirq).map_err(|_| Error::InvalidArgument)?;
    if index >= domain.hwirq_count as usize {
        return Err(Error::InvalidArgument);
    }
    let virq = domain.map[index].load(Ordering::Acquire);
    if virq == 0 {
        return Err(Error::NotFound);
    }
    let record = desc(virq).ok_or(Error::NotFound)?;
    // Serializes against both a new mapping and a handler attachment.
    let _registration = registry()?.registration.lock();
    {
        let inner = record.inner.lock();
        if !inner.actions.is_empty()
            || inner
                .domain
                .as_ref()
                .is_none_or(|mapped| !Arc::ptr_eq(mapped, domain))
            || inner.hwirq != hwirq
        {
            return Err(Error::Busy);
        }
    }
    domain.mask(hwirq);
    while record.in_flight.load(Ordering::Acquire) != 0 {
        core::hint::spin_loop();
    }
    if let Some(teardown) = domain.ops.teardown {
        // SAFETY: registration keeps the callback executable until every
        // mapping is removed and no dispatch remains in flight here.
        unsafe { teardown(domain.ops.context, hwirq, virq) };
    }
    domain.map[index]
        .compare_exchange(virq, 0, Ordering::AcqRel, Ordering::Acquire)
        .map_err(|_| Error::Busy)?;
    unbind_desc(virq);
    Ok(())
}

/// Resolves the `index`-th firmware interrupt of `device` to a virtual
/// interrupt.
pub fn device_virq(device: &Arc<Device>, index: usize) -> Result<u32> {
    let entry = device.irqs().get(index).ok_or(Error::NotFound)?;
    if let Some(virq) = entry.virq() {
        return Ok(virq);
    }
    let domain = entry
        .domain()
        .or_else(inherited_domain)
        .ok_or(Error::Deferred)?;
    let (hwirq, flags) = domain.translate(entry.cells())?;
    let virq = create_mapping(&domain, hwirq, flags)?;
    entry.set_domain(domain);
    entry.set_virq(virq);
    Ok(virq)
}

/// Returns the domain that decodes specifiers for a device with no explicit
/// controller.
fn inherited_domain() -> Option<Arc<IrqDomain>> {
    let mut pointer = DEFAULT_DOMAIN.load(Ordering::Acquire);
    if pointer.is_null() {
        pointer = ROOT_DOMAIN.load(Ordering::Acquire);
    }
    if pointer.is_null() {
        return None;
    }
    // SAFETY: the root pointer is only published while the domain is
    // registered, and unregistration clears it before dropping the domain.
    let domain = unsafe { &*pointer };
    registry().ok().and_then(|registry| {
        registry
            .domains
            .lock()
            .iter()
            .find(|entry| core::ptr::eq(Arc::as_ptr(entry), domain))
            .cloned()
    })
}

/// Attaches a handler to a virtual interrupt.
///
/// # Safety
///
/// `handler` and `thread` must follow the interrupt ABI and stay executable
/// until the action is released.
#[allow(clippy::too_many_arguments)]
pub unsafe fn request(
    owner: Option<&Arc<Module>>,
    device: Option<&Arc<Device>>,
    virq: u32,
    name: &str,
    flags: u32,
    handler: HandlerFn,
    thread: Option<ThreadFn>,
    context: *mut c_void,
) -> Result<Arc<IrqAction>> {
    let record = desc(virq).ok_or(Error::NotFound)?;
    let _registration = registry()?.registration.lock();
    let action = Arc::new(IrqAction {
        name: String::from(name).into_boxed_str(),
        virq,
        handler,
        thread,
        context,
        owner: owner.cloned(),
        device: device.map(Arc::downgrade),
        wake: Event::new(),
        finished: Event::new(),
        stopping: AtomicBool::new(false),
        threaded: AtomicBool::new(false),
        calls: AtomicU64::new(0),
    });

    // Build the replacement list outside the interrupt-safe lock: allocation
    // must not run with interrupts masked.
    let (domain, hwirq, unmask) = {
        let snapshot = {
            let inner = record.inner.lock();
            if !inner.actions.is_empty()
                && (inner.flags & trigger::SHARED == 0 || flags & trigger::SHARED == 0)
            {
                return Err(Error::Busy);
            }
            inner.actions.clone()
        };
        let mut replacement = Vec::with_capacity(snapshot.len() + 1);
        replacement.extend(snapshot.iter().cloned());
        replacement.push(action.clone());
        let replacement: Arc<[Arc<IrqAction>]> = Arc::from(replacement);

        let mut inner = record.inner.lock();
        let previous = core::mem::replace(&mut inner.actions, replacement);
        let should_unmask = inner.masked && flags & trigger::START_MASKED == 0;
        if should_unmask {
            inner.masked = false;
        }
        let domain = inner.domain.clone();
        let hwirq = inner.hwirq;
        drop(inner);
        drop(previous);
        (domain, hwirq, should_unmask)
    };

    if thread.is_some() {
        start_thread(&action)?;
    }
    if unmask && let Some(domain) = domain.as_ref() {
        domain.unmask(hwirq);
    }
    Ok(action)
}

fn start_thread(action: &Arc<IrqAction>) -> Result<()> {
    let worker = action.clone();
    action.threaded.store(true, Ordering::Release);
    sched::create_ithread(
        move |_| {
            loop {
                worker.wake.wait();
                worker.wake.reset();
                if worker.stopping.load(Ordering::Acquire) {
                    break;
                }
                if let Some(thread) = worker.thread {
                    // SAFETY: the action holds the callback alive, and the
                    // owning module cannot unload until this thread exits.
                    unsafe { thread(worker.context, worker.virq) };
                }
            }
            worker.finished.signal();
        },
        0,
    );
    Ok(())
}

/// Detaches a handler.
///
/// The handler must not be released from its own threaded worker: the final
/// wait observes that thread's exit and would deadlock against itself.
pub fn release(action: &Arc<IrqAction>) -> Result<()> {
    let record = desc(action.virq).ok_or(Error::NotFound)?;
    let (domain, hwirq, mask) = {
        // Held only across the list read-modify-write it exists to serialize.
        // The drain loop below can spin for as long as a dispatch is running,
        // and holding the global mutex through it stalled every unrelated
        // request and release in the system.
        let _registration = registry()?.registration.lock();
        let snapshot = {
            let inner = record.inner.lock();
            inner.actions.clone()
        };
        let replacement: Arc<[Arc<IrqAction>]> = snapshot
            .iter()
            .filter(|entry| !Arc::ptr_eq(entry, action))
            .cloned()
            .collect();
        let empty = replacement.is_empty();

        let mut inner = record.inner.lock();
        let previous = core::mem::replace(&mut inner.actions, replacement);
        if empty && !inner.masked {
            inner.masked = true;
        }
        let domain = inner.domain.clone();
        let hwirq = inner.hwirq;
        drop(inner);
        drop(previous);
        (domain, hwirq, empty)
    };

    if mask && let Some(domain) = domain.as_ref() {
        domain.mask(hwirq);
    }

    // The handler is no longer reachable from the published list, but a
    // dispatch that started before the swap may still be inside it. Wait for
    // those to finish so the caller may free the context or unload the module.
    while record.in_flight.load(Ordering::Acquire) != 0 {
        core::hint::spin_loop();
    }

    if action.threaded.load(Ordering::Acquire) {
        // The worker thread executes module code, so it must be observed to
        // exit before the module may be unloaded.
        action.stopping.store(true, Ordering::Release);
        action.wake.signal();
        action.finished.wait();
    }
    Ok(())
}

/// Masks a virtual interrupt.
pub fn mask(virq: u32) -> Result<()> {
    let record = desc(virq).ok_or(Error::NotFound)?;
    let (domain, hwirq) = {
        let mut inner = record.inner.lock();
        if inner.masked {
            return Ok(());
        }
        inner.masked = true;
        (inner.domain.clone(), inner.hwirq)
    };
    if let Some(domain) = domain {
        domain.mask(hwirq);
    }
    Ok(())
}

/// Unmasks a virtual interrupt.
pub fn unmask(virq: u32) -> Result<()> {
    let record = desc(virq).ok_or(Error::NotFound)?;
    let (domain, hwirq) = {
        let mut inner = record.inner.lock();
        if !inner.masked {
            return Ok(());
        }
        inner.masked = false;
        (inner.domain.clone(), inner.hwirq)
    };
    if let Some(domain) = domain {
        domain.unmask(hwirq);
    }
    Ok(())
}

/// Redirects a virtual interrupt to `cpu`.
pub fn set_affinity(virq: u32, cpu: u32) -> Result<()> {
    let record = desc(virq).ok_or(Error::NotFound)?;
    let (domain, hwirq) = {
        let inner = record.inner.lock();
        (inner.domain.clone(), inner.hwirq)
    };
    let domain = domain.ok_or(Error::NotFound)?;
    let set_affinity = domain.ops.set_affinity.ok_or(Error::Unsupported)?;
    // SAFETY: registration validated the callback.
    error::from_status(unsafe { set_affinity(domain.ops.context, hwirq, cpu) })
}

/// Reserves an architecture interrupt vector for a controller.
///
/// Only x86 has a vector space; other architectures report
/// [`Error::Unsupported`].
pub fn allocate_vector(virq: u32) -> Result<u32> {
    if !cfg!(target_arch = "x86_64") {
        return Err(Error::Unsupported);
    }
    if desc(virq).is_none() {
        return Err(Error::NotFound);
    }
    let registry = registry()?;
    let mut vectors = registry.vectors.lock();
    for vector in FIRST_VECTOR..LAST_VECTOR {
        if vectors[vector] {
            continue;
        }
        vectors[vector] = true;
        VECTOR_MAP[vector].store(virq, Ordering::Release);
        return Ok(vector as u32);
    }
    Err(Error::NoSpace)
}

/// Releases a vector previously reserved by [`allocate_vector`].
pub fn free_vector(vector: u32) -> Result<()> {
    let index = vector as usize;
    if !(FIRST_VECTOR..LAST_VECTOR).contains(&index) {
        return Err(Error::InvalidArgument);
    }
    let registry = registry()?;
    VECTOR_MAP[index].store(0, Ordering::Release);
    registry.vectors.lock()[index] = false;
    Ok(())
}

/// Result of one architecture interrupt dispatch.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct DispatchOutcome {
    /// The delivery belonged to the interrupt subsystem.
    pub handled: bool,
    /// A handler asked for a reschedule on trap return.
    pub reschedule: bool,
}

/// Dispatches an architecture vector. This is the x86 delivery path.
#[cfg(target_arch = "x86_64")]
pub(crate) fn dispatch_vector(vector: u8) -> DispatchOutcome {
    let virq = VECTOR_MAP[vector as usize].load(Ordering::Acquire);
    if virq == 0 {
        return DispatchOutcome::default();
    }
    run(virq)
}

/// Dispatches a generic external interrupt. This is the RISC-V delivery path.
#[cfg(target_arch = "riscv64")]
pub(crate) fn dispatch_external(cpu: u32, platform_id: u64) -> DispatchOutcome {
    let pointer = ROOT_DOMAIN.load(Ordering::Acquire);
    if pointer.is_null() {
        return DispatchOutcome::default();
    }
    // SAFETY: the root pointer is published only while its domain is
    // registered and is cleared before the domain is dropped.
    let domain = unsafe { &*pointer };
    let Some(claim) = domain.ops.claim else {
        return DispatchOutcome::default();
    };

    let mut outcome = DispatchOutcome::default();
    loop {
        let mut hwirq = 0u64;
        // SAFETY: registration validated the callback and the output pointer
        // addresses local storage.
        let status = unsafe { claim(domain.ops.context, cpu, platform_id, &raw mut hwirq) };
        if status <= 0 {
            break;
        }
        outcome.handled = true;
        if let Some(virq) = domain.lookup(hwirq) {
            let result = run(virq);
            outcome.reschedule |= result.reschedule;
        } else {
            // A device can raise a line nobody claimed, and it will keep
            // doing so every time it fires. Reporting each one at warn would
            // bury the console under a single misbehaving device.
            log::trace!("unmapped hardware interrupt {hwirq} on {}", domain.name);
        }
        if let Some(complete) = domain.ops.complete {
            // SAFETY: registration validated the callback and `hwirq` was just
            // reported by the matching claim.
            unsafe { complete(domain.ops.context, cpu, platform_id, hwirq) };
        }
    }
    outcome
}

fn run(virq: u32) -> DispatchOutcome {
    let Some(record) = desc(virq) else {
        return DispatchOutcome::default();
    };
    record.count.fetch_add(1, Ordering::Relaxed);

    let mut result = DispatchOutcome::default();
    let mut wake: Option<Arc<IrqAction>> = None;
    let mut claimed = false;

    // Take a counted reference to the handler list and release the lock before
    // running any driver code. Holding it across a callback would deadlock the
    // moment a handler masked or reconfigured the line it is servicing, which
    // is exactly what a threaded handler is expected to do.
    record.in_flight.fetch_add(1, Ordering::AcqRel);
    let (actions, domain, hwirq) = {
        let inner = record.inner.lock();
        (inner.actions.clone(), inner.domain.clone(), inner.hwirq)
    };

    // The framework owns this line, so the architecture must complete the
    // delivery even when no handler claims it. Reporting it unhandled would
    // leave the controller asserted and take the system down over one confused
    // device.
    result.handled = !actions.is_empty();
    for action in actions.iter() {
        action.calls.fetch_add(1, Ordering::Relaxed);
        // SAFETY: the action is published in this list, and detaching one waits
        // for the in-flight count to drain before returning, so the callback
        // stays executable for the whole call.
        let status = unsafe { (action.handler)(action.context, virq) };
        if status & outcome::HANDLED != 0 || status & outcome::WAKE_THREAD != 0 {
            claimed = true;
        }
        if status & outcome::RESCHEDULE != 0 {
            result.reschedule = true;
        }
        if status & outcome::WAKE_THREAD != 0 && action.thread.is_some() {
            // Requests coalesce: two pends raised between the worker's wake
            // and its next wait yield exactly one thread run.
            wake = Some(action.clone());
        }
    }
    drop(actions);
    record.in_flight.fetch_sub(1, Ordering::Release);

    if let Some(domain) = domain {
        domain.eoi(hwirq);
    }
    if let Some(action) = wake {
        action.wake.signal();
        result.reschedule = true;
    }
    if !claimed {
        record.spurious.fetch_add(1, Ordering::Relaxed);
    }
    result
}

pub(crate) fn release_device_interrupts(device: &Arc<Device>) {
    for_each_action(|action| {
        action
            .device
            .as_ref()
            .and_then(Weak::upgrade)
            .is_some_and(|owner| Arc::ptr_eq(&owner, device))
    });
}

pub(crate) fn release_device_module_interrupts(device: &Arc<Device>, module: &Arc<Module>) {
    for_each_action(|action| {
        let device_matches = action
            .device
            .as_ref()
            .and_then(Weak::upgrade)
            .is_some_and(|owner| Arc::ptr_eq(&owner, device));
        let module_matches = action
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module));
        device_matches && module_matches
    });
}

pub(crate) fn remove_module_interrupts(module: &Arc<Module>) {
    for_each_action(|action| {
        action
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module))
    });
    let Ok(registry) = registry() else {
        return;
    };
    let owned: Vec<Arc<IrqDomain>> = registry
        .domains
        .lock()
        .iter()
        .filter(|domain| {
            domain
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module))
        })
        .cloned()
        .collect();
    for domain in owned {
        force_unregister_domain(&domain);
    }
}

fn for_each_action<F: Fn(&Arc<IrqAction>) -> bool>(select: F) {
    let limit = NEXT_VIRQ.load(Ordering::Acquire).min(MAX_VIRQ as u32);
    for virq in 1..limit {
        let Some(record) = desc(virq) else {
            continue;
        };
        let matching: Vec<Arc<IrqAction>> = {
            let inner = record.inner.lock();
            inner
                .actions
                .iter()
                .filter(|a| select(a))
                .cloned()
                .collect()
        };
        for action in matching {
            if let Err(error) = release(&action) {
                error!(
                    "failed to release {} on virq {virq}: {error:?}",
                    action.name
                );
            }
        }
    }
}

/// Snapshot of one virtual interrupt.
#[derive(Clone, Debug)]
pub struct IrqInfo {
    /// Virtual interrupt number.
    pub virq: u32,
    /// Owning domain name.
    pub domain: Option<Box<str>>,
    /// Hardware interrupt number within the domain.
    pub hwirq: u64,
    /// Total deliveries.
    pub count: u64,
    /// Deliveries no handler claimed.
    pub spurious: u64,
    /// Registered handler names.
    pub actions: Vec<Box<str>>,
}

/// Returns a snapshot of every mapped interrupt.
pub fn list() -> Vec<IrqInfo> {
    let limit = NEXT_VIRQ.load(Ordering::Acquire).min(MAX_VIRQ as u32);
    let mut entries = Vec::new();
    for virq in 1..limit {
        let Some(record) = desc(virq) else {
            continue;
        };
        let (domain, hwirq, actions) = {
            let inner = record.inner.lock();
            (
                inner
                    .domain
                    .as_ref()
                    .map(|domain| domain.name.to_string().into_boxed_str()),
                inner.hwirq,
                inner
                    .actions
                    .iter()
                    .map(|action| action.name.to_string().into_boxed_str())
                    .collect(),
            )
        };
        if domain.is_none() {
            continue;
        }
        entries.push(IrqInfo {
            virq,
            domain,
            hwirq,
            count: record.count.load(Ordering::Relaxed),
            spurious: record.spurious.load(Ordering::Relaxed),
            actions,
        });
    }
    entries
}
