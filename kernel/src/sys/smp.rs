//!
//! # Symmetric Multiprocessing
//!
//! This module owns CPU discovery, per-core bootstrap state, and the kernel's
//! small IPI work queue.
//!
//! The IPI path is intentionally tiny:
//!
//! - callers submit a small `fn()` callback through [`send_ipi`]
//! - the callback pointer is queued in the target CPU's lock-protected mailbox
//! - the target CPU is nudged with an architecture IPI or local software
//!   reschedule interrupt
//! - [`drain_ipi_queue`] runs the callback on the destination CPU before that
//!   CPU returns from the interrupt/trap path
//!
//! This keeps cross-core coordination allocation-free, works in early kernel
//! contexts, and lets subsystems express intent directly instead of smuggling
//! state through ad-hoc interrupt side effects.
//!

use alloc::{boxed::Box, vec::Vec};
use core::{
    cell::UnsafeCell,
    hint::spin_loop,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use limine::{mp, request::MpRequest};
use log::info;

#[cfg(target_arch = "x86_64")]
use limine::mp::RequestFlags;

use crate::{
    arch,
    sys::{clock::PerCpuClock, sched::PerCpuScheduler, sync::Once},
};

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
#[cfg(target_arch = "x86_64")]
static SMP_REQUEST: MpRequest = MpRequest::new().with_flags(RequestFlags::X2APIC);

#[used]
#[doc(hidden)]
#[unsafe(link_section = ".requests")]
#[cfg(target_arch = "riscv64")]
static SMP_REQUEST: MpRequest = MpRequest::new();

/// Platform specific core-local fields.
pub struct PlatformFields {
    /// Bitmap of supported x86 extensions.
    #[cfg(target_arch = "x86_64")]
    pub feats: arch::cpu::CpuFeatures,
    /// Calibrated local TSC frequency in Hz.
    #[cfg(target_arch = "x86_64")]
    pub tsc_hz: u64,
    /// Calibrated local APIC timer frequency in Hz, or `0` when unused.
    #[cfg(target_arch = "x86_64")]
    pub lapic_timer_hz: u64,
}

/// Kernel context unique to each CPU core.
///
/// The RISC-V trap entry assembly indexes the leading fields by fixed offsets.
#[repr(C)]
pub struct CoreLocal {
    /// Stack used by the kernel on IRQs.
    pub kernel_stack: u64,

    /// Placeholder for user stack on IRQs.
    pub user_stack: u64,

    /// ID of current CPU core.
    pub id: usize,

    /// Currently running thread ID on this CPU, if any.
    pub current_thread: usize,

    /// Timer state owned by this CPU.
    pub(crate) clock: Once<PerCpuClock>,

    /// Scheduler state owned by this CPU.
    pub(crate) scheduler: Once<PerCpuScheduler>,

    /// Deferred IPI work queued for this CPU.
    pub(crate) ipi: Once<PerCpuIpi>,

    /// Nesting depth for trap/interrupt handling on this CPU.
    pub interrupt_depth: usize,

    /// Platform specific context.
    #[allow(dead_code)]
    pub platform: PlatformFields,
}

impl PlatformFields {
    /// Creates a new `PlatformFields` instance.
    pub const fn new() -> Self {
        PlatformFields {
            #[cfg(target_arch = "x86_64")]
            feats: arch::cpu::CpuFeatures::empty(),
            #[cfg(target_arch = "x86_64")]
            tsc_hz: 0,
            #[cfg(target_arch = "x86_64")]
            lapic_timer_hz: 0,
        }
    }
}

impl Default for PlatformFields {
    fn default() -> Self {
        Self::new()
    }
}

impl CoreLocal {
    /// Creates a new core local context, with CPU ID `cid`.
    pub const fn new(cid: usize) -> Self {
        Self {
            id: cid,
            kernel_stack: 0,
            user_stack: 0,
            current_thread: 0,
            clock: Once::new(),
            scheduler: Once::new(),
            ipi: Once::new(),
            interrupt_depth: 0,
            platform: PlatformFields::new(),
        }
    }
}

/// RAII guard marking execution inside trap/interrupt context on the local CPU.
pub struct InterruptContextGuard {
    active: bool,
}

struct CpuRecord {
    logical_id: usize,
    platform_id: u64,
    core_local_addr: usize,
    online: AtomicBool,
}

struct SmpState {
    cpus: Box<[CpuRecord]>,
}

const IPI_QUEUE_CAPACITY: usize = 32;
const IRQ_SPIN_BACKOFF_MAX: u32 = 64;

struct IpiJob {
    callback: fn(),
}

struct IpiQueue {
    head: usize,
    len: usize,
    jobs: [Option<IpiJob>; IPI_QUEUE_CAPACITY],
}

pub(crate) struct PerCpuIpi {
    queue: IrqSpinLock<IpiQueue>,
}

/// Broadcast selector used by [`send_ipi`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum IpiTarget {
    /// Deliver to every other online CPU.
    All,
    /// Deliver to one logical CPU, which may be the current CPU.
    Single(usize),
}

static SMP_STATE: Once<SmpState> = Once::new();
static TOTAL_CPUS: AtomicUsize = AtomicUsize::new(1);
static ONLINE_CPUS: AtomicUsize = AtomicUsize::new(1);

impl CpuRecord {
    fn new_bsp(platform_id: u64, core_local: *const CoreLocal) -> Self {
        Self {
            logical_id: 0,
            platform_id,
            core_local_addr: core_local as usize,
            online: AtomicBool::new(true),
        }
    }

    fn new_ap(logical_id: usize, platform_id: u64) -> Self {
        Self {
            logical_id,
            platform_id,
            core_local_addr: Box::leak(Box::new(CoreLocal::new(logical_id))) as *mut CoreLocal
                as usize,
            online: AtomicBool::new(false),
        }
    }

    fn is_online(&self) -> bool {
        self.online.load(Ordering::Acquire)
    }

    fn core_local_ptr(&self) -> *const CoreLocal {
        self.core_local_addr as *const CoreLocal
    }

    fn mark_online(&self) -> usize {
        let was_online = self.online.swap(true, Ordering::AcqRel);
        assert!(
            !was_online,
            "smp: cpu{} marked online twice",
            self.logical_id
        );
        ONLINE_CPUS.fetch_add(1, Ordering::AcqRel) + 1
    }
}

impl SmpState {
    fn cpu_count(&self) -> usize {
        self.cpus.len()
    }

    fn by_platform_id(&self, platform_id: u64) -> Option<&CpuRecord> {
        self.cpus.iter().find(|cpu| cpu.platform_id == platform_id)
    }

    fn by_logical_id(&self, logical_id: usize) -> Option<&CpuRecord> {
        self.cpus.iter().find(|cpu| cpu.logical_id == logical_id)
    }
}

impl IpiJob {
    fn new(callback: fn()) -> Self {
        Self { callback }
    }

    fn is_equivalent(&self, other: &Self) -> bool {
        self.callback as usize == other.callback as usize
    }

    fn run(self) {
        (self.callback)();
    }
}

impl IpiQueue {
    const fn new() -> Self {
        Self {
            head: 0,
            len: 0,
            jobs: [const { None }; IPI_QUEUE_CAPACITY],
        }
    }

    fn push(&mut self, job: IpiJob) -> bool {
        if self.contains_equivalent(&job) {
            return false;
        }

        assert!(self.len < IPI_QUEUE_CAPACITY, "smp: cpu IPI queue overflow");

        let slot = (self.head + self.len) % IPI_QUEUE_CAPACITY;
        self.jobs[slot] = Some(job);
        self.len += 1;
        true
    }

    fn pop(&mut self) -> Option<IpiJob> {
        if self.len == 0 {
            return None;
        }

        let slot = self.head;
        let job = self.jobs[slot].take();
        self.head = (self.head + 1) % IPI_QUEUE_CAPACITY;
        self.len -= 1;
        job
    }

    fn contains_equivalent(&self, needle: &IpiJob) -> bool {
        for offset in 0..self.len {
            let slot = (self.head + offset) % IPI_QUEUE_CAPACITY;
            if self.jobs[slot]
                .as_ref()
                .map(|job| job.is_equivalent(needle))
                .unwrap_or(false)
            {
                return true;
            }
        }

        false
    }
}

impl PerCpuIpi {
    fn new() -> Self {
        Self {
            queue: IrqSpinLock::new(IpiQueue::new()),
        }
    }
}

fn smp_state() -> &'static SmpState {
    SMP_STATE.get().expect("smp: init required before use")
}

/// Compact spinlock that masks local interrupts while held.
///
/// Unlike [`crate::sys::sync::Mutex`], this lock is valid in interrupt
/// context. It is the kernel's primitive for data that must be reachable from
/// trap handlers while still preventing local IRQ re-entry deadlocks.
///
/// Contended CPUs use test-and-test-and-set with bounded exponential backoff,
/// reducing cache-line writes while preserving a one-byte lock state for the
/// many fine-grained instances embedded in kernel objects.
pub struct IrqSpinLock<T: ?Sized> {
    locked: AtomicBool,
    value: UnsafeCell<T>,
}

/// Guard for [`IrqSpinLock`].
#[must_use = "if unused the spinlock will immediately unlock"]
pub struct IrqSpinLockGuard<'a, T: ?Sized> {
    lock: &'a IrqSpinLock<T>,
    restore_irqs: bool,
    held: bool,
    _nosend: PhantomData<*mut ()>,
}

impl<T> IrqSpinLock<T> {
    /// Creates an IRQ-safe mutex with initial payload `value`.
    pub const fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            value: UnsafeCell::new(value),
        }
    }
}

// SAFETY: ownership of the protected value can move with the lock.
unsafe impl<T: ?Sized + Send> Send for IrqSpinLock<T> {}
// SAFETY: shared access to the value is serialized by the atomic lock state.
unsafe impl<T: ?Sized + Send> Sync for IrqSpinLock<T> {}

impl<T: ?Sized> IrqSpinLock<T> {
    /// Locks the mutex while interrupts are masked.
    #[inline]
    pub fn lock(&self) -> IrqSpinLockGuard<'_, T> {
        let restore_irqs = arch::irqstate();
        arch::irqset(false);
        self.acquire();

        IrqSpinLockGuard {
            lock: self,
            restore_irqs,
            held: true,
            _nosend: PhantomData,
        }
    }

    /// Attempts to lock the mutex while interrupts are masked.
    #[inline]
    pub fn try_lock(&self) -> Option<IrqSpinLockGuard<'_, T>> {
        let restore_irqs = arch::irqstate();
        arch::irqset(false);

        if self.try_acquire() {
            return Some(IrqSpinLockGuard {
                lock: self,
                restore_irqs,
                held: true,
                _nosend: PhantomData,
            });
        }

        if restore_irqs {
            arch::irqset(true);
        }
        None
    }

    /// Returns whether the spinlock is currently held.
    ///
    /// This is only a momentary observation and provides no synchronization.
    #[inline]
    pub fn is_locked(&self) -> bool {
        self.locked.load(Ordering::Relaxed)
    }

    /// Forcibly releases the spinlock without restoring the
    /// interrupted CPU's prior interrupt state.
    ///
    /// # Safety
    ///
    /// This is only sound in fatal recovery paths where the lock owner will
    /// never resume normal execution, such as global panic shutdown.
    pub unsafe fn force_unlock(&self) {
        self.release();
    }

    #[inline]
    fn try_acquire(&self) -> bool {
        self.locked
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    }

    #[inline]
    fn acquire(&self) {
        if !self.try_acquire() {
            self.acquire_slow();
        }
    }

    #[cold]
    #[inline(never)]
    fn acquire_slow(&self) {
        let mut backoff = 1u32;

        loop {
            while self.locked.load(Ordering::Relaxed) {
                for _ in 0..backoff {
                    spin_loop();
                }
                backoff = (backoff << 1).min(IRQ_SPIN_BACKOFF_MAX);
            }

            if self
                .locked
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
            {
                return;
            }
        }
    }

    #[inline]
    fn release(&self) {
        self.locked.store(false, Ordering::Release);
    }
}

impl<T: ?Sized> Deref for IrqSpinLockGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: this guard owns the lock and only exposes shared access.
        unsafe { &*self.lock.value.get() }
    }
}

impl<T: ?Sized> DerefMut for IrqSpinLockGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: this unique guard owns the lock and serializes mutation.
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T: ?Sized> IrqSpinLockGuard<'_, T> {
    /// Returns whether local interrupts were enabled before this lock attempt.
    #[inline]
    pub fn irqs_were_enabled(&self) -> bool {
        self.restore_irqs
    }

    /// Releases the lock while keeping local IRQs masked.
    ///
    /// The returned value reports whether the caller must eventually restore
    /// interrupts. The guard is consumed so protected data cannot be accessed
    /// after the raw lock has been released.
    pub fn unlock_keep_irqs_disabled(mut self) -> bool {
        self.release();
        let restore_irqs = self.restore_irqs;
        self.restore_irqs = false;
        restore_irqs
    }

    #[inline]
    fn release(&mut self) {
        if self.held {
            self.lock.release();
            self.held = false;
        }
    }
}

impl<T: ?Sized> Drop for IrqSpinLockGuard<'_, T> {
    fn drop(&mut self) {
        self.release();

        if self.restore_irqs {
            arch::irqset(true);
        }
    }
}

impl Drop for InterruptContextGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }

        // SAFETY: trap handling keeps local interrupts disabled, so no
        // interrupt-context mutation can alias this borrow.
        let cpu = unsafe { arch::thiscpu_mut() };
        assert!(
            cpu.interrupt_depth == 1,
            "smp: interrupt context underflow on cpu{}",
            cpu.id
        );
        cpu.interrupt_depth = 0;
    }
}

/// Discovers CPUs exposed by the bootloader and prepares their core-local state.
pub fn init() {
    if SMP_STATE.get().is_some() {
        return;
    }

    let response = SMP_REQUEST
        .get_response()
        .expect("smp: Limine SMP response missing");
    let cpus = discover_cpus(response);
    let total = cpus.len().max(1);

    TOTAL_CPUS.store(total, Ordering::Release);
    ONLINE_CPUS.store(1, Ordering::Release);
    SMP_STATE.call_once(|| SmpState { cpus });

    for cpu_id in 0..total {
        core_local(cpu_id)
            .unwrap_or_else(|| panic!("smp: missing core-local record for cpu{cpu_id}"))
            .ipi
            .call_once(PerCpuIpi::new);
    }
}

/// Starts every application processor discovered during [`init`].
pub fn start() {
    let response = match SMP_REQUEST.get_response() {
        Some(response) => response,
        None => return,
    };
    let state = smp_state();
    if state.cpu_count() <= 1 {
        return;
    }

    let bsp_platform_id = bsp_platform_id(response);
    for cpu in response.cpus() {
        let platform_id = cpu_platform_id(cpu);
        if platform_id == bsp_platform_id {
            continue;
        }

        let _ = state
            .by_platform_id(platform_id)
            .expect("smp: AP record missing during startup");
        cpu.goto_address.write(ap_entry);
    }

    let deadline = crate::sys::clock::monotonic_ns().saturating_add(1_000_000_000);
    while online_cpus() < state.cpu_count() {
        if crate::sys::clock::monotonic_ns() >= deadline {
            panic!(
                "smp: timed out waiting for APs ({}/{})",
                online_cpus(),
                state.cpu_count()
            );
        }
        spin_loop();
    }

    while scheduler_online_aps() + 1 < state.cpu_count() {
        if crate::sys::clock::monotonic_ns() >= deadline {
            panic!(
                "smp: timed out waiting for AP schedulers ({}/{})",
                scheduler_online_aps() + 1,
                state.cpu_count()
            );
        }
        spin_loop();
    }

    info!("smp: all {} CPU(s) online", state.cpu_count());
}

/// Returns the number of CPUs known to the kernel.
pub fn cpu_count() -> usize {
    TOTAL_CPUS.load(Ordering::Acquire)
}

/// Returns the number of CPUs that have completed bring-up.
pub fn online_cpus() -> usize {
    ONLINE_CPUS.load(Ordering::Acquire)
}

/// Returns the architecture-specific platform identifier for `cpu_id`.
pub fn platform_id(cpu_id: usize) -> Option<u64> {
    smp_state().by_logical_id(cpu_id).map(|cpu| cpu.platform_id)
}

/// Returns the immutable core-local record for `cpu_id`.
pub(crate) fn core_local(cpu_id: usize) -> Option<&'static CoreLocal> {
    let cpu = smp_state().by_logical_id(cpu_id)?;
    // SAFETY: each record stores a stable, leaked `CoreLocal` allocation that
    // remains valid for the kernel lifetime.
    Some(unsafe { &*cpu.core_local_ptr() })
}

/// Returns whether `cpu_id` is currently online.
pub fn is_online(cpu_id: usize) -> bool {
    smp_state()
        .by_logical_id(cpu_id)
        .map(CpuRecord::is_online)
        .unwrap_or(false)
}

/// Marks the local CPU as executing inside trap/interrupt context.
pub fn enter_interrupt_context() -> InterruptContextGuard {
    let Some(cpu) = arch::thiscpu_opt() else {
        return InterruptContextGuard { active: false };
    };

    assert!(
        !arch::irqstate(),
        "smp: interrupt entered with IRQs enabled on cpu{}",
        cpu.id
    );
    assert!(
        cpu.interrupt_depth == 0,
        "smp: nested interrupt on cpu{}",
        cpu.id
    );

    // SAFETY: interrupt entry runs with local interrupts disabled and the
    // assertions above rule out nested mutation.
    unsafe { arch::thiscpu_mut() }.interrupt_depth = 1;
    InterruptContextGuard { active: true }
}

/// Returns whether the local CPU is currently handling a trap/interrupt.
pub fn in_interrupt_context() -> bool {
    arch::thiscpu_opt()
        // Trap handlers in this kernel keep local IRQs masked for their full
        // lifetime. Once thread code has resumed with IRQs restored, treat the
        // CPU as normal thread context even if the depth bit lags behind for a
        // handoff path.
        .map(|cpu| cpu.interrupt_depth != 0 && !arch::irqstate())
        .unwrap_or(false)
}

/// Queues `callback` for one or more CPUs and nudges them through the kernel's
/// IPI path.
///
/// The callback must be a plain `'static` function item.
///
/// [`IpiTarget::All`] broadcasts to every other online CPU. Use
/// [`IpiTarget::Single`] with the local CPU id when the current CPU also needs
/// to observe the callback.
///
/// Returns the number of CPUs that accepted a new queued callback.
pub fn send_ipi(callback: fn(), target: IpiTarget) -> usize {
    let this_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
    let mut queued = 0usize;

    match target {
        IpiTarget::All => {
            for cpu_id in 0..cpu_count() {
                if Some(cpu_id) == this_cpu || !is_online(cpu_id) {
                    continue;
                }

                if queue_ipi_job(cpu_id, callback) {
                    kick_cpu(cpu_id, this_cpu);
                    queued += 1;
                }
            }
        }
        IpiTarget::Single(cpu_id) => {
            if queue_ipi_job(cpu_id, callback) {
                kick_cpu(cpu_id, this_cpu);
                queued = 1;
            }
        }
    }

    queued
}

/// Executes all pending IPI callbacks queued for the current CPU.
///
/// This is called from the scheduler's trap-return path so every delivered IPI
/// callback runs before the CPU decides whether to resume or switch threads.
pub(crate) fn drain_ipi_queue() {
    let cpu_id = arch::thiscpu().id;
    let queue = &core_local(cpu_id)
        .unwrap_or_else(|| panic!("smp: missing core-local record for cpu{cpu_id}"))
        .ipi
        .get()
        .unwrap_or_else(|| panic!("smp: cpu{cpu_id} IPI queue not initialized"))
        .queue;

    loop {
        let job = {
            let mut queue = queue.lock();
            queue.pop()
        };

        let Some(job) = job else {
            return;
        };

        job.run();
    }
}

fn queue_ipi_job(cpu_id: usize, callback: fn()) -> bool {
    let Some(target) = smp_state().by_logical_id(cpu_id) else {
        return false;
    };
    if !target.is_online() {
        return false;
    }

    core_local(cpu_id)
        .unwrap_or_else(|| panic!("smp: missing core-local record for cpu{cpu_id}"))
        .ipi
        .get()
        .unwrap_or_else(|| panic!("smp: cpu{cpu_id} IPI queue not initialized"))
        .queue
        .lock()
        .push(IpiJob::new(callback))
}

fn kick_cpu(cpu_id: usize, this_cpu: Option<usize>) {
    if this_cpu == Some(cpu_id) {
        if arch::irqstate() && !in_interrupt_context() {
            arch::reschedule();
        }
        return;
    }

    arch::send_ipi(cpu_id);
}

fn scheduler_online_aps() -> usize {
    let mut ready = 0usize;

    for cpu_id in 1..cpu_count() {
        let Some(scheduler) = core_local(cpu_id).and_then(|core| core.scheduler.get()) else {
            continue;
        };

        ready += usize::from(scheduler.is_online());
    }

    ready
}

fn discover_cpus(response: &limine::response::MpResponse) -> Box<[CpuRecord]> {
    let bsp_platform_id = bsp_platform_id(response);
    let mut cpus = Vec::with_capacity(response.cpus().len().max(1));

    let bsp_core_local = arch::thiscpu() as *const CoreLocal;
    cpus.push(CpuRecord::new_bsp(bsp_platform_id, bsp_core_local));

    let mut next_logical_id = 1usize;
    for cpu in response.cpus() {
        if cpu_platform_id(cpu) == bsp_platform_id {
            continue;
        }

        cpus.push(CpuRecord::new_ap(next_logical_id, cpu_platform_id(cpu)));
        next_logical_id += 1;
    }

    cpus.into_boxed_slice()
}

unsafe extern "C" fn ap_entry(cpu: &mp::Cpu) -> ! {
    arch::irqset(false);

    let record = smp_state()
        .by_platform_id(cpu_platform_id(cpu))
        .expect("smp: missing AP record");

    arch::init_secondary(record.core_local_ptr());
    crate::mem::alloc::register_tlb_cpu(record.logical_id);
    crate::mem::register_tlb_cpu(record.logical_id);
    crate::sys::clock::start();
    let _ = record.mark_online();
    crate::sys::sched::start_secondary();
}

#[cfg(target_arch = "x86_64")]
fn cpu_platform_id(cpu: &mp::Cpu) -> u64 {
    cpu.lapic_id as u64
}

#[cfg(target_arch = "x86_64")]
fn bsp_platform_id(response: &limine::response::MpResponse) -> u64 {
    response.bsp_lapic_id() as u64
}

#[cfg(target_arch = "riscv64")]
fn cpu_platform_id(cpu: &mp::Cpu) -> u64 {
    cpu.hartid
}

#[cfg(target_arch = "riscv64")]
fn bsp_platform_id(response: &limine::response::MpResponse) -> u64 {
    response.bsp_hartid()
}
