//!
//! # Symmetric Multiprocessing
//!
//! This module owns CPU discovery, per-core bootstrap state, and the kernel's
//! small IPI work queue.
//!
//! The IPI path is intentionally tiny:
//!
//! - callers submit a small `Copy` callback through [`send_ipi`]
//! - the callback is copied into the target CPU's lock-protected mailbox
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
use core::hint::spin_loop;
use core::mem::{align_of, size_of};
use core::ops::{Deref, DerefMut};
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use limine::{mp, request::MpRequest};
use log::info;
use spin::{Mutex, MutexGuard, Once};

#[cfg(target_arch = "x86_64")]
use limine::mp::RequestFlags;

use crate::{
    arch,
    sys::{clock::PerCpuClock, sched::PerCpuScheduler},
};

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
#[cfg(target_arch = "x86_64")]
static SMP_REQUEST: MpRequest = MpRequest::new().with_flags(RequestFlags::X2APIC);

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
#[cfg(target_arch = "riscv64")]
static SMP_REQUEST: MpRequest = MpRequest::new();

/// Platform specific core-local fields.
pub struct PlatformFields {
    /// Bitmap of supported x86 extensions.
    #[cfg(target_arch = "x86_64")]
    pub feats: arch::cpu::CpuFeatures,
}

/// Kernel context unique to each CPU core.
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
const IPI_INLINE_WORDS: usize = 4;
const IPI_INLINE_BYTES: usize = IPI_INLINE_WORDS * size_of::<usize>();

#[repr(C)]
struct InlineIpiPayload {
    words: [usize; IPI_INLINE_WORDS],
}

struct IpiJob {
    invoke: unsafe fn(*const u8),
    size: u8,
    payload: InlineIpiPayload,
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

impl InlineIpiPayload {
    const fn zeroed() -> Self {
        Self {
            words: [0; IPI_INLINE_WORDS],
        }
    }
}

impl IpiJob {
    fn new<F>(callback: F) -> Self
    where
        F: FnOnce() + Copy + Send + 'static,
    {
        assert!(
            size_of::<F>() <= IPI_INLINE_BYTES,
            "smp: IPI callback size {} exceeds {} bytes",
            size_of::<F>(),
            IPI_INLINE_BYTES
        );
        assert!(
            align_of::<F>() <= align_of::<InlineIpiPayload>(),
            "smp: IPI callback alignment {} exceeds {}",
            align_of::<F>(),
            align_of::<InlineIpiPayload>()
        );

        let mut payload = InlineIpiPayload::zeroed();
        if size_of::<F>() != 0 {
            unsafe {
                ptr::copy_nonoverlapping(
                    (&callback as *const F).cast::<u8>(),
                    payload.words.as_mut_ptr().cast::<u8>(),
                    size_of::<F>(),
                );
            }
        }

        Self {
            invoke: invoke_ipi_job::<F>,
            size: size_of::<F>() as u8,
            payload,
        }
    }

    fn is_equivalent(&self, other: &Self) -> bool {
        self.size == 0 && other.size == 0 && self.invoke as usize == other.invoke as usize
    }

    unsafe fn run(self) {
        assert!(
            self.invoke as usize != 0,
            "smp: invalid IPI callback pointer (size={})",
            self.size
        );
        (self.invoke)(self.payload.words.as_ptr().cast::<u8>());
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
        if needle.size != 0 {
            return false;
        }

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

unsafe fn invoke_ipi_job<F>(payload: *const u8)
where
    F: FnOnce() + Copy + Send + 'static,
{
    let callback = ptr::read(payload.cast::<F>());
    callback();
}

/// `spin::Mutex` wrapper that masks interrupts while holding the lock.
///
/// Unlike [`crate::sys::sync::Mutex`], this lock is valid in interrupt
/// context. It is the kernel's primitive for data that must be reachable from
/// trap handlers while still preventing local IRQ re-entry deadlocks.
pub struct IrqSpinLock<T> {
    inner: Mutex<T>,
}

/// Guard for [`IrqSpinLock`].
pub struct IrqSpinLockGuard<'a, T> {
    guard: Option<MutexGuard<'a, T>>,
    irq_enabled: bool,
}

impl<T> IrqSpinLock<T> {
    /// Creates an IRQ-safe mutex with initial payload `value`.
    pub const fn new(value: T) -> Self {
        Self {
            inner: Mutex::new(value),
        }
    }

    /// Locks the mutex while interrupts are masked.
    pub fn lock(&self) -> IrqSpinLockGuard<'_, T> {
        let irq_enabled = arch::irqstate();
        arch::irqset(false);

        IrqSpinLockGuard {
            guard: Some(self.inner.lock()),
            irq_enabled,
        }
    }

    /// Attempts to lock the mutex while interrupts are masked.
    pub fn try_lock(&self) -> Option<IrqSpinLockGuard<'_, T>> {
        let irq_enabled = arch::irqstate();
        arch::irqset(false);

        let guard = self.inner.try_lock();
        if guard.is_none() && irq_enabled {
            arch::irqset(true);
        }

        guard.map(|guard| IrqSpinLockGuard {
            guard: Some(guard),
            irq_enabled,
        })
    }

    /// Returns whether the underlying spin mutex is currently held.
    pub fn is_locked(&self) -> bool {
        self.inner.is_locked()
    }

    /// Forcibly releases the underlying spin mutex without restoring the
    /// interrupted CPU's prior interrupt state.
    ///
    /// # Safety
    ///
    /// This is only sound in fatal recovery paths where the lock owner will
    /// never resume normal execution, such as global panic shutdown.
    pub unsafe fn force_unlock(&self) {
        self.inner.force_unlock();
    }
}

impl<T> Deref for IrqSpinLockGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.guard.as_ref().unwrap()
    }
}

impl<T> DerefMut for IrqSpinLockGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.guard.as_mut().unwrap()
    }
}

impl<T> Drop for IrqSpinLockGuard<'_, T> {
    fn drop(&mut self) {
        drop(self.guard.take());

        if self.irq_enabled {
            arch::irqset(true);
        }
    }
}

impl Drop for InterruptContextGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }

        let cpu = arch::thiscpu();
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
    smp_state()
        .by_logical_id(cpu_id)
        .map(|cpu| unsafe { &*cpu.core_local_ptr() })
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

    cpu.interrupt_depth = 1;
    InterruptContextGuard { active: true }
}

/// Returns whether the local CPU is currently handling a trap/interrupt.
pub fn in_interrupt_context() -> bool {
    arch::thiscpu_opt()
        .map(|cpu| cpu.interrupt_depth != 0)
        .unwrap_or(false)
}

/// Queues `callback` for one or more CPUs and nudges them through the kernel's
/// IPI path.
///
/// The callback is copied into a fixed-size per-CPU mailbox, so it must be:
///
/// - `Copy`, because broadcast delivery duplicates the callback
/// - small enough to fit in the inline IPI payload
/// - `Send + 'static`, because it executes later on another CPU
///
/// [`IpiTarget::All`] broadcasts to every other online CPU. Use
/// [`IpiTarget::Single`] with the local CPU id when the current CPU also needs
/// to observe the callback.
///
/// Returns the number of CPUs that accepted a new queued callback.
pub fn send_ipi<F>(callback: F, target: IpiTarget) -> usize
where
    F: FnOnce() + Copy + Send + 'static,
{
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

        unsafe {
            job.run();
        }
    }
}

fn queue_ipi_job<F>(cpu_id: usize, callback: F) -> bool
where
    F: FnOnce() + Copy + Send + 'static,
{
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

    let bsp_core_local = arch::thiscpu() as *mut CoreLocal as *const CoreLocal;
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
