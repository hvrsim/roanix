//!
//! # Multicore Support
//!
//! CPU discovery and AP bring-up with a minimal immutable CPU table.
//!

use alloc::{boxed::Box, vec::Vec};
use core::hint::spin_loop;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use limine::{mp, request::MpRequest};
use log::info;
use spin::{Mutex, MutexGuard, Once};

use crate::arch;

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
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

    /// Per-CPU timer tick counter.
    pub ticks: u64,

    /// Next local scheduler deadline in monotonic nanoseconds.
    pub next_stat_deadline_ns: u64,

    /// Currently running thread ID on this CPU, if any.
    pub current_thread: usize,

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

impl CoreLocal {
    /// Creates a new core local context, with CPU ID `cid`.
    pub const fn new(cid: usize) -> Self {
        Self {
            id: cid,
            kernel_stack: 0,
            user_stack: 0,
            ticks: 0,
            next_stat_deadline_ns: 0,
            current_thread: 0,
            platform: PlatformFields::new(),
        }
    }
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

fn smp_state() -> &'static SmpState {
    SMP_STATE.get().expect("smp: init required before use")
}

/// `spin::Mutex` wrapper that masks interrupts while holding the lock.
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

/// Sends a reschedule IPI to `cpu_id` when it refers to another online CPU.
pub fn send_ipi(cpu_id: usize) {
    let state = smp_state();
    let this_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
    let Some(target) = state.by_logical_id(cpu_id) else {
        return;
    };

    if this_cpu == Some(cpu_id) || !target.is_online() {
        return;
    }

    arch::send_ipi(cpu_id);
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
    crate::sys::clock::start_secondary();
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
