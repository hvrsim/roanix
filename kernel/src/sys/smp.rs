//!
//! # Multicore Support
//!
//! This module is responsible for managing multiple CPU cores. Main duties
//! include AP bringup and core local data definitions.
//!

use alloc::{boxed::Box, vec::Vec};
use core::hint::spin_loop;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicUsize, Ordering};

use limine::{mp, request::MpRequest};
use log::info;
use spin::{Mutex, MutexGuard};

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

#[derive(Copy, Clone)]
struct BootCpu {
    id: usize,
    key: BootCpuKey,
    core_local: *const CoreLocal,
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct BootCpuKey(u64);

struct SmpState {
    boot_cpus: Box<[BootCpu]>,
}

unsafe impl Send for SmpState {}
unsafe impl Sync for SmpState {}

impl SmpState {
    fn by_key(&self, key: BootCpuKey) -> Option<&BootCpu> {
        self.boot_cpus.iter().find(|entry| entry.key == key)
    }

    fn by_id(&self, id: usize) -> Option<&BootCpu> {
        self.boot_cpus.iter().find(|entry| entry.id == id)
    }
}

static SMP_STATE: IrqSpinLock<Option<SmpState>> = IrqSpinLock::new(None);
static TOTAL_CPUS: AtomicUsize = AtomicUsize::new(1);
static ONLINE_CPUS: AtomicUsize = AtomicUsize::new(1);

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

pub fn init() {
    let mut guard = SMP_STATE.lock();
    if guard.is_some() {
        return;
    }

    let response = SMP_REQUEST
        .get_response()
        .expect("smp: Limine SMP response missing");
    let bsp = bsp_key(response);
    let mut next_id = 1usize;
    let mut boot_cpus = Vec::with_capacity(response.cpus().len());

    for cpu in response.cpus() {
        let cpu = *cpu;
        let key = cpu_key(cpu);
        let core_local = if key == bsp {
            arch::thiscpu() as *mut CoreLocal as *const CoreLocal
        } else {
            Box::leak(Box::new(CoreLocal::new(next_id))) as *mut CoreLocal as *const CoreLocal
        };

        if key != bsp {
            next_id += 1;
        }

        let id = unsafe { (&*core_local).id };
        boot_cpus.push(BootCpu {
            id,
            key,
            core_local,
        });
    }

    let total_cpus = boot_cpus.len().max(1);
    TOTAL_CPUS.store(total_cpus, Ordering::Release);
    ONLINE_CPUS.store(1, Ordering::Release);
    *guard = Some(SmpState {
        boot_cpus: boot_cpus.into_boxed_slice(),
    });
}

pub fn start() {
    let response = match SMP_REQUEST.get_response() {
        Some(response) => response,
        None => return,
    };
    let total = cpu_count();
    if total <= 1 {
        return;
    }

    let bsp = bsp_key(response);
    for cpu in response.cpus() {
        if cpu_key(cpu) == bsp {
            continue;
        }

        cpu.goto_address.write(ap_entry);
    }

    info!("smp: startup sent to {} AP(s)", total - 1);
    while online_cpus() < total {
        spin_loop();
    }

    info!("smp: all {} CPU(s) online", total);
}

pub fn cpu_count() -> usize {
    TOTAL_CPUS.load(Ordering::Acquire)
}

pub fn online_cpus() -> usize {
    ONLINE_CPUS.load(Ordering::Acquire)
}

fn core_local_for_cpu(cpu: &mp::Cpu) -> Option<*const CoreLocal> {
    let key = cpu_key(cpu);
    let guard = SMP_STATE.lock();
    let state = guard.as_ref()?;
    state.by_key(key).map(|entry| entry.core_local)
}

pub fn platform_id(cpu_id: usize) -> Option<u64> {
    let guard = SMP_STATE.lock();
    let state = guard.as_ref()?;
    state.by_id(cpu_id).map(|entry| entry.key.0)
}

pub fn send_ipi(cpu_id: usize) {
    let this_cpu = arch::thiscpu_opt().map(|cpu| cpu.id);
    if this_cpu == Some(cpu_id) || cpu_id >= cpu_count() {
        return;
    }

    arch::send_ipi(cpu_id);
}

fn mark_online(id: usize) {
    let prev = ONLINE_CPUS.fetch_add(1, Ordering::AcqRel);
    let total = cpu_count();
    info!("smp: cpu{} online ({}/{})", id, prev + 1, total);
}

unsafe extern "C" fn ap_entry(cpu: &mp::Cpu) -> ! {
    arch::irqset(false);
    let core_local = core_local_for_cpu(cpu).expect("smp: missing AP core-local");
    arch::init_secondary(core_local);
    mark_online(arch::thiscpu().id);
    crate::sys::clock::start_secondary();
    crate::sys::sched::start_secondary();
}

#[cfg(target_arch = "x86_64")]
fn cpu_key(cpu: &mp::Cpu) -> BootCpuKey {
    BootCpuKey(cpu.lapic_id as u64)
}

#[cfg(target_arch = "x86_64")]
fn bsp_key(response: &limine::response::MpResponse) -> BootCpuKey {
    BootCpuKey(response.bsp_lapic_id() as u64)
}

#[cfg(target_arch = "riscv64")]
fn cpu_key(cpu: &mp::Cpu) -> BootCpuKey {
    BootCpuKey(cpu.hartid)
}

#[cfg(target_arch = "riscv64")]
fn bsp_key(response: &limine::response::MpResponse) -> BootCpuKey {
    BootCpuKey(response.bsp_hartid())
}
