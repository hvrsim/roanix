//! POSIX signal state, delivery, and signal file descriptors.

use alloc::{
    collections::VecDeque,
    sync::{Arc, Weak},
};
use core::{
    mem::{MaybeUninit, offset_of, size_of},
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    time::Duration,
};

use crate::{
    arch::cpu::TrapFrame,
    fs::PollEvents,
    mem::{USER_ADDRESS_MAX, USER_ADDRESS_MIN, VirtAddr},
    proc::{Descriptor, Error, Process, Result},
    sys::{clock, event::Event, sched, sync::Mutex},
    syscall::{
        Errno, Result as SyscallResult, current_process, map_memory_error, raw_result,
        read_user_timespec,
    },
};

const SIGNAL_MAX: usize = 64;
const SIG_DFL: u64 = 0;
const SIG_IGN: u64 = 1;

pub(crate) const SIGHUP: u8 = 1;
pub(crate) const SIGINT: u8 = 2;
pub(crate) const SIGQUIT: u8 = 3;
pub(crate) const SIGILL: u8 = 4;
pub(crate) const SIGTRAP: u8 = 5;
pub(crate) const SIGBUS: u8 = 7;
pub(crate) const SIGFPE: u8 = 8;
const SIGKILL: u8 = 9;
pub(crate) const SIGSEGV: u8 = 11;
pub(crate) const SIGPIPE: u8 = 13;
pub(crate) const SIGCHLD: u8 = 17;
pub(crate) const SIGCONT: u8 = 18;
const SIGSTOP: u8 = 19;
pub(crate) const SIGTSTP: u8 = 20;
pub(crate) const SIGTTIN: u8 = 21;
pub(crate) const SIGTTOU: u8 = 22;
const SIGURG: u8 = 23;
pub(crate) const SIGWINCH: u8 = 28;
const SIGRTMIN: u8 = 35;

const SIG_BLOCK: i32 = 0;
const SIG_UNBLOCK: i32 = 1;
const SIG_SETMASK: i32 = 2;

const SA_NOCLDSTOP: u64 = 0x0000_0001;
const SA_NOCLDWAIT: u64 = 0x0000_0002;
const SA_SIGINFO: u64 = 0x0000_0004;
const SA_RESTORER: u64 = 0x0400_0000;
const SA_ONSTACK: u64 = 0x0800_0000;
const SA_RESTART: u64 = 0x1000_0000;
const SA_NODEFER: u64 = 0x4000_0000;
const SA_RESETHAND: u64 = 0x8000_0000;
const SUPPORTED_ACTION_FLAGS: u64 = SA_NOCLDSTOP
    | SA_NOCLDWAIT
    | SA_SIGINFO
    | SA_RESTORER
    | SA_ONSTACK
    | SA_RESTART
    | SA_NODEFER
    | SA_RESETHAND;

const SFD_NONBLOCK: u64 = 0o4000;
const SFD_CLOEXEC: u64 = 0o2000000;
const SS_ONSTACK: i32 = 1;
const SS_DISABLE: i32 = 2;
const MINSIGSTKSZ: u64 = 2048;
const SIGNALFD_RECORD_SIZE: usize = 128;
const USER_SIGSET_SIZE: usize = 128;
const USER_SIGACTION_SIZE: usize = 24 + USER_SIGSET_SIZE;
const USER_STACK_SIZE: usize = 24;
const SIGNAL_FRAME_MAGIC: u64 = 0x524f_414e_5349_4746;
const SI_USER: i32 = 0;
const SI_KERNEL: i32 = 128;

#[derive(Clone, Copy)]
pub(crate) struct SignalAction {
    handler: u64,
    flags: u64,
    restorer: u64,
    mask: u64,
}

/// Applies pending signal state to the frame selected for userspace return.
#[unsafe(no_mangle)]
extern "C" fn rsignal_return(frame: &mut TrapFrame) -> *mut TrapFrame {
    super::exit_current_thread_if_process_exited();
    deliver_pending(frame);
    frame
}

impl SignalAction {
    const DEFAULT: Self = Self {
        handler: SIG_DFL,
        flags: 0,
        restorer: 0,
        mask: 0,
    };
}

fn siginfo(info: SignalInfo) -> [u8; SIGNALFD_RECORD_SIZE] {
    let mut record = [0u8; SIGNALFD_RECORD_SIZE];
    record[0..4].copy_from_slice(&u32::from(info.signal).to_ne_bytes());
    record[8..12].copy_from_slice(&info.code.to_ne_bytes());
    record[16..20].copy_from_slice(&info.sender_pid.to_ne_bytes());
    record[20..24].copy_from_slice(&info.sender_uid.to_ne_bytes());
    record
}

#[derive(Clone, Copy)]
struct SignalInfo {
    signal: u8,
    code: i32,
    sender_pid: u32,
    sender_uid: u32,
    synchronous: bool,
}

enum SignalWait {
    Ready(SignalInfo),
    Interrupted,
    Sleep,
}

#[derive(Clone, Copy)]
struct SignalStack {
    pointer: u64,
    size: u64,
}

struct SignalState {
    actions: [SignalAction; SIGNAL_MAX + 1],
    pending: VecDeque<SignalInfo>,
    mask: u64,
    alt_stack: Option<SignalStack>,
    suspend_restore_mask: Option<u64>,
    stopped: Option<(usize, u64)>,
}

/// Process-owned signal dispositions, pending queue, and wait event.
pub(crate) struct SignalManager {
    state: Mutex<SignalState>,
    event: Event,
}

/// A signalfd open-file description.
pub(crate) struct SignalFd {
    process: Weak<Process>,
    mask: AtomicU64,
    nonblocking: AtomicBool,
}

#[repr(C)]
struct UserSignalFrame {
    magic: u64,
    old_mask: u64,
    saved: TrapFrame,
    info: [u8; SIGNALFD_RECORD_SIZE],
}

impl SignalManager {
    pub(crate) fn new() -> Self {
        Self {
            state: Mutex::new(SignalState {
                actions: [SignalAction::DEFAULT; SIGNAL_MAX + 1],
                pending: VecDeque::new(),
                mask: 0,
                alt_stack: None,
                suspend_restore_mask: None,
                stopped: None,
            }),
            event: Event::new(),
        }
    }

    pub(crate) fn fork_from(parent: &Self) -> Self {
        let parent = parent.state.lock();
        Self {
            state: Mutex::new(SignalState {
                actions: parent.actions,
                pending: VecDeque::new(),
                mask: parent.mask,
                alt_stack: parent.alt_stack,
                suspend_restore_mask: None,
                stopped: None,
            }),
            event: Event::new(),
        }
    }

    pub(crate) fn reset_for_exec(&self) {
        let mut state = self.state.lock();
        for action in &mut state.actions[1..] {
            if action.handler != SIG_IGN {
                *action = SignalAction::DEFAULT;
            }
        }
        state.alt_stack = None;
        state.suspend_restore_mask = None;
        state.stopped = None;
    }

    fn action(&self, signal: u8) -> SignalAction {
        self.state.lock().actions[signal as usize]
    }

    fn set_action(&self, signal: u8, action: SignalAction) -> SignalAction {
        let mut state = self.state.lock();
        let previous = core::mem::replace(&mut state.actions[signal as usize], action);
        if action.handler == SIG_IGN {
            state.pending.retain(|pending| pending.signal != signal);
        }
        previous
    }

    fn mask(&self) -> u64 {
        self.state.lock().mask
    }

    fn update_mask(&self, how: i32, mask: u64) -> SyscallResult<u64> {
        let mut state = self.state.lock();
        let old = state.mask;
        state.mask = match how {
            SIG_BLOCK => old | mask,
            SIG_UNBLOCK => old & !mask,
            SIG_SETMASK => mask,
            _ => return Err(Errno::Invalid),
        } & !unblockable_mask();
        Ok(old)
    }

    fn restore_mask(&self, mask: u64) {
        self.state.lock().mask = mask & !unblockable_mask();
    }

    fn pending_mask(&self) -> u64 {
        self.state
            .lock()
            .pending
            .iter()
            .fold(0, |mask, info| mask | signal_bit(info.signal))
    }

    fn alt_stack(&self) -> Option<SignalStack> {
        self.state.lock().alt_stack
    }

    fn replace_alt_stack(&self, stack: Option<SignalStack>) {
        self.state.lock().alt_stack = stack;
    }

    fn begin_suspend(&self, mask: u64) {
        let mut state = self.state.lock();
        if state.suspend_restore_mask.is_none() {
            state.suspend_restore_mask = Some(state.mask);
        }
        state.mask = mask & !unblockable_mask();
    }

    fn take_suspend_restore_mask(&self) -> Option<u64> {
        self.state.lock().suspend_restore_mask.take()
    }

    fn prepare_interrupt_wait(&self) -> bool {
        let state = self.state.lock();
        if state
            .pending
            .iter()
            .any(|info| interrupts_suspend(&state, info))
        {
            return true;
        }
        self.event.reset();
        false
    }

    fn prepare_signal_wait(&self, waited: u64) -> SignalWait {
        let mut state = self.state.lock();
        if let Some(index) = state
            .pending
            .iter()
            .position(|info| waited & signal_bit(info.signal) != 0)
        {
            let info = state
                .pending
                .remove(index)
                .expect("signal: selected waited signal vanished");
            if state.pending.is_empty() {
                self.event.reset();
            }
            return SignalWait::Ready(info);
        }
        if state.pending.iter().any(|info| {
            info.synchronous
                || info.signal == SIGKILL
                || info.signal == SIGSTOP
                || state.mask & signal_bit(info.signal) == 0
        }) {
            return SignalWait::Interrupted;
        }
        self.event.reset();
        SignalWait::Sleep
    }

    fn enqueue(&self, info: SignalInfo) {
        let mut wake_stopped = None;
        {
            let mut state = self.state.lock();
            let signal = info.signal;
            if signal == SIGCONT || signal == SIGKILL {
                state
                    .pending
                    .retain(|pending| !is_stop_signal(pending.signal));
                wake_stopped = state.stopped.take();
            } else if is_stop_signal(signal) {
                state.pending.retain(|pending| pending.signal != SIGCONT);
            }

            let ignored = state.actions[signal as usize].handler == SIG_IGN
                && signal != SIGKILL
                && signal != SIGSTOP;
            let duplicate =
                signal < SIGRTMIN && state.pending.iter().any(|pending| pending.signal == signal);
            if !ignored && !duplicate {
                state.pending.push_back(info);
                self.event.signal();
            }
        }
        if let Some((thread, sequence)) = wake_stopped {
            let _ = sched::wake(thread as *mut crate::sys::thread::Thread, sequence);
        }
    }

    fn take_deliverable(&self) -> Option<SignalInfo> {
        let mut state = self.state.lock();
        let mask = state.mask;
        let index = state.pending.iter().position(|pending| {
            pending.synchronous
                || pending.signal == SIGKILL
                || pending.signal == SIGSTOP
                || mask & signal_bit(pending.signal) == 0
        })?;
        let info = state
            .pending
            .remove(index)
            .expect("signal: selected pending signal vanished");
        if state.pending.is_empty() {
            self.event.reset();
        }
        Some(info)
    }

    fn take_matching(&self, mask: u64) -> Option<SignalInfo> {
        let mut state = self.state.lock();
        let index = state
            .pending
            .iter()
            .position(|pending| mask & signal_bit(pending.signal) != 0)?;
        let info = state
            .pending
            .remove(index)
            .expect("signalfd: selected pending signal vanished");
        if state.pending.is_empty() {
            self.event.reset();
        }
        Some(info)
    }

    fn has_matching(&self, mask: u64) -> bool {
        self.state
            .lock()
            .pending
            .iter()
            .any(|pending| mask & signal_bit(pending.signal) != 0)
    }

    fn prepare_matching_wait(&self, mask: u64) -> bool {
        let state = self.state.lock();
        if state
            .pending
            .iter()
            .any(|pending| mask & signal_bit(pending.signal) != 0)
        {
            return true;
        }
        self.event.reset();
        false
    }

    fn mark_stopped(&self, thread: *mut crate::sys::thread::Thread, sequence: u64) {
        self.state.lock().stopped = Some((thread as usize, sequence));
    }
}

impl SignalFd {
    fn new(process: &Arc<Process>, mask: u64, nonblocking: bool) -> Arc<Self> {
        Arc::new(Self {
            process: Arc::downgrade(process),
            mask: AtomicU64::new(mask & !unblockable_mask()),
            nonblocking: AtomicBool::new(nonblocking),
        })
    }

    fn update(&self, mask: u64, nonblocking: bool) {
        self.mask
            .store(mask & !unblockable_mask(), Ordering::Release);
        self.nonblocking.store(nonblocking, Ordering::Release);
    }

    pub(crate) fn read(&self, buffer: &mut [u8]) -> SyscallResult<usize> {
        let records = buffer.len() / SIGNALFD_RECORD_SIZE;
        if records == 0 {
            return Err(Errno::Invalid);
        }
        let process = self.process.upgrade().ok_or(Errno::BadFileDescriptor)?;
        let mask = self.mask.load(Ordering::Acquire);

        loop {
            let mut written = 0usize;
            while written < records {
                let Some(info) = process.signals.take_matching(mask) else {
                    break;
                };
                let record = &mut buffer[written * SIGNALFD_RECORD_SIZE..][..SIGNALFD_RECORD_SIZE];
                encode_signalfd_info(record, info);
                written += 1;
            }
            if written != 0 {
                return Ok(written * SIGNALFD_RECORD_SIZE);
            }
            if self.nonblocking.load(Ordering::Acquire) {
                return Err(Errno::TryAgain);
            }
            if process.signals.prepare_matching_wait(mask) {
                continue;
            }
            process.signals.event.wait();
        }
    }

    pub(crate) fn poll(&self, requested: PollEvents) -> PollEvents {
        let Some(process) = self.process.upgrade() else {
            return PollEvents::HUP;
        };
        if requested.contains(PollEvents::IN)
            && process
                .signals
                .has_matching(self.mask.load(Ordering::Acquire))
        {
            PollEvents::IN
        } else {
            PollEvents::empty()
        }
    }

    pub(crate) fn poll_events<'a>(
        &'a self,
        _requested: PollEvents,
        _output: &mut alloc::vec::Vec<&'a Event>,
    ) -> bool {
        false
    }

    pub(crate) fn is_nonblocking(&self) -> bool {
        self.nonblocking.load(Ordering::Acquire)
    }

    pub(crate) fn set_nonblocking(&self, nonblocking: bool) {
        self.nonblocking.store(nonblocking, Ordering::Release);
    }
}

/// Queues `signal` for processes selected by POSIX `kill` rules.
pub(crate) fn kill(selector: i64, signal: i32) -> Result<()> {
    let signal = validate_optional_signal(signal)?;
    let current = super::current().ok_or(Error::InvalidArgument)?;
    let registry = super::process_registry().lock();
    let targets = registry
        .processes
        .values()
        .filter_map(Weak::upgrade)
        .filter(|process| !process.is_exited())
        .filter(|process| match selector {
            value if value > 0 => process.pid() == value as usize,
            0 => process.process_group() == current.process_group(),
            -1 => process.pid() != 1,
            value => process.process_group() == value.unsigned_abs() as usize,
        })
        .collect::<alloc::vec::Vec<_>>();
    drop(registry);

    if targets.is_empty() {
        return Err(Error::NoSuchProcess);
    }
    let Some(signal) = signal else {
        return Ok(());
    };
    for target in targets {
        target.signals.enqueue(SignalInfo {
            signal,
            code: SI_USER,
            sender_pid: current.pid() as u32,
            sender_uid: 0,
            synchronous: false,
        });
    }
    Ok(())
}

/// Queues a kernel-generated signal for one process.
pub(crate) fn send_kernel(process: &Arc<Process>, signal: u8) {
    process.signals.enqueue(SignalInfo {
        signal,
        code: SI_KERNEL,
        sender_pid: 0,
        sender_uid: 0,
        synchronous: false,
    });
}

/// Queues a kernel-generated signal for every live member of a process group.
pub(crate) fn send_kernel_process_group(group: usize, signal: u8) {
    if group == 0 || signal == 0 || usize::from(signal) > SIGNAL_MAX {
        return;
    }
    let targets = super::process_registry()
        .lock()
        .processes
        .values()
        .filter_map(Weak::upgrade)
        .filter(|process| !process.is_exited() && process.process_group() == group)
        .collect::<alloc::vec::Vec<_>>();
    for process in targets {
        send_kernel(&process, signal);
    }
}

/// Queues a synchronous kernel-generated signal for the current process.
pub(crate) fn send_current(signal: u8) {
    if let Some(process) = super::current() {
        process.signals.enqueue(SignalInfo {
            signal,
            code: SI_KERNEL,
            sender_pid: 0,
            sender_uid: 0,
            synchronous: true,
        });
    }
}

/// Delivers one pending signal before returning to userspace.
pub(crate) fn deliver_pending(frame: &mut TrapFrame) {
    if !frame.is_user() {
        return;
    }
    let Some(process) = super::current() else {
        return;
    };

    loop {
        let Some(info) = process.signals.take_deliverable() else {
            if let Some(mask) = process.signals.take_suspend_restore_mask() {
                process.signals.restore_mask(mask);
            }
            return;
        };
        let signal = info.signal;
        let mut action = process.signals.action(signal);

        if signal == SIGKILL {
            super::exit_current_signal(signal);
        }
        if signal == SIGSTOP || (action.handler == SIG_DFL && is_stop_signal(signal)) {
            let thread = sched::current_thread();
            // SAFETY: the current scheduler thread remains live while this
            // kernel continuation parks and is later resumed.
            let sequence = unsafe { &*thread }.prepare_park();
            process.signals.mark_stopped(thread, sequence);
            sched::park_current(thread, sequence);
            continue;
        }
        if action.handler == SIG_IGN || (action.handler == SIG_DFL && default_ignored(signal)) {
            continue;
        }
        if action.handler == SIG_DFL {
            super::exit_current_signal(signal);
        }

        let active_mask = process.signals.mask();
        let restore_mask = process
            .signals
            .take_suspend_restore_mask()
            .unwrap_or(active_mask);
        let mut next_mask = active_mask | action.mask;
        if action.flags & SA_NODEFER == 0 {
            next_mask |= signal_bit(signal);
        }
        process.signals.restore_mask(next_mask);
        if action.flags & SA_RESETHAND != 0 {
            process.signals.set_action(signal, SignalAction::DEFAULT);
            action.flags &= !SA_RESETHAND;
        }

        let signal_frame = UserSignalFrame {
            magic: SIGNAL_FRAME_MAGIC,
            old_mask: restore_mask,
            saved: *frame,
            info: siginfo(info),
        };
        let stack = if action.flags & SA_ONSTACK != 0 {
            process
                .signals
                .alt_stack()
                .filter(|stack| !stack_contains(*stack, frame.user_stack()))
                .and_then(|stack| stack.pointer.checked_add(stack.size))
                .unwrap_or_else(|| frame.user_stack())
        } else {
            frame.user_stack()
        };
        let Some((frame_address, handler_stack, return_slot)) =
            signal_frame_addresses(stack, size_of::<UserSignalFrame>() as u64)
        else {
            super::exit_current_signal(SIGSEGV);
        };
        if write_signal_frame(&process, frame_address, &signal_frame).is_err() {
            super::exit_current_signal(SIGSEGV);
        }
        if let Some(return_slot) = return_slot {
            if process
                .address_space()
                .write_user(VirtAddr::new(return_slot), &action.restorer.to_ne_bytes())
                .is_err()
            {
                super::exit_current_signal(SIGSEGV);
            }
        }

        let info_address = frame_address + offset_of!(UserSignalFrame, info) as u64;
        let context_address = frame_address + offset_of!(UserSignalFrame, saved) as u64;
        frame.setup_signal_handler(
            handler_stack,
            action.handler,
            action.restorer,
            u64::from(signal),
            info_address,
            context_address,
        );
        return;
    }
}

crate::syscall_handler! {
    syscall_signal_action(
        _frame,
        signal: i32 = 0,
        new_action: u64 = 1,
        old_action: u64 = 2,
    ) {
        let signal = validate_signal(signal)?;
        let process = current_process()?;
        let current = process.signals.action(signal);
        if old_action != 0 {
            write_user_action(&process, old_action, current)?;
        }
        if new_action != 0 {
            if signal == SIGKILL || signal == SIGSTOP {
                return Err(Errno::Invalid);
            }
            let action = read_user_action(&process, new_action)?;
            let custom_handler = action.handler > SIG_IGN;
            if action.flags & !SUPPORTED_ACTION_FLAGS != 0
                || (custom_handler && !valid_user_target(action.handler))
                || (custom_handler && !valid_user_target(action.restorer))
            {
                return Err(Errno::Invalid);
            }
            process.signals.set_action(signal, action);
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_signal_mask(
        _frame,
        how: i32 = 0,
        new_mask: u64 = 1,
        old_mask: u64 = 2,
    ) {
        let process = current_process()?;
        let current = process.signals.mask();
        if old_mask != 0 {
            write_user_mask(&process, old_mask, current)?;
        }
        if new_mask != 0 {
            let mask = read_user_mask(&process, new_mask)?;
            process.signals.update_mask(how, mask)?;
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_signal_return(frame) -> raw {
        raw_result(restore_signal_frame(frame).map(|value| value as u64))
    }
}

crate::syscall_handler! {
    syscall_signal_fd(_frame, fd: i32 = 0, mask: u64 = 1, flags: u64 = 2) {
        if mask == 0 || flags & !(SFD_NONBLOCK | SFD_CLOEXEC) != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let mask = read_user_mask(&process, mask)? & !unblockable_mask();
        if fd == -1 {
            let signal_fd = SignalFd::new(&process, mask, flags & SFD_NONBLOCK != 0);
            let fd = process
                .install_descriptor_value(
                    Descriptor::SignalFd(signal_fd),
                    flags & SFD_CLOEXEC != 0,
                )
                .map_err(crate::syscall::map_process_error)?;
            Ok(fd as u64)
        } else {
            let Descriptor::SignalFd(signal_fd) =
                process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?
            else {
                return Err(Errno::Invalid);
            };
            signal_fd.update(mask, flags & SFD_NONBLOCK != 0);
            Ok(fd as u64)
        }
    }
}

crate::syscall_handler! {
    syscall_signal_altstack(frame, new_stack: u64 = 0, old_stack: u64 = 1) {
        let process = current_process()?;
        let current = process.signals.alt_stack();
        let on_stack = current.is_some_and(|stack| stack_contains(stack, frame.user_stack()));
        if old_stack != 0 {
            write_user_stack(&process, old_stack, current, on_stack)?;
        }
        if new_stack != 0 {
            if on_stack {
                return Err(Errno::Permission);
            }
            let (pointer, flags, size) = read_user_stack(&process, new_stack)?;
            match flags {
                SS_DISABLE => process.signals.replace_alt_stack(None),
                0 => {
                    let end = pointer.checked_add(size).ok_or(Errno::OutOfMemory)?;
                    if pointer < USER_ADDRESS_MIN
                        || end > USER_ADDRESS_MAX
                        || size < MINSIGSTKSZ
                    {
                        return Err(Errno::OutOfMemory);
                    }
                    process
                        .signals
                        .replace_alt_stack(Some(SignalStack { pointer, size }));
                }
                _ => return Err(Errno::Invalid),
            }
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_signal_pending(_frame, output: u64 = 0) {
        let process = current_process()?;
        write_user_mask(&process, output, process.signals.pending_mask())?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_signal_suspend(_frame, mask: u64 = 0) {
        let process = current_process()?;
        process
            .signals
            .begin_suspend(read_user_mask(&process, mask)?);
        loop {
            if process.signals.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            process.signals.event.wait();
        }
    }
}

crate::syscall_handler! {
    syscall_signal_timed_wait(
        _frame,
        mask: u64 = 0,
        info: u64 = 1,
        timeout: u64 = 2,
    ) {
        let process = current_process()?;
        let mask = read_user_mask(&process, mask)? & !unblockable_mask();
        if mask == 0 {
            return Err(Errno::Invalid);
        }
        let timeout = if timeout == 0 {
            None
        } else {
            Some(read_user_timespec(&process, timeout)?)
        };
        let signal = wait_for_matching_signal(&process, mask, timeout)?;
        if info != 0 {
            write_user_signal_info(&process, info, signal)?;
        }
        Ok(u64::from(signal.signal))
    }
}

fn wait_for_matching_signal(
    process: &Process,
    mask: u64,
    timeout: Option<Duration>,
) -> SyscallResult<SignalInfo> {
    let deadline = timeout.map(|duration| {
        let nanoseconds = u64::try_from(duration.as_nanos()).unwrap_or(u64::MAX);
        clock::monotonic_ns().saturating_add(nanoseconds)
    });

    loop {
        match process.signals.prepare_signal_wait(mask) {
            SignalWait::Ready(info) => return Ok(info),
            SignalWait::Interrupted => return Err(Errno::Interrupted),
            SignalWait::Sleep => {}
        }

        let remaining = deadline.map(|deadline| deadline.saturating_sub(clock::monotonic_ns()));
        if remaining == Some(0) {
            return Err(Errno::TryAgain);
        }
        match remaining {
            Some(nanoseconds) => {
                if !clock::wait_timeout(&process.signals.event, Duration::from_nanos(nanoseconds)) {
                    return Err(Errno::TryAgain);
                }
            }
            None => process.signals.event.wait(),
        }
    }
}

fn restore_signal_frame(frame: &mut TrapFrame) -> SyscallResult<i64> {
    let process = current_process()?;
    let signal_frame = read_signal_frame(&process, frame.user_stack())?;
    if signal_frame.magic != SIGNAL_FRAME_MAGIC || !frame.restore_signal(&signal_frame.saved) {
        return Err(Errno::Fault);
    }
    process.signals.restore_mask(signal_frame.old_mask);
    Ok(frame.syscall_result())
}

fn read_user_action(process: &Process, address: u64) -> SyscallResult<SignalAction> {
    let mut bytes = [0u8; USER_SIGACTION_SIZE];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(SignalAction {
        handler: u64::from_ne_bytes(bytes[0..8].try_into().expect("signal handler width")),
        flags: u64::from_ne_bytes(bytes[8..16].try_into().expect("signal flags width")),
        restorer: u64::from_ne_bytes(bytes[16..24].try_into().expect("signal restorer width")),
        mask: u64::from_ne_bytes(bytes[24..32].try_into().expect("signal mask width"))
            & !unblockable_mask(),
    })
}

fn write_user_action(process: &Process, address: u64, action: SignalAction) -> SyscallResult<()> {
    let mut bytes = [0u8; USER_SIGACTION_SIZE];
    bytes[0..8].copy_from_slice(&action.handler.to_ne_bytes());
    bytes[8..16].copy_from_slice(&action.flags.to_ne_bytes());
    bytes[16..24].copy_from_slice(&action.restorer.to_ne_bytes());
    bytes[24..32].copy_from_slice(&action.mask.to_ne_bytes());
    process
        .address_space()
        .write_user(VirtAddr::new(address), &bytes)
        .map_err(map_memory_error)
}

fn read_user_mask(process: &Process, address: u64) -> SyscallResult<u64> {
    let mut bytes = [0u8; USER_SIGSET_SIZE];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(u64::from_ne_bytes(
        bytes[..8].try_into().expect("signal mask width"),
    ))
}

fn write_user_mask(process: &Process, address: u64, mask: u64) -> SyscallResult<()> {
    let mut bytes = [0u8; USER_SIGSET_SIZE];
    bytes[..8].copy_from_slice(&mask.to_ne_bytes());
    process
        .address_space()
        .write_user(VirtAddr::new(address), &bytes)
        .map_err(map_memory_error)
}

fn read_user_stack(process: &Process, address: u64) -> SyscallResult<(u64, i32, u64)> {
    let mut bytes = [0u8; USER_STACK_SIZE];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok((
        u64::from_ne_bytes(bytes[..8].try_into().expect("stack pointer width")),
        i32::from_ne_bytes(bytes[8..12].try_into().expect("stack flags width")),
        u64::from_ne_bytes(bytes[16..24].try_into().expect("stack size width")),
    ))
}

fn write_user_stack(
    process: &Process,
    address: u64,
    stack: Option<SignalStack>,
    on_stack: bool,
) -> SyscallResult<()> {
    let mut bytes = [0u8; USER_STACK_SIZE];
    let (pointer, flags, size) = match stack {
        Some(stack) => (
            stack.pointer,
            if on_stack { SS_ONSTACK } else { 0 },
            stack.size,
        ),
        None => (0, SS_DISABLE, 0),
    };
    bytes[..8].copy_from_slice(&pointer.to_ne_bytes());
    bytes[8..12].copy_from_slice(&flags.to_ne_bytes());
    bytes[16..24].copy_from_slice(&size.to_ne_bytes());
    process
        .address_space()
        .write_user(VirtAddr::new(address), &bytes)
        .map_err(map_memory_error)
}

fn write_user_signal_info(process: &Process, address: u64, info: SignalInfo) -> SyscallResult<()> {
    process
        .address_space()
        .write_user(VirtAddr::new(address), &siginfo(info))
        .map_err(map_memory_error)
}

fn write_signal_frame(
    process: &Process,
    address: u64,
    frame: &UserSignalFrame,
) -> SyscallResult<()> {
    // SAFETY: `UserSignalFrame` contains only initialized integer fields,
    // arrays, and the fully initialized architecture trap frame.
    let bytes = unsafe {
        core::slice::from_raw_parts(
            core::ptr::from_ref(frame).cast::<u8>(),
            size_of::<UserSignalFrame>(),
        )
    };
    process
        .address_space()
        .write_user(VirtAddr::new(address), bytes)
        .map_err(map_memory_error)
}

fn read_signal_frame(process: &Process, address: u64) -> SyscallResult<UserSignalFrame> {
    let mut frame = MaybeUninit::<UserSignalFrame>::zeroed();
    // SAFETY: the destination covers the complete `UserSignalFrame` storage,
    // and it is only assumed initialized after a successful full user copy.
    let bytes = unsafe {
        core::slice::from_raw_parts_mut(
            frame.as_mut_ptr().cast::<u8>(),
            size_of::<UserSignalFrame>(),
        )
    };
    process
        .address_space()
        .read_user(VirtAddr::new(address), bytes)
        .map_err(map_memory_error)?;
    // SAFETY: every byte was initialized by `read_user`.
    Ok(unsafe { frame.assume_init() })
}

fn validate_signal(signal: i32) -> SyscallResult<u8> {
    let signal = u8::try_from(signal).map_err(|_| Errno::Invalid)?;
    if signal == 0 || usize::from(signal) > SIGNAL_MAX {
        return Err(Errno::Invalid);
    }
    Ok(signal)
}

fn validate_optional_signal(signal: i32) -> Result<Option<u8>> {
    if signal == 0 {
        return Ok(None);
    }
    let signal = u8::try_from(signal).map_err(|_| Error::InvalidArgument)?;
    if usize::from(signal) > SIGNAL_MAX {
        return Err(Error::InvalidArgument);
    }
    Ok(Some(signal))
}

const fn signal_bit(signal: u8) -> u64 {
    1u64 << (signal - 1)
}

const fn unblockable_mask() -> u64 {
    signal_bit(SIGKILL) | signal_bit(SIGSTOP)
}

fn default_ignored(signal: u8) -> bool {
    matches!(signal, SIGCHLD | SIGCONT | SIGURG | SIGWINCH)
}

fn is_stop_signal(signal: u8) -> bool {
    matches!(signal, SIGSTOP | SIGTSTP | SIGTTIN | SIGTTOU)
}

fn valid_user_target(address: u64) -> bool {
    (crate::mem::USER_ADDRESS_MIN..crate::mem::USER_ADDRESS_MAX).contains(&address)
}

fn encode_signalfd_info(record: &mut [u8], info: SignalInfo) {
    record.fill(0);
    record[0..4].copy_from_slice(&u32::from(info.signal).to_ne_bytes());
    record[8..12].copy_from_slice(&info.code.to_ne_bytes());
    record[12..16].copy_from_slice(&info.sender_pid.to_ne_bytes());
    record[16..20].copy_from_slice(&info.sender_uid.to_ne_bytes());
}

fn signalfd_info(info: SignalInfo) -> [u8; SIGNALFD_RECORD_SIZE] {
    let mut record = [0u8; SIGNALFD_RECORD_SIZE];
    encode_signalfd_info(&mut record, info);
    record
}

fn stack_contains(stack: SignalStack, pointer: u64) -> bool {
    stack
        .pointer
        .checked_add(stack.size)
        .is_some_and(|end| (stack.pointer..end).contains(&pointer))
}

fn interrupts_suspend(state: &SignalState, info: &SignalInfo) -> bool {
    if !info.synchronous
        && info.signal != SIGKILL
        && info.signal != SIGSTOP
        && state.mask & signal_bit(info.signal) != 0
    {
        return false;
    }
    let action = state.actions[info.signal as usize];
    action.handler != SIG_IGN && !(action.handler == SIG_DFL && default_ignored(info.signal))
}

#[cfg(target_arch = "x86_64")]
fn signal_frame_addresses(stack: u64, frame_size: u64) -> Option<(u64, u64, Option<u64>)> {
    let frame_address = stack
        .checked_sub(128)?
        .checked_sub(frame_size)?
        .checked_sub(15)?
        & !15;
    let return_slot = frame_address.checked_sub(8)?;
    (return_slot >= USER_ADDRESS_MIN).then_some((frame_address, return_slot, Some(return_slot)))
}

#[cfg(target_arch = "riscv64")]
fn signal_frame_addresses(stack: u64, frame_size: u64) -> Option<(u64, u64, Option<u64>)> {
    let frame_address = stack.checked_sub(frame_size)?.checked_sub(15)? & !15;
    (frame_address >= USER_ADDRESS_MIN).then_some((frame_address, frame_address, None))
}
