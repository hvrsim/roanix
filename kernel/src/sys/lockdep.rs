//! Mutex ownership and explicit lock-order diagnostics for debug kernels.
//!
//! Every mutex checks recursion. Classified locks additionally follow ascending
//! ranks; instances in the same class follow ascending addresses. Unclassified
//! locks do not establish an ordering guarantee. This is an explicit hierarchy,
//! not a dynamically inferred dependency graph.

/// A static lock class with an ascending acquisition rank.
pub struct LockClass {
    /// Name included in ordering diagnostics.
    pub name: &'static str,
    /// Acquire lower ranks before higher ranks.
    pub rank: u16,
}

impl LockClass {
    /// Defines a class. Keep one static instance per logical lock class.
    pub const fn new(name: &'static str, rank: u16) -> Self {
        Self { name, rank }
    }
}

#[cfg(debug_assertions)]
mod debug {
    use super::LockClass;
    use crate::sys::sched;

    const MAX_HELD: usize = 32;

    #[derive(Clone, Copy)]
    struct Held {
        key: usize,
        class: Option<&'static LockClass>,
    }

    pub(crate) struct HeldLocks {
        locks: [Option<Held>; MAX_HELD],
    }

    impl HeldLocks {
        pub(crate) const fn new() -> Self {
            Self {
                locks: [None; MAX_HELD],
            }
        }
    }

    fn with_held(action: impl FnOnce(&mut HeldLocks)) {
        if let Some(thread) = sched::current_thread_opt() {
            // SAFETY: the current thread remains live throughout this call;
            // its diagnostic state is independently protected by a raw lock.
            action(&mut unsafe { &*thread }.held_locks.lock());
        }
    }

    pub(crate) fn check(key: usize, class: Option<&'static LockClass>) {
        with_held(|held| {
            for lock in held.locks.iter().flatten() {
                assert_ne!(key, lock.key, "lockdep: recursive mutex acquisition");
                if let (Some(next), Some(previous)) = (class, lock.class) {
                    assert!(
                        next.rank > previous.rank
                            || (core::ptr::eq(next, previous) && key > lock.key),
                        "lockdep: {} acquired after {} violates rank/address order",
                        next.name,
                        previous.name
                    );
                }
            }
        });
    }

    pub(crate) fn acquired(key: usize, class: Option<&'static LockClass>) {
        with_held(|held| {
            let slot = held
                .locks
                .iter_mut()
                .find(|slot| slot.is_none())
                .expect("lockdep: too many held mutexes");
            *slot = Some(Held { key, class });
        });
    }

    pub(crate) fn released(key: usize) {
        with_held(|held| {
            let slot = held
                .locks
                .iter_mut()
                .find(|slot| slot.is_some_and(|lock| lock.key == key))
                .expect("lockdep: mutex released by a non-owner");
            *slot = None;
        });
    }
}

#[cfg(debug_assertions)]
pub(crate) use debug::{HeldLocks, acquired, check, released};

#[cfg(not(debug_assertions))]
pub(crate) fn check(_: usize, _: Option<&'static LockClass>) {}
#[cfg(not(debug_assertions))]
pub(crate) fn acquired(_: usize, _: Option<&'static LockClass>) {}
#[cfg(not(debug_assertions))]
pub(crate) fn released(_: usize) {}
