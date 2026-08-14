//! Module-owned events.
//!
//! Events are handed to modules as plain addresses so that an operation table
//! can hand one straight back to the framework - the terminal and device-node
//! layers both accept an event address to attach readiness notification.
//!
//! A handle is the address of the event record itself rather than a key into a
//! table, because a driver signals an event from its interrupt handler and that
//! path must not take a sleeping lock or perform a lookup. The record carries a
//! tag so a malformed or already-destroyed handle is rejected in constant time.

use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicU32, Ordering};

use crate::sys::{event::Event, sync::Mutex};

use super::super::{
    core::module::Module,
    error::{Error, Result},
};

const MAGIC_LIVE: u32 = 0x5244_4645;
const MAGIC_DEAD: u32 = 0xDEAD_4645;

#[repr(C)]
struct Record {
    magic: AtomicU32,
    event: Event,
}

struct Entry {
    record: Arc<Record>,
    owner: Option<Arc<Module>>,
}

/// Registry of live events. Only creation and destruction consult it; signal,
/// wait, and reset go straight through the handle.
static EVENTS: Mutex<Vec<Entry>> = Mutex::new(Vec::new());

/// Creates an event owned by `owner` and returns its stable address.
pub fn create(owner: Option<&Arc<Module>>) -> Result<usize> {
    let record = Arc::new(Record {
        magic: AtomicU32::new(MAGIC_LIVE),
        event: Event::new(),
    });
    let handle = Arc::as_ptr(&record) as usize;
    EVENTS.lock().push(Entry {
        record,
        owner: owner.cloned(),
    });
    Ok(handle)
}

/// Destroys an event.
///
/// The handle must not be used afterwards, which is the same contract every
/// other resource receipt in the ABI carries.
pub fn destroy(handle: usize) -> Result<()> {
    let mut events = EVENTS.lock();
    let position = events
        .iter()
        .position(|entry| Arc::as_ptr(&entry.record) as usize == handle)
        .ok_or(Error::NotFound)?;
    events[position].record.magic.store(MAGIC_DEAD, Ordering::Release);
    events.remove(position);
    Ok(())
}

/// Borrows the event behind a handle without taking any lock.
///
/// This is what makes signalling safe from an interrupt handler.
fn borrow(handle: usize) -> Result<&'static Event> {
    if handle == 0 || !handle.is_multiple_of(align_of::<Record>()) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: a handle is produced only by `create`, and the ABI forbids using
    // one after `destroy`, so it addresses a live record allocation.
    let record = unsafe { &*(handle as *const Record) };
    if record.magic.load(Ordering::Acquire) != MAGIC_LIVE {
        return Err(Error::NotFound);
    }
    Ok(&record.event)
}

/// Runs `action` against a registered event.
pub fn with<F: FnOnce(&Event) -> Result<()>>(handle: usize, action: F) -> Result<()> {
    action(borrow(handle)?)
}

/// Returns the event behind a handle, for the kernel-side class adapters.
pub fn resolve(handle: usize) -> Option<&'static Event> {
    borrow(handle).ok()
}

/// Releases every event owned by `module`.
pub fn remove_module_events(module: &Arc<Module>) {
    EVENTS.lock().retain(|entry| {
        let owned = entry
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, module));
        if owned {
            entry.record.magic.store(MAGIC_DEAD, Ordering::Release);
        }
        !owned
    });
}
