//! Module-owned event identities. Lookups retain storage across waits and IRQ
//! signals, so destruction cannot invalidate an event already being observed.

use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::sys::{event::Event, sync::Once};

use super::super::{
    core::module::Module,
    error::{Error, Result},
    handle::HandleTable,
};

#[derive(Clone)]
struct Entry {
    event: Arc<Event>,
    owner: Option<Arc<Module>>,
}

static EVENTS: Once<HandleTable<Entry>> = Once::new();
static NEXT_ID: AtomicUsize = AtomicUsize::new(1);

fn events() -> &'static HandleTable<Entry> {
    EVENTS.call_once(HandleTable::new)
}

/// Creates an event and returns its unique opaque identity.
pub fn create(owner: Option<&Arc<Module>>) -> Result<usize> {
    let id = NEXT_ID
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
        .map_err(|_| Error::NoSpace)?;
    events().insert(
        id,
        Entry {
            event: Arc::new(Event::new()),
            owner: owner.cloned(),
        },
    );
    Ok(id)
}

/// Revokes future lookup and wakes observers already retaining this event.
pub fn destroy(handle: usize) -> Result<()> {
    let event = events().remove(handle).ok_or(Error::NotFound)?;
    event.event.signal();
    Ok(())
}

/// Runs an operation with counted storage retained until the call finishes.
pub fn with<F: FnOnce(&Event) -> Result<()>>(handle: usize, action: F) -> Result<()> {
    let event = resolve(handle).ok_or(Error::NotFound)?;
    action(&event)
}

/// IRQ-safe level notification without releasing counted storage on the IRQ CPU.
pub fn notify(handle: usize, signal: bool) -> Result<()> {
    EVENTS
        .get()
        .and_then(|events| {
            events.with(handle, |entry| {
                if signal {
                    entry.event.signal();
                } else {
                    entry.event.reset();
                }
            })
        })
        .ok_or(Error::NotFound)
}

/// Retains a live event for a kernel adapter or an entire wait registration.
pub fn resolve(handle: usize) -> Option<Arc<Event>> {
    EVENTS.get()?.get(handle).map(|entry| entry.event)
}

/// Revokes module events after their producers and workers have stopped.
pub fn remove_module_events(module: &Arc<Module>) {
    if let Some(events) = EVENTS.get() {
        events.retain(|_, entry| {
            let owned = entry
                .owner
                .as_ref()
                .is_some_and(|owner| Arc::ptr_eq(owner, module));
            if owned {
                entry.event.signal();
            }
            !owned
        });
    }
}
