//! Movable queue ownership: the spin lock protects only taking and returning
//! a queue. Completion waits and buffer access happen outside that lock.

use core::ops::{Deref, DerefMut};
use ddk::{Event, Result, TicketLock};

pub struct Lease<T> {
    value: TicketLock<Option<T>>,
    pub ready: Event,
}

impl<T> Lease<T> {
    pub fn new(value: T) -> Result<Self> {
        Ok(Self {
            value: TicketLock::new(Some(value)),
            ready: Event::create()?,
        })
    }

    pub fn try_take(&self) -> Result<Option<Guard<'_, T>>> {
        let mut value = self.value.lock();
        // Reset and transfer ownership under the same lock used by return.
        // A later return cannot be lost between checking and waiting.
        self.ready.reset()?;
        Ok(value.take().map(|value| Guard {
            owner: self,
            value: Some(value),
        }))
    }

    pub fn take(&self) -> Result<Guard<'_, T>> {
        loop {
            if let Some(guard) = self.try_take()? {
                return Ok(guard);
            }
            self.ready.wait()?;
        }
    }
}

impl<T> Drop for Lease<T> {
    fn drop(&mut self) {
        let _ = self.ready.destroy();
    }
}

pub struct Guard<'a, T> {
    owner: &'a Lease<T>,
    value: Option<T>,
}

impl<T> Deref for Guard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.value.as_ref().expect("live queue lease")
    }
}

impl<T> DerefMut for Guard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.value.as_mut().expect("live queue lease")
    }
}

impl<T> Drop for Guard<'_, T> {
    fn drop(&mut self) {
        let mut slot = self.owner.value.lock();
        *slot = self.value.take();
        let _ = self.owner.ready.signal();
    }
}
