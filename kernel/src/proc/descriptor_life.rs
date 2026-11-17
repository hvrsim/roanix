//! Descriptor-table ownership, independent of references retained by I/O watches.

use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::sys::event::Event;

pub(crate) struct DescriptorLife {
    handles: AtomicUsize,
    socket: Option<Arc<super::unix::UnixSocket>>,
    pub(crate) closed: Event,
}

impl DescriptorLife {
    pub(crate) fn is_open(&self) -> bool {
        self.handles.load(Ordering::Acquire) != 0
    }
}

/// One actual descriptor-table slot. Clone only for dup or fork; observers
/// retain `life()` instead, so watches cannot keep a description logically open.
pub(crate) struct OpenHandle(Arc<DescriptorLife>);

impl OpenHandle {
    pub(crate) fn new() -> Self {
        Self(Arc::new(DescriptorLife {
            handles: AtomicUsize::new(1),
            socket: None,
            closed: Event::new(),
        }))
    }

    pub(crate) fn for_descriptor(descriptor: &super::Descriptor) -> Self {
        if let super::Descriptor::Unix(socket) = descriptor {
            Self(Arc::new(DescriptorLife {
                handles: AtomicUsize::new(1),
                socket: Some(socket.clone()),
                closed: Event::new(),
            }))
        } else {
            Self::new()
        }
    }

    pub(crate) fn life(&self) -> Arc<DescriptorLife> {
        self.0.clone()
    }
}

impl Clone for OpenHandle {
    fn clone(&self) -> Self {
        self.0
            .handles
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |count| {
                count.checked_add(1)
            })
            .expect("descriptor handle count exhausted");
        Self(self.0.clone())
    }
}

impl Drop for OpenHandle {
    fn drop(&mut self) {
        if self.0.handles.fetch_sub(1, Ordering::AcqRel) == 1 {
            if let Some(socket) = &self.0.socket {
                socket.close();
            }
            self.0.closed.signal();
        }
    }
}
