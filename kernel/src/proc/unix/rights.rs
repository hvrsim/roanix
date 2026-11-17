//! Bounded descriptor transfer without socket ownership cycles.
//!
//! Queue entries retain actual open-description ownership, not observer refs.
//! Unix sockets and epoll sets require a separate cycle policy; signal streams
//! currently bind to their creating process. None are transferable here.

use super::super::{Descriptor, DescriptorEntry, Process};
use crate::syscall::{Errno, Result};
use alloc::vec::Vec;

pub(super) const MAX_RIGHTS: usize = 64;
pub(super) const RIGHT_CHARGE: usize = 64;

impl Process {
    pub(super) fn capture_rights(&self, fds: &[i32]) -> Result<Vec<DescriptorEntry>> {
        if fds.len() > MAX_RIGHTS {
            return Err(Errno::MessageSize);
        }
        let files = self.files.lock();
        // Validate the complete set before cloning ownership. All descriptors
        // come from one table snapshot, including repeated entries.
        let entries = fds
            .iter()
            .map(|fd| {
                let index = usize::try_from(*fd).map_err(|_| Errno::BadFileDescriptor)?;
                let entry = files
                    .slots
                    .get(index)
                    .and_then(Option::as_ref)
                    .ok_or(Errno::BadFileDescriptor)?;
                match &entry.descriptor {
                    Descriptor::File(_)
                    | Descriptor::Pipe(_)
                    | Descriptor::EventFd(_)
                    | Descriptor::TimerFd(_)
                    | Descriptor::Inotify(_) => Ok(entry),
                    _ => Err(Errno::NotSupported),
                }
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(entries.into_iter().cloned().collect())
    }

    /// Reserve free numbers, publish all user output, then install ownership.
    /// Failed copyout leaves both the table and the queued message unchanged.
    /// As with truncated control storage, an exhausted table delivers a prefix
    /// and reports truncation; it never consumes an unreported installed FD.
    pub(super) fn receive_rights<E>(
        &self,
        rights: &[DescriptorEntry],
        capacity: usize,
        close_on_exec: bool,
        publish: impl FnOnce(&[i32], bool) -> core::result::Result<(), E>,
    ) -> core::result::Result<usize, E> {
        if rights.is_empty() || capacity == 0 {
            publish(&[], !rights.is_empty())?;
            return Ok(0);
        }
        let mut files = self.files.lock();
        let numbers = files
            .available(0)
            .take(capacity.min(rights.len()))
            .map(|index| index as i32)
            .collect::<Vec<_>>();
        publish(&numbers, numbers.len() < rights.len())?;
        for (fd, entry) in numbers.iter().zip(rights) {
            let mut entry = entry.clone();
            entry.close_on_exec = close_on_exec;
            files.insert(*fd as usize, entry);
        }
        Ok(numbers.len())
    }
}
