//! Direction-typed I/O buffers that avoid kernel bounce allocations.
//!
//! A transfer target is either kernel memory or a range in a user address
//! space. User ranges are exposed through short-lived windows that alias the
//! resident frame through the higher-half direct map, so a filesystem copies
//! straight from its page cache into the user page and back.
//!
//! Windows never span a page boundary and are resolved without holding the
//! address-space lock. Resolving before the copy is what keeps the page-cache
//! locks and the address-space lock strictly ordered: a caller holding a page
//! backing lock must never fault, and a window is already faulted in.

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
    slice,
};

use alloc::sync::Arc;

use super::{Error, PAGE_SIZE, Result, VirtAddr, map::FaultAccess, page::VmPage, pmap::VmSpace};

/// Resolves one page-bounded window of a user range.
///
/// Faults the page in for `access` and wires its frame, so the returned alias
/// stays valid until the paired window guard drops - however long the caller
/// keeps it, including across blocking driver callbacks.
#[allow(clippy::type_complexity)]
fn resolve_window(
    space: &VmSpace,
    address: VirtAddr,
    offset: usize,
    maximum: usize,
    access: FaultAccess,
) -> Result<(Arc<VmPage>, *mut u8, usize)> {
    let current = address
        .as_u64()
        .checked_add(offset as u64)
        .map(VirtAddr::new)
        .ok_or(Error::InvalidAddress)?;
    let page_remaining = (PAGE_SIZE - (current.as_u64() % PAGE_SIZE)) as usize;
    let length = maximum.min(page_remaining);
    let (page, physical) = space.pin_user_page(current, access)?;
    let pointer = super::phys_to_virt(physical).as_mut_ptr::<u8>();
    Ok((page, pointer, length))
}

/// A writable window produced by [`IoSink::window`].
///
/// Dereferences to the window bytes. Dropping it releases the user-frame pin,
/// so keep the guard alive for exactly as long as the memory is in use.
pub enum SinkWindow<'a> {
    /// Kernel-resident window; no pin is required.
    Kernel(&'a mut [u8]),
    /// Direct-map alias of one wired user frame.
    User {
        /// Holds the frame's logical page, which owns the wire.
        page: Arc<VmPage>,
        /// Direct-map start of the window.
        pointer: *mut u8,
        /// Window length in bytes, bounded by one page.
        length: usize,
        /// Ties the guard to the sink borrow.
        marker: PhantomData<&'a mut [u8]>,
    },
}

impl SinkWindow<'_> {
    fn empty() -> Self {
        Self::Kernel(&mut [])
    }
}

impl Drop for SinkWindow<'_> {
    fn drop(&mut self) {
        if let Self::User { page, .. } = self {
            // Releases the reclamation wire taken by `pin_user_page`.
            page.release_window();
        }
    }
}

impl Deref for SinkWindow<'_> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        match self {
            Self::Kernel(buffer) => buffer,
            Self::User {
                pointer, length, ..
            } => {
                // SAFETY: `resolve_window` produced this aligned direct-map
                // alias and the retained page keeps all `length` bytes live.
                unsafe { slice::from_raw_parts_mut(*pointer, *length) }
            }
        }
    }
}

impl DerefMut for SinkWindow<'_> {
    fn deref_mut(&mut self) -> &mut [u8] {
        match self {
            Self::Kernel(buffer) => buffer,
            Self::User {
                pointer, length, ..
            } => {
                // SAFETY: the retained page keeps this unique, page-bounded
                // direct-map alias writable for `length` bytes.
                unsafe { slice::from_raw_parts_mut(*pointer, *length) }
            }
        }
    }
}

/// A readable window produced by [`IoSource::window`].
///
/// Dereferences to the window bytes. Dropping it releases the user-frame pin,
/// so keep the guard alive for exactly as long as the memory is in use.
pub enum SourceWindow<'a> {
    /// Kernel-resident window; no pin is required.
    Kernel(&'a [u8]),
    /// Direct-map alias of one wired user frame.
    User {
        /// Holds the frame's logical page, which owns the wire.
        page: Arc<VmPage>,
        /// Direct-map start of the window.
        pointer: *const u8,
        /// Window length in bytes, bounded by one page.
        length: usize,
        /// Ties the guard to the source borrow.
        marker: PhantomData<&'a [u8]>,
    },
}

impl SourceWindow<'_> {
    fn empty() -> Self {
        Self::Kernel(&[])
    }
}

impl Drop for SourceWindow<'_> {
    fn drop(&mut self) {
        if let Self::User { page, .. } = self {
            // Releases the reclamation wire taken by `pin_user_page`.
            page.release_window();
        }
    }
}

impl Deref for SourceWindow<'_> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        match self {
            Self::Kernel(buffer) => buffer,
            Self::User {
                pointer, length, ..
            } => {
                // SAFETY: `resolve_window` produced this aligned direct-map
                // alias and the retained page keeps all `length` bytes live.
                unsafe { slice::from_raw_parts(*pointer, *length) }
            }
        }
    }
}

/// Destination for bytes produced by a read-style operation.
pub enum IoSink<'a> {
    /// Kernel-resident destination buffer.
    Kernel(&'a mut [u8]),
    /// Range in a user address space.
    User {
        /// Address space owning the range.
        space: &'a VmSpace,
        /// First byte of the range.
        address: VirtAddr,
        /// Range length in bytes.
        length: usize,
        /// Ties the borrow to the address space.
        marker: PhantomData<&'a mut [u8]>,
    },
}

impl<'a> IoSink<'a> {
    /// Creates a sink targeting kernel memory.
    pub fn kernel(buffer: &'a mut [u8]) -> Self {
        Self::Kernel(buffer)
    }

    /// Creates a sink targeting a validated user range.
    pub fn user(space: &'a VmSpace, address: VirtAddr, length: usize) -> Result<Self> {
        space.validate_user(address, length)?;
        Ok(Self::User {
            space,
            address,
            length,
            marker: PhantomData,
        })
    }

    /// Returns the total transfer length.
    pub fn len(&self) -> usize {
        match self {
            Self::Kernel(buffer) => buffer.len(),
            Self::User { length, .. } => *length,
        }
    }

    /// Returns whether the transfer is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Borrows a shorter sink covering `offset..offset + length`.
    pub fn slice(&mut self, offset: usize, length: usize) -> Result<IoSink<'_>> {
        let end = offset.checked_add(length).ok_or(Error::InvalidAddress)?;
        if end > self.len() {
            return Err(Error::InvalidAddress);
        }
        Ok(match self {
            Self::Kernel(buffer) => IoSink::Kernel(&mut buffer[offset..end]),
            Self::User { space, address, .. } => IoSink::User {
                space,
                address: VirtAddr::new(
                    address
                        .as_u64()
                        .checked_add(offset as u64)
                        .ok_or(Error::InvalidAddress)?,
                ),
                length,
                marker: PhantomData,
            },
        })
    }

    /// Returns a writable window of at most `maximum` bytes starting at `offset`.
    ///
    /// The window stops at the next page boundary, so callers must loop until
    /// the requested count is satisfied. The returned guard pins a user frame
    /// against reclamation; drop it as soon as the bytes are no longer in use.
    pub fn window(&mut self, offset: usize, maximum: usize) -> Result<SinkWindow<'_>> {
        let available = self
            .len()
            .checked_sub(offset)
            .ok_or(Error::InvalidAddress)?;
        let maximum = maximum.min(available);
        if maximum == 0 {
            return Ok(SinkWindow::empty());
        }
        match self {
            Self::Kernel(buffer) => Ok(SinkWindow::Kernel(&mut buffer[offset..offset + maximum])),
            Self::User { space, address, .. } => {
                let (page, pointer, length) =
                    resolve_window(space, *address, offset, maximum, FaultAccess::Write)?;
                Ok(SinkWindow::User {
                    page,
                    pointer,
                    length,
                    marker: PhantomData,
                })
            }
        }
    }

    /// Copies `source` into the sink at `offset`.
    pub fn store(&mut self, offset: usize, source: &[u8]) -> Result<()> {
        let mut written = 0usize;
        while written < source.len() {
            let mut window = self.window(offset + written, source.len() - written)?;
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            window.copy_from_slice(&source[written..written + count]);
            written += count;
        }
        Ok(())
    }

    /// Zero-fills `length` bytes at `offset`.
    pub fn fill_zero(&mut self, offset: usize, length: usize) -> Result<()> {
        let mut filled = 0usize;
        while filled < length {
            let mut window = self.window(offset + filled, length - filled)?;
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            window.fill(0);
            filled += count;
        }
        Ok(())
    }
}

/// Source of bytes consumed by a write-style operation.
pub enum IoSource<'a> {
    /// Kernel-resident source buffer.
    Kernel(&'a [u8]),
    /// Range in a user address space.
    User {
        /// Address space owning the range.
        space: &'a VmSpace,
        /// First byte of the range.
        address: VirtAddr,
        /// Range length in bytes.
        length: usize,
        /// Ties the borrow to the address space.
        marker: PhantomData<&'a [u8]>,
    },
}

impl<'a> IoSource<'a> {
    /// Creates a source backed by kernel memory.
    pub fn kernel(buffer: &'a [u8]) -> Self {
        Self::Kernel(buffer)
    }

    /// Creates a source backed by a validated user range.
    pub fn user(space: &'a VmSpace, address: VirtAddr, length: usize) -> Result<Self> {
        space.validate_user(address, length)?;
        Ok(Self::User {
            space,
            address,
            length,
            marker: PhantomData,
        })
    }

    /// Returns the total transfer length.
    pub fn len(&self) -> usize {
        match self {
            Self::Kernel(buffer) => buffer.len(),
            Self::User { length, .. } => *length,
        }
    }

    /// Returns whether the transfer is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Borrows a shorter source covering `offset..offset + length`.
    pub fn slice(&self, offset: usize, length: usize) -> Result<IoSource<'_>> {
        let end = offset.checked_add(length).ok_or(Error::InvalidAddress)?;
        if end > self.len() {
            return Err(Error::InvalidAddress);
        }
        Ok(match self {
            Self::Kernel(buffer) => IoSource::Kernel(&buffer[offset..end]),
            Self::User { space, address, .. } => IoSource::User {
                space,
                address: VirtAddr::new(
                    address
                        .as_u64()
                        .checked_add(offset as u64)
                        .ok_or(Error::InvalidAddress)?,
                ),
                length,
                marker: PhantomData,
            },
        })
    }

    /// Returns a readable window of at most `maximum` bytes starting at `offset`.
    ///
    /// The window stops at the next page boundary, so callers must loop until
    /// the requested count is satisfied. The returned guard pins a user frame
    /// against reclamation; drop it as soon as the bytes are no longer in use.
    pub fn window(&self, offset: usize, maximum: usize) -> Result<SourceWindow<'_>> {
        let available = self
            .len()
            .checked_sub(offset)
            .ok_or(Error::InvalidAddress)?;
        let maximum = maximum.min(available);
        if maximum == 0 {
            return Ok(SourceWindow::empty());
        }
        match self {
            Self::Kernel(buffer) => Ok(SourceWindow::Kernel(&buffer[offset..offset + maximum])),
            Self::User { space, address, .. } => {
                let (page, pointer, length) =
                    resolve_window(space, *address, offset, maximum, FaultAccess::Read)?;
                Ok(SourceWindow::User {
                    page,
                    pointer,
                    length,
                    marker: PhantomData,
                })
            }
        }
    }

    /// Copies bytes from the source at `offset` into `destination`.
    pub fn load(&self, offset: usize, destination: &mut [u8]) -> Result<()> {
        let mut read = 0usize;
        while read < destination.len() {
            let window = self.window(offset + read, destination.len() - read)?;
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            destination[read..read + count].copy_from_slice(&window);
            read += count;
        }
        Ok(())
    }
}
