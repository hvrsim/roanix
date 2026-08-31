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
//!
//! # Aliasing invariant
//!
//! These windows follow the kernel's user-memory convention: userspace may
//! mutate a wired frame concurrently, so its direct-map alias is externally
//! mutable memory. The `IoSink` borrow prevents competing kernel windows made
//! through the same sink, while the page wire proves only that the frame stays
//! resident. Code that needs strict Rust aliasing must copy through raw-pointer
//! primitives rather than retain one of these slice views.

use core::{
    ops::{Deref, DerefMut},
    slice,
};

use alloc::sync::Arc;

use super::{Error, PAGE_SIZE, Result, VirtAddr, map::FaultAccess, page::VmPage, pmap::VmSpace};

/// Wired, page-bounded HHDM alias shared by readable and writable guards.
struct PinnedWindow {
    page: Arc<VmPage>,
    pointer: *mut u8,
    length: usize,
}

impl PinnedWindow {
    /// Faults and wires one user page without allocating or copying payload data.
    fn resolve(
        space: &VmSpace,
        address: VirtAddr,
        offset: usize,
        maximum: usize,
        access: FaultAccess,
    ) -> Result<Self> {
        let current = address
            .as_u64()
            .checked_add(offset as u64)
            .map(VirtAddr::new)
            .ok_or(Error::InvalidAddress)?;
        let remaining = (PAGE_SIZE - current.as_u64() % PAGE_SIZE) as usize;
        let (page, physical) = space.pin_user_page(current, access)?;
        Ok(Self {
            page,
            pointer: super::phys_to_virt(physical).as_mut_ptr(),
            length: maximum.min(remaining),
        })
    }

    fn as_slice(&self) -> &[u8] {
        // SAFETY: `resolve` produced this page-bounded direct-map alias and the
        // retained wire keeps all bytes live. See the aliasing invariant above.
        unsafe { slice::from_raw_parts(self.pointer, self.length) }
    }

    fn as_mut_slice(&mut self) -> &mut [u8] {
        // SAFETY: the retained wire keeps this page-bounded alias live; the
        // guard's mutable borrow excludes another view through the same sink.
        unsafe { slice::from_raw_parts_mut(self.pointer, self.length) }
    }
}

impl Drop for PinnedWindow {
    fn drop(&mut self) {
        self.page.release_window();
    }
}

enum Window<B> {
    Kernel(B),
    User(PinnedWindow),
}

/// A writable window produced by [`IoSink::window`].
///
/// Dereferences to the window bytes. Dropping it releases the user-frame pin,
/// so keep the guard alive for exactly as long as the memory is in use.
pub struct SinkWindow<'a>(Window<&'a mut [u8]>);

impl SinkWindow<'_> {
    fn empty() -> Self {
        Self(Window::Kernel(&mut []))
    }
}

impl Deref for SinkWindow<'_> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        match &self.0 {
            Window::Kernel(buffer) => buffer,
            Window::User(window) => window.as_slice(),
        }
    }
}

impl DerefMut for SinkWindow<'_> {
    fn deref_mut(&mut self) -> &mut [u8] {
        match &mut self.0 {
            Window::Kernel(buffer) => buffer,
            Window::User(window) => window.as_mut_slice(),
        }
    }
}

/// A readable window produced by [`IoSource::window`].
///
/// Dereferences to the window bytes. Dropping it releases the user-frame pin,
/// so keep the guard alive for exactly as long as the memory is in use.
pub struct SourceWindow<'a>(Window<&'a [u8]>);

impl SourceWindow<'_> {
    fn empty() -> Self {
        Self(Window::Kernel(&[]))
    }
}

impl Deref for SourceWindow<'_> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        match &self.0 {
            Window::Kernel(buffer) => buffer,
            Window::User(window) => window.as_slice(),
        }
    }
}

/// Validated userspace range shared by source and sink wrappers.
struct UserRange<'a> {
    space: &'a VmSpace,
    address: VirtAddr,
    length: usize,
}

impl<'a> UserRange<'a> {
    fn new(space: &'a VmSpace, address: VirtAddr, length: usize) -> Result<Self> {
        space.validate_user(address, length)?;
        Ok(Self {
            space,
            address,
            length,
        })
    }

    fn slice(&self, offset: usize, length: usize) -> Result<UserRange<'_>> {
        checked_subrange(self.length, offset, length)?;
        let address = self
            .address
            .checked_add(offset as u64)
            .ok_or(Error::InvalidAddress)?;
        Ok(UserRange {
            space: self.space,
            address,
            length,
        })
    }

    fn window(&self, offset: usize, maximum: usize, access: FaultAccess) -> Result<PinnedWindow> {
        PinnedWindow::resolve(self.space, self.address, offset, maximum, access)
    }
}

fn checked_subrange(total: usize, offset: usize, length: usize) -> Result<core::ops::Range<usize>> {
    let end = offset.checked_add(length).ok_or(Error::InvalidAddress)?;
    (end <= total)
        .then_some(offset..end)
        .ok_or(Error::InvalidAddress)
}

fn window_length(total: usize, offset: usize, maximum: usize) -> Result<usize> {
    total
        .checked_sub(offset)
        .map(|available| maximum.min(available))
        .ok_or(Error::InvalidAddress)
}

enum Buffer<'a, B> {
    Kernel(B),
    User(UserRange<'a>),
}

/// Destination for bytes produced by a read-style operation.
pub struct IoSink<'a>(Buffer<'a, &'a mut [u8]>);

impl<'a> IoSink<'a> {
    /// Creates a sink targeting kernel memory.
    pub fn kernel(buffer: &'a mut [u8]) -> Self {
        Self(Buffer::Kernel(buffer))
    }

    /// Creates a sink targeting a validated user range.
    pub fn user(space: &'a VmSpace, address: VirtAddr, length: usize) -> Result<Self> {
        UserRange::new(space, address, length).map(|range| Self(Buffer::User(range)))
    }

    /// Returns the total transfer length.
    pub fn len(&self) -> usize {
        match &self.0 {
            Buffer::Kernel(buffer) => buffer.len(),
            Buffer::User(range) => range.length,
        }
    }

    /// Returns whether the transfer is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Borrows a shorter sink covering `offset..offset + length`.
    pub fn slice(&mut self, offset: usize, length: usize) -> Result<IoSink<'_>> {
        Ok(match &mut self.0 {
            Buffer::Kernel(buffer) => {
                let range = checked_subrange(buffer.len(), offset, length)?;
                IoSink(Buffer::Kernel(&mut buffer[range]))
            }
            Buffer::User(range) => IoSink(Buffer::User(range.slice(offset, length)?)),
        })
    }

    /// Returns a writable window of at most `maximum` bytes starting at `offset`.
    ///
    /// The window stops at the next page boundary, so callers must loop until
    /// the requested count is satisfied. The returned guard pins a user frame
    /// against reclamation; drop it as soon as the bytes are no longer in use.
    pub fn window(&mut self, offset: usize, maximum: usize) -> Result<SinkWindow<'_>> {
        let maximum = window_length(self.len(), offset, maximum)?;
        if maximum == 0 {
            return Ok(SinkWindow::empty());
        }
        match &mut self.0 {
            Buffer::Kernel(buffer) => Ok(SinkWindow(Window::Kernel(
                &mut buffer[offset..offset + maximum],
            ))),
            Buffer::User(range) => Ok(SinkWindow(Window::User(range.window(
                offset,
                maximum,
                FaultAccess::Write,
            )?))),
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
pub struct IoSource<'a>(Buffer<'a, &'a [u8]>);

impl<'a> IoSource<'a> {
    /// Creates a source backed by kernel memory.
    pub fn kernel(buffer: &'a [u8]) -> Self {
        Self(Buffer::Kernel(buffer))
    }

    /// Creates a source backed by a validated user range.
    pub fn user(space: &'a VmSpace, address: VirtAddr, length: usize) -> Result<Self> {
        UserRange::new(space, address, length).map(|range| Self(Buffer::User(range)))
    }

    /// Returns the total transfer length.
    pub fn len(&self) -> usize {
        match &self.0 {
            Buffer::Kernel(buffer) => buffer.len(),
            Buffer::User(range) => range.length,
        }
    }

    /// Returns whether the transfer is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Borrows a shorter source covering `offset..offset + length`.
    pub fn slice(&self, offset: usize, length: usize) -> Result<IoSource<'_>> {
        Ok(match &self.0 {
            Buffer::Kernel(buffer) => IoSource(Buffer::Kernel(
                &buffer[checked_subrange(buffer.len(), offset, length)?],
            )),
            Buffer::User(range) => IoSource(Buffer::User(range.slice(offset, length)?)),
        })
    }

    /// Returns a readable window of at most `maximum` bytes starting at `offset`.
    ///
    /// The window stops at the next page boundary, so callers must loop until
    /// the requested count is satisfied. The returned guard pins a user frame
    /// against reclamation; drop it as soon as the bytes are no longer in use.
    pub fn window(&self, offset: usize, maximum: usize) -> Result<SourceWindow<'_>> {
        let maximum = window_length(self.len(), offset, maximum)?;
        if maximum == 0 {
            return Ok(SourceWindow::empty());
        }
        match &self.0 {
            Buffer::Kernel(buffer) => Ok(SourceWindow(Window::Kernel(
                &buffer[offset..offset + maximum],
            ))),
            Buffer::User(range) => Ok(SourceWindow(Window::User(range.window(
                offset,
                maximum,
                FaultAccess::Read,
            )?))),
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
