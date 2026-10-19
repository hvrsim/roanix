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
//! # User memory
//!
//! User frames remain externally mutable while pinned. Windows expose copy
//! operations, never Rust references to those bytes. Callers needing a slice
//! ABI use bounded kernel storage and explicitly copy the transferred bytes.
//! A writable pin also preserves the fork decision for its entire lifetime.

use core::ptr;

use super::{
    Error, PAGE_SIZE, Result, VirtAddr,
    map::FaultAccess,
    pmap::{UserPagePin, VmSpace},
};

/// Wired, page-bounded HHDM alias shared by readable and writable guards.
struct PinnedWindow<'a> {
    _pin: UserPagePin<'a>,
    pointer: *mut u8,
    length: usize,
}

impl<'a> PinnedWindow<'a> {
    /// Faults and wires one user page without allocating or copying payload data.
    fn resolve(
        space: &'a VmSpace,
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
            _pin: page,
            pointer: super::phys_to_virt(physical).as_mut_ptr(),
            length: maximum.min(remaining),
        })
    }
}

enum Window<'a, B> {
    Kernel(B),
    User(PinnedWindow<'a>),
}

/// A writable, page-bounded transfer window. Dropping it releases its pin.
/// User bytes are accessible only through copy operations.
pub struct SinkWindow<'a>(Window<'a, &'a mut [u8]>);

impl SinkWindow<'_> {
    fn empty() -> Self {
        Self(Window::Kernel(&mut []))
    }

    /// Returns the number of available bytes.
    pub fn len(&self) -> usize {
        match &self.0 {
            Window::Kernel(buffer) => buffer.len(),
            Window::User(window) => window.length,
        }
    }

    /// Returns whether this window is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Copies a prefix into the window; untransferred bytes are untouched.
    pub fn copy_from_slice(&mut self, source: &[u8]) {
        assert!(source.len() <= self.len());
        match &mut self.0 {
            Window::Kernel(buffer) => buffer[..source.len()].copy_from_slice(source),
            Window::User(window) => {
                // SAFETY: the pin retains a writable frame for this bounded
                // transfer. No Rust reference to user memory is constructed.
                unsafe { ptr::copy(source.as_ptr(), window.pointer, source.len()) };
            }
        }
    }

    /// Fills the entire window with `value`.
    pub fn fill(&mut self, value: u8) {
        match &mut self.0 {
            Window::Kernel(buffer) => buffer.fill(value),
            Window::User(window) => {
                // SAFETY: the writable pin retains all `length` bytes.
                unsafe { ptr::write_bytes(window.pointer, value, window.length) };
            }
        }
    }

    pub(super) fn read_page(&mut self, page: &super::PageRef, offset: usize) -> Result<()> {
        match &mut self.0 {
            Window::Kernel(buffer) => page.read(offset, buffer),
            Window::User(window) => {
                // SAFETY: the pin retains the complete destination window;
                // read_raw handles overlap with the source page itself.
                unsafe { page.read_raw(offset, window.pointer, window.length) }
            }
        }
    }
}

/// A readable, page-bounded transfer window. Dropping it releases its pin.
/// User bytes are accessible only through copy operations.
pub struct SourceWindow<'a>(Window<'a, &'a [u8]>);

impl SourceWindow<'_> {
    fn empty() -> Self {
        Self(Window::Kernel(&[]))
    }

    /// Returns the number of available bytes.
    pub fn len(&self) -> usize {
        match &self.0 {
            Window::Kernel(buffer) => buffer.len(),
            Window::User(window) => window.length,
        }
    }

    /// Returns whether this window is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Copies a prefix of this window into a kernel buffer.
    pub fn copy_to_slice(&self, destination: &mut [u8]) {
        assert!(destination.len() <= self.len());
        match &self.0 {
            Window::Kernel(buffer) => destination.copy_from_slice(&buffer[..destination.len()]),
            Window::User(window) => {
                // SAFETY: the pin retains a readable frame for this bounded
                // transfer. No Rust reference to user memory is constructed.
                unsafe { ptr::copy(window.pointer, destination.as_mut_ptr(), destination.len()) };
            }
        }
    }

    pub(super) fn write_page(&self, page: &super::PageRef, offset: usize) -> Result<()> {
        match &self.0 {
            Window::Kernel(buffer) => page.write(offset, buffer),
            Window::User(window) => {
                // SAFETY: the pin retains the complete source window;
                // write_raw handles overlap with the destination page itself.
                unsafe { page.write_raw(offset, window.pointer, window.length) }
            }
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

    fn window(
        &self,
        offset: usize,
        maximum: usize,
        access: FaultAccess,
    ) -> Result<PinnedWindow<'_>> {
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
        checked_subrange(self.len(), offset, source.len())?;
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
        checked_subrange(self.len(), offset, length)?;
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
        checked_subrange(self.len(), offset, destination.len())?;
        let mut read = 0usize;
        while read < destination.len() {
            let window = self.window(offset + read, destination.len() - read)?;
            if window.is_empty() {
                return Err(Error::InvalidAddress);
            }
            let count = window.len();
            window.copy_to_slice(&mut destination[read..read + count]);
            read += count;
        }
        Ok(())
    }
}
