//! Register windows, port I/O, and DMA buffers.

pub mod dma;
pub mod mmio;
pub mod port;

/// Initializes the I/O services.
pub(super) fn init() {
    mmio::init();
    dma::init();
}
