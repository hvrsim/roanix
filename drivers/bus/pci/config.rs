//! Validates ECAM ranges and calculates configuration offsets for PCI functions.

use super::routing::Address;

/// A contiguous ECAM mapping for a range of buses in one PCI segment.
#[derive(Clone, Copy, Debug)]
pub struct ConfigRange {
    /// PCI segment whose configuration space this mapping covers.
    pub segment: u16,

    /// First bus backed by this mapping.
    pub start_bus: u8,

    /// Last bus backed by this mapping, inclusive.
    pub end_bus: u8,

    /// Physical base of the first bus's configuration space.
    pub physical: u64,

    /// Mapped bytes required to cover all buses in the range.
    pub length: usize,
}

impl ConfigRange {
    /// Validates an aligned ECAM region and trims its length to the advertised bus range.
    pub fn new(
        physical: u64,
        available: u64,
        segment: u16,
        start_bus: u8,
        end_bus: u8,
    ) -> Option<Self> {
        if start_bus > end_bus || physical == 0 || physical & ((1 << 20) - 1) != 0 {
            return None;
        }

        let length = (usize::from(end_bus - start_bus) + 1) << 20;
        if available < length as u64 || physical.checked_add(length as u64 - 1).is_none() {
            return None;
        }

        Some(Self {
            segment,
            start_bus,
            end_bus,
            physical,
            length,
        })
    }

    /// Converts an ACPI bus-zero ECAM base to the first bus in this allocation.
    #[cfg(target_arch = "x86_64")]
    pub fn mcfg(base: u64, segment: u16, start: u8, end: u8) -> Option<Self> {
        if base == 0 {
            return None;
        }

        Self::new(
            base.checked_add(u64::from(start) << 20)?,
            u64::MAX,
            segment,
            start,
            end,
        )
    }

    /// Returns the byte offset of a function within this ECAM allocation.
    pub fn offset(self, address: Address) -> Option<usize> {
        if self.segment != address.segment
            || !(self.start_bus..=self.end_bus).contains(&address.bus)
            || address.device >= 32
            || address.function >= 8
        {
            return None;
        }

        Some(
            (usize::from(address.bus - self.start_bus) << 20)
                | (usize::from(address.device) << 15)
                | (usize::from(address.function) << 12),
        )
    }

    /// Checks for overlapping bus identities in one segment or overlapping physical mappings.
    pub fn conflicts(self, other: Self) -> bool {
        let last = self.physical + (self.length as u64 - 1);
        let other_last = other.physical + (other.length as u64 - 1);

        (self.segment == other.segment
            && self.start_bus <= other.end_bus
            && other.start_bus <= self.end_bus)
            || (self.physical <= other_last && other.physical <= last)
    }
}
