//! Validated ECAM ownership and address calculation.
use super::routing::Address;

#[derive(Clone, Copy, Debug)]
pub struct ConfigRange {
    pub segment: u16,
    pub start_bus: u8,
    pub end_bus: u8,
    pub physical: u64,
    pub length: usize,
}

impl ConfigRange {
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

    // MCFG's base describes bus zero, whereas FDT reg starts at bus-range[0].
    #[cfg_attr(target_arch = "riscv64", allow(dead_code))]
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

    pub fn conflicts(self, other: Self) -> bool {
        let last = self.physical + (self.length as u64 - 1);
        let other_last = other.physical + (other.length as u64 - 1);
        (self.segment == other.segment
            && self.start_bus <= other.end_bus
            && other.start_bus <= self.end_bus)
            || (self.physical <= other_last && other.physical <= last)
    }
}
