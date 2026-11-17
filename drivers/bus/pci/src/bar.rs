//! BAR sizing independent of configuration transport and address allocation.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Kind {
    #[default]
    Memory,
    Io,
}
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Bar {
    pub kind: Kind,
    pub base: u64,
    pub size: u64,
    pub address_bits: u8,
    pub prefetchable: bool,
}
impl Bar {
    pub fn flags(self) -> u32 {
        (u32::from(self.prefetchable) * 2) | if self.address_bits == 64 { 8 } else { 0 }
    }
    pub fn fits(self, base: u64) -> bool {
        let limit = if self.address_bits == 64 {
            u64::MAX
        } else {
            (1u64 << self.address_bits) - 1
        };
        self.size != 0
            && base.is_multiple_of(self.size)
            && base
                .checked_add(self.size - 1)
                .is_some_and(|end| end <= limit)
    }
}
pub trait Config {
    fn read(&self, index: usize) -> u32;
    fn write(&mut self, index: usize, value: u32);
}
pub struct Probe {
    /// Includes the upper slot even when a 64-bit BAR has no usable resource.
    pub slots: usize,
    pub bar: Option<Bar>,
}
/// Decode must be disabled by the caller while probing. All touched BAR words
/// are restored before returning, including malformed and unimplemented cases.
pub fn probe(config: &mut impl Config, index: usize, count: usize) -> Probe {
    let empty = |slots| Probe { slots, bar: None };
    if index >= count {
        return empty(1);
    }
    let low = config.read(index);
    let io = low & 1 != 0;
    let wide = !io && low & 6 == 4;
    let slots = if wide { 2 } else { 1 };
    if (wide && index + 1 >= count) || (!io && low & 6 == 6) || low == u32::MAX {
        return empty(1);
    }
    let high = if wide { config.read(index + 1) } else { 0 };
    config.write(index, u32::MAX);
    if wide {
        config.write(index + 1, u32::MAX);
    }
    let mask_low = config.read(index);
    let mask_high = if wide { config.read(index + 1) } else { 0 };
    if wide {
        config.write(index + 1, high);
    }
    config.write(index, low);
    if mask_low == u32::MAX {
        return empty(slots);
    }
    let address_bits = if io || low & 6 == 0 {
        32
    } else if wide {
        64
    } else {
        20
    };
    let address_mask = if io { !3u32 } else { !15u32 };
    let base = (u64::from(high) << 32) | u64::from(low & address_mask);
    let mask = (u64::from(mask_high) << 32) | u64::from(mask_low & address_mask);
    if mask == 0 {
        return empty(slots);
    }
    // The least significant implemented address bit gives the decode size.
    // Leading zero bits are allowed for implementations with 16-bit I/O decode.
    let size = mask & mask.wrapping_neg();
    let ceiling = mask.wrapping_add(size);
    if ceiling != 0 && !ceiling.is_power_of_two() {
        return empty(slots);
    }
    let bar = Bar {
        kind: if io { Kind::Io } else { Kind::Memory },
        base,
        size,
        address_bits,
        prefetchable: !io && low & 8 != 0,
    };
    Probe {
        slots,
        bar: bar.fits(base).then_some(bar),
    }
}
