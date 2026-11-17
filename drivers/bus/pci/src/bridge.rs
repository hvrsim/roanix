//! Conventional PCI bridge forwarding windows and ancestor constraints.
use super::{
    bar::{Bar, Kind},
    routing::Address,
};
use alloc::vec::Vec;
use core::ops::RangeInclusive;

#[derive(Clone, Debug)]
struct Aperture {
    kind: Kind,
    prefetchable: bool,
    span: RangeInclusive<u64>,
}

#[derive(Clone, Debug)]
pub struct Bridge {
    pub address: Address,
    pub secondary: u8,
    subordinate: u8,
    windows: Vec<Aperture>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Malformed,
    Unsupported,
}

pub trait Config {
    fn read32(&self, offset: u16) -> u32;
    fn write32(&mut self, offset: u16, value: u32);
    fn write16(&mut self, offset: u16, value: u16);
}

impl Bridge {
    /// Probe optional apertures with forwarding disabled. In particular, use
    /// word stores for I/O base/limit to avoid echoing secondary W1C status.
    pub fn probe(address: Address, config: &mut impl Config) -> Result<Self, Error> {
        let command = config.read32(4) as u16;
        config.write16(4, command & !3);
        let io = config.read32(0x1c) as u16;
        let prefetch = config.read32(0x24);
        let has_io = if io == 0 {
            config.write16(0x1c, 0xe0f0);
            let available = config.read32(0x1c) as u16 != 0;
            config.write16(0x1c, io);
            available
        } else {
            true
        };
        let has_prefetch = if prefetch == 0 {
            config.write32(0x24, 0xffe0_fff0);
            let available = config.read32(0x24) != 0;
            config.write32(0x24, prefetch);
            available
        } else {
            true
        };
        let result = Self::decode(
            address,
            |offset| config.read32(offset),
            has_io,
            has_prefetch,
        );
        config.write16(4, command);
        result
    }

    fn decode(
        address: Address,
        read: impl Fn(u16) -> u32,
        has_io: bool,
        has_prefetch: bool,
    ) -> Result<Self, Error> {
        if read(8) & 0xff00 != 0 {
            return Err(Error::Unsupported);
        }

        let buses = read(0x18);
        let mut result = Self {
            address,
            secondary: (buses >> 8) as u8,
            subordinate: (buses >> 16) as u8,
            windows: Vec::new(),
        };
        // A bridge with no secondary bus is present but not configured.
        if result.secondary == 0 {
            return Ok(result);
        }
        if buses as u8 != address.bus || result.subordinate < result.secondary {
            return Err(Error::Malformed);
        }
        let io = read(0x1c);
        let io_type = io & 15;
        if io_type > 1 || (io >> 8) & 15 != io_type {
            return Err(Error::Malformed);
        }
        let upper = if io_type == 1 { read(0x30) } else { 0 };
        if has_io {
            result.add(
                Kind::Io,
                false,
                u64::from(((io & 0xf0) << 8) | ((upper & 0xffff) << 16)),
                u64::from((io & 0xf000) | (upper & 0xffff_0000) | 0xfff),
            );
        }
        let memory = read(0x20);
        result.add(
            Kind::Memory,
            false,
            u64::from((memory & 0xfff0) << 16),
            u64::from(memory & 0xfff0_0000) | 0xf_ffff,
        );
        let prefetch = read(0x24);
        let prefetch_type = prefetch & 15;
        if prefetch_type > 1 || (prefetch >> 16) & 15 != prefetch_type {
            return Err(Error::Malformed);
        }
        let (base_upper, limit_upper) = if prefetch_type == 1 {
            (read(0x28), read(0x2c))
        } else {
            (0, 0)
        };
        if has_prefetch {
            result.add(
                Kind::Memory,
                true,
                (u64::from(prefetch & 0xfff0) << 16) | (u64::from(base_upper) << 32),
                u64::from(prefetch & 0xfff0_0000) | 0xf_ffff | (u64::from(limit_upper) << 32),
            );
        }
        Ok(result)
    }

    fn add(&mut self, kind: Kind, prefetchable: bool, base: u64, limit: u64) {
        if base <= limit {
            self.windows.push(Aperture {
                kind,
                prefetchable,
                span: base..=limit,
            });
        }
    }

    pub fn forwards(&self, bar: Bar) -> bool {
        bar.base != 0
            && bar.size != 0
            && bar.base.checked_add(bar.size - 1).is_some_and(|last| {
                self.windows.iter().any(|window| {
                    window.accepts(bar)
                        && window.span.contains(&bar.base)
                        && window.span.contains(&last)
                })
            })
    }
}

impl Aperture {
    fn accepts(&self, bar: Bar) -> bool {
        self.kind == bar.kind && (bar.prefetchable || !self.prefetchable)
    }
}

/// FDT supplies the root bus; MCFG describes configuration coverage and may
/// split one hierarchy across entries, so ACPI traversal ends at the top bridge.
pub fn path(root: Option<u8>, address: Address, bridges: &[Bridge]) -> Result<Vec<Bridge>, Error> {
    let mut target = address;
    let mut path = Vec::new();
    for _ in 0..256 {
        if root == Some(target.bus) {
            return Ok(path);
        }
        let mut parents = bridges.iter().filter(|bridge| {
            bridge.address.segment == target.segment
                && bridge.secondary != 0
                && bridge.secondary == target.bus
        });
        let Some(parent) = parents.next() else {
            return if root.is_none() {
                Ok(path)
            } else {
                Err(Error::Malformed)
            };
        };
        if parents.next().is_some()
            || !(parent.secondary..=parent.subordinate).contains(&address.bus)
        {
            return Err(Error::Malformed);
        }
        path.push(parent.clone());
        target = parent.address;
    }
    Err(Error::Malformed)
}

/// Siblings cannot both claim the same configuration bus or positive-decode
/// address. Ancestor/descendant windows intentionally contain one another.
pub fn validate(bridges: &[Bridge]) -> Result<(), Error> {
    for (index, bridge) in bridges.iter().enumerate() {
        if bridge.secondary == 0 {
            continue;
        }
        for other in &bridges[..index] {
            if other.secondary == 0
                || bridge.address.segment != other.address.segment
                || bridge.address.bus != other.address.bus
            {
                continue;
            }
            if bridge.secondary <= other.subordinate && other.secondary <= bridge.subordinate {
                return Err(Error::Malformed);
            }
            for left in &bridge.windows {
                if other.windows.iter().any(|right| {
                    left.kind == right.kind
                        && left.span.start() <= right.span.end()
                        && right.span.start() <= left.span.end()
                }) {
                    return Err(Error::Malformed);
                }
            }
        }
    }
    Ok(())
}

/// Intersect each ancestor's eligible apertures. Merge aliases at every step
/// so overlapping memory/prefetch windows cannot multiply the result size.
pub fn limits(path: &[Bridge], bar: Bar) -> Vec<RangeInclusive<u64>> {
    let mut spans = alloc::vec![0..=u64::MAX];
    for bridge in path {
        let mut next = Vec::new();
        for span in &spans {
            for window in bridge.windows.iter().filter(|window| window.accepts(bar)) {
                let base = (*span.start()).max(*window.span.start());
                let limit = (*span.end()).min(*window.span.end());
                if base <= limit {
                    next.push(base..=limit);
                }
            }
        }
        next.sort_unstable_by_key(|span| *span.start());
        spans.clear();
        for span in next {
            if let Some(previous) = spans.last_mut() {
                if previous.end().saturating_add(1) >= *span.start() {
                    *previous = *previous.start()..=(*previous.end()).max(*span.end());
                    continue;
                }
            }
            spans.push(span);
        }
    }
    spans
}
