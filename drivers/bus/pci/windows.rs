//! Translates PCI bus addresses and tracks BAR reservations within host windows.

use super::bar::{Bar, Kind};
use alloc::vec::Vec;

/// Failure to describe a host window or reserve a BAR range.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// A window or BAR has an invalid size, address, or alignment.
    Invalid,

    /// Windows translate overlapping addresses differently, or BAR reservations collide.
    Overlap,
}

/// A host bridge window that translates a contiguous PCI range to CPU addresses.
#[derive(Clone, Copy, Debug)]
pub struct Window {
    /// PCI address space covered by the window.
    pub kind: Kind,

    /// Resource attributes, including the prefetchable memory flag.
    pub flags: u32,

    /// First address on the PCI bus side.
    pub bus: u64,

    /// First address on the CPU side.
    pub cpu: u64,

    /// Length in bytes on both sides of the mapping.
    pub size: u64,
}

/// Computes the last byte of a nonempty range if it fits in the address space.
fn end(start: u64, size: u64) -> Option<u64> {
    start.checked_add(size.checked_sub(1)?)
}

/// Checks whether two already validated ranges share a byte.
fn overlaps(a: u64, a_size: u64, b: u64, b_size: u64) -> bool {
    a <= end(b, b_size).expect("validated range") && b <= end(a, a_size).expect("validated range")
}

impl Window {
    /// Validates a nonempty host window whose bus and CPU ranges fit in 64 bits.
    #[cfg_attr(target_arch = "x86_64", allow(dead_code))]
    pub fn new(kind: Kind, flags: u32, bus: u64, cpu: u64, size: u64) -> Result<Self, Error> {
        end(bus, size).ok_or(Error::Invalid)?;
        end(cpu, size).ok_or(Error::Invalid)?;

        if kind == Kind::Io && end(bus, size).is_none_or(|last| last > u64::from(u32::MAX)) {
            return Err(Error::Invalid);
        }

        Ok(Self {
            kind,
            flags,
            bus,
            cpu,
            size,
        })
    }

    /// Maps a range wholly contained in this window into the CPU address space.
    fn translate(self, start: u64, size: u64) -> Option<u64> {
        if size == 0 {
            return None;
        }

        let offset = start.checked_sub(self.bus)?;
        if end(offset, size)? >= self.size {
            return None;
        }

        self.cpu.checked_add(offset)
    }
}

/// A bus address range claimed by one BAR.
#[derive(Clone)]
struct Reservation {
    /// PCI function and BAR slot encoded as a single key.
    key: u64,

    /// Address space in which the range is reserved.
    kind: Kind,

    /// First reserved bus address.
    bus: u64,

    /// Number of reserved bytes.
    size: u64,
}

/// A host's address translation windows and active BAR reservations.
#[derive(Clone)]
pub struct AddressSpace {
    /// Whether assigned bus addresses are already CPU addresses.
    identity: bool,

    /// Explicit translation and allocation windows for this host.
    windows: Vec<Window>,

    /// Current reservations, including firmware-assigned BARs.
    used: Vec<Reservation>,
}

impl AddressSpace {
    /// Uses firmware-assigned addresses directly. Without an allocation
    /// aperture, unassigned BARs remain unassigned.
    pub const fn identity() -> Self {
        Self {
            identity: true,
            windows: Vec::new(),
            used: Vec::new(),
        }
    }

    /// Builds the host address space after checking for conflicting windows.
    #[cfg_attr(target_arch = "x86_64", allow(dead_code))]
    pub fn new(windows: Vec<Window>) -> Result<Self, Error> {
        for (index, window) in windows.iter().enumerate() {
            Window::new(
                window.kind,
                window.flags,
                window.bus,
                window.cpu,
                window.size,
            )?;

            for other in &windows[..index] {
                if window.kind == other.kind
                    && overlaps(window.bus, window.size, other.bus, other.size)
                {
                    let shared = window.bus.max(other.bus);
                    if window.cpu + (shared - window.bus) != other.cpu + (shared - other.bus) {
                        return Err(Error::Overlap);
                    }
                }

                if overlaps(window.cpu, window.size, other.cpu, other.size) {
                    let shared = window.cpu.max(other.cpu);
                    let left = window
                        .bus
                        .checked_add(shared - window.cpu)
                        .ok_or(Error::Invalid)?;
                    let right = other
                        .bus
                        .checked_add(shared - other.cpu)
                        .ok_or(Error::Invalid)?;
                    if window.kind != other.kind || left != right {
                        return Err(Error::Overlap);
                    }
                }
            }
        }

        Ok(Self {
            identity: false,
            windows,
            used: Vec::new(),
        })
    }

    /// Reserves an existing BAR assignment and checks its bus and CPU ranges for collisions.
    pub fn reserve(&mut self, key: u64, bar: Bar) -> Result<(), Error> {
        if bar.size == 0 || bar.base == 0 || !bar.fits(bar.base) {
            return Err(Error::Invalid);
        }

        if self.used.iter().any(|used| {
            used.key != key
                && used.kind == bar.kind
                && overlaps(used.bus, used.size, bar.base, bar.size)
        }) {
            return Err(Error::Overlap);
        }

        self.used.retain(|used| used.key != key);
        self.used.push(Reservation {
            key,
            kind: bar.kind,
            bus: bar.base,
            size: bar.size,
        });
        Ok(())
    }

    /// Frees the host-window reservation associated with a BAR key.
    pub fn release(&mut self, key: u64) {
        self.used.retain(|used| used.key != key);
    }

    /// Checks whether a translated range would alias an existing reservation.
    fn cpu_overlaps(&self, kind: Kind, start: u64, size: u64) -> bool {
        if self.identity {
            self.used
                .iter()
                .any(|used| used.kind == kind && overlaps(start, size, used.bus, used.size))
        } else {
            self.windows
                .iter()
                .any(|window| overlaps(start, size, window.cpu, window.size))
        }
    }

    /// Different hosts must own disjoint CPU apertures. Identity hosts have no
    /// allocation aperture, so only their firmware-assigned spans are checked.
    pub fn conflicts(&self, other: &Self) -> bool {
        if self.identity {
            self.used
                .iter()
                .any(|used| other.cpu_overlaps(used.kind, used.bus, used.size))
        } else {
            self.windows
                .iter()
                .any(|window| other.cpu_overlaps(window.kind, window.cpu, window.size))
        }
    }

    /// Finds the CPU address for an assigned BAR covered by a compatible host window.
    pub fn translate(&self, bar: Bar) -> Option<u64> {
        if bar.base == 0 || bar.size == 0 || !bar.fits(bar.base) {
            return None;
        }

        if self.identity {
            return Some(bar.base);
        }

        for preferred in [true, false] {
            for window in &self.windows {
                let prefetch = window.flags & 2 != 0;
                if window.kind != bar.kind || (!bar.prefetchable && prefetch) {
                    continue;
                }

                if preferred && prefetch != bar.prefetchable {
                    continue;
                }

                if let Some(cpu) = window.translate(bar.base, bar.size) {
                    return Some(cpu);
                }
            }
        }

        None
    }

    /// Reserves aligned BAR space within both the host window and bridge forwarding limits.
    pub fn allocate_within(
        &mut self,
        key: u64,
        mut bar: Bar,
        limits: &[core::ops::RangeInclusive<u64>],
    ) -> Option<u64> {
        if bar.size == 0
            || !bar.size.is_power_of_two()
            || self.used.iter().any(|used| used.key == key)
        {
            return None;
        }

        for preferred in [true, false] {
            for window in &self.windows {
                let prefetch = window.flags & 2 != 0;
                if window.kind != bar.kind || (!bar.prefetchable && prefetch) {
                    continue;
                }

                if preferred && prefetch != bar.prefetchable {
                    continue;
                }

                for limit in limits {
                    // Address zero is the unassigned BAR sentinel.
                    let mut candidate = window.bus.max(*limit.start()).max(1);

                    loop {
                        candidate = match candidate.checked_add(bar.size - 1) {
                            Some(value) => value & !(bar.size - 1),
                            None => break,
                        };

                        if !bar.fits(candidate)
                            || window.translate(candidate, bar.size).is_none()
                            || candidate
                                .checked_add(bar.size - 1)
                                .is_none_or(|last| last > *limit.end())
                        {
                            break;
                        }

                        let obstruction = self
                            .used
                            .iter()
                            .filter(|used| {
                                used.kind == bar.kind
                                    && overlaps(candidate, bar.size, used.bus, used.size)
                            })
                            .map(|used| end(used.bus, used.size).expect("reservation range"))
                            .max();

                        if let Some(last) = obstruction {
                            let Some(next) = last.checked_add(1) else {
                                break;
                            };
                            candidate = next;
                        } else {
                            bar.base = candidate;
                            self.reserve(key, bar).ok()?;
                            return Some(candidate);
                        }
                    }
                }
            }
        }

        None
    }
}
