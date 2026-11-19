//! Resolves PCI INTx interrupts through firmware maps and upstream bridges.

/// Configuration address of one PCI function.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Address {
    /// PCI segment containing this bus.
    pub segment: u16,

    /// Bus number within the segment.
    pub bus: u8,

    /// Device slot on the bus, from 0 through 31.
    pub device: u8,

    /// Function within the device, from 0 through 7.
    pub function: u8,
}

/// Result of consulting a firmware interrupt map at one level of the hierarchy.
pub enum Lookup<R> {
    /// Continue looking for a route at the next upstream bridge.
    Inherited,

    /// An authoritative map result. `None` means the map contains no matching route.
    Mapped(Option<R>),
}

/// Failure to resolve a PCI INTx route.
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// The bridge path is ambiguous or cannot be traversed within the bus limit.
    InvalidTopology,

    /// The firmware lookup failed.
    Firmware(E),
}

/// Resolves an INTx pin through firmware maps, swizzling at bridges that inherit routing.
pub fn resolve<R, E>(
    root_bus: u8,
    mut target: Address,
    mut pin: u8,
    bridges: &[(Address, u8)],
    route: impl Fn(Address, u8, Option<Address>) -> Result<Lookup<R>, E>,
) -> Result<Option<R>, Error<E>> {
    if !(1..=4).contains(&pin) {
        return Ok(None);
    }

    for _ in 0..256 {
        if target.bus == root_bus {
            return route(target, pin, None)
                .map(|lookup| match lookup {
                    Lookup::Inherited => None,
                    Lookup::Mapped(route) => route,
                })
                .map_err(Error::Firmware);
        }

        let mut parents = bridges
            .iter()
            .filter(|(parent, bus)| parent.segment == target.segment && *bus == target.bus);
        let Some((parent, _)) = parents.next() else {
            return Ok(None);
        };

        if parents.next().is_some() {
            return Err(Error::InvalidTopology);
        }

        // An explicit bridge nexus consumes the child's unswizzled pin.
        if let Lookup::Mapped(mapped) =
            route(target, pin, Some(*parent)).map_err(Error::Firmware)?
        {
            return Ok(mapped);
        }

        pin = (pin - 1 + target.device) % 4 + 1;
        target = *parent;
    }

    Err(Error::InvalidTopology)
}
