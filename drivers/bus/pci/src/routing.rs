//! PCI INTx traversal, independent of configuration access and firmware format.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Address {
    pub segment: u16,
    pub bus: u8,
    pub device: u8,
    pub function: u8,
}

pub enum Lookup<R> {
    Inherited,
    // A present map is authoritative even when it has no matching route.
    Mapped(Option<R>),
}

#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    InvalidTopology,
    Firmware(E),
}

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
