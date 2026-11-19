//!
//! # Flattened Device Tree
//!
//! The parser borrows names and property bytes from a firmware blob and
//! builds a parent-first list of nodes. It validates property widths,
//! address ranges, and interrupt routes as callers decode them.
//!
//! Address translation walks ancestor buses and applies each bus window.
//! Interrupt lookup follows firmware maps while bounding the number of
//! hops. Callers decide which decoded nodes and resources to publish.
//!

use alloc::vec::Vec;
use core::ffi::CStr;

/// Failure to decode a firmware record or interpret its layout.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// Invalid firmware encoding, inconsistent references, or a range outside the blob.
    Malformed,

    /// A well-formed feature or address width the parser cannot represent.
    Unsupported,
}

/// The result type used by device tree decoding.
pub type Result<T> = core::result::Result<T, Error>;

/// Address space encoded by a device tree bus.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Space {
    /// No address cells were supplied.
    Unspecified,

    /// Ordinary address space without a PCI selector.
    Native,

    /// PCI I/O port space.
    Io,

    /// PCI memory space.
    Memory,

    /// PCI configuration space.
    Configuration,
}

/// Decoded bus address and the width available for its range.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Address {
    /// Address space selected by the bus encoding.
    pub space: Space,

    /// Decoded numeric bus address.
    pub value: u64,

    /// Address width used to check range bounds.
    bits: u8,
}

impl Address {
    fn covers(self, size: u64) -> bool {
        size == 0
            || self.value.checked_add(size - 1).is_some_and(|end| {
                end <= if self.bits == 32 {
                    u64::from(u32::MAX)
                } else {
                    u64::MAX
                }
            })
    }
}

/// A node's register range and optional firmware name.
#[derive(Clone, Copy)]
pub struct Register<'a> {
    /// Starting address in the parent bus domain.
    pub address: Address,

    /// Register span in bytes.
    pub size: u64,

    /// Optional name from the node's register-name list.
    pub name: Option<&'a CStr>,
}

/// Address translation window from a child bus to its parent.
#[derive(Clone, Copy)]
pub struct Range {
    /// Starting address in the child bus.
    pub child: Address,

    /// Corresponding address in the parent bus.
    pub parent: Address,

    /// Number of bytes covered by the translation.
    pub size: u64,

    /// Decoded bus window attributes such as PCI prefetchability.
    pub flags: u32,
}

/// Terminal controller identity and specifier after interrupt-nexus translation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InterruptRoute {
    /// Firmware identity of the controller that receives this specifier.
    pub phandle: u32,

    /// Controller-specific interrupt specifier.
    pub cells: Vec<u32>,
}

/// Decodes interrupt specifier bytes into big-endian 32-bit cells.
fn irq_cells(bytes: &[u8]) -> Result<Vec<u32>> {
    if !bytes.len().is_multiple_of(4) || bytes.len() > 16384 {
        return Err(Error::Malformed);
    }

    Ok(bytes
        .chunks_exact(4)
        .map(|cell| u32::from_be_bytes(cell.try_into().expect("cell width")))
        .collect())
}

/// Borrowed name and value from the firmware blob.
struct Property<'a> {
    /// Property name borrowed from the string block.
    name: &'a CStr,

    /// Property value borrowed from the structure block.
    value: &'a [u8],
}

/// A device tree node with its effective enable state and borrowed properties.
pub struct Node<'a> {
    /// Node name borrowed from the firmware blob.
    pub name: &'a CStr,

    /// Index of the parent in the parsed node list.
    pub parent: Option<usize>,

    /// Whether this node and its ancestors permit publication.
    pub enabled: bool,

    /// Properties attached directly to this node.
    properties: Vec<Property<'a>>,
}
impl<'a> Node<'a> {
    /// Returns the raw bytes of the named device-tree property.
    #[must_use]
    pub fn bytes(&self, name: &CStr) -> Option<&'a [u8]> {
        self.properties
            .iter()
            .find(|p| p.name == name)
            .map(|p| p.value)
    }

    /// Returns a scalar property.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn cell(&self, name: &CStr) -> Result<Option<u32>> {
        self.bytes(name)
            .map(|bytes| {
                let bytes: [u8; 4] = bytes.try_into().map_err(|_| Error::Malformed)?;
                Ok(u32::from_be_bytes(bytes))
            })
            .transpose()
    }

    /// Returns the bounded strings in a property.
    ///
    /// # Errors
    /// Rejects a property whose last string lacks a terminating NUL.
    pub fn strings(&self, name: &CStr) -> Result<Vec<&'a CStr>> {
        let mut result = Vec::new();
        if let Some(mut bytes) = self.bytes(name) {
            while !bytes.is_empty() {
                let string = CStr::from_bytes_until_nul(bytes).map_err(|_| Error::Malformed)?;
                result.push(string);
                bytes = &bytes[string.to_bytes_with_nul().len()..];
            }
        }

        Ok(result)
    }

    /// Returns this node’s canonical firmware identity.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn phandle(&self) -> Result<Option<u32>> {
        let first = self.cell(c"phandle")?;
        let second = self.cell(c"linux,phandle")?;
        if first.zip(second).is_some_and(|(a, b)| a != b) {
            return Err(Error::Malformed);
        }

        let value = first.or(second);
        if value.is_some_and(|v| v == 0 || v == u32::MAX) {
            return Err(Error::Malformed);
        }

        Ok(value)
    }

    /// Returns the address and size cell widths used for this node's children.
    fn widths(&self) -> Result<(usize, usize)> {
        let address = self.cell(c"#address-cells")?.unwrap_or(2);
        let size = self.cell(c"#size-cells")?.unwrap_or(1);
        if address > 4 || size > 2 {
            return Err(Error::Unsupported);
        }

        Ok((address as usize, size as usize))
    }

    fn pci(&self) -> bool {
        self.bytes(c"device_type") == Some(b"pci\0")
    }
}

/// Parsed device tree in parent-first order.
pub struct Tree<'a> {
    /// Decoded nodes in parent-first order.
    nodes: Vec<Node<'a>>,
}

/// Reads a bounded big-endian word from an offset in the blob.
fn word(bytes: &[u8], offset: usize) -> Result<u32> {
    let end = offset.checked_add(4).ok_or(Error::Malformed)?;
    let word = bytes.get(offset..end).ok_or(Error::Malformed)?;
    Ok(u32::from_be_bytes(
        word.try_into().map_err(|_| Error::Malformed)?,
    ))
}

/// Rounds a structure-block offset up to the next four-byte token boundary.
fn aligned(value: usize) -> Result<usize> {
    value.checked_add(3).map(|v| v & !3).ok_or(Error::Malformed)
}

/// Reads a NUL-terminated device-tree string and returns the following offset.
fn nul(bytes: &[u8], offset: usize) -> Result<(&CStr, usize)> {
    let remaining = bytes.get(offset..).ok_or(Error::Malformed)?;
    let name = CStr::from_bytes_until_nul(remaining).map_err(|_| Error::Malformed)?;
    name.to_str().map_err(|_| Error::Malformed)?;
    Ok((name, offset + name.to_bytes_with_nul().len()))
}

/// Decodes a multi-cell integer when its value fits in 64 bits.
fn number(bytes: &[u8], cells: usize) -> Result<u64> {
    if bytes.len() != cells * 4 {
        return Err(Error::Malformed);
    }

    let mut value = 0u64;
    for part in bytes.chunks_exact(4) {
        if value > u64::from(u32::MAX) {
            return Err(Error::Unsupported);
        }
        value = (value << 32)
            | u64::from(u32::from_be_bytes(
                part.try_into().map_err(|_| Error::Malformed)?,
            ));
    }

    Ok(value)
}

/// Decodes an address with the parent bus's layout, including PCI space flags.
fn address(bus: &Node<'_>, bytes: &[u8], cells: usize) -> Result<Address> {
    if cells == 0 {
        return Ok(Address {
            space: Space::Unspecified,
            value: 0,
            bits: 0,
        });
    }

    if bus.pci() && cells == 3 {
        let flags = word(bytes, 0)?;
        let space = match flags & 0x0300_0000 {
            0 => Space::Configuration,
            0x0100_0000 => Space::Io,
            _ => Space::Memory,
        };

        Ok(Address {
            space,
            value: number(bytes.get(4..).ok_or(Error::Malformed)?, 2)?,
            bits: if flags & 0x0300_0000 == 0x0300_0000 {
                64
            } else {
                32
            },
        })
    } else {
        Ok(Address {
            space: Space::Native,
            value: number(bytes, cells)?,
            bits: if cells == 1 { 32 } else { 64 },
        })
    }
}
fn blob_blocks(blob: &[u8]) -> Result<(&[u8], &[u8])> {
    if blob.len() < 40 || word(blob, 0)? != 0xd00d_feed {
        return Err(Error::Malformed);
    }

    let total = word(blob, 4)? as usize;
    let blob = blob.get(..total).ok_or(Error::Malformed)?;
    let version = word(blob, 20)?;
    if version < 17 || word(blob, 24)? > 17 {
        return Err(Error::Unsupported);
    }

    let structure = word(blob, 8)? as usize;
    let structure_len = word(blob, 36)? as usize;
    let string_offset = word(blob, 12)? as usize;
    let strings_len = word(blob, 32)? as usize;
    let structure_end = structure
        .checked_add(structure_len)
        .ok_or(Error::Malformed)?;
    let string_end = string_offset
        .checked_add(strings_len)
        .ok_or(Error::Malformed)?;
    if structure < 40
        || !structure.is_multiple_of(4)
        || string_offset < 40
        || (structure < string_end && string_offset < structure_end)
    {
        return Err(Error::Malformed);
    }

    let structure = blob.get(structure..structure_end).ok_or(Error::Malformed)?;
    let strings = blob
        .get(string_offset..string_end)
        .ok_or(Error::Malformed)?;

    Ok((structure, strings))
}

impl<'a> Tree<'a> {
    /// Returns parsed firmware nodes in parent-first order.
    #[must_use]
    pub fn nodes(&self) -> &[Node<'a>] {
        &self.nodes
    }
    /// Parses a bounded v17-compatible firmware blob.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn parse(blob: &'a [u8]) -> Result<Self> {
        let (structure, strings) = blob_blocks(blob)?;

        let mut nodes: Vec<Node<'a>> = Vec::new();
        let mut current = None;
        let mut offset = 0;
        let mut ended = false;
        while offset < structure.len() {
            let token = word(structure, offset)?;
            offset += 4;
            match token {
                1 => {
                    if current.is_none() && !nodes.is_empty() {
                        return Err(Error::Malformed);
                    }

                    let (name, end) = nul(structure, offset)?;
                    offset = aligned(end)?;
                    if (nodes.is_empty() && !name.is_empty())
                        || (!nodes.is_empty() && name.is_empty())
                    {
                        return Err(Error::Malformed);
                    }
                    nodes.push(Node {
                        name,
                        parent: current,
                        enabled: true,
                        properties: Vec::new(),
                    });
                    current = Some(nodes.len() - 1);
                }
                2 => {
                    current = nodes
                        .get(current.ok_or(Error::Malformed)?)
                        .ok_or(Error::Malformed)?
                        .parent;
                }
                3 => {
                    let index = current.ok_or(Error::Malformed)?;
                    let len = word(structure, offset)? as usize;
                    let name_offset = word(structure, offset + 4)? as usize;
                    offset = offset.checked_add(8).ok_or(Error::Malformed)?;
                    let end = offset.checked_add(len).ok_or(Error::Malformed)?;
                    let value = structure.get(offset..end).ok_or(Error::Malformed)?;
                    let (name, _) = nul(strings, name_offset)?;
                    if nodes[index].bytes(name).is_some() {
                        return Err(Error::Malformed);
                    }
                    nodes[index].properties.push(Property { name, value });
                    offset = aligned(end)?;
                }
                4 => {}
                9 => {
                    if current.is_some() || nodes.is_empty() {
                        return Err(Error::Malformed);
                    }
                    ended = true;
                    break;
                }
                _ => return Err(Error::Malformed),
            }

            if offset > structure.len() {
                return Err(Error::Malformed);
            }
        }

        if !ended {
            return Err(Error::Malformed);
        }

        let mut phandles = alloc::collections::BTreeSet::new();
        for index in 0..nodes.len() {
            if let Some(phandle) = nodes[index].phandle()? {
                if !phandles.insert(phandle) {
                    return Err(Error::Malformed);
                }
            }

            let inherited = nodes[index].parent.is_none_or(|p| nodes[p].enabled);
            let status = nodes[index].strings(c"status")?;
            if status.len() > 1 || (nodes[index].bytes(c"status").is_some() && status.is_empty()) {
                return Err(Error::Malformed);
            }
            nodes[index].enabled =
                inherited && status.first().is_none_or(|s| *s == c"ok" || *s == c"okay");
        }

        Ok(Self { nodes })
    }

    /// Finds the inherited interrupt parent identity.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn interrupt_parent(&self, index: usize) -> Result<u32> {
        let mut node = Some(index);
        while let Some(index) = node {
            let current = self.nodes.get(index).ok_or(Error::Malformed)?;
            if let Some(parent) = current.cell(c"interrupt-parent")? {
                return Ok(parent);
            }
            node = current.parent;
            if let Some(parent) = node {
                if self.nodes[parent].cell(c"#interrupt-cells")?.is_some() {
                    if let Some(phandle) = self.nodes[parent].phandle()? {
                        return Ok(phandle);
                    }
                }
            }
        }

        Ok(0)
    }

    /// Finds a node by its validated unique identity.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn phandle_node(&self, phandle: u32) -> Result<usize> {
        for (index, node) in self.nodes.iter().enumerate() {
            if node.phandle()? == Some(phandle) {
                return Ok(index);
            }
        }

        Err(Error::Malformed)
    }

    /// Finds a PCI function below one host by its configuration unit address.
    /// Address-space flags and register-number bits do not identify the function.
    /// Disabled nodes remain visible so their explicit routing cannot fall back
    /// to a different ancestor's map.
    ///
    /// # Errors
    /// Rejects an invalid host, truncated PCI addresses or duplicate identities.
    pub fn pci_node(&self, host: usize, address: u32) -> Result<Option<usize>> {
        self.nodes.get(host).ok_or(Error::Malformed)?;
        let mut found = None;
        for (index, node) in self.nodes.iter().enumerate() {
            let Some(parent) = node.parent else { continue };
            if !self.nodes[parent].pci() {
                continue;
            }

            let mut ancestor = Some(parent);
            let mut below_host = false;
            while let Some(current) = ancestor {
                if current == host {
                    below_host = true;
                    break;
                }
                ancestor = self.nodes[current].parent;
            }

            if !below_host {
                continue;
            }

            let Some(reg) = node.bytes(c"reg") else {
                continue;
            };

            if word(reg, 0)? & 0x00ff_ff00 == address & 0x00ff_ff00
                && found.replace(index).is_some()
            {
                return Err(Error::Malformed);
            }
        }

        Ok(found)
    }

    /// Returns the address and interrupt widths of an interrupt controller or nexus.
    fn irq_widths(&self, node: usize) -> Result<(usize, usize)> {
        let node = self.nodes.get(node).ok_or(Error::Malformed)?;
        let address = node.cell(c"#address-cells")?.unwrap_or(0) as usize;
        let interrupt = node.cell(c"#interrupt-cells")?.ok_or(Error::Malformed)? as usize;
        if address > 4 || interrupt == 0 || interrupt > 16 {
            return Err(Error::Unsupported);
        }

        Ok((address, interrupt))
    }

    /// Resolves a unit address/specifier through interrupt-map nexuses. No
    /// matching entry or a disabled controller produces no route. Malformed
    /// tables, unknown phandles and cycles are errors, never default IRQs.
    ///
    /// # Errors
    /// Rejects malformed tables, cycles and unsupported pass-through geometry.
    #[allow(clippy::too_many_lines)]
    pub fn resolve_interrupt(
        &self,
        mut nexus: usize,
        address: &[u32],
        specifier: &[u32],
    ) -> Result<Option<InterruptRoute>> {
        let mut address = address.to_vec();
        let mut specifier = specifier.to_vec();
        let mut visited = Vec::new();
        loop {
            if visited.contains(&nexus) {
                return Err(Error::Malformed);
            }
            visited.push(nexus);
            let node = self.nodes.get(nexus).ok_or(Error::Malformed)?;
            if !node.enabled {
                return Ok(None);
            }

            let (ac, ic) = self.irq_widths(nexus)?;
            if address.len() != ac || specifier.len() != ic {
                return Err(Error::Malformed);
            }

            if node.bytes(c"interrupt-controller").is_some() {
                return Ok(Some(InterruptRoute {
                    phandle: node.phandle()?.ok_or(Error::Malformed)?,
                    cells: specifier,
                }));
            }

            let Some(table) = node.bytes(c"interrupt-map") else {
                let parent = self.interrupt_parent(nexus)?;
                if parent == 0 {
                    return Ok(None);
                }

                let next = self.phandle_node(parent)?;
                let (parent_address_cells, parent_interrupt_cells) = self.irq_widths(next)?;
                if parent_interrupt_cells != ic {
                    return Err(Error::Malformed);
                }

                let registers = irq_cells(node.bytes(c"reg").unwrap_or_default())?;
                address = registers
                    .get(..parent_address_cells)
                    .ok_or(Error::Malformed)?
                    .to_vec();
                nexus = next;
                continue;
            };

            let mut key = address;
            key.extend_from_slice(&specifier);
            let mask = node
                .bytes(c"interrupt-map-mask")
                .map(irq_cells)
                .transpose()?
                .unwrap_or_else(|| alloc::vec![u32::MAX; key.len()]);
            let pass = node
                .bytes(c"interrupt-map-pass-thru")
                .map(irq_cells)
                .transpose()?
                .unwrap_or_else(|| alloc::vec![0; key.len()]);
            if mask.len() != key.len() || pass.len() != key.len() {
                return Err(Error::Malformed);
            }

            let table = irq_cells(table)?;
            let mut cursor = 0;
            let mut selected = None;
            while cursor < table.len() {
                let child = table
                    .get(cursor..cursor + key.len())
                    .ok_or(Error::Malformed)?;
                cursor += key.len();
                let phandle = *table.get(cursor).ok_or(Error::Malformed)?;
                cursor += 1;
                let parent = self.phandle_node(phandle)?;
                let (parent_address_cells, parent_interrupt_cells) = self.irq_widths(parent)?;
                let target = table
                    .get(cursor..cursor + parent_address_cells + parent_interrupt_cells)
                    .ok_or(Error::Malformed)?;
                cursor += parent_address_cells + parent_interrupt_cells;
                if selected.is_none()
                    && child
                        .iter()
                        .zip(&key)
                        .zip(&mask)
                        .all(|((&entry, &input), &mask)| (entry & mask) == (input & mask))
                {
                    let mut target = target.to_vec();
                    if pass.iter().any(|cell| *cell != 0) {
                        if target.len() != key.len() {
                            return Err(Error::Unsupported);
                        }

                        for ((target, input), pass) in target.iter_mut().zip(&key).zip(&pass) {
                            *target = (*target & !pass) | (input & pass);
                        }
                    }
                    selected = Some((
                        parent,
                        target[..parent_address_cells].to_vec(),
                        target[parent_address_cells..].to_vec(),
                    ));
                }
            }

            let Some((parent, next_address, next_specifier)) = selected else {
                return Ok(None);
            };
            nexus = parent;
            address = next_address;
            specifier = next_specifier;
        }
    }

    /// Resolves all interrupt specifiers, preserving each controller identity.
    ///
    /// # Errors
    /// Rejects truncated tuples, invalid parents and malformed nexus mappings.
    pub fn device_interrupts(&self, index: usize) -> Result<Vec<InterruptRoute>> {
        let node = self.nodes.get(index).ok_or(Error::Malformed)?;
        let registers = irq_cells(node.bytes(c"reg").unwrap_or_default())?;
        let mut routes = Vec::new();
        if let Some(bytes) = node.bytes(c"interrupts-extended") {
            let values = irq_cells(bytes)?;
            let mut cursor = 0;
            while cursor < values.len() {
                let parent = self.phandle_node(values[cursor])?;
                let (address_cells, specifier_cells) = self.irq_widths(parent)?;
                cursor += 1;
                let specifier = values
                    .get(cursor..cursor + specifier_cells)
                    .ok_or(Error::Malformed)?;
                cursor += specifier_cells;
                let address = registers.get(..address_cells).ok_or(Error::Malformed)?;
                if let Some(route) = self.resolve_interrupt(parent, address, specifier)? {
                    routes.push(route);
                }
            }
        } else if let Some(bytes) = node.bytes(c"interrupts") {
            let parent = self.phandle_node(self.interrupt_parent(index)?)?;
            let (address_cells, specifier_cells) = self.irq_widths(parent)?;
            let values = irq_cells(bytes)?;
            if !values.len().is_multiple_of(specifier_cells) {
                return Err(Error::Malformed);
            }

            let address = registers.get(..address_cells).ok_or(Error::Malformed)?;
            for specifier in values.chunks_exact(specifier_cells) {
                if let Some(route) = self.resolve_interrupt(parent, address, specifier)? {
                    routes.push(route);
                }
            }
        }

        Ok(routes)
    }

    /// Decodes every register tuple and optional name.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn registers(&self, index: usize) -> Result<Vec<Register<'a>>> {
        let mut registers = Vec::new();
        let node = self.nodes.get(index).ok_or(Error::Malformed)?;
        let Some(bytes) = node.bytes(c"reg") else {
            return Ok(registers);
        };

        let parent = &self.nodes[node.parent.ok_or(Error::Malformed)?];
        let (ac, sc) = parent.widths()?;
        let stride = (ac + sc) * 4;
        if stride == 0 || !bytes.len().is_multiple_of(stride) {
            return Err(Error::Malformed);
        }

        let names = node.strings(c"reg-names")?;
        if names.iter().any(|name| name.is_empty())
            || (!names.is_empty() && names.len() != bytes.len() / stride)
        {
            return Err(Error::Malformed);
        }

        for (i, record) in bytes.chunks_exact(stride).enumerate() {
            let address = address(parent, &record[..ac * 4], ac)?;
            let size = number(&record[ac * 4..], sc)?;
            if !address.covers(size) {
                return Err(Error::Malformed);
            }
            registers.push(Register {
                address,
                size,
                name: names.get(i).copied(),
            });
        }

        Ok(registers)
    }

    /// Decodes this bus’s address translation windows.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn ranges(&self, bus: usize) -> Result<Option<Vec<Range>>> {
        let node = self.nodes.get(bus).ok_or(Error::Malformed)?;
        let Some(bytes) = node.bytes(c"ranges") else {
            return Ok(None);
        };

        let parent = &self.nodes[node.parent.ok_or(Error::Malformed)?];
        let (ac, sc) = node.widths()?;
        let (pac, _) = parent.widths()?;
        let stride = (ac + pac + sc) * 4;
        if stride == 0 || !bytes.len().is_multiple_of(stride) {
            return Err(Error::Malformed);
        }

        let mut ranges = Vec::new();
        for record in bytes.chunks_exact(stride) {
            let child = address(node, &record[..ac * 4], ac)?;
            let parent = address(parent, &record[ac * 4..(ac + pac) * 4], pac)?;
            let size = number(&record[(ac + pac) * 4..], sc)?;
            if !child.covers(size) || !parent.covers(size) {
                return Err(Error::Malformed);
            }

            let flags = if node.pci() && ac == 3 {
                let hi = word(record, 0)?;
                (u32::from(hi & 0x4000_0000 != 0) * 2)
                    | (u32::from(hi & 0x0300_0000 == 0x0300_0000) * 8)
            } else {
                0
            };
            ranges.push(Range {
                child,
                parent,
                size,
                flags,
            });
        }

        Ok(Some(ranges))
    }

    /// Translate an entire span, never just its first byte. The root's child
    /// address space is physical. Each intermediate bus must provide a mapping.
    /// Translates a complete register span to a CPU address.
    ///
    /// # Errors
    /// Rejects malformed properties, unsupported cell widths or invalid spans.
    pub fn translate(&self, mut bus: usize, mut value: Address, size: u64) -> Result<Option<u64>> {
        if size == 0 || matches!(value.space, Space::Configuration | Space::Unspecified) {
            return Ok(None);
        }

        if !value.covers(size) {
            return Err(Error::Malformed);
        }

        while let Some(parent) = self.nodes.get(bus).ok_or(Error::Malformed)?.parent {
            let Some(ranges) = self.ranges(bus)? else {
                return Ok(None);
            };

            if !ranges.is_empty() {
                let mapped = ranges.iter().find_map(|range| {
                    if value.space != range.child.space {
                        return None;
                    }

                    let offset = value.value.checked_sub(range.child.value)?;
                    if offset.checked_add(size)? > range.size {
                        return None;
                    }
                    Some(Address {
                        space: range.parent.space,
                        value: range.parent.value.checked_add(offset)?,
                        bits: range.parent.bits,
                    })
                });
                let Some(mapped) = mapped else {
                    return Ok(None);
                };
                value = mapped;
            } else if value.space == Space::Memory && !self.nodes[parent].pci() {
                // Identity mapping preserves the numeric memory address when
                // leaving the PCI address encoding for a native CPU bus.
                value.space = Space::Native;
            }

            if self.nodes[parent].widths()?.0 == 1
                && value
                    .value
                    .checked_add(size - 1)
                    .is_none_or(|end| end > u64::from(u32::MAX))
            {
                return Ok(None);
            }
            bus = parent;
        }

        Ok((value.space == Space::Native).then_some(value.value))
    }
}
