//! Driver-to-device matching.
//!
//! A driver declares a table of match entries. Each entry describes one way the
//! driver can recognise hardware, and carries a private cookie handed back to
//! the probe callback so a single driver can serve several device variants.
//!
//! The entry kinds cover the identification schemes the framework needs to
//! support: device-tree `compatible` lists, ACPI hardware identifiers, numeric
//! identifier tables with masks for buses like PCI and USB, and direct property
//! comparisons. A bus may additionally override matching entirely when its
//! hardware needs a rule that a table cannot express.

use alloc::{boxed::Box, string::String, sync::Arc, vec::Vec};

use super::{
    super::error::{Error, Result},
    device::Device,
    property::PropValue,
};

/// Match entry kinds.
pub mod kind {
    /// Matches a string in the device's `compatible` property list.
    ///
    /// Earlier entries in the device's list are more specific, so a match
    /// against position zero outranks a match against a later fallback.
    pub const COMPATIBLE: u32 = 1;
    /// Matches the device's node name.
    pub const NAME: u32 = 2;
    /// Matches an ACPI hardware or compatible identifier.
    pub const ACPI_ID: u32 = 3;
    /// Compares a named property against a string or integer.
    pub const PROPERTY: u32 = 4;
    /// Compares masked numeric identifiers held in a named property.
    ///
    /// This expresses PCI vendor/device/subsystem tables, USB
    /// vendor/product tables, and class-code matching.
    pub const ID: u32 = 5;
    /// Matches every device on the driver's bus.
    pub const ANY: u32 = 6;
}

/// Match entry flags.
pub mod flags {
    /// The entry must not contribute a specificity bonus.
    pub const NO_BONUS: u32 = 1 << 0;
}

/// Property holding the primary numeric identifier of a bus device.
pub const PROP_ID: &str = "id";
/// Property holding the secondary numeric identifier, such as a class code.
pub const PROP_ID_CLASS: &str = "class";
/// Property holding device-tree compatible strings.
pub const PROP_COMPATIBLE: &str = "compatible";
/// Property holding ACPI hardware identifiers.
pub const PROP_ACPI_HID: &str = "acpi.hid";
/// Property holding ACPI compatible identifiers.
pub const PROP_ACPI_CID: &str = "acpi.cid";

/// Bonus applied to the most specific `compatible` match.
const COMPATIBLE_BONUS: i32 = 1024;

/// One way a driver can recognise a device.
pub struct MatchEntry {
    /// Entry kind, one of the [`kind`] constants.
    pub kind: u32,
    /// Entry flags drawn from [`flags`].
    pub flags: u32,
    /// String operand, or the property name for [`kind::PROPERTY`] and
    /// [`kind::ID`].
    pub key: Option<Box<str>>,
    /// Expected string value for [`kind::PROPERTY`].
    pub value: Option<Box<str>>,
    /// Expected primary identifier, compared under `mask0`.
    pub id0: u64,
    /// Bits of `id0` that participate in the comparison.
    pub mask0: u64,
    /// Expected secondary identifier, compared under `mask1`.
    pub id1: u64,
    /// Bits of `id1` that participate in the comparison.
    pub mask1: u64,
    /// Driver-private cookie handed to the probe callback.
    pub data: usize,
    /// Additional score contributed when this entry matches.
    pub score: i32,
}

impl MatchEntry {
    /// Creates an entry matching a device-tree compatible string.
    pub fn compatible(text: &str) -> Self {
        Self {
            kind: kind::COMPATIBLE,
            key: Some(String::from(text).into_boxed_str()),
            ..Self::empty()
        }
    }

    /// Creates an entry matching every device on the bus.
    pub fn any() -> Self {
        Self {
            kind: kind::ANY,
            ..Self::empty()
        }
    }

    fn empty() -> Self {
        Self {
            kind: kind::ANY,
            flags: 0,
            key: None,
            value: None,
            id0: 0,
            mask0: 0,
            id1: 0,
            mask1: 0,
            data: 0,
            score: 0,
        }
    }

    /// Validates the entry's operands.
    pub fn validate(&self) -> Result<()> {
        match self.kind {
            kind::COMPATIBLE | kind::NAME | kind::ACPI_ID => {
                if self.key.as_ref().is_none_or(|key| key.is_empty()) {
                    return Err(Error::InvalidArgument);
                }
            }
            kind::PROPERTY => {
                if self.key.as_ref().is_none_or(|key| key.is_empty()) {
                    return Err(Error::InvalidArgument);
                }
            }
            kind::ID => {
                if self.mask0 == 0 && self.mask1 == 0 {
                    return Err(Error::InvalidArgument);
                }
            }
            kind::ANY => {}
            _ => return Err(Error::InvalidArgument),
        }
        Ok(())
    }

    /// Scores this entry against `device`, returning `None` when it does not
    /// apply.
    pub fn evaluate(&self, device: &Arc<Device>) -> Option<i32> {
        let base = match self.kind {
            kind::COMPATIBLE => self.evaluate_compatible(device)?,
            kind::NAME => self.evaluate_name(device)?,
            kind::ACPI_ID => self.evaluate_acpi(device)?,
            kind::PROPERTY => self.evaluate_property(device)?,
            kind::ID => self.evaluate_id(device)?,
            kind::ANY => 0,
            _ => return None,
        };
        Some(base + self.score)
    }

    fn evaluate_compatible(&self, device: &Arc<Device>) -> Option<i32> {
        let wanted = self.key.as_deref()?;
        let value = device.properties().get(PROP_COMPATIBLE)?;
        let position = match value {
            PropValue::Str(text) if &**text == wanted => 0,
            PropValue::StrList(list) => list.iter().position(|text| &**text == wanted)?,
            _ => return None,
        };
        if self.flags & flags::NO_BONUS != 0 {
            return Some(0);
        }
        Some(
            COMPATIBLE_BONUS
                - i32::try_from(position)
                    .unwrap_or(i32::MAX)
                    .min(COMPATIBLE_BONUS),
        )
    }

    fn evaluate_name(&self, device: &Arc<Device>) -> Option<i32> {
        let wanted = self.key.as_deref()?;
        if device.name() == wanted {
            return Some(0);
        }
        let value = device.properties().get("name")?;
        value.contains_string(wanted).then_some(0)
    }

    fn evaluate_acpi(&self, device: &Arc<Device>) -> Option<i32> {
        let wanted = self.key.as_deref()?;
        let properties = device.properties();
        if properties.has_string(PROP_ACPI_HID, wanted) {
            return Some(if self.flags & flags::NO_BONUS != 0 {
                0
            } else {
                COMPATIBLE_BONUS
            });
        }
        properties.has_string(PROP_ACPI_CID, wanted).then_some(0)
    }

    fn evaluate_property(&self, device: &Arc<Device>) -> Option<i32> {
        let name = self.key.as_deref()?;
        let value = device.properties().get(name)?;
        match self.value.as_deref() {
            Some(expected) => value.contains_string(expected).then_some(0),
            None => {
                if self.mask0 == 0 {
                    // Presence test.
                    return Some(0);
                }
                let actual = value.as_u64()?;
                (actual & self.mask0 == self.id0 & self.mask0).then_some(0)
            }
        }
    }

    fn evaluate_id(&self, device: &Arc<Device>) -> Option<i32> {
        let properties = device.properties();
        let primary_name = self.key.as_deref().unwrap_or(PROP_ID);
        if self.mask0 != 0 {
            let actual = properties.get(primary_name)?.as_u64()?;
            if actual & self.mask0 != self.id0 & self.mask0 {
                return None;
            }
        }
        if self.mask1 != 0 {
            let actual = properties.get(PROP_ID_CLASS)?.as_u64()?;
            if actual & self.mask1 != self.id1 & self.mask1 {
                return None;
            }
        }
        if self.flags & flags::NO_BONUS != 0 {
            return Some(0);
        }
        // Narrower masks describe more specific hardware.
        let bits = (self.mask0.count_ones() + self.mask1.count_ones()) as i32;
        Some(bits)
    }
}

/// Outcome of matching a driver's table against a device.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct MatchResult {
    /// Cookie carried by the winning entry.
    pub data: usize,
    /// Total score, used to choose between competing drivers.
    pub score: i32,
}

/// Returns the best-scoring entry of `table` for `device`.
pub fn best(table: &[MatchEntry], device: &Arc<Device>) -> Option<MatchResult> {
    let mut best: Option<MatchResult> = None;
    for entry in table {
        let Some(score) = entry.evaluate(device) else {
            continue;
        };
        if best.is_none_or(|current| score > current.score) {
            best = Some(MatchResult {
                data: entry.data,
                score,
            });
        }
    }
    best
}

/// Validates every entry of a match table.
pub fn validate(table: &[MatchEntry]) -> Result<()> {
    if table.is_empty() {
        return Err(Error::InvalidArgument);
    }
    for entry in table {
        entry.validate()?;
    }
    Ok(())
}

/// Builds a match table from entries, rejecting malformed ones.
pub fn build(entries: Vec<MatchEntry>) -> Result<Box<[MatchEntry]>> {
    validate(&entries)?;
    Ok(entries.into_boxed_slice())
}
