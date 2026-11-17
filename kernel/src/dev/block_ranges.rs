//! Sleepable exclusion for overlapping whole-disk block ranges.

use alloc::vec::Vec;
use core::ops::Range;

use crate::sys::{event::Event, sync::Mutex};

pub(crate) struct WriteRanges {
    active: Mutex<Vec<Range<u64>>>,
    changed: Event,
}

impl WriteRanges {
    pub(crate) fn new() -> Self {
        Self {
            active: Mutex::new(Vec::new()),
            changed: Event::new(),
        }
    }

    pub(crate) fn lock(&self, range: Range<u64>) -> WriteRange<'_> {
        assert!(range.start < range.end, "empty block write range");
        let mut active = self.active.lock();
        while active.iter().any(|other| overlaps(other, &range)) {
            active = self.changed.listen().wait_releasing(active);
        }
        active.push(range.clone());
        WriteRange {
            ranges: self,
            range,
        }
    }
}

fn overlaps(left: &Range<u64>, right: &Range<u64>) -> bool {
    left.start < right.end && right.start < left.end
}

pub(crate) struct WriteRange<'a> {
    ranges: &'a WriteRanges,
    range: Range<u64>,
}

impl Drop for WriteRange<'_> {
    fn drop(&mut self) {
        let mut active = self.ranges.active.lock();
        let index = active
            .iter()
            .position(|range| *range == self.range)
            .expect("block write range must remain admitted until release");
        active.swap_remove(index);
        self.ranges.changed.wake_all();
    }
}
