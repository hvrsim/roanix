//! Physical memory segments and compact page-frame database indexing.
//!
//! Boot-time initialization sorts non-overlapping segments and assigns consecutive
//! metadata slices. Lookups translate physical frame numbers without allocating
//! metadata for address holes. The segment layout is immutable after boot;
//! physical memory hotplug is not supported.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

#[derive(Clone, Copy)]
#[repr(C)]
pub(super) struct Segment {
    pub(super) start: usize,
    pub(super) end: usize,
    pub(super) first: usize,
}

/// Assign consecutive page-array slices to the sorted physical segments.
pub(super) fn initialize(segments: &mut [Segment]) -> usize {
    segments.sort_unstable_by_key(|segment| segment.start);
    let mut count = 0usize;
    let mut previous_end = 0usize;
    for segment in segments {
        assert!(
            segment.start >= previous_end && segment.start < segment.end,
            "mem: invalid physical segments"
        );
        segment.first = count;
        count = count
            .checked_add(segment.end - segment.start)
            .expect("mem: physical page count overflow");
        previous_end = segment.end;
    }
    count
}

/// Return the compact page-array index for a physical frame number.
pub(super) fn lookup(segments: &[Segment], pfn: usize) -> Option<usize> {
    let index = segments
        .partition_point(|segment| segment.start <= pfn)
        .checked_sub(1)?;
    let segment = &segments[index];
    (pfn < segment.end).then(|| segment.first + pfn - segment.start)
}

/// Skip physical address holes when searching for a contiguous free run.
pub(super) fn next(segments: &[Segment], pfn: usize) -> Option<usize> {
    let index = segments.partition_point(|segment| segment.end <= pfn);
    segments.get(index).map(|segment| pfn.max(segment.start))
}
