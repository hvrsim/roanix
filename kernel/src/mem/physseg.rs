/*
 * Copyright (c) 1997 Charles D. Cranor and Washington University.
 * Copyright (c) 1991, 1993, The Regents of the University of California.
 *
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * The Mach Operating System project at Carnegie-Mellon University.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)vm_page.h   7.3 (Berkeley) 4/21/91
 * from: Id: uvm_page.h,v 1.1.2.6 1998/02/04 02:31:42 chuck Exp
 *
 *
 * Copyright (c) 1987, 1990 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

//! UVM physical segments with a compact vm_page array. Address holes have
//! no metadata. Boot-loaded segments are immutable in Roanix (no hotplug).
#[derive(Clone, Copy)]
#[repr(C)]
pub(super) struct Segment {
    pub(super) start: usize,
    pub(super) end: usize,
    pub(super) first: usize,
}

/// Assign consecutive page-array slices, as uvm_physseg_init_seg does.
pub(super) fn initialize(segments: &mut [Segment]) -> usize {
    segments.sort_unstable_by_key(|segment| segment.start);
    let mut count = 0usize;
    let mut previous_end = 0usize;
    for segment in segments {
        assert!(
            segment.start >= previous_end && segment.start < segment.end,
            "uvm: invalid physical segments"
        );
        segment.first = count;
        count = count
            .checked_add(segment.end - segment.start)
            .expect("uvm: physical page count overflow");
        previous_end = segment.end;
    }
    count
}

/// PHYS_TO_VM_PAGE / uvm_physseg_find: return the compact array index.
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
