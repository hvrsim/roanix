//! Physical-page reserve eligibility and reclamation watermarks.
//!
//! Free-bucket decisions preserve kernel and page-daemon reserves, allowing a
//! single-page allocation near a reserve instead of refilling a CPU cache.
//! Already populated caches use their own fast path. Watermarks scale with usable
//! memory and CPU count to trigger reclamation before reserves are exhausted.
//!
//! Third-party notices and provenance are in the repository-root `COPYING` file.

pub(super) const KERNEL_PAGES_PER_CPU: usize = 5;
pub(super) const DAEMON_PAGES: usize = 1;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Access {
    Denied,
    Single,
    Fill,
}
pub(super) fn access(free: usize, cpus: usize, use_reserve: bool, daemon: bool) -> Access {
    if free == 0 {
        return Access::Denied;
    }
    if free <= KERNEL_PAGES_PER_CPU * cpus.max(1) {
        if !use_reserve || (free <= DAEMON_PAGES && !daemon) {
            Access::Denied
        } else {
            Access::Single
        }
    } else {
        Access::Fill
    }
}

/// Use 0.5% of RAM, bounded to 128 KiB..1 MiB per CPU;
/// freetarg is 4/3 freemin and always exceeds it.
pub(super) fn watermarks(pages: usize, cpus: usize) -> (u64, u64) {
    let cpus = cpus.max(1);
    let minimum = ((pages / 200).clamp(
        128 * 1024 / super::PAGE_SIZE as usize,
        1024 * 1024 / super::PAGE_SIZE as usize,
    ) * cpus)
        .max(KERNEL_PAGES_PER_CPU * cpus + 1) as u64;
    (minimum, (minimum * 4 / 3).max(minimum + 1))
}
