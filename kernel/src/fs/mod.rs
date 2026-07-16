//! Vnode-based virtual filesystem and memory-backed root filesystem.

extern crate alloc;

pub mod error;
pub mod file;
pub mod path;
pub mod tmpfs;
pub mod vfs;
pub mod vnode;

use alloc::sync::Arc;

use log::info;

use crate::mem::{self, PAGE_SIZE};

pub use error::{Error, Result};
pub use file::{FileRef, OpenFile, OpenFlags, SeekFrom};
pub use vfs::{
    create_dir, create_file, link, lookup, mount, open, remove_dir, rename, statfs, symlink,
    sync_all, unlink,
};
pub use vnode::{
    CreateKind, DirEntry, FileSystem, FileSystemRef, FilesystemId, NodeId, SetAttr, StatFs, Vnode,
    VnodeAttr, VnodeKey, VnodeKind,
};

/// Initializes the VFS and mounts tmpfs as the initial root filesystem.
pub fn init() {
    let stats = mem::phys::stats().expect("fs: physical memory manager is not initialized");
    let maximum_pages = (stats.free_pages as u64 / 2).max(1);
    let options = tmpfs::TmpfsOptions {
        maximum_bytes: maximum_pages.saturating_mul(PAGE_SIZE),
        maximum_nodes: maximum_pages.saturating_mul(4).max(1024),
    };
    let filesystem = tmpfs::Tmpfs::new(options).expect("fs: failed to create root tmpfs");
    let filesystem: Arc<dyn FileSystem> = filesystem;
    vfs::init_root(filesystem).expect("fs: failed to mount root filesystem");
    info!(
        "fs: mounted tmpfs root (limit={} MiB)",
        options.maximum_bytes / (1024 * 1024)
    );
}
