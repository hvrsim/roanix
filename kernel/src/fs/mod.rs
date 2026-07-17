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
    let filesystem = tmpfs::Tmpfs::new().expect("fs: failed to create root tmpfs");
    let filesystem: Arc<dyn FileSystem> = filesystem;
    vfs::init_root(filesystem).expect("fs: failed to mount root filesystem");
    info!("fs: mounted tmpfs root");
}
