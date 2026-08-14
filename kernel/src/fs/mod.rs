//! Vnode-based virtual filesystem and memory-backed root filesystem.

extern crate alloc;

pub mod devtempfs;
pub mod error;
pub mod file;
pub mod path;
mod syscall;
pub mod tmpfs;
pub mod vfs;
pub mod vnode;

use alloc::sync::Arc;

use log::info;

pub use error::{Error, Result};
pub use file::{FileRef, OpenFile, OpenFlags, SeekFrom};
pub use vfs::{
    create_dir, create_file, link, lookup, lookup_nofollow, mount, open, remove_dir, rename,
    statfs, symlink, sync_all, unlink, unmount,
};
pub(crate) use vfs::{
    create_dir_at, link_at, open_at, rename_at, resolve_at, root_anchor, symlink_at, unlink_at,
    PathAnchor,
};
pub use vnode::{
    CreateKind, DirEntry, FileSystem, FileSystemRef, FilesystemId, IoctlContext, NodeId,
    PollEvents, SetAttr, StatFs, Vnode, VnodeAttr, VnodeKey, VnodeKind,
};

/// Initializes the VFS, mounts tmpfs as root, and mounts devtempfs at `/dev`.
pub fn init() {
    let filesystem = tmpfs::Tmpfs::new().expect("fs: failed to create root tmpfs");
    let filesystem: Arc<dyn FileSystem> = filesystem;
    vfs::init_root(filesystem).expect("fs: failed to mount root filesystem");
    devtempfs::mount_global().expect("fs: failed to mount devtempfs");
    info!("fs: mounted tmpfs root");
    info!("fs: mounted devtempfs at /dev");
}
