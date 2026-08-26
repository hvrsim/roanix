//! Vnode-based virtual filesystem contracts and provider adapters.

extern crate alloc;

pub mod error;
pub mod file;
pub mod path;
pub mod provider;
mod syscall;
pub mod vfs;
pub mod vnode;

use log::debug;

pub use error::{Error, Result};
pub use file::{FileRef, OpenFile, OpenFlags, SeekFrom};
pub(crate) use vfs::{
    PathAnchor, create_dir_at, link_at, open_at, rename_at, resolve_at, root_anchor, symlink_at,
    unlink_at,
};
pub use vfs::{
    create_dir, create_file, link, lookup, lookup_nofollow, mount, open, remove_dir, rename,
    statfs, symlink, sync_all, unlink, unmount,
};
pub use vnode::{
    CreateKind, DirEntry, FileSystem, FileSystemRef, FilesystemId, IoctlContext, NodeId,
    PollEvents, SetAttr, StatFs, Vnode, VnodeAttr, VnodeKey, VnodeKind,
};

/// Initializes the VFS from the early filesystem providers.
pub fn init() {
    let filesystem = provider::mount_named(
        "tmpfs",
        provider::FsMountOptions {
            size: core::mem::size_of::<provider::FsMountOptions>() as u32,
            flags: 0,
            page_limit: 0,
        },
    )
    .expect("fs: failed to create root tmpfs");
    vfs::init_root(filesystem).expect("fs: failed to mount root filesystem");
    create_dir(b"/dev", 0o755).expect("fs: failed to create /dev mountpoint");
    let devfs = provider::mount_named(
        "devfs",
        provider::FsMountOptions {
            size: core::mem::size_of::<provider::FsMountOptions>() as u32,
            flags: 0,
            page_limit: 0,
        },
    )
    .expect("fs: failed to create devfs");
    mount(b"/dev", devfs).expect("fs: failed to mount devfs");
    crate::sys::klog::dev::register().expect("fs: failed to publish kernel log devices");
    debug!("mounted tmpfs on / and devfs on /dev");
}
