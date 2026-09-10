//! Vnode-based virtual filesystem contracts and native implementations.

extern crate alloc;

pub mod devfs;
pub mod error;
pub mod ext4;
pub mod fat32;
pub mod file;
mod io;
pub mod partition;
pub mod path;
mod syscall;
pub mod tmpfs;
pub mod vfs;
pub mod vnode;

use log::debug;

pub use error::{Error, Result};
pub use file::{FileRef, OpenFile, OpenFlags, SeekFrom};
pub(crate) use vfs::{
    PathAnchor, create_dir_at, link_at, open_at, rename_at, replace_root, resolve_at, root_anchor,
    symlink_at, unlink_at,
};
pub use vfs::{
    create_dir, create_file, link, lookup, lookup_nofollow, mount, open, remove_dir, rename,
    statfs, symlink, sync_all, unlink, unmount,
};
pub use vnode::{
    CreateKind, DirEntry, FileSystem, FileSystemRef, FilesystemId, IoctlContext, NodeId,
    PollEvents, SetAttr, StatFs, Vnode, VnodeAttr, VnodeKey, VnodeKind,
};

/// Mounts a block-backed filesystem by source basename or `/dev/...` path.
pub fn mount_block(source: &str, target: &[u8], filesystem: &str) -> Result<()> {
    let name = source.strip_prefix("/dev/").unwrap_or(source);
    if name.is_empty() || name.as_bytes().contains(&b'/') {
        return Err(Error::InvalidArgument);
    }
    let device = crate::dev::block::find(name).map_err(Error::from)?;
    let mounted: FileSystemRef = match filesystem {
        "ext4" => ext4::Ext4::new(device)?,
        "fat32" | "vfat" => fat32::Fat32::new(device)?,
        _ => return Err(Error::Unsupported),
    };
    if target == b"/" {
        validate_system_root(&mounted)?;
        replace_root(mounted)?;
        devfs::remount_global()?;
        let temporary: FileSystemRef = tmpfs::Tmpfs::new()?;
        vfs::mount(b"/tmp", temporary)?;
    } else {
        vfs::mount(target, mounted)?;
    }
    Ok(())
}

/// Rejects arbitrary disk volumes before the initramfs is discarded.
fn validate_system_root(filesystem: &FileSystemRef) -> Result<()> {
    let root = filesystem.root();
    for directory in [b"dev".as_slice(), b"etc", b"sbin", b"tmp", b"usr"] {
        if root.lookup(directory)?.kind() != VnodeKind::Directory {
            return Err(Error::NotDirectory);
        }
    }
    let etc = root.lookup(b"etc")?;
    if etc.lookup(b"roanix-root")?.kind() != VnodeKind::Regular {
        return Err(Error::InvalidArgument);
    }
    Ok(())
}

/// Initializes the VFS with kernel-owned root and device filesystems.
pub fn init() {
    let filesystem = tmpfs::Tmpfs::new().expect("fs: failed to create root tmpfs");
    vfs::init_root(filesystem).expect("fs: failed to mount root filesystem");
    devfs::mount_global().expect("fs: failed to mount devfs");
    crate::sys::klog::dev::register().expect("fs: failed to publish kernel log devices");
    debug!("mounted kernel tmpfs on / and devfs on /dev");
}
