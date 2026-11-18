//! Vnode-based virtual filesystem contracts and native implementations.

extern crate alloc;

mod cache;
pub mod cgroupfs;
pub mod devfs;
pub mod error;
pub mod ext4;
pub mod fat32;
mod fifo;
pub mod file;
mod io;
mod location;
pub(crate) mod namecache;
mod namespace;
pub mod partition;
pub mod path;
pub mod procfs;
mod syscall;
pub mod sysfs;
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
        // Prepare memory-backed filesystems before discarding the initramfs,
        // so allocation failures do not leave a partially replaced root.
        let temporary = runtime_tmpfs(0o1777)?;
        let runtime = runtime_tmpfs(0o755)?;
        let shared = runtime_tmpfs(0o1777)?;
        replace_root(mounted, source.as_bytes())?;
        devfs::remount_global()?;
        vfs::mount(b"/tmp", temporary)?;
        vfs::mount(b"/run", runtime)?;
        vfs::mount(b"/dev/shm", shared)?;
        mount_device_links()?;
    } else {
        vfs::mount_with_source(target, mounted, source.as_bytes())?;
    }
    Ok(())
}

/// Device nodes remain driver-owned; udev owns links in these writable trees.
fn mount_device_links() -> Result<()> {
    let device_fs = devfs::global()?;
    for path in [
        "char",
        "block",
        "disk",
        "serial",
        "input/by-id",
        "input/by-path",
        "dri/by-path",
    ] {
        let mut parent = device_fs.root_id();
        for component in path.split('/') {
            parent = match device_fs.lookup_child(parent, component.as_bytes()) {
                Ok(id) => id,
                Err(Error::NotFound) => device_fs.create_dir(
                    devfs::OwnerId::KERNEL,
                    parent,
                    component.as_bytes(),
                    0o755,
                )?,
                Err(error) => return Err(error),
            };
        }
        vfs::mount(
            alloc::format!("/dev/{path}").as_bytes(),
            runtime_tmpfs(0o755)?,
        )?;
    }
    Ok(())
}

fn runtime_tmpfs(mode: u16) -> Result<FileSystemRef> {
    let filesystem: FileSystemRef = tmpfs::Tmpfs::new()?;
    filesystem.root().setattr(SetAttr {
        mode: Some(mode),
        size: None,
    })?;
    Ok(filesystem)
}

/// Rejects arbitrary disk volumes before the initramfs is discarded.
fn validate_system_root(filesystem: &FileSystemRef) -> Result<()> {
    let root = filesystem.root();
    for directory in [b"dev".as_slice(), b"etc", b"run", b"sbin", b"tmp", b"usr"] {
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
