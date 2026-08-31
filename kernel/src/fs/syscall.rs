//! Filesystem syscall implementations.

use alloc::vec::Vec;
use core::{mem::size_of, time::Duration};

use crate::{
    fs::{
        self, IoctlContext, OpenFlags, PathAnchor, PollEvents, SeekFrom, Vnode, VnodeAttr,
        VnodeKind,
    },
    mem::{IoSink, IoSource, VirtAddr},
    proc::{self, Descriptor, PipeEnd, PipeError, Process},
    sys::{clock, event::Event},
    syscall::{
        Errno, Result, current_process, map_fs_error, map_memory_error, map_process_error,
        read_user_path,
    },
};

const MAX_IO_SIZE: usize = 16 * 1024 * 1024;
const MAX_IOCTL_SIZE: usize = 4096;

const O_ACCMODE: u64 = 0o3;
const O_WRONLY: u64 = 0o1;
const O_RDWR: u64 = 0o2;
const O_CREAT: u64 = 0o100;
const O_EXCL: u64 = 0o200;
const O_NOCTTY: u64 = 0o400;
const O_TRUNC: u64 = 0o1000;
const O_APPEND: u64 = 0o2000;
const O_NONBLOCK: u64 = 0o4000;
const O_LARGEFILE: u64 = 0o100000;
const O_DIRECTORY: u64 = 0o200000;
const O_NOFOLLOW: u64 = 0o400000;
const O_CLOEXEC: u64 = 0o2000000;
const FD_CLOEXEC: u64 = 1;

const F_DUPFD: u64 = 0;
const F_GETFD: u64 = 1;
const F_SETFD: u64 = 2;
const F_GETFL: u64 = 3;
const F_SETFL: u64 = 4;
const F_DUPFD_CLOEXEC: u64 = 1030;

const PIPE_CLOEXEC: u64 = O_CLOEXEC;
const PIPE_NONBLOCK: u64 = O_NONBLOCK;
const AT_FDCWD: i32 = -100;
const AT_SYMLINK_NOFOLLOW: u64 = 0x100;
const AT_REMOVEDIR: u64 = 0x200;
const AT_EACCESS: u64 = 0x200;
const AT_SYMLINK_FOLLOW: u64 = 0x400;
const AT_NO_AUTOMOUNT: u64 = 0x800;
const AT_EMPTY_PATH: u64 = 0x1000;
const USER_DIRENT_SIZE: usize = 280;

const SEEK_SET: u64 = 0;
const SEEK_CUR: u64 = 1;
const SEEK_END: u64 = 2;

/// Userspace `stat` record layout.
#[derive(Clone, Copy)]
struct UserStat {
    device: u64,
    inode: u64,
    mode: u32,
    kind: u32,
    links: u64,
    size: u64,
    accessed_ns: u64,
    modified_ns: u64,
    changed_ns: u64,
}

impl UserStat {
    /// Builds the record reported for a descriptor with no filesystem identity.
    fn anonymous(mode: u32, kind: u32) -> Self {
        Self {
            device: 0,
            inode: 0,
            mode,
            kind,
            links: 1,
            size: 0,
            accessed_ns: 0,
            modified_ns: 0,
            changed_ns: 0,
        }
    }
}

#[derive(Clone, Copy)]
struct UserPollFd {
    fd: i32,
    events: i16,
    revents: i16,
}

enum AtPath {
    Vfs { base: PathAnchor, path: Vec<u8> },
    Pipe,
    Anonymous,
}

#[derive(Clone, Copy)]
enum EmptyPath {
    Reject,
    AllowVfs,
    AllowAny,
}

crate::syscall_handler! {
    syscall_file_open(_frame, path: u64 = 0, flags: u64 = 1, mode: u64 = 2) {
        let process = current_process()?;
        file_open_at(&process, AT_FDCWD, path, flags, mode)
    }
}

crate::syscall_handler! {
    syscall_file_openat(
        _frame,
        dirfd: i32 = 0,
        path: u64 = 1,
        flags: u64 = 2,
        mode: u64 = 3,
    ) {
        let process = current_process()?;
        file_open_at(&process, dirfd, path, flags, mode)
    }
}

crate::syscall_handler! {
    syscall_file_close(_frame, fd: i32 = 0) {
        if current_process()?.close_descriptor(fd) {
            Ok(0)
        } else {
            Err(Errno::BadFileDescriptor)
        }
    }
}

crate::syscall_handler! {
    syscall_file_read(_frame, fd: i32 = 0, buffer: u64 = 1, size: u64 = 2) {
        let size = clamped_io_size(size)?;
        if size == 0 {
            return Ok(0);
        }
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        check_terminal_job_control(&process, &descriptor, false)?;
        let address_space = process.address_space();
        let mut sink = IoSink::user(&address_space, VirtAddr::new(buffer), size)
            .map_err(map_memory_error)?;
        let read = descriptor_read(&descriptor, &mut sink)?;
        Ok(read as u64)
    }
}

crate::syscall_handler! {
    syscall_file_write(_frame, fd: i32 = 0, buffer: u64 = 1, size: u64 = 2) {
        let size = clamped_io_size(size)?;
        if size == 0 {
            return Ok(0);
        }
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        check_terminal_job_control(&process, &descriptor, true)?;
        let address_space = process.address_space();
        let source = IoSource::user(&address_space, VirtAddr::new(buffer), size)
            .map_err(map_memory_error)?;
        let written = descriptor_write(&descriptor, &source)?;
        Ok(written as u64)
    }
}

crate::syscall_handler! {
    syscall_file_seek(_frame, fd: i32 = 0, offset: i64 = 1, whence: u64 = 2) {
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let Descriptor::File(file) = descriptor else {
            return Err(Errno::IllegalSeek);
        };
        let from = match whence {
            SEEK_SET if offset >= 0 => SeekFrom::Start(offset as u64),
            SEEK_SET => return Err(Errno::Invalid),
            SEEK_CUR => SeekFrom::Current(offset),
            SEEK_END => SeekFrom::End(offset),
            _ => return Err(Errno::Invalid),
        };
        file.seek(from).map_err(map_fs_error)
    }
}

crate::syscall_handler! {
    syscall_file_ioctl(_frame, fd: i32 = 0, request: u64 = 1, argument: u64 = 2) {
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let Descriptor::File(file) = descriptor else {
            return Err(Errno::NotTty);
        };
        let spec = crate::driver::class::console::ioctl_spec(request);
        if spec.size > MAX_IOCTL_SIZE {
            return Err(Errno::Invalid);
        }
        let mut bytes = zeroed_bytes(spec.size)?;
        if spec.input && spec.size != 0 {
            process
                .address_space()
                .read_user(VirtAddr::new(argument), &mut bytes)
                .map_err(map_memory_error)?;
        }
        if request == crate::driver::class::console::TIOCSPGRP {
            let group =
                i32::from_ne_bytes(bytes.as_slice().try_into().map_err(|_| Errno::Invalid)?);
            if group <= 0 || !proc::process_group_in_session(group as usize, process.session()) {
                return Err(Errno::Permission);
            }
        }
        if request == crate::driver::class::console::TIOCSCTTY
            && process
                .controlling_tty_key()
                .is_some_and(|key| key != file.vnode().key())
        {
            return Err(Errno::Access);
        }
        let result = file
            .ioctl(ioctl_context(&process)?, request, argument, &mut bytes)
            .map_err(|error| match error {
                // A filesystem that does not implement an ioctl treats the
                // request as inappropriate for the object. POSIX reports that
                // as ENOTTY - reporting ENOTSUP instead makes every probe of
                // a plain file look like a hard failure to libc.
                fs::Error::Unsupported => Errno::NotTty,
                other => map_fs_error(other),
            })?;
        match request {
            crate::driver::class::console::TIOCSCTTY => process.set_controlling_tty(file.clone()),
            crate::driver::class::console::TIOCNOTTY if process.session() == process.pid() => {
                proc::clear_session_controlling_tty(process.session(), file.vnode().key());
            }
            crate::driver::class::console::TIOCNOTTY => process.clear_controlling_tty(file.vnode().key()),
            _ => {}
        }
        if spec.output && spec.size != 0 {
            process
                .address_space()
                .write_user(VirtAddr::new(argument), &bytes)
                .map_err(map_memory_error)?;
        }
        Ok(result)
    }
}

crate::syscall_handler! {
    syscall_file_dup(_frame, fd: i32 = 0, minimum: u64 = 1, flags: u64 = 2) {
        if flags & !O_CLOEXEC != 0 {
            return Err(Errno::Invalid);
        }
        let minimum = usize::try_from(minimum).map_err(|_| Errno::Invalid)?;
        current_process()?
            .duplicate_descriptor(fd, minimum, flags & O_CLOEXEC != 0)
            .map(|fd| fd as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_file_dup2(_frame, old_fd: i32 = 0, new_fd: i32 = 1, flags: u64 = 2) {
        if flags & !O_CLOEXEC != 0 || (old_fd == new_fd && flags != 0) {
            return Err(Errno::Invalid);
        }
        current_process()?
            .duplicate_descriptor_to(old_fd, new_fd, flags & O_CLOEXEC != 0)
            .map(|fd| fd as u64)
            .map_err(map_process_error)
    }
}

crate::syscall_handler! {
    syscall_file_umask(_frame, mask: u64 = 0) {
        let process = current_process()?;
        Ok(u64::from(process.set_umask(mask as u16)))
    }
}

crate::syscall_handler! {
    syscall_file_umount(_frame, path: u64 = 0) {
        let process = current_process()?;
        let path = read_user_path(&process, path)?;
        fs::unmount(&path).map_err(map_fs_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_chdir(_frame, path: u64 = 0) {
        let process = current_process()?;
        let path = read_user_path(&process, path)?;
        process.set_cwd(&path).map_err(map_process_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_getcwd(_frame, buffer: u64 = 0, size: u64 = 1) {
        let size = usize::try_from(size).map_err(|_| Errno::Overflow)?;
        let process = current_process()?;
        let cwd = process.cwd();
        let required = cwd.len().checked_add(1).ok_or(Errno::Overflow)?;
        if size < required {
            return Err(Errno::Range);
        }
        let mut bytes = cwd;
        bytes.push(0);
        process
            .address_space()
            .write_user(VirtAddr::new(buffer), &bytes)
            .map_err(map_memory_error)?;
        Ok(buffer)
    }
}

crate::syscall_handler! {
    syscall_file_access(_frame, path: u64 = 0, mode: u64 = 1) {
        let process = current_process()?;
        file_access_at(&process, AT_FDCWD, path, mode, 0)
    }
}

crate::syscall_handler! {
    syscall_file_accessat(
        _frame,
        dirfd: i32 = 0,
        path: u64 = 1,
        mode: u64 = 2,
        flags: u64 = 3,
    ) {
        let process = current_process()?;
        file_access_at(&process, dirfd, path, mode, flags)
    }
}

crate::syscall_handler! {
    syscall_file_stat(_frame, path: u64 = 0, flags: u64 = 1, output: u64 = 2) {
        let process = current_process()?;
        file_stat_at(&process, AT_FDCWD, path, flags, output)
    }
}

crate::syscall_handler! {
    syscall_file_statat(
        _frame,
        dirfd: i32 = 0,
        path: u64 = 1,
        flags: u64 = 2,
        output: u64 = 3,
    ) {
        let process = current_process()?;
        file_stat_at(&process, dirfd, path, flags, output)
    }
}

crate::syscall_handler! {
    syscall_file_fstat(_frame, fd: i32 = 0, output: u64 = 1) {
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        match descriptor {
            Descriptor::File(file) => {
                write_user_stat(&process, output, file.getattr().map_err(map_fs_error)?)
            }
            Descriptor::Pipe(_) => {
                write_user_stat_record(&process, output, UserStat::anonymous(0o666, 6))
            }
            Descriptor::SignalFd(_) => {
                write_user_stat_record(&process, output, UserStat::anonymous(0o600, 1))
            }
            Descriptor::Epoll(_) | Descriptor::Inotify(_) | Descriptor::TimerFd(_) => {
                write_user_stat_record(&process, output, UserStat::anonymous(0o600, 1))
            }
        }
    }
}

crate::syscall_handler! {
    syscall_file_fcntl(_frame, fd: i32 = 0, request: u64 = 1, argument: u64 = 2) {
        let process = current_process()?;
        match request {
            F_DUPFD | F_DUPFD_CLOEXEC => {
                let minimum = usize::try_from(argument).map_err(|_| Errno::Invalid)?;
                process
                    .duplicate_descriptor(fd, minimum, request == F_DUPFD_CLOEXEC)
                    .map(|fd| fd as u64)
                    .map_err(map_process_error)
            }
            F_GETFD => process
                .descriptor_close_on_exec(fd)
                .map(|value| if value { FD_CLOEXEC } else { 0 })
                .ok_or(Errno::BadFileDescriptor),
            F_SETFD => {
                if argument & !FD_CLOEXEC != 0 {
                    return Err(Errno::Invalid);
                }
                process
                    .set_descriptor_close_on_exec(fd, argument & FD_CLOEXEC != 0)
                    .then_some(0)
                    .ok_or(Errno::BadFileDescriptor)
            }
            F_GETFL => match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
                Descriptor::File(file) => Ok(open_flags_to_user(file.flags())),
                Descriptor::Pipe(pipe) => {
                    Ok(pipe.access_mode() | if pipe.is_nonblocking() { O_NONBLOCK } else { 0 })
                }
                Descriptor::SignalFd(signal_fd) => {
                    Ok(if signal_fd.is_nonblocking() {
                        O_NONBLOCK
                    } else {
                        0
                    })
                }
                Descriptor::Epoll(_) => Ok(0),
                Descriptor::Inotify(inotify) => {
                    Ok(if inotify.is_nonblocking() {
                        O_NONBLOCK
                    } else {
                        0
                    })
                }
                Descriptor::TimerFd(timer) => {
                    Ok(if timer.is_nonblocking() {
                        O_NONBLOCK
                    } else {
                        0
                    })
                }
            },
            F_SETFL => {
                if argument & !(O_ACCMODE | O_APPEND | O_NONBLOCK) != 0 {
                    return Err(Errno::Invalid);
                }
                match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
                    Descriptor::File(file) => {
                        file.set_status_flags(argument & O_APPEND != 0, argument & O_NONBLOCK != 0)
                    }
                    Descriptor::Pipe(pipe) => pipe.set_nonblocking(argument & O_NONBLOCK != 0),
                    Descriptor::SignalFd(signal_fd) => {
                        signal_fd.set_nonblocking(argument & O_NONBLOCK != 0)
                    }
                    Descriptor::Epoll(_) => {}
                    Descriptor::Inotify(inotify) => {
                        inotify.set_nonblocking(argument & O_NONBLOCK != 0)
                    }
                    Descriptor::TimerFd(timer) => {
                        timer.set_nonblocking(argument & O_NONBLOCK != 0)
                    }
                }
                Ok(0)
            }
            _ => Err(Errno::Invalid),
        }
    }
}

crate::syscall_handler! {
    syscall_file_pipe(_frame, output: u64 = 0, flags: u64 = 1) {
        if flags & !(PIPE_CLOEXEC | PIPE_NONBLOCK) != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let close_on_exec = flags & PIPE_CLOEXEC != 0;
        let (reader, writer) = PipeEnd::pair();
        if flags & PIPE_NONBLOCK != 0 {
            reader.set_nonblocking(true);
            writer.set_nonblocking(true);
        }
        let read_fd = process
            .install_descriptor_value(Descriptor::Pipe(reader), close_on_exec)
            .map_err(map_process_error)?;
        let write_fd = match process
            .install_descriptor_value(Descriptor::Pipe(writer), close_on_exec)
            .map_err(map_process_error)
        {
            Ok(fd) => fd,
            Err(error) => {
                process.close_descriptor(read_fd);
                return Err(error);
            }
        };
        let mut bytes = [0u8; size_of::<i32>() * 2];
        bytes[..4].copy_from_slice(&read_fd.to_ne_bytes());
        bytes[4..].copy_from_slice(&write_fd.to_ne_bytes());
        if let Err(error) = process
            .address_space()
            .write_user(VirtAddr::new(output), &bytes)
            .map_err(map_memory_error)
        {
            process.close_descriptor(read_fd);
            process.close_descriptor(write_fd);
            return Err(error);
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_readdir(_frame, fd: i32 = 0, buffer: u64 = 1, size: u64 = 2) {
        let size = checked_io_size(size)?;
        let maximum = size / USER_DIRENT_SIZE;
        if maximum == 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let Descriptor::File(file) = descriptor else {
            return Err(Errno::NotDirectory);
        };
        let entries = file.readdir(maximum).map_err(map_fs_error)?;
        let output_size = entries
            .len()
            .checked_mul(USER_DIRENT_SIZE)
            .ok_or(Errno::Overflow)?;
        let mut output = zeroed_bytes(output_size)?;
        for (index, entry) in entries.iter().enumerate() {
            let record = &mut output[index * USER_DIRENT_SIZE..][..USER_DIRENT_SIZE];
            record[..8].copy_from_slice(&entry.key.node.get().to_ne_bytes());
            record[8..16].copy_from_slice(&entry.offset.to_ne_bytes());
            record[16..18].copy_from_slice(&(USER_DIRENT_SIZE as u16).to_ne_bytes());
            record[18] = dirent_kind(entry.kind);
            let length = entry.name.len().min(255);
            record[19..19 + length].copy_from_slice(&entry.name[..length]);
            record[19 + length] = 0;
        }
        process
            .address_space()
            .write_user(VirtAddr::new(buffer), &output)
            .map_err(map_memory_error)?;
        Ok(output.len() as u64)
    }
}

crate::syscall_handler! {
    syscall_file_readlink(_frame, path: u64 = 0, buffer: u64 = 1, size: u64 = 2) {
        let process = current_process()?;
        file_readlink_at(&process, AT_FDCWD, path, buffer, size)
    }
}

crate::syscall_handler! {
    syscall_file_readlinkat(
        _frame,
        dirfd: i32 = 0,
        path: u64 = 1,
        buffer: u64 = 2,
        size: u64 = 3,
    ) {
        let process = current_process()?;
        file_readlink_at(&process, dirfd, path, buffer, size)
    }
}

crate::syscall_handler! {
    syscall_file_unlink(_frame, path: u64 = 0, flags: u64 = 1) {
        let process = current_process()?;
        file_unlink_at(&process, AT_FDCWD, path, flags)
    }
}

crate::syscall_handler! {
    syscall_file_unlinkat(_frame, dirfd: i32 = 0, path: u64 = 1, flags: u64 = 2) {
        let process = current_process()?;
        file_unlink_at(&process, dirfd, path, flags)
    }
}

crate::syscall_handler! {
    syscall_file_mkdir(_frame, path: u64 = 0, mode: u64 = 1) {
        let process = current_process()?;
        file_mkdir_at(&process, AT_FDCWD, path, mode)
    }
}

crate::syscall_handler! {
    syscall_file_mkdirat(_frame, dirfd: i32 = 0, path: u64 = 1, mode: u64 = 2) {
        let process = current_process()?;
        file_mkdir_at(&process, dirfd, path, mode)
    }
}

crate::syscall_handler! {
    syscall_file_fchmod(_frame, fd: i32 = 0, mode: u64 = 1) {
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let Descriptor::File(file) = descriptor else {
            return Err(Errno::BadFileDescriptor);
        };
        file.set_mode(mode as u16).map_err(map_fs_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_rename(_frame, source: u64 = 0, target: u64 = 1) {
        let process = current_process()?;
        file_rename_at(&process, AT_FDCWD, source, AT_FDCWD, target)
    }
}

crate::syscall_handler! {
    syscall_file_renameat(
        _frame,
        source_dirfd: i32 = 0,
        source: u64 = 1,
        target_dirfd: i32 = 2,
        target: u64 = 3,
    ) {
        let process = current_process()?;
        file_rename_at(&process, source_dirfd, source, target_dirfd, target)
    }
}

crate::syscall_handler! {
    syscall_file_fchmodat(
        _frame,
        dirfd: i32 = 0,
        path: u64 = 1,
        mode: u64 = 2,
        flags: u64 = 3,
    ) {
        let process = current_process()?;
        file_chmod_at(&process, dirfd, path, mode, flags)
    }
}

crate::syscall_handler! {
    syscall_file_linkat(
        _frame,
        source_dirfd: i32 = 0,
        source: u64 = 1,
        target_dirfd: i32 = 2,
        target: u64 = 3,
        flags: u64 = 4,
    ) {
        let process = current_process()?;
        file_link_at(
            &process,
            source_dirfd,
            source,
            target_dirfd,
            target,
            flags,
        )
    }
}

crate::syscall_handler! {
    syscall_file_symlinkat(
        _frame,
        target: u64 = 0,
        dirfd: i32 = 1,
        path: u64 = 2,
    ) {
        let process = current_process()?;
        file_symlink_at(&process, target, dirfd, path)
    }
}

crate::syscall_handler! {
    syscall_file_poll(_frame, poll_fds: u64 = 0, count: u64 = 1, timeout_ms: i32 = 2) {
        const POLL_FD_SIZE: usize = size_of::<i32>() + size_of::<i16>() * 2;
        const POLL_INTERVAL_NS: u64 = 1_000_000;

        // A process cannot hold more descriptors than the table allows, so a
        // larger set is always a malformed request rather than a huge wait.
        let count = usize::try_from(count).map_err(|_| Errno::Invalid)?;
        if count > proc::MAX_FILES {
            return Err(Errno::Invalid);
        }
        let byte_len = count
            .checked_mul(POLL_FD_SIZE)
            .ok_or(Errno::Invalid)?;
        let process = current_process()?;
        let mut bytes = zeroed_bytes(byte_len)?;
        if byte_len != 0 {
            process
                .address_space()
                .read_user(VirtAddr::new(poll_fds), &mut bytes)
                .map_err(map_memory_error)?;
        }
        let (records, remainder) = bytes.as_chunks::<POLL_FD_SIZE>();
        debug_assert!(remainder.is_empty());
        let mut entries = Vec::new();
        entries
            .try_reserve_exact(records.len())
            .map_err(|_| Errno::OutOfMemory)?;
        entries.extend(records.iter().map(|record| UserPollFd {
                fd: i32::from_ne_bytes(record[..4].try_into().expect("poll fd width")),
                events: i16::from_ne_bytes(
                    record[4..6].try_into().expect("poll events width"),
                ),
                revents: 0,
            }));
        let deadline = (timeout_ms >= 0).then(|| {
            clock::monotonic_ns().saturating_add((timeout_ms as u64).saturating_mul(1_000_000))
        });

        let mut fds = Vec::new();
        fds.try_reserve_exact(entries.len())
            .map_err(|_| Errno::OutOfMemory)?;
        fds.extend(entries.iter().map(|entry| entry.fd));
        loop {
            let mut ready = 0usize;
            let descriptors = process.descriptors(&fds);
            for (entry, descriptor) in entries.iter_mut().zip(&descriptors) {
                entry.revents = 0;
                if entry.fd < 0 {
                    continue;
                }
                let requested = PollEvents::from_bits_retain(entry.events as u16);
                let events = match descriptor {
                    Some(descriptor) => descriptor.poll(requested),
                    None => PollEvents::NVAL,
                };
                entry.revents = events.bits() as i16;
                if !events.is_empty() {
                    ready += 1;
                }
            }

            let now = clock::monotonic_ns();
            if ready != 0 || timeout_ms == 0 || deadline.is_some_and(|value| now >= value) {
                let (records, remainder) = bytes.as_chunks_mut::<POLL_FD_SIZE>();
                debug_assert!(remainder.is_empty());
                for (entry, record) in entries.iter().zip(records) {
                    record[..4].copy_from_slice(&entry.fd.to_ne_bytes());
                    record[4..6].copy_from_slice(&entry.events.to_ne_bytes());
                    record[6..8].copy_from_slice(&entry.revents.to_ne_bytes());
                }
                if byte_len != 0 {
                    process
                        .address_space()
                        .write_user(VirtAddr::new(poll_fds), &bytes)
                        .map_err(map_memory_error)?;
                }
                return Ok(ready as u64);
            }

            // Arm signal interruption before sleeping so a blocked poll can be
            // aborted by a signal instead of waiting forever.
            if process.prepare_interrupt_wait() {
                return Err(Errno::Interrupted);
            }
            let mut wait_events = Vec::new();
            wait_events
                .try_reserve(entries.len().saturating_add(1))
                .map_err(|_| Errno::OutOfMemory)?;
            wait_events.push(process.interrupt_event());
            let mut event_driven = true;
            for (entry, descriptor) in entries.iter().zip(&descriptors) {
                if entry.fd < 0 {
                    continue;
                }
                let Some(descriptor) = descriptor else {
                    continue;
                };
                let requested = PollEvents::from_bits_retain(entry.events as u16)
                    | PollEvents::ERR
                    | PollEvents::HUP;
                event_driven &= descriptor.poll_events(requested, &mut wait_events);
            }
            wait_events.sort_unstable_by_key(|event| *event as *const Event as usize);
            wait_events.dedup_by_key(|event| *event as *const Event as usize);
            if event_driven && wait_events.len() > 1 {
                if let Some(deadline) = deadline {
                    let remaining = deadline.saturating_sub(now).max(1);
                    let _ = clock::wait_any_timeout(
                        &wait_events,
                        Duration::from_nanos(remaining),
                    );
                } else {
                    Event::wait_any(&wait_events);
                }
            } else {
                let sleep_ns = deadline
                    .map(|value| value.saturating_sub(now).min(POLL_INTERVAL_NS))
                    .unwrap_or(POLL_INTERVAL_NS);
                clock::sleep(Duration::from_nanos(sleep_ns));
            }
        }
    }
}

fn file_open_at(process: &Process, dirfd: i32, path: u64, flags: u64, mode: u64) -> Result<u64> {
    let AtPath::Vfs { base, path } = resolve_at_path(process, dirfd, path, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    let open_flags = parse_open_flags(flags)?;
    let mode = process.apply_umask(mode as u16);
    let file = fs::open_at(&base, &path, open_flags, mode).map_err(map_fs_error)?;
    let fd = process
        .install_file(file.clone(), flags & O_CLOEXEC != 0)
        .map_err(map_process_error)?;
    if flags & O_NOCTTY == 0
        && process.session() == process.pid()
        && !process.has_controlling_tty()
        && file
            .ioctl(
                ioctl_context(process)?,
                crate::driver::class::console::TIOCSCTTY,
                0,
                &mut [],
            )
            .is_ok()
    {
        process.set_controlling_tty(file);
    }
    Ok(fd as u64)
}

fn file_access_at(process: &Process, dirfd: i32, path: u64, mode: u64, flags: u64) -> Result<u64> {
    if mode & !0o7 != 0 || flags & !(AT_EACCESS | AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH) != 0 {
        return Err(Errno::Invalid);
    }
    let empty = if flags & AT_EMPTY_PATH != 0 {
        EmptyPath::AllowAny
    } else {
        EmptyPath::Reject
    };
    let mode_bits = match resolve_at_path(process, dirfd, path, empty)? {
        AtPath::Vfs { base, path } => {
            lookup_at(&base, &path, flags & AT_SYMLINK_NOFOLLOW == 0)?
                .getattr()
                .map_err(map_fs_error)?
                .mode
        }
        AtPath::Pipe => 0o666,
        AtPath::Anonymous => 0o600,
    };
    check_access(mode_bits, mode)?;
    Ok(0)
}

fn file_stat_at(process: &Process, dirfd: i32, path: u64, flags: u64, output: u64) -> Result<u64> {
    if flags & !(AT_SYMLINK_NOFOLLOW | AT_NO_AUTOMOUNT | AT_EMPTY_PATH) != 0 {
        return Err(Errno::Invalid);
    }
    let empty = if flags & AT_EMPTY_PATH != 0 {
        EmptyPath::AllowAny
    } else {
        EmptyPath::Reject
    };
    match resolve_at_path(process, dirfd, path, empty)? {
        AtPath::Vfs { base, path } => {
            let vnode = lookup_at(&base, &path, flags & AT_SYMLINK_NOFOLLOW == 0)?;
            write_user_stat(process, output, vnode.getattr().map_err(map_fs_error)?)
        }
        AtPath::Pipe => write_user_stat_record(process, output, UserStat::anonymous(0o666, 6)),
        AtPath::Anonymous => write_user_stat_record(process, output, UserStat::anonymous(0o600, 1)),
    }
}

fn file_readlink_at(
    process: &Process,
    dirfd: i32,
    path: u64,
    buffer: u64,
    size: u64,
) -> Result<u64> {
    let size = clamped_io_size(size)?;
    if size == 0 {
        return Err(Errno::Invalid);
    }
    let AtPath::Vfs { base, path } = resolve_at_path(process, dirfd, path, EmptyPath::AllowVfs)?
    else {
        unreachable!("pipe paths are rejected by the resolver");
    };
    let target = lookup_at(&base, &path, false)?
        .readlink()
        .map_err(map_fs_error)?;
    let length = target.len().min(size);
    process
        .address_space()
        .write_user(VirtAddr::new(buffer), &target[..length])
        .map_err(map_memory_error)?;
    Ok(length as u64)
}

fn file_unlink_at(process: &Process, dirfd: i32, path: u64, flags: u64) -> Result<u64> {
    if flags & !AT_REMOVEDIR != 0 {
        return Err(Errno::Invalid);
    }
    let AtPath::Vfs { base, path } = resolve_at_path(process, dirfd, path, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    fs::unlink_at(&base, &path, flags & AT_REMOVEDIR != 0).map_err(map_fs_error)?;
    Ok(0)
}

fn file_mkdir_at(process: &Process, dirfd: i32, path: u64, mode: u64) -> Result<u64> {
    let AtPath::Vfs { base, path } = resolve_at_path(process, dirfd, path, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    fs::create_dir_at(&base, &path, process.apply_umask(mode as u16)).map_err(map_fs_error)?;
    Ok(0)
}

fn file_chmod_at(process: &Process, dirfd: i32, path: u64, mode: u64, flags: u64) -> Result<u64> {
    if flags & !(AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH) != 0 {
        return Err(Errno::Invalid);
    }
    let empty = if flags & AT_EMPTY_PATH != 0 {
        EmptyPath::AllowVfs
    } else {
        EmptyPath::Reject
    };
    let AtPath::Vfs { base, path } = resolve_at_path(process, dirfd, path, empty)? else {
        unreachable!("pipe paths are rejected by the resolver");
    };
    let vnode = lookup_at(&base, &path, flags & AT_SYMLINK_NOFOLLOW == 0)?;
    if flags & AT_SYMLINK_NOFOLLOW != 0 && vnode.kind() == VnodeKind::Symlink {
        return Err(Errno::NotSupported);
    }
    vnode
        .setattr(fs::SetAttr {
            size: None,
            mode: Some(mode as u16),
        })
        .map_err(map_fs_error)?;
    Ok(0)
}

fn file_rename_at(
    process: &Process,
    source_dirfd: i32,
    source: u64,
    target_dirfd: i32,
    target: u64,
) -> Result<u64> {
    let AtPath::Vfs {
        base: source_base,
        path: source,
    } = resolve_at_path(process, source_dirfd, source, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    let AtPath::Vfs {
        base: target_base,
        path: target,
    } = resolve_at_path(process, target_dirfd, target, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    fs::rename_at(&source_base, &source, &target_base, &target).map_err(map_fs_error)?;
    Ok(0)
}

fn file_link_at(
    process: &Process,
    source_dirfd: i32,
    source: u64,
    target_dirfd: i32,
    target: u64,
    flags: u64,
) -> Result<u64> {
    if flags & !(AT_SYMLINK_FOLLOW | AT_EMPTY_PATH) != 0 {
        return Err(Errno::Invalid);
    }
    let source_empty = if flags & AT_EMPTY_PATH != 0 {
        EmptyPath::AllowVfs
    } else {
        EmptyPath::Reject
    };
    let AtPath::Vfs {
        base: source_base,
        path: source,
    } = resolve_at_path(process, source_dirfd, source, source_empty)?
    else {
        unreachable!("pipe paths are rejected by the resolver");
    };
    let AtPath::Vfs {
        base: target_base,
        path: target,
    } = resolve_at_path(process, target_dirfd, target, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    fs::link_at(
        &source_base,
        &source,
        flags & AT_SYMLINK_FOLLOW != 0,
        &target_base,
        &target,
    )
    .map_err(map_fs_error)?;
    Ok(0)
}

fn file_symlink_at(process: &Process, target: u64, dirfd: i32, path: u64) -> Result<u64> {
    let target = read_user_path(process, target)?;
    let AtPath::Vfs { base, path } = resolve_at_path(process, dirfd, path, EmptyPath::Reject)?
    else {
        unreachable!("non-empty paths cannot resolve to pipes");
    };
    fs::symlink_at(&target, &base, &path).map_err(map_fs_error)?;
    Ok(0)
}

fn resolve_at_path(
    process: &Process,
    dirfd: i32,
    address: u64,
    empty: EmptyPath,
) -> Result<AtPath> {
    let path = read_user_path(process, address)?;
    if path.first() == Some(&b'/') {
        return Ok(AtPath::Vfs {
            base: fs::root_anchor().map_err(map_fs_error)?,
            path,
        });
    }
    if path.is_empty() && matches!(empty, EmptyPath::Reject) {
        return Err(Errno::NoEntry);
    }

    if dirfd == AT_FDCWD {
        return Ok(AtPath::Vfs {
            base: process.cwd_anchor(),
            path: if path.is_empty() { b".".to_vec() } else { path },
        });
    }

    match process.descriptor(dirfd).ok_or(Errno::BadFileDescriptor)? {
        Descriptor::File(file) => {
            if !path.is_empty() && file.vnode().kind() != VnodeKind::Directory {
                return Err(Errno::NotDirectory);
            }
            Ok(AtPath::Vfs {
                base: file.path_anchor(),
                path: if path.is_empty() { b".".to_vec() } else { path },
            })
        }
        Descriptor::Pipe(_) if path.is_empty() && matches!(empty, EmptyPath::AllowAny) => {
            Ok(AtPath::Pipe)
        }
        Descriptor::Pipe(_) if path.is_empty() => Err(Errno::BadFileDescriptor),
        Descriptor::Pipe(_) => Err(Errno::NotDirectory),
        Descriptor::SignalFd(_) if path.is_empty() && matches!(empty, EmptyPath::AllowAny) => {
            Ok(AtPath::Anonymous)
        }
        Descriptor::SignalFd(_) if path.is_empty() => Err(Errno::BadFileDescriptor),
        Descriptor::SignalFd(_) => Err(Errno::NotDirectory),
        Descriptor::Epoll(_) | Descriptor::Inotify(_) | Descriptor::TimerFd(_)
            if path.is_empty() && matches!(empty, EmptyPath::AllowAny) =>
        {
            Ok(AtPath::Anonymous)
        }
        Descriptor::Epoll(_) | Descriptor::Inotify(_) | Descriptor::TimerFd(_)
            if path.is_empty() =>
        {
            Err(Errno::BadFileDescriptor)
        }
        Descriptor::Epoll(_) | Descriptor::Inotify(_) | Descriptor::TimerFd(_) => {
            Err(Errno::NotDirectory)
        }
    }
}

fn lookup_at(base: &PathAnchor, path: &[u8], follow_final: bool) -> Result<Vnode> {
    fs::resolve_at(base, path, follow_final)
        .map(|anchor| anchor.vnode().clone())
        .map_err(map_fs_error)
}

fn parse_open_flags(flags: u64) -> Result<OpenFlags> {
    let allowed = O_ACCMODE
        | O_CREAT
        | O_EXCL
        | O_NOCTTY
        | O_TRUNC
        | O_APPEND
        | O_NONBLOCK
        | O_LARGEFILE
        | O_DIRECTORY
        | O_NOFOLLOW
        | O_CLOEXEC;
    if flags & !allowed != 0 {
        return Err(Errno::Invalid);
    }

    let mut open_flags = match flags & O_ACCMODE {
        0 => OpenFlags::READ,
        O_WRONLY => OpenFlags::WRITE,
        O_RDWR => OpenFlags::READ | OpenFlags::WRITE,
        _ => return Err(Errno::Invalid),
    };
    open_flags.set(OpenFlags::CREATE, flags & O_CREAT != 0);
    open_flags.set(OpenFlags::EXCLUSIVE, flags & O_EXCL != 0);
    open_flags.set(OpenFlags::TRUNCATE, flags & O_TRUNC != 0);
    open_flags.set(OpenFlags::APPEND, flags & O_APPEND != 0);
    open_flags.set(OpenFlags::DIRECTORY, flags & O_DIRECTORY != 0);
    open_flags.set(OpenFlags::NOFOLLOW, flags & O_NOFOLLOW != 0);
    open_flags.set(OpenFlags::NONBLOCK, flags & O_NONBLOCK != 0);
    open_flags.set(OpenFlags::NOCTTY, flags & O_NOCTTY != 0);
    Ok(open_flags)
}

fn check_access(mode_bits: u16, requested: u64) -> Result<()> {
    if requested & 0o4 != 0 && mode_bits & 0o444 == 0 {
        return Err(Errno::Access);
    }
    if requested & 0o2 != 0 && mode_bits & 0o222 == 0 {
        return Err(Errno::Access);
    }
    if requested & 0o1 != 0 && mode_bits & 0o111 == 0 {
        return Err(Errno::Access);
    }
    Ok(())
}

fn ioctl_context(process: &Process) -> Result<IoctlContext> {
    Ok(IoctlContext {
        process_id: process.pid(),
        process_group: i32::try_from(process.process_group()).map_err(|_| Errno::Overflow)?,
        session_id: i32::try_from(process.session()).map_err(|_| Errno::Overflow)?,
        is_session_leader: process.session() == process.pid(),
    })
}

fn write_user_stat(process: &Process, output: u64, attributes: VnodeAttr) -> Result<u64> {
    write_user_stat_record(
        process,
        output,
        UserStat {
            device: attributes.key.filesystem.get(),
            inode: attributes.key.node.get(),
            mode: u32::from(attributes.mode),
            kind: vnode_kind(attributes.kind),
            links: attributes.links,
            size: attributes.size,
            accessed_ns: attributes.accessed_ns,
            modified_ns: attributes.modified_ns,
            changed_ns: attributes.changed_ns,
        },
    )
}

/// The complete record is assembled once and copied to user memory in a single
/// transfer, so a partially written structure can never be observed.
fn write_user_stat_record(process: &Process, output: u64, stat: UserStat) -> Result<u64> {
    let mut bytes = [0u8; 64];
    bytes[0..8].copy_from_slice(&stat.device.to_ne_bytes());
    bytes[8..16].copy_from_slice(&stat.inode.to_ne_bytes());
    bytes[16..20].copy_from_slice(&stat.mode.to_ne_bytes());
    bytes[20..24].copy_from_slice(&stat.kind.to_ne_bytes());
    bytes[24..32].copy_from_slice(&stat.links.to_ne_bytes());
    bytes[32..40].copy_from_slice(&stat.size.to_ne_bytes());
    bytes[40..48].copy_from_slice(&stat.accessed_ns.to_ne_bytes());
    bytes[48..56].copy_from_slice(&stat.modified_ns.to_ne_bytes());
    bytes[56..64].copy_from_slice(&stat.changed_ns.to_ne_bytes());
    process
        .address_space()
        .write_user(VirtAddr::new(output), &bytes)
        .map_err(map_memory_error)?;
    Ok(0)
}

fn vnode_kind(kind: VnodeKind) -> u32 {
    match kind {
        VnodeKind::Regular => 1,
        VnodeKind::Directory => 2,
        VnodeKind::Symlink => 3,
        VnodeKind::CharacterDevice => 4,
        VnodeKind::BlockDevice => 5,
        VnodeKind::Fifo => 6,
        VnodeKind::Socket => 7,
    }
}

fn dirent_kind(kind: VnodeKind) -> u8 {
    match kind {
        VnodeKind::Regular => 8,
        VnodeKind::Directory => 4,
        VnodeKind::Symlink => 10,
        VnodeKind::CharacterDevice => 2,
        VnodeKind::BlockDevice => 6,
        VnodeKind::Fifo => 1,
        VnodeKind::Socket => 12,
    }
}

fn open_flags_to_user(flags: OpenFlags) -> u64 {
    let mut result = match (
        flags.contains(OpenFlags::READ),
        flags.contains(OpenFlags::WRITE),
    ) {
        (true, true) => O_RDWR,
        (false, true) => O_WRONLY,
        _ => 0,
    };
    if flags.contains(OpenFlags::APPEND) {
        result |= O_APPEND;
    }
    if flags.contains(OpenFlags::NONBLOCK) {
        result |= O_NONBLOCK;
    }
    result
}

fn descriptor_read(descriptor: &Descriptor, sink: &mut IoSink<'_>) -> Result<usize> {
    match descriptor {
        Descriptor::File(file) => file.read(sink).map_err(map_fs_error),
        Descriptor::Pipe(pipe) => pipe.read(sink).map_err(map_pipe_error),
        Descriptor::SignalFd(signal_fd) => signal_fd.read(sink),
        Descriptor::Epoll(_) => Err(Errno::BadFileDescriptor),
        Descriptor::Inotify(inotify) => inotify.read(sink),
        Descriptor::TimerFd(timer) => timer.read(sink),
    }
}

fn descriptor_write(descriptor: &Descriptor, source: &IoSource<'_>) -> Result<usize> {
    match descriptor {
        Descriptor::File(file) => file.write(source).map_err(map_fs_error),
        Descriptor::Pipe(pipe) => pipe.write(source).map_err(map_pipe_error),
        Descriptor::SignalFd(_)
        | Descriptor::Epoll(_)
        | Descriptor::Inotify(_)
        | Descriptor::TimerFd(_) => Err(Errno::BadFileDescriptor),
    }
}

fn check_terminal_job_control(
    process: &Process,
    descriptor: &Descriptor,
    writing: bool,
) -> Result<()> {
    let Descriptor::File(file) = descriptor else {
        return Ok(());
    };
    if process.controlling_tty_key() != Some(file.vnode().key()) {
        return Ok(());
    }
    let Some(terminal) = file.terminal_state() else {
        return Ok(());
    };
    if terminal.session <= 0
        || terminal.session as usize != process.session()
        || terminal.foreground_group <= 0
        || terminal.foreground_group as usize == process.process_group()
        || writing && !terminal.stop_background_output
    {
        return Ok(());
    }

    let signal = if writing {
        proc::signal::SIGTTOU
    } else {
        proc::signal::SIGTTIN
    };
    proc::signal::send_kernel_process_group(process.process_group(), signal);
    Err(Errno::Interrupted)
}

/// Clamps a user-supplied transfer length to the per-call maximum.
///
/// POSIX allows a short transfer, so an oversized request is truncated rather
/// than rejected.
fn clamped_io_size(size: u64) -> Result<usize> {
    Ok(usize::try_from(size).unwrap_or(usize::MAX).min(MAX_IO_SIZE))
}

fn checked_io_size(size: u64) -> Result<usize> {
    let size = usize::try_from(size).map_err(|_| Errno::Overflow)?;
    if size > MAX_IO_SIZE {
        return Err(Errno::Invalid);
    }
    Ok(size)
}

fn zeroed_bytes(length: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| Errno::OutOfMemory)?;
    bytes.resize(length, 0);
    Ok(bytes)
}

fn map_pipe_error(error: PipeError) -> Errno {
    match error {
        PipeError::BadDescriptor => Errno::BadFileDescriptor,
        PipeError::TryAgain => Errno::TryAgain,
        PipeError::Fault => Errno::Fault,
        PipeError::BrokenPipe => {
            if let Some(process) = proc::current() {
                proc::signal::send_kernel(&process, proc::signal::SIGPIPE);
            }
            Errno::BrokenPipe
        }
    }
}
