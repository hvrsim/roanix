//! Filesystem syscall implementations.

use alloc::{vec, vec::Vec};
use core::{mem::size_of, time::Duration};

use crate::{
    fs::{self, IoctlContext, OpenFlags, PollEvents, SeekFrom, VnodeAttr, VnodeKind},
    mem::VirtAddr,
    proc::{self, Descriptor, PipeEnd, PipeError, Process},
    sys::clock,
    syscall::{
        Errno, Result, current_process, map_fs_error, map_memory_error, map_process_error,
        read_user_string,
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
const AT_SYMLINK_NOFOLLOW: u64 = 0x100;
const AT_REMOVEDIR: u64 = 0x200;
const USER_DIRENT_SIZE: usize = 280;

const SEEK_SET: u64 = 0;
const SEEK_CUR: u64 = 1;
const SEEK_END: u64 = 2;

#[derive(Clone, Copy)]
struct UserPollFd {
    fd: i32,
    events: i16,
    revents: i16,
}

crate::syscall_handler! {
    syscall_file_open(_frame, path: u64 = 0, flags: u64 = 1, mode: u64 = 2) {
        let process = current_process()?;
        let path = process.resolve_path(&read_user_string(&process, path)?);
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
        if flags & O_CREAT != 0 {
            open_flags |= OpenFlags::CREATE;
        }
        if flags & O_EXCL != 0 {
            open_flags |= OpenFlags::EXCLUSIVE;
        }
        if flags & O_TRUNC != 0 {
            open_flags |= OpenFlags::TRUNCATE;
        }
        if flags & O_APPEND != 0 {
            open_flags |= OpenFlags::APPEND;
        }
        if flags & O_DIRECTORY != 0 {
            open_flags |= OpenFlags::DIRECTORY;
        }
        if flags & O_NOFOLLOW != 0 {
            open_flags |= OpenFlags::NOFOLLOW;
        }
        if flags & O_NONBLOCK != 0 {
            open_flags |= OpenFlags::NONBLOCK;
        }
        if flags & O_NOCTTY != 0 {
            open_flags |= OpenFlags::NOCTTY;
        }

        let file = fs::open(&path, open_flags, mode as u16).map_err(map_fs_error)?;
        let fd = process
            .install_file(file.clone(), flags & O_CLOEXEC != 0)
            .map_err(map_process_error)?;
        if flags & O_NOCTTY == 0
            && process.session() == process.pid()
            && !process.has_controlling_tty()
            && file
                .ioctl(
                    ioctl_context(&process)?,
                    crate::dev::console::TIOCSCTTY,
                    0,
                    &mut [],
                )
                .is_ok()
        {
            process.set_controlling_tty(file.clone());
        }
        Ok(fd as u64)
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
        let size = checked_io_size(size)?;
        if size == 0 {
            return Ok(0);
        }
        let process = current_process()?;
        let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
        let mut bytes = vec![0u8; size];
        let read = match descriptor {
            Descriptor::File(file) => file.read(&mut bytes).map_err(map_fs_error)?,
            Descriptor::Pipe(pipe) => pipe.read(&mut bytes).map_err(map_pipe_error)?,
        };
        process
            .address_space()
            .write_user(VirtAddr::new(buffer), &bytes[..read])
            .map_err(map_memory_error)?;
        Ok(read as u64)
    }
}

crate::syscall_handler! {
    syscall_file_write(_frame, fd: i32 = 0, buffer: u64 = 1, size: u64 = 2) {
        let size = checked_io_size(size)?;
        if size == 0 {
            return Ok(0);
        }
        let process = current_process()?;
        let mut bytes = vec![0u8; size];
        process
            .address_space()
            .read_user(VirtAddr::new(buffer), &mut bytes)
            .map_err(map_memory_error)?;
        let written = match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
            Descriptor::File(file) => file.write(&bytes).map_err(map_fs_error)?,
            Descriptor::Pipe(pipe) => pipe.write(&bytes).map_err(map_pipe_error)?,
        };
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
        let spec = crate::dev::console::ioctl_spec(request);
        if spec.size > MAX_IOCTL_SIZE {
            return Err(Errno::Invalid);
        }
        let mut bytes = vec![0u8; spec.size];
        if spec.input && spec.size != 0 {
            process
                .address_space()
                .read_user(VirtAddr::new(argument), &mut bytes)
                .map_err(map_memory_error)?;
        }
        if request == crate::dev::console::TIOCSPGRP {
            let group =
                i32::from_ne_bytes(bytes.as_slice().try_into().map_err(|_| Errno::Invalid)?);
            if group <= 0 || !proc::process_group_in_session(group as usize, process.session()) {
                return Err(Errno::Permission);
            }
            if request == crate::dev::console::TIOCSCTTY
                && process
                    .controlling_tty_key()
                    .is_some_and(|key| key != file.vnode().key())
            {
                return Err(Errno::Access);
            }
        }
        let result = file
            .ioctl(ioctl_context(&process)?, request, argument, &mut bytes)
            .map_err(map_fs_error)?;
        match request {
            crate::dev::console::TIOCSCTTY => process.set_controlling_tty(file.clone()),
            crate::dev::console::TIOCNOTTY if process.session() == process.pid() => {
                proc::clear_session_controlling_tty(process.session(), file.vnode().key());
            }
            crate::dev::console::TIOCNOTTY => process.clear_controlling_tty(file.vnode().key()),
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
    syscall_file_chdir(_frame, path: u64 = 0) {
        let process = current_process()?;
        let path = read_user_string(&process, path)?;
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
        let mut bytes = cwd.into_bytes();
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
        if mode & !0o7 != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let path = process.resolve_path(&read_user_string(&process, path)?);
        let attributes = fs::lookup(&path)
            .map_err(map_fs_error)?
            .getattr()
            .map_err(map_fs_error)?;
        if mode & 0o4 != 0 && attributes.mode & 0o444 == 0 {
            return Err(Errno::Access);
        }
        if mode & 0o2 != 0 && attributes.mode & 0o222 == 0 {
            return Err(Errno::Access);
        }
        if mode & 0o1 != 0 && attributes.mode & 0o111 == 0 {
            return Err(Errno::Access);
        }
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_stat(_frame, path: u64 = 0, flags: u64 = 1, output: u64 = 2) {
        if flags & !AT_SYMLINK_NOFOLLOW != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let path = process.resolve_path(&read_user_string(&process, path)?);
        let vnode = if flags & AT_SYMLINK_NOFOLLOW != 0 {
            fs::lookup_nofollow(&path)
        } else {
            fs::lookup(&path)
        }
        .map_err(map_fs_error)?;
        write_user_stat(&process, output, vnode.getattr().map_err(map_fs_error)?)
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
                write_user_stat_values(&process, output, 0, 0, 0o666, 6, 1, 0)
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
                }
                Ok(0)
            }
            _ => Err(Errno::Invalid),
        }
    }
}

crate::syscall_handler! {
    syscall_file_pipe(_frame, output: u64 = 0, flags: u64 = 1) {
        if flags & !PIPE_CLOEXEC != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let close_on_exec = flags & PIPE_CLOEXEC != 0;
        let (reader, writer) = PipeEnd::pair();
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
        let mut output = vec![0u8; entries.len() * USER_DIRENT_SIZE];
        for (index, entry) in entries.iter().enumerate() {
            let record = &mut output[index * USER_DIRENT_SIZE..][..USER_DIRENT_SIZE];
            record[..8].copy_from_slice(&entry.key.node.get().to_ne_bytes());
            record[8..16].copy_from_slice(&0i64.to_ne_bytes());
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
        let size = checked_io_size(size)?;
        let process = current_process()?;
        let path = process.resolve_path(&read_user_string(&process, path)?);
        let target = fs::lookup_nofollow(&path)
            .map_err(map_fs_error)?
            .readlink()
            .map_err(map_fs_error)?;
        let length = target.len().min(size);
        process
            .address_space()
            .write_user(VirtAddr::new(buffer), &target[..length])
            .map_err(map_memory_error)?;
        Ok(length as u64)
    }
}

crate::syscall_handler! {
    syscall_file_unlink(_frame, path: u64 = 0, flags: u64 = 1) {
        if flags & !AT_REMOVEDIR != 0 {
            return Err(Errno::Invalid);
        }
        let process = current_process()?;
        let path = process.resolve_path(&read_user_string(&process, path)?);
        if flags & AT_REMOVEDIR != 0 {
            fs::remove_dir(&path)
        } else {
            fs::unlink(&path)
        }
        .map_err(map_fs_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_mkdir(_frame, path: u64 = 0, mode: u64 = 1) {
        let process = current_process()?;
        let path = process.resolve_path(&read_user_string(&process, path)?);
        fs::create_dir(&path, mode as u16).map_err(map_fs_error)?;
        Ok(0)
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
        let source = process.resolve_path(&read_user_string(&process, source)?);
        let target = process.resolve_path(&read_user_string(&process, target)?);
        fs::rename(&source, &target).map_err(map_fs_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_file_poll(_frame, poll_fds: u64 = 0, count: u64 = 1, timeout_ms: i32 = 2) {
        const POLL_FD_SIZE: usize = size_of::<i32>() + size_of::<i16>() * 2;
        const POLL_INTERVAL_NS: u64 = 1_000_000;

        let count = usize::try_from(count).map_err(|_| Errno::Invalid)?;
        let byte_len = count
            .checked_mul(POLL_FD_SIZE)
            .filter(|length| *length <= MAX_IO_SIZE)
            .ok_or(Errno::Invalid)?;
        let process = current_process()?;
        let mut bytes = vec![0u8; byte_len];
        if byte_len != 0 {
            process
                .address_space()
                .read_user(VirtAddr::new(poll_fds), &mut bytes)
                .map_err(map_memory_error)?;
        }
        let mut entries = bytes
            .chunks_exact(POLL_FD_SIZE)
            .map(|record| UserPollFd {
                fd: i32::from_ne_bytes(record[..4].try_into().expect("poll fd width")),
                events: i16::from_ne_bytes(
                    record[4..6].try_into().expect("poll events width"),
                ),
                revents: 0,
            })
            .collect::<Vec<_>>();
        let deadline = (timeout_ms >= 0).then(|| {
            clock::monotonic_ns().saturating_add((timeout_ms as u64).saturating_mul(1_000_000))
        });

        loop {
            let mut ready = 0usize;
            for entry in &mut entries {
                entry.revents = 0;
                if entry.fd < 0 {
                    continue;
                }
                let requested = PollEvents::from_bits_retain(entry.events as u16);
                let events = match process.descriptor(entry.fd) {
                    Some(Descriptor::File(file)) => {
                        file.poll(requested).unwrap_or(PollEvents::ERR)
                    }
                    Some(Descriptor::Pipe(pipe)) => pipe.poll(requested),
                    None => PollEvents::NVAL,
                };
                entry.revents = events.bits() as i16;
                if !events.is_empty() {
                    ready += 1;
                }
            }

            let now = clock::monotonic_ns();
            if ready != 0 || timeout_ms == 0 || deadline.is_some_and(|value| now >= value) {
                for (entry, record) in entries.iter().zip(bytes.chunks_exact_mut(POLL_FD_SIZE)) {
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

            let sleep_ns = deadline
                .map(|value| value.saturating_sub(now).min(POLL_INTERVAL_NS))
                .unwrap_or(POLL_INTERVAL_NS);
            clock::sleep(Duration::from_nanos(sleep_ns));
        }
    }
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
    write_user_stat_values(
        process,
        output,
        attributes.key.filesystem.get(),
        attributes.key.node.get(),
        u32::from(attributes.mode),
        vnode_kind(attributes.kind),
        attributes.links,
        attributes.size,
    )?;
    let mut times = [0u8; 24];
    times[0..8].copy_from_slice(&attributes.accessed_ns.to_ne_bytes());
    times[8..16].copy_from_slice(&attributes.modified_ns.to_ne_bytes());
    times[16..24].copy_from_slice(&attributes.changed_ns.to_ne_bytes());
    let times_output = output.checked_add(40).ok_or(Errno::Fault)?;
    process
        .address_space()
        .write_user(VirtAddr::new(times_output), &times)
        .map_err(map_memory_error)?;
    Ok(0)
}

#[allow(clippy::too_many_arguments)]
fn write_user_stat_values(
    process: &Process,
    output: u64,
    device: u64,
    inode: u64,
    mode: u32,
    kind: u32,
    links: u64,
    size: u64,
) -> Result<u64> {
    let mut bytes = [0u8; 64];
    bytes[0..8].copy_from_slice(&device.to_ne_bytes());
    bytes[8..16].copy_from_slice(&inode.to_ne_bytes());
    bytes[16..20].copy_from_slice(&mode.to_ne_bytes());
    bytes[20..24].copy_from_slice(&kind.to_ne_bytes());
    bytes[24..32].copy_from_slice(&links.to_ne_bytes());
    bytes[32..40].copy_from_slice(&size.to_ne_bytes());
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

fn checked_io_size(size: u64) -> Result<usize> {
    let size = usize::try_from(size).map_err(|_| Errno::Overflow)?;
    if size > MAX_IO_SIZE {
        return Err(Errno::Invalid);
    }
    Ok(size)
}

fn map_pipe_error(error: PipeError) -> Errno {
    match error {
        PipeError::BadDescriptor => Errno::BadFileDescriptor,
        PipeError::TryAgain => Errno::TryAgain,
        PipeError::BrokenPipe => Errno::BrokenPipe,
    }
}
