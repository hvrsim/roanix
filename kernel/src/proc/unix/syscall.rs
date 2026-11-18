//! Fixed-width native socket ABI; libc translates sockaddr and ancillary data.

use super::rights::MAX_RIGHTS;
use super::{Address, DONTWAIT, Kind, MAX_MESSAGE, PEEK, TRUNC, UnixSocket};
use crate::{
    mem::VirtAddr,
    proc::{Descriptor, Process},
    syscall::{Errno, Result, current_process, map_memory_error, map_process_error},
};
use alloc::{sync::Arc, vec::Vec};

const EXTENDED: u64 = 16;
const RIGHTS_CLOEXEC: u64 = 32;

fn socket(process: &Process, fd: i32) -> Result<Arc<UnixSocket>> {
    match process.descriptor(fd).ok_or(Errno::BadFileDescriptor)? {
        Descriptor::Unix(socket) => Ok(socket),
        _ => Err(Errno::NotSocket),
    }
}
fn read<const N: usize>(process: &Process, address: u64) -> Result<[u8; N]> {
    let mut bytes = [0; N];
    process
        .address_space()
        .read_user(VirtAddr::new(address), &mut bytes)
        .map_err(map_memory_error)?;
    Ok(bytes)
}
fn write(process: &Process, address: u64, bytes: &[u8]) -> Result<()> {
    process
        .address_space()
        .write_user(VirtAddr::new(address), bytes)
        .map_err(map_memory_error)
}
// The original 40-byte record remains valid without the extension flag.
fn read_message(process: &Process, pointer: u64, flags: u64) -> Result<[u8; 56]> {
    if flags & EXTENDED != 0 {
        return read(process, pointer);
    }
    let mut header = [0; 56];
    header[..40].copy_from_slice(&read::<40>(process, pointer)?);
    Ok(header)
}
fn word(bytes: &[u8], offset: usize) -> u64 {
    u64::from_ne_bytes(bytes[offset..offset + 8].try_into().unwrap())
}
fn address(process: &Process, pointer: u64) -> Result<Option<Address>> {
    let bytes = read::<116>(process, pointer)?;
    let kind = u32::from_ne_bytes(bytes[..4].try_into().unwrap());
    let length = u32::from_ne_bytes(bytes[4..8].try_into().unwrap()) as usize;
    if length > 107 {
        return Err(Errno::Invalid);
    }
    let data = &bytes[8..8 + length];
    match kind {
        0 if length == 0 => Ok(Some(Address::Unnamed)),
        1 if length != 0 && !data.contains(&0) => Ok(Some(Address::Path(data.to_vec()))),
        2 => Ok(Some(Address::Abstract(data.to_vec()))),
        4 if length == 8 => Ok(Some(Address::Netlink {
            port: u32::from_ne_bytes(data[..4].try_into().unwrap()),
            groups: u32::from_ne_bytes(data[4..].try_into().unwrap()),
        })),
        3 if length == 0 => Ok(None), // Explicit disconnect.
        _ => Err(Errno::Invalid),
    }
}
fn write_address(process: &Process, pointer: u64, address: &Address) -> Result<()> {
    if pointer == 0 {
        return Ok(());
    }
    let mut bytes = [0u8; 116];
    let mut netlink = [0u8; 8];
    let (kind, name): (u32, &[u8]) = match address {
        Address::Netlink { port, groups } => {
            netlink[..4].copy_from_slice(&port.to_ne_bytes());
            netlink[4..].copy_from_slice(&groups.to_ne_bytes());
            (4, &netlink)
        }
        Address::Unnamed => (0, &[]),
        Address::Path(name) => (1, name),
        Address::Abstract(name) => (2, name),
    };
    bytes[..4].copy_from_slice(&kind.to_ne_bytes());
    bytes[4..8].copy_from_slice(&(name.len() as u32).to_ne_bytes());
    bytes[8..8 + name.len()].copy_from_slice(name);
    write(process, pointer, &bytes)
}
fn vectors(process: &Process, pointer: u64, count: u64) -> Result<(Vec<(u64, usize)>, usize)> {
    if count > 1024 {
        return Err(Errno::MessageSize);
    }
    let mut result = Vec::new();
    let mut total = 0usize;
    for i in 0..count {
        let bytes = read::<16>(process, pointer.checked_add(i * 16).ok_or(Errno::Fault)?)?;
        let length = usize::try_from(word(&bytes, 8)).map_err(|_| Errno::Invalid)?;
        total = total
            .checked_add(length)
            .filter(|n| *n <= isize::MAX as usize)
            .ok_or(Errno::Invalid)?;
        result.push((word(&bytes, 0), length));
    }
    Ok((result, total))
}

crate::syscall_handler! {
    syscall_socket_create(_frame, kind: u64 = 0, flags: u64 = 1) {
        let kind = Kind::try_from(kind)?;
        if flags & !3 != 0 { return Err(Errno::Invalid); }
        let process = current_process()?;
        let fd = process.install_descriptor_value(Descriptor::Unix(UnixSocket::new(kind, flags & 1 != 0)),
            flags & 2 != 0).map_err(map_process_error)?;
        Ok(fd as u64)
    }
}
crate::syscall_handler! {
    syscall_socket_pair(_frame, kind: u64 = 0, flags: u64 = 1, output: u64 = 2) {
        let kind = Kind::try_from(kind)?;
        if flags & !3 != 0 { return Err(Errno::Invalid); }
        let process = current_process()?;
        if matches!(kind, Kind::UeventRaw | Kind::UeventDatagram) { return Err(Errno::NotSupported); }
        let (a,b) = UnixSocket::pair(kind, flags & 1 != 0, process.pid());
        let first = process.install_descriptor_value(Descriptor::Unix(a), flags & 2 != 0).map_err(map_process_error)?;
        let second = match process.install_descriptor_value(Descriptor::Unix(b), flags & 2 != 0) {
            Ok(fd) => fd, Err(e) => { process.close_descriptor(first); return Err(map_process_error(e)); }
        };
        let mut bytes = [0; 8]; bytes[..4].copy_from_slice(&first.to_ne_bytes());
        bytes[4..].copy_from_slice(&second.to_ne_bytes());
        if let Err(e) = write(&process, output, &bytes) {
            process.close_descriptor(first); process.close_descriptor(second); return Err(e);
        }
        Ok(0)
    }
}
crate::syscall_handler! {
    syscall_socket_bind(_frame, fd: i32 = 0, pointer: u64 = 1) {
        let process = current_process()?;
        let target = socket(&process, fd)?;
        target.bind(address(&process, pointer)?.ok_or(Errno::Invalid)?)?;
        Ok(0)
    }
}
crate::syscall_handler! {
    syscall_socket_connect(_frame, fd: i32 = 0, pointer: u64 = 1) {
        let process = current_process()?;
        let target = socket(&process, fd)?;
        target.connect(address(&process, pointer)?.as_ref())?;
        Ok(0)
    }
}
crate::syscall_handler! {
    syscall_socket_name(_frame, fd: i32 = 0, peer: u64 = 1, output: u64 = 2) {
        let process = current_process()?;
        if peer > 1 { return Err(Errno::Invalid); }
        write_address(&process, output, &socket(&process, fd)?.address(peer != 0)?)?;
        Ok(0)
    }
}
crate::syscall_handler! {
    syscall_socket_option(_frame, fd: i32 = 0, option: u64 = 1, setting: u64 = 2, value: i32 = 3) {
        if setting > 1 { return Err(Errno::Invalid); }
        let process = current_process()?;
        Ok(socket(&process, fd)?.option(option, (setting != 0).then_some(value))? as u64)
    }
}
crate::syscall_handler! {
    syscall_socket_shutdown(_frame, fd: i32 = 0, how: u64 = 1) {
        let process = current_process()?;
        socket(&process, fd)?.shutdown(how)?;
        Ok(0)
    }
}
crate::syscall_handler! {
    syscall_socket_send(_frame, fd: i32 = 0, message: u64 = 1, flags: u64 = 2) {
        if flags & !(DONTWAIT | 8 | EXTENDED) != 0 { return Err(Errno::NotSupported); }
        let process = current_process()?;
        let target = socket(&process, fd)?;
        // address, iovec pointer/count, credentials pointer, output flags.
        let header = read_message(&process, message, flags)?;
        let destination = if word(&header, 0) != 0 { address(&process, word(&header, 0))? } else { None };
        if word(&header, 24) != 0 {
            let credentials = read::<16>(&process, word(&header, 24))?;
            if word(&credentials, 0) != process.pid() as u64 || word(&credentials, 8) != 0 {
                return Err(Errno::Permission);
            }
        }
        let rights_count = word(&header, 48);
        if rights_count > MAX_RIGHTS as u64 { return Err(Errno::MessageSize); }
        let mut numbers = alloc::vec![0; rights_count as usize * 4];
        if !numbers.is_empty() {
            process.address_space().read_user(VirtAddr::new(word(&header, 40)), &mut numbers).map_err(map_memory_error)?;
        }
        let numbers = numbers.as_chunks::<4>().0.iter().map(|fd| i32::from_ne_bytes(*fd)).collect::<Vec<_>>();
        let rights = process.capture_rights(&numbers)?;
        let (iov, total) = vectors(&process, word(&header, 8), word(&header, 16))?;
        if total > MAX_MESSAGE && !target.is_stream() { return Err(Errno::MessageSize); }
        let mut data = alloc::vec![0; total.min(MAX_MESSAGE)]; let mut offset = 0;
        for (base, length) in iov {
            let length = length.min(data.len() - offset);
            if length == 0 { continue; }
            process.address_space().read_user(VirtAddr::new(base), &mut data[offset..offset+length]).map_err(map_memory_error)?;
            offset += length;
        }
        match target.send(data, destination.as_ref(), flags & DONTWAIT != 0, rights) {
            Err(Errno::BrokenPipe) if flags & 8 == 0 => {
                super::super::signal::send_kernel(&process, super::super::signal::SIGPIPE);
                Err(Errno::BrokenPipe)
            }
            result => result.map(|n| n as u64),
        }
    }
}
crate::syscall_handler! {
    syscall_socket_receive(_frame, fd: i32 = 0, message: u64 = 1, flags: u64 = 2) {
        if flags & !(DONTWAIT | PEEK | TRUNC | EXTENDED | RIGHTS_CLOEXEC) != 0 || (flags & RIGHTS_CLOEXEC != 0 && flags & EXTENDED == 0) { return Err(Errno::NotSupported); }
        let process = current_process()?;
        let target = socket(&process, fd)?;
        let header = read_message(&process, message, flags)?;
        let rights_capacity = word(&header, 48).min(MAX_RIGHTS as u64) as usize;
        let (iov, capacity) = vectors(&process, word(&header, 8), word(&header, 16))?;
        let mut delivered = false;
        let n = target.receive(flags, capacity, |data, address, pid, passcred, rights| {
            delivered = true;
            let count = data.len().min(capacity); let mut offset = 0;
            for (base, length) in &iov {
                let n = (*length).min(count-offset);
                if n != 0 { write(&process, *base, &data[offset..offset+n])?; }
                offset += n;
                if offset == count { break; }
            }
            write_address(&process, word(&header, 0), address)?;
            if passcred && word(&header, 24) != 0 {
                let mut bytes = [0; 16]; bytes[..8].copy_from_slice(&(pid as u64).to_ne_bytes());
                write(&process, word(&header, 24), &bytes)?;
            }
            let status = u64::from(count < data.len()) | (u64::from(passcred) << 1) | (u64::from(target.kind == Kind::SeqPacket) << 3);
            process.receive_rights(rights, rights_capacity, flags & RIGHTS_CLOEXEC != 0, |fds, truncated| {
                if !fds.is_empty() {
                    let mut bytes = Vec::with_capacity(fds.len() * 4);
                    for fd in fds { bytes.extend_from_slice(&fd.to_ne_bytes()); }
                    write(&process, word(&header, 40), &bytes)?;
                }
                if flags & EXTENDED != 0 {
                    write(&process, message.checked_add(48).ok_or(Errno::Fault)?, &(fds.len() as u64).to_ne_bytes())?;
                }
                let status = status | (u64::from(truncated) << 2);
                write(&process, message.checked_add(32).ok_or(Errno::Fault)?, &status.to_ne_bytes())
            })?;
            Ok(if flags & TRUNC != 0 { data.len() } else { count })
        })?;
        if !delivered {
            // EOF and zero-capacity stream reads do not invoke the delivery
            // callback. Never leave the input rights capacity as an output count.
            if flags & EXTENDED != 0 {
                write(&process, message.checked_add(48).ok_or(Errno::Fault)?, &0u64.to_ne_bytes())?;
            }
            write(&process, message.checked_add(32).ok_or(Errno::Fault)?, &0u64.to_ne_bytes())?;
        }
        Ok(n as u64)
    }
}

crate::syscall_handler! {
    syscall_socket_listen(_frame, fd: i32 = 0, backlog: u64 = 1) {
        let process = current_process()?;
        socket(&process, fd)?.listen(backlog.min(128) as usize)?;
        Ok(0)
    }
}
crate::syscall_handler! {
    syscall_socket_accept(_frame, fd: i32 = 0, flags: u64 = 1, output: u64 = 2) {
        if flags & !3 != 0 { return Err(Errno::Invalid); }
        let process = current_process()?;
        let listener = socket(&process, fd)?;
        listener.accept(false, |child| {
            write_address(&process, output, &child.address(true)?)?;
            let fd = process.install_descriptor_value(Descriptor::Unix(child.clone()), flags & 2 != 0).map_err(map_process_error)?;
            child.set_nonblocking(flags & 1 != 0);
            Ok(fd as u64)
        })
    }
}
crate::syscall_handler! {
    syscall_socket_peercred(_frame, fd: i32 = 0, output: u64 = 1) {
        let process = current_process()?;
        let pid = socket(&process, fd)?.peer_pid()?;
        let mut bytes = [0; 16]; bytes[..8].copy_from_slice(&(pid as u64).to_ne_bytes());
        write(&process, output, &bytes)?;
        Ok(0)
    }
}
