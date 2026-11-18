//! Small DRM/KMS device for the display mode supplied by the bootloader.
//!
//! Dumb buffers live in ordinary VM objects. Modesets and page flips copy the
//! selected buffer into the boot display; userspace never maps physical memory.
//! Universal primary-plane and atomic KMS ioctls share state with legacy KMS.
//! Commits complete after copying; the boot display has no vblank interrupt.

use crate::sys::event::EventSource;

use alloc::{
    collections::{BTreeMap, VecDeque},
    format,
    sync::Arc,
    vec::Vec,
};
use limine::request::FramebufferRequest;

use crate::{
    driver::{Result as DriverResult, io::mmio},
    fs::{Error, IoctlContext, OpenFlags, PollEvents, Result, devfs::DeviceNodeOps},
    mem::{self, IoSink, ObjectKind, PAGE_SIZE, VirtAddr, VmObject},
    sys::{clock, event::Event, sync::Mutex},
};

mod kms;
use super::node;
use kms::{Blob, DisplayState, PreparedCommit};

const CONNECTOR: u32 = 1;
const ENCODER: u32 = 2;
const CRTC: u32 = 3;
const XRGB8888: u32 = u32::from_le_bytes(*b"XR24");
const MAX_BUFFER: u64 = 64 * 1024 * 1024;
const MAX_SESSION_BYTES: u64 = 128 * 1024 * 1024;
const EVENT_SIZE: usize = 32;

#[used]
#[unsafe(link_section = ".requests")]
static REQUEST: FramebufferRequest = FramebufferRequest::new();

struct Buffer {
    object: Arc<VmObject>,
    width: u32,
    height: u32,
    pitch: u32,
}

#[derive(Default)]
struct Session {
    buffers: BTreeMap<u32, Arc<Buffer>>,
    framebuffers: BTreeMap<u32, Arc<Buffer>>,
    events: VecDeque<[u8; EVENT_SIZE]>,
    blobs: BTreeMap<u32, Arc<Blob>>,
    universal: bool,
    atomic: bool,
}

struct State {
    sessions: BTreeMap<usize, Session>,
    next_session: usize,
    next_id: u32,
    master: Option<usize>,
    display: DisplayState,
    sequence: u32,
    pending: bool,
    work: Option<PreparedCommit>,
}

struct Card {
    scanout: Arc<mmio::Mapping>,
    width: u32,
    height: u32,
    pitch: u32,
    mode: [u8; 68],
    state: Mutex<State>,
    readable: Event,
    queued: Event,
    completed: Event,
}

fn get32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_ne_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn get64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_ne_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

fn put32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_ne_bytes());
}

fn put64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_ne_bytes());
}

fn copy_to_user(context: IoctlContext, address: u64, bytes: &[u8]) -> Result<()> {
    crate::proc::find(context.process_id)
        .ok_or(Error::Io)?
        .address_space()
        .write_user(VirtAddr::new(address), bytes)
        .map_err(Error::from)
}

fn next_id(state: &mut State) -> Result<u32> {
    let id = state.next_id;
    if id >= i32::MAX as u32 {
        return Err(Error::NoSpace);
    }
    state.next_id += 1;
    Ok(id)
}

impl Card {
    fn prepare_buffer(&self, buffer: &Buffer) -> Result<Vec<u8>> {
        // Stage all reads before touching scanout, so a failed preparation
        // cannot partially apply a commit to the visible display.
        let row_bytes = self.width as usize * 4;
        let length = row_bytes * self.height as usize;
        let mut pixels = Vec::new();
        pixels
            .try_reserve_exact(length)
            .map_err(|_| Error::OutOfMemory)?;
        pixels.resize(length, 0);
        for (y, row) in pixels.chunks_exact_mut(row_bytes).enumerate() {
            if buffer
                .object
                .read_at(y as u64 * u64::from(buffer.pitch), row)?
                != row_bytes
            {
                return Err(Error::Io);
            }
        }
        Ok(pixels)
    }

    fn write_scanout(&self, pixels: Option<&[u8]>) {
        for y in 0..self.height as usize {
            for x in 0..self.width as usize {
                let source = (y * self.width as usize + x) * 4;
                let value = pixels.map_or(0, |data| {
                    u32::from_ne_bytes(data[source..source + 4].try_into().unwrap())
                });
                let offset = y * self.pitch as usize + x * 4;
                // SAFETY: publication validates mapping bounds and alignment.
                // The pending-commit gate serializes this worker with ioctl
                // scanout writes; all source reads were staged before commit.
                unsafe {
                    self.scanout
                        .address()
                        .as_mut_ptr::<u8>()
                        .add(offset)
                        .cast::<u32>()
                        .write_volatile(value);
                }
            }
        }
        // Drain write-combining stores before publishing completion.
        #[cfg(target_arch = "x86_64")]
        // SAFETY: sfence orders scanout stores without accessing other memory.
        unsafe {
            core::arch::asm!("sfence", options(nostack, preserves_flags));
        }
        #[cfg(target_arch = "riscv64")]
        // SAFETY: this fence orders coherent memory and device I/O stores.
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack));
        }
    }

    fn ioctl_locked(
        &self,
        file: usize,
        context: IoctlContext,
        command: u8,
        bytes: &mut [u8],
        state: &mut State,
    ) -> Result<()> {
        if !state.sessions.contains_key(&file) {
            return Err(Error::BadFileDescriptor);
        }
        match command {
            0x00 => {
                // VERSION
                put32(bytes, 0, 1);
                put32(bytes, 4, 0);
                put32(bytes, 8, 0);
                for (offset, text) in [
                    (16, &b"roanix-boot"[..]),
                    (32, &b"20261020"[..]),
                    (48, &b"Roanix boot display KMS"[..]),
                ] {
                    let count = (get64(bytes, offset) as usize).min(text.len());
                    if count != 0 && get64(bytes, offset + 8) != 0 {
                        copy_to_user(context, get64(bytes, offset + 8), &text[..count])?;
                    }
                    put64(bytes, offset, text.len() as u64);
                }
            }
            0x0c => {
                // GET_CAP
                let value = match get64(bytes, 0) {
                    1 | 4 | 6 | 0x12 => 1, // dumb/shadow buffers, monotonic time, event CRTC IDs
                    3 => 24,               // preferred depth
                    2 | 5 | 7..=9 | 0x10..=0x11 | 0x13..=0x15 => 0,
                    _ => return Err(Error::InvalidArgument),
                };
                put64(bytes, 8, value);
            }
            0x0d => {
                // SET_CLIENT_CAP: each open file description negotiates separately.
                let value = get64(bytes, 8);
                let session = state.sessions.get_mut(&file).unwrap();
                match get64(bytes, 0) {
                    2 if value <= 1 => session.universal = value != 0,
                    3 if value <= 2 => {
                        session.atomic = value != 0;
                        session.universal = value != 0;
                    }
                    _ => return Err(Error::InvalidArgument),
                }
            }
            0x1e => {
                // SET_MASTER
                if state.master.is_some_and(|master| master != file) {
                    return Err(Error::Busy);
                }
                state.master = Some(file);
            }
            0x1f => {
                // DROP_MASTER
                if state.master == Some(file) {
                    state.master = None;
                }
            }
            0xa0 => {
                // GETRESOURCES
                for (pointer, count, id) in [(8, 36, CRTC), (16, 40, CONNECTOR), (24, 44, ENCODER)]
                {
                    if get32(bytes, count) != 0 {
                        copy_to_user(context, get64(bytes, pointer), &id.to_ne_bytes())?;
                    }
                    put32(bytes, count, 1);
                }
                put32(bytes, 32, 0);
                for (offset, value) in [
                    (48, self.width),
                    (52, self.width),
                    (56, self.height),
                    (60, self.height),
                ] {
                    put32(bytes, offset, value);
                }
            }
            0xa1 => {
                // GETCRTC
                if get32(bytes, 12) != CRTC {
                    return Err(Error::NotFound);
                }
                bytes.fill(0);
                put32(bytes, 12, CRTC);
                put32(bytes, 16, state.display.fb());
                put32(bytes, 32, u32::from(state.display.has_mode()));
                bytes[36..104].copy_from_slice(&self.mode);
            }
            0xa2 => {
                // SETCRTC
                if state.master != Some(file) {
                    return Err(Error::PermissionDenied);
                }
                if get32(bytes, 12) != CRTC || get32(bytes, 20) != 0 || get32(bytes, 24) != 0 {
                    return Err(Error::InvalidArgument);
                }
                if get32(bytes, 32) == 0 {
                    if get32(bytes, 16) != 0 || get32(bytes, 8) != 0 {
                        return Err(Error::InvalidArgument);
                    }
                    return self.apply_display(file, DisplayState::default(), None, state);
                }
                if get32(bytes, 32) != 1 || bytes[36..104] != self.mode || get32(bytes, 8) != 1 {
                    return Err(Error::InvalidArgument);
                }
                let mut connector = [0; 4];
                crate::proc::find(context.process_id)
                    .ok_or(Error::Io)?
                    .address_space()
                    .read_user(VirtAddr::new(get64(bytes, 0)), &mut connector)?;
                if u32::from_ne_bytes(connector) != CONNECTOR {
                    return Err(Error::InvalidArgument);
                }
                let id = get32(bytes, 16);
                let display = self.legacy_display(file, id, state)?;
                self.apply_display(file, display, None, state)?;
            }
            0xa6 => {
                // GETENCODER
                if get32(bytes, 0) != ENCODER {
                    return Err(Error::NotFound);
                }
                bytes.fill(0);
                for (offset, value) in [
                    (0, ENCODER),
                    (4, 0),
                    (8, state.display.routed_crtc()),
                    (12, 1),
                ] {
                    put32(bytes, offset, value);
                }
            }
            0xa7 => {
                // GETCONNECTOR
                if get32(bytes, 48) != CONNECTOR {
                    return Err(Error::NotFound);
                }
                if get32(bytes, 32) != 0 {
                    copy_to_user(context, get64(bytes, 8), &self.mode)?;
                }
                if get32(bytes, 40) != 0 {
                    copy_to_user(context, get64(bytes, 0), &ENCODER.to_ne_bytes())?;
                }
                let count = self.connector_properties(
                    file,
                    context,
                    get32(bytes, 36),
                    get64(bytes, 16),
                    get64(bytes, 24),
                    state,
                )?;
                for (offset, value) in [
                    (32, 1),
                    (36, count),
                    (40, 1),
                    (
                        44,
                        if state.display.routed_crtc() == 0 {
                            0
                        } else {
                            ENCODER
                        },
                    ),
                    (52, 15),
                    (56, 1),
                    (60, 1),
                    (64, 0),
                    (68, 0),
                    (72, 0),
                ] {
                    put32(bytes, offset, value);
                }
            }
            0xa3 => {
                // CURSOR: permit disabling a cursor, never advertise a cursor plane
                if state.master != Some(file) {
                    return Err(Error::PermissionDenied);
                }
                if get32(bytes, 0) != 1 || get32(bytes, 4) != CRTC || get32(bytes, 24) != 0 {
                    return Err(Error::Unsupported);
                }
            }
            0xaf => {
                // RMFB
                let id = get32(bytes, 0);
                state
                    .sessions
                    .get_mut(&file)
                    .unwrap()
                    .framebuffers
                    .remove(&id)
                    .ok_or(Error::NotFound)?;
                if state.display.fb() == id {
                    self.apply_display(file, DisplayState::default(), None, state)?;
                }
            }
            0xb0 => {
                // PAGE_FLIP uses the same primary-plane state as atomic commits.
                if state.master != Some(file) {
                    return Err(Error::PermissionDenied);
                }
                if get32(bytes, 0) != CRTC
                    || get32(bytes, 8) & !1 != 0
                    || get32(bytes, 12) != 0
                    || !state.display.active()
                {
                    return Err(Error::InvalidArgument);
                }
                self.legacy_flip(
                    file,
                    get32(bytes, 4),
                    (get32(bytes, 8) & 1 != 0).then(|| get64(bytes, 16)),
                    state,
                )?;
            }
            0xb2 => {
                // CREATE_DUMB
                let (width, height) = (get32(bytes, 4), get32(bytes, 0));
                if width == 0
                    || height == 0
                    || width > 16384
                    || height > 16384
                    || get32(bytes, 8) != 32
                    || get32(bytes, 12) != 0
                {
                    return Err(Error::InvalidArgument);
                }
                let pitch = (width * 4 + 63) & !63;
                let size =
                    (u64::from(pitch) * u64::from(height) + PAGE_SIZE - 1) & !(PAGE_SIZE - 1);
                let session = &state.sessions[&file];
                let used: u64 = session
                    .buffers
                    .values()
                    .map(|buffer| buffer.object.length())
                    .sum();
                if size > MAX_BUFFER
                    || used + size > MAX_SESSION_BYTES
                    || session.buffers.len() >= 32
                {
                    return Err(Error::NoSpace);
                }
                let object = VmObject::new(ObjectKind::Kernel);
                object.truncate(size)?;
                let id = next_id(state)?;
                state.sessions.get_mut(&file).unwrap().buffers.insert(
                    id,
                    Arc::new(Buffer {
                        object,
                        width,
                        height,
                        pitch,
                    }),
                );
                put32(bytes, 16, id);
                put32(bytes, 20, pitch);
                put64(bytes, 24, size);
            }
            0xb3 => {
                // MAP_DUMB
                let id = get32(bytes, 0);
                if get32(bytes, 4) != 0 || !state.sessions[&file].buffers.contains_key(&id) {
                    return Err(Error::InvalidArgument);
                }
                put64(bytes, 8, u64::from(id) << 32);
            }
            0xb4 | 0x09 => {
                // DESTROY_DUMB / GEM_CLOSE
                let id = get32(bytes, 0);
                state
                    .sessions
                    .get_mut(&file)
                    .unwrap()
                    .buffers
                    .remove(&id)
                    .ok_or(Error::NotFound)?;
            }
            0xb8 => {
                // ADDFB2
                if get32(bytes, 4) != self.width
                    || get32(bytes, 8) != self.height
                    || get32(bytes, 12) != XRGB8888
                    || get32(bytes, 16) != 0
                    || bytes[24..36].iter().any(|&b| b != 0)
                    || bytes[40..].iter().any(|&b| b != 0)
                {
                    return Err(Error::InvalidArgument);
                }
                let session = &state.sessions[&file];
                if session.framebuffers.len() >= 32 {
                    return Err(Error::NoSpace);
                }
                let buffer = session
                    .buffers
                    .get(&get32(bytes, 20))
                    .ok_or(Error::NotFound)?
                    .clone();
                if buffer.width != self.width
                    || buffer.height != self.height
                    || get32(bytes, 36) != buffer.pitch
                {
                    return Err(Error::InvalidArgument);
                }
                let id = next_id(state)?;
                state
                    .sessions
                    .get_mut(&file)
                    .unwrap()
                    .framebuffers
                    .insert(id, buffer);
                put32(bytes, 0, id);
            }
            0xaa | 0xac | 0xb5..=0xb7 | 0xb9 | 0xbc..=0xbe => {
                self.kms_ioctl(file, context, command, bytes, state)?;
            }
            _ => return Err(Error::NotTty),
        }
        Ok(())
    }
}

impl DeviceNodeOps for Card {
    fn open(&self, _flags: u32) -> Result<usize> {
        let mut state = self.state.lock();
        let id = state.next_session;
        state.next_session = id.checked_add(1).ok_or(Error::NoSpace)?;
        state.sessions.insert(id, Session::default());
        if state.master.is_none() {
            state.master = Some(id);
        }
        Ok(id)
    }

    fn close(&self, file: usize, _flags: u32) {
        let mut state = self.state.lock();
        state.sessions.remove(&file);
        if state.master == Some(file) {
            state.master = None;
            state.display = DisplayState::default();
        }
    }

    fn mapping_object(
        &self,
        file: usize,
        offset: u64,
        length: u64,
    ) -> Result<(Arc<VmObject>, u64)> {
        let state = self.state.lock();
        let session = state.sessions.get(&file).ok_or(Error::BadFileDescriptor)?;
        let buffer = session
            .buffers
            .get(&((offset >> 32) as u32))
            .ok_or(Error::InvalidArgument)?;
        let start = offset & u64::from(u32::MAX);
        if start
            .checked_add(length)
            .is_none_or(|end| end > buffer.object.length())
        {
            return Err(Error::InvalidArgument);
        }
        Ok((buffer.object.clone(), start))
    }

    fn read_at_with_flags(
        &self,
        file: usize,
        _offset: u64,
        sink: &mut IoSink<'_>,
        flags: u32,
    ) -> Result<usize> {
        if sink.is_empty() {
            return Ok(0);
        }
        if sink.len() < EVENT_SIZE {
            return Err(Error::InvalidArgument);
        }
        loop {
            let wait = self.readable.listen();
            {
                let mut state = self.state.lock();
                let session = state
                    .sessions
                    .get_mut(&file)
                    .ok_or(Error::BadFileDescriptor)?;
                if let Some(event) = session.events.front() {
                    sink.store(0, event)?;
                    session.events.pop_front();
                    return Ok(EVENT_SIZE);
                }
            }
            if flags & OpenFlags::NONBLOCK.bits() != 0 {
                return Err(Error::WouldBlock);
            }
            wait.wait();
        }
    }

    fn poll(&self, file: usize, _: u64, events: PollEvents, _: u32) -> Result<PollEvents> {
        let state = self.state.lock();
        let session = state.sessions.get(&file).ok_or(Error::BadFileDescriptor)?;
        Ok(if session.events.is_empty() {
            PollEvents::empty()
        } else {
            events & (PollEvents::IN | PollEvents::RDNORM)
        })
    }

    fn poll_events<'a>(
        &'a self,
        _: usize,
        events: PollEvents,
        output: &mut Vec<EventSource<'a>>,
    ) -> bool {
        if events.intersects(PollEvents::IN | PollEvents::RDNORM) {
            output.push((&self.readable).into());
        }
        true
    }

    fn ioctl(
        &self,
        file: usize,
        context: IoctlContext,
        request: u64,
        _: u64,
        bytes: &mut [u8],
    ) -> Result<u64> {
        let command = request as u8;
        let (direction, size) = match command {
            0x00 => (3, 64),
            0x09 => (1, 8),
            0x0c | 0xb3 => (3, 16),
            0x0d => (1, 16),
            0x1e | 0x1f => (0, 0),
            0xa0 => (3, 64),
            0xa1 | 0xa2 | 0xb8 => (3, 104),
            0xa6 => (3, 20),
            0xa7 => (3, 80),
            0xa3 => (3, 28),
            0xaf | 0xb4 => (3, 4),
            0xb0 => (3, 24),
            0xb2 | 0xb6 | 0xb9 => (3, 32),
            0xaa => (3, 64),
            0xac | 0xb5 | 0xbd => (3, 16),
            0xb7 => (3, 48),
            0xbc => (3, 56),
            0xbe => (3, 4),
            _ => return Err(Error::NotTty),
        };
        let expected =
            (direction << 30) | ((size as u64) << 16) | (u64::from(b'd') << 8) | u64::from(command);
        if request != expected || bytes.len() != size {
            return Err(Error::NotTty);
        }
        loop {
            let wait = self.completed.listen();
            let mut state = self.state.lock();
            let update = matches!(command, 0xa2 | 0xaf | 0xb0 | 0xb7)
                || command == 0xbc && get32(bytes, 0) & 0x100 == 0;
            if update && state.pending {
                if command == 0xbc && get32(bytes, 0) & 0x200 != 0 {
                    return Err(Error::Busy);
                }
                drop(state);
                wait.wait();
                continue;
            }
            self.ioctl_locked(file, context, command, bytes, &mut state)?;
            return Ok(0);
        }
    }
}

pub(super) fn publish() -> DriverResult<()> {
    let Some(response) = REQUEST.get_response() else {
        return Ok(());
    };
    let Some(display) = response.framebuffers().next() else {
        return Ok(());
    };
    let (Ok(width), Ok(height), Ok(pitch)) = (
        u32::try_from(display.width()),
        u32::try_from(display.height()),
        u32::try_from(display.pitch()),
    ) else {
        return Ok(());
    };
    if width == 0
        || height == 0
        || width > u16::MAX as u32
        || height > u16::MAX as u32
        || display.bpp() != 32
        || pitch % 4 != 0
        || u64::from(pitch) < u64::from(width) * 4
        || display.memory_model() != limine::framebuffer::MemoryModel::RGB
        || display.red_mask_size() != 8
        || display.red_mask_shift() != 16
        || display.green_mask_size() != 8
        || display.green_mask_shift() != 8
        || display.blue_mask_size() != 8
        || display.blue_mask_shift() != 0
    {
        return Ok(());
    }
    let Some(length) = u64::from(pitch).checked_mul(u64::from(height)) else {
        return Ok(());
    };
    let Some(physical) = mem::virt_to_phys_hhdm(VirtAddr::from_ptr(display.addr())) else {
        return Ok(());
    };
    if physical.as_u64() % 4 != 0 {
        return Ok(());
    }
    let scanout = mmio::map(
        None,
        physical.as_u64(),
        length as usize,
        mmio::flags::WRITE_COMBINE,
    )?;
    let mut mode = [0; 68];
    for (offset, value) in [
        (4, width),
        (6, width),
        (8, width),
        (10, width),
        (14, height),
        (16, height),
        (18, height),
        (20, height),
    ] {
        mode[offset..offset + 2].copy_from_slice(&(value as u16).to_ne_bytes());
    }
    put32(&mut mode, 28, 0);
    put32(&mut mode, 32, (1 << 3) | (1 << 6)); // preferred, driver-provided mode
    let name = format!("{width}x{height}");
    mode[36..36 + name.len()].copy_from_slice(name.as_bytes());
    let directory = node::create_directory(None, node::root()?, "dri", 0o755)?;
    let card = Arc::new(Card {
        scanout,
        width,
        height,
        pitch,
        mode,
        state: Mutex::new(State {
            sessions: BTreeMap::new(),
            next_session: 1,
            next_id: 32,
            master: None,
            display: DisplayState::default(),
            sequence: 0,
            pending: false,
            work: None,
        }),
        readable: Event::new(),
        queued: Event::new(),
        completed: Event::new(),
    });
    node::create_native_node(
        None,
        directory,
        "card0",
        node::kind::CHARACTER,
        0o600,
        card.clone(),
    )?;
    // The boot display and its native device node live for the kernel's lifetime.
    crate::sys::sched::run(move || card.commit_worker());
    log::info!("drm: /dev/dri/card0 {width}x{height}");
    Ok(())
}
