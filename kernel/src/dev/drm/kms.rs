//! Fixed-mode primary plane and atomic KMS state, following simpledrm's model.
use super::*;

pub(super) const PLANE: u32 = 4;
const TYPE: u32 = 5;
const FB_ID: u32 = 6;
const CRTC_ID: u32 = 7;
const SRC_X: u32 = 8;
const SRC_Y: u32 = 9;
const SRC_W: u32 = 10;
const SRC_H: u32 = 11;
const CRTC_X: u32 = 12;
const CRTC_Y: u32 = 13;
const CRTC_W: u32 = 14;
const CRTC_H: u32 = 15;
const ACTIVE: u32 = 16;
const MODE_ID: u32 = 17;
const BOOT_MODE: u32 = 18;
const OBJECT_CRTC: u32 = 0xcccc_cccc;
const OBJECT_CONNECTOR: u32 = 0xc0c0_c0c0;
const OBJECT_PLANE: u32 = 0xeeee_eeee;
const OBJECT_FB: u32 = 0xfbfb_fbfb;
const ATOMIC: u32 = 0x8000_0000;
const RANGE: u32 = 1 << 1;
const IMMUTABLE: u32 = 1 << 2;
const ENUM: u32 = 1 << 3;
const BLOB: u32 = 1 << 4;
const OBJECT: u32 = 1 << 6;
const SIGNED_RANGE: u32 = 2 << 6;
const FLIP_EVENT: u32 = 1;
const TEST_ONLY: u32 = 0x100;
const NONBLOCK: u32 = 0x200;
const ALLOW_MODESET: u32 = 0x400;

struct PropertyOutput {
    capacity: u32,
    ids: u64,
    values: u64,
}

pub(super) struct PreparedCommit {
    file: usize,
    event: Option<u64>,
    pixels: Option<Vec<u8>>,
}

pub(super) struct Blob {
    id: u32,
    data: Vec<u8>,
}

#[derive(Clone, Default)]
pub(super) struct DisplayState {
    active: bool,
    mode: Option<Arc<Blob>>,
    connector: u32,
    // FB_ID, CRTC_ID, SRC_X/Y/W/H, CRTC_X/Y/W/H, in property order.
    plane: [u64; 10],
    buffer: Option<Arc<Buffer>>,
}

impl DisplayState {
    pub(super) fn fb(&self) -> u32 {
        self.plane[0] as u32
    }
    pub(super) fn active(&self) -> bool {
        self.active
    }
    pub(super) fn has_mode(&self) -> bool {
        self.mode.is_some()
    }
    pub(super) fn routed_crtc(&self) -> u32 {
        self.connector
    }
    fn mode_id(&self) -> u32 {
        self.mode.as_ref().map_or(0, |mode| mode.id)
    }
    fn value(&self, object: u32, property: u32) -> u64 {
        match (object, property) {
            (PLANE, TYPE) => 1, // DRM_PLANE_TYPE_PRIMARY
            (PLANE, FB_ID..=CRTC_H) => self.plane[(property - FB_ID) as usize],
            (CONNECTOR, CRTC_ID) => u64::from(self.connector),
            (CRTC, ACTIVE) => u64::from(self.active),
            (CRTC, MODE_ID) => u64::from(self.mode_id()),
            _ => 0,
        }
    }
}

fn properties(object: u32, kind: u32) -> Result<&'static [u32]> {
    match (object, kind) {
        (CONNECTOR, OBJECT_CONNECTOR) => Ok(&[CRTC_ID]),
        (CRTC, OBJECT_CRTC) => Ok(&[ACTIVE, MODE_ID]),
        (PLANE, OBJECT_PLANE) => Ok(&[
            TYPE, FB_ID, CRTC_ID, SRC_X, SRC_Y, SRC_W, SRC_H, CRTC_X, CRTC_Y, CRTC_W, CRTC_H,
        ]),
        _ => Err(Error::NotFound),
    }
}

fn object_kind(object: u32) -> Result<u32> {
    match object {
        CONNECTOR => Ok(OBJECT_CONNECTOR),
        CRTC => Ok(OBJECT_CRTC),
        PLANE => Ok(OBJECT_PLANE),
        _ => Err(Error::NotFound),
    }
}

fn read_array(context: IoctlContext, address: u64, count: usize, size: usize) -> Result<Vec<u8>> {
    let mut bytes = alloc::vec![0; count * size];
    if !bytes.is_empty() {
        crate::proc::find(context.process_id)
            .ok_or(Error::Io)?
            .address_space()
            .read_user(VirtAddr::new(address), &mut bytes)?;
    }
    Ok(bytes)
}

impl Card {
    pub(super) fn connector_properties(
        &self,
        file: usize,
        context: IoctlContext,
        capacity: u32,
        ids: u64,
        values: u64,
        state: &State,
    ) -> Result<u32> {
        self.object_properties(
            file,
            context,
            CONNECTOR,
            OBJECT_CONNECTOR,
            PropertyOutput {
                capacity,
                ids,
                values,
            },
            state,
        )
    }

    pub(super) fn legacy_flip(
        &self,
        file: usize,
        id: u32,
        event: Option<u64>,
        state: &mut State,
    ) -> Result<()> {
        let mut display = state.display.clone();
        display.plane[0] = u64::from(id);
        display.buffer = Some(
            state.sessions[&file]
                .framebuffers
                .get(&id)
                .cloned()
                .ok_or(Error::NotFound)?,
        );
        self.apply_display(file, display, event, state)
    }

    fn blob(&self, state: &State, id: u32) -> Result<Arc<Blob>> {
        if id == BOOT_MODE {
            return Ok(Arc::new(Blob {
                id,
                data: self.mode.to_vec(),
            }));
        }
        state
            .sessions
            .values()
            .find_map(|session| session.blobs.get(&id).cloned())
            .or_else(|| {
                state
                    .display
                    .mode
                    .as_ref()
                    .filter(|blob| blob.id == id)
                    .cloned()
            })
            .ok_or(Error::NotFound)
    }

    fn object_properties(
        &self,
        file: usize,
        context: IoctlContext,
        object: u32,
        kind: u32,
        output: PropertyOutput,
        state: &State,
    ) -> Result<u32> {
        let PropertyOutput {
            capacity,
            ids,
            values,
        } = output;
        let atomic = state.sessions[&file].atomic;
        let props: Vec<_> = properties(object, kind)?
            .iter()
            .copied()
            .filter(|id| atomic || *id == TYPE)
            .collect();
        if capacity as usize >= props.len() && !props.is_empty() {
            let names: Vec<_> = props.iter().flat_map(|id| id.to_ne_bytes()).collect();
            let data: Vec<_> = props
                .iter()
                .flat_map(|id| state.display.value(object, *id).to_ne_bytes())
                .collect();
            copy_to_user(context, ids, &names)?;
            copy_to_user(context, values, &data)?;
        }
        Ok(props.len() as u32)
    }

    fn get_property(&self, context: IoctlContext, bytes: &mut [u8]) -> Result<()> {
        let id = get32(bytes, 16);
        let (name, flags, values): (&[u8], u32, &[u64]) = match id {
            TYPE => (b"type", IMMUTABLE | ENUM, &[0, 1, 2]),
            FB_ID => (b"FB_ID", ATOMIC | OBJECT, &[u64::from(OBJECT_FB)]),
            CRTC_ID => (b"CRTC_ID", ATOMIC | OBJECT, &[u64::from(OBJECT_CRTC)]),
            SRC_X => (b"SRC_X", ATOMIC | RANGE, &[0, u32::MAX as u64]),
            SRC_Y => (b"SRC_Y", ATOMIC | RANGE, &[0, u32::MAX as u64]),
            SRC_W => (b"SRC_W", ATOMIC | RANGE, &[0, u32::MAX as u64]),
            SRC_H => (b"SRC_H", ATOMIC | RANGE, &[0, u32::MAX as u64]),
            CRTC_X => (
                b"CRTC_X",
                ATOMIC | SIGNED_RANGE,
                &[i32::MIN as u64, i32::MAX as u64],
            ),
            CRTC_Y => (
                b"CRTC_Y",
                ATOMIC | SIGNED_RANGE,
                &[i32::MIN as u64, i32::MAX as u64],
            ),
            CRTC_W => (b"CRTC_W", ATOMIC | RANGE, &[0, i32::MAX as u64]),
            CRTC_H => (b"CRTC_H", ATOMIC | RANGE, &[0, i32::MAX as u64]),
            ACTIVE => (b"ACTIVE", ATOMIC | RANGE, &[0, 1]),
            MODE_ID => (b"MODE_ID", ATOMIC | BLOB, &[]),
            _ => return Err(Error::NotFound),
        };
        if get32(bytes, 56) as usize >= values.len() && !values.is_empty() {
            let data: Vec<_> = values
                .iter()
                .flat_map(|value| value.to_ne_bytes())
                .collect();
            copy_to_user(context, get64(bytes, 0), &data)?;
        }
        if id == TYPE && get32(bytes, 60) >= 3 {
            let mut enums = [0; 120];
            for (index, name) in [b"Overlay".as_slice(), b"Primary", b"Cursor"]
                .iter()
                .enumerate()
            {
                put64(&mut enums, index * 40, index as u64);
                enums[index * 40 + 8..index * 40 + 8 + name.len()].copy_from_slice(name);
            }
            copy_to_user(context, get64(bytes, 8), &enums)?;
        }
        put32(bytes, 20, flags);
        bytes[24..56].fill(0);
        bytes[24..24 + name.len()].copy_from_slice(name);
        put32(bytes, 56, values.len() as u32);
        put32(bytes, 60, if id == TYPE { 3 } else { 0 });
        Ok(())
    }

    fn validate_display(&self, display: &DisplayState) -> Result<()> {
        if display.connector != 0 && display.connector != CRTC {
            return Err(Error::InvalidArgument);
        }
        if display
            .mode
            .as_ref()
            .is_some_and(|mode| mode.data != self.mode)
            || display.mode.is_some() != (display.connector != 0)
            || (display.active && (display.mode.is_none() || display.connector != CRTC))
            || (display.fb() == 0) != (display.plane[1] == 0)
        {
            return Err(Error::InvalidArgument);
        }
        if display.fb() != 0 {
            // A primary plane must cover the fixed mode without scaling or positioning.
            let expected = [
                u64::from(display.fb()),
                u64::from(CRTC),
                0,
                0,
                u64::from(self.width) << 16,
                u64::from(self.height) << 16,
                0,
                0,
                u64::from(self.width),
                u64::from(self.height),
            ];
            if display.mode.is_none() || display.buffer.is_none() || display.plane != expected {
                return Err(Error::InvalidArgument);
            }
        }
        Ok(())
    }

    pub(super) fn apply_display(
        &self,
        file: usize,
        display: DisplayState,
        event: Option<u64>,
        state: &mut State,
    ) -> Result<()> {
        self.commit_display(file, display, event, false, state)
    }

    fn commit_display(
        &self,
        file: usize,
        display: DisplayState,
        event: Option<u64>,
        nonblocking: bool,
        state: &mut State,
    ) -> Result<()> {
        self.validate_display(&display)?;
        if event.is_some() {
            let events = &mut state.sessions.get_mut(&file).unwrap().events;
            if events.len() >= 64 {
                return Err(Error::Busy);
            }
            events.try_reserve(1).map_err(|_| Error::OutOfMemory)?;
        }
        let pixels = if display.active {
            display
                .buffer
                .as_ref()
                .map(|buffer| self.prepare_buffer(buffer))
                .transpose()?
        } else {
            None
        };
        // Preparation is the last fallible operation. Logical state is swapped
        // before handing the nonblocking update to the serial scanout worker.
        state.display = display;
        if nonblocking {
            state.pending = true;
            state.work = Some(PreparedCommit {
                file,
                event,
                pixels,
            });
            self.queued.wake_all();
        } else {
            self.write_scanout(pixels.as_deref());
            self.finish_event(file, event, state);
        }
        Ok(())
    }

    fn finish_event(&self, file: usize, event: Option<u64>, state: &mut State) {
        if let Some(cookie) = event
            && let Some(session) = state.sessions.get_mut(&file)
        {
            state.sequence = state.sequence.wrapping_add(1);
            let time = clock::monotonic_ns();
            let mut bytes = [0; EVENT_SIZE];
            put32(&mut bytes, 0, 2);
            put32(&mut bytes, 4, EVENT_SIZE as u32);
            put64(&mut bytes, 8, cookie);
            put32(&mut bytes, 16, (time / 1_000_000_000) as u32);
            put32(&mut bytes, 20, ((time % 1_000_000_000) / 1000) as u32);
            put32(&mut bytes, 24, state.sequence);
            put32(&mut bytes, 28, CRTC);
            session.events.push_back(bytes);
            self.readable.wake_all();
        }
    }

    pub(super) fn commit_worker(&self) -> ! {
        loop {
            let wait = self.queued.listen();
            let work = { self.state.lock().work.take() };
            let Some(work) = work else {
                wait.wait();
                continue;
            };
            self.write_scanout(work.pixels.as_deref());
            {
                let mut state = self.state.lock();
                state.pending = false;
                self.finish_event(work.file, work.event, &mut state);
            }
            self.completed.wake_all();
        }
    }

    pub(super) fn legacy_display(
        &self,
        file: usize,
        id: u32,
        state: &State,
    ) -> Result<DisplayState> {
        let buffer = state.sessions[&file]
            .framebuffers
            .get(&id)
            .cloned()
            .ok_or(Error::NotFound)?;
        Ok(DisplayState {
            active: true,
            mode: Some(self.blob(state, BOOT_MODE)?),
            connector: CRTC,
            plane: [
                u64::from(id),
                u64::from(CRTC),
                0,
                0,
                u64::from(self.width) << 16,
                u64::from(self.height) << 16,
                0,
                0,
                u64::from(self.width),
                u64::from(self.height),
            ],
            buffer: Some(buffer),
        })
    }

    fn set_property(
        &self,
        file: usize,
        object: u32,
        id: u32,
        value: u64,
        display: &mut DisplayState,
        state: &State,
    ) -> Result<()> {
        if !properties(object, object_kind(object)?)?.contains(&id) || id == TYPE {
            return Err(Error::InvalidArgument);
        }
        if id != CRTC_X && id != CRTC_Y && value > u64::from(u32::MAX) {
            return Err(Error::InvalidArgument);
        }
        if matches!(id, CRTC_X | CRTC_Y) && i32::try_from(value as i64).is_err()
            || matches!(id, CRTC_W | CRTC_H) && value > i32::MAX as u64
        {
            return Err(Error::InvalidArgument);
        }
        match (object, id) {
            (CONNECTOR, CRTC_ID) => display.connector = value as u32,
            (CRTC, ACTIVE) if value <= 1 => display.active = value != 0,
            (CRTC, MODE_ID) => {
                display.mode = if value == 0 {
                    None
                } else {
                    Some(self.blob(state, value as u32)?)
                }
            }
            (PLANE, FB_ID..=CRTC_H) => {
                display.plane[(id - FB_ID) as usize] = value;
                if id == FB_ID {
                    display.buffer = if value == 0 {
                        None
                    } else {
                        Some(
                            state.sessions[&file]
                                .framebuffers
                                .get(&(value as u32))
                                .cloned()
                                .ok_or(Error::NotFound)?,
                        )
                    };
                }
            }
            _ => return Err(Error::InvalidArgument),
        }
        Ok(())
    }

    fn atomic_commit(
        &self,
        file: usize,
        context: IoctlContext,
        bytes: &[u8],
        state: &mut State,
    ) -> Result<()> {
        let flags = get32(bytes, 0);
        let test = flags & TEST_ONLY != 0;
        if !state.sessions[&file].atomic
            || flags & !(FLIP_EVENT | TEST_ONLY | NONBLOCK | ALLOW_MODESET) != 0
            || get64(bytes, 40) != 0
            || (test && flags & FLIP_EVENT != 0)
        {
            return Err(Error::InvalidArgument);
        }
        let count = get32(bytes, 4) as usize;
        if count > 3 {
            return Err(Error::InvalidArgument);
        }
        let objects = read_array(context, get64(bytes, 8), count, 4)?;
        let counts = read_array(context, get64(bytes, 16), count, 4)?;
        let total: usize = counts
            .as_chunks::<4>()
            .0
            .iter()
            .map(|value| get32(value, 0) as usize)
            .sum();
        if total > 16 {
            return Err(Error::InvalidArgument);
        }
        let ids = read_array(context, get64(bytes, 24), total, 4)?;
        let values = read_array(context, get64(bytes, 32), total, 8)?;
        let mut display = state.display.clone();
        let mut offset = 0;
        for index in 0..count {
            let object = get32(&objects, index * 4);
            object_kind(object)?;
            if objects[..index * 4]
                .as_chunks::<4>()
                .0
                .iter()
                .any(|value| get32(value, 0) == object)
            {
                return Err(Error::InvalidArgument);
            }
            for _ in 0..get32(&counts, index * 4) {
                self.set_property(
                    file,
                    object,
                    get32(&ids, offset * 4),
                    get64(&values, offset * 8),
                    &mut display,
                    state,
                )?;
                offset += 1;
            }
        }
        self.validate_display(&display)?;
        let modeset = display.active != state.display.active
            || display.connector != state.display.connector
            || display.mode.as_ref().map(|mode| &mode.data)
                != state.display.mode.as_ref().map(|mode| &mode.data);
        if modeset && flags & ALLOW_MODESET == 0 {
            return Err(Error::InvalidArgument);
        }
        if test {
            return Ok(());
        }
        if flags & FLIP_EVENT != 0 && (!display.active || count == 0) {
            return Err(Error::InvalidArgument);
        }
        self.commit_display(
            file,
            display,
            (flags & FLIP_EVENT != 0).then(|| get64(bytes, 48)),
            flags & NONBLOCK != 0,
            state,
        )
    }

    pub(super) fn kms_ioctl(
        &self,
        file: usize,
        context: IoctlContext,
        command: u8,
        bytes: &mut [u8],
        state: &mut State,
    ) -> Result<()> {
        if matches!(command, 0xb7 | 0xbc) && state.master != Some(file) {
            return Err(Error::PermissionDenied);
        }
        match command {
            0xb5 => {
                let visible = state.sessions[&file].universal;
                if visible && get32(bytes, 8) != 0 {
                    copy_to_user(context, get64(bytes, 0), &PLANE.to_ne_bytes())?;
                }
                put32(bytes, 8, u32::from(visible));
            }
            0xb6 => {
                if get32(bytes, 0) != PLANE {
                    return Err(Error::NotFound);
                }
                if get32(bytes, 20) != 0 {
                    copy_to_user(context, get64(bytes, 24), &XRGB8888.to_ne_bytes())?;
                }
                put32(bytes, 4, state.display.plane[1] as u32);
                put32(bytes, 8, state.display.fb());
                put32(bytes, 12, 1);
                put32(bytes, 16, 0);
                put32(bytes, 20, 1);
            }
            0xb7 => {
                if get32(bytes, 0) != PLANE || get32(bytes, 12) != 0 {
                    return Err(Error::InvalidArgument);
                }
                let mut display = state.display.clone();
                let id = get32(bytes, 8);
                display.plane = [
                    u64::from(id),
                    if id == 0 {
                        0
                    } else {
                        u64::from(get32(bytes, 4))
                    },
                    u64::from(get32(bytes, 32)),
                    u64::from(get32(bytes, 36)),
                    u64::from(get32(bytes, 44)),
                    u64::from(get32(bytes, 40)),
                    get32(bytes, 16) as i32 as u64,
                    get32(bytes, 20) as i32 as u64,
                    u64::from(get32(bytes, 24)),
                    u64::from(get32(bytes, 28)),
                ];
                display.buffer = if id == 0 {
                    None
                } else {
                    Some(
                        state.sessions[&file]
                            .framebuffers
                            .get(&id)
                            .cloned()
                            .ok_or(Error::NotFound)?,
                    )
                };
                self.apply_display(file, display, None, state)?;
            }
            0xb9 => {
                let count = self.object_properties(
                    file,
                    context,
                    get32(bytes, 20),
                    get32(bytes, 24),
                    PropertyOutput {
                        capacity: get32(bytes, 16),
                        ids: get64(bytes, 0),
                        values: get64(bytes, 8),
                    },
                    state,
                )?;
                put32(bytes, 16, count);
            }
            0xaa => self.get_property(context, bytes)?,
            0xac => {
                let blob = self.blob(state, get32(bytes, 0))?;
                if get32(bytes, 4) as usize == blob.data.len() {
                    copy_to_user(context, get64(bytes, 8), &blob.data)?;
                }
                put32(bytes, 4, blob.data.len() as u32);
            }
            0xbd => {
                let length = get32(bytes, 8) as usize;
                if length == 0 || length > 65536 || state.sessions[&file].blobs.len() >= 32 {
                    return Err(Error::InvalidArgument);
                }
                let data = read_array(context, get64(bytes, 0), length, 1)?;
                let id = next_id(state)?;
                state
                    .sessions
                    .get_mut(&file)
                    .unwrap()
                    .blobs
                    .insert(id, Arc::new(Blob { id, data }));
                put32(bytes, 12, id);
            }
            0xbe => {
                state
                    .sessions
                    .get_mut(&file)
                    .unwrap()
                    .blobs
                    .remove(&get32(bytes, 0))
                    .ok_or(Error::NotFound)?;
            }
            0xbc => self.atomic_commit(file, context, bytes, state)?,
            _ => return Err(Error::NotTty),
        }
        Ok(())
    }
}
