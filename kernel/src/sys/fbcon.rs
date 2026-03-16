//!
//! # Framebuffer Debug Console
//!
//! Minimal framebuffer-backed log sink used once the bootloader-provided
//! framebuffer is available. The console only targets packed RGB framebuffers
//! and renders an 8x16 bitmap font with a single global cursor.
//!

use core::{ptr, slice};

use limine::framebuffer::MemoryModel;
use limine::request::FramebufferRequest;

use crate::sys::{debug, smp::IrqSpinLock};

const FONT_WIDTH: usize = 8;
const FONT_HEIGHT: usize = 16;
const FONT_BYTES: usize = FONT_HEIGHT * 256;
const FONT: &[u8; FONT_BYTES] = include_bytes!("fbcon_font.bin");
const FG_RGB: u32 = 0xD8DEE9;

static FBCON: IrqSpinLock<Option<FbCon>> = IrqSpinLock::new(None);

#[used]
#[doc(hidden)]
#[link_section = ".requests"]
static FRAMEBUFFER_REQUEST: FramebufferRequest = FramebufferRequest::new();

struct FbCon {
    base: *mut u8,
    width: usize,
    height: usize,
    pitch: usize,
    bytes_per_pixel: usize,
    fb_len: usize,
    cols: usize,
    rows: usize,
    cursor_x: usize,
    cursor_y: usize,
    fg: u32,
}

unsafe impl Send for FbCon {}

impl FbCon {
    fn probe() -> Option<Self> {
        let response = FRAMEBUFFER_REQUEST.get_response()?;
        let fb = response.framebuffers().next()?;
        if fb.memory_model() != MemoryModel::RGB {
            return None;
        }

        let bytes_per_pixel = (fb.bpp() as usize).div_ceil(8);
        if bytes_per_pixel < 3 {
            return None;
        }

        let width = fb.width() as usize;
        let height = fb.height() as usize;
        let pitch = fb.pitch() as usize;
        if width < FONT_WIDTH || height < FONT_HEIGHT {
            return None;
        }
        let fb_len = pitch.checked_mul(height)?;

        let mut console = Self {
            base: fb.addr(),
            width,
            height,
            pitch,
            bytes_per_pixel,
            fb_len,
            cols: width / FONT_WIDTH,
            rows: height / FONT_HEIGHT,
            cursor_x: 0,
            cursor_y: 0,
            fg: pack_color(
                FG_RGB,
                fb.red_mask_size(),
                fb.red_mask_shift(),
                fb.green_mask_size(),
                fb.green_mask_shift(),
                fb.blue_mask_size(),
                fb.blue_mask_shift(),
            ),
        };
        console.clear();
        Some(console)
    }

    fn clear(&mut self) {
        unsafe {
            ptr::write_bytes(self.base, 0, self.fb_len);
        }
    }

    fn write_record(&mut self, bytes: &[u8]) {
        let mut escape = Escape::None;

        for &byte in bytes {
            match escape {
                Escape::None if byte == 0x1b => escape = Escape::Esc,
                Escape::None => self.put_byte(byte),
                Escape::Esc if byte == b'[' => escape = Escape::Csi,
                Escape::Esc => escape = Escape::None,
                Escape::Csi if (0x40..=0x7e).contains(&byte) => escape = Escape::None,
                Escape::Csi => {}
            }
        }

        self.newline();
    }

    fn put_byte(&mut self, byte: u8) {
        match byte {
            b'\n' => self.newline(),
            b'\r' => self.cursor_x = 0,
            b'\t' => {
                for _ in 0..4 {
                    self.put_byte(b' ');
                }
            }
            0x20..=0x7e => self.draw_char(byte),
            _ => self.draw_char(b'?'),
        }
    }

    fn draw_char(&mut self, byte: u8) {
        if self.cursor_x >= self.cols {
            self.newline();
        }

        let x = self.cursor_x * FONT_WIDTH;
        let y = self.cursor_y * FONT_HEIGHT;
        let glyph = &FONT[byte as usize * FONT_HEIGHT..][..FONT_HEIGHT];

        for (row, bits) in glyph.iter().copied().enumerate() {
            for col in 0..FONT_WIDTH {
                let on = bits & (0x80 >> col) != 0;
                self.put_pixel(x + col, y + row, if on { self.fg } else { 0 });
            }
        }

        self.cursor_x += 1;
    }

    fn put_pixel(&mut self, x: usize, y: usize, color: u32) {
        if x >= self.width || y >= self.height {
            return;
        }

        let row = match y.checked_mul(self.pitch) {
            Some(v) => v,
            None => return,
        };
        let col = match x.checked_mul(self.bytes_per_pixel) {
            Some(v) => v,
            None => return,
        };
        let offset = match row.checked_add(col) {
            Some(v) => v,
            None => return,
        };
        let end = match offset.checked_add(self.bytes_per_pixel) {
            Some(v) => v,
            None => return,
        };
        if end > self.fb_len {
            return;
        }

        unsafe {
            let pixel = self.base.add(offset);
            for idx in 0..self.bytes_per_pixel {
                let shift = idx.saturating_mul(8);
                let byte = if shift < u32::BITS as usize {
                    (color >> shift) as u8
                } else {
                    0
                };
                pixel.add(idx).write_volatile(byte);
            }
        }
    }

    fn newline(&mut self) {
        self.cursor_x = 0;
        if self.cursor_y + 1 < self.rows {
            self.cursor_y += 1;
        } else {
            self.scroll();
        }
    }

    fn scroll(&mut self) {
        let row_bytes = match self.pitch.checked_mul(FONT_HEIGHT) {
            Some(v) => v,
            None => return,
        };
        let visible = self.fb_len;
        if row_bytes > visible {
            return;
        }
        unsafe {
            ptr::copy(self.base.add(row_bytes), self.base, visible - row_bytes);
            ptr::write_bytes(self.base.add(visible - row_bytes), 0, row_bytes);
        }
    }
}

#[derive(Copy, Clone)]
enum Escape {
    None,
    Esc,
    Csi,
}

fn pack_color(
    rgb: u32,
    red_size: u8,
    red_shift: u8,
    green_size: u8,
    green_shift: u8,
    blue_size: u8,
    blue_shift: u8,
) -> u32 {
    pack_component((rgb >> 16) as u8, red_size, red_shift)
        | pack_component((rgb >> 8) as u8, green_size, green_shift)
        | pack_component(rgb as u8, blue_size, blue_shift)
}

fn pack_component(value: u8, size: u8, shift: u8) -> u32 {
    if size == 0 {
        return 0;
    }

    let mask = (1u32 << size) - 1;
    (((value as u32 * mask) + 127) / 255) << shift
}

/// Registers the framebuffer console as a debug sink when Limine provided a
/// compatible RGB framebuffer.
pub fn register() -> bool {
    let mut state = FBCON.lock();
    if state.is_some() {
        return true;
    }

    let Some(console) = FbCon::probe() else {
        return false;
    };

    *state = Some(console);
    drop(state);
    debug::register_sink(write);
    true
}

/// Force-unlocks the framebuffer console state for panic-time recovery.
///
/// # Safety
///
/// This must only be used after other CPUs have been stopped or abandoned.
pub(crate) unsafe fn force_unlock_for_panic() {
    if FBCON.is_locked() {
        FBCON.force_unlock();
    }
}

fn write(buf: *const u8, buflen: usize) {
    let bytes = unsafe { slice::from_raw_parts(buf, buflen) };
    let mut state = FBCON.lock();
    if let Some(console) = state.as_mut() {
        console.write_record(bytes);
    }
}
