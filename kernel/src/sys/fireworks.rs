//!
//! # Framebuffer Fireworks Stress Test
//!
//! Mintia2-inspired firework launcher that keeps the original stress profile:
//! one background launcher thread, one thread per rocket, and one thread per
//! explosion particle. Drawing is clipped to a small viewport in the top-right
//! corner of the framebuffer.
//!

use core::{
    cmp::min,
    sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
    time::Duration,
};

use limine::framebuffer::MemoryModel;
use log::info;
use spin::{Mutex, Once};

use crate::{
    mem::alloc,
    sys::{clock, framebuffer::FRAMEBUFFER_REQUEST, sched},
};

const VIEW_WIDTH: usize = 320;
const VIEW_HEIGHT: usize = 240;

const BACKGROUND_RGB: u32 = 0x000000;
const INTERVAL_MS: u64 = 16;
const GRAVITY: i32 = 10;

const FP_SHIFT: u32 = 12;
const FP_ONE: i32 = 1 << FP_SHIFT;
const FP_MASK: u32 = (1 << FP_SHIFT) - 1;

const ROCKET_BASE_SPEED_MIN: i32 = 100;
const ROCKET_BASE_SPEED_RANGE: u32 = 100;
const ROCKET_LIFETIME_MIN_MS: u32 = 500;
const ROCKET_LIFETIME_RANGE_MS: u32 = 500;
const PARTICLE_LIFETIME_MIN_MS: u32 = 2000;
const PARTICLE_LIFETIME_RANGE_MS: u32 = 1000;
const EXPLOSION_RANGE_MIN: i32 = 100;
const EXPLOSION_RANGE_RANGE: u32 = 100;
const EXPLOSION_COUNT_MIN: usize = 100;
const EXPLOSION_COUNT_RANGE: u32 = 100;
const MAX_ACTIVE_PARTICLES: usize = 400;

const PALETTE_RGB: [u32; 8] = [
    0xF94144, 0xF3722C, 0xF9C74F, 0x90BE6D, 0x43AA8B, 0x577590, 0x9B5DE5, 0xF15BB5,
];
const PIXEL_LOCK_COUNT: usize = 64;

static FIREWORKS: Once<Canvas> = Once::new();
static PIXEL_LOCKS: [Mutex<()>; PIXEL_LOCK_COUNT] = [const { Mutex::new(()) }; PIXEL_LOCK_COUNT];
static STARTED: AtomicBool = AtomicBool::new(false);
static RNG_STATE: AtomicU32 = AtomicU32::new(0x5EED_C0DE);
static ACTIVE_ROCKETS: AtomicUsize = AtomicUsize::new(0);
static ACTIVE_PARTICLES: AtomicUsize = AtomicUsize::new(0);
static TOTAL_ROCKETS: AtomicUsize = AtomicUsize::new(0);
static TOTAL_PARTICLES: AtomicUsize = AtomicUsize::new(0);
static PEAK_ROCKETS: AtomicUsize = AtomicUsize::new(0);
static PEAK_PARTICLES: AtomicUsize = AtomicUsize::new(0);

struct Canvas {
    base: *mut u8,
    width: usize,
    height: usize,
    pitch: usize,
    bytes_per_pixel: usize,
    view_x: usize,
    view_y: usize,
    view_width: usize,
    view_height: usize,
    bg: u32,
    palette: [u32; PALETTE_RGB.len()],
}

#[derive(Copy, Clone)]
struct ParticleSeed {
    x: i32,
    y: i32,
    explosion_range: i32,
}

unsafe impl Send for Canvas {}
unsafe impl Sync for Canvas {}

impl Canvas {
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
        if width == 0 || height == 0 {
            return None;
        }

        let view_width = min(width, VIEW_WIDTH);
        let view_height = min(height, VIEW_HEIGHT);
        let bg = pack_color(
            BACKGROUND_RGB,
            fb.red_mask_size(),
            fb.red_mask_shift(),
            fb.green_mask_size(),
            fb.green_mask_shift(),
            fb.blue_mask_size(),
            fb.blue_mask_shift(),
        );
        let palette = PALETTE_RGB.map(|rgb| {
            pack_color(
                rgb,
                fb.red_mask_size(),
                fb.red_mask_shift(),
                fb.green_mask_size(),
                fb.green_mask_shift(),
                fb.blue_mask_size(),
                fb.blue_mask_shift(),
            )
        });

        Some(Self {
            base: fb.addr(),
            width,
            height,
            pitch: fb.pitch() as usize,
            bytes_per_pixel,
            view_x: width - view_width,
            view_y: 0,
            view_width,
            view_height,
            bg,
            palette,
        })
    }

    fn clear_view(&self) {
        for y in 0..self.view_height {
            for x in 0..self.view_width {
                self.put_pixel_unchecked(x, y, self.bg);
            }
        }
    }

    fn set_pixel(&self, color: u32, x: i32, y: i32) {
        if x < 0 || y < 0 {
            return;
        }

        let Ok(x) = usize::try_from(x) else {
            return;
        };
        let Ok(y) = usize::try_from(y) else {
            return;
        };
        if x >= self.view_width || y >= self.view_height {
            return;
        }

        let _guard = PIXEL_LOCKS[pixel_lock_index(x, y)].lock();
        self.put_pixel_unchecked(x, y, color);
    }

    fn put_pixel_unchecked(&self, x: usize, y: usize, color: u32) {
        let fb_x = self.view_x + x;
        let fb_y = self.view_y + y;
        debug_assert!(fb_x < self.width);
        debug_assert!(fb_y < self.height);

        let offset = fb_y * self.pitch + fb_x * self.bytes_per_pixel;
        unsafe {
            for idx in 0..self.bytes_per_pixel {
                self.base
                    .add(offset + idx)
                    .write_volatile((color >> (idx * 8)) as u8);
            }
        }
    }
}

/// Probes the framebuffer and reserves the firework viewport.
pub fn register() -> bool {
    if FIREWORKS.get().is_some() {
        return true;
    }

    let Some(canvas) = Canvas::probe() else {
        return false;
    };
    canvas.clear_view();
    FIREWORKS.call_once(|| canvas);
    true
}

/// Starts the stress-test launcher thread once the scheduler is available.
pub fn start() -> bool {
    if !register() {
        return false;
    }

    if STARTED.swap(true, Ordering::AcqRel) {
        return true;
    }

    let launcher_tid = sched::run(launcher_thread);
    let stats_tid = sched::run(stats_thread);
    info!(
        "fireworks: launcher thread {} stats thread {}",
        launcher_tid, stats_tid
    );
    true
}

fn launcher_thread() {
    let mut next_node = 0usize;

    loop {
        let count = 1 + (rand_u32() % 2) as usize;
        for _ in 0..count {
            spawn_rocket(next_node);
            next_node = next_node.wrapping_add(1);
        }

        let delay_ms = 2000 + (rand_u32() % 2000) as u64;
        clock::sleep(Duration::from_millis(delay_ms));
    }
}

fn spawn_rocket(_node_hint: usize) {
    TOTAL_ROCKETS.fetch_add(1, Ordering::Relaxed);
    let active = ACTIVE_ROCKETS.fetch_add(1, Ordering::AcqRel) + 1;
    update_peak(&PEAK_ROCKETS, active);
    let _ = sched::run(rocket_thread);
}

fn rocket_thread() {
    let _active = ActiveCounter::new(&ACTIVE_ROCKETS);
    let Some((view_width, view_height)) = view_size() else {
        return;
    };

    let offset_x = ((view_width as i32) * 400) / 1024;
    let mut act_x = int_to_fp(view_width as i32 / 2);
    let mut act_y = int_to_fp(view_height as i32 - 1);
    let vel_x = mul_fp(int_to_fp(offset_x), rand_signed_unit_fp());
    let mut vel_y =
        -int_to_fp(ROCKET_BASE_SPEED_MIN + (rand_u32() % ROCKET_BASE_SPEED_RANGE) as i32);
    let color = random_color();
    let explosion_range = EXPLOSION_RANGE_MIN + (rand_u32() % EXPLOSION_RANGE_RANGE) as i32;
    let expire_ms = ROCKET_LIFETIME_MIN_MS + (rand_u32() % ROCKET_LIFETIME_RANGE_MS);

    let mut x = fp_to_int(act_x);
    let mut y = fp_to_int(act_y);
    let mut elapsed_ms = 0;

    while elapsed_ms < expire_ms {
        set_pixel(color, x, y);
        clock::sleep(Duration::from_millis(INTERVAL_MS));
        elapsed_ms += INTERVAL_MS as u32;
        set_pixel(background_color(), x, y);

        act_x += scale_step(vel_x);
        act_y += scale_step(vel_y);
        x = fp_to_int(act_x);
        y = fp_to_int(act_y);
        vel_y += scale_gravity(GRAVITY);
    }

    let burst_count = EXPLOSION_COUNT_MIN + (rand_u32() % EXPLOSION_COUNT_RANGE) as usize;
    for _ in 0..burst_count {
        let Some(active) = try_reserve_active(&ACTIVE_PARTICLES, MAX_ACTIVE_PARTICLES) else {
            break;
        };
        let seed = ParticleSeed {
            x,
            y,
            explosion_range,
        };
        TOTAL_PARTICLES.fetch_add(1, Ordering::Relaxed);
        update_peak(&PEAK_PARTICLES, active);
        let _ = sched::run(move || particle_thread(seed));
    }
}

fn particle_thread(seed: ParticleSeed) {
    let _active = ActiveCounter::new(&ACTIVE_PARTICLES);
    let color = random_color();
    let mut act_x = int_to_fp(seed.x);
    let mut act_y = int_to_fp(seed.y);
    let direction = rand_u32() & FP_MASK;
    let (dir_x, dir_y) = unit_vector(direction);
    let speed = mul_fp(int_to_fp(seed.explosion_range), quarter_to_one_unit_fp());
    let vel_x = mul_fp(dir_x, speed);
    let mut vel_y = mul_fp(dir_y, speed);
    let expire_ms = PARTICLE_LIFETIME_MIN_MS + (rand_u32() % PARTICLE_LIFETIME_RANGE_MS);

    let mut x = fp_to_int(act_x);
    let mut y = fp_to_int(act_y);
    let mut elapsed_ms = 0;

    while elapsed_ms < expire_ms {
        set_pixel(color, x, y);
        clock::sleep(Duration::from_millis(INTERVAL_MS));
        elapsed_ms += INTERVAL_MS as u32;
        set_pixel(background_color(), x, y);

        act_x += scale_step(vel_x);
        act_y += scale_step(vel_y);
        x = fp_to_int(act_x);
        y = fp_to_int(act_y);
        vel_y += scale_gravity(GRAVITY);
    }
}

fn stats_thread() {
    loop {
        let heap = alloc::stats();
        let sched = sched::stats();
        let active_rockets = ACTIVE_ROCKETS.load(Ordering::Relaxed);
        let active_particles = ACTIVE_PARTICLES.load(Ordering::Relaxed);
        let total_rockets = TOTAL_ROCKETS.load(Ordering::Relaxed);
        let total_particles = TOTAL_PARTICLES.load(Ordering::Relaxed);
        let peak_rockets = PEAK_ROCKETS.load(Ordering::Relaxed);
        let peak_particles = PEAK_PARTICLES.load(Ordering::Relaxed);
        let heap_used_kib = (heap.slab_used_bytes + heap.large_bytes) / 1024;
        let heap_free_kib = heap.slab_free_bytes / 1024;

        info!(
            "fw: hu={}K hf={}K hp={}/{}/{}/{} sl={} ss={} rc={} ac={} cp={}/{} bc={}:{} tc={} ar={} ap={} tr={} tp={} pr={} pp={}",
            heap_used_kib,
            heap_free_kib,
            heap.free_pages,
            heap.slab_pages,
            heap.large_pages,
            heap.reserved_pages + heap.retired_pages,
            sched.total_load,
            sched.total_sysload,
            sched.runnable_cpus,
            sched.active_cpus,
            sched.online_cpus,
            sched.cpu_count,
            sched.busiest_cpu,
            sched.busiest_load,
            sched.total_threads_created,
            active_rockets,
            active_particles,
            total_rockets,
            total_particles,
            peak_rockets,
            peak_particles,
        );

        clock::sleep(Duration::from_secs(1));
    }
}

fn set_pixel(color: u32, x: i32, y: i32) {
    if let Some(canvas) = canvas() {
        canvas.set_pixel(color, x, y);
    }
}

fn background_color() -> u32 {
    canvas().map_or(0, |canvas| canvas.bg)
}

fn random_color() -> u32 {
    let idx = (rand_u32() as usize) % PALETTE_RGB.len();
    canvas().map_or(0, |canvas| canvas.palette[idx])
}

fn view_size() -> Option<(usize, usize)> {
    canvas().map(|canvas| (canvas.view_width, canvas.view_height))
}

fn rand_u32() -> u32 {
    let mut value = RNG_STATE.load(Ordering::Relaxed);
    loop {
        let next = value.wrapping_mul(1103515245).wrapping_add(12345);
        match RNG_STATE.compare_exchange_weak(value, next, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => return temper(next),
            Err(observed) => value = observed,
        }
    }
}

fn temper(mut x: u32) -> u32 {
    x ^= x >> 11;
    x ^= (x << 7) & 0x9d2c_5680;
    x ^= (x << 15) & 0xefc6_0000;
    x ^ (x >> 18)
}

fn rand_signed_unit_fp() -> i32 {
    let value = (rand_u32() & FP_MASK) as i32;
    if rand_u32() & 1 == 0 {
        value
    } else {
        -value
    }
}

fn quarter_to_one_unit_fp() -> i32 {
    let quarter = FP_ONE / 4;
    quarter + (rand_u32() & (FP_MASK - quarter as u32)) as i32
}

fn int_to_fp(value: i32) -> i32 {
    value << FP_SHIFT
}

fn fp_to_int(value: i32) -> i32 {
    value >> FP_SHIFT
}

fn mul_fp(lhs: i32, rhs: i32) -> i32 {
    (((lhs as i64) * (rhs as i64)) >> FP_SHIFT) as i32
}

fn scale_step(velocity_fp: i32) -> i32 {
    (((velocity_fp as i64) * (INTERVAL_MS as i64)) / 1000) as i32
}

fn scale_gravity(gravity: i32) -> i32 {
    ((int_to_fp(gravity) as i64 * INTERVAL_MS as i64) / 1000) as i32
}

fn update_peak(peak: &AtomicUsize, current: usize) {
    let mut observed = peak.load(Ordering::Relaxed);
    while current > observed {
        match peak.compare_exchange_weak(observed, current, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => break,
            Err(next) => observed = next,
        }
    }
}

fn try_reserve_active(counter: &AtomicUsize, limit: usize) -> Option<usize> {
    let mut observed = counter.load(Ordering::Relaxed);
    loop {
        if observed >= limit {
            return None;
        }

        match counter.compare_exchange_weak(
            observed,
            observed + 1,
            Ordering::AcqRel,
            Ordering::Relaxed,
        ) {
            Ok(_) => return Some(observed + 1),
            Err(next) => observed = next,
        }
    }
}

fn pixel_lock_index(x: usize, y: usize) -> usize {
    ((y << 4) ^ x) % PIXEL_LOCK_COUNT
}

fn canvas() -> Option<&'static Canvas> {
    FIREWORKS.get()
}

fn unit_vector(angle: u32) -> (i32, i32) {
    let octant_shift = FP_SHIFT - 3;
    let octant = ((angle >> octant_shift) & 0x7) as u8;
    let frac_mask = (1 << octant_shift) - 1;
    let frac = (angle & frac_mask) as i32;
    let frac_fp = frac << 3;
    let inv_fp = FP_ONE - frac_fp;

    match octant {
        0 => (inv_fp, frac_fp),
        1 => (frac_fp, inv_fp),
        2 => (-frac_fp, inv_fp),
        3 => (-inv_fp, frac_fp),
        4 => (-inv_fp, -frac_fp),
        5 => (-frac_fp, -inv_fp),
        6 => (frac_fp, -inv_fp),
        _ => (inv_fp, -frac_fp),
    }
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

struct ActiveCounter(&'static AtomicUsize);

impl ActiveCounter {
    fn new(counter: &'static AtomicUsize) -> Self {
        Self(counter)
    }
}

impl Drop for ActiveCounter {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::AcqRel);
    }
}
