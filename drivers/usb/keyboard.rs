//!
//! # USB Boot Keyboards
//!
//! A worker reads HID boot reports and converts modifier and key changes
//! into evdev events. Each attached keyboard claims a static evdev endpoint.
//!
//! Removal cancels outstanding transfers and joins the worker before
//! releasing its context. A disconnected reader may keep the static
//! endpoint alive until it closes its file.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code, clippy::cast_possible_truncation)]
extern crate alloc;
mod boot;

use core::ptr;
use ddk::{Module, Result, raw};

static SLOTS: [boot::Slot; 8] = [const { boot::Slot::new(c"USB HID keyboard") }; 8];
static DRIVER: boot::Driver = boot::Driver::new(&SLOTS, 8, publish, report);

const MODIFIERS: [u16; 8] = [29, 42, 56, 125, 97, 54, 100, 126];

/// Maps a USB HID keyboard usage to its evdev key code.
fn keycode(usage: u8) -> u16 {
    match usage {
        4..=29 => [
            30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38, 50, 49, 24, 25, 16, 19, 31, 20, 22, 47,
            17, 45, 21, 44,
        ][usize::from(usage - 4)],
        30..=39 => u16::from(usage - 28),
        40..=57 => [
            28, 1, 14, 15, 57, 12, 13, 26, 27, 43, 43, 39, 40, 41, 51, 52, 53, 58,
        ][usize::from(usage - 40)],
        58..=69 => [59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 87, 88][usize::from(usage - 58)],
        70..=103 => [
            99, 70, 119, 110, 102, 104, 111, 107, 109, 106, 105, 108, 103, 69, 98, 55, 74, 78, 96,
            79, 80, 81, 75, 76, 77, 71, 72, 73, 82, 83, 86, 127, 116, 117,
        ][usize::from(usage - 70)],
        104..=115 => u16::from(usage) + 79,
        117 => 138,
        118 => 139,
        120 => 128,
        121 => 129,
        122 => 131,
        123 => 137,
        124 => 133,
        125 => 135,
        126 => 136,
        127 => 113,
        128 => 115,
        129 => 114,
        _ => 0,
    }
}

/// Builds the bitmap of keys supported by the boot keyboard decoder.
fn capabilities() -> [u8; evdev::KEY_BYTES] {
    let mut keys = [0; evdev::KEY_BYTES];
    for code in (0..=255).map(keycode).chain(MODIFIERS) {
        if code != 0 {
            keys[usize::from(code) / 8] |= 1 << (code % 8);
        }
    }
    keys
}

/// Publishes the boot keyboard's evdev capability bitmap.
fn publish(endpoint: &'static evdev::Endpoint) -> Result<()> {
    endpoint.publish(capabilities())
}

/// Reports changed modifiers and keys while preserving held keys across rollover reports.
fn report(keyboard: &evdev::Endpoint, previous: &mut [u8; 8], current: &[u8]) {
    let Ok(current) = <[u8; 8]>::try_from(current) else {
        return;
    };

    for (bit, code) in MODIFIERS.into_iter().enumerate() {
        if (previous[0] ^ current[0]) & (1 << bit) != 0 {
            keyboard.event(1, code, i32::from(current[0] & (1 << bit) != 0));
        }
    }
    previous[0] = current[0];
    // ErrorRollOver/POSTFail/Undefined reports do not release held keys.
    if !current[2..].iter().any(|usage| (1..=3).contains(usage)) {
        for usage in &previous[2..] {
            let code = keycode(*usage);
            if code != 0 && !current[2..].contains(usage) {
                keyboard.event(1, code, 0);
            }
        }

        for (index, usage) in current[2..].iter().enumerate() {
            let code = keycode(*usage);
            if code != 0 && !previous[2..].contains(usage) && !current[2..2 + index].contains(usage)
            {
                keyboard.event(1, code, 1);
            }
        }
        *previous = current;
    }
    keyboard.event(0, 0, 0);
}

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_ID,
    flags: 0,
    key: c"class".as_ptr(),
    value: ptr::null(),
    id0: 0x03_01_01,
    mask0: 0xff_ffff,
    id1: 0,
    mask1: 0,
    data: 0,
    score: 0,
}];
/// Registers the USB boot keyboard class driver.
fn init(_: Module) -> Result<()> {
    DRIVER.register(c"usb-keyboard", &MATCHES)
}

/// Withdraws the USB boot keyboard class driver.
fn exit(_: Module) {
    DRIVER.unregister();
}

ddk::module!(b"usb-keyboard\0", b"USB HID boot keyboard\0", init, exit);
