//!
//! # USB Boot Mice
//!
//! The USB boot protocol provides signed relative X and Y motion and three
//! buttons. The shared boot driver owns the interrupt worker and evdev node.
//! Each completed report is published as one evdev frame.
//!

#![no_std]
#![no_main]
#![allow(unsafe_code)]

extern crate alloc;

mod boot;

use core::ptr;
use ddk::{Module, Result, raw};

static SLOTS: [boot::Slot; 8] = [const { boot::Slot::new(c"USB HID mouse") }; 8];
static DRIVER: boot::Driver = boot::Driver::new(&SLOTS, 3, publish, report);

/// Publishes the axes and buttons guaranteed by the USB boot mouse format.
fn publish(endpoint: &'static evdev::Endpoint) -> Result<()> {
    endpoint.publish_mouse(false)
}

/// Decodes the standard portion of a boot mouse report.
fn report(endpoint: &evdev::Endpoint, _: &mut [u8; 8], bytes: &[u8]) {
    let [buttons, x, y, ..] = bytes else {
        return;
    };
    endpoint.mouse(
        *buttons & 7,
        i32::from(i8::from_ne_bytes([*x])),
        i32::from(i8::from_ne_bytes([*y])),
        0,
    );
}

static MATCHES: [raw::Match; 1] = [raw::Match {
    kind: ddk::MATCH_ID,
    flags: 0,
    key: c"class".as_ptr(),
    value: ptr::null(),
    id0: 0x03_01_02,
    mask0: 0xff_ffff,
    id1: 0,
    mask1: 0,
    data: 0,
    score: 0,
}];

/// Registers the USB boot mouse class driver.
fn init(_: Module) -> Result<()> {
    DRIVER.register(c"usb-mouse", &MATCHES)
}

/// Withdraws the USB boot mouse class driver.
fn exit(_: Module) {
    DRIVER.unregister();
}

ddk::module!(b"usb-mouse\0", b"USB HID boot mouse\0", init, exit);
