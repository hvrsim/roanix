use core::arch::asm;

mod log;

pub fn early() {
    log::setup();
}

pub fn hcf() -> ! {
    loop {
        unsafe {
            asm!("hlt");
        }
    }
}
