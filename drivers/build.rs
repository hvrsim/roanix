//! Configures the shared Cargo build for loadable driver modules.

/// Configures freestanding linking for every loadable driver target.
fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    ddk::configure_module_links(&[
        "pci",
        "i8042",
        "ioapic",
        "plic",
        "acpi",
        "fdt",
        "nvme",
        "uart8250",
        "usb-keyboard",
        "usb-mouse",
        "usb-xhci",
        "usb-storage",
    ]);
}
