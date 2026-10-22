# Roanix hardware drivers

Only hardware-facing code is a loadable module. Every in-tree module is a Rust
crate whose `build.rs` asks the DDK to link a freestanding `.ko` image. The
kernel loads those images from `/usr/lib/roanix/drivers`.

Modules use ABI generation 2 from `include/roanix/`: they enter through
`rdf_module_entry`, import only the `rdf_api_v2_*` functions they call, and
expose size-prefixed operation records made of plain function pointers. There
is no general kernel symbol namespace.

## Layout

| Directory          | Module            | Purpose                                  |
| ------------------ | ----------------- | ---------------------------------------- |
| `ddk/`             | —                 | The no_std driver development kit        |
| `platform/acpi/`   | `acpi.ko`         | x86_64 ACPI platform enumerator           |
| `platform/fdt/`    | `fdt.ko`          | riscv64 device-tree enumerator            |
| `irqchip/ioapic/`  | `ioapic.ko`       | I/O APIC interrupt controller             |
| `irqchip/plic/`    | `plic.ko`         | RISC-V interrupt controller               |
| `bus/pci/`         | `pci.ko`          | PCI/PCIe enumeration and configuration    |
| `storage/nvme/`    | `storage-nvme.ko` | NVMe hardware driver                      |
| `tty/uart8250/`    | `uart8250.ko`     | 8250/16550 serial hardware driver         |
| `input/evdev/`     | —                | Shared evdev keyboard queues and ioctls   |
| `input/i8042/`     | `i8042.ko`        | PC keyboard controller                   |
| `usb/usb/`         | —                | USB host service ABI                     |
| `usb/xhci/`        | `usb-xhci.ko`     | xHCI root ports, control and interrupt IN |
| `usb/keyboard/`    | `usb-keyboard.ko` | USB HID boot keyboards                   |

Enumerators publish devices on the kernel `platform` bus; drivers bind through
match tables. Kernel policy and always-present services are deliberately not
modules: tmpfs/devfs live in `kernel/src/fs`, while TTY, console, PTY, and
special-device services live in `kernel/src/dev`. A UART module supplies only
the hardware backend to the native terminal layer.

## Building

```
./x.py --arch x86_64        # build, install, and boot x86_64
./x.py --arch riscv64       # build, install, and boot RISC-V
./x.py build drivers       # build only the hardware modules
```

The workspace targets `x86_64-unknown-none` and `riscv64gc-unknown-none-elf`
with the pinned nightly toolchain in `rust-toolchain.toml`. RISC-V builds
rebuild `core` and `alloc` with `-Z build-std` so every image stays position
independent. `x.py` lets Cargo detect changes and keeps intermediates under
`build/cargo/drivers`. It copies finished modules to `build/<arch>/drivers-rust`
and installs them directly into the staged sysroot alongside the public headers.

Drivers always use the release profile. Add new module packages to the
architecture's `DRIVERS` tuple in `x.py`.

## Writing a driver

Depend on `ddk`, implement probe/remove callbacks over its safe wrappers,
and finish with:

```rust
ddk::module!("my-driver", "What it does", init, exit);
```

The kernel resolves every used `rdf_api_v2_*` import before calling the module
entry point; unknown or internal kernel symbols make the module fail to load.

Storage drivers register `raw::BlockOps` through `BlockRegistration`. The
callbacks operate on whole logical blocks only; the kernel owns unaligned byte
I/O, devfs nodes, transfer splitting, partition parsing, and mount exclusion.
After the controller and callback context are fully published, call
`BlockRegistration::rescan()` once to discover GPT or MBR partitions. Dropping
or explicitly unregistering the receipt withdraws the disk only when its nodes
and mounted filesystems are idle.

## USB keyboards

`usb-xhci` claims PCI xHCI functions, owns their coherent command/event rings,
and enumerates directly attached USB devices. It publishes USB interfaces on
its `usb` bus and supplies a versioned `usb.host` service for control requests
and interrupt IN transfers. x86 uses one MSI/MSI-X interrupter; RISC-V uses
PLIC-routed INTx. Enumeration and waits run in worker context; the hard
handler acknowledges and drains completion events without allocating.

`usb-keyboard` binds HID boot keyboard interfaces, selects boot protocol, and
translates modifier and six-key reports into Linux evdev keycodes. It shares
the evdev implementation with i8042. Up to eight keyboard endpoints, including
readers retaining disconnected devices, can coexist. Disconnection cancels
pending reads before freeing controller DMA state. A descriptor kept open after
unplug retains its static evdev endpoint until it can be reclaimed safely.

The initial host service supports root-port keyboards. External hubs, bulk and
isochronous transfers, HID report-protocol-only devices, and other USB classes
are not implemented. These limits do not require changes in kmscon's input path.

Modules may return `EDEFERRED` when a required bus is unavailable. The packaged
module loader retries them after other modules have loaded, stopping if a pass
makes no progress. Device-probe deferral remains a separate mechanism.
