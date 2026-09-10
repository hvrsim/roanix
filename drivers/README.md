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

Enumerators publish devices on the kernel `platform` bus; drivers bind through
match tables. Kernel policy and always-present services are deliberately not
modules: tmpfs/devfs live in `kernel/src/fs`, while TTY, console, PTY, and
special-device services live in `kernel/src/dev`. A UART module supplies only
the hardware backend to the native terminal layer.

## Building

```
./x.py --arch x86_64        # build, install, and boot x86_64
./x.py --arch riscv64       # build, install, and boot RISC-V
```

The workspace targets `x86_64-unknown-none` and `riscv64gc-unknown-none-elf`
with the pinned nightly toolchain in `rust-toolchain.toml`. RISC-V builds
rebuild `core` and `alloc` with `-Z build-std` so every image stays position
independent. xtool keeps Cargo intermediates under `build/cargo/drivers`,
copies finished modules to `build/<arch>/drivers-rust`, and installs them
directly into the staged sysroot alongside the public headers.

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
