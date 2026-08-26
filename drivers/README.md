# Roanix drivers

Every in-tree driver is a Rust crate in this Cargo workspace, linked into a
freestanding `.ko` shared object by `build-module.py` and loaded by the kernel
from `/usr/lib/roanix/drivers`. Modules use the C ABI described by
`include/roanix/`: they enter through `rdf_module_entry`, receive a
size-prefixed service table, and expose operation records made of plain
function pointers.

## Layout

| Directory          | Module      | Purpose                                    |
| ------------------ | ----------- | ------------------------------------------ |
| `ddk/`             | —           | The no_std driver development kit           |
| `platform/acpi/`   | `acpi.ko`   | x86_64 ACPI (MADT) platform enumerator      |
| `platform/fdt/`    | `fdt.ko`    | riscv64 flattened device-tree enumerator    |
| `irqchip/ioapic/`  | `ioapic.ko` | I/O APIC interrupt controller and domain    |
| `tty/uart8250/`    | `uart8250.ko` | 8250/16550 serial port driver             |
| `tty/console/`     | `console.ko`  | Terminal line discipline and tty provider |
| `tty/pty/`         | `pty.ko`    | ptmx/pts pseudo-terminal driver             |
| `tty/special/`     | `special.ko`  | null, zero, random, and urandom devices   |
| `fs/tmpfs/`        | `tmpfs.ko`  | Loadable sparse temporary filesystem        |
| `fs/devfs/`        | `devfs.ko`  | Device filesystem and namespace broker      |

Enumerators publish devices on the kernel `platform` bus; driver crates bind
to them through match tables. `console`, `tmpfs`, and `devfs` are providers:
they register class, filesystem-provider, and devfs-broker interfaces that the
other modules consume.

## Building

```
make ARCH=x86_64            # build out/x86_64/*.ko
make ARCH=riscv64           # build out/riscv64/*.ko
make ARCH=x86_64 install    # install modules and headers under DESTDIR/PREFIX
make clean                  # remove build and output trees
```

The workspace targets `x86_64-unknown-none` and `riscv64gc-unknown-none-elf`
with the pinned nightly toolchain in `rust-toolchain.toml`. RISC-V builds
rebuild `core` and `alloc` with `-Z build-std` so every image stays position
independent.

A module image may instead be dropped into `prebuilt/<arch>/<name>.ko`; the
Makefile prefers prebuilt images, which is how xtool stages host-built modules
for packaging without requiring a toolchain in the packaging environment.

## Writing a driver

Depend on `ddk`, implement probe/remove callbacks over its safe wrappers,
and finish with:

```rust
ddk::module!("my-driver", "What it does", init, exit);
```

Build the crate with `--features kernel-imports` (as the Makefile does) so
kernel services resolve to the curated versioned `rdf_api_v1_*` imports
instead of runtime service-table lookups.
