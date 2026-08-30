# Roanix drivers

Every in-tree driver is a Rust crate in this Cargo workspace. Its `build.rs`
asks the DDK to configure Cargo's final link as a freestanding `.ko` shared
object loaded by the kernel from `/usr/lib/roanix/drivers`. Modules use the C
ABI described by `include/roanix/`: they enter through `rdf_module_entry`,
import only the versioned `rdf_api_v1_*` functions they call, and expose
operation records made of plain function pointers.

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

The kernel resolves every used `rdf_api_v1_*` import before calling the module
entry point; unknown or internal kernel symbols make the module fail to load.
