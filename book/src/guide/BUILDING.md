# Building & Running Roanix

Roanix is built with **xtool**, a single-file Python program in the project root
(`x.py`). It drives the Rust kernel build, the [Jinx] userland package manager,
image creation, and QEMU, so that every common task is a single command.

A quick note before starting: only Linux is supported as a build host. Other
platforms will either not work (Windows and other non-UNIX-like systems) or are
untested (\*BSD, macOS).

[Jinx]: https://github.com/Mintsuki/Jinx

## The short version

```bash
$ ./x.py doctor     # check that the host has everything xtool needs
$ ./x.py            # build whatever changed and boot it in QEMU
```

`./x.py` with no arguments is shorthand for `./x.py run`: it rebuilds the
kernel if the sources changed, rebuilds any userland package whose recipe
changed, reinstalls the sysroot, repacks the bootstrap initramfs, refreshes the
ext4 root disk and boot image, and boots the result. Anything already up to
date is skipped, so a no-op run finishes in well under a second.

## Dependencies

Because the kernel selects its toolchain through `rust-toolchain.toml`, an
installation of [rustup] is required. Disk image creation needs `xorriso`,
`mtools`, `e2fsprogs`, and `sgdisk`; Python 3.11 or newer is required to run xtool itself;
and QEMU is used for testing. Building the userland additionally needs the Jinx
host dependencies: Bash, awk, findutils, Git, GNU Make, grep, gzip, sed, tar,
zstd, coreutils, procps, and util-linux. Jinx normally uses `wget`; xtool also
accepts `curl` and provides a shim.

[rustup]: https://rustup.rs/

**Ubuntu/Debian:**

```bash
$ sudo apt-get install make python3 xorriso mtools e2fsprogs gdisk zstd qemu-system-x86
```

**Alpine:**

```bash
$ sudo apk add make python3 xorriso mtools e2fsprogs sgdisk zstd qemu qemu-system-x86_64
```

**Arch:**

```bash
$ sudo pacman -S make python xorriso mtools e2fsprogs gptfdisk zstd qemu-system-x86
```

Rather than checking these by hand, run:

```bash
$ ./x.py doctor
```

It reports every tool xtool needs, prints the exact install command for
anything missing, verifies the required Rust target is installed, and tells you
whether KVM is usable.

## Configuring the build

Every command accepts the same common options:

| Option | Meaning |
|---|---|
| `-a`, `--arch` | Target architecture: `x86_64` or `riscv64`; repeat to act on several |
| `-A`, `--all-arches` | Act on every architecture |
| `-p`, `--profile` | Cargo profile: `dev` or `release` |
| `-r`, `--release` | Shorthand for `--profile release` |
| `-j`, `--jobs` | Parallel jobs handed to Cargo and Jinx |
| `-f`, `--force` | Rebuild even when nothing changed |
| `-v`, `--verbose` | Echo every command and stream all output |
| `-q`, `--quiet` | Print warnings and errors only |
| `-n`, `--dry-run` | Describe what would happen without doing it |
| `--offline` | Never touch the network |

If you always work on one architecture, save it once instead of repeating the
flag:

```bash
$ ./x.py config set arch riscv64
$ ./x.py config set profile release
$ ./x.py config                     # show everything xtool remembers
```

Saved settings live in `build/config.json`. Resolution order is
**command line flag → `ROANIX_*` environment variable → saved config →
built-in default**, so a flag always wins.

## Working on two architectures at once

Nothing is shared between architectures except immutable downloads, so x86_64
and riscv64 builds coexist without ever invalidating each other. Switching back
and forth costs nothing; both keep their own incremental state.

The most comfortable setup is one terminal per architecture:

```bash
# terminal 1                    # terminal 2
$ export ROANIX_ARCH=x86_64     $ export ROANIX_ARCH=riscv64
$ ./x.py                        $ ./x.py
```

Alternatively, save your primary architecture and override it ad hoc, or drive
both from a single command:

```bash
$ ./x.py config set arch x86_64      # the default from now on
$ ./x.py -a riscv64 build kernel     # one-off override

$ ./x.py -A status                   # report on both
$ ./x.py -a x86_64 -a riscv64 build kernel
$ ./x.py -A pkg bash                 # rebuild Bash for both
```

`run`, `shell`, `regen`, and `port` act on one architecture and will say so if
handed more than one.

The cost of a second architecture is disk, not correctness. Each needs its own
Jinx tree, and most of that is the cross-toolchain, which is per-target and
cannot be shared:

```bash
$ du -sh build/*
214M    build/cache        # shared: Limine, OVMF, Jinx
1.3G    build/cargo        # shared: Cargo target dir, namespaced by triple
9.8G    build/x86_64
42M     build/riscv64
```

To reclaim one entirely:

```bash
$ ./x.py clean arch -a riscv64       # or simply: rm -rf build/riscv64
```

## Seeing what a build will do

```bash
$ ./x.py status
```

`status` reports, for the current architecture and profile, whether the kernel,
the userland, the initramfs, and the images are up to date - and, when the
userland is stale, exactly which recipes changed. It is read-only and fast.

## Build targets

```bash
# Build the bootable HDD image (kernel + userland + bootloader)
$ ./x.py build hdd

# Build the ISO instead
$ ./x.py build iso

# Kernel only
$ ./x.py build kernel

# Build every userland package and install it into
# build/<arch>/sysroot
$ ./x.py build sysroot

# Pack the bootstrap userland and hardware modules as
# build/<arch>/out/<profile>/roanix-<arch>.initramfs.tar.gz
$ ./x.py build initramfs

# Put the complete sysroot in a GPT-partitioned ext4 disk image
$ ./x.py build rootfs

# Both images at once, for release
$ ./x.py build all --release
```

Builds are incremental. xtool fingerprints the kernel sources, each userland
recipe, and the image inputs, and stores that state under `build/<arch>/state/`.
Pass `--force` to rebuild the whole chain regardless.

To build and boot in one step, add `--run`, or simply use `./x.py run`.

## Working on the userland

The userland is a set of [Jinx] recipes under `userland/recipes/`. xtool drives
Jinx directly, so you rarely need to invoke `jinx` yourself.

Which packages end up in the system image is controlled by
`userland/system.list`:

```
[install]      # top-level packages installed into the image
bash
coreutils
...

[build]        # built, but not installed directly
mlibc-headers
mlibc
```

Runtime dependencies are resolved automatically, so only the things you
actually want on the system need to be listed.

### Rebuilding a package

```bash
# Rebuild Bash and reinstall the sysroot
$ ./x.py pkg bash

# ...and boot the result straight away
$ ./x.py pkg bash --run

# Changing a library rebuilds everything linked against it
$ ./x.py pkg mlibc

# ...unless you explicitly ask for just the one package
$ ./x.py pkg mlibc --only

# Globs work
$ ./x.py pkg 'lib*'
```

You usually do not even need `pkg`: editing a recipe or `userland/util-roanix/`
marks its package stale, while editing `drivers/` marks the directly installed
driver modules stale. A plain `./x.py` rebuilds what changed, reinstalls the
sysroot, and boots.

Use `./x.py list` to see every recipe, its version, whether it has been built,
and whether it is part of the system image.

### Porting something new

```bash
$ ./x.py port zstd \
    --url https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz \
    --version 1.5.7 \
    --template meson
```

`port` downloads the tarball, records its BLAKE2b checksum, writes a recipe
skeleton for the chosen build system (`autotools`, `meson`, `cmake`, or
`make`), and adds the package to `userland/system.list`. Edit the generated
recipe, then:

```bash
$ ./x.py pkg zstd       # build it
$ ./x.py run            # boot Roanix with it installed
```

### Debugging a failing port

```bash
# Drop into the exact container Jinx would build the package in
$ ./x.py shell zstd

# Run a one-off command in that environment
$ ./x.py shell zstd meson test

# Host recipes work too
$ ./x.py shell host:gcc-host
```

The sysroot, host tools, and image dependencies are all present, and the shell
starts in the recipe's build directory.

### Iterating on patches

Edit the unpacked tree in `userland/sources/<pkg>-workdir/`, then:

```bash
$ ./x.py regen zstd     # turn those edits into a patch and re-run prepare()
$ ./x.py pkg zstd       # rebuild with the new patch
```

### Escape hatch

Anything xtool does not wrap can be run directly; xtool supplies the correct
build directory, architecture, and environment:

```bash
$ ./x.py jinx dry-run '*'
$ ./x.py jinx build host:gcc-host
$ ./x.py revbump mlibc          # bump the revision of every dependent recipe
```

## Running Roanix

xtool only supports QEMU. Using other emulators such as [VMware] or [Simics] is
possible but not supported.

[VMware]: https://vmware.com
[Simics]: https://www.intel.com/content/www/us/en/developer/articles/tool/simics-simulator.html

```bash
# Build anything stale, then boot
$ ./x.py run

# Boot the ISO under riscv64
$ ./x.py run iso --arch riscv64

# x86_64 through SeaBIOS instead of UEFI
$ ./x.py run --firmware bios

# Boot the existing image without rebuilding anything
$ ./x.py run --no-build

# Pass raw flags straight to QEMU
$ ./x.py run -- -d int -no-reboot
```

Useful emulator options:

| Option | Meaning |
|---|---|
| `--gdb` | Expose a GDB stub on `tcp::1234` and halt at reset |
| `--gdb-port` | Use a different port for the stub |
| `--no-wait` | With `--gdb`, start running instead of halting |
| `--tcg` | Disable KVM and use pure emulation |
| `-m`, `--smp` | Guest memory size and CPU count |
| `--serial-log` | Also write the serial console to a file |
| `--trace` | QEMU `-d` items to log, e.g. `int,cpu_reset` |
| `--monitor` | Multiplex the QEMU monitor onto the serial console |
| `--display` | QEMU display backend, e.g. `none` |

### RISC-V device-tree modules

If RISC-V UEFI firmware does not publish a device tree, add the board DTB as
an uncompressed Limine module:

```text
    module_path: boot():/boot/k1-milkv-jupiter.dtb
    module_string: dtb
```

Roanix prefers a valid module tagged `dtb`, `devicetree`, or `device-tree`
over Limine's device-tree response. It next considers modules whose path ends
in `.dtb`, then any module with a valid FDT header. Do not prefix the path with
`$` unless the file itself is gzip-compressed; Limine uses that marker to
request transparent decompression.

### Debugging the kernel

```bash
$ ./x.py run --gdb
```

xtool prints the exact `rust-gdb` command to connect with. In another terminal:

```bash
$ rust-gdb build/x86_64/out/dev/roanix -ex 'target remote :1234'
```

## Kernel maintenance

```bash
$ ./x.py check          # cargo check
$ ./x.py lint           # cargo clippy, warnings denied
$ ./x.py fmt            # rustfmt
$ ./x.py fmt --check    # verify formatting without rewriting
$ ./x.py docs --serve   # serve this book
$ ./x.py docs rust --serve
```

## Cleaning up

```bash
$ ./x.py clean                  # this arch's images and build state (the default)
$ ./x.py clean sysroot
$ ./x.py clean arch             # everything for this architecture
$ ./x.py clean arch -a riscv64  # ...or for a specific one
$ ./x.py clean all --yes        # the whole build/ tree
```

Per-architecture targets — `out`, `cargo`, `sysroot`, `packages`, `state`, and
`arch` — only ever touch `build/<arch>/`, so cleaning one architecture never
disturbs another. `sources` and `cache` are shared by every architecture and are
labelled as such in the output. Anything that forces a long rebuild asks for
confirmation first unless `--yes` is given.

## Layout

Everything under `build/` follows one rule: **anything that depends on the
target architecture lives in `build/<arch>/`, and anything shared sits beside
it.** So `du -sh build/*` tells you what each architecture costs, and
`rm -rf build/<arch>` forgets one completely.

```
build/
├── config.json              saved defaults from `x.py config set`
├── cache/                   shared, immutable, pinned downloads
│   ├── downloads/           verified tarballs
│   ├── limine/  edk2-ovmf/  bootloader and UEFI firmware
│   ├── jinx/                pinned Jinx checkout
│   └── host-tools/          the wget shim, when only curl is present
├── cargo/                   CARGO_TARGET_DIR; Cargo namespaces by triple
└── x86_64/
    ├── out/dev/             roanix, *.initramfs.tar.gz, *.root.img,
    │                        *.hdd, *.iso
    ├── out/release/
    ├── jinx/                Jinx build directory, pkgs/, builds/
    ├── sysroot/             installed userland
    ├── firmware/            per-machine UEFI variables
    ├── state/               incremental build fingerprints
    └── tmp/                 scratch space
```

Older checkouts used a `build/runtime/` tree. Nothing under `build/` is tracked
by Git, so if you have one, `rm -rf build` and rebuild — or move the pieces into
place by hand if you want to keep a populated Jinx tree.

## Driver modules

Only hardware-facing code is loadable. In-tree modules are freestanding Rust
shared objects from the `drivers/` workspace:

```text
drivers/
├── ddk/                 no_std driver development kit
├── platform/{acpi,fdt}/ firmware enumerators
├── irqchip/{ioapic,plic}/ interrupt controllers
├── bus/pci/             PCI/PCIe bus driver
├── storage/nvme/        NVMe controller driver
└── tty/uart8250/        8250/16550 serial hardware driver
```

Always-present software belongs in the kernel. `kernel/src/fs/{tmpfs,devfs}.rs`
provides the root and device filesystems directly; `kernel/src/dev/` owns the
TTY line discipline, console adapter, PTYs, and special devices. Consequently
boot does not depend on loading filesystem or terminal-provider modules, and
ordinary VFS/TTY operations avoid a module ABI callback layer.

The remaining driver framework has four concepts: devices form one topology;
buses define enumeration and matching; drivers bind to devices using match
tables; and named, versioned interfaces support the few cross-driver protocols.
The old generic class layer is gone. Kernel subsystems should use native Rust
traits and explicit registries instead of turning software policy into a
module.

Storage follows the same boundary. `kernel/src/dev/block.rs` owns block-device
geometry, byte-range adaptation, `/dev` publication, transfer splitting, and
mount lifetime claims. A hardware module supplies only exact logical-block
callbacks, then requests a scan after its controller state reaches its final
address. `kernel/src/fs/partition.rs` validates GPT (including both CRCs) and
MBR/EBR tables and publishes partitions as derived block devices.

Disk filesystems are kernel-owned. `kernel/src/fs/ext4.rs` supplies the system
root and currently mounts a deliberately constrained ext4 feature set
read-only. It supports 1--4 KiB blocks, 32- and 64-bit group descriptors,
extent trees, sparse and uninitialized extents, typed linear directories,
Unix modes, fast and extent-backed symbolic links, statfs, and executable
mappings. It rejects journals requiring replay, directory indexes, bigalloc,
inline data, encryption, metadata checksums, and unknown incompatible features.
The generated root image disables those features explicitly.

`kernel/src/fs/fat32.rs` remains available for FAT32/VFAT data volumes. It
validates cluster chains and BPB geometry, decodes checksum-protected UTF-16
long names, and implements lookup, readdir, reads, mappings, statfs, and sync.
Both implementations stay read-only until their metadata allocation and crash
ordering rules are implemented. Regular reads and mappings now share one
mount-owned `VmObject` per file. Immutable pagers load missing pages on demand,
batch contiguous reads up to 128 KiB, and read ahead for sequential access.
Clean file frames participate in VM reclamation and can be discarded and
reloaded without occupying swap. Metadata uses the same VM infrastructure;
there is no separate cache of regular file data in the block layer.

`kernel/src/fs/io.rs` provides a validated extent pager for other immutable
filesystems. FAT32 retains its cluster index across reads and combines adjacent
clusters into device transfers. Pagers retain device claims independently of
the mount, preserving existing mappings after unmount. File page fills release
the address-space lock before I/O and revalidate mappings before installing
PTEs. NVMe queues are leased outside spin locks; callers can borrow an idle
queue or sleep for admission. DMA transport still uses bounded staging buffers,
and writable disk pagers require a future writeback and crash-ordering design.

Kernel callers use `fs::mount_block`; syscall
clients use the `file_mount` entry with a source such as `/dev/nvme0n1p1` and
filesystem name `ext4`, `vfat`, or `fat32`.

All hardware modules use ABI generation 2. `rdf_module_entry` returns a module
descriptor, operation records are size-prefixed, and a module imports only the
`rdf_api_v2_*` services it actually calls. The loader resolves a closed export
list and exposes no internal Rust symbols. The C contract remains in
`drivers/include/roanix/`; in-tree Rust drivers use `drivers/ddk`.

The workspace targets `x86_64-unknown-none` and `riscv64gc-unknown-none-elf`,
uses `panic = "abort"`, and links self-contained ET_DYN images. For RISC-V
modules that use `alloc`, xtool rebuilds `core`, `alloc`, and
`compiler_builtins` as position-independent code.

Handles for framework objects are direct opaque pointers, so a call does not
perform a global ID lookup. Registration APIs return unique receipts; borrowed
bus/device handles cannot tear down registrations. Once MMIO is mapped, normal
register access is inline and makes no framework call.

xtool fingerprints `drivers/`, rebuilds changed modules, and installs them
under `/usr/lib/roanix/drivers/` in the staged sysroot:

```bash
$ vim drivers/storage/nvme/src/lib.rs
$ ./x.py                # rebuilds the module and boots the result
```

Cargo intermediates live under `build/cargo/drivers`, and finished modules live
under `build/<arch>/drivers-rust`.

## Notes on the sysroot

The userland sysroot is dynamically linked against `/usr/lib/ld.so`. Its mlibc
port uses the Roanix syscall ABI. The initramfs is deliberately only a bootstrap:
it contains coreutils, Bash, mlibc, util-roanix, their required runtime
dependencies, and the architecture's hardware modules. Drivers needed to reach
the root NVMe device must remain there.

The complete sysroot is instead materialized directly into a GPT-partitioned
ext4 image at `build/<arch>/out/<profile>/roanix-<arch>.root.img`. `mke2fs`
populates the partition at its final byte offset, preserving symlinks, modes,
and case-sensitive names without an expanded staging tree or a second image
copy. The image uses 4 KiB blocks, extents, typed directories, and no journal.
QEMU attaches it as an NVMe disk using an ephemeral snapshot, keeping the build
artifact immutable.

The bootstrap `/sbin/init` waits for `/dev/nvme0n1p1`, asks the kernel to mount
it read-only on `/`, and re-executes `/sbin/init` from the new root. The kernel
accepts a root replacement only from PID 1 and only when the filesystem contains
`/etc/roanix-root` plus the required system directories. Existing devfs is then
reattached at `/dev`, a fresh tmpfs is mounted at `/tmp`, and the process working
directory is reset. If discovery or validation fails, init retains the
bootstrap root and provides an emergency shell.

For faster iteration, `dev` builds compress the initramfs at gzip level 1 while
`release` builds use level 9.
