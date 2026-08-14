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
changed, reinstalls the sysroot, repacks the initramfs, refreshes the disk
image, and boots the result. Anything already up to date is skipped, so a
no-op run finishes in well under a second.

## Dependencies

Because the kernel selects its toolchain through `rust-toolchain.toml`, an
installation of [rustup] is required. Disk image creation needs `xorriso`,
`mtools`, and `sgdisk`; Python 3.11 or newer is required to run xtool itself;
and QEMU is used for testing. Building the userland additionally needs the Jinx
host dependencies: Bash, awk, findutils, Git, GNU Make, grep, gzip, sed, tar,
zstd, coreutils, procps, and util-linux. Jinx normally uses `wget`; xtool also
accepts `curl` and provides a shim.

[rustup]: https://rustup.rs/

**Ubuntu/Debian:**

```bash
$ sudo apt-get install make python3 xorriso mtools gdisk zstd qemu-system-x86
```

**Alpine:**

```bash
$ sudo apk add make python3 xorriso mtools sgdisk zstd qemu qemu-system-x86_64
```

**Arch:**

```bash
$ sudo pacman -S make python xorriso mtools gptfdisk zstd qemu-system-x86
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

# Pack that sysroot as build/<arch>/<profile>/roanix-<arch>.initramfs.tar.gz
$ ./x.py build initramfs

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

You usually do not even need `pkg`: editing a recipe (or the in-tree sources
under `drivers/` and `userland/init/`) marks that package stale, so a plain
`./x.py` rebuilds it, reinstalls the sysroot, and boots.

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
    ├── out/dev/             roanix, *.initramfs.tar.gz, *.hdd, *.iso
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

## Driver modules and DevKit

Native drivers are freestanding C shared objects built from `drivers/`. Each
module links the static DevKit runtime built from `drivers/devkit/`, includes
`<devkit/devkit.h>`, and exports one `dk_driver_definition`.

DevKit negotiates versioned kernel service tables during module startup.
Drivers publish and acquire typed resources through leases; resource lookup is
kept on the control path while acquired protocols use cached operation tables
for direct calls. The kernel retains ownership of device topology, MMIO and
interrupt authorization, resource lifetimes, TTY/devtmpfs frontends, and driver
unload ordering.

Modules may also register declarative `dk_driver_class` records. Classes match
provider nodes by kind, exact properties, and required inherited resources;
higher-priority matches bind first, and `DK_EDEFER` retries binding after the
provider tree changes. Provider nodes may contain buses or devices, allowing
layered stacks such as PCI function → NVMe controller and USB interface → HID.

The `drivers` userland package builds DevKit automatically and links it into
every packaged module. xtool mirrors `drivers/` into the Jinx workspace and
treats it as an input to that package, so editing a driver is enough to make
`./x.py` rebuild and reinstall it:

```bash
$ vim drivers/char/pty.c
$ ./x.py                # rebuilds the drivers package and boots the result
```

Do not maintain a second generated driver tree; always build through `x.py`.

## Notes on the sysroot

The userland sysroot is dynamically linked against `/usr/lib/ld.so`. Its mlibc
port uses the Roanix syscall ABI; the kernel imports the initramfs and starts
`/sbin/init` through that ABI. HDD and ISO builds automatically build or reuse
the sysroot, repack it, and attach it as the Limine initramfs module.

For faster iteration, `dev` builds compress the initramfs at gzip level 1 while
`release` builds use level 9.
