# Building and running Roanix

`x.py` has three commands: `run`, `build`, and `rebuild`. Run it from the
repository root. With no arguments, it builds a bootable HDD image and starts
QEMU.

## Requirements

Use a Linux host with Python 3.12 or newer, rustup, a C compiler, GNU Make,
Git, Bash, wget, awk, findutils, grep, gzip, sed, tar, zstd, coreutils, procps,
and util-linux. [Jinx] uses unprivileged user namespaces to build packages in
containers; run `x.py` as your normal user.

Images also require `sgdisk` (gdisk/gptfdisk), `mtools`, and `e2fsprogs`.
ISO creation requires `xorriso`. Install `qemu-system-x86_64` and/or
`qemu-system-riscv64` for the architecture you want to run; distribution
package names vary.

Cargo installs the pinned nightly toolchains and targets from the kernel and
driver `rust-toolchain.toml` files. The driver toolchain also requests
`rust-src` for RISC-V's position-independent standard-library build. `x.py`
downloads pinned Jinx, Limine, and UEFI firmware into `build/cache/`; bootloader
and firmware archives are checked against their SHA256 hashes.

[Jinx]: https://github.com/Mintsuki/Jinx

## Build an image and boot it

```bash
./x.py                         # build changed parts, then boot the HDD image
./x.py run iso                 # build and boot an ISO
./x.py build                   # build an HDD image without starting QEMU
./x.py build iso -a riscv64    # build a RISC-V ISO
./x.py run -r                  # build and boot a release kernel
```

Each image build:

1. Invokes Cargo for the kernel and the architecture's hardware drivers.
   Cargo recompiles only what changed; drivers always use the release profile.
2. Asks Jinx to download missing userland packages and dependencies from the
   binary repository configured in `userland/Jinxfile`. Existing local XBPS
   packages, including your rebuilt packages, are reused. If packages are
   unavailable, it tries the host-tool repository and builds the missing
   packages locally.
3. Installs the packages and drivers into `build/<arch>/sysroot`, packs a
   bootstrap initramfs, and creates the ext4 root disk and selected boot image.
   Assembly steps are skipped when their inputs have not changed.
4. For `run`, starts QEMU with the boot image and the root disk attached as an
   NVMe device. The guest CPU count defaults to `nproc`, independently of the
   number of build jobs.

The HDD or ISO contains Limine, the kernel, and the bootstrap initramfs. The
complete userland lives in the companion `roanix-<arch>.root.img`. **Keep both
images together** when running outside `x.py`. Init mounts the ext4 root from
`/dev/nvme0n1p1`; QEMU uses a disposable snapshot of that disk. The filesystem
is formatted with the feature subset supported by Roanix's read-only ext4
driver.

`SYSTEM_PACKAGES` in `x.py` selects the top-level packages in the system.
Jinx resolves their dependencies. `BOOTSTRAP_PACKAGES` selects the small early
boot environment. Add a recipe under `userland/recipes/` and add its name to
`SYSTEM_PACKAGES` when it should be included in the image.

## Develop the kernel or drivers

Edit the Rust sources and run `./x.py` again. Cargo checks changes on every
invocation, including changes to dependencies, build scripts, and flags.
To compile without assembling images:

```bash
./x.py build kernel
./x.py build drivers -a riscv64
./x.py build userland           # packages, drivers, and installed sysroot
```

Kernel-only changes refresh the boot image while reusing the userland and
root disk. Driver changes also refresh the installed modules and initramfs.
See [the driver guide](../../../drivers/README.md) for module layout and APIs.

## Develop a Jinx package

Image builds consider a package current when Jinx has an XBPS file matching
its recipe version and revision. Source edits, patch edits, and recipe edits
that keep that version and revision need an explicit package build. This
also applies to the local C programs under `userland/util-roanix/`.

To get working sources for a package that was downloaded as a binary:

```bash
./x.py build bash
```

This runs a local Jinx build even if the binary package already exists.
Edit `userland/sources/bash-workdir/`, then:

```bash
./x.py rebuild bash
./x.py                         # install the new package, refresh images, boot
```

`rebuild` asks Jinx to save the working source changes as
`userland/recipes/bash/patches/jinx-working-patch.patch`, runs preparation,
and rebuilds the package from a fresh build directory. It generates and
indexes the new XBPS package in `build/<arch>/jinx/pkgs/`. It does not bump
recipe revisions or rebuild reverse dependencies automatically.

To verify that your patch works on freshly downloaded sources:

```bash
./x.py rebuild bash -f
```

The patch is saved **before** the downloaded source, clean copy, working
copy, and Jinx source markers are removed. Jinx then downloads the sources
again and applies the saved patches. A failed patch-generation step stops
before deletion. If no working copy exists yet, `rebuild` builds from the
recipe and its already saved patches.

Other package types use the same commands:

```bash
./x.py rebuild util-roanix      # rebuild local repository sources in place
./x.py rebuild mlibc-headers    # regenerate mlibc's patch, rebuild its headers
./x.py rebuild host:binutils    # host-sources/binutils-workdir; host-pkgs output
```

Shared-source recipes regenerate their source owner's patch. Local
`source_dir` recipes such as `util-roanix` have no generated working patch;
commit those source edits directly. `-f` never deletes local `source_dir`
trees. Host packages are build tools and are not installed into the guest.

After an ABI change, rebuild affected consumers explicitly or bump the
appropriate recipe revisions using Jinx. Commit patches and recipe changes;
downloaded working directories and package archives are ignored by Git.

## Options and QEMU

| Option | Meaning |
| --- | --- |
| `-a`, `--arch` | `x86_64` (default) or `riscv64` |
| `-r`, `--release` | Use a release kernel instead of the default `dev` profile |
| `-j`, `--jobs` | Cargo and Jinx build jobs; defaults to `nproc` |
| `-v`, `--verbose` | Print subprocess commands |
| `rebuild -f` | Save the patch and refetch downloaded sources |
| `run --bios` | Use x86_64 BIOS instead of the default UEFI |
| `run -- …` | Append raw QEMU arguments |

Options can appear before or after the command. `ROANIX_ARCH` sets a default
architecture; `-a` overrides it. Set `ROANIX_BINARY_PACKAGES=0` to skip cloud
packages, or `ROANIX_PACKAGE_REPO` to use another repository. Disabling binary
downloads still reuses existing local packages and allows source downloads.
`ROANIX_RUSTFLAGS` appends kernel compiler flags.

```bash
./x.py run -- -display none                # serial console and QEMU monitor
./x.py run -- -accel tcg -smp 4 -m 2G     # override emulator defaults
./x.py run -- -s -S                       # GDB server, halted at reset
```

For GDB, connect from another terminal:

```bash
rust-gdb build/x86_64/out/dev/roanix -ex 'target remote :1234'
```

QEMU uses KVM for x86_64 when the host provides access to `/dev/kvm`;
otherwise it uses software emulation. RISC-V uses the `virt` machine with
UEFI, a framebuffer, USB input, and NVMe storage. On the serial terminal,
Ctrl-A X exits QEMU; Ctrl-A C switches to its monitor.

## Layout and direct tools

```text
build/
├── cache/                   pinned Jinx, Limine, firmware, and downloads
├── cargo/                   Cargo kernel outputs, namespaced by target/profile
│   └── drivers/             driver workspace Cargo outputs
└── <arch>/
    ├── jinx/                package builds, pkgs/, host-pkgs/, and host tools
    ├── drivers-rust/        finished .ko modules
    ├── sysroot/             installed packages, modules, and public headers
    └── out/<profile>/       kernel, initramfs, ext4 root disk, HDD/ISO
```

Small `.stamp` files beside assembled outputs track their inputs. Existing
Jinx packages and downloaded sources from older builds are reused; the old
`build/config.json` and `state/` files are no longer used. Architectures keep
separate packages and images, but Jinx's source trees and container cache are
shared. Run package builds/rebuilds one at a time across architectures.

Use the underlying tools for less common work:

```bash
(cd build/x86_64/jinx && ../../cache/jinx/jinx dry-run '*')
(cd build/x86_64/jinx && ../../cache/jinx/jinx run-in bash bash)
(cd build/x86_64/jinx && ../../cache/jinx/jinx revbump mlibc)
(cd kernel && cargo fmt --check)
mdbook serve book
python3 -m unittest discover -s tests
```

To discard generated images, remove `build/<arch>/out/`. Remove that
architecture's `sysroot/` to reinstall its packages. Removing `build/<arch>/`
also discards its package builds. Save patches with `rebuild` before removing
anything under `userland/sources/` or `userland/host-sources/`.
