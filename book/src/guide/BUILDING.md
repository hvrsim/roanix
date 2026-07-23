# Building & Running Roanix

This chapter serves as a quick guide on how to Roanix from source and run it in an emulator. A quick note before starting, currently only the Linux platform is supported as a build host for Roanix. Other platforms will either not work (Windows/non UNIX-like) or are untested (*BSD, MacOS).

**NOTE: Roanix currently uses a custom build system called xtool, which is a single file python script in the project root (`x.py`)**

## Dependencies

Due to the use of `rust-toolchain.toml` for selecting rust toolchains, an installation of `rustup` is required. For instructions on how to install rustup, see [here](https://rustup.rs/).

Packages such as `xorriso`, `mtools` and `sgdisk` are also required for disk image creation. Additionally, Python 3.11 or newer is required to run xtool, and QEMU is preferred for testing Roanix. Building the userspace sysroot requires the Jinx host dependencies: Bash, awk, findutils, Git, GNU Make, grep, gzip, sed, tar, zstd, coreutils, procps, and util-linux. Jinx normally uses wget; xtool also supports curl as a fallback. Commands to install the core packages have been provided below for multiple Linux distros.

**Ubuntu:**

```bash
$ sudo apt-get install make python3 xorriso mtools gdisk qemu-system-x86_64
```

**Alpine:**

```bash
$ sudo apk add make python3 xorriso mtools sgdisk qemu qemu-system-x86_64
```

**Arch:**

```bash
$ sudo pacman -S make python xorriso mtools gptfdisk qemu-system-x86_64
```

## Configuring the build

The xtool build system uses command line flags for configuration:

- `--arch`: CPU architecture (`x86_64` or `riscv64`) to build Roanix for.
- `--profile`: Cargo profile (`dev` or `release`).
- `--force`: Rebuild components even when they are up to date.

Use `python3 x.py help` or `python3 x.py help <command>` to view the available commands and options. `python3 x.py doctor` checks the host dependencies required by the default build.

*For example, to build a riscv64 ISO image with release kernel:*
```bash
$ python3 x.py build iso --arch riscv64 --profile release
```

## Build Targets

Commands to build the kernel and disk images are provided below:

```bash
# Build HDD image with kernel, userspace, and bootloader
$ python3 x.py

# Build ISO image with kernel, userspace, and bootloader
$ python3 x.py build iso

# Build kernel only
$ python3 x.py build

# Build mlibc, libgcc, libatomic, libstdc++, ncurses, readline, Bash,
# coreutils, Python, and init, then install them into
# build/runtime/sysroots/<architecture>.
$ python3 x.py build sysroot --arch x86_64

# Rebuild one package (and affected dependents) and replace the existing sysroot.
$ python3 x.py package init --arch x86_64

# Pack that sysroot as build/<architecture>/<profile>/roanix-<architecture>.initramfs.tar.gz.
$ python3 x.py build initramfs --arch x86_64

# Clean up build directories while retaining downloaded runtime assets
$ python3 x.py clean

# Clean up build directories and cached files.
$ python3 x.py clean --all
```

Kernel, userspace, initramfs, and image builds are incremental. xtool stores build state and downloaded assets under `build/runtime/`; pass `--force` to `build` or `run` to rebuild the complete dependency chain.

The userspace sysroot is dynamically linked with `/usr/lib/ld.so`. Its mlibc
port uses the minimal Roanix syscall ABI needed by the loader and a hello-world
program; the kernel imports the initramfs and starts `/sbin/init` through that
ABI.
HDD and ISO builds automatically build or reuse the userspace sysroot, repack
it, and attach it as the Limine initramfs module.

## Driver modules and DevKit

Native drivers are freestanding C shared objects built from `drivers/`. Each
module links the static DevKit runtime built from `drivers/devkit/`, includes
`<devkit/devkit.h>`, and exports one `dk_driver_definition`.

DevKit negotiates versioned kernel service tables during module startup.
Drivers publish and acquire typed resources through leases; resource lookup is
kept on the control path while acquired protocols use cached operation tables
for direct calls. The kernel retains ownership of device topology, MMIO and
interrupt authorization, resource lifetimes, TTY/devtmpfs frontends, and
driver unload ordering.

Modules may also register declarative `dk_driver_class` records. Classes match
provider nodes by kind, exact properties, and required inherited resources;
higher-priority matches bind first, and `DK_EDEFER` retries binding after the
provider tree changes. Provider nodes may contain buses or devices, allowing
layered stacks such as PCI function → NVMe controller and USB interface → HID.

The `drivers` userspace package builds DevKit automatically and links it into
every packaged module. xtool copies the source tree into the Jinx workspace, so
driver builds should be invoked through `x.py` rather than by maintaining a
second generated driver tree.

## Running Roanix

Currently, xtool only supports running Roanix in the QEMU emulator. Using other emulators such as [VMWare](https://vmware.com) or [Simics](https://www.intel.com/content/www/us/en/developer/articles/tool/simics-simulator.html) is possible, but are not supported within xtool.

### QEMU

[QEMU](https://qemu.org) (Quick Emulator) is an open-source program that emulates hardware platforms and CPUs. It also supports native virtualization through the Linux KVM and Apple HVF frameworks.

Run Roanix in QEMU with the xtool `run` command:

```bash
# Run on QEMU with x86_64 BIOS enviorment
$ python3 x.py run hdd --firmware bios

# Run with QEMU with riscv64 UEFI enviorment
$ python3 x.py run --arch riscv64

# Run with QEMU with riscv64 UEFI enviorment (ISO image)
$ python3 x.py run iso --arch riscv64

# Pass extra raw QEMU args after `--`
$ python3 x.py run -- --no-reboot
```
