# Building & Running Roanix

This chapter serves as a quick guide on how to Roanix from source and run it in an emulator. A quick note before starting, currently only the Linux platform is supported as a build host for Roanix. Other platforms will either not work (Windows/non UNIX-like) or are untested (*BSD, MacOS).

**NOTE: Roanix currently uses a custom build system called xtool, which is a single file python script in the project root (`x.py`)**

## Dependencies

Due to the use of `rust-toolchain.toml` for selecting rust toolchains, an installation of `rustup` is required. For instructions on how to install rustup, see [here](https://rustup.rs/).

Packages such as `xorriso`, `mtools` and `sgdisk` are also required for disk image creation. Additionally, `python3` is required to run xtool, and QEMU is preferred for testing Roanix. Commands to install these packages have been provided below for multiple linux distros.

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

### NixOS

A NixOS flake is present in the project root, which contains all the dependencies needed to compile Roanix. Use of this flake is **highly recommended** on NixOS.

## Configuring the build

The xtool build system uses command line flags for configuration:

- `--arch`: CPU architecture to build Roanix for.
- `--rust-profile`: Cargo profile (`release`, `dev`, etc).
- `--rust-target`: Override Rust target triple.
- `--qemu-flags`: Extra flags to pass to QEMU.

*For example, to build a riscv64 ISO image with release kernel:*
```bash
$ python3 x.py --arch riscv64 --rust-profile release gen-iso
```

## Build Targets

Commands to build the kernel and disk images are provided below:

```bash
# Build HDD image with kernel and bootloader
$ python3 x.py

# Build ISO image with kernel and bootloader
$ python3 x.py gen-iso

# Build kernel only
$ python3 x.py build

# Clean up build directories
$ python3 x.py clean

# Clean up build directories and cached files.
$ python3 x.py distclean
```

## Running Roanix

Currently, xtool only supports running Roanix in the QEMU emulator. Using other emulators such as [VMWare](https://vmware.com) or [Simics](https://www.intel.com/content/www/us/en/developer/articles/tool/simics-simulator.html) is possible, but are not supported within xtool.

### QEMU

[QEMU](https://qemu.org) (Quick Emulator) is an open-source program that emulates hardware platforms and CPUs. It also supports native virtualization through the Linux KVM and Apple HVF frameworks.

Run Roanix in QEMU with the `run-*` family of xtool commands:

```bash
# Run on QEMU with x86_64 BIOS enviorment
$ python3 x.py run-bios

# Like above but enable KVM acceleration
$ python3 x.py --qemu-flags "--enable-kvm" run-bios

# Run with QEMU with riscv64 UEFI enviorment
$ python3 x.py run-riscv64

# Run with QEMU with riscv64 UEFI enviorment (ISO image)
$ python3 x.py run-iso-riscv64

# Pass extra raw QEMU args after `--`
$ python3 x.py run -- --enable-kvm
```
