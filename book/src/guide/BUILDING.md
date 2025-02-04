# Building & Running Roanix

This chapter explains how to build Roanix from source and run it in a emulator (or on real hardware). The only supported host platform for compiling Roanix is Linux. Other platforms will either not work (Windows/non UNIX-like) or are untested (*BSD, MacOS).

## Dependencies

Due to the use of `rust-toolchain.toml` for selecting rust toolchains, an installation of `rustup` is required. For instructions on how to install rustup, see [here](https://rustup.rs/).

Packages such as `xorriso`, `mtools` and `sgdisk` are also required for image generation. For testing Roanix, the recommended emulator is QEMU (more on it in a later section). Commands to install these packages have been provided below for multiple linux distros...

**Ubuntu:**

```bash
$ sudo apt-get install xorriso mtools gdisk make qemu-system-x86_64
```

**Alpine:**

```bash
$ sudo apk add make xorriso mtools sgdisk qemu qemu-system-x86_64
```

**Arch:**

```bash
$ sudo pacman -S make xorriso mtools sgdisk qemu-system-x86_64
```

### NixOS

A NixOS flake is present in the project root, which contains all the dependencies needed to compile Roanix. Use of this flake is **highly recommended** on NixOS.

## Configuring the build

The Makefile build system also accepts various enviorment variables for configuring the build. These variables all have sane defaults, and I have described them below: which I have described below:

- `KARCH`: CPU architecture to build Roanix for.
- `RUST_PROFILE`: Build profile for cargo (release, dev, etc)
- `IMAGE_NAME`: Basename of ISO/HDD image.
- `QEMU_FLAGS`: Extra flags to pass to the QEMU emulator.

*For example, to build a riscv64 ISO image with release kernel:*
```bash
$ KARCH=riscv64 RUST_PROFILE=release make all-iso
```

## Build Targets

The root Makefile provides the build-interface for this project. I have described all targets below, except for the run targets which are covered in a later chapter.

*NOTE: I recommend you read `GNUmakefile` in the project root for all the possible targets.*

Build hard drive image with kernel and bootloader (for virtualization).

```bash
# Build HDD image with kernel and bootloader
$ make

# Build ISO image with kernel and bootloader
$ make all-iso

# Clean up build directories
$ make clean

# Clean up build directories and cached files.
$ make distclean
```

## Running Roanix

From emulators to real hardware, there are tons of platforms on which Ronaix can operate. I have decided to cover the 2 most common platforms in sections below.

### QEMU

[QEMU](https://qemu.org) (Quick Emulator) is an open-source program that emulates hardware and CPUs. It can be used to run operating systems and applications on a different machine than the one they were originally designed for.

QEMU is used as a the default hypervisor for testing Roanix. Other hyperviors such as [VMWare](https://vmware.com) or [Simics](https://www.intel.com/content/www/us/en/developer/articles/tool/simics-simulator.html) are capable of running Roanix, but are not first-class citizens.

Run Roanix in QEMU with the `run-*` family of make targets:

```bash
# Run on QEMU with x86_64 BIOS enviorment
$ make run-bios

# Like above but enable KVM acceleration
$ QEMU_FLAGS="--enable-kvm" make run-bios

# Run with QEMU with <arch> UEFI enviorment
$ make run-<arch>

# Run with QEMU with <arch> UEFI enviorment (ISO image)
$ make run-iso-<arch>
```