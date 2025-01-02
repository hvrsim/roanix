# Building Roanix

This chapter explains how to build Roanix from source. The only supported host platform for compiling Roanix is Linux. Other platforms will either not work (Windows/non UNIX-like) or are untested (*BSD, MacOS).

## Dependencies

Due to the use of `rust-toolchain.toml` for selecting rust toolchains, an installation of `rustup` is required. For instructions on how to install rustup, see [here](https://rustup.rs/).

Packages such as `xorriso`, `mtools` and `sgdisk` are also required for image generation. Commands to install these packages have been provided below for multiple linux distros...

**Ubuntu:**

```bash
$ sudo apt-get install xorriso mtools gdisk make
```

**Alpine:**

```bash
$ sudo apk add make xorriso mtools sgdisk
```

**Arch:**

```bash
$ sudo pacman -S make xorriso mtools sgdisk
```

### NixOS

A NixOS flake is present in the project root, which contains all the dependencies needed to compile Roanix. Use of this flake is **highly recommended** on NixOS.

## Build Targets

The root Makefile provides the build-interface for this project. I have described all targets below, except for the run targets which are covered in a later chapter.

Build hard drive image with kernel and bootloader (for virtualization).

```bash
$ make
```

Build ISO image with kernel and bootloader (for booting on real hardware).

```bash
$ make all-iso
```

Clean up build assets, but don't remove cached files.

```bash
$ make clean
```

Clean everything including cached files.

```bash
$ make distclean
```

## Configuring the build

The root Makefile also accepts 3 enviorment variables for configuring the build, which I have described below:

- `KARCH`: CPU architecture to build Roanix for.
- `IMAGE_NAME`: Basename of ISO/HDD image.
- `QEMU_FLAGS`: Extra flags to pass to the QEMU emulator.

For example, to build a riscv64 ISO image:

```bash
$ KARCH=riscv64 make all-iso
```