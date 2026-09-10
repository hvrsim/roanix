# Roanix

![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/hvrsim/roanix/ci)
![License BSD+Patent](https://img.shields.io/badge/license-BSD%2BPatent-blue)

## Overview

**Roanix** is an experimental operating system written in Rust, targeting the x86_64 PC platform and various RISC-V boards. With a focus on POSIX compatibility, Roanix aims to run existing userland programs, such as GNU Bash and Python 3.

## Key Goals

* Advanced memory management system featuring demand paging and swapping with [LZ4 compression].
* Clean, extensible driver stack with support for modern hardware including [NVMe] devices.
* High-performance thread scheduler, based on [FreeBSD ULE].
* Built from the ground up using [Rust], ensuring memory safety and thread safety without sacrificing performance.
* First class support for both the [x86_64] and [riscv64] CPU architectures. Support also extends to modern CPU features (Intel CET, RVV 1.0, and more).

[Rust]: https://www.rust-lang.org/
[x86_64]: https://en.wikipedia.org/wiki/X86-64
[riscv64]: https://en.wikipedia.org/wiki/RISC-V
[NVMe]: https://en.wikipedia.org/wiki/NVM_Express
[FreeBSD ULE]: https://web.cs.ucdavis.edu/~roper/ecs150/ULE.pdf
[LZ4 compression]: https://lz4.org/

## Building Roanix

Roanix is built with `x.py`, a single-file build tool in the project root:

```bash
./x.py                  # build changed parts and boot in QEMU
./x.py build iso        # build an ISO and its ext4 root disk
./x.py rebuild bash -f  # save source edits as a patch, refetch, and rebuild Bash
```

For detailed build instructions, see the *Roanix Internals* book under
[Building Roanix](book/src/guide/BUILDING.md).

Builds require Linux. Image builds use cached binary packages where available;
Cargo detects kernel and driver changes, and `rebuild` handles userland edits.

## Contributing

Contributions are always welcome! If you're interested in contributing, please review our [contributing guidelines](book/src/guide/CONTRIB.md).

## License

Roanix is released under the [BSD+Patent License](https://opensource.org/license/BSDplusPatent).
