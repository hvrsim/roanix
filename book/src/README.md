# Introduction

**Roanix** is an experimental operating system written in Rust, targeting the x86_64 PC platform and various RISC-V boards. With a focus on POSIX compatibility, Roanix aims to run existing userland programs, such as GNU Bash and Python 3.

> The name "Roanix" combines "Roan" and "UNIX". Roan refers to Roan Mountain, located on the North Carolina/Tennessee border in the United States, which serves as the namesake for this project.

## Key Goals

* Advanced memory management system featuring demand paging and swapping with [LZ4 compression].
* Clean, extensible driver stack with support for modern hardware including [NVMe] devices.
* High-performance thread scheduler, based on [FreeBSD ULE].
* Built from the ground up using [Rust], ensuring memory safety and thread safety without sacrificing performance.
* First class support for both the [x86_64] and [riscv64] CPU architectures. Support also extends to modern CPU features (Intel MPK, RVV 1.0, and more).

[Rust]: https://www.rust-lang.org/
[x86_64]: https://en.wikipedia.org/wiki/X86-64
[riscv64]: https://en.wikipedia.org/wiki/RISC-V
[NVMe]: https://en.wikipedia.org/wiki/NVM_Express
[FreeBSD ULE]: https://web.cs.ucdavis.edu/~roper/ecs150/ULE.pdf
[LZ4 compression]: https://lz4.org/

## Contributing

For information about contributing to Roanix, check out our  [Contributing Guide](guide/CONTRIB.md).

## License

Roanix is released under the [BSD+Patent License](https://opensource.org/license/BSDplusPatent).