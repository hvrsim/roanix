# Build and run Roanix

Use a Linux host with Python 3.12 or newer and Rust installed through rustup.
Run the commands in this guide from the repository root.

## Try Roanix in QEMU

```sh
./x.py doctor run
./x.py run
```

Install any tools that `doctor` reports as missing. The first run downloads
build inputs and may compile packages that are unavailable as binaries. When
QEMU starts, wait for the Roanix shell. After a code change, run `./x.py run`
again. It reuses build outputs that are still current.

## Build a bootable image

```sh
./x.py doctor image
./x.py image
```

The default build produces a standalone raw disk for x86_64 PCs in
`build/dist/`. It includes a read-only system partition. On a physical PC, you
need UEFI or legacy BIOS and a storage controller that Roanix supports.
Use `./x.py image --format iso` to build a live ISO that loads the system into
memory. For an ISO build, check the host with `./x.py doctor image --format iso`.

Each image comes with a SHA-256 checksum and a JSON build manifest. Pass the
image path to `x.py run` to boot it in QEMU.

```sh
./x.py run --image build/dist/roanix-pc-x86_64-standard-release.raw
```

Use `test boot --image` with the same path to run a headless boot test. The test
runs a command in the guest shell and saves the serial log in
`build/work/<arch>/logs/`.

## Work on the kernel or drivers

Run `./x.py check kernel` to check your kernel changes or
`./x.py build kernel` to produce the kernel binary. Use `check driver nvme`
or `build driver nvme` to work on a single driver. For a debugging session,
`./x.py run --debug` pauses QEMU and prints the path to the kernel symbols
and the GDB connection command.

Both `run` and `image` accept `--machine qemu-riscv64-virt` to target the
RISC-V QEMU machine and `--profile minimal` to use a smaller package set.
`run` defaults to the development kernel, and `image` defaults to the release
kernel. Select either with `--mode dev` or `--mode release`.

## Edit a userland package

1. Run `./x.py package edit bash` and make your changes in the worktree it
   prints. The first call may build the package to prepare its sources.
2. Run `./x.py run` to test the change in QEMU, or use
   `./x.py build package bash` to build the XBPS archive on its own.
3. Run `./x.py package export-patch bash` when you're ready to save the changes
   as a patch in the recipe directory.

For local packages such as `util-roanix`, edit the tracked source directly.
Prefix host-tool recipes with `host:`, as in
`./x.py build package host:binutils`. Package build logs are in
`build/work/<arch>/logs/`. Add `--verbose` to see the build output as it runs.

## Prepare for an offline build

```sh
./x.py fetch run
./x.py run --offline
```

Use the same machine and profile options for both commands. Use `fetch image`
when you plan to build portable media. During prefetch, `x.py` checks the
version and hash of downloaded XBPS packages against the repository index.
Jinx can build packages that are missing from the feed. To skip binary
downloads and reuse any packages already staged locally, add
`--packages source`.

## Reclaim disk space

Run `./x.py disk-usage` to see where the build uses space. `./x.py clean`
removes rebuildable work for the selected machine and keeps your downloads,
staged packages, editable worktrees, and finished images. Preview cache
cleanup with `./x.py cache prune --dry-run`.

Run `./x.py --help` for the full command list.
