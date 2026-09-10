#!/usr/bin/env python3
"""Build & Run the Roanix Operating System"""

import argparse
import hashlib
import json
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path


# Project layout and package selection.

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"
CACHE = BUILD / "cache"
USERLAND = ROOT / "userland"

SYSTEM_PACKAGES = (
    "linux-headers",
    "bash",
    "coreutils",
    "python",
    "stress-ng",
    "util-roanix",
)
BOOTSTRAP_PACKAGES = ("coreutils", "bash", "mlibc", "util-roanix")

TARGETS = {
    "x86_64": "x86_64-unknown-none",
    "riscv64": "riscv64gc-unknown-none-elf",
}
DRIVERS = {
    "x86_64": ("acpi", "ioapic", "pci", "nvme", "uart8250"),
    "riscv64": ("fdt", "plic", "pci", "nvme", "uart8250"),
}

# Pinned build tools and firmware.

JINX_COMMIT = "287ceaf9a2c08b43dbc56d38d8b815fd990a2192"
# name: (version, archive URL, SHA256, directory inside archive)
DOWNLOADS = {
    "limine": (
        "12.5.0",
        "https://github.com/limine-bootloader/limine/releases/download/v12.5.0/limine-binary.tar.gz",
        "8bc0d0f2a2cd0e212f529c57d8a2033a25996dcdd9f58c916b7bf5594a0282eb",
        "limine-binary",
    ),
    "ovmf": (
        "20260531T041444Z",
        "https://github.com/osdev0/edk2-ovmf-nightly/releases/download/20260531T041444Z/edk2-ovmf.tar.gz",
        "534384e4b971730143f54708493fe43932eabbfd275094b636561fb2028bde62",
        "edk2-ovmf",
    ),
}

VERBOSE = False
MIB = 1024 * 1024


# Shared command, file, and cache helpers.


def msg(message, *, detail=False, error=False):
    if error:
        prefix, color = "==> ERROR:", "31"
    elif detail:
        prefix, color = "  ->", "34"
    else:
        prefix, color = "==>", "32"

    stream = sys.stderr if error else sys.stdout
    if stream.isatty() and "NO_COLOR" not in os.environ:
        prefix = f"\033[1;{color}m{prefix}\033[0m"
    print(f"{prefix} {message}", file=stream, flush=True)


def run(*args, cwd=ROOT, env=None, capture=False, check=True):
    args = [str(arg) for arg in args]
    if VERBOSE:
        msg(shlex.join(args), detail=True)

    result = subprocess.run(
        args,
        cwd=cwd,
        env=os.environ | (env or {}),
        text=True,
        stdout=subprocess.PIPE if capture else None,
        check=False,
    )
    if check and result.returncode:
        raise RuntimeError(f"command failed ({result.returncode}): {shlex.join(args)}")
    return result


def remove(path):
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    else:
        path.unlink(missing_ok=True)


def copy(source, destination):
    """Leave unchanged files alone so copying does not invalidate image caches."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.is_file() or sha256(source) != sha256(destination):
        shutil.copy2(source, destination)


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def stamp(path):
    return path.with_name(f".{path.name}.stamp")


def assemble(output, inputs, values, action):
    """Cache only assembly steps; publish a new output after a successful build."""
    state = []
    for path in [ROOT / "x.py", *inputs]:
        stat = path.lstat()
        state.append((
            str(path),
            stat.st_ino,
            stat.st_mode,
            stat.st_size,
            stat.st_mtime_ns,
            stat.st_ctime_ns,
        ))

    encoded_state = json.dumps([state, values], sort_keys=True).encode()
    digest = hashlib.sha256(encoded_state).hexdigest()
    marker = stamp(output)
    if output.exists() and marker.is_file() and marker.read_text() == digest:
        msg(f"{output.relative_to(ROOT)} is up to date", detail=True)
        return

    msg(f"Assembling {output.relative_to(ROOT)}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as scratch:
        temporary = Path(scratch) / output.name
        action(temporary)

        if output.is_dir():
            # Keep the previous tree until its replacement has been assembled.
            output.rename(Path(scratch) / "previous")
        temporary.replace(output)
    marker.write_text(digest)


def resource(name):
    """Download and unpack a pinned tool or firmware archive when needed."""
    version, url, checksum, archive_root = DOWNLOADS[name]
    destination = CACHE / ("edk2-ovmf" if name == "ovmf" else name)
    marker = destination / ".sha256"
    if marker.is_file() and marker.read_text() == checksum:
        return destination

    downloads = CACHE / "downloads"
    downloads.mkdir(parents=True, exist_ok=True)
    archive = downloads / f"{name}-{version}-{checksum[:12]}.tar.gz"
    if not archive.is_file():
        msg(f"Downloading {name} {version}")
        partial = archive.with_suffix(".part")
        try:
            with urllib.request.urlopen(url, timeout=60) as response, partial.open("wb") as out:
                shutil.copyfileobj(response, out)
            partial.replace(archive)
        finally:
            partial.unlink(missing_ok=True)

    if sha256(archive) != checksum:
        raise RuntimeError(f"checksum mismatch: {archive}; remove it and retry")

    with tempfile.TemporaryDirectory(dir=CACHE) as scratch:
        with tarfile.open(archive) as tar:
            tar.extractall(scratch, filter="data")
        remove(destination)
        shutil.move(Path(scratch) / archive_root, destination)
    marker.write_text(checksum)
    return destination


# Recipe lookup. Jinx remains responsible for package dependencies and builds.


def recipe_path(package):
    host = package.startswith("host:")
    name = package.removeprefix("host:")
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9+_.-]*", name):
        raise ValueError(f"invalid package name: {package}")

    recipes = USERLAND / ("host-recipes" if host else "recipes")
    path = recipes / name / "recipe"
    if not path.is_file():
        raise ValueError(f"no recipe for {package}")
    return path


def source_recipe(package, arch):
    """Use the recipe's shell definitions to locate shared or local sources."""
    seen = set()
    while package not in seen:
        seen.add(package)
        recipe = recipe_path(package)
        script = '''
base_dir=$1
JINX_ARCH=$2
. "$base_dir/Jinxfile"
from_source=
from_host_source=
source_dir=
git_url=
tarball_url=
. "$3"
printf '%s\\n' "$from_source" "$from_host_source" "$source_dir" "$git_url$tarball_url"
'''
        result = run(
            "bash", "-e", "-c", script, "recipe", USERLAND, arch, recipe,
            capture=True,
        )
        shared, host_shared, local, remote = result.stdout.splitlines()

        if not shared and not host_shared:
            return package, local, bool(remote)

        # A host recipe's from_source stays in the host namespace.
        source_is_host = bool(host_shared) or package.startswith("host:")
        source_name = host_shared or shared
        package = f"host:{source_name}" if source_is_host else source_name

    raise ValueError(f"source recipe cycle involving {package}")


class Build:
    """Build packages and images for one architecture and kernel profile."""

    def __init__(self, args):
        self.args = args
        self.arch = args.arch
        self.target = TARGETS[self.arch]
        self.profile = "release" if args.release else "dev"

        self.base = BUILD / self.arch
        self.out = self.base / "out" / self.profile
        self.sysroot = self.base / "sysroot"
        self.jinx_dir = self.base / "jinx"

        self.kernel = self.out / "roanix"
        self.initramfs = self.out / f"roanix-{self.arch}.initramfs.tar.gz"
        self.root_disk = self.out / f"roanix-{self.arch}.root.img"
        self.modules = []
        for package in DRIVERS[self.arch]:
            filename = "storage-nvme.ko" if package == "nvme" else f"{package}.ko"
            self.modules.append(self.base / "drivers-rust" / filename)

        self.jinx_ready = False

    def jinx(self, *args, **kwargs):
        """Initialize the pinned Jinx checkout once, then run a Jinx command."""
        checkout = CACHE / "jinx"
        binary = checkout / "jinx"
        env = {"JINX_PARALLELISM": str(self.args.jobs), "JINX_CLEAN_WORKDIRS": "no"}

        if not self.jinx_ready:
            checkout.mkdir(parents=True, exist_ok=True)
            if not (checkout / ".git").exists():
                run("git", "init", "-q", checkout)
            result = run(
                "git", "rev-parse", "--verify", "--quiet", "HEAD",
                cwd=checkout,
                capture=True,
                check=False,
            )
            head = result.stdout.strip()
            if head != JINX_COMMIT:
                msg("Fetching Jinx")
                run(
                    "git", "fetch", "--depth=1",
                    "https://github.com/Mintsuki/Jinx.git", JINX_COMMIT,
                    cwd=checkout,
                )
                run("git", "checkout", "-q", "--detach", JINX_COMMIT, cwd=checkout)

            self.jinx_dir.mkdir(parents=True, exist_ok=True)
            if not (self.jinx_dir / ".jinx-parameters").exists():
                run(binary, "init", USERLAND, f"ARCH={self.arch}", cwd=self.jinx_dir, env=env)
            self.jinx_ready = True

        return run(binary, *args, cwd=self.jinx_dir, env=env, **kwargs)

    def download(self, *packages):
        binary_packages = os.environ.get("ROANIX_BINARY_PACKAGES", "1").lower()
        if binary_packages in ("0", "no", "false"):
            return

        result = self.jinx("download", *packages, check=False)
        if result.returncode:
            msg(
                "Binary packages unavailable; Jinx will build missing packages locally",
                detail=True,
            )

    def build_kernel(self):
        msg(f"Building kernel ({self.arch}, {self.profile})")
        extra_flags = os.environ.get("ROANIX_RUSTFLAGS", "")
        flags = f"-Crelocation-model=static -Cforce-frame-pointers=yes {extra_flags}"
        target_dir = BUILD / "cargo"

        run(
            "cargo", "build",
            "--target", self.target,
            "--profile", self.profile,
            "--jobs", self.args.jobs,
            "--target-dir", target_dir,
            cwd=ROOT / "kernel",
            env={"RUSTFLAGS": flags.strip()},
        )

        cargo_profile = "release" if self.args.release else "debug"
        copy(target_dir / self.target / cargo_profile / "roanix", self.kernel)

    def build_drivers(self):
        msg(f"Building drivers ({self.arch})")
        package_args = []
        for package in DRIVERS[self.arch]:
            package_args.extend(["--package", package])

        extra_flags = []
        if self.arch == "riscv64":
            extra_flags = [
                "-Zbuild-std=core,alloc,compiler_builtins",
                "-Zbuild-std-features=compiler-builtins-mem",
            ]

        target_dir = BUILD / "cargo" / "drivers"
        run(
            "cargo", "build", *package_args,
            "--target", self.target,
            "--release",
            "--jobs", self.args.jobs,
            "--target-dir", target_dir,
            *extra_flags,
            cwd=ROOT / "drivers",
        )

        for package, module in zip(DRIVERS[self.arch], self.modules):
            copy(target_dir / self.target / "release" / package, module)

    def install(self, root, packages, *, bootstrap=False):
        """Install packages and modules into a fresh root or bootstrap tree."""
        self.jinx("install", root, *packages)
        for module in self.modules:
            copy(module, root / "usr/lib/roanix/drivers" / module.name)

        if bootstrap:
            for name in ("dev/.roanix-keep", "tmp/.roanix-keep", "etc/roanix-root"):
                remove(root / name)
        else:
            for header in (ROOT / "drivers/include/roanix").glob("*.h"):
                copy(header, root / "usr/include/roanix" / header.name)

    def userland(self):
        self.build_drivers()
        msg(f"Building userland ({self.arch})")
        self.download(*SYSTEM_PACKAGES)

        # Fetch host tools only if a target package still needs a local build.
        pending = self.jinx("dry-run", *SYSTEM_PACKAGES, capture=True).stdout.strip()
        if pending:
            self.download("host:*")
            self.jinx("update", "-b", *SYSTEM_PACKAGES)

        packages = sorted((self.jinx_dir / "pkgs").glob("*.xbps"))
        headers = sorted((ROOT / "drivers/include/roanix").glob("*.h"))
        inputs = [*packages, *self.modules, *headers, USERLAND / "Jinxfile"]
        assemble(
            self.sysroot,
            inputs=inputs,
            values=SYSTEM_PACKAGES,
            action=lambda root: self.install(root, SYSTEM_PACKAGES),
        )
        return packages

    def pack_initramfs(self, output):
        """Archive the bootstrap packages and drivers with root ownership."""
        with tempfile.TemporaryDirectory(dir=self.out) as scratch:
            root = Path(scratch)
            self.install(root, BOOTSTRAP_PACKAGES, bootstrap=True)

            def as_root(info):
                info.uid = info.gid = 0
                info.uname = info.gname = "root"
                return info

            with tarfile.open(
                output,
                "w:gz",
                format=tarfile.USTAR_FORMAT,
                compresslevel=9 if self.args.release else 1,
            ) as archive:
                for entry in sorted(root.rglob("*")):
                    archive.add(
                        entry,
                        arcname=entry.relative_to(root),
                        recursive=False,
                        filter=as_root,
                    )

    def make_root_disk(self, output):
        """Create the GPT disk containing the complete ext4 system root."""
        required = ("dev", "etc", "sbin", "tmp", "usr")
        layout_exists = all((self.sysroot / name).is_dir() for name in required)
        if not layout_exists or not (self.sysroot / "etc/roanix-root").is_file():
            raise RuntimeError("incomplete root filesystem; run ./x.py rebuild util-roanix")

        payload_size = sum(path.lstat().st_size for path in self.sysroot.rglob("*"))
        required_size = int(payload_size * 1.1) + 64 * MIB
        partition_size = max(128, (required_size + MIB - 1) // MIB) * MIB
        partition_end = (MIB + partition_size) // 512 - 1

        # Reserve a MiB at either end for GPT metadata and partition alignment.
        with output.open("wb") as disk:
            disk.truncate(partition_size + 2 * MIB)
        run(
            "sgdisk", output,
            "-n", f"1:2048:{partition_end}",
            "-t", "1:8300",
            "-c", "1:ROANIX_ROOT",
        )

        # Match the feature subset supported by the kernel's read-only ext4 driver.
        run(
            "mke2fs", "-q", "-t", "ext4", "-F",
            "-b", "4096",
            "-I", "256",
            "-i", "8192",
            "-m", "0",
            "-L", "ROANIX_ROOT",
            "-O", "none,filetype,extent,sparse_super,large_file",
            "-E", f"offset={MIB}",
            "-d", self.sysroot,
            output, partition_size // 4096,
        )

    def make_boot_image(self, output, kind, limine):
        """Stage the common boot files, then package them as an ISO or HDD."""
        with tempfile.TemporaryDirectory(dir=self.out) as scratch:
            root = Path(scratch)
            payload = {
                "boot/roanix": self.kernel,
                "boot/roanix-root.tar.gz": self.initramfs,
                "boot/splash.jpg": USERLAND / "distro-files/splash.jpg",
            }

            if self.arch == "x86_64":
                efi_files = ("BOOTX64.EFI", "BOOTIA32.EFI")
            else:
                efi_files = ("BOOTRISCV64.EFI",)
            for name in efi_files:
                payload[f"EFI/BOOT/{name}"] = limine / name

            if self.arch == "x86_64":
                payload["boot/limine/limine-bios.sys"] = limine / "limine-bios.sys"
            if kind == "iso":
                payload["boot/limine/limine-uefi-cd.bin"] = limine / "limine-uefi-cd.bin"
                if self.arch == "x86_64":
                    payload["boot/limine/limine-bios-cd.bin"] = limine / "limine-bios-cd.bin"

            for destination, source in payload.items():
                copy(source, root / destination)

            config = root / "boot/limine/limine.conf"
            config.parent.mkdir(parents=True, exist_ok=True)
            config.write_text(
                (USERLAND / "distro-files/limine.conf").read_text()
                + "    module_path: $boot():/boot/roanix-root.tar.gz\n"
                + "    module_string: initramfs\n"
            )

            if kind == "iso":
                self.make_iso(output, root)
            else:
                payload_size = sum(path.stat().st_size for path in payload.values())
                self.make_hdd(output, root, payload_size)

            if self.arch == "x86_64":
                run(limine / "limine", "bios-install", output)

    def make_iso(self, output, root):
        bios_args = []
        if self.arch == "x86_64":
            bios_args = [
                "-b", "boot/limine/limine-bios-cd.bin",
                "-no-emul-boot",
                "-boot-load-size", "4",
                "-boot-info-table",
            ]

        run(
            "xorriso", "-as", "mkisofs", "-R", "-r", "-J", *bios_args,
            "-hfsplus", "-apm-block-size", "2048",
            "--efi-boot", "boot/limine/limine-uefi-cd.bin",
            "-efi-boot-part", "--efi-boot-image", "--protective-msdos-label",
            root, "-o", output,
        )

    def make_hdd(self, output, root, payload_size):
        required_size = int(payload_size * 1.4) + 32 * MIB
        disk_size = max(128, (required_size + MIB - 1) // MIB) * MIB
        with output.open("wb") as disk:
            disk.truncate(disk_size)

        hybrid_args = ["-m", "1"] if self.arch == "x86_64" else []
        run("sgdisk", output, "-n", "1:2048", "-t", "1:ef00", *hybrid_args)

        fat_partition = f"{output}@@1M"
        run("mformat", "-v", "ROANIX", "-i", fat_partition, "::")
        run("mcopy", "-s", "-m", "-i", fat_partition, root / "boot", root / "EFI", "::/")

    def image(self, kind):
        """Build changed components and assemble the root disk and boot image."""
        self.build_kernel()
        packages = self.userland()

        assemble(
            self.initramfs,
            inputs=[*packages, *self.modules, USERLAND / "Jinxfile"],
            values=BOOTSTRAP_PACKAGES,
            action=self.pack_initramfs,
        )
        assemble(
            self.root_disk,
            inputs=[stamp(self.sysroot), self.sysroot],
            values=(),
            action=self.make_root_disk,
        )

        limine = resource("limine")
        if self.arch == "x86_64" and not (limine / "limine").is_file():
            run("make", "-C", limine)

        image = self.out / f"roanix-{self.arch}.{kind}"
        distro_files = sorted((USERLAND / "distro-files").iterdir())
        assemble(
            image,
            inputs=[self.kernel, self.initramfs, *distro_files],
            values=DOWNLOADS["limine"],
            action=lambda output: self.make_boot_image(output, kind, limine),
        )
        return image

    def rebuild(self, package):
        """Save source edits before optionally refetching and rebuilding a package."""
        owner, local_source, has_remote_source = source_recipe(package, self.arch)
        name = owner.removeprefix("host:")
        sources = USERLAND / ("host-sources" if owner.startswith("host:") else "sources")
        workdir = sources / f"{name}-workdir"
        downloaded_source = not local_source and has_remote_source

        if downloaded_source and workdir.exists():
            clean_source = sources / f"{name}-clean"
            patched_marker = sources / f"{name}.patched"
            if not clean_source.is_dir() or not patched_marker.is_file():
                raise RuntimeError(
                    f"incomplete Jinx source state for {owner}; "
                    f"preserve {workdir} before repairing it"
                )

            msg(f"Saving {owner} working patch")
            # Jinx leaves an old patch in place when the new diff is empty.
            patch = recipe_path(owner).parent / "patches/jinx-working-patch.patch"
            previous = patch.read_bytes() if patch.exists() else None
            patch.unlink(missing_ok=True)

            try:
                self.jinx("regen", owner)
            except BaseException:
                if previous is not None and not patch.exists():
                    patch.write_bytes(previous)
                raise
        elif downloaded_source:
            msg(
                f"No working sources for {owner}; building from its recipe and saved patches",
                detail=True,
            )

        if self.args.force and not local_source:
            msg(f"Removing downloaded sources for {owner}", detail=True)
            suffixes = (
                "", "-clean", "-workdir",
                ".version", ".patched", ".prepared", ".revision", ".host-revision",
            )
            for suffix in suffixes:
                remove(sources / f"{name}{suffix}")

        self.download("host:*")
        msg(f"Rebuilding {package}")
        self.jinx("rebuild", package)
        msg(f"Package ready in {self.jinx_dir.relative_to(ROOT)}; ./x.py will refresh the images")

    def qemu_command(self, image, extra):
        cpu_count = run("nproc", capture=True).stdout.strip()
        args = [
            f"qemu-system-{self.arch}",
            "-m", "4G",
            "-smp", cpu_count,
        ]

        if image.suffix == ".iso":
            args += ["-cdrom", str(image)]
        else:
            args += ["-drive", f"file={image},format=raw"]

        args += [
            "-drive", f"file={self.root_disk},if=none,format=raw,snapshot=on,id=roanix-root",
            "-device", "nvme,drive=roanix-root,serial=roanix-root",
            "-serial", "mon:stdio",
        ]

        if self.arch == "x86_64":
            # Use KVM when accessible; respect explicit acceleration flags.
            kvm_available = (
                platform.machine() == "x86_64"
                and os.access("/dev/kvm", os.R_OK | os.W_OK)
            )
            accel_overridden = any(
                arg in ("-accel", "-enable-kvm", "-no-kvm") or "accel=" in arg
                for arg in extra
            )
            machine = "q35,smm=off" if self.args.bios else "q35"
            args += ["-M", machine]

            if kvm_available and not accel_overridden:
                args += ["-accel", "kvm", "-cpu", "host,+invtsc"]
            else:
                args += ["-cpu", "max,+invtsc,+tsc-deadline,+fsgsbase"]
        else:
            args += [
                "-M", "virt,acpi=off",
                "-cpu", "rv64",
                "-device", "ramfb",
                "-device", "qemu-xhci",
                "-device", "usb-kbd",
                "-device", "usb-mouse",
            ]

        if not self.args.bios:
            firmware = resource("ovmf") / f"ovmf-code-{self.arch}.fd"
            args += ["-drive", f"if=pflash,unit=0,format=raw,file={firmware},readonly=on"]
        return [*args, *extra]


# Command-line interface.


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    extra = []
    if "--" in argv:
        split = argv.index("--")
        argv, extra = argv[:split], argv[split + 1:]

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  ./x.py                          build changed parts and boot the HDD image
  ./x.py build iso -a riscv64     build a RISC-V ISO and ext4 root disk
  ./x.py build kernel             build just the kernel (also: drivers, userland)
  ./x.py build bash               build a package locally, creating working sources
  ./x.py rebuild bash -f          save a patch, refetch sources, rebuild the package
  ./x.py run -- -display none     pass extra flags to QEMU""",
    )
    parser.add_argument(
        "command", nargs="?", choices=("run", "build", "rebuild"), default="run",
    )
    parser.add_argument(
        "target", nargs="?",
        help="image, component, or Jinx package (host:NAME for host tools)",
    )
    parser.add_argument(
        "-a", "--arch", choices=TARGETS,
        default=os.environ.get("ROANIX_ARCH", "x86_64"),
    )
    parser.add_argument(
        "-r", "--release", action="store_true",
        help="build a release kernel",
    )
    parser.add_argument(
        "-j", "--jobs", type=int,
        help="Cargo/Jinx build jobs (default: nproc)",
    )
    parser.add_argument(
        "-f", "--force", action="store_true",
        help="rebuild only: refetch downloaded sources after saving the patch",
    )
    parser.add_argument(
        "--bios", action="store_true",
        help="run only: use x86_64 BIOS instead of UEFI",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="print subprocess commands",
    )

    args = parser.parse_intermixed_args(argv)

    if args.arch not in TARGETS:
        parser.error("ROANIX_ARCH must be x86_64 or riscv64")
    if args.jobs is not None and args.jobs < 1:
        parser.error("--jobs must be positive")
    if args.force and args.command != "rebuild":
        parser.error("-f is only supported by rebuild")
    if args.bios and (args.command != "run" or args.arch != "x86_64"):
        parser.error("--bios requires run --arch x86_64")
    if extra and args.command != "run":
        parser.error("arguments after -- are only supported by run")
    if args.command == "rebuild" and args.target is None:
        parser.error("rebuild requires a package name")

    args.target = args.target or "hdd"
    if args.command == "run" and args.target not in ("hdd", "iso"):
        parser.error("run accepts hdd or iso")

    components = ("kernel", "drivers", "userland", "hdd", "iso")
    if args.command == "rebuild" or (args.command == "build" and args.target not in components):
        try:
            recipe_path(args.target)
        except ValueError as error:
            parser.error(str(error))
    return args, extra


def main(argv=None):
    global VERBOSE

    args, extra = parse_args(argv)
    VERBOSE = args.verbose

    if sys.version_info < (3, 12):
        raise RuntimeError("x.py requires Python 3.12 or newer")
    if sys.platform != "linux":
        raise RuntimeError("Roanix builds require Linux")

    os.environ["PATH"] = os.environ.get("PATH", "") + ":/usr/sbin:/sbin"
    args.jobs = args.jobs or int(run("nproc", capture=True).stdout)
    build = Build(args)

    if args.command == "rebuild":
        build.rebuild(args.target)
    elif args.target in ("hdd", "iso"):
        image = build.image(args.target)
        msg(f"Ready: {image.relative_to(ROOT)} + {build.root_disk.relative_to(ROOT)}")
        if args.command == "run":
            msg(f"Starting QEMU ({args.arch})")
            run(*build.qemu_command(image, extra))
    elif args.target == "kernel":
        build.build_kernel()
    elif args.target == "drivers":
        build.build_drivers()
    elif args.target == "userland":
        build.userland()
    else:
        build.download("host:*")
        build.jinx("build", args.target)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        msg("Interrupted", error=True)
        sys.exit(130)
    except (OSError, RuntimeError, ValueError, tarfile.TarError) as error:
        msg(str(error), error=True)
        sys.exit(1)
