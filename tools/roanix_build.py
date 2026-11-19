"""Cargo, Jinx, and media assembly for Roanix."""

import json
import os
import platform
import re
import shutil
import tarfile
import tempfile
import tomllib
import uuid
from pathlib import Path

from .roanix_common import BUILD, CACHE, DIST, ROOT, USERLAND, WORK
from .roanix_common import assemble, atomic_text, copy, fetch_archive, msg, remove, run, sha256, stamp
from .roanix_packages import Jinx


# Project layout and package selection.

SYSTEM_PROFILES = tomllib.loads((USERLAND / "build-support/profiles.toml").read_text())
BOOTSTRAP_PACKAGES = tuple(SYSTEM_PROFILES.pop("bootstrap"))
MACHINES = tomllib.loads((ROOT / "tools/machines.toml").read_text())


# Pinned build tools and firmware.

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

MIB = 1024 * 1024


# Shared command, file, and cache helpers.


def resource(name, *, offline=False):
    version, url, checksum, archive_root = DOWNLOADS[name]
    destination = CACHE / ("edk2-ovmf" if name == "ovmf" else name)
    return fetch_archive(name, version, url, checksum, archive_root, destination, offline=offline)


class Build:
    """Build packages and images for one architecture and kernel profile."""

    def __init__(self, args):
        self.args = args
        self.arch = args.arch
        machine = MACHINES[args.machine]
        self.target = machine["rust-target"]
        self.drivers = machine["drivers"]
        self.profile = args.mode
        self.packages = tuple(SYSTEM_PROFILES[args.profile])

        self.base = WORK / self.arch
        self.out = self.base / args.profile / self.profile
        self.sysroot = self.base / args.profile / "root"
        self.jinx = Jinx(args, self.base)
        self.machine_id = BUILD / "local" / f"{self.arch}-machine-id"

        self.kernel = self.out / "roanix"
        self.initramfs = self.out / f"roanix-{self.arch}.initramfs.tar.gz"
        self.root_disk = self.sysroot.parent / "root.img"
        self.modules = []
        for name in self.drivers:
            filename = "storage-nvme.ko" if name == "nvme" else f"{name}.ko"
            self.modules.append(self.base / "drivers-rust" / filename)

    def cargo(self, action, *, drivers=False, driver=None):
        target_dir = BUILD / "cargo"
        options = [
            "--target", self.target, "--locked",
            "--jobs", self.args.jobs,
        ]
        if self.args.offline:
            options.append("--offline")
        if drivers:
            target_dir /= "drivers"
            options.append("--release")
            names = (driver,) if driver else self.drivers
            for name in names:
                options.extend(("--bin", name))
            if self.arch == "riscv64":
                options.extend((
                    "-Zbuild-std=core,alloc,compiler_builtins",
                    "-Zbuild-std-features=compiler-builtins-mem",
                ))
            directory, env = "drivers", None
        else:
            extra = os.environ.get("ROANIX_RUSTFLAGS", "")
            env = {"RUSTFLAGS": f"-Crelocation-model=static -Cforce-frame-pointers=yes {extra}".strip()}
            if action != "fetch":
                options.extend(("--profile", self.profile))
            directory = "kernel"

        if action == "fetch":
            options = ["--target", self.target, "--locked"]
        else:
            options.extend(("--target-dir", target_dir))

        command = ("cargo",)
        if self.args.offline:
            toolchain = tomllib.loads((ROOT / "rust-toolchain.toml").read_text())
            channel = toolchain["toolchain"]["channel"]
            command = ("rustup", "run", channel, "cargo")
        run(*command, action, *options, cwd=ROOT / directory, env=env)
        return target_dir / self.target

    def build_kernel(self):
        msg(f"Building kernel ({self.arch})")
        target_dir = self.cargo("build")
        cargo_profile = "release" if self.profile == "release" else "debug"
        copy(target_dir / cargo_profile / "roanix", self.kernel)
        return self.kernel

    def build_drivers(self, driver=None):
        msg(f"Building {driver or 'drivers'} ({self.arch})")
        target_dir = self.cargo("build", drivers=True, driver=driver) / "release"
        names = (driver,) if driver else self.drivers
        for name in names:
            filename = "storage-nvme.ko" if name == "nvme" else f"{name}.ko"
            output = self.base / "drivers-rust" / filename
            copy(target_dir / name, output)
        return output if driver else self.modules

    def prepare_machine_id(self):
        """Retain one identity for this architecture's read-only system images."""
        self.machine_id.parent.mkdir(parents=True, exist_ok=True)
        if not self.machine_id.exists():
            # Publish a complete file without replacing an identity created by
            # another image build. Neither packages nor source control own it.
            with tempfile.TemporaryDirectory(prefix=".machine-id-", dir=self.machine_id.parent) as scratch:
                candidate = Path(scratch) / "machine-id"
                candidate.write_text(uuid.uuid4().hex + "\n")
                candidate.chmod(0o444)
                try:
                    os.link(candidate, self.machine_id)
                except FileExistsError:
                    pass
        if self.machine_id.is_symlink() or not self.machine_id.is_file():
            raise RuntimeError(f"machine identity must be a regular file: {self.machine_id}")
        identity = self.machine_id.read_bytes()
        if not re.fullmatch(rb"[0-9a-f]{32}\n", identity) or identity == b"0" * 32 + b"\n":
            raise RuntimeError(f"invalid cached machine identity: {self.machine_id}")

    def install_machine_id(self, root):
        """Install the retained identity without following guest file symlinks."""
        self.prepare_machine_id()
        destination = root / "etc/machine-id"
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            destination.unlink()
        if not destination.is_file() or destination.read_bytes() != self.machine_id.read_bytes():
            destination.unlink(missing_ok=True)
            shutil.copy2(self.machine_id, destination)

    @property
    def driver_headers(self):
        return sorted((ROOT / "drivers/include/roanix").glob("*.h"))

    def install(self, root, packages, *, bootstrap=False):
        """Install packages and modules into a fresh root or bootstrap tree."""
        self.jinx.install(root, packages)
        for module in self.modules:
            copy(module, root / "usr/lib/roanix/drivers" / module.name)

        if bootstrap:
            for name in ("dev/.roanix-keep", "tmp/.roanix-keep", "etc/roanix-root", "sbin/init"):
                remove(root / name)
            (root / "tmp").chmod(0o1777)
        else:
            self.install_machine_id(root)
            for header in self.driver_headers:
                copy(header, root / "usr/include/roanix" / header.name)

    def userland(self):
        self.build_drivers()
        runtime = self.jinx.resolve_packages(self.packages, runtime=True)
        bootstrap = self.jinx.resolve_packages(BOOTSTRAP_PACKAGES, runtime=True)
        self.jinx.ensure_packages((*self.packages, *BOOTSTRAP_PACKAGES))
        runtime_files = self.jinx.package_files(runtime)
        bootstrap_files = self.jinx.package_files(bootstrap)
        self.prepare_machine_id()
        assemble(
            self.sysroot,
            inputs=[*runtime_files, *self.modules, *self.driver_headers, self.machine_id],
            values=self.packages,
            action=lambda root: self.install(root, self.packages),
        )
        return runtime_files, bootstrap_files

    def write_initramfs(self, root, output):
        def as_root(info):
            info.uid = info.gid = 0
            info.uname = info.gname = "root"
            return info

        with tarfile.open(
            output, "w:gz", format=tarfile.USTAR_FORMAT,
            compresslevel=6 if self.profile == "release" else 1,
        ) as archive:
            for entry in sorted(root.rglob("*")):
                name = entry.relative_to(root)
                # The kernel has already mounted devfs when it imports this archive.
                if name.parts[0] != "dev":
                    archive.add(entry, arcname=name, recursive=False, filter=as_root)

    def pack_initramfs(self, output):
        with tempfile.TemporaryDirectory(dir=self.out) as scratch:
            root = Path(scratch)
            self.install(root, BOOTSTRAP_PACKAGES, bootstrap=True)
            self.write_initramfs(root, output)

    def root_size(self):
        required = ("dev", "etc", "sbin", "tmp", "usr")
        layout_exists = all((self.sysroot / name).is_dir() for name in required)
        if not layout_exists or not (self.sysroot / "etc/roanix-root").is_file():
            raise RuntimeError("incomplete system root; inspect the util-roanix package")
        payload = sum(path.lstat().st_size for path in self.sysroot.rglob("*"))
        return max(128, (int(payload * 1.1) + 65 * MIB - 1) // MIB) * MIB

    def format_root(self, output, offset, size):
        # Match the feature subset supported by the kernel's read-only ext4 driver.
        run(
            "mke2fs", "-q", "-t", "ext4", "-F",
            "-b", "4096", "-I", "256", "-i", "8192", "-m", "0",
            "-L", "ROANIX_ROOT", "-O", "none,filetype,extent,sparse_super,large_file",
            "-E", f"offset={offset}", "-d", self.sysroot, output, size // 4096,
        )

    def make_root_disk(self, output):
        size = self.root_size()
        with output.open("wb") as disk:
            disk.truncate(size + 2 * MIB)
        run("sgdisk", output, "-n", f"1:2048:{(size + MIB) // 512 - 1}", "-t", "1:8300")
        self.format_root(output, MIB, size)

    def make_boot_image(self, output, kind, limine):
        """Stage the common boot files, then package them as an ISO or HDD."""
        with tempfile.TemporaryDirectory(dir=self.out) as scratch:
            root = Path(scratch)
            partition_id = uuid.uuid4() if kind == "raw" else None
            initramfs = self.live_initramfs if kind == "iso" else self.initramfs
            payload = {
                "boot/roanix": self.kernel,
                "boot/roanix-root.tar.gz": initramfs,
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
            template = (USERLAND / "distro-files/limine.conf").read_text()
            if kind == "iso":
                template = template.replace("root=/dev/nvme0n1p1", "root=initramfs")
            elif partition_id:
                template = template.replace("root=/dev/nvme0n1p1", f"root=PARTUUID={partition_id}")
            config.write_text(
                template
                + "    module_path: $boot():/boot/roanix-root.tar.gz\n"
                + "    module_string: initramfs\n"
            )

            if kind == "iso":
                self.make_iso(output, root)
            else:
                payload_size = sum(path.stat().st_size for path in payload.values())
                self.make_hdd(output, root, payload_size, partition_id)

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

    def make_hdd(self, output, root, payload_size, partition_id):
        required_size = int(payload_size * 1.4) + 32 * MIB
        disk_size = max(128, (required_size + MIB - 1) // MIB) * MIB
        root_size = self.root_size() if partition_id else 0
        root_offset = disk_size + MIB
        with output.open("wb") as disk:
            disk.truncate(root_offset + root_size + MIB if partition_id else disk_size)

        boot_end = disk_size // 512 - 1 if partition_id else 0
        partition_args = ["-n", f"1:2048:{boot_end}", "-t", "1:ef00"]
        if partition_id:
            partition_args.extend((
                "-n", f"2:{root_offset // 512}:{(root_offset + root_size) // 512 - 1}",
                "-t", "2:8300", "-c", "2:ROANIX_ROOT", "-u", f"2:{partition_id}",
            ))
        if self.arch == "x86_64":
            if partition_id:
                partition_args.extend(("-n", "3:34:2047", "-t", "3:ef02"))
            else:
                partition_args.extend(("-m", "1"))
        run("sgdisk", output, *partition_args)
        if partition_id:
            self.format_root(output, root_offset, root_size)

        fat_partition = f"{output}@@1M"
        size = ("-T", str((disk_size - MIB) // 512)) if partition_id else ()
        run("mformat", *size, "-v", "ROANIX", "-i", fat_partition, "::")
        run("mcopy", "-s", "-m", "-i", fat_partition, root / "boot", root / "EFI", "::/")

    def image(self, kind):
        self.build_kernel()
        runtime, bootstrap = self.userland()
        self.live_initramfs = self.out / "live-root.tar.gz"
        if kind == "iso":
            assemble(
                self.live_initramfs, inputs=[stamp(self.sysroot)], values=self.packages,
                action=lambda output: self.write_initramfs(self.sysroot, output),
            )
        else:
            assemble(
                self.initramfs, inputs=[*bootstrap, *self.modules], values=BOOTSTRAP_PACKAGES,
                action=self.pack_initramfs,
            )
        if kind == "hdd":
            assemble(
                self.root_disk, inputs=[stamp(self.sysroot)], values=(),
                action=self.make_root_disk,
            )

        limine = self.limine()
        distro_files = sorted((USERLAND / "distro-files").iterdir())
        if kind == "hdd":
            image = self.out / "boot.hdd"
        else:
            image = DIST / f"roanix-{self.args.machine}-{self.args.profile}-{self.profile}.{kind}"
        inputs = [self.kernel, *distro_files]
        inputs.append(self.live_initramfs if kind == "iso" else self.initramfs)
        if kind == "raw":
            inputs.append(stamp(self.sysroot))
        created = assemble(
            image, inputs=inputs, values=DOWNLOADS["limine"],
            action=lambda output: self.make_boot_image(output, kind, limine),
        )
        if kind != "hdd":
            sidecars = [image.with_suffix(f".{kind}.{extension}") for extension in ("json", "sha256")]
            if created:
                for path in sidecars:
                    path.unlink(missing_ok=True)
            if not all(path.is_file() for path in sidecars):
                self.publish_manifest(image, runtime, bootstrap if kind == "raw" else [])
        return image

    def limine(self):
        limine = resource("limine", offline=self.args.offline)
        if self.arch == "x86_64" and not (limine / "limine").is_file():
            run("make", "-C", limine)
        return limine

    def publish_manifest(self, image, runtime, bootstrap):
        checksum = sha256(image)
        manifest = {
            "image": image.name,
            "machine": self.args.machine,
            "profile": self.args.profile,
            "mode": self.profile,
            "sha256": checksum,
            "kernel": sha256(self.kernel),
            "drivers": {path.name: sha256(path) for path in self.modules},
            "system": {path.name: sha256(path) for path in runtime},
            "bootstrap": {path.name: sha256(path) for path in bootstrap},
        }
        for extension, content in (
            ("sha256", f"{checksum}  {image.name}\n"),
            ("json", json.dumps(manifest, indent=2, sort_keys=True) + "\n"),
        ):
            path = image.with_suffix(f"{image.suffix}.{extension}")
            atomic_text(path, content)

    @staticmethod
    def qemu_path(path):
        return str(path).replace(",", ",,")

    def qemu_command(self, image, extra, *, split=False):
        cpu_count = self.args.cpus or min(len(os.sched_getaffinity(0)), 4)
        args = [
            f"qemu-system-{self.arch}",
            "-m", self.args.memory,
            "-smp", cpu_count,
        ]

        if image.suffix == ".iso":
            args += ["-cdrom", str(image), "-boot", "order=d"]
        elif split or self.args.bios:
            # SeaBIOS boots from SATA. The guest uses its NVMe view for root.
            args += ["-drive", f"file={self.qemu_path(image)},format=raw,snapshot=on"]

        if split or image.suffix != ".iso":
            root = self.qemu_path(self.root_disk if split else image)
            nvme = "nvme,drive=roanix-root,serial=roanix-root"
            if not split and not self.args.bios:
                nvme += ",bootindex=1"
            args += [
                "-drive", f"file={root},if=none,format=raw,snapshot=on,id=roanix-root",
                "-device", nvme,
            ]
        if self.args.command == "test":
            args += ["-serial", "stdio", "-monitor", "none"]
        else:
            args += ["-serial", "mon:stdio"]

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
            firmware = resource("ovmf", offline=self.args.offline) / f"ovmf-code-{self.arch}.fd"
            args += ["-drive", f"if=pflash,unit=0,format=raw,file={self.qemu_path(firmware)},readonly=on"]
        return [*args, *extra]


