#!/usr/bin/env python3
"""
x.py - xtool build system

Lightweight all-in-one script for developers working on Roanix.
"""

import argparse
import gzip
import hashlib
import os
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import List, Mapping, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parent
KERNEL_DIR = ROOT / "kernel"
STORE_DIR = ROOT / "store"
USERLAND_DIR = ROOT / "userland"
ISO_ROOT = ROOT / "iso_root"
LIMINE_DIR = STORE_DIR / "limine"
OVMF_DIR = STORE_DIR / "edk2-ovmf"
JINX_DIR = STORE_DIR / "jinx"
JINX_HOST_TOOLS_DIR = STORE_DIR / "jinx-host-tools"
JINX_COMMIT = "287ceaf9a2c08b43dbc56d38d8b815fd990a2192"
LIMINE_VERSION = "12.5.0"
LIMINE_RELEASE_URL = (
    "https://github.com/limine-bootloader/limine/releases/download/"
    f"v{LIMINE_VERSION}/limine-binary.tar.gz"
)
LIMINE_RELEASE_SHA256 = "8bc0d0f2a2cd0e212f529c57d8a2033a25996dcdd9f58c916b7bf5594a0282eb"

SUPPORTED_ARCHES = ("x86_64", "riscv64")
USERLAND_PACKAGES = ("bash", "init", "os-test")
USERLAND_BUILD_PACKAGES = ("mlibc-headers", "mlibc", *USERLAND_PACKAGES)
OVMF_RELEASE_URL = (
    "https://github.com/osdev0/edk2-ovmf-nightly/releases/latest/download/"
    "edk2-ovmf.tar.gz"
)


class BuildError(RuntimeError):
    """Raised for recoverable build failures."""


class UI:
    """xbuild-style terminal output helper."""

    def __init__(self, color: bool = True, quiet: bool = False):
        self.quiet = quiet
        self.color = color and sys.stdout.isatty()
        self.interactive = sys.stdout.isatty() and not quiet
        self.total = 0
        self.current = 0
        self._line_open = False
        self._last_line_len = 0
        self._started_at = 0.0
        self._palette = {
            "muted": "\033[2m",
            "blue": "\033[34m",
            "cyan": "\033[36m",
            "green": "\033[32m",
            "yellow": "\033[33m",
            "red": "\033[31m",
            "bold": "\033[1m",
            "reset": "\033[0m",
        }

    def _fmt(self, text: str, *styles: str) -> str:
        if not self.color or not styles:
            return text
        prefix = "".join(self._palette[name] for name in styles)
        return f"{prefix}{text}{self._palette['reset']}"

    def begin(self, action: str, total_steps: int) -> None:
        self.total = max(total_steps, 1)
        self.current = 0
        self._started_at = time.monotonic()
        if self.quiet:
            return
        if not self.interactive:
            print(self._fmt(f"xbuild: {action}", "blue", "bold"))

    def step(self, text: str) -> None:
        if self.quiet:
            return
        self.current += 1
        effective_total = max(self.total, self.current)
        line = self._fmt(f"[{self.current}/{effective_total}] {text}", "bold")
        if self.interactive:
            pad = ""
            plain_len = len(f"[{self.current}/{effective_total}] {text}")
            if plain_len < self._last_line_len:
                pad = " " * (self._last_line_len - plain_len)
            print("\r" + line + pad, end="", flush=True)
            self._last_line_len = plain_len
            self._line_open = True
            return
        print(line)

    def _flush_progress_line(self) -> None:
        if self._line_open:
            print()
            self._line_open = False
            self._last_line_len = 0

    def info(self, text: str) -> None:
        if self.quiet:
            return
        self._flush_progress_line()
        print(self._fmt(text, "muted"))

    def warn(self, text: str) -> None:
        self._flush_progress_line()
        print(self._fmt(f"! {text}", "yellow"), file=sys.stderr)

    def error(self, text: str) -> None:
        self._flush_progress_line()
        print(self._fmt(f"error: {text}", "red", "bold"), file=sys.stderr)

    def command(self, argv: Sequence[str], cwd: Path) -> None:
        self._flush_progress_line()
        rel = cwd.relative_to(ROOT) if cwd != ROOT else Path(".")
        cmd = shlex.join(argv)
        print(self._fmt(f"[cmd:{rel}] {cmd}", "muted"))

    def end(self, show_time: bool = False) -> None:
        if self.quiet:
            return
        self._flush_progress_line()
        elapsed = time.monotonic() - self._started_at if self._started_at else 0.0
        if show_time:
            print(
                self._fmt(f"xtool: build completed in {elapsed:.2f}s", "muted", "cyan")
            )


@dataclass(frozen=True)
class Config:
    arch: str
    rust_profile: str
    rust_target: str
    qemu_flags: Tuple[str, ...]
    qemu_passthrough: Tuple[str, ...]
    no_bootstrap: bool
    verbose: bool
    show_time: bool
    ui: UI

    @property
    def kernel_artifact(self) -> Path:
        return KERNEL_DIR / f"roanix-{self.arch}"

    @property
    def image_iso(self) -> Path:
        return ROOT / f"roanix-{self.arch}.iso"

    @property
    def image_hdd(self) -> Path:
        return ROOT / f"roanix-{self.arch}.hdd"

    @property
    def sysroot(self) -> Path:
        return STORE_DIR / "sysroots" / self.arch

    @property
    def initramfs(self) -> Path:
        return STORE_DIR / "initramfs" / f"roanix-{self.arch}.tar.gz"


def default_rust_target(arch: str) -> str:
    if arch == "riscv64":
        return "riscv64gc-unknown-none-elf"
    return f"{arch}-unknown-none"


def executable_files(directory: Path) -> List[Path]:
    files: List[Path] = []
    for path in directory.iterdir():
        if path.is_file() and os.access(path, os.X_OK):
            files.append(path)
    return sorted(files)


def kvm_available() -> bool:
    try:
        mode = os.stat("/dev/kvm").st_mode
    except FileNotFoundError:
        return False
    except PermissionError:
        return False

    return stat.S_ISCHR(mode)


def qemu_accel_overridden(cfg: Config) -> bool:
    return any(
        argument == "-accel" or argument.startswith("-accel=")
        for argument in (*cfg.qemu_flags, *cfg.qemu_passthrough)
    )


def run(
    cfg: Config,
    argv: Sequence[str],
    *,
    step: Optional[str] = None,
    cwd: Path = ROOT,
    env_updates: Optional[Mapping[str, str]] = None,
    capture: Optional[bool] = None,
) -> None:
    env = os.environ.copy()
    if env_updates:
        env.update(env_updates)
    if step:
        cfg.ui.step(step)
    if cfg.verbose:
        cfg.ui.command(list(argv), cwd)
    use_capture = (not cfg.verbose) if capture is None else capture
    try:
        if use_capture:
            completed = subprocess.run(
                list(argv),
                cwd=cwd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
            if completed.returncode != 0:
                output = completed.stdout or ""
                if output:
                    cfg.ui.error(output.rstrip())
                raise BuildError(
                    f"Command failed with exit code {completed.returncode}: {shlex.join(argv)}"
                )
        else:
            cfg.ui._flush_progress_line()
            subprocess.run(list(argv), cwd=cwd, env=env, check=True)
    except FileNotFoundError as exc:
        raise BuildError(f"Required tool is missing: {argv[0]!r}") from exc
    except subprocess.CalledProcessError as exc:
        raise BuildError(
            f"Command failed with exit code {exc.returncode}: {shlex.join(argv)}"
        ) from exc


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def remove_path(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    else:
        path.unlink()


def copy_file(src: Path, dst: Path) -> None:
    ensure_dir(dst.parent)
    shutil.copy2(src, dst)


def safe_extract_tar(archive: Path, destination: Path) -> None:
    dest_resolved = destination.resolve()
    with tarfile.open(archive, "r:gz") as tf:
        for member in tf.getmembers():
            member_path = destination / member.name
            try:
                member_path.resolve().relative_to(dest_resolved)
            except ValueError:
                raise BuildError(
                    f"Refusing to extract unsafe path from archive: {member.name!r}"
                )
        tf.extractall(destination)


def create_initramfs(cfg: Config) -> None:
    if not cfg.sysroot.is_dir():
        raise BuildError(
            f"Userspace sysroot is missing for {cfg.arch}: {cfg.sysroot}"
        )

    cfg.ui.step(f"create gzipped initramfs ({cfg.arch})")
    ensure_dir(cfg.initramfs.parent)
    remove_path(cfg.initramfs)

    def normalize(info: tarfile.TarInfo) -> tarfile.TarInfo:
        info.uid = 0
        info.gid = 0
        info.uname = "root"
        info.gname = "root"
        info.mtime = 0
        return info

    with cfg.initramfs.open("wb") as output:
        with gzip.GzipFile(
            filename="", mode="wb", fileobj=output, mtime=0
        ) as compressed:
            with tarfile.open(
                fileobj=compressed, mode="w", format=tarfile.USTAR_FORMAT
            ) as archive:
                for path in sorted(
                    cfg.sysroot.rglob("*"),
                    key=lambda item: item.relative_to(cfg.sysroot).as_posix(),
                ):
                    archive.add(
                        path,
                        arcname=path.relative_to(cfg.sysroot).as_posix(),
                        recursive=False,
                        filter=normalize,
                    )


def create_optional_initramfs(cfg: Config) -> bool:
    if not cfg.sysroot.is_dir():
        remove_path(cfg.initramfs)
        return False

    create_initramfs(cfg)
    return True


def limine_config_text(*, with_initramfs: bool) -> str:
    config = (USERLAND_DIR / "distro-files/limine.conf").read_text(encoding="ascii")
    if with_initramfs:
        config += (
            "    module_path: $boot():/boot/roanix-root.tar.gz\n"
            "    module_string: initramfs\n"
        )
    return config


def ensure_ovmf(cfg: Config) -> None:
    if OVMF_DIR.exists():
        return
    cfg.ui.step("fetch OVMF firmware archive")
    ensure_dir(STORE_DIR)
    with tempfile.NamedTemporaryFile(
        prefix="edk2-ovmf-", suffix=".tar.gz", delete=False
    ) as tmp:
        tmp_path = Path(tmp.name)
    try:
        with (
            urllib.request.urlopen(OVMF_RELEASE_URL) as response,
            tmp_path.open("wb") as out,
        ):
            shutil.copyfileobj(response, out)
        safe_extract_tar(tmp_path, STORE_DIR)
    except Exception as exc:  # noqa: BLE001
        raise BuildError(f"Failed to download/extract OVMF archive: {exc}") from exc
    finally:
        tmp_path.unlink(missing_ok=True)
    if not OVMF_DIR.exists():
        raise BuildError("OVMF archive did not produce 'store/edk2-ovmf'.")


def limine_ready() -> bool:
    version_file = LIMINE_DIR / ".roanix-version"
    if not (LIMINE_DIR / "limine").exists() or not version_file.is_file():
        return False
    return version_file.read_text(encoding="ascii").strip() == LIMINE_VERSION


def ensure_limine(cfg: Config) -> None:
    if limine_ready():
        return

    cfg.ui.step(f"fetch Limine {LIMINE_VERSION} binary release")
    ensure_dir(STORE_DIR)
    extracted = STORE_DIR / "limine-binary"
    remove_path(extracted)
    with tempfile.NamedTemporaryFile(
        prefix="limine-", suffix=".tar.gz", delete=False
    ) as tmp:
        tmp_path = Path(tmp.name)
    try:
        with (
            urllib.request.urlopen(LIMINE_RELEASE_URL) as response,
            tmp_path.open("wb") as output,
        ):
            shutil.copyfileobj(response, output)
        digest = hashlib.sha256(tmp_path.read_bytes()).hexdigest()
        if digest != LIMINE_RELEASE_SHA256:
            raise BuildError(
                f"Limine archive checksum mismatch: expected "
                f"{LIMINE_RELEASE_SHA256}, got {digest}"
            )
        safe_extract_tar(tmp_path, STORE_DIR)
        if not extracted.is_dir():
            raise BuildError("Limine archive did not produce 'store/limine-binary'.")
        remove_path(LIMINE_DIR)
        extracted.rename(LIMINE_DIR)
        (LIMINE_DIR / ".roanix-version").write_text(
            LIMINE_VERSION + "\n", encoding="ascii"
        )
    except BuildError:
        remove_path(extracted)
        raise
    except Exception as exc:
        remove_path(extracted)
        raise BuildError(f"Failed to download/extract Limine: {exc}") from exc
    finally:
        tmp_path.unlink(missing_ok=True)

    run(cfg, ["make", "-C", str(LIMINE_DIR)], step="build limine host tools")


def ensure_jinx(cfg: Config) -> None:
    jinx = JINX_DIR / "jinx"
    if jinx.exists():
        return

    ensure_dir(JINX_DIR)
    run(cfg, ["git", "init", str(JINX_DIR)], step="initialize Jinx checkout")
    run(
        cfg,
        [
            "git",
            "-C",
            str(JINX_DIR),
            "fetch",
            "--depth=1",
            "https://github.com/Mintsuki/Jinx.git",
            JINX_COMMIT,
        ],
        step="fetch pinned Jinx revision",
    )
    run(
        cfg,
        ["git", "-C", str(JINX_DIR), "checkout", "--detach", "FETCH_HEAD"],
        step="check out Jinx build system",
    )


def jinx_env() -> Mapping[str, str]:
    path = os.environ.get("PATH", "")
    if shutil.which("wget") is not None:
        return {"PATH": path}

    curl = shutil.which("curl")
    if curl is None:
        raise BuildError("Jinx requires wget, or curl for the built-in wget fallback.")

    ensure_dir(JINX_HOST_TOOLS_DIR)
    wget = JINX_HOST_TOOLS_DIR / "wget"
    wget.write_text(
        "#!/bin/sh\n"
        'output=""\n'
        'user_agent=""\n'
        'insecure=""\n'
        'url=""\n'
        'while [ "$#" -gt 0 ]; do\n'
        '    case "$1" in\n'
        '        -O) output="$2"; shift 2 ;;\n'
        '        -qO-) output="-"; shift ;;\n'
        '        -U) user_agent="$2"; shift 2 ;;\n'
        '        -nv|-q) shift ;;\n'
        '        --no-check-certificate) insecure="-k"; shift ;;\n'
        '        --ca-certificate=*) ca_file="${1#*=}"; shift ;;\n'
        '        --certificate=*) certificate="${1#*=}"; shift ;;\n'
        '        --private-key=*) private_key="${1#*=}"; shift ;;\n'
        '        --) shift; break ;;\n'
        '        -*) echo "unsupported wget option: $1" >&2; exit 2 ;;\n'
        '        *) url="$1"; shift ;;\n'
        "    esac\n"
        "done\n"
        'if [ -z "$url" ]; then\n'
        '    echo "wget fallback requires a URL" >&2\n'
        "    exit 2\n"
        "fi\n"
        f"set -- {shlex.quote(curl)} -fL\n"
        '[ -n "$output" ] && set -- "$@" -o "$output"\n'
        '[ -n "$user_agent" ] && set -- "$@" -A "$user_agent"\n'
        '[ -n "$insecure" ] && set -- "$@" "$insecure"\n'
        '[ -n "${ca_file-}" ] && set -- "$@" --cacert "$ca_file"\n'
        '[ -n "${certificate-}" ] && set -- "$@" --cert "$certificate"\n'
        '[ -n "${private_key-}" ] && set -- "$@" --key "$private_key"\n'
        'exec "$@" "$url"\n',
        encoding="ascii",
    )
    wget.chmod(0o755)
    combined_path = f"{JINX_HOST_TOOLS_DIR}:{path}" if path else str(
        JINX_HOST_TOOLS_DIR
    )
    return {"PATH": combined_path}


def build_kernel(cfg: Config) -> None:
    rflags = "-Crelocation-model=static -Cforce-frame-pointers=yes"
    run(
        cfg,
        ["cargo", "build", "--target", cfg.rust_target, "--profile", cfg.rust_profile],
        step=f"cargo build kernel ({cfg.arch}, {cfg.rust_profile})",
        cwd=KERNEL_DIR,
        env_updates={"RUSTFLAGS": rflags},
    )
    profile_subdir = "debug" if cfg.rust_profile == "dev" else cfg.rust_profile
    output_dir = KERNEL_DIR / "target" / cfg.rust_target / profile_subdir
    preferred = output_dir / "roanix"
    if preferred.exists():
        source = preferred
    else:
        executables = executable_files(output_dir)
        if not executables:
            raise BuildError(f"No executable kernel artifact found in {output_dir}")
        source = executables[0]
    shutil.copy2(source, cfg.kernel_artifact)


def build_iso(cfg: Config) -> None:
    if not cfg.no_bootstrap:
        ensure_limine(cfg)
    build_kernel(cfg)
    with_initramfs = create_optional_initramfs(cfg)

    remove_path(ISO_ROOT)
    ensure_dir(ISO_ROOT / "boot" / "limine")
    ensure_dir(ISO_ROOT / "EFI" / "BOOT")
    cfg.ui.step(f"copy boot files for ISO ({cfg.arch})")

    copy_file(cfg.kernel_artifact, ISO_ROOT / "boot" / "roanix")
    if with_initramfs:
        copy_file(cfg.initramfs, ISO_ROOT / "boot" / "roanix-root.tar.gz")
    copy_file(ROOT / "userland/distro-files/splash.jpg", ISO_ROOT / "boot/splash.jpg")
    (ISO_ROOT / "boot/limine/limine.conf").write_text(
        limine_config_text(with_initramfs=with_initramfs),
        encoding="ascii",
    )

    if cfg.arch == "x86_64":
        copy_file(
            LIMINE_DIR / "limine-bios.sys", ISO_ROOT / "boot/limine/limine-bios.sys"
        )
        copy_file(
            LIMINE_DIR / "limine-bios-cd.bin",
            ISO_ROOT / "boot/limine/limine-bios-cd.bin",
        )
        copy_file(
            LIMINE_DIR / "limine-uefi-cd.bin",
            ISO_ROOT / "boot/limine/limine-uefi-cd.bin",
        )
        copy_file(LIMINE_DIR / "BOOTX64.EFI", ISO_ROOT / "EFI/BOOT/BOOTX64.EFI")
        copy_file(LIMINE_DIR / "BOOTIA32.EFI", ISO_ROOT / "EFI/BOOT/BOOTIA32.EFI")
        run(
            cfg,
            [
                "xorriso",
                "-as",
                "mkisofs",
                "-R",
                "-r",
                "-J",
                "-b",
                "boot/limine/limine-bios-cd.bin",
                "-no-emul-boot",
                "-boot-load-size",
                "4",
                "-boot-info-table",
                "-hfsplus",
                "-apm-block-size",
                "2048",
                "--efi-boot",
                "boot/limine/limine-uefi-cd.bin",
                "-efi-boot-part",
                "--efi-boot-image",
                "--protective-msdos-label",
                str(ISO_ROOT),
                "-o",
                str(cfg.image_iso),
            ],
            step=f"pack ISO image ({cfg.arch})",
        )
        run(
            cfg,
            [str(LIMINE_DIR / "limine"), "bios-install", str(cfg.image_iso)],
            step="install limine BIOS hooks into ISO",
        )
    elif cfg.arch == "riscv64":
        copy_file(
            LIMINE_DIR / "limine-uefi-cd.bin",
            ISO_ROOT / "boot/limine/limine-uefi-cd.bin",
        )
        copy_file(LIMINE_DIR / "BOOTRISCV64.EFI", ISO_ROOT / "EFI/BOOT/BOOTRISCV64.EFI")
        run(
            cfg,
            [
                "xorriso",
                "-as",
                "mkisofs",
                "-R",
                "-r",
                "-J",
                "-hfsplus",
                "-apm-block-size",
                "2048",
                "--efi-boot",
                "boot/limine/limine-uefi-cd.bin",
                "-efi-boot-part",
                "--efi-boot-image",
                "--protective-msdos-label",
                str(ISO_ROOT),
                "-o",
                str(cfg.image_iso),
            ],
            step=f"pack ISO image ({cfg.arch})",
        )
    else:
        raise BuildError(f"Unsupported architecture: {cfg.arch}")

    remove_path(ISO_ROOT)


def build_hdd(cfg: Config) -> None:
    if not cfg.no_bootstrap:
        ensure_limine(cfg)
    build_kernel(cfg)
    with_initramfs = create_optional_initramfs(cfg)

    remove_path(cfg.image_hdd)
    with cfg.image_hdd.open("wb") as disk:
        disk.truncate(64 * 1024 * 1024)

    path_env = os.environ.get("PATH", "")
    with_sbin = f"{path_env}:/usr/sbin:/sbin" if path_env else "/usr/sbin:/sbin"
    if cfg.arch == "x86_64":
        run(
            cfg,
            ["sgdisk", str(cfg.image_hdd), "-n", "1:2048", "-t", "1:ef00", "-m", "1"],
            env_updates={"PATH": with_sbin},
            step=f"partition disk image ({cfg.arch})",
        )
        run(
            cfg,
            [str(LIMINE_DIR / "limine"), "bios-install", str(cfg.image_hdd)],
            step="install limine BIOS hooks into HDD",
        )
    elif cfg.arch == "riscv64":
        run(
            cfg,
            ["sgdisk", str(cfg.image_hdd), "-n", "1:2048", "-t", "1:ef00"],
            env_updates={"PATH": with_sbin},
            step=f"partition disk image ({cfg.arch})",
        )
    else:
        raise BuildError(f"Unsupported architecture: {cfg.arch}")

    mtools = str(cfg.image_hdd) + "@@1M"
    run(cfg, ["mformat", "-i", mtools], step="format EFI partition (FAT)")
    run(
        cfg,
        ["mmd", "-i", mtools, "::/EFI", "::/EFI/BOOT", "::/boot", "::/boot/limine"],
        step="create boot directories",
    )
    cfg.ui.step(f"copy boot files into HDD image ({cfg.arch})")
    run(cfg, ["mcopy", "-i", mtools, str(cfg.kernel_artifact), "::/boot/roanix"])
    if with_initramfs:
        run(
            cfg,
            [
                "mcopy",
                "-i",
                mtools,
                str(cfg.initramfs),
                "::/boot/roanix-root.tar.gz",
            ],
        )
    run(
        cfg,
        [
            "mcopy",
            "-i",
            mtools,
            str(ROOT / "userland/distro-files/splash.jpg"),
            "::/boot/",
        ],
    )
    with tempfile.NamedTemporaryFile(
        prefix="roanix-limine-", suffix=".conf", mode="w", encoding="ascii"
    ) as config_file:
        config_file.write(limine_config_text(with_initramfs=with_initramfs))
        config_file.flush()
        run(
            cfg,
            [
                "mcopy",
                "-i",
                mtools,
                config_file.name,
                "::/boot/limine/limine.conf",
            ],
        )
    if cfg.arch == "x86_64":
        run(
            cfg,
            [
                "mcopy",
                "-i",
                mtools,
                str(LIMINE_DIR / "limine-bios.sys"),
                "::/boot/limine",
            ],
        )
        run(
            cfg, ["mcopy", "-i", mtools, str(LIMINE_DIR / "BOOTX64.EFI"), "::/EFI/BOOT"]
        )
        run(
            cfg,
            ["mcopy", "-i", mtools, str(LIMINE_DIR / "BOOTIA32.EFI"), "::/EFI/BOOT"],
        )
    if cfg.arch == "riscv64":
        run(
            cfg,
            ["mcopy", "-i", mtools, str(LIMINE_DIR / "BOOTRISCV64.EFI"), "::/EFI/BOOT"],
        )


def run_qemu(
    cfg: Config, *, use_iso: bool, bios: bool = False, forced_arch: Optional[str] = None
) -> None:
    arch = forced_arch or cfg.arch
    script_cfg = (
        cfg
        if arch == cfg.arch
        else Config(
            arch=arch,
            rust_profile=cfg.rust_profile,
            rust_target=default_rust_target(arch),
            qemu_flags=cfg.qemu_flags,
            qemu_passthrough=cfg.qemu_passthrough,
            no_bootstrap=cfg.no_bootstrap,
            verbose=cfg.verbose,
            show_time=cfg.show_time,
            ui=cfg.ui,
        )
    )

    if bios and arch != "x86_64":
        raise BuildError("BIOS runs are only supported for x86_64.")

    if bios:
        build_iso(script_cfg) if use_iso else build_hdd(script_cfg)
    else:
        ensure_ovmf(script_cfg)
        build_iso(script_cfg) if use_iso else build_hdd(script_cfg)

    argv = [f"qemu-system-{arch}", "-m", "2G"]

    if use_iso:
        argv += ["-cdrom", str(script_cfg.image_iso)]
    else:
        argv += ["-hda", str(script_cfg.image_hdd)]

    if bios:
        argv += ["-M", "q35,smm=off"]
        if kvm_available() and not qemu_accel_overridden(script_cfg):
            argv += ["-accel", "kvm", "-cpu", "host,+invtsc"]
        else:
            argv += ["-cpu", "max,+invtsc,+tsc-deadline,+fsgsbase"]

        argv += ["-serial", "stdio"]
    else:
        argv += [
            "-drive",
            f"if=pflash,unit=0,format=raw,file={OVMF_DIR / f'ovmf-code-{arch}.fd'},readonly=on",
        ]

        if arch == "x86_64":
            argv += ["-M", "q35"]
            if kvm_available() and not qemu_accel_overridden(script_cfg):
                argv += ["-accel", "kvm", "-cpu", "host,+invtsc"]
            else:
                argv += ["-cpu", "max,+invtsc,+tsc-deadline,+fsgsbase"]

            argv += [
                "-drive",
                f"if=pflash,unit=1,format=raw,file={OVMF_DIR / f'ovmf-vars-{arch}.fd'}",
                "-debugcon",
                "stdio",
            ]
        elif arch == "riscv64":
            argv += [
                "-M",
                "virt,acpi=off",
                "-m",
                "2G",
                "-cpu",
                "rv64",
                "-device",
                "ramfb",
                "-device",
                "qemu-xhci",
                "-device",
                "usb-kbd",
                "-device",
                "usb-mouse",
            ]
            argv += ["-serial", "stdio"]
        else:
            raise BuildError(f"Unsupported architecture: {arch}")

    argv += list(script_cfg.qemu_flags)
    argv += list(script_cfg.qemu_passthrough)
    step = (
        "launch qemu (x86_64 BIOS)"
        if bios
        else f"launch qemu ({arch}, {'iso' if use_iso else 'hdd'})"
    )
    run(
        script_cfg,
        argv,
        step=step,
        capture=False,
    )


def cmd_kernel(cfg: Config) -> None:
    build_kernel(cfg)


def cmd_check(cfg: Config) -> None:
    run(
        cfg,
        ["cargo", "check", "--target", cfg.rust_target],
        cwd=KERNEL_DIR,
        step=f"cargo check kernel ({cfg.arch})",
    )


def cmd_clippy(cfg: Config) -> None:
    run(
        cfg,
        ["cargo", "clippy", "--target", cfg.rust_target, "--", "-D", "warnings"],
        cwd=KERNEL_DIR,
        step=f"cargo clippy kernel ({cfg.arch})",
    )


def cmd_fmt(cfg: Config, *, check: bool) -> None:
    argv = ["cargo", "fmt", "--all"]
    if check:
        argv += ["--", "--check"]
    run(
        cfg,
        argv,
        cwd=KERNEL_DIR,
        step="check Rust formatting" if check else "format Rust sources",
    )


def cmd_rustdoc(cfg: Config) -> None:
    run(cfg, ["cargo", "doc", "--no-deps"], cwd=KERNEL_DIR, step="build rustdoc")
    run(
        cfg,
        ["python3", "-m", "http.server", "8080"],
        cwd=KERNEL_DIR / "target" / "doc",
        step="serve docs at http://127.0.0.1:8080",
        capture=False,
    )


def cmd_book(cfg: Config) -> None:
    run(cfg, ["mdbook", "serve"], cwd=ROOT / "book", step="serve mdBook", capture=False)


def cmd_sysroot(cfg: Config) -> None:
    ensure_jinx(cfg)
    env = jinx_env()

    build_dir = STORE_DIR / f"jinx-build-{cfg.arch}"
    ensure_dir(build_dir)
    if not (build_dir / ".jinx-parameters").exists():
        run(
            cfg,
            [
                str(JINX_DIR / "jinx"),
                "init",
                str(USERLAND_DIR),
                f"ARCH={cfg.arch}",
            ],
            cwd=build_dir,
            env_updates=env,
            step=f"initialize Jinx build ({cfg.arch})",
        )

    run(
        cfg,
        [str(JINX_DIR / "jinx"), "update", "-b", *USERLAND_BUILD_PACKAGES],
        cwd=build_dir,
        env_updates=env,
        step=f"update userspace package closure ({cfg.arch})",
        capture=False,
    )
    remove_path(cfg.sysroot)
    ensure_dir(cfg.sysroot)
    run(
        cfg,
        [
            str(JINX_DIR / "jinx"),
            "install",
            str(cfg.sysroot),
            *USERLAND_PACKAGES,
        ],
        cwd=build_dir,
        env_updates=env,
        step=f"install userspace sysroot ({cfg.arch})",
        capture=False,
    )


def cmd_initramfs(cfg: Config) -> None:
    cmd_sysroot(cfg)
    create_initramfs(cfg)


def cmd_clean(cfg: Config) -> None:
    run(cfg, ["cargo", "clean"], cwd=KERNEL_DIR, step="cargo clean kernel")
    for pattern in ("roanix-*.iso", "roanix-*.hdd"):
        for path in ROOT.glob(pattern):
            remove_path(path)
    remove_path(ISO_ROOT)
    remove_path(cfg.initramfs)
    for path in KERNEL_DIR.glob("roanix-*"):
        remove_path(path)


def cmd_distclean(cfg: Config) -> None:
    cmd_clean(cfg)
    cfg.ui.step("remove cached tooling directories")
    for extra in (STORE_DIR, ROOT / "book/book", ROOT / "target"):
        remove_path(extra)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="x.py",
        description="Roanix build system",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--arch",
        choices=SUPPORTED_ARCHES,
        default=None,
        help="Kernel/image architecture.",
    )
    parser.add_argument(
        "--rust-profile", default="dev", help="Cargo profile for kernel builds."
    )
    parser.add_argument(
        "--rust-target",
        default=None,
        help="Rust target triple override. Default is inferred from --arch.",
    )
    parser.add_argument(
        "--qemu-flags",
        default="",
        help="Extra flags appended to QEMU command lines.",
    )
    parser.add_argument(
        "--no-color", action="store_true", help="Disable ANSI colors in output."
    )
    parser.add_argument("--quiet", action="store_true", help="Reduce status output.")
    parser.add_argument(
        "--verbose", action="store_true", help="Stream all command output."
    )
    parser.add_argument(
        "--show-time",
        action="store_true",
        help="Show completion timing message for build commands.",
    )
    parser.add_argument(
        "--no-bootstrap",
        action="store_true",
        help="Skip fetching/building Limine automatically (assume store/limine already exists).",
    )

    parser.set_defaults(qemu_passthrough=[])
    subparsers = parser.add_subparsers(dest="command", metavar="command")
    subparsers.add_parser("gen-hdd", help="Build HDD image (default).")
    subparsers.add_parser("gen-iso", help="Build ISO image.")
    subparsers.add_parser("build", help="Build kernel only.")
    subparsers.add_parser("check", help="Type-check the kernel.")
    subparsers.add_parser("clippy", help="Lint the kernel with Clippy.")
    subparsers.add_parser("fmt", help="Format Rust sources.")
    subparsers.add_parser("fmt-check", help="Check Rust source formatting.")
    subparsers.add_parser("rustdoc", help="Build and serve kernel rustdoc on :8080.")
    subparsers.add_parser("book", help="Run mdBook preview server.")
    subparsers.add_parser(
        "sysroot", help="Build and install the Jinx Bash userspace sysroot."
    )
    subparsers.add_parser(
        "initramfs", help="Build the userspace sysroot and pack its Limine module."
    )
    subparsers.add_parser("clean", help="Remove build outputs.")
    subparsers.add_parser("distclean", help="Remove build outputs and caches.")
    run_parser = subparsers.add_parser("run", help="Build HDD and run on QEMU (UEFI).")
    run_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_iso_parser = subparsers.add_parser(
        "run-iso", help="Build ISO and run on QEMU (UEFI)."
    )
    run_iso_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_x86_parser = subparsers.add_parser("run-x86_64", help="Run x86_64 UEFI image.")
    run_x86_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_rv_parser = subparsers.add_parser("run-riscv64", help="Run riscv64 UEFI image.")
    run_rv_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_iso_x86_parser = subparsers.add_parser(
        "run-iso-x86_64", help="Run x86_64 UEFI ISO."
    )
    run_iso_x86_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_iso_rv_parser = subparsers.add_parser(
        "run-iso-riscv64", help="Run riscv64 UEFI ISO."
    )
    run_iso_rv_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_bios_parser = subparsers.add_parser(
        "run-bios", help="Run x86_64 HDD in BIOS mode."
    )
    run_bios_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    run_iso_bios_parser = subparsers.add_parser(
        "run-iso-bios", help="Run x86_64 ISO in BIOS mode."
    )
    run_iso_bios_parser.add_argument(
        "qemu_passthrough", nargs=argparse.REMAINDER, help=argparse.SUPPRESS
    )
    return parser


def make_config(args: argparse.Namespace) -> Config:
    command = args.command or "gen-hdd"
    if args.arch is None and command in {"sysroot", "initramfs"}:
        raise BuildError(f"{command} requires an explicit --arch")
    arch = args.arch or "x86_64"
    if arch not in SUPPORTED_ARCHES:
        raise BuildError(f"Unsupported architecture: {arch}")

    rust_target = args.rust_target or default_rust_target(arch)
    qemu_flags = tuple(shlex.split(args.qemu_flags)) if args.qemu_flags else ()
    qemu_passthrough = tuple(args.qemu_passthrough or [])
    if qemu_passthrough and qemu_passthrough[0] == "--":
        qemu_passthrough = qemu_passthrough[1:]
    ui = UI(color=not args.no_color, quiet=args.quiet)
    return Config(
        arch=arch,
        rust_profile=args.rust_profile,
        rust_target=rust_target,
        qemu_flags=qemu_flags,
        qemu_passthrough=qemu_passthrough,
        no_bootstrap=args.no_bootstrap,
        verbose=args.verbose,
        show_time=args.show_time,
        ui=ui,
    )


def _limine_bootstrap_steps(cfg: Config) -> int:
    if cfg.no_bootstrap or limine_ready():
        return 0
    return 2


def _ovmf_bootstrap_steps() -> int:
    return 0 if OVMF_DIR.exists() else 1


def _jinx_bootstrap_steps(cfg: Config) -> int:
    steps = 0 if (JINX_DIR / "jinx").exists() else 3
    build_dir = STORE_DIR / f"jinx-build-{cfg.arch}"
    if not (build_dir / ".jinx-parameters").exists():
        steps += 1
    return steps


def _sysroot_steps(cfg: Config) -> int:
    return _jinx_bootstrap_steps(cfg) + 2


def _initramfs_steps(cfg: Config) -> int:
    return _sysroot_steps(cfg) + 1


def _optional_initramfs_steps(cfg: Config) -> int:
    return 1 if cfg.sysroot.is_dir() else 0


def _iso_steps(cfg: Config, arch: str) -> int:
    return (
        _limine_bootstrap_steps(cfg)
        + _optional_initramfs_steps(cfg)
        + (4 if arch == "x86_64" else 3)
    )


def _hdd_steps(cfg: Config, arch: str) -> int:
    if arch == "x86_64":
        return _limine_bootstrap_steps(cfg) + _optional_initramfs_steps(cfg) + 6
    return _limine_bootstrap_steps(cfg) + _optional_initramfs_steps(cfg) + 5


def estimate_steps(cfg: Config, command: Optional[str]) -> int:
    cmd = command or "gen-hdd"
    if cmd == "gen-hdd":
        return _hdd_steps(cfg, cfg.arch)
    if cmd == "gen-iso":
        return _iso_steps(cfg, cfg.arch)
    if cmd == "build":
        return 1
    if cmd in {"check", "clippy", "fmt", "fmt-check"}:
        return 1
    if cmd == "rustdoc":
        return 2
    if cmd == "book":
        return 1
    if cmd == "sysroot":
        return _sysroot_steps(cfg)
    if cmd == "initramfs":
        return _initramfs_steps(cfg)
    if cmd == "clean":
        return 1
    if cmd == "distclean":
        return 2
    if cmd == "run":
        return _ovmf_bootstrap_steps() + _hdd_steps(cfg, cfg.arch) + 1
    if cmd == "run-iso":
        return _ovmf_bootstrap_steps() + _iso_steps(cfg, cfg.arch) + 1
    if cmd == "run-x86_64":
        return _ovmf_bootstrap_steps() + _hdd_steps(cfg, "x86_64") + 1
    if cmd == "run-riscv64":
        return _ovmf_bootstrap_steps() + _hdd_steps(cfg, "riscv64") + 1
    if cmd == "run-iso-x86_64":
        return _ovmf_bootstrap_steps() + _iso_steps(cfg, "x86_64") + 1
    if cmd == "run-iso-riscv64":
        return _ovmf_bootstrap_steps() + _iso_steps(cfg, "riscv64") + 1
    if cmd == "run-bios":
        return _hdd_steps(cfg, "x86_64") + 1
    if cmd == "run-iso-bios":
        return _iso_steps(cfg, "x86_64") + 1
    return 1


def allow_completion_message(command: Optional[str]) -> bool:
    cmd = command or "gen-hdd"
    if cmd.startswith("run"):
        return False
    if cmd in {"clean", "distclean"}:
        return False
    return True


def dispatch(cfg: Config, command: Optional[str]) -> None:
    cmd = command or "gen-hdd"
    if cmd == "gen-hdd":
        build_hdd(cfg)
    elif cmd == "gen-iso":
        build_iso(cfg)
    elif cmd == "build":
        cmd_kernel(cfg)
    elif cmd == "check":
        cmd_check(cfg)
    elif cmd == "clippy":
        cmd_clippy(cfg)
    elif cmd == "fmt":
        cmd_fmt(cfg, check=False)
    elif cmd == "fmt-check":
        cmd_fmt(cfg, check=True)
    elif cmd == "rustdoc":
        cmd_rustdoc(cfg)
    elif cmd == "book":
        cmd_book(cfg)
    elif cmd == "sysroot":
        cmd_sysroot(cfg)
    elif cmd == "initramfs":
        cmd_initramfs(cfg)
    elif cmd == "clean":
        cmd_clean(cfg)
    elif cmd == "distclean":
        cmd_distclean(cfg)
    elif cmd == "run":
        run_qemu(cfg, use_iso=False)
    elif cmd == "run-iso":
        run_qemu(cfg, use_iso=True)
    elif cmd == "run-x86_64":
        run_qemu(cfg, use_iso=False, forced_arch="x86_64")
    elif cmd == "run-riscv64":
        run_qemu(cfg, use_iso=False, forced_arch="riscv64")
    elif cmd == "run-iso-x86_64":
        run_qemu(cfg, use_iso=True, forced_arch="x86_64")
    elif cmd == "run-iso-riscv64":
        run_qemu(cfg, use_iso=True, forced_arch="riscv64")
    elif cmd == "run-bios":
        if cfg.arch != "x86_64":
            raise BuildError(
                "run-bios only supports --arch x86_64; use run-riscv64 for riscv64."
            )
        run_qemu(cfg, use_iso=False, bios=True, forced_arch="x86_64")
    elif cmd == "run-iso-bios":
        if cfg.arch != "x86_64":
            raise BuildError(
                "run-iso-bios only supports --arch x86_64; use run-iso-riscv64 for riscv64."
            )
        run_qemu(cfg, use_iso=True, bios=True, forced_arch="x86_64")
    else:
        raise BuildError(f"Unknown command: {cmd}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    cfg: Optional[Config] = None
    try:
        cfg = make_config(args)
        command = args.command or "gen-hdd"
        cfg.ui.begin(command, estimate_steps(cfg, command))
        dispatch(cfg, args.command)
        cfg.ui.end(show_time=(cfg.show_time and allow_completion_message(command)))
    except BuildError as exc:
        if cfg is not None:
            cfg.ui.error(str(exc))
        else:
            ui = UI(color=not getattr(args, "no_color", False), quiet=False)
            ui.error(str(exc))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
