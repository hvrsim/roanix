#!/usr/bin/env python3
"""Roanix developer build tool."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.request
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence
from urllib.error import HTTPError, URLError

if sys.version_info < (3, 11):
    print("error: x.py requires Python 3.11 or newer", file=sys.stderr)
    raise SystemExit(2)


VERSION = "2.0"
ROOT = Path(__file__).resolve().parent
KERNEL_DIR = ROOT / "kernel"
DRIVERS_DIR = ROOT / "drivers"
USERLAND_DIR = ROOT / "userland"
JINX_DRIVERS_SOURCE = USERLAND_DIR / "drivers"
DEFAULT_ARCH = "x86_64"
DEFAULT_PROFILE = "dev"
DEFAULT_OUTPUT_DIR = ROOT / "build"
RUNTIME_ROOT = DEFAULT_OUTPUT_DIR / "runtime"
LEGACY_STORE_DIR = ROOT / "store"
SUPPORTED_PROFILES = ("dev", "release")

LEGACY_COMMANDS: dict[str, list[str]] = {
    "gen-hdd": ["build", "hdd"],
    "gen-iso": ["build", "iso"],
    "clippy": ["lint"],
    "fmt-check": ["fmt", "--check"],
    "rustdoc": ["docs", "rust", "--serve"],
    "book": ["docs", "book", "--serve"],
    "sysroot": ["build", "sysroot"],
    "initramfs": ["build", "initramfs"],
    "distclean": ["clean", "--all"],
    "run-iso": ["run", "iso"],
    "run-x86_64": ["run", "hdd", "--arch", "x86_64"],
    "run-riscv64": ["run", "hdd", "--arch", "riscv64"],
    "run-iso-x86_64": ["run", "iso", "--arch", "x86_64"],
    "run-iso-riscv64": ["run", "iso", "--arch", "riscv64"],
    "run-bios": ["run", "hdd", "--arch", "x86_64", "--firmware", "bios"],
    "run-iso-bios": ["run", "iso", "--arch", "x86_64", "--firmware", "bios"],
}

OPTIONS_WITH_VALUES = {
    "--arch",
    "--profile",
    "--rust-profile",
    "--rust-target",
    "--qemu-flags",
    "--color",
    "--out-dir",
}

PACKAGE_HINTS = {
    "cargo": "rustup",
    "rustup": "rustup",
    "git": "git",
    "make": "make",
    "xorriso": "xorriso",
    "sgdisk": "gdisk",
    "mformat": "mtools",
    "mmd": "mtools",
    "mcopy": "mtools",
    "qemu-system-x86_64": "qemu-system-x86",
    "qemu-system-riscv64": "qemu-system-misc",
    "mdbook": "mdbook",
    "zstd": "zstd",
}


class BuildError(RuntimeError):
    """A user-facing build failure."""


class HelpFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    """Readable command help with defaults and examples."""


class UI:
    """Small terminal UI."""

    _COLORS = {
        "muted": "\033[2m",
        "blue": "\033[34m",
        "cyan": "\033[36m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "red": "\033[31m",
        "bold": "\033[1m",
        "reset": "\033[0m",
    }

    def __init__(self, *, color: str, quiet: bool, verbose: bool) -> None:
        if color == "always":
            self.color = True
        elif color == "never":
            self.color = False
        else:
            self.color = sys.stdout.isatty() and "NO_COLOR" not in os.environ
        self.quiet = quiet
        self.verbose = verbose
        self.started_at = time.monotonic()

    def _format(self, text: str, *styles: str) -> str:
        if not self.color:
            return text
        prefix = "".join(self._COLORS[style] for style in styles)
        return f"{prefix}{text}{self._COLORS['reset']}"

    def heading(self, text: str) -> None:
        if not self.quiet:
            print(self._format(f"roanix: {text}", "blue", "bold"))

    def step(self, text: str) -> None:
        if not self.quiet:
            print(self._format(f"  -> {text}", "bold"))

    def detail(self, text: str) -> None:
        if not self.quiet:
            print(self._format(f"     {text}", "muted"))

    def command(self, argv: Sequence[str], cwd: Path) -> None:
        if not self.verbose:
            return
        try:
            shown_cwd = cwd.relative_to(ROOT)
        except ValueError:
            shown_cwd = cwd
        print(self._format(f"     [{shown_cwd or '.'}] {shlex.join(argv)}", "muted"))

    def success(self, text: str, *, show_time: bool) -> None:
        if self.quiet:
            return
        suffix = ""
        if show_time:
            suffix = f" in {time.monotonic() - self.started_at:.2f}s"
        print(self._format(f"  ok {text}{suffix}", "green", "bold"))

    def warning(self, text: str) -> None:
        print(self._format(f"warning: {text}", "yellow"), file=sys.stderr)

    def error(self, text: str) -> None:
        print(self._format(f"error: {text}", "red", "bold"), file=sys.stderr)


@dataclass(frozen=True)
class Architecture:
    name: str
    rust_target: str
    qemu_binary: str
    qemu_memory: str


@dataclass(frozen=True)
class DownloadPin:
    name: str
    version: str
    url: str
    sha256: str
    archive_root: str
    directory: str

    def marker(self) -> dict[str, str]:
        return {
            "name": self.name,
            "version": self.version,
            "url": self.url,
            "sha256": self.sha256,
            "archive_root": self.archive_root,
        }


@dataclass(frozen=True)
class GitPin:
    name: str
    url: str
    commit: str
    directory: str


ARCHITECTURES: Mapping[str, Architecture] = {
    "x86_64": Architecture(
        name="x86_64",
        rust_target="x86_64-unknown-none",
        qemu_binary="qemu-system-x86_64",
        qemu_memory="2G",
    ),
    "riscv64": Architecture(
        name="riscv64",
        rust_target="riscv64gc-unknown-none-elf",
        qemu_binary="qemu-system-riscv64",
        qemu_memory="2G",
    ),
}

USERSPACE_PACKAGES = (
    "linux-headers",
    "bash",
    "coreutils",
    "python",
    "init",
    "drivers",
)
USERSPACE_BUILD_PACKAGES = (
    "mlibc-headers",
    "mlibc",
    "ncurses",
    "readline",
    *USERSPACE_PACKAGES,
)

DOWNLOADS: Mapping[str, DownloadPin] = {
    "limine": DownloadPin(
        name="limine",
        version="12.5.0",
        url=(
            "https://github.com/limine-bootloader/limine/releases/download/"
            "v12.5.0/limine-binary.tar.gz"
        ),
        sha256="8bc0d0f2a2cd0e212f529c57d8a2033a25996dcdd9f58c916b7bf5594a0282eb",
        archive_root="limine-binary",
        directory="limine",
    ),
    "ovmf": DownloadPin(
        name="ovmf",
        version="20260531T041444Z",
        url=(
            "https://github.com/osdev0/edk2-ovmf-nightly/releases/download/"
            "20260531T041444Z/edk2-ovmf.tar.gz"
        ),
        sha256="534384e4b971730143f54708493fe43932eabbfd275094b636561fb2028bde62",
        archive_root="edk2-ovmf",
        directory="edk2-ovmf",
    ),
}

JINX = GitPin(
    name="jinx",
    url="https://github.com/Mintsuki/Jinx.git",
    commit="287ceaf9a2c08b43dbc56d38d8b815fd990a2192",
    directory="jinx",
)


@dataclass(frozen=True)
class Context:
    arch: Architecture
    profile: str
    rust_target: str
    output_dir: Path
    force: bool
    no_bootstrap: bool
    show_time: bool
    ui: UI
    qemu_args: tuple[str, ...] = ()

    @property
    def artifact_dir(self) -> Path:
        return self.output_dir / self.arch.name / self.profile

    @property
    def work_dir(self) -> Path:
        return self.output_dir / ".work" / self.arch.name / self.profile

    @property
    def cargo_target_dir(self) -> Path:
        return self.output_dir / ".cargo"

    @property
    def kernel_artifact(self) -> Path:
        return self.artifact_dir / "roanix"

    @property
    def image_iso(self) -> Path:
        return self.artifact_dir / f"roanix-{self.arch.name}.iso"

    @property
    def image_hdd(self) -> Path:
        return self.artifact_dir / f"roanix-{self.arch.name}.hdd"

    @property
    def sysroot(self) -> Path:
        return RUNTIME_ROOT / "sysroots" / self.arch.name

    @property
    def initramfs(self) -> Path:
        return self.artifact_dir / f"roanix-{self.arch.name}.initramfs.tar.gz"

    @property
    def iso_root(self) -> Path:
        return self.work_dir / "iso-root"

    @property
    def runtime_dir(self) -> Path:
        return RUNTIME_ROOT / "qemu" / self.arch.name

    def environment(self, updates: Mapping[str, str] | None = None) -> dict[str, str]:
        env = os.environ.copy()
        if updates:
            env.update(updates)
        return env


def _capture(argv: Sequence[str], *, cwd: Path = ROOT) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            list(argv),
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise BuildError(f"Required tool is missing: {argv[0]}") from exc


def make_context(args: argparse.Namespace) -> Context:
    arch_name = getattr(args, "arch", None) or DEFAULT_ARCH
    try:
        arch = ARCHITECTURES[arch_name]
    except KeyError as exc:
        supported = ", ".join(sorted(ARCHITECTURES))
        raise BuildError(
            f"Unsupported architecture {arch_name!r}; choose one of: {supported}"
        ) from exc

    profile = getattr(args, "profile", None) or DEFAULT_PROFILE
    if profile not in SUPPORTED_PROFILES:
        supported = ", ".join(SUPPORTED_PROFILES)
        raise BuildError(
            f"Unsupported profile {profile!r}; choose one of: {supported}"
        )

    rust_target_override = getattr(args, "rust_target", None)
    if rust_target_override and rust_target_override != arch.rust_target:
        raise BuildError(
            f"Rust target {rust_target_override!r} does not match "
            f"architecture {arch.name!r} ({arch.rust_target})"
        )
    rust_target = arch.rust_target
    output_dir_arg = getattr(args, "out_dir", None)
    output_dir = Path(output_dir_arg) if output_dir_arg else DEFAULT_OUTPUT_DIR
    if not output_dir.is_absolute():
        output_dir = ROOT / output_dir
    output_dir = output_dir.resolve()
    default_output = DEFAULT_OUTPUT_DIR.resolve()
    runtime_root = RUNTIME_ROOT.resolve()
    if output_dir != default_output:
        try:
            output_dir.relative_to(default_output)
        except ValueError as exc:
            raise BuildError(
                f"Custom output directory must be inside "
                f"{_display_path(DEFAULT_OUTPUT_DIR)}"
            ) from exc
        if output_dir.is_relative_to(runtime_root) or runtime_root.is_relative_to(
            output_dir
        ):
            raise BuildError("Custom output directory must not overlap build/runtime")
    color = getattr(args, "color", "auto")
    quiet = bool(getattr(args, "quiet", False))
    verbose = bool(getattr(args, "verbose", False))
    ui = UI(color=color, quiet=quiet, verbose=verbose)
    return Context(
        arch=arch,
        profile=profile,
        rust_target=rust_target,
        output_dir=output_dir,
        force=bool(getattr(args, "force", False)),
        no_bootstrap=bool(getattr(args, "no_bootstrap", False)),
        show_time=bool(getattr(args, "show_time", False)),
        ui=ui,
    )


def run(
    ctx: Context,
    argv: Sequence[str],
    *,
    step: str | None = None,
    cwd: Path = ROOT,
    env_updates: Mapping[str, str] | None = None,
    capture: bool | None = None,
) -> subprocess.CompletedProcess[str]:
    if step:
        ctx.ui.step(step)
    ctx.ui.command(argv, cwd)
    use_capture = not ctx.ui.verbose if capture is None else capture
    try:
        completed = subprocess.run(
            list(argv),
            cwd=cwd,
            env=ctx.environment(env_updates),
            stdout=subprocess.PIPE if use_capture else None,
            stderr=subprocess.STDOUT if use_capture else None,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise BuildError(f"Required tool is missing: {argv[0]}") from exc
    if completed.returncode != 0:
        if use_capture and completed.stdout:
            print(completed.stdout.rstrip(), file=sys.stderr)
        raise BuildError(
            f"Command failed with exit code {completed.returncode}: "
            f"{shlex.join(argv)}"
        )
    return completed


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def remove_path(path: Path) -> None:
    if not path.exists() and not path.is_symlink():
        return
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    else:
        path.unlink()


def copy_file(source: Path, destination: Path) -> None:
    ensure_dir(destination.parent)
    shutil.copy2(source, destination)


def write_text(path: Path, text: str, *, mode: int = 0o644) -> None:
    ensure_dir(path.parent)
    path.write_text(text, encoding="utf-8")
    path.chmod(mode)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _fingerprint_path(digest: Any, path: Path) -> None:
    try:
        label = path.relative_to(ROOT).as_posix()
    except ValueError:
        label = str(path)
    digest.update(label.encode("utf-8"))
    digest.update(b"\0")
    if path.is_symlink():
        digest.update(b"L")
        digest.update(os.readlink(path).encode("utf-8"))
        digest.update(b"\0")
        return
    if path.is_file():
        digest.update(b"F")
        with path.open("rb") as file:
            for block in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(block)
        digest.update(b"\0")
        return
    if path.is_dir():
        digest.update(b"D\0")
        for child in sorted(path.rglob("*")):
            if child.is_file() or child.is_symlink():
                _fingerprint_path(digest, child)
        return
    digest.update(b"MISSING\0")


def build_fingerprint(
    name: str,
    *,
    values: Sequence[str] = (),
    paths: Sequence[Path] = (),
) -> str:
    digest = hashlib.sha256()
    digest.update(name.encode("utf-8"))
    digest.update(b"\0")
    for value in values:
        digest.update(value.encode("utf-8"))
        digest.update(b"\0")
    for path in paths:
        _fingerprint_path(digest, path)
    return digest.hexdigest()


def output_stamp(path: Path) -> str:
    stat = path.stat()
    return f"{stat.st_size}:{stat.st_mtime_ns}"


def output_namespace(ctx: Context) -> str:
    return hashlib.sha256(str(ctx.output_dir).encode("utf-8")).hexdigest()[:12]


def build_state_path(key: str) -> Path:
    return RUNTIME_ROOT / "state" / f"{key}.json"


def build_is_current(
    ctx: Context,
    *,
    key: str,
    fingerprint: str,
    outputs: Sequence[Path],
    label: str,
) -> bool:
    if ctx.force or not all(path.exists() for path in outputs):
        return False
    try:
        state = json.loads(build_state_path(key).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    if not isinstance(state, dict):
        return False
    if state.get("fingerprint") != fingerprint:
        return False
    recorded_stamps = state.get("output_stamps")
    if not isinstance(recorded_stamps, dict):
        return False
    for output in outputs:
        try:
            current_stamp = output_stamp(output)
        except OSError:
            return False
        if recorded_stamps.get(str(output)) != current_stamp:
            return False
    ctx.ui.step(f"reuse {label} (up to date)")
    return True


def record_build_state(
    *,
    key: str,
    fingerprint: str,
    outputs: Sequence[Path],
) -> None:
    path = build_state_path(key)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    ensure_dir(path.parent)
    try:
        temporary.write_text(
            json.dumps(
                {
                    "fingerprint": fingerprint,
                    "output_stamps": {
                        str(output): output_stamp(output) for output in outputs
                    },
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        remove_path(temporary)


def kernel_fingerprint(ctx: Context) -> str:
    return build_fingerprint(
        "kernel-v1",
        values=(
            ctx.arch.name,
            ctx.profile,
            ctx.rust_target,
            os.environ.get("ROANIX_RUSTFLAGS", ""),
        ),
        paths=(
            ROOT / "x.py",
            KERNEL_DIR / "Cargo.toml",
            KERNEL_DIR / "Cargo.lock",
            KERNEL_DIR / "rust-toolchain.toml",
            KERNEL_DIR / "build.rs",
            KERNEL_DIR / f"linker-{ctx.arch.name}.ld",
            KERNEL_DIR / ".cargo",
            KERNEL_DIR / "src",
        ),
    )


def userspace_fingerprint(ctx: Context) -> str:
    return build_fingerprint(
        "userspace-v1",
        values=(
            ctx.arch.name,
            JINX.commit,
            *USERSPACE_BUILD_PACKAGES,
            *USERSPACE_PACKAGES,
        ),
        paths=(
            ROOT / "x.py",
            USERLAND_DIR / "Jinxfile",
            USERLAND_DIR / "build-support",
            USERLAND_DIR / "host-recipes",
            USERLAND_DIR / "recipes",
            USERLAND_DIR / "init",
            DRIVERS_DIR,
        ),
    )


def initramfs_fingerprint(ctx: Context) -> str:
    return build_fingerprint(
        "initramfs-v1",
        values=(
            ctx.arch.name,
            ctx.profile,
            userspace_fingerprint(ctx),
            output_stamp(ctx.sysroot),
        ),
    )


def image_fingerprint(ctx: Context, kind: str) -> str:
    return build_fingerprint(
        f"{kind}-image-v1",
        values=(
            ctx.arch.name,
            ctx.profile,
            kernel_fingerprint(ctx),
            initramfs_fingerprint(ctx),
            output_stamp(ctx.kernel_artifact),
            output_stamp(ctx.initramfs),
            json.dumps(DOWNLOADS["limine"].marker(), sort_keys=True),
        ),
        paths=(
            ROOT / "x.py",
            USERLAND_DIR / "distro-files" / "limine.conf",
            USERLAND_DIR / "distro-files" / "splash.jpg",
        ),
    )


def _safe_archive_path(destination: Path, name: str) -> Path:
    pure = PurePosixPath(name)
    if pure.is_absolute() or ".." in pure.parts:
        raise BuildError(f"Archive contains unsafe path: {name!r}")
    resolved = (destination / Path(*pure.parts)).resolve()
    try:
        resolved.relative_to(destination.resolve())
    except ValueError as exc:
        raise BuildError(f"Archive contains unsafe path: {name!r}") from exc
    return resolved


def safe_extract_tar(archive: Path, destination: Path) -> None:
    with tarfile.open(archive, "r:*") as tar:
        for member in tar.getmembers():
            target = _safe_archive_path(destination, member.name)
            if member.isdev() or member.isfifo():
                raise BuildError(
                    f"Archive contains unsupported special file: {member.name!r}"
                )
            if member.issym():
                link = PurePosixPath(member.linkname)
                if link.is_absolute():
                    raise BuildError(
                        f"Archive contains unsafe symlink: {member.name!r}"
                    )
                link_target = (target.parent / Path(*link.parts)).resolve()
                try:
                    link_target.relative_to(destination.resolve())
                except ValueError as exc:
                    raise BuildError(
                        f"Archive contains unsafe symlink: {member.name!r}"
                    ) from exc
            if member.islnk():
                _safe_archive_path(destination, member.linkname)
        if hasattr(tarfile, "fully_trusted_filter"):
            tar.extractall(destination, filter="fully_trusted")
        else:
            tar.extractall(destination)


def _download_cache_path(pin: DownloadPin) -> Path:
    suffix = "".join(Path(urllib.request.url2pathname(pin.url)).suffixes)
    if not suffix:
        suffix = ".archive"
    return RUNTIME_ROOT / "downloads" / f"{pin.name}-{pin.version}-{pin.sha256[:12]}{suffix}"


def fetch_download(ctx: Context, pin: DownloadPin) -> Path:
    cache_path = _download_cache_path(pin)
    if cache_path.is_file():
        digest = sha256_file(cache_path)
        if digest == pin.sha256:
            return cache_path
        remove_path(cache_path)

    ensure_dir(cache_path.parent)
    ctx.ui.step(f"download {pin.name} {pin.version}")
    request = urllib.request.Request(
        pin.url, headers={"User-Agent": f"roanix-x.py/{VERSION}"}
    )
    temporary = cache_path.with_suffix(cache_path.suffix + ".part")
    remove_path(temporary)
    digest = hashlib.sha256()
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            with temporary.open("wb") as output:
                while block := response.read(1024 * 1024):
                    output.write(block)
                    digest.update(block)
    except (HTTPError, URLError, TimeoutError) as exc:
        remove_path(temporary)
        raise BuildError(f"Failed to download {pin.name}: {exc}") from exc
    actual = digest.hexdigest()
    if actual != pin.sha256:
        remove_path(temporary)
        raise BuildError(
            f"{pin.name} checksum mismatch: expected {pin.sha256}, got {actual}"
        )
    temporary.replace(cache_path)
    return cache_path


def _marker_matches(path: Path, pin: DownloadPin) -> bool:
    marker = path / ".x-version.json"
    try:
        value = json.loads(marker.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return False
    return value == pin.marker()


def install_download(ctx: Context, pin: DownloadPin) -> Path:
    destination = RUNTIME_ROOT / pin.directory
    if destination.is_dir() and _marker_matches(destination, pin):
        return destination

    archive = fetch_download(ctx, pin)
    ensure_dir(RUNTIME_ROOT)
    ctx.ui.step(f"extract {pin.name} {pin.version}")
    with tempfile.TemporaryDirectory(prefix=f".{pin.name}-", dir=RUNTIME_ROOT) as temporary:
        temporary_path = Path(temporary)
        safe_extract_tar(archive, temporary_path)
        extracted = temporary_path / pin.archive_root
        if not extracted.is_dir():
            raise BuildError(
                f"{pin.name} archive did not contain {pin.archive_root!r}"
            )
        remove_path(destination)
        shutil.move(str(extracted), destination)
    write_text(
        destination / ".x-version.json",
        json.dumps(pin.marker(), indent=2, sort_keys=True) + "\n",
    )
    return destination


def ensure_limine(ctx: Context) -> Path:
    pin = DOWNLOADS["limine"]
    destination = RUNTIME_ROOT / pin.directory
    if ctx.no_bootstrap and not _marker_matches(destination, pin):
        raise BuildError(
            "Limine is not bootstrapped at the locked version; remove "
            "--no-bootstrap or run 'x.py fetch limine'"
        )
    if not ctx.no_bootstrap:
        destination = install_download(ctx, pin)
    executable = destination / "limine"
    if not executable.exists():
        if ctx.no_bootstrap:
            raise BuildError(f"Missing Limine host tool: {executable}")
        run(
            ctx,
            ["make", "-C", str(destination)],
            step="build Limine host tools",
        )
    required = [
        destination / "limine-uefi-cd.bin",
        destination / "BOOTX64.EFI",
        destination / "BOOTIA32.EFI",
        destination / "BOOTRISCV64.EFI",
    ]
    if ctx.arch.name == "x86_64":
        required += [
            destination / "limine-bios.sys",
            destination / "limine-bios-cd.bin",
        ]
    missing = [path.name for path in required if not path.exists()]
    if missing:
        raise BuildError(f"Locked Limine release is missing: {', '.join(missing)}")
    return destination


def ensure_ovmf(ctx: Context) -> Path:
    pin = DOWNLOADS["ovmf"]
    destination = RUNTIME_ROOT / pin.directory
    if ctx.no_bootstrap and not _marker_matches(destination, pin):
        raise BuildError(
            "OVMF is not bootstrapped at the locked version; remove "
            "--no-bootstrap or run 'x.py fetch ovmf'"
        )
    if not ctx.no_bootstrap:
        destination = install_download(ctx, pin)
    for kind in ("code", "vars"):
        firmware = destination / f"ovmf-{kind}-{ctx.arch.name}.fd"
        if not firmware.is_file():
            raise BuildError(f"Locked OVMF release is missing {firmware.name}")
    return destination


def _git_head(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    completed = _capture(["git", "-C", str(path), "rev-parse", "HEAD"])
    if completed.returncode != 0:
        return None
    return completed.stdout.strip().lower()


def _git_worktree_clean(path: Path) -> bool:
    completed = _capture(
        [
            "git",
            "-C",
            str(path),
            "status",
            "--porcelain",
            "--untracked-files=no",
        ]
    )
    return completed.returncode == 0 and not completed.stdout.strip()


def ensure_jinx(ctx: Context) -> Path:
    pin = JINX
    destination = RUNTIME_ROOT / pin.directory
    if _git_head(destination) == pin.commit and (destination / "jinx").is_file():
        if not _git_worktree_clean(destination):
            raise BuildError(
                "The cached Jinx checkout has local modifications; remove "
                "build/runtime/jinx before rebuilding"
            )
        return destination
    if ctx.no_bootstrap:
        raise BuildError(
            "Jinx is not checked out at the locked commit; remove --no-bootstrap "
            "or run 'x.py fetch jinx'"
        )

    ensure_dir(destination)
    if not (destination / ".git").exists():
        run(ctx, ["git", "init", str(destination)], step="initialize Jinx checkout")

    run(
        ctx,
        [
            "git",
            "-C",
            str(destination),
            "fetch",
            "--depth=1",
            pin.url,
            pin.commit,
        ],
        step=f"fetch Jinx {pin.commit[:12]}",
    )
    run(
        ctx,
        ["git", "-C", str(destination), "checkout", "--detach", pin.commit],
        step="check out locked Jinx commit",
    )
    if _git_head(destination) != pin.commit or not (destination / "jinx").is_file():
        raise BuildError("Jinx checkout does not match the expected commit")
    if not _git_worktree_clean(destination):
        raise BuildError("Jinx checkout contains local modifications")
    return destination


def jinx_environment(ctx: Context) -> Mapping[str, str]:
    path = os.environ.get("PATH", "")
    if shutil.which("wget") is not None:
        return {"PATH": path}

    curl = shutil.which("curl")
    if curl is None:
        raise BuildError("Jinx requires wget, or curl for the built-in fallback")

    host_tools = RUNTIME_ROOT / "jinx-host-tools"
    ensure_dir(host_tools)
    wget = host_tools / "wget"
    script = (
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
        '[ -n "$url" ] || { echo "wget fallback requires a URL" >&2; exit 2; }\n'
        f"set -- {shlex.quote(curl)} -fL\n"
        '[ -n "$output" ] && set -- "$@" -o "$output"\n'
        '[ -n "$user_agent" ] && set -- "$@" -A "$user_agent"\n'
        '[ -n "$insecure" ] && set -- "$@" "$insecure"\n'
        '[ -n "${ca_file-}" ] && set -- "$@" --cacert "$ca_file"\n'
        '[ -n "${certificate-}" ] && set -- "$@" --cert "$certificate"\n'
        '[ -n "${private_key-}" ] && set -- "$@" --key "$private_key"\n'
        'exec "$@" "$url"\n'
    )
    write_text(wget, script, mode=0o755)
    combined = f"{host_tools}:{path}" if path else str(host_tools)
    return {"PATH": combined}


def cargo_arguments(*arguments: str) -> list[str]:
    return ["cargo", *arguments]


def rust_environment(ctx: Context) -> Mapping[str, str]:
    flags = [
        "-Crelocation-model=static",
        "-Cforce-frame-pointers=yes",
    ]
    extra = os.environ.get("ROANIX_RUSTFLAGS")
    if extra:
        flags.append(extra)
    return {
        "CARGO_TARGET_DIR": str(ctx.cargo_target_dir),
        "RUSTFLAGS": " ".join(flags),
    }


def _profile_directory(profile: str) -> str:
    return "debug" if profile == "dev" else profile


def cargo_profile_dir(ctx: Context) -> Path:
    return (
        ctx.cargo_target_dir
        / ctx.rust_target
        / _profile_directory(ctx.profile)
    )


def build_kernel(ctx: Context) -> Path:
    fingerprint = kernel_fingerprint(ctx)
    state_key = (
        f"kernel-{ctx.arch.name}-{ctx.profile}-{output_namespace(ctx)}"
    )
    if build_is_current(
        ctx,
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.kernel_artifact,),
        label=f"kernel ({ctx.arch.name}, {ctx.profile})",
    ):
        return ctx.kernel_artifact

    output_dir = cargo_profile_dir(ctx)
    if ctx.force:
        remove_path(output_dir)
        remove_path(ctx.kernel_artifact)
    ensure_dir(ctx.artifact_dir)
    argv = cargo_arguments(
        "build",
        "--target",
        ctx.rust_target,
        "--profile",
        ctx.profile,
    )
    run(
        ctx,
        argv,
        cwd=KERNEL_DIR,
        env_updates=rust_environment(ctx),
        step=f"build kernel ({ctx.arch.name}, {ctx.profile})",
    )
    preferred = output_dir / "roanix"
    if preferred.is_file():
        source = preferred
    else:
        candidates = sorted(
            path
            for path in output_dir.iterdir()
            if path.is_file() and os.access(path, os.X_OK)
        )
        if not candidates:
            raise BuildError(f"No kernel executable was produced in {output_dir}")
        source = candidates[0]
    copy_file(source, ctx.kernel_artifact)
    record_build_state(
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.kernel_artifact,),
    )
    return ctx.kernel_artifact


def check_kernel(ctx: Context) -> None:
    run(
        ctx,
        cargo_arguments(
            "check",
            "--target",
            ctx.rust_target,
            "--profile",
            ctx.profile,
        ),
        cwd=KERNEL_DIR,
        env_updates=rust_environment(ctx),
        step=f"check kernel ({ctx.arch.name})",
    )


def lint_kernel(ctx: Context) -> None:
    argv = cargo_arguments(
        "clippy",
        "--target",
        ctx.rust_target,
        "--profile",
        ctx.profile,
    )
    argv += ["--", "-D", "warnings"]
    run(
        ctx,
        argv,
        cwd=KERNEL_DIR,
        env_updates=rust_environment(ctx),
        step=f"lint kernel ({ctx.arch.name})",
    )


def format_kernel(ctx: Context, *, check: bool) -> None:
    argv = ["cargo", "fmt", "--all"]
    if check:
        argv += ["--", "--check"]
    run(
        ctx,
        argv,
        cwd=KERNEL_DIR,
        step="check Rust formatting" if check else "format Rust sources",
    )


def available_userland_packages() -> tuple[str, ...]:
    return tuple(
        sorted(
            path.parent.name
            for path in (USERLAND_DIR / "recipes").glob("*/recipe")
            if path.parent.name != "mlibc-source"
        )
    )


def recipe_dependencies(package: str) -> tuple[str, ...]:
    recipe = USERLAND_DIR / "recipes" / package / "recipe"
    dependencies: list[str] = []
    found = False
    for line in recipe.read_text(encoding="utf-8").splitlines():
        if not line.startswith("deps="):
            continue
        if found:
            raise BuildError(f"Multiple deps assignments in {recipe}")
        found = True
        value = line.removeprefix("deps=").strip().strip("\"'")
        if "$" in value or "`" in value:
            raise BuildError(f"Dynamic deps assignment is unsupported in {recipe}")
        dependencies.extend(value.split())
    known = {
        path.parent.name
        for path in (USERLAND_DIR / "recipes").glob("*/recipe")
    }
    unknown = sorted(set(dependencies) - known)
    if unknown:
        raise BuildError(
            f"Unknown dependencies in {recipe}: {', '.join(unknown)}"
        )
    return tuple(dependencies)


def recipe_source(package: str) -> str | None:
    recipe = USERLAND_DIR / "recipes" / package / "recipe"
    for line in recipe.read_text(encoding="utf-8").splitlines():
        if line.startswith("from_source="):
            return line.removeprefix("from_source=").strip().strip("\"'")
    return None


def discard_failed_tarball(package: str, ui: UI) -> None:
    recipe = USERLAND_DIR / "recipes" / package / "recipe"
    assignments: dict[str, str] = {}
    for line in recipe.read_text(encoding="utf-8").splitlines():
        if "=" not in line or line.lstrip().startswith("#"):
            continue
        key, value = line.split("=", 1)
        if key in {"version", "tarball_url"}:
            assignments[key] = value.strip().strip("\"'")
    url = assignments.get("tarball_url")
    if not url:
        return
    url = url.replace("${version}", assignments.get("version", ""))
    cached = USERLAND_DIR / "sources" / Path(url.split("?", 1)[0]).name
    if cached.is_file():
        ui.detail(f"discard failed source download: {_display_path(cached)}")
        remove_path(cached)


def reset_patched_source(package: str, ui: UI) -> None:
    source = recipe_source(package) or package
    patches = USERLAND_DIR / "recipes" / source / "patches"
    if not patches.is_dir():
        return
    source_root = USERLAND_DIR / "sources"
    candidates = (
        source_root / source,
        source_root / f"{source}-clean",
        source_root / f"{source}-workdir",
        source_root / f"{source}.version",
        source_root / f"{source}.patched",
        source_root / f"{source}.prepared",
        source_root / f"{source}.revision",
        source_root / f"{source}.host-revision",
    )
    if any(path.exists() for path in candidates):
        ui.detail(f"refresh patched source: {source}")
    for path in candidates:
        remove_path(path)
    discard_failed_tarball(source, ui)


def package_rebuild_order(package: str) -> tuple[str, ...]:
    packages = available_userland_packages()
    dependencies = {name: recipe_dependencies(name) for name in packages}
    sources = {name: recipe_source(name) for name in packages}
    closure = {package}
    changed = True
    while changed:
        changed = False
        selected_sources = {
            source for name, source in sources.items() if name in closure and source
        }
        for name, source in sources.items():
            if name not in closure and source in selected_sources:
                closure.add(name)
                changed = True
        for name, required in dependencies.items():
            if name not in closure and closure.intersection(required):
                closure.add(name)
                changed = True

    ordered: list[str] = []
    visiting: set[str] = set()

    def visit(name: str) -> None:
        if name in ordered:
            return
        if name in visiting:
            raise BuildError(f"Userspace package dependency cycle at {name}")
        visiting.add(name)
        for dependency in dependencies.get(name, ()):
            if dependency in closure:
                visit(dependency)
        visiting.remove(name)
        ordered.append(name)

    for name in sorted(closure):
        visit(name)
    return tuple(ordered)


def package_dependency_closure(packages: Sequence[str]) -> tuple[str, ...]:
    closure = set(packages)
    pending = list(packages)
    while pending:
        package = pending.pop()
        related = list(recipe_dependencies(package))
        source = recipe_source(package)
        if source:
            related.append(source)
        for dependency in related:
            if dependency not in closure:
                closure.add(dependency)
                pending.append(dependency)
    return tuple(sorted(closure))


def replace_directory(staging: Path, destination: Path, backup_name: str) -> None:
    if not destination.exists():
        staging.rename(destination)
        return

    backup = destination.parent / backup_name
    remove_path(backup)
    destination.rename(backup)
    installed = False
    try:
        staging.rename(destination)
        installed = True
    finally:
        if not installed and backup.exists() and not destination.exists():
            backup.rename(destination)
    remove_path(backup)


def prepare_jinx_build(ctx: Context) -> tuple[Path, Path, Mapping[str, str]]:
    sync_driver_source()
    jinx = ensure_jinx(ctx)
    build_dir = RUNTIME_ROOT / f"jinx-build-{ctx.arch.name}"
    if build_dir.is_symlink():
        raise BuildError(f"Refusing to use symlinked Jinx build directory: {build_dir}")
    ensure_dir(build_dir)
    env = jinx_environment(ctx)
    if not (build_dir / ".jinx-parameters").is_file():
        run(
            ctx,
            [str(jinx / "jinx"), "init", str(USERLAND_DIR), f"ARCH={ctx.arch.name}"],
            cwd=build_dir,
            env_updates=env,
            step=f"initialize userspace build ({ctx.arch.name})",
        )
    return jinx, build_dir, env


def sync_driver_source() -> None:
    staging = USERLAND_DIR / ".drivers.tmp"
    remove_path(staging)
    shutil.copytree(
        DRIVERS_DIR,
        staging,
        ignore=shutil.ignore_patterns("build", "out", "*.o", "*.d", "*.so"),
    )
    remove_path(JINX_DRIVERS_SOURCE)
    staging.rename(JINX_DRIVERS_SOURCE)


def build_sysroot(ctx: Context) -> Path:
    fingerprint = userspace_fingerprint(ctx)
    state_key = f"userspace-{ctx.arch.name}"
    if build_is_current(
        ctx,
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.sysroot,),
        label=f"userspace ({ctx.arch.name})",
    ):
        return ctx.sysroot

    build_dir = RUNTIME_ROOT / f"jinx-build-{ctx.arch.name}"
    if ctx.force:
        remove_path(build_dir)
        remove_path(ctx.sysroot)

    jinx, build_dir, env = prepare_jinx_build(ctx)
    for package in package_dependency_closure(USERSPACE_BUILD_PACKAGES):
        reset_patched_source(package, ctx.ui)
        discard_failed_tarball(package, ctx.ui)
    run(
        ctx,
        [
            str(jinx / "jinx"),
            "update",
            "-b",
            *USERSPACE_BUILD_PACKAGES,
        ],
        cwd=build_dir,
        env_updates=env,
        step=f"build userspace package closure ({ctx.arch.name})",
        capture=False,
    )

    staging = RUNTIME_ROOT / "sysroots" / f".{ctx.arch.name}.tmp"
    remove_path(staging)
    ensure_dir(staging)
    try:
        run(
            ctx,
            [
                str(jinx / "jinx"),
                "install",
                str(staging),
                *USERSPACE_PACKAGES,
            ],
            cwd=build_dir,
            env_updates=env,
            step=f"install userspace sysroot ({ctx.arch.name})",
            capture=False,
        )
        replace_directory(staging, ctx.sysroot, f".{ctx.arch.name}.old")
    finally:
        remove_path(staging)
    record_build_state(
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.sysroot,),
    )
    return ctx.sysroot


def rebuild_userland_package(ctx: Context, package: str) -> Path:
    if package not in available_userland_packages():
        supported = ", ".join(available_userland_packages())
        raise BuildError(
            f"Unknown userspace package {package!r}; choose one of: {supported}"
        )
    expected_sysroot_root = RUNTIME_ROOT / "sysroots"
    for path in (DEFAULT_OUTPUT_DIR, RUNTIME_ROOT, expected_sysroot_root):
        if path.is_symlink():
            raise BuildError(f"Refusing to use symlinked runtime path: {path}")
    if ctx.sysroot.is_symlink():
        raise BuildError(f"Refusing to use symlinked sysroot: {ctx.sysroot}")
    if ctx.sysroot.parent != expected_sysroot_root or ctx.sysroot.name != ctx.arch.name:
        raise BuildError(f"Unexpected sysroot path: {ctx.sysroot}")
    if not ctx.sysroot.is_dir():
        raise BuildError(
            f"Userspace sysroot is missing: {ctx.sysroot}; run "
            f"'x.py build sysroot --arch {ctx.arch.name}' first"
        )

    jinx, build_dir, env = prepare_jinx_build(ctx)
    rebuild_order = package_rebuild_order(package)
    for rebuild_package in package_dependency_closure(rebuild_order):
        reset_patched_source(rebuild_package, ctx.ui)
        discard_failed_tarball(rebuild_package, ctx.ui)
    run(
        ctx,
        [str(jinx / "jinx"), "update", "-b", package],
        cwd=build_dir,
        env_updates=env,
        step=f"update dependencies for {package} ({ctx.arch.name})",
        capture=False,
    )
    run(
        ctx,
        [str(jinx / "jinx"), "rebuild", *rebuild_order],
        cwd=build_dir,
        env_updates=env,
        step=f"rebuild {' '.join(rebuild_order)} ({ctx.arch.name})",
        capture=False,
    )
    staging = RUNTIME_ROOT / "sysroots" / f".{ctx.arch.name}.package.tmp"
    remove_path(staging)
    ensure_dir(staging)
    try:
        run(
            ctx,
            [
                str(jinx / "jinx"),
                "install",
                str(staging),
                *USERSPACE_PACKAGES,
            ],
            cwd=build_dir,
            env_updates=env,
            step=f"assemble updated sysroot with {package} ({ctx.arch.name})",
            capture=False,
        )
        replace_directory(
            staging,
            ctx.sysroot,
            f".{ctx.arch.name}.package.old",
        )
    finally:
        remove_path(staging)
    record_build_state(
        key=f"userspace-{ctx.arch.name}",
        fingerprint=userspace_fingerprint(ctx),
        outputs=(ctx.sysroot,),
    )
    return ctx.sysroot


def pack_initramfs(ctx: Context) -> Path:
    if not ctx.sysroot.is_dir():
        raise BuildError(
            f"Userspace sysroot is missing: {ctx.sysroot}; run "
            f"'x.py build sysroot --arch {ctx.arch.name}'"
        )
    ensure_dir(ctx.initramfs.parent)
    temporary = ctx.initramfs.with_suffix(ctx.initramfs.suffix + ".tmp")
    remove_path(temporary)
    ctx.ui.step(f"pack initramfs ({ctx.arch.name})")

    def root_owned(info: tarfile.TarInfo) -> tarfile.TarInfo:
        info.uid = 0
        info.gid = 0
        info.uname = "root"
        info.gname = "root"
        return info

    try:
        with tarfile.open(
            temporary,
            mode="w:gz",
            format=tarfile.USTAR_FORMAT,
            dereference=False,
            compresslevel=9,
        ) as archive:
            paths = sorted(
                ctx.sysroot.rglob("*"),
                key=lambda item: item.relative_to(ctx.sysroot).as_posix(),
            )
            for path in paths:
                archive.add(
                    path,
                    arcname=path.relative_to(ctx.sysroot).as_posix(),
                    recursive=False,
                    filter=root_owned,
                )
        temporary.replace(ctx.initramfs)
        ctx.initramfs.chmod(0o644)
    finally:
        remove_path(temporary)
    return ctx.initramfs


def build_initramfs(ctx: Context) -> Path:
    build_sysroot(ctx)
    fingerprint = initramfs_fingerprint(ctx)
    state_key = (
        f"initramfs-{ctx.arch.name}-{ctx.profile}-{output_namespace(ctx)}"
    )
    if build_is_current(
        ctx,
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.initramfs,),
        label=f"initramfs ({ctx.arch.name})",
    ):
        return ctx.initramfs
    initramfs = pack_initramfs(ctx)
    record_build_state(
        key=state_key,
        fingerprint=fingerprint,
        outputs=(initramfs,),
    )
    return initramfs


def limine_config_text(*, with_initramfs: bool) -> str:
    path = USERLAND_DIR / "distro-files" / "limine.conf"
    text = path.read_text(encoding="ascii")
    if with_initramfs:
        text += (
            "    module_path: $boot():/boot/roanix-root.tar.gz\n"
            "    module_string: initramfs\n"
        )
    return text


def prepare_iso_root(ctx: Context, limine: Path, initramfs: Path) -> Path:
    root = ctx.iso_root
    remove_path(root)
    ensure_dir(root / "boot" / "limine")
    ensure_dir(root / "EFI" / "BOOT")
    copy_file(ctx.kernel_artifact, root / "boot" / "roanix")
    copy_file(initramfs, root / "boot" / "roanix-root.tar.gz")
    copy_file(
        USERLAND_DIR / "distro-files" / "splash.jpg",
        root / "boot" / "splash.jpg",
    )
    write_text(
        root / "boot" / "limine" / "limine.conf",
        limine_config_text(with_initramfs=True),
    )

    copy_file(
        limine / "limine-uefi-cd.bin",
        root / "boot" / "limine" / "limine-uefi-cd.bin",
    )
    if ctx.arch.name == "x86_64":
        copy_file(
            limine / "limine-bios.sys",
            root / "boot" / "limine" / "limine-bios.sys",
        )
        copy_file(
            limine / "limine-bios-cd.bin",
            root / "boot" / "limine" / "limine-bios-cd.bin",
        )
        copy_file(
            limine / "BOOTX64.EFI",
            root / "EFI" / "BOOT" / "BOOTX64.EFI",
        )
        copy_file(
            limine / "BOOTIA32.EFI",
            root / "EFI" / "BOOT" / "BOOTIA32.EFI",
        )
    elif ctx.arch.name == "riscv64":
        copy_file(
            limine / "BOOTRISCV64.EFI",
            root / "EFI" / "BOOT" / "BOOTRISCV64.EFI",
        )
    else:
        raise BuildError(f"Unsupported image architecture: {ctx.arch.name}")
    return root


def build_iso(ctx: Context) -> Path:
    build_kernel(ctx)
    initramfs = build_initramfs(ctx)
    fingerprint = image_fingerprint(ctx, "iso")
    state_key = f"iso-{ctx.arch.name}-{ctx.profile}-{output_namespace(ctx)}"
    if build_is_current(
        ctx,
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.image_iso,),
        label=f"ISO image ({ctx.arch.name})",
    ):
        return ctx.image_iso

    limine = ensure_limine(ctx)
    root = prepare_iso_root(ctx, limine, initramfs)
    ensure_dir(ctx.image_iso.parent)
    remove_path(ctx.image_iso)
    argv = [
        "xorriso",
        "-as",
        "mkisofs",
        "-R",
        "-r",
        "-J",
        "-V",
        f"ROANIX_{ctx.arch.name.upper()}",
    ]
    if ctx.arch.name == "x86_64":
        argv += [
            "-b",
            "boot/limine/limine-bios-cd.bin",
            "-no-emul-boot",
            "-boot-load-size",
            "4",
            "-boot-info-table",
        ]
    argv += [
        "-hfsplus",
        "-apm-block-size",
        "2048",
        "--efi-boot",
        "boot/limine/limine-uefi-cd.bin",
        "-efi-boot-part",
        "--efi-boot-image",
        "--protective-msdos-label",
        str(root),
        "-o",
        str(ctx.image_iso),
    ]
    try:
        run(
            ctx,
            argv,
            step=f"create ISO ({ctx.arch.name})",
        )
        if ctx.arch.name == "x86_64":
            run(
                ctx,
                [str(limine / "limine"), "bios-install", str(ctx.image_iso)],
                step="install Limine BIOS support",
            )
    finally:
        remove_path(root)
    record_build_state(
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.image_iso,),
    )
    return ctx.image_iso


def mcopy(
    ctx: Context, image_spec: str, source: Path, destination: str, *, step: str | None = None
) -> None:
    run(
        ctx,
        ["mcopy", "-m", "-i", image_spec, str(source), destination],
        step=step,
    )


def build_hdd(ctx: Context) -> Path:
    build_kernel(ctx)
    initramfs = build_initramfs(ctx)
    fingerprint = image_fingerprint(ctx, "hdd")
    state_key = f"hdd-{ctx.arch.name}-{ctx.profile}-{output_namespace(ctx)}"
    if build_is_current(
        ctx,
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.image_hdd,),
        label=f"HDD image ({ctx.arch.name})",
    ):
        return ctx.image_hdd

    limine = ensure_limine(ctx)
    ensure_dir(ctx.image_hdd.parent)
    remove_path(ctx.image_hdd)
    with ctx.image_hdd.open("wb") as disk:
        disk.truncate(128 * 1024 * 1024)

    path = os.environ.get("PATH", "")
    tool_path = f"{path}:/usr/sbin:/sbin" if path else "/usr/sbin:/sbin"
    partition_args = [
        "sgdisk",
        str(ctx.image_hdd),
        "-n",
        "1:2048",
        "-t",
        "1:ef00",
    ]
    if ctx.arch.name == "x86_64":
        partition_args += ["-m", "1"]
    run(
        ctx,
        partition_args,
        env_updates={"PATH": tool_path},
        step=f"partition disk image ({ctx.arch.name})",
    )
    if ctx.arch.name == "x86_64":
        run(
            ctx,
            [str(limine / "limine"), "bios-install", str(ctx.image_hdd)],
            step="install Limine BIOS support",
        )

    image_spec = f"{ctx.image_hdd}@@1M"
    run(
        ctx,
        [
            "mformat",
            "-v",
            "ROANIX",
            "-i",
            image_spec,
            "::",
        ],
        step="format EFI filesystem",
    )
    run(
        ctx,
        [
            "mmd",
            "-i",
            image_spec,
            "::/EFI",
            "::/EFI/BOOT",
            "::/boot",
            "::/boot/limine",
        ],
        step="create boot filesystem layout",
    )
    mcopy(ctx, image_spec, ctx.kernel_artifact, "::/boot/roanix")
    mcopy(ctx, image_spec, initramfs, "::/boot/roanix-root.tar.gz")
    mcopy(
        ctx,
        image_spec,
        USERLAND_DIR / "distro-files" / "splash.jpg",
        "::/boot/splash.jpg",
    )
    config = ctx.work_dir / "limine.conf"
    write_text(
        config,
        limine_config_text(with_initramfs=True),
    )
    mcopy(ctx, image_spec, config, "::/boot/limine/limine.conf")
    if ctx.arch.name == "x86_64":
        mcopy(
            ctx,
            image_spec,
            limine / "limine-bios.sys",
            "::/boot/limine/limine-bios.sys",
        )
        mcopy(ctx, image_spec, limine / "BOOTX64.EFI", "::/EFI/BOOT/BOOTX64.EFI")
        mcopy(ctx, image_spec, limine / "BOOTIA32.EFI", "::/EFI/BOOT/BOOTIA32.EFI")
    elif ctx.arch.name == "riscv64":
        mcopy(
            ctx,
            image_spec,
            limine / "BOOTRISCV64.EFI",
            "::/EFI/BOOT/BOOTRISCV64.EFI",
        )
    else:
        raise BuildError(f"Unsupported image architecture: {ctx.arch.name}")
    record_build_state(
        key=state_key,
        fingerprint=fingerprint,
        outputs=(ctx.image_hdd,),
    )
    return ctx.image_hdd


def _display_path(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return str(path)


def build_target(ctx: Context, target: str) -> list[Path]:
    if target == "kernel":
        return [build_kernel(ctx)]
    if target == "sysroot":
        return [build_sysroot(ctx)]
    if target == "initramfs":
        return [build_initramfs(ctx)]
    if target == "iso":
        return [build_iso(ctx)]
    if target == "hdd":
        return [build_hdd(ctx)]
    raise BuildError(f"Unknown build target: {target}")


def kvm_available() -> bool:
    try:
        descriptor = os.open("/dev/kvm", os.O_RDWR | os.O_CLOEXEC)
    except OSError:
        return False
    os.close(descriptor)
    return True


def qemu_acceleration_overridden(arguments: Sequence[str]) -> bool:
    for index, argument in enumerate(arguments):
        if argument in {"-accel", "-enable-kvm", "-no-kvm"}:
            return True
        if argument.startswith("-accel="):
            return True
        if argument in {"-machine", "-M"}:
            if index + 1 < len(arguments) and "accel=" in arguments[index + 1]:
                return True
        if (
            argument.startswith("-machine=") or argument.startswith("-M=")
        ) and "accel=" in argument:
            return True
    return False


def run_qemu(ctx: Context, *, image: str, firmware: str) -> None:
    if firmware == "bios" and ctx.arch.name != "x86_64":
        raise BuildError("BIOS boot is only available for x86_64")
    artifact = build_iso(ctx) if image == "iso" else build_hdd(ctx)
    argv = [ctx.arch.qemu_binary, "-m", ctx.arch.qemu_memory]
    if image == "iso":
        argv += ["-cdrom", str(artifact)]
    else:
        argv += ["-drive", f"file={artifact},format=raw"]

    if firmware == "bios":
        argv += ["-M", "q35,smm=off"]
        if kvm_available() and not qemu_acceleration_overridden(ctx.qemu_args):
            argv += ["-accel", "kvm", "-cpu", "host,+invtsc"]
        else:
            argv += ["-cpu", "max,+invtsc,+tsc-deadline,+fsgsbase"]
        argv += ["-serial", "stdio"]
    else:
        ovmf = ensure_ovmf(ctx)
        ensure_dir(ctx.runtime_dir)
        variables = ctx.runtime_dir / "ovmf-vars.fd"
        copy_file(ovmf / f"ovmf-vars-{ctx.arch.name}.fd", variables)
        argv += [
            "-drive",
            "if=pflash,unit=0,format=raw,"
            f"file={ovmf / f'ovmf-code-{ctx.arch.name}.fd'},readonly=on",
            "-drive",
            f"if=pflash,unit=1,format=raw,file={variables}",
        ]
        if ctx.arch.name == "x86_64":
            argv += ["-M", "q35"]
            if kvm_available() and not qemu_acceleration_overridden(ctx.qemu_args):
                argv += ["-accel", "kvm", "-cpu", "host,+invtsc"]
            else:
                argv += ["-cpu", "max,+invtsc,+tsc-deadline,+fsgsbase"]
            argv += ["-serial", "stdio"]
        elif ctx.arch.name == "riscv64":
            argv += [
                "-M",
                "virt,acpi=off",
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
                "-serial",
                "stdio",
            ]
    argv += list(ctx.qemu_args)
    run(ctx, argv, step=f"run {ctx.arch.name} in QEMU", capture=False)


def build_docs(ctx: Context, *, kind: str, serve: bool, port: int) -> None:
    if kind == "book":
        argv = ["mdbook", "serve", "--port", str(port)] if serve else ["mdbook", "build"]
        run(
            ctx,
            argv,
            cwd=ROOT / "book",
            step="serve the Roanix book" if serve else "build the Roanix book",
            capture=not serve,
        )
        return

    run(
        ctx,
        cargo_arguments(
            "doc",
            "--no-deps",
            "--target",
            ctx.rust_target,
        ),
        cwd=KERNEL_DIR,
        env_updates=rust_environment(ctx),
        step=f"build kernel API documentation ({ctx.arch.name})",
    )
    if serve:
        doc_dir = ctx.cargo_target_dir / ctx.rust_target / "doc"
        run(
            ctx,
            [sys.executable, "-m", "http.server", str(port)],
            cwd=doc_dir,
            step=f"serve API documentation on http://127.0.0.1:{port}",
            capture=False,
        )


def _safe_clean_root(path: Path) -> None:
    resolved = path.resolve()
    forbidden = {
        Path("/"),
        ROOT.resolve(),
        ROOT.parent.resolve(),
        Path.home().resolve(),
    }
    if resolved in forbidden:
        raise BuildError(f"Refusing to remove unsafe output directory: {resolved}")
    try:
        resolved.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise BuildError(
            f"Refusing to clean an output directory outside the repository: {resolved}"
        ) from exc
    remove_path(resolved)


def clean_output_dir(ctx: Context) -> None:
    output = ctx.output_dir.resolve()
    if output != DEFAULT_OUTPUT_DIR.resolve():
        _safe_clean_root(output)
        return
    if not output.is_dir():
        return

    runtime = RUNTIME_ROOT.resolve()
    for child in output.iterdir():
        if child.resolve() == runtime:
            continue
        remove_path(child)


def clean(ctx: Context, *, all_files: bool) -> None:
    ctx.ui.step(f"remove build outputs from {_display_path(ctx.output_dir)}")
    clean_output_dir(ctx)
    for pattern in ("roanix-*.iso", "roanix-*.hdd"):
        for path in ROOT.glob(pattern):
            remove_path(path)
    remove_path(ROOT / "iso_root")
    for path in KERNEL_DIR.glob("roanix-*"):
        remove_path(path)
    remove_path(KERNEL_DIR / "target")
    if all_files:
        ctx.ui.step("remove downloaded tools and userspace caches")
        remove_path(RUNTIME_ROOT)
        remove_path(LEGACY_STORE_DIR)
        remove_path(ROOT / "book" / "book")


def fetch_resources(ctx: Context, resources: Sequence[str]) -> None:
    fetch_ctx = replace(ctx, no_bootstrap=False)
    selected = list(resources) or ["limine", "ovmf", "jinx"]
    for resource in selected:
        if resource in DOWNLOADS:
            install_download(fetch_ctx, DOWNLOADS[resource])
        elif resource == "jinx":
            ensure_jinx(fetch_ctx)
        else:
            raise BuildError(
                f"Unknown resource {resource!r}; choose limine, ovmf, or jinx"
            )


def _tool_version(tool: str) -> str | None:
    completed = subprocess.run(
        [tool, "--version"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if completed.returncode != 0 or not completed.stdout:
        return None
    return completed.stdout.splitlines()[0].strip()


def doctor(ctx: Context, *, target: str, json_output: bool) -> bool:
    kernel_tools = {"cargo", "rustup", "git"}
    userspace_tools = {
        "bash",
        "awk",
        "find",
        "git",
        "grep",
        "gzip",
        "make",
        "sed",
        "tar",
        "zstd",
    }
    tools_by_target = {
        "kernel": kernel_tools,
        "hdd": kernel_tools
        | userspace_tools
        | {"sgdisk", "mformat", "mmd", "mcopy"},
        "iso": kernel_tools | userspace_tools | {"xorriso"},
        "run": kernel_tools
        | userspace_tools
        | {"sgdisk", "mformat", "mmd", "mcopy", ctx.arch.qemu_binary},
        "sysroot": userspace_tools,
        "docs": {"cargo", "rustup", "mdbook"},
    }
    if target == "all":
        required = set().union(*tools_by_target.values())
    else:
        required = tools_by_target[target]
    result: dict[str, Any] = {
        "host": platform.platform(),
        "python": platform.python_version(),
        "target": target,
        "architecture": ctx.arch.name,
        "rust_target": ctx.rust_target,
        "tools": {},
        "problems": [],
    }
    if platform.system() != "Linux":
        result["problems"].append("Roanix builds currently require a Linux host")

    for tool in sorted(required):
        path = shutil.which(tool)
        entry: dict[str, Any] = {"path": path}
        if path:
            entry["version"] = _tool_version(tool)
        else:
            result["problems"].append(f"missing tool: {tool}")
        result["tools"][tool] = entry

    downloader = shutil.which("wget") or shutil.which("curl")
    if target in {"hdd", "iso", "run", "sysroot", "all"} and not downloader:
        result["problems"].append("missing tool: wget or curl")
    result["downloader"] = downloader

    if shutil.which("rustup") and target in {
        "kernel",
        "hdd",
        "iso",
        "run",
        "docs",
        "all",
    }:
        installed = subprocess.run(
            ["rustup", "target", "list", "--installed"],
            cwd=KERNEL_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        targets = installed.stdout.splitlines() if installed.returncode == 0 else []
        result["installed_rust_targets"] = targets
        if ctx.rust_target not in targets:
            result["problems"].append(
                f"Rust target is not installed: {ctx.rust_target}"
            )

    if json_output:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"Host:        {result['host']}")
        print(f"Python:      {result['python']}")
        print(f"Build check: {target} ({ctx.arch.name})")
        for tool, entry in result["tools"].items():
            if entry["path"]:
                version = f" - {entry['version']}" if entry.get("version") else ""
                print(f"  [ok]      {tool}: {entry['path']}{version}")
            else:
                hint = PACKAGE_HINTS.get(tool)
                suffix = f" (package: {hint})" if hint else ""
                print(f"  [missing] {tool}{suffix}")
        if target in {"hdd", "iso", "run", "sysroot", "all"}:
            status = "[ok]" if downloader else "[missing]"
            print(f"  {status:<9} wget or curl: {downloader or 'not found'}")
        if result["problems"]:
            print("\nProblems:")
            for problem in result["problems"]:
                print(f"  - {problem}")
        else:
            print("\nReady.")
    return not result["problems"]


def show_config(ctx: Context, *, json_output: bool) -> None:
    data = {
        "root": str(ROOT),
        "architecture": ctx.arch.name,
        "profile": ctx.profile,
        "rust_target": ctx.rust_target,
        "output_dir": str(ctx.output_dir),
        "artifact_dir": str(ctx.artifact_dir),
        "runtime_dir": str(RUNTIME_ROOT),
    }
    if json_output:
        print(json.dumps(data, indent=2, sort_keys=True))
        return
    print(f"Architecture:       {ctx.arch.name}")
    print(f"Rust target:        {ctx.rust_target}")
    print(f"Profile:            {ctx.profile}")
    print(f"Output directory:   {_display_path(ctx.output_dir)}")
    print(f"Artifact directory: {_display_path(ctx.artifact_dir)}")
    print(f"Runtime directory:  {_display_path(RUNTIME_ROOT)}")


def _common_parser(*, include_force: bool = False) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False, argument_default=argparse.SUPPRESS)
    parser.add_argument(
        "--arch",
        choices=tuple(ARCHITECTURES),
        help="Target architecture.",
    )
    parser.add_argument(
        "--profile",
        "--rust-profile",
        dest="profile",
        choices=SUPPORTED_PROFILES,
        help="Cargo profile.",
    )
    if include_force:
        parser.add_argument(
            "--force",
            action="store_true",
            help="Rebuild even when outputs are up to date.",
        )
    parser.add_argument(
        "--rust-target",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--out-dir",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--no-bootstrap",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--color",
        choices=("auto", "always", "never"),
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--no-color",
        dest="color",
        action="store_const",
        const="never",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("-q", "--quiet", action="store_true", help="Only print errors.")
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Show commands and stream their output.",
    )
    parser.add_argument(
        "--timings",
        "--show-time",
        dest="show_time",
        action="store_true",
        help="Show total command duration.",
    )
    parser.add_argument(
        "--qemu-flags",
        help=argparse.SUPPRESS,
    )
    return parser


def build_parser() -> tuple[argparse.ArgumentParser, dict[str, argparse.ArgumentParser]]:
    common = _common_parser()
    build_common = _common_parser(include_force=True)
    parser = argparse.ArgumentParser(
        prog="x.py",
        parents=[build_common],
        formatter_class=HelpFormatter,
        description="Build, inspect, and run Roanix.",
        epilog="""Examples:
  ./x.py                         Build the default x86_64 HDD image
  ./x.py build iso --arch riscv64 --profile release
  ./x.py build hdd --force
  ./x.py package init
  ./x.py run -- --no-reboot
  ./x.py doctor
  ./x.py help build""",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {VERSION}")
    subparsers = parser.add_subparsers(dest="command", metavar="COMMAND")
    commands: dict[str, argparse.ArgumentParser] = {}

    build = subparsers.add_parser(
        "build",
        parents=[build_common],
        formatter_class=HelpFormatter,
        help="Build a kernel, image, or userspace artifact.",
        description="Build one Roanix artifact.",
        epilog="""Examples:
  ./x.py build                   Build the kernel only
  ./x.py build hdd               Build the bootable HDD image
  ./x.py build iso --arch riscv64
  ./x.py build initramfs --profile release
  ./x.py build hdd --force       Rebuild every dependency""",
    )
    build.add_argument(
        "target",
        nargs="?",
        choices=("kernel", "hdd", "iso", "sysroot", "initramfs"),
        default="kernel",
        help="Artifact to build.",
    )
    commands["build"] = build

    run_parser = subparsers.add_parser(
        "run",
        parents=[build_common],
        formatter_class=HelpFormatter,
        help="Build and boot Roanix in QEMU.",
        description="Build an image and launch it using UEFI firmware or BIOS.",
        epilog="""Examples:
  ./x.py run
  ./x.py run iso --arch riscv64
  ./x.py run hdd --firmware bios
  ./x.py run -- --no-reboot -d int""",
    )
    run_parser.add_argument(
        "image",
        nargs="?",
        choices=("hdd", "iso"),
        default="hdd",
        help="Image format to boot.",
    )
    run_parser.add_argument(
        "--firmware",
        choices=("uefi", "bios"),
        default="uefi",
        help="Firmware mode.",
    )
    commands["run"] = run_parser

    package_parser = subparsers.add_parser(
        "package",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Rebuild a userspace package and reinstall the sysroot.",
        description=(
            "Rebuild one Jinx package plus affected reverse dependencies, "
            "then replace the existing architecture sysroot."
        ),
        epilog="""Examples:
  ./x.py package init
  ./x.py package bash --arch riscv64
  ./x.py package mlibc""",
    )
    package_parser.add_argument(
        "package",
        choices=available_userland_packages(),
        help="Userspace package recipe to rebuild.",
    )
    commands["package"] = package_parser

    check = subparsers.add_parser(
        "check",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Type-check the kernel.",
    )
    commands["check"] = check

    lint = subparsers.add_parser(
        "lint",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Run Clippy with warnings denied.",
    )
    commands["lint"] = lint

    fmt = subparsers.add_parser(
        "fmt",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Format Rust sources.",
    )
    fmt.add_argument("--check", action="store_true", help="Check without modifying files.")
    commands["fmt"] = fmt

    docs = subparsers.add_parser(
        "docs",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Build or serve project documentation.",
    )
    docs.add_argument(
        "kind",
        nargs="?",
        choices=("book", "rust"),
        default="book",
        help="Documentation set.",
    )
    docs.add_argument("--serve", action="store_true", help="Start a preview server.")
    docs.add_argument("--port", type=int, default=8080, help="Preview server port.")
    commands["docs"] = docs

    clean_parser = subparsers.add_parser(
        "clean",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Remove generated files.",
    )
    clean_parser.add_argument(
        "--all",
        action="store_true",
        help="Also remove downloads, firmware, Jinx, and userspace caches.",
    )
    commands["clean"] = clean_parser

    fetch = subparsers.add_parser(
        "fetch",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Fetch bootloader, firmware, and userspace tooling.",
    )
    fetch.add_argument(
        "resources",
        nargs="*",
        choices=("limine", "ovmf", "jinx"),
        help="Resources to fetch; all are fetched by default.",
    )
    commands["fetch"] = fetch

    doctor_parser = subparsers.add_parser(
        "doctor",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Diagnose the host build environment.",
    )
    doctor_parser.add_argument(
        "target",
        nargs="?",
        choices=("kernel", "hdd", "iso", "run", "sysroot", "docs", "all"),
        default="hdd",
        help="Workflow whose dependencies should be checked.",
    )
    doctor_parser.add_argument(
        "--json", dest="json_output", action="store_true", help="Emit machine-readable JSON."
    )
    commands["doctor"] = doctor_parser

    config = subparsers.add_parser(
        "config",
        parents=[common],
        formatter_class=HelpFormatter,
        help="Show the fully resolved build configuration.",
    )
    config.add_argument(
        "--json", dest="json_output", action="store_true", help="Emit machine-readable JSON."
    )
    commands["config"] = config

    help_parser = subparsers.add_parser(
        "help",
        formatter_class=HelpFormatter,
        help="Show help for a command.",
    )
    help_parser.add_argument("topic", nargs="?", help="Command name.")
    commands["help"] = help_parser
    return parser, commands


def rewrite_legacy_command(argv: Sequence[str]) -> list[str]:
    rewritten = list(argv)
    index = 0
    while index < len(rewritten):
        token = rewritten[index]
        if token == "--":
            break
        if token in OPTIONS_WITH_VALUES:
            index += 2
            continue
        if any(token.startswith(option + "=") for option in OPTIONS_WITH_VALUES):
            index += 1
            continue
        if token.startswith("-"):
            index += 1
            continue
        replacement = LEGACY_COMMANDS.get(token)
        if replacement:
            return rewritten[:index] + replacement + rewritten[index + 1 :]
        break
    return rewritten


def main(argv: Sequence[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    if not raw_argv:
        raw_argv = ["build", "hdd"]
    raw_argv = rewrite_legacy_command(raw_argv)
    qemu_passthrough: list[str] = []
    if "--" in raw_argv:
        separator = raw_argv.index("--")
        qemu_passthrough = raw_argv[separator + 1 :]
        raw_argv = raw_argv[:separator]
    parser, commands = build_parser()
    args = parser.parse_args(raw_argv)
    if args.command is None:
        raw_argv += ["build", "hdd"]
        args = parser.parse_args(raw_argv)
    if args.command == "help":
        if args.topic is None:
            parser.print_help()
            return 0
        command_parser = commands.get(args.topic)
        if command_parser is None:
            parser.error(f"unknown help topic: {args.topic}")
        command_parser.print_help()
        return 0

    ctx: Context | None = None
    try:
        ctx = make_context(args)
        legacy_qemu_flags = getattr(args, "qemu_flags", None)
        if qemu_passthrough and args.command != "run":
            raise BuildError("Arguments after '--' are only supported by the run command")
        qemu_args = qemu_passthrough
        if legacy_qemu_flags:
            qemu_args = shlex.split(legacy_qemu_flags) + qemu_args
        ctx = replace(ctx, qemu_args=tuple(qemu_args))
        command = args.command
        if ctx.force and command not in {"build", "run"}:
            raise BuildError("--force is only supported by build and run commands")
        machine_output = (
            (command == "config" and args.json_output)
            or (command == "doctor" and args.json_output)
        )
        if not machine_output:
            ctx.ui.heading(command)

        if command == "build":
            artifacts = build_target(ctx, args.target)
            for artifact in artifacts:
                ctx.ui.detail(f"artifact: {_display_path(artifact)}")
        elif command == "run":
            run_qemu(ctx, image=args.image, firmware=args.firmware)
        elif command == "package":
            sysroot = rebuild_userland_package(ctx, args.package)
            ctx.ui.detail(f"sysroot: {_display_path(sysroot)}")
        elif command == "check":
            check_kernel(ctx)
        elif command == "lint":
            lint_kernel(ctx)
        elif command == "fmt":
            format_kernel(ctx, check=args.check)
        elif command == "docs":
            build_docs(ctx, kind=args.kind, serve=args.serve, port=args.port)
        elif command == "clean":
            clean(ctx, all_files=args.all)
        elif command == "fetch":
            fetch_resources(ctx, args.resources)
        elif command == "doctor":
            return 0 if doctor(ctx, target=args.target, json_output=args.json_output) else 1
        elif command == "config":
            show_config(ctx, json_output=args.json_output)
        else:
            raise BuildError(f"Unknown command: {command}")
        if command != "config":
            ctx.ui.success(f"{command} completed", show_time=ctx.show_time)
        return 0
    except BuildError as exc:
        if ctx is not None:
            ctx.ui.error(str(exc))
        else:
            UI(color="auto", quiet=False, verbose=False).error(str(exc))
        return 1
    except KeyboardInterrupt:
        if ctx is not None:
            ctx.ui.error("interrupted")
        else:
            print("error: interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
