#!/usr/bin/env python3

from __future__ import annotations

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
import time
import urllib.request
from dataclasses import dataclass, field, replace
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Mapping, Sequence
from urllib.error import HTTPError, URLError

if sys.version_info < (3, 11):
    sys.stderr.write("==> ERROR: xtool requires Python 3.11 or newer\n")
    raise SystemExit(2)


VERSION = "3.0"

# --------------------------------------------------------------------------
# Layout
# --------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parent
KERNEL_DIR = ROOT / "kernel"
DRIVERS_DIR = ROOT / "drivers"
RUST_DRIVER_MODULES = {
    "x86_64": (
        ("acpi", "acpi.ko"),
        ("ioapic", "ioapic.ko"),
        ("pci", "pci.ko"),
        ("nvme", "storage-nvme.ko"),
        ("tmpfs", "tmpfs.ko"),
        ("devfs", "devfs.ko"),
        ("console", "console.ko"),
        ("pty", "pty.ko"),
        ("uart8250", "uart8250.ko"),
        ("special", "special.ko"),
    ),
    "riscv64": (
        ("fdt", "fdt.ko"),
        ("plic", "plic.ko"),
        ("pci", "pci.ko"),
        ("nvme", "storage-nvme.ko"),
        ("tmpfs", "tmpfs.ko"),
        ("devfs", "devfs.ko"),
        ("console", "console.ko"),
        ("pty", "pty.ko"),
        ("uart8250", "uart8250.ko"),
        ("special", "special.ko"),
    ),
}
BOOK_DIR = ROOT / "book"
USERLAND_DIR = ROOT / "userland"
RECIPES_DIR = USERLAND_DIR / "recipes"
DISTRO_FILES_DIR = USERLAND_DIR / "distro-files"
SYSTEM_MANIFEST = USERLAND_DIR / "system.list"

BUILD_DIR = ROOT / "build"

CACHE_DIR = BUILD_DIR / "cache"          # pinned, immutable, arch independent
DOWNLOADS_DIR = CACHE_DIR / "downloads"
CARGO_DIR = BUILD_DIR / "cargo"          # CARGO_TARGET_DIR; Cargo namespaces
CONFIG_FILE = BUILD_DIR / "config.json"  # saved defaults, not a build artefact

DEFAULT_ARCH = "x86_64"
DEFAULT_PROFILE = "dev"
PROFILES = ("dev", "release")

# Fallbacks used only when userland/system.list is missing.
FALLBACK_SYSTEM_PACKAGES = (
    "linux-headers",
    "bash",
    "coreutils",
    "stress-ng",
    "python",
    "util-roanix",
)
FALLBACK_EXTRA_BUILD_PACKAGES = ("mlibc-headers", "mlibc", "ncurses", "readline")

# Distro package names for the host tools xtool needs, used by `x.py doctor`.
TOOL_PACKAGES = {
    "cargo": {"apt": "rustup", "pacman": "rustup", "apk": "rust", "dnf": "rustup"},
    "rustup": {"apt": "rustup", "pacman": "rustup", "apk": "rust", "dnf": "rustup"},
    "git": {"apt": "git", "pacman": "git", "apk": "git", "dnf": "git"},
    "make": {"apt": "make", "pacman": "make", "apk": "make", "dnf": "make"},
    "xorriso": {"apt": "xorriso", "pacman": "xorriso", "apk": "xorriso", "dnf": "xorriso"},
    "sgdisk": {"apt": "gdisk", "pacman": "gptfdisk", "apk": "sgdisk", "dnf": "gdisk"},
    "mformat": {"apt": "mtools", "pacman": "mtools", "apk": "mtools", "dnf": "mtools"},
    "mmd": {"apt": "mtools", "pacman": "mtools", "apk": "mtools", "dnf": "mtools"},
    "mcopy": {"apt": "mtools", "pacman": "mtools", "apk": "mtools", "dnf": "mtools"},
    "zstd": {"apt": "zstd", "pacman": "zstd", "apk": "zstd", "dnf": "zstd"},
    "tar": {"apt": "tar", "pacman": "tar", "apk": "tar", "dnf": "tar"},
    "gzip": {"apt": "gzip", "pacman": "gzip", "apk": "gzip", "dnf": "gzip"},
    "sed": {"apt": "sed", "pacman": "sed", "apk": "sed", "dnf": "sed"},
    "awk": {"apt": "gawk", "pacman": "gawk", "apk": "gawk", "dnf": "gawk"},
    "grep": {"apt": "grep", "pacman": "grep", "apk": "grep", "dnf": "grep"},
    "find": {"apt": "findutils", "pacman": "findutils", "apk": "findutils", "dnf": "findutils"},
    "bash": {"apt": "bash", "pacman": "bash", "apk": "bash", "dnf": "bash"},
    "curl": {"apt": "curl", "pacman": "curl", "apk": "curl", "dnf": "curl"},
    "mdbook": {"apt": "mdbook", "pacman": "mdbook", "apk": "mdbook", "dnf": "mdbook"},
    "qemu-system-x86_64": {
        "apt": "qemu-system-x86",
        "pacman": "qemu-system-x86",
        "apk": "qemu-system-x86_64",
        "dnf": "qemu-system-x86",
    },
    "qemu-system-riscv64": {
        "apt": "qemu-system-misc",
        "pacman": "qemu-system-riscv",
        "apk": "qemu-system-riscv64",
        "dnf": "qemu-system-riscv",
    },
}


# --------------------------------------------------------------------------
# Errors
# --------------------------------------------------------------------------


class Failure(Exception):
    """A user-facing failure, optionally carrying a suggested fix."""

    def __init__(self, message: str, *, hint: str | None = None) -> None:
        super().__init__(message)
        self.hint = hint


# --------------------------------------------------------------------------
# Logging
# --------------------------------------------------------------------------

QUIET, NORMAL, VERBOSE = -1, 0, 1


class Log:
    """makepkg-flavoured terminal output."""

    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    MAGENTA = "\033[1;35m"
    CYAN = "\033[1;36m"

    def __init__(self, *, color: str = "auto", level: int = NORMAL) -> None:
        if color == "always":
            self.color = True
        elif color == "never":
            self.color = False
        else:
            self.color = sys.stdout.isatty() and "NO_COLOR" not in os.environ
        self.level = level

    # -- primitives --------------------------------------------------------

    def paint(self, text: str, *styles: str) -> str:
        if not self.color or not styles:
            return text
        return "".join(styles) + text + self.RESET

    def _write(self, text: str, *, stream: Any = None) -> None:
        target = stream or sys.stdout
        target.write(text + "\n")
        target.flush()

    # -- levels ------------------------------------------------------------

    def msg(self, text: str) -> None:
        """``==> Doing a big thing``"""
        if self.level < NORMAL:
            return
        self._write(self.paint("==>", self.GREEN) + " " + self.paint(text, self.BOLD))

    def msg2(self, text: str) -> None:
        """``  -> a step within a big thing``"""
        if self.level < NORMAL:
            return
        self._write(
            "  " + self.paint("->", self.BLUE) + " " + self.paint(text, self.BOLD)
        )

    def msg3(self, text: str) -> None:
        """``     * a minor detail``"""
        if self.level < NORMAL:
            return
        self._write("     " + self.paint("* " + text, self.DIM))

    def plain(self, text: str = "") -> None:
        if self.level < NORMAL:
            return
        self._write(text)

    def field(self, name: str, value: str, *, width: int = 16) -> None:
        if self.level < NORMAL:
            return
        self._write(
            "  "
            + self.paint("->", self.BLUE)
            + " "
            + self.paint(name.ljust(width), self.BOLD)
            + value
        )

    def output(self, text: str) -> None:
        """A line of captured subprocess output."""
        # Leave lines that already carry their own escape sequences alone,
        # otherwise the tool's colours and our dimming fight each other.
        body = text if "\033" in text else self.paint(text, self.DIM)
        self._write("    " + body)

    def warn(self, text: str) -> None:
        self._write(
            self.paint("==> WARNING:", self.YELLOW) + " " + self.paint(text, self.BOLD),
            stream=sys.stderr,
        )

    def error(self, text: str) -> None:
        self._write(
            self.paint("==> ERROR:", self.RED) + " " + self.paint(text, self.BOLD),
            stream=sys.stderr,
        )

    def hint(self, text: str) -> None:
        self._write(
            "  " + self.paint("->", self.CYAN) + " " + text, stream=sys.stderr
        )

    def detail(self, text: str) -> None:
        """A dimmed continuation line belonging to the previous error."""
        self._write(self.paint(text, self.DIM), stream=sys.stderr)

    def separator(self) -> None:
        self._write("", stream=sys.stderr)

    def note(self, text: str) -> None:
        if self.level < NORMAL:
            return
        self._write("  " + self.paint("->", self.CYAN) + " " + text)

    def debug(self, text: str) -> None:
        if self.level < VERBOSE:
            return
        self._write("    " + self.paint("$ " + text, self.DIM))

    def finished(self, what: str, seconds: float) -> None:
        if self.level < NORMAL:
            return
        self._write(
            self.paint("==>", self.GREEN)
            + " "
            + self.paint(f"Finished {what}", self.BOLD)
            + " "
            + self.paint(f"({human_time(seconds)})", self.DIM)
        )


def human_time(seconds: float) -> str:
    if seconds < 1:
        return f"{seconds * 1000:.0f}ms"
    if seconds < 60:
        return f"{seconds:.1f}s"
    minutes, rest = divmod(int(seconds), 60)
    if minutes < 60:
        return f"{minutes}m {rest:02d}s"
    hours, minutes = divmod(minutes, 60)
    return f"{hours}h {minutes:02d}m"


def human_size(size: float) -> str:
    for unit in ("B", "KiB", "MiB", "GiB"):
        if size < 1024 or unit == "GiB":
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} GiB"


def rel(path: Path) -> str:
    """Render a path relative to the repository root when possible."""
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(path)


def tree_size(path: Path) -> int:
    total = 0
    if path.is_file():
        return path.stat().st_size
    for item in path.rglob("*"):
        if item.is_file() and not item.is_symlink():
            try:
                total += item.stat().st_size
            except OSError:
                pass
    return total


def age(path: Path) -> str:
    try:
        delta = time.time() - path.stat().st_mtime
    except OSError:
        return "unknown"
    if delta < 90:
        return "just now"
    if delta < 3600:
        return f"{delta / 60:.0f} minutes ago"
    if delta < 86400:
        return f"{delta / 3600:.0f} hours ago"
    return f"{delta / 86400:.0f} days ago"


# --------------------------------------------------------------------------
# Persistent settings
# --------------------------------------------------------------------------

SETTINGS_KEYS = {
    "arch": "Default target architecture (x86_64, riscv64)",
    "profile": "Default Cargo profile (dev, release)",
    "image": "Default image format for run/build (hdd, iso)",
    "firmware": "Default firmware for run (uefi, bios)",
    "jobs": "Parallelism handed to Cargo and Jinx",
    "qemu-args": "Extra QEMU arguments always appended",
}


def load_settings() -> dict[str, str]:
    try:
        raw = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(raw, dict):
        return {}
    return {str(k): str(v) for k, v in raw.items() if k in SETTINGS_KEYS}


def save_settings(settings: Mapping[str, str]) -> None:
    CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    temporary = CONFIG_FILE.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(dict(sorted(settings.items())), indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(CONFIG_FILE)


def setting(
    settings: Mapping[str, str], key: str, cli: Any, default: Any = None
) -> Any:
    """Resolve one option: CLI flag > environment > saved config > default."""
    if cli is not None:
        return cli
    env = os.environ.get("ROANIX_" + key.replace("-", "_").upper())
    if env:
        return env
    if key in settings:
        return settings[key]
    return default


# --------------------------------------------------------------------------
# Architectures
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Arch:
    name: str
    rust_target: str
    qemu: str
    memory: str
    efi_files: tuple[str, ...]
    firmwares: tuple[str, ...]


ARCHES: Mapping[str, Arch] = {
    "x86_64": Arch(
        name="x86_64",
        rust_target="x86_64-unknown-none",
        qemu="qemu-system-x86_64",
        memory="4G",
        efi_files=("BOOTX64.EFI", "BOOTIA32.EFI"),
        firmwares=("uefi", "bios"),
    ),
    "riscv64": Arch(
        name="riscv64",
        rust_target="riscv64gc-unknown-none-elf",
        qemu="qemu-system-riscv64",
        memory="4G",
        efi_files=("BOOTRISCV64.EFI",),
        firmwares=("uefi",),
    ),
}


# --------------------------------------------------------------------------
# Pinned host resources
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Tarball:
    name: str
    version: str
    url: str
    sha256: str
    root: str
    directory: str

    def marker(self) -> dict[str, str]:
        return {
            "name": self.name,
            "version": self.version,
            "url": self.url,
            "sha256": self.sha256,
            "root": self.root,
        }


@dataclass(frozen=True)
class GitPin:
    name: str
    url: str
    commit: str
    directory: str


TARBALLS: Mapping[str, Tarball] = {
    "limine": Tarball(
        name="limine",
        version="12.5.0",
        url=(
            "https://github.com/limine-bootloader/limine/releases/download/"
            "v12.5.0/limine-binary.tar.gz"
        ),
        sha256="8bc0d0f2a2cd0e212f529c57d8a2033a25996dcdd9f58c916b7bf5594a0282eb",
        root="limine-binary",
        directory="limine",
    ),
    "ovmf": Tarball(
        name="ovmf",
        version="20260531T041444Z",
        url=(
            "https://github.com/osdev0/edk2-ovmf-nightly/releases/download/"
            "20260531T041444Z/edk2-ovmf.tar.gz"
        ),
        sha256="534384e4b971730143f54708493fe43932eabbfd275094b636561fb2028bde62",
        root="edk2-ovmf",
        directory="edk2-ovmf",
    ),
}

JINX_PIN = GitPin(
    name="jinx",
    url="https://github.com/Mintsuki/Jinx.git",
    commit="287ceaf9a2c08b43dbc56d38d8b815fd990a2192",
    directory="jinx",
)


# --------------------------------------------------------------------------
# Context
# --------------------------------------------------------------------------


@dataclass
class Context:
    arch: Arch
    profile: str
    log: Log
    jobs: int
    force: bool = False
    offline: bool = False
    dry_run: bool = False
    assume_yes: bool = False
    qemu_args: tuple[str, ...] = ()
    started: float = field(default_factory=time.monotonic)

    # -- paths -------------------------------------------------------------
    #
    #   build/<arch>/out/<profile>/   kernel, initramfs, bootable images
    #   build/<arch>/jinx/            Jinx build directory and packages
    #   build/<arch>/sysroot/         installed userland
    #   build/<arch>/firmware/        per-machine UEFI variables
    #   build/<arch>/state/           xtool's incremental fingerprints
    #   build/<arch>/tmp/<profile>/   scratch space

    @property
    def arch_root(self) -> Path:
        return BUILD_DIR / self.arch.name

    @property
    def artifacts(self) -> Path:
        return self.arch_root / "out" / self.profile

    @property
    def work(self) -> Path:
        return self.arch_root / "tmp" / self.profile

    @property
    def state_dir(self) -> Path:
        return self.arch_root / "state"

    @property
    def cargo_target(self) -> Path:
        # Shared on purpose: Cargo namespaces by target triple internally and
        # reuses host-side build-script output across architectures.
        return CARGO_DIR

    @property
    def cargo_arch_dir(self) -> Path:
        return CARGO_DIR / self.arch.rust_target

    @property
    def kernel_binary(self) -> Path:
        return self.artifacts / "roanix"

    @property
    def initramfs(self) -> Path:
        return self.artifacts / f"roanix-{self.arch.name}.initramfs.tar.gz"

    @property
    def sysroot(self) -> Path:
        return self.arch_root / "sysroot"

    @property
    def jinx_build(self) -> Path:
        return self.arch_root / "jinx"

    @property
    def firmware(self) -> Path:
        return self.arch_root / "firmware"

    def image(self, kind: str) -> Path:
        return self.artifacts / f"roanix-{self.arch.name}.{kind}"

    def env(self, updates: Mapping[str, str] | None = None) -> dict[str, str]:
        environment = os.environ.copy()
        if updates:
            environment.update(updates)
        return environment


# --------------------------------------------------------------------------
# Running commands
# --------------------------------------------------------------------------

#: Streamed output is passed through this to give Jinx/Cargo lines some colour.
LineFilter = Callable[[str], tuple[str, str | None]]


def plain_filter(line: str) -> tuple[str, str | None]:
    return line, None


def jinx_filter(line: str) -> tuple[str, str | None]:
    """Fold Jinx's own progress chatter into makepkg-style lines."""
    stripped = line.strip()
    if stripped.startswith("***"):
        return stripped.lstrip("* "), "warn"
    if stripped.startswith("* "):
        return stripped[2:].rstrip(".").rstrip(), "step"
    lowered = stripped.lower()
    if lowered.startswith(("error:", "fatal:", "jinx: ")) or " error:" in lowered:
        return stripped, "error"
    if lowered.startswith("warning:"):
        return stripped, "warn"
    return line, None


def cargo_filter(line: str) -> tuple[str, str | None]:
    stripped = line.strip()
    if stripped.startswith(("Compiling", "Checking", "Documenting", "Building")):
        return stripped, "step"
    if stripped.startswith("Finished"):
        return stripped, "step"
    if stripped.startswith("error") or stripped.startswith("error["):
        return line, "error"
    if stripped.startswith("warning"):
        return line, "warn"
    return line, None


class Runner:
    """Subprocess helper with three output modes.

    ``quiet``  buffer everything, show it only if the command fails.
    ``stream`` print output live, indented and dimmed (for slow builds).
    ``raw``    inherit stdio, for interactive things such as QEMU.
    """

    def __init__(self, ctx: Context) -> None:
        self.ctx = ctx
        self.log = ctx.log
        self.last_status = 0

    def __call__(
        self,
        argv: Sequence[str],
        *,
        cwd: Path = ROOT,
        env: Mapping[str, str] | None = None,
        mode: str = "quiet",
        filter: LineFilter = plain_filter,
        check: bool = True,
        tail: int = 40,
        always: bool = False,
    ) -> str:
        argv = [str(item) for item in argv]
        self.log.debug(f"[{rel(cwd)}] {shlex.join(argv)}")
        if self.ctx.dry_run and not always:
            self.log.msg3(f"would run: {shlex.join(argv)}")
            return ""
        if self.ctx.log.level >= VERBOSE and mode == "quiet":
            mode = "stream"
        if self.ctx.log.level <= QUIET and mode == "stream":
            mode = "quiet"

        if mode == "raw":
            return self._raw(argv, cwd, env, check)
        return self._piped(argv, cwd, env, mode, filter, check, tail)

    # -- implementations ---------------------------------------------------

    def _raw(
        self,
        argv: list[str],
        cwd: Path,
        env: Mapping[str, str] | None,
        check: bool,
    ) -> str:
        try:
            completed = subprocess.run(argv, cwd=cwd, env=self.ctx.env(env), check=False)
        except FileNotFoundError as exc:
            raise missing_tool(argv[0]) from exc
        self.last_status = completed.returncode
        if check and completed.returncode != 0:
            raise command_failure(argv, cwd, completed.returncode, [])
        return ""

    def _piped(
        self,
        argv: list[str],
        cwd: Path,
        env: Mapping[str, str] | None,
        mode: str,
        line_filter: LineFilter,
        check: bool,
        tail: int,
    ) -> str:
        try:
            process = subprocess.Popen(
                argv,
                cwd=cwd,
                env=self.ctx.env(env),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
                bufsize=1,
            )
        except FileNotFoundError as exc:
            raise missing_tool(argv[0]) from exc

        collected: list[str] = []
        assert process.stdout is not None
        for raw_line in process.stdout:
            line = raw_line.rstrip("\n")
            collected.append(line)
            if mode != "stream":
                continue
            text, kind = line_filter(line)
            if not text.strip():
                continue
            if kind == "step":
                self.log.msg3(text)
            elif kind == "warn":
                self.log.output(self.log.paint(text, Log.YELLOW))
            elif kind == "error":
                self.log.output(self.log.paint(text, Log.RED))
            else:
                self.log.output(text)
        returncode = process.wait()
        self.last_status = returncode
        if check and returncode != 0:
            raise command_failure(argv, cwd, returncode, collected[-tail:])
        return "\n".join(collected)


def missing_tool(tool: str) -> Failure:
    packages = TOOL_PACKAGES.get(tool, {})
    hint = None
    if packages:
        manager = detect_package_manager()
        package = packages.get(manager) or next(iter(packages.values()))
        hint = f"install it with: {install_command(manager, package)}"
    return Failure(f"required tool not found: {tool}", hint=hint)


def command_failure(
    argv: Sequence[str], cwd: Path, returncode: int, tail: Sequence[str]
) -> Failure:
    lines = [f"command failed with exit code {returncode}"]
    lines.append(f"    in {rel(cwd)}: {shlex.join(list(argv))}")
    for line in tail:
        if line.strip():
            lines.append("    | " + line)
    return Failure("\n".join(lines))


def capture(argv: Sequence[str], *, cwd: Path = ROOT) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            [str(item) for item in argv],
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            check=False,
        )
    except FileNotFoundError:
        return 127, ""
    return completed.returncode, completed.stdout


def detect_package_manager() -> str:
    for manager in ("pacman", "apt-get", "apk", "dnf"):
        if shutil.which(manager):
            return "apt" if manager == "apt-get" else manager
    return "apt"


def install_command(manager: str, package: str) -> str:
    return {
        "apt": f"sudo apt-get install {package}",
        "pacman": f"sudo pacman -S {package}",
        "apk": f"sudo apk add {package}",
        "dnf": f"sudo dnf install {package}",
    }.get(manager, f"install {package}")


# --------------------------------------------------------------------------
# Filesystem helpers
# --------------------------------------------------------------------------


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def remove(path: Path) -> None:
    if path.is_symlink() or path.is_file():
        path.unlink(missing_ok=True)
    elif path.is_dir():
        shutil.rmtree(path)


def copy(source: Path, destination: Path) -> None:
    ensure_dir(destination.parent)
    shutil.copy2(source, destination)


def write_file(path: Path, text: str, *, mode: int = 0o644) -> None:
    ensure_dir(path.parent)
    path.write_text(text, encoding="utf-8")
    path.chmod(mode)


def swap_directory(staging: Path, destination: Path) -> None:
    """Atomically-ish replace ``destination`` with ``staging``."""
    if not destination.exists():
        staging.rename(destination)
        return
    backup = destination.with_name(f".{destination.name}.old")
    remove(backup)
    destination.rename(backup)
    try:
        staging.rename(destination)
    except OSError:
        backup.rename(destination)
        raise
    remove(backup)


# --------------------------------------------------------------------------
# Fingerprints and the build cache
# --------------------------------------------------------------------------

IGNORED_FINGERPRINT_NAMES = {".git", "target", "build", "out", "__pycache__"}


def _hash_path(digest: Any, path: Path, *, base: Path | None = None) -> None:
    label = path.name if base is None else path.relative_to(base).as_posix()
    digest.update(label.encode("utf-8", "surrogateescape"))
    digest.update(b"\0")
    if path.is_symlink():
        digest.update(b"L" + os.readlink(path).encode("utf-8", "surrogateescape"))
        return
    if path.is_file():
        stat = path.stat()
        digest.update(b"F")
        digest.update(str(stat.st_mode & 0o111).encode())
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1 << 20), b""):
                digest.update(block)
        return
    if path.is_dir():
        digest.update(b"D")
        root = base or path
        for child in sorted(path.iterdir(), key=lambda item: item.name):
            if child.name in IGNORED_FINGERPRINT_NAMES:
                continue
            _hash_path(digest, child, base=root)
        return
    digest.update(b"?")


def fingerprint(
    label: str, *, values: Sequence[str] = (), paths: Sequence[Path] = ()
) -> str:
    digest = hashlib.sha256()
    digest.update(label.encode("utf-8"))
    digest.update(b"\0")
    for value in values:
        digest.update(value.encode("utf-8", "surrogateescape"))
        digest.update(b"\0")
    for path in paths:
        if path.exists() or path.is_symlink():
            _hash_path(digest, path, base=path.parent)
        else:
            digest.update(b"MISSING\0")
    return digest.hexdigest()


def stamp(path: Path) -> str:
    try:
        stat = path.stat()
    except OSError:
        return "missing"
    return f"{stat.st_size}:{stat.st_mtime_ns}"


class Cache:
    """Tracks whether a build step's inputs changed since it last succeeded."""

    def __init__(self, ctx: Context) -> None:
        self.ctx = ctx

    def _path(self, key: str) -> Path:
        return self.ctx.state_dir / f"{key}.json"

    def read(self, key: str) -> dict[str, Any]:
        try:
            data = json.loads(self._path(key).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return data if isinstance(data, dict) else {}

    def is_current(self, key: str, digest: str, outputs: Sequence[Path]) -> bool:
        if self.ctx.force:
            return False
        if not all(path.exists() for path in outputs):
            return False
        state = self.read(key)
        if state.get("fingerprint") != digest:
            return False
        stamps = state.get("stamps")
        if not isinstance(stamps, dict):
            return False
        return all(stamps.get(rel(path)) == stamp(path) for path in outputs)

    def record(self, key: str, digest: str, outputs: Sequence[Path], **extra: Any) -> None:
        path = self._path(key)
        ensure_dir(path.parent)
        payload: dict[str, Any] = {
            "fingerprint": digest,
            "stamps": {rel(item): stamp(item) for item in outputs},
        }
        payload.update(extra)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        temporary.replace(path)


# --------------------------------------------------------------------------
# Downloads and pinned checkouts
# --------------------------------------------------------------------------


def safe_extract(archive: Path, destination: Path) -> None:
    def check(name: str) -> None:
        pure = PurePosixPath(name)
        if pure.is_absolute() or ".." in pure.parts:
            raise Failure(f"archive contains an unsafe path: {name!r}")

    with tarfile.open(archive, "r:*") as tar:
        for member in tar.getmembers():
            check(member.name)
            if member.isdev() or member.isfifo():
                raise Failure(f"archive contains a special file: {member.name!r}")
            if member.issym() or member.islnk():
                link = PurePosixPath(member.linkname)
                if link.is_absolute():
                    raise Failure(f"archive contains an unsafe link: {member.name!r}")
                target = (Path(member.name).parent / Path(*link.parts)).as_posix()
                check(os.path.normpath(target))
        tar.extractall(destination, filter="tar")


def download(ctx: Context, pin: Tarball, run: Runner) -> Path:
    suffix = "".join(Path(pin.url.split("?", 1)[0]).suffixes[-2:]) or ".tar"
    cached = DOWNLOADS_DIR / f"{pin.name}-{pin.version}-{pin.sha256[:12]}{suffix}"
    if cached.is_file() and sha256_of(cached) == pin.sha256:
        return cached
    if ctx.offline:
        raise Failure(
            f"{pin.name} {pin.version} is not cached and --offline was given",
            hint=f"run './x.py fetch {pin.name}' while online",
        )
    remove(cached)
    ensure_dir(cached.parent)
    ctx.log.msg2(f"downloading {pin.name} {pin.version}")
    request = urllib.request.Request(pin.url, headers={"User-Agent": f"xtool/{VERSION}"})
    partial = cached.with_suffix(cached.suffix + ".part")
    digest = hashlib.sha256()
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            total = int(response.headers.get("Content-Length") or 0)
            done = 0
            with partial.open("wb") as output:
                while block := response.read(1 << 20):
                    output.write(block)
                    digest.update(block)
                    done += len(block)
                    if total and ctx.log.color and ctx.log.level >= NORMAL:
                        percent = done * 100 // total
                        sys.stdout.write(
                            f"\r     {ctx.log.paint(f'* {percent:3d}%  {human_size(done)}', Log.DIM)}"
                        )
                        sys.stdout.flush()
            if total and ctx.log.color and ctx.log.level >= NORMAL:
                sys.stdout.write("\r\033[K")
                sys.stdout.flush()
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        remove(partial)
        raise Failure(f"failed to download {pin.name}: {exc}") from exc
    if digest.hexdigest() != pin.sha256:
        remove(partial)
        raise Failure(
            f"{pin.name} checksum mismatch",
            hint=f"expected {pin.sha256}, got {digest.hexdigest()}",
        )
    partial.replace(cached)
    return cached


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def install_tarball(ctx: Context, pin: Tarball, run: Runner) -> Path:
    destination = CACHE_DIR / pin.directory
    marker = destination / ".xtool-pin.json"
    try:
        if json.loads(marker.read_text(encoding="utf-8")) == pin.marker():
            return destination
    except (OSError, json.JSONDecodeError):
        pass

    archive = download(ctx, pin, run)
    ensure_dir(CACHE_DIR)
    ctx.log.msg2(f"unpacking {pin.name} {pin.version}")
    with tempfile.TemporaryDirectory(prefix=f".{pin.name}-", dir=CACHE_DIR) as scratch:
        scratch_path = Path(scratch)
        safe_extract(archive, scratch_path)
        extracted = scratch_path / pin.root
        if not extracted.is_dir():
            raise Failure(f"{pin.name} archive does not contain {pin.root!r}")
        remove(destination)
        shutil.move(str(extracted), destination)
    write_file(marker, json.dumps(pin.marker(), indent=2, sort_keys=True) + "\n")
    return destination


def ensure_limine(ctx: Context, run: Runner) -> Path:
    limine = install_tarball(ctx, TARBALLS["limine"], run)
    if not (limine / "limine").exists():
        run(["make", "-C", str(limine)], mode="quiet")
    required = [limine / "limine-uefi-cd.bin", *(limine / name for name in ctx.arch.efi_files)]
    if ctx.arch.name == "x86_64":
        required += [limine / "limine-bios.sys", limine / "limine-bios-cd.bin"]
    missing = [item.name for item in required if not item.exists()]
    if missing:
        raise Failure(f"the pinned Limine release is missing: {', '.join(missing)}")
    return limine


def ensure_ovmf(ctx: Context, run: Runner) -> Path:
    ovmf = install_tarball(ctx, TARBALLS["ovmf"], run)

    candidate = ovmf / f"ovmf-code-{ctx.arch.name}.fd"
    if not candidate.is_file():
        raise Failure(f"the pinned OVMF release is missing {candidate.name}")
    return ovmf


def git_head(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    code, out = capture(["git", "-C", str(path), "rev-parse", "HEAD"])
    return out.strip().lower() if code == 0 else None


def ensure_jinx(ctx: Context, run: Runner) -> Path:
    """Check out the pinned Jinx revision under build/cache/jinx."""
    destination = CACHE_DIR / JINX_PIN.directory
    executable = destination / "jinx"
    if git_head(destination) == JINX_PIN.commit and executable.is_file():
        return destination
    if ctx.offline:
        raise Failure(
            "Jinx is not checked out at the pinned commit and --offline was given",
            hint="run './x.py fetch jinx' while online",
        )
    ctx.log.msg2(f"fetching Jinx {JINX_PIN.commit[:12]}")
    ensure_dir(destination)
    if not (destination / ".git").exists():
        run(["git", "init", "-q", str(destination)])
    run(["git", "-C", str(destination), "fetch", "--depth=1", JINX_PIN.url, JINX_PIN.commit])
    run(["git", "-C", str(destination), "checkout", "-q", "--detach", JINX_PIN.commit])
    if git_head(destination) != JINX_PIN.commit or not executable.is_file():
        raise Failure("the Jinx checkout does not match the pinned commit")
    return destination


# --------------------------------------------------------------------------
# Recipes
# --------------------------------------------------------------------------

_ASSIGNMENT = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)=(.*)$")
_FUNCTION = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*\s*\(\)")


@dataclass(frozen=True)
class Recipe:
    name: str
    directory: Path
    version: str
    revision: str
    deps: tuple[str, ...]
    builddeps: tuple[str, ...]
    from_source: str | None

    @property
    def version_revision(self) -> str:
        if not self.version:
            return f"?-{self.revision}"
        return f"{self.version}-{self.revision}"


def _expand(value: str, variables: Mapping[str, str]) -> str:
    value = value.strip()
    if value.startswith(("'", '"')) and value.endswith(value[0]) and len(value) >= 2:
        value = value[1:-1]
    if "$(" in value or "`" in value:
        return ""  # dynamically computed; not statically knowable

    def substitute(match: re.Match[str]) -> str:
        return variables.get(match.group(1) or match.group(2), "")

    return re.sub(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)", substitute, value)


def read_recipe(name: str) -> Recipe:
    directory = RECIPES_DIR / name
    path = directory / "recipe"
    if not path.is_file():
        raise Failure(
            f"no such recipe: {name}",
            hint="run './x.py list' to see the available packages",
        )
    variables: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if _FUNCTION.match(line):
            break
        text = line.strip()
        if not text or text.startswith("#"):
            continue
        match = _ASSIGNMENT.match(text)
        if match:
            variables[match.group(1)] = _expand(match.group(2), variables)
    return Recipe(
        name=name,
        directory=directory,
        version=variables.get("version", ""),
        revision=variables.get("revision", "0"),
        deps=tuple(variables.get("deps", "").split()),
        builddeps=tuple(variables.get("builddeps", "").split()),
        from_source=variables.get("from_source") or None,
    )


class Recipes:
    """The userland package graph, loaded straight out of userland/recipes."""

    def __init__(self) -> None:
        self._cache: dict[str, Recipe] = {}
        self.names: tuple[str, ...] = tuple(
            sorted(item.parent.name for item in RECIPES_DIR.glob("*/recipe"))
        )

    def __contains__(self, name: str) -> bool:
        return name in self.names

    def get(self, name: str) -> Recipe:
        if name not in self._cache:
            recipe = read_recipe(name)
            # A `from_source` recipe inherits the source recipe's version, which
            # is what ends up in the .xbps filename Jinx looks for.
            if not recipe.version and recipe.from_source and recipe.from_source != name:
                try:
                    recipe = replace(recipe, version=self.get(recipe.from_source).version)
                except Failure:
                    pass
            self._cache[name] = recipe
        return self._cache[name]

    def resolve(self, names: Sequence[str]) -> tuple[str, ...]:
        """Expand globs and validate names, keeping the caller's order."""
        selected: list[str] = []
        for pattern in names:
            if any(character in pattern for character in "*?["):
                matched = [item for item in self.names if _glob_match(pattern, item)]
                if not matched:
                    raise Failure(f"no package matches {pattern!r}")
                selected.extend(matched)
                continue
            if pattern not in self.names:
                raise Failure(
                    f"unknown package: {pattern}", hint=self._suggest(pattern)
                )
            selected.append(pattern)
        seen: dict[str, None] = {}
        for name in selected:
            seen.setdefault(name, None)
        return tuple(seen)

    def _suggest(self, name: str) -> str:
        import difflib

        close = difflib.get_close_matches(name, self.names, n=3, cutoff=0.5)
        if close:
            return "did you mean: " + ", ".join(close) + "?"
        return "run './x.py list' to see the available packages"

    # -- graph ------------------------------------------------------------

    def requires(self, name: str) -> tuple[str, ...]:
        """Build-order dependencies.

        ``from_source`` is deliberately excluded: it only says that two recipes
        share one source tree, not that one must be built before the other.
        Treating it as an edge introduces cycles such as mlibc <-> mlibc-headers.
        """
        recipe = self.get(name)
        related = list(recipe.deps) + list(recipe.builddeps)
        return tuple(dict.fromkeys(item for item in related if item in self.names))

    def dependency_closure(self, names: Sequence[str]) -> tuple[str, ...]:
        seen = set(names)
        pending = list(names)
        while pending:
            current = pending.pop()
            related = list(self.requires(current))
            source = self.get(current).from_source
            if source and source in self.names:
                related.append(source)
            for dependency in related:
                if dependency not in seen:
                    seen.add(dependency)
                    pending.append(dependency)
        return tuple(sorted(seen))

def _glob_match(pattern: str, name: str) -> bool:
    import fnmatch

    return fnmatch.fnmatchcase(name, pattern)


# --------------------------------------------------------------------------
# Image Manifest
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Manifest:
    install: tuple[str, ...]
    extra_build: tuple[str, ...]
    generated: bool

    @property
    def build(self) -> tuple[str, ...]:
        return tuple(dict.fromkeys((*self.extra_build, *self.install)))


def load_manifest(log: Log) -> Manifest:
    if not SYSTEM_MANIFEST.is_file():
        log.warn(f"{rel(SYSTEM_MANIFEST)} is missing; using built-in defaults")
        return Manifest(FALLBACK_SYSTEM_PACKAGES, FALLBACK_EXTRA_BUILD_PACKAGES, True)
    section = "install"
    buckets: dict[str, list[str]] = {"install": [], "build": []}
    for number, line in enumerate(
        SYSTEM_MANIFEST.read_text(encoding="utf-8").splitlines(), start=1
    ):
        text = line.split("#", 1)[0].strip()
        if not text:
            continue
        if text.startswith("[") and text.endswith("]"):
            section = text[1:-1].strip().lower()
            if section not in buckets:
                raise Failure(
                    f"{rel(SYSTEM_MANIFEST)}:{number}: unknown section [{section}]",
                    hint="valid sections are [install] and [build]",
                )
            continue
        buckets[section].append(text)
    if not buckets["install"]:
        raise Failure(
            f"{rel(SYSTEM_MANIFEST)} lists no packages under [install]",
            hint="add at least 'util-roanix' so the image can boot",
        )
    return Manifest(tuple(buckets["install"]), tuple(buckets["build"]), False)


# --------------------------------------------------------------------------
# Jinx driver
# --------------------------------------------------------------------------

WGET_SHIM = """#!/bin/sh
# Minimal wget shim backed by curl, generated by xtool.
output=""; agent=""; insecure=""; url=""
while [ "$#" -gt 0 ]; do
    case "$1" in
        -O) output="$2"; shift 2 ;;
        -qO-) output="-"; shift ;;
        -U) agent="$2"; shift 2 ;;
        -nv|-q) shift ;;
        --no-check-certificate) insecure="-k"; shift ;;
        --ca-certificate=*) ca_file="${1#*=}"; shift ;;
        --certificate=*) certificate="${1#*=}"; shift ;;
        --private-key=*) private_key="${1#*=}"; shift ;;
        --) shift; break ;;
        -*) echo "wget shim: unsupported option: $1" >&2; exit 2 ;;
        *) url="$1"; shift ;;
    esac
done
[ -n "$url" ] || { echo "wget shim: no URL given" >&2; exit 2; }
set -- CURL -fsSL
[ -n "$output" ] && set -- "$@" -o "$output"
[ -n "$agent" ] && set -- "$@" -A "$agent"
[ -n "$insecure" ] && set -- "$@" "$insecure"
[ -n "${ca_file-}" ] && set -- "$@" --cacert "$ca_file"
[ -n "${certificate-}" ] && set -- "$@" --cert "$certificate"
[ -n "${private_key-}" ] && set -- "$@" --key "$private_key"
exec "$@" "$url"
"""


class Jinx:
    """Driver for the Jinx tool."""

    def __init__(self, ctx: Context, run: Runner, recipes: Recipes) -> None:
        self.ctx = ctx
        self.run = run
        self.recipes = recipes
        self._binary: Path | None = None
        self._env: dict[str, str] | None = None

    # -- bootstrap ---------------------------------------------------------

    def _environment(self) -> dict[str, str]:
        if self._env is not None:
            return self._env
        path = os.environ.get("PATH", "")
        env = {"JINX_PARALLELISM": str(self.ctx.jobs)}
        if shutil.which("wget") is None:
            curl = shutil.which("curl")
            if curl is None:
                raise Failure(
                    "Jinx needs wget to download sources",
                    hint=f"install wget, or curl for xtool's fallback: "
                    f"{install_command(detect_package_manager(), 'wget')}",
                )
            shim_dir = ensure_dir(CACHE_DIR / "host-tools")
            write_file(
                shim_dir / "wget", WGET_SHIM.replace("CURL", shlex.quote(curl)), mode=0o755
            )
            path = f"{shim_dir}:{path}" if path else str(shim_dir)
        env["PATH"] = path
        self._env = env
        return env

    def prepare(self) -> Path:
        """Make sure Jinx exists and its build directory is initialised."""
        if self._binary is not None:
            return self._binary
        checkout = ensure_jinx(self.ctx, self.run)
        binary = checkout / "jinx"
        build = self.ctx.jinx_build
        if build.is_symlink():
            raise Failure(f"refusing to use a symlinked Jinx build directory: {build}")
        ensure_dir(build)
        if not (build / ".jinx-parameters").is_file():
            self.ctx.log.msg2(f"initialising the Jinx build directory for {self.ctx.arch.name}")
            self.run(
                [binary, "init", str(USERLAND_DIR), f"ARCH={self.ctx.arch.name}"],
                cwd=build,
                env=self._environment(),
            )
        self._binary = binary
        return binary

    # -- raw invocation ----------------------------------------------------

    def __call__(self, arguments: Sequence[str], *, mode: str = "stream", always: bool = False) -> str:
        binary = self.prepare()
        return self.run(
            [binary, *arguments],
            cwd=self.ctx.jinx_build,
            env=self._environment(),
            mode=mode,
            filter=jinx_filter,
            always=always,
        )

    # -- commands ----------------------------------------------------------

    def preview(self, names: Sequence[str]) -> tuple[str, ...]:
        """``jinx dry-run``: what would be built, in order."""
        try:
            output = self(["dry-run", *names], mode="quiet", always=True)
        except Failure:
            return ()
        return tuple(output.split())

    def update(self, names: Sequence[str], *, build_missing: bool = True) -> None:
        arguments = ["update"]
        if build_missing:
            arguments.append("-b")
        self(arguments + list(names))

    def download(self, names: Sequence[str]) -> None:
        self(["download", *names])

    def build(self, name: str, *, force: bool = False) -> None:
        command = "rebuild" if force else "build"
        self([command, name])

    def regen(self, name: str) -> None:
        self(["regen", name], mode="stream")

    def install(self, sysroot: Path, names: Sequence[str], *, force: bool = False) -> None:
        arguments = ["install"]
        if force:
            arguments.append("-f")
        self(arguments + [str(sysroot), *names])

    # -- introspection -----------------------------------------------------

    def built(self) -> dict[str, set[str]]:
        """Map package name -> every ``version_revision`` present in pkgs/."""
        result: dict[str, set[str]] = {}
        packages = self.ctx.jinx_build / "pkgs"
        if not packages.is_dir():
            return result
        suffix = f".{self.ctx.arch.name}.xbps"
        for item in packages.glob(f"*{suffix}"):
            stem = item.name[: -len(suffix)]
            name, separator, version = stem.rpartition("-")
            if name and separator:
                result.setdefault(name, set()).add(version)
        return result

    def has_package(self, recipe: Recipe, built: Mapping[str, set[str]]) -> bool:
        """Is this exact recipe revision already built?"""
        versions = built.get(recipe.name)
        if not versions:
            return False
        if not recipe.version:
            return True  # dynamically computed version; trust that it exists
        return f"{recipe.version}_{recipe.revision}" in versions

    def is_initialised(self) -> bool:
        return (self.ctx.jinx_build / ".jinx-parameters").is_file()


# --------------------------------------------------------------------------
# Kernel
# --------------------------------------------------------------------------


def rust_env(ctx: Context) -> dict[str, str]:
    flags = ["-Crelocation-model=static", "-Cforce-frame-pointers=yes"]
    extra = os.environ.get("ROANIX_RUSTFLAGS")
    if extra:
        flags.append(extra)
    return {"CARGO_TARGET_DIR": str(ctx.cargo_target), "RUSTFLAGS": " ".join(flags)}


def kernel_digest(ctx: Context) -> str:
    return fingerprint(
        "kernel",
        values=(
            ctx.arch.name,
            ctx.profile,
            ctx.arch.rust_target,
            os.environ.get("ROANIX_RUSTFLAGS", ""),
        ),
        paths=(
            KERNEL_DIR / "Cargo.toml",
            KERNEL_DIR / "Cargo.lock",
            KERNEL_DIR / "rust-toolchain.toml",
            KERNEL_DIR / "build.rs",
            KERNEL_DIR / f"linker-{ctx.arch.name}.ld",
            KERNEL_DIR / ".cargo",
            KERNEL_DIR / "src",
            KERNEL_DIR / "include",
        ),
    )


def cargo(
    ctx: Context, run: Runner, subcommand: str, *arguments: str, mode: str = "stream", jobs: bool = True
) -> None:
    argv = ["cargo", subcommand]
    if jobs and ctx.jobs:
        argv += ["--jobs", str(ctx.jobs)]
    argv += list(arguments)
    run(argv, cwd=KERNEL_DIR, env=rust_env(ctx), mode=mode, filter=cargo_filter)


def rust_driver_digest(ctx: Context, package: str, output_name: str) -> str:
    """Fingerprint the Rust module workspace independently from the kernel."""
    return fingerprint(
        f"rust-driver-{package}",
        values=(
            ctx.arch.name,
            ctx.arch.rust_target,
            output_name,
            "release",
        ),
        paths=(DRIVERS_DIR,),
    )


def build_rust_module(
    ctx: Context, run: Runner, cache: Cache, *, package: str, output_name: str
) -> Path:
    """Build one Rust module for direct installation into the sysroot."""
    output = ctx.arch_root / "drivers-rust" / output_name
    key = f"rust-driver-{package}-release"
    digest = rust_driver_digest(ctx, package, output_name)
    if cache.is_current(key, digest, (output,)):
        return output

    ctx.log.msg2(f"Building Rust {package} driver ({ctx.arch.name})")
    target_dir = ctx.cargo_target / "drivers"
    arguments = [
        "cargo",
        "build",
        "--package",
        package,
        "--target",
        ctx.arch.rust_target,
        "--release",
        "--target-dir",
        str(target_dir),
    ]
    if ctx.arch.name == "riscv64":
        arguments += [
            "-Z",
            "build-std=core,alloc,compiler_builtins",
            "-Z",
            "build-std-features=compiler-builtins-mem",
        ]
    run(
        arguments,
        cwd=DRIVERS_DIR,
        mode="stream",
        filter=cargo_filter,
    )
    if ctx.dry_run:
        return output
    produced = target_dir / ctx.arch.rust_target / "release" / package
    if not produced.is_file():
        raise Failure(f"Cargo produced no Rust driver at {rel(produced)}")
    copy(produced, output)
    if not output.is_file():
        raise Failure(f"Cargo produced no Rust driver at {rel(output)}")
    cache.record(key, digest, (output,))
    return output


def build_rust_drivers(ctx: Context, run: Runner, cache: Cache) -> tuple[Path, ...]:
    return tuple(
        build_rust_module(
            ctx,
            run,
            cache,
            package=package,
            output_name=output_name,
        )
        for package, output_name in RUST_DRIVER_MODULES[ctx.arch.name]
    )


def stale_rust_drivers(ctx: Context, cache: Cache) -> tuple[str, ...]:
    return tuple(
        output_name
        for package, output_name in RUST_DRIVER_MODULES[ctx.arch.name]
        if not cache.is_current(
            f"rust-driver-{package}-release",
            rust_driver_digest(ctx, package, output_name),
            (ctx.arch_root / "drivers-rust" / output_name,),
        )
    )


def build_kernel(ctx: Context, run: Runner, cache: Cache) -> Path:
    key = f"kernel-{ctx.profile}"
    digest = kernel_digest(ctx)
    if cache.is_current(key, digest, (ctx.kernel_binary,)):
        ctx.log.msg2(f"kernel is up to date ({rel(ctx.kernel_binary)})")
        return ctx.kernel_binary

    started = time.monotonic()
    ctx.log.msg(f"Building the kernel ({ctx.arch.name}, {ctx.profile})")
    output_dir = ctx.cargo_arch_dir / (
        "debug" if ctx.profile == "dev" else ctx.profile
    )
    if ctx.force:
        remove(output_dir)
        remove(ctx.kernel_binary)
    ensure_dir(ctx.artifacts)
    cargo(ctx, run, "build", "--target", ctx.arch.rust_target, "--profile", ctx.profile)
    if ctx.dry_run:
        return ctx.kernel_binary

    produced = output_dir / "roanix"
    if not produced.is_file():
        candidates = sorted(
            item
            for item in output_dir.iterdir()
            if item.is_file() and os.access(item, os.X_OK)
        )
        if not candidates:
            raise Failure(f"cargo produced no kernel executable in {rel(output_dir)}")
        produced = candidates[0]
    copy(produced, ctx.kernel_binary)
    cache.record(key, digest, (ctx.kernel_binary,))
    ctx.log.msg2(
        f"{rel(ctx.kernel_binary)} ({human_size(ctx.kernel_binary.stat().st_size)})"
    )
    ctx.log.finished("the kernel", time.monotonic() - started)
    return ctx.kernel_binary


# --------------------------------------------------------------------------
# Userland
# --------------------------------------------------------------------------


def prefetch_packages(ctx: Context, jinx: Jinx, names: Sequence[str]) -> None:
    """Populate Jinx's local repositories from the public binary cache."""
    enabled = os.environ.get("ROANIX_BINARY_PACKAGES", "1").strip().lower()
    if ctx.offline or enabled in ("0", "false", "no", "off"):
        return
    ctx.log.msg2("checking the Roanix binary package repository")
    try:
        jinx.download(names)
    except Failure as error:
        message = str(error).lower()
        if "sha256" in message or "checksum" in message:
            raise
        ctx.log.warn("package download incomplete, falling back to local builds")
        if ctx.log.level >= VERBOSE:
            for line in str(error).splitlines():
                ctx.log.debug(line)


def announce_plan(ctx: Context, jinx: Jinx, targets: Sequence[str]) -> None:
    """Ask Jinx itself what it would build, and show it."""
    planned = jinx.preview(targets)
    if planned:
        ctx.log.msg2(f"Jinx would build {len(planned)} package(s), in order:")
        ctx.log.msg3(" ".join(planned))
    else:
        ctx.log.msg2("Jinx has nothing to build")


def install_rust_drivers(root: Path, modules: Sequence[Path]) -> None:
    module_dir = root / "usr/lib/roanix/drivers"
    include_dir = root / "usr/include/roanix"
    for module in modules:
        copy(module, module_dir / module.name)
    for header in sorted((DRIVERS_DIR / "include/roanix").glob("*.h")):
        copy(header, include_dir / header.name)


def install_sysroot(
    ctx: Context,
    jinx: Jinx,
    manifest: Manifest,
    modules: Sequence[Path],
    *,
    force: bool = False,
) -> Path:
    """Assemble a fresh sysroot from the built packages, then swap it in."""
    ensure_dir(ctx.arch_root)
    staging = ctx.arch_root / ".sysroot.staging"
    remove(staging)
    ensure_dir(staging)
    try:
        jinx.install(staging, manifest.install, force=force)
        if ctx.dry_run:
            return ctx.sysroot
        install_rust_drivers(staging, modules)
        swap_directory(staging, ctx.sysroot)
    finally:
        remove(staging)
    ctx.log.msg2(
        f"{rel(ctx.sysroot)} ({human_size(tree_size(ctx.sysroot))}, "
        f"{len(manifest.install)} top-level packages)"
    )
    return ctx.sysroot


def build_userland(
    ctx: Context,
    jinx: Jinx,
    manifest: Manifest,
) -> Path:
    """Ask Jinx to update the manifest packages and assemble the sysroot."""
    started = time.monotonic()
    cache = Cache(ctx)
    modules = build_rust_drivers(ctx, jinx.run, cache)
    jinx.prepare()
    wanted = manifest.build
    # Host tools use a separate repository. Fetch them first so a target
    # package missing from the server can still be built locally without also
    # compiling the cross-toolchain from scratch.
    prefetch_packages(ctx, jinx, ("host:*",))
    prefetch_packages(ctx, jinx, wanted)
    planned = jinx.preview(wanted)
    digest = sysroot_digest(ctx, manifest, modules)
    if not planned and cache.is_current("sysroot", digest, (ctx.sysroot,)):
        ctx.log.msg2(f"userland is up to date ({len(jinx.built())} packages built)")
        return ctx.sysroot
    if ctx.dry_run:
        announce_plan(ctx, jinx, wanted)
        return ctx.sysroot
    if ctx.force:
        for name in wanted:
            jinx.build(name, force=True)
    elif planned:
        jinx.update(wanted)

    ctx.log.msg(f"Installing the sysroot ({ctx.arch.name})")
    install_sysroot(ctx, jinx, manifest, modules, force=ctx.force)
    if not ctx.dry_run:
        cache.record("sysroot", digest, (ctx.sysroot,))
    ctx.log.finished("the userland", time.monotonic() - started)
    return ctx.sysroot


def sysroot_digest(ctx: Context, manifest: Manifest, modules: Sequence[Path]) -> str:
    packages = ctx.jinx_build / "pkgs"
    package_stamps = tuple(
        f"{item.name}:{stamp(item)}" for item in sorted(packages.glob("*.xbps"))
    )
    return fingerprint(
        "sysroot",
        values=(
            ctx.arch.name,
            JINX_PIN.commit,
            *manifest.build,
            *manifest.install,
            *package_stamps,
            *(f"{module.name}:{stamp(module)}" for module in modules),
        ),
        paths=(USERLAND_DIR / "Jinxfile", DRIVERS_DIR / "include/roanix"),
    )


# --------------------------------------------------------------------------
# Initramfs
# --------------------------------------------------------------------------


def sysroot_token(ctx: Context) -> str:
    """A cheap identity for the current sysroot.

    The sysroot is only ever replaced wholesale (``swap_directory`` renames a
    freshly staged tree into place), so its stamp plus the fingerprint recorded
    by the last successful install identifies it exactly - without walking a
    few hundred megabytes of files on every single invocation.
    """
    recorded = Cache(ctx).read("sysroot").get("fingerprint", "")
    return f"{stamp(ctx.sysroot)}|{recorded}"


def initramfs_digest(ctx: Context) -> str:
    return fingerprint("initramfs", values=(ctx.arch.name, ctx.profile, sysroot_token(ctx)))


def build_initramfs(ctx: Context, cache: Cache) -> Path:
    if not ctx.sysroot.is_dir():
        if ctx.dry_run:
            # The sysroot would have been produced by the preceding step; a dry
            # run must describe the plan rather than fail on its own simulation.
            ctx.log.msg(f"Packing the initramfs ({ctx.arch.name})")
            ctx.log.msg3("requires the sysroot built by the previous step")
            return ctx.initramfs
        raise Failure(
            f"the {ctx.arch.name} sysroot does not exist yet",
            hint=f"run './x.py run --arch {ctx.arch.name}' to assemble it",
        )
    key = f"initramfs-{ctx.profile}"
    digest = initramfs_digest(ctx)
    if cache.is_current(key, digest, (ctx.initramfs,)):
        ctx.log.msg2(f"initramfs is up to date ({rel(ctx.initramfs)})")
        return ctx.initramfs

    ctx.log.msg(f"Packing the initramfs ({ctx.arch.name})")
    if ctx.dry_run:
        return ctx.initramfs
    level = 1 if ctx.profile == "dev" else 9
    ctx.log.msg3(f"gzip level {level} from {rel(ctx.sysroot)}")
    ensure_dir(ctx.initramfs.parent)
    temporary = ctx.initramfs.with_suffix(".tmp")
    remove(temporary)

    def as_root(info: tarfile.TarInfo) -> tarfile.TarInfo:
        info.uid = info.gid = 0
        info.uname = info.gname = "root"
        return info

    try:
        with tarfile.open(
            temporary,
            mode="w:gz",
            format=tarfile.USTAR_FORMAT,
            dereference=False,
            compresslevel=level,
        ) as archive:
            entries = sorted(
                ctx.sysroot.rglob("*"),
                key=lambda item: item.relative_to(ctx.sysroot).as_posix(),
            )
            for entry in entries:
                archive.add(
                    entry,
                    arcname=entry.relative_to(ctx.sysroot).as_posix(),
                    recursive=False,
                    filter=as_root,
                )
        temporary.replace(ctx.initramfs)
        ctx.initramfs.chmod(0o644)
    finally:
        remove(temporary)
    cache.record(key, digest, (ctx.initramfs,))
    ctx.log.msg2(
        f"{rel(ctx.initramfs)} ({human_size(ctx.initramfs.stat().st_size)})"
    )
    return ctx.initramfs


# --------------------------------------------------------------------------
# Bootable images
# --------------------------------------------------------------------------


def limine_config() -> str:
    text = (DISTRO_FILES_DIR / "limine.conf").read_text(encoding="ascii")
    return text + (
        "    module_path: $boot():/boot/roanix-root.tar.gz\n"
        "    module_string: initramfs\n"
    )


def image_digest(ctx: Context, kind: str) -> str:
    return fingerprint(
        f"image-{kind}",
        values=(
            ctx.arch.name,
            ctx.profile,
            stamp(ctx.kernel_binary),
            stamp(ctx.initramfs),
            json.dumps(TARBALLS["limine"].marker(), sort_keys=True),
        ),
        paths=(DISTRO_FILES_DIR / "limine.conf", DISTRO_FILES_DIR / "splash.jpg"),
    )


def boot_payload(ctx: Context) -> list[tuple[Path, str]]:
    """(source, destination) pairs shared by the ISO and HDD layouts."""
    return [
        (ctx.kernel_binary, "boot/roanix"),
        (ctx.initramfs, "boot/roanix-root.tar.gz"),
        (DISTRO_FILES_DIR / "splash.jpg", "boot/splash.jpg"),
    ]


def build_iso(ctx: Context, run: Runner, cache: Cache) -> Path:
    target = ctx.image("iso")
    key = f"iso-{ctx.profile}"
    digest = image_digest(ctx, "iso")
    if cache.is_current(key, digest, (target,)):
        ctx.log.msg2(f"ISO image is up to date ({rel(target)})")
        return target

    started = time.monotonic()
    ctx.log.msg(f"Creating the ISO image ({ctx.arch.name})")
    limine = ensure_limine(ctx, run)
    if ctx.dry_run:
        return target

    root = ctx.work / "iso-root"
    remove(root)
    ensure_dir(root / "boot" / "limine")
    ensure_dir(root / "EFI" / "BOOT")
    for source, destination in boot_payload(ctx):
        copy(source, root / destination)
    write_file(root / "boot" / "limine" / "limine.conf", limine_config())
    copy(limine / "limine-uefi-cd.bin", root / "boot/limine/limine-uefi-cd.bin")
    for name in ctx.arch.efi_files:
        copy(limine / name, root / "EFI" / "BOOT" / name)
    if ctx.arch.name == "x86_64":
        copy(limine / "limine-bios.sys", root / "boot/limine/limine-bios.sys")
        copy(limine / "limine-bios-cd.bin", root / "boot/limine/limine-bios-cd.bin")

    ensure_dir(target.parent)
    remove(target)
    argv = ["xorriso", "-as", "mkisofs", "-R", "-r", "-J", "-V", f"ROANIX_{ctx.arch.name.upper()}"]
    if ctx.arch.name == "x86_64":
        argv += [
            "-b", "boot/limine/limine-bios-cd.bin",
            "-no-emul-boot", "-boot-load-size", "4", "-boot-info-table",
        ]
    argv += [
        "-hfsplus", "-apm-block-size", "2048",
        "--efi-boot", "boot/limine/limine-uefi-cd.bin",
        "-efi-boot-part", "--efi-boot-image", "--protective-msdos-label",
        str(root), "-o", str(target),
    ]
    try:
        run(argv)
        if ctx.arch.name == "x86_64":
            run([limine / "limine", "bios-install", str(target)])
    finally:
        remove(root)
    cache.record(key, digest, (target,))
    ctx.log.msg2(f"{rel(target)} ({human_size(target.stat().st_size)})")
    ctx.log.finished("the ISO image", time.monotonic() - started)
    return target


def build_hdd(ctx: Context, run: Runner, cache: Cache) -> Path:
    target = ctx.image("hdd")
    key = f"hdd-{ctx.profile}"
    digest = image_digest(ctx, "hdd")
    if cache.is_current(key, digest, (target,)):
        ctx.log.msg2(f"HDD image is up to date ({rel(target)})")
        return target

    started = time.monotonic()
    ctx.log.msg(f"Creating the HDD image ({ctx.arch.name})")
    limine = ensure_limine(ctx, run)
    if ctx.dry_run:
        return target

    payload = sum(source.stat().st_size for source, _ in boot_payload(ctx))
    size = max(128 * 1024 * 1024, int(payload * 1.4) + 32 * 1024 * 1024)
    size = (size + 1024 * 1024 - 1) // (1024 * 1024) * (1024 * 1024)

    ensure_dir(target.parent)
    remove(target)
    with target.open("wb") as disk:
        disk.truncate(size)

    path = os.environ.get("PATH", "")
    sbin_path = f"{path}:/usr/sbin:/sbin" if path else "/usr/sbin:/sbin"
    partition = ["sgdisk", str(target), "-n", "1:2048", "-t", "1:ef00"]
    if ctx.arch.name == "x86_64":
        partition += ["-m", "1"]
    run(partition, env={"PATH": sbin_path})
    if ctx.arch.name == "x86_64":
        run([limine / "limine", "bios-install", str(target)])

    spec = f"{target}@@1M"
    run(["mformat", "-v", "ROANIX", "-i", spec, "::"])
    run(["mmd", "-i", spec, "::/EFI", "::/EFI/BOOT", "::/boot", "::/boot/limine"])
    for source, destination in boot_payload(ctx):
        run(["mcopy", "-m", "-i", spec, str(source), f"::/{destination}"])
    config = ctx.work / "limine.conf"
    write_file(config, limine_config())
    run(["mcopy", "-m", "-i", spec, str(config), "::/boot/limine/limine.conf"])
    if ctx.arch.name == "x86_64":
        run(["mcopy", "-m", "-i", spec, str(limine / "limine-bios.sys"),
             "::/boot/limine/limine-bios.sys"])
    for name in ctx.arch.efi_files:
        run(["mcopy", "-m", "-i", spec, str(limine / name), f"::/EFI/BOOT/{name}"])

    cache.record(key, digest, (target,))
    ctx.log.msg2(f"{rel(target)} ({human_size(target.stat().st_size)})")
    ctx.log.finished("the HDD image", time.monotonic() - started)
    return target


# --------------------------------------------------------------------------
# QEMU
# --------------------------------------------------------------------------


@dataclass
class QemuOptions:
    image: str = "hdd"
    firmware: str = "uefi"
    gdb: bool = False
    gdb_port: int = 1234
    wait: bool = True
    accel: str = "auto"
    memory: str | None = None
    smp: int | None = None
    monitor: bool = False
    display: str | None = None
    serial_log: Path | None = None
    trace: str | None = None
    extra: tuple[str, ...] = ()


def kvm_available() -> bool:
    try:
        handle = os.open("/dev/kvm", os.O_RDWR | os.O_CLOEXEC)
    except OSError:
        return False
    os.close(handle)
    return True


def accel_overridden(arguments: Sequence[str]) -> bool:
    for index, argument in enumerate(arguments):
        if argument in {"-accel", "-enable-kvm", "-no-kvm"} or argument.startswith("-accel="):
            return True
        if argument in {"-machine", "-M"} and index + 1 < len(arguments):
            if "accel=" in arguments[index + 1]:
                return True
        if argument.startswith(("-machine=", "-M=")) and "accel=" in argument:
            return True
    return False


def qemu_command(ctx: Context, run: Runner, image: Path, options: QemuOptions) -> list[str]:
    arch = ctx.arch
    argv = [arch.qemu, "-m", options.memory or arch.memory]
    if options.smp:
        argv += ["-smp", str(options.smp)]
    if options.image == "iso":
        argv += ["-cdrom", str(image)]
    else:
        argv += ["-drive", f"file={image},format=raw"]

    use_kvm = (
        options.accel != "tcg"
        and arch.name == "x86_64"
        and platform.machine() == "x86_64"
        and kvm_available()
        and not accel_overridden(options.extra)
    )

    if options.firmware == "bios":
        argv += ["-M", "q35,smm=off"]
    else:
        ovmf = ensure_ovmf(ctx, run)
        ensure_dir(ctx.firmware)
        variables = ctx.firmware / "ovmf-vars.fd"
        if not variables.is_file():
            copy(ovmf / f"ovmf-vars-{arch.name}.fd", variables)
        argv += [
            "-drive",
            f"if=pflash,unit=0,format=raw,file={ovmf / f'ovmf-code-{arch.name}.fd'},readonly=on",
        ]
        if arch.name == "x86_64":
            argv += ["-M", "q35"]
        else:
            argv += [
                "-M", "virt,acpi=off", "-cpu", "rv64",
                "-device", "ramfb",
                "-device", "qemu-xhci", "-device", "usb-kbd", "-device", "usb-mouse",
            ]

    if arch.name == "x86_64":
        if use_kvm:
            argv += ["-accel", "kvm", "-cpu", "host,+invtsc"]
        else:
            argv += ["-cpu", "max,+invtsc,+tsc-deadline,+fsgsbase"]

    if options.serial_log is not None:
        ensure_dir(options.serial_log.parent)
        mux = "on" if options.monitor else "off"
        argv += [
            "-chardev",
            f"stdio,id=roanix-serial,logfile={options.serial_log},mux={mux}",
            "-serial",
            "chardev:roanix-serial",
        ]
        if options.monitor:
            argv += ["-mon", "chardev=roanix-serial"]
    else:
        argv += ["-serial", "mon:stdio" if options.monitor else "stdio"]

    if options.display:
        argv += ["-display", options.display]
    if options.trace:
        argv += ["-d", options.trace, "-D", str(ctx.firmware / "qemu.log")]
    if options.gdb:
        argv += ["-gdb", f"tcp::{options.gdb_port}"]
        if options.wait:
            argv.append("-S")
    argv += list(options.extra)
    return argv


def run_qemu(ctx: Context, run: Runner, image: Path, options: QemuOptions) -> None:
    if options.firmware not in ctx.arch.firmwares:
        raise Failure(
            f"{options.firmware.upper()} boot is not supported on {ctx.arch.name}",
            hint=f"supported firmware: {', '.join(ctx.arch.firmwares)}",
        )
    argv = qemu_command(ctx, run, image, options)
    accel = "KVM" if "-accel" in argv and "kvm" in argv else "TCG"

    ctx.log.msg(f"Booting Roanix {ctx.arch.name} ({options.firmware.upper()}, {accel})")
    ctx.log.msg2(f"image: {rel(image)} ({options.image})")
    if options.serial_log is not None:
        ctx.log.msg2(f"serial log: {rel(options.serial_log)}")
    if options.trace:
        ctx.log.msg2(f"QEMU trace: {rel(ctx.firmware / 'qemu.log')}")
    if options.gdb:
        ctx.log.msg2(f"GDB server listening on tcp::{options.gdb_port}"
                     + (" (halted, waiting for a debugger)" if options.wait else ""))
        ctx.log.note(
            f"connect with: rust-gdb {rel(ctx.kernel_binary)} "
            f"-ex 'target remote :{options.gdb_port}'"
        )
    ctx.log.note(
        "press Ctrl-A X to quit QEMU" if options.monitor else "press Ctrl-C to stop QEMU"
    )
    ctx.log.plain()
    run(argv, mode="raw", check=False)
    if run.last_status not in (0, 130, -2):
        ctx.log.plain()
        ctx.log.warn(f"QEMU exited with status {run.last_status}")
        ctx.log.hint("re-run with -v to see the exact command line")


# --------------------------------------------------------------------------
# Build planning
# --------------------------------------------------------------------------

TARGETS = ("kernel", "sysroot", "initramfs", "iso", "hdd", "all")


@dataclass
class Component:
    name: str
    state: str  # "ready", "stale", "missing"
    detail: str = ""


def survey(
    ctx: Context,
    jinx: Jinx,
    manifest: Manifest,
    *,
    default_image: str = "hdd",
) -> list[Component]:
    """Work out what is up to date without building anything."""
    cache = Cache(ctx)
    components: list[Component] = []

    if not ctx.kernel_binary.exists():
        components.append(Component("kernel", "missing", "never built"))
    elif not cache.is_current(
        f"kernel-{ctx.profile}", kernel_digest(ctx), (ctx.kernel_binary,)
    ):
        components.append(Component("kernel", "stale", "sources changed"))
    else:
        components.append(
            Component("kernel", "ready", f"{human_size(ctx.kernel_binary.stat().st_size)}")
        )

    if not jinx.is_initialised() or not ctx.sysroot.is_dir():
        components.append(Component("userland", "missing", "sysroot not built"))
    else:
        planned = jinx.preview(manifest.build)
        drivers = stale_rust_drivers(ctx, cache)
        if planned or drivers:
            details = []
            if planned:
                shown = " ".join(planned[:4]) + (" ..." if len(planned) > 4 else "")
                details.append(f"{len(planned)} pending in Jinx: {shown}")
            if drivers:
                details.append("drivers changed: " + " ".join(drivers))
            components.append(
                Component("userland", "stale", "; ".join(details))
            )
        else:
            components.append(
                Component("userland", "ready", f"{len(jinx.built())} packages built, "
                          f"{human_size(tree_size(ctx.sysroot))}")
            )

    if not ctx.initramfs.exists():
        components.append(Component("initramfs", "missing", "never packed"))
    elif not ctx.sysroot.is_dir() or not cache.is_current(
        f"initramfs-{ctx.profile}", initramfs_digest(ctx), (ctx.initramfs,)
    ):
        components.append(Component("initramfs", "stale", "sysroot changed"))
    else:
        components.append(
            Component("initramfs", "ready", human_size(ctx.initramfs.stat().st_size))
        )

    for kind in ("hdd", "iso"):
        image = ctx.image(kind)
        if not image.exists():
            # Only nag about the image format that is actually in use.
            if kind != default_image:
                continue
            components.append(Component(f"{kind} image", "missing", "never built"))
        elif not cache.is_current(
            f"{kind}-{ctx.profile}", image_digest(ctx, kind), (image,)
        ):
            components.append(Component(f"{kind} image", "stale", "inputs changed"))
        else:
            components.append(
                Component(f"{kind} image", "ready", human_size(image.stat().st_size))
            )
    return components


def build_chain(
    ctx: Context,
    run: Runner,
    jinx: Jinx,
    manifest: Manifest,
    target: str,
) -> list[Path]:
    """Build ``target`` and everything it depends on."""
    cache = Cache(ctx)
    if target == "kernel":
        return [build_kernel(ctx, run, cache)]
    if target == "sysroot":
        return [build_userland(ctx, jinx, manifest)]
    if target == "initramfs":
        build_userland(ctx, jinx, manifest)
        return [build_initramfs(ctx, cache)]
    if target in ("iso", "hdd", "all"):
        build_kernel(ctx, run, cache)
        build_userland(ctx, jinx, manifest)
        build_initramfs(ctx, cache)
        if target == "iso":
            return [build_iso(ctx, run, cache)]
        if target == "hdd":
            return [build_hdd(ctx, run, cache)]
        return [build_hdd(ctx, run, cache), build_iso(ctx, run, cache)]
    raise Failure(f"unknown build target: {target}", hint=f"valid targets: {', '.join(TARGETS)}")


# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


@dataclass
class World:
    """Everything a command needs, assembled once by main()."""

    ctx: Context
    run: Runner
    recipes: Recipes
    manifest: Manifest
    jinx: Jinx

    @property
    def log(self) -> Log:
        return self.ctx.log


def qemu_options_from(args: argparse.Namespace, ctx: Context, settings: Mapping[str, str]) -> QemuOptions:
    extra = list(ctx.qemu_args)
    saved = settings.get("qemu-args")
    if saved:
        extra = shlex.split(saved) + extra
    serial_log = getattr(args, "serial_log", None)
    return QemuOptions(
        image=getattr(args, "image", None) or setting(settings, "image", None, "hdd"),
        firmware=getattr(args, "firmware", None) or setting(settings, "firmware", None, "uefi"),
        gdb=bool(getattr(args, "gdb", False)),
        gdb_port=int(getattr(args, "gdb_port", 1234)),
        wait=not bool(getattr(args, "no_wait", False)),
        accel="tcg" if getattr(args, "tcg", False) else "auto",
        memory=getattr(args, "memory", None),
        smp=getattr(args, "smp", None),
        monitor=bool(getattr(args, "monitor", False)),
        display=getattr(args, "display", None),
        serial_log=Path(serial_log).resolve() if serial_log else None,
        trace=getattr(args, "trace", None),
        extra=tuple(extra),
    )


def cmd_run(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    options = qemu_options_from(args, ctx, settings)
    image = ctx.image(options.image)
    if args.no_build:
        if not image.exists():
            raise Failure(
                f"no {options.image} image to boot: {rel(image)}",
                hint="drop --no-build so xtool can create it",
            )
    else:
        header(ctx, world.manifest)
        build_chain(ctx, world.run, world.jinx, world.manifest, options.image)
    if ctx.dry_run:
        ctx.log.msg2("would boot: " + shlex.join(qemu_command(ctx, world.run, image, options)))
        return 0
    run_qemu(ctx, world.run, image, options)
    return 0


def jinx_recipe_paths(name: str) -> tuple[Path, Path, bool]:
    """Return (work tree, working patch, is host recipe) for one Jinx package."""
    is_host = name.startswith("host:")
    package = name.removeprefix("host:")
    if (
        not package
        or "/" in package
        or package in (".", "..")
        or any(character in package for character in "*?[")
    ):
        raise Failure(f"invalid package name: {name!r}")
    recipes_dir = USERLAND_DIR / ("host-recipes" if is_host else "recipes")
    sources_dir = USERLAND_DIR / ("host-sources" if is_host else "sources")
    recipe = recipes_dir / package / "recipe"
    if not recipe.is_file():
        raise Failure(
            f"no such {'host ' if is_host else ''}recipe: {package}",
            hint="run './x.py list' to see the available target packages",
        )
    return (
        sources_dir / f"{package}-workdir",
        recipes_dir / package / "patches" / "jinx-working-patch.patch",
        is_host,
    )


def cmd_build(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    name = args.package
    _, _, is_host = jinx_recipe_paths(name)
    header(ctx, world.manifest)
    prefetch_packages(ctx, world.jinx, ("host:*",))
    prefetch_packages(ctx, world.jinx, (name,))
    action = "Rebuilding" if ctx.force else "Building"
    ctx.log.msg(f"{action} package {name}")
    world.jinx.build(name, force=ctx.force)
    if is_host:
        ctx.log.msg2("host package built; host packages are consumed by Jinx, not the target sysroot")
    else:
        ctx.log.msg(f"Installing package {name} into {rel(ctx.sysroot)}")
        if not ctx.dry_run:
            ensure_dir(ctx.sysroot)
        world.jinx.install(ctx.sysroot, [name], force=ctx.force)
        if not ctx.dry_run:
            Cache(ctx).record(
                "sysroot",
                fingerprint(
                    "package-install",
                    values=(ctx.arch.name, name, str(time.time_ns())),
                ),
                (ctx.sysroot,),
            )
    ctx.log.finished(f"package {name}", time.monotonic() - ctx.started)
    if not is_host:
        ctx.log.note("test the installed result with: ./x.py run")
    return 0


def cmd_regen(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    name = args.package
    work_tree, patch, _ = jinx_recipe_paths(name)
    ctx.log.msg(f"Regenerating the working patch for {name}")
    ctx.log.note(f"reading local edits from {rel(work_tree)}")
    world.jinx.regen(name)
    ctx.log.msg2(f"updated {rel(patch)}")
    ctx.log.note(f"verify it with: ./x.py build -f {name}")
    return 0


def cmd_status(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    log = ctx.log
    components = survey(
        ctx,
        world.jinx,
        world.manifest,
        default_image=str(setting(settings, "image", None, "hdd")),
    )
    if args.json_output:
        print(
            json.dumps(
                {
                    "arch": ctx.arch.name,
                    "profile": ctx.profile,
                    "sysroot": str(ctx.sysroot),
                    "components": [
                        {"name": item.name, "state": item.state, "detail": item.detail}
                        for item in components
                    ],
                },
                indent=2,
            )
        )
        return 0

    log.msg(f"Roanix {ctx.arch.name} ({ctx.profile})")
    log.field("repository", str(ROOT))
    log.field("artifacts", rel(ctx.artifacts))
    log.field("sysroot", rel(ctx.sysroot) + (f" - {age(ctx.sysroot)}" if ctx.sysroot.is_dir() else " - absent"))
    log.field("recipes", f"{len(world.recipes.names)} packages, {len(world.manifest.install)} installed")
    log.plain()

    log.msg("Components")
    marks = {
        "ready": (log.paint("ready  ", Log.GREEN)),
        "stale": (log.paint("stale  ", Log.YELLOW)),
        "missing": (log.paint("missing", Log.RED)),
    }
    for item in components:
        log.plain(f"     {item.name:<14} {marks[item.state]}  {log.paint(item.detail, Log.DIM)}")
    log.plain()

    pending = [item for item in components if item.state != "ready"]
    if pending:
        log.msg("Next")
        log.msg2("./x.py            build what changed and boot it")
    else:
        log.msg("Everything is up to date")
        log.msg2("./x.py run --no-build     boot the existing image immediately")
    return 0


def cmd_list(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    log = ctx.log
    if args.what == "targets":
        log.msg("Build targets")
        for target, description in (
            ("kernel", "the Rust kernel binary"),
            ("sysroot", "every userland package, installed into a sysroot"),
            ("initramfs", "the sysroot packed as a boot module"),
            ("hdd", "a bootable GPT disk image"),
            ("iso", "a bootable hybrid ISO"),
            ("all", "both images"),
        ):
            log.plain(f"     {target:<12} {log.paint(description, Log.DIM)}")
        return 0

    built = world.jinx.built()
    installed = set(world.recipes.dependency_closure(world.manifest.install))
    rows = []
    for name in world.recipes.names:
        recipe = world.recipes.get(name)
        present = sorted(built.get(name, ()))
        wanted = recipe.version_revision
        if not recipe.version and present:
            wanted = present[-1].replace("_", "-")
        if not present:
            state, style = "not built", Log.DIM
        elif world.jinx.has_package(recipe, built):
            state, style = "built", Log.GREEN
        else:
            state, style = f"needs rebuild (have {present[-1].replace('_', '-')})", Log.YELLOW
        rows.append((name, wanted, state, style, name in installed))

    if args.json_output:
        print(
            json.dumps(
                [
                    {"name": n, "version": v, "built": s == "built", "in_image": i}
                    for n, v, s, _, i in rows
                ],
                indent=2,
            )
        )
        return 0

    name_width = max((len(row[0]) for row in rows), default=4) + 2
    version_width = max((len(row[1]) for row in rows), default=7) + 2
    log.msg(f"Userland packages ({len(rows)})")
    log.plain(
        "       "
        + log.paint("NAME".ljust(name_width) + "VERSION".ljust(version_width) + "STATUS", Log.BOLD)
    )
    for name, version, state, style, in_image in rows:
        mark = log.paint("*", Log.CYAN) if in_image else " "
        log.plain(
            f"     {mark} {name.ljust(name_width)}{version.ljust(version_width)}"
            + log.paint(state, style)
        )
    log.plain()
    log.note(f"{log.paint('*', Log.CYAN)} = part of the system image ({rel(SYSTEM_MANIFEST)})")
    return 0


def cmd_doctor(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    log = ctx.log
    if args.json_output:
        log.level = QUIET  # collect everything silently, then emit one JSON blob
    groups: dict[str, tuple[str, ...]] = {
        "kernel": ("cargo", "rustup"),
        "userland": ("bash", "git", "make", "tar", "gzip", "zstd", "sed", "awk", "grep", "find"),
        "images": ("xorriso", "sgdisk", "mformat", "mmd", "mcopy"),
        "emulator": (ctx.arch.qemu,),
    }
    problems: list[tuple[str, str | None]] = []
    report: dict[str, Any] = {"host": platform.platform(), "python": platform.python_version()}

    if platform.system() != "Linux":
        problems.append(("Roanix can only be built on Linux hosts", None))

    log.msg(f"Host environment for {ctx.arch.name}")
    log.field("system", platform.platform())
    log.field("python", platform.python_version())
    log.field("cpus", str(os.cpu_count() or 1) + f" (using -j{ctx.jobs})")
    log.plain()

    tools: dict[str, Any] = {}
    for group, names in groups.items():
        log.msg2(group)
        for tool in names:
            path = shutil.which(tool)
            tools[tool] = path
            if path:
                code, out = capture([tool, "--version"])
                version = out.splitlines()[0].strip() if code == 0 and out else ""
                log.plain(
                    f"       {log.paint('ok', Log.GREEN)}      {tool.ljust(20)}"
                    f"{log.paint(version[:56], Log.DIM)}"
                )
            else:
                packages = TOOL_PACKAGES.get(tool, {})
                manager = detect_package_manager()
                package = packages.get(manager) or tool
                log.plain(
                    f"       {log.paint('missing', Log.RED)} {tool.ljust(20)}"
                    f"{log.paint(install_command(manager, package), Log.DIM)}"
                )
                problems.append((f"missing tool: {tool}", install_command(manager, package)))
    report["tools"] = tools

    downloader = shutil.which("wget") or shutil.which("curl")
    if not downloader:
        problems.append(("neither wget nor curl is available", "install wget"))

    if shutil.which("rustup"):
        code, out = capture(["rustup", "target", "list", "--installed"], cwd=KERNEL_DIR)
        installed = out.split() if code == 0 else []
        report["rust_targets"] = installed
        log.plain()
        log.msg2("rust targets")
        if ctx.arch.rust_target in installed:
            log.plain(f"       {log.paint('ok', Log.GREEN)}      {ctx.arch.rust_target}")
        else:
            log.plain(f"       {log.paint('missing', Log.RED)} {ctx.arch.rust_target}")
            problems.append(
                (
                    f"rust target not installed: {ctx.arch.rust_target}",
                    f"rustup target add {ctx.arch.rust_target}",
                )
            )

    if ctx.arch.name == "x86_64":
        log.plain()
        log.msg2("acceleration")
        if kvm_available():
            log.plain(f"       {log.paint('ok', Log.GREEN)}      /dev/kvm is usable")
        else:
            log.plain(
                f"       {log.paint('note', Log.YELLOW)}    /dev/kvm is unavailable; "
                f"{log.paint('QEMU will fall back to TCG', Log.DIM)}"
            )

    report["problems"] = [message for message, _ in problems]
    if args.json_output:
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if not problems else 1

    log.plain()
    if problems:
        log.msg(f"{len(problems)} problem(s) found")
        for message, fix in problems:
            log.msg2(message)
            if fix:
                log.note(fix)
        return 1
    log.msg("Everything xtool needs is installed")
    log.msg2("build and boot Roanix with: ./x.py")
    return 0


def cmd_config(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    log = world.ctx.log
    stored = load_settings()
    if args.action in (None, "show", "list"):
        log.msg("Saved defaults")
        if not stored:
            log.msg2(f"none yet - try: ./x.py config set arch {world.ctx.arch.name}")
        for key in sorted(SETTINGS_KEYS):
            value = stored.get(key)
            shown = (
                value.ljust(18)
                if value is not None
                else log.paint("(unset)".ljust(18), Log.DIM)
            )
            log.plain(f"     {key.ljust(12)}{shown}{log.paint(SETTINGS_KEYS[key], Log.DIM)}")
        log.plain()
        log.msg("Effective configuration")
        log.field("arch", world.ctx.arch.name)
        log.field("profile", world.ctx.profile)
        log.field("jobs", str(world.ctx.jobs))
        log.field("config file", rel(CONFIG_FILE))
        return 0
    if args.action == "set":
        if not args.key or args.value is None:
            raise Failure("usage: ./x.py config set <key> <value>")
        if args.key not in SETTINGS_KEYS:
            raise Failure(
                f"unknown setting: {args.key}",
                hint="valid settings: " + ", ".join(sorted(SETTINGS_KEYS)),
            )
        validate_setting(args.key, args.value)
        stored[args.key] = args.value
        save_settings(stored)
        log.msg(f"{args.key} = {args.value}")
        return 0
    if args.action == "unset":
        if not args.key:
            raise Failure("usage: ./x.py config unset <key>")
        stored.pop(args.key, None)
        save_settings(stored)
        log.msg(f"cleared {args.key}")
        return 0
    raise Failure(f"unknown config action: {args.action}")


def validate_setting(key: str, value: str) -> None:
    if key == "arch" and value not in ARCHES:
        raise Failure(f"unknown architecture: {value}", hint=", ".join(ARCHES))
    if key == "profile" and value not in PROFILES:
        raise Failure(f"unknown profile: {value}", hint=", ".join(PROFILES))
    if key == "image" and value not in ("hdd", "iso"):
        raise Failure(f"unknown image format: {value}", hint="hdd, iso")
    if key == "firmware" and value not in ("uefi", "bios"):
        raise Failure(f"unknown firmware: {value}", hint="uefi, bios")
    if key == "jobs" and (not value.isdigit() or int(value) < 1):
        raise Failure("jobs must be a positive integer")


CLEAN_TARGETS: Mapping[str, str] = {
    # per-architecture
    "out": "kernel binary, initramfs, and images for this arch and profile",
    "cargo": "the Cargo target directory for this architecture",
    "sysroot": "the installed sysroot for this architecture",
    "packages": "this architecture's Jinx build tree and packages",
    "state": "this architecture's incremental build state",
    "arch": "everything for this architecture (all of the above)",
    # shared by every architecture
    "sources": "downloaded and patched userland sources (shared)",
    "cache": "pinned Limine, OVMF, and Jinx downloads (shared)",
    "all": "everything under build/",
}

#: Targets that wipe work shared by every architecture.
SHARED_CLEAN_TARGETS = {"sources", "cache"}
#: Targets whose removal forces a very long rebuild.
EXPENSIVE_CLEAN_TARGETS = {"packages", "sources", "cache", "arch", "all"}


def clean_plan(ctx: Context) -> list[tuple[str, list[Path]]]:
    return [
        ("out", [ctx.artifacts]),
        ("cargo", [ctx.cargo_arch_dir]),
        ("sysroot", [ctx.sysroot]),
        ("packages", [ctx.jinx_build]),
        ("state", [ctx.state_dir, ctx.work]),
        ("arch", [ctx.arch_root, ctx.cargo_arch_dir]),
        ("sources", [
            USERLAND_DIR / "sources",
            USERLAND_DIR / "host-sources",
            USERLAND_DIR / ".jinx-cache",
        ]),
        ("cache", [CACHE_DIR]),
        ("all", [BUILD_DIR]),
    ]


def cmd_clean(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    log = ctx.log
    wanted = set(args.targets or ["out", "state"])
    unknown = wanted - set(CLEAN_TARGETS)
    if unknown:
        raise Failure(
            f"unknown clean target(s): {', '.join(sorted(unknown))}",
            hint="valid targets: " + ", ".join(CLEAN_TARGETS),
        )
    if "all" in wanted:
        wanted = {"all", "sources"}

    expensive = wanted & EXPENSIVE_CLEAN_TARGETS
    if expensive and not ctx.assume_yes and sys.stdin.isatty():
        log.warn(f"about to remove: {', '.join(sorted(expensive))}")
        log.hint("this forces a full userland rebuild, which takes a long time")
        if input("  -> continue? [y/N] ").strip().lower() not in ("y", "yes"):
            log.msg("Nothing was removed")
            return 1

    shared = wanted & SHARED_CLEAN_TARGETS or ({"all"} & wanted)
    log.msg(f"Cleaning {ctx.arch.name}" + (" and shared state" if shared else ""))
    removed = 0
    for target, paths in clean_plan(ctx):
        if target not in wanted:
            continue
        for path in paths:
            if not path.exists():
                continue
            if not is_inside_repo(path):
                log.warn(f"refusing to remove {path} (outside the repository)")
                continue
            size = tree_size(path)
            removed += size
            scope = " (shared)" if target in SHARED_CLEAN_TARGETS else ""
            log.msg2(f"{rel(path)} ({human_size(size)}){scope}")
            if not ctx.dry_run:
                remove(path)
    if "all" in wanted and not ctx.dry_run:
        for pattern in ("roanix-*.iso", "roanix-*.hdd"):
            for stray in ROOT.glob(pattern):
                remove(stray)
        remove(BOOK_DIR / "book")
    log.msg2(f"reclaimed {human_size(removed)}")
    log.finished("cleaning", time.monotonic() - ctx.started)
    return 0


def is_inside_repo(path: Path) -> bool:
    try:
        resolved = path.resolve()
    except OSError:
        return False
    if resolved == ROOT.resolve():
        return False
    return resolved.is_relative_to(ROOT.resolve())


def cmd_check(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    ctx.log.msg(f"Type-checking the kernel ({ctx.arch.name})")
    cargo(ctx, world.run, "check", "--target", ctx.arch.rust_target, "--profile", ctx.profile)
    ctx.log.finished("the check", time.monotonic() - ctx.started)
    return 0


def cmd_lint(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    ctx.log.msg(f"Running Clippy ({ctx.arch.name})")
    cargo(
        ctx, world.run, "clippy",
        "--target", ctx.arch.rust_target, "--profile", ctx.profile,
        "--", "-D", "warnings",
    )
    ctx.log.finished("the lint", time.monotonic() - ctx.started)
    return 0


def cmd_fmt(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    argv = ["cargo", "fmt", "--all"]
    if args.check:
        argv += ["--", "--check"]
    ctx.log.msg("Checking Rust formatting" if args.check else "Formatting Rust sources")
    world.run(argv, cwd=KERNEL_DIR, mode="stream")
    return 0


def cmd_docs(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    if args.kind == "book":
        argv = ["mdbook", "serve", "--port", str(args.port)] if args.serve else ["mdbook", "build"]
        ctx.log.msg("Serving the Roanix book" if args.serve else "Building the Roanix book")
        if args.serve:
            ctx.log.note(f"http://127.0.0.1:{args.port}")
        world.run(argv, cwd=BOOK_DIR, mode="raw" if args.serve else "stream")
        return 0
    ctx.log.msg(f"Building the kernel API documentation ({ctx.arch.name})")
    cargo(ctx, world.run, "doc", "--no-deps", "--target", ctx.arch.rust_target)
    docs = ctx.cargo_arch_dir / "doc"
    ctx.log.msg2(rel(docs))
    if args.serve:
        ctx.log.note(f"http://127.0.0.1:{args.port}/roanix/index.html")
        world.run([sys.executable, "-m", "http.server", str(args.port)], cwd=docs, mode="raw")
    return 0


def cmd_fetch(world: World, args: argparse.Namespace, settings: Mapping[str, str]) -> int:
    ctx = world.ctx
    ctx.log.msg("Fetching pinned build resources")
    wanted = args.resources or ["limine", "ovmf", "jinx"]
    for resource in wanted:
        if resource in TARBALLS:
            install_tarball(ctx, TARBALLS[resource], world.run)
        elif resource == "jinx":
            ensure_jinx(ctx, world.run)
        else:
            raise Failure(
                f"unknown resource: {resource}", hint="valid resources: limine, ovmf, jinx"
            )
        ctx.log.msg2(f"{resource} is ready")
    return 0


def header(ctx: Context, manifest: Manifest) -> None:
    ctx.log.msg(
        f"Roanix {ctx.arch.name} ({ctx.profile})"
        + (" [forced rebuild]" if ctx.force else "")
    )
    if manifest.generated:
        ctx.log.msg2(f"create {rel(SYSTEM_MANIFEST)} to control the package set")


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------

OVERVIEW = """xtool builds, packages, and boots the Roanix operating system.

basic commands:
  run                 build whatever changed and boot it in QEMU  (default)
  build               build and install one Jinx package
  status              show what is built, what is stale, and what is next

userland and Jinx:
  build               build/install a package; -f rebuilds from prepared sources
  regen               update jinx-working-patch.patch from the package work tree
  list                list userland packages or build targets

kernel:
  check               cargo check the kernel
  lint                cargo clippy with warnings denied
  fmt                 format the Rust sources
  docs                build or serve the book and the kernel API docs

environment:
  doctor              verify the host has everything xtool needs
  config              show or change saved defaults such as the architecture
  fetch               pre-download Limine, OVMF, and Jinx
  clean               remove build artefacts
"""

EXAMPLES = """workflow examples:
  ./x.py                              build what changed and boot it
  ./x.py -a riscv64                   the same, targeting riscv64
  ./x.py -r                           build and boot a release kernel

  ./x.py build bash                   build Bash and install it into the sysroot
  ./x.py build -f bash                rebuild Bash from the prepared source tree
  ./x.py regen bash                   turn local source edits into the working patch

  ./x.py run --gdb                    boot halted with a GDB stub on :1234
  ./x.py run iso --firmware bios      boot the ISO through SeaBIOS
  ./x.py run -- -d int -no-reboot     pass raw flags straight to QEMU

  ./x.py config set arch riscv64      stop typing --arch every time
  ./x.py -A status                    report on every architecture
  ./x.py status                       what would a build actually do?

Run './x.py help <command>' for the full options of one command.
"""

COMMANDS = (
    "run", "build", "status", "regen", "list", "check", "lint", "fmt",
    "docs", "doctor", "config",
    "fetch", "clean", "help",
)
ALIASES = {"r": "run", "b": "build", "st": "status", "ls": "list"}
VALUE_OPTIONS = {
    "-a", "--arch", "-p", "--profile", "-j", "--jobs", "--color", "-m",
    "--memory", "--smp", "--gdb-port", "--display", "--serial-log", "--trace",
    "--image", "--firmware", "-F", "--port",
}


class Formatter(argparse.RawDescriptionHelpFormatter):
    def __init__(self, prog: str, **kwargs: Any) -> None:
        super().__init__(prog, max_help_position=32, width=96, **kwargs)


def global_options() -> argparse.ArgumentParser:
    # SUPPRESS keeps unset options out of the namespace entirely, so that a
    # subparser's copy of --arch cannot overwrite a value the root parser
    # already accepted (`./x.py -a riscv64 status`).
    parser = argparse.ArgumentParser(add_help=False, argument_default=argparse.SUPPRESS)
    group = parser.add_argument_group("common options")
    group.add_argument(
        "-a", "--arch", action="append", choices=tuple(ARCHES),
        help="target architecture; repeat to act on several",
    )
    group.add_argument(
        "-A", "--all-arches", action="store_true", help="act on every architecture",
    )
    group.add_argument("-p", "--profile", choices=PROFILES, help="Cargo profile")
    group.add_argument(
        "-r", "--release", action="store_true", help="shorthand for --profile release"
    )
    group.add_argument("-j", "--jobs", type=int, help="parallel jobs for Cargo and Jinx")
    group.add_argument(
        "-f", "--force", action="store_true", help="discard build/install outputs before rebuilding"
    )
    group.add_argument("-q", "--quiet", action="store_true", help="only print warnings and errors")
    group.add_argument("-v", "--verbose", action="store_true", help="show every command and all output")
    group.add_argument("--color", choices=("auto", "always", "never"), help="colourise output")
    group.add_argument("--offline", action="store_true", help="never touch the network")
    group.add_argument("-n", "--dry-run", action="store_true", help="show what would happen")
    group.add_argument("-y", "--yes", action="store_true", help="assume yes for confirmations")
    return parser


def qemu_options() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    group = parser.add_argument_group("emulator options")
    group.add_argument("-F", "--firmware", choices=("uefi", "bios"), help="firmware to boot with")
    group.add_argument("--gdb", action="store_true", help="expose a GDB stub and halt at reset")
    group.add_argument("--gdb-port", type=int, default=1234, help="port for the GDB stub")
    group.add_argument("--no-wait", action="store_true", help="with --gdb, start running immediately")
    group.add_argument("--tcg", action="store_true", help="disable KVM and use pure emulation")
    group.add_argument("-m", "--memory", help="guest memory size, e.g. 2G")
    group.add_argument("--smp", type=int, help="number of guest CPUs")
    group.add_argument("--monitor", action="store_true", help="multiplex the QEMU monitor onto serial")
    group.add_argument("--display", help="QEMU display backend, e.g. none")
    group.add_argument("--serial-log", help="also write the serial console to this file")
    group.add_argument("--trace", help="QEMU -d items to log, e.g. int,cpu_reset")
    return parser


def build_cli() -> tuple[argparse.ArgumentParser, dict[str, argparse.ArgumentParser]]:
    common = global_options()
    emulator = qemu_options()
    parser = argparse.ArgumentParser(
        prog="x.py",
        parents=[common],
        formatter_class=Formatter,
        usage="%(prog)s [<command>] [options] [-- qemu arguments]",
        description=OVERVIEW,
        epilog=EXAMPLES,
        add_help=False,
    )
    parser.add_argument("-h", "--help", action="help", help="show this message")
    parser.add_argument("-V", "--version", action="version", version=f"xtool {VERSION}")
    subparsers = parser.add_subparsers(
        dest="command", metavar="<command>", help=argparse.SUPPRESS
    )
    registry: dict[str, argparse.ArgumentParser] = {}

    def add(name: str, aliases: Sequence[str] = (), **kwargs: Any) -> argparse.ArgumentParser:
        sub = subparsers.add_parser(
            name,
            aliases=list(aliases),
            parents=kwargs.pop("parents", [common]),
            formatter_class=Formatter,
            **kwargs,
        )
        registry[name] = sub
        return sub

    run = add(
        "run", ["r"], parents=[common, emulator],
        description="Build anything that changed, then boot Roanix in QEMU.",
        epilog="examples:\n"
               "  ./x.py run\n"
               "  ./x.py run iso --arch riscv64\n"
               "  ./x.py run --gdb --tcg\n"
               "  ./x.py run --no-build          boot the existing image as-is\n"
               "  ./x.py run -- -d int           pass raw arguments to QEMU\n",
    )
    run.add_argument("image", nargs="?", choices=("hdd", "iso"), help="image format to boot")
    run.add_argument("--no-build", action="store_true", help="boot the existing image without rebuilding")

    build = add(
        "build", ["b"],
        description=(
            "Build one Jinx package and install it into the target sysroot.\n"
            "With -f, discard its build/install outputs and rebuild from the prepared source tree."
        ),
        epilog="examples:\n"
               "  ./x.py build bash\n"
               "  ./x.py build -f bash\n"
               "  ./x.py build host:cmake\n",
    )
    build.add_argument("package", help="recipe name, optionally prefixed with host:")

    regen = add(
        "regen",
        description=(
            "Regenerate jinx-working-patch.patch from edits in the package's work tree,\n"
            "then re-run the recipe's prepare() step. Host recipes use the host: prefix."
        ),
    )
    regen.add_argument("package", help="recipe whose patches should be regenerated")

    listing = add("list", ["ls"], description="List userland packages or build targets.")
    listing.add_argument(
        "what", nargs="?", choices=("packages", "targets"), default="packages", help="what to list"
    )
    listing.add_argument("--json", dest="json_output", action="store_true", help="emit JSON")

    status = add("status", ["st"], description="Report what is built and what is stale.")
    status.add_argument("--json", dest="json_output", action="store_true", help="emit JSON")

    add("check", description="Type-check the kernel with cargo check.")
    add("lint", description="Run Clippy over the kernel with warnings denied.")
    fmt = add("fmt", description="Format the kernel sources with rustfmt.")
    fmt.add_argument("--check", action="store_true", help="fail instead of rewriting files")

    docs = add("docs", description="Build or serve the Roanix book or the kernel API docs.")
    docs.add_argument("kind", nargs="?", choices=("book", "rust"), default="book", help="which docs")
    docs.add_argument("--serve", action="store_true", help="serve them over HTTP")
    docs.add_argument("--port", type=int, default=8080, help="port for --serve")

    doctor = add("doctor", description="Check that the host has every tool xtool needs.")
    doctor.add_argument("--json", dest="json_output", action="store_true", help="emit JSON")

    config = add(
        "config",
        description="Show or change the defaults xtool remembers between runs.",
        epilog="examples:\n"
               "  ./x.py config                      show everything\n"
               "  ./x.py config set arch riscv64\n"
               "  ./x.py config set qemu-args '-display none'\n"
               "  ./x.py config unset profile\n",
    )
    config.add_argument("action", nargs="?", choices=("show", "list", "set", "unset"), help="what to do")
    config.add_argument("key", nargs="?", help="setting name")
    config.add_argument("value", nargs="?", help="new value")

    fetch = add("fetch", description="Pre-download Limine, OVMF, and Jinx.")
    fetch.add_argument("resources", nargs="*", choices=("limine", "ovmf", "jinx", []), help="what to fetch")

    clean = add(
        "clean",
        description=(
            "Remove build artefacts.\n\n"
            "Per-architecture targets only ever touch build/<arch>/, so cleaning one\n"
            "architecture never disturbs another. Targets:\n"
            + "\n".join(f"  {name:<10} {text}" for name, text in CLEAN_TARGETS.items())
        ),
        epilog="examples:\n"
               "  ./x.py clean                   this arch's images and build state\n"
               "  ./x.py clean arch -a riscv64   forget riscv64 entirely\n"
               "  ./x.py clean all --yes         the whole build/ tree\n",
    )
    clean.add_argument("targets", nargs="*", help="what to remove (default: out state)")

    help_parser = add("help", description="Show help for a command.")
    help_parser.add_argument("topic", nargs="?", help="command name")
    return parser, registry


HANDLERS: Mapping[str, Callable[[World, argparse.Namespace, Mapping[str, str]], int]] = {
    "run": cmd_run,
    "build": cmd_build,
    "regen": cmd_regen,
    "status": cmd_status,
    "list": cmd_list,
    "check": cmd_check,
    "lint": cmd_lint,
    "fmt": cmd_fmt,
    "docs": cmd_docs,
    "doctor": cmd_doctor,
    "config": cmd_config,
    "fetch": cmd_fetch,
    "clean": cmd_clean,
}

#: Commands that never need Jinx, a sysroot, or the recipe graph.
LIGHTWEIGHT = {"doctor", "config", "fetch", "check", "lint", "fmt", "docs", "clean"}

#: Commands that can sensibly act on several architectures in one invocation.
MULTI_ARCH = {"build", "status", "list", "clean", "fetch", "check", "lint", "doctor"}


def locate_command(argv: Sequence[str]) -> str | None:
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "--":
            return None
        if token in VALUE_OPTIONS:
            index += 2
            continue
        if token.startswith("-"):
            index += 1
            continue
        return token
    return None


def split_qemu_arguments(argv: Sequence[str], command: str | None) -> tuple[list[str], list[str]]:
    """Everything after a bare ``--`` goes to QEMU."""
    if "--" not in argv:
        return list(argv), []
    index = list(argv).index("--")
    return list(argv[:index]), list(argv[index + 1 :])


def opt(args: argparse.Namespace, name: str, default: Any = None) -> Any:
    """Read an option that may have been suppressed when it was not given."""
    return getattr(args, name, default)


def resolve_arches(args: argparse.Namespace, settings: Mapping[str, str]) -> tuple[str, ...]:
    """Which architectures should this invocation act on?"""
    if opt(args, "all_arches"):
        return tuple(ARCHES)
    selected = opt(args, "arch") or []
    if not selected:
        selected = [str(setting(settings, "arch", None, DEFAULT_ARCH))]
    ordered = tuple(dict.fromkeys(selected))
    unknown = [name for name in ordered if name not in ARCHES]
    if unknown:
        raise Failure(
            f"unknown architecture: {', '.join(unknown)}",
            hint="supported architectures: " + ", ".join(ARCHES),
        )
    return ordered


def make_context(
    args: argparse.Namespace, settings: Mapping[str, str], log: Log, arch_name: str
) -> Context:
    profile = (
        "release"
        if opt(args, "release")
        else str(setting(settings, "profile", opt(args, "profile"), DEFAULT_PROFILE))
    )
    if profile not in PROFILES:
        raise Failure(f"unknown profile: {profile}", hint="supported profiles: " + ", ".join(PROFILES))
    jobs = int(setting(settings, "jobs", opt(args, "jobs"), os.cpu_count() or 1))
    return Context(
        arch=ARCHES[arch_name],
        profile=profile,
        log=log,
        jobs=max(1, jobs),
        force=bool(opt(args, "force")),
        offline=bool(opt(args, "offline")),
        dry_run=bool(opt(args, "dry_run")),
        assume_yes=bool(opt(args, "yes")),
    )


def command_suggestions(token: str) -> list[str]:
    """Turn a plausible-looking mistake into an actionable hint."""
    import difflib

    hints: list[str] = []
    if token in TARGETS:
        if token in ("hdd", "iso"):
            hints.append(f"to boot it, run: ./x.py run {token}")
        else:
            hints.append("that OS component is assembled by: ./x.py run")
        return hints
    if (RECIPES_DIR / token / "recipe").is_file():
        hints.append(f"that is a userland package; run: ./x.py build {token}")
        return hints
    close = difflib.get_close_matches(token, COMMANDS, n=3, cutoff=0.4)
    if close:
        hints.append("did you mean: " + ", ".join(close) + "?")
    return hints


def main(argv: Sequence[str] | None = None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    command = locate_command(raw)
    resolved = ALIASES.get(command or "", command)

    if resolved is not None and resolved not in COMMANDS:
        log = Log()
        log.error(f"unknown command: {resolved}")
        for suggestion in command_suggestions(resolved):
            log.hint(suggestion)
        log.hint("run './x.py --help' for the command list")
        return 2

    raw, qemu_extra = split_qemu_arguments(raw, resolved)
    if command is None and not any(item in raw for item in ("-h", "--help", "-V", "--version")):
        raw.append("run")  # `./x.py` and `./x.py -a riscv64` both mean "run"

    parser, registry = build_cli()
    args = parser.parse_args(raw)
    if opt(args, "command") is None:
        parser.print_help()
        return 0
    name = ALIASES.get(args.command, args.command)

    if name == "help":
        topic = opt(args, "topic") or ""
        target = registry.get(ALIASES.get(topic, topic))
        (target or parser).print_help()
        return 0

    level = VERBOSE if opt(args, "verbose") else QUIET if opt(args, "quiet") else NORMAL
    log = Log(color=opt(args, "color") or "auto", level=level)

    try:
        settings = load_settings()
        arches = resolve_arches(args, settings)
        if len(arches) > 1 and name not in MULTI_ARCH:
            raise Failure(
                f"'{name}' works on one architecture at a time",
                hint=f"run it once per architecture, or use a command from: "
                f"{', '.join(sorted(MULTI_ARCH))}",
            )
        if qemu_extra and name != "run":
            raise Failure("arguments after '--' are only understood by run")

        manifest = Manifest((), (), True) if name in LIGHTWEIGHT else load_manifest(log)
        status = 0
        for index, arch_name in enumerate(arches):
            if len(arches) > 1:
                log.plain()
                log.msg(log.paint(f"[{index + 1}/{len(arches)}] {arch_name}", Log.MAGENTA))
            ctx = make_context(args, settings, log, arch_name)
            ctx.qemu_args = tuple(qemu_extra)
            runner = Runner(ctx)
            recipes = Recipes()
            world = World(ctx, runner, recipes, manifest, Jinx(ctx, runner, recipes))
            status = max(status, HANDLERS[name](world, args, settings))
        return status
    except Failure as error:
        log.separator()
        for line in str(error).splitlines():
            if line.startswith("    "):
                log.detail(line)
            else:
                log.error(line)
        if error.hint:
            log.hint(error.hint)
        return 1
    except KeyboardInterrupt:
        log.separator()
        log.error("interrupted")
        return 130
    except BrokenPipeError:
        return 0


if __name__ == "__main__":
    try:
        status = main()
        sys.stdout.flush()
    except BrokenPipeError:
        # Something like `./x.py list | head` closed the pipe early; make sure
        # the interpreter does not complain again while shutting down.
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        status = 0
    raise SystemExit(status)
