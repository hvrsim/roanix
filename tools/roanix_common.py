"""Workspace paths and shared subprocess and artifact helpers."""

import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build"
CACHE = BUILD / "cache"
WORK = BUILD / "work"
DIST = BUILD / "dist"
EDIT = BUILD / "edit"
USERLAND = ROOT / "userland"

VERBOSE = False


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


def run(*args, cwd=ROOT, env=None, capture=False, check=True, quiet=False):
    args = [str(arg) for arg in args]
    if VERBOSE:
        msg(shlex.join(args), detail=True)

    result = subprocess.run(
        args,
        cwd=cwd,
        env=os.environ | (env or {}),
        text=True,
        stdout=subprocess.PIPE if capture or quiet else None,
        stderr=subprocess.STDOUT if quiet else None,
        check=False,
    )
    if check and result.returncode:
        if result.stdout:
            print(result.stdout, file=sys.stderr, end="")
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
    tools = [Path(__file__), ROOT / "tools/roanix_build.py", ROOT / "tools/roanix_packages.py"]
    for path in [*tools, *inputs]:
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
            previous = Path(scratch) / "previous"
            output.rename(previous)
            try:
                temporary.replace(output)
            except OSError:
                previous.rename(output)
                raise
        else:
            temporary.replace(output)
    atomic_text(marker, digest)
    return True


def atomic_text(path, text):
    with tempfile.TemporaryDirectory(dir=path.parent) as scratch:
        temporary = Path(scratch) / path.name
        temporary.write_text(text)
        temporary.replace(path)


def download_to(url, destination, timeout):
    request = urllib.request.Request(url, headers={"User-Agent": "Roanix-build/1.0"})
    with urllib.request.urlopen(request, timeout=timeout) as response, destination.open("wb") as output:
        shutil.copyfileobj(response, output)


def fetch_archive(name, version, url, checksum, archive_root, destination, *, offline):
    """Download and unpack a pinned tool or firmware archive when needed."""
    marker = destination / ".sha256"
    if marker.is_file() and marker.read_text() == checksum:
        return destination

    downloads = CACHE / "downloads"
    downloads.mkdir(parents=True, exist_ok=True)
    archive = downloads / f"{name}-{version}-{checksum[:12]}.tar.gz"
    if not archive.is_file():
        if offline:
            raise RuntimeError(f"missing cached {name}; run ./x.py fetch first")
        msg(f"Downloading {name} {version}")
        partial = archive.with_suffix(".part")
        try:
            download_to(url, partial, timeout=60)
            if sha256(partial) != checksum:
                raise RuntimeError(f"checksum mismatch for {name} {version}")
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
