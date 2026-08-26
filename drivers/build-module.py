#!/usr/bin/env python3
"""Link a named Rust staticlib workspace package as a driver module."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def run(arguments: list[str], *, cwd: Path) -> str:
    return subprocess.check_output(arguments, cwd=cwd, text=True).strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--target-dir", required=True, type=Path)
    parser.add_argument("--profile", default="release")
    parser.add_argument("--toolchain", default="nightly-2026-07-15")
    parser.add_argument("--imports", action="store_true")
    arguments = parser.parse_args()
    arguments.output = arguments.output.resolve()
    arguments.target_dir = arguments.target_dir.resolve()

    cargo = ["cargo", f"+{arguments.toolchain}"]
    cargo_arguments = [
        *cargo,
        "build",
        "--manifest-path",
        "Cargo.toml",
        "--package",
        arguments.package,
        "--target",
        arguments.target,
        "--profile",
        arguments.profile,
    ]
    if arguments.imports:
        cargo_arguments += ["--features", "kernel-imports"]
    if arguments.target.startswith("riscv64"):
        # The distributed RISC-V `alloc` archive contains absolute relocations
        # that cannot be linked into a loadable shared object. Rebuild the
        # no_std runtime with the module's PIC model so Rust modules using
        # `alloc` remain position-independent.
        cargo_arguments += [
            "-Z",
            "build-std=core,alloc,compiler_builtins",
            "-Z",
            "build-std-features=compiler-builtins-mem",
        ]
    subprocess.run(
        cargo_arguments,
        cwd=ROOT,
        env={
            **os.environ,
            "CARGO_TARGET_DIR": str(arguments.target_dir),
            "RUSTFLAGS": f"{os.environ.get('RUSTFLAGS', '')} -Crelocation-model=pic".strip(),
        },
        check=True,
    )
    profile_dir = "debug" if arguments.profile == "dev" else arguments.profile
    archive_name = arguments.package.replace("-", "_")
    archive = arguments.target_dir / arguments.target / profile_dir / f"lib{archive_name}.a"
    if not archive.is_file():
        raise SystemExit(
            f"Cargo produced no static library for package {arguments.package!r} at {archive}"
        )

    sysroot = Path(run(["rustc", f"+{arguments.toolchain}", "--print", "sysroot"], cwd=ROOT))
    verbose = run(["rustc", f"+{arguments.toolchain}", "-vV"], cwd=ROOT)
    host = next(
        (line.removeprefix("host: ").strip() for line in verbose.splitlines() if line.startswith("host: ")),
        None,
    )
    if host is None:
        raise SystemExit("could not determine the Rust host triple")
    linker = sysroot / "lib" / "rustlib" / host / "bin" / "rust-lld"
    if not linker.is_file():
        found = shutil.which("rust-lld")
        if found is None:
            raise SystemExit(f"could not find rust-lld under {sysroot}")
        linker = Path(found)

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    linker_arguments = [
        str(linker),
        "-flavor",
        "gnu",
        "-shared",
        "-Bsymbolic",
        "--gc-sections",
        "--build-id=none",
        "--strip-debug",
        "--hash-style=sysv",
        "-z",
        "now",
        "-z",
        "relro",
        "-z",
        "separate-code",
        "-z",
        "noexecstack",
        "-z",
        "max-page-size=4096",
        "-soname",
        arguments.output.name,
        "-e",
        "rdf_module_entry",
        "-o",
        str(arguments.output),
        str(archive),
    ]
    if arguments.imports:
        linker_arguments.insert(5, "--allow-shlib-undefined")
    else:
        linker_arguments.insert(5, "--no-undefined")
    subprocess.run(
        linker_arguments,
        cwd=ROOT,
        check=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
