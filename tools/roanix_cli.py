"""Command-line interface for the Roanix developer tools."""

import argparse
import contextlib
import fcntl
import json
import os
import selectors
import shutil
import subprocess
import sys
import tarfile
import time
import tomllib
from pathlib import Path

from . import roanix_common as common
from .roanix_build import BOOTSTRAP_PACKAGES, DOWNLOADS, MACHINES, SYSTEM_PROFILES, Build, resource
from .roanix_common import BUILD, CACHE, DIST, EDIT, ROOT, USERLAND, WORK, msg, remove, run
from .roanix_packages import Jinx, recipe_path


def positive(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def common_args(parser):
    parser.add_argument("--machine", choices=MACHINES, help="machine to build or boot")
    parser.add_argument("--profile", choices=SYSTEM_PROFILES, default="standard")
    parser.add_argument("--mode", choices=("dev", "release"))
    parser.add_argument("-j", "--jobs", type=positive, default=len(os.sched_getaffinity(0)))
    parser.add_argument("--offline", action="store_true", help="use cached inputs only")
    parser.add_argument("--packages", choices=("auto", "source"), default="auto",
                        help="package download policy (source skips binary downloads)")
    parser.add_argument("-v", "--verbose", action="store_true", help="show subprocess commands")


def vm_args(parser):
    parser.add_argument("--image", type=Path, help="boot a standalone .raw or .iso without building")
    parser.add_argument("--cpus", type=positive, help="guest CPU count (default: up to 4)")
    parser.add_argument("--memory", help="guest memory (default: 4G)")
    parser.add_argument("--bios", action="store_true", help="boot with x86 BIOS instead of UEFI")


def image_args(parser):
    parser.add_argument("--format", choices=("raw", "iso"), help="portable media format (default: raw)")


def parse_args(argv):
    shared = argparse.ArgumentParser(add_help=False)
    common_args(shared)
    parser = argparse.ArgumentParser(
        description="Build, image, and run Roanix.",
        parents=[shared],
        epilog="Quick start: ./x.py image makes a bootable disk. ./x.py run boots Roanix in QEMU.",
    )
    commands = parser.add_subparsers(dest="command", metavar="COMMAND")
    image_args(commands.add_parser("image", help="make portable media"))
    vm = commands.add_parser("run", help="build and boot in QEMU")
    vm_args(vm)
    vm.add_argument("--debug", action="store_true", help="pause QEMU for GDB")
    for name, help_text in (("build", "build one component"), ("check", "run Cargo checks")):
        command = commands.add_parser(name, help=help_text)
        component = command.add_subparsers(dest="target", required=True, metavar="COMPONENT")
        component.add_parser("kernel", help="the Roanix kernel")
        component.add_parser("drivers", help="all drivers selected for this machine")
        component.add_parser("driver", help="one driver for this machine").add_argument("name")
        if name == "build":
            component.add_parser("package", help="one Jinx recipe").add_argument("name")
    test = commands.add_parser("test", help="run a headless boot smoke test")
    test.add_argument("target", choices=("boot",))
    test.add_argument("--timeout", type=positive, default=120, help="boot timeout in seconds")
    vm_args(test)
    package = commands.add_parser("package", help="manage editable package sources")
    package.add_argument("target", choices=("edit", "export-patch"))
    package.add_argument("name")
    for name, help_text in (
        ("doctor", "check build and runtime prerequisites"),
        ("plan", "show selected inputs and cached outputs"),
        ("fetch", "prefetch build inputs for offline use"),
    ):
        command = commands.add_parser(name, help=help_text)
        command.add_argument("target", choices=("image", "run"))
        image_args(command)
    commands.add_parser("disk-usage", help="show how much space generated files use")
    cleanup = commands.add_parser("clean", help="remove rebuildable work")
    cleanup.add_argument("--all", action="store_true", help="clean every configured machine")
    cleanup.add_argument("--dry-run", action="store_true", help="show what would be removed")
    cache = commands.add_parser("cache", help="manage reusable downloads")
    cache.add_argument("target", choices=("prune",))
    cache.add_argument("--dry-run", action="store_true", help="show what would be removed")

    extra = []
    if "--" in argv:
        split = argv.index("--")
        argv, extra = argv[:split], argv[split + 1:]
    global_args, argv = shared.parse_known_args(argv)
    args = parser.parse_args(argv, namespace=global_args)
    if args.command is None:
        parser.print_help()
        return None
    if extra and args.command != "run":
        parser.error("only run accepts QEMU arguments after --")
    args.extra = extra
    if args.command == "clean" and args.all and args.machine:
        parser.error("choose --all or --machine for clean")
    args.image = getattr(args, "image", None)
    if args.image:
        args.image = args.image.resolve()
        if args.image.suffix not in (".iso", ".raw") or not args.image.is_file():
            parser.error("--image requires an existing standalone .raw or .iso file")
    if args.image:
        sidecar = args.image.with_suffix(f"{args.image.suffix}.json")
        if sidecar.is_file():
            manifest = json.loads(sidecar.read_text())
            if not isinstance(manifest, dict):
                parser.error(f"invalid image manifest {sidecar}")
            machine = manifest.get("machine")
            if isinstance(machine, str) and machine in MACHINES:
                if args.machine and args.machine != machine:
                    parser.error(f"this image is built for {machine}")
                args.machine = machine
            if manifest.get("mode") in ("dev", "release"):
                args.mode = manifest["mode"]
            profile = manifest.get("profile")
            if isinstance(profile, str) and profile in SYSTEM_PROFILES:
                args.profile = profile
        if args.machine is None:
            args.machine = next(
                (name for name in MACHINES if args.image.name.startswith(f"roanix-{name}-")),
                None,
            )
    local = BUILD / "local/config.toml"
    settings = tomllib.loads(local.read_text()) if local.is_file() else {}
    args.machine = args.machine or settings.get("machine", "pc-x86_64")
    if args.machine not in MACHINES:
        parser.error(f"unknown machine in {local.relative_to(ROOT)}: {args.machine}")
    args.arch = MACHINES[args.machine]["arch"]
    if args.command in ("run", "test"):
        args.cpus = args.cpus or settings.get("cpus")
        args.memory = args.memory or settings.get("memory", "4G")
        if args.cpus is not None and (type(args.cpus) is not int or args.cpus < 1):
            parser.error("cpus in the local configuration must be a positive integer")
        if not isinstance(args.memory, str) or not args.memory:
            parser.error("memory in the local configuration must be a QEMU memory string")
    command = args.command
    workflow = getattr(args, "target", None) if command in ("doctor", "plan", "fetch") else command
    args.mode = args.mode or ("release" if workflow == "image" else "dev")
    args.format = getattr(args, "format", None) or "raw"
    args.bios = getattr(args, "bios", False)
    if args.bios and args.arch != "x86_64":
        parser.error("--bios requires the pc-x86_64 machine")
    if command == "fetch" and args.offline:
        parser.error("fetch requires network access; use --offline with image or run")
    if command in ("build", "check") and args.target == "driver":
        drivers = MACHINES[args.machine]["drivers"]
        if args.name not in drivers:
            parser.error(f"{args.name} is not a driver for {args.machine}; choose from {', '.join(drivers)}")
    return args


@contextlib.contextmanager
def build_lock():
    BUILD.mkdir(exist_ok=True)
    with (BUILD / ".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            msg("Waiting for the active Roanix build")
            fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def doctor(args):
    programs = {"bash", "cargo", "git", "rustup", "zstd"}
    if args.arch == "x86_64":
        programs.update(("make", "cc"))
    if args.target == "run" or args.format == "raw":
        programs.update(("sgdisk", "mformat", "mcopy", "mke2fs"))
    if args.target == "image" and args.format == "iso":
        programs.add("xorriso")
    if args.target == "run":
        programs.add(f"qemu-system-{args.arch}")
    source_tools = {"awk", "find", "grep", "gzip", "make", "sed", "tar", "unshare", "wget"}
    if args.packages == "source" or args.offline:
        programs.update(source_tools if args.packages == "source" else {"unshare"})
    missing = sorted(program for program in programs if not shutil.which(program))
    if missing:
        raise RuntimeError(f"missing host tools: {', '.join(missing)}")
    missing_source = sorted(program for program in source_tools if not shutil.which(program))
    if missing_source:
        msg(f"Building missing packages from source also needs {', '.join(missing_source)}", detail=True)
    elif run("unshare", "--user", "--map-current-user", "true", check=False, quiet=True).returncode:
        if args.packages == "source" or args.offline:
            raise RuntimeError("unprivileged user namespaces are unavailable; Jinx source builds need them")
        msg("Source package builds need unprivileged user namespaces", detail=True)
    if args.target == "run":
        acceleration = "KVM" if args.arch == os.uname().machine and os.access("/dev/kvm", os.R_OK | os.W_OK) else "software emulation"
        msg(f"QEMU acceleration: {acceleration}")
    msg("Required host tools are available")


def output_path(build):
    args = build.args
    return DIST / f"roanix-{args.machine}-{args.profile}-{args.mode}.{args.format}"


def plan(build):
    args = build.args
    image = output_path(build) if args.target == "image" else build.out / "boot.hdd"
    msg(f"Output: {image.relative_to(ROOT)}")
    print("  Cargo checks the kernel and selected drivers for changes.")
    print("  Drivers: " + ", ".join(build.drivers))
    packages = build.jinx.resolve_packages(build.packages, runtime=True)
    missing = []
    for name in packages:
        path = build.jinx.package_file(name)
        legacy = BUILD / args.arch / "jinx" / "pkgs" / path.name
        if not (path.is_file() or legacy.is_file()):
            missing.append(name)
    print(f"  Runtime packages present: {len(packages) - len(missing)}/{len(packages)}")
    if missing:
        print("  Missing locally: " + ", ".join(missing))
    inputs = build.jinx.package_inputs(build.packages)
    rebuilds = [
        name for name in inputs
        if build.jinx.needs_rebuild(name, inputs[name])
    ]
    if rebuilds:
        print("  Local package rebuilds: " + ", ".join(rebuilds))
    if missing:
        print("  Missing archives will be fetched or built according to the package policy.")
    if image.is_file():
        print("  A previous image exists. Assembly is reused when its inputs still match.")
    else:
        print("  Media assembly is required because no image has been built.")
    if args.target == "run":
        print(f"  QEMU: qemu-system-{args.arch}, split boot and root disks")


def fetch(build):
    build.cargo("fetch")
    build.cargo("fetch", drivers=True)
    build.limine()
    if build.args.target == "run":
        resource("ovmf")
    build.jinx.ensure_packages((*build.packages, *BOOTSTRAP_PACKAGES))
    msg("Inputs are ready. Repeat the workflow with --offline.")


def machine_storage(name, machine):
    arch = machine["arch"]
    target = machine["rust-target"]
    cargo = BUILD / "cargo"
    return {
        "Cargo targets": (cargo / target, cargo / "drivers" / target),
        "Build work": (WORK / arch, CACHE / "jinx-project" / "targets" / arch),
        "Editable sources": (EDIT / arch,),
        "Cached packages": (CACHE / "packages" / arch,),
        "Portable images": tuple(sorted(DIST.glob(f"*roanix-{name}-*"))),
        "Legacy build": (BUILD / arch,),
        "Legacy Cargo target": (ROOT / "kernel" / "target" / target,),
    }


def known_builds():
    return [
        name for name, machine in MACHINES.items()
        if any(path.exists() for paths in machine_storage(name, machine).values() for path in paths)
    ]


def report_sizes(groups):
    paths = {
        path: label
        for label, entries in groups.items()
        for path in entries
        if path.exists()
    }
    if not paths:
        return
    sizes = dict.fromkeys(groups, 0)
    for line in run("du", "-sk", "--", *paths, capture=True).stdout.splitlines():
        size, path = line.split("\t", 1)
        sizes[paths[Path(path)]] += int(size)
    for label, size in sizes.items():
        if label in paths.values():
            print(f"  {size / 1024:9.1f} MiB  {label}")


def children_except(directory, excluded):
    return [path for path in directory.iterdir() if path.name not in excluded] if directory.is_dir() else []


def disk_usage():
    for name in known_builds():
        machine = MACHINES[name]
        msg(f"{name} ({machine['arch']})")
        storage = machine_storage(name, machine)
        report_sizes(storage)
        for image in storage["Portable images"]:
            if image.suffix in (".raw", ".iso"):
                print(f"               {image.name}")

    cargo = BUILD / "cargo"
    targets = {machine["rust-target"] for machine in MACHINES.values()}
    arches = {machine["arch"] for machine in MACHINES.values()}
    msg("Shared and legacy storage")
    report_sizes({
        "Tool downloads and shared cache": [
            *children_except(CACHE, {"packages", "jinx-project"}),
            *children_except(CACHE / "jinx-project", {"targets"}),
        ],
        "Shared Cargo files": [
            *children_except(cargo, targets | {"drivers"}),
            *children_except(cargo / "drivers", targets),
        ],
        "Shared work and settings": [*children_except(WORK, arches), BUILD / "local"],
        "Legacy host Cargo files": children_except(ROOT / "kernel/target", targets),
        "Legacy Jinx source and cache": [
            USERLAND / name for name in (".jinx-cache", "sources", "host-sources")
        ],
    })


def clean_paths(name):
    machine = MACHINES[name]
    storage = machine_storage(name, machine)
    work, sources = storage["Build work"]
    jinx = work / "jinx"
    return [
        *children_except(work, {"jinx"}),
        *children_except(jinx, {"pkgs", "host-pkgs", ".jinx-parameters"}),
        *storage["Cargo targets"],
        sources,
    ]


def clean(args):
    builds = known_builds()
    if builds:
        msg("Local builds: " + ", ".join(builds))
    machines = MACHINES if args.all else (args.machine,)
    for name in machines:
        paths = [path for path in clean_paths(name) if path.exists()]
        if not paths:
            msg(f"No rebuildable work for {name}")
            continue
        if args.dry_run:
            msg(f"Would clean {name}")
            for path in sorted(paths):
                print(f"  {path.relative_to(ROOT)}")
        else:
            for path in paths:
                remove(path)
            msg(f"Cleaned rebuildable work for {name}")


def prune_cache(args):
    live_archives = {
        f"{name}-{version}-{checksum[:12]}.tar.gz"
        for name, (version, _, checksum, _) in DOWNLOADS.items()
    }
    downloads = CACHE / "downloads"
    stale = [
        path for path in downloads.glob("*.tar.gz")
        if path.name not in live_archives and not path.name.startswith("xbps-")
    ]
    stale.extend(downloads.glob("*.part"))
    msg(f"Shared downloads: {len(stale)} obsolete")
    cache = CACHE / "packages"
    for name in known_builds():
        arch = MACHINES[name]["arch"]
        directory = cache / arch
        obsolete = []
        if directory.is_dir():
            jinx = Jinx(argparse.Namespace(arch=arch), WORK / arch)
            expected = [jinx.package_file(package) for package in jinx.recipe_index()]
            current = {directory / path.parent.name / path.name for path in expected}
            obsolete = [path for path in directory.glob("*/*.xbps") if path not in current]
        msg(f"{name}: {len(obsolete)} obsolete cached package(s)")
        stale.extend(obsolete)

    if not stale:
        return
    size = sum(path.stat().st_size for path in stale)
    for path in sorted(stale):
        print(path.relative_to(ROOT))
        if not args.dry_run:
            path.unlink()
    msg(f"{'Reclaimable' if args.dry_run else 'Reclaimed'} space: {size / (1024 * 1024):.1f} MiB")


def smoke_test(command, log, timeout):
    log.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + timeout
    output = b""
    started_init = False
    sent_check = False
    with log.open("wb") as stream, subprocess.Popen(
        list(map(str, command)), cwd=ROOT, stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    ) as guest, selectors.DefaultSelector() as selector:
        try:
            selector.register(guest.stdout, selectors.EVENT_READ)
            while time.monotonic() < deadline:
                if not selector.select(max(0, deadline - time.monotonic())):
                    break
                chunk = os.read(guest.stdout.fileno(), 8192)
                if not chunk:
                    break
                stream.write(chunk)
                stream.flush()
                output = (output + chunk)[-65536:]
                started_init |= b"started /sbin/init as pid 1" in output
                if started_init and not sent_check and output.endswith(b"# "):
                    guest.stdin.write(b"printf 'ROANIX_SMOKE_%s\\n' OK\r")
                    guest.stdin.flush()
                    sent_check = True
                if started_init and b"ROANIX_SMOKE_OK" in output:
                    msg(f"Boot passed. Serial log: {log.relative_to(ROOT)}")
                    return
        finally:
            guest.terminate()
            try:
                guest.wait(timeout=5)
            except subprocess.TimeoutExpired:
                guest.kill()
                guest.wait()
    stage = "reach the login shell" if started_init else "start /sbin/init"
    raise RuntimeError(f"guest did not {stage}; see {log.relative_to(ROOT)}")


def prepare_vm(build):
    args = build.args
    image = args.image or build.image("hdd")
    extra = args.extra
    if getattr(args, "debug", False):
        if args.image is None:
            msg(f"Symbols: {build.kernel}")
        msg("GDB: target remote 127.0.0.1:1234")
        extra = ["-gdb", "tcp:127.0.0.1:1234", "-S", *extra]
    if args.command == "test":
        extra = ["-display", "none", "-no-reboot"]
    return build.qemu_command(image, extra, split=args.image is None)


def dispatch(build):
    args = build.args
    command = args.command
    if command == "fetch":
        fetch(build)
    elif command == "image":
        image = build.image(args.format)
        msg(f"Ready: {image.relative_to(ROOT)}")
        msg(f"Machine: {args.machine}; checksum: {image.name}.sha256")
    elif command == "build":
        if args.target == "kernel":
            outputs = [build.build_kernel()]
        elif args.target == "package":
            recipe_path(args.name)
            outputs = build.jinx.ensure_packages((args.name,), force=args.name)
        else:
            result = build.build_drivers(args.name if args.target == "driver" else None)
            outputs = [result] if isinstance(result, Path) else result
        for path in outputs:
            msg(f"Ready: {path.relative_to(ROOT)}")
    elif command == "check":
        build.cargo("check", drivers=args.target != "kernel", driver=getattr(args, "name", None))
    elif command == "package":
        operation = build.jinx.edit_package if args.target == "edit" else build.jinx.export_package
        path = operation(args.name)
        msg(str(path.relative_to(ROOT)))
    elif command == "clean":
        clean(args)
    elif command == "cache":
        prune_cache(args)


def main(argv=None):
    try:
        args = parse_args(list(sys.argv[1:] if argv is None else argv))
        if args is None:
            return 0
        if sys.platform != "linux" or sys.version_info < (3, 12):
            raise RuntimeError("Roanix tools require Linux and Python 3.12 or newer")
        os.environ["PATH"] = os.environ.get("PATH", "") + ":/usr/sbin:/sbin"
        common.VERBOSE = args.verbose
        build = Build(args)
        if args.command not in ("disk-usage", "clean", "cache"):
            msg(f"{args.machine} · {args.profile} · {args.mode}")
        if args.command == "doctor":
            doctor(args)
        elif args.command == "plan":
            plan(build)
        elif args.command == "disk-usage":
            disk_usage()
        elif args.command == "check":
            dispatch(build)
        elif args.command in ("run", "test"):
            with build_lock():
                command = prepare_vm(build)
            if args.command == "test":
                stem = args.image.name if args.image else "boot"
                firmware = "-bios" if args.bios else ""
                smoke_test(command, build.base / "logs" / f"{stem}{firmware}.log", args.timeout)
            else:
                run(*command)
        else:
            with build_lock():
                dispatch(build)
    except KeyboardInterrupt:
        msg("Interrupted", error=True)
        return 130
    except (OSError, RuntimeError, ValueError, tarfile.TarError) as error:
        msg(str(error), error=True)
        return 1
    return 0
