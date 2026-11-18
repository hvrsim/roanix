"""Recipe inputs and package worktrees for the Jinx backend."""

import hashlib
import json
import os
import platform
import plistlib
import re
import shutil
import subprocess
import tarfile
import tempfile
from dataclasses import dataclass
from pathlib import Path

from . import roanix_common as common
from .roanix_common import (
    BUILD, CACHE, EDIT, ROOT, USERLAND, atomic_text, copy, download_to,
    fetch_archive, msg, remove, run, sha256, stamp,
)


JINX_COMMIT = "287ceaf9a2c08b43dbc56d38d8b815fd990a2192"


@dataclass(frozen=True)
class Recipe:
    version: str
    revision: str
    runtime: tuple[str, ...]
    build: tuple[str, ...]
    source: str
    local: str
    remote: str
    bootstrap: bool


@dataclass(frozen=True)
class PackageInputs:
    fingerprint: str
    dirty: bool
    edit: str


# Let the shell evaluate architecture-specific settings in the recipes.
READ_RECIPES = r"""
base_dir=$1
JINX_ARCH=$2
shift 2
. "$base_dir/Jinxfile"
for file; do (
    recipe_dir=${file%/*}
    name=${recipe_dir##*/}
    namespace=${recipe_dir%/*}
    prefix=
    if [[ ${namespace##*/} == host-recipes ]]; then prefix=host:; fi
    . "$file"
    source=
    if [[ -n $from_source ]]; then source=$from_source; fi
    if [[ -n $from_host_source ]]; then source=host:$from_host_source; fi
    hosts=
    for host in $hostdeps $hostrundeps $source_hostdeps; do hosts="$hosts host:$host"; done
    runtime=$deps
    if [[ -n $prefix ]]; then runtime=$hosts; fi
    printf '%s\0' "$prefix$name" "$version" "$revision" "$runtime" \
        "$deps $builddeps $source_deps $hosts" "$source" "$source_dir" \
        "$git_url$tarball_url" "$bootstrap_pkg"
); done
"""


def read_recipes(run, directory, arch):
    files = sorted(directory.glob("*recipes/*/recipe"))
    result = run("bash", "-e", "-c", READ_RECIPES, "recipes", directory, arch, *files, capture=True)
    fields = iter(result.stdout.removesuffix("\0").split("\0"))
    recipes = {}
    for name, version, revision, runtime, build, source, local, remote, bootstrap in zip(
        fields, fields, fields, fields, fields, fields, fields, fields, fields, strict=True
    ):
        recipes[name] = Recipe(
            version, revision, tuple(runtime.split()), tuple(build.split()),
            source, local, remote, bootstrap == "yes",
        )
    return recipes


def tree_digest(paths):
    digest = hashlib.sha256()
    for root in paths:
        paths_in_root = sorted(root.rglob("*")) if root.is_dir() else [root]
        for path in paths_in_root:
            if ".git" in path.parts:
                continue
            if path.is_symlink():
                data = os.fsencode(path.readlink())
            elif path.is_file():
                with path.open("rb") as stream:
                    data = hashlib.file_digest(stream, "sha256").digest()
            else:
                continue
            digest.update(os.fsencode(path.relative_to(root.parent)) + b"\0")
            digest.update(str(path.lstat().st_mode & 0o170111).encode() + b"\0")
            digest.update(data + b"\0")
    return digest.hexdigest()


def sync_tree(source, target, copy, remove):
    """Synchronize a small configuration tree without touching unchanged files."""
    target.mkdir(parents=True, exist_ok=True)
    for item in target.iterdir():
        if not (source / item.name).exists():
            remove(item)
    for item in source.iterdir():
        destination = target / item.name
        if item.is_dir():
            sync_tree(item, destination, copy, remove)
        else:
            copy(item, destination)


def export_patch(run, worktree, baseline, destination):
    """Jinx applies paths worktree-name/path with patch -p1."""
    result = run(
        "git", "diff", "--no-index", "--no-prefix", "--binary", baseline.name, worktree.name,
        cwd=worktree.parent, capture=True, check=False,
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(f"could not compare {worktree} against its baseline")
    if "\nGIT binary patch\n" in result.stdout:
        raise ValueError("Jinx patches support text changes only")
    if not result.stdout:
        destination.unlink(missing_ok=True)
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        patch = Path(temporary) / destination.name
        patch.write_text(result.stdout)
        patch.replace(destination)


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


class Jinx:
    """A machine-specific package repository backed by a shared Jinx environment."""

    def __init__(self, args, base):
        self.args = args
        self.arch = args.arch
        self.work = base / "jinx"
        self.logs = base / "logs"
        self.project = CACHE / "jinx-project"
        self.environment = self.project / ".jinx-cache"
        self.ready = False
        self.recipes = None
        self.repository_indexes = {}
        self.binary = None
        self.in_checkout = None

    def prepare_jinx_source(self):
        self.project.mkdir(parents=True, exist_ok=True)
        # Jinx addresses its environment as /base_dir/.jinx-cache inside its
        # build container. Keep one bootstrap and select the machine's sources.
        if not self.environment.exists():
            migrated = CACHE / "jinx-env"
            legacy = USERLAND / ".jinx-cache"
            if (migrated / "version").is_file():
                migrated.rename(self.environment)
            elif (legacy / "version").is_file():
                run("cp", "-a", "--reflink=auto", legacy, self.environment)
        for kind in ("sources", "host-sources"):
            link = self.project / kind
            # Inside the chroot Jinx mounts these same directories at /base_dir.
            # Use a directory inside the generated project for the bind target.
            local_source = self.project / "targets" / self.arch / kind
            local_source.mkdir(parents=True, exist_ok=True)
            relative = local_source.relative_to(self.project)
            if not link.is_symlink() or link.readlink() != relative:
                link.unlink(missing_ok=True)
                link.symlink_to(relative, target_is_directory=True)
        copy(USERLAND / "Jinxfile", self.project / "Jinxfile")
        for directory in ("recipes", "host-recipes"):
            sync_tree(USERLAND / directory, self.project / directory, copy, remove)
        self.prepare_cross_files()
        for recipe in self.recipe_index().values():
            if recipe.local and (USERLAND / recipe.local).is_dir():
                sync_tree(USERLAND / recipe.local, self.project / recipe.local, copy, remove)

    def prepare_cross_files(self):
        source = USERLAND / "build-support"
        target = self.project / "build-support"
        target.mkdir(exist_ok=True)
        for name in ("config.guess", "config.sub"):
            copy(source / name, target / name)
        for template, filename in (
            ("meson.cross.in", f"cross_file-{self.arch}.txt"),
            ("cmake.toolchain.in", f"CMakeToolchain-{self.arch}.txt"),
        ):
            rendered = (source / template).read_text().replace("@ARCH@", self.arch)
            path = target / filename
            if not path.is_file() or path.read_text() != rendered:
                atomic_text(path, rendered)

    def prepare_repository(self):
        legacy = BUILD / self.arch / "jinx"
        for directory in ("pkgs", "host-pkgs"):
            target = self.work / directory
            if not target.exists() and (legacy / directory).is_dir():
                target.parent.mkdir(parents=True, exist_ok=True)
                run("cp", "-a", "--reflink=auto", legacy / directory, target)

    def recipe_index(self):
        if self.recipes is None:
            self.recipes = read_recipes(run, USERLAND, self.arch)
        return self.recipes

    def recipe_owner(self, name):
        recipes = self.recipe_index()
        recipe_path(name)
        seen = set()
        while source := recipes[name].source:
            if name in seen:
                raise ValueError(f"source recipe cycle involving {name}")
            seen.add(name)
            name = source
        return name

    def resolve_packages(self, names, *, runtime=False):
        recipes = self.recipe_index()
        resolved = set()
        ordered = []

        def visit(name):
            if name in resolved:
                return
            resolved.add(name)
            if name not in recipes:
                raise ValueError(f"no recipe for {name}")
            recipe = recipes[name]
            for dependency in (recipe.runtime if runtime else recipe.build):
                visit(dependency)
            if not runtime or not recipe.bootstrap:
                ordered.append(name)

        for name in names:
            visit(name)
        return ordered

    def package_file(self, name):
        recipes = self.recipe_index()
        recipe = recipes[name]
        version = recipes[self.recipe_owner(name)].version
        host = name.startswith("host:")
        arch = platform.machine() if host else self.arch
        folder = "host-pkgs" if host else "pkgs"
        return self.work / folder / f"{name.removeprefix('host:')}-{version}_{recipe.revision}.{arch}.xbps"

    def package_files(self, names):
        files = [self.package_file(name) for name in names]
        return sorted(path for path in files if path.is_file())

    def edit_paths(self, name):
        folder = EDIT / self.arch / ("host" if name.startswith("host:") else "target")
        basename = name.removeprefix("host:")
        return folder / f"{basename}-workdir", folder / f"{basename}-clean"

    def source_paths(self, name):
        owner = self.recipe_owner(name)
        basename = owner.removeprefix("host:")
        kind = "host-sources" if owner.startswith("host:") else "sources"
        folder = self.project / "targets" / self.arch / kind
        return folder, basename

    def recipe_inputs(self, name):
        owner = self.recipe_owner(name)
        paths = [recipe_path(name).parent]
        if owner != name:
            paths.append(recipe_path(owner).parent)
        if local := self.recipe_index()[owner].local:
            paths.append(USERLAND / local)
        return paths

    def checkout_changes(self, paths):
        if self.in_checkout is None:
            checkout = run("git", "rev-parse", "--show-toplevel", cwd=ROOT, check=False, quiet=True)
            self.in_checkout = checkout.returncode == 0 and Path(checkout.stdout.strip()) == ROOT
        if not self.in_checkout:
            return set()
        args = ("--", *paths)
        tracked = run("git", "diff", "HEAD", "--name-only", "-z", *args, capture=True, cwd=ROOT)
        untracked = run("git", "ls-files", "--others", "--exclude-standard", "-z", *args, capture=True, cwd=ROOT)
        return {ROOT / path for path in (tracked.stdout + untracked.stdout).split("\0") if path}

    def package_inputs(self, names):
        support = USERLAND / "build-support"
        templates = {
            support / "meson.cross.in": f"cross_file-{self.arch}.txt",
            support / "cmake.toolchain.in": f"CMakeToolchain-{self.arch}.txt",
        }
        config = [USERLAND / "Jinxfile", support / "config.guess", support / "config.sub", *templates]
        changed_config = self.checkout_changes(config)
        for template in changed_config.intersection(templates):
            # Accept the equivalent cross file in older checkouts during migration.
            committed = run(
                "git", "show", f"HEAD:userland/build-support/{templates[template]}",
                quiet=True, check=False, cwd=ROOT,
            )
            if not committed.returncode and template.read_text().replace("@ARCH@", self.arch) == committed.stdout:
                changed_config.remove(template)

        ordered = self.resolve_packages(names)
        recipe_paths = {name: self.recipe_inputs(name) for name in ordered}
        changed_sources = self.checkout_changes(sorted({path for paths in recipe_paths.values() for path in paths}))
        config_key = tree_digest(config)
        edits = {owner: self.worktree_change(owner) for owner in {self.recipe_owner(name) for name in ordered}}
        fingerprints = {}
        dirty = set()
        for name, paths in recipe_paths.items():
            edit = edits[self.recipe_owner(name)]
            fingerprints[name] = (tree_digest(paths), edit)
            if edit or any(path in change.parents or path == change for path in paths for change in changed_sources):
                dirty.add(name)

        inputs = {}
        for name in ordered:
            closure = sorted(self.resolve_packages((name,)))
            values = [config_key, JINX_COMMIT, [(dep, fingerprints[dep]) for dep in closure]]
            fingerprint = hashlib.sha256(json.dumps(values).encode()).hexdigest()
            inputs[name] = PackageInputs(
                fingerprint, bool(changed_config or dirty.intersection(closure)), fingerprints[name][1],
            )
        return inputs

    @staticmethod
    def archive_identity(archive):
        stat = archive.stat()
        return [stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns]

    def cached_inputs(self, archive):
        marker = stamp(archive)
        if marker.is_file() and archive.is_file():
            try:
                record = json.loads(marker.read_text())
            except ValueError:
                return None
            if isinstance(record, dict) and record.get("archive") == self.archive_identity(archive):
                return record
        return None

    def record_inputs(self, archive, fingerprint, *, local):
        atomic_text(stamp(archive), json.dumps({
            "inputs": fingerprint, "archive": self.archive_identity(archive), "local": local,
        }))

    def needs_rebuild(self, name, inputs, *, explicit=False, need_sources=False):
        archive = self.package_file(name)
        record = self.cached_inputs(archive)
        if record:
            return (record.get("inputs") != inputs.fingerprint
                    or (explicit and (need_sources or not record.get("local"))))
        return explicit or inputs.dirty or (archive.is_file() and stamp(archive).is_file())

    def worktree_change(self, owner):
        worktree, _ = self.edit_paths(owner)
        if not worktree.is_dir():
            return ""
        current = tree_digest([worktree])
        initial = worktree.with_suffix(".initial")
        return "" if initial.is_file() and initial.read_text() == current else current

    def ensure_packages(self, names, *, force=None, need_sources=False):
        self.prepare_repository()
        recipes = self.recipe_index()
        inputs = self.package_inputs(names)
        for name, package in inputs.items():
            owner = self.recipe_owner(name)
            archive = self.package_file(name)
            if not self.needs_rebuild(name, package, explicit=name == force, need_sources=need_sources):
                if archive.is_file() and not self.cached_inputs(archive):
                    self.record_inputs(archive, package.fingerprint, local=False)
                continue
            worktree, _ = self.edit_paths(owner)
            if recipes[name].build:
                self.download(*recipes[name].build)
                pending = self.pending(recipes[name].build)
                if pending:
                    if self.args.offline:
                        raise RuntimeError(f"offline build dependencies missing for {name}: {pending}")
                    self.command("update", "-b", *recipes[name].build)
            self.command("dry-run", name, capture=True)
            owner_key = tree_digest([recipe_path(owner).parent])
            self.prepare_source_override(owner, owner_key, worktree if package.edit else None)
            msg(f"Building package {name}")
            self.rebuild(name, archive)
            self.record_inputs(archive, package.fingerprint, local=True)

        self.download(*names, runtime=True)
        runtime = self.resolve_packages(names, runtime=True)
        missing = [name for name in runtime if not self.package_file(name).is_file()]
        built = set()
        if missing:
            if self.args.offline:
                raise RuntimeError(f"offline packages missing: {', '.join(missing)}; run ./x.py fetch first")
            self.download(*missing)
            pending = self.pending(missing)
            if pending:
                msg(f"Building missing packages ({pending})")
                built = {name for name in inputs if not self.package_file(name).is_file()}
                self.command("update", "-b", *missing)
        for name, package in inputs.items():
            archive = self.package_file(name)
            if archive.is_file() and not self.cached_inputs(archive):
                self.record_inputs(archive, package.fingerprint, local=name in built)
        return self.package_files(names)

    def install(self, root, packages):
        installable = [name for name in packages if not self.recipe_index()[name].bootstrap]
        archives = [self.package_file(name) for name in self.resolve_packages(installable, runtime=True)]
        # The source-build repo may contain other versions. Present only the
        # selected archives so XBPS resolves against the exact build inputs.
        with tempfile.TemporaryDirectory(prefix="image-repo-", dir=self.work) as directory:
            repo = Path(directory)
            staged = []
            for archive in archives:
                path = repo / archive.name
                path.symlink_to(archive)
                staged.append(path)
            self.xbps("rindex", "-a", *staged, arch=self.arch, quiet=True)
            # Keep Jinx's package-by-package installation order. The layout
            # package introduces merged-/usr aliases after their dependencies.
            for name in installable:
                self.xbps(
                    "install", "-y", "-i", "-r", root, "-R", repo,
                    name, arch=self.arch, quiet=True,
                )

    def xbps(self, command, *args, arch, quiet=False):
        bundle = self.xbps_bundle()
        run(
            bundle / "bin" / f"xbps-{command}", *args, quiet=quiet,
            env={"XBPS_ARCH": "invalid", "XBPS_TARGET_ARCH": arch,
                 "LD_LIBRARY_PATH": str(bundle / "lib")},
        )

    def xbps_bundle(self):
        # Source builds already bootstrap this same pinned bundle through Jinx.
        legacy = USERLAND / ".jinx-cache/xbps"
        installed = self.environment / "xbps"
        for bundle in (installed, legacy):
            if (bundle / "bin/xbps-install").is_file():
                return bundle

        pin = CACHE / "xbps"
        if (pin / "bin/xbps-install").is_file():
            return pin
        script = self.executable().read_text()
        arch = platform.machine()
        url = re.search(r'^XBPS_RELEASE_URL_BASE="([^"]+)"', script, re.MULTILINE)[1]
        checksum = re.search(rf"^    \[{re.escape(arch)}\]=([0-9a-f]{{64}})$", script, re.MULTILINE)
        if checksum is None:
            raise RuntimeError(f"Jinx does not publish XBPS tools for Linux {arch}")
        name = f"xbps-static-bin-linux-{arch}"
        version = url.rsplit("/", 1)[-1]
        return fetch_archive(
            "xbps", version, f"{url}/{name}.tar.gz", checksum[1], name, pin,
            offline=self.args.offline,
        )

    def rebuild(self, name, archive):
        with tempfile.TemporaryDirectory(dir=self.work) as directory:
            backup = Path(directory) / archive.name
            if archive.is_file():
                copy(archive, backup)
            try:
                self.command("rebuild", name)
                if not archive.is_file():
                    raise RuntimeError(f"Jinx did not produce {archive.name}")
            except (OSError, RuntimeError, KeyboardInterrupt):
                if backup.is_file():
                    backup.replace(archive)
                    arch = platform.machine() if name.startswith("host:") else self.arch
                    self.index_archive(archive, arch)
                else:
                    archive.unlink(missing_ok=True)
                raise

    def prepare_source_override(self, owner, key, worktree):
        recipe = self.recipe_index()[owner]
        if recipe.local or not recipe.remote:
            return
        directory, name = self.source_paths(owner)
        marker = directory / f"{name}.roanix-source"
        if worktree is not None:
            source = directory / name
            remove(source)
            shutil.copytree(worktree, source, symlinks=True)
            for state in ("patched", "version"):
                (directory / f"{name}.{state}").write_text(recipe.version)
            (directory / f"{name}.prepared").unlink(missing_ok=True)
            marker.unlink(missing_ok=True)
        else:
            if not marker.is_file() or marker.read_text() != key:
                for state in ("version", "patched", "prepared"):
                    (directory / f"{name}.{state}").unlink(missing_ok=True)
                marker.write_text(key)

    def edit_package(self, name):
        owner = self.recipe_owner(name)
        recipe = self.recipe_index()[owner]
        if recipe.local:
            return USERLAND / recipe.local
        if not recipe.remote:
            raise ValueError(f"{name} has no editable source")
        worktree, baseline = self.edit_paths(owner)
        if worktree.is_dir():
            return worktree
        directory, basename = self.source_paths(owner)
        source = directory / f"{basename}-workdir"
        if not source.is_dir():
            self.ensure_packages((name,), force=name, need_sources=True)
        worktree.parent.mkdir(parents=True, exist_ok=True)
        for src, dest in ((directory / f"{basename}-clean", baseline), (source, worktree)):
            with tempfile.TemporaryDirectory(dir=worktree.parent) as temporary:
                candidate = Path(temporary) / dest.name
                shutil.copytree(src, candidate, symlinks=True, ignore=shutil.ignore_patterns(".git"))
                remove(dest)
                candidate.rename(dest)
        atomic_text(worktree.with_suffix(".initial"), tree_digest([worktree]))
        return worktree

    def export_package(self, name):
        owner = self.recipe_owner(name)
        worktree, baseline = self.edit_paths(owner)
        if not worktree.is_dir() or not baseline.is_dir():
            raise ValueError(f"no editable worktree for {name}; run ./x.py package edit {name}")
        destination = recipe_path(owner).parent / "patches" / "jinx-working-patch.patch"
        export_patch(run, worktree, baseline, destination)
        return destination

    def pending(self, names):
        output = self.command("dry-run", *names, capture=True).stdout.splitlines()
        return output[-1].strip() if output else ""

    def command(self, *args, **kwargs):
        """Initialize the pinned Jinx checkout once, then run a Jinx command."""
        binary = self.executable()
        env = {
            "JINX_PARALLELISM": str(self.args.jobs),
            "JINX_CLEAN_WORKDIRS": "no",
            "JINX_CACHE_DIR": str(self.environment),
        }
        command = ("unshare", "--user", "--map-current-user", "--net", binary) if self.args.offline else (binary,)

        if not self.ready:
            self.prepare_jinx_source()
            if not self.args.offline and not (self.environment / "version").is_file():
                msg("Preparing the Jinx source-build environment")
            if self.args.offline:
                self.check_jinx_cache(binary)
            self.work.mkdir(parents=True, exist_ok=True)
            parameters = self.work / ".jinx-parameters"
            if parameters.is_file() and str(self.project) not in parameters.read_text():
                parameters.unlink()
            if not parameters.is_file():
                run(*command, "init", self.project, f"ARCH={self.arch}", cwd=self.work, env=env)
            self.ready = True

        if common.VERBOSE or args[0] not in ("update", "rebuild"):
            return run(*command, *args, cwd=self.work, env=env, **kwargs)
        package = args[1] if args[0] == "rebuild" else "update"
        self.logs.mkdir(parents=True, exist_ok=True)
        path = self.logs / f"package-{package}.log"
        msg(f"Build log: {path.relative_to(ROOT)}", detail=True)
        with path.open("w") as log:
            result = subprocess.run(
                [str(arg) for arg in (*command, *args)],
                cwd=self.work, env=os.environ | env, stdout=log, stderr=subprocess.STDOUT,
                check=False,
            )
        if result.returncode:
            raise RuntimeError(f"Jinx failed ({result.returncode}); see {path.relative_to(ROOT)}")
        return result

    def executable(self):
        if self.binary is not None:
            return self.binary
        checkout = CACHE / "jinx"
        binary = checkout / "jinx"
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
            if self.args.offline:
                raise RuntimeError("pinned Jinx is missing; run ./x.py fetch first")
            msg("Fetching Jinx")
            run(
                "git", "fetch", "--depth=1",
                "https://github.com/Mintsuki/Jinx.git", JINX_COMMIT,
                cwd=checkout,
            )
            run("git", "checkout", "-q", "--detach", JINX_COMMIT, cwd=checkout)

        self.binary = binary
        return binary

    def check_jinx_cache(self, binary):
        settings = (USERLAND / "Jinxfile").read_text()
        script = binary.read_text()
        versions = re.findall(r'^jinx_(?:major|minor)_ver="([^"]+)"', script, re.MULTILINE)
        expected = {
            "version": ".".join(versions),
            "debian_snapshot": re.search(r"^JINX_DEBIAN_SNAPSHOT=(.*)$", settings, re.MULTILINE)[1],
            "base_packages": re.search(r'^JINX_BASE_PACKAGES="([^"]+)"', settings, re.MULTILINE)[1],
        }
        for key, value in expected.items():
            marker = self.environment / key
            if not marker.is_file() or (value and marker.read_text().strip() != value):
                raise RuntimeError(f"Jinx environment needs {key}; run ./x.py fetch first")

    def index_archive(self, path, arch):
        self.xbps("rindex", "-f", "-a", path, arch=arch)

    def repository_index(self, repository, arch):
        key = (repository, arch)
        if key in self.repository_indexes:
            return self.repository_indexes[key]
        try:
            with tempfile.TemporaryDirectory(dir=CACHE) as directory:
                archive = Path(directory) / "repodata"
                download_to(f"{repository}/{arch}-repodata", archive, timeout=20)
                with tempfile.TemporaryFile() as decoded:
                    subprocess.run(["zstd", "-dc", archive], stdout=decoded, check=True)
                    decoded.seek(0)
                    with tarfile.open(fileobj=decoded) as data:
                        index = plistlib.load(data.extractfile("index.plist"))
        except (OSError, KeyError, subprocess.CalledProcessError, tarfile.TarError, plistlib.InvalidFileException) as error:
            msg(f"Package index unavailable at {repository}: {error}", detail=True)
            index = {}
        self.repository_indexes[key] = index
        return index

    def download(self, *names, runtime=False):
        disabled = os.environ.get("ROANIX_BINARY_PACKAGES", "1").lower() in ("0", "no", "false")
        if self.args.offline or self.args.packages == "source" or disabled:
            return
        base = os.environ.get("ROANIX_PACKAGE_REPO", "https://packages.hvrsim.com/roanix/main").rstrip("/")
        for name in self.resolve_packages(names, runtime=runtime):
            archive = self.package_file(name)
            if archive.is_file():
                continue
            host = name.startswith("host:")
            arch = platform.machine() if host else self.arch
            repository = f"{base}/host/{self.arch}" if host else f"{base}/{self.arch}"
            key = name.removeprefix("host:")
            entry = self.repository_index(repository, arch).get(key, {})
            checksum = entry.get("filename-sha256", "")
            expected_version = archive.name.removesuffix(f".{arch}.xbps")
            if entry.get("pkgver") != expected_version or not re.fullmatch(r"[0-9a-f]{64}", checksum):
                continue

            archive.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(dir=archive.parent) as temporary:
                partial = Path(temporary) / archive.name
                msg(f"Downloading {archive.name}", detail=True)
                try:
                    download_to(f"{repository}/{archive.name}", partial, timeout=45)
                except OSError as error:
                    msg(f"Package unavailable: {archive.name} ({error})", detail=True)
                    continue
                if sha256(partial) != checksum:
                    raise RuntimeError(f"package checksum mismatch: {archive.name}")
                partial.replace(archive)
                try:
                    self.index_archive(archive, arch)
                except RuntimeError:
                    archive.unlink()
                    raise

