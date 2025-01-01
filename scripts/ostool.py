#!/usr/bin/env python3
# ===----------------------------------------------------------------------===##
#
# Part of the Roanix Project, under the Mozilla Public License 2.0.
# See LICENSE in the project root for license information.
# SPDX-License-Identifier: MPL-2.0
#
# ===----------------------------------------------------------------------===##

"""
Manages and runs an operating system build, generating configurations
and orchestrating tools as needed...
"""

import argparse
import os
import platform
import stat
import subprocess
import sys

from pathlib import Path
from string import Template
from urllib.request import urlretrieve

VERBOSE = False
JINX_VERSION = "0.4.7"

CFLAGS = {
    "debug": "-Og -ggdb",
    "release": "-O2",
    "minsize": "-Os",
}

LDFLAGS = "-Wl,-O1 -Wl,--sort-common -Wl,--as-needed -Wl,-z,relro -Wl,-z,now"

def printv(msg):
    if VERBOSE:
        print(msg)

def scall(cmd):
    return subprocess.call(cmd, shell=True)

def create_jinx_config(args):
    infile = open("scripts/jinx-config.in", "r")
    config = open("jinx-config", "w")
    t = Template(infile.read())

    cflags = CFLAGS[args.btype]
    ldflags = LDFLAGS

    if args.arch == 'x86_64':
        cflags += " -fcf-protection -fno-omit-frame-pointer -mno-omit-leaf-frame-pointer"
        ldflags += "  -Wl,-z,pack-relative-relocs"
    else:
        cflags += " -fno-omit-frame-pointer"

    if platform.uname().machine == args.arch:
        cflags += " -march=native"

    config.write(
        t.substitute(
            {
                "JINX_MAJOR_VER": JINX_VERSION[:3],
                "COMMON_ARCH": args.arch,
                "COMMON_TARGET": args.arch + "-roanix-mlibc",
                "COMMON_BUILDTYPE": args.btype,
                "COMMON_CFLAGS": cflags,
                "COMMON_LDFLAGS": ldflags,
            }
        )
    )

    infile.close()
    config.close()


def do_setup(args):
    jinx_script = Path("jinx")
    jinx_config = Path("jinx-config")

    jinx_link = (
        "https://raw.githubusercontent.com/mintsuki/jinx/v" + JINX_VERSION + "/jinx"
    )

    if not jinx_script.is_file():
        printv("setup: downloading jinx bash script")
        urlretrieve(jinx_link, "jinx")
        st = os.stat("jinx")
        os.chmod("jinx", st.st_mode | stat.S_IEXEC)

    if not jinx_config.is_file():
        printv("setup: generating jinx configuration...")
        create_jinx_config(args)

    return 0


def do_build(args):
    if args.package:
        printv("build: compiling package " + args.package)
        return scall("./jinx build" + args.package)
    else:
        printv("build: building world...")
        return scall("./jinx build world")


def do_clean(args):
    scall(
        "cd "
        + args.source_dir
        + " && rm -rf sources builds pkgs host-builds host-pkgs jinx-config roanix.hdd"
    )

    if args.deep:
        scall("cd " + args.source_dir + " && rm -rf .jinx-cache")
        printv("ostool: deep cleaned the working tree.")
    else:
        printv("ostool: cleaned the working tree.")

    return 0


def build_parser(parser):
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="increase output verbosity"
    )
    parser.add_argument(
        "-C",
        type=str,
        dest="source_dir",
        help="source dir (in place of cwd)",
        default=".",
    )
    subparsers = parser.add_subparsers(dest="command")

    parser_setup = subparsers.add_parser("setup", help="setup build configuration")
    parser_setup.add_argument(
        "--arch",
        type=str,
        dest="arch",
        choices=["riscv64", "x86_64"],
        default="x86_64",
        help="target architecture",
    )
    parser_setup.add_argument(
        "-b",
        type=str,
        dest="btype",
        choices=["debug", "release", "minsize"],
        default="release",
        help="compiler build type",
    )

    parser_build = subparsers.add_parser(
        "build", help="build the operating system (or specific package)"
    )
    parser_build.add_argument(
        "--pkg", type=str, dest="package", help="package to build"
    )

    parser_clean = subparsers.add_parser(
        "clean", help="clean repository of build artifacts"
    )
    parser_clean.add_argument(
        "-d",
        "--deep",
        action="store_true",
        help="perform a deep clean (remove jinx cache)",
    )


def main():
    if os.geteuid() == 0:
        sys.exit("error: don't run ostool as root!")

    parser = argparse.ArgumentParser()
    build_parser(parser)

    args = parser.parse_args()

    if args.verbose:
        global VERBOSE
        VERBOSE = True

    if args.source_dir:
        os.chdir(args.source_dir)

    if not args.command:
        sys.exit('error: no command passed to ostool!')

    if args.command == "setup":
        return do_setup(args)
    elif args.command == "build":
        return do_build(args)
    elif args.command == "clean":
        return do_clean(args)
    else:
        sys.exit('error: unknown command "' + args.command + '"')


if __name__ == "__main__":
    exit(main())
