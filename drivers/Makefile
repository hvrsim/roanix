# Driver module build.
#
# Every driver is a Rust workspace crate linked into a freestanding,
# self-contained shared object by build-module.py. Modules link against no
# library at all: the kernel API is reached through the service table handed to
# the module entry point, and a curated set of versioned `rdf_api_v1_*` symbols
# is resolved by the kernel loader at import time.
#
# Usage:
#   make ARCH=x86_64            build every x86_64 module into out/x86_64/
#   make ARCH=riscv64 install   build and install under DESTDIR/PREFIX

ARCH ?=
RUST_TOOLCHAIN ?= nightly-2026-07-15
RUST_PROFILE ?= release
DESTDIR ?=
PREFIX ?= /usr
.DEFAULT_GOAL := all

BUILD_DIR := build/$(ARCH)
OUTPUT_DIR := out/$(ARCH)
MODULE_DIR := $(DESTDIR)$(PREFIX)/lib/roanix/drivers
INCLUDE_DIR := $(DESTDIR)$(PREFIX)/include/roanix
PREBUILT_DIR := prebuilt/$(ARCH)
TARGET_DIR := $(abspath $(BUILD_DIR)/target)

# `make clean` works with or without ARCH; everything else needs one.
ifneq ($(MAKECMDGOALS),clean)
ifeq ($(ARCH),x86_64)
TARGET := x86_64-unknown-none
MODULES := acpi ioapic tmpfs devfs console pty uart8250 special
else ifeq ($(ARCH),riscv64)
TARGET := riscv64gc-unknown-none-elf
MODULES := fdt tmpfs devfs console pty uart8250 special
else
$(error Unsupported ARCH "$(ARCH)"; pass ARCH=x86_64 or ARCH=riscv64)
endif
endif

OUTPUT_MODULES := $(addprefix $(OUTPUT_DIR)/,$(addsuffix .ko,$(MODULES)))

# Rebuild a module whenever any workspace source, manifest, or build script
# changes. Build and output trees are excluded so artefacts never trigger a
# rebuild of themselves.
SOURCES := $(shell find . -type f \
	\( -name '*.rs' -o -name '*.py' -o -name 'Cargo.toml' -o -name 'Cargo.lock' \
	-o -name 'build.rs' -o -name 'rust-toolchain.toml' \) \
	-not -path './build/*' -not -path './out/*' -not -path './prebuilt/*')

.PHONY: all clean install FORCE

FORCE:

all: $(OUTPUT_MODULES)

# A staged tree may carry a prebuilt image (xtool builds modules on the host
# before packaging); prefer it so in-distribution builds need no toolchain.
$(OUTPUT_DIR)/%.ko: FORCE $(SOURCES)
	@mkdir -p $(@D)
	@if test -f "$(PREBUILT_DIR)/$*.ko"; then \
		install -m644 "$(PREBUILT_DIR)/$*.ko" "$@"; \
	else \
		python3 ./build-module.py \
			--package $* \
			--target "$(TARGET)" \
			--output "$@" \
			--target-dir "$(TARGET_DIR)" \
			--profile "$(RUST_PROFILE)" \
			--toolchain "$(RUST_TOOLCHAIN)" \
			--imports; \
	fi

install: all
	install -d "$(MODULE_DIR)"
	for module in $(OUTPUT_MODULES); do \
		install -m644 "$$module" "$(MODULE_DIR)/$${module##*/}"; \
	done
	install -d "$(INCLUDE_DIR)"
	install -m644 include/roanix/*.h "$(INCLUDE_DIR)"

clean:
	rm -rf build/x86_64 build/riscv64 out/x86_64 out/riscv64
	rmdir build out prebuilt 2>/dev/null || true
