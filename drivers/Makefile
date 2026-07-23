ARCH ?=
CC ?= cc
LD ?= ld
AR ?= ar
STRIP ?= strip
DESTDIR ?=
PREFIX ?= /usr

BUILD_DIR := build/$(ARCH)
OUTPUT_DIR := out/$(ARCH)
INSTALL_DIR := $(DESTDIR)$(PREFIX)/lib/roanix/drivers
DEVKIT_INSTALL_DIR := $(DESTDIR)$(PREFIX)/lib/roanix
DEVKIT_INCLUDE_DIR := $(DESTDIR)$(PREFIX)/include/devkit

COMMON_CFLAGS := \
	-O3 -std=c11 -ffreestanding -fPIC -fvisibility=hidden \
	-fno-builtin -fno-stack-protector -fno-unwind-tables \
	-fno-asynchronous-unwind-tables -ffunction-sections -fdata-sections \
	-Wall -Wextra -Werror -Idevkit/include

COMMON_LDFLAGS := \
	-shared -Bsymbolic --no-undefined --gc-sections --build-id=none \
	--hash-style=sysv -z now -z relro -z separate-code -z noexecstack \
	-z max-page-size=4096 -e roanix_driver_entry

ifeq ($(ARCH),x86_64)
ARCH_CFLAGS := \
	-march=x86-64 -mtune=generic -mcmodel=small -mno-red-zone \
	-mgeneral-regs-only -fcf-protection=none
MODULES := \
	$(OUTPUT_DIR)/acpi.so \
	$(OUTPUT_DIR)/ioapic.so \
	$(OUTPUT_DIR)/uart16550.so \
	$(OUTPUT_DIR)/special.so \
	$(OUTPUT_DIR)/pty.so
else ifeq ($(ARCH),riscv64)
ARCH_CFLAGS := \
	-march=rv64imac_zicsr_zifencei -mabi=lp64 -mcmodel=medany \
	-msmall-data-limit=0 -mno-save-restore
MODULES := \
	$(OUTPUT_DIR)/dtb.so \
	$(OUTPUT_DIR)/uart8250.so \
	$(OUTPUT_DIR)/special.so \
	$(OUTPUT_DIR)/pty.so
else
$(error Unsupported ARCH "$(ARCH)")
endif

CFLAGS := $(COMMON_CFLAGS) $(ARCH_CFLAGS) $(EXTRA_CFLAGS)
DEVKIT_OBJECTS := \
	$(BUILD_DIR)/devkit/src/devkit.o \
	$(BUILD_DIR)/devkit/src/runtime.o
DEVKIT_LIBRARY := $(BUILD_DIR)/devkit/libdevkit.a

.PHONY: all clean install

all: $(MODULES)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(DEVKIT_LIBRARY): $(DEVKIT_OBJECTS)
	@mkdir -p $(@D)
	$(AR) rcs $@ $^

define link_module
$(1): $(2) $(DEVKIT_LIBRARY)
	@mkdir -p $$(@D)
	$(LD) $(COMMON_LDFLAGS) -soname $$(@F) -o $$@ $$^
	$(STRIP) --strip-debug $$@
endef

$(eval $(call link_module,$(OUTPUT_DIR)/acpi.so,$(BUILD_DIR)/bus/acpi.o))
$(eval $(call link_module,$(OUTPUT_DIR)/dtb.so,$(BUILD_DIR)/bus/dtb.o))
$(eval $(call link_module,$(OUTPUT_DIR)/ioapic.so,$(BUILD_DIR)/irq/ioapic.o))
$(eval $(call link_module,$(OUTPUT_DIR)/uart16550.so,$(BUILD_DIR)/char/uart16550.o))
$(eval $(call link_module,$(OUTPUT_DIR)/uart8250.so,$(BUILD_DIR)/char/uart8250.o))
$(eval $(call link_module,$(OUTPUT_DIR)/special.so,$(BUILD_DIR)/char/special.o))
$(eval $(call link_module,$(OUTPUT_DIR)/pty.so,$(BUILD_DIR)/char/pty.o))

install: all
	install -d "$(INSTALL_DIR)"
	for module in $(MODULES); do \
		install -m644 "$$module" "$(INSTALL_DIR)/$${module##*/}"; \
	done
	install -d "$(DEVKIT_INSTALL_DIR)" "$(DEVKIT_INCLUDE_DIR)"
	install -m644 "$(DEVKIT_LIBRARY)" "$(DEVKIT_INSTALL_DIR)/libdevkit.a"
	install -m644 devkit/include/devkit/*.h "$(DEVKIT_INCLUDE_DIR)"

clean:
	rm -rf "$(BUILD_DIR)" "$(OUTPUT_DIR)"

-include $(shell find "$(BUILD_DIR)" -name '*.d' -type f 2>/dev/null)
