ARCH ?=
CC ?= cc
LD ?= ld
STRIP ?= strip
DESTDIR ?=
PREFIX ?= /usr

BUILD_DIR := build/$(ARCH)
OUTPUT_DIR := out/$(ARCH)
MODULE_DIR := $(DESTDIR)$(PREFIX)/lib/roanix/drivers
INCLUDE_DIR := $(DESTDIR)$(PREFIX)/include/roanix

# Modules are freestanding, self-contained shared objects. They link against no
# library at all: the framework API is header-only and every kernel service is
# reached through the table handed to the module entry point.
COMMON_CFLAGS := \
	-O3 -std=c11 -ffreestanding -fPIC -fvisibility=hidden \
	-fno-builtin -fno-stack-protector -fno-unwind-tables \
	-fno-asynchronous-unwind-tables -ffunction-sections -fdata-sections \
	-Wall -Wextra -Werror -Iinclude

# The loader accepts only self-contained images, so undefined symbols and text
# relocations are rejected here rather than at boot.
COMMON_LDFLAGS := \
	-shared -Bsymbolic --no-undefined --gc-sections --build-id=none \
	--hash-style=sysv -z now -z relro -z separate-code -z noexecstack \
	-z max-page-size=4096 -e rdf_module_entry

ifeq ($(ARCH),x86_64)
ARCH_CFLAGS := \
	-march=x86-64 -mtune=generic -mcmodel=small -mno-red-zone \
	-mgeneral-regs-only -fcf-protection=none
MODULES := \
	$(OUTPUT_DIR)/acpi.ko \
	$(OUTPUT_DIR)/ioapic.ko \
	$(OUTPUT_DIR)/pty.ko \
	$(OUTPUT_DIR)/special.ko \
	$(OUTPUT_DIR)/uart8250.ko
else ifeq ($(ARCH),riscv64)
ARCH_CFLAGS := \
	-march=rv64imac_zicsr_zifencei -mabi=lp64 -mcmodel=medany \
	-msmall-data-limit=0 -mno-save-restore
MODULES := \
	$(OUTPUT_DIR)/fdt.ko \
	$(OUTPUT_DIR)/pty.ko \
	$(OUTPUT_DIR)/special.ko \
	$(OUTPUT_DIR)/uart8250.ko
else
$(error Unsupported ARCH "$(ARCH)")
endif

CFLAGS := $(COMMON_CFLAGS) $(ARCH_CFLAGS) $(EXTRA_CFLAGS)

.PHONY: all clean install

all: $(MODULES)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Each module is one translation unit today, but the rule takes a list so a
# driver can grow into several files without changing anything else.
define link_module
$(OUTPUT_DIR)/$(1).ko: $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(2)))
	@mkdir -p $$(@D)
	$(LD) $(COMMON_LDFLAGS) -soname $$(@F) -o $$@ $$^
	$(STRIP) --strip-debug $$@
endef

$(eval $(call link_module,acpi,platform/acpi))
$(eval $(call link_module,fdt,platform/fdt))
$(eval $(call link_module,ioapic,irqchip/ioapic))
$(eval $(call link_module,uart8250,tty/uart8250))
$(eval $(call link_module,pty,tty/pty))
$(eval $(call link_module,special,char/special))

install: all
	install -d "$(MODULE_DIR)"
	for module in $(MODULES); do \
		install -m644 "$$module" "$(MODULE_DIR)/$${module##*/}"; \
	done
	install -d "$(INCLUDE_DIR)"
	install -m644 include/roanix/*.h "$(INCLUDE_DIR)"

clean:
	rm -rf "$(BUILD_DIR)" "$(OUTPUT_DIR)"

-include $(shell find "$(BUILD_DIR)" -name '*.d' -type f 2>/dev/null)
