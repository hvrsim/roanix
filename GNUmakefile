# Nuke built-in rules and variables.
MAKEFLAGS += -rR
.SUFFIXES:

# Convenience macro to reliably declare user overridable variables.
override USER_VARIABLE = $(if $(filter $(origin $(1)),default undefined),$(eval override $(1) := $(2)))

# Target architecture to build for. Default to x86_64.
$(call USER_VARIABLE,KARCH,x86_64)

# Default user QEMU flags. These are appended to the QEMU command calls.
$(call USER_VARIABLE,QEMUFLAGS,-m 2G)

override IMAGE_NAME := roanix-$(KARCH)

.PHONY: all
all: $(IMAGE_NAME).hdd

.PHONY: all-iso
all-iso: $(IMAGE_NAME).iso

.PHONY: run
run: run-$(KARCH)

.PHONY: run-iso
run-iso: run-iso-$(KARCH)

.PHONY: run-x86_64
run-x86_64: store/edk2-ovmf $(IMAGE_NAME).hdd
	qemu-system-$(KARCH) \
		-M q35 \
		-cpu qemu64,+fsgsbase \
		-drive if=pflash,unit=0,format=raw,file=store/edk2-ovmf/ovmf-code-$(KARCH).fd,readonly=on \
		-hda $(IMAGE_NAME).hdd \
		$(QEMUFLAGS)

.PHONY: run-iso-x86_64
run-iso-x86_64: store/edk2-ovmf $(IMAGE_NAME).iso
	qemu-system-$(KARCH) \
		-M q35 \
		-cpu qemu64,+fsgsbase \
		-drive if=pflash,unit=0,format=raw,file=store/edk2-ovmf/ovmf-code-$(KARCH).fd,readonly=on \
		-cdrom $(IMAGE_NAME).iso \
		$(QEMUFLAGS)

.PHONY: run-riscv64
run-riscv64: store/edk2-ovmf $(IMAGE_NAME).hdd
	qemu-system-$(KARCH) \
		-M virt \
		-cpu rv64 \
		-device ramfb \
		-device qemu-xhci \
		-device usb-kbd \
		-device usb-mouse \
		-drive if=pflash,unit=0,format=raw,file=store/edk2-ovmf/ovmf-code-$(KARCH).fd,readonly=on \
		-hda $(IMAGE_NAME).hdd \
		$(QEMUFLAGS)

.PHONY: run-iso-riscv64
run-iso-riscv64: store/edk2-ovmf $(IMAGE_NAME).iso
	qemu-system-$(KARCH) \
		-M virt \
		-cpu rv64 \
		-device ramfb \
		-device qemu-xhci \
		-device usb-kbd \
		-device usb-mouse \
		-drive if=pflash,unit=0,format=raw,file=store/edk2-ovmf/ovmf-code-$(KARCH).fd,readonly=on \
		-cdrom $(IMAGE_NAME).iso \
		$(QEMUFLAGS)

.PHONY: run-bios
run-bios: $(IMAGE_NAME).hdd
	qemu-system-$(KARCH) \
		-M q35 \
		-cpu qemu64,+fsgsbase \
		-hda $(IMAGE_NAME).hdd \
		$(QEMUFLAGS)

.PHONY: run-iso-bios
run-iso-bios: $(IMAGE_NAME).iso
	qemu-system-$(KARCH) \
		-M q35 \
		-cpu qemu64,+fsgsbase \
		-cdrom $(IMAGE_NAME).iso \
		-boot d \
		$(QEMUFLAGS)

store/edk2-ovmf:
	rm -rf store/edk2-ovmf
	mkdir -p store/edk2-ovmf
	cd store && curl -L https://github.com/osdev0/edk2-ovmf-nightly/releases/latest/download/edk2-ovmf.tar.gz | gunzip | tar -xf -

store/limine:
	rm -rf store/limine
	mkdir -p store/limine
	git clone https://github.com/limine-bootloader/limine.git --branch=v8.x-binary --depth=1 store/limine
	$(MAKE) -C store/limine

.PHONY: kernel
kernel:
	$(MAKE) -C kernel

.PHONY: rustdoc
rustdoc:
	$(MAKE) -C kernel rustdoc

$(IMAGE_NAME).iso: store/limine kernel
	rm -rf iso_root
	mkdir -p iso_root/boot
	cp -v kernel/roanix iso_root/boot/
	cp -v userland/distro-files/splash.jpg iso_root/boot/
	mkdir -p iso_root/boot/limine
	cp -v userland/distro-files/limine.conf iso_root/boot/limine/
	mkdir -p iso_root/EFI/BOOT
ifeq ($(KARCH),x86_64)
	cp -v store/limine/limine-bios.sys store/limine/limine-bios-cd.bin store/limine/limine-uefi-cd.bin iso_root/boot/limine/
	cp -v store/limine/BOOTX64.EFI iso_root/EFI/BOOT/
	cp -v store/limine/BOOTIA32.EFI iso_root/EFI/BOOT/
	xorriso -as mkisofs -b boot/limine/limine-bios-cd.bin \
		-no-emul-boot -boot-load-size 4 -boot-info-table \
		--efi-boot boot/limine/limine-uefi-cd.bin \
		-efi-boot-part --efi-boot-image --protective-msdos-label \
		iso_root -o $(IMAGE_NAME).iso
	./store/limine/limine bios-install $(IMAGE_NAME).iso
endif
ifeq ($(KARCH),riscv64)
	cp -v store/limine/limine-uefi-cd.bin iso_root/boot/limine/
	cp -v store/limine/BOOTRISCV64.EFI iso_root/EFI/BOOT/
	xorriso -as mkisofs \
		--efi-boot boot/limine/limine-uefi-cd.bin \
		-efi-boot-part --efi-boot-image --protective-msdos-label \
		iso_root -o $(IMAGE_NAME).iso
endif
	rm -rf iso_root

$(IMAGE_NAME).hdd: store/limine kernel
	rm -f $(IMAGE_NAME).hdd
	dd if=/dev/zero bs=1M count=0 seek=64 of=$(IMAGE_NAME).hdd
	sgdisk $(IMAGE_NAME).hdd -n 1:2048 -t 1:ef00
ifeq ($(KARCH),x86_64)
	./store/limine/limine bios-install $(IMAGE_NAME).hdd
endif
	mformat -i $(IMAGE_NAME).hdd@@1M
	mmd -i $(IMAGE_NAME).hdd@@1M ::/EFI ::/EFI/BOOT ::/boot ::/boot/limine
	mcopy -i $(IMAGE_NAME).hdd@@1M kernel/roanix ::/boot
	mcopy -i $(IMAGE_NAME).hdd@@1M userland/distro-files/splash.jpg ::/boot
	mcopy -i $(IMAGE_NAME).hdd@@1M userland/distro-files/limine.conf ::/boot/limine
ifeq ($(KARCH),x86_64)
	mcopy -i $(IMAGE_NAME).hdd@@1M store/limine/limine-bios.sys ::/boot/limine
	mcopy -i $(IMAGE_NAME).hdd@@1M store/limine/BOOTX64.EFI ::/EFI/BOOT
	mcopy -i $(IMAGE_NAME).hdd@@1M store/limine/BOOTIA32.EFI ::/EFI/BOOT
endif
ifeq ($(KARCH),riscv64)
	mcopy -i $(IMAGE_NAME).hdd@@1M store/limine/BOOTRISCV64.EFI ::/EFI/BOOT
endif

.PHONY: book
book:
	cd book && mdbook serve

.PHONY: clean
clean:
	$(MAKE) -C kernel clean
	rm -rf iso_root $(IMAGE_NAME).iso $(IMAGE_NAME).hdd

.PHONY: distclean
distclean: clean
	$(MAKE) -C kernel distclean
	rm -rf store book/book target
