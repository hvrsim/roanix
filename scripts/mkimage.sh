#!/usr/bin/env sh
# ===----------------------------------------------------------------------===##
#
# Part of the Roanix Project, under the Mozilla Public License 2.0.
# See LICENSE in the project root for license information.
# SPDX-License-Identifier: MPL-2.0
#
# ===----------------------------------------------------------------------===##

image_type="$1"
image_name="$2"

KERNEL_PATH="$(realpath $(dirname $0))/../pkgs/kernel/boot/roanix"
LIMINE_PATH="$(realpath $(dirname $0))/../pkgs/limine/usr/share/limine"
DISTRO_PATH="$(realpath $(dirname $0))/../distro-files"

. "$(realpath $(dirname $0))/../jinx-config"

chkcmd() { 
    command -v $1 >/dev/null 2>&1 || { echo >&2 "Command $1 is not installed! Aborting..."; exit 1; }
}

if [ "$image_type" = "iso" ]; then
    chkcmd "xorriso"

    rm -rf iso_root
    mkdir -p iso_root/boot
    cp -v $KERNEL_PATH iso_root/boot/
    cp -v $DISTRO_PATH/splash.jpg iso_root/boot/
    mkdir -p iso_root/boot/limine
    cp -v $DISTRO_PATH/limine.conf iso_root/boot/limine/
    mkdir -p iso_root/EFI/BOOT

    if [ "$common_arch" = "riscv64" ]; then
        cp -v "$LIMINE_PATH/limine-uefi-cd.bin" iso_root/boot/limine/
        cp -v "$LIMINE_PATH/BOOTRISCV64.EFI" iso_root/EFI/BOOT/

	xorriso -as mkisofs -R -r -J \
		-hfsplus -apm-block-size 2048 \
                --efi-boot boot/limine/limine-uefi-cd.bin \
                -efi-boot-part --efi-boot-image --protective-msdos-label \
                iso_root -o "$image_name.iso"
    elif [ "$common_arch" = "x86_64" ]; then
        cp -v "$LIMINE_PATH/limine-uefi-cd.bin" iso_root/boot/limine/
        cp -v "$LIMINE_PATH/limine-bios-cd.bin" iso_root/boot/limine/
        cp -v "$LIMINE_PATH/limine-bios.sys" iso_root/boot/limine/
        cp -v "$LIMINE_PATH/BOOTX64.EFI" iso_root/EFI/BOOT/
        cp -v "$LIMINE_PATH/BOOTIA32.EFI" iso_root/EFI/BOOT/

        xorriso -as mkisofs -R -r -J -b boot/limine/limine-bios-cd.bin \
		-no-emul-boot -boot-load-size 4 -boot-info-table -hfsplus \
		-apm-block-size 2048 --efi-boot boot/limine/limine-uefi-cd.bin \
                -efi-boot-part --efi-boot-image --protective-msdos-label \
                iso_root -o "$image_name.iso"
   fi

    rm -rf iso_root
elif [ "$image_type" = "hdd" ]; then
    chkcmd "sgdisk"
    chkcmd "mformat"

    rm -f "$image_name.hdd"
    dd if=/dev/zero bs=1M count=0 seek=32 of="$image_name.hdd"
    sgdisk "$image_name.hdd" -n 1:2048 -t 1:ef00
    
    mformat -i "$image_name.hdd"@@1M
    mmd -i "$image_name.hdd"@@1M ::/EFI ::/EFI/BOOT ::/boot ::/boot/limine
    mcopy -i "$image_name.hdd"@@1M $KERNEL_PATH ::/boot
    mcopy -i "$image_name.hdd"@@1M $DISTRO_PATH/splash.jpg ::/boot
    mcopy -i "$image_name.hdd"@@1M $DISTRO_PATH/limine.conf ::/boot/limine

    if [ "$common_arch" = "riscv64" ]; then
        mcopy -i "$image_name.hdd"@@1M "$LIMINE_PATH/BOOTRISCV64.EFI" ::/EFI/BOOT
    elif [ "$common_arch" = "x86_64" ]; then
        mcopy -i "$image_name.hdd"@@1M "$LIMINE_PATH/BOOTX64.EFI" ::/EFI/BOOT
        mcopy -i "$image_name.hdd"@@1M "$LIMINE_PATH/BOOTIA32.EFI" ::/EFI/BOOT
    fi
fi
