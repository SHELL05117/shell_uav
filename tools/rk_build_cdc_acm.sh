#!/usr/bin/env bash
# Build only cdc-acm for the inspected Toybrick 4.4.194 BSP source.
# Offline: sources/bc are supplied by the PC. Does not install/load any module.
# Usage: bash rk_build_cdc_acm.sh SOURCE_DIRECTORY OUTPUT_DIRECTORY [BC_BIN_DIR]
set -eo pipefail
source_dir=$(cd -- "${1:?kernel source directory required}" && pwd)
output_dir=${2:?output directory required}
if [[ -n ${3:-} ]]; then export PATH="$3:$PATH"; fi
test "$(sed -n 's/^VERSION = //p' "$source_dir/Makefile" | head -n1)" = 4
test "$(sed -n 's/^PATCHLEVEL = //p' "$source_dir/Makefile" | head -n1)" = 4
test "$(sed -n 's/^SUBLEVEL = //p' "$source_dir/Makefile" | head -n1)" = 194
command -v bc >/dev/null
cd "$source_dir"
make ARCH=arm64 rockchip_linux_defconfig
scripts/config --disable MODVERSIONS --disable LOCALVERSION_AUTO --set-str LOCALVERSION '' --module USB_ACM
make ARCH=arm64 olddefconfig
make ARCH=arm64 -j2 prepare modules_prepare
make ARCH=arm64 -j2 M=drivers/usb/class CONFIG_USB_ACM=m modules
mkdir -p "$output_dir"
cp drivers/usb/class/cdc-acm.ko "$output_dir/cdc-acm.ko"
cp .config "$output_dir/build.config"
modinfo "$output_dir/cdc-acm.ko" > "$output_dir/modinfo.txt"
sha256sum "$output_dir/cdc-acm.ko"
printf 'Build complete. Verify running-image provenance, structure layouts, exports and vermagic before installation.\n'
