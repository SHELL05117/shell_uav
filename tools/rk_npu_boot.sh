#!/usr/bin/env bash
# Load the USB NPU firmware into the ProD NPU using the vendor RAM-boot flow.
# Source sequence: JeffyCN/mirrors 2a532b012b5179dd573d8b7f98fc2c51b3046409,
# bin/npu_upgrade. Does not reset NPU power or write the host eMMC partitions.
set -euo pipefail
action=${1:-check}
[[ $action == check || $action == start || $action == resume ]] || { echo 'Expected check, start or resume' >&2; exit 2; }
[[ $EUID == 0 ]] || { echo 'Run on RK with sudo bash rk_npu_boot.sh [check|start|resume]' >&2; exit 2; }
tool=/home/jcfh/npu_runtime_175/upgrade_tool
repo=/home/jcfh/RK3399Pro_npu
fw=$repo/drivers/npu_firmware/npu_fw
expected_commit=8858114555f1700e41e185b14ec3d266342f8059
expected_tool=09dfa63f4a9f3e38bab3c0ee2731a08e96e0fb52f21d259a028080204621da4c
[[ $(sha256sum "$tool" | cut -d ' ' -f 1) == "$expected_tool" ]] || { echo 'Tool SHA256 differs' >&2; exit 1; }
# systemd root has no sudo caller identity; trust only this audited repository
# for these read-only Git checks, without changing global safe.directory.
[[ $(git -c safe.directory="$repo" -C "$repo" rev-parse HEAD) == "$expected_commit" ]] || { echo 'Firmware repository commit differs' >&2; exit 1; }
for name in MiniLoaderAll.bin uboot.img trust.img boot.img; do
    path=drivers/npu_firmware/npu_fw/$name
    [[ $(git -c safe.directory="$repo" -C "$repo" hash-object "$path") == $(git -c safe.directory="$repo" -C "$repo" rev-parse "HEAD:$path") ]] \
        || { echo "Firmware blob differs: $name" >&2; exit 1; }
done

guard_usb() {
    python3 - <<'PY'
import json
from pathlib import Path
compat=Path('/proc/device-tree/compatible').read_bytes().split(b'\0')
assert b'rockchip,rk3399pro-toybrick-prod-linux' in compat, 'Wrong board'
state=json.loads(Path('/run/shell-rk-npu-power.json').read_text())
assert state['phase']=='powered' and state['profile']=='prod-20181217', 'NPU power not owned/ready'
assert state['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip(), 'Stale power record'
devices=[]
for p in Path('/sys/bus/usb/devices').iterdir():
    try:
        if (p/'idVendor').read_text().strip()=='2207':
            devices.append((p.name,(p/'idProduct').read_text().strip()))
    except OSError:
        pass
assert devices==[('3-1.1','180a')], 'Expected exactly one NPU at 3-1.1; got %r'%devices
assert Path('/sys/bus/usb/devices/3-1/idVendor').read_text().strip()=='05e3', 'Wrong parent hub'
assert Path('/sys/bus/usb/devices/3-1/idProduct').read_text().strip()=='0610', 'Wrong parent hub'
print('Unique NPU USB target: 3-1.1 / 2207:180a')
PY
}

run_stage() {
    guard_usb
    printf '\nNPU stage: %s\n' "$1"
    shift
    timeout 30 "$tool" "$@"
}

guard_usb
listing=$("$tool" ld)
printf '%s\n' "$listing"
[[ $(printf '%s\n' "$listing" | grep -c '^DevNo=') == 1 ]] || { echo 'Ambiguous rockusb target' >&2; exit 1; }
mode=Maskrom
[[ $action != resume ]] || mode=Loader
[[ $listing == *LocationID=30101* && $listing == *Mode=$mode* ]] \
    || { echo "Expected NPU $mode at LocationID=30101; no automatic reset" >&2; exit 1; }
[[ $action != check ]] || { echo 'Check passed; no firmware transferred.'; exit 0; }

if [[ $action == start ]]; then
    run_stage loader db "$fw/MiniLoaderAll.bin"
    sleep 1
fi
run_stage loader-ready td
run_stage uboot-RAM wl 0x20000 "$fw/uboot.img"
run_stage trust-RAM wl 0x20800 "$fw/trust.img"
run_stage boot-RAM wl 0x21000 "$fw/boot.img"
run_stage run-system rs 0x20000 0x20800 0x21000 "$fw/uboot.img" "$fw/trust.img" "$fw/boot.img"
echo 'NPU RAM boot commands completed; verify runtime USB and proxy separately.'
