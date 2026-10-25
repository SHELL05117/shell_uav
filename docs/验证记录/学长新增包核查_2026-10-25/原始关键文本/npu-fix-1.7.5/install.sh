#!/bin/bash
# Board-side installer: put the RK3399Pro NPU (RK1808) firmware and tools into
# an already flashed system. No partition has to be re-flashed.
#
#   sudo ./install.sh              # install + load the NPU now
#   sudo ./install.sh --no-start   # install only, load on next boot
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
FW_DIR=/usr/lib/firmware/npu_fw
UNIT=rknn.service
START=1
[ "${1:-}" = "--no-start" ] && START=0

die()  { echo "错误: $*" >&2; exit 1; }
info() { echo "[npu-fix] $*"; }
warn() { echo "[npu-fix] 警告: $*" >&2; }

[ "$(id -u)" -eq 0 ] || die "请用 root 运行：sudo $0"
[ "$(uname -m)" = "aarch64" ] || die "只能在板端（aarch64）运行"
[ -d "$HERE/firmware" ] || die "文件不完整：缺少 firmware/"

info "校验文件"
( cd "$HERE" && sha256sum -c SHA256SUMS ) >/dev/null || die "校验失败，请重新拷贝"

# The USB3 host side (usbdrd3_1) comes from the DTB and is already fine; these
# two are what npu_powerctrl needs on top of it.
[ -e /sys/class/gpio/export ] || die "内核缺少 CONFIG_GPIO_SYSFS，NPU 无法上电（需要重编内核）"
if [ ! -d /sys/kernel/debug/clk ]; then
  mountpoint -q /sys/kernel/debug 2>/dev/null ||
    mount -t debugfs debugfs /sys/kernel/debug 2>/dev/null || true
fi
[ -d /sys/kernel/debug/clk ] ||
  warn "看不到 /sys/kernel/debug/clk，npu_powerctrl 可能失败（内核缺 CONFIG_DEBUG_FS）"

# npu_upgrade picks its branch from the model string (TB-RK3399Pro*).
BOARD=${NPU_BOARD:-}
if [ -z "$BOARD" ] && [ -r /proc/device-tree/model ]; then
  model=$(tr -d '\0' < /proc/device-tree/model)
  case $model in TB-RK3399Pro*) BOARD=$model ;; esac
fi
BOARD=${BOARD:-TB-RK3399ProD}

BACKUP=/var/backups/npu-fix-$(date +%Y%m%d-%H%M%S)
backup() {
  [ -e "$1" ] || return 0
  mkdir -p "$BACKUP"
  cp -a --parents "$1" "$BACKUP/" 2>/dev/null ||
    cp -a "$1" "$BACKUP/$(basename "$1")"
}

info "安装 NPU 固件 -> $FW_DIR"
install -d "$FW_DIR"
for f in "$HERE"/firmware/npu_fw/*; do
  backup "$FW_DIR/$(basename "$f")"
  install -m 0644 "$f" "$FW_DIR/"
done

info "安装工具 -> /usr/bin"
install -d /usr/bin
for f in "$HERE"/bin/*; do
  backup "/usr/bin/$(basename "$f")"
  install -m 0755 "$f" /usr/bin/
done

info "安装 librknn_api.so"
backup /usr/lib/aarch64-linux-gnu/librknn_api.so
install -D -m 0644 "$HERE/lib/librknn_api.so" \
  /usr/lib/aarch64-linux-gnu/librknn_api.so
ldconfig 2>/dev/null || true

info "安装开机加载服务（$BOARD）"
install -d /usr/local/sbin
install -m 0755 "$HERE/sbin/rknn-start" "$HERE/sbin/rknn-stop" /usr/local/sbin/
install -d /etc/default
printf 'NPU_BOARD=%s\n' "$BOARD" > /etc/default/rknn
install -d /etc/systemd/system
install -m 0644 "$HERE/systemd/rknn.service" "/etc/systemd/system/$UNIT"
systemctl daemon-reload
systemctl enable "$UNIT" >/dev/null 2>&1

if [ "$START" -eq 0 ]; then
  echo
  info "已安装，NPU 将在下次开机由 rknn.service 加载"
  exit 0
fi

# The RK1808 sits on the Rockchip USB VID 0x2207; scanning sysfs avoids a
# dependency on usbutils and works before/after the firmware boot.
npu_seen() {
  grep -l -x 2207 /sys/bus/usb/devices/*/idVendor >/dev/null 2>&1
}

info "推送固件并启动 npu_transfer_proxy（约 10~30 秒）"
# restart waits for the oneshot unit, i.e. for npu_upgrade to finish.
if ! systemctl restart "$UNIT"; then
  warn "rknn.service 启动失败，详见下面的日志"
fi
for _ in $(seq 1 15); do
  npu_seen && break
  sleep 2
done

# A maskrom NPU also shows VID 2207, so an active unit (firmware push returned
# 0) is what tells the two apart.
if systemctl is-active --quiet "$UNIT" && npu_seen; then
  echo
  info "完成：NPU 固件已加载并在 USB 上枚举"
  command -v lsusb >/dev/null 2>&1 && lsusb -d 2207:
  echo "  查看传输方式: npu_transfer_proxy devices"
  echo "  失败重试:     sudo systemctl restart rknn"
  [ -d "$BACKUP" ] && echo "  旧文件备份:   $BACKUP"
  exit 0
fi

echo
warn "NPU 未就绪：USB 上没有 2207 设备，或固件推送失败"
systemctl --no-pager --full status "$UNIT" 2>&1 | tail -n 15 || true
echo "--- journalctl -u rknn -b ---"
journalctl -u "$UNIT" -b --no-pager 2>/dev/null | tail -n 20 || true
exit 1
