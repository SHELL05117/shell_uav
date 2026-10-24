#!/usr/bin/env bash
# Temporary RNDIS setup for SHELL's TB-RK3399ProD; run through Wi-Fi SSH.
# References: Linux 4.4 configfs / f_rndis; Microsoft OS 1.0 descriptors.
set -euo pipefail

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
[[ $EUID -eq 0 ]] || die 'Run with sudo bash rk_usb_ssh.sh [start|stop].'
action=${1:-start}
[[ $action == start || $action == stop ]] || die 'Expected start or stop.'
g=/sys/kernel/config/usb_gadget/shell_usb_ssh
udc=fe800000.dwc3
address=192.168.77.1/24
func=$g/functions/rndis.usb0

show_interface() {
    local iface
    iface=$(cat "$func/ifname")
    [[ $iface =~ ^[a-zA-Z0-9_.:-]+$ && -d /sys/class/net/$iface ]] \
        || die "USB interface not ready: $iface"
    printf '%s' "$iface"
}

if [[ -d $g ]]; then
    [[ -f $g/strings/0x409/manufacturer ]] \
        && [[ $(cat "$g/strings/0x409/manufacturer") == SHELL ]] \
        || die "Unrecognized existing gadget at $g; inspect it manually."
fi

if [[ $action == stop ]]; then
    if [[ -d $g ]]; then
        iface=$(cat "$func/ifname" 2>/dev/null || true)
        if [[ $iface =~ ^[a-zA-Z0-9_.:-]+$ && -d /sys/class/net/$iface ]]; then
            ip address del "$address" dev "$iface" 2>/dev/null || true
            ip link set "$iface" down
        fi
        if [[ -n $(cat "$g/UDC") ]]; then printf '\n' > "$g/UDC"; fi
    fi
    printf 'SHELL USB gadget unbound. Wi-Fi is unchanged.\n'
    exit 0
fi

[[ -d /sys/class/udc/$udc ]] || die "Controller $udc is not registered."
[[ -d /sys/kernel/config/usb_gadget ]] || die 'USB gadget configfs is not available.'
for other in /sys/kernel/config/usb_gadget/*; do
    [[ -d $other && $other != "$g" && -f $other/UDC ]] || continue
    [[ $(cat "$other/UDC") != "$udc" ]] || die "Controller is used by $other."
done

if [[ ! -d $g ]]; then
    mkdir "$g"
    mkdir -p "$g/strings/0x409"
    printf 'SHELL\n' > "$g/strings/0x409/manufacturer"
fi

if [[ -z $(cat "$g/UDC") ]]; then
    # Development-only Linux multifunction gadget identity.
    printf '0x1d6b\n' > "$g/idVendor"
    printf '0x0104\n' > "$g/idProduct"
    printf '0x0200\n' > "$g/bcdUSB"
    printf '0x0100\n' > "$g/bcdDevice"
    printf '0xef\n' > "$g/bDeviceClass"
    printf '0x02\n' > "$g/bDeviceSubClass"
    printf '0x01\n' > "$g/bDeviceProtocol"
    printf 'SHELL-RK3399PROD-USBSSH\n' > "$g/strings/0x409/serialnumber"
    printf 'SHELL RK USB SSH\n' > "$g/strings/0x409/product"
    mkdir -p "$g/configs/c.1/strings/0x409"
    printf 'RNDIS network\n' > "$g/configs/c.1/strings/0x409/configuration"
    printf '0xc0\n' > "$g/configs/c.1/bmAttributes"
    printf '2\n' > "$g/configs/c.1/MaxPower"
    mkdir -p "$func"
    if [[ ! -L $g/configs/c.1/rndis.usb0 ]]; then
        printf '02:53:48:45:4c:01\n' > "$func/dev_addr"
        printf '02:53:48:45:4c:02\n' > "$func/host_addr"
    fi
    [[ -d $func/os_desc/interface.rndis ]] || die 'RNDIS OS descriptors are unavailable.'
    printf 'RNDIS\n' > "$func/os_desc/interface.rndis/compatible_id"
    # Keep the fresh function's default zero-filled sub-compatible ID.
    printf '1\n' > "$g/os_desc/use"
    printf '0xcd\n' > "$g/os_desc/b_vendor_code"
    printf 'MSFT100\n' > "$g/os_desc/qw_sign"
    [[ -L $g/configs/c.1/rndis.usb0 ]] || ln -s "$func" "$g/configs/c.1/rndis.usb0"
    [[ -L $g/os_desc/c.1 ]] || ln -s "$g/configs/c.1" "$g/os_desc/c.1"
    printf '%s\n' "$udc" > "$g/UDC"
fi

iface=$(show_interface)
ip address replace "$address" dev "$iface"
ip link set "$iface" up
ip -br address show dev "$iface"
printf 'Board: 192.168.77.1/24; Windows: 192.168.77.2/24, no gateway/DNS.\n'
printf 'Windows USB MAC: 02-53-48-45-4C-02. This setup lasts until reboot.\n'
printf 'Rollback from Wi-Fi: sudo bash %q stop\n' "$0"
