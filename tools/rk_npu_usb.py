#!/usr/bin/env python3
"""Identify the unique board-internal NPU without fixing a Linux USB bus number.

Python 3.6 standard library. Read-only sysfs/ownership/listing validation;
firmware transfer remains in rk_npu_boot.sh behind its hash checks.
"""
import argparse
import re
import sys
from pathlib import Path

USB_ROOT = Path('/sys/bus/usb/devices')
CONTROLLER = 'fe900000.dwc3'


def internal_target(root=USB_ROOT, allow_absent=False):
    devices = []
    for path in root.iterdir():
        try:
            vendor = (path / 'idVendor').read_text().strip().lower()
        except OSError:
            continue  # USB interface nodes have no device descriptor.
        if vendor == '2207':
            devices.append(path)
    if not devices and allow_absent:
        return None
    if len(devices) != 1:
        raise RuntimeError('Expected exactly one Rockchip USB device; got %r' %
                           [p.name for p in devices])
    path = devices[0]
    match = re.fullmatch(r'([1-9][0-9]*)-1\.1', path.name)
    if not match:
        raise RuntimeError('Rockchip device is not on internal hub port 1: ' + path.name)
    bus = int((path / 'busnum').read_text().strip())
    if bus != int(match.group(1)) or (path / 'devpath').read_text().strip() != '1.1':
        raise RuntimeError('USB bus/path attributes disagree with device identity')
    resolved = path.resolve(strict=True)
    hub = resolved.parent
    if (resolved.parts[-3:] != ('usb%d' % bus, '%d-1' % bus, path.name) or
            CONTROLLER not in resolved.parts or
            not resolved.parent.parent.parent.name.startswith('xhci-hcd.')):
        raise RuntimeError('Rockchip USB device is outside the board internal controller')
    if ((hub / 'idVendor').read_text().strip().lower() != '05e3' or
            (hub / 'idProduct').read_text().strip().lower() not in ('0610', '0616')):
        raise RuntimeError('Unexpected parent hub for internal NPU')
    product = (path / 'idProduct').read_text().strip().lower()
    if product not in ('180a', '0019'):
        raise RuntimeError('Unexpected internal Rockchip USB product: ' + product)
    return dict(port=path.name, product=product, bus=bus)


def listing_location(listing, mode, expected=''):
    rows = [line.strip() for line in listing.splitlines()
            if line.strip().startswith('DevNo=')]
    if len(rows) != 1:
        raise RuntimeError('Ambiguous rockusb tool listing')
    match = re.fullmatch(r'DevNo=\d+\s+Vid=0x2207,Pid=0x180a,LocationID=([0-9]+)\s+Mode=(Maskrom|Loader)(?:\s+SerialNo=\S+)?', rows[0])
    if not match or match.group(2) != mode:
        raise RuntimeError('Unexpected tool device or mode: ' + rows[0])
    location = match.group(1)
    if expected and location != expected:
        raise RuntimeError('Rockusb location changed during this transfer')
    return location


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('Maskrom', 'Loader'))
    parser.add_argument('expected_location', nargs='?', default='')
    args = parser.parse_args()
    try:
        from rk_npu_power import COMPATIBLE, Controller, System
        io = System()
        compatible = Path('/proc/device-tree/compatible').read_bytes().split(b'\0')
        if COMPATIBLE.encode('ascii') not in compatible:
            raise RuntimeError('Wrong board')
        state = io.state()
        if not state or state.get('phase') != 'powered':
            raise RuntimeError('NPU power ownership is not ready')
        Controller(io).healthy(state)
        target = internal_target()
        if target['product'] != '180a':
            raise RuntimeError('NPU is not in boot USB mode; refusing firmware transfer')
        # Both sysfs and the vendor tool must see exactly one Rockchip device.
        # That binds the tool row to the checked physical target without guessing
        # the proprietary LocationID encoding. Pin its observed ID for this run.
        location = listing_location(sys.stdin.read(), args.mode, args.expected_location)
        print('Unique internal NPU: %s / 2207:180a LocationID=%s mode=%s' %
              (target['port'], location, args.mode), file=sys.stderr)
        print(location)
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        print('NPU USB guard refused: %s' % error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
