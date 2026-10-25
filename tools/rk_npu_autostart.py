#!/usr/bin/env python3
"""Bounded firmware/runtime readiness stages for SHELL's systemd NPU units.

Python 3.6 standard library. Power/GPIO ownership stays in rk_npu_power.py;
firmware/tool/USB target checks stay in rk_npu_boot.sh. No automatic reset.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

from rk_npu_power import Controller, System
from rk_npu_usb import internal_target

ROOT = Path('/usr/local/lib/shell-rk-npu')


def owned_runtime(io):
    state = io.state()
    if not state or state.get('phase') != 'powered':
        raise RuntimeError('NPU power ownership is not ready')
    Controller(io).healthy(state)
    target = internal_target(allow_absent=True)
    return {target['port']: '2207:' + target['product']} if target else {}


def wait_runtime(io, timeout=60):
    deadline = time.monotonic() + timeout
    while True:
        usb = owned_runtime(io)
        if len(usb) == 1 and list(usb.values()) == ['2207:0019']:
            port = next(iter(usb))
            try:
                serial = (Path('/sys/bus/usb/devices') / port / 'serial').read_text().strip()
            except OSError:
                serial = ''
            # Firmware first exposes a placeholder serial while still booting.
            if serial and serial != '0123456789ABCDEF':
                print('NPU runtime USB ready: %s serial=%s' % (port, serial), flush=True)
                return
        if time.monotonic() >= deadline:
            raise RuntimeError('NPU runtime USB/serial not ready within %ss: %r' % (timeout, usb))
        time.sleep(1)


def firmware(io):
    usb = owned_runtime(io)
    if len(usb) == 1 and list(usb.values()) == ['2207:0019']:
        print('Runtime already present; no firmware transfer/reset', flush=True)
    elif len(usb) == 1 and list(usb.values()) == ['2207:180a']:
        # The existing helper independently checks hashes, board and target.
        subprocess.run(['/bin/bash', str(ROOT / 'rk_npu_boot.sh'), 'start'], check=True, timeout=190)
    else:
        raise RuntimeError('Unexpected NPU boot USB state: %r' % usb)
    wait_runtime(io)
    # USB enumeration alone did not imply rknn_server readiness in live tests.
    time.sleep(10)
    owned_runtime(io)
    print('NPU firmware stage ready', flush=True)


def proxy(io, timeout=60):
    wait_runtime(io, timeout=5)
    deadline = time.monotonic() + timeout
    last = ''
    while True:
        usb = owned_runtime(io)
        try:
            result = subprocess.run(['/usr/local/bin/npu_transfer_proxy', 'devices'],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    universal_newlines=True, timeout=5)
            last = result.stdout
            rows = [line.split() for line in last.splitlines() if 'USB_DEVICE' in line]
            if result.returncode == 0 and len(rows) == 1 and len(rows[0]) == 3:
                port = next(iter(usb))
                serial = (Path('/sys/bus/usb/devices') / port / 'serial').read_text().strip()
                if rows[0][0] == serial:
                    print(last, end='', flush=True)
                    print('NPU proxy device ready', flush=True)
                    return
        except subprocess.TimeoutExpired:
            last = 'devices query timed out'
        if time.monotonic() >= deadline:
            raise RuntimeError('NPU proxy not ready: ' + last)
        time.sleep(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage', choices=['firmware', 'proxy'])
    args = ap.parse_args()
    try:
        {'firmware': firmware, 'proxy': proxy}[args.stage](System())
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print('NPU startup failed: %s' % exc, file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
