#!/usr/bin/env python3
"""Disable every PCA9685 output and stop its oscillator; never enables PWM.

Use before a physical OE wiring change and after a servo bench run. Not a
servo-motion, voltage, or mechanical acceptance test. Other PCA writers must
be stopped first. TFmini on the same bus/address 0x10 is left untouched.
"""
import datetime
import fcntl
import json
import os
import sys


def disable():
    fd = os.open('/dev/i2c-6', os.O_RDWR)
    fcntl.ioctl(fd, 0x0703, 0x40)
    def read(reg):
        if os.write(fd, bytes([reg])) != 1:
            raise OSError('PCA pointer short write')
        data = os.read(fd, 1)
        if len(data) != 1:
            raise OSError('PCA short read')
        return data[0]
    def write(reg, value):
        if os.write(fd, bytes([reg, value])) != 2:
            raise OSError('PCA short write')
    try:
        before = read(0)
        for channel in range(16):
            write(0x09 + channel*4, 0x10)  # FULL_OFF has priority over FULL_ON
        write(0, (before | 0x10) & 0x7f)  # SLEEP, no RESTART command
        after = read(0)
        channels = [read(0x09 + channel*4) for channel in range(16)]
        result = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            mode1_before=before, mode1_after=after, off_high=channels,
            passed=bool(after & 0x10) and all(value & 0x10 for value in channels),
            oe_level_not_measured=True, motion_or_voltage_acceptance=False)
        if not result['passed']:
            raise RuntimeError('PCA disable readback failed')
        return result
    finally:
        os.close(fd)


if __name__ == '__main__':
    result = disable()
    if len(sys.argv) == 2:
        with open(sys.argv[1], 'x') as target:
            json.dump(result, target, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))
