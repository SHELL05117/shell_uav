#!/usr/bin/env python3
"""Bounded TFmini raw-frame/PCA register observation; never writes PWM registers.

Run alone on the bus (stop the location/servo nodes first). The only TFmini
command is obtain-distance; no mode, address or persistent settings are changed.
PCA writes select the register pointer only. OE voltage needs a physical check.
"""
import argparse
import datetime
import fcntl
import json
import os
import statistics
import time


def select(fd, address):
    fcntl.ioctl(fd, 0x0703, address)  # I2C_SLAVE; never FORCE a bound driver


def pca_snapshot(fd):
    select(fd, 0x40)
    result = {}
    for reg in [0, 1, 0xfe, 0xfa, 0xfb, 0xfc, 0xfd] + list(range(6, 14)):
        if os.write(fd, bytes([reg])) != 1:
            raise OSError('Short PCA register-pointer write')
        data = os.read(fd, 1)
        if len(data) != 1:
            raise OSError('Short PCA register read')
        result['0x%02x' % reg] = data[0]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--response-delay-ms', type=float, default=1.0)
    args = parser.parse_args()
    if not 20 <= args.samples <= 1200:
        parser.error('samples must be 20..1200')
    if not 0 <= args.response_delay_ms <= 20:
        parser.error('response delay must be 0..20 ms')
    if os.path.exists(args.output):
        parser.error('refusing to overwrite evidence')
    report = dict(start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  bus=6, oe_voltage_measured_by_software=False,
                  response_delay_ms=args.response_delay_ms,
                  pwm_writes=0, samples=[], errors=[], passed=False)
    fd = os.open('/dev/i2c-6', os.O_RDWR)
    begin = time.monotonic()
    try:
        report['pca_before'] = pca_snapshot(fd)
        for index in range(args.samples):
            target = begin + index * .05
            time.sleep(max(0, target - time.monotonic()))
            item = dict(t=time.monotonic()-begin)
            try:
                select(fd, 0x10)
                if os.write(fd, bytes.fromhex('5a05000160')) != 5:
                    raise OSError('Short TFmini command write')
                time.sleep(args.response_delay_ms / 1000)
                raw = os.read(fd, 9)
                item['hex'] = raw.hex()
                if len(raw) != 9 or raw[:2] != b'\x59\x59' or sum(raw[:8]) & 255 != raw[8]:
                    raise ValueError('Invalid TFmini frame length/header/checksum')
                distance = int.from_bytes(raw[2:4], 'little')
                strength = int.from_bytes(raw[4:6], 'little')
                item.update(distance_cm=distance, strength=strength,
                    temperature_c=int.from_bytes(raw[6:8], 'little')/8-256,
                    valid=10 <= distance <= 1200 and 100 <= strength < 65535)
                if not item['valid']:
                    report['errors'].append('Invalid distance/strength at %d' % index)
                if index % 20 == 0:
                    if pca_snapshot(fd) != report['pca_before']:
                        raise ValueError('PCA registers changed during observation')
            except (OSError, ValueError) as error:
                item.update(valid=False, error=str(error))
                report['errors'].append('%d: %s' % (index, error))
            report['samples'].append(item)
        report['pca_after'] = pca_snapshot(fd)
        distances = [s['distance_cm'] for s in report['samples'] if s.get('valid')]
        times = [s['t'] for s in report['samples']]
        report['summary'] = dict(total=len(times), valid=len(distances),
            hz=(len(times)-1)/(times[-1]-times[0]),
            min_cm=min(distances) if distances else None,
            max_cm=max(distances) if distances else None,
            mean_cm=statistics.mean(distances) if distances else None,
            std_cm=statistics.pstdev(distances) if distances else None,
            max_gap=max(b-a for a,b in zip(times,times[1:])))
        report['passed'] = (len(distances)==args.samples and not report['errors']
                            and report['pca_after']==report['pca_before'])
    except Exception as error:
        report['errors'].append(str(error))
    finally:
        os.close(fd)
        report['elapsed_seconds'] = time.monotonic()-begin
        with open(args.output, 'x') as target:
            json.dump(report, target, indent=2, sort_keys=True)
    print(json.dumps({key:value for key,value in report.items() if key!='samples'}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
