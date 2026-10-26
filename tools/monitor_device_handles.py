#!/usr/bin/env python2
"""Read /proc only; never open a sensor. Use sudo for complete FD visibility."""
from __future__ import print_function
import errno
import json
import os
import sys


def inspect(paths):
    missing = [path for path in paths if not os.path.exists(path)]
    targets = {os.path.realpath(path): path for path in paths}
    busy, errors = [], []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        directory = '/proc/'+pid+'/fd'
        try:
            entries = os.listdir(directory)
            for fd in entries:
                try:
                    target = os.readlink(directory+'/'+fd)
                    if target in targets:
                        busy.append(dict(pid=int(pid), fd=fd, device=targets[target]))
                except OSError as error:
                    if error.errno not in (errno.ENOENT, errno.ESRCH):
                        errors.append(dict(pid=int(pid), errno=error.errno))
        except OSError as error:
            if error.errno not in (errno.ENOENT, errno.ESRCH):
                errors.append(dict(pid=int(pid), errno=error.errno))
    return dict(missing=missing, busy=busy, errors=errors,
                passed=not (missing or busy or errors), opens_hardware=False)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit('Usage: monitor_device_handles.py PATH [PATH ...]')
    result = inspect(sys.argv[1:])
    print(json.dumps(result, sort_keys=True))
    raise SystemExit(0 if result['passed'] else 2)
