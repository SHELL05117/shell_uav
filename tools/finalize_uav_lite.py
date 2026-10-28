#!/usr/bin/env python3
"""Read-only end-of-test audit; no FCU services, hardware writes or shutdown."""
import argparse
import glob
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

BASELINE = {
    'src/location/src/location_with_ftmini.cpp': '7017f0415f5395c5f15bf90f0f9124e47e5bd8d885e7c1d4bd8b19a3c9e61a2c',
    'src/navigation_2d/src/shoot_with_move_base.cpp': '01eab407cc619db1be18f5689dff54a35123c708528810462d7591d2e1c9577b',
    'devel/lib/location/location_node': '3c883c87df8d528bf0dd00f82fba6e6fa9ff022560ccb42a818833c782a24b12',
    'devel/lib/navigation_2d/shoot_with_move_base_node': 'e1c5e29da9ce687f067867a3c3c12a1eeee83080a0d3d3f0a4da7dd3eb5a5d28',
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    workspace = Path(os.environ.get('UAV_WS',str(Path.home()/'catkin_ws')))
    home = workspace.parent
    installed = home/'.local/share/uav'
    report = dict(files={}, source_and_executables_unchanged={}, services={}, processes=[], device_handles=[])
    for name,expected in BASELINE.items():
        actual = digest(workspace/name)
        report['source_and_executables_unchanged'][name] = dict(sha256=actual, matches_baseline=actual==expected)
    for name in ['uav','uav_env.sh','uav_start.py','uav_log_lite.py','uav_arm.py']:
        a,b = digest(installed/name),digest(workspace/'tools'/name)
        report['files'][name] = dict(installed_sha256=a, tools_sha256=b, equal=a==b)
    service_names = ['shell-npu.target','shell-npu-power.service','shell-npu-firmware.service',
                     'shell-npu-proxy.service','rk-usb-ssh.service']
    for name in service_names:
        report['services'][name] = subprocess.check_output(['systemctl','is-active',name],universal_newlines=True).strip()
    markers = {'roscore','rosmaster','roslaunch','mavros_node','location_node','shoot_with_move_base_node',
               'cartographer_node','cartographer_occupancy_grid_node','rplidarNode','usb_cam_node','record',
               'uav_start.py','uav_log_lite.py','uav_lite_benchmark.py','uav_lite_integration.py'}
    devices = {'/dev/ttyACM0','/dev/ttyUSB0','/dev/video0','/dev/i2c-6'}
    for folder in glob.glob('/proc/[0-9]*'):
        try:
            raw = Path(folder+'/stat').read_text()
            if raw[raw.rfind(')')+2:].split()[0] == 'Z': continue
            argv = Path(folder+'/cmdline').read_bytes().decode(errors='replace').split('\0')
            if markers.intersection(Path(x).name for x in argv if x):
                report['processes'].append(dict(pid=int(Path(folder).name),argv=argv[:-1]))
            for fd in glob.glob(folder+'/fd/*'):
                try:
                    target = os.readlink(fd)
                    if target in devices:
                        report['device_handles'].append(dict(pid=int(Path(folder).name),device=target))
                except OSError: pass
        except (OSError,IndexError): pass
    report['disk_available_gib'] = os.statvfs(str(home)).f_bavail*os.statvfs(str(home)).f_frsize/1024**3
    report['passed'] = all(x['matches_baseline'] for x in report['source_and_executables_unchanged'].values()) \
        and all(x['equal'] for x in report['files'].values()) and not report['processes'] and not report['device_handles'] \
        and all(x=='active' for x in report['services'].values())
    args.output.write_bytes((json.dumps(report,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
