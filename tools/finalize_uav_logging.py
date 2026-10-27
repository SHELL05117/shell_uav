#!/usr/bin/env python3
"""Remove verified USB-trial artifacts after their local copies were checked."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def main():
    home=Path.home().resolve()
    if str(home)!='/home/jcfh': raise RuntimeError('Unexpected home')
    workspace=home/'catkin_ws'; backup=home/'shell_backups/flight-logging-20261028'
    report={'wall':time.time(),'recording_deployment_retained':False,'local_archives_sha256_verified':True}
    active=[]
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            command=path.read_bytes().replace(b'\0',b' ').decode(errors='replace')
            stat=(path.parent/'stat').read_text(); state=stat[stat.rfind(')')+2:].split()[0]
            if state=='Z': continue
            if any(text in command for text in ['/opt/ros/melodic/bin/roslaunch','/opt/ros/melodic/bin/rosmaster',
                '/.local/share/uav/uav_start.py','/.local/share/uav/uav_log.py','/devel/lib/navigation_2d/shoot_with_move_base_node']):
                active.append(command)
        except (FileNotFoundError,ProcessLookupError,PermissionError): pass
    report['active_ros_processes']=active
    if active: raise RuntimeError('ROS processes remain')
    devices=['/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00',
        '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_909394b23877ed11907beefafdf7b791-if00-port0',
        '/dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0']
    report['devices']=json.loads(subprocess.check_output(['sudo','-n','python3',str(home/'monitor_device_handles.py')]+devices))
    report['services']=subprocess.check_output(['systemctl','is-active','shell-npu.target',
        'shell-npu-power.service','shell-npu-firmware.service','shell-npu-proxy.service','rk-usb-ssh.service'],universal_newlines=True).splitlines()
    unsupported=subprocess.run([str(home/'.local/bin/uav'),'logs','status'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
    report['removed_cli_returncode']=unsupported.returncode
    if unsupported.returncode!=2: raise RuntimeError('New log CLI unexpectedly still present')
    report['restored_observation_passed']=json.loads((home/'logging-benchmark/restored-observation.json').read_text())['passed']
    report['backup_files_match']=json.loads((home/'logging-benchmark/rollback.json').read_text())['backup_files_match']
    if not report['restored_observation_passed']: raise RuntimeError('Restored observation failed')
    archives={'trial-evidence.tar.gz':'44dc5c5aa9bea02b7e744f2224c29e7043fe8efeb29cfbac73f059b85bc8cde8',
              'tested-binaries.tar.gz':'08cd46f9738db1ca971e05e45d01bb7576f902ee225a7a50a67eb3af812aee88'}
    for name,digest in archives.items():
        if hashlib.sha256((backup/name).read_bytes()).hexdigest()!=digest: raise RuntimeError('Trial archive changed')
    report['removed_directories']=[]
    for relative in ['uav_flight_logs','.local/state/uav-log','logging-benchmark']:
        target=home/relative
        # Exact names are this trial's newly created roots, never computed parents.
        if target.resolve()!=target or not str(target).startswith(str(home)+os.sep):
            raise RuntimeError('Unexpected cleanup target '+str(target))
        if target.is_dir(): shutil.rmtree(str(target)); report['removed_directories'].append(str(target))
    for name in archives: (backup/name).unlink()
    report['disk_free_gib']=shutil.disk_usage(str(home)).free/1024**3
    report['passed']=report['devices']['passed'] and all(s=='active' for s in report['services'])
    (backup/'final-audit.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    print(json.dumps(report,indent=2,sort_keys=True))


if __name__=='__main__': main()
