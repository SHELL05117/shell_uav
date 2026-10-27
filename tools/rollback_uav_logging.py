#!/usr/bin/env python3
"""Restore this trial's exact RK backup. Run only after copying trial evidence."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile


def main():
    home=Path.home().resolve(); workspace=home/'catkin_ws'; backup=home/'shell_backups/flight-logging-20261028'
    if str(home)!='/home/jcfh' or not (workspace/'src/location').is_dir():
        raise RuntimeError('Unexpected rollback workspace')
    processes=subprocess.check_output(['ps','-eo','args'],universal_newlines=True).splitlines()
    if any(line.startswith(('roscore ','rosmaster ','/opt/ros/melodic/bin/rosmaster ',
                            'python2 /home/jcfh/.local/share/uav/uav_start.py ',
                            '/home/jcfh/catkin_ws/devel/lib/navigation_2d/shoot_with_move_base_node '))
           for line in processes):
        raise RuntimeError('ROS processes still running; rollback refused')
    report={'source':'USB trial backup','restored':{},'removed':[]}
    def checked(path):
        path=Path(path); resolved=path.resolve()
        if not str(resolved).startswith(str(home)+os.sep): raise RuntimeError('Path outside home: '+str(path))
        return path
    def restore_archive(name,members=None):
        with tarfile.open(str(backup/name),'r:gz') as archive:
            selected=archive.getmembers() if members is None else [archive.getmember(n) for n in members]
            for member in selected:
                checked(home/member.name)
                if member.issym() or member.islnk(): raise RuntimeError('Unexpected link in restore archive')
            for member in selected:
                archive.extract(member,str(home))
                if member.isfile(): report['restored'][member.name]=hashlib.sha256((home/member.name).read_bytes()).hexdigest()
    digest=hashlib.sha256((backup/'before.tar.gz').read_bytes()).hexdigest()
    if digest!='f6a15d8cd452a6f63e8f965cbf7f064dde7a2ea1993bdcb2475ec52d27bfce6a':
        raise RuntimeError('Backup hash mismatch')
    restore_archive('before.tar.gz'); restore_archive('monitor-before.tar.gz')
    shutil.copy2(str(home/'.local/share/uav/uav'),str(workspace/'tools/uav'))
    report['restored']['catkin_ws/tools/uav']=hashlib.sha256((workspace/'tools/uav').read_bytes()).hexdigest()
    new_files=['src/navigation_2d/include/audit_json.h']
    for name in ['uav_log','uav_perf','uav_log_report','uav_logging_benchmark','uav_logging_fault_probe',
                 'test_uav_logging','dwa_mission_probe']:
        new_files+=['tools/'+name+'.py','tools/'+name+'.pyc']
    new_files+=['tools/deployment-flight-logging.json','tools/run_uav_logging_verification.sh']
    for relative in new_files:
        target=checked(workspace/relative)
        if target.is_file(): target.unlink(); report['removed'].append(str(target))
    for name in ['uav_log.py','uav_log.pyc','uav_perf.py','uav_perf.pyc','uav_log_report.py','uav_log_report.pyc']:
        target=checked(home/'.local/share/uav'/name)
        if target.is_file(): target.unlink(); report['removed'].append(str(target))
    for folder in [workspace/'tools/__pycache__',home/'.local/share/uav/__pycache__']:
        if folder.is_dir():
            for prefix in ['uav_log.','uav_perf.','uav_log_report.','uav_logging_','test_uav_logging.']:
                for target in folder.glob(prefix+'*.pyc'):
                    checked(target).unlink(); report['removed'].append(str(target))
            if not any(folder.iterdir()): folder.rmdir()
    # The read-only NSH helper was temporarily extended; use its saved baseline.
    original_helper=backup/'rk_px4_serial_shell-original.py'
    if original_helper.is_file():
        shutil.copyfile(str(original_helper),str(workspace/'tools/rk_px4_serial_shell.py'))
        cached=workspace/'tools/rk_px4_serial_shell.pyc'
        if cached.is_file(): cached.unlink()
    # Old mtimes must not let make reuse objects compiled from trial sources.
    for name in ['src/navigation_2d/src/detect.cpp','src/navigation_2d/src/shoot_with_move_base.cpp',
                 'src/location/src/location_with_ftmini.cpp']:
        os.utime(str(workspace/name),None)
    build=home/'logging-benchmark/rollback-build.log'
    with build.open('w') as output:
        subprocess.check_call(['bash','-c','source /opt/ros/melodic/setup.bash; catkin_make --pkg location navigation_2d -j2 -l3'],
                              cwd=str(workspace),stdout=output,stderr=subprocess.STDOUT)
    # Main flight/location executables additionally return to their exact bytes.
    restore_archive('before.tar.gz',['catkin_ws/devel/lib/navigation_2d/shoot_with_move_base_node',
                                     'catkin_ws/devel/lib/location/location_node'])
    for archive_name in ['before.tar.gz','monitor-before.tar.gz']:
        with tarfile.open(str(backup/archive_name),'r:gz') as archive:
            for member in archive.getmembers():
                if member.isfile():
                    expected=hashlib.sha256(archive.extractfile(member).read()).hexdigest()
                    actual=hashlib.sha256((home/member.name).read_bytes()).hexdigest()
                    if actual!=expected: raise RuntimeError('Restore mismatch '+member.name)
    report['backup_files_match']=True
    # Clear only new bytecode of restored launchers; Python can regenerate it.
    for folder in [workspace/'tools',home/'.local/share/uav']:
        for name in ['uav_start.pyc','uav_env.pyc']:
            target=checked(folder/name)
            # Do not remove a bytecode file that was present in the original archive.
            with tarfile.open(str(backup/'before.tar.gz'),'r:gz') as archive:
                existed=str(target.relative_to(home)) in archive.getnames()
            if target.is_file() and not existed: target.unlink()
    report['function_files_absent']=all(not (workspace/path).exists() for path in new_files)
    destination=home/'logging-benchmark/rollback.json'
    destination.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    print(json.dumps(report,indent=2,sort_keys=True))


if __name__=='__main__': main()
