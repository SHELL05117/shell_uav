#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Onboard ROS1 evidence recorder. Never sends commands, goals or FCU services.

Detached writer, bounded rosbag buffers, media is expendable, no automatic deletion.
Recording failure is reported independently and never kills the flight stack.
"""
from __future__ import print_function
import argparse
import errno
import fcntl
import glob
import hashlib
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from uav_perf import Sampler, steady, read

CORE_TOPICS = [
    '/tf','/tf_static','/scan','/map','/rosout_agg','/diagnostics',
    '/mavros/state','/mavros/extended_state','/mavros/battery','/mavros/rc/in',
    '/mavros/imu/data','/mavros/local_position/pose','/mavros/local_position/odom',
    '/mavros/local_position/velocity_local','/mavros/vision_pose/pose',
    '/mavros/setpoint_raw/local','/mavros/setpoint_raw/target_local',
    '/mavros/statustext/recv','/mavros/time_reference','/mavros/timesync_status',
    '/cmd_vel','/move_base/goal','/move_base/status','/move_base/result',
    '/move_base/current_goal','/move_base/DWAPlannerROS/global_plan',
    '/move_base/DWAPlannerROS/local_plan','/gpio_cmd','/gpio_ack','/servo_ready',
    '/mission/status','/mission/readiness','/mission/audit',
    '/mission/inference','/location_node/status','/location_node/range_audit',
    '/uav_log/health']
MEDIA_TOPICS = ['/monitor/camera/log/compressed','/mission/inference_frame/compressed']
STATE_DIR = os.path.expanduser('~/.local/state/uav-log')
DEFAULT_ROOT = os.path.expanduser('~/uav_flight_logs')
ACTIVE = os.path.join(STATE_DIR,'active.json')


def atomic_json(path, value):
    temp = path+'.tmp'
    with open(temp,'w') as stream:
        json.dump(value,stream,sort_keys=True,indent=2,allow_nan=False)
        stream.write('\n'); stream.flush()
    os.rename(temp,path)


def identity(pid):
    try:
        raw=read('/proc/%d/stat' % pid); tail=raw[raw.rfind(')')+2:].split()
        if tail[0]=='Z': return None
        return dict(pid=int(pid),start=tail[19],boot=read('/proc/sys/kernel/random/boot_id').strip())
    except (IOError,ValueError,IndexError):
        return None


def active():
    try:
        value=json.loads(read(ACTIVE))
        return value if identity(value['process']['pid'])==value['process'] else None
    except (IOError,ValueError,KeyError):
        return None


def free_bytes(path):
    fs=os.statvfs(path); return fs.f_bavail*fs.f_frsize


def prepare(mode):
    """Called before roslaunch: captures ROS textual logs even during startup."""
    root=os.path.abspath(os.path.expanduser(os.environ.get('UAV_LOG_ROOT',DEFAULT_ROOT)))
    if not os.path.isdir(root): os.makedirs(root)
    if free_bytes(root)<3*1024**3:
        raise RuntimeError('Less than 3 GiB free; logging unavailable (no logs deleted)')
    directory=os.path.join(root,time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    os.mkdir(directory); os.mkdir(directory+'/ros-logs')
    atomic_json(directory+'/session.json',dict(schema=1,session=os.path.basename(directory),
        mode=mode,wall_start=time.time(),steady_start=steady(),
        boot=read('/proc/sys/kernel/random/boot_id').strip(),
        ros_master=os.environ.get('ROS_MASTER_URI'),hostname=socket.gethostname(),
        core_topics=CORE_TOPICS,media_topics=MEDIA_TOPICS if os.environ.get('UAV_LOG_PROFILE','full')=='full' else [],
        profile=os.environ.get('UAV_LOG_PROFILE','full'),
        context_hz=2 if os.environ.get('UAV_LOG_PROFILE','full')=='full' else 0,source_timestamps_preserved=True,
        safety='Recorder has no actuation publishers or FCU service calls',
        px4_ulog='Independent FCU SD logging must be verified; not copied by this recorder'))
    return directory


def launch(directory):
    if not os.path.isdir(STATE_DIR): os.makedirs(STATE_DIR)
    with open(directory+'/recorder.stdout.log','ab') as stream:
        process=subprocess.Popen([sys.executable,os.path.abspath(__file__),'daemon',directory],
            stdin=open(os.devnull,'rb'),stdout=stream,stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,close_fds=True)
    # Startup is bounded and occurs before mission actuation. Failure never stops A.
    deadline=steady()+7
    while steady()<deadline and process.poll() is None:
        if os.path.exists(directory+'/ready.json'): return directory
        time.sleep(.1)
    raise RuntimeError('Recorder not ready; inspect '+directory+'/recorder.stdout.log')


def attach(mode, arguments):
    value=active()
    if not value: return None
    directory=value['directory']
    # Independent files avoid append interleaving between A and C terminals.
    atomic_json(directory+'/command-'+uuid.uuid4().hex[:8]+'.json',dict(
        wall=time.time(),steady=steady(),mode=mode,arguments=arguments))
    if mode in ('mission','c','check'): open(directory+'/snapshot-request','a').close()
    return directory


class Recorder(object):
    def __init__(self,directory):
        self.directory=os.path.abspath(directory); self.stopping=False
        self.children=[]; self.streams=[]; self.errors=[]; self.media=None
        self.ready=False; self.state=None; self.armed=False; self.sortie=0
        self.context_last=0.0; self.params_running=None; self.params_counter=0
        self.param_last=0; self.param_pending=True; self.last_master_ok=steady()
        self.start_time=steady(); self.health='STARTING'; self.rejected_context=0
        self.param_due=self.start_time+8
        self.sources_stopped=False

    def event(self,kind,**fields):
        fields.update(kind=kind,wall=time.time(),steady=steady())
        self.events.write(json.dumps(fields,sort_keys=True)+'\n'); self.events.flush()

    def spawn(self,args,name):
        stream=open(self.directory+'/'+name+'.stdout.log','ab'); self.streams.append(stream)
        proc=subprocess.Popen(args,stdin=open(os.devnull,'rb'),stdout=stream,stderr=subprocess.STDOUT,
                              preexec_fn=os.setsid,close_fds=True)
        self.children.append(proc); return proc

    def stop_child(self,proc):
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            limit=steady()+8
            while proc.poll() is None and steady()<limit: time.sleep(.05)
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM); self.errors.append('writer_required_SIGTERM')
                limit=steady()+2
                while proc.poll() is None and steady()<limit: time.sleep(.05)
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGKILL); self.errors.append('writer_required_SIGKILL')
        if proc: proc.wait()

    def state_callback(self,message):
        # ROS callbacks enqueue facts only. File IO belongs to the main loop.
        self.state=(steady(),message)

    def metadata(self):
        workspace=os.environ.get('UAV_WS',os.path.expanduser('~/catkin_ws'))
        paths=['src/navigation_2d/detect_mode/26/best.rknn',
               'src/navigation_2d/detect_mode/26/labels.txt',
               'src/navigation_2d/src/shoot_with_move_base.cpp','src/navigation_2d/src/detect.cpp',
               'src/location/src/location_with_ftmini.cpp',
               'src/location/scripts/monitor_jpeg.py',
               'src/location/param/rk_height_reference_20261027.yaml',
               'tools/uav_log.py','tools/uav_start.py','tools/deployment-flight-logging.json']
        hashes={}
        for name in paths:
            path=os.path.join(workspace,name)
            if os.path.isfile(path):
                with open(path,'rb') as stream: hashes[name]=hashlib.sha256(stream.read()).hexdigest()
        atomic_json(self.directory+'/versions.json',dict(files=hashes,uname=list(os.uname())))
        deployment=os.path.join(workspace,'tools/deployment-flight-logging.json')
        if os.path.isfile(deployment):
            with open(self.directory+'/deployment.json','w') as stream: stream.write(read(deployment))

    def snapshot(self):
        if self.params_running:
            if self.params_running.poll() is None: return
            self.event('parameter_snapshot',returncode=self.params_running.returncode)
            self.params_running=None
        request=self.directory+'/snapshot-request'
        if os.path.exists(request):
            os.unlink(request); self.param_pending=True; self.param_due=steady()+5
        # Initial snapshot after sources initialize; C requests another snapshot.
        if self.param_pending and steady()>=self.param_due:
            self.params_counter+=1; self.param_pending=False
            self.params_running=self.spawn(['timeout','12','rosparam','dump',
                self.directory+'/parameters-%02d.yaml'%self.params_counter],'parameters-%02d'%self.params_counter)

    def run(self):
        import rospy
        import rosgraph
        from sensor_msgs.msg import CompressedImage
        from mavros_msgs.msg import State
        from std_msgs.msg import String
        self.rospy=rospy
        os.nice(10)
        self.lock=open(STATE_DIR+'/lock','a')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.events=open(self.directory+'/events.jsonl','a')
        self.perf=open(self.directory+'/performance.jsonl','a')
        atomic_json(ACTIVE,dict(directory=self.directory,process=identity(os.getpid())))
        self.event('recorder_start')
        rospy.init_node('uav_log',disable_signals=True)
        self.health_pub=rospy.Publisher('/uav_log/health',String,queue_size=1,latch=True)
        rospy.Subscriber('/mavros/state',State,self.state_callback,queue_size=1)
        # Explicit whitelist, bounded buffers, no expensive bag compression on RK.
        self.core=self.spawn(['rosbag','record','--split','--duration=60','--buffsize=32',
            '--chunksize=256','-O',self.directory+'/core']+
            CORE_TOPICS+['__name:=uav_core_recorder'],'core')
        if os.environ.get('UAV_LOG_PROFILE','full')=='full':
            self.media=self.spawn(['rosbag','record','--split','--duration=60','--buffsize=8',
                '--chunksize=256','-O',self.directory+'/media']+
                MEDIA_TOPICS+['__name:=uav_media_recorder'],'media')
        self.metadata(); sampler=Sampler(); master=rosgraph.Master('/uav_log')
        next_sample=steady(); last_graph=0; graph_seen=False
        while not self.stopping and not rospy.is_shutdown():
            now=steady()
            if now>=next_sample:
                sample=sampler.sample(); self.perf.write(json.dumps(sample,sort_keys=True)+'\n'); self.perf.flush()
                next_sample=now+1
                if self.state:
                    received,state=self.state
                    if received>now-3 and state.armed!=self.armed:
                        self.armed=state.armed
                        if self.armed: self.sortie+=1
                        self.event('armed' if self.armed else 'disarmed',sortie=self.sortie)
                if self.core.poll() is not None:
                    self.errors.append('core_recorder_exit_%s'%self.core.returncode)
                    self.event('recording_failure',error=self.errors[-1]); break
                available=free_bytes(self.directory)
                if self.media and (available<2*1024**3 or self.media.poll() is not None):
                    self.event('media_stopped',free_bytes=available,returncode=self.media.poll())
                    self.stop_child(self.media); self.media=None
                if available<1100*1024**2:
                    self.errors.append('disk_reserve_reached'); self.event('recording_failure',error=self.errors[-1]); break
                if glob.glob(self.directory+'/core*.bag.active') and not self.ready:
                    self.ready=True; atomic_json(self.directory+'/ready.json',dict(wall=time.time()))
                    self.event('recording_ready')
                self.health='RECORDING' if self.media else 'RECORDING_CORE_ONLY'
                self.health_pub.publish(String(data=json.dumps(dict(status=self.health,
                    session=os.path.basename(self.directory),free_bytes=available,
                    sortie=self.sortie,context_rejected=self.rejected_context))))
                self.snapshot()
            if now-last_graph>3:
                last_graph=now
                try:
                    graph=master.getSystemState(); self.last_master_ok=now
                    names=set(n for group in graph for topic,nodes in group for n in nodes)
                    sources=any(n in names for n in ('/mavros','/location_node','/shoot_with_move_base_node','/camera'))
                    if sources: graph_seen=True; self.last_sources=now
                    if graph_seen and not sources and now-self.last_sources>10:
                        self.sources_stopped=True; self.event('sources_stopped'); break
                except (socket.error,Exception) as error:
                    if now-self.last_master_ok>10:
                        self.sources_stopped=True; self.event('master_stopped',error=str(error)); break
            time.sleep(.1)
        self.finish()

    def finish(self):
        if hasattr(self,'health_pub'):
            self.health_pub.publish(json.dumps(dict(status='STOPPING',errors=self.errors)))
        # media first, core last so the core captures shutdown status.
        for child in reversed(self.children): self.stop_child(child)
        for stream in self.streams: stream.close()
        self.event('recorder_stop',errors=self.errors)
        atomic_json(self.directory+'/closed.json',dict(wall=time.time(),steady=steady(),
            errors=self.errors,sorties=self.sortie,complete=not self.errors and not glob.glob(self.directory+'/*.active'),
            active_bags=glob.glob(self.directory+'/*.active')))
        self.events.close(); self.perf.close()
        value=active()
        if value and value['process']==identity(os.getpid()): os.unlink(ACTIVE)
        # Reporting scans bags and belongs after the sources have stopped, with
        # the last observed FCU state disarmed. Explicit report is also available.
        if self.sources_stopped and self.state and not self.armed:
            with open(self.directory+'/report.stdout.log','ab') as stream:
                subprocess.Popen([sys.executable,os.path.join(os.path.dirname(__file__),'uav_log_report.py'),
                    self.directory],stdout=stream,stderr=subprocess.STDOUT,close_fds=True,preexec_fn=os.setsid)


def stop_active():
    value=active()
    if not value: print('No active recorder'); return
    # Identity includes kernel starttime and boot ID: never signal a reused PID.
    if identity(value['process']['pid'])!=value['process']: raise RuntimeError('Recorder identity changed')
    os.kill(value['process']['pid'],signal.SIGINT)
    deadline=steady()+24
    while active() and steady()<deadline: time.sleep(.1)
    if active(): raise RuntimeError('Recorder still closing; do not kill flight processes')
    print('Saved '+value['directory'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['start','stop','status','daemon','report'])
    parser.add_argument('directory',nargs='?')
    args=parser.parse_args()
    if args.command=='status':
        value=active(); print(json.dumps(value,indent=2) if value else 'No active recorder')
    elif args.command=='stop': stop_active()
    elif args.command=='start':
        if active(): raise RuntimeError('Recorder already active')
        import rosgraph
        rosgraph.Master('/uav_log_start_check').getPid()
        directory=prepare('manual'); launch(directory); print('Recording '+directory)
    elif args.command=='report':
        from uav_log_report import generate
        if not args.directory: raise RuntimeError('directory required')
        print(json.dumps(generate(args.directory),indent=2))
    else:
        if not args.directory: raise RuntimeError('directory required')
        recorder=Recorder(args.directory)
        signal.signal(signal.SIGINT,lambda *_: setattr(recorder,'stopping',True))
        signal.signal(signal.SIGTERM,lambda *_: setattr(recorder,'stopping',True))
        try: recorder.run()
        except Exception:
            if hasattr(recorder,'events'):
                recorder.errors.append('recorder_exception')
                recorder.finish()
            raise


if __name__=='__main__':
    main()
