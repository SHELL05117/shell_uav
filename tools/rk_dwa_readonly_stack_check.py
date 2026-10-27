#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded real-hardware checks; never starts a flight or commands a servo.

Source the RK catkin workspace, place rk_fourclass_camera_check.py beside this
file, then run: python2 rk_dwa_readonly_stack_check.py OUTPUT_DIR
Uses ROS/Melodic system packages and cv2/numpy already used by the camera check.
Default observation sends no vision. --i2c-range reads the production TFmini
with height_reference_calibrated=false. --fuse-vision explicitly measures a real
relative zero and forwards vision to the unarmed FCU. --installed-preflight
checks the deployed measured profile. Fusion changes estimator inputs; it is
not a receive-only mode. No service calls, FCU parameter writes, synthetic
sensor messages, goals, setpoints or servo commands are used in any mode.
"""
from __future__ import print_function
import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET

from rk_fourclass_camera_check import Children, Images, steady, text_file, log_evidence


def proc_table():
    result = {}
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            raw = text_file('/proc/'+name+'/stat')
            tail = raw[raw.rfind(')')+2:].split()
            status = text_file('/proc/'+name+'/status')
            rss = [line.split()[1] for line in status.splitlines() if line.startswith('VmRSS:')]
            result[int(name)] = dict(ppid=int(tail[1]), pgid=int(tail[2]),
                start=tail[19], ticks=int(tail[11])+int(tail[12]),
                rss_kib=int(rss[0]) if rss else 0,
                command=text_file('/proc/'+name+'/cmdline').replace('\0',' ').strip())
        except (IOError, OSError, ValueError, IndexError):
            pass  # A process may exit between proc reads.
    return result


class Resources(object):
    def __init__(self, children):
        self.children = children
        self.owned = {}
        self.peak_rss_kib = 0
        self.minimum_available_kib = None
        self.peak_cpu_percent = 0.0
        self.previous = None
        self.clock_ticks = os.sysconf('SC_CLK_TCK')
        self.dwa_library_pids = set()

    def sample(self):
        table = proc_table()
        roots = set(c['process'].pid for c in self.children.children)
        selected = set(roots)
        changed = True
        while changed:
            previous_size = len(selected)
            selected.update(pid for pid,item in table.items() if item['ppid'] in selected)
            changed = len(selected) != previous_size
        for pid in selected:
            if pid in table:
                item = table[pid]
                key = (pid,item['start'])
                record = self.owned.setdefault(key,dict(pid=pid,start=item['start'],
                    pgid=item['pgid'],command=item['command'],peak_rss_kib=0))
                record['peak_rss_kib'] = max(record['peak_rss_kib'],item['rss_kib'])
                if 'move_base' in item['command']:
                    try:
                        if 'libdwa_local_planner.so' in text_file('/proc/%d/maps'%pid):
                            self.dwa_library_pids.add(pid)
                    except IOError:
                        pass
        # Include observer itself, whose image deserialization also costs memory.
        selected.add(os.getpid())
        samples = {pid:table[pid] for pid in selected if pid in table}
        now = steady()
        rss = sum(item['rss_kib'] for item in samples.values())
        self.peak_rss_kib = max(self.peak_rss_kib,rss)
        if self.previous is not None:
            before, prior = self.previous
            ticks = sum(max(0,item['ticks']-prior[pid]['ticks']) for pid,item in samples.items()
                        if pid in prior and item['start']==prior[pid]['start'])
            if now > before:
                self.peak_cpu_percent = max(self.peak_cpu_percent,
                    ticks*100.0/self.clock_ticks/(now-before))
        self.previous = (now,samples)
        for line in text_file('/proc/meminfo').splitlines():
            if line.startswith('MemAvailable:'):
                available = int(line.split()[1])
                if self.minimum_available_kib is None or available < self.minimum_available_kib:
                    self.minimum_available_kib = available

    def remaining(self):
        table = proc_table()
        return [dict(record,pgid=table[pid]['pgid']) for (pid,start),record in self.owned.items()
                if pid in table and table[pid]['start']==start and table[pid]['rss_kib']>0]

    def finish_owned_groups(self):
        """Handle a roslaunch descendant that outlived its launcher, by identity.

        A process group is signalled only when every current member is a PID +
        kernel start-time identity previously observed beneath our launchers.
        """
        signals = []
        for signum,seconds in ((signal.SIGINT,5),(signal.SIGTERM,2),(signal.SIGKILL,2)):
            table = proc_table()
            live = [item for item in self.remaining()]
            groups = set(item['pgid'] for item in live)
            if not groups:
                break
            for group in groups:
                members = [(pid,item) for pid,item in table.items() if item['pgid']==group]
                if group==os.getpgrp() or not members or any(
                        (pid,item['start']) not in self.owned for pid,item in members):
                    continue
                try:
                    os.killpg(group,signum)
                    signals.append(dict(pgid=group,signal=signum))
                except OSError:
                    pass  # All verified members may have exited together.
            deadline = steady()+seconds
            while self.remaining() and steady()<deadline:
                time.sleep(.05)
        return signals

    def report(self):
        return dict(peak_aggregate_rss_kib=self.peak_rss_kib,
            minimum_system_mem_available_kib=self.minimum_available_kib,
            peak_sampled_cpu_percent_one_core_100=self.peak_cpu_percent,
            dwa_library_pids=sorted(self.dwa_library_pids),
            processes=sorted(self.owned.values(),key=lambda value:value['pid']))


class Telemetry(object):
    def __init__(self, rospy, allow_vision=False):
        from sensor_msgs.msg import LaserScan, Imu
        from mavros_msgs.msg import State, EstimatorStatus, RCIn
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from nav_msgs.msg import OccupancyGrid
        self.rospy = rospy
        self.lock = threading.Lock()
        self.begin = None
        self.times = dict(scan=[],imu=[],state=[],readiness=[],vision=[],pose=[])
        self.imu_orientation = None
        self.positions = dict(vision=[],pose=[])
        self.estimators = []
        self.rc = []
        self.states = []
        self.strings = dict(mission=[],readiness=[],location=[])
        self.forbidden = {}
        self.forbidden_subs = {}
        self.violation = None
        self.map = dict(received=False)
        self.subscribers = [
            rospy.Subscriber('/scan',LaserScan,self.sensor,callback_args='scan',queue_size=5),
            rospy.Subscriber('/mavros/imu/data',Imu,self.sensor,callback_args='imu',queue_size=10),
            rospy.Subscriber('/mavros/state',State,self.state,queue_size=10),
            rospy.Subscriber('/mission/status',String,self.string,callback_args='mission',queue_size=20),
            rospy.Subscriber('/mission/readiness',String,self.string,callback_args='readiness',queue_size=20),
            rospy.Subscriber('/location/status',String,self.string,callback_args='location',queue_size=20),
            rospy.Subscriber('/map',OccupancyGrid,self.mapping,queue_size=1),
            rospy.Subscriber('/mavros/vision_pose/pose',PoseStamped,self.position,callback_args='vision',queue_size=5),
            rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.position,callback_args='pose',queue_size=5),
            rospy.Subscriber('/mavros/estimator_status',EstimatorStatus,self.estimator,queue_size=5),
            rospy.Subscriber('/mavros/rc/in',RCIn,self.receiver,queue_size=5)]
        for topic in ['/gpio_cmd','/gpio_ack','/gpio_debug_cmd',
                '/mavros/setpoint_raw/local','/mavros/setpoint_position/local',
                '/mavros/setpoint_velocity/cmd_vel','/mavros/vision_pose/pose']:
            if not (allow_vision and topic=='/mavros/vision_pose/pose'):
                self.watch(topic)

    def watch(self, topic):
        if topic not in self.forbidden_subs:
            self.forbidden[topic] = 0
            self.forbidden_subs[topic] = self.rospy.Subscriber(topic,self.rospy.AnyMsg,
                self.forbidden_message,callback_args=topic,queue_size=20)

    def discover(self):
        for topic,unused_type in self.rospy.get_published_topics('/'):
            if topic.startswith('/mavros/setpoint') or topic.startswith('/gpio'):
                self.watch(topic)

    def forbidden_message(self, message, topic):
        with self.lock:
            self.forbidden[topic] += 1
            self.violation = 'Unexpected message on '+topic

    def sensor(self, message, name):
        with self.lock:
            self.times[name].append(steady())
            if name=='imu':
                q=message.orientation
                self.imu_orientation=(q.x,q.y,q.z,q.w,steady())

    def position(self, message, name):
        with self.lock:
            now=steady(); self.times[name].append(now)
            p=message.pose.position
            self.positions[name].append(dict(time=now,x=p.x,y=p.y,z=p.z,
                stamp=message.header.stamp.to_sec(),frame=message.header.frame_id))

    def estimator(self, message):
        with self.lock:
            self.estimators.append(dict(time=steady(),attitude=message.attitude_status_flag,
                local_xy=message.pos_horiz_rel_status_flag,vertical=message.pos_vert_abs_status_flag,
                const_position=message.const_pos_mode_status_flag))

    def receiver(self, message):
        with self.lock:
            self.rc.append(dict(time=steady(),rssi=message.rssi,channels=list(message.channels)))

    def state(self, message):
        with self.lock:
            now = steady()
            self.times['state'].append(now)
            self.states.append(dict(time=now,connected=bool(message.connected),
                armed=bool(message.armed),mode=message.mode))
            if message.armed:
                self.violation = 'FCU reported armed; stopping observation'

    def string(self, message, name):
        with self.lock:
            now = steady()
            value = message.data
            if name=='readiness':
                try:
                    value = json.loads(value)
                except ValueError:
                    self.violation = 'Readiness is not JSON'
                self.times[name].append(now)
                if isinstance(value,dict) and value.get('actuation_enabled') is not False:
                    self.violation = 'Mission reported actuation enabled or missing gate'
            self.strings[name].append(dict(time=now,value=value))

    def mapping(self, message):
        values = message.data
        with self.lock:
            self.map = dict(received=True,width=message.info.width,height=message.info.height,
                resolution=message.info.resolution,frame=message.header.frame_id,
                known_cells=sum(value>=0 for value in values),
                free_cells=sum(value==0 for value in values),
                occupied_cells=sum(value>50 for value in values))

    def check(self):
        with self.lock:
            if self.violation:
                raise RuntimeError(self.violation)

    def snapshot(self, began, ended):
        with self.lock:
            streams = {}
            for name,values in self.times.items():
                values = [value for value in values if began<=value<=ended]
                gaps = [b-a for a,b in zip(values,values[1:])]
                streams[name] = dict(count=len(values),
                    hz=(len(values)-1)/(values[-1]-values[0]) if len(values)>1 else 0.0,
                    max_gap=max(gaps) if gaps else None,
                    first_delay=values[0]-began if values else None,
                    last_age=ended-values[-1] if values else None)
            states = [state for state in self.states if began<=state['time']<=ended]
            strings = {name:[record for record in values if began<=record['time']<=ended]
                       for name,values in self.strings.items()}
            # Location status is latched and emitted only when changed.
            if not strings['location'] and self.strings['location']:
                strings['location'] = [self.strings['location'][-1]]
            return dict(streams=streams,states=states,status=strings,
                positions={name:[r for r in values if began<=r['time']<=ended]
                           for name,values in self.positions.items()},
                estimators=[r for r in self.estimators if began<=r['time']<=ended],
                rc=[r for r in self.rc if began<=r['time']<=ended],
                zero_expected_topics=dict(self.forbidden),map=dict(self.map),violation=self.violation)


def readiness_health(records, began, ended):
    """Observe health continuity, not inference FPS or distinct result count.

    A true readiness sample means Detect's latest result is within its source
    and completion deadlines. Both absent messages and sustained false states
    are bounded, so two isolated true samples cannot pass a twenty-second run.
    """
    samples = sorted((item for item in records if began<=item['time']<=ended),
                     key=lambda item:item['time'])
    true_count = 0
    false_started = None
    maximum_false = 0.0
    for item in samples:
        value = item['value']
        good = isinstance(value,dict) and value.get('inference_ready') is True
        if good:
            true_count += 1
            if false_started is not None:
                maximum_false = max(maximum_false,item['time']-false_started)
                false_started = None
        elif false_started is None:
            false_started = item['time']
    if false_started is not None:
        maximum_false = max(maximum_false,ended-false_started)
    first_gap = samples[0]['time']-began if samples else ended-began
    last_gap = ended-samples[-1]['time'] if samples else ended-began
    gaps = [b['time']-a['time'] for a,b in zip(samples,samples[1:])]
    maximum_message_gap = max(gaps) if gaps else ended-began
    maximum_bad_gap = max(maximum_false,maximum_message_gap,first_gap,last_gap)
    ratio = float(true_count)/len(samples) if samples else 0.0
    return dict(sample_count=len(samples),inference_ready_true_count=true_count,
        inference_ready_true_ratio=ratio,max_continuous_false_seconds=maximum_false,
        max_message_gap_seconds=maximum_message_gap,first_sample_delay_seconds=first_gap,
        last_sample_age_seconds=last_gap,max_bad_gap_seconds=maximum_bad_gap,
        minimum_samples=10,minimum_true_ratio=.95,maximum_bad_gap_limit_seconds=1.0,
        is_inference_fps=False,
        passed=len(samples)>=10 and ratio>=.95 and maximum_bad_gap<=1.0)


def measured_ground_reference(telemetry):
    """Real static range/IMU zero, accepting isolated corrupt frames (never reused).

    SHELL confirmed vertical mounting and permits centimetre-level geometry
    approximations. Zero sensor lever-arm is an explicit approximation here.
    This datum belongs to the current support height; re-zero after moving it.
    """
    import fcntl
    deadline=steady()+20
    while steady()<deadline:
        telemetry.check()
        with telemetry.lock:
            orientation=telemetry.imu_orientation
            connected=bool(telemetry.states) and telemetry.states[-1]['connected']
        if connected and orientation and steady()-orientation[4]<.3:
            break
        time.sleep(.05)
    else:
        raise RuntimeError('Ground zero needs connected disarmed FCU and fresh IMU')
    fd=os.open('/dev/i2c-6',os.O_RDWR); fcntl.ioctl(fd,0x0703,0x10)
    values=[]; errors=0; began=steady()
    try:
        for index in range(80):
            time.sleep(max(0,began+index*.05-steady()))
            telemetry.check()
            try:
                if os.write(fd,b'\x5a\x05\x00\x01\x60')!=5:
                    raise ValueError('short command')
                time.sleep(.001)
                frame=bytearray(os.read(fd,9))
                if len(frame)!=9 or list(frame[:2])!=[89,89] or sum(frame[:8])&255!=frame[8]:
                    raise ValueError('invalid frame')
                distance=(frame[2]+256*frame[3])*.01
                flux=frame[4]+256*frame[5]
                with telemetry.lock:
                    q=telemetry.imu_orientation
                norm=sum(v*v for v in q[:4])
                down=1-2*(q[0]*q[0]+q[1]*q[1])/norm
                if steady()-q[4]>.3 or down<.82 or not .10<=distance<=6 or not 100<=flux<65535:
                    raise ValueError('invalid range, tilt, signal or stale IMU')
                values.append(dict(t=steady()-began,range=distance,down=down,height=distance*down))
            except (OSError,ValueError,ZeroDivisionError):
                errors+=1
    finally:
        os.close(fd)
    heights=sorted(item['height'] for item in values)
    if len(values)<72:
        raise RuntimeError('Insufficient real ground-zero samples')
    median=heights[len(heights)//2]
    deviation=math.sqrt(sum((v-median)**2 for v in heights)/len(heights))
    if deviation>.03 or max(b['t']-a['t'] for a,b in zip(values,values[1:]))>.30:
        raise RuntimeError('Ground zero moved or lost range freshness')
    return dict(height_reference_metres=median,standard_deviation_metres=deviation,
        valid=len(values),rejected=errors,samples=values,sensor_offset_approximation=[0,0,0],
        relative_zero_for_current_support_only=True)


def run(arguments):
    directory = os.path.abspath(arguments.output_dir)
    if not os.path.isdir(directory):
        os.makedirs(directory)
    result_path = os.path.join(directory,'dwa-readonly-stack-results.json')
    if os.path.exists(result_path):
        raise RuntimeError('Refusing to overwrite a completed observation')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11333',ROS_IP='127.0.0.1',
        ROS_HOME=directory+'/ros-home',ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    os.environ.pop('ROS_NAMESPACE',None)
    reservation = socket.socket()
    reservation.bind(('127.0.0.1',11333))
    reservation.close()
    children = Children(directory)
    resources = Resources(children)
    report = dict(passed=False,flight_acceptance=False,private_master=os.environ['ROS_MASTER_URI'],
        observation_seconds=20.0,output_hz_is_inference_hz=False,
        location_mode=('real I2C6, measured zero, FCU vision fusion' if arguments.fuse_vision else
                       'real I2C6 TFmini, height uncalibrated' if arguments.i2c_range else
                       'unconnected topic range: no I2C access, no SLAM-z fallback'),errors=[])
    rospy = None
    images = None
    telemetry = None
    active = []
    began = ended = None
    try:
        master = children.start('master',['roscore','-p','11333'])
        deadline = steady()+15
        while steady()<deadline:
            children.require_alive([master])
            connection = socket.socket()
            try:
                connection.connect(('127.0.0.1',11333))
                break
            except socket.error:
                time.sleep(.1)
            finally:
                connection.close()
        else:
            raise RuntimeError('Private master startup timed out')
        import rospy
        rospy.init_node('shell_dwa_readonly_observer',disable_signals=True)
        telemetry = Telemetry(rospy,allow_vision=arguments.fuse_vision)
        images = Images(rospy,dict(camera='/camera/image_raw',output='/detect_result'))
        mavros_command=['roslaunch','location','rk_mavros_bench.launch']
        allowed_plugins=['global_position','imu','local_position','sys_status']
        if arguments.fuse_vision:
            allowed_plugins+=['vision_pose_estimate','sys_time','rc_io']
            import rospkg
            launch=ET.parse(os.path.join(rospkg.RosPack().get_path('location'),
                'launch','rk_mavros_bench.launch'))
            launch.find(".//rosparam[@param='plugin_whitelist']").text=str(allowed_plugins)
            node=launch.find('node')
            ET.SubElement(node,'param',name='conn/timesync_rate',value='10.0')
            ET.SubElement(node,'param',name='conn/system_time_rate',value='1.0')
            generated=os.path.join(directory,'mavros-vision-only.launch')
            launch.write(generated)
            mavros_command=['roslaunch',generated]
        commands = [
            ('camera',['roslaunch','usb_cam','rk_usb_camera.launch']),
            ('mavros',mavros_command),
            ('lidar',['roslaunch','location','rplidar.launch']),
            ('laser-tf',['rosrun','tf','static_transform_publisher','0','0','0','3.14159','0','0',
                         'base_link','laser_link','30']),
            ('cartographer',['roslaunch','location','cartographer_2d.launch']),
            ('location',['rosrun','location','location_node','_use_tf_mini:=true',
                '_tf_mini_type:='+('i2c' if arguments.i2c_range else 'topic'),
                '_range_topic:=/shell_readonly_unconnected_range','_bus:=6','_address:=16',
                '_height_reference_calibrated:=false']),
            ('navigation',['roslaunch','navigation_2d','navigation_2d.launch',
                'enable_actuation:=false','auto_start:=false','use_real_servos:=false',
                'target_hazard:=flammable','detect_backend:=rknn'])]
        if arguments.installed_preflight:
            commands=[('preflight',['roslaunch','location','rk_preflight.launch','target_hazard:=flammable'])]
            report['ground_reference']=dict(persistent_measured_profile=
                'location/param/rk_height_reference_20261027.yaml',height_reference_metres=.12)
        for name,command in commands:
            if name=='location' and arguments.fuse_vision:
                report['ground_reference']=measured_ground_reference(telemetry)
                command=[arg for arg in command if not arg.startswith('_height_reference_calibrated:=')]
                command+=['_height_reference_calibrated:=true','_tf_mini_height:='+
                    str(report['ground_reference']['height_reference_metres'])]
            active.append(children.start(name,command))
            resources.sample()
        # Start observation only after the actual sensor and model chains warm up;
        # still enforce zero outputs and unarmed FCU during all startup time.
        deadline = steady()+60
        while steady()<deadline:
            children.require_alive(active+[master])
            telemetry.discover(); telemetry.check(); resources.sample()
            with telemetry.lock:
                ready = any(isinstance(item['value'],dict) and
                    item['value'].get('inference_ready') for item in telemetry.strings['readiness'])
                sensors = all(telemetry.times[name] for name in ('scan','imu','state'))
            if ready and sensors and images.count('camera')>0 and images.count('output')>0:
                break
            time.sleep(.25)
        else:
            raise RuntimeError('Startup deadline: sensors, image output or inference_ready absent')
        mission_ns = '/shoot_with_move_base_node/'
        report['parameters'] = {name:rospy.get_param(name,None) for name in [
            mission_ns+'enable_actuation',mission_ns+'auto_start',mission_ns+'backend',
            mission_ns+'model_file',mission_ns+'publish_scale',
            '/mavros/startup_px4_usb_quirk','/mavros/plugin_whitelist',
            '/mavros/plugin_blacklist','/move_base/base_local_planner',
            '/location/use_tf_mini','/location/tf_mini_type','/location/height_reference_calibrated']}
        began = steady()
        # Reset image statistics for the same twenty-second observation window.
        with images.lock:
            images.times = {name:[] for name in images.times}
            images.shapes = {name:{} for name in images.shapes}
        while steady()-began < 20.0:
            children.require_alive(active+[master])
            telemetry.discover(); telemetry.check(); resources.sample()
            time.sleep(.25)
        ended = steady()
        report['observation_seconds'] = ended-began
        report['telemetry'] = telemetry.snapshot(began,ended)
        with images.lock:
            report['image_window_edges'] = {name:dict(
                first_delay=values[0]-began if values else None,
                last_age=ended-values[-1] if values else None)
                for name,values in images.times.items()}
        report['images'] = images.finish(directory); images = None
        data = report['telemetry']; streams = data['streams']; params=report['parameters']
        states = data['states']
        modes = sorted(set(state['mode'] for state in states if state['connected']))
        readies = [item['value'] for item in data['status']['readiness']
                   if isinstance(item['value'],dict)]
        report['readiness_health'] = readiness_health(data['status']['readiness'],began,ended)
        img = report['images']['streams']
        scale = params.get(mission_ns+'publish_scale')
        shape_ok = len(img['camera']['shapes'])==1 and len(img['output']['shapes'])==1
        if shape_ok:
            source,out = img['camera']['shapes'][0],img['output']['shapes'][0]
            shape_ok = (isinstance(scale,(int,float)) and
                abs(out['width']-source['width']*scale)<=1 and
                abs(out['height']-source['height']*scale)<=1)
        continuous = lambda name,rate,gap: (streams[name]['hz']>=rate and
            streams[name]['max_gap'] is not None and streams[name]['max_gap']<=gap and
            streams[name]['first_delay']<=gap and streams[name]['last_age']<=gap)
        report['checks'] = dict(
            camera_continuous=img['camera']['count']>=120 and img['camera']['hz']>=20
                and img['camera']['max_gap']<=1.0
                and report['image_window_edges']['camera']['first_delay']<=1.0
                and report['image_window_edges']['camera']['last_age']<=1.0,
            scan_continuous=continuous('scan',3.0,1.0),imu_continuous=continuous('imu',5.0,1.0),
            fcu_connected_unarmed_stable_mode=bool(states) and all(state['connected'] and not state['armed'] for state in states)
                and len(modes)==1 and bool(modes[0]) and continuous('state',.5,3.0),
            inference_ready_observed=report['readiness_health']['passed'],
            model_loaded=bool(readies) and all(value.get('model_loaded') is True for value in readies),
            # Resolved after all owned children have stopped and their log file
            # handles are closed; redirected ROS stdout may still be buffered.
            multiple_real_inferences=False,
            dwa_config_and_loaded=params['/move_base/base_local_planner']=='dwa_local_planner/DWAPlannerROS'
                and bool(resources.dwa_library_pids),
            safe_gate=params[mission_ns+'enable_actuation'] is False and params[mission_ns+'auto_start'] is False
                and all(value.get('actuation_enabled') is False for value in readies)
                and not any('servo_controller_node' in value['command'] for value in resources.owned.values()),
            mavros_readonly=params['/mavros/startup_px4_usb_quirk'] is False
                and sorted(params['/mavros/plugin_whitelist'] or [])==sorted(allowed_plugins)
                and params['/mavros/plugin_blacklist']==['*'],
            no_setpoints_gpio_or_vision=all(value==0 for value in data['zero_expected_topics'].values()),
            scaled_image_dimensions_consistent=shape_ok,images_valid=not report['images']['image_errors'],
            annotated_image_stream=img['output']['count']>=60 and img['output']['hz']>=5.0,
            location_range_blocked=bool(data['status']['location']) and
                all(item['value']!='READY' for item in data['status']['location']))
        if arguments.i2c_range and not arguments.fuse_vision:
            report['checks']['real_range_reaches_uncalibrated_gate'] = (
                params['/location/tf_mini_type']=='i2c' and
                params['/location/height_reference_calibrated'] is False and
                any(item['value']=='HEIGHT_REFERENCE_UNCALIBRATED'
                    for item in data['status']['location']) and
                not any(item['value'] in ('WAIT_RANGE','RANGE_STALE','IMU_STALE')
                    for item in data['status']['location']))
        if arguments.fuse_vision:
            report['checks']['mavros_fusion_plugins_only']=report['checks'].pop('mavros_readonly')
            report['checks']['no_setpoints_or_gpio']=report['checks'].pop('no_setpoints_gpio_or_vision')
            report['checks'].pop('location_range_blocked')
            report['checks']['location_ready']=bool(data['status']['location']) and all(
                item['value']=='READY' for item in data['status']['location'])
            report['checks']['real_vision_and_fcu_pose']=continuous('vision',5,1) and continuous('pose',5,1)
            report['checks']['fcu_estimator_local_position']=bool(data['estimators']) and all(
                r['local_xy'] and r['vertical'] for r in data['estimators'][-5:])
            report['receiver_observation']=data['rc'][-1] if data['rc'] else None
        report['fcu_modes'] = modes
        report['map_observation'] = dict(data['map'],required_for_readonly_pass=False,
            system_slam_acceptance=False,
            explanation='Observed mapping only; no valid flight localization or flight acceptance inferred')
        report['passed'] = all(report['checks'].values())
    except BaseException as error:
        report['errors'].append('%s: %s'%(type(error).__name__,error))
        report['traceback'] = traceback.format_exc()
        if telemetry is not None:
            report['telemetry'] = telemetry.snapshot(began or 0,steady())
        if images is not None:
            try:
                report['images'] = images.finish(directory)
            except Exception as image_error:
                report['errors'].append('Images: '+str(image_error))
        report['passed'] = False
    finally:
        try:
            resources.sample()
        except Exception as error:
            report['errors'].append('Resource sampling: '+str(error)); report['passed']=False
        report['errors'].extend(children.finish_all())
        report['orphan_group_stop_signals'] = resources.finish_owned_groups()
        # Let publisher processes close their TCPROS connections before the
        # observer shuts down. Concurrent peer/observer close races in Melodic.
        if rospy is not None:
            rospy.signal_shutdown('Bounded readonly observation complete')
        report['processes'] = children.report()
        report['resources'] = resources.report()
        report['remaining_owned_processes'] = resources.remaining()
        navigation = next((child for child in children.children if child['name'] in ('navigation','preflight')),None)
        if navigation is not None:
            try:
                report['inference_log'] = log_evidence(navigation['log'])
                report['inference_log_scope'] = 'whole process lifetime, including warm-up; read after child exit and log close'
                if 'checks' in report:
                    report['checks']['multiple_real_inferences'] = report['inference_log']['inference_ok']>=3
            except (IOError,OSError) as error:
                report['errors'].append('Final inference log read: '+str(error))
        normal = all(item['exit_code']==0 and item['process_group_empty'] and
            all(number==signal.SIGINT for number in item['stop_signals']) for item in report['processes'])
        report['normal_cleanup'] = normal and not report['remaining_owned_processes'] and not report['orphan_group_stop_signals']
        report['passed'] = bool(report.get('checks')) and all(report['checks'].values()) and report['normal_cleanup'] and not report['errors']
        with open(result_path,'w') as target:
            json.dump(report,target,indent=2,sort_keys=True)
        print(result_path)
    return 0 if report['passed'] else 1


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir')
    parser.add_argument('--i2c-range', action='store_true',
        help='Read the connected TFmini on I2C6 while withholding uncalibrated vision pose')
    parser.add_argument('--fuse-vision',action='store_true',
        help='Explicit unarmed ground check: measure relative zero then send real vision to FCU; no actuation')
    parser.add_argument('--installed-preflight',action='store_true',
        help='Validate the deployed rk_preflight.launch and saved measured height profile')
    args = parser.parse_args()
    if args.installed_preflight:
        args.fuse_vision=True
    if args.fuse_vision:
        args.i2c_range=True
    try:
        sys.exit(run(args))
    except Exception as error:
        print('Preflight failed: '+str(error),file=sys.stderr)
        sys.exit(2)
