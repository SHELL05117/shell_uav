#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Small ROS1 flight evidence recorder. No actuation or FCU services.

Only the C++ rosbag writer subscribes to ROS. The Python supervisor samples
system/selected process counters every five seconds; it does not decode images,
scans or TF. Reporting is explicit and offline after closing a session.
"""
from __future__ import print_function
import argparse
import fcntl
import glob
import hashlib
import json
import os
import signal
import socket
import subprocess
import sys
import time
import uuid

TOPICS = ['/mavros/vision_pose/pose', '/mavros/local_position/pose',
          '/mavros/local_position/velocity_local', '/mavros/setpoint_raw/local',
          '/mavros/setpoint_raw/target_local', '/mavros/state',
          '/mavros/extended_state', '/mavros/estimator_status',
          '/mavros/statustext/recv', '/mavros/rc/in', '/mavros/battery',
          '/location_node/status', '/mission/status', '/mission/readiness',
          '/cmd_vel', '/move_base/goal', '/move_base/result',
          '/gpio_cmd', '/gpio_ack', '/servo_ready']
STATE_DIR = os.path.expanduser('~/.local/state/uav-log-lite')
ACTIVE = os.path.join(STATE_DIR, 'active.json')
ROOT = os.path.expanduser('~/uav_flight_logs')
START_RESERVE = 3*1024**3
STOP_RESERVE = 1100*1024**2


def read(path):
    with open(path) as stream:
        return stream.read()


def steady():
    return float(read('/proc/uptime').split()[0])


def atomic_json(path, value):
    temp = path+'.tmp-'+str(os.getpid())
    with open(temp, 'w') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n'); stream.flush()
    os.rename(temp, path)


def free_bytes(path):
    stat = os.statvfs(path)
    return stat.f_bavail*stat.f_frsize


def identity(pid):
    try:
        raw = read('/proc/%d/stat' % pid)
        fields = raw[raw.rfind(')')+2:].split()
        if fields[0] == 'Z':
            return None
        return dict(pid=int(pid), start=fields[19],
                    boot=read('/proc/sys/kernel/random/boot_id').strip())
    except (IOError, OSError, IndexError, ValueError):
        return None


def active():
    try:
        value = json.loads(read(ACTIVE))
        return value if identity(value['process']['pid']) == value['process'] else None
    except (IOError, ValueError, KeyError, TypeError):
        return None


def child_session():
    os.setsid()
    # Detached SSH/background parents may have inherited SIGINT=SIG_IGN.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)


def prepare(mode):
    if active():
        raise RuntimeError('A recorder is already active; inspect uav logs status')
    root = os.path.abspath(os.path.expanduser(os.environ.get('UAV_LOG_ROOT', ROOT)))
    if not os.path.isdir(root):
        os.makedirs(root)
    if free_bytes(root) < START_RESERVE:
        raise RuntimeError('Less than 3 GiB free; no existing logs deleted')
    directory = os.path.join(root, time.strftime('%Y%m%d-%H%M%S')+'-lite-'+uuid.uuid4().hex[:8])
    os.mkdir(directory); os.mkdir(directory+'/ros-logs')
    atomic_json(directory+'/session.json', dict(schema=1, profile='lite', mode=mode,
        wall_start=time.time(), steady_start=steady(),
        boot=read('/proc/sys/kernel/random/boot_id').strip(),
        master=os.environ.get('ROS_MASTER_URI'), topics=TOPICS,
        performance_interval_s=5, split_s=60, buffer_mib=16,
        limitations=['No images, scan, map, full TF, inference source frames or raw TFmini samples',
                     'ROS header/bag times do not prove sensor latency or true flight trajectory',
                     'PX4 ULog is independent and must be retrieved separately']))
    return directory


def launch(directory):
    if not os.path.isdir(STATE_DIR):
        os.makedirs(STATE_DIR)
    with open(directory+'/recorder.stdout.log', 'ab') as output:
        proc = subprocess.Popen([sys.executable, os.path.abspath(__file__), 'daemon', directory],
            stdin=open(os.devnull, 'rb'), stdout=output, stderr=subprocess.STDOUT,
            preexec_fn=child_session, close_fds=True)
    deadline = steady()+7
    while steady() < deadline:
        if proc.poll() is not None:
            raise RuntimeError('Recorder failed during startup; see '+directory)
        if os.path.isfile(directory+'/ready.json'):
            return
        time.sleep(.1)
    raise RuntimeError('Recorder startup not confirmed; inspect uav logs status')


def attach(mode, arguments):
    state = active()
    if not state:
        return None
    directory = state['directory']
    session = json.loads(read(directory+'/session.json'))
    if session.get('master') != os.environ.get('ROS_MASTER_URI'):
        raise RuntimeError('Recorder belongs to a different ROS master')
    atomic_json(directory+'/command-%d-%d.json' % (int(time.time()*1000), os.getpid()),
                dict(mode=mode, arguments=arguments, wall=time.time(), steady=steady()))
    with open(directory+'/snapshot-request', 'w') as stream:
        stream.write(mode)
    return directory


def stop_active():
    state = active()
    if not state:
        return None
    original = state['process']
    if identity(original['pid']) != original:
        raise RuntimeError('Recorder identity changed; refusing signal')
    os.kill(original['pid'], signal.SIGINT)
    deadline = steady()+12
    while steady() < deadline and identity(original['pid']) == original:
        time.sleep(.1)
    if identity(original['pid']) == original:
        raise RuntimeError('Recorder did not close; no flight processes were signalled')
    return state['directory']


class Sampler(object):
    """Five-second counters, PID discovery only every 30 seconds."""
    def __init__(self):
        self.previous = None; self.last_scan = -100; self.pids = []
        self.hz = os.sysconf('SC_CLK_TCK')

    def sample(self):
        now = steady()
        if now-self.last_scan > 30:
            self.pids = []
            for path in glob.glob('/proc/[0-9]*/comm'):
                try:
                    name = read(path).strip()
                    if name.startswith(('record', 'rosbag', 'cartographer', 'location_node',
                                         'shoot_with_move', 'detectnet', 'usb_cam', 'mavros')):
                        self.pids.append((int(path.split('/')[2]), name))
                except IOError:
                    pass
            self.last_scan = now
        cpu = [int(x) for x in read('/proc/stat').splitlines()[0].split()[1:9]]
        mem = dict((x.split(':')[0], int(x.split()[1])) for x in read('/proc/meminfo').splitlines())
        row = dict(wall=time.time(), steady=now, cpu_busy_pct=None, iowait_pct=None,
                   mem_available_mib=mem.get('MemAvailable', mem['MemFree'])/1024.,
                   swap_used_mib=(mem['SwapTotal']-mem['SwapFree'])/1024., processes={})
        if self.previous:
            delta = [a-b for a,b in zip(cpu, self.previous['cpu'])]; total = sum(delta)
            if total > 0:
                row['cpu_busy_pct'] = 100.*(total-delta[3]-delta[4])/total
                row['iowait_pct'] = 100.*delta[4]/total
        for pid,name in self.pids:
            try:
                raw = read('/proc/%d/stat' % pid); fields = raw[raw.rfind(')')+2:].split()
                item = dict(name=name, start=fields[19], ticks=int(fields[11])+int(fields[12]),
                            rss_mib=int(fields[21])*os.sysconf('SC_PAGE_SIZE')/1048576.)
                old = self.previous['processes'].get(str(pid)) if self.previous else None
                if old and old['start'] == item['start'] and now > self.previous['steady']:
                    item['cpu_one_core_pct'] = 100.*(item['ticks']-old['ticks'])/self.hz/(now-self.previous['steady'])
                row['processes'][str(pid)] = item
            except (IOError, ValueError, IndexError):
                pass
        row['temperatures_c'] = {}
        for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
            try:
                row['temperatures_c'][path.split('/')[-2]] = float(read(path))/1000.
            except (IOError, ValueError):
                pass
        self.previous = dict(cpu=cpu, steady=now, processes=row['processes'])
        return row


class Recorder(object):
    def __init__(self, directory):
        self.directory = directory; self.stopping = False; self.writer = None
        self.params = None; self.errors = []; self.param_count = 0

    def event(self, name, **values):
        values.update(event=name, wall=time.time(), steady=steady())
        self.events.write(json.dumps(values, sort_keys=True)+'\n'); self.events.flush()

    def stop_child(self, proc):
        if proc is None:
            return
        for sig, seconds in [(signal.SIGINT, 7), (signal.SIGTERM, 2)]:
            if proc.poll() is not None:
                break
            os.killpg(proc.pid, sig)
            deadline = steady()+seconds
            while proc.poll() is None and steady() < deadline:
                time.sleep(.05)
        if proc.poll() is None:
            self.errors.append('owned_child_required_SIGKILL')
            os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()

    def versions(self):
        workspace = os.environ.get('UAV_WS', os.path.expanduser('~/catkin_ws'))
        names = ['src/navigation_2d/src/shoot_with_move_base.cpp', 'src/navigation_2d/src/flight_control.cpp',
                 'src/location/src/location_with_ftmini.cpp', 'src/location/launch/location.launch',
                 'src/location/param/rk_height_reference_20261027.yaml',
                 'src/location/param/ROSSLAM/cartographer_2Dlidar_mapping_location_params.lua',
                 'src/navigation_2d/detect_mode/26/best.rknn', 'src/navigation_2d/detect_mode/26/labels.txt',
                 'devel/lib/location/location_node', 'devel/lib/navigation_2d/shoot_with_move_base_node']
        hashes = {}
        for name in names:
            path = os.path.join(workspace, name)
            if os.path.isfile(path):
                digest = hashlib.sha256()
                with open(path, 'rb') as stream:
                    for block in iter(lambda: stream.read(1024*1024), b''):
                        digest.update(block)
                hashes[name] = digest.hexdigest()
        for name in ['uav_start.py', 'uav_log_lite.py']:
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
            if os.path.isfile(path):
                with open(path, 'rb') as stream:
                    hashes['installed/'+name] = hashlib.sha256(stream.read()).hexdigest()
        atomic_json(self.directory+'/versions.json', dict(files=hashes, uname=list(os.uname())))

    def run(self):
        import rosgraph
        os.nice(10); socket.setdefaulttimeout(2)
        self.lock = open(STATE_DIR+'/lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        if active():
            raise RuntimeError('Another recorder owns active state')
        self.process = identity(os.getpid())
        atomic_json(ACTIVE, dict(directory=self.directory, process=self.process, profile='lite'))
        self.events = open(self.directory+'/events.jsonl', 'a')
        self.perf = open(self.directory+'/performance.jsonl', 'a')
        self.event('start')
        try:
            writer_log = open(self.directory+'/writer.stdout.log', 'ab')
            self.writer = subprocess.Popen(['rosbag', 'record', '--split', '--duration=60',
                '--buffsize=16', '--chunksize=256', '-O', self.directory+'/core']+
                TOPICS+['__name:=uav_lite_recorder'], stdout=writer_log, stderr=subprocess.STDOUT,
                stdin=open(os.devnull, 'rb'), preexec_fn=child_session)
            self.versions()
            sampler = Sampler(); master = rosgraph.Master('/uav_lite_watchdog')
            began = steady(); last_master_ok = began; last_sources = began
            graph_seen = False; ready = False; next_sample = 0; param_due = began+5
            while not self.stopping:
                now = steady()
                if self.writer.poll() is not None:
                    self.errors.append('writer_exit_%s' % self.writer.returncode); break
                if not ready and glob.glob(self.directory+'/core*.bag.active'):
                    ready = True; atomic_json(self.directory+'/ready.json', dict(wall=time.time(), steady=now))
                    self.event('recording')
                if now >= next_sample:
                    self.perf.write(json.dumps(sampler.sample(), sort_keys=True)+'\n'); self.perf.flush()
                    available = free_bytes(self.directory)
                    atomic_json(self.directory+'/status.json', dict(state='RECORDING', wall=time.time(),
                        steady=now, free_bytes=available, writer=identity(self.writer.pid), profile='lite'))
                    if available < STOP_RESERVE:
                        self.errors.append('disk_reserve_reached'); break
                    try:
                        graph = master.getSystemState(); last_master_ok = now
                        names = set(n for group in graph for topic,nodes in group for n in nodes)
                        if any(n in names for n in ('/mavros', '/location_node', '/shoot_with_move_base_node', '/camera')):
                            graph_seen = True; last_sources = now
                        if graph_seen and now-last_sources > 10:
                            self.event('sources_stopped'); break
                    except Exception:
                        if now-last_master_ok > 15:
                            self.event('master_gone'); break
                    next_sample = now+5
                request = self.directory+'/snapshot-request'
                if os.path.exists(request):
                    os.unlink(request); param_due = now+3
                if self.params and self.params.poll() is not None:
                    self.event('parameters', returncode=self.params.returncode); self.params = None
                if param_due and now >= param_due and self.params is None:
                    self.param_count += 1; param_due = None
                    self.params = subprocess.Popen(['timeout', '10', 'rosparam', 'dump',
                        self.directory+'/parameters-%02d.yaml' % self.param_count],
                        stdout=open(self.directory+'/parameters.stdout.log', 'ab'), stderr=subprocess.STDOUT,
                        preexec_fn=child_session)
                time.sleep(.25)
        except Exception as error:
            self.errors.append(str(error)); raise
        finally:
            self.stop_child(self.params); self.stop_child(self.writer)
            unclosed = glob.glob(self.directory+'/*.bag.active')
            atomic_json(self.directory+'/closed.json', dict(wall=time.time(), steady=steady(),
                profile='lite', errors=self.errors, unclosed_bags=unclosed,
                writer_returncode=self.writer.returncode if self.writer else None))
            self.event('closed', errors=self.errors, unclosed_bags=unclosed)
            if active() and active().get('process') == self.process:
                os.unlink(ACTIVE)
            self.events.close(); self.perf.close()


def report(directory):
    import rosbag
    directory = os.path.abspath(directory)
    state = active()
    if state and state['directory'] == directory or glob.glob(directory+'/*.bag.active'):
        raise RuntimeError('Recording is active; analyze only after landing and closure')
    if not os.path.isfile(directory+'/closed.json'):
        raise RuntimeError('No close marker; recover/reindex interrupted bags before reporting')
    from collections import defaultdict
    rows = defaultdict(lambda: dict(count=0, previous=None, max_gap_s=None, header_age_max_s=None))
    events = []; states = []; errors = []
    for path in sorted(glob.glob(directory+'/*.bag')):
        try:
            with rosbag.Bag(path) as bag:
                for topic,msg,stamp in bag.read_messages():
                    now = stamp.to_sec(); row = rows[topic]; row['count'] += 1
                    row.setdefault('first', now); row['last'] = now
                    if row['previous'] is not None:
                        gap = now-row['previous']
                        row['max_gap_s'] = max(row['max_gap_s'] or 0, gap)
                    row['previous'] = now
                    header = getattr(msg, 'header', None)
                    if header and header.stamp.to_sec() > 0:
                        age = now-header.stamp.to_sec()
                        old = row['header_age_max_s']
                        row['header_age_max_s'] = age if old is None else max(old, age)
                    if topic in ('/mission/status', '/location_node/status', '/gpio_cmd', '/gpio_ack'):
                        events.append(dict(time=now, topic=topic, value=msg.data))
                    if topic == '/mavros/state':
                        value = [msg.armed, msg.mode, msg.connected]
                        if not states or value != states[-1]['value']:
                            states.append(dict(time=now, value=value))
        except Exception as error:
            errors.append(os.path.basename(path)+': '+str(error))
    for row in rows.values():
        row.pop('previous'); elapsed = row['last']-row['first']
        row['hz'] = (row['count']-1)/elapsed if elapsed > 0 else None
    writer = read(directory+'/writer.stdout.log').lower()
    if 'buffer exceeded' in writer or 'dropping oldest' in writer:
        errors.append('Writer reported buffer overflow')
    closed = json.loads(read(directory+'/closed.json')); errors += closed.get('errors', [])
    summary = dict(profile='lite', topics=dict(rows), states=states, events=events, errors=errors,
        recording_closed_without_writer_errors=not errors and not closed.get('unclosed_bags'),
        missing_topics=[topic for topic in TOPICS if topic not in rows],
        limitations=json.loads(read(directory+'/session.json'))['limitations'])
    atomic_json(directory+'/summary.json', summary)
    with open(directory+'/report.md', 'w') as stream:
        stream.write('# Lightweight flight evidence\n\nProfile: lite.\n\n')
        stream.write('Readable/closed without reported writer error: %s\n\n' % summary['recording_closed_without_writer_errors'])
        stream.write('Missing topics are not evidence of zero commands; verify expected flight phase.\n\n')
        for topic,row in sorted(rows.items()):
            stream.write('- %s: %d messages; max receive gap %s s\n' % (topic,row['count'],row['max_gap_s']))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['daemon', 'status', 'stop', 'report'])
    parser.add_argument('directory', nargs='?')
    args = parser.parse_args()
    if args.command == 'daemon':
        if not args.directory:
            parser.error('daemon requires directory')
        recorder = Recorder(os.path.abspath(args.directory))
        signal.signal(signal.SIGINT, lambda *_: setattr(recorder, 'stopping', True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(recorder, 'stopping', True))
        recorder.run()
    elif args.command == 'status':
        state = active()
        if state:
            try:
                state['recording'] = json.loads(read(state['directory']+'/status.json'))
            except (IOError, ValueError):
                state['recording'] = 'STARTING'
        print(json.dumps(state or dict(state='INACTIVE', root=ROOT), indent=2, sort_keys=True))
    elif args.command == 'stop':
        print(stop_active() or 'No active lightweight recorder')
    elif args.command == 'report':
        if not args.directory:
            parser.error('report requires a closed session directory')
        print(json.dumps(report(args.directory), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
