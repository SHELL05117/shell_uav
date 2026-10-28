#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded Wi-Fi ground diagnostics: real sensors, no height/flight/servos.

Owns its ROS master and child groups only. stage.txt labels sample windows;
STOP ends the run. Raw camera images are not subscribed by the observer.
MAVROS uses the sensor-only whitelist, never command/setpoint plugins.
"""
from __future__ import print_function
import argparse,glob,json,os,signal,socket,subprocess,time,xmlrpclib


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output'); parser.add_argument('--seconds',type=int,default=720)
    parser.add_argument('--ip',required=True); parser.add_argument('--port',type=int,default=11311)
    parser.add_argument('--staged',action='store_true',required=True,help='Required: advance sources using level.txt: 1 localization, 2 camera, 3 dry inference, 4 JPEG relay')
    args=parser.parse_args(); root=os.path.abspath(args.output)
    if not os.path.isdir(root): os.makedirs(root)
    os.environ.update(ROS_MASTER_URI='http://'+args.ip+':'+str(args.port),ROS_IP=args.ip,
                      ROS_LOG_DIR=root+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    guard=socket.socket();guard.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    guard.bind((args.ip,args.port));guard.close()
    children=[]; streams=[]; latest={}; counters={}; maximum_gap={}; received={}
    violations=[]
    def interrupted(signum,frame):
        raise KeyboardInterrupt('Diagnostic interrupted by signal %d'%signum)
    for signum in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(signum,interrupted)
    def stop(process):
        if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        deadline=time.time()+10
        while process.poll() is None and time.time()<deadline: time.sleep(.1)
        if process.poll() is None: os.killpg(process.pid,signal.SIGKILL)
        process.wait()
    def start(name,command):
        stream=open(root+'/'+name+'.log','w');streams.append(stream)
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
        children.append((name,process));return process
    def stat():
        with open('/proc/stat') as f: values=[int(x) for x in f.readline().split()[1:]]
        return sum(values),values[3]+values[4]
    def net():
        return [int(open('/sys/class/net/wlan0/statistics/'+key).read())
                for key in ('rx_bytes','tx_bytes','rx_dropped','tx_dropped','rx_errors','tx_errors')]
    def processes():
        result={}
        for path in glob.glob('/proc/[0-9]*/stat'):
            try:
                data=open(path).read().split(') ',1)[1].split()
                command=open(path.replace('/stat','/cmdline'),'rb').read().replace('\0',' ')
                if any(key in command for key in ('cartographer_node','usb_cam_node','shoot_with_move_base_node',
                       'monitor_jpeg.py','mavros_node','rplidarNode','uav_start.py','gate_live_worker')):
                    result[path.split('/')[2]]=dict(ticks=int(data[11])+int(data[12]),
                        rss_bytes=int(data[21])*os.sysconf('SC_PAGE_SIZE'),command=command)
            except (IOError,IndexError): pass
        return result
    start('master',['roscore','-p',str(args.port)])
    try:
        time.sleep(3)
        import rospy,rosgraph,tf2_ros
        from sensor_msgs.msg import Imu,LaserScan
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State,PositionTarget
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        rospy.init_node('wifi_ground_observer',disable_signals=True)
        master=rosgraph.Master(rospy.get_name())
        initial_nodes=set(n for group in master.getSystemState() for topic,nodes in group for n in nodes)
        if initial_nodes-set(['/rosout',rospy.get_name()]): raise RuntimeError('Existing graph; refuse overlapping sources')
        def callback(key):
            def cb(message):
                now=time.time();counters[key]=counters.get(key,0)+1
                if key in received:maximum_gap[key]=max(maximum_gap.get(key,0),now-received[key])
                received[key]=now
                if key=='state':
                    latest[key]=dict(connected=message.connected,armed=message.armed,mode=message.mode)
                    if message.armed: violations.append('ARMED')
                elif key=='map': latest[key]=dict(width=message.info.width,height=message.info.height)
                elif key in ('mission','location','readiness'): latest[key]=message.data
                if key in ('setpoint','gpio','vision'): violations.append('unexpected '+key)
            return cb
        subscriptions=[rospy.Subscriber(topic,kind,callback(key),queue_size=2) for key,topic,kind in [
            ('state','/mavros/state',State),('imu','/mavros/imu/data',Imu),('scan','/scan',LaserScan),
            ('map','/map',OccupancyGrid),('vision','/mavros/vision_pose/pose',PoseStamped),
            ('mission','/mission/status',String),('readiness','/mission/readiness',String),
            ('location','/location_node/status',String),('setpoint','/mavros/setpoint_raw/local',PositionTarget),
            ('gpio','/gpio_cmd',String)]]
        buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
        start('fcu',['roslaunch','location','rk_mavros_vision.launch'])
        start('lidar',['roslaunch','location','rplidar.launch','serial_port:='+glob.glob('/dev/serial/by-id/*CP2102*')[0]])
        start('laser_tf',['rosrun','tf','static_transform_publisher','0','0','0','3.14159','0','0','base_link','laser_link','30'])
        start('slam',['roslaunch','location','cartographer_2d.launch'])
        start('location',['rosrun','location','location_node','__name:=location_node','_use_tf_mini:=false'])
        level=[1]
        def advance(requested):
            if level[0]<2 and requested>=2:
                start('camera',['roslaunch','usb_cam','rk_usb_camera.launch','mjpeg_backend:=mpp_rga',
                                'video_device:='+glob.glob('/dev/v4l/by-id/*Camera*video-index0')[0]])
                level[0]=2
            if level[0]<3 and requested>=3:
                start('navigation',['roslaunch','navigation_2d','navigation_2d.launch','detect_backend:=rknn',
                                    'target_hazard:=flammable','enable_actuation:=false','auto_start:=false','use_real_servos:=false'])
                level[0]=3
            if level[0]<4 and requested>=4:
                start('monitor',['roslaunch','location','lichtblick_bridge_readonly.launch','jpeg_relay:=true','start_bridge:=false'])
                level[0]=4
        with open(root+'/started.json','w') as f:
            json.dump(dict(pid=os.getpid(),boot=open('/proc/sys/kernel/random/boot_id').read().strip(),
                           master=os.environ['ROS_MASTER_URI'],height=False,actuation=False),f,indent=2)
        started=time.time();previous_cpu=stat();previous_net=net();previous_time=started
        previous_processes=processes();clock=float(os.sysconf('SC_CLK_TCK'));last_snapshot=0
        sample_count=0;socket.setdefaulttimeout(2)
        with open(root+'/samples.jsonl','w') as f:
            while time.time()-started<args.seconds and not os.path.exists(root+'/STOP'):
                time.sleep(1);now=time.time();elapsed=now-previous_time;cpu=stat();network=net();proc=processes()
                if args.staged and os.path.isfile(root+'/level.txt'):
                    advance(int(open(root+'/level.txt').read().strip()))
                stage=open(root+'/stage.txt').read().strip() if os.path.isfile(root+'/stage.txt') else 'warmup'
                age={}
                for frame in ('map','odom'):
                    try:
                        transform=buffer.lookup_transform(frame,'base_link',rospy.Time(0))
                        age[frame]=(rospy.Time.now()-transform.header.stamp).to_sec()
                    except Exception:age[frame]=None
                pstats=[]
                for pid,p in proc.items():
                    if pid in previous_processes:
                        pstats.append(dict(pid=pid,command=p['command'],rss_bytes=p['rss_bytes'],
                            cpu_one_core_percent=100.*(p['ticks']-previous_processes[pid]['ticks'])/clock/elapsed))
                row=dict(wall=now,elapsed_s=now-started,stage=stage,tf_age_s=age,
                    source_level=level[0],
                    cpu_system_percent=100.*(1.-float(cpu[1]-previous_cpu[1])/(cpu[0]-previous_cpu[0])),
                    wlan_rx_mbit_s=8.*(network[0]-previous_net[0])/elapsed/1e6,
                    wlan_tx_mbit_s=8.*(network[1]-previous_net[1])/elapsed/1e6,
                    wlan_counters=network,counters=dict(counters),max_receive_gap_s=dict(maximum_gap),
                    latest=dict(latest),processes=pstats,violations=list(violations))
                f.write(json.dumps(row)+'\n');f.flush();sample_count+=1
                if now-last_snapshot>10:
                    nodes=sorted(set(n for group in master.getSystemState() for topic,ns in group for n in ns))
                    if any('servo_controller' in n for n in nodes):violations.append('servo node')
                    snapshot=dict(nodes=nodes,services=master.getSystemState()[2],buses={})
                    for node in ('/camera','/monitor_jpeg','/shoot_with_move_base_node'):
                        try:
                            proxy=xmlrpclib.ServerProxy(master.lookupNode(node))
                            entry=dict(info=proxy.getBusInfo(rospy.get_name()))
                            snapshot['buses'][node]=entry
                            try:entry['stats']=proxy.getBusStats(rospy.get_name())
                            except Exception as error:entry['stats_error']=str(error)
                        except Exception as error:snapshot['buses'][node]=str(error)
                    with open(root+'/graph-%04d.json'%sample_count,'w') as sf:json.dump(snapshot,sf,indent=2)
                    with open(root+'/kernel-latest.log','w') as kf:
                        subprocess.call(['sudo','-n','dmesg'],stdout=kf,stderr=subprocess.STDOUT)
                    last_snapshot=now
                if violations:raise RuntimeError(', '.join(violations))
                if any(p.poll() is not None for _,p in children):raise RuntimeError('Owned source exited')
                previous_cpu,previous_net,previous_time,previous_processes=cpu,network,now,proc
        for sub in subscriptions:sub.unregister()
        listener.unregister()
        with open(root+'/closed.json','w') as f:json.dump(dict(samples=sample_count,violations=violations),f,indent=2)
    finally:
        for _,process in reversed(children):stop(process)
        for stream in streams:stream.close()


if __name__=='__main__': main()
