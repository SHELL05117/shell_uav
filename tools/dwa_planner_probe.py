#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Run real Navfn + DWA with synthetic inputs on a private ROS master.

Usage after sourcing devel/setup.bash:
  python2 tools/dwa_planner_probe.py OUTPUT_DIR

No MAVROS, mission, GPIO, I2C, servo or camera node is started. The simulator
integrates the planner's body-frame Twist; this is software execution evidence,
not validation of the real aircraft footprint or flying performance.
"""
from __future__ import print_function
import argparse
import ast
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time


def _monotonic_clock():
    if hasattr(time,'monotonic'):
        return time.monotonic,'time.monotonic'
    # ROS Melodic's Python 2 lacks time.monotonic; never silently use wall time.
    import ctypes
    class Timespec(ctypes.Structure):
        _fields_=[('seconds',ctypes.c_long),('nanoseconds',ctypes.c_long)]
    try: library=ctypes.CDLL(None,use_errno=True); clock=library.clock_gettime
    except AttributeError:
        library=ctypes.CDLL('librt.so.1',use_errno=True); clock=library.clock_gettime
    clock.argtypes=[ctypes.c_int,ctypes.POINTER(Timespec)]; clock.restype=ctypes.c_int
    def read():
        value=Timespec()
        if clock(1,ctypes.byref(value))!=0:
            raise OSError(ctypes.get_errno(),'CLOCK_MONOTONIC failed')
        return value.seconds+value.nanoseconds*1e-9
    read()
    return read,'clock_gettime(CLOCK_MONOTONIC)'


monotonic,CLOCK_SOURCE=_monotonic_clock()


def integrate_command_history(pose,start,end,command,events,max_step=.025):
    """Integrate every elapsed second, splitting at command arrivals and small steps.

    events are (monotonic arrival time, body vx/vy/yaw-rate). Returns final pose,
    command active at end, and timestamped substep poses for collision checks.
    """
    if end<start or max_step<=0 or any(math.isnan(x) or math.isinf(x)
            for x in list(pose)+list(command)+[start,end,max_step]):
        raise ValueError('Invalid integration interval or state')
    state=list(pose); active=tuple(command); cursor=start; samples=[]
    changes=sorted(events,key=lambda event:event[0])
    for when,values in changes:
        if any(math.isnan(x) or math.isinf(x) for x in [when]+list(values)) or len(values)!=3:
            raise ValueError('Invalid command arrival')
        if when<=start: active=tuple(values)
    samples.append([start]+state+list(active))
    def advance(until,velocity):
        duration=until-cursor
        if duration<=0: return
        steps=max(1,int(math.ceil(duration/max_step))); dt=duration/steps
        vx,vy,omega=velocity
        for index in range(steps):
            heading=state[2]; angle=heading+omega*dt
            if abs(omega)<1e-12:
                state[0]+=(math.cos(heading)*vx-math.sin(heading)*vy)*dt
                state[1]+=(math.sin(heading)*vx+math.cos(heading)*vy)*dt
            else:
                state[0]+=(vx*(math.sin(angle)-math.sin(heading))+vy*(math.cos(angle)-math.cos(heading)))/omega
                state[1]+=(vx*(math.cos(heading)-math.cos(angle))+vy*(math.sin(angle)-math.sin(heading)))/omega
            state[2]=angle
            samples.append([cursor+(index+1)*dt]+list(state)+list(velocity))
    for when,values in changes:
        if when<=start: continue
        if when>end: break
        advance(when,active); cursor=when; active=tuple(values)
    advance(end,active)
    return state,active,samples


def parse_footprint(value):
    """Costmap dynamic reconfigure may normalize the ROS array to a string."""
    if not isinstance(value, (list, tuple)):
        try:
            value = ast.literal_eval(value)
        except (ValueError, SyntaxError, TypeError):
            raise ValueError('Configured footprint is not a literal polygon')
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError('Configured footprint needs at least three vertices')
    points = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError('Configured footprint vertex needs x and y')
        pair = tuple(float(number) for number in point)
        if any(math.isnan(number) or math.isinf(number) for number in pair):
            raise ValueError('Configured footprint contains nonfinite coordinates')
        points.append(pair)
    twice_area = sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))
    if abs(twice_area) < 1e-12:
        raise ValueError('Configured footprint has zero area')
    return points


def ray_box_range(x,y,dx,dy,box):
    """First positive distance along a unit ray to an axis-aligned rectangle."""
    near=0.0; far=float('inf')
    for origin,direction,low,high in ((x,dx,box[0],box[1]),(y,dy,box[2],box[3])):
        if abs(direction)<1e-12:
            if origin<low or origin>high: return float('inf')
        else:
            first=(low-origin)/direction; second=(high-origin)/direction
            near=max(near,min(first,second)); far=min(far,max(first,second))
            if far<near: return float('inf')
    return near if far>=near and far>=0 else float('inf')


def point_segment_distance(point,start,end):
    vx=end[0]-start[0]; vy=end[1]-start[1]
    length=vx*vx+vy*vy
    fraction=0 if length==0 else max(0,min(1,((point[0]-start[0])*vx+(point[1]-start[1])*vy)/length))
    return math.hypot(point[0]-start[0]-fraction*vx,point[1]-start[1]-fraction*vy)


def footprint_box_clearance(pose,footprint,box):
    """Exact convex polygon/AABB gap; zero also reports overlap or touching."""
    c=math.cos(pose[2]); s=math.sin(pose[2])
    polygon=[(pose[0]+c*x-s*y,pose[1]+s*x+c*y) for x,y in footprint]
    rectangle=[(box[0],box[2]),(box[1],box[2]),(box[1],box[3]),(box[0],box[3])]
    separated=False
    for shape in (polygon,rectangle):
        for index,point in enumerate(shape):
            next_point=shape[(index+1)%len(shape)]
            axis=(next_point[1]-point[1],point[0]-next_point[0])
            first=[x*axis[0]+y*axis[1] for x,y in polygon]
            second=[x*axis[0]+y*axis[1] for x,y in rectangle]
            if max(first)<min(second) or max(second)<min(first): separated=True
    if not separated: return 0.0
    distances=[]
    for first,second in ((polygon,rectangle),(rectangle,polygon)):
        for point in first:
            for index,start in enumerate(second):
                distances.append(point_segment_distance(point,start,second[(index+1)%len(second)]))
    return min(distances)


def stop(process):
    for signum,seconds in ((signal.SIGINT,7),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        if process.poll() is not None: return process.returncode
        try: os.killpg(process.pid,signum)
        except OSError:
            if process.poll() is None: raise
        deadline=monotonic()+seconds
        while process.poll() is None and monotonic()<deadline:
            time.sleep(.05)
    if process.poll() is None: raise RuntimeError('Owned child did not stop after SIGKILL')
    return process.returncode


def launch_xml(namespace,forward_point_distance=None,occdist_scale=None):
    xml='''<launch>
  <group ns="%s">
    <node pkg="move_base" type="move_base" name="move_base" output="screen">
      <rosparam file="$(find navigation_2d)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
      <rosparam file="$(find navigation_2d)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
      <rosparam file="$(find navigation_2d)/param/global_costmap_params.yaml" command="load" />
      <rosparam file="$(find navigation_2d)/param/local_costmap_params.yaml" command="load" />
      <rosparam file="$(find navigation_2d)/param/move_base_params.yaml" command="load" />
      <rosparam file="$(find navigation_2d)/param/dwa_local_planner_params.yaml" command="load" />
      <param name="base_global_planner" value="navfn/NavfnROS" />
      <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
      <param name="global_costmap/global_frame" value="%s_map" />
      <param name="local_costmap/global_frame" value="%s_map" />
      <param name="global_costmap/robot_base_frame" value="%s_base" />
      <param name="local_costmap/robot_base_frame" value="%s_base" />
      <param name="global_costmap/scan/sensor_frame" value="%s_laser" />
      <param name="local_costmap/scan/sensor_frame" value="%s_laser" />
      <param name="global_costmap/scan/topic" value="/%s/scan" />
      <param name="local_costmap/scan/topic" value="/%s/scan" />
      <!-- Complete snapshots are diagnostic evidence; planner inputs are unchanged. -->
      <param name="global_costmap/always_send_full_costmap" value="true" />
      <param name="local_costmap/always_send_full_costmap" value="true" />
      <param name="global_costmap/static_layer/map_topic" value="/%s/map" />
      <param name="global_costmap/map_topic" value="/%s/map" />
      <param name="DWAPlannerROS/odom_topic" value="/%s/odom" />
      <param name="clearing_rotation_allowed" value="false" />
      <param name="recovery_behavior_enabled" value="false" />
      <remap from="cmd_vel" to="/%s/cmd_vel" />
      <remap from="odom" to="/%s/odom" />
      <remap from="map" to="/%s/map" />
    </node>
  </group>
</launch>''' % tuple([namespace]*15)
    if forward_point_distance is not None:
        xml=xml.replace('    </node>',
            '      <param name="DWAPlannerROS/forward_point_distance" value="%s" />\n    </node>' % forward_point_distance)
    if occdist_scale is not None:
        xml=xml.replace('    </node>',
            '      <param name="DWAPlannerROS/occdist_scale" value="%s" />\n    </node>' % occdist_scale)
    return xml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--occdist-scale',type=float,default=None,
                        help='Diagnostic override only; does not edit production YAML')
    parser.add_argument('--case',choices=('all','straight','yaw90','obstacle'),default='all')
    parser.add_argument('--forward-point-distance',type=float,default=None,
                        help='Diagnostic override only; production YAML is otherwise used unchanged')
    args=parser.parse_args()
    directory=os.path.abspath(args.output)
    if not os.path.isdir(directory): os.makedirs(directory)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11331',ROS_IP='127.0.0.1',
                      ROS_HOME=directory+'/ros-home',ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    reservation=socket.socket()
    reservation.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    reservation.bind(('127.0.0.1',11331)); reservation.close()
    children=[]; streams=[]; active_trace=[None]; active_evidence=[None]; active_simulation=[None]
    def save_simulation(snapshot):
        namespace,timing,seen,integration_trace,collision=snapshot
        with open(directory+'/'+namespace+'-integration-trace.json','w') as output:
            json.dump(integration_trace,output)
        with open(directory+'/'+namespace+'-commands.json','w') as output: json.dump(seen,output)
        timing['integration_error_seconds']=timing['integrated_seconds']-timing['observed_elapsed_seconds']
        with open(directory+'/'+namespace+'-timing.json','w') as output:
            json.dump(dict(timing=timing,collision=collision),output,indent=2,sort_keys=True)
    def start(name,command):
        stream=open(directory+'/'+name+'.log','w'); streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
        children.append((name,child)); return child
    result=dict(cases=[],exits={},scope='synthetic software planner execution only',
                aircraft_footprint_measured=False,master='127.0.0.1:11331',
                requested_case=args.case,forward_point_distance_override=args.forward_point_distance,
                clock_source=CLOCK_SOURCE,trace_columns=['monotonic_seconds','x','y','yaw','body_vx','body_vy','yaw_rate','ros_seconds'])
    master=start('master',['roscore','-p','11331'])
    try:
        deadline=monotonic()+15
        while monotonic()<deadline:
            sock=socket.socket()
            try: sock.connect(('127.0.0.1',11331)); break
            except socket.error: time.sleep(.1)
            finally: sock.close()
        else: raise RuntimeError('Private master did not start')
        import rospy
        import actionlib
        import tf2_ros
        from geometry_msgs.msg import Twist,TransformStamped
        from nav_msgs.msg import OccupancyGrid,Odometry,Path
        from sensor_msgs.msg import LaserScan
        from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
        from actionlib_msgs.msg import GoalStatus
        rospy.init_node('dwa_software_probe',disable_signals=True)

        def run_case(name,yaw,obstacle=False):
            namespace='dwa_'+name
            prefix='/'+namespace
            frame=lambda part:namespace+'_'+part
            path=directory+'/'+namespace+'.launch'
            with open(path,'w') as output: output.write(launch_xml(namespace,args.forward_point_distance,args.occdist_scale))
            command=[0.0,0.0,0.0]
            pending_commands=[]; command_lock=threading.Lock()
            seen=[]; plans=[]; local_plans=[]; costmaps={}
            def velocity(message):
                values=(message.linear.x,message.linear.y,message.angular.z)
                # Timestamp under the same lock used by pump: a callback cannot
                # append a retroactive event after that interval was integrated.
                with command_lock:
                    event=(monotonic(),values)
                    pending_commands.append(event); seen.append(event)
            def plan(message): plans.append(message)
            def local_plan(message): local_plans.append(message)
            def costmap(kind,message): costmaps[kind]=message
            subscribers=[rospy.Subscriber(prefix+'/cmd_vel',Twist,velocity,queue_size=30),
                         rospy.Subscriber(prefix+'/move_base/NavfnROS/plan',Path,plan,queue_size=10),
                         rospy.Subscriber(prefix+'/move_base/DWAPlannerROS/local_plan',Path,local_plan,queue_size=10),
                         rospy.Subscriber(prefix+'/move_base/global_costmap/costmap',OccupancyGrid,
                             lambda message:costmap('global',message),queue_size=1),
                         rospy.Subscriber(prefix+'/move_base/local_costmap/costmap',OccupancyGrid,
                             lambda message:costmap('local',message),queue_size=1)]
            active_evidence[0]=(namespace,plans,local_plans,costmaps)
            odom_pub=rospy.Publisher(prefix+'/odom',Odometry,queue_size=5)
            scan_pub=rospy.Publisher(prefix+'/scan',LaserScan,queue_size=5)
            map_pub=rospy.Publisher(prefix+'/map',OccupancyGrid,queue_size=1,latch=True)
            broadcaster=tf2_ros.TransformBroadcaster()
            grid=OccupancyGrid(); grid.header.frame_id=frame('map')
            grid.info.resolution=.05; grid.info.width=120; grid.info.height=120
            grid.info.origin.position.x=-3; grid.info.origin.position.y=-3
            grid.info.origin.orientation.w=1
            # Obstacle occupies complete map cells: x=[-.25,.25], y=[-.5,.5].
            # The empty-field cases retain their original map and start/goal.
            box=(-.25,.25,-.5,.5)
            grid.data=[100 if x in (0,119) or y in (0,119) or
                       (obstacle and 55<=x<65 and 50<=y<70) else 0
                       for y in range(120) for x in range(120)]
            pose=[-1.5 if obstacle else 0.0,0.0,yaw]
            previous=[monotonic()]; map_due=[0.0]
            trace=[]; integration_trace=[]; collision_footprint=[None]
            timing=dict(clock_source=CLOCK_SOURCE,integration_step_limit=.025,
                        observed_elapsed_seconds=0.0,integrated_seconds=0.0,
                        integration_error_seconds=0.0,
                        maximum_pump_gap=0.0,maximum_integration_step=0.0,
                        integration_substeps=0,pump_calls=0,startup_hold_seconds=0.0)
            collision=dict(checked_substep_poses=0,overlapping_substep_poses=0,
                           minimum_footprint_clearance=None)
            active_simulation[0]=(namespace,timing,seen,integration_trace,collision)
            active_trace[0]=(directory+'/'+namespace+'-trace.json',trace)
            def pump(integrate=True):
                with command_lock:
                    now=monotonic(); stamp=rospy.Time.now()
                    changes=list(pending_commands); del pending_commands[:]
                before=previous[0]; delta=now-before; previous[0]=now
                timing['pump_calls']+=1
                timing['maximum_pump_gap']=max(timing['maximum_pump_gap'],delta)
                if integrate:
                    new_pose,new_command,samples=integrate_command_history(pose,before,now,command,changes)
                    pose[:]=new_pose; command[:]=new_command
                    integration_trace.extend(samples)
                    gaps=[b[0]-a[0] for a,b in zip(samples,samples[1:])]
                    timing['observed_elapsed_seconds']+=delta
                    timing['integrated_seconds']+=sum(gaps)
                    timing['integration_error_seconds']=timing['integrated_seconds']-timing['observed_elapsed_seconds']
                    timing['integration_substeps']+=len(gaps)
                    timing['maximum_integration_step']=max([timing['maximum_integration_step']]+gaps)
                    if obstacle and collision_footprint[0] is not None:
                        for sample in samples:
                            gap=footprint_box_clearance(sample[1:4],collision_footprint[0],box)
                            old=collision['minimum_footprint_clearance']
                            collision['minimum_footprint_clearance']=gap if old is None else min(old,gap)
                            collision['checked_substep_poses']+=1
                            if gap<=0: collision['overlapping_substep_poses']+=1
                else:
                    timing['startup_hold_seconds']+=delta
                    if changes: command[:]=changes[-1][1]
                transforms=[]
                for parent,child,x,y,angle in [(frame('map'),frame('odom'),0,0,0),
                    (frame('odom'),frame('base'),pose[0],pose[1],pose[2]),
                    (frame('base'),frame('laser'),0,0,0)]:
                    transform=TransformStamped(); transform.header.stamp=stamp
                    transform.header.frame_id=parent; transform.child_frame_id=child
                    transform.transform.translation.x=x; transform.transform.translation.y=y
                    transform.transform.rotation.z=math.sin(angle/2)
                    transform.transform.rotation.w=math.cos(angle/2); transforms.append(transform)
                broadcaster.sendTransform(transforms)
                odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id=frame('odom')
                odom.child_frame_id=frame('base'); odom.pose.pose.position.x=pose[0]
                odom.pose.pose.position.y=pose[1]; odom.pose.pose.orientation.z=math.sin(pose[2]/2)
                odom.pose.pose.orientation.w=math.cos(pose[2]/2)
                odom.twist.twist.linear.x=command[0]; odom.twist.twist.linear.y=command[1]
                odom.twist.twist.angular.z=command[2]; odom_pub.publish(odom)
                scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id=frame('laser')
                scan.angle_min=-math.pi; scan.angle_max=math.pi; scan.angle_increment=2*math.pi/360
                scan.range_min=.05; scan.range_max=8.0; scan.scan_time=delta
                if obstacle:
                    walls=[(-3,-2.95,-3,3),(2.95,3,-3,3),(-3,3,-3,-2.95),(-3,3,2.95,3)]
                    ranges=[]
                    for index in range(361):
                        angle=pose[2]+scan.angle_min+index*scan.angle_increment
                        distance=min(ray_box_range(pose[0],pose[1],math.cos(angle),math.sin(angle),bounds)
                                     for bounds in [box]+walls)
                        ranges.append(distance if scan.range_min<=distance<=scan.range_max else float('inf'))
                    scan.ranges=ranges
                else:
                    scan.ranges=[float('inf')]*361
                scan_pub.publish(scan)
                if now>=map_due[0]:
                    grid.header.stamp=stamp; map_pub.publish(grid); map_due[0]=now+.5
                trace.append([now,pose[0],pose[1],pose[2],command[0],command[1],command[2],stamp.to_sec()])
                time.sleep(.025)
            child=start(namespace,['roslaunch',path])
            client=actionlib.SimpleActionClient(prefix+'/move_base',MoveBaseAction)
            deadline=monotonic()+18
            while monotonic()<deadline:
                if child.poll() is not None: raise RuntimeError(namespace+' exited during startup')
                pump(False)
                if client.wait_for_server(rospy.Duration(.01)) and monotonic()>deadline-15: break
            else: raise RuntimeError(namespace+' action server unavailable')
            for _ in range(30): pump(False)
            plugin=rospy.get_param(prefix+'/move_base/base_local_planner')
            if plugin!='dwa_local_planner/DWAPlannerROS': raise RuntimeError('Wrong local planner')
            limits=dict(x=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_x'),
                        y=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_y'),
                        translation=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_trans'),
                        angular=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_theta'))
            frequencies=dict(move_base=rospy.get_param(prefix+'/move_base/controller_frequency'),
                             dwa=rospy.get_param(prefix+'/move_base/DWAPlannerROS/controller_frequency'))
            # Record effective values before motion, including aborted runs.
            result.setdefault('effective_parameters',{})[name]=dict(controller_frequencies=frequencies,
                occdist_scale=rospy.get_param(prefix+'/move_base/DWAPlannerROS/occdist_scale'),
                forward_point_distance=rospy.get_param(prefix+'/move_base/DWAPlannerROS/forward_point_distance'))
            def goal(x,y):
                message=MoveBaseGoal(); message.target_pose.header.frame_id=frame('map')
                message.target_pose.header.stamp=rospy.Time.now()
                message.target_pose.pose.position.x=x; message.target_pose.pose.position.y=y
                message.target_pose.pose.orientation.z=math.sin(yaw/2)
                message.target_pose.pose.orientation.w=math.cos(yaw/2); return message
            goal_x=1.5 if obstacle else .8
            raw_footprint=None
            if obstacle:
                raw_footprint=rospy.get_param(prefix+'/move_base/local_costmap/footprint')
                collision_footprint[0]=parse_footprint(raw_footprint)
            sent=monotonic(); substep_start=len(integration_trace)
            client.send_goal(goal(goal_x,0))
            deadline=monotonic()+(45 if obstacle else 18)
            while monotonic()<deadline:
                if child.poll() is not None: raise RuntimeError(namespace+' exited during goal')
                pump()
                state=client.get_state()
                if state==GoalStatus.SUCCEEDED: break
                if state in (GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.LOST):
                    raise RuntimeError(namespace+' goal failed: '+client.get_goal_status_text())
            else: raise RuntimeError(namespace+' goal timeout at '+repr(pose))
            nonzero=[v for when,v in seen if when>=sent and math.hypot(v[0],v[1])>.02]
            bounded=bool(nonzero) and all(abs(v[0])<=abs(limits['x'])+.005 and
                abs(v[1])<=abs(limits['y'])+.005 and math.hypot(v[0],v[1])<=abs(limits['translation'])+.005
                and abs(v[2])<=abs(limits['angular'])+.005 for when,v in seen)
            route=[p for p in plans if len(p.poses)>=3 and p.header.frame_id==frame('map')]
            end_error=math.hypot(pose[0]-goal_x,pose[1])
            semantics=True
            means=[sum(v[i] for v in nonzero)/len(nonzero) if nonzero else 0 for i in (0,1)]
            if abs(yaw-math.pi/2)<.01:
                semantics=means[1]<-.02 and abs(means[1])>abs(means[0])
            obstacle_metrics={}
            if obstacle:
                goal_trace=integration_trace[substep_start:]
                # Accept the installed polygon, never invent a measured aircraft size.
                footprint=collision_footprint[0]
                clearance=min(footprint_box_clearance(sample[1:4],footprint,box) for sample in goal_trace)
                lateral=max(abs(sample[2]) for sample in goal_trace)
                planned_lateral=max(abs(p.pose.position.y) for path in route for p in path.poses) if route else 0
                traveled=sum(math.hypot(second[1]-first[1],second[2]-first[2])
                             for first,second in zip(goal_trace,goal_trace[1:]))
                # Both a curved global route and an executed detour are required.
                # A stopped planner or a straight-line crossing cannot pass.
                detoured=lateral>.6 and planned_lateral>.6 and traveled>3.15
                obstacle_metrics=dict(obstacle_bounds=list(box),start=[-1.5,0],goal=[1.5,0],
                    raw_footprint_parameter=raw_footprint,raw_footprint_type=type(raw_footprint).__name__,
                    synthetic_footprint=footprint,minimum_footprint_clearance=clearance,
                    required_footprint_clearance=.01,executed_lateral_excursion=lateral,
                    planned_lateral_excursion=planned_lateral,traveled_distance=traveled,
                    direct_distance=3.0,detoured=detoured,goal_elapsed=monotonic()-sent,
                    collision_check_timebase='all integration substeps <=25ms',
                    goal_timeout=45,synthetic_map_and_scan_obstacle=True)
                # Preserve trajectory and global paths even if this new acceptance fails.
                with open(directory+'/'+namespace+'-obstacle.json','w') as output:
                    json.dump(obstacle_metrics,output,indent=2,sort_keys=True)
                with open(directory+'/'+namespace+'-global-plans.json','w') as output:
                    json.dump([[[p.pose.position.x,p.pose.position.y] for p in path.poses] for path in route],output)
            # A second real goal, then action cancellation must publish zero.
            # Keep this goal far enough away to cancel an ACTIVE action; the
            # obstacle goal already ends at x=1.5, within tolerance of x=1.6.
            second_goal=(1.5,1.5) if obstacle else (1.6,0.0)
            second_sent=monotonic(); client.send_goal(goal(*second_goal))
            deadline=monotonic()+5
            while monotonic()<deadline:
                pump()
                if client.get_state()==GoalStatus.ACTIVE and any(when>second_sent and
                    math.hypot(v[0],v[1])>.02 for when,v in seen): break
            else: raise RuntimeError(namespace+' did not move for cancellation check')
            cancelled=monotonic(); client.cancel_goal(); deadline=monotonic()+3
            while monotonic()<deadline:
                pump()
                if client.get_state()==GoalStatus.PREEMPTED and any(when>=cancelled and
                    max(abs(x) for x in v)<1e-6 for when,v in seen): break
            else: raise RuntimeError(namespace+' cancellation did not produce zero velocity')
            timing_consistent=(timing['observed_elapsed_seconds']>0 and
                abs(timing['integration_error_seconds'])<=1e-6 and
                timing['maximum_integration_step']<=.025+1e-8)
            case=dict(name=name,passed=bounded and bool(route) and end_error<.15 and semantics and timing_consistent,
                      plugin=plugin,global_plans=len(route),velocity_samples=len(nonzero),
                      velocity_limits=limits,mean_body_velocity=means,initial_yaw=yaw,
                      controller_frequencies=frequencies,
                      reached_goal_error=end_error,cancelled_action_zero_velocity=True,
                      cancellation_goal=list(second_goal),cancelled_while_active=True,
                      timing_consistent=timing_consistent,timing=dict(timing),substep_collision=dict(collision))
            if obstacle:
                case.update(obstacle_metrics)
                case['passed']=case['passed'] and obstacle_metrics['detoured'] and obstacle_metrics['minimum_footprint_clearance']>=.01
            result['cases'].append(case)
            with open(directory+'/'+namespace+'-trace.json','w') as output: json.dump(trace,output)
            save_simulation(active_simulation[0])
            print(json.dumps(case)); sys.stdout.flush()
            result['exits'][namespace]=stop(child)
            for subscription in subscribers: subscription.unregister()
            odom_pub.unregister(); scan_pub.unregister(); map_pub.unregister()
            if not case['passed']: raise RuntimeError(namespace+' acceptance failed')

        if args.case in ('all','straight'): run_case('straight',0.0)
        if args.case in ('all','yaw90'): run_case('yaw90',math.pi/2)
        if args.case in ('all','obstacle'): run_case('obstacle',0.0,obstacle=True)
        topics=[topic for topic,kind in rospy.get_published_topics()]
        result['forbidden_topics_absent']=not any(topic.startswith('/mavros') or topic.startswith('/gpio') for topic in topics)
        rospy.signal_shutdown('Planner software probes completed')
    except Exception as error:
        result['error']=repr(error)
        raise
    finally:
        if active_trace[0] is not None:
            path,trace=active_trace[0]
            with open(path,'w') as output: json.dump(trace,output)
        if active_simulation[0] is not None: save_simulation(active_simulation[0])
        if active_evidence[0] is not None:
            namespace,plans,local_plans,costmaps=active_evidence[0]
            for kind,messages in (('global',plans),('local',local_plans)):
                with open(directory+'/'+namespace+'-'+kind+'-plans.json','w') as output:
                    json.dump([dict(stamp=message.header.stamp.to_sec(),frame=message.header.frame_id,
                        points=[[p.pose.position.x,p.pose.position.y] for p in message.poses])
                        for message in messages],output)
            for kind,message in costmaps.items():
                with open(directory+'/'+namespace+'-'+kind+'-costmap.json','w') as output:
                    json.dump(dict(stamp=message.header.stamp.to_sec(),frame=message.header.frame_id,
                        resolution=message.info.resolution,width=message.info.width,height=message.info.height,
                        origin=[message.info.origin.position.x,message.info.origin.position.y],
                        data=list(message.data)),output)
        for name,child in reversed(children):
            if name not in result['exits']: result['exits'][name]=stop(child)
        for stream in streams: stream.close()
        expected_cases=3 if args.case=='all' else 1
        result['passed']=len(result['cases'])==expected_cases and all(case['passed'] for case in result['cases']) and result.get('forbidden_topics_absent',False) and all(code==0 for code in result['exits'].values())
        with open(directory+'/dwa-planner-results.json','w') as output: json.dump(result,output,indent=2,sort_keys=True)
        print(json.dumps(result,indent=2,sort_keys=True))
    if not result['passed']: raise SystemExit(1)


if __name__=='__main__':
    main()
