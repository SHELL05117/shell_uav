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
import time


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
        deadline=time.time()+seconds
        while process.poll() is None and time.time()<deadline:
            time.sleep(.05)
    if process.poll() is None: raise RuntimeError('Owned child did not stop after SIGKILL')
    return process.returncode


def launch_xml(namespace,forward_point_distance=None):
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
      <param name="controller_frequency" value="10.0" />
      <param name="DWAPlannerROS/controller_frequency" value="10.0" />
      <param name="clearing_rotation_allowed" value="false" />
      <param name="recovery_behavior_enabled" value="false" />
      <remap from="cmd_vel" to="/%s/cmd_vel" />
      <remap from="odom" to="/%s/odom" />
      <remap from="map" to="/%s/map" />
    </node>
  </group>
</launch>''' % tuple([namespace]*15)
    if forward_point_distance is not None:
        xml=xml.replace('      <param name="controller_frequency"',
            '      <param name="DWAPlannerROS/forward_point_distance" value="%s" />\n      <param name="controller_frequency"' % forward_point_distance)
    return xml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--case',choices=('all','straight','yaw90','obstacle'),default='all')
    parser.add_argument('--forward-point-distance',type=float,default=None,
                        help='Diagnostic override only; production YAML is otherwise used unchanged')
    args=parser.parse_args()
    directory=os.path.abspath(args.output)
    if not os.path.isdir(directory): os.makedirs(directory)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11331',ROS_IP='127.0.0.1',
                      ROS_HOME=directory+'/ros-home',ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    reservation=socket.socket(); reservation.bind(('127.0.0.1',11331)); reservation.close()
    children=[]; streams=[]; active_trace=[None]; active_evidence=[None]
    def start(name,command):
        stream=open(directory+'/'+name+'.log','w'); streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
        children.append((name,child)); return child
    result=dict(cases=[],exits={},scope='synthetic software planner execution only',
                aircraft_footprint_measured=False,master='127.0.0.1:11331',
                requested_case=args.case,forward_point_distance_override=args.forward_point_distance)
    master=start('master',['roscore','-p','11331'])
    try:
        deadline=time.time()+15
        while time.time()<deadline:
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
            with open(path,'w') as output: output.write(launch_xml(namespace,args.forward_point_distance))
            command=[0.0,0.0,0.0]
            seen=[]; plans=[]; local_plans=[]; costmaps={}
            def velocity(message):
                values=(message.linear.x,message.linear.y,message.angular.z)
                command[:]=values
                seen.append((time.time(),values))
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
            previous=[time.time()]; map_due=[0.0]
            trace=[]
            active_trace[0]=(directory+'/'+namespace+'-trace.json',trace)
            def pump(integrate=True):
                now=time.time(); delta=min(.05,max(0,now-previous[0])); previous[0]=now
                if integrate:
                    c=math.cos(pose[2]); s=math.sin(pose[2])
                    pose[0]+=(c*command[0]-s*command[1])*delta
                    pose[1]+=(s*command[0]+c*command[1])*delta
                    pose[2]+=command[2]*delta
                stamp=rospy.Time.now()
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
                scan.range_min=.05; scan.range_max=8.0; scan.scan_time=.025
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
                trace.append([now,pose[0],pose[1],pose[2],command[0],command[1],command[2]])
                time.sleep(.025)
            child=start(namespace,['roslaunch',path])
            client=actionlib.SimpleActionClient(prefix+'/move_base',MoveBaseAction)
            deadline=time.time()+18
            while time.time()<deadline:
                if child.poll() is not None: raise RuntimeError(namespace+' exited during startup')
                pump(False)
                if client.wait_for_server(rospy.Duration(.01)) and time.time()>deadline-15: break
            else: raise RuntimeError(namespace+' action server unavailable')
            for _ in range(30): pump(False)
            plugin=rospy.get_param(prefix+'/move_base/base_local_planner')
            if plugin!='dwa_local_planner/DWAPlannerROS': raise RuntimeError('Wrong local planner')
            limits=dict(x=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_x'),
                        y=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_y'),
                        translation=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_trans'),
                        angular=rospy.get_param(prefix+'/move_base/DWAPlannerROS/max_vel_theta'))
            def goal(x,y):
                message=MoveBaseGoal(); message.target_pose.header.frame_id=frame('map')
                message.target_pose.header.stamp=rospy.Time.now()
                message.target_pose.pose.position.x=x; message.target_pose.pose.position.y=y
                message.target_pose.pose.orientation.z=math.sin(yaw/2)
                message.target_pose.pose.orientation.w=math.cos(yaw/2); return message
            goal_x=1.5 if obstacle else .8
            sent=time.time(); trace_start=len(trace); client.send_goal(goal(goal_x,0))
            deadline=time.time()+(45 if obstacle else 18)
            while time.time()<deadline:
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
                goal_trace=trace[trace_start:]
                raw_footprint=rospy.get_param(prefix+'/move_base/local_costmap/footprint')
                # Accept the installed polygon, never invent a measured aircraft size.
                footprint=parse_footprint(raw_footprint)
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
                    direct_distance=3.0,detoured=detoured,goal_elapsed=time.time()-sent,
                    goal_timeout=45,synthetic_map_and_scan_obstacle=True)
                # Preserve trajectory and global paths even if this new acceptance fails.
                with open(directory+'/'+namespace+'-obstacle.json','w') as output:
                    json.dump(obstacle_metrics,output,indent=2,sort_keys=True)
                with open(directory+'/'+namespace+'-global-plans.json','w') as output:
                    json.dump([[[p.pose.position.x,p.pose.position.y] for p in path.poses] for path in route],output)
            # A second real goal, then action cancellation must publish zero.
            second_sent=time.time(); client.send_goal(goal(1.6,0))
            deadline=time.time()+5
            while time.time()<deadline:
                pump()
                if any(when>second_sent and math.hypot(v[0],v[1])>.02 for when,v in seen): break
            else: raise RuntimeError(namespace+' did not move for cancellation check')
            cancelled=time.time(); client.cancel_goal(); deadline=time.time()+3
            while time.time()<deadline:
                pump()
                if client.get_state()==GoalStatus.PREEMPTED and any(when>=cancelled and
                    max(abs(x) for x in v)<1e-6 for when,v in seen): break
            else: raise RuntimeError(namespace+' cancellation did not produce zero velocity')
            case=dict(name=name,passed=bounded and bool(route) and end_error<.15 and semantics,
                      plugin=plugin,global_plans=len(route),velocity_samples=len(nonzero),
                      velocity_limits=limits,mean_body_velocity=means,initial_yaw=yaw,
                      reached_goal_error=end_error,cancelled_action_zero_velocity=True)
            if obstacle:
                case.update(obstacle_metrics)
                case['passed']=case['passed'] and obstacle_metrics['detoured'] and obstacle_metrics['minimum_footprint_clearance']>=.01
            result['cases'].append(case)
            with open(directory+'/'+namespace+'-trace.json','w') as output: json.dump(trace,output)
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
