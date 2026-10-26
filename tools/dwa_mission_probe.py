#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Exercise the real mission and RKNN with fake FCU/action endpoints only.

Usage, after sourcing catkin devel/setup.bash:
  python2 dwa_mission_probe.py OUTPUT_DIR --image /absolute/IMG_2766.jpg
    [--person-image /absolute/IMG_2706.jpg]

Owns a private localhost:11332 master. It never starts MAVROS, a servo node,
camera hardware or real flight services. Every armed/mode/pose sample is
synthetic; output commands are recorded by this test, never sent to hardware.
The NPU/model inference is real. Tests do not substitute cached detections.
"""
from __future__ import print_function
import argparse
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time


ANCHORS='10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326'
CASE_NAMES=['missing_model','dry_run','control_rotation_and_timeouts','preserved_descent_timeout_drop',
            'no_hazard_no_fire_drop','manual_takeover_latches_block','disarm_latches_block',
            'takeoff_timeout_blocks','old_image_header_blocks','expired_inference_source_blocks']


def stop(process):
    for signum,seconds in ((signal.SIGINT,7),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        if process.poll() is not None: return process.returncode
        try: os.killpg(process.pid,signum)
        except OSError:
            if process.poll() is None: raise
        deadline=time.time()+seconds
        while process.poll() is None and time.time()<deadline: time.sleep(.05)
    if process.poll() is None: raise RuntimeError('Owned mission child did not stop')
    return process.returncode


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--image',required=True)
    parser.add_argument('--model-dir')
    parser.add_argument('--skip-drop',action='store_true',help='Skip the separately reported simulated timeout/drop case')
    parser.add_argument('--person-image',help='Real person image for the no-hazard case; defaults to IMG_2706.jpg beside --image')
    parser.add_argument('--skip-no-hazard',action='store_true',help='Run the original four cases without the separate person-image regression')
    parser.add_argument('--only-case',choices=CASE_NAMES,help='Run one independent mission process for focused regression')
    args=parser.parse_args()
    selected_cases=[name for name in CASE_NAMES if (not args.only_case or args.only_case==name) and
                    (not args.skip_drop or name!='preserved_descent_timeout_drop') and
                    (not args.skip_no_hazard or name!='no_hazard_no_fire_drop')]
    if not selected_cases: parser.error('The selected case was also skipped')
    output=os.path.abspath(args.output)
    if not os.path.isdir(output): os.makedirs(output)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11332',ROS_IP='127.0.0.1',
                      ROS_HOME=output+'/ros-home',ROS_LOG_DIR=output+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    reservation=socket.socket(); reservation.bind(('127.0.0.1',11332)); reservation.close()
    master_log=open(output+'/master.log','w')
    master=subprocess.Popen(['roscore','-p','11332'],stdout=master_log,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
    owned=[]; active=[None]
    result=dict(cases=[],master='127.0.0.1:11332',all_fcu_state_synthetic=True,
                real_npu_inference=True,real_mavros_started=False,real_servos_started=False)
    try:
        deadline=time.time()+15
        while time.time()<deadline:
            sock=socket.socket()
            try: sock.connect(('127.0.0.1',11332)); break
            except socket.error: time.sleep(.1)
            finally: sock.close()
        else: raise RuntimeError('Private master unavailable')
        import rospy
        import rosgraph
        import actionlib
        import tf2_ros
        import cv2
        from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from mavros_msgs.msg import State,PositionTarget
        from mavros_msgs.srv import CommandBool,CommandBoolResponse,SetMode,SetModeResponse
        from move_base_msgs.msg import MoveBaseAction,MoveBaseResult
        rospy.init_node('dwa_mission_probe_harness',disable_signals=True)
        model_dir=args.model_dir or subprocess.check_output(['rospack','find','navigation_2d']).strip()+'/detect_mode/26'
        def fixture(path):
            source=cv2.imread(path)
            if source is None: raise RuntimeError('Real test image cannot be read: '+path)
            original_height,original_width=source.shape[:2]
            scale=min(1.0,1280.0/original_width,720.0/original_height)
            if scale<1.0:
                source=cv2.resize(source,(max(1,int(original_width*scale)),max(1,int(original_height*scale))),
                                  interpolation=cv2.INTER_AREA)
            metadata=dict(path=os.path.abspath(path),source_sha256=hashlib.sha256(open(path,'rb').read()).hexdigest(),
                          original_width=original_width,original_height=original_height,
                          published_width=source.shape[1],published_height=source.shape[0],
                          preserved_aspect_ratio=True,upscaled=False)
            return dict(metadata=metadata,frame=source,data=source.tostring())

        fixtures=dict(flammable=fixture(args.image))
        if 'no_hazard_no_fire_drop' in selected_cases:
            fixtures['person']=fixture(args.person_image or os.path.join(os.path.dirname(args.image),'IMG_2706.jpg'))
        if not os.path.isfile(model_dir+'/best.rknn'): raise RuntimeError('Real candidate model absent')
        result['model_sha256']=hashlib.sha256(open(model_dir+'/best.rknn','rb').read()).hexdigest()
        result['image_sha256']=fixtures['flammable']['metadata']['source_sha256']
        result['images']={key:value['metadata'] for key,value in fixtures.items()}
        state_pub=rospy.Publisher('/mavros/state',State,queue_size=5)
        pose_pub=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=5)
        image_pub=rospy.Publisher('/camera/image_raw',Image,queue_size=1)
        velocity_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
        broadcaster=tf2_ros.TransformBroadcaster()

        def mark(bucket,**values):
            case=active[0]
            if case is None: return
            values['elapsed']=time.time()-case['started']
            case[bucket].append(values)

        def mode(request):
            mark('services',service='set_mode',mode=request.custom_mode)
            if active[0] is not None: active[0]['mode']=request.custom_mode
            return SetModeResponse(mode_sent=True)

        def arm(request):
            mark('services',service='arming',armed=request.value)
            if active[0] is not None: active[0]['armed']=request.value
            return CommandBoolResponse(success=True,result=0)

        services=[rospy.Service('/mavros/set_mode',SetMode,mode),
                  rospy.Service('/mavros/cmd/arming',CommandBool,arm)]

        def raw(message):
            case=active[0]
            if case is None: return
            mark('controls',kind='raw',x=message.position.x,y=message.position.y,z=message.position.z,
                 vx=message.velocity.x,vy=message.velocity.y,vz=message.velocity.z,
                 mask=message.type_mask,frame=message.coordinate_frame,
                 phase=case['readiness'].get('phase'))
            if case['follow_altitude'] and not (message.type_mask & PositionTarget.IGNORE_PZ):
                target=message.position.z
                if not math.isnan(target) and not math.isinf(target) and -.2<=target<=3.0:
                    case['altitude_target']=target

        def position(message):
            mark('controls',kind='position',x=message.pose.position.x,y=message.pose.position.y,z=message.pose.position.z)

        def gpio(message): mark('gpio',command=message.data)
        def status(message):
            case=active[0]
            if case is None: return
            mark('statuses',value=message.data)
            case['status']=message.data
            if message.data=='TAKEOFF' and case['takeoff_at'] is None: case['takeoff_at']=time.time()

        def readiness(message):
            case=active[0]
            if case is None: return
            try: case['readiness']=json.loads(message.data)
            except ValueError: return
            mark('readiness_samples',value=case['readiness'])

        subscriptions=[rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,raw,queue_size=200),
            rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,position,queue_size=100),
            rospy.Subscriber('/gpio_cmd',String,gpio,queue_size=50),
            rospy.Subscriber('/mission/status',String,status,queue_size=100),
            rospy.Subscriber('/mission/readiness',String,readiness,queue_size=100)]
        server=actionlib.SimpleActionServer('/move_base',MoveBaseAction,auto_start=False)
        def new_goal():
            goal=server.accept_new_goal()
            mark('goals',frame=goal.target_pose.header.frame_id,x=goal.target_pose.pose.position.x,
                 y=goal.target_pose.pose.position.y)
            if active[0] is not None and active[0]['succeed_goals']:
                server.set_succeeded(MoveBaseResult(),'synthetic goal success')
        def preempt():
            mark('cancels',requested=True)
            if server.is_active(): server.set_preempted(MoveBaseResult(),'synthetic cancellation')
        server.register_goal_callback(new_goal); server.register_preempt_callback(preempt); server.start()

        def isolated_outputs():
            subscribers=rosgraph.Master(rospy.get_name()).getSystemState()[1]
            for topic,nodes in subscribers:
                if topic.startswith('/mavros/setpoint') or topic=='/gpio_cmd':
                    if any(name!=rospy.get_name() for name in nodes):
                        raise RuntimeError('Unexpected output consumer in private master: '+repr((topic,nodes)))

        def pump(duration=.05):
            case=active[0]; stamp=rospy.Time.now(); wall=time.time()
            message=State(); message.header.stamp=stamp; message.connected=True
            message.mode=case['mode']; message.armed=case['armed']; state_pub.publish(message)
            if case['follow_altitude']:
                change=case['altitude_target']-case['altitude']
                maximum=1.5*max(0.0,min(.2,wall-case['last_pump']))
                case['altitude']+=max(-maximum,min(maximum,change))
                mark('altitude_samples',z=case['altitude'],target_z=case['altitude_target'],
                     phase=case['readiness'].get('phase'))
            elif not case['freeze_altitude'] and case['takeoff_at'] is not None and wall-case['takeoff_at']>.6:
                case['altitude']=1.5
            case['last_pump']=wall
            if case['pose_enabled']:
                pose=PoseStamped(); pose.header.stamp=stamp; pose.header.frame_id='px4_local'
                pose.pose.position.x=10.0; pose.pose.position.y=20.0; pose.pose.position.z=case['altitude']
                pose.pose.orientation.z=math.sqrt(.5); pose.pose.orientation.w=math.sqrt(.5)
                pose_pub.publish(pose)
            transform=TransformStamped(); transform.header.stamp=stamp; transform.header.frame_id='map'
            transform.child_frame_id='base_link'; transform.transform.translation.x=2.0
            transform.transform.translation.y=3.0; transform.transform.rotation.w=1.0
            broadcaster.sendTransform(transform)
            if wall>=case['image_due']:
                selected=fixtures[case['image_kind']]; image=selected['frame']
                frame=Image(); frame.header.stamp=stamp-rospy.Duration(case['image_stamp_offset'])
                frame.header.seq=case['image_sequence']
                frame.header.frame_id='camera'; frame.height=image.shape[0]; frame.width=image.shape[1]
                frame.encoding='bgr8'; frame.step=image.shape[1]*3; frame.data=selected['data']
                image_pub.publish(frame); case['image_due']=wall+.15; case['image_sequence']+=1
            if case['planner_velocity'] is not None:
                twist=Twist(); twist.linear.x=case['planner_velocity'][0]; twist.linear.y=case['planner_velocity'][1]
                velocity_pub.publish(twist)
            time.sleep(duration)

        def until(predicate,seconds,label,child):
            deadline=time.time()+seconds
            while time.time()<deadline:
                if child.poll() is not None: raise RuntimeError(label+': mission exited '+str(child.returncode))
                pump()
                if predicate(): return
            raise RuntimeError(label+' timed out; status='+active[0]['status']+' ready='+repr(active[0]['readiness']))

        def hold(seconds,child):
            deadline=time.time()+seconds
            while time.time()<deadline:
                if child.poll() is not None: raise RuntimeError('Mission exited during hold')
                pump()

        def run_case(name,enable,missing=False,drop=False,no_hazard=False,authority_loss=None,
                     takeoff_fail=False,old_header=False,source_age=False):
            case=dict(name=name,started=time.time(),services=[],controls=[],gpio=[],goals=[],cancels=[],
                      statuses=[],readiness_samples=[],readiness={},status='',mode='MANUAL',armed=False,
                      altitude=0.0,takeoff_at=None,pose_enabled=True,planner_velocity=None,
                      image_due=0.0,image_sequence=0,succeed_goals=drop or no_hazard,
                      image_kind='person' if no_hazard else 'flammable',follow_altitude=no_hazard,
                      altitude_target=0.0,last_pump=time.time(),altitude_samples=[],freeze_altitude=takeoff_fail,
                      image_stamp_offset=5.0 if old_header else (.75 if source_age else 0.0))
            active[0]=case
            params=dict(model_path=model_dir,model_file='missing-do-not-create.rknn' if missing else 'best.rknn',
                labels_file='labels.txt',backend='rknn',input_size=416,yolo_style='v5-interleaved',
                yolo_anchors=ANCHORS,yolo_strides='8,16,32',input_image_topic='/camera/image_raw',
                confidence_threshold=.5,nms_iou=.45,frame_timeout=1.0,inference_timeout=3.0,
                publish_rate=2.0,publish_scale=.5,enable_actuation=enable,auto_start=True,
                target_hazard='flammable',target_person='person',field_yaw_deg=0.0,enforce_safe_field=False,
                pose_timeout=.4,state_timeout=1.0,tf_timeout=.4,planner_timeout=.35,alignment_tolerance=.2,
                hazard_route_num=1,hazard_detect_num=1 if drop else 0,hazard_point0_x=2.0,hazard_point0_y=3.0,
                person_route_num=0,return_route_num=0,fly_height=1.5,detect_height=1.5,drop_height=.6,
                settle_time=.15,drop_settle_time=.15,detect_times=1,detect_timeout=12.0,
                detect_confirmation_window=6.0,max_center_offset=10.0,drop_align_mode='fixed',goal_timeout=30.0)
            if no_hazard:
                # Both hazard cards must time out on real person inference before
                # the preserved stage-1 person survey can identify the person.
                params.update(hazard_route_num=2,hazard_detect_num=2,hazard_point1_x=2.2,hazard_point1_y=3.0,
                    person_route_num=1,person_detect_num=1,person_point0_x=2.4,person_point0_y=3.0,
                    stage=1,rescue_drop_enabled=False,detect_height=1.2,detect_timeout=3.0)
            if takeoff_fail: params['takeoff_timeout']=1.0
            if source_age: params['inference_timeout']=1.1
            node='mission_probe_'+name
            case['parameters']=params
            rospy.set_param('/'+node,params)
            isolated_outputs()
            stream=open(output+'/'+name+'.log','w')
            child=subprocess.Popen(['rosrun','navigation_2d','shoot_with_move_base_node','__name:='+node],
                                   stdout=stream,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
            owned.append(child)
            summary=dict(name=name,passed=False,all_fcu_state_synthetic=True)
            try:
                if missing:
                    deadline=time.time()+15
                    while child.poll() is None and time.time()<deadline: pump()
                    if child.poll() is None: raise RuntimeError('Missing model did not terminate')
                    summary.update(exit_code=child.returncode,controls=len(case['controls']),
                                   services=len(case['services']),gpio=len(case['gpio']))
                    summary['passed']=child.returncode!=0 and not case['controls'] and not case['services'] and not case['gpio']
                elif old_header or source_age:
                    hold(6.0,child)
                    observed_ready=any(x['value'].get('inference_ready',False) for x in case['readiness_samples'])
                    frame_seen=any(x['value'].get('frame_fresh',False) for x in case['readiness_samples'])
                    expected='BLOCKED_CAMERA_STALE' if old_header else 'BLOCKED_MODEL_INFERENCE'
                    blocked=any(x['value']==expected for x in case['statuses'])
                    stream.flush()
                    with open(output+'/'+name+'.log','r') as source: case_log=source.read()
                    source_rejection='completed inference rejected: source stale' in case_log if source_age else True
                    summary.update(image_header_offset_seconds=case['image_stamp_offset'],
                        frame_fresh_observed=frame_seen,inference_ready_observed=observed_ready,
                        expected_block_status=expected,source_age_rejection_logged=source_rejection,
                        services=len(case['services']),gpio=len(case['gpio']),goals=len(case['goals']))
                    summary['passed']=(not observed_ready and blocked and not case['services'] and
                        not case['gpio'] and not case['goals'] and bool(case['readiness_samples']) and
                        (frame_seen and source_rejection if source_age else not frame_seen))
                else:
                    until(lambda:case['readiness'].get('inference_ready',False),30,'real RKNN warmup',child)
                    if not enable:
                        until(lambda:case['status']=='DRY_RUN_READY',5,'dry-run readiness',child)
                        hold(1.0,child)
                        summary.update(controls=len(case['controls']),services=len(case['services']),
                                       gpio=len(case['gpio']),readiness=case['readiness'])
                        summary['passed']=not case['controls'] and not case['services'] and not case['gpio']
                    elif takeoff_fail:
                        until(lambda:case['readiness'].get('mission_aborted',False),10,'takeoff timeout block',child)
                        aborted_at=time.time()-case['started']; hold(.8,child)
                        case['mode']='OFFBOARD'; case['armed']=True
                        hold(.6,child)
                        late=[x for x in case['controls'] if x['elapsed']>aborted_at+.2]
                        reason=case['readiness'].get('abort_reason')
                        blocked=any(x['value']=='BLOCKED_TAKEOFF_TIMEOUT' for x in case['statuses'])
                        summary.update(mission_aborted=case['readiness'].get('mission_aborted',False),
                            abort_reason=reason,simulated_altitude=case['altitude'],
                            navigation_goals=len(case['goals']),gpio=len(case['gpio']),
                            post_abort_control_samples=len(late),takeoff_timeout_seconds=1.0)
                        summary['passed']=(blocked and reason=='TAKEOFF_TIMEOUT' and
                            case['readiness'].get('mission_aborted',False) and case['altitude']==0.0 and
                            not case['goals'] and not case['gpio'] and not late)
                    elif authority_loss:
                        until(lambda:case['readiness'].get('phase')==2 and len(case['goals'])>0,12,
                              'active synthetic flight before authority loss',child)
                        case['planner_velocity']=(.2,0.0); hold(.5,child)
                        fault_at=time.time()-case['started']
                        if authority_loss=='mode': case['mode']='MANUAL'
                        else: case['armed']=False
                        until(lambda:case['readiness'].get('mission_aborted',False),3,'flight authority loss',child)
                        aborted_at=time.time()-case['started']; hold(.7,child)
                        case['mode']='OFFBOARD'; case['armed']=True
                        hold(.7,child)
                        cutoff=aborted_at+.2
                        late_controls=[x for x in case['controls'] if x['elapsed']>cutoff]
                        late_services=[x for x in case['services'] if x['elapsed']>cutoff]
                        late_goals=[x for x in case['goals'] if x['elapsed']>cutoff]
                        late_gpio=[x for x in case['gpio'] if x['elapsed']>cutoff]
                        blocked=any(x['value']=='BLOCKED_FLIGHT_AUTHORITY_LOST' for x in case['statuses'])
                        summary.update(fault=authority_loss,mission_aborted=case['readiness'].get('mission_aborted',False),
                            abort_reason=case['readiness'].get('abort_reason'),
                            fault_to_abort_seconds=aborted_at-fault_at,goal_cancellations=len(case['cancels']),
                            post_abort_controls=len(late_controls),post_abort_services=len(late_services),
                            post_abort_goals=len(late_goals),post_abort_gpio=len(late_gpio),
                            restored_fcu_did_not_resume=case['readiness'].get('mission_aborted',False))
                        summary['passed']=(blocked and case['readiness'].get('mission_aborted',False) and
                            case['readiness'].get('abort_reason')=='FLIGHT_AUTHORITY_LOST' and bool(case['cancels']) and
                            not late_controls and not late_services and not late_goals and not late_gpio)
                    elif no_hazard:
                        until(lambda:case['readiness'].get('phase') in (8,9) and
                              any(x['value'].startswith('PERSON_FOUND person') for x in case['statuses']),
                              45,'two hazard timeouts followed by a real person detection',child)
                        hold(.6,child)
                        detected_points=sorted(set(x['value'].split()[1] for x in case['statuses']
                            if x['value'].startswith('DETECT hazard')))
                        absent_status=any(x['value']=='HAZARD_NOT_FOUND_NO_DROP' for x in case['statuses'])
                        absent_readiness=any(x['value'].get('hazard_detection')=='not_found'
                            for x in case['readiness_samples'])
                        person_started=any(x['value']=='TASK_PERSON' for x in case['statuses'])
                        person_found=any(x['value'].startswith('PERSON_FOUND person') for x in case['statuses'])
                        fire_drops=[x for x in case['gpio'] if x['command']=='DROP_FIRE']
                        fire_statuses=[x for x in case['statuses'] if x['value'] in ('DESCEND_FIRE','DROP_FIRE')]
                        summary.update(image_fixture='person',target_hazard='flammable',
                            hazard_detect_points=detected_points,expected_detect_points=['hazard0','hazard1'],
                            hazard_not_found_status=absent_status,hazard_not_found_readiness=absent_readiness,
                            person_survey_entered=person_started,real_person_detection_confirmed=person_found,
                            move_base_goals=len(case['goals']),drop_fire_requests=len(fire_drops),
                            fire_drop_statuses=len(fire_statuses),
                            synthetic_altitude_follows_control=True,detect_timeout_seconds=params['detect_timeout'])
                        summary['passed']=(detected_points==['hazard0','hazard1'] and
                            (absent_status or absent_readiness) and person_started and person_found and
                            len(case['goals'])>=3 and not fire_drops and not fire_statuses)
                    elif drop:
                        until(lambda:any(x['command']=='DROP_FIRE' for x in case['gpio']),35,
                              'real detection plus simulated DESCEND timeout',child)
                        found=[x for x in case['statuses'] if x['value'].startswith('HAZARD_FOUND flammable')]
                        descend=[x for x in case['statuses'] if x['value']=='DESCEND_FIRE']
                        drops=[x for x in case['gpio'] if x['command']=='DROP_FIRE']
                        elapsed=drops[0]['elapsed']-descend[0]['elapsed'] if descend else None
                        summary.update(real_flammable_detection_confirmed=bool(found),
                            descent_to_drop_seconds=elapsed,simulated_altitude=case['altitude'],
                            requested_drop_altitude=.6,drop_fire_requests=len(drops))
                        summary['passed']=bool(found) and elapsed is not None and elapsed>=9.9 and case['altitude']==1.5 and len(drops)==1
                    else:
                        until(lambda:case['readiness'].get('phase')==2 and len(case['goals'])>0,12,'synthetic takeoff and active goal',child)
                        takeoff_start=next(x['elapsed'] for x in case['statuses'] if x['value']=='TAKEOFF')
                        takeoff_end=next(x['elapsed'] for x in case['statuses'] if x['value']=='TASK_HAZARD')
                        takeoff=[x for x in case['controls'] if x['kind']=='raw' and
                            takeoff_start<=x['elapsed']<takeoff_end and
                            not (x['mask'] & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY))]
                        origin_ok=len(takeoff)>=5 and all(abs(x['x']-10)<.01 and abs(x['y']-20)<.01 for x in takeoff)
                        started=time.time()-case['started']; case['planner_velocity']=(.2,0.0); hold(.9,child)
                        straight=[x for x in case['controls'] if x['elapsed']>started+.2 and x['kind']=='raw' and x.get('phase')==2]
                        rotated=len(straight)>=10 and all(abs(x['vx'])<.015 and abs(x['vy']-.2)<.015 for x in straight)
                        started=time.time()-case['started']; case['planner_velocity']=(1.0,1.0); hold(.8,child)
                        limited=[x for x in case['controls'] if x['elapsed']>started+.2 and x['kind']=='raw' and x.get('phase')==2]
                        norm_ok=len(limited)>=10 and all(math.hypot(x['vx'],x['vy'])<=.3501 for x in limited) and any(math.hypot(x['vx'],x['vy'])>.34 for x in limited)
                        stopped=time.time()-case['started']; case['planner_velocity']=None; hold(.95,child)
                        zeros=[x for x in case['controls'] if x['elapsed']>stopped+.6 and x['kind']=='raw']
                        zero_ok=len(zeros)>=5 and all(abs(x['vx'])<1e-6 and abs(x['vy'])<1e-6 for x in zeros)
                        stale=time.time()-case['started']; case['pose_enabled']=False; hold(1.0,child)
                        old_controls=[x for x in case['controls'] if x['elapsed']>stale+.7]
                        stale_ok=not old_controls and any(x['value']=='BLOCKED_POSE_STALE' for x in case['statuses'])
                        summary.update(takeoff_kept_nonzero_xy=origin_ok,yaw90_body_to_local=rotated,
                            planar_norm_limit=norm_ok,planner_timeout_zero=zero_ok,stale_pose_stops_controls=stale_ok,
                            fake_service_requests=len(case['services']),takeoff_control_samples=len(takeoff),
                            takeoff_status_start=takeoff_start,takeoff_status_end=takeoff_end,
                            rotated_control_samples=len(straight))
                        summary['passed']=origin_ok and rotated and norm_ok and zero_ok and stale_ok
                if not summary['passed']: raise RuntimeError('Acceptance failed: '+name+' '+repr(summary))
            except Exception as error:
                summary['error']=repr(error)
                raise
            finally:
                summary['exit_code']=stop(child)
                stream.close()
                if not missing and summary['exit_code']!=0: summary['passed']=False
                # Keep all observed commands, including fake arming and drop, for audit.
                active[0]=None
                with open(output+'/'+name+'-events.json','w') as target: json.dump(case,target,indent=2,sort_keys=True)
                result['cases'].append(summary)
                print(json.dumps(summary,sort_keys=True)); sys.stdout.flush()

        specifications=dict(missing_model=dict(enable=True,missing=True),dry_run=dict(enable=False),
            control_rotation_and_timeouts=dict(enable=True),
            preserved_descent_timeout_drop=dict(enable=True,drop=True),
            no_hazard_no_fire_drop=dict(enable=True,no_hazard=True),
            manual_takeover_latches_block=dict(enable=True,authority_loss='mode'),
            disarm_latches_block=dict(enable=True,authority_loss='armed'),
            takeoff_timeout_blocks=dict(enable=True,takeoff_fail=True),
            old_image_header_blocks=dict(enable=True,old_header=True),
            expired_inference_source_blocks=dict(enable=True,source_age=True))
        for name in selected_cases: run_case(name,**specifications[name])
        isolated_outputs()
        result['forbidden_hardware_consumers_absent']=True
        rospy.signal_shutdown('Synthetic mission checks complete')
    except Exception as error:
        result['error']=repr(error)
        raise
    finally:
        # rospy/actionlib own worker threads even after every child has exited.
        # Signal shutdown on failures too, before cleaning our process groups.
        if 'rospy' in locals():
            try: rospy.signal_shutdown('Mission probe completed or failed')
            except Exception as error: result['shutdown_error']=repr(error)
        for child in reversed(owned):
            if child.poll() is None: stop(child)
        result['master_exit']=stop(master); master_log.close()
        expected_cases=len(selected_cases)
        result['passed']=len(result['cases'])==expected_cases and all(x['passed'] for x in result['cases']) and result.get('forbidden_hardware_consumers_absent',False) and result['master_exit']==0
        with open(output+'/dwa-mission-results.json','w') as target: json.dump(result,target,indent=2,sort_keys=True)
        print(json.dumps(result,indent=2,sort_keys=True))
    if not result['passed']: raise SystemExit(1)


if __name__=='__main__':
    main()
