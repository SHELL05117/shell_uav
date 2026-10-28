from __future__ import print_function
import os,sys,time,json,signal,subprocess,socket,math,glob
root='/tmp/usb-readonly-20261028';os.makedirs(root) if not os.path.isdir(root) else None
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11342',ROS_IP='127.0.0.1',ROS_LOG_DIR=root+'/ros-logs')
os.environ.pop('ROS_HOSTNAME',None)
s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('127.0.0.1',11342));s.close()
children=[];streams=[]
def start(name,args):
 f=open(root+'/'+name+'.log','w');streams.append(f)
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,preexec_fn=os.setsid);children.append(p);return p
def halt(p):
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 deadline=time.time()+12
 while p.poll() is None and time.time()<deadline:time.sleep(.1)
 if p.poll() is None:os.killpg(p.pid,signal.SIGKILL)
 p.wait()
result=dict(hardware=True,range_disabled=True,actuation=False,auto_start=False,real_servos=False)
try:
 start('master',['roscore','-p','11342']);time.sleep(3)
 # Fixed read-only MAVROS plugin list: no cmd, set-mode or setpoint plugins.
 start('fcu',['roslaunch','location','rk_mavros_vision.launch'])
 start('lidar',['roslaunch','location','rplidar.launch','serial_port:='+glob.glob('/dev/serial/by-id/*CP2102*')[0]])
 start('laser_tf',['rosrun','tf','static_transform_publisher','0','0','0','3.14159','0','0','base_link','laser_link','30'])
 start('slam',['roslaunch','location','cartographer_2d.launch'])
 start('location',['rosrun','location','location_node','__name:=location_node','_use_tf_mini:=false'])
 start('camera',['roslaunch','usb_cam','rk_usb_camera.launch','mjpeg_backend:=mpp_rga','video_device:='+glob.glob('/dev/v4l/by-id/*Camera*video-index0')[0]])
 start('navigation',['roslaunch','navigation_2d','navigation_2d.launch','detect_backend:=rknn','target_hazard:=flammable','enable_actuation:=false','auto_start:=false','use_real_servos:=false'])
 import rospy,rosgraph,tf2_ros
 from sensor_msgs.msg import Imu,LaserScan,Image
 from geometry_msgs.msg import PoseStamped
 from mavros_msgs.msg import State,PositionTarget
 from std_msgs.msg import String
 from nav_msgs.msg import OccupancyGrid
 rospy.init_node('usb_readonly_observer',disable_signals=True)
 values={};counts={};stamps={};gaps={};subs=[]
 def receive(key):
  def cb(msg):
   now=time.time();counts[key]=counts.get(key,0)+1
   if key in stamps:gaps[key]=max(gaps.get(key,0),now-stamps[key])
   stamps[key]=now
   if key=='state':
    if msg.armed:result['armed_seen']=True
    values[key]=dict(connected=msg.connected,armed=msg.armed,mode=msg.mode)
   elif hasattr(msg,'data') and key!='camera':values[key]=msg.data
   elif key=='map':values[key]=[msg.info.width,msg.info.height]
  return cb
 for key,topic,kind in [('state','/mavros/state',State),('imu','/mavros/imu/data',Imu),('scan','/scan',LaserScan),('camera','/camera/image_raw',Image),('map','/map',OccupancyGrid),('vision','/mavros/vision_pose/pose',PoseStamped),('mission','/mission/status',String),('location','/location_node/status',String),('setpoint','/mavros/setpoint_raw/local',PositionTarget),('gpio','/gpio_cmd',String)]:
  subs.append(rospy.Subscriber(topic,kind,receive(key),queue_size=1,buff_size=8388608))
 buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf)
 time.sleep(25)
 counts.clear();stamps.clear();gaps.clear();ages=[];cpu=[]
 def stat():
  with open('/proc/stat') as f:v=[int(x) for x in f.readline().split()[1:]]
  return sum(v),v[3]+v[4]
 prev=stat();start_time=time.time()
 for i in range(120):
  time.sleep(.5);cur=stat();cpu.append(100.*(1.-float(cur[1]-prev[1])/(cur[0]-prev[0])));prev=cur
  try:
   tr=buf.lookup_transform('map','base_link',rospy.Time(0));ages.append((rospy.Time.now()-tr.header.stamp).to_sec())
  except Exception:ages.append(None)
  if result.get('armed_seen'):raise RuntimeError('FCU armed; stop owned diagnostic sources')
 elapsed=time.time()-start_time
 state=rosgraph.Master(rospy.get_name()).getSystemState()
 nodes=sorted(set(n for group in state for topic,ns in group for n in ns))
 result.update(seconds=elapsed,counts=counts,max_receive_gap_s=gaps,latest=values,nodes=nodes,
  cpu_system_mean_percent=sum(cpu)/len(cpu),tf_age_samples=ages,
  services=rosgraph.Master(rospy.get_name()).getSystemState()[2],
  no_servo_node=not any('servo' in n for n in nodes),
  no_setpoints_or_gpio=counts.get('setpoint',0)==counts.get('gpio',0)==0)
 result['passed']=bool(values.get('state',{}).get('connected') and not result.get('armed_seen') and result['no_servo_node'] and result['no_setpoints_or_gpio'] and counts.get('imu',0)>1000 and counts.get('scan',0)>300 and counts.get('camera',0)>1000 and counts.get('vision',0)==0 and values.get('location')=='HEIGHT_DISABLED')
 for sub in subs:sub.unregister()
 listener.unregister()
 with open(root+'/result.json','w') as f:json.dump(result,f,indent=2)
 print(json.dumps({k:result[k] for k in ('passed','latest','counts','cpu_system_mean_percent')}))
finally:
 for p in reversed(children):halt(p)
 for f in streams:f.close()
