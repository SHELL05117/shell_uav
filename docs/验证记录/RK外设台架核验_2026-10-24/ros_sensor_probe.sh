#!/usr/bin/env bash
# Bounded camera/lidar verification. Never starts MAVROS, mission or servo nodes.
set -eo pipefail
ulimit -c 0
source /home/jcfh/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11324
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME ROS_NAMESPACE
export PYTHONDONTWRITEBYTECODE=1
probe_dir=$(mktemp -d /tmp/rk-sensor-probe-20261024.XXXXXX)
printf 'PROBE_DIR=%s\n' "$probe_dir"
export ROS_HOME="$probe_dir/ros-home"
export ROS_LOG_DIR="$probe_dir/ros-logs"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
probe_pids=()
cleanup() {
    for probe_pid in "${probe_pids[@]}"; do kill -INT -- "-$probe_pid" 2>/dev/null || true; done
    for probe_pid in "${probe_pids[@]}"; do
        for unused in {1..40}; do
            kill -0 "$probe_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM -- "-$probe_pid" 2>/dev/null || true
        probe_exit=0
        wait "$probe_pid" 2>/dev/null || probe_exit=$?
        printf 'CHILD_EXIT pid=%s status=%s\n' "$probe_pid" "$probe_exit"
    done
    # Restore the camera mode observed before this check; leave no capture process.
    v4l2-ctl -d /dev/video10 --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=60 >"$probe_dir/camera-restore.log" 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python2 - <<'PY'
import socket
s = socket.socket()
s.bind(('127.0.0.1', 11324))
s.close()
PY
setsid roscore -p 11324 >"$probe_dir/roscore.log" 2>&1 &
probe_pids+=("$!")
python2 - <<'PY'
import socket, time
deadline = time.time() + 15
while time.time() < deadline:
    s = socket.socket()
    try:
        s.connect(('127.0.0.1', 11324))
        break
    except socket.error:
        time.sleep(.1)
    finally:
        s.close()
else:
    raise RuntimeError('Master startup timeout')
PY
setsid rosrun rplidar_ros rplidarNode __name:=bench_lidar _serial_port:=/dev/ttyUSB0 _serial_baudrate:=1000000 _frame_id:=laser _inverted:=false _angle_compensate:=true _scan_frequency:=10.0 >"$probe_dir/lidar.log" 2>&1 &
probe_pids+=("$!")
setsid rosrun usb_cam usb_cam_node __name:=bench_camera __ns:=/bench_camera _video_device:=/dev/video10 _image_width:=1280 _image_height:=720 _pixel_format:=mjpeg _io_method:=mmap _framerate:=30 _camera_frame_id:=camera >"$probe_dir/camera.log" 2>&1 &
probe_pids+=("$!")
python2 - "$probe_dir" <<'PY'
from __future__ import print_function
import json, math, os, sys, time
import rospy
from sensor_msgs.msg import Image, LaserScan, CameraInfo
records = {'scan': [], 'image': [], 'camera_info': []}
samples = {}
def scan_cb(m):
    valid = [x for x in m.ranges if not math.isnan(x) and not math.isinf(x) and m.range_min <= x <= m.range_max]
    records['scan'].append((time.time(), m.header.stamp.to_sec()))
    samples['scan'] = {'frame': m.header.frame_id, 'points': len(m.ranges), 'valid_points': len(valid), 'closest_m': min(valid) if valid else None, 'furthest_m': max(valid) if valid else None, 'scan_time_s': m.scan_time, 'range_min': m.range_min, 'range_max': m.range_max}
def image_cb(m):
    records['image'].append((time.time(), m.header.stamp.to_sec()))
    samples['image'] = {'frame': m.header.frame_id, 'width': m.width, 'height': m.height, 'encoding': m.encoding, 'step': m.step, 'data_bytes': len(m.data), 'size_consistent': len(m.data) == m.step * m.height}
def info_cb(m):
    records['camera_info'].append((time.time(), m.header.stamp.to_sec()))
    samples['camera_info'] = {'width': m.width, 'height': m.height, 'K': list(m.K), 'D': list(m.D), 'calibrated': bool(m.K[0])}
rospy.init_node('bench_sensor_observer', disable_signals=True)
subs = [rospy.Subscriber('/scan', LaserScan, scan_cb, queue_size=50)]
observed_topics = {}
deadline = time.time() + 18
while time.time() < deadline and not rospy.is_shutdown():
    if 'image' not in observed_topics or 'camera_info' not in observed_topics:
        for topic, message_type in rospy.get_published_topics():
            if message_type == 'sensor_msgs/Image' and 'image' not in observed_topics:
                observed_topics['image'] = topic
                subs.append(rospy.Subscriber(topic, Image, image_cb, queue_size=3, buff_size=8000000))
            if message_type == 'sensor_msgs/CameraInfo' and 'camera_info' not in observed_topics:
                observed_topics['camera_info'] = topic
                subs.append(rospy.Subscriber(topic, CameraInfo, info_cb, queue_size=5))
    time.sleep(.1)
result = {'samples': samples, 'streams': {}, 'observed_topics': observed_topics}
for name, items in records.items():
    n = len(items)
    result['streams'][name] = {'count': n, 'received_hz': (n-1)/(items[-1][0]-items[0][0]) if n > 1 else None, 'max_gap_s': max([b[0]-a[0] for a,b in zip(items,items[1:])] or [0]), 'stamp_strictly_increasing': all(b[1] > a[1] for a,b in zip(items,items[1:])), 'latest_stamp_age_s': time.time()-items[-1][1] if n else None}
result['passed'] = (len(records['scan']) >= 30 and len(records['image']) >= 60 and samples.get('scan',{}).get('valid_points',0) > 0 and samples.get('image',{}).get('size_consistent',False))
text = json.dumps(result, indent=2, sort_keys=True)
print(text)
with open(os.path.join(sys.argv[1], 'sensor-results.json'), 'w') as f:
    f.write(text+'\n')
rospy.signal_shutdown('Bounded observation finished')
sys.exit(0 if result['passed'] else 1)
PY
