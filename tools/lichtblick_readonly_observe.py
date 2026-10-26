#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Short read-only evidence collection for the Lichtblick ground session."""
from __future__ import print_function
import json
import time
import rospy
from sensor_msgs.msg import LaserScan, CompressedImage, Imu
from nav_msgs.msg import OccupancyGrid
from mavros_msgs.msg import State

rospy.init_node('lichtblick_teaching_observer', anonymous=True)
report = dict(counts={}, states=[], map=None, jpeg=None, bridge={}, forbidden={})
subscriptions = []


def receive(message, topic):
    report['counts'][topic] = report['counts'].get(topic, 0) + 1
    if topic == '/mavros/state':
        report['states'].append(dict(connected=message.connected, armed=message.armed, mode=message.mode))
    elif topic == '/map':
        report['map'] = dict(frame=message.header.frame_id, width=message.info.width,
                            height=message.info.height, resolution=message.info.resolution)
    elif topic.endswith('/compressed'):
        report['jpeg'] = dict(bytes=len(message.data), frame=message.header.frame_id,
                             format=message.format, source_stamp=message.header.stamp.to_sec())


for topic, kind in [('/scan', LaserScan), ('/mavros/imu/data', Imu),
                    ('/mavros/state', State), ('/map', OccupancyGrid),
                    ('/monitor/camera/image/compressed', CompressedImage)]:
    subscriptions.append(rospy.Subscriber(topic, kind, receive, callback_args=topic, queue_size=2))


def forbidden(message, topic):
    report['forbidden'][topic] += 1


for topic in ['/gpio_cmd', '/gpio_ack', '/mavros/setpoint_raw/local',
              '/mavros/setpoint_position/local', '/mavros/setpoint_velocity/cmd_vel',
              '/mavros/vision_pose/pose', '/move_base_simple/goal']:
    report['forbidden'][topic] = 0
    subscriptions.append(rospy.Subscriber(topic, rospy.AnyMsg, forbidden,
                                          callback_args=topic, queue_size=10))
for name in ['capabilities', 'client_topic_whitelist', 'param_whitelist',
             'service_whitelist', 'topic_whitelist']:
    report['bridge'][name] = rospy.get_param('/foxglove_bridge/'+name)
begin = time.time()
rospy.sleep(12)
report['seconds'] = time.time()-begin
report['master'] = rospy.get_master().getSystemState()
report['passed'] = (all(report['counts'].get(topic, 0)>0 for topic in
                        ['/scan', '/mavros/imu/data', '/mavros/state', '/map',
                         '/monitor/camera/image/compressed'])
                    and all(item['connected'] and not item['armed'] for item in report['states'])
                    and not any(report['forbidden'].values())
                    and all(report['bridge'][name] == [] for name in
                            ['capabilities', 'client_topic_whitelist', 'param_whitelist', 'service_whitelist']))
with open('/home/jcfh/lichtblick-observation.json', 'w') as target:
    json.dump(report, target, indent=2, sort_keys=True)
print(json.dumps({key: report[key] for key in ['counts', 'states', 'map', 'jpeg', 'forbidden', 'passed']}, indent=2))
rospy.signal_shutdown('Observation complete')
