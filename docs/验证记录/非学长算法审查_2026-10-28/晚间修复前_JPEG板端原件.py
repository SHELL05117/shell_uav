#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Optional display-only JPEG relay. No image_transport plugin required."""
from __future__ import print_function
import threading
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class Relay(object):
    def __init__(self, source, destination, fps, width, quality):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.width = width
        self.quality = quality
        self.publisher = rospy.Publisher(destination, CompressedImage, queue_size=1)
        self.subscriber = rospy.Subscriber(source, Image, self.receive,
                                          queue_size=1, buff_size=16777216)
        self.timer = rospy.Timer(rospy.Duration(1.0/fps), self.publish)

    def receive(self, message):
        with self.lock:
            self.latest = message

    def publish(self, event):
        if not self.publisher.get_num_connections():
            return
        with self.lock:
            message, self.latest = self.latest, None
        if message is None or (rospy.Time.now()-message.header.stamp).to_sec() > 1.0:
            return
        frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        if frame.shape[1] > self.width:
            frame = cv2.resize(frame, (self.width, int(round(frame.shape[0]*float(self.width)/frame.shape[1]))))
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        output = CompressedImage()
        output.header = message.header
        output.format = 'bgr8; jpeg compressed bgr8'
        output.data = encoded.tostring()
        self.publisher.publish(output)


if __name__ == '__main__':
    rospy.init_node('monitor_jpeg')
    fps = float(rospy.get_param('~fps', 15.0))
    width = int(rospy.get_param('~width', 960))
    quality = int(rospy.get_param('~quality', 75))
    if not 1 <= fps <= 30 or width < 32 or not 1 <= quality <= 100:
        raise ValueError('Invalid JPEG relay settings')
    relays = [Relay('/camera/image_raw', '/monitor/camera/raw/compressed', fps, width, quality),
              Relay('/detect_result', '/monitor/camera/image/compressed', fps, width, quality)]
    rospy.spin()
