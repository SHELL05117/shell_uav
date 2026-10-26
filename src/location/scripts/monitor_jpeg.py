#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Optional display-only JPEG relay. No image_transport plugin required."""
from __future__ import print_function
import time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class Relay(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.last = 0.0
        self.publisher = rospy.Publisher('/monitor/camera/image/compressed',
                                         CompressedImage, queue_size=1)
        self.subscriber = rospy.Subscriber('/detect_result', Image, self.receive,
                                          queue_size=1, buff_size=16777216)

    def receive(self, message):
        now = time.time()
        if now - self.last < 0.2:
            return
        self.last = now
        frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        if frame.shape[1] > 960:
            frame = cv2.resize(frame, (960, int(round(frame.shape[0]*960.0/frame.shape[1]))))
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        output = CompressedImage()
        output.header = message.header
        output.format = 'jpeg'
        output.data = encoded.tostring()
        self.publisher.publish(output)


if __name__ == '__main__':
    rospy.init_node('monitor_jpeg')
    relay = Relay()
    rospy.spin()
