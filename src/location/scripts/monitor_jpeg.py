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
    def __init__(self, source, destination, fps, width, quality, archive_destination=None):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.width = width
        self.quality = quality
        self.publisher = rospy.Publisher(destination, CompressedImage, queue_size=1)
        self.archive = rospy.Publisher(archive_destination, CompressedImage, queue_size=1) if archive_destination else None
        self.archive_last = 0.0
        self.source = source
        self.subscriber = None
        self.timer = rospy.Timer(rospy.Duration(1.0/fps), self.publish)

    def receive(self, message):
        with self.lock:
            self.latest = message

    def publish(self, event):
        live = bool(self.publisher.get_num_connections())
        archive = bool(self.archive and self.archive.get_num_connections())
        # Idle display helpers must not keep deserializing full RGB frames.
        # Only this timer owns the subscription; receive() only owns latest.
        if not live and not archive:
            if self.subscriber is not None:
                self.subscriber.unregister()
                self.subscriber = None
            with self.lock:
                self.latest = None
            return
        if self.subscriber is None:
            with self.lock:
                self.latest = None
            self.subscriber = rospy.Subscriber(self.source, Image, self.receive,
                                               queue_size=1, buff_size=16777216)
            return
        now = rospy.Time.now().to_sec()
        archive_due = archive and now-self.archive_last >= .5
        # A recorder alone must not activate the 15fps display encoder.
        # With a live viewer, reuse the same encoded JPEG at <=2fps.
        if not live and not archive_due:
            return
        with self.lock:
            message, self.latest = self.latest, None
        if message is None or message.header.stamp.is_zero():
            return
        age=(rospy.Time.now()-message.header.stamp).to_sec()
        if age > 1.0 or age < -.05:
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
        if live: self.publisher.publish(output)
        if archive_due:
            self.archive.publish(output)
            self.archive_last=now


if __name__ == '__main__':
    rospy.init_node('monitor_jpeg')
    fps = float(rospy.get_param('~fps', 15.0))
    width = int(rospy.get_param('~width', 960))
    quality = int(rospy.get_param('~quality', 75))
    if not 1 <= fps <= 30 or width < 32 or not 1 <= quality <= 100:
        raise ValueError('Invalid JPEG relay settings')
    relays = [Relay('/camera/image_raw', '/monitor/camera/raw/compressed', fps, width, quality,
                    '/monitor/camera/log/compressed'),
              Relay('/detect_result', '/monitor/camera/image/compressed', fps, width, quality)]
    rospy.spin()
