#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Bounded four-class image and real-camera checks on a private ROS master.

Source catkin's devel/setup.bash before running on RK/Melodic:
  python2 rk_fourclass_camera_check.py OUTPUT_DIR /path/to/IMG_2766.jpg

Cases run sequentially and exclusively: real RKNN detectnet with a known image,
then usb_cam plus real detectnet, then a missing-model exit check. No MAVROS,
flight/servo nodes, model conversion, calibration, service restart or training.
Only child process groups created by this script receive stop signals.
"""
from __future__ import print_function
import argparse
import ctypes
import errno
import hashlib
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback


class Timespec(ctypes.Structure):
    _fields_ = [('seconds', ctypes.c_long), ('nanoseconds', ctypes.c_long)]


clock_gettime = ctypes.CDLL(None, use_errno=True).clock_gettime
clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
clock_gettime.restype = ctypes.c_int


def steady():
    sample = Timespec()
    if clock_gettime(1, ctypes.byref(sample)) != 0:  # CLOCK_MONOTONIC
        raise OSError(ctypes.get_errno(), 'clock_gettime failed')
    return sample.seconds + sample.nanoseconds * 1e-9


def sha256(path):
    result = hashlib.sha256()
    with open(path, 'rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def text_file(path):
    with open(path, 'rb') as source:
        return source.read().decode('utf-8', 'replace')


def log_evidence(path):
    content = re.sub(r'\x1b\[[0-9;]*m', '', text_file(path))
    return dict(inference_ok=len(re.findall(r'Detect: inference OK\b', content)),
                flammable_detections=len(re.findall(r'\bdetect:\s+flammable\s+', content)),
                model_ready='Detect: RKNN ready' in content,
                model_open_error='cannot open RKNN model' in content)


class Children(object):
    def __init__(self, directory):
        self.directory = directory
        self.children = []

    def start(self, name, arguments):
        path = os.path.join(self.directory, name + '.log')
        output = open(path, 'wb')
        process = subprocess.Popen(arguments, stdout=output, stderr=subprocess.STDOUT,
                                   preexec_fn=os.setsid)
        child = dict(name=name, process=process, output=output, log=path,
                     arguments=arguments, exit_code=None, stop_signals=[], process_group_empty=False)
        self.children.append(child)
        return child

    def require_alive(self, children):
        for child in children:
            if child['process'].poll() is not None:
                raise RuntimeError('%s exited during observation with %s' %
                                   (child['name'], child['process'].returncode))

    def stop(self, child):
        if child['process_group_empty']:
            return child['exit_code']
        process = child['process']
        def group_alive():
            try:
                os.killpg(process.pid, 0)
                return True
            except OSError as error:
                if error.errno == errno.ESRCH:
                    return False
                raise
        for signum, seconds in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
            process.poll()
            if not group_alive():
                break
            try:
                os.killpg(process.pid, signum)
                child['stop_signals'].append(signum)
            except OSError:
                if group_alive():
                    raise
            deadline = steady() + seconds
            while group_alive() and steady() < deadline:
                process.poll()
                time.sleep(.05)
        if group_alive():
            raise RuntimeError('Owned child group %s did not terminate' % child['name'])
        child['exit_code'] = process.wait()
        child['process_group_empty'] = True
        child['output'].close()
        return process.returncode

    def finish_all(self):
        errors = []
        for child in reversed(self.children):
            try:
                self.stop(child)
            except Exception as error:
                errors.append('%s: %s' % (child['name'], error))
        return errors

    def report(self):
        return [dict(name=c['name'], arguments=c['arguments'], log=os.path.basename(c['log']),
                     exit_code=c['exit_code'], stop_signals=c['stop_signals'],
                     process_group_empty=c['process_group_empty'])
                for c in self.children]


class Images(object):
    def __init__(self, rospy, topics, baseline=None):
        import cv2
        import numpy as np
        from sensor_msgs.msg import Image
        self.cv2, self.np = cv2, np
        self.lock = threading.Lock()
        self.times = {name: [] for name in topics}
        self.shapes = {name: {} for name in topics}
        self.last = {}
        self.errors = []
        self.baseline = baseline
        self.best_annotated = None
        self.best_red_changes = 0
        self.last_mask_check = 0.0
        self.subscribers = []
        for name, topic in topics.items():
            self.subscribers.append(rospy.Subscriber(topic, Image, self.callback,
                callback_args=name, queue_size=1, buff_size=64000000))

    def callback(self, message, name):
        try:
            if (message.encoding not in ('bgr8', 'rgb8') or message.width <= 0 or
                    message.height <= 0 or message.step < message.width * 3 or
                    len(message.data) != message.height * message.step):
                raise ValueError('Image size/encoding mismatch for ' + name)
            values = self.np.frombuffer(message.data, dtype=self.np.uint8)
            image = values.reshape(message.height, message.step)[:, :message.width*3]
            image = image.reshape(message.height, message.width, 3)
            if message.encoding == 'rgb8':
                image = self.cv2.cvtColor(image, self.cv2.COLOR_RGB2BGR)
            arrived = steady()
            with self.lock:
                self.times[name].append(arrived)
                shape = (message.width, message.height, message.encoding)
                self.shapes[name][shape] = self.shapes[name].get(shape, 0) + 1
                self.last[name] = image
                check_mask = (name == 'output' and self.baseline is not None and
                              arrived-self.last_mask_check >= .5)
                if check_mask:
                    self.last_mask_check = arrived
            # Known-image baseline subtraction distinguishes drawn red boxes/text
            # from red pixels already in the photographed target itself.
            if check_mask:
                baseline = self.baseline
                if image.shape != baseline.shape:
                    baseline = self.cv2.resize(baseline, (image.shape[1], image.shape[0]))
                changed = self.np.max(self.cv2.absdiff(image, baseline), axis=2) > 25
                red = (image[:, :, 2] > 150) & (image[:, :, 1] < 110) & (image[:, :, 0] < 110)
                score = int(self.np.count_nonzero(changed & red))
                with self.lock:
                    if score > self.best_red_changes:
                        self.best_red_changes = score
                        self.best_annotated = image.copy()
        except Exception as error:
            with self.lock:
                if len(self.errors) < 10:
                    self.errors.append(repr(error))

    def count(self, name):
        with self.lock:
            return len(self.times[name])

    def finish(self, directory):
        for subscriber in self.subscribers:
            subscriber.unregister()
        streams = {}
        with self.lock:
            for name, values in self.times.items():
                differences = [b-a for a, b in zip(values, values[1:])]
                streams[name] = dict(count=len(values),
                    hz=(len(values)-1)/(values[-1]-values[0]) if len(values)>1 else None,
                    max_gap=max(differences) if differences else None,
                    shapes=[dict(width=k[0], height=k[1], encoding=k[2], count=v)
                            for k,v in sorted(self.shapes[name].items())])
            images = dict(self.last)
            errors = list(self.errors)
            best_annotated, best_red_changes = self.best_annotated, self.best_red_changes
        saved = {}
        for name, image in images.items():
            path = os.path.join(directory, name+'-last.jpg')
            if not self.cv2.imwrite(path, image):
                raise RuntimeError('Could not save ' + path)
            saved[name] = os.path.basename(path)
        if best_annotated is not None:
            path = os.path.join(directory, 'known-flammable-annotated.jpg')
            if not self.cv2.imwrite(path, best_annotated):
                raise RuntimeError('Could not save known-image annotation')
            saved['annotated'] = os.path.basename(path)
        return dict(streams=streams, image_errors=errors, saved_images=saved,
                    annotation_red_changed_pixels=best_red_changes)


def detector_parameters(rospy):
    values = {name: rospy.get_param('/detectnet/'+name) for name in
              ('confidence_threshold','nms_iou','input_size','backend','model_file','labels_file')}
    if (abs(values['confidence_threshold']-.5)>1e-9 or abs(values['nms_iou']-.45)>1e-9 or
            values['input_size']!=416 or values['backend']!='rknn' or
            values['model_file']!='best.rknn' or values['labels_file']!='labels.txt'):
        raise RuntimeError('Detector parameters differ from the unchanged deployment protocol')
    return values


def wait_for_images(rospy, children, processes, observer, names, timeout, publish=None):
    deadline = steady()+timeout
    while steady() < deadline and not rospy.is_shutdown():
        children.require_alive(processes)
        if publish:
            publish()
        if all(observer.count(name)>0 for name in names):
            return
        time.sleep(.1)
    raise RuntimeError('Image startup deadline exceeded: ' + ','.join(names))


def observe(rospy, children, processes, seconds, publish=None):
    started = steady()
    while steady()-started < seconds and not rospy.is_shutdown():
        children.require_alive(processes)
        if publish:
            publish()
        time.sleep(.1)
    if rospy.is_shutdown():
        raise RuntimeError('Private ROS observer shut down unexpectedly')
    return steady()-started


def run(arguments):
    import cv2
    from sensor_msgs.msg import Image
    directory = os.path.abspath(arguments.output_dir)
    if not os.path.isdir(directory):
        os.makedirs(directory)
    result_path = os.path.join(directory, 'fourclass-camera-results.json')
    if os.path.exists(result_path):
        raise RuntimeError('Refusing to overwrite an existing completed check')
    known_path = os.path.abspath(arguments.known_image_path)
    baseline = cv2.imread(known_path, cv2.IMREAD_COLOR)
    if baseline is None:
        raise RuntimeError('Cannot read the supplied known image')
    original_height, original_width = baseline.shape[:2]
    scale = min(1.0, 1280.0 / original_width, 720.0 / original_height)
    if scale < 1.0:
        published_size = (max(1, int(round(original_width * scale))),
                          max(1, int(round(original_height * scale))))
        baseline = cv2.resize(baseline, published_size, interpolation=cv2.INTER_AREA)
    known_geometry = dict(original_width=original_width, original_height=original_height,
        published_width=baseline.shape[1], published_height=baseline.shape[0],
        downscale=scale, interpolation='INTER_AREA' if scale < 1.0 else 'none',
        upscaled=False, purpose='camera-sized still-image replay; not original-resolution throughput')
    package = subprocess.check_output(['rospack','find','navigation_2d']).decode('utf-8').strip()
    model_dir = os.path.join(package, 'detect_mode', '26')
    expected_labels = ['person','explosive','flammable','toxic']
    labels = text_file(os.path.join(model_dir,'labels.txt')).splitlines()
    if labels != expected_labels:
        raise RuntimeError('Four-class labels do not match the received model contract')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11330', ROS_IP='127.0.0.1',
        ROS_HOME=directory+'/ros-home', ROS_LOG_DIR=directory+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    reservation = socket.socket()
    reservation.bind(('127.0.0.1',11330))
    reservation.close()
    children = Children(directory)
    report = dict(cases={}, model_dir=model_dir, labels=labels, private_master=os.environ['ROS_MASTER_URI'],
        known_image_geometry=known_geometry, output_hz_is_inference_hz=False,
        input_sha256=dict(known_image=sha256(known_path),
            rknn=sha256(os.path.join(model_dir,'best.rknn')),
            onnx=sha256(os.path.join(model_dir,'best.onnx'))), passed=False)
    rospy = None
    master = None
    active_observer = None
    try:
        master = children.start('master',['roscore','-p','11330'])
        deadline = steady()+15
        while steady()<deadline:
            children.require_alive([master])
            connection = socket.socket()
            try:
                connection.connect(('127.0.0.1',11330))
                break
            except socket.error:
                time.sleep(.1)
            finally:
                connection.close()
        else:
            raise RuntimeError('Private master startup failed')
        import rospy
        rospy.init_node('fourclass_bounded_observer', disable_signals=True)
        known_dir = os.path.join(directory,'known-image')
        os.makedirs(known_dir)
        if not cv2.imwrite(os.path.join(known_dir,'known-published-input.jpg'), baseline):
            raise RuntimeError('Could not save known-image published-size preview')
        active_observer = Images(rospy,dict(output='/detect_result'), baseline=baseline)
        publisher = rospy.Publisher('/shell_fourclass_known/image',Image,queue_size=1)
        message = Image()
        message.height, message.width = baseline.shape[:2]
        message.encoding, message.step = 'bgr8', baseline.shape[1]*3
        message.header.frame_id = 'shell_known_image'
        message.data = baseline.tostring()
        def publish_known():
            message.header.stamp = rospy.Time.now()
            publisher.publish(message)
        detector = children.start('known-detector', ['roslaunch','navigation_2d','detect.launch',
            'detect_backend:=rknn','input_image_topic:=/shell_fourclass_known/image','target_name:=flammable'])
        try:
            wait_for_images(rospy,children,[detector],active_observer,['output'],40,publish_known)
            parameters = detector_parameters(rospy)
            duration = observe(rospy,children,[detector],14,publish_known)
        finally:
            exit_code = children.stop(detector)
            publisher.unregister()
        known = active_observer.finish(known_dir)
        active_observer = None
        known.update(log_evidence(detector['log']))
        known.update(observation_seconds=duration, detector_exit=exit_code, parameters=parameters)
        known['published_input_preview'] = 'known-published-input.jpg'
        known['passed'] = (exit_code==0 and known['streams']['output']['count']>=30 and
            not known['image_errors'] and known['inference_ok']>=2 and
            known['flammable_detections']>=1 and known['annotation_red_changed_pixels']>=10)
        report['cases']['known_image'] = known
        print('known_image_passed='+str(known['passed']))
        sys.stdout.flush()
        if not known['passed']:
            raise RuntimeError('Known-image detector/annotation check failed')

        camera_dir = os.path.join(directory,'real-camera')
        os.makedirs(camera_dir)
        active_observer = Images(rospy,dict(camera='/camera/image_raw',output='/detect_result'))
        camera = children.start('real-camera',['roslaunch','usb_cam','rk_usb_camera.launch'])
        detector = children.start('camera-detector',['roslaunch','navigation_2d','detect.launch',
            'detect_backend:=rknn','input_image_topic:=/camera/image_raw'])
        try:
            wait_for_images(rospy,children,[camera,detector],active_observer,['camera','output'],40)
            parameters = detector_parameters(rospy)
            duration = observe(rospy,children,[camera,detector],18)
        finally:
            detector_exit = children.stop(detector)
            camera_exit = children.stop(camera)
        actual = active_observer.finish(camera_dir)
        active_observer = None
        actual.update(log_evidence(detector['log']))
        actual.update(observation_seconds=duration,detector_exit=detector_exit,camera_exit=camera_exit,
                      parameters=parameters)
        source, output = actual['streams']['camera'], actual['streams']['output']
        consistent = (len(source['shapes'])==1 and len(output['shapes'])==1 and
            source['shapes'][0]['width']==output['shapes'][0]['width'] and
            source['shapes'][0]['height']==output['shapes'][0]['height'])
        actual['dimensions_consistent'] = consistent
        actual['passed'] = (detector_exit==0 and camera_exit==0 and not actual['image_errors'] and
            source['count']>=120 and output['count']>=60 and
            source['hz'] is not None and source['hz']>=20 and
            output['hz'] is not None and output['hz']>=5 and consistent and actual['inference_ok']>=2)
        report['cases']['real_camera'] = actual
        print('real_camera_passed='+str(actual['passed']))
        sys.stdout.flush()
        if not actual['passed']:
            raise RuntimeError('Real-camera receive/inference/exit check failed')

        missing = '__shell_missing_model_preflight__.rknn'
        if os.path.exists(os.path.join(model_dir,missing)):
            raise RuntimeError('Missing-model sentinel unexpectedly exists')
        anchors = '10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326'
        detector = children.start('missing-model',['rosrun','navigation_2d','detectnet',
            '_backend:=rknn','_model_path:='+model_dir,'_model_file:='+missing,
            '_labels_file:=labels.txt','_yolo_anchors:='+anchors,'_yolo_strides:=8,16,32',
            '_input_image_topic:=/shell_missing_model/image'])
        deadline = steady()+25
        while detector['process'].poll() is None and steady()<deadline:
            time.sleep(.1)
        exited_by_itself = detector['process'].poll() is not None
        exit_code = children.stop(detector)
        missing_case = log_evidence(detector['log'])
        missing_case.update(exit_code=exit_code,exited_by_itself=exited_by_itself)
        missing_case['passed'] = (exited_by_itself and exit_code==2 and
            missing_case['model_open_error'] and not missing_case['inference_ok'])
        report['cases']['missing_model'] = missing_case
        if not missing_case['passed']:
            raise RuntimeError('Missing-model initialization was not blocked with exit 2')
    except Exception as error:
        report['error'] = repr(error)
        report['traceback'] = traceback.format_exc()
    finally:
        if active_observer is not None:
            for subscriber in active_observer.subscribers:
                subscriber.unregister()
        if rospy is not None:
            rospy.signal_shutdown('Bounded four-class check finished')
        report['cleanup_errors'] = children.finish_all()
        report['children'] = children.report()
        report['normal_child_stops'] = all(all(s==signal.SIGINT for s in c['stop_signals'])
                                           for c in report['children'])
        report['master_exit'] = master['exit_code'] if master else None
        report['passed'] = (len(report['cases'])==3 and
            all(case['passed'] for case in report['cases'].values()) and
            report['master_exit']==0 and report['normal_child_stops'] and
            not report['cleanup_errors'] and 'error' not in report)
        with open(result_path,'w') as output:
            json.dump(report,output,indent=2,sort_keys=True)
        print(json.dumps(report,indent=2,sort_keys=True))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir')
    parser.add_argument('known_image_path')
    sys.exit(run(parser.parse_args()))
