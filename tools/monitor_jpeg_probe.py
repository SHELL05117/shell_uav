#!/usr/bin/env python2
"""Private ROS JPEG subscription lifecycle test; no camera or flight devices."""
from __future__ import print_function
import argparse,json,os,signal,socket,subprocess,time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output');parser.add_argument('--relay',required=True)
    args=parser.parse_args();root=os.path.abspath(args.output)
    if not os.path.isdir(root):os.makedirs(root)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11344',ROS_IP='127.0.0.1',ROS_LOG_DIR=root+'/ros-logs')
    os.environ.pop('ROS_HOSTNAME',None)
    guard=socket.socket();guard.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    guard.bind(('127.0.0.1',11344));guard.close()
    processes=[];logs=[]
    def start(name,command):
        stream=open(root+'/'+name+'.log','w');logs.append(stream)
        p=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
        processes.append(p)
    def stop(p):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        deadline=time.time()+6
        while p.poll() is None and time.time()<deadline:time.sleep(.05)
        if p.poll() is None:os.killpg(p.pid,signal.SIGKILL)
        p.wait()
    def wait_for(predicate,label,seconds=5):
        deadline=time.time()+seconds
        while time.time()<deadline:
            if predicate():return
            time.sleep(.05)
        raise AssertionError(label)
    cases=[]
    try:
        start('master',['roscore','-p','11344']);time.sleep(3)
        import rospy
        from sensor_msgs.msg import Image,CompressedImage
        from std_msgs.msg import Header
        rospy.init_node('jpeg_lifecycle_probe',disable_signals=True)
        source=rospy.Publisher('/camera/image_raw',Image,queue_size=1)
        sent=set();live=[];archive=[];stale=[False]
        def publish(event):
            stamp=rospy.Time.now()-rospy.Duration(3 if stale[0] else 0)
            sent.add(stamp.to_nsec())
            source.publish(Image(header=Header(stamp=stamp,frame_id='camera'),
                                 height=48,width=64,encoding='rgb8',step=192,data=b'\x40\x80\xc0'*(64*48)))
        timer=rospy.Timer(rospy.Duration(.2),publish)
        start('relay',['python2',args.relay,'__name:=monitor_jpeg'])
        time.sleep(2)
        assert source.get_num_connections()==0;cases.append('idle_has_no_raw_subscriber')
        sub=rospy.Subscriber('/monitor/camera/raw/compressed',CompressedImage,lambda m:live.append(m),queue_size=5)
        wait_for(lambda:len(live)>=4,'live JPEG delivery')
        assert source.get_num_connections()==1
        assert all(m.header.stamp.to_nsec() in sent and m.header.frame_id=='camera' and m.data[:2]==b'\xff\xd8' for m in live)
        cases.append('live_encodes_real_jpeg_preserving_source_header')
        sub.unregister();wait_for(lambda:source.get_num_connections()==0,'disconnect releases source')
        cases.append('last_viewer_disconnect_releases_source')
        live[:]=[];reconnect=rospy.Time.now().to_sec()
        sub=rospy.Subscriber('/monitor/camera/raw/compressed',CompressedImage,lambda m:live.append(m),queue_size=5)
        wait_for(lambda:len(live)>=3,'reconnect resumes')
        assert all(m.header.stamp.to_sec()>=reconnect-.2 for m in live)
        cases.append('reconnect_resumes_fresh_frames')
        stale[0]=True;time.sleep(.8);n=len(live);time.sleep(.8)
        assert len(live)==n;cases.append('stale_frames_withheld_without_retimestamping')
        sub.unregister();wait_for(lambda:source.get_num_connections()==0,'second disconnect')
        stale[0]=False
        sub=rospy.Subscriber('/monitor/camera/log/compressed',CompressedImage,lambda m:archive.append(m),queue_size=5)
        wait_for(lambda:len(archive)>=4,'archive-only JPEG delivery')
        times=[m.header.stamp.to_sec() for m in archive]
        assert all(b-a>=.35 for a,b in zip(times,times[1:]))
        assert all(m.header.stamp.to_nsec() in sent for m in archive)
        cases.append('archive_only_keeps_two_hz_limit_and_original_header')
        sub.unregister();wait_for(lambda:source.get_num_connections()==0,'archive disconnect')
        cases.append('archive_disconnect_releases_source')
        timer.shutdown()
        with open(root+'/result.json','w') as f:json.dump(dict(passed=True,cases=cases,hardware=False),f,indent=2)
        print(json.dumps(dict(passed=True,cases=cases)))
    finally:
        for p in reversed(processes):stop(p)
        for stream in logs:stream.close()


if __name__=='__main__':main()
