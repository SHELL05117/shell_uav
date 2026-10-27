#!/usr/bin/env python3
"""Read-only LAN Foxglove protocol check, using only Python's standard library."""
import argparse
import base64
import collections
import hashlib
import json
import os
import socket
import struct
import time


def run(host, seconds):
    sock=socket.create_connection((host,8765),timeout=8)
    key=base64.b64encode(os.urandom(16)).decode()
    sock.sendall(('GET / HTTP/1.1\r\nHost: %s:8765\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n'%(host,key)).encode())
    data=b''
    while b'\r\n\r\n' not in data: data+=sock.recv(8192)
    headers,data=data.split(b'\r\n\r\n',1)
    accept=base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
    if not headers.startswith(b'HTTP/1.1 101') or accept.lower() not in headers.lower():
        raise RuntimeError('Invalid WebSocket handshake: '+headers.decode())
    def exact(n):
        nonlocal data
        while len(data)<n:
            chunk=sock.recv(max(8192,n-len(data)))
            if not chunk: raise RuntimeError('WebSocket closed')
            data+=chunk
        out,data=data[:n],data[n:]
        return out
    def send(value,op=1):
        payload=json.dumps(value).encode() if op==1 else value
        length=len(payload); mask=os.urandom(4)
        frame=bytes([0x80|op,0x80|length]) if length<126 else bytes([0x80|op,0x80|126])+struct.pack('!H',length)
        sock.sendall(frame+mask+bytes(v^mask[i%4] for i,v in enumerate(payload)))
    channels={}; subscriptions={}; counts=collections.Counter(); sizes=collections.Counter()
    report={'host':host,'capabilities':None,'map':{},'camera_age_ms':[],'topics':[]}
    deadline=time.monotonic()+seconds
    try:
        while time.monotonic()<deadline:
            sock.settimeout(max(.1,deadline-time.monotonic()))
            try: a,b=exact(2)
            except socket.timeout: break
            length=b&127
            if length==126: length=struct.unpack('!H',exact(2))[0]
            elif length==127: length=struct.unpack('!Q',exact(8))[0]
            if length>32*1024*1024: raise ValueError('Oversized WebSocket frame')
            payload=exact(length); op=a&15
            if op==9: send(payload,10); continue
            if op==1:
                msg=json.loads(payload)
                if msg['op']=='serverInfo': report['capabilities']=msg.get('capabilities')
                if msg['op']=='advertise':
                    new=[]
                    for c in msg['channels']:
                        channels[c['topic']]=c['id']
                        if c['topic'] in ['/map','/tf','/monitor/camera/raw/compressed','/monitor/camera/image/compressed']:
                            sid=len(subscriptions)+1; subscriptions[sid]=c['topic']
                            new.append({'id':sid,'channelId':c['id']})
                    if new: send({'op':'subscribe','subscriptions':new})
            elif op==2 and payload and payload[0]==1:
                sid=struct.unpack_from('<I',payload,1)[0]
                topic=subscriptions[sid]; wire=payload[13:]
                counts[topic]+=1; sizes[topic]+=len(payload)
                if topic=='/map':
                    offset=12+4+struct.unpack_from('<I',wire,12)[0]
                    resolution,w,h=struct.unpack_from('<fII',wire,offset+8)
                    report['map']={'width':w,'height':h,'resolution':resolution}
                if topic.endswith('/raw/compressed'):
                    sec,nsec=struct.unpack_from('<II',wire,4)
                    report['camera_age_ms'].append((time.time()-sec-nsec/1e9)*1000)
    finally:
        send(b'',8); sock.close()
    report['topics']=sorted(channels)
    report['messages']=dict(counts)
    report['bytes']=dict(sizes)
    ages=sorted(report.pop('camera_age_ms'))
    report['camera_age_ms']={'samples':len(ages),'median':ages[len(ages)//2] if ages else None,'max':max(ages) if ages else None}
    report['passed']=bool(counts['/map'] and counts['/tf'] and counts['/monitor/camera/raw/compressed']) and report['capabilities']==[]
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('host'); parser.add_argument('output'); parser.add_argument('--seconds',type=float,default=15)
    args=parser.parse_args()
    result=run(args.host,args.seconds)
    with open(args.output,'w',encoding='utf-8') as f: json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
