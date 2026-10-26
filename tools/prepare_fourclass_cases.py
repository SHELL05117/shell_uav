"""Prepare fixed existing images and CPU reference tensors; no training."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

parser=argparse.ArgumentParser()
parser.add_argument('--workspace',type=Path,required=True)
parser.add_argument('--work',type=Path,required=True)
parser.add_argument('--runtime',type=Path,required=True)
args=parser.parse_args()
sys.path.insert(0,str(args.runtime))
import cv2
import numpy as np
import onnxruntime as ort

root=args.workspace
split=json.loads((root/'docs/验证记录/YOLO训练成果验收_2026-10-24/交回原始证据/reports/split_manifest.json').read_text(encoding='utf-8'))['files']
selected=[]
for category in range(6):
    selected.extend([r for r in split if r['split']=='test' and r['class_id']==category][:2])
if len(selected)!=12:raise RuntimeError('Expected two frozen examples of each original category')
for name in ('rgb','images','cpu'):(args.work/name).mkdir(parents=True,exist_ok=True)
model=root/'src/navigation_2d/detect_mode/26/best.onnx'
opts=ort.SessionOptions();opts.intra_op_num_threads=4
session=ort.InferenceSession(str(model),sess_options=opts,providers=['CPUExecutionProvider'])
rows=[]
with zipfile.ZipFile(root.parent/'TargetDelection1.1.zip') as source:
    members={Path(n).name.lower():n for n in source.namelist() if not n.endswith('/')}
    for rec in selected:
        stem=rec['stem'];jpg=source.read(members[(stem+'.jpg').lower()])
        if hashlib.sha256(jpg).hexdigest()!=rec['image_sha256']:raise RuntimeError('Input source differs')
        bgr=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
        h,w=bgr.shape[:2];scale=min(416/w,416/h);nw,nh=round(w*scale),round(h*scale)
        dx,dy=(416-nw)//2,(416-nh)//2
        canvas=cv2.copyMakeBorder(cv2.resize(bgr,(nw,nh)),dy,416-nh-dy,dx,416-nw-dx,cv2.BORDER_CONSTANT,value=(114,114,114))
        rgb=cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB)
        (args.work/'rgb'/(stem+'.rgb')).write_bytes(rgb.tobytes())
        (args.work/'images'/(stem+'.jpg')).write_bytes(jpg)
        expected=session.run(None,{session.get_inputs()[0].name:np.ascontiguousarray(rgb.transpose(2,0,1)[None],dtype=np.float32)/255.0})
        for i,head in enumerate(expected):
            if list(head.shape)!=[1,3,52>>i,52>>i,9]:raise RuntimeError('Unexpected CPU head shape')
            np.save(args.work/'cpu'/(stem+'_'+str(i)+'.npy'),head)
        rows.append(dict(stem=stem,original_class=rec['class_id'],new_class={0:0,1:3,2:2,3:1}.get(rec['class_id']),source_sha256=rec['image_sha256']))
        print('prepared '+stem,flush=True)
(args.work/'manifest.txt').write_bytes(''.join(r['stem']+'\n' for r in rows).encode('ascii'))
(args.work/'cases.json').write_text(json.dumps(rows,indent=2)+'\n')
