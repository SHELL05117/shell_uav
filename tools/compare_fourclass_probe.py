"""Inspect fixed CPU/RKNN raw tensors and detections; no training or tuning."""
import argparse
import json
from pathlib import Path
import sys

p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--runtime',type=Path,required=True)
args=p.parse_args();sys.path.insert(0,str(args.runtime))
import cv2
import numpy as np
from PIL import Image,ImageDraw
from verify_yolo_delivery import nms,box_ious

anchors=np.array([10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326],dtype=float).reshape(3,3,2)
labels=['person','explosive','flammable','toxic']
def decode(heads):
    rows=[]
    for i,head in enumerate(heads):
        value=1/(1+np.exp(-np.clip(head[0],-80,80)))
        _,h,w,_=value.shape;stride=8<<i
        gy,gx=np.meshgrid(np.arange(h),np.arange(w),indexing='ij')
        xy=(value[...,:2]*2-.5+np.stack((gx,gy),-1))*stride
        wh=(value[...,2:4]*2)**2*anchors[i,:,None,None,:]
        boxes=np.concatenate((xy-wh/2,xy+wh/2),-1).reshape(-1,4)
        score=(value[...,4:5]*value[...,5:]).reshape(-1,4)
        cls=score.argmax(1);conf=score[np.arange(len(cls)),cls];selected=np.flatnonzero(conf>=.5)
        rows.append(np.c_[boxes[selected],conf[selected],cls[selected]])
    return nms(np.concatenate(rows),.45)

cases=json.loads((args.work/'cases.json').read_text());report=[]
sheet=Image.new('RGB',(1600,1000),'#202020');pen=ImageDraw.Draw(sheet)
for index,rec in enumerate(cases):
    stem=rec['stem'];cpu=[];npu=[];metrics=[]
    for i,side in enumerate((52,26,13)):
        expected=np.load(args.work/'cpu'/(stem+'_'+str(i)+'.npy'))
        actual=np.fromfile(args.work/'raw-outputs'/(stem+'_'+str(i)+'.f32'),dtype='<f4').reshape(1,3,side,side,9)
        if not np.isfinite(actual).all():raise RuntimeError('Invalid raw tensor')
        cpu.append(expected);npu.append(actual)
        metrics.append(dict(mean_abs=float(np.abs(expected-actual).mean()),max_abs=float(np.abs(expected-actual).max()),correlation=float(np.corrcoef(expected.ravel(),actual.ravel())[0,1])))
    first=decode(cpu);second=decode(npu)
    matches=[]
    for det in second:
        same=first[first[:,5]==det[5]]
        matches.append(float(box_ious(same[:,:4],det[:4]).max()) if len(same) else 0.)
    report.append(dict(stem=stem,original_class=rec['original_class'],new_gt_class=rec['new_class'],heads=metrics,cpu_detections=first.tolist(),rknn_detections=second.tolist(),rknn_to_same_class_cpu_best_iou=matches))
    bgr=cv2.imread(str(args.work/'images'/(stem+'.jpg')));h,w=bgr.shape[:2];scale=min(416/w,416/h);nw,nh=round(w*scale),round(h*scale);dx,dy=(416-nw)//2,(416-nh)//2
    image=Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB));image.thumbnail((390,285));draw=ImageDraw.Draw(image)
    for detections,color in ((first,'#22ff88'),(second,'#ff5555')):
        for det in detections:
            box=det[:4].copy();box[[0,2]]=(box[[0,2]]-dx)/scale*image.width/w;box[[1,3]]=(box[[1,3]]-dy)/scale*image.height/h
            draw.rectangle(box.tolist(),outline=color,width=2);draw.text((box[0],max(0,box[1]-10)),labels[int(det[5])],fill=color)
    x,y=index%4*400,index//4*333;sheet.paste(image,(x,y+30));pen.text((x+2,y+2),stem+' GT='+str(rec['new_class'])+' CPU='+str(len(first))+' NPU='+str(len(second)),fill='white')
args.out.mkdir(parents=True,exist_ok=True)
result=dict(protocol='12 unchanged frozen images; supplied four-class labels/anchors; fixed416 RGB; raw logits; conf0.5 NMS0.45; independent regression, not a blind accuracy test',cases=report,all_raw_finite=True,mean_head_abs=float(np.mean([h['mean_abs'] for r in report for h in r['heads']])),minimum_head_correlation=float(min(h['correlation'] for r in report for h in r['heads'])),cpu_detection_count=sum(len(r['cpu_detections']) for r in report),rknn_detection_count=sum(len(r['rknn_detections']) for r in report))
(args.out/'fourclass-cpu-rknn-comparison.json').write_text(json.dumps(result,indent=2)+'\n')
sheet.save(args.out/'fourclass-comparison.jpg',quality=90)
print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))
