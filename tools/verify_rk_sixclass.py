"""Prepare frozen RGB test inputs, then evaluate real RKNN head dumps on the PC.

Uses the same preprocessing/decoder as verify_yolo_delivery.py. No training,
threshold selection or deployment into ROS. Large raw tensors stay outside Git.
"""
import argparse
import csv
import hashlib
import json
import sys
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

from verify_yolo_delivery import decode, evaluate, nms, save


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage', choices=['prepare', 'evaluate'])
    ap.add_argument('--package', type=Path, required=True)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--work', type=Path, required=True)
    ap.add_argument('--report', type=Path, required=True)
    ap.add_argument('--runtime', type=Path, required=True)
    args = ap.parse_args()
    sys.path.insert(0, str(args.runtime))
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw
    p, work, report = args.package, args.work, args.report
    files = json.loads((p/'reports/split_manifest.json').read_text(encoding='utf-8'))['files']
    tests = [x for x in files if x['split']=='test']
    if len(tests)!=36 or len({x['stem'] for x in tests})!=36:
        raise ValueError('expected frozen 36-image test split')
    names = [x['name'] for x in json.loads((p/'config/class_map.json').read_text(encoding='utf-8'))['classes']]
    anchors = np.loadtxt(p/'config/RK_anchors.txt').reshape(3,3,2)
    work.mkdir(parents=True,exist_ok=True)
    report.mkdir(parents=True,exist_ok=True)
    manifest, predictions, best_predictions, ranges = [], [], [], []
    sheet = Image.new('RGB',(1800,2160),'#181818')
    painter = ImageDraw.Draw(sheet)
    with zipfile.ZipFile(args.source) as z:
        members = {Path(x).name.lower():x for x in z.namelist() if not x.endswith('/')}
        for i, rec in enumerate(tests):
            stem=rec['stem']
            jpg=z.read(members[(stem+'.jpg').lower()])
            xml=z.read(members[(stem+'.xml').lower()])
            if hashlib.sha256(jpg).hexdigest()!=rec['image_sha256'] or hashlib.sha256(xml).hexdigest()!=rec['xml_sha256']:
                raise ValueError('source hash mismatch: '+stem)
            rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
            h,w=rgb.shape[:2]; r=min(640/w,640/h)
            nw,nh=round(w*r),round(h*r); dx,dy=(640-nw)//2,(640-nh)//2
            lb=cv2.copyMakeBorder(cv2.resize(rgb,(nw,nh),interpolation=cv2.INTER_LINEAR),dy,640-nh-dy,dx,640-nw-dx,cv2.BORDER_CONSTANT,value=(114,114,114))
            data=lb.tobytes()
            manifest.append(dict(stem=stem,class_id=rec['class_id'],rgb_sha256=hashlib.sha256(data).hexdigest(),bytes=len(data)))
            if args.stage=='prepare':
                (work/(stem+'.rgb')).write_bytes(data)
                continue
            heads=[np.fromfile(work/'outputs'/(stem+'_'+str(j)+'.f32'),dtype='<f4').reshape(1,33,side,side) for j,side in enumerate((80,40,20))]
            if not all(np.isfinite(x).all() and x.min()>=-.01 and x.max()<=1.01 for x in heads):
                raise ValueError('invalid sigmoid head data: '+stem)
            ranges.append(dict(stem=stem,heads=[dict(min=float(x.min()),max=float(x.max())) for x in heads]))
            bbox=ET.fromstring(xml).find('object/bndbox')
            gt=[float(bbox.findtext(k)) for k in ['xmin','ymin','xmax','ymax']]
            for multi, destination in [(True,predictions),(False,best_predictions)]:
                det=nms(decode(heads,anchors,multi_label=multi,threshold=.001 if multi else .25),.6)
                det[:,[0,2]]=np.clip((det[:,[0,2]]-dx)/r,0,w)
                det[:,[1,3]]=np.clip((det[:,[1,3]]-dy)/r,0,h)
                destination.append(dict(stem=stem,gt_class=rec['class_id'],gt_box=gt,detections=det.tolist()))
            thumb=Image.fromarray(rgb); thumb.thumbnail((295,320)); pen=ImageDraw.Draw(thumb); scale=thumb.width/w
            pen.rectangle([v*scale for v in gt],outline='#00ff00',width=2)
            for row in best_predictions[-1]['detections']:
                box=[v*scale for v in row[:4]]
                pen.rectangle(box,outline='#ff4040',width=2)
                pen.text((box[0],max(0,box[1]-11)),f'{int(row[5])} {row[4]:.2f}',fill='red')
            x,y=i%6*300,i//6*360
            sheet.paste(thumb,(x,y+24)); painter.text((x+2,y+2),f'{stem} GT:{rec["class_id"]}',fill='white')
    if args.stage=='prepare':
        (work/'manifest.txt').write_bytes(''.join(x['stem']+'\n' for x in manifest).encode('ascii'))
        save(report/'sixclass_input_manifest.json',manifest)
        print('Prepared 36 frozen test RGB inputs; 1,228,800 bytes each.')
    else:
        expected=json.loads((report/'sixclass_input_manifest.json').read_text(encoding='utf-8'))
        if expected!=manifest: raise ValueError('preprocessing manifest changed')
        timings=list(csv.DictReader((work/'outputs/timings.tsv').open(),delimiter='\t'))
        if [x['stem'] for x in timings]!=[x['stem'] for x in manifest]: raise ValueError('incomplete timings')
        ms=[float(x['set_run_get_ms']) for x in timings]
        result=dict(protocol='Same frozen 36 images, RGB uint8 NHWC, SDK mean=0 scale=255; fixed letterbox 640, sigmoid heads, grid -0.5, class-aware NMS 0.6. No tuning.',
                    model_sha256=hashlib.sha256((p/'models/sixclass_yolov5s_relu_rk3399pro.rknn').read_bytes()).hexdigest(),
                    rgb=evaluate(predictions,names),rgb_bestclass_working_point=evaluate(best_predictions,names)['working_point'],
                    timing_ms=dict(count=len(ms),mean=float(np.mean(ms)),median=float(np.median(ms)),min=min(ms),max=max(ms),
                                   scope='RK CPU-side rknn_inputs_set + rknn_run + blocking rknn_outputs_get, excludes preprocess/decode/render/disk. Includes first inference; not camera FPS.'),
                    note='Real RK3399Pro NPU; model/runtime validation is separate from business accuracy and independent-scene/flight acceptance.')
        save(report/'sixclass_rknn_evaluation.json',result)
        save(report/'sixclass_rknn_predictions.json',predictions)
        save(report/'sixclass_rknn_bestclass_predictions.json',best_predictions)
        save(report/'sixclass_head_ranges.json',ranges)
        sheet.save(report/'sixclass_rknn_test_sheet.jpg',quality=90)
        print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
