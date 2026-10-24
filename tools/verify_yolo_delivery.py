"""Read-only delivery audit and independent CPU ONNX evaluation; never trains.

Requires numpy, Pillow, onnx, onnxruntime, opencv-python-headless.
Original delivery, source archive and deployment state are never modified.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import pickletools
import sys
import time
import zipfile
from collections import Counter
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree as ET
from zoneinfo import ZoneInfo


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def audit(args):
    p = args.package
    entries = []
    for line in (p / 'SHA256SUMS.txt').read_text(encoding='utf-8-sig').splitlines():
        expected, name = line.split(None, 1)
        name = name.lstrip('*')
        target = (p / name).resolve()
        if not target.is_relative_to(p.resolve()):
            raise ValueError('unsafe manifest path')
        actual = file_sha(target) if target.is_file() else None
        entries.append(dict(path=name, expected=expected, actual=actual, ok=expected == actual))
    listed = {x['path'] for x in entries}
    extras = sorted(x.relative_to(p).as_posix() for x in p.rglob('*') if x.is_file()
                    and x.relative_to(p).as_posix() not in listed and x.name != 'SHA256SUMS.txt')
    outer = p.parents[1] / 'YOLO异机训练交付_2026-10-24.zip'
    with zipfile.ZipFile(outer) as z:
        crc_error = z.testzip()
        zip_files = [x for x in z.infolist() if not x.is_dir()]
        zip_vs_local = []
        for member in zip_files:
            parts = Path(member.filename).parts
            name = Path(*parts[1:]) if parts[0] == p.name else Path(*parts)
            local = p / name
            zip_vs_local.append(dict(path=member.filename, ok=local.is_file() and sha(z.read(member)) == file_sha(local)))
    model_manifest = json.loads((p / 'models/model_manifest.json').read_text(encoding='utf-8'))
    models = []
    for name, meta in model_manifest['weights'].items():
        actual = file_sha(p / 'models' / name)
        models.append(dict(path=name, bytes=(p / 'models' / name).stat().st_size,
                           sha256=actual, manifest_sha256=meta['sha256'], manifest_ok=actual == meta['sha256']))
    structures = {}
    for name in ['best.pt', 'best.torchscript']:
        with zipfile.ZipFile(p / 'models' / name) as z:
            codes = [(x, z.read(x).decode('utf-8', errors='replace')) for x in z.namelist() if '/code/' in x and x.endswith('.py')]
            structures[name] = dict(zip_crc_error=z.testzip(), members=len(z.namelist()),
                                   relu_calls=sum(t.count('torch.relu') for _, t in codes),
                                   silu_calls=sum(t.count('torch.silu') for _, t in codes))
            if name == 'best.pt':
                ops = list(pickletools.genops(z.read(next(x for x in z.namelist() if x.endswith('/data.pkl')))))
                structures[name]['pickle_globals'] = sorted({a for o, a, _ in ops if o.name == 'GLOBAL'})
                structures[name]['epoch_minus_one'] = any(a == 'epoch' and ops[i+2][1] == -1
                                                         for i, (_, a, _) in enumerate(ops[:-2]))
    structures['rknn_header_hex'] = (p / 'models/sixclass_yolov5s_relu_rk3399pro.rknn').read_bytes()[:64].hex()
    split = json.loads((p / 'reports/split_manifest.json').read_text(encoding='utf-8'))
    files = split['files']
    calibration = json.loads((p / 'reports/calib_manifest.json').read_text(encoding='utf-8'))
    training = {x['image']: x for x in files if x['split'] == 'train'}
    calib_errors = [x for x in calibration['files'] if x['source'] not in training
                    or x['source_sha256'] != training[x['source']]['image_sha256']]
    source_matches = []
    with zipfile.ZipFile(args.source) as z:
        members = {Path(x).name.lower(): x for x in z.namelist() if not x.endswith('/')}
        for rec in files:
            jpg = z.read(members[(rec['stem'] + '.jpg').lower()])
            xml = z.read(members[(rec['stem'] + '.xml').lower()])
            source_matches.append(dict(stem=rec['stem'], jpg_ok=sha(jpg) == rec['image_sha256'], xml_ok=sha(xml) == rec['xml_sha256']))
    csv_rows = list(csv.DictReader((p / 'metrics/selected_exp1_warm_coco_silu/results.csv').open(encoding='utf-8')))
    csv_rows = [{k.strip(): float(v) for k, v in x.items()} for x in csv_rows]
    best_fitness = max(csv_rows, key=lambda x: .1 * x['metrics/mAP_0.5'] + .9 * x['metrics/mAP_0.5:0.95'])
    best_map50 = max(csv_rows, key=lambda x: x['metrics/mAP_0.5'])
    historical = {}
    for subset in ['test', 'val']:
        dets = list(csv.DictReader((p / f'metrics/{subset}_eval/detections.csv').open(encoding='utf-8')))
        reported = json.loads((p / f'metrics/{subset}_eval/per_image_predictions.json').read_text(encoding='utf-8'))
        mismatch = {'.001': [], '.25': []}
        for rec in reported:
            rows = [x for x in dets if x['stem'] == rec['stem']]
            for label, threshold in [('.001', .001), ('.25', .25)]:
                chosen = max((x for x in rows if float(x['conf']) >= threshold), key=lambda x: float(x['conf']), default=None)
                predicted = int(chosen['class_id']) if chosen and float(chosen['iou_with_gt']) >= .5 else None
                if predicted != rec['pred']:
                    mismatch[label].append(rec['stem'])
        historical[subset] = dict(reported_images=len(reported), top1_reconstruction_mismatches=mismatch)
    out = dict(time=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(), package=str(p),
               archive=dict(path=str(outer), bytes=outer.stat().st_size, sha256=file_sha(outer),
                            expected_sha256=outer.with_suffix('.zip.sha256').read_text(encoding='utf-8-sig').split()[0],
                            crc_error=crc_error, files=len(zip_files), zip_vs_local=zip_vs_local),
               checksum_entries=entries, unlisted_files=extras, models=models, structures=structures,
               source=dict(path=str(args.source), bytes=args.source.stat().st_size, sha256=file_sha(args.source),
                           expected_sha256=split['source_archive_sha256'], matches=source_matches),
               splits=dict(counts=dict(Counter(x['split'] for x in files)),
                           per_class={s: dict(Counter(x['class_name'] for x in files if x['split'] == s)) for s in ['train', 'val', 'test']},
                           duplicate_stems=len(files)-len({x['stem'] for x in files}),
                           duplicate_image_hashes=len(files)-len({x['image_sha256'] for x in files}),
                           group_independent=split['split_policy']['group_independent'],
                           calibration_count=len(calibration['files']), calibration_errors=calib_errors),
               training=dict(epochs=len(csv_rows), first=csv_rows[0], last=csv_rows[-1],
                             best_fitness=best_fitness, best_map50=best_map50), historical_working_point=historical)
    save(args.out / 'integrity_audit.json', out)
    print(json.dumps(dict(hash_entries=len(entries), hash_failed=sum(not x['ok'] for x in entries), unlisted=extras,
                          zip_crc=crc_error, zip_local_failed=sum(not x['ok'] for x in zip_vs_local),
                          model_manifest_failed=[x['path'] for x in models if not x['manifest_ok']],
                          splits=out['splits'], epochs=len(csv_rows), best_fitness=best_fitness,
                          historical=historical), ensure_ascii=False), flush=True)
    return files


def nms(det, threshold=.6):
    import numpy as np
    if not len(det):
        return det
    boxes = det[:, :4] + det[:, 5:6] * 7680
    order = np.argsort(-det[:, 4], kind='stable')[:30000]
    selected = []
    while len(order) and len(selected) < 300:
        i = order[0]
        selected.append(i)
        rest = order[1:]
        if not len(rest):
            break
        lo = np.maximum(boxes[i, :2], boxes[rest, :2])
        hi = np.minimum(boxes[i, 2:], boxes[rest, 2:])
        inter = np.maximum(hi-lo, 0).prod(1)
        area = (boxes[:, 2:] - boxes[:, :2]).prod(1)
        iou = inter / np.maximum(area[i] + area[rest] - inter, 1e-9)
        order = rest[iou <= threshold]
    return det[selected]


def decode(heads, anchors, grid_offset=-.5, multi_label=True, threshold=.001):
    import numpy as np
    rows = []
    for stride, head, anchor in zip((8, 16, 32), sorted(heads, key=lambda x: -x.shape[-1]), anchors):
        _, _, h, w = head.shape
        v = head.reshape(1, 3, 11, h, w).transpose(0, 1, 3, 4, 2)[0]
        gy, gx = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        xy = (v[..., :2] * 2 + np.stack((gx, gy), -1) + grid_offset) * stride
        wh = (v[..., 2:4] * 2) ** 2 * anchor[:, None, None, :]
        boxes = np.concatenate((xy-wh/2, xy+wh/2), -1).reshape(-1, 4)
        score = (v[..., 4:5] * v[..., 5:]).reshape(-1, 6)
        if multi_label:
            i, c = np.nonzero(score > threshold)
        else:
            c = score.argmax(1)
            i = np.nonzero(score[np.arange(len(c)), c] > threshold)[0]
            c = c[i]
        rows.append(np.concatenate((boxes[i], score[i, c, None], c[:, None]), 1))
    return np.concatenate(rows)


def box_ious(boxes, gt):
    import numpy as np
    inter = np.maximum(np.minimum(boxes[:, 2:], gt[2:])-np.maximum(boxes[:, :2], gt[:2]), 0).prod(1)
    return inter / np.maximum((boxes[:, 2:]-boxes[:, :2]).prod(1)+(gt[2:]-gt[:2]).prod()-inter, 1e-9)


def evaluate(predictions, names):
    import numpy as np
    per_class = []
    for cls, name in enumerate(names):
        n_gt = sum(x['gt_class'] == cls for x in predictions)
        candidates = []
        for rec in predictions:
            d = np.array(rec['detections'], dtype=float).reshape(-1, 6)
            d = d[d[:, 5] == cls]
            ious = box_ious(d[:, :4], np.array(rec['gt_box'])) if len(d) else np.empty(0)
            # One labelled object per image; only highest-IoU detection can match at each threshold.
            for k, row in enumerate(d):
                tp = [bool(rec['gt_class'] == cls and ious[k] >= t and k == int(np.argmax(ious)))
                      for t in np.linspace(.5, .95, 10)]
                candidates.append((row[4], tp))
        candidates.sort(key=lambda x: -x[0])
        ap = []
        for j in range(10):
            tp = np.array([x[1][j] for x in candidates], dtype=float).cumsum()
            recall = tp / max(n_gt, 1)
            precision = tp / np.arange(1, len(tp)+1)
            mrec = np.concatenate(([0.], recall, [1.]))
            mpre = np.concatenate(([1.], precision, [0.]))
            mpre = np.maximum.accumulate(mpre[::-1])[::-1]
            ap.append(float(np.trapz(np.interp(np.linspace(0, 1, 101), mrec, mpre), np.linspace(0, 1, 101))))
        per_class.append(dict(id=cls, name=name, gt=n_gt, ap50=ap[0], ap50_95=float(np.mean(ap))))
    matrix = np.zeros((6, 7), dtype=int)
    false_action_images, localized_false_action_images = [], []
    correct_counts, any_correct_counts = [0]*6, [0]*6
    for rec in predictions:
        d = np.array(rec['detections'], dtype=float).reshape(-1, 6)
        d = d[d[:, 4] >= .25]
        ious = box_ious(d[:, :4], np.array(rec['gt_box'])) if len(d) else np.empty(0)
        gt = rec['gt_class']
        if len(d) and ious[0] >= .5:
            pred = int(d[0, 5])
        else:
            pred = 6
        matrix[gt, pred] += 1
        correct_counts[gt] += int(pred == gt)
        any_correct_counts[gt] += int(bool(((d[:, 5] == gt) & (ious >= .5)).any()))
        if gt >= 4 and bool((d[:, 5] < 4).any()):
            false_action_images.append(rec['stem'])
        if gt >= 4 and bool(((d[:, 5] < 4) & (ious >= .5)).any()):
            localized_false_action_images.append(rec['stem'])
    return dict(images=len(predictions), per_class=per_class, map50=float(np.mean([x['ap50'] for x in per_class])),
                map50_95=float(np.mean([x['ap50_95'] for x in per_class])),
                working_point=dict(conf=.25, localization_iou=.5, nms_iou=.6,
                                   top1_localized_confusion=matrix.tolist(), top1_correct_per_class=correct_counts,
                                   any_localized_correct_per_class=any_correct_counts,
                                   distractor_images_with_any_actionable=false_action_images,
                                   distractor_images_with_localized_actionable=localized_false_action_images))


def infer(args, files):
    import cv2
    import numpy as np
    import onnx
    import onnxruntime as ort
    from PIL import Image, ImageDraw
    model = onnx.load(args.package / 'models/best.onnx')
    onnx.checker.check_model(model)
    shape = lambda x: [d.dim_value or d.dim_param for d in x.type.tensor_type.shape.dim]
    info = dict(checker='passed', opsets=[dict(domain=x.domain, version=x.version) for x in model.opset_import],
                inputs={x.name: shape(x) for x in model.graph.input}, outputs={x.name: shape(x) for x in model.graph.output},
                operators=dict(Counter(x.op_type for x in model.graph.node)),
                versions=dict(python=sys.version, numpy=np.__version__, onnx=onnx.__version__, onnxruntime=ort.__version__, opencv=cv2.__version__))
    save(args.out / 'onnx_structure.json', info)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(args.package / 'models/best.onnx'), sess_options=options, providers=['CPUExecutionProvider'])
    anchors = np.loadtxt(args.package / 'config/RK_anchors.txt').reshape(3, 3, 2)
    names = [x['name'] for x in json.loads((args.package / 'config/class_map.json').read_text(encoding='utf-8'))['classes']]
    predictions, bgr_predictions, bestclass_predictions = [], [], []
    sheet = Image.new('RGB', (1800, 6*360), '#181818')
    draw = ImageDraw.Draw(sheet)
    elapsed = []
    shifts = []
    with zipfile.ZipFile(args.source) as z:
        members = {Path(x).name.lower(): x for x in z.namelist() if not x.endswith('/')}
        for index, rec in enumerate(x for x in files if x['split'] == 'test'):
            jpg = z.read(members[(rec['stem']+'.jpg').lower()])
            im = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            r = min(640/w, 640/h)
            nw, nh = round(w*r), round(h*r)
            dx, dy = (640-nw)//2, (640-nh)//2
            lb = cv2.copyMakeBorder(cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR), dy, 640-nh-dy, dx, 640-nw-dx, cv2.BORDER_CONSTANT, value=(114,114,114))
            xml = ET.fromstring(z.read(members[(rec['stem']+'.xml').lower()]))
            bbox = xml.find('object/bndbox')
            gt = [float(bbox.findtext(k)) for k in ['xmin','ymin','xmax','ymax']]
            for tag, image, destination in [('rgb', lb, predictions), ('bgr', lb[:,:,::-1], bgr_predictions)]:
                tensor = np.ascontiguousarray(image.transpose(2,0,1)[None], dtype=np.float32)/255
                start = time.perf_counter()
                heads = session.run(None, {session.get_inputs()[0].name: tensor})
                elapsed.append(time.perf_counter()-start)
                det = nms(decode(heads, anchors), .6)
                if tag == 'rgb':
                    bestclass = nms(decode(heads, anchors, multi_label=False, threshold=.25), .6)
                    bestclass[:, [0,2]] = np.clip((bestclass[:, [0,2]]-dx)/r, 0, w)
                    bestclass[:, [1,3]] = np.clip((bestclass[:, [1,3]]-dy)/r, 0, h)
                    bestclass_predictions.append(dict(stem=rec['stem'], gt_class=rec['class_id'], gt_box=gt,
                                                       detections=bestclass.tolist()))
                    # Same head candidate confidence, missing grid offset causes stride/2 displacement.
                    old = decode(heads, anchors, grid_offset=0)
                    good = decode(heads, anchors)
                    shifts.append(dict(stem=rec['stem'], unique_xyxy_offsets=np.unique(np.round(old[:,:4]-good[:,:4], 4),axis=0).tolist()))
                det[:, [0,2]] = np.clip((det[:, [0,2]]-dx)/r, 0, w)
                det[:, [1,3]] = np.clip((det[:, [1,3]]-dy)/r, 0, h)
                destination.append(dict(stem=rec['stem'], gt_class=rec['class_id'], gt_box=gt, detections=det.tolist()))
                if tag == 'rgb':
                    thumb = Image.fromarray(rgb)
                    thumb.thumbnail((295,320))
                    pen = ImageDraw.Draw(thumb)
                    scale = thumb.width/w
                    pen.rectangle([v*scale for v in gt], outline='#00ff00', width=2)
                    for row in det[det[:,4] >= .25]:
                        box = [v*scale for v in row[:4]]
                        pen.rectangle(box, outline='#ff4040', width=2)
                        pen.text((box[0],max(0,box[1]-11)), f'{int(row[5])} {row[4]:.2f}', fill='red')
                    x, y = (index%6)*300, (index//6)*360
                    sheet.paste(thumb, (x,y+24))
                    draw.text((x+2,y+2), f"{rec['stem']} GT:{rec['class_id']}", fill='white')
            print(f'inference {index+1}/36 {rec["stem"]}', flush=True)
    save(args.out / 'rgb_predictions.json', predictions)
    save(args.out / 'bgr_diagnostic_predictions.json', bgr_predictions)
    save(args.out / 'rgb_bestclass_working_point_predictions.json', bestclass_predictions)
    save(args.out / 'decode_offset_diagnostic.json', shifts)
    summary = dict(protocol='Frozen test split; independent ONNX CPU check, no tuning. RGB/OpenCV bilinear fixed 640 square; sigmoid heads; grid=index-0.5; multi-label NMS 0.6; low conf 0.001; working conf 0.25. Not identical to original PyTorch rectangular batch evaluation.',
                   rgb=evaluate(predictions,names), bgr_diagnostic=evaluate(bgr_predictions,names),
                   rgb_bestclass_working_point=evaluate(bestclass_predictions,names)['working_point'],
                   cpu_inference_seconds=dict(mean=float(np.mean(elapsed)), min=min(elapsed), max=max(elapsed)),
                   note='CPU timings are diagnostic only, not RK performance. Both paths include the same fixed model, anchors and postprocessing. No thresholds or weights optimized on test data.')
    save(args.out / 'independent_evaluation.json', summary)
    sheet.save(args.out / 'corrected_rgb_test_sheet.jpg', quality=90)
    print(json.dumps(summary, ensure_ascii=False), flush=True)


def parity(args, files):
    """Compare the delivered TorchScript and ONNX, not the sender's temporary snapshot."""
    import cv2
    import numpy as np
    import onnxruntime as ort
    import torch
    torch.set_num_threads(4)
    # torch 2.5.1 on Windows may fail fopen for non-ASCII paths; Python handles the bytes.
    model = torch.jit.load(io.BytesIO((args.package / 'models/best.torchscript').read_bytes()), map_location='cpu').eval()
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 4
    session = ort.InferenceSession(str(args.package / 'models/best.onnx'), sess_options=opts, providers=['CPUExecutionProvider'])
    rows = []
    with zipfile.ZipFile(args.source) as z:
        members = {Path(x).name.lower(): x for x in z.namelist() if not x.endswith('/')}
        for rec in (x for x in files if x['split'] == 'test'):
            im = cv2.imdecode(np.frombuffer(z.read(members[(rec['stem']+'.jpg').lower()]),np.uint8),cv2.IMREAD_COLOR)
            im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
            h,w = im.shape[:2]
            r = min(640/w,640/h)
            nw,nh = round(w*r),round(h*r)
            dx,dy = (640-nw)//2,(640-nh)//2
            im = cv2.copyMakeBorder(cv2.resize(im,(nw,nh)),dy,640-nh-dy,dx,640-nw-dx,cv2.BORDER_CONSTANT,value=(114,114,114))
            tensor = np.ascontiguousarray(im.transpose(2,0,1)[None],dtype=np.float32)/255
            expected = session.run(None,{session.get_inputs()[0].name:tensor})
            with torch.inference_mode():
                actual = model(torch.from_numpy(tensor))
            if not isinstance(actual,(tuple,list)):
                actual = [actual]
            errors = [float(np.abs(x-y.numpy()).max()) for x,y in zip(expected,actual)]
            assert len(actual) == len(expected) == 3
            rows.append(dict(stem=rec['stem'],head_max_absolute_errors=errors))
    save(args.out / 'delivered_torchscript_onnx_parity.json',dict(
        torch=torch.__version__, images=len(rows), onnx_sha256=file_sha(args.package/'models/best.onnx'),
        torchscript_sha256=file_sha(args.package/'models/best.torchscript'),
        max_absolute_error=max(max(x['head_max_absolute_errors']) for x in rows), rows=rows,
        note='CPU fp32 on identical RGB tensors. This compares delivered three-head exports, not best.pt loading or RKNN.'))
    print('delivered TorchScript/ONNX max head error:',max(max(x['head_max_absolute_errors']) for x in rows),flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--package', required=True, type=Path)
    ap.add_argument('--source', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--runtime', type=Path)
    ap.add_argument('--torch-runtime', type=Path)
    ap.add_argument('--integrity-only', action='store_true')
    ap.add_argument('--parity-only', action='store_true')
    args = ap.parse_args()
    if args.out.resolve().is_relative_to(args.package.resolve()):
        raise ValueError('output must be outside the original delivery directory')
    if args.runtime:
        sys.path.insert(0, str(args.runtime))
    if args.torch_runtime:
        sys.path.insert(0, str(args.torch_runtime))
    args.out.mkdir(parents=True, exist_ok=True)
    files = audit(args)
    if not args.integrity_only and not args.parity_only:
        infer(args, files)
    if args.torch_runtime and not args.integrity_only:
        parity(args, files)


if __name__ == '__main__':
    main()
