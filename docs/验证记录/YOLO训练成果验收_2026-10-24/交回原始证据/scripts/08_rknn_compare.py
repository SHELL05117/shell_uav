"""Stage 8: quantised (.rknn) vs floating-point (.onnx) comparison on the test split.

Runs on the toolkit **simulator** (target=None): it gives accuracy-level comparison of
the 8-bit asymmetric model, not RK3399Pro latency.  Real NPU timing and the board-side
C++ post-process stay on the target-machine checklist.

Kept Python 3.6 compatible because it must run inside env/venv-rknn, side by side with
the converter toolkit.  The decoding maths mirrors 06_infer_onnx.py exactly
(three sigmoid heads, anchors in pixels, per-class NMS).

Usage (inside env\\venv-rknn):
  python 08_rknn_compare.py --onnx best.onnx --rknn model.rknn --conf 0.25
"""
from __future__ import print_function

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
NAMES = ["person_marker", "hazard_toxic", "hazard_flammable", "hazard_explosive", "distractor_1", "distractor_2"]
ACTIONABLE = set(range(4))
STRIDES = (8, 16, 32)
CONF = 0.25
IOU = 0.45


def letterbox(im, new=640, colour=(114, 114, 114)):
    w, h = im.size
    r = min(float(new) / w, float(new) / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = im.resize((nw, nh), Image.BILINEAR)
    canvas = Image.new("RGB", (new, new), colour)
    dx, dy = (new - nw) // 2, (new - nh) // 2
    canvas.paste(resized, (dx, dy))
    return np.asarray(canvas, dtype=np.uint8), r, dx, dy


def load_anchors(path):
    vals = [float(x) for x in Path(path).read_text().split() if x.strip()]
    return np.array(vals, dtype=np.float32).reshape(3, 3, 2)


def decode(outs, anchors, conf_thres):
    boxes, scores, classes = [], [], []
    for li, out in enumerate(outs):
        out = np.asarray(out, dtype=np.float32)
        na, no = 3, 5 + len(NAMES)
        _, _, ny, nx = out.shape
        pred = out.reshape(1, na, no, ny, nx).transpose(0, 1, 3, 4, 2)[0]
        gy, gx = np.meshgrid(np.arange(ny), np.arange(nx), indexing="ij")
        grid = np.stack((gx, gy), -1).astype(np.float32)
        s = STRIDES[li]
        xy = (pred[..., 0:2] * 2.0 + grid) * s
        wh = (pred[..., 2:4] * 2.0) ** 2 * anchors[li][:, None, None, :]
        score = pred[..., 4][..., None] * pred[..., 5:]
        keep = score > conf_thres
        if not keep.any():
            continue
        a, y, x, c = np.nonzero(keep)
        boxes.append(np.concatenate([xy[a, y, x] - wh[a, y, x] / 2, xy[a, y, x] + wh[a, y, x] / 2], axis=1))
        scores.append(score[a, y, x, c])
        classes.append(c)
    if not boxes:
        return np.zeros((0, 6), dtype=np.float32)
    return np.concatenate([np.concatenate(boxes),
                           np.concatenate(scores)[:, None],
                           np.concatenate(classes)[:, None].astype(np.float32)], axis=1)


def nms(det, iou_thres, max_wh=7680.0):
    if len(det) == 0:
        return det
    boxes = det[:, :4] + det[:, 5:6] * max_wh
    order = np.argsort(-det[:, 4])
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        rest = order[1:]
        if rest.size == 0:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        ai = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        ar = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        iou = inter / np.clip(ai + ar - inter, 1e-9, None)
        order = rest[iou <= iou_thres]
    return det[np.array(keep, dtype=int)]


def unletterbox(box, ratio, dx, dy, w, h):
    out = box.copy()
    out[:, [0, 2]] = np.clip((out[:, [0, 2]] - dx) / ratio, 0, w)
    out[:, [1, 3]] = np.clip((out[:, [1, 3]] - dy) / ratio, 0, h)
    return out


def iou_between(a, b):
    xx1 = max(a[0], b[0]); yy1 = max(a[1], b[1])
    xx2 = min(a[2], b[2]); yy2 = min(a[3], b[3])
    inter = max(0.0, xx2 - xx1) * max(0.0, yy2 - yy1)
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / max(area, 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--rknn", required=True)
    ap.add_argument("--images", default=str(ROOT / "work" / "dataset" / "sixclass" / "images" / "test"))
    ap.add_argument("--anchors", default=str(ROOT / "work" / "repos" / "yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab" / "RK_anchors.txt"))
    ap.add_argument("--out", default=str(ROOT / "work" / "rknn"))
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--iou", type=float, default=IOU)
    args = ap.parse_args()

    import onnxruntime as ort
    from rknn.api import RKNN

    anchors = load_anchors(args.anchors)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    rknn = RKNN(verbose=False)
    ret = rknn.config(channel_mean_value="0 0 0 255", reorder_channel="0 1 2",
                      target_platform="rk3399pro", quantized_dtype="asymmetric_quantized-u8",
                      optimization_level=3, batch_size=1)
    if ret != 0:
        raise RuntimeError("config failed")
    if rknn.load_rknn(args.rknn) != 0:
        raise RuntimeError("load_rknn failed")
    if rknn.init_runtime(target=None) != 0:
        raise RuntimeError("init_runtime(simulator) failed")

    images = sorted(p for p in Path(args.images).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    rows = []
    ious, class_agree, fp_conf = [], 0, []
    float_counts, quant_counts = {}, {}
    for p in images:
        im0 = Image.open(str(p)).convert("RGB")
        arr, ratio, dx, dy = letterbox(im0)
        fdet = nms(decode(sess.run(None, {in_name: (arr.transpose(2, 0, 1)[None] / 255.0).astype(np.float32)}),
                          anchors, args.conf), args.iou)
        qouts = rknn.inference(inputs=[arr[None]], data_format="nhwc")
        qdet = nms(decode(qouts, anchors, args.conf), args.iou)
        if len(fdet):
            fdet[:, :4] = unletterbox(fdet[:, :4], ratio, dx, dy, im0.width, im0.height)
        if len(qdet):
            qdet[:, :4] = unletterbox(qdet[:, :4], ratio, dx, dy, im0.width, im0.height)
        for tag, det, count in (("float", fdet, float_counts), ("quant", qdet, quant_counts)):
            for d in det:
                count[int(d[5])] = count.get(int(d[5]), 0) + 1
        row = {"image": p.name, "float_n": int(len(fdet)), "quant_n": int(len(qdet))}
        if len(fdet) and len(qdet):
            fbest = fdet[int(np.argmax(fdet[:, 4]))]
            qbest = qdet[int(np.argmax(qdet[:, 4]))]
            iou = iou_between(fbest[:4], qbest[:4])
            agree = int(fbest[5]) == int(qbest[5])
            rows.append(row)
            row.update({"top1_float": NAMES[int(fbest[5])], "top1_float_conf": round(float(fbest[4]), 4),
                        "top1_quant": NAMES[int(qbest[5])], "top1_quant_conf": round(float(qbest[4]), 4),
                        "top1_iou": round(float(iou), 4), "top1_class_agree": agree})
            ious.append(iou)
            class_agree += int(agree)
            fp_conf.append(abs(float(fbest[4]) - float(qbest[4])))
        else:
            rows.append(row)
            row.update({"top1_float": NAMES[int(fdet[0][5])] if len(fdet) else None,
                        "top1_quant": NAMES[int(qdet[0][5])] if len(qdet) else None})

    rknn.release()
    summary = {
        "onnx": args.onnx, "rknn": args.rknn, "images": len(images),
        "conf": args.conf, "iou": args.iou,
        "top1_compared": len(ious),
        "top1_class_agreement": class_agree,
        "top1_iou_mean": round(float(np.mean(ious)), 4) if ious else None,
        "top1_iou_min": round(float(np.min(ious)), 4) if ious else None,
        "top1_conf_absdiff_mean": round(float(np.mean(fp_conf)), 5) if fp_conf else None,
        "float_detections_per_class": {NAMES[k]: v for k, v in sorted(float_counts.items())},
        "quant_detections_per_class": {NAMES[k]: v for k, v in sorted(quant_counts.items())},
        "note": "toolkit simulator (target=None); NPU timing and board post-process are NOT measured here",
    }
    (out_dir / "quant_vs_float.json").write_text(json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False, indent=1),
                                                 encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
