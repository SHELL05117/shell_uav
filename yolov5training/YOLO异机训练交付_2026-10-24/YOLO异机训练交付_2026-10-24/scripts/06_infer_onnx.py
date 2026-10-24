"""Stage 6: RK-style ONNX inference reference implementation + parity check.

The RK3399Pro demo consumes the ONNX exported by this repo's `export.py --rknpu`:
  * input  1x3x640x640, RGB, float32, /255, letterbox padding = 114
  * output 3 tensors, one per stride level, already sigmoid-activated,
    shape (1, 3*(5+nc), H, W) with H/W = 80/40/20 for a 640 input
  * anchors in pixel units per level are written by export.py to RK_anchors.txt

This module decodes those tensors the same way the C++ post-process must, so the
numbers can be checked against the PyTorch model on identical input before any RKNN
conversion.  Everything here is framework-only: no RK hardware is touched.

Usage:
  python 06_infer_onnx.py --onnx model.onnx --images <dir> --out <dir> [--conf 0.25] [--iou 0.45]
  python 06_infer_onnx.py --onnx model.onnx --images <dir> --out <dir> --parity-ckpt best.pt
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
REPO = ROOT / "work" / "repos" / "yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab"
CLASS_MAP = json.loads((ROOT / "YOLO异机训练交接包_2026-10-24" / "config" / "class_map.json").read_text(encoding="utf-8"))
NAMES = [c["name"] for c in CLASS_MAP["classes"]]
ACTIONABLE = {c["id"] for c in CLASS_MAP["classes"] if c["actionable"]}
STRIDES = (8, 16, 32)


def letterbox(im: Image.Image, new=640, colour=(114, 114, 114)):
    w, h = im.size
    r = min(new / w, new / h)
    nw, nh = round(w * r), round(h * r)
    resized = im.resize((nw, nh), Image.Resampling.BILINEAR)
    canvas = Image.new("RGB", (new, new), colour)
    dx, dy = (new - nw) // 2, (new - nh) // 2
    canvas.paste(resized, (dx, dy))
    return np.asarray(canvas, dtype=np.float32), (r, dx, dy)


def load_anchors(path: Path, nc: int) -> np.ndarray:
    vals = [float(x) for x in path.read_text().split() if x.strip()]
    return np.array(vals, dtype=np.float32).reshape(3, 3, 2)  # level, anchor, (w, h) in pixels


def decode(outputs: list[np.ndarray], anchors: np.ndarray, conf_thres: float) -> np.ndarray:
    """Returns (n, 6) = x1, y1, x2, y2, score, class_id in letterboxed input pixels."""
    boxes, scores, classes = [], [], []
    for li, out in enumerate(outputs):
        na, no = 3, 5 + len(NAMES)
        _, _, ny, nx = out.shape
        pred = out.reshape(1, na, no, ny, nx).transpose(0, 1, 3, 4, 2)[0]  # (na, ny, nx, no)
        gy, gx = np.meshgrid(np.arange(ny), np.arange(nx), indexing="ij")
        grid = np.stack((gx, gy), -1).astype(np.float32)
        s = STRIDES[li]
        xy = (pred[..., 0:2] * 2.0 + grid) * s
        wh = (pred[..., 2:4] * 2.0) ** 2 * anchors[li][:, None, None, :]
        obj = pred[..., 4]
        cls_prob = pred[..., 5:]
        score = obj[..., None] * cls_prob
        keep = score > conf_thres
        if not keep.any():
            continue
        a, y, x, c = np.nonzero(keep)
        xy_ = xy[a, y, x]
        wh_ = wh[a, y, x]
        boxes.append(np.concatenate([xy_ - wh_ / 2, xy_ + wh_ / 2], axis=1))
        scores.append(score[a, y, x, c])
        classes.append(c)
    if not boxes:
        return np.zeros((0, 6), dtype=np.float32)
    b = np.concatenate(boxes)
    sc = np.concatenate(scores)
    cl = np.concatenate(classes)
    return np.concatenate([b, sc[:, None], cl[:, None].astype(np.float32)], axis=1)


def nms(det: np.ndarray, iou_thres: float, nc: int | None = None) -> np.ndarray:
    """Per-class NMS with plain numpy, mirroring YOLOv5's non_max_suppression.

    YOLOv5 offsets boxes by class*max_wh so that different classes never suppress
    each other; the C++ post-process must do the same.
    """
    if len(det) == 0:
        return det
    nc = len(NAMES) if nc is None else nc
    max_wh = 7680.0
    boxes = det[:, :4] + det[:, 5:6] * max_wh
    order = np.argsort(-det[:, 4])
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        w = np.clip(xx2 - xx1, 0, None)
        h = np.clip(yy2 - yy1, 0, None)
        inter = w * h
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        iou = inter / np.clip(area_i + area_r - inter, 1e-9, None)
        order = rest[iou <= iou_thres]
    return det[np.array(keep, dtype=int)]


def unletterbox(box_xyxy: np.ndarray, ratio: float, dx: int, dy: int, w: int, h: int) -> np.ndarray:
    out = box_xyxy.copy()
    out[:, [0, 2]] = (out[:, [0, 2]] - dx) / ratio
    out[:, [1, 3]] = (out[:, [1, 3]] - dy) / ratio
    out[:, [0, 2]] = np.clip(out[:, [0, 2]], 0, w)
    out[:, [1, 3]] = np.clip(out[:, [1, 3]], 0, h)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True, type=Path)
    ap.add_argument("--images", required=True, type=Path, help="directory with jpg images")
    ap.add_argument("--anchors", type=Path, default=ROOT / "work" / "repos" / "yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab" / "RK_anchors.txt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--out", type=Path, default=ROOT / "work" / "reports" / "onnx_infer")
    ap.add_argument("--parity-ckpt", type=Path, default=None, help="PyTorch checkpoint to compare against")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    anchors = load_anchors(args.anchors, len(NAMES))
    sess = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    images = sorted(p for p in args.images.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if args.limit:
        images = images[: args.limit]

    parity = None
    if args.parity_ckpt:
        import torch
        sys.path.insert(0, str(REPO))
        from models.common import DetectMultiBackend
        from utils.general import non_max_suppression
        # parity runs on CPU on purpose: TF32 on Ampere would add ~1e-2 activation
        # differences between the two frameworks and blur the comparison
        dev = torch.device("cpu")
        pmodel = DetectMultiBackend(str(args.parity_ckpt), device=dev, dnn=False, data=str(ROOT / "work" / "dataset" / "sixclass" / "data.yaml"))
        pmodel.eval()
        parity = {"images": 0, "max_box_diff_px": 0.0, "class_mismatch": 0, "conf_max_diff": 0.0, "matched": 0}

    results = {}
    sheets = []
    for img_path in images:
        with Image.open(img_path) as im0:
            im0 = im0.convert("RGB")
        x, (ratio, dx, dy) = letterbox(im0)
        # YOLOv5 trains on RGB CHW /255 input, so the ONNX input is RGB (no channel flip here).
        inp = (x.transpose(2, 0, 1)[None] / 255.0).astype(np.float32)
        outs = sess.run(None, {in_name: inp})
        det = nms(decode(outs, anchors, args.conf), args.iou)
        if len(det):
            det[:, :4] = unletterbox(det[:, :4], ratio, dx, dy, im0.width, im0.height)
        rows = [{
            "class_id": int(d[5]), "class_name": NAMES[int(d[5])],
            "actionable": int(d[5]) in ACTIONABLE,
            "conf": round(float(d[4]), 4),
            "xyxy": [round(float(v), 1) for v in d[:4]],
        } for d in det]
        results[img_path.name] = {"width": im0.width, "height": im0.height, "detections": rows,
                                  "letterbox": {"ratio": ratio, "pad_x": dx, "pad_y": dy}}

        canvas = im0.copy()
        d = ImageDraw.Draw(canvas)
        for r in rows:
            col = (255, 60, 60) if r["actionable"] else (0, 170, 255)
            d.rectangle(r["xyxy"], outline=col, width=max(2, canvas.width // 500))
            d.text((r["xyxy"][0] + 2, max(0, r["xyxy"][1] - 11)), f"{r['class_name']} {r['conf']:.2f}", fill=col)
        canvas.thumbnail((760, 760), Image.Resampling.LANCZOS)
        sheets.append((img_path.stem, canvas))

        if parity is not None:
            # ---- three-level parity -------------------------------------------
            # (1) head parity: ONNX outputs vs the PyTorch Conv heads + sigmoid.
            # (2) decode parity: the same head tensors decoded by this module.
            # (3) detection parity: best-match IoU after each side's own NMS
            #     (informational: the two NMS implementations may keep different
            #      near-duplicate boxes).
            heads = []
            hooks = []
            det_mod = pmodel.model.model[-1]
            for i, m in enumerate(det_mod.m):
                hooks.append(m.register_forward_hook(
                    lambda mod, _i, o, i=i: heads.append(torch.sigmoid(o.detach()))))
            with torch.no_grad():
                pt_out = pmodel(torch.from_numpy(inp).to(dev))
            for h in hooks:
                h.remove()
            heads.sort(key=lambda t: -t.shape[-1])
            parity["images"] += 1
            for li in range(3):
                d = float(np.abs(heads[li].cpu().numpy() - outs[li]).max())
                parity["head_max_abs_diff"] = max(parity.get("head_max_abs_diff", 0.0), d)

            cand_onnx = decode(outs, anchors, args.conf)
            cand_torch = decode([h.cpu().numpy() for h in heads], anchors, args.conf)
            if len(cand_onnx) and len(cand_torch):
                a = cand_onnx[np.argmax(cand_onnx[:, 4])]
                b = cand_torch[np.argmax(cand_torch[:, 4])]
                parity["decode_top1_box_diff_px"] = max(
                    parity.get("decode_top1_box_diff_px", 0.0), float(np.abs(a[:4] - b[:4]).max()))
                parity["decode_top1_conf_diff"] = max(
                    parity.get("decode_top1_conf_diff", 0.0), float(abs(a[4] - b[4])))

            pt_det = non_max_suppression(pt_out, args.conf, args.iou, max_det=100)[0].cpu().numpy()
            if len(pt_det):
                pt_det[:, :4] = unletterbox(pt_det[:, :4], ratio, dx, dy, im0.width, im0.height)
            iou_list = []
            for r in rows:
                if len(pt_det) == 0:
                    continue
                best_iou = 0.0
                for pb in pt_det:
                    xx1 = max(r["xyxy"][0], pb[0]); yy1 = max(r["xyxy"][1], pb[1])
                    xx2 = min(r["xyxy"][2], pb[2]); yy2 = min(r["xyxy"][3], pb[3])
                    inter = max(0.0, xx2 - xx1) * max(0.0, yy2 - yy1)
                    union = ((r["xyxy"][2] - r["xyxy"][0]) * (r["xyxy"][3] - r["xyxy"][1]) +
                             (pb[2] - pb[0]) * (pb[3] - pb[1]) - inter)
                    best_iou = max(best_iou, inter / max(union, 1e-9))
                iou_list.append(best_iou)
            parity["detections_compared"] = parity.get("detections_compared", 0) + len(rows)
            parity["nms_match_iou_sum"] = parity.get("nms_match_iou_sum", 0.0) + sum(iou_list)
            if iou_list:
                parity["nms_match_iou_min"] = min(parity.get("nms_match_iou_min", 1.0), min(iou_list))

    (args.out / "predictions.json").write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")
    if sheets:
        cols, tw, th = 5, 300, 340
        rws = (len(sheets) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * tw, rws * th), (18, 18, 18))
        d = ImageDraw.Draw(sheet)
        for i, (stem, im) in enumerate(sheets):
            im.thumbnail((tw - 4, th - 16), Image.Resampling.LANCZOS)
            sheet.paste(im, ((i % cols) * tw + (tw - im.width) // 2, (i // cols) * th + 14))
            d.text(((i % cols) * tw + 3, (i // cols) * th + 1), stem, fill=(235, 235, 235))
        sheet.save(args.out / "onnx_predictions_sheet.jpg", quality=85)

    summary = {"onnx": str(args.onnx), "images": len(images), "conf": args.conf, "iou": args.iou,
               "total_detections": sum(len(v["detections"]) for v in results.values()),               "per_class": {NAMES[c]: sum(1 for v in results.values() for d in v["detections"] if d["class_id"] == c)
                             for c in range(len(NAMES))}}
    if parity:
        parity["nms_match_iou_mean"] = (
            parity.get("nms_match_iou_sum", 0.0) / parity["detections_compared"] if parity.get("detections_compared") else None)
        summary["parity_vs_pytorch"] = {k: (float(v) if isinstance(v, (np.floating, np.integer)) else v)
                                        for k, v in parity.items()}
    (args.out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
