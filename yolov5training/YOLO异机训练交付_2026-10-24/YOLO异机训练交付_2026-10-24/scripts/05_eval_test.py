"""Stage 5: evaluate a trained checkpoint on one split of the derived dataset.

Produces, in one run:
  * per-class precision / recall / mAP@0.5 / mAP@0.5:0.95 (YOLOv5 ap_per_class)
  * a 7x7 confusion matrix at a documented working point (6 classes + background)
  * the task-specific failures the handoff asks for: distractor predicted as an
    actionable class, actionable class predicted as distractor, actionable class missed
  * annotated images (green = ground truth, red = prediction) for the whole split

The split must not be used for tuning; run it once on "test" after model selection.

Usage:
  python 05_eval_test.py --weights <best.pt> --split test --out work/reports/eval_test
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import torch
import yaml
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
REPO = ROOT / "work" / "repos" / "yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab"
sys.path.insert(0, str(REPO))

from models.common import DetectMultiBackend  # noqa: E402
from utils.augmentations import letterbox  # noqa: E402
from utils.general import non_max_suppression, scale_boxes  # noqa: E402
from utils.metrics import ap_per_class, box_iou  # noqa: E402

DS = ROOT / "work" / "dataset" / "sixclass"
PKG = next(p for p in ROOT.iterdir() if p.is_dir() and (p / "config" / "class_map.json").exists())
CLASS_MAP = json.loads((PKG / "config" / "class_map.json").read_text(encoding="utf-8"))
NAMES = {c["id"]: c["name"] for c in CLASS_MAP["classes"]}
ACTIONABLE = {c["id"] for c in CLASS_MAP["classes"] if c["actionable"]}
DISTRACTOR = {c["id"] for c in CLASS_MAP["classes"] if not c["actionable"]}
BG = len(NAMES)  # background index in the confusion matrix


def load_split(split: str):
    manifest = json.loads((DS / "split_manifest.json").read_text(encoding="utf-8"))
    rows = [r for r in manifest["files"] if r["split"] == split]
    return sorted(rows, key=lambda r: r["stem"])


def gt_pixels(label_path: Path, w: int, h: int):
    boxes, classes = [], []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        c, xc, yc, bw, bh = line.split()
        c, xc, yc, bw, bh = int(c), float(xc), float(yc), float(bw), float(bh)
        boxes.append([(xc - bw / 2) * w, (yc - bh / 2) * h, (xc + bw / 2) * w, (yc + bh / 2) * h])
        classes.append(c)
    return np.array(boxes, dtype=np.float32).reshape(-1, 4), np.array(classes, dtype=np.int64)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True, type=Path)
    ap.add_argument("--split", default="test", choices=("train", "val", "test"))
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25, help="working-point threshold for the tables")
    ap.add_argument("--iou", type=float, default=0.5, help="match IoU for the tables")
    ap.add_argument("--out", type=Path, default=ROOT / "work" / "reports" / "eval_test")
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    device = torch.device(f"cuda:{args.device}" if args.device.isdigit() and torch.cuda.is_available() else "cpu")
    model = DetectMultiBackend(str(args.weights), device=device, dnn=False, data=str(DS / "data.yaml"), fp16=False)
    stride, names, pt = model.stride, model.names, model.pt
    model.eval()
    model.warmup(imgsz=(1, 3, args.imgsz, args.imgsz))

    rows = load_split(args.split)
    stats = {"tp": [], "conf": [], "pred_cls": [], "target_cls": []}
    conf_mat = np.zeros((BG + 1, BG + 1), dtype=np.int64)  # rows = GT (last = none), cols = pred (last = none)
    per_image = []
    matched_ious = []
    annotated = []
    det_rows = []

    for rec in rows:
        img_path = DS / rec["image"]
        with Image.open(img_path) as im0:
            im0 = im0.convert("RGB")
        orig = np.array(im0)
        gt_box, gt_cls = gt_pixels(DS / rec["label"], im0.width, im0.height)
        img_lb, ratio, pad = letterbox(orig, (args.imgsz, args.imgsz), stride=stride, auto=pt)
        im = np.ascontiguousarray(img_lb[:, :, ::-1].transpose(2, 0, 1))
        im = torch.from_numpy(im).to(device).float() / 255.0
        im = im[None]
        with torch.no_grad():
            pred = model(im, augment=False, visualize=False)
        det = non_max_suppression(pred, args.conf if args.conf < 0.01 else 0.001, 0.65, classes=None,
                                  agnostic=False, max_det=100)[0]
        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], orig.shape).round() if len(det) else det[:, :4]

        # ---- AP inputs (low conf, IoU sweep handled by caller metrics) ----
        if len(det):
            iou = box_iou(torch.from_numpy(gt_box), det[:, :4].cpu())
            iou_max, idx = iou.max(1) if iou.numel() else (torch.zeros(len(gt_box)), torch.zeros(len(gt_box), dtype=torch.long))
            for gi in range(len(gt_box)):
                stats["target_cls"].append(int(gt_cls[gi]))
            for di in range(len(det)):
                # TP when a ground-truth box of the same class reaches IoU >= 0.5 (one GT per image
                # here, so duplicate claims cannot occur on this dataset)
                cls_d = int(det[di, 5])
                same = ((gt_cls == cls_d) & (iou[:, di] >= 0.5).cpu().numpy()) if iou.numel() else np.zeros(0, dtype=bool)
                stats["tp"].append(1 if same.any() else 0)
                stats["conf"].append(float(det[di, 4]))
                stats["pred_cls"].append(cls_d)
                det_rows.append({
                    "stem": rec["stem"], "class_id": cls_d, "class_name": NAMES.get(cls_d, str(cls_d)),
                    "conf": round(float(det[di, 4]), 4),
                    "box_xyxy": [round(float(v), 1) for v in det[di, :4].tolist()],
                    "iou_with_gt": round(float(iou[:, di].max()) if iou.numel() else 0.0, 3),
                })
        else:
            stats["target_cls"].append(int(gt_cls[0]))

        # ---- working-point confusion for this image (one GT per image) ----
        best_pred = None
        if len(det):
            order = torch.argsort(det[:, 4], descending=True)
            best = det[order[0]]
            ious = box_iou(torch.from_numpy(gt_box).to(device), best[:4].reshape(1, 4))
            if float(ious.max()) >= args.iou:
                best_pred = int(best[5])
                matched_ious.append(float(ious.max()))
        gt_c = int(gt_cls[0])
        if best_pred is None:
            conf_mat[gt_c, BG] += 1
        else:
            conf_mat[gt_c, best_pred] += 1
        per_image.append({
            "stem": rec["stem"], "gt": gt_c, "gt_name": NAMES[gt_c],
            "pred": best_pred, "pred_name": NAMES.get(best_pred) if best_pred is not None else "missed",
            "n_det": int(len(det)),
            "top1_conf": round(float(det[:, 4].max()), 4) if len(det) else 0.0,
        })

        # ---- annotated image ----
        canvas = im0.copy()
        d = ImageDraw.Draw(canvas)
        for b in gt_box:
            d.rectangle([float(b[0]), float(b[1]), float(b[2]), float(b[3])], outline=(0, 255, 0), width=max(2, canvas.width // 400))
        for det_row in det:
            b = det_row[:4].tolist()
            col = (255, 40, 40) if int(det_row[5]) in ACTIONABLE else (0, 160, 255)
            d.rectangle([b[0], b[1], b[2], b[3]], outline=col, width=max(1, canvas.width // 600))
            d.text((b[0] + 3, max(0, b[1] - 12)), f"{NAMES.get(int(det_row[5]),'?')} {float(det_row[4]):.2f}", fill=col)
        canvas.thumbnail((900, 900), Image.Resampling.LANCZOS)
        annotated.append((rec["stem"], canvas))

    # ---- per-class AP (IoU 0.5 and 0.5:0.95) ----
    metric_rows = []
    tp = np.array(stats["tp"], dtype=np.float32)
    conf = np.array(stats["conf"], dtype=np.float32)
    pred_cls = np.array(stats["pred_cls"], dtype=np.int64)
    target_cls = np.array(stats["target_cls"], dtype=np.int64)
    if len(tp):
        results = ap_per_class(tp[:, None], conf, pred_cls, target_cls, names=NAMES)
        p, r, ap, classes = results[2], results[3], results[5], results[6]
        ap50 = ap[:, 0]
        for i, c in enumerate(classes):
            metric_rows.append({
                "class_id": int(c), "class_name": NAMES[int(c)],
                "precision": round(float(p[i]), 4), "recall": round(float(r[i]), 4),
                "mAP50": round(float(ap50[i]), 4),
                "gt_count": int((target_cls == c).sum()), "pred_count": int((pred_cls == c).sum()),
            })

    # ---- task-specific failures ----
    task = {
        "working_point": {"conf": args.conf, "iou": args.iou, "source": "chosen for reporting only; no business threshold was provided"},
        "images": len(rows),
        "distractor_predicted_actionable": [x for x in per_image if x["gt"] in DISTRACTOR and x["pred"] in ACTIONABLE],
        "actionable_predicted_distractor": [x for x in per_image if x["gt"] in ACTIONABLE and x["pred"] in DISTRACTOR],
        "actionable_missed": [x for x in per_image if x["gt"] in ACTIONABLE and x["pred"] is None],
        "correct": sum(1 for x in per_image if x["pred"] == x["gt"]),
        "missed_total": sum(1 for x in per_image if x["pred"] is None),
        "matched_iou_mean": round(float(np.mean(matched_ious)), 4) if matched_ious else None,
    }

    # ---- write outputs ----
    (args.out / "metrics_per_class.json").write_text(json.dumps({
        "weights": str(args.weights), "split": args.split, "rows": metric_rows,
        "note": "single-IoU (0.5) AP per class at the low confidence used for AP input; the full 10-IoU table is produced by YOLOv5 val.py and stored next to this file",
    }, ensure_ascii=False, indent=1), encoding="utf-8")
    (args.out / "per_image_predictions.json").write_text(json.dumps(per_image, ensure_ascii=False, indent=1), encoding="utf-8")
    (args.out / "task_failures.json").write_text(json.dumps(task, ensure_ascii=False, indent=1), encoding="utf-8")

    header = ["GT \\ Pred"] + [f"{c}:{NAMES[c]}" for c in sorted(NAMES)] + ["background"]
    with (args.out / "confusion_matrix.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for gi in range(len(NAMES)):
            w.writerow([f"{gi}:{NAMES[gi]}"] + [int(conf_mat[gi, pi]) for pi in range(BG + 1)])
    with (args.out / "detections.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["stem", "class_id", "class_name", "conf", "box_xyxy", "iou_with_gt"])
        w.writeheader()
        w.writerows(det_rows)

    cols, tw, th = 6, 320, 360
    rws = (len(annotated) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tw, rws * th), (18, 18, 18))
    d = ImageDraw.Draw(sheet)
    for i, (stem, img) in enumerate(annotated):
        img.thumbnail((tw - 4, th - 18), Image.Resampling.LANCZOS)
        x = (i % cols) * tw + (tw - img.width) // 2
        y = (i // cols) * th + 16
        sheet.paste(img, (x, y))
        d.text(((i % cols) * tw + 3, (i // cols) * th + 2), stem, fill=(235, 235, 235))
    sheet.save(args.out / f"{args.split}_predictions_sheet.jpg", quality=85)

    print(json.dumps({
        "split": args.split, "images": len(rows), "metrics": metric_rows,
        "confusion_matrix": conf_mat.tolist(),
        "task": {k: (v if not isinstance(v, list) else len(v)) for k, v in task.items()},
    }, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

