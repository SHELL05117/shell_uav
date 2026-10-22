#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用部署的那个 .onnx 在一批 VOC 图上量化"误识别率"。

为什么需要这个脚本：训练 loss 是合成域上的数字，它回答不了"飞机拍到的
真实画面里误报多少"。这个脚本跑的是**部署路径本身**——同一张 onnx、
同一套预处理（300x300 非等比、RGB、(x/255-0.5)/0.5）、同一个阈值和
NMS——所以它的输出就是运行时会发生的事，不是另一个近似。

它报告三件事，分别对应三种完全不同的"误识别"：

  1. per-class precision / recall     哪一类在滥报、哪一类在漏
  2. 混淆矩阵                        哪一类被当成了哪一类（A 认成 B）
  3. 空图误报                          没有 GT 的帧上报了什么（背景误报）

外加一个阈值扫描：同一个模型在 0.3/0.5/0.7 下的 FP/FN，用来判断
"误报多"是不是只需要抬阈值就能解决——如果 FP 集中在低分段，那就是
免费的；如果 FP 均匀分布在高分段，抬阈值只会换来漏报。

box 顺序问题：pytorch-ssd 的 center_form_to_corner_form 写的是
[xmin,ymin,xmax,ymax]，而 detectNet 的 postProcessSSD_ONNX 按
boxes[n*4+0]=Left 解。两边对不上的时候 IoU 恒 0，现象就是"检测不到"。
本脚本对两种顺序都算一遍并按平均 IoU 报告哪一种与 GT 吻合，不猜。

用法：
    python eval_onnx.py --voc data/voc_task --split test
    python eval_onnx.py --voc data/voc_task --split test --thresholds 0.3 0.5 0.7
    python eval_onnx.py --onnx data/models/real4/ssd-mobilenet.onnx \
        --labels data/models/real4/labels.txt --voc data/real_verify --split ""
退出码 0 = 跑完；1 = 输入有问题（缺文件、split 为空）。
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np

OK = "OK  "
BAD = "FAIL"


# ---------------------------------------------------------------------------
# VOC 读取
# ---------------------------------------------------------------------------

def read_split(voc_dir, split):
    """-> [image_id]；split 为空字符串时扫 JPEGImages 下所有图。"""
    if not split:
        jpeg = os.path.join(voc_dir, "JPEGImages")
        if not os.path.isdir(jpeg):
            return []
        return sorted(os.path.splitext(f)[0]
                      for f in os.listdir(jpeg)
                      if f.lower().endswith((".jpg", ".jpeg", ".png")))
    path = os.path.join(voc_dir, "ImageSets", "Main", split + ".txt")
    if not os.path.isfile(path):
        raise IOError("no split file: %s" % path)
    with open(path, "r", encoding="utf-8") as handle:
        return [line.strip() for line in handle if line.strip()]


def find_image(voc_dir, image_id):
    for ext in (".jpg", ".jpeg", ".png"):
        candidate = os.path.join(voc_dir, "JPEGImages", image_id + ext)
        if os.path.isfile(candidate):
            return candidate
    return None


def read_gt(voc_dir, image_id, class_index, resolve=None):
    """-> [(class_id, xmin, ymin, xmax, ymax)]，越界框已裁掉。

    resolve: 可选的名字映射（labelImg 写的中文名 -> 训练类名）。
    tools/ssd_training/labels.py 的 resolve_class 就是干这个的；实拍集
    (TargetDelection1.1) 的 XML 里写的是"识别项-有毒"这类名字，不映射
    的话每帧 GT 都是 0，评估会把完美模型说成全是误报。
    """
    path = os.path.join(voc_dir, "Annotations", image_id + ".xml")
    if not os.path.isfile(path):
        return []
    root = ET.parse(path).getroot()
    size = root.find("size")
    width = float(size.find("width").text)
    height = float(size.find("height").text)
    boxes = []
    for obj in root.findall("object"):
        name = obj.find("name").text.strip()
        if resolve is not None:
            name = resolve(name) or name
        if name not in class_index:
            continue
        bnd = obj.find("bndbox")
        xmin = max(0.0, float(bnd.find("xmin").text))
        ymin = max(0.0, float(bnd.find("ymin").text))
        xmax = min(width, float(bnd.find("xmax").text))
        ymax = min(height, float(bnd.find("ymax").text))
        if xmax <= xmin or ymax <= ymin:
            continue
        boxes.append((class_index[name], xmin, ymin, xmax, ymax))
    return boxes


# ---------------------------------------------------------------------------
# 与运行时一致的预处理 / 后处理
# ---------------------------------------------------------------------------

def preprocess(bgr, resolution):
    """300x300 非等比 + RGB + (x/255-0.5)/0.5。与 detectNet MODEL_ONNX 同款。"""
    resized = cv2.resize(bgr, (resolution, resolution),
                         interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32)
    norm = (rgb / 255.0 - 0.5) / 0.5
    return np.transpose(norm, (2, 0, 1))[np.newaxis, :]


def nms(boxes, scores, iou_thresh):
    """boxes: [N,4] xyxy；-> 保留的下标。"""
    if boxes is None or len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = np.argsort(scores)[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        ious = inter / np.maximum(union, 1e-12)
        order = rest[ious <= iou_thresh]
    return keep


def parse_outputs(scores, boxes, num_classes, threshold, nms_iou,
                  frame_w, frame_h, order):
    """-> [(class_id, score, xmin, ymin, xmax, ymax)] 原图坐标。

    class_id 与 labels.txt 的行号一致（0=BACKGROUND）。scores 的列数
    num_classes 含背景列，所以列 1..num_classes-1 才是可报告类——
    少切一列就会静悄悄丢掉最后一个类。

    order="xyxy": boxes 行 = [xmin,ymin,xmax,ymax]（pytorch-ssd 的写法）
    order="yxyx": boxes 行 = [ymin,xmin,ymax,xmax]（笔记里 detectNet 实测）
    """
    cls_scores = scores[:, 1:num_classes]
    best = np.argmax(cls_scores, axis=1) + 1
    best_scores = cls_scores[np.arange(len(cls_scores)), best - 1]

    sel = np.where(best_scores >= threshold)[0]
    if sel.size == 0:
        return []
    cand_boxes, cand_scores, cand_cls = boxes[sel], best_scores[sel], best[sel]

    keep = nms(cand_boxes, cand_scores, nms_iou)
    out = []
    for i in keep:
        row = cand_boxes[i]
        if order == "xyxy":
            a, b, c, d = row[0], row[1], row[2], row[3]
        else:
            b, a, d, c = row[0], row[1], row[2], row[3]
        out.append((int(cand_cls[i]), float(cand_scores[i]),
                    float(np.clip(a, 0.0, 1.0)) * frame_w,
                    float(np.clip(b, 0.0, 1.0)) * frame_h,
                    float(np.clip(c, 0.0, 1.0)) * frame_w,
                    float(np.clip(d, 0.0, 1.0)) * frame_h))
    return out


def iou(pred, gt):
    """pred = (cls, score, xmin, ymin, xmax, ymax)；gt = (cls, xmin, ymin, xmax, ymax)。

    两者的坐标下标差一位（pred 多一个 score），写错过一次：拿 a[1] 当 xmin
    实际拿到 score，所有 IoU 都是垃圾，现象是"模型明明很好、评估说全错"。
    """
    ax1, ay1, ax2, ay2 = pred[2], pred[3], pred[4], pred[5]
    bx1, by1, bx2, by2 = gt[1], gt[2], gt[3], gt[4]
    xx1, yy1 = max(ax1, bx1), max(ay1, by1)
    xx2, yy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, xx2 - xx1) * max(0.0, yy2 - yy1)
    if inter <= 0.0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return inter / (area_a + area_b - inter)


# ---------------------------------------------------------------------------
# 评估一帧
# ---------------------------------------------------------------------------

def match_frame(preds, gts, iou_thresh):
    """贪心按 score 匹配。-> (tp 列表, fp 列表)

    TP 要求**同类且** IoU 达标：一个高 IoU 但类别的框不是命中，是误报，
    它同时还对应一个漏检。不比对类别会把最严重的故障（A 全被认成 B）
    统计成完美命中。
    """
    gts = list(gts)
    matched = [False] * len(gts)
    tps, fps = [], []
    for pred in sorted(preds, key=lambda p: -p[1]):
        best_j, best_i = -1, iou_thresh
        for j, gt in enumerate(gts):
            if matched[j] or gt[0] != pred[0]:
                continue
            v = iou(pred, gt)
            if v > best_i:
                best_i, best_j = v, j
        if best_j >= 0:
            matched[best_j] = True
            tps.append((pred, gts[best_j]))
        else:
            fps.append(pred)
    return tps, fps


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def evaluate(session, input_name, output_names, labels, ids, voc_dir,
             resolution, thresholds, nms_iou, iou_thresh, order,
             image_cache):
    """-> {threshold: 统计 dict}。

    labels 是 labels.txt 全文（含第 0 行 BACKGROUND），下标即 class id；
    per_cls 与 confusion 数组按 class id 索引，0 位空置。
    """
    num_classes = len(labels)
    class_index = {name: i for i, name in enumerate(labels)}

    # 标注名 -> 训练类名。tools/ssd_training/labels.py 有 CLASS_ALIASES；
    # 拿不到就原样用（VOC 原生英文标注不需要映射）。
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import labels as _L
        resolve = _L.resolve_class
    except Exception:                                     # noqa: BLE001
        resolve = None

    results = {}
    for threshold in thresholds:
        stats = {"tp": 0, "fp": 0, "fn": 0,
                 "per_cls_tp": [0] * num_classes,
                 "per_cls_fp": [0] * num_classes,
                 "per_cls_fn": [0] * num_classes,
                 "confusion": [[0] * num_classes for _ in range(num_classes)],
                 "fp_empty_frames": 0, "empty_frames": 0,
                 "frames": 0, "iou_sum": 0.0, "iou_n": 0}
        for image_id in ids:
            bgr = image_cache.get(image_id)
            if bgr is None:
                path = find_image(voc_dir, image_id)
                if path is None:
                    continue
                bgr = cv2.imread(path)
                if bgr is None:
                    continue
                image_cache[image_id] = bgr
            h, w = bgr.shape[:2]
            gts = read_gt(voc_dir, image_id, class_index, resolve)
            if not gts:
                stats["empty_frames"] += 1
            stats["frames"] += 1

            blob = preprocess(bgr, resolution)
            raw_scores, raw_boxes = session.run(output_names,
                                                {input_name: blob})
            scores = np.asarray(raw_scores)
            boxes = np.asarray(raw_boxes)
            if scores.ndim == 3:
                scores = scores.reshape(-1, scores.shape[-1])
            if boxes.ndim == 3:
                boxes = boxes.reshape(-1, boxes.shape[-1])
            preds = parse_outputs(scores, boxes, num_classes, threshold,
                                  nms_iou, w, h, order)
            tps, fps = match_frame(preds, gts, iou_thresh)
            stats["tp"] += len(tps)
            stats["fp"] += len(fps)
            stats["fn"] += len(gts) - len(tps)
            for pred, gt in tps:
                stats["per_cls_tp"][gt[0]] += 1
                stats["iou_sum"] += iou(pred, gt)
                stats["iou_n"] += 1
            for pred in fps:
                stats["per_cls_fp"][pred[0]] += 1
                # 这一帧里重叠最大的 GT 是哪一类——FP 是"无中生有"还是"认错了邻居"
                if gts:
                    nearest = max(gts, key=lambda g: iou(pred, g))
                    stats["confusion"][nearest[0]][pred[0]] += 1
                else:
                    stats["fp_empty_frames"] += 1
        results[threshold] = stats
    return results


def main():
    parser = argparse.ArgumentParser(
        description="量化部署 onnx 在 VOC 集上的误识别（FP / 混淆 / 空图误报）。")
    parser.add_argument("--onnx",
                        default="src/ssd_detection/models/ssd-mobilenet.onnx")
    parser.add_argument("--labels",
                        default="src/ssd_detection/models/labels.txt")
    parser.add_argument("--voc", default="data/voc_task",
                        help="VOC 风格目录（JPEGImages/ + Annotations/）")
    parser.add_argument("--split", default="test",
                        help="ImageSets/Main/<split>.txt；空串 = 全量图片")
    parser.add_argument("--input-name", default="input_0")
    parser.add_argument("--cvg-name", default="scores")
    parser.add_argument("--bbox-name", default="boxes")
    parser.add_argument("--resolution", type=int, default=300)
    parser.add_argument("--thresholds", type=float, nargs="+",
                        default=[0.3, 0.5, 0.7])
    parser.add_argument("--nms-iou", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.5,
                        help="TP 判定 IoU（detectNet 用 argmax+阈值，这里对齐常用 0.5）")
    parser.add_argument("--box-order", choices=("xyxy", "yxyx", "auto"),
                        default="auto",
                        help="boxes 行的坐标顺序；auto = 两种都跑，按 TP 数选")
    parser.add_argument("--limit", type=int, default=0,
                        help="只跑前 N 张（调试用）")
    args = parser.parse_args()

    import onnxruntime

    for path in (args.onnx, args.labels):
        if not os.path.isfile(path):
            print("[%s] missing %s" % (BAD, path))
            return 1
    with open(args.labels, "r", encoding="utf-8-sig") as handle:
        labels = [line.strip() for line in handle if line.strip()]
    if not labels:
        print("[%s] %s 是空文件" % (BAD, args.labels))
        return 1
    reportable = labels[1:] if labels[0].upper() == "BACKGROUND" else labels
    print("=" * 74)
    print("ONNX 部署路径误识别评估")
    print("=" * 74)
    print("  onnx        %s (%.1f MB)"
          % (args.onnx, os.path.getsize(args.onnx) / 1048576.0))
    print("  labels      %d 行（第 0 行 %r）；可报告 %d 类: %s"
          % (len(labels), labels[0], len(reportable), ", ".join(reportable)))
    print("  voc         %s  split=%s" % (args.voc, args.split or "<all>"))

    try:
        ids = read_split(args.voc, args.split)
    except IOError as exc:
        print("[%s] %s" % (BAD, exc))
        return 1
    if args.limit:
        ids = ids[:args.limit]
    if not ids:
        print("[%s] split 为空" % BAD)
        return 1
    print("  frames      %d" % len(ids))
    print("  thresholds  %s" % ", ".join("%.2f" % t for t in args.thresholds))
    print()

    so = onnxruntime.SessionOptions()
    so.log_severity_level = 3
    session = onnxruntime.InferenceSession(args.onnx, so,
                                           providers=["CPUExecutionProvider"])
    output_names = [args.cvg_name, args.bbox_name]

    orders = ["xyxy", "yxyx"] if args.box_order == "auto" else [args.box_order]
    image_cache = {}
    all_results = {}
    for order in orders:
        res = evaluate(session, args.input_name, output_names, labels, ids,
                       args.voc, args.resolution, args.thresholds, args.nms_iou,
                       args.iou, order, image_cache)
        all_results[order] = res
        if args.box_order != "auto":
            break

    # auto：按主阈值下的 TP 数选顺序，并说明另一种顺序的表现（IoU 恒 0 的现场）
    if args.box_order == "auto":
        main_thr = args.thresholds[len(args.thresholds) // 2]
        tp_by_order = {o: all_results[o][main_thr]["tp"] for o in orders}
        chosen = max(tp_by_order, key=lambda o: tp_by_order[o])
        print("  box order   %s (TP@%.2f: %s)" % (
            chosen, main_thr,
            ", ".join("%s=%d" % (o, tp_by_order[o]) for o in orders)))
        if tp_by_order["xyxy"] != tp_by_order["yxyx"]:
            worse = "yxyx" if chosen == "xyxy" else "xyxy"
            print("              %s 顺序下 TP=%d —— 若这个数字接近 0，"
                  "运行时的 boxes 解顺序就是它" % (worse, tp_by_order[worse]))
    else:
        chosen = args.box_order
    print()

    results = all_results[chosen]
    for threshold in args.thresholds:
        stats = results[threshold]
        tp, fp, fn = stats["tp"], stats["fp"], stats["fn"]
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        print("-- threshold %.2f %s" % (threshold, "-" * 52))
        print("   frames %d (其中无 GT %d)   GT %d  预测 %d"
              % (stats["frames"], stats["empty_frames"], tp + fn, tp + fp))
        print("   TP %d   FP %d   FN %d    precision %.3f   recall %.3f"
              % (tp, fp, fn, precision, recall))
        if stats["iou_n"]:
            print("   TP 平均 IoU %.3f" % (stats["iou_sum"] / stats["iou_n"]))
        if stats["empty_frames"]:
            print("   空图误报 %d 个（%.1f%% 的空图至少误报一次）"
                  % (stats["fp_empty_frames"],
                     100.0 * stats["fp_empty_frames"] / max(1, stats["empty_frames"])))
        print("   per-class:")
        print("     %-28s %5s %5s %5s %8s %8s"
              % ("class", "TP", "FP", "FN", "prec", "rec"))
        for i, name in enumerate(labels):
            if i == 0:
                continue                 # BACKGROUND，不参与报告
            ctp, cfp, cfn = (stats["per_cls_tp"][i], stats["per_cls_fp"][i],
                             stats["per_cls_fn"][i])
            if ctp + cfp + cfn == 0:
                continue
            cp = ctp / (ctp + cfp) if ctp + cfp else 0.0
            cr = ctp / (ctp + cfn) if ctp + cfn else 0.0
            print("     %-28s %5d %5d %5d %8.3f %8.3f"
                  % (name, ctp, cfp, cfn, cp, cr))
        # 混淆矩阵：只在有内容的行上打
        rows = [(i, r) for i, r in enumerate(stats["confusion"])
                if any(r)]
        if rows:
            print("   误报流向（行=该帧重叠最大的 GT 类，列=被报成的类）:")
            for i, r in rows:
                cols = ", ".join("%s x%d" % (labels[j], r[j])
                                 for j in range(1, len(labels)) if r[j])
                print("     %-28s -> %s" % (labels[i], cols))
        print()

    print("=" * 74)
    print("读法：FP 高 + recall 高 = 滥报（抬阈值或加形状过滤）；")
    print("      FP 高 + recall 低 = 域不匹配（合成训得多、真机长得不一样），")
    print("      抬阈值只会更糟；混淆集中在两类之间 = 那两类需要更多区分数据。")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
