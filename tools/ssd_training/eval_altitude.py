#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练高度 vs 巡航高度：同一渲染器、同一模型，只改相机离垫的高度。

训练集的高度带是任务高度 1.2 m（相机 1.05 m）±0.12 m。航线飞行高度
1.5 m ⇒ 相机 1.35 m，卡片看着小 30%。这个脚本把唯一变量换成高度，
量化"飞过任务区时 detector 还能不能认"。

    python eval_altitude.py --alts 1.05 1.35 --frames 40
"""

import argparse
import collections
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2
import numpy as np

import gen_synth_dataset as G
import labels as L

sys.path.insert(0, HERE)
import eval_onnx as E                                        # noqa: E402


def render_at(seed, alt_cam, count, augment_p=1.0):
    """-> [(bgr, [(class_name, xmin, ymin, xmax, ymax)])] 相机在 alt_cam 高度。

    渲染器的随机源是标准库 random.Random（rng.sample/uniform/random），
    不是 numpy Generator——传错类型第一行就炸。
    """
    import random as _random

    rng = _random.Random(seed)
    out = []
    for _ in range(count):
        cards = []
        picks = rng.sample(L.hazard_names(), len(L.HAZARD_Y))
        for name, base_y in zip(picks, L.HAZARD_Y):
            cards.append(G.Card(name,
                                L.HAZARD_X + rng.uniform(-0.05, 0.05),
                                base_y + rng.uniform(-0.04, 0.04),
                                rng.uniform(-0.12, 0.12)))
        cx = sum(c.x for c in cards) / len(cards)
        cy = sum(c.y for c in cards) / len(cards)
        scene_yaw = rng.uniform(0.0, 2.0 * np.pi)
        rc, rs = np.cos(scene_yaw), np.sin(scene_yaw)
        for card in cards:
            dx, dy = card.x - cx, card.y - cy
            card.x = cx + rc * dx - rs * dy
            card.y = cy + rs * dx + rc * dy
            card.yaw += scene_yaw
        cam = G.Camera(
            position=(cx + rng.uniform(-0.22, 0.22),
                      cy + rng.uniform(-0.22, 0.22), alt_cam),
            yaw=rng.uniform(0.0, 2.0 * np.pi),
            pitch=np.radians(rng.uniform(-5.0, 5.0)),
            roll=np.radians(rng.uniform(-5.0, 5.0)),
            hfov=L.CAM_HFOV_RAD, width=L.CAM_WIDTH, height=L.CAM_HEIGHT)
        img, boxes = G.render(cards, cam, rng)
        if rng.random() < augment_p:
            img = G.augment(img, rng)
        out.append((np.asarray(img)[:, :, ::-1].copy(), boxes))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx",
                    default=os.path.join(HERE, "..", "..", "src",
                                         "ssd_detection", "models",
                                         "ssd-mobilenet.onnx"))
    ap.add_argument("--labels",
                    default=os.path.join(HERE, "..", "..", "src",
                                         "ssd_detection", "models",
                                         "labels.txt"))
    ap.add_argument("--alts", type=float, nargs="+",
                    default=[1.05, 1.17, 1.35, 1.65],
                    help="相机离垫高度（m）；1.05=训练带中心，1.35=巡航 1.5 m")
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--threshold", type=float, default=0.5)
    args = ap.parse_args()

    import onnxruntime
    with open(args.labels, "r", encoding="utf-8-sig") as fh:
        labels = [l.strip() for l in fh if l.strip()]
    so = onnxruntime.SessionOptions()
    so.log_severity_level = 3
    sess = onnxruntime.InferenceSession(args.onnx, so,
                                        providers=["CPUExecutionProvider"])

    print("=" * 78)
    print("高度敏感性：同一渲染器同一模型，只改相机高度")
    print("=" * 78)
    print("  %-10s %-8s %-8s %-9s %-9s %-9s %s"
          % ("alt(m)", "GT", "检出", "类对", "IoU", "误报", "漏检类名(前5)"))
    for alt in args.alts:
        frames = render_at(20261017 + int(alt * 100), alt, args.frames)
        gt_n = det_n = cls_ok = fp_n = miss_n = 0
        ious = []
        missed = collections.Counter()
        wrong = collections.Counter()
        for bgr, boxes in frames:
            h, w = bgr.shape[:2]
            gts = [(labels.index(n), x1, y1, x2, y2)
                   for n, x1, y1, x2, y2 in boxes if n in labels]
            blob = E.preprocess(bgr, 300)
            sc, bx = sess.run(["scores", "boxes"], {"input_0": blob})
            sc = np.asarray(sc).reshape(-1, sc.shape[-1])
            bx = np.asarray(bx)
            if bx.ndim == 3:
                bx = bx.reshape(-1, bx.shape[-1])
            preds = E.parse_outputs(sc, bx, len(labels), args.threshold,
                                    0.45, w, h, "xyxy")
            tps, fps = E.match_frame(preds, gts, 0.5)
            gt_n += len(gts)
            det_n += len(tps)
            fp_n += len(fps)
            for pred, gt in tps:
                ious.append(E.iou(pred, gt))
                if pred[0] == gt[0]:
                    cls_ok += 1
                else:
                    wrong[(labels[gt[0]], labels[pred[0]])] += 1
            matched_gt = {id(g) for _, g in tps}
            for g in gts:
                if id(g) not in matched_gt:
                    miss_n += 1
                    missed[labels[g[0]]] += 1
        mean_iou = float(np.mean(ious)) if ious else 0.0
        print("  %-10.2f %-8d %-8d %-9d %-9.3f %-9d %s"
              % (alt, gt_n, det_n, cls_ok, mean_iou, fp_n,
                 ", ".join("%s x%d" % kv for kv in missed.most_common(5)) or "-"))
        if wrong:
            print("             认错: %s" % ", ".join(
                "%s->%s x%d" % (a, b, c) for (a, b), c in wrong.most_common(6)))
    print()
    print("读法：类对/检出 随高度掉 = 尺度外推不足；误报随高度涨 = 小卡被")
    print("      拆成多个低质量框或背景被激活。巡航高度掉得厉害，就按")
    print("      build_dataset.py 的高度带加一档巡航高度样本再重训。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
