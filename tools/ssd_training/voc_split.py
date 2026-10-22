#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The train/val split, and the report that describes it.  No TensorFlow.

WHY THIS FILE EXISTS
--------------------
It used to be the top half of `voc_to_tfrecord.py`.  That script wrote
`train.record` / `val.record` for the TF Object Detection API, and when the
project moved to PyTorch `pytorch-ssd` (README §6) the writing half died with
TF 1.15.  The reading half did not: `pytorch-ssd`'s `VOCDataset` gets its
train/val lists from `ImageSets/Main/*.txt`, and `prepare_pytorch_voc.py` fills
those in from `pick_split()` below.

Keeping the split here rather than inlining it into `prepare_pytorch_voc.py` is
the point.  When two routes decide "which images are in val" separately, they
drift the first time a dataset is regenerated, and then every accuracy number
from the new route is incomparable with the old one - a regression nobody can
explain.  There is exactly one `pick_split` and both routes call it.

WHAT IT DECIDES
---------------
    val  iff  sha256("<seed>:<image stem>")[:4] as a big integer / 2**32
              <  --val-fraction

Hashing the name rather than shuffling the list is deliberate: the assignment
does not move when a file is added or removed, which matters because the
synthetic half is re-rendered with a larger --count and the real photographs are
appended to it afterwards.  A shuffle-based split would silently re-roll the
validation set on every rebuild.

Usage
-----
    python3 tools/ssd_training/voc_split.py \
        --images data/task_ds/images \
        --annotations data/task_ds/annotations \
        --label-map data/task_ds/label_map.pbtxt \
        --split-report

Standard library only.  Exit status 1 if the validation split came out empty.
"""

import argparse
import glob
import hashlib
import os
import sys
import xml.etree.ElementTree as ET


def pick_split(stem, val_fraction, seed):
    """Deterministic per-image assignment, stable under re-runs and reordering.

    Hashing the name rather than shuffling the list means the split does not
    move when a file is added or removed, which matters when the synthetic set
    is regenerated with a larger --count and the fine-tuning set is appended.
    """
    digest = hashlib.sha256(("%d:%s" % (seed, stem)).encode("utf-8")).digest()
    return (int.from_bytes(digest[:4], "big") / 2 ** 32) < val_fraction


def load_label_map(path):
    """Minimal reader: the pbtxt body is 'item { id: N name: 'x' }' repeated."""
    mapping = {}
    current_id = None
    for raw in open(path, encoding="utf-8"):
        line = raw.strip()
        if line.startswith("id:"):
            current_id = int(line.split(":", 1)[1].strip())
        elif line.startswith("name:"):
            name = line.split(":", 1)[1].strip().strip("'\"")
            if current_id is not None:
                mapping[name] = current_id
    if not mapping:
        raise SystemExit("no classes read from %s" % path)
    return mapping


def _flag(node, tag):
    """读 <truncated> / <difficult> 这类 0/1 标记；缺了或读不动就当 0。"""
    child = node.find(tag)
    if child is None or child.text is None:
        return 0
    try:
        return 1 if int(float(child.text)) else 0
    except ValueError:
        return 0


def read_boxes(ann_path):
    """-> (width, height, [(name, x1, y1, x2, y2, truncated, difficult)], skipped).

    Clamped to the frame, with degenerate boxes dropped.

    The report and `prepare_pytorch_voc.py` share this on purpose, so both
    describe exactly the boxes that reach training.  An earlier version had the
    report re-read the annotation and count everything it found, which
    quietly over-counted whatever the writer had just thrown away.

    truncated/difficult are carried through rather than invented here: the
    annotator marks a sign clipped by the frame, and that provenance is the only
    record of it.  Two of the real photographs are like that.
    """
    root = ET.parse(ann_path).getroot()
    size = root.find("size")
    width = int(size.find("width").text)
    height = int(size.find("height").text)
    boxes, skipped = [], 0
    for obj in root.findall("object"):
        name = obj.find("name").text.strip()
        box = obj.find("bndbox")
        x1 = max(0.0, min(float(width - 1), float(box.find("xmin").text)))
        y1 = max(0.0, min(float(height - 1), float(box.find("ymin").text)))
        x2 = max(0.0, min(float(width), float(box.find("xmax").text)))
        y2 = max(0.0, min(float(height), float(box.find("ymax").text)))
        if x2 - x1 < 1.0 or y2 - y1 < 1.0:
            skipped += 1
            continue
        boxes.append((name, x1, y1, x2, y2,
                      _flag(obj, "truncated"), _flag(obj, "difficult")))
    return width, height, boxes, skipped


def collect(anns, images_dir, val_fraction, seed):
    """Group the annotations by split.  No image decoding."""
    groups = {"train": [], "val": []}
    missing = []
    for ann in anns:
        stem = os.path.splitext(os.path.basename(ann))[0]
        if not os.path.exists(os.path.join(images_dir, stem + ".jpg")):
            missing.append(os.path.basename(ann))
            continue
        split = "val" if pick_split(stem, val_fraction, seed) else "train"
        groups[split].append(ann)
    return groups, missing


def split_report(groups, label_map, val_fraction, seed):
    """Print the exact counts, per class and per split."""
    per = {"train": {n: 0 for n in label_map}, "val": {n: 0 for n in label_map}}
    degenerate = 0
    empty = 0
    trunc_names = []
    for split, anns in groups.items():
        for ann in anns:
            _w, _h, boxes, skipped = read_boxes(ann)
            degenerate += skipped
            if not boxes:
                empty += 1
                continue
            for name, _x1, _y1, _x2, _y2, tr, _df in boxes:
                per[split][name] += 1
                if tr:
                    trunc_names.append("%s (%s)" % (
                        os.path.basename(ann), name))

    n_train = len(groups["train"]) - empty
    n_val = len(groups["val"])
    print("split by sha256(%d:<stem>) < %.2f   (--seed, --val-fraction)"
          % (seed, val_fraction))
    print()
    print("%-30s %8s %8s" % ("class", "train", "val"))
    print("%-30s %8s %8s" % ("-" * 30, "-" * 8, "-" * 8))
    for name in sorted(label_map, key=lambda n: label_map[n]):
        flag = "   <-- absent from val" if per["val"][name] == 0 else ""
        print("%-30s %8d %8d%s"
              % (name, per["train"][name], per["val"][name], flag))
    print()
    print("images   : %d train + %d val = %d"
          % (n_train, n_val, n_train + n_val))
    if empty:
        print("           %d image(s) had no usable box and will be dropped"
              % empty)
    if degenerate:
        print("           %d degenerate box(es) will be dropped" % degenerate)
    # 这一行是刻意留下的：truncated 曾经在写 record 时被写死成 0，
    # 报告里也看不见，于是"有 2 张照片本身被切"这条事实在整条链路里消失。
    print("truncated: %d box(es) carry truncated=1" % len(trunc_names))
    for item in trunc_names:
        print("           %s" % item)
    print()
    print("  val images = %d" % n_val)
    if n_val == 0:
        print("\nFAIL: the validation split is empty at --val-fraction %.2f"
              % val_fraction, file=sys.stderr)
        return 1
    val_classes = [n for n in label_map if per["val"][n] == 0]
    if val_classes:
        print("\nnote: %d class(es) have no val instance (%s)."
              % (len(val_classes), ", ".join(sorted(val_classes))))
        print("      这不影响训练，但它们没法被评估。")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Report the train/val split and the per-class box counts.")
    ap.add_argument("--images", required=True)
    ap.add_argument("--annotations", required=True)
    ap.add_argument("--label-map", required=True)
    ap.add_argument("--val-fraction", type=float, default=0.12)
    ap.add_argument("--seed", type=int, default=20261006)
    args = ap.parse_args()

    if not 0.0 <= args.val_fraction < 1.0:
        raise SystemExit("--val-fraction must be in [0, 1)")

    label_map = load_label_map(args.label_map)
    print("label map: %d classes, ids %d..%d"
          % (len(label_map), min(label_map.values()), max(label_map.values())))

    anns = sorted(glob.glob(os.path.join(args.annotations, "*.xml")))
    if not anns:
        raise SystemExit("no .xml under %s" % args.annotations)

    groups, missing = collect(anns, args.images, args.val_fraction, args.seed)
    if missing:
        print("  %d annotation(s) have no matching image and are excluded: %s"
              % (len(missing), ", ".join(missing[:5])))
    print()
    return split_report(groups, label_map, args.val_fraction, args.seed)


if __name__ == "__main__":
    sys.exit(main())
