#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lay the merged VOC set out the way `pytorch-ssd` reads it (README 6.5.4).

WHY THIS EXISTS
---------------
README 6.5 says the ONNX route is deploy-side simpler than the UFF route, and that
switching to it means switching the trainer from TF Object Detection API to
PyTorch `pytorch-ssd`.  What that section also has to admit is that the trainer
wants a different *directory layout*, and the layout is not negotiable:

    <root>/Annotations/<id>.xml
    <root>/JPEGImages/<id>.jpg
    <root>/ImageSets/Main/trainval.txt      (one id per line, NO extension)
    <root>/ImageSets/Main/test.txt
    <root>/labels.txt                       (class names, NO background)

while `build_dataset.py` writes

    <out>/images/*.jpg
    <out>/annotations/*.xml
    <out>/label_map.pbtxt

Same data, same XML schema, same filenames - only the directory names and the
two ImageSets files differ.  That is all this script produces.

THE SPLIT IS THE SAME ONE, BY CONSTRUCTION
------------------------------------------
`pytorch-ssd` trains on `trainval.txt` and validates on `test.txt`; the TF route
trains on `train.record` and validates on `val.record`.  If the two disagreed,
every accuracy number would be incomparable and a switch of trainer would look
like a regression that nobody could explain.  So this script does NOT re-roll a
split: it calls `voc_split.pick_split()` directly, rather than re-deriving it.
uses, with the same `--seed`.  A given image is in val here iff it is in val
there, forever, as long as both are run with the same `--val-fraction`.

THE FOUR THINGS pytorch-ssd DOES SILENTLY
-----------------------------------------
`vision/datasets/voc_dataset.py` drops things without failing, and every one of
them looks like "the model is just not very good".  All four are checked here,
because the whole point of this script is that the alternative route should not
also be the route where bugs go to hide:

1. `_read_image_ids()` skips any id in the ImageSets file that has no XML or no
   image, printing a warning per image.  Thousands of lines, then a smaller
   dataset than you expected.
2. `_get_annotation()` skips any `<object>` whose `<name>` is not in
   `class_dict`, printing one line per box.  This is the dangerous one: a class
   renamed in `labels.py` but not in the XMLs deletes every box of that class
   from the training set, and the loss curve still goes down.
3. `keep_difficult` defaults to False, so every `<difficult>1</difficult>` box is
   dropped.  The TF route keeps them.  This is a real asymmetry between the two
   routes and it is reported rather than fixed.
4. Coordinates get `-1`.  Pascal VOC is 1-based, so that is right for VOC files;
   ours are 0-based pixel bounds written by `gen_synth_dataset.py` /
   `prepare_real_dataset.py`, so every box would arrive one pixel up and one
   pixel wider.  At 445 px/m that is 2.2 mm of systematic bias - small enough to
   ignore per box, large enough to make the two routes' numbers incomparable, so
   `--shift-compensation` (default ON) writes `+1` into the converted XMLs and
   the reader takes it back off.

Usage
-----
    python3 tools/ssd_training/prepare_pytorch_voc.py --out data/voc_task
    python3 tools/ssd_training/prepare_pytorch_voc.py --out data/voc_task --dry-run

Needs nothing but the standard library and this directory's own modules.

Exit status 1 if any class would be missing from the training split, or if an id
in the split has no XML / no image, or if an XML names a class that is not in
`labels.py`.  Those are the failures that would otherwise only show up as a
quietly worse model.
"""

import argparse
import hashlib
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import labels as L                    # noqa: E402  (path set up above)
import voc_split as V                  # noqa: E402  (the split, single source of truth)

IMAGE_EXT = ".jpg"
MANIFEST = "prepare_manifest.json"


def read_manifest(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (IOError, OSError, ValueError):
        return None


def write_manifest(path, fingerprint):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(fingerprint, handle, sort_keys=True, indent=2)
        handle.write("\n")


def load_boxes(ann_path):
    """Return (class names, n_difficult) for one XML.  Read-only, tolerant."""
    root = ET.parse(ann_path).getroot()
    names = []
    difficult = 0
    for obj in root.findall("object"):
        node = obj.find("name")
        if node is None or not (node.text or "").strip():
            continue
        names.append(node.text.strip())
        flag = obj.find("difficult")
        if flag is not None and (flag.text or "").strip() not in ("", "0"):
            difficult += 1
    return names, difficult


def link_or_copy(src, dst, mode):
    """Put src at dst.  Images are large and the XMLs are not, so the default is
    a hard link (no admin, no extra bytes, same filesystem) with a copy fallback
    for when the two paths are on different volumes."""
    if mode == "symlink":
        try:
            os.symlink(os.path.abspath(src), dst)
            return "symlink"
        except (OSError, AttributeError, NotImplementedError):
            mode = "copy"          # Windows without developer mode
    if mode == "hardlink":
        try:
            os.link(src, dst)
            return "hardlink"
        except OSError:
            mode = "copy"
    shutil.copyfile(src, dst)
    return "copy"


def rewrite_annotation(src, dst, compensate):
    """Copy one XML, optionally adding 1 to every bndbox coordinate.

    See the module docstring, item 4.  Returns True if the file was rewritten.
    """
    if not compensate:
        shutil.copyfile(src, dst)
        return False
    tree = ET.parse(src)
    changed = False
    for box in tree.getroot().iter("bndbox"):
        for tag in ("xmin", "ymin", "xmax", "ymax"):
            node = box.find(tag)
            if node is None or node.text is None:
                continue
            try:
                node.text = str(int(round(float(node.text))) + 1)
                changed = True
            except ValueError:
                pass
    tree.write(dst, encoding="UTF-8", xml_declaration=False)
    return changed


def main():
    parser = argparse.ArgumentParser(
        description="Lay the merged VOC set out for pytorch-ssd (README 6.5.4).")
    parser.add_argument("--src", default="data/task_ds",
                        help="build_dataset.py output, with images/ and "
                             "annotations/ inside (default: data/task_ds)")
    parser.add_argument("--out", default="data/voc_task",
                        help="destination root; created if absent (default: data/voc_task)")
    parser.add_argument("--val-fraction", type=float, default=0.12,
                        help="MUST match what voc_split.py was run with, or "
                             "the two routes will not be comparable (default: 0.12)")
    parser.add_argument("--seed", type=int, default=20261006,
                        help="MUST match voc_split.py (default: 20261006)")
    parser.add_argument("--shift-compensation", dest="compensate",
                        action="store_true", default=True,
                        help="write bndbox +1 so pytorch-ssd's unconditional -1 "
                             "lands back on our 0-based coordinates (default: on)")
    parser.add_argument("--no-shift-compensation", dest="compensate",
                        action="store_false",
                        help="leave the coordinates alone; every box then arrives "
                             "1 px up and 1 px wider than the TF route sees it")
    parser.add_argument("--link", choices=("hardlink", "copy", "symlink"),
                        default="hardlink",
                        help="how to place the images (default: hardlink, which "
                             "costs no disk and needs no privileges; falls back "
                             "to copy across volumes)")
    parser.add_argument("--model-dir", default="data/models/task",
                        help="where to write the 11-line labels.txt that detectNet "
                             "and the sim node both need (default: data/models/task). "
                             "Its content is fixed before training: BACKGROUND + the "
                             "class list, which is exactly what train_ssd.py writes "
                             "next to the .onnx")
    parser.add_argument("--dry-run", action="store_true",
                        help="report everything, write nothing")
    parser.add_argument("--rebuild", action="store_true",
                        help="把 <out> 里本脚本产出的东西删掉重铺。当源数据集"
                             "换过（重新渲染过、类别改过）而 <out> 还在时**必须**"
                             "给，否则脚本会拒绝写并告诉你原因 —— 详见写入段"
                             "那段注释里记的真实事故")
    args = parser.parse_args()

    images_dir = os.path.join(args.src, "images")
    anns_dir = os.path.join(args.src, "annotations")
    for path in (images_dir, anns_dir):
        if not os.path.isdir(path):
            raise SystemExit("no such directory: %s\n"
                             "Run tools/ssd_training/build_dataset.py --out %s first."
                             % (path, args.src))

    class_names = list(L.CLASS_NAMES)
    known = set(class_names)
    print("classes (%d, from labels.py - no background line):" % len(class_names))
    for index, name in enumerate(class_names, start=1):
        print("  %2d  %s" % (index, name))
    print()

    ann_files = sorted(f for f in os.listdir(anns_dir) if f.lower().endswith(".xml"))

    # ---- walk the split exactly the way the TF route does ----------------
    groups = {"train": [], "val": []}
    problems = []
    unknown_names = {}
    per_split = {s: {n: 0 for n in class_names} for s in groups}
    difficult_total = 0
    stems = set()

    for ann in ann_files:
        stem = os.path.splitext(ann)[0]
        if stem in stems:
            problems.append("duplicate id: %s" % stem)
        stems.add(stem)
        image = os.path.join(images_dir, stem + IMAGE_EXT)
        if not os.path.exists(image):
            # pytorch-ssd's _read_image_ids() would skip this silently.
            problems.append("no image for %s (VOCDataset would drop it)" % stem)
            continue
        split = "val" if V.pick_split(stem, args.val_fraction, args.seed) else "train"
        groups[split].append(stem)

        names, difficult = load_boxes(os.path.join(anns_dir, ann))
        difficult_total += difficult
        if not names:
            problems.append("%s has no <object>; VOCDataset would drop it" % stem)
        for name in names:
            if name not in known:
                unknown_names.setdefault(name, []).append(stem)
            else:
                per_split[split][name] += 1

    for name, where in sorted(unknown_names.items()):
        problems.append("class %r is not in labels.py; %d box(es) would be dropped "
                        "silently by VOCDataset (e.g. %s)"
                        % (name, len(where), where[0]))

    # 源数据集的指纹。写入段用它判断"上一轮铺的 <out> 还能不能继续用"。
    split_digest = hashlib.sha256(
        "\n".join(sorted(stems)).encode("utf-8")).hexdigest()[:16]

    # ---- report ----------------------------------------------------------
    print("split by voc_split.pick_split: sha256(%d:<stem>) < %.2f  - the one"
          " place this is decided" % (args.seed, args.val_fraction))
    print("  trainval.txt (train) %5d images" % len(groups["train"]))
    print("  test.txt     (val)   %5d images" % len(groups["val"]))
    print()

    header = "  %-30s %8s %8s" % ("class", "train", "val")
    print(header)
    print("  " + "-" * (len(header) - 2))
    for name in class_names:
        print("  %-30s %8d %8d" % (name, per_split["train"][name], per_split["val"][name]))
    print("  " + "-" * (len(header) - 2))
    print("  %-30s %8d %8d" % ("TOTAL boxes",
                               sum(per_split["train"].values()),
                               sum(per_split["val"].values())))
    print()

    if difficult_total:
        print("  <difficult> boxes: %d" % difficult_total)
        print("    VOCDataset defaults to keep_difficult=False and drops these;")
        print("    the TF route keeps them.  Real asymmetry between the routes.")
    else:
        print("  <difficult> boxes: 0 (nothing dropped on that account)")
    print()

    # A class with no training boxes is the one failure that cannot be tuned
    # around later, so it is fatal rather than a warning.
    for name in class_names:
        if per_split["train"][name] == 0:
            problems.append("class %r has 0 boxes in the training split" % name)
    for name in class_names:
        if per_split["val"][name] == 0:
            print("  WARNING: %r has 0 boxes in val - it cannot be evaluated"
                  % name)
    print()

    if problems:
        print("=" * 72)
        print("FAIL - %d problem(s); do not train on this layout:" % len(problems))
        for item in problems[:40]:
            print("   - %s" % item)
        if len(problems) > 40:
            print("   ... and %d more" % (len(problems) - 40))
        print("=" * 72)
        return 1

    print("layout is consistent.  What pytorch-ssd will read:")
    print("  <root>/Annotations/<id>.xml      %d files" % len(ann_files))
    print("  <root>/JPEGImages/<id>%s        %d files" % (IMAGE_EXT, len(ann_files)))
    print("  <root>/ImageSets/Main/trainval.txt   %d ids" % len(groups["train"]))
    print("  <root>/ImageSets/Main/test.txt       %d ids" % len(groups["val"]))
    print("  <root>/labels.txt                    %d lines, line 0 is a REAL class"
          % len(class_names))
    print("      VOCDataset prepends BACKGROUND itself, so class ids come out")
    print("      as 1..%d and train_ssd.py writes an %d-line labels.txt next to the"
          % (len(class_names), len(class_names) + 1))
    print("      model.  Those two files are supposed to differ by that one line.")
    print()

    if args.dry_run:
        print("--dry-run: nothing written.")
        return 0

    # ---- write -----------------------------------------------------------
    # 防呆：源数据集换了而上一轮铺的 <out> 还在，就**拒绝**，不静默复用。
    #
    # 这是从真机上真踩出来的：2026-10-07 把合成集从"含人物卡"改成"只画危险品"
    # 之后重跑了 build_dataset.py，再跑本脚本时它跳过了所有已存在的文件 ——
    # 图片因为是硬链接而**自动**变成了新的（源图是原地覆写，inode 不变），
    # XML 与划分文件却还是旧的。于是 <out> 里是"新图 + 旧标注"：1931 个 XML
    # 仍然带 person_injured，而新源数据集里那个人物卡一张都没有。
    # 训练会照跑，loss 也会降，只是学到一批指向不存在的东西的框。
    fingerprint = {
        "images": len(ann_files),
        "annotations": len(ann_files),
        "trainval": len(groups["train"]),
        "test": len(groups["val"]),
        "split_digest": split_digest,
        "boxes": sum(sum(counts.values()) for counts in per_split.values()),
    }
    manifest_path = os.path.join(args.out, "prepare_manifest.json")
    previous = read_manifest(manifest_path)
    if previous is not None and previous != fingerprint and not args.rebuild:
        print("=" * 72)
        print("拒绝写入：%s 是上一轮铺的，而源数据集已经变了。" % args.out)
        print("=" * 72)
        print("  目录里记的: %s" % json.dumps(previous, sort_keys=True))
        print("  源数据集:   %s" % json.dumps(fingerprint, sort_keys=True))
        print()
        print("这不能默默过去。图片是硬链接，源图原地覆写时它们会自动跟着变；")
        print("而 XML 与划分文件不会。于是一个**新的数据集**会配上**旧的标注**，")
        print("训练照跑、loss 照降、模型学到错的东西。")
        print()
        print("确认要把 %s 整个重铺，就加 --rebuild：" % args.out)
        print("  python3 %s --src %s --out %s --rebuild"
              % (os.path.basename(__file__), args.src, args.out))
        return 1

    jpg_dir = os.path.join(args.out, "JPEGImages")
    xml_dir = os.path.join(args.out, "Annotations")
    sets_dir = os.path.join(args.out, "ImageSets", "Main")

    if args.rebuild and os.path.isdir(args.out):
        # 只删本脚本产出的那三样，别碰用户放在同一个目录里的别的东西。
        for path in (jpg_dir, xml_dir, sets_dir):
            if os.path.isdir(path):
                shutil.rmtree(path)
        for stray in ("labels.txt", "prepare_manifest.json"):
            stray_path = os.path.join(args.out, stray)
            if os.path.isfile(stray_path):
                os.remove(stray_path)

    for path in (jpg_dir, xml_dir, sets_dir):
        os.makedirs(path, exist_ok=True)

    # 图片：已存在且确实指向同一个文件（硬链接的 samefile 为真）就跳过，
    # 否则重新放。**不**因为"文件在"就假定它对 —— 那正是上面那个坑。
    how = {"hardlink": 0, "copy": 0, "symlink": 0, "reused": 0, "replaced": 0}
    for stem in groups["train"] + groups["val"]:
        src_img = os.path.join(images_dir, stem + IMAGE_EXT)
        dst_img = os.path.join(jpg_dir, stem + IMAGE_EXT)
        if os.path.exists(dst_img):
            try:
                if os.path.samefile(src_img, dst_img):
                    how["reused"] += 1
                    continue
            except OSError:
                pass
            os.remove(dst_img)          # 陈旧（源图换了 inode / 曾是复制品）
            how["replaced"] += 1
        how[link_or_copy(src_img, dst_img, args.link)] += 1

    # XML 与划分文件：**无条件重写**。它们很小，而"小的文件被跳���"正是会
    # 悄悄错很久的那种。条件是"与源一致"，不是"已经存在"。
    for stem in groups["train"] + groups["val"]:
        rewrite_annotation(os.path.join(anns_dir, stem + ".xml"),
                           os.path.join(xml_dir, stem + ".xml"),
                           args.compensate)

    for name, stems in (("trainval.txt", groups["train"]), ("test.txt", groups["val"])):
        with open(os.path.join(sets_dir, name), "w", encoding="utf-8") as handle:
            handle.write("".join(stem + "\n" for stem in stems))

    with open(os.path.join(args.out, "labels.txt"), "w", encoding="utf-8") as handle:
        handle.write("".join(name + "\n" for name in class_names))

    write_manifest(manifest_path, fingerprint)

    # 模型目录那份 labels.txt：**训练前就是确定的**（BACKGROUND + 类别表），所以
    # 在这里写出来，而不是等 train_ssd.py 去写。理由有两个：
    #   * 仿真侧与真机两个 launch 的默认 label_file 都指向它，文件在 ⇒ 门禁 13
    #     的"默认路径存在"这一项在训练之前就是过的（缺的只是 .onnx）；
    #   * train_pytorch_ssd.py 训练后会逐行比对它与 train_ssd.py 的输出，
    #     所以"类 id 顺序错了"这件事有了一个独立的对照物。
    model_labels = os.path.join(args.model_dir, "labels.txt")
    if not args.dry_run:
        os.makedirs(args.model_dir, exist_ok=True)
        with open(model_labels, "w", encoding="utf-8") as handle:
            handle.write("BACKGROUND\n")
            handle.write("".join(name + "\n" for name in class_names))

    print("written to %s" % args.out)
    print("  %d images (%s), %d annotations, %d + %d split lines, labels.txt (%d lines)"
          % (len(ann_files),
             ", ".join("%d %s" % (n, k) for k, n in how.items() if n),
             len(ann_files), len(groups["train"]), len(groups["val"]),
             len(class_names)))
    print("  %s  (%d lines, line 0 = BACKGROUND)  <- 给 detectNet 与仿真节点"
          % (model_labels, len(class_names) + 1))
    if args.compensate:
        print("  bndbox written +1 (pytorch-ssd subtracts 1 on read, so the two")
        print("  routes see identical coordinates)")
    print()
    print("next, on a machine with PyTorch:")
    print("  cd <jetson-inference>/python/training/detection")
    print("  python3 train_ssd.py --dataset-type voc --dataset %s \\" % args.out)
    print("      --net mb1-ssd --resolution 300 --batch-size 16 \\")
    print("      --num-epochs 100 --checkpoint-folder models/task")
    print("  python3 onnx_export.py --net mb1-ssd --model-dir models/task")
    print("then, before copying the .onnx to the aircraft:")
    print("  python3 tools/ssd_training/check_onnx_contract.py \\")
    print("      --onnx models/task/ssd-mobilenet.onnx \\")
    print("      --labels models/task/labels.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())