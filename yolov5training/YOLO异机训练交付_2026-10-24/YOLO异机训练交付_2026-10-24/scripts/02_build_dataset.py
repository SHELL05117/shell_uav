"""Stage 2: convert Pascal VOC to YOLO labels, build the split, render review overlays.

Input : work/dataset/raw/{JPEGImages,Annotations} + work/reports/audit_pre_conversion.json
Output: work/dataset/sixclass/{images,labels}/{train,val,test}, data.yaml, split_manifest.json
        work/reports/overlays/*.jpg (box-on-image review sheets)

Coordinate convention (documented, evidence based):
  The XML files carry the labelImg fingerprint (root attribute verified="yes",
  <folder>JPEGImages</folder>, relative <path>, pose/truncated/difficult per object).
  labelImg writes the coordinates it holds, i.e. 0-based pixel positions, and the
  audit found integer coordinates only, xmin/ymin >= 0, xmax/ymax <= W/H. At 4284 px
  width a 1-px convention error shifts a normalised coordinate by < 0.0003, below the
  precision written out here, so no cumulative "-1" correction is applied; the value
  is kept as a continuous 0-based boundary: xc=(x1+x2)/2W, w=(x2-x1)/W.

Split policy:
  Files are contiguous per class (IMG_2682..2711 = 人员, then 30 per class) and the
  images carry no capture timestamps, so every class forms exactly one capture session.
  Whole-session disjointness is impossible without dropping a class, therefore each
  class is split in capture order 18/6/6 (train/val/test). This is recorded as an
  experimental split: it reduces but does not remove the burst-correlation risk.

Usage: python 02_build_dataset.py [--copy-mode copy|hardlink]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
RAW = ROOT / "work" / "dataset" / "raw"
OUT = ROOT / "work" / "dataset" / "sixclass"
REPORTS = ROOT / "work" / "reports"
PKG = ROOT / "YOLO异机训练交接包_2026-10-24"
CLASS_MAP = json.loads((PKG / "config" / "class_map.json").read_text(encoding="utf-8"))
AUDIT = REPORTS / "audit_pre_conversion.json"

SPLIT_FRACTIONS = (18, 6, 6)  # per class, in capture order
SPLITS = ("train", "val", "test")


def yolo_line(class_id: int, box: list[float], width: int, height: int) -> tuple[str, list[float]]:
    x1, y1, x2, y2 = box
    xc = (x1 + x2) / 2.0 / width
    yc = (y1 + y2) / 2.0 / height
    w = (x2 - x1) / width
    h = (y2 - y1) / height
    vals = [min(max(v, 0.0), 1.0) for v in (xc, yc, w, h)]
    return f"{class_id} {vals[0]:.6f} {vals[1]:.6f} {vals[2]:.6f} {vals[3]:.6f}", vals


def draw_overlay(img: Image.Image, boxes, colours, scale_to=900) -> Image.Image:
    im = img.convert("RGB")
    s = scale_to / max(im.size)
    im = im.resize((round(im.width * s), round(im.height * s)), Image.Resampling.LANCZOS)
    d = ImageDraw.Draw(im)
    for box, colour in zip(boxes, colours):
        x1, y1, x2, y2 = (v * s for v in box)
        d.rectangle([x1, y1, x2, y2], outline=colour, width=4)
        d.rectangle([x1, y1 - 18, x1 + 70, y1], fill=colour)
    return im


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--copy-mode", choices=("copy", "hardlink"), default="copy")
    args = ap.parse_args()

    audit = json.loads(AUDIT.read_text(encoding="utf-8"))
    images = audit["images"]
    colours = [(255, 64, 64), (255, 160, 0), (255, 255, 0), (0, 255, 128), (0, 180, 255), (200, 120, 255)]

    # --- split: per class, capture order = ascending filename -----------------
    by_class: dict[int, list[dict]] = defaultdict(list)
    for rec in images:
        by_class[rec["objects"][0]["class_id"]].append(rec)
    for cid in by_class:
        by_class[cid].sort(key=lambda r: r["stem"])

    split_of: dict[str, str] = {}
    for cid, recs in sorted(by_class.items()):
        n_train, n_val, n_test = SPLIT_FRACTIONS
        if len(recs) != n_train + n_val + n_test:
            raise SystemExit(f"class {cid}: expected {n_train+n_val+n_test} images, got {len(recs)}")
        for i, rec in enumerate(recs):
            split_of[rec["stem"]] = "train" if i < n_train else ("val" if i < n_train + n_val else "test")

    # --- write images / labels ------------------------------------------------
    for split in SPLITS:
        (OUT / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT / "labels" / split).mkdir(parents=True, exist_ok=True)

    manifest_rows = []
    label_stats = []
    for rec in sorted(images, key=lambda r: r["stem"]):
        stem = rec["stem"]
        split = split_of[stem]
        src = RAW / rec["image"]
        dst = OUT / "images" / split / src.name
        if dst.exists():
            dst.unlink()
        if args.copy_mode == "hardlink":
            dst.hardlink_to(src)
        else:
            shutil.copy2(src, dst)
        (OUT / "labels" / split).joinpath(stem + ".txt").write_text("", encoding="utf-8")

        lines, vals_all = [], []
        for obj in rec["objects"]:
            line, vals = yolo_line(obj["class_id"], obj["box_xyxy"], rec["width"], rec["height"])
            lines.append(line)
            vals_all.append(vals)
        (OUT / "labels" / split / (stem + ".txt")).write_text("\n".join(lines) + "\n", encoding="utf-8")
        label_stats.append(
            {
                "stem": stem,
                "split": split,
                "class_id": rec["objects"][0]["class_id"],
                "box_px": rec["objects"][0]["box_xyxy"],
                "box_norm_xywh": vals_all[0],
            }
        )
        manifest_rows.append(
            {
                "stem": stem,
                "split": split,
                "class_id": rec["objects"][0]["class_id"],
                "class_name": CLASS_MAP["classes"][rec["objects"][0]["class_id"]]["name"],
                "session": f"class{rec['objects'][0]['class_id']}_block",
                "image": f"images/{split}/{src.name}",
                "label": f"labels/{split}/{stem}.txt",
                "image_sha256": rec["sha256"],
                "xml_sha256": rec.get("xml_sha256"),
                "width": rec["width"],
                "height": rec["height"],
            }
        )

    # --- data.yaml ------------------------------------------------------------
    names = "\n".join(f"  {c['id']}: {c['name']}" for c in CLASS_MAP["classes"])
    data_yaml = (
        "# Generated by work/scripts/02_build_dataset.py from the handoff package.\n"
        f"# dataset root (absolute, this machine): {OUT}\n"
        f"path: {OUT.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "nc: 6\n"
        "names:\n"
        f"{names}\n"
    )
    (OUT / "data.yaml").write_text(data_yaml, encoding="utf-8")

    # --- split manifest -------------------------------------------------------
    per_split_class = Counter()
    for row in manifest_rows:
        per_split_class[(row["split"], row["class_id"])] += 1
    split_summary = {
        split: {
            "images": sum(1 for r in manifest_rows if r["split"] == split),
            "per_class": {CLASS_MAP["classes"][cid]["name"]: per_split_class[(split, cid)] for cid in sorted(by_class)},
            "stems": [r["stem"] for r in manifest_rows if r["split"] == split],
        }
        for split in SPLITS
    }
    manifest = {
        "schema_version": 1,
        "generated_by": "work/scripts/02_build_dataset.py",
        "source_archive_sha256": audit["source_zip_sha256"],
        "class_map": CLASS_MAP["classes"],
        "coordinate_convention": {
            "voc_stored": "labelImg integer pixel coordinates, 0-based boundary",
            "shift_applied": 0,
            "formula": "xc=(x1+x2)/(2W), yc=(y1+y2)/(2H), w=(x2-x1)/W, h=(y2-y1)/H",
            "note": "a 1-px inclusive-exclusive difference would move a normalised value by <0.0003 at 4284 px; not resolvable from the archive and not applied",
        },
        "split_policy": {
            "type": "per-class capture-order split (experimental)",
            "why": "each of the 6 classes is one contiguous capture block without EXIF timestamps, so session-disjoint splitting cannot keep every class in every split",
            "fractions_per_class_train_val_test": list(SPLIT_FRACTIONS),
            "group_independent": False,
        },
        "splits": split_summary,
        "files": manifest_rows,
    }
    (OUT / "split_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")

    # --- review overlays ------------------------------------------------------
    ov = REPORTS / "overlays"
    ov.mkdir(parents=True, exist_ok=True)
    per_class = defaultdict(list)
    for rec in sorted(images, key=lambda r: r["stem"]):
        per_class[rec["objects"][0]["class_id"]].append(rec)
    for cid, recs in sorted(per_class.items()):
        thumbs = []
        for rec in recs:
            with Image.open(RAW / rec["image"]) as im:
                boxes = [o["box_xyxy"] for o in rec["objects"]]
                thumbs.append(draw_overlay(im, boxes, [colours[o["class_id"]] for o in rec["objects"]]))
        cols, tw, th = 6, 300, 400
        rows = (len(thumbs) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * tw, rows * th), (20, 20, 20))
        d = ImageDraw.Draw(sheet)
        for i, t in enumerate(thumbs):
            t2 = t.copy()
            t2.thumbnail((tw, th - 14), Image.Resampling.LANCZOS)
            x = (i % cols) * tw + (tw - t2.width) // 2
            y = (i // cols) * th + 14 + (th - 14 - t2.height) // 2
            sheet.paste(t2, (x, y))
            d.text(((i % cols) * tw + 4, (i // cols) * th + 1), recs[i]["stem"], fill=(230, 230, 230))
        name = CLASS_MAP["classes"][cid]["name"]
        sheet.save(ov / f"{cid}_{name}_sheet.jpg", quality=82)

    print(json.dumps({
        "dataset_root": str(OUT),
        "split_sizes": {s: split_summary[s]["images"] for s in SPLITS},
        "per_split_per_class": {s: split_summary[s]["per_class"] for s in SPLITS},
        "labels_written": len(label_stats),
        "overlay_sheets": sorted(p.name for p in ov.glob("*.jpg")),
    }, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
