"""Stage 3: build the INT8 calibration set for RKNN (never from the frozen test split).

YOLOv5 pre-processes images with letterbox to 640x640 (grey 114 padding).  The NPU
model therefore sees letterboxed 640x640 RGB input, so the calibration images are
written in exactly that form.  Only images/train/ is used; val/test never calibrate.

Usage: python 03_make_calib.py [--per-class 8]
Output: work/dataset/calib/{*.jpg, dataset.txt}
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
DS = ROOT / "work" / "dataset" / "sixclass"
OUT = ROOT / "work" / "dataset" / "calib"
MANIFEST = DS / "split_manifest.json"


def letterbox(im: Image.Image, new: int = 640, colour=(114, 114, 114)) -> Image.Image:
    w, h = im.size
    r = min(new / w, new / h)
    nw, nh = round(w * r), round(h * r)
    im = im.resize((nw, nh), Image.Resampling.BILINEAR)
    canvas = Image.new("RGB", (new, new), colour)
    canvas.paste(im, ((new - nw) // 2, (new - nh) // 2))
    return canvas


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-class", type=int, default=8)
    args = ap.parse_args()

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    train = [r for r in manifest["files"] if r["split"] == "train"]
    by_class: dict[int, list[dict]] = defaultdict(list)
    for row in train:
        by_class[row["class_id"]].append(row)

    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*.jpg"):
        old.unlink()

    rows, written = [], 0
    for cid in sorted(by_class):
        # spread the sample across the capture order instead of taking the first N
        recs = sorted(by_class[cid], key=lambda r: r["stem"])
        step = max(1, len(recs) // args.per_class)
        picked = recs[::step][: args.per_class]
        for rec in picked:
            src = DS / rec["image"]
            with Image.open(src) as im:
                lb = letterbox(im.convert("RGB"))
            name = f"c{cid}_{rec['stem']}.jpg"
            lb.save(OUT / name, quality=95)
            rows.append({"file": name, "class_id": cid, "source": rec["image"], "source_sha256": rec["image_sha256"]})
            written += 1

    (OUT / "dataset.txt").write_text(
        "\n".join(str((OUT / r["file"]).as_posix()) for r in rows) + "\n", encoding="utf-8"
    )
    (OUT / "calib_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generated_by": "work/scripts/03_make_calib.py",
                "note": "letterboxed 640x640 RGB; sourced only from images/train",
                "count": written,
                "per_class": {str(c): sum(1 for r in rows if r["class_id"] == c) for c in sorted(by_class)},
                "files": rows,
            },
            ensure_ascii=False,
            indent=1,
        ),
        encoding="utf-8",
    )
    print(json.dumps({"calib_dir": str(OUT), "count": written,
                      "per_class": {str(c): sum(1 for r in rows if r["class_id"] == c) for c in sorted(by_class)}},
                     ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
