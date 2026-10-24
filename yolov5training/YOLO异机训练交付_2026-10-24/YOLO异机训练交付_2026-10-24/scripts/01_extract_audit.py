"""Stage 1: extract the read-only handoff ZIP copy and audit every image / VOC annotation.

Reads (never writes) the original archive inside the handoff package, writes a
derived copy under work/dataset/raw plus a machine-readable audit report.

Checks:
  * per-file SHA256 against evidence/素材只读核查.json
  * full pixel decode of every JPEG (PIL) and EXIF orientation
  * XML <-> JPEG pairing, filename and size agreement
  * exactly one object per image, class name in the fixed class map
  * box geometry (finite, inside bounds, non-degenerate)
  * exact-duplicate detection (sha256) and near-duplicate hashing (dHash)
  * annotation-tool fingerprint for the coordinate convention decision

Usage: python 01_extract_audit.py
"""
from __future__ import annotations

import hashlib
import io
import json
import sys
import zipfile
from collections import Counter, defaultdict
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "YOLO异机训练交接包_2026-10-24"
ZIP_INNER = PKG / "data" / "TargetDelection1.1.zip"
EVIDENCE = PKG / "evidence" / "素材只读核查.json"
CLASS_MAP = PKG / "config" / "class_map.json"
OUT_RAW = ROOT / "work" / "dataset" / "raw"
OUT_REPORTS = ROOT / "work" / "reports"


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def dhash(img: Image.Image, size: int = 16) -> str:
    """Difference hash; Hamming distance is used later for burst grouping."""
    small = img.convert("L").resize((size + 1, size), Image.Resampling.LANCZOS)
    px = list(small.getdata())
    bits = []
    for row in range(size):
        base = row * (size + 1)
        for col in range(size):
            bits.append("1" if px[base + col] > px[base + col + 1] else "0")
    return f"{int(''.join(bits), 2):0{size * size // 4}x}"


def main() -> int:
    OUT_RAW.mkdir(parents=True, exist_ok=True)
    OUT_REPORTS.mkdir(parents=True, exist_ok=True)

    classes = json.loads(CLASS_MAP.read_text(encoding="utf-8"))["classes"]
    voc_names = {c["voc_name"]: c for c in classes}
    evidence = json.loads(EVIDENCE.read_text(encoding="utf-8"))
    ev_by_stem = {row["stem"]: row for row in evidence["images"]}

    zf = zipfile.ZipFile(ZIP_INNER)
    jpgs = sorted(n for n in zf.namelist() if n.lower().endswith((".jpg", ".jpeg")))
    xmls = sorted(n for n in zf.namelist() if n.lower().endswith(".xml"))

    images: dict[str, dict] = {}
    problems: list[str] = []
    decoded = 0

    for name in jpgs:
        stem = Path(name).stem
        data = zf.read(name)
        dest = OUT_RAW / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists() or dest.stat().st_size != len(data):
            dest.write_bytes(data)
        sha = sha256_bytes(data)
        if stem in ev_by_stem and ev_by_stem[stem]["sha256"] != sha:
            problems.append(f"{stem}: sha256 differs from handoff evidence")

        with Image.open(io.BytesIO(data)) as im:
            im.load()  # full pixel decode
            decoded += 1
            width, height = im.size
            exif = im.getexif()
            orientation = exif.get(274, 1)
            fmt, mode = im.format, im.mode
            # cheap scene statistics used for the split/grouping report
            gray = im.convert("L").resize((64, 64), Image.Resampling.BILINEAR)
            px = list(gray.getdata())
            mean = sum(px) / len(px)
            d = dhash(im)
        images[stem] = {
            "stem": stem,
            "image": name,
            "sha256": sha,
            "bytes": len(data),
            "width": width,
            "height": height,
            "format": fmt,
            "mode": mode,
            "exif_orientation": orientation,
            "gray_mean": round(mean, 2),
            "dhash": d,
            "xml": None,
            "objects": [],
        }
        ev = ev_by_stem.get(stem)
        if ev and (ev["width"], ev["height"]) != (width, height):
            problems.append(f"{stem}: size differs from evidence {width}x{height}")

    for name in xmls:
        stem = Path(name).stem
        data = zf.read(name)
        raw_text = data.decode("utf-8", errors="replace")
        dest = OUT_RAW / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists():
            dest.write_bytes(data)
        if stem not in images:
            problems.append(f"{stem}: XML without JPEG")
            continue
        rec = images[stem]
        rec["xml"] = name
        rec["xml_sha256"] = sha256_bytes(data)
        try:
            root = ET.fromstring(data)
        except ET.ParseError as exc:
            problems.append(f"{stem}: XML parse error {exc}")
            continue

        # annotation-tool fingerprint (needed for the coordinate-convention decision)
        rec["tool_fingerprint"] = {
            "annotation_attrs": dict(root.attrib),
            "folder": (root.findtext("folder") or "").strip(),
            "path": (root.findtext("path") or "").strip(),
            "database": (root.findtext("source/database") or "").strip(),
            "has_pose": root.find("object/pose") is not None,
            "has_truncated_difficult": root.find("object/difficult") is not None,
        }

        fn = (root.findtext("filename") or "").strip()
        if fn != Path(rec["image"]).name:
            problems.append(f"{stem}: XML filename '{fn}' != '{Path(rec['image']).name}'")
        w = int(root.findtext("size/width"))
        h = int(root.findtext("size/height"))
        if (w, h) != (rec["width"], rec["height"]):
            problems.append(f"{stem}: XML size {w}x{h} != JPEG {rec['width']}x{rec['height']}")
        objs = root.findall("object")
        if len(objs) != 1:
            problems.append(f"{stem}: {len(objs)} objects (expected 1)")
        for obj in objs:
            cname = (obj.findtext("name") or "").strip()
            if cname not in voc_names:
                problems.append(f"{stem}: unknown class '{cname}'")
                continue
            box = obj.find("bndbox")
            x1, y1, x2, y2 = (float(box.findtext(k)) for k in ("xmin", "ymin", "xmax", "ymax"))
            if not (x2 > x1 and y2 > y1):
                problems.append(f"{stem}: degenerate box {x1,y1,x2,y2}")
            if x1 < 0 or y1 < 0 or x2 > rec["width"] or y2 > rec["height"]:
                problems.append(f"{stem}: box outside image {x1,y1,x2,y2}")
            rec["objects"].append(
                {
                    "voc_class": cname,
                    "class_id": voc_names[cname]["id"],
                    "box_xyxy": [x1, y1, x2, y2],
                    "truncated": (obj.findtext("truncated") or "").strip(),
                    "difficult": (obj.findtext("difficult") or "").strip(),
                    "box_fraction": round(((x2 - x1) * (y2 - y1)) / (rec["width"] * rec["height"]), 6),
                }
            )
        if not rec["objects"]:
            problems.append(f"{stem}: no usable object")
        rec["xml_head"] = raw_text[:400]

    # ---- aggregate checks -------------------------------------------------
    per_class = Counter()
    for rec in images.values():
        for o in rec["objects"]:
            per_class[o["voc_class"]] += 1

    touch = Counter()
    for rec in images.values():
        for o in rec["objects"]:
            x1, y1, x2, y2 = o["box_xyxy"]
            touch["xmin_le_1"] += int(x1 <= 1)
            touch["ymin_le_1"] += int(y1 <= 1)
            touch["xmax_ge_W-1"] += int(x2 >= rec["width"] - 1)
            touch["ymax_ge_H-1"] += int(y2 >= rec["height"] - 1)
            touch["xmin_eq_0"] += int(x1 == 0)
            touch["ymin_eq_0"] += int(y1 == 0)
            touch["all_integer_coords"] += int(float(x1).is_integer() and float(y1).is_integer())
    touch["boxes"] = sum(per_class.values())

    sizes = Counter((r["width"], r["height"]) for r in images.values())
    exif_orient = Counter(r["exif_orientation"] for r in images.values())
    sha_dupes = [k for k, v in Counter(r["sha256"] for r in images.values()).items() if v > 1]

    stems = sorted(images)
    pairs = defaultdict(int)
    for i, a in enumerate(stems):
        for b in stems[i + 1:]:
            if images[a]["width"] != images[b]["width"] or images[a]["height"] != images[b]["height"]:
                continue
            dist = bin(int(images[a]["dhash"], 16) ^ int(images[b]["dhash"], 16)).count("1")
            if dist <= 12:
                pairs[(a, b)] = dist
    near = sorted(pairs.items(), key=lambda kv: kv[1])

    report = {
        "schema_version": 1,
        "generated_at": __import__("datetime").datetime.now().astimezone().isoformat(timespec="seconds"),
        "source_zip": str(ZIP_INNER),
        "source_zip_bytes": ZIP_INNER.stat().st_size,
        "source_zip_sha256": hashlib.sha256(ZIP_INNER.read_bytes()).hexdigest(),
        "counts": {
            "jpeg_entries": len(jpgs),
            "xml_entries": len(xmls),
            "images_audited": len(images),
            "fully_decoded": decoded,
        },
        "per_class_voc": dict(per_class),
        "image_sizes": {f"{w}x{h}": n for (w, h), n in sorted(sizes.items(), key=lambda kv: -kv[1])},
        "exif_orientation": dict(exif_orient),
        "box_touch_stats": dict(touch),
        "exact_duplicate_sha256": sha_dupes,
        "near_duplicate_pairs_dhash_le12": [[a, b, d] for (a, b), d in near],
        "near_duplicate_pair_count": len(near),
        "problems": problems,
        "images": [images[s] for s in stems],
    }
    (OUT_REPORTS / "audit_pre_conversion.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8"
    )

    print(json.dumps({k: report[k] for k in (
        "counts", "per_class_voc", "image_sizes", "exif_orientation", "box_touch_stats",
        "exact_duplicate_sha256", "near_duplicate_pair_count", "problems")}, ensure_ascii=False, indent=1))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
