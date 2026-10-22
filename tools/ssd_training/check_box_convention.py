#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Do the synthetic and hand-labelled halves agree on what a box covers?

The two training sets are made by different processes - one renders, one was
annotated by hand - and if they disagree about the size of the target, the
detector learns to predict the average of two wrong boxes.  The failure is
quiet: the reported centre stays correct, so the drop still lands, and the only
symptom is a mIoU number nobody reads.

There is no shared ground truth to compare, so this uses a feature both halves
contain: the SIGN ITSELF.  A GB 2894 warning sign is a yellow triangle inside a
black border, so

    box_width / yellow_mask_width

is a fixed property of the artwork, whatever the scale, tilt or lighting.  Two
independent statements follow, and both have to hold:

  1. labels.FACE_ARTWORK must equal the artwork actually present in the
     textures.  This is the definition the renderer boxes against.
  2. the ratio measured on rendered frames must equal the ratio measured on the
     photographs.  This is whether the two halves describe the same object.

Statement 2 is checked for the hazard classes only.  The person's most
colourful feature is skin, which is not stable across the lighting in the
photographs, so the person convention is carried by statement 1 alone - which is
the stronger of the two anyway, since it is a measurement of the artwork rather
than of a photograph of it.

Exit status 1 means do not train a merged set.

Usage
-----
    python3 tools/ssd_training/check_box_convention.py
    python3 tools/ssd_training/check_box_convention.py --print-artwork
"""

import argparse
import glob
import os
import random
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np                                             # noqa: E402
from PIL import Image, ImageOps                                # noqa: E402

import labels as L                                             # noqa: E402
import gen_synth_dataset as G                                  # noqa: E402
from render_field import rgb2hsv_cv                            # noqa: E402

# Exactly the hazard window in mission_vision.cpp: H [15,40], S >= 70, V >= 70.
HAZARD_H = (15.0, 40.0)
HAZARD_S = 70.0
HAZARD_V = 70.0

ARTWORK_TOL = 0.020        # fraction of the tile, i.e. 8 mm on a 0.40 m face
RATIO_TOL = 0.12           # relative, on box_width / yellow_width


def yellow_mask(rgb):
    a = np.asarray(rgb, np.float32)
    H, S, V = np.split(rgb2hsv_cv(a), 3, axis=2)
    H, S, V = H[..., 0], S[..., 0], V[..., 0]
    return ((H >= HAZARD_H[0]) & (H <= HAZARD_H[1])
            & (S >= HAZARD_S) & (V >= HAZARD_V))


def artwork_bbox(rgb):
    """bbox of the printed artwork on a tile, as fractions of the tile side.

    Returns (x1, y1, x2, y2).  The tiles carry a 1-3 px decorative outline just
    inside their edge, and it is a closed ring sitting 23 px in, so it touches
    neither the image border nor anything else - it can only be removed by a
    thickness test.  Eroding until the thin ring is gone leaves a seed inside
    the artwork, and a morphological reconstruction from that seed recovers the
    artwork's exact extent.  Skipping this returns 0.910 of the tile in both
    axes: square, 8% too large, and entirely plausible.
    """
    a = np.asarray(rgb, np.int16)
    W, H = a.shape[1], a.shape[0]
    m = a.min(axis=2) < 200
    from scipy import ndimage
    struct = np.ones((3, 3), bool)
    seed = ndimage.binary_erosion(m, struct, iterations=2)
    if not seed.any():
        return None
    lbl, count = ndimage.label(seed)
    sizes = ndimage.sum(np.ones_like(lbl), lbl, index=range(1, count + 1))
    grown = ndimage.binary_propagation(lbl == (int(np.argmax(sizes)) + 1), mask=m)
    ys, xs = np.nonzero(grown)
    if not len(xs):
        return None
    return (xs.min() / float(W), ys.min() / float(H),
            (xs.max() + 1) / float(W), (ys.max() + 1) / float(H))


def component_bbox(mask, box):
    """bbox of the one blob that belongs to `box`, or None.

    Isolation is by connected component, not by a rectangle around the box.  A
    hazard zone holds two cards 0.45 m apart and the box is 0.336 m wide, so a
    generous search window swallows the neighbour's triangle, doubles the
    measured yellow width and turns a correct annotation into a ratio near 1.0.
    That is exactly how this check first reported a false failure.
    """
    if not mask.any():
        return None
    from scipy import ndimage
    lbl, count = ndimage.label(mask)
    if count == 0:
        return None
    x1, y1, x2, y2 = box
    best = None
    for index, sl in enumerate(ndimage.find_objects(lbl), start=1):
        ys, xs = sl
        overlap = (max(0.0, min(xs.stop, x2) - max(xs.start, x1))
                   * max(0.0, min(ys.stop, y2) - max(ys.start, y1)))
        area = int((lbl[sl] == index).sum())
        if overlap < 0.5 * area or area < 0.0002 * mask.size:
            continue
        if best is None or overlap > best[0]:
            best = (overlap, float(xs.start), float(ys.start),
                    float(xs.stop), float(ys.stop))
    return None if best is None else best[1:]


def texture_facts():
    """Per hazard class: artwork bbox, and the expected box/yellow ratio."""
    facts = {}
    for name in L.CLASS_NAMES:
        img = Image.open(L.texture_path(name)).convert("RGB")
        art = artwork_bbox(img)
        yellow = component_bbox(yellow_mask(img), (0, 0, img.width, img.height))
        ratio = None
        if art and yellow:
            yellow_w = (yellow[2] - yellow[0]) / float(img.width)
            if yellow_w > 0.01:
                ratio = (art[2] - art[0]) / yellow_w
        facts[name] = {"artwork": art, "ratio": ratio}
    return facts


def measure(records):
    """records: iterable of (rgb_array, box).  -> list of box_w/yellow_w."""
    out = []
    for rgb, box in records:
        yb = component_bbox(yellow_mask(rgb), box)
        if yb is None or yb[2] - yb[0] < 4:
            continue
        out.append((box[2] - box[0]) / (yb[2] - yb[0]))
    return out


def synth_records(frames, seed, mode, mixed=False):
    """mode: 'artwork' or 'card'."""
    G.BOX_EXTENT_M = None if mode == "artwork" else L.CARD_FRAME_M
    out = []
    rng = random.Random(seed)
    done = 0
    index = 0
    while done < frames and index < frames * 40:
        sample = G.make_sample(index, rng, mixed)
        index += 1
        if sample is None:
            continue
        img, boxes = sample
        hazard = [b for b in boxes if L.zone_of(b[0]) == "hazard"]
        if not hazard:
            continue
        done += 1
        rgb = np.asarray(img.convert("RGB"), np.float32)
        for _, x1, y1, x2, y2 in hazard:
            out.append((rgb, (x1, y1, x2, y2)))
    return out


def real_records(src, limit):
    ann_dir = os.path.join(src, "Annotations")
    img_dir = os.path.join(src, "JPEGImages")
    if not os.path.isdir(ann_dir):
        return None
    out = []
    for path in sorted(glob.glob(os.path.join(ann_dir, "*.xml")))[:limit]:
        root = ET.parse(path).getroot()
        stem = os.path.splitext(os.path.basename(path))[0]
        image_path = os.path.join(img_dir, stem + ".jpg")
        if not os.path.exists(image_path):
            continue
        box = None
        for obj in root.findall("object"):
            name = L.resolve_class(obj.find("name").text)
            if name is None or L.zone_of(name) != "hazard":
                continue
            b = obj.find("bndbox")
            box = tuple(float(b.find(t).text)
                        for t in ("xmin", "ymin", "xmax", "ymax"))
            break
        if box is None:
            continue
        img = ImageOps.exif_transpose(Image.open(image_path)).convert("RGB")
        scale = 1.0
        if max(img.size) > 1000:                  # the mask does not need 6 MP
            scale = 1000.0 / max(img.size)
            img = img.resize((max(1, int(img.width * scale)),
                              max(1, int(img.height * scale))))
        out.append((np.asarray(img, np.float32),
                    tuple(v * scale for v in box)))
    return out


def print_artwork(facts):
    """Emit the FACE_ARTWORK block, read straight off the textures."""
    print("FACE_ARTWORK = {")
    for zone in ("hazard", "person"):
        boxes = [facts[n]["artwork"] for n in L.CLASS_NAMES
                 if L.zone_of(n) == zone and facts[n]["artwork"]]
        if not boxes:
            continue
        x1 = sum(b[0] for b in boxes) / len(boxes)
        y1 = sum(b[1] for b in boxes) / len(boxes)
        x2 = sum(b[2] for b in boxes) / len(boxes)
        y2 = sum(b[3] for b in boxes) / len(boxes)
        if zone == "hazard":
            pts = [(0.5 * (x1 + x2), y1), (x1, y2), (x2, y2)]
            comment = "    # apex, then the two base corners"
        else:
            pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
            comment = ""
        body = ", ".join("(%.3f, %.3f)" % (round(u, 3), round(v, 3))
                         for u, v in pts)
        print('    "%s": (%s),%s' % (zone, body, comment))
    print("}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=30,
                    help="synthetic frames to measure (default 30)")
    ap.add_argument("--seed", type=int, default=20261006)
    ap.add_argument("--real-src", default=L.REAL_DATASET_DIR)
    ap.add_argument("--real-limit", type=int, default=120)
    ap.add_argument("--print-artwork", action="store_true",
                    help="print the FACE_ARTWORK block for labels.py and exit")
    args = ap.parse_args()

    facts = texture_facts()

    if args.print_artwork:
        print_artwork(facts)
        return 0

    failures = []

    # ---------------------------------------------------------- statement 1
    print("1. labels.FACE_ARTWORK vs the artwork in the textures")
    print("   %-26s %-16s %-16s %s" % ("class", "texture", "labels", "delta"))
    for name in L.CLASS_NAMES:
        measured = facts[name]["artwork"]
        dw_, dh_, _, _ = L.artwork_box(name)
        declared = (dw_ / L.CARD_FACE_M, dh_ / L.CARD_FACE_M)
        if measured is None:
            failures.append("%s: no artwork found in the texture" % name)
            print("   %-26s %-16s %-16s no artwork" % (name, "-", "-"))
            continue
        mw, mh = measured[2] - measured[0], measured[3] - measured[1]
        dw = abs(mw - declared[0])
        dh = abs(mh - declared[1])
        flag = "" if max(dw, dh) <= ARTWORK_TOL else "   <-- FAIL"
        print("   %-26s %-16s %-16s %.3f, %.3f%s"
              % (name, "%.3f %.3f" % (mw, mh),
                 "%.3f %.3f" % declared, dw, dh, flag))
        if max(dw, dh) > ARTWORK_TOL:
            failures.append(
                "%s: texture artwork is %.3f x %.3f of the tile, labels says "
                "%.3f x %.3f - the renderer boxes a region the card does not "
                "have (re-run with --print-artwork)"
                % (name, mw, mh, declared[0], declared[1]))

    # ---------------------------------------------------------- statement 2
    print()
    print("2. box_width / yellow_width, rendered vs photographed")
    synth = measure(synth_records(args.frames, args.seed, "artwork"))
    real = None
    real_records_ = real_records(args.real_src, args.real_limit)
    if real_records_ is None:
        print("   real set not present at %s - only statement 1 is checked"
              % args.real_src)
    else:
        real = measure(real_records_)

    def stat(values, label):
        if not values:
            print("   %-16s no measurable sample" % label)
            return None
        arr = np.asarray(values, float)
        print("   %-16s n=%-4d median %.3f  mean %.3f  p10-p90 %.2f-%.2f"
              % (label, len(arr), float(np.median(arr)), float(arr.mean()),
                 float(np.percentile(arr, 10)), float(np.percentile(arr, 90))))
        return float(np.median(arr))

    print("   expected from the artwork : %s"
          % ", ".join("%s %.3f" % (n.split("_")[1], facts[n]["ratio"])
                      for n in L.CLASS_NAMES
                      if L.zone_of(n) == "hazard" and facts[n]["ratio"]))
    synth_ratio = stat(synth, "rendered")
    real_ratio = stat(real, "photographed")

    if synth_ratio is not None and real_ratio is not None:
        gap = abs(synth_ratio - real_ratio) / real_ratio
        print("   agreement: %.1f%% apart (%.3f vs %.3f)"
              % (gap * 100.0, synth_ratio, real_ratio))
        if gap > RATIO_TOL:
            failures.append(
                "the two halves disagree by %.0f%% about the box: rendered "
                "%.3f, photographed %.3f.  A merged set now teaches the "
                "regressor to split the difference."
                % (gap * 100, synth_ratio, real_ratio))

    if synth_ratio is not None:
        wanted = [facts[n]["ratio"] for n in L.CLASS_NAMES
                  if L.zone_of(n) == "hazard" and facts[n]["ratio"]]
        expected = sum(wanted) / len(wanted)
        if abs(synth_ratio - expected) / expected > RATIO_TOL:
            failures.append(
                "the renderer boxes %.3f but the artwork implies %.3f - the "
                "box is not coming from labels.artwork_box()"
                % (synth_ratio, expected))

    # ------------------------------------------------- plumbing sanity check
    print()
    print("3. --box-extent actually changes the box")
    card_ratio = measure(synth_records(max(8, args.frames // 3),
                                       args.seed, "card"))
    if synth_ratio is not None and card_ratio:
        card_median = float(np.median(np.asarray(card_ratio, float)))
        # Head-on, a card box would be CARD_FRAME_M / artwork width times the
        # artwork ratio.  It always measures higher, because a square box's
        # bbox grows as w|cos t| + h|sin t| once the card is yawed while a
        # triangle's does not - which is the whole reason the artwork entry is
        # modelled as a triangle.  So this is a lower bound, not a prediction.
        lower = synth_ratio * (L.CARD_FRAME_M / L.artwork_box("hazard_flammable")[0])
        print("   artwork %.3f, card %.3f (head-on lower bound %.3f)"
              % (synth_ratio, card_median, lower))
        if abs(card_median - synth_ratio) < 0.15:
            failures.append("--box-extent card gave the same box as artwork, so "
                            "the extent is not plumbed through")
        elif card_median < lower * 0.9:
            failures.append("--box-extent card gave %.3f, below the head-on "
                            "lower bound %.3f - the card extent is not being "
                            "applied as a square outline"
                            % (card_median, lower))
    G.BOX_EXTENT_M = None        # leave the module as found

    print()
    if failures:
        for f in failures:
            print("FAIL: %s" % f)
        return 1
    print("PASS: labels, renderer and photographs describe the same box.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
