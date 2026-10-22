#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Turn the hand-labelled real photographs into the same VOC form as the
synthetic set, so the two can be trained on together.

WHY THE TWO SETS ARE MERGED RATHER THAN TRAINED SEPARATELY
----------------------------------------------------------
The fine-tuning set on the desktop (TargetDelection1.1) has 120 photographs and
four of the ten classes: 人员 / 识别项-易燃 / 识别项-易爆 / 识别项-有毒.  There
is no 识别项-腐蚀, and nothing at all for the five signs added later
(1-06, 1-27, 1-28, 1-42, 1-43).  Training on it alone would produce a detector
that can never fire six of its ten classes - and because the task names the sign
it wants, a miss is not a near miss, it is a box around the wrong card.

The synthetic renderer, by contrast, draws all ten textures from the real
world-file geometry and can produce tens of thousands of frames.  So the split
of responsibilities is:

    synthetic  -> teaches the label set, the geometry, the scale, and the
                  shadow-free flat appearance
    real       -> teaches what the print actually looks like through a lens:
                  the yellow that comes out ochre, the thicker black rule, the
                  specular sheen, the more oblique perspective

Merging keeps the six synthetic-only classes alive while the real half pulls the
other four onto the true appearance.  That is the whole reason this script
exists rather than a rename of the dataset folders.

WHAT IT CHANGES AND WHY
-----------------------
  * label names   Chinese -> the class names in labels.py, via resolve_class().
                  An unrecognised name drops that one box and is reported; it
                  does not abort the run and it does not reach the TFRecord,
                  where prepare_pytorch_voc.py would refuse to continue.
  * resolution    4284x5712 etc -> 640x480, the resolution the flight camera
                  actually delivers.  Training at six megapixels and inferring
                  at VGA wastes the only expensive resource there is.
  * aspect ratio  letterboxed, NOT stretched.  The source photographs are
                  portrait (3:4 and similar); the camera is 4:3 landscape.  The
                  GB 2894 pictogram is a square with a prescriptive border
                  ratio, so squashing it to fit would teach the network a shape
                  that does not exist on the field.
  * filenames     prefixed (default "real_") so appending these into a
                  synthetic dataset directory cannot collide with the renderer's
                  own names, and so the origin of every image stays obvious.

TRUNCATED BOXES
---------------
Two of the 120 annotations are marked truncated in the source, and both really
are: in IMG_2802 the sign runs off the bottom edge of the photograph, in
IMG_2686 off the right edge.  Nothing here can put back what the camera never
recorded, so those boxes stay - a sign half out of frame is a thing that
happens in flight, and two frames out of 4120 is not enough to bend the
detector.

What must NOT happen is losing the fact.  The output used to hardcode
truncated="0", which erased the flag that the labeller had taken the trouble to
set.  The flag is now carried through, and our own transforms can raise it:
clamp_and_filter() reports when it had to clip a box to the canvas, and
crop_to_fraction() refuses to crop so tightly that a box ends up cut.  Note the
Object Detection API's SSD path ignores truncated entirely - the field is kept
for whoever reads the dataset next, not because it changes the loss.

EXIF
----
Every photograph in the current set carries orientation 1, so the stored pixel
size and the annotation's declared size agree and nothing is rotated here.  The
conversion still calls exif_transpose() and still verifies the declared size
against the real one, because a future batch shot on a differently held phone
would carry orientation 6 or 8 - and labelImg writes the size of the image as
it displayed it, which is the transposed one.  Blending a rotation into a
dataset after the fact is silent: the boxes stay plausible-looking and simply
sit on the wrong part of the picture.

Usage
-----
    python3 tools/ssd_training/prepare_real_dataset.py \
        --src ~/Desktop/TargetDelection1.1 --out data/task_real --preview 12

    # or append straight into the synthetic set:
    python3 tools/ssd_training/prepare_real_dataset.py \
        --src ~/Desktop/TargetDelection1.1 --out data/task
"""

import argparse
import glob
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import labels as L  # noqa: E402  (path set up above)

DEFAULT_SRC = L.REAL_DATASET_DIR

# Neutral grey.  Used when --fill is not given and the border sample is
# degenerate; also the background most photo-editing canvases use, so it reads
# as "not part of the scene" rather than as another surface.
NEUTRAL_FILL = (114, 114, 114)


def _flag(node, tag):
    """Read an optional 0/1 flag; labelImg omits <difficult> on old files."""
    child = node.find(tag)
    if child is None or child.text is None:
        return 0
    try:
        return 1 if int(float(child.text)) else 0
    except ValueError:
        return 0


def load_boxes(ann_path):
    """Read one labelImg XML -> (declared_size, [(name, x1,y1,x2,y2, truncated)]).

    Every box carries a truncated flag from here on, because it is the only
    place that knows: once the box is a tuple of floats the reason it is flush
    with the frame edge is gone.
    """
    root = ET.parse(ann_path).getroot()
    size = root.find("size")
    declared = (int(size.find("width").text), int(size.find("height").text))
    boxes = []
    for obj in root.findall("object"):
        name_node = obj.find("name")
        box = obj.find("bndbox")
        if name_node is None or box is None:
            continue
        boxes.append((
            (name_node.text or "").strip(),
            float(box.find("xmin").text),
            float(box.find("ymin").text),
            float(box.find("xmax").text),
            float(box.find("ymax").text),
            _flag(obj, "truncated")))
    return declared, boxes


def border_fill(image, width):
    """Median colour of the image border, as a letterbox fill.

    Sampling beats a fixed grey: these photographs are taken on whatever
    surface was to hand, and a fill that matches the surrounding surface keeps
    the padded band from reading as a hard artificial edge that the network may
    learn to key on.
    """
    if width < 4 or image.height < 4:
        return NEUTRAL_FILL
    import numpy as np
    px = np.asarray(image.convert("RGB"))
    band = max(2, width // 40)
    strips = [px[:band, :, :], px[-band:, :, :], px[:, :band, :], px[:, -band:, :]]
    flat = np.concatenate([s.reshape(-1, 3) for s in strips], axis=0)
    return tuple(int(v) for v in np.median(flat, axis=0))


def parse_fill(text):
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--fill wants R,G,B")
    try:
        rgb = tuple(int(p) for p in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("--fill wants three integers")
    if any(not 0 <= v <= 255 for v in rgb):
        raise argparse.ArgumentTypeError("--fill components must be 0..255")
    return rgb


def letterbox(image, boxes, target_w, target_h, allow_upscale, fill):
    """Scale to fit, centre on a canvas of the exact target size.

    Returns (canvas, mapped_boxes, scale, offset).  Boxes are mapped in the same
    transform that is applied to the pixels, which is the point: computing the
    two separately is how a dataset ends up with boxes that are consistently a
    few percent off.
    """
    from PIL import Image
    w, h = image.size
    scale = min(target_w / float(w), target_h / float(h))
    if not allow_upscale:
        scale = min(scale, 1.0)
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    offset = ((target_w - new_w) // 2, (target_h - new_h) // 2)

    canvas = Image.new("RGB", (target_w, target_h), fill)
    canvas.paste(image.convert("RGB").resize((new_w, new_h), Image.LANCZOS),
                 offset)

    ox, oy = offset
    mapped = []
    for name, x1, y1, x2, y2, trunc in boxes:
        mapped.append((name,
                       x1 * scale + ox, y1 * scale + oy,
                       x2 * scale + ox, y2 * scale + oy,
                       trunc))
    return canvas, mapped, scale, offset


def clamp_and_filter(boxes, width, height, min_px):
    """Clip to the canvas, drop degenerate boxes.

    Returns (kept, dropped, clipped).  `clipped` counts boxes the clamp had to
    shorten - those are ours, not the photographer's, so they get their
    truncated flag raised and are reported rather than disappearing into a
    coordinate that happens to equal the frame size.
    """
    kept, dropped, clipped = [], 0, 0
    for name, x1, y1, x2, y2, trunc in boxes:
        cx1 = max(0.0, min(float(width - 1), x1))
        cy1 = max(0.0, min(float(height - 1), y1))
        cx2 = max(0.0, min(float(width), x2))
        cy2 = max(0.0, min(float(height), y2))
        if cx2 - cx1 < min_px or cy2 - cy1 < min_px:
            dropped += 1
            continue
        if (cx1, cy1, cx2, cy2) != (x1, y1, x2, y2):
            clipped += 1
            trunc = 1
        kept.append((name, cx1, cy1, cx2, cy2, trunc))
    return kept, dropped, clipped


def crop_to_fraction(image, boxes, target_w, target_h, fraction):
    """Zoom in until the labelled box spans `fraction` of the frame width.

    Letterboxing a portrait photograph into a 4:3 frame shrinks the sign to
    about 0.13 of the width, while in flight the 0.336 m sign occupies 0.234 of
    it.  Zooming onto the deployment scale closes that gap - and because the
    window keeps the target's aspect ratio, the sign-to-background proportion
    inside it is unchanged, so the composition still matches.  It also crops
    away the letterbox band, which is the one part of the frame that never
    exists on the field.

    Two things stop the zoom from being a pure division, and both of them bite:

      * the window has the frame's 4:3 shape, so a box that is taller than 4:3
        can be taller than a window sized from its width alone.  A window that
        cuts a box teaches the network that half a sign is a whole sign, so the
        window is grown until every box fits;
      * a window wider than the frame would need the padding we just cropped
        away, so it is clamped instead and the effective fraction is whatever
        that leaves - which is why this returns the number it achieved and not
        the one it was asked for.

    Returns (image, boxes, scale, achieved_fraction).
    """
    from PIL import Image
    if not boxes:
        return image, boxes, 1.0, 0.0
    x1 = min(b[1] for b in boxes)
    y1 = min(b[2] for b in boxes)
    x2 = max(b[3] for b in boxes)
    y2 = max(b[4] for b in boxes)
    box_w = x2 - x1
    box_h = y2 - y1
    if box_w <= 1.0:
        return image, boxes, 1.0, 0.0

    win_w = box_w / float(fraction)
    win_w = max(win_w, box_w, box_h * target_w / float(target_h))  # keep the boxes whole
    win_w = max(1.0, min(float(target_w), win_w))
    win_h = win_w * target_h / float(target_w)

    # Place the window on the boxes' centre, clamped into the range that keeps
    # the window inside the frame AND every box inside the window.  Since
    # win_w >= box_w that range is never empty: at worst it collapses to the
    # single value that is the full frame.
    def place(lo_edge, hi_edge, window, target):
        centre = (lo_edge + hi_edge) / 2.0 - window / 2.0
        lo = max(0.0, hi_edge - window)
        hi = min(float(target) - window, lo_edge)
        return min(max(centre, lo), hi)

    left = place(x1, x2, win_w, target_w)
    top = place(y1, y2, win_h, target_h)
    crop_box = (int(round(left)), int(round(top)),
                int(round(left + win_w)), int(round(top + win_h)))

    cropped = image.crop(crop_box)
    if cropped.width < 2 or cropped.height < 2:
        return image, boxes, 1.0, box_w / float(target_w)
    resized = cropped.resize((target_w, target_h), Image.LANCZOS)
    scale = target_w / float(cropped.width)
    moved = [(name,
              (bx1 - crop_box[0]) * scale, (by1 - crop_box[1]) * scale,
              (bx2 - crop_box[0]) * scale, (by2 - crop_box[1]) * scale,
              trunc)
             for name, bx1, by1, bx2, by2, trunc in boxes]
    achieved = box_w * scale / float(target_w)
    return resized, moved, scale, achieved


def voc_xml(stem, boxes, width, height, source_note):
    """Same shape as the renderer's output, plus the provenance note."""
    root = ET.Element("annotation", {"verified": "yes"})
    ET.SubElement(root, "folder").text = "images"
    ET.SubElement(root, "filename").text = stem + ".jpg"
    ET.SubElement(root, "path").text = "images/" + stem + ".jpg"
    src = ET.SubElement(root, "source")
    ET.SubElement(src, "database").text = source_note
    size = ET.SubElement(root, "size")
    ET.SubElement(size, "width").text = str(width)
    ET.SubElement(size, "height").text = str(height)
    ET.SubElement(size, "depth").text = "3"
    ET.SubElement(root, "segmented").text = "0"
    for name, x1, y1, x2, y2, trunc in boxes:
        obj = ET.SubElement(root, "object")
        ET.SubElement(obj, "name").text = name
        ET.SubElement(obj, "pose").text = "Unspecified"
        ET.SubElement(obj, "truncated").text = str(int(trunc))
        ET.SubElement(obj, "difficult").text = "0"
        bnd = ET.SubElement(obj, "bndbox")
        for tag, value in (("xmin", x1), ("ymin", y1), ("xmax", x2), ("ymax", y2)):
            ET.SubElement(bnd, tag).text = str(int(round(value)))
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    return tree


def draw_preview(records, path, columns=4, tile_max=320):
    """Contact sheet of the converted frames, hazard red / person blue.

    Worth the twenty lines: every silent geometry bug in this pipeline - a
    dropped exif_transpose, a scale applied twice, a box written in pixels
    instead of after mapping - is invisible in a log line and obvious here.
    """
    from PIL import Image, ImageDraw
    if not records:
        return False
    tiles = []
    for image, boxes in records:
        tile = image.copy()
        scale = min(tile_max / tile.width, tile_max / tile.height, 1.0)
        tile = tile.resize((int(tile.width * scale), int(tile.height * scale)),
                           Image.LANCZOS)
        draw = ImageDraw.Draw(tile)
        for name, x1, y1, x2, y2, trunc in boxes:
            colour = (255, 0, 0) if L.zone_of(name) == "hazard" else (0, 80, 255)
            draw.rectangle([x1 * scale, y1 * scale, x2 * scale, y2 * scale],
                           outline=colour, width=2)
            if trunc:
                # One of the two boxes the photographer cut.  Marked here
                # because on the contact sheet it is otherwise invisible: a
                # truncated sign looks exactly like a sign that is simply
                # close to the edge.
                draw.text((x1 * scale + 3, max(0.0, y1 * scale - 12)),
                          "trunc", fill=colour)
        tiles.append(tile)

    cols = max(1, min(columns, len(tiles)))
    rows = (len(tiles) + cols - 1) // cols
    cell_w = max(t.width for t in tiles)
    cell_h = max(t.height for t in tiles)
    sheet = Image.new("RGB", (cols * cell_w, rows * cell_h), (24, 24, 24))
    for index, tile in enumerate(tiles):
        cx = (index % cols) * cell_w + (cell_w - tile.width) // 2
        cy = (index // cols) * cell_h + (cell_h - tile.height) // 2
        sheet.paste(tile, (cx, cy))
    sheet.save(path, quality=88)
    return True


def main():
    ap = argparse.ArgumentParser(
        description="Convert the hand-labelled real photographs to dataset VOC.")
    ap.add_argument("--src", default=DEFAULT_SRC,
                    help="folder holding Annotations/ and JPEGImages/ "
                         "(default: %s)" % DEFAULT_SRC)
    ap.add_argument("--out", required=True,
                    help="dataset root; images/ and annotations/ are created "
                         "inside it, and an existing set is appended to")
    ap.add_argument("--width", type=int, default=L.CAM_WIDTH)
    ap.add_argument("--height", type=int, default=L.CAM_HEIGHT)
    ap.add_argument("--prefix", default="real_",
                    help="filename prefix so a merge cannot collide with the "
                         "renderer's names (default: real_)")
    ap.add_argument("--fill", type=parse_fill, default=None,
                    help="letterbox colour as R,G,B; default samples each "
                         "image's own border")
    ap.add_argument("--min-box-px", type=int, default=8,
                    help="drop a box thinner than this after scaling; the "
                         "Object Detection API turns degenerate boxes into NaN "
                         "(default: 8)")
    ap.add_argument("--fit-box-fraction", type=float, default=0.0, metavar="F",
                    help="zoom each frame until the labelled box spans F of the "
                         "frame width.  0 (default) keeps the plain letterbox; "
                         "the deployment scale is %.3f"
                         % L.deployment_box_fraction("hazard_flammable"))
    ap.add_argument("--quality", type=int, default=92)
    ap.add_argument("--allow-upscale", action="store_true",
                    help="permit scaling up; off by default because a source "
                         "smaller than the target has no detail to invent")
    ap.add_argument("--preview", type=int, default=0, metavar="N",
                    help="also write preview_real.jpg with N converted frames")
    ap.add_argument("--dry-run", action="store_true",
                    help="report exactly what would be written, write nothing")
    args = ap.parse_args()

    if args.width < 32 or args.height < 32:
        raise SystemExit("--width/--height look wrong: %d x %d"
                         % (args.width, args.height))
    if args.fit_box_fraction and not 0.0 < args.fit_box_fraction <= 1.0:
        raise SystemExit("--fit-box-fraction must be in (0, 1], got %g"
                         % args.fit_box_fraction)

    ann_dir = os.path.join(args.src, "Annotations")
    img_dir = os.path.join(args.src, "JPEGImages")
    if not os.path.isdir(ann_dir) or not os.path.isdir(img_dir):
        raise SystemExit(
            "expected Annotations/ and JPEGImages/ under %s.\n"
            "Pass --src if the dataset moved; the current default is %s"
            % (args.src, DEFAULT_SRC))

    try:
        from PIL import Image, ImageOps
    except ImportError as exc:
        raise SystemExit(
            "Pillow is required (pip install Pillow).  original error: %s" % exc)

    anns = sorted(glob.glob(os.path.join(ann_dir, "*.xml")))
    if not anns:
        raise SystemExit("no .xml under %s" % ann_dir)

    out_images = os.path.join(args.out, "images")
    out_anns = os.path.join(args.out, "annotations")
    if not args.dry_run:
        os.makedirs(out_images, exist_ok=True)
        os.makedirs(out_anns, exist_ok=True)

    stats = {
        "read": 0, "written": 0, "empty": 0, "missing_image": 0,
        "size_mismatch": 0, "dropped_box": 0, "unknown_label": 0,
        "truncated_box": 0, "clipped_box": 0,
    }
    per_class = {name: 0 for name in L.CLASS_NAMES}
    unknown_seen = {}
    fractions = []          # box width / frame width, for the scale check
    achieved_fractions = []  # what --fit-box-fraction actually landed on
    wide_frames = 0          # frames where the sign was already past the target
    truncated_seen = []      # (stem, class, edge) for the report
    preview_records = []
    preview_every = max(1, len(anns) // args.preview) if args.preview else 0

    for index, ann_path in enumerate(anns):
        stem = os.path.splitext(os.path.basename(ann_path))[0]
        image_path = os.path.join(img_dir, stem + ".jpg")
        raw_declared, raw_boxes = load_boxes(ann_path)

        if not os.path.exists(image_path):
            stats["missing_image"] += 1
            print("  missing image for %s" % ann_path, file=sys.stderr)
            continue

        image = ImageOps.exif_transpose(Image.open(image_path)).convert("RGB")
        stats["read"] += 1

        # The annotation was drawn on a specific pixel grid; if the orientation
        # bake moved us off it, every box is wrong and it is better to say so
        # than to write a quietly misaligned record.
        if (image.width, image.height) != raw_declared:
            stats["size_mismatch"] += 1
            print("  size mismatch %s: annotation says %dx%d, image is %dx%d - "
                  "skipped" % (stem, raw_declared[0], raw_declared[1],
                               image.width, image.height), file=sys.stderr)
            continue

        resolved = []
        for name, x1, y1, x2, y2, trunc in raw_boxes:
            mapped_name = L.resolve_class(name)
            if mapped_name is None:
                stats["unknown_label"] += 1
                unknown_seen[name] = unknown_seen.get(name, 0) + 1
                continue
            resolved.append((mapped_name, x1, y1, x2, y2, trunc))

        fill = args.fill or border_fill(image, args.width)
        canvas, mapped, _, _ = letterbox(
            image, resolved, args.width, args.height,
            args.allow_upscale, fill)
        if args.fit_box_fraction:
            canvas, mapped, _, achieved = crop_to_fraction(
                canvas, mapped, args.width, args.height, args.fit_box_fraction)
            achieved_fractions.append((stem, achieved))
            if achieved > args.fit_box_fraction + 0.005:
                # The clamp only ever widens the window, so a frame can end up
                # with a box LARGER than requested but never smaller: the sign
                # was already bigger than the deployment scale in this shot.
                wide_frames += 1
        kept, dropped, clipped = clamp_and_filter(
            mapped, args.width, args.height, args.min_box_px)
        stats["dropped_box"] += dropped
        stats["clipped_box"] += clipped

        if not kept:
            stats["empty"] += 1
            continue

        for name, x1, y1, x2, y2, trunc in kept:
            per_class[name] += 1
            fractions.append((x2 - x1) / float(args.width))
            if trunc:
                stats["truncated_box"] += 1
                edges = []
                if x1 <= 0.5:
                    edges.append("left")
                if y1 <= 0.5:
                    edges.append("top")
                if x2 >= args.width - 0.5:
                    edges.append("right")
                if y2 >= args.height - 0.5:
                    edges.append("bottom")
                truncated_seen.append((stem, name, "+".join(edges) or "inside"))

        out_stem = args.prefix + stem
        if not args.dry_run:
            canvas.save(os.path.join(out_images, out_stem + ".jpg"),
                        quality=args.quality)
            voc_xml(out_stem, kept, args.width, args.height,
                    "GC 2026 task markers (real photograph)").write(
                os.path.join(out_anns, out_stem + ".xml"))

        stats["written"] += 1
        if args.preview and index % preview_every == 0 \
                and len(preview_records) < args.preview:
            preview_records.append((canvas, kept))

    # ------------------------------------------------------------------ report
    print("source        : %s" % args.src)
    print("destination   : %s%s"
          % (args.out, "   (dry run, nothing written)" if args.dry_run else ""))
    print("target size   : %d x %d%s"
          % (args.width, args.height,
             "" if (args.width, args.height) == (L.CAM_WIDTH, L.CAM_HEIGHT)
             else "   (not the flight camera's %dx%d)"
                  % (L.CAM_WIDTH, L.CAM_HEIGHT)))
    if args.fit_box_fraction:
        print("zoom          : labelled box -> %.3f of frame width, then crop"
              % args.fit_box_fraction)
        if achieved_fractions:
            widths = [a for _, a in achieved_fractions]
            print("                landed on %.3f min / %.3f mean"
                  % (min(widths), sum(widths) / len(widths)))
            if wide_frames:
                print("                %d frame(s) already had the sign larger "
                      "than that and kept" % wide_frames)
                print("                the full frame - the window has to fit "
                      "inside the picture.")
    print()
    print("read          : %d" % stats["read"])
    print("written       : %d" % stats["written"])
    for key in ("empty", "missing_image", "size_mismatch",
                "dropped_box", "unknown_label"):
        if stats[key]:
            print("%-14s: %d" % (key, stats[key]))

    if stats["truncated_box"]:
        print()
        print("truncated     : %d box(es) left the photograph's edge, %d of "
              "them clipped by this script" % (stats["truncated_box"],
                                               stats["clipped_box"]))
        for stem, name, edge in truncated_seen:
            print("  %-18s %-26s %s" % (stem, name, edge))
        print("  Kept on purpose: a sign half out of frame is a thing that "
              "happens in flight.")
        print("  The Object Detection API's SSD path ignores <truncated>, so "
              "these")
        print("  boxes still teach 'a cut sign is a whole sign'.  At %d of "
              "%d frames that is noise."
              % (stats["truncated_box"], stats["written"] or 1))
        if stats["clipped_box"]:
            print("  A non-zero 'clipped by this script' means a window or "
                  "letterbox")
            print("  cut into a box: widen --fit-box-fraction or drop "
                  "--allow-upscale.")

    if unknown_seen:
        print()
        print("unrecognised labels (box dropped, not fatal).  Add them to "
              "labels.CLASS_ALIASES if they are real:")
        for name, count in sorted(unknown_seen.items(), key=lambda kv: -kv[1]):
            print("  %-24s %6d" % (name, count))

    print()
    print("instances per class:")
    for name in L.CLASS_NAMES:
        mark = "" if per_class[name] else "   <- no examples in this set"
        print("  %-26s %6d%s" % (name, per_class[name], mark))

    missing = [n for n in L.CLASS_NAMES if per_class[n] == 0]
    if missing:
        print()
        print("NOTE: %s ha%s no labelled example here.  The synthetic set is "
              "what keeps %s trainable; merge the two before training."
              % (", ".join(missing), "s" if len(missing) == 1 else "ve",
                 "it" if len(missing) == 1 else "them"))

    if fractions:
        mean = sum(fractions) / len(fractions)
        ppm = L.frame_width_px_per_m()
        # The box covers the printed sign, not the card, so the scale to match
        # is the artwork's - 0.336 m - and not the 0.450 m frame.  Using the
        # frame here reports a target 34% too large and then advises a
        # --fit-box-fraction that over-zooms every photograph.
        want = L.deployment_box_fraction("hazard_flammable")
        print()
        print("box width / frame width, mean over %d box(es): %.3f"
              % (len(fractions), mean))
        print("  in flight the %.3f m sign fills %.3f of the frame width "
              "(%.0f px/m at %.2f m)"
              % (L.artwork_box("hazard_flammable")[0], want, ppm, L.CAM_ALT_M))
        ratio = mean / want if want else 0.0
        if ratio >= 1.25 or ratio <= 0.80:
            print("  -> %.2fx the flight size.  Not fatal - SSD's anchor "
                  "pyramid covers a wide range - but the synthetic half is "
                  "what anchors the true scale." % ratio)
            if not args.fit_box_fraction:
                print("     To land on the deployment scale instead, re-run "
                      "with --fit-box-fraction %.3f" % want)
        else:
            print("  -> %.2fx the flight size, close enough to reinforce it."
                  % ratio)

    if args.preview:
        target = os.path.join(args.out, "preview_real.jpg")
        if args.dry_run:
            print()
            print("would write a %d-frame preview to %s"
                  % (len(preview_records), target))
        elif draw_preview(preview_records, target):
            print()
            print("preview: %s  (red = hazard, blue = person)" % target)

    if stats["written"] == 0:
        print("\nnothing written.  Fix the source before training.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
