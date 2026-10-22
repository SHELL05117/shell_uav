#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The one place that decides what the detector can see.

Everything else in tools/ssd_training derives from this file: the synthetic
renderer draws these textures, the TFRecord converter writes these ids, and the
runtime node maps ids back to names.  Change a name here and every artefact
that was trained before the change becomes stale, which is deliberate - a
label map that drifts from the trained model produces confident nonsense.

CLASS IDS START AT 1
--------------------
TensorFlow's Object Detection API reserves 0 for background, and
ssd_ros/data/ssd_coco_labels.txt follows the same convention (its first line is
"person", i.e. id 1).  num_classes in the pipeline config is 10 for the ten
entries below, and the UFF graphsurgeon config must say 11 wherever the COCO one
says 91 - the plugin wants background included, the training config does not.
Getting that pair wrong is the classic way to end up with a network that
detects nothing and reports no error.

WHY THE HAZARD CLASSES ARE SPLIT BY PICTOGRAM
---------------------------------------------
The competition asks for the CORRECT hazardous-material target to be boxed and
dropped on.  "Correct" is a property of the pictogram - the task names one
hazard, and GB 2894-2025 附录C 表C.1 defines a different warning sign for each
- so the detector has to tell them apart rather than just saying "a yellow card
is here".  Position alone would not survive a change of task assignment, and a
single class would force the aircraft to guess and drop on the wrong box.

Nine hazard signs are in scope, all pulled from the standard's own artwork
(see tools/make_task_markers.py):

    1-02 当心火灾      1-03 当心爆炸      1-04 当心腐蚀
    1-05 当心中毒      1-06 当心生物危害  1-27 当心电离辐射
    1-28 当心裂变物质  1-42 当心氧化物    1-43 当心高压气瓶

1-27 and 1-28 are the one pair that is easy to confuse: 1-27 is the plain
trefoil, 1-28 has an arc under it.  The archived originals in
tools/assets/gb2894/reference/ settle it; do not go by memory.

The person class stays single: rules only ever ask for 受伤人员 to be boxed, and
there is no second person-card artwork to distinguish it from.
"""

import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TEXTURE_DIR = os.path.join(
    ROOT, "src/gc_2026_sim/gazebo/models/gc_task_markers/materials/textures")

# (name, texture file, zone) - list order IS the class id, first entry is 1.
#
# Ordered by GB 2894-2025 code so that the list reads the same way the task
# sheet and tools/assets/gb2894/ do, with the person card last because it is
# the one class that is not a GB warning sign.
CLASSES = [
    # 1-02
    ("hazard_flammable", "gc_sign_flammable.png", "hazard"),
    # 1-03
    ("hazard_explosive", "gc_sign_explosive.png", "hazard"),
    # 1-04
    ("hazard_corrosive", "gc_sign_corrosive.png", "hazard"),
    # 1-05
    ("hazard_toxic",     "gc_sign_toxic.png",     "hazard"),
    # 1-06
    ("hazard_biohazard", "gc_sign_biohazard.png", "hazard"),
    # 1-27
    ("hazard_ionizing_radiation", "gc_sign_ionizing_radiation.png", "hazard"),
    # 1-28
    ("hazard_fissionable", "gc_sign_fissionable.png", "hazard"),
    # 1-42
    ("hazard_oxidizer",  "gc_sign_oxidizer.png",  "hazard"),
    # 1-43
    ("hazard_gas_cylinder", "gc_sign_gas_cylinder.png", "hazard"),
    # the portrait card - no GB warning sign exists for it
    ("person_injured",   "gc_sign_person.png",    "person"),
]

CLASS_NAMES = [name for name, _, _ in CLASSES]
NUM_CLASSES = len(CLASSES)

# ---------------------------------------------------------------------------
# Aliases - what the label files in the wild actually say.
#
# The fine-tuning set on the desktop (TargetDelection1.1) was labelled in
# Chinese with labelImg, and its names do not match the class names the model
# is trained with.  Rather than renaming files in the dataset - which would
# have to be redone every time a batch is re-exported - the mapping lives here,
# next to the class list it maps onto, and prepare_real_dataset.py applies it.
#
# Note the gap.  The real set has 易燃 / 易爆 / 有毒 but no 腐蚀, and nothing at
# all for the four signs added later (1-06, 1-27, 1-28, 1-42, 1-43).  Those five
# classes are carried by the synthetic set alone, which is why the merged set -
# not the photographs on their own - is what gets trained.
# ---------------------------------------------------------------------------
CLASS_ALIASES = {
    # ---- the person card ---------------------------------------------------
    "人员": "person_injured",
    "受伤人员": "person_injured",
    "受伤": "person_injured",
    # ---- Chinese, as written by labelImg in TargetDelection1.1, and the
    #      variants the same names turn up as once the "识别项-" prefix is
    #      dropped (resolve_class strips that prefix on its own) ------------
    "识别项-易燃": "hazard_flammable",
    "识别项-易爆": "hazard_explosive",
    "识别项-有毒": "hazard_toxic",
    "识别项-有毒有害": "hazard_toxic",
    "识别项-腐蚀": "hazard_corrosive",
    "识别项-易腐蚀": "hazard_corrosive",
    "识别项-生物危害": "hazard_biohazard",
    "识别项-电离辐射": "hazard_ionizing_radiation",
    "识别项-裂变物质": "hazard_fissionable",
    "识别项-氧化物": "hazard_oxidizer",
    "识别项-高压气瓶": "hazard_gas_cylinder",
    "易燃": "hazard_flammable",
    "易爆": "hazard_explosive",
    "有毒": "hazard_toxic",
    "有毒有害": "hazard_toxic",
    "腐蚀": "hazard_corrosive",
    "易腐蚀": "hazard_corrosive",
    "生物危害": "hazard_biohazard",
    "电离辐射": "hazard_ionizing_radiation",
    "辐射": "hazard_ionizing_radiation",
    "裂变物质": "hazard_fissionable",
    "核裂变": "hazard_fissionable",
    "氧化物": "hazard_oxidizer",
    "氧化剂": "hazard_oxidizer",
    "高压气瓶": "hazard_gas_cylinder",
    "气瓶": "hazard_gas_cylinder",
    # ---- The ssd_ros / COCO label file convention, and the names the English
    #      side of GB 2894 uses, in case a set arrives labelled that way -----
    "person": "person_injured",
    "fire": "hazard_flammable",
    "flammable": "hazard_flammable",
    "explosive": "hazard_explosive",
    "corrosive": "hazard_corrosive",
    "toxic": "hazard_toxic",
    "biohazard": "hazard_biohazard",
    "biological_hazard": "hazard_biohazard",
    "ionizing_radiation": "hazard_ionizing_radiation",
    "radiation": "hazard_ionizing_radiation",
    "fissionable": "hazard_fissionable",
    "fission": "hazard_fissionable",
    "oxidizer": "hazard_oxidizer",
    "oxidizing": "hazard_oxidizer",
    "gas_cylinder": "hazard_gas_cylinder",
    "cylinder": "hazard_gas_cylinder",
}

# labelImg sets in the wild write this prefix inconsistently - with an ASCII
# dash, with a full-width one, or with no separator at all.  resolve_class
# removes whichever it finds so that a set typed the other way does not lose
# every single box to an unreadable label.
_PREFIXES = ("识别项-", "识别项－", "识别项_", "识别项 ", "识别项")

# Where the hand-labelled real photographs live, if they are on this machine.
# prepare_real_dataset.py takes it as an argument; this is only the default.
REAL_DATASET_DIR = os.path.join(
    os.path.expanduser("~"), "Desktop", "TargetDelection1.1")


def resolve_class(raw):
    """Map a label found in an annotation file onto a class name, or None.

    Returning None (rather than raising) is deliberate: a fine-tuning set that
    picked up one stray class should have that box dropped and reported, not
    abort the whole conversion after twenty minutes of resizing.
    """
    text = (raw or "").strip()
    if text in CLASS_NAMES:
        return text
    if text in CLASS_ALIASES:
        return CLASS_ALIASES[text]
    lowered = text.lower()
    for prefix in _PREFIXES:
        if lowered.startswith(prefix):
            lowered = lowered[len(prefix):].lstrip("-－_ ").strip()
            break
    if lowered in CLASS_NAMES:
        return lowered
    for alias, target in CLASS_ALIASES.items():
        if alias.lower() == lowered:
            return target
    for name in CLASS_NAMES:
        if lowered == name.lower():
            return name
    return None


def class_id(name):
    """1-based id for a class name; raises rather than returning a bad id."""
    try:
        return CLASS_NAMES.index(name) + 1
    except ValueError:
        raise KeyError("%r is not one of %s" % (name, ", ".join(CLASS_NAMES)))


def class_name(index):
    return CLASS_NAMES[index - 1]


def texture_path(name):
    for candidate, texture, _ in CLASSES:
        if candidate == name:
            return os.path.join(TEXTURE_DIR, texture)
    raise KeyError("%r is not a known class" % name)


def zone_of(name):
    for candidate, _, zone in CLASSES:
        if candidate == name:
            return zone
    raise KeyError("%r is not a known class" % name)


def hazard_names():
    return [name for name, _, zone in CLASSES if zone == "hazard"]


def person_names():
    return [name for name, _, zone in CLASSES if zone == "person"]


# ---------------------------------------------------------------------------
# Marker geometry and layout, copied from
# src/gc_2026_sim/gazebo/worlds/gc_2026_field.world.
#
# These are the REAL dimensions the aircraft will fly over, so the synthetic
# renderer uses them unscaled.  A training set rendered at a different card size
# than the flight field is a scale mismatch the model cannot recover from,
# because a 0.45 m frame at a fixed altitude always lands on the same pixel size.
# ---------------------------------------------------------------------------
CARD_FACE_M = 0.40          # textured sign face (box 0.40 x 0.40 x 0.008)
CARD_FRAME_M = 0.45         # outer extent of the four frame bars
FRAME_BAR_M = 0.025         # bar width, so (0.45 - 0.40) / 2
FRAME_RED = (230, 13, 13)   # <diffuse> 0.90 0.05 0.05 on hazard frames
FRAME_BLUE = (77, 179, 242)  # <diffuse> 0.30 0.70 0.95 on person frames

# ---------------------------------------------------------------------------
# WHAT THE ANNOTATION BOX COVERS - the one thing the two datasets must agree on
# ---------------------------------------------------------------------------
# A card is three nested things:
#
#   0.450 x 0.450 m  the four red (or blue) frame bars, outer extent
#   0.400 x 0.400 m  the textured face they enclose
#   0.336 x 0.291 m  the sign printed on that face   (hazard, head-on)
#   0.248 x 0.251 m  the figure printed on that face (person)
#
# The box has to be the third one, not the first, and that is not a preference
# - it is what the hand-labelled photographs already do.  Measured on
# TargetDelection1.1: for every hazard frame the annotated box is 1.24x the bbox
# of the yellow colour mask, and 1.24x is what the sign's own black border adds
# around the yellow triangle.  The printed sign, border and all, is what a human
# boxes and what the competition means by "the target".
#
# The hazard entry below is a TRIANGLE, not a rectangle, and the difference is
# not cosmetic.  An axis-aligned bbox of a rotating rectangle grows as
# w|cos t| + h|sin t|; of a rotating triangle it does not.  Modelling the sign
# as a rectangle therefore yields rendered boxes 1.33-1.70x the yellow width
# where the photographs sit at a tight 1.22-1.26, and the training set ends up
# teaching a box whose aspect ratio wanders with heading.  The photographs show
# the other behaviour: their box aspect ranges 1.02-1.16 over the set, which is
# what a triangle does and a rectangle does not.
#
# Values are (u, v) on the 0.40 m face, u left->right and v top->bottom in the
# texture, i.e. exactly the texture's own pixel coordinates divided by its side.
# All nine hazard pictograms share one outline because GB 2894 gives them all
# the same warning triangle; only the symbol inside differs.  That is why this
# table is keyed by zone and not by class - and it is an assumption, not a
# fact, so check_box_convention.py measures every texture individually and
# fails if any one of them has drifted away from the shared outline.
#
# check_box_convention.py re-derives these from the artwork on every run and
# fails if they have drifted.  One trap cost real time while measuring: the
# tiles carry a 1-3 px decorative outline just inside their edge, so a naive
# "everything that is not white" bbox reports 0.910 of the tile - square, 8%
# too large, wrong aspect ratio, and entirely plausible.
FACE_ARTWORK = {
    # apex, then the two base corners
    "hazard": ((0.500, 0.136), (0.080, 0.864), (0.920, 0.864)),
    # the figure fills a stand-in rectangle; there is no triangle to model
    "person": ((0.199, 0.164), (0.818, 0.164), (0.818, 0.791), (0.199, 0.791)),
}


def artwork_points(class_name):
    """The printed artwork's outline, in metres from the face centre.

    Hand this to a projection and take the bbox of the result; that is what a
    human annotator does, and it is why the box should be built from the outline
    rather than from a stored rectangle.
    """
    return [((u - 0.5) * CARD_FACE_M, (v - 0.5) * CARD_FACE_M)
            for u, v in FACE_ARTWORK[zone_of(class_name)]]


def artwork_box(class_name):
    """The artwork's head-on bounding box as (width, height, dx, dy) metres."""
    pts = artwork_points(class_name)
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    return (max(us) - min(us), max(vs) - min(vs),
            (max(us) + min(us)) / 2.0, (max(vs) + min(vs)) / 2.0)

# Column x and the spacing between cards, straight from the world file.  The
# world instantiates two hazard cards and three person cards, but all ten
# textures ship, and which hazard sign stands where is a one-word edit there:
#   hazard_zone_card_1  (1.40,  0.00)   hazard_zone_card_2  (1.40, -0.45)
#   person_zone_card_1  (2.70, -1.05)   card_2 (2.70, -1.85)  card_3 (2.70, -2.65)
HAZARD_X = 1.40
HAZARD_Y = (0.00, -0.45)
PERSON_X = 2.70
PERSON_Y = (-1.05, -1.85, -2.65)

# ---------------------------------------------------------------------------
# The on-board camera, from px4_sensor_rig/model.sdf.
#
#   <horizontal_fov>1.20</horizontal_fov>, 640x480, camera_link at z -0.15 with
#   pitch 1.5708 (straight down).  A synthetic frame rendered with a different
#   fov or a different mount drop would train the model on a field of view the
#   aircraft does not have.
# ---------------------------------------------------------------------------
CAM_HFOV_RAD = 1.20
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_MOUNT_DROP_M = 0.15     # camera is this far below the flight controller
TASK_HEIGHT_M = 1.20        # task_height, the altitude a task is done at
CAM_ALT_M = TASK_HEIGHT_M - CAM_MOUNT_DROP_M   # 1.05 m above the mat


def frame_width_px_per_m(alt_m=None):
    """Image scale on the mat: fx / altitude, with fx = W / (2 tan(hfov/2)).

    At the task altitude this is 445 px/m, which is why the 0.336 m sign lands
    on 150 px of a 640 px frame.  Both the renderer and prepare_real_dataset.py
    need this, so it lives here rather than being spelled out twice.
    """
    return (CAM_WIDTH
            / (2.0 * (CAM_ALT_M if alt_m is None else alt_m)
               * math.tan(CAM_HFOV_RAD / 2.0)))


def deployment_box_fraction(class_name):
    """Fraction of the frame width the annotation box spans in flight.

    This is the number the fine-tuning set is scaled to match.  Measured raw
    off the photographs the box covers 0.177 of the frame, i.e. 0.76x of what
    the aircraft sees; a detector trained on that alone is tuned to the wrong
    apparent size, which is what --fit-box-fraction exists to correct.
    """
    return artwork_box(class_name)[0] * frame_width_px_per_m() / CAM_WIDTH


def label_map_pbtxt():
    """The label_map.pbtxt body the Object Detection API expects."""
    lines = []
    for index, name in enumerate(CLASS_NAMES, start=1):
        lines.append("item {")
        lines.append("  id: %d" % index)
        lines.append("  name: '%s'" % name)
        lines.append("}")
    return "\n".join(lines) + "\n"


def ssd_ros_labels_txt():
    """ssd_ros/data/ssd_coco_labels.txt uses one name per line, id = line no.

    Line 0 is background and is expected by the C++ reader, so it must not be
    the first class name."""
    lines = ["background"] + list(CLASS_NAMES)
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--label-map", action="store_true",
                    help="print the label_map.pbtxt body and exit")
    ap.add_argument("--ssd-ros-labels", action="store_true",
                    help="print ssd_ros/data/<task>_labels.txt (line 0 is "
                         "background) and exit")
    ap.add_argument("--check-alias", metavar="NAME",
                    help="show what an annotation label would resolve to")
    args = ap.parse_args()

    if args.label_map:
        sys.stdout.write(label_map_pbtxt())
        raise SystemExit(0)
    if args.ssd_ros_labels:
        sys.stdout.write(ssd_ros_labels_txt())
        raise SystemExit(0)
    if args.check_alias is not None:
        resolved = resolve_class(args.check_alias)
        print("%r -> %r" % (args.check_alias, resolved))
        raise SystemExit(0 if resolved else 1)

    print("classes (%d, ids start at 1):" % NUM_CLASSES)
    for index, (name, texture, zone) in enumerate(CLASSES, start=1):
        exists = "ok" if os.path.exists(texture_path(name)) else "MISSING"
        print("  %d  %-26s %-7s %-30s %s" % (index, name, zone, texture, exists))
    print()
    print("aliases: %d (%s, ...)"
          % (len(CLASS_ALIASES), ", ".join(list(CLASS_ALIASES)[:3])))
    for zone in sorted(FACE_ARTWORK):
        w, h, _, _ = artwork_box(
            [n for n, _, z in CLASSES if z == zone][0])
        print("box extent : %-6s %.3f x %.3f m on the %.3f m face "
              "(%d-gon; frame outer %.3f m)"
              % (zone, w, h, CARD_FACE_M, len(FACE_ARTWORK[zone]),
                 CARD_FRAME_M))
    print("real dataset default: %s%s"
          % (REAL_DATASET_DIR,
             "" if os.path.isdir(REAL_DATASET_DIR) else "  (not present)"))
    print()
    print(label_map_pbtxt())
