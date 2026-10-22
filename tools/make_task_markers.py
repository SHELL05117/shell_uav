#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build the task-zone marker textures for gc_2026_field.world.

The hazard signs are NOT drawn by hand.  They are the official pictograms out
of GB 2894-2025 附录C 表C.1 警告标志, extracted from the standard itself:

    1-02 当心火灾      Warning fire            -> 易燃
    1-03 当心爆炸      Warning explosive       -> 易爆
    1-04 当心腐蚀      Warning corrosive substance
    1-05 当心中毒      Warning toxic material  -> 有毒有害
    1-06 当心生物危害  Warning biological hazard
    1-27 当心电离辐射  Warning ionizing radiation
    1-28 当心裂变物质  Warning fissionable material
    1-42 当心氧化物    Warning oxidizing substance
    1-43 当心高压气瓶  Warning gas cylinder

Table C.1 holds 1-01 through 1-52; the nine above are the ones the task uses.
1-27 and 1-28 are the one easy pair to mix up: 1-27 is the plain trefoil,
1-28 is the trefoil with an arc underneath it.  Compare against
tools/assets/gb2894/reference/ rather than from memory.

Each entry carries the (page index, image xref) it lives at in the PDF.  The
xref is the identity that matters - page indices shift if the extract changes,
and the page text numbers (1-02 etc.) sit in a different column from the
artwork, so a number-to-page table alone would silently pull the wrong
triangle.  find_xrefs.py in this directory re-derives the table.

Source PDF (searched for automatically, override with --pdf):
    ~/Desktop/页面提取自－GB+2894-2025.pdf     (extracted pages)
    ~/Downloads/GB+2894-2025.pdf              (full standard)

The extracted pictograms are cached in tools/assets/gb2894/ so the textures
can be rebuilt later without the PDF.  Delete that directory to force a
re-extract.

The 人员救助任务区 card IS drawn here - there is no pictogram for it in the
standard, and the field drawing just shows a framed portrait photo.

Output: src/gc_2026_sim/gazebo/models/gc_task_markers/
          materials/scripts/gc_task_markers.material
          materials/textures/*.png
          model.config, model.sdf

Colour contract with the on-board detector (navigation_2d/src/mission_vision.cpp):
  hazard  -> cv::inRange(hsv, Scalar(15, 70, 70),  Scalar(40, 255, 255))   yellow
  person  -> cv::inRange(hsv, Scalar(75, 55, 70),  Scalar(125, 255, 255))  blue
Both use RETR_EXTERNAL, so the black pictogram and the black triangle border
are *holes* inside the yellow and do not split the blob, and the blue frame
around the portrait card is a ring whose external contour is the full square.
Every pictogram is snapped to a 3-colour GB 2894 palette below, so the yellow
is exactly the safety yellow the mask is tuned for and the JPEG mosquito noise
in the extracted artwork cannot drag it out of the window.

All nine warning signs are the same GB 2894 triangle, so they all land on the
same artwork bbox and labels.FACE_ARTWORK can describe them with one shared
outline.  Only the symbol inside the triangle differs.  If a future sign is
not a plain triangle, that shortcut breaks - check_box_convention.py measures
every texture individually and will say so.

Usage:
    python3 tools/make_task_markers.py [--pdf PATH]
"""

import argparse
import glob
import io
import math
import os
import shutil
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(
    ROOT, "src", "gc_2026_sim", "gazebo", "models", "gc_task_markers")
TEX_DIR = os.path.join(MODEL_DIR, "materials", "textures")
SCRIPT_DIR = os.path.join(MODEL_DIR, "materials", "scripts")
ASSET_DIR = os.path.join(ROOT, "tools", "assets", "gb2894")

# 512 px for a 0.40 m card face -> 1280 px/m.  SS is the supersampling factor;
# artwork is drawn/composited at 512*SS and box-filtered down, which is what
# keeps the triangle edges clean at Gazebo's viewing distance.
PX = 512
SS = 4

# --- palette (GB 2894 safety colours + the neutrals the cards use) ---------
BLACK = (16, 16, 16)
WHITE = (250, 250, 250)
SAFETY_YELLOW = (250, 205, 20)      # GB 2894 安全色 黄
CARD_GREY = (168, 172, 176)         # thin inner square outline on the card
SKIN = (245, 214, 180)
SKIN_SHADE = (226, 188, 152)
HAIR = (38, 30, 28)
SHIRT = (36, 96, 200)
MOUTH = (176, 84, 74)

# name, material, GB 2894 编号, 名称, (page index, xref) in the source PDF
#
# xref, not page, is the identity: doc.extract_image() takes an xref directly,
# and the page index is only what find_gb2894_xrefs.py narrows the search with.
# Order matters only for readability here - labels.py decides the class ids.
SIGNS = [
    ("gc_sign_flammable", "GC/Sign/Flammable", "1-02", "当心火灾 / 易燃", (1, 4)),
    ("gc_sign_explosive", "GC/Sign/Explosive", "1-03", "当心爆炸 / 易爆", (1, 5)),
    ("gc_sign_corrosive", "GC/Sign/Corrosive", "1-04", "当心腐蚀", (2, 8)),
    ("gc_sign_toxic", "GC/Sign/Toxic", "1-05", "当心中毒 / 有毒有害", (2, 9)),
    ("gc_sign_biohazard", "GC/Sign/Biohazard", "1-06", "当心生物危害", (2, 10)),
    ("gc_sign_ionizing_radiation", "GC/Sign/IonizingRadiation", "1-27",
     "当心电离辐射", (7, 41)),
    ("gc_sign_fissionable", "GC/Sign/Fissionable", "1-28", "当心裂变物质", (8, 44)),
    ("gc_sign_oxidizer", "GC/Sign/Oxidizer", "1-42", "当心氧化物", (11, 64)),
    ("gc_sign_gas_cylinder", "GC/Sign/GasCylinder", "1-43", "当心高压气瓶", (11, 65)),
]

MATERIAL_TEMPLATE = """// Generated by tools/make_task_markers.py - do not hand-edit.
//
// The pictograms are the official GB 2894-2025 附录C 表C.1 artwork, snapped to
// the three safety colours, so the yellow stays inside the HSV window that
// mission_vision.cpp matches (H 15-40, S>=70, V>=70).  ambient/diffuse are
// left at white on purpose: tinting them here would drag the safety yellow
// out of that window.
material {name}
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0.06 0.06 0.06 1 8
      texture_unit
      {{
        texture {tex}
        filtering anisotropic
        max_anisotropy 8
      }}
    }}
  }}
}}
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>gc_task_markers</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>gc_2026_sim</name>
    <email>expload233@gmail.com</email>
  </author>
  <description>
    Texture + OGRE material resources for the two task-zone markers
    (危险品任务区 / 人员救助任务区).  Hazard pictograms are GB 2894-2025
    附录C 表C.1.  This model is a resource container: the markers themselves
    are instantiated inline in gc_2026_field.world so that
    tools/check_config.py can still read them out of the world file.
  </description>
</model>
"""

MODEL_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <!-- Resource container only.  Nothing includes this model; the world file
       points its <material><script><uri> at this directory.  The single link
       exists so the directory is still a loadable Gazebo model (handy for
       `gz model -m` sanity checks). -->
  <model name="gc_task_markers">
    <static>true</static>
    <link name="marker_link">
      <visual name="marker_face">
        <pose>0 0 0.004 0 0 0</pose>
        <geometry><box><size>0.40 0.40 0.008</size></box></geometry>
        <material>
          <script>
            <uri>model://gc_task_markers/materials/scripts</uri>
            <uri>model://gc_task_markers/materials/textures</uri>
            <name>GC/Sign/Flammable</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


# ===========================================================================
#  1. pull the official pictograms out of GB 2894-2025
# ===========================================================================

def find_pdf(explicit):
    if explicit:
        if os.path.exists(explicit):
            return explicit
        sys.exit("--pdf %s not found" % explicit)
    home = os.path.expanduser("~")
    cands = [
        os.path.join(home, "Desktop", "页面提取自－GB+2894-2025.pdf"),
        os.path.join(home, "Downloads", "GB+2894-2025.pdf"),
    ]
    cands += sorted(glob.glob(os.path.join(home, "Desktop", "*2894-2025*.pdf")))
    cands += sorted(glob.glob(os.path.join(home, "Downloads", "*2894-2025*.pdf")))
    for c in cands:
        if os.path.exists(c):
            return c
    sys.exit("GB 2894-2025 PDF not found; pass --pdf PATH")


def extract_pictograms(pdf_path):
    import fitz  # PyMuPDF

    doc = fitz.open(pdf_path)
    os.makedirs(ASSET_DIR, exist_ok=True)
    for _name, _mat, code, _cn, (page, xref) in SIGNS:
        out = os.path.join(ASSET_DIR, "gb_%s.png" % code)
        info = doc.extract_image(xref)
        im = Image.open(io.BytesIO(info["image"])).convert("RGB")
        im.save(out)
        aspect = dark_bbox_aspect(im)
        flag = ""
        if aspect is None:
            flag = "   <-- blank! the xref does not point at artwork"
        elif abs(aspect - TRIANGLE_ASPECT) > 0.06:
            flag = ("   <-- w/h %.3f, expected ~%.3f for the warning triangle; "
                    "check the xref" % (aspect, TRIANGLE_ASPECT))
        print("  %s  %s  %dx%d  w/h %.3f%s"
              % (code, os.path.relpath(out, ROOT), im.width, im.height,
                 aspect if aspect else 0.0, flag))


# ===========================================================================
#  2. clean + normalise a pictogram
# ===========================================================================

PALETTE = np.array([WHITE, SAFETY_YELLOW, BLACK], dtype=np.float32)

# An equilateral triangle's bounding box is s x s*sqrt(3)/2, so w/h = 2/sqrt(3).
# Every GB 2894 warning sign is that same triangle, which makes this a cheap
# guard on the xref table: a wrong xref does not fail, it quietly returns some
# other picture, and the only ones on these pages that are *not* triangles are
# the table headers and the odd non-warning sign.
TRIANGLE_ASPECT = 2.0 / math.sqrt(3.0)


def dark_bbox_aspect(im, bg=245):
    """w/h of the non-background content, or None if the tile is blank."""
    a = np.asarray(im.convert("L"))
    ys, xs = np.where(a < bg)
    if not len(xs):
        return None
    return ((xs.max() - xs.min() + 1)
            / float(ys.max() - ys.min() + 1))


def snap_to_palette(im):
    """Force every pixel to white / safety yellow / black.

    The artwork comes out of the standard as JPEG, so the yellow has visible
    mosquito noise around every black edge.  Snapping to the three GB 2894
    colours removes it and pins the yellow exactly where the detector's HSV
    window expects it.  Doing it at 2x and then box-filtering back down keeps
    the edges antialiased instead of stair-stepped.
    """
    big = im.resize((im.width * 2, im.height * 2), Image.LANCZOS)
    big = big.filter(ImageFilter.MedianFilter(5))
    a = np.asarray(big, dtype=np.float32)
    d = ((a[:, :, None, :] - PALETTE[None, None, :, :]) ** 2).sum(axis=3)
    idx = d.argmin(axis=2)
    out = PALETTE[idx].astype(np.uint8)
    return Image.fromarray(out, "RGB").resize(im.size, Image.LANCZOS)


def autocrop(im, bg=245):
    """Trim the white margin so every pictogram is scaled the same way."""
    a = np.asarray(im.convert("L"))
    mask = a < bg
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return im
    return im.crop((int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1))


def card_face_with_pictogram(pict):
    """White 0.40 m card + thin grey square + the pictogram, 84% of the width."""
    im = Image.new("RGB", (PX * SS, PX * SS), WHITE)
    d = ImageDraw.Draw(im)

    m = 0.045 * PX * SS
    d.rectangle([m, m, PX * SS - m, PX * SS - m],
                outline=CARD_GREY, width=max(1, int(0.006 * PX * SS)))

    target_w = int(0.84 * PX * SS)
    scale = target_w / float(pict.width)
    pict = pict.resize((target_w, max(1, int(round(pict.height * scale)))), Image.LANCZOS)
    im.paste(pict, ((PX * SS - pict.width) // 2, (PX * SS - pict.height) // 2))
    return im


# ===========================================================================
#  3. the 人员救助任务区 portrait card - drawn here, no standard pictogram
# ===========================================================================

def card_backdrop(d):
    m = 0.045 * PX * SS
    d.rectangle([m, m, PX * SS - m, PX * SS - m],
                outline=CARD_GREY, width=max(1, int(0.006 * PX * SS)))


def draw_person_card():
    im = Image.new("RGB", (PX * SS, PX * SS), WHITE)
    d = ImageDraw.Draw(im)
    card_backdrop(d)

    # inner grey portrait box, like the framed photo on the field drawing
    a = 0.21 * PX * SS
    b = 0.79 * PX * SS
    d.rectangle([a, a, b, b], outline=CARD_GREY, width=max(1, int(0.006 * PX * SS)))

    # shoulders / shirt
    d.polygon([
        (0.20 * PX * SS, 0.79 * PX * SS),
        (0.25 * PX * SS, 0.690 * PX * SS),
        (0.37 * PX * SS, 0.640 * PX * SS),
        (0.63 * PX * SS, 0.640 * PX * SS),
        (0.75 * PX * SS, 0.690 * PX * SS),
        (0.80 * PX * SS, 0.79 * PX * SS),
    ], fill=SHIRT)
    d.polygon([
        (0.425 * PX * SS, 0.640 * PX * SS), (0.50 * PX * SS, 0.700 * PX * SS),
        (0.575 * PX * SS, 0.640 * PX * SS),
    ], fill=WHITE)

    d.rectangle([0.450 * PX * SS, 0.570 * PX * SS, 0.550 * PX * SS, 0.660 * PX * SS],
                fill=SKIN_SHADE)

    for u in (0.232, 0.748):
        d.ellipse([u * PX * SS, 0.355 * PX * SS,
                   (u + 0.070) * PX * SS, 0.460 * PX * SS], fill=SKIN)

    d.ellipse([0.268 * PX * SS, 0.195 * PX * SS,
               0.732 * PX * SS, 0.625 * PX * SS], fill=SKIN)

    # hair: cap + sideburns + fringe
    d.pieslice([0.258 * PX * SS, 0.165 * PX * SS,
                0.742 * PX * SS, 0.555 * PX * SS], 180, 360, fill=HAIR)
    d.ellipse([0.670 * PX * SS, 0.278 * PX * SS,
               0.748 * PX * SS, 0.500 * PX * SS], fill=HAIR)
    d.ellipse([0.252 * PX * SS, 0.278 * PX * SS,
               0.330 * PX * SS, 0.500 * PX * SS], fill=HAIR)
    d.polygon([
        (0.258 * PX * SS, 0.255 * PX * SS),
        (0.40 * PX * SS, 0.212 * PX * SS),
        (0.60 * PX * SS, 0.220 * PX * SS),
        (0.742 * PX * SS, 0.262 * PX * SS),
        (0.60 * PX * SS, 0.292 * PX * SS),
        (0.40 * PX * SS, 0.284 * PX * SS),
    ], fill=HAIR)

    for (u0, u1) in ((0.338, 0.458), (0.542, 0.662)):
        d.line([(u0 * PX * SS, 0.360 * PX * SS), (u1 * PX * SS, 0.344 * PX * SS)],
               fill=HAIR, width=max(1, int(0.017 * PX * SS)))

    for u in (0.346, 0.544):
        d.ellipse([u * PX * SS, 0.380 * PX * SS,
                   (u + 0.110) * PX * SS, 0.448 * PX * SS], fill=WHITE)
        d.ellipse([(u + 0.030) * PX * SS, 0.394 * PX * SS,
                   (u + 0.080) * PX * SS, 0.444 * PX * SS], fill=(62, 46, 40))
        d.ellipse([(u + 0.042) * PX * SS, 0.401 * PX * SS,
                   (u + 0.062) * PX * SS, 0.421 * PX * SS], fill=BLACK)

    d.polygon([(0.494 * PX * SS, 0.462 * PX * SS),
               (0.472 * PX * SS, 0.520 * PX * SS),
               (0.524 * PX * SS, 0.520 * PX * SS)], fill=SKIN_SHADE)
    d.arc([0.408 * PX * SS, 0.462 * PX * SS, 0.592 * PX * SS, 0.590 * PX * SS],
          15, 165, fill=MOUTH, width=max(1, int(0.013 * PX * SS)))

    return im


# ===========================================================================

def save(im, name):
    im = im.resize((PX, PX), Image.LANCZOS)
    path = os.path.join(TEX_DIR, name)
    im.save(path)
    print("  wrote", os.path.relpath(path, ROOT))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pdf", default=None, help="path to GB 2894-2025 (PDF)")
    args = ap.parse_args()

    missing = [c for _n, _m, c, _cn, _x in SIGNS
               if not os.path.exists(os.path.join(ASSET_DIR, "gb_%s.png" % c))]
    if missing:
        pdf = find_pdf(args.pdf)
        print("extracting GB 2894-2025 pictograms from", pdf)
        extract_pictograms(pdf)
    else:
        print("using cached pictograms in", os.path.relpath(ASSET_DIR, ROOT))

    for d in (TEX_DIR, SCRIPT_DIR):
        if os.path.isdir(d):
            shutil.rmtree(d)
        os.makedirs(d)

    print("composing card faces ...")
    for name, _mat, code, cn, _x in SIGNS:
        pict = Image.open(os.path.join(ASSET_DIR, "gb_%s.png" % code)).convert("RGB")
        pict = snap_to_palette(autocrop(pict))
        save(card_face_with_pictogram(pict), name + ".png")
        print("     GB 2894 %s  %s" % (code, cn))
    save(draw_person_card(), "gc_sign_person.png")

    print("writing material script ...")
    body = "".join(
        MATERIAL_TEMPLATE.format(name=mat, tex=name + ".png")
        for name, mat, _c, _cn, _x in SIGNS)
    body += MATERIAL_TEMPLATE.format(name="GC/Sign/Person", tex="gc_sign_person.png")
    with open(os.path.join(SCRIPT_DIR, "gc_task_markers.material"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(body)

    with open(os.path.join(MODEL_DIR, "model.config"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(MODEL_CONFIG)
    with open(os.path.join(MODEL_DIR, "model.sdf"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write(MODEL_SDF)

    print("done ->", os.path.relpath(MODEL_DIR, ROOT))


if __name__ == "__main__":
    main()
