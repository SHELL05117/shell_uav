#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render gc_2026_field.world to a plan view, and self-check the task markers
against the on-board detector.

Two jobs:

  1. PLAN VIEW.  Parse the world (and navigation_2d.launch's routes) and draw
     what Gazebo will actually put on the mat, top-down, with the field
     drawing's dimension chain annotated over it.  This is the picture to hold
     next to 图纸.png when checking geometry and waypoints.

  2. FRAMING SELF-CHECK (--check-vision).  Render the field at the true nadir
     scale (640x480, hfov from the sensor rig, camera 0.15 m under the airframe
     at task height) and measure, for each task card, how far its painted extent
     sits from the frame edge.  MissionVision rejects any detection whose box
     touches the edge - a clipped box's centre is biased toward the image
     centre, so the correction it produces is always too small - which means a
     card that overflows the frame can never be aligned to.  This says so before
     a flight does.  (Before 2026-10-17 this mode re-implemented the deleted HSV
     colour windows instead; there is no colour rule left to mirror.)

Usage:
    python3 tools/render_field.py
    python3 tools/render_field.py --check-vision
    python3 tools/render_field.py --out tools/_crops/plan.png
"""

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(ROOT, "src/gc_2026_sim/gazebo/worlds/gc_2026_field.world")
MARKERS = os.path.join(
    ROOT, "src/gc_2026_sim/gazebo/models/gc_task_markers")
LAUNCH = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")

# mat <-> gazebo:  mat_x/1000 - 0.65 = gazebo x,  mat_y/1000 - 3.35 = gazebo y
MAT_OX, MAT_OY = 0.65, 3.35

PX_PER_M = 250.0
PAD = 0.55                      # metres of margin around the field
FLOOR = (238, 238, 236)
WALL = (74, 82, 92)
DIM = (120, 60, 170)


# ===========================================================================
#  world parsing
# ===========================================================================

def _f(txt, n, default=0.0):
    if not txt:
        return [default] * n
    v = [float(x) for x in txt.split()]
    while len(v) < n:
        v.append(0.0)
    return v[:n]


def parse_materials(path):
    """material name -> texture file name (relative to materials/textures)."""
    out = {}
    if not os.path.exists(path):
        return out
    txt = open(path, encoding="utf-8").read()
    for m in re.finditer(r"material\s+(\S+)\s*\{(.*?)\n\}", txt, re.S):
        name, body = m.group(1), m.group(2)
        t = re.search(r"texture\s+(\S+)", body)
        if t:
            out[name] = t.group(1)
    return out


def parse_world(path):
    """-> list of visuals, each dict(model, pose6, kind, size/radius, material)."""
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        world = root
    visuals = []
    for model in world.findall("model"):
        mname = model.get("name")
        mpose = _f(model.findtext("pose"), 6)
        for link in model.findall("link"):
            lpose = _f(link.findtext("pose"), 6)
            for vis in link.findall("visual"):
                vpose = _f(vis.findtext("pose"), 6)
                geo = vis.find("geometry")
                if geo is None:
                    continue
                box = geo.find("box/size")
                cyl = geo.find("cylinder")
                if box is not None:
                    kind, size = "box", _f(box.text, 3)
                elif cyl is not None:
                    kind = "cyl"
                    size = [float(cyl.findtext("radius")), float(cyl.findtext("length"))]
                else:
                    continue
                mat = vis.find("material")
                tex, colour = None, (200, 200, 200)
                if mat is not None:
                    nm = mat.findtext("script/name")
                    if nm:
                        tex = nm
                    amb = mat.findtext("ambient")
                    dif = mat.findtext("diffuse")
                    if dif:
                        c = _f(dif, 4, 1.0)
                        colour = tuple(int(255 * min(1.0, x)) for x in c[:3])
                    elif amb:
                        c = _f(amb, 4, 1.0)
                        colour = tuple(int(255 * min(1.0, x)) for x in c[:3])
                visuals.append(dict(model=mname, pose=add_pose(mpose, lpose, vpose),
                                    kind=kind, size=size, tex=tex, colour=colour))
    return visuals


def add_pose(a, b, c):
    """Compose three poses.  Every pose here is yaw-only or pitch-free in xy,
    so an exact 2D composition is enough for a plan view."""
    out = [a[i] + b[i] + c[i] for i in range(6)]
    return out


# ===========================================================================
#  plan rendering
# ===========================================================================

def world_to_px(x, y, x0, y1, s):
    return ((x - x0) * s, (y1 - y) * s)


def draw_visual(d, v, x0, y1, s, tex_cache):
    x, y, z, _r, _p, yaw = v["pose"]
    if v["kind"] == "cyl":
        r = v["size"][0] * s
        cx, cy = world_to_px(x, y, x0, y1, s)
        col = v["colour"]
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col, outline=(90, 90, 90))
        return
    sx, sy = v["size"][0], v["size"][1]
    pts = []
    for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)):
        rx = dx * math.cos(yaw) - dy * math.sin(yaw)
        ry = dx * math.sin(yaw) + dy * math.cos(yaw)
        pts.append(world_to_px(x + rx, y + ry, x0, y1, s))
    if v["tex"]:
        img = tex_cache.get(v["tex"])
        if img is not None:
            w = max(1, int(round(sx * s)))
            h = max(1, int(round(sy * s)))
            tile = img.resize((w, h), Image.LANCZOS)
            if abs(yaw) < 1e-6:
                ox = min(p[0] for p in pts)
                oy = min(p[1] for p in pts)
                d._image.paste(tile, (int(round(ox)), int(round(oy))))
            else:
                tile = tile.rotate(-math.degrees(yaw), resample=Image.BICUBIC,
                                   expand=True)
                cx = sum(p[0] for p in pts) / 4.0
                cy = sum(p[1] for p in pts) / 4.0
                d._image.paste(tile, (int(cx - tile.width / 2),
                                      int(cy - tile.height / 2)))
            return
    d.polygon(pts, fill=v["colour"], outline=(70, 70, 70))


def render_plan(px_per_m=PX_PER_M):
    mats = parse_materials(os.path.join(MARKERS, "materials/scripts/gc_task_markers.material"))
    visuals = parse_world(WORLD)

    tex_cache = {}
    for name, fn in mats.items():
        p = os.path.join(MARKERS, "materials/textures", fn)
        if os.path.exists(p):
            tex_cache[name] = Image.open(p).convert("RGB")

    xs = [-0.65 - PAD, 3.35 + PAD]
    ys = [-3.35 - PAD, 0.65 + PAD]
    x0, x1 = xs
    y0, y1 = ys
    W = int(round((x1 - x0) * px_per_m))
    H = int(round((y1 - y0) * px_per_m))

    im = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(im)
    d._image = im

    # sort back to front so the walls do not erase the mat
    visuals.sort(key=lambda v: v["pose"][2])
    for v in visuals:
        draw_visual(d, v, x0, y1, px_per_m, tex_cache)

    return im, (x0, y1, px_per_m), visuals


def annotate(im, origin):
    """Overlay the field drawing's dimension chain + the launch routes."""
    x0, y1, s = origin
    d = ImageDraw.Draw(im)
    try:
        f = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 17)
        fs = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 14)
    except Exception:
        f = fs = ImageFont.load_default()

    def P(mx, my):
        return world_to_px(mx / 1000.0 - MAT_OX, my / 1000.0 - MAT_OY, x0, y1, s)

    def dim(p0, p1, text, off=0, vert=False):
        a, b = P(*p0), P(*p1)
        if vert:
            a = (a[0] + off, a[1])
            b = (b[0] + off, b[1])
        else:
            a = (a[0], a[1] + off)
            b = (b[0], b[1] + off)
        d.line([a, b], fill=DIM, width=2)
        for p, q in ((a, b), (b, a)):
            d.line([p, (p[0] + (8 if q[0] > p[0] else -8), p[1] + (8 if q[1] > p[1] else -8))],
                   fill=DIM, width=2)
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        tw = d.textlength(text, font=f)
        d.rectangle([mx - tw / 2 - 4, my - 11, mx + tw / 2 + 4, my + 11], fill=(255, 255, 255))
        d.text((mx - tw / 2, my - 9), text, fill=DIM, font=f)

    # mat edge
    a, b = P(0, 0), P(4000, 4000)
    d.rectangle([min(a[0], b[0]), min(a[1], b[1]),
                 max(a[0], b[0]), max(a[1], b[1])], outline=(40, 40, 40), width=2)

    # ---- the field drawing's dimension chain, mat mm ----
    dim((0, 4000), (1300, 4000), "1300", off=-34)          # -> fixed obstacle 1
    dim((0, 4000), (650, 4000), "650", off=-62)            # -> takeoff pad
    dim((4000, 4000), (2050, 4000), "1950", off=-34)       # -> hazard column
    dim((3350, 4000), (4000, 4000), "650", off=-34)        # -> person column
    dim((0, 4000), (0, 3350), "650", off=-34, vert=True)   # takeoff row
    dim((0, 4000), (0, 0), "4000", off=-64, vert=True)
    dim((4000, 4000), (4000, 3350), "650", off=34, vert=True)
    dim((4000, 3500), (4000, 3000), "500", off=64, vert=True)
    dim((0, 0), (1300, 0), "1300", off=34)
    dim((1300, 0), (2700, 0), "1400", off=34)
    dim((2700, 0), (2800, 0), "100", off=34)
    dim((2800, 0), (4000, 0), "1200", off=34)
    dim((0, 0), (4000, 0), "4000", off=62)
    dim((2150 - 190, 2900), (2150 - 190, 3350), "454", off=0, vert=True)

    routes = parse_routes(LAUNCH)
    cols = {"hazard_route": (205, 40, 45), "person_route": (30, 110, 200),
            "return_route": (60, 140, 70)}
    for rname, pts in routes.items():
        col = cols.get(rname, (0, 0, 0))
        xy = [P(p["x"] * 1000 + 650, p["y"] * 1000 + 3350) for p in pts]
        for a2, b2 in zip(xy, xy[1:]):
            d.line([a2, b2], fill=col, width=3)
        for i, (px, py) in enumerate(xy):
            d.ellipse([px - 7, py - 7, px + 7, py + 7], fill=col,
                      outline=(255, 255, 255), width=2)
            d.text((px + 10, py - 20), "%s%d" % (rname.split("_")[0][:1].upper(), i),
                   fill=col, font=fs)

    # legend, dropped into the empty left-middle of the mat - the field drawing
    # puts its own labels there too, and the outer margins are all taken by the
    # dimension chain
    lx, ly = P(160, 3120)
    d.rectangle([lx - 10, ly - 10, lx + 268, ly + 104], fill=(255, 255, 255),
                outline=(120, 120, 120))
    d.text((lx, ly), "gc_2026 场地平面（单位 mm）", fill=(20, 20, 20), font=f)
    for i, (rn, col) in enumerate(cols.items()):
        d.line([(lx, ly + 28 + i * 20), (lx + 26, ly + 28 + i * 20)], fill=col, width=3)
        d.text((lx + 32, ly + 19 + i * 20), rn, fill=col, font=fs)
    return routes


def parse_routes(path):
    if not os.path.exists(path):
        return {}
    txt = open(path, encoding="utf-8").read()
    out = {}
    for m in re.finditer(r'param="(\w+_route)">(.*?)</rosparam>', txt, re.S):
        pts = []
        for p in re.finditer(r"\{x:\s*([-\d.]+),\s*y:\s*([-\d.]+)", m.group(2)):
            pts.append({"x": float(p.group(1)), "y": float(p.group(2))})
        out[m.group(1)] = pts
    return out


# ===========================================================================
#  framing self-check
# ===========================================================================

def nadir_view(px_per_m):
    """Re-render the field at true camera scale."""
    saved = globals()["PX_PER_M"]
    globals()["PX_PER_M"] = px_per_m
    try:
        im, origin, _ = render_plan(px_per_m)
    finally:
        globals()["PX_PER_M"] = saved
    return im, origin


def check_vision():
    """Each task marker must fit inside the camera frame at task height.

    Until 2026-10-17 this function re-implemented the HSV colour windows of
    mission_vision.cpp in numpy, so that "would this card still be detected"
    could be answered without a flight. Those windows are gone: the detector is
    a learned model now, and there is no colour rule left to mirror.

    What replaced them in the mission is a geometric rule that is worth checking
    statically, because it fails silently in exactly the same way: MissionVision
    drops any detection whose box touches the frame edge. A clipped box has its
    centre biased toward the image centre, so the correction it produces is
    always too small - and dropping it means the align never gets a target, the
    align times out and the mission aborts without ever dropping.

    So this renders the field at the true nadir scale (640x480, hfov from the
    sensor rig, camera 0.15 m under the airframe at task height) and asks, for
    each card: is the card's painted extent completely inside the frame, and by
    how many pixels. The measurement is done on the rendered image rather than
    from model dimensions, so it stays true if a card is resized.
    """
    rig = os.path.join(ROOT, "src/gc_2026_sim/gazebo/models/px4_sensor_rig/model.sdf")
    hfov = 1.20
    width, height = 640, 480
    if os.path.exists(rig):
        t = open(rig, encoding="utf-8").read()
        m = re.search(r"<horizontal_fov>([\d.]+)</horizontal_fov>", t)
        if m:
            hfov = float(m.group(1))
        m = re.search(r"<width>(\d+)</width>", t)
        if m:
            width = int(m.group(1))
        m = re.search(r"<height>(\d+)</height>", t)
        if m:
            height = int(m.group(1))
    altitude = 1.20 - 0.15          # task_height minus the camera's mount drop
    ppm = width / (2.0 * altitude * math.tan(hfov / 2.0))
    print("nadir view: %dx%d, hfov %.2f rad, %.0f px/m at task height (1.20 m)"
          % (width, height, hfov, ppm))
    print("  frame covers %.2f x %.2f m of ground"
          % (2.0 * altitude * math.tan(hfov / 2.0),
             2.0 * altitude * math.tan(hfov / 2.0) * height / float(width)))

    im, (x0, y1, s) = nadir_view(ppm)
    print("  rendered field at %.0f px/m (%dx%d)" % (s, im.width, im.height))
    rgb = np.asarray(im.convert("RGB")).astype(np.int16)
    floor = np.array(FLOOR, dtype=np.int16)

    def crop(centre_mm):
        wx = centre_mm[0] / 1000.0 - MAT_OX
        wy = centre_mm[1] / 1000.0 - MAT_OY
        cx, cy = world_to_px(wx, wy, x0, y1, s)
        return (rgb[max(0, int(cy - height / 2)):int(cy + height / 2),
                    max(0, int(cx - width / 2)):int(cx + width / 2)],
                int(cx - max(0, int(cx - width / 2))),
                int(cy - max(0, int(cy - height / 2))))

    def measure(centre_mm, label):
        frame, ccx, ccy = crop(centre_mm)
        if frame.size == 0:
            print("  %-24s FAIL  crop is empty (marker outside the rendered field)"
                  % label)
            return False
        painted = np.abs(frame - floor).sum(axis=2) > 30
        lab, _n = ndimage.label(painted, structure=np.ones((3, 3)))
        if not 0 <= ccx < lab.shape[1] or not 0 <= ccy < lab.shape[0]:
            print("  %-24s FAIL  frame centre fell outside the crop" % label)
            return False
        ident = lab[ccy, ccx]
        if ident == 0:
            # The centre pixel is floor - the card is not where the waypoint is,
            # which is the ONE thing the whole approach depends on.
            print("  %-24s FAIL  nothing painted at the frame centre" % label)
            return False
        rows, cols = np.where(lab == ident)
        top, bottom = int(rows.min()), int(rows.max())
        left, right = int(cols.min()), int(cols.max())
        margins = (left, top, lab.shape[1] - 1 - right, lab.shape[0] - 1 - bottom)
        span = "%dx%d px" % (right - left + 1, bottom - top + 1)
        if min(margins) <= 0:
            side = ("left", "top", "right", "bottom")[margins.index(min(margins))]
            print("  %-24s FAIL  %s touches the %s frame edge (margins %s); "
                  "MissionVision drops a clipped box, so this card could never "
                  "be aligned to" % (label, span, side, margins))
            return False
        print("  %-24s PASS  %s, smallest margin %d px (left/top/right/bottom %s)"
              % (label, span, min(margins), margins))
        return True

    ok = True
    for centre, label in (((2050, 3350), "危险品 #1 易燃 nadir"),
                          ((2050, 2900), "危险品 #2 易爆 nadir"),
                          ((3350, 2300), "人员救助 #1 nadir")):
        ok = measure(centre, label) and ok
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "field_plan.png"))
    ap.add_argument("--ppm", type=float, default=PX_PER_M)
    ap.add_argument("--check-vision", action="store_true")
    ap.add_argument("--no-annotate", action="store_true")
    args = ap.parse_args()

    if args.check_vision:
        print("=== task-card framing at task height ===")
        ok = check_vision()
        print("=== %s ===" % ("EVERY CARD FITS THE FRAME" if ok
                              else "A CARD WOULD BE REJECTED AS CLIPPED"))

    im, origin, _ = render_plan(args.ppm)
    if not args.no_annotate:
        annotate(im, origin)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    im.save(args.out)
    print("plan view -> %s  (%dx%d)" % (os.path.relpath(args.out, ROOT), im.width, im.height))


if __name__ == "__main__":
    main()
