#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reconcile gc_2026_field.world + navigation_2d.launch against the field mat
drawing (图纸.png).

Where the DRAWING column comes from
-----------------------------------
Two independent sources, and the script says which one it used for each row:

  callout   a dimension printed on the drawing.  Exact; tolerance 5 mm.
  raster    measured off the drawing by colour segmentation:
            mat frame at px x[161,725] y[78,641] -> 564 px per 4000 mm
            (0.14100 px/mm ~ 7.09 mm/px), so a raster row carries ~25 mm of
            measurement error; tolerance 35 mm.

Everything is in mat millimetres.  Mat -> gazebo is
    gazebo_x = mat_x/1000 - 0.65      gazebo_y = mat_y/1000 - 3.35
and mat y is measured UP from the bottom edge of the printed mat, the same way
the drawing's callouts read.

Usage:
    python3 tools/check_field_vs_drawing.py
    python3 tools/check_field_vs_drawing.py -v      # show the derivation
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_field import parse_world, parse_routes, LAUNCH, WORLD  # noqa: E402

OK = "PASS"
SOFT = "note"
BAD = "FAIL"

# name, drawing value, source, tolerance, how to read it off the world
ROWS = [
    # --- the mat itself -------------------------------------------------
    ("field size", (4000, 4000, 4000, 4000), "callout", 5, "mat"),
    # --- start pad: drawing "650" off the left edge, "650" off the top edge
    ("起飞点 centre", (650, 650, 3350, 3350), "callout", 5, "takeoff_pad"),
    # --- fixed obstacles -------------------------------------------------
    ("固定障碍物1 (left face .. right face, bottom .. top)",
     (1300, 1400, 2300, 4000), "callout", 5, "fixed_obstacle_1"),
    ("固定障碍物1 bar length (drawing 1700)",
     (1700, 1700, 1700, 1700), "callout", 5, "fixed_obstacle_1_len"),
    ("固定障碍物2 centre",
     (3404, 3404, 3397, 3397), "raster", 35, "fixed_obstacle_2"),
    ("固定障碍物2 bbox (1600 @45 deg -> 1202 mm)",
     (1177, 1177, 1177, 1177), "raster", 45, "fixed_obstacle_2_bbox"),
    ("固定障碍物3",
     (1305, 1397, 21, 504), "raster", 35, "fixed_obstacle_3"),
    ("固定障碍物4",
     (2709, 2794, 21, 504), "raster", 35, "fixed_obstacle_4"),
    # --- random obstacles: the mat marks two candidate spots per column ----
    ("随机障碍物1 (column x, option 1 y)",
     (1350, 1350, 1900, 1900), "raster", 35, "random_obstacle_1"),
    ("随机障碍物2 (column x, option 1 y)",
     (2750, 2750, 1900, 1900), "raster", 35, "random_obstacle_2"),
    # --- task zones ------------------------------------------------------
    ("危险品任务区 #1 centre (drawing 1950 off the right edge)",
     (2053, 2053, 3351, 3351), "raster", 35, "hazard_card_1"),
    ("危险品任务区 #2 centre",
     (2053, 2053, 2897, 2897), "raster", 35, "hazard_card_2"),
    ("危险品任务区 card pitch", (454, 454, 454, 454), "raster", 35, "hazard_pitch"),
    ("人员救助任务区 #1 centre (drawing 650 off the right edge)",
     (3353, 3353, 2270, 2270), "raster", 35, "person_card_1"),
    ("人员救助任务区 #2 centre",
     (3355, 3355, 1450, 1450), "raster", 45, "person_card_2"),
    ("人员救助任务区 #3 centre (drawing 500 to the frame bottom)",
     (3351, 3351, 690, 690), "raster", 35, "person_card_3"),
    # --- routes (odom numbers, converted to mat mm for comparison) --------
    ("hazard_route 终点 = 危险品任务区列",
     (2050, 2050, 3350, 3350), "raster", 35, "route_hazard"),
    # The person tour must call at ALL three cards before it hands over to
    # return_route; it used to stop at card #1, which is why these are three
    # separate coverage rows and not one "route end" row.  The world value is
    # the nearest person_route waypoint to that card's centre, so a route that
    # skips a card shows up as a FAIL on that card's row.
    ("person_route 经过 人员救助任务区 #1",
     (3353, 3353, 2270, 2270), "raster", 35, "route_person_1"),
    ("person_route 经过 人员救助任务区 #2",
     (3355, 3355, 1450, 1450), "raster", 45, "route_person_2"),
    ("person_route 经过 人员救助任务区 #3",
     (3351, 3351, 690, 690), "raster", 35, "route_person_3"),
    ("return_route 终点 = 起飞点",
     (650, 650, 3350, 3350), "callout", 5, "route_return"),
]


def mat_of(x, y):
    """gazebo metres -> mat millimetres"""
    return (x + 0.65) * 1000.0, (y + 3.35) * 1000.0


def model_box(visuals, name):
    """world/model xy extent of every visual belonging to `name`"""
    xs, ys, zs = [], [], []
    for v in visuals:
        if v["model"] != name:
            continue
        x, y, z, _r, _p, yaw = v["pose"]
        if v["kind"] == "cyl":
            r = v["size"][0]
            xs += [x - r, x + r]
            ys += [y - r, y + r]
        else:
            sx, sy = v["size"][0], v["size"][1]
            import math
            for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2),
                           (sx / 2, sy / 2), (-sx / 2, sy / 2)):
                xs.append(x + dx * math.cos(yaw) - dy * math.sin(yaw))
                ys.append(y + dx * math.sin(yaw) + dy * math.cos(yaw))
        zs += [z, z + (v["size"][1] if v["kind"] == "cyl" else v["size"][2])]
    if not xs:
        return None
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def model_centre(visuals, name):
    b = model_box(visuals, name)
    if b is None:
        return None
    x0, x1, y0, y1 = b[:4]
    mx0, my0 = mat_of(x0, y0)
    mx1, my1 = mat_of(x1, y1)
    return ((mx0 + mx1) / 2.0, (mx0 + mx1) / 2.0, (my0 + my1) / 2.0, (my0 + my1) / 2.0)


def measure(visuals, what):
    if what == "mat":
        b = model_box(visuals, "competition_floor")
        x0, x1, y0, y1 = b[:4]
        return (abs(x1 - x0) * 1000,) * 4
    if what == "takeoff_pad":
        return model_centre(visuals, "takeoff_pad")
    if what == "fixed_obstacle_2":
        return model_centre(visuals, "fixed_obstacle_2")
    if what == "fixed_obstacle_2_bbox":
        b = model_box(visuals, "fixed_obstacle_2")
        return ((b[1] - b[0]) * 1000.0,) * 4
    if what == "fixed_obstacle_1_len":
        b = model_box(visuals, "fixed_obstacle_1")
        v = (b[3] - b[2]) * 1000.0
        return (v,) * 4
    if what.startswith("fixed_obstacle_"):
        b = model_box(visuals, what)
        x0, x1, y0, y1 = b[:4]
        mx0, mx1 = mat_of(x0, 0)[0], mat_of(x1, 0)[0]
        my0, my1 = mat_of(0, y0)[1], mat_of(0, y1)[1]
        return (mx0, mx1, my0, my1)
    if what in ("random_obstacle_1", "random_obstacle_2"):
        c = model_centre(visuals, what)
        return c
    if what in ("hazard_card_1", "hazard_card_2"):
        return model_centre(visuals, "hazard_zone_card_" + what[-1])
    if what == "hazard_pitch":
        a = model_centre(visuals, "hazard_zone_card_1")
        b = model_centre(visuals, "hazard_zone_card_2")
        v = abs(a[2] - b[2])
        return (v,) * 4
    if what.startswith("person_card_"):
        return model_centre(visuals, "person_zone_card_" + what[-1])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    visuals = parse_world(WORLD)
    routes = parse_routes(LAUNCH)

    def route_end(name):
        p = routes.get(name)
        if not p:
            return None
        x, y = p[-1]["x"], p[-1]["y"]
        mx, my = mat_of(x, y)
        return (mx, mx, my, my)

    def route_nearest(name, target):
        """mat mm of the route waypoint closest to a world-metre target point"""
        p = routes.get(name)
        if not p:
            return None
        tx, ty = target
        best = min(p, key=lambda q: (q["x"] - tx) ** 2 + (q["y"] - ty) ** 2)
        mx, my = mat_of(best["x"], best["y"])
        return (mx, mx, my, my)

    def card_centre_gazebo(model):
        b = model_box(visuals, model)
        if b is None:
            return None
        return ((b[0] + b[1]) / 2.0, (b[2] + b[3]) / 2.0)

    fails = 0
    notes = 0
    print("mat mm  (gazebo_x = mat_x/1000 - 0.65,  gazebo_y = mat_y/1000 - 3.35)")
    print("%-52s %-8s %-26s %-26s %s" %
          ("item", "source", "drawing", "world", "verdict"))
    print("-" * 132)
    for name, want, src, tol, what in ROWS:
        if what == "route_hazard":
            got = route_end("hazard_route")
        elif what.startswith("route_person_"):
            idx = what[-1]
            tgt = card_centre_gazebo("person_zone_card_" + idx)
            got = None if tgt is None else route_nearest("person_route", tgt)
        elif what == "route_return":
            got = route_end("return_route")
        else:
            got = measure(visuals, what)
        if got is None:
            print("%-52s %-8s %-26s %-26s %s" % (name, src, fmt(want), "-", "MISSING"))
            fails += 1
            continue
        dmax = max(abs(a - b) for a, b in zip(want, got))
        verdict = OK if dmax <= tol else (SOFT if dmax <= 4 * tol else BAD)
        if verdict == BAD:
            fails += 1
        elif verdict == SOFT:
            notes += 1
        print("%-52s %-8s %-26s %-26s %-4s max|d|=%.0f mm (tol %d)"
              % (name, src, fmt(want), fmt(got), verdict, dmax, tol))
        if args.verbose:
            print("      derivation: %s" % what)

    print("-" * 132)
    print("%d FAIL, %d note" % (fails, notes))
    if fails:
        print("FAIL rows mean the world no longer matches the drawing - fix the "
              "world AND the route together, never one of the pair.")
    return 1 if fails else 0


def fmt(v):
    return "[" + ", ".join("%.0f" % x for x in v) + "]"


if __name__ == "__main__":
    sys.exit(main())
