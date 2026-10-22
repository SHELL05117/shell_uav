#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print where the launch-file route tables land in odom, for every field_yaw_deg.

This exists because the answer to "do the waypoints end up outside the wall?"
depends on a sign convention that is very easy to get backwards by hand, and the
consequence of getting it backwards is a flight test that proves nothing.

Convention, encoded once here and once in rescue_mission.cpp:

  field_yaw_deg (psi) = the direction the nose points, measured in the FIELD
      frame, in degrees.  0 = field +x, +90 = field +y, 180 = field -x,
      -90 = field -y.  "机头对着 -y" is therefore psi = -90.

  odom is laid down by cartographer at boot, so odom +x is the nose and odom
  origin is the pad.  A point whose FIELD coordinates are q therefore has ODOM
  coordinates

      odom(q) = R(-psi) * q            (R(t)(x,y) = (x cos t - y sin t, x sin t + y cos t))

  and the field's walls, which never move, occupy in odom the rectangle
  odom(field rectangle) -- i.e. the walls rotate too.

  What rescue_mission_node did before field_yaw_deg existed is publish q
  itself as an odom goal.  That is the "no fix" column below, and the gap
  between the two columns is the bug.

Run:  python3 tools/route_frame_report.py
"""

import math

# Field geometry, from the drawing (tools/check_field_vs_drawing.py asserts the
# same numbers against the world file).
FIELD_X = (-0.65, 3.35)
FIELD_Y = (-3.35, 0.65)

# safe_field_* in navigation_2d.launch, inset 0.05 m from the walls.
SAFE_X = (-0.60, 3.30)
SAFE_Y = (-3.30, 0.60)

# The route tables, copied from navigation_2d.launch.  FIELD-frame numbers.
ROUTES = [
    ("hazard_route", [(-0.25, -1.80), (1.40, -1.80), (1.40, 0.00)]),
    ("person_route", [(1.50, -0.90), (2.70, -0.90), (2.70, -1.05),
                      (2.70, -1.85), (2.70, -2.65)]),
    ("return_route", [(2.70, -2.00), (0.00, -2.00), (0.00, 0.00)]),
]


def rot(deg, x, y):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return (x * c - y * s, x * s + y * c)


def field_to_odom(psi_deg, x, y):
    """odom coordinates of a point given in the field frame."""
    return rot(-psi_deg, x, y)


def bbox(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def rect_in_odom(psi_deg, xr, yr):
    corners = [(xr[0], yr[0]), (xr[0], yr[1]), (xr[1], yr[0]), (xr[1], yr[1])]
    return bbox([field_to_odom(psi_deg, x, y) for x, y in corners])


def inside(p, box):
    return box[0] <= p[0] <= box[1] and box[2] <= p[1] <= box[3]


def fmt_box(b):
    return "x [%+.2f, %+.2f]  y [%+.2f, %+.2f]" % b


def main():
    print("Field rectangle, FIELD frame: x %s  y %s" % (FIELD_X, FIELD_Y))
    print("Pad = field origin = odom origin.  Pad is 0.65 m from two walls,")
    print("3.35 m from the other two -- it is a CORNER, not the centre.")
    print()

    for psi in (0.0, 90.0, 180.0, -90.0):
        walls = rect_in_odom(psi, FIELD_X, FIELD_Y)
        safe = rect_in_odom(psi, SAFE_X, SAFE_Y)
        print("=" * 74)
        print("field_yaw_deg = %+.0f  (nose along field %s)"
              % (psi, {0.0: "+x", 90.0: "+y", 180.0: "-x", -90.0: "-y"}[psi]))
        print("  walls  in odom: %s" % fmt_box(walls))
        print("  safe   in odom: %s" % fmt_box(safe))

        n_pts = 0
        n_fixed_bad = 0
        n_nofix_bad = 0
        for name, pts in ROUTES:
            print("  %s (field -> odom%s):" % (
                name, "" if abs(psi) < 1e-9 else "; correct vs 'no fix'"))
            for q in pts:
                n_pts += 1
                fixed = field_to_odom(psi, *q)
                nofix = q
                ok_f = inside(fixed, walls)
                ok_n = inside(nofix, walls)
                n_fixed_bad += 0 if ok_f else 1
                n_nofix_bad += 0 if ok_n else 1
                if abs(psi) < 1e-9:
                    print("    (%+.2f, %+.2f) -> (%+.2f, %+.2f)  %s"
                          % (q[0], q[1], fixed[0], fixed[1],
                             "ok" if ok_f else "OUTSIDE"))
                else:
                    print("    (%+.2f, %+.2f) -> (%+.2f, %+.2f) %s"
                          "   |  no fix (%+.2f, %+.2f) %s"
                          % (q[0], q[1], fixed[0], fixed[1],
                             "ok    " if ok_f else "OUTSIDE",
                             nofix[0], nofix[1],
                             "ok" if ok_n else "OUTSIDE"))
        print("  --- %d waypoints: with the fix %d outside, without it %d outside"
              % (n_pts, n_fixed_bad, n_nofix_bad))
        print()

    print("=" * 74)
    print("How to read this")
    print("  * psi=0 is the simulation's arrangement and the only case where the")
    print("    two columns agree, which is why the bug never showed up in Gazebo.")
    print("  * With the fix the routes are the field's own routes and are inside")
    print("    by construction for every psi.  A waypoint can only be OUTSIDE in")
    print("    the 'no fix' column.")
    print("  * The walls move with psi.  Comparing rotated waypoints against the")
    print("    psi=0 wall rectangle is the mistake that produces false alarms.")
    print("  * 'safe in odom' is what rescue_mission_node prints before arming.")
    print("    All four angles give a DIFFERENT rectangle, so that one line is")
    print("    enough to identify the angle: the pad is a field corner, so no two")
    print("    quarter turns put the square in the same place. (That would fail")
    print("    if the safety field were centred on the pad - a rotation about the")
    print("    centre leaves a centred square alone - which it is not.)")


if __name__ == "__main__":
    main()
