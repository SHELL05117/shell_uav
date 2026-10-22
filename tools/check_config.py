#!/usr/bin/env python3
"""Pre-flight checks before rebuilding or flying.

Every check here exists because the corresponding mistake actually cost a run.
Run it after ANY edit to a launch / yaml / lua file, and before you rebuild:

    python3 tools/check_config.py
    python3 tools/check_config.py --baseline /home/expload/catkin_ws_bak/src

Checks
  1  YAML parses              - caught two indentation slips pasted into a param
                                file; the file still parses for roslaunch, the
                                key just ends up in the wrong namespace
  2  YAML duplicate keys      - a key written twice silently keeps the LAST one
  3  launch XML well-formed   - caught a missing </node> when -  -  editing by hand
  4  Lua parses               - real parser; needs `pip install luaparser`
  5  Lua define/use order     - THE 20:45 CRASH: executable statements above
                                include "trajectory_builder.lua" abort cartographer at
                                startup with "attempt to index global
                                'TRAJECTORY_BUILDER_2D' (a nil value)".
                                A comment edit is enough to reintroduce it, because the
                                natural place to explain TRAJECTORY_BUILDER_2D is right
                                next to the value you meant to change - and the values
                                live at the BOTTOM of the file, after the includes.
  6  Frame consistency        - the frame the mission stamps on a goal vs the frame the
                                route table is authored in. These MUST be reasoned
                                about together; see the long comment in
                                rescue_mission.cpp::publishNextGoal. (The costmap half
                                left with move_base on 2026-10-17; there is no second
                                frame to disagree any more, only this assumption.)
  7  Planner frames          - every goal-subscribing node's global_frame vs the frame
                                the mission stamps on its goals. A mismatch makes the
                                planner drop every goal silently: 2026-09-25 EGO was
                                left on map while the mission had moved to odom, and
                                the vehicle hovered for a whole run.
  8  Ego local planner        - the EGO grid map and its route waypoints must
                                agree. Three silent killers at once: the optimizer
                                needs > 4 reference points or lbfgs gets n <= 0 and
                                returns LBFGSERR_INVALID_N (-1021); the local map
                                must be bigger than sensing_range or in-range
                                obstacles are dropped AND out-of-map reads as
                                solid; and every route waypoint must clear the
                                inflated band by position_tolerance, or the
                                vehicle freezes short of it forever.
                                (2026-09-25: all three were live in one run.)
  9  cmd_vel: one rotation    - the chain must contain no frame conversion at all.
                                It used to contain two, one per node, taken from two
                                DIFFERENT attitude estimates; they do not cancel, the
                                residual steers the aircraft sideways and the
                                mission's position loop cannot see it because its
                                setpoint follows the vehicle. The switches that
                                allowed it are gone, so this gate asserts the absence
                                structurally: no launch may declare them again, and
                                neither end of /cmd_vel may contain trigonometry.
                                Also checks that the pursuit lookahead is at least
                                one second of travel, since steer angle is
                                atan(cross_track / lookahead). (2026-09-25 late: the
                                sway.)
 10  Stage and drop plan        - which round the launch flies, where it drops,
                                and whether the person route really covers every
                                person task area. The two rounds differ by one
                                extra drop at a position drawn on the day, so
                                "which round" and "where" are set by hand at the
                                venue - and a hand-set parameter that is wrong in
                                the same way every time is exactly what a static
                                check is for.
 11  World model resources    - every model:// a world references must sit in a
                                src/**/models/ directory, AND the launch that
                                loads that world (or something that includes it)
                                must export GAZEBO_MODEL_PATH. Gazebo sends an
                                unresolvable model:// to the online model
                                database; with that disabled the world never
                                finishes loading and the GUI just says
                                "Preparing your world", with no error anywhere.
                                (2026-09-30: model://gc_task_markers was added to
                                the world, but GAZEBO_MODEL_PATH lived only in
                                gazebo_field.launch, which the PX4 launch path
                                never touches.)
 12  Drop servo angles        - the two drop angles, the parked angle and the
                                PCA9685 settings behind them. The servo node
                                resolves four fixed commands to four parameters,
                                and on this mechanism the ANGLE is what selects
                                which container releases - so two angles that are
                                nearly equal release the wrong material, and an
                                angle outside 0..180 is silently clamped by
                                setAngle() before the pulse width is computed.
                                The node only WARNs about this and then flies;
                                here it fails, because the node's own validation
                                never runs until the servo node is already up.
                                (2026-09-30, after the servo control rewrite.)
 13  Vision handshake        - the detector include is the only way the mission
                                learns anything, so the include, the topic it
                                forwards and the label file have to line up. A topic
                                the two sides disagree on produces no messages and
                                no error; on the aircraft the detector opens the
                                camera itself through videoSource, so an empty
                                input_URI means nobody opens it and the whole vision
                                chain is silent while every drop falls back to its
                                waypoint; and the label file's line 0 must be
                                `background` because the class id IS the line
                                number, so an off-by-one mislabels everything while
                                every box stays put. None of this shows up in a
                                startup log.
 14  Pre-arm gates            - what has to be true before auto_start requests
                                OFFBOARD (a detection message has arrived, because
                                the TensorRT engine needs ~26.5 s) and whether the
                                mission's arrival tolerance is compatible with the
                                point the follower is willing to stop at. Both were
                                wrong on 2026-10-17: the vehicle could arm before
                                the detector existed, and it sat 0.097 m from a
                                waypoint waiting to "arrive".
 15  Baseline parameter diff  - with --baseline, every differing key, so a deliberate
                                exception and an accidental one never look the same
 16  Vision locking           - MissionVision must never take detection_mutex_
                                twice on one thread. The 2026-10-18 shape was a
                                function-scope guard in detectionCallback plus the
                                inner guard in findDetectionCandidates: glibc
                                deadlocks, the node keeps publishing nothing, and
                                "no stable detection (last: found=0 stable=0)" is
                                byte-identical to what an empty frame reports.
                                The same thread then held the mutex setMode needs,
                                so abortMission blocked before state_ = ABORT and
                                the aircraft hovered with "Mission aborted" already
                                printed. This gate is what separates "the vision
                                layer is wedged" from "the detector saw nothing".
 17  Simulated payload         - a drop command must put an object into the world,
                                or the simulation reports a successful drop that
                                never happened. Silently wrong in five separate
                                ways: no branch for DROP_FIRE/DROP_RESCUE, a
                                <static> payload (spawns, then hangs in mid-air),
                                no <collision> (falls through the floor), a
                                release depth inside the down camera's near field,
                                and no marker (the box exists only in Gazebo's own
                                GUI while the operator watches Foxglove).
                                It also holds the blocking rule: the spawn may not
                                be called from the /gpio_cmd callback, which shares
                                a spinner thread with /gazebo/model_states, and the
                                CMakeLists/package.xml pair: a component named in
                                find_package() or CATKIN_DEPENDS but not declared
                                as a <build_depend> stops the build at configure
                                time, with a message that points at catkin_package
                                rather than at the missing line (2026-10-18,
                                visualization_msgs).
 18  Route goal topic         - the mission must advertise its waypoint stream on a
                                topic nothing else publishes, and the planner's
                                'goal' remap must name that same topic. The old
                                default, /move_base_simple/goal, is also advertised
                                by mavros_extras' guided_target plugin, which
                                republishes the FCU's POSITION_TARGET_GLOBAL_INT
                                (~10 Hz) as a 'map'-frame PoseStamped; the planner's
                                frame check answers that with clearPlan(), wiping
                                the plan of the leg being flown. The plugin's own log
                                line, "PositionTargetGlobal failed because no origin",
                                prints at the full 10 Hz and reads like the
                                localisation package failing - it is not.
                                (2026-10-18.)

Exit code 0 = clean, 1 = at least one failure. Warnings never fail the run.
"""

import argparse
import glob
import io
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAILURES = []
WARNINGS = []


def fail(check, path, msg):
    FAILURES.append((check, path, msg))
    print("  FAIL  %-34s %s" % (os.path.relpath(path, ROOT), msg))


def warn(check, path, msg):
    WARNINGS.append((check, path, msg))
    print("  warn  %-34s %s" % (os.path.relpath(path, ROOT), msg))


def heading(n, text):
    print("\n[%d] %s" % (n, text))


# ---------------------------------------------------------------- 1 & 2: YAML
def dup_key_loader(relpath):
    """Yield (line, key) for any key that appears twice in the same mapping."""
    dups = []
    stack = []  # (indent, keys_seen)
    for lineno, raw in enumerate(open(relpath, encoding="utf-8"), 1):
        line = raw.split("#")[0].rstrip()
        if not line.strip():
            continue
        m = re.match(r"^(\s*)([^\s:#][^:]*):", line)
        if not m:
            continue
        indent, key = len(m.group(1)), m.group(2).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if stack and stack[-1][0] == indent:
            if key in stack[-1][1]:
                dups.append((lineno, key))
            else:
                stack[-1][1].add(key)
        else:
            stack.append((indent, {key}))
    return dups


def check_yaml():
    heading(1, "YAML parses")
    try:
        import yaml
    except ImportError:
        warn("yaml", ROOT, "PyYAML not installed; skipped (pip install pyyaml)")
        return
    files = sorted(glob.glob(os.path.join(ROOT, "src/**/*.yaml"), recursive=True) +
                   glob.glob(os.path.join(ROOT, "src/**/*.yml"), recursive=True))
    for f in files:
        try:
            yaml.safe_load(open(f, encoding="utf-8"))
        except Exception as e:
            fail("yaml", f, str(e).replace("\n", " ")[:160])
    print("  %d file(s) checked" % len(files))

    heading(2, "YAML duplicate keys")
    for f in files:
        for lineno, key in dup_key_loader(f):
            fail("yaml-dup", f, "line %d: key %r defined twice in one mapping" % (lineno, key))
    print("  %d file(s) checked" % len(files))


# ------------------------------------------------------------------ 3: launch
def check_launch():
    heading(3, "launch XML well-formed")
    import xml.etree.ElementTree as ET
    files = sorted(glob.glob(os.path.join(ROOT, "src/**/*.launch"), recursive=True))
    for f in files:
        try:
            ET.parse(f)
        except Exception as e:
            fail("xml", f, str(e).replace("\n", " ")[:160])
    print("  %d file(s) checked" % len(files))


# --------------------------------------------------------------------- 4 & 5
INCLUDE_ORDER = {
    "TRAJECTORY_BUILDER_2D": "trajectory_builder.lua",
    "TRAJECTORY_BUILDER_3D": "trajectory_builder.lua",
    "TRAJECTORY_BUILDER": "trajectory_builder.lua",
    "POSE_GRAPH": "trajectory_builder.lua",
    "MAP_BUILDER": "map_builder.lua",
}


def strip_lua_comments(text):
    """Blank out comments, keeping line numbering. Good enough for config files."""
    out = []
    for raw in text.splitlines():
        i = raw.find("--")
        out.append(raw if i < 0 else raw[:i])
    return out


def check_lua():
    heading(4, "Lua parses")
    files = sorted(glob.glob(os.path.join(ROOT, "src/**/*.lua"), recursive=True))
    try:
        from luaparser import ast
    except ImportError:
        ast = None
        warn("lua", ROOT, "luaparser not installed; structural check only "
                          "(pip install luaparser)")
    parsed = 0
    for f in files:
        if ast is None:
            break
        try:
            ast.parse(open(f, encoding="utf-8").read())
            parsed += 1
        except Exception as e:
            fail("lua-parse", f, str(e).replace("\n", " ")[:200])
    if ast is not None:
        print("  %d of %d file(s) parsed" % (parsed, len(files)))

    heading(5, "Lua define/use order")
    for f in files:
        lines = strip_lua_comments(open(f, encoding="utf-8").read())
        code = [(n, s.strip()) for n, s in enumerate(lines, 1) if s.strip()]
        if not code:
            continue
        inc = {}
        for n, s in code:
            m = re.match(r'^include\s+"([^"]+)"', s)
            if m:
                inc.setdefault(m.group(1), n)
        first_code = code[0]
        if not re.match(r'^include\s+"', first_code[1]):
            fail("lua-order", f,
                 "line %d runs %r before any include; every global these files "
                 "touch is created by an include" % (first_code[0], first_code[1][:50]))
            continue
        for globalname, defining in sorted(INCLUDE_ORDER.items()):
            use = next((n for n, s in code if s.startswith(globalname + ".")
                        or s.startswith(globalname + " ")), None)
            if use is None:
                continue
            base = os.path.basename(defining)
            if base not in inc:
                continue
            if use < inc[base]:
                fail("lua-order", f,
                     "line %d uses %s but %s is only included at line %d"
                     % (use, globalname, base, inc[base]))
        print("  %-30s includes at %s"
              % (os.path.relpath(f, ROOT), sorted(inc.values()) or "none"))


# ------------------------------------------------------------ 6: frame wiring
def check_frames():
    """Which frame the route is authored in, and which frame it is flown in.

    2026-10-17: the costmap half of this gate is gone with DWA/TEB/move_base.
    What is left is the statement that used to follow it - the route table is
    field-frame data and odom is *assumed* to be the field frame - because that
    assumption is the single thing that decides whether every waypoint in the
    launch file means what its comment says. Nothing else checks it, and at
    field_yaw_deg != 0 it stops being true on purpose.
    """
    heading(6, "frame consistency (mission goal frame and route table frame)")
    mission = os.path.join(ROOT, "src/navigation_2d/src/rescue_mission.cpp")
    if not os.path.exists(mission):
        warn("frames", mission, "not found; skipped")
        return
    src = open(mission, encoding="utf-8", errors="replace").read()
    m = re.search(r'goal\.header\.frame_id\s*=\s*"([^"]+)"', src)
    if not m:
        warn("frames", mission, "no goal.header.frame_id assignment found")
        return
    goal_frame = m.group(1)
    print("  mission publishes its goal in : %s" % goal_frame)

    # The route table is authored in the field frame. In the simulation that frame
    # IS odom by construction; on the real vehicle it is not, and that mismatch is
    # what rescue_mission_node's field_yaw_deg exists to close. So print the
    # arrangement this launch actually configures instead of asserting the sim's.
    launch = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if os.path.exists(launch) and "hazard_route" in open(launch, encoding="utf-8").read():
        import xml.etree.ElementTree as ET
        field_yaw = None
        try:
            raw = _launch_arg(ET.parse(launch).getroot(), "field_yaw_deg", "0.0")
            field_yaw = float(raw)
        except Exception:
            field_yaw = None
        if field_yaw is None or abs(field_yaw) < 1e-6:
            print("  route table in %s is field-frame data and odom is taken to be "
                  "the field frame (field_yaw_deg=0), which is the simulation's "
                  "arrangement" % os.path.relpath(launch, ROOT))
        else:
            print("  route table in %s is field-frame data, published rotated by "
                  "%+g deg into odom (field_yaw_deg=%g); checks 8 and 10 compare it "
                  "against world-file geometry, both in the field frame, so they "
                  "hold at this angle too"
                  % (os.path.relpath(launch, ROOT), -field_yaw, field_yaw))


# --------------------------------------- 7: planner frames vs mission goal frame
def mission_goal_frame():
    mission = os.path.join(ROOT, "src/navigation_2d/src/rescue_mission.cpp")
    if not os.path.exists(mission):
        return None, None
    src = open(mission, encoding="utf-8", errors="replace").read()
    m = re.search(r'goal\.header\.frame_id\s*=\s*"([^"]+)"', src)
    return (m.group(1) if m else None), mission


def check_planner_frames():
    heading(7, "planner frames vs mission goal frame")
    goal_frame, mission = mission_goal_frame()
    if goal_frame is None:
        warn("planner-frames", mission or ROOT,
             "could not read goal.header.frame_id; skipped")
        return

    launch = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if not os.path.exists(launch):
        warn("planner-frames", launch, "not found; skipped")
        return

    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(launch).getroot()
    except Exception as exc:
        fail("planner-frames", launch, "cannot parse: %s" % exc)
        return

    # Any node in the mission's control path: it subscribes to the topic the
    # mission publishes goals on, so it ends up comparing its global_frame
    # against the goal's frame_id, and both the planner and the follower
    # hard-reject on a mismatch rather than warning. (Until 2026-10-17 these
    # nodes were also selected by a planner_backend "if=" gate; that switch is
    # gone and the remap is now the only selection criterion, which is a
    # stronger one - a node cannot stop consuming goals without losing the
    # remap.)
    checked = []
    for node in root.iter("node"):
        remaps_goal = any((r.get("from") or "") == "goal" for r in node.iter("remap"))
        for param in node.iter("param"):
            if param.get("name") != "global_frame":
                continue
            if not remaps_goal:
                continue
            value = param.get("value") or ""
            label = node.get("type") or node.get("name") or "?"
            checked.append((label, value, remaps_goal))

    if not checked:
        print("  no goal-subscribing node sets a global_frame")
        return

    for label, value, remaps_goal in checked:
        why = "consumes the mission's goal"
        if "$(" in value:
            warn("planner-frames", launch,
                 "%s global_frame=%r is an unexpanded substitution; cannot check"
                 % (label, value))
            continue
        print("  %-38s global_frame=%-6s (%s)" % (label, value, why))
        if value != goal_frame:
            fail("planner-frames", launch,
                 "%s runs in frame %r but the mission stamps every goal %r. "
                 "EgoPlanner2DNode::goalCallback and EgoTrajectoryController both "
                 "drop a message whose frame_id differs from global_frame, so this "
                 "node would silently ignore every goal and the vehicle would sit "
                 "still (2026-09-25: the ego backend hovered for exactly this "
                 "reason). Make this equal the mission's goal frame."
                 % (label, value, goal_frame))


# --------------------------------------------------------- 8: baseline delta
def _world_footprints(fly_height):
    """2D footprints of every obstacle the aircraft can hit at fly_height.

    Reads the Gazebo world and keeps only models whose vertical extent covers
    the flight altitude, so the floor and the take-off pad (both a couple of
    centimetres tall) drop out without hard-coding their names.

    Returns [(name, cx, cy, yaw, ('box', hx, hy) | ('cylinder', r))].
    """
    import xml.etree.ElementTree as ET

    world = os.path.join(ROOT, "src/gc_2026_sim/gazebo/worlds/gc_2026_field.world")
    if not os.path.exists(world):
        return None
    try:
        root = ET.parse(world).getroot()
    except Exception:
        return None

    out = []
    for model in root.iter("model"):
        name = model.get("name") or "?"
        pose = model.find("pose")
        if pose is None:
            continue
        vals = [float(v) for v in (pose.text or "").split()]
        if len(vals) < 6:
            continue
        cx, cy, cz, _r, _p, yaw = vals[:6]

        geom = None
        height = None
        for collision in model.iter("collision"):
            box = collision.find(".//box/size")
            cyl = collision.find(".//cylinder")
            if box is not None:
                sx, sy, sz = [float(v) for v in box.text.split()[:3]]
                geom = ("box", sx / 2.0, sy / 2.0)
                height = sz
                break
            if cyl is not None:
                radius = float(cyl.find("radius").text)
                length = float(cyl.find("length").text)
                geom = ("cylinder", radius)
                height = length
                break
        if geom is None or height is None:
            continue

        # keep only what actually blocks the flight band
        if not (cz - height / 2.0 <= fly_height <= cz + height / 2.0):
            continue
        out.append((name, cx, cy, yaw, geom))
    return out


def _footprint_clearance(px, py, cx, cy, yaw, geom):
    """Shortest horizontal distance from (px, py) to a footprint's boundary.

    Negative means the point is inside the footprint.
    """
    import math

    dx, dy = px - cx, py - cy
    ca, sa = math.cos(-yaw), math.sin(-yaw)
    lx = ca * dx - sa * dy          # point in the body frame
    ly = sa * dx + ca * dy

    if geom[0] == "box":
        _kind, hx, hy = geom
        ox = max(abs(lx) - hx, 0.0)
        oy = max(abs(ly) - hy, 0.0)
        if ox == 0.0 and oy == 0.0:
            # inside: distance to the nearest face, still signed
            return -min(hx - abs(lx), hy - abs(ly))
        return math.hypot(ox, oy)

    radius = geom[1]
    return math.hypot(lx, ly) - radius


def _segment_clearance(ax, ay, bx, by, cx, cy, yaw, geom, samples=200):
    """Shortest horizontal distance from segment a->b to a footprint boundary.

    Negative means the segment passes through the footprint (or, for a box,
    through its interior). Sampled rather than solved analytically: a route
    leg is a few metres long, 200 samples is 2 cm on a 4 m leg, and this is a
    diagnostic, not a controller.
    """
    import math

    ca, sa = math.cos(-yaw), math.sin(-yaw)

    def to_local(px, py):
        dx, dy = px - cx, py - cy
        return (ca * dx - sa * dy, sa * dx + ca * dy)

    la = to_local(ax, ay)
    lb = to_local(bx, by)

    best = None
    for i in range(samples + 1):
        t = i / float(samples)
        lx = la[0] + (lb[0] - la[0]) * t
        ly = la[1] + (lb[1] - la[1]) * t
        if geom[0] == "box":
            _kind, hx, hy = geom
            ox = max(abs(lx) - hx, 0.0)
            oy = max(abs(ly) - hy, 0.0)
            if ox == 0.0 and oy == 0.0:
                d = -min(hx - abs(lx), hy - abs(ly))
            else:
                d = math.hypot(ox, oy)
        else:
            d = math.hypot(lx, ly) - geom[1]
        if best is None or d < best:
            best = d
    return best


def _launch_param(root, node_type, name, default=None):
    for node in root.iter("node"):
        if (node.get("type") or "") != node_type:
            continue
        for param in node.iter("param"):
            if param.get("name") == name:
                return (param.get("value") or "").strip()
    return default


def _launch_rosparam(root, node_type, name):
    for node in root.iter("node"):
        if (node.get("type") or "") != node_type:
            continue
        for params in node.iter("rosparam"):
            if params.get("param") != name:
                continue
            pts = []
            for line in (params.text or "").splitlines():
                m = re.search(r"\{x:\s*([-\d.]+),\s*y:\s*([-\d.]+)", line)
                if m:
                    pts.append((float(m.group(1)), float(m.group(2))))
            return pts
    return []


def check_ego_local_planner():
    """EGO's grid map must be able to represent what it is asked to fly to.

    Three separate ways the 2D port silently produced an unreachable waypoint,
    all of them found on 2026-09-25 from the same frozen run:

      a) BsplineOptimizer::rebound_optimize() computes
             variable_num_ = 2 * ((K + 2) - 2 * order_),  order_ = 3
         so the reference path must keep K >= 5 points. Below that lbfgs is
         handed n <= 0 and returns LBFGSERR_INVALID_N (-1021) "Invalid number
         of variables specified. Skip this planning." - the plan is dropped.

      b) GridMap2D is map_x_size x map_y_size metres centred on the vehicle,
         but getInflateOccupancy() returns TRUE outside it. If sensing_range
         exceeds half the map size, obstacles the node has already accepted are
         dropped by setObstacle() and the whole region beyond the map edge is
         treated as solid.

      c) A route waypoint must sit outside the inflated band, with enough room
         for the mission's own arrival tolerance. The inflation is quantised to
         the grid, so the effective clearance is
             inflate + resolution/2 (rasterisation) + one lattice step,
         and the vehicle can only ever stop on a cell centre.
    """
    import xml.etree.ElementTree as ET

    heading(8, "ego local planner: map, inflation and route waypoints")

    launch = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if not os.path.exists(launch):
        warn("ego-map", launch, "not found; skipped")
        return
    try:
        root = ET.parse(launch).getroot()
    except Exception as exc:
        fail("ego-map", launch, "cannot parse: %s" % exc)
        return

    node_type = "ego_planner_2d_node"
    if _launch_param(root, node_type, "map_inflate_value") is None:
        print("  no %s node in this launch; skipped" % node_type)
        return

    def num(name, default=None):
        raw = _launch_param(root, node_type, name)
        if raw is None:
            return default
        try:
            return float(raw)
        except ValueError:
            return default

    resolution = num("map_resolution", 0.1)
    inflate = num("map_inflate_value", 0.5)
    sensing = num("sensing_range", 10.0)
    x_size = num("map_x_size", 50.0)
    y_size = num("map_y_size", 50.0)
    window = num("local_window_points", 50)
    min_ref = num("min_ref_points", None)

    fly_height = None
    for arg in root.iter("arg"):
        if arg.get("name") == "fly_height":
            fly_height = float(arg.get("default") or 1.5)
    if fly_height is None:
        fly_height = 1.5

    pos_tol = float(_launch_param(root, "rescue_mission_node",
                                  "position_tolerance", 0.12) or 0.12)

    # The airframe radius used to come from costmap_common_params.yaml
    # (robot_radius). That file left with move_base on 2026-10-17, but the
    # number itself did not go anywhere: rescue_mission now applies it directly
    # as align_body_clearance, the margin it shrinks the safe field by before
    # commanding an align setpoint. Same physical quantity, one owner.
    robot_radius = None
    raw_clearance = _launch_param(root, "rescue_mission_node",
                                  "align_body_clearance")
    if raw_clearance is not None:
        try:
            robot_radius = float(raw_clearance)
        except ValueError:
            robot_radius = None

    print("  resolution=%.3f m  inflate=%.3f m  sensing=%.2f m  "
          "map=%.1fx%.1f m  fly_height=%.2f m  position_tolerance=%.2f m"
          "  airframe_r=%s"
          % (resolution, inflate, sensing, x_size, y_size, fly_height, pos_tol,
             "?" if robot_radius is None else "%.2f m" % robot_radius))

    # (b) the map must hold the sensor range
    for label, size in (("map_x_size", x_size), ("map_y_size", y_size)):
        if size / 2.0 < sensing:
            fail("ego-map", launch,
                 "%s=%.1f m gives the local map a half-extent of %.2f m, smaller "
                 "than sensing_range=%.2f m. GridMap2D::getInflateOccupancy() "
                 "reports TRUE outside the map and GridMap2D::setObstacle() drops "
                 "anything outside it with '栅格索引... 超出范围', so obstacles "
                 "between %.2f m and %.2f m are silently discarded AND treated as "
                 "solid. Make the map at least %.1f m, or lower sensing_range."
                 % (label, size, size / 2.0, sensing, size / 2.0, sensing,
                    2.0 * sensing))

    # safety floor: the inflation must not be inside the airframe
    half_diagonal = resolution * 1.41421356 / 2.0
    required_inflate = (robot_radius + half_diagonal) if robot_radius is not None else None
    if robot_radius is not None and inflate < robot_radius:
        fail("ego-map", launch,
             "map_inflate_value=%.3f m is smaller than align_body_clearance="
             "%.3f m (the airframe radius) in navigation_2d.launch. No plan can "
             "be trusted below that: a free cell is one whose centre is further "
             "than inflate from the nearest obstacle cell, so anything under the "
             "radius puts the rotor tips inside the surface by construction"
             % (inflate, robot_radius))
    elif required_inflate is not None and inflate < required_inflate - 1e-9:
        # 带边界 = 障碍格心 + inflate，而格心本身可以比真实表面外凸/内缩半个
        # 对角（half_diagonal）。所以 inflate 只等于机身半径时，"刚好合法"的
        # 规划点就能让桨尖压到表面上。
        print("  inflation budget: radius %.3f + half a lattice diagonal %.3f = "
              "%.3f m needed, configured %.3f m (%.0f mm short)"
              % (robot_radius, half_diagonal, required_inflate, inflate,
                 1000 * (required_inflate - inflate)))
        warn("ego-map", launch,
             "map_inflate_value=%.3f m leaves %.0f mm of the airframe uncovered "
             "at the worst lattice phase (radius %.3f m + %.3f m half-diagonal = "
             "%.3f m). A plan that hugs the band edge can then put the rotor tips "
             "on the surface. hazard_route[0] sits 0.40 m off the west wall and "
             "tolerates at most %.3f m at %.2f m resolution, so this value is at "
             "that legal ceiling - going higher needs the waypoint moved or the "
             "resolution halved."
             % (inflate, 1000 * (required_inflate - inflate), robot_radius,
                half_diagonal, required_inflate, 0.40 - resolution / 2.0,
                resolution))
    elif required_inflate is not None:
        print("  inflation budget: radius %.3f + half a lattice diagonal %.3f = "
              "%.3f m needed, configured %.3f m (covered)"
              % (robot_radius, half_diagonal, required_inflate, inflate))

    # (a) the optimizer needs enough reference points
    if min_ref is None:
        warn("ego-map", launch,
             "ego_planner_2d has no min_ref_points; the node falls back to its "
             "own default (8). Reading it here keeps the check honest - the "
             "optimizer needs > 4.")
    elif min_ref <= 4:
        fail("ego-map", launch,
             "min_ref_points=%g, but BsplineOptimizer::rebound_optimize() needs "
             "K >= 5 reference points (variable_num_ = 2 * ((K + 2) - 2 * 3) > 0). "
             "Below that lbfgs returns LBFGSERR_INVALID_N (-1021) and every plan "
             "is dropped." % min_ref)
    else:
        print("  min_ref_points=%g (optimizer needs > 4)" % min_ref)

    if min_ref is not None and window < min_ref:
        warn("ego-map", launch,
             "local_window_points=%g < min_ref_points=%g; normal-length paths "
             "would need re-sampling every cycle" % (window, min_ref))

    # (c) every route waypoint must clear the inflated band
    footprints = _world_footprints(fly_height)
    if footprints is None:
        warn("ego-map", ROOT, "could not read the Gazebo world; waypoint "
                              "clearance not checked")
        footprints = []

    # How much clearance a waypoint needs, worked out from what the code does
    # rather than from a safety habit.
    #
    # A cell is inflated iff the distance from that cell to the nearest OBSTACLE
    # CELL is <= inflate. Cell centres sit on a lattice of size resolution, so
    # that distance is always resolution * sqrt(dx^2 + dy^2) for integers dx, dy
    # - it is NOT a continuous quantity:
    #
    #   res=0.10, inflate=0.35 -> free requires a lattice distance > 0.35 m,
    #   and the smallest lattice distance above 0.35 is sqrt(13)*0.10 = 0.3606
    #   (0.30, sqrt(10)*0.1 = 0.3162 and sqrt(12) are not lattice distances).
    #
    # worldToGrid() rounds with std::round(), so a footprint point and the
    # waypoint can each land up to resolution*sqrt(2)/2 away from their own cell
    # centre; together that is resolution*sqrt(2) of slack. Hence:
    #
    #   clearance >= next_free_lattice_distance + resolution*sqrt(2)
    #   => free for EVERY map origin, i.e. for every vehicle position.
    #
    # Below resolution/2 of that (clearance < inflate + resolution/2) the
    # waypoint can be inside the band outright - fatal, because both EGO's A*
    # start/end check and its collision check use getInflateOccupancy, so an
    # inflated goal is never targeted and the mission's arrival test can never
    # pass.
    import math

    def _next_free_lattice(inflate, resolution, limit=10):
        best = None
        for dx in range(0, limit + 1):
            for dy in range(0, limit + 1):
                if dx == 0 and dy == 0:
                    continue
                d = resolution * math.hypot(dx, dy)
                if d > inflate and (best is None or d < best):
                    best = d
        return best

    next_free = _next_free_lattice(inflate, resolution)
    inside_band = inflate + resolution / 2.0
    if next_free is None:
        warn("ego-map", launch,
             "inflate=%.2f m is far larger than %d cells at %.2f m resolution; "
             "waypoint clearance not checked" % (inflate, 10, resolution))
    wants_slack = (next_free + resolution * math.sqrt(2.0)) if next_free else inside_band

    routes = [("hazard_route", _launch_rosparam(root, "rescue_mission_node", "hazard_route")),
              ("person_route", _launch_rosparam(root, "rescue_mission_node", "person_route")),
              ("return_route", _launch_rosparam(root, "rescue_mission_node", "return_route"))]

    print("  waypoint clearance: fail < %.4f m, phase-robust >= %.4f m "
          "(inflate %.3f; free needs one lattice step above it = %.4f m, plus %.4f m "
          "of rounding slack)" % (inside_band, wants_slack, inflate,
                                  next_free if next_free else float("nan"),
                                  resolution * math.sqrt(2.0)))

    checked = 0
    for route_name, waypoints in routes:
        for idx, (wx, wy) in enumerate(waypoints):
            if not footprints:
                break
            nearest, best = None, None
            for name, cx, cy, yaw, geom in footprints:
                clear = _footprint_clearance(wx, wy, cx, cy, yaw, geom)
                if best is None or clear < best:
                    nearest, best = name, clear
            if best is None:
                continue
            checked += 1

            if best < inside_band:
                fail("ego-map", launch,
                     "%s[%d] (%.2f, %.2f) is only %.4f m from '%s' - less than "
                     "inflate + a quarter of the band. With map_inflate_value=%.2f "
                     "at %.2f m resolution the waypoint's cell can be inside the "
                     "inflated band, and EGO will then refuse to target it: both "
                     "AStar::ConvertToIndexAndAdjustStartEndPoints and the "
                     "B-spline collision check use getInflateOccupancy. The "
                     "mission measures arrival against its own odom position with "
                     "position_tolerance=%.2f m, so the leg can never complete - "
                     "the vehicle freezes short of the waypoint with a clean log "
                     "(2026-09-25: hazard_route[0] at x=-0.25 was 0.425 m off the "
                     "west wall and the vehicle stopped dead at x=-0.11, "
                     "d=0.406 m, for the rest of the run). Move the waypoint out, "
                     "or use a finer map_resolution."
                     % (route_name, idx, wx, wy, best, nearest, inflate,
                        resolution, pos_tol))
            elif best < wants_slack:
                # Whether the waypoint's cell is free depends on the map origin
                # UNLESS the clearance is an exact whole number of cells: rounding
                # is translation invariant by an integer, so the index difference
                # between the obstacle cell and the waypoint's cell is then the
                # same for every vehicle position.
                sep_cells = best / resolution
                phase_free = abs(sep_cells - round(sep_cells)) < 1e-6
                warn("ego-map", launch,
                     "%s[%d] (%.2f, %.2f) clears '%s' by %.4f m; margin over "
                     "inflate is %.4f m. %s One stray scan point landing more than "
                     "%.4f m inside the face moves the obstacle cell a whole step "
                     "closer and swallows the waypoint, putting the vehicle back in "
                     "the 2026-09-25 freeze.%s"
                     % (route_name, idx, wx, wy, nearest, best, best - inflate,
                        ("It is an exact %.0f cells (%.4f / %.2f), so the waypoint's "
                         "cell is free for every map origin - that is the only "
                         "reason this is not a failure." % (round(sep_cells), best,
                                                            resolution))
                        if phase_free else
                        ("It is %.2f cells (%.4f / %.2f), not a whole number, so "
                         "whether the waypoint's cell is free depends on where the "
                         "vehicle is and can flip between two planning cycles."
                         % (sep_cells, best, resolution)),
                        resolution / 2.0,
                        " hazard_route[0] is the one that actually burned a run - "
                        "give it more clearance."
                        if route_name == "hazard_route" and idx == 0 else ""))
            else:
                print("  %-12s [%d] (%.2f, %.2f) clearance %.4f m to '%s' "
                      "(margin over inflate %.4f m)"
                      % (route_name, idx, wx, wy, best, nearest, best - inflate))

    if footprints and checked == 0:
        warn("ego-map", launch, "no route waypoints found to check")

    # ---- 航点之间的走廊 -------------------------------------------------
    #
    # 上面只看航点。2026-10-17 实机卡住的位置不在任何航点上，而在
    # person_route 第 1 段到第 2 段的中间：
    #
    #   fixed_obstacle_2 的下端 (2.184, -0.516) 与 random_obstacle_2
    #   (2.10, -1.45) 之间表面间隙 0.788 m，两边各膨胀 0.35 m 之后自由带只剩
    #   0.088 m，而机身直径 0.44 m。
    #
    # 航点全部干净（旧路线每个航点都 >= 0.40 m），走廊却是死的。飞机飞进
    # 去之后 EGO 一直报 "the vehicle's own cell ... is inside the inflated
    # obstacle band"，6 s 后 mission 以 "Planner never started moving toward
    # the waypoint" 中止。
    #
    # 这里**只报告不判 FAIL**：EGO 会绕障，一条穿过膨胀带的直线不代表整段不
    # 可通行，真正的判据是"两侧有没有绕行空间"，那需要看具体几何。但净空为
    # 负的段必须出现在屏幕上，它是"这段要么能绕、要么根本过不去"的唯一提示。
    corridor_negative = []
    body_negative = []
    for route_name, waypoints in routes:
        if not footprints:
            break
        for idx in range(len(waypoints) - 1):
            ax, ay = waypoints[idx]
            bx, by = waypoints[idx + 1]
            nearest, best = None, None
            for name, cx, cy, yaw, geom in footprints:
                clear = _segment_clearance(ax, ay, bx, by, cx, cy, yaw, geom)
                if best is None or clear < best:
                    nearest, best = name, clear
            if best is None:
                continue
            margin = best - inflate
            # 到机身表面还剩多少。判据不是"膨胀"而是"桨尖"：膨胀只决定规划点
            # 能落在哪，能不能飞过去取决于障碍表面到航线的距离与桨尖半径。
            # 负值意味着这条直线本身就要穿过桨盘 —— EGO 必须鼓出去绕，鼓出去
            # 的那一段就是现场看到的"它非要往过不去的缝里钻"。
            body_margin = None if robot_radius is None else best - robot_radius
            flag = "  " if margin >= 0.0 else "!!"
            print("  %s %-12s leg %d (%.2f, %.2f)->(%.2f, %.2f) corridor "
                  "%.4f m to '%s' (margin over inflate %+.4f m, over the "
                  "airframe %s)"
                  % (flag, route_name, idx, ax, ay, bx, by, best, nearest, margin,
                     "?" if body_margin is None else "%+.4f m" % body_margin))
            if margin < 0.0:
                corridor_negative.append((route_name, idx, margin, nearest))
            if body_margin is not None and body_margin < 0.0:
                body_negative.append((route_name, idx, body_margin, nearest))

    if body_negative:
        print("  ** %d leg(s) run closer to an obstacle than the %.2f m airframe "
              "radius (up to %.2f m of overlap). The straight line is not "
              "flyable, so EGO has to bulge around it - that bulge is what looks "
              "like 'it drives into a gap it cannot fit through'. Not a failure, "
              "but if the bulge has nowhere to go the leg is impossible."
              % (len(body_negative), robot_radius or 0.0,
                 max(-b for _, _, b, _ in body_negative)))
        for route_name, idx, body_margin, nearest in body_negative:
            print("     %s leg %d overlaps '%s' by %.3f m"
                  % (route_name, idx, nearest, -body_margin))


    if corridor_negative:
        print("  ** %d leg(s) cross the inflated band. EGO will only fly them "
              "if there is room to go around; check the two-sided gap before "
              "trusting the route." % len(corridor_negative))
        for route_name, idx, margin, nearest in corridor_negative:
            print("     %s leg %d: %.3f m inside the band near '%s'"
                  % (route_name, idx, -margin, nearest))

    # ---- 代码结构：三处"算不到位" ---------------------------------------
    #
    # 2026-10-17 的原话是"它没有算到位直接撞到墙上了……进不去的缝隙也要进"。
    # 查下来是三个互相独立的洞，共同点是它们都只影响"安全"，不影响"能不能起
    # 飞"，所以飞一次看不出区别，必须在静态门禁里断言住：
    #
    #   1. BsplineOptimizer 的碰撞采样只覆盖 t < tmp * 2/3，也就是前三分之二
    #      的时长；尾巴不查就 return success。而尾巴恰恰是机身正要飞进去的那
    #      一段 —— 于是"规划成功"和"这段路是干净的"不是一回事。
    #   2. AStar 的 8 邻域只查目标格。对角移动的线段会贴着两格共用的顶点擦过
    #      去，最近 0.0707 m（0.10 m 分辨率），因此在障碍物角上能"找到"一条
    #      几何上已经压进膨胀带的路：进不去的缝隙在栅格层就放行了。
    #   3. EgoPlanner2DNode 重规划失败时无条件"保留上一条轨迹"，而逼它重规划的
    #      判据正是 collisionDetection()。保留 = 继续飞一条自己刚判成撞着的轨
    #      迹，/cmd_vel 一路照发。
    #
    # 三条都是"通过时也打印解析到的证据"，静默通过无法审计。
    optimizer_src = os.path.join(
        ROOT, "src/ego_planner_2d/src/bspline_opt/bspline_optimizer.cpp")
    astar_src = os.path.join(
        ROOT, "src/ego_planner_2d/src/path_searching/dyn_a_star.cpp")
    ego_node_src = os.path.join(
        ROOT, "src/ego_planner_2d/src/ego_planner_2d_node.cpp")
    planner_iface_src = os.path.join(
        ROOT, "src/ego_planner_2d/src/plan_manage/planner_interface.cpp")

    for path, label in ((optimizer_src, "bspline_optimizer.cpp"),
                        (astar_src, "dyn_a_star.cpp"),
                        (ego_node_src, "ego_planner_2d_node.cpp"),
                        (planner_iface_src, "planner_interface.cpp")):
        if not os.path.exists(path):
            warn("ego-map", ROOT, "%s is missing; the clearance assertions "
                                  "cannot be checked" % label)
            return

    # 断言必须看代码而不是看注释：这几处的解释性注释里就写着旧写法
    # （"原来只查 t < tmp * 2 / 3"），不剥注释的话门禁会被自己的说明文字绊倒。
    def _strip_comments(text):
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        return re.sub(r"//[^\n]*", "", text)

    optimizer_code = _strip_comments(_read_text(optimizer_src))
    whole_curve_sites = optimizer_code.count("t <= tmp + 1e-9")

    if "tmp * 2 / 3" in optimizer_code:
        fail("ego-map", optimizer_src,
             "a collision sample loop still stops at t < tmp * 2 / 3, so the "
             "last third of every plan is published unchecked. BsplineOptimizer "
             "returns success for a spline whose tail sits in the inflated "
             "band, and EgoTrajectoryController flies that tail. Sample to "
             "t <= tmp + 1e-9 in both rebound_optimize() and refine_optimize().")
    elif whole_curve_sites < 2:
        fail("ego-map", optimizer_src,
             "only %d of 2 collision loops sample the whole spline "
             "(rebound_optimize + refine_optimize); the other one leaves the "
             "plan's tail unchecked" % whole_curve_sites)
    else:
        print("  planner clearance: whole-spline sampling at %d site(s) "
              "(rebound + refine), not the first 2/3" % whole_curve_sites)

    astar_code = _strip_comments(_read_text(astar_src))
    if "Vector2i side_a" not in astar_code or "Vector2i side_b" not in astar_code:
        fail("ego-map", astar_src,
             "AStar still checks only the destination cell of a diagonal move. "
             "The straight segment between two free cells passes within "
             "resolution*sqrt(2)/2 (0.0707 m at 0.10 m) of the shared corner, "
             "so A* accepts a gap whose geometry is already inside the inflated "
             "band - the 'it drives into gaps it cannot fit through' case. "
             "Require both orthogonal side cells to be free for dx != 0 && dy "
             "!= 0.")
    else:
        print("  planner clearance: A* rejects diagonal corner squeezing "
              "(both side cells of a diagonal move are checked)")

    ego_node_code = _strip_comments(_read_text(ego_node_src))
    keep_branch_ok = "else if (collisionDetection(planned_traj_))" in ego_node_code
    if not keep_branch_ok:
        fail("ego-map", ego_node_src,
             "replan() falls back to 'keep the previous trajectory' without "
             "asking whether that trajectory still collides. collisionDetection() "
             "is what forced the replan, so keeping it means /cmd_vel keeps "
             "steering along a plan this node has already called unsafe - the "
             "vehicle drives into the wall with every consumer reporting "
             "healthy. Drop the stale plan (empty path = brake) when it "
             "collides.")
    else:
        print("  planner clearance: a stale plan is dropped instead of "
              "republished once it collides")

    # A* 的格子必须和地图的格子是同一套坐标。dyn_a_star 的 Index2Coord 是
    # (index - CENTER_IDX) * step_size + center_，所以 step_size 一旦写死 0.1
    # 而地图换成 0.05，A* 检查的格子就只有地图的一半 —— 地图细了、A* 还是粗的，
    # 于是"找到的路"比"地图认为自由的地方"粗一倍。三处调用点必须都跟随分辨率。
    planner_iface_code = _strip_comments(_read_text(planner_iface_src))
    # 必须用正则而不是字符串匹配：调用点写成
    #     AstarSearch(
    #         0.1, start, goal)
    # 时，"AstarSearch(0.1" 是匹配不到的 —— 第一版就是这么漏掉一次负向测试的。
    literal_step = (re.findall(r"AstarSearch\(\s*[0-9][0-9.]*",
                               planner_iface_code) +
                    re.findall(r"AstarSearch\(\s*[0-9][0-9.]*", optimizer_code))
    call_sites = (planner_iface_code.count("AstarSearch(") +
                  optimizer_code.count("AstarSearch("))
    resolution_used = ("getResolution()" in planner_iface_code and
                       "getResolution()" in optimizer_code)
    if literal_step:
        fail("ego-map", planner_iface_src,
             "%d AstarSearch call(s) pass a numeric literal as step_size (%s). "
             "AStar::Index2Coord() maps index -> world with that step, so a "
             "literal 0.1 against a %.2f m map searches only every other cell "
             "and the A* fallback path can be twice as coarse as the map. Pass "
             "grid_map_->getResolution() at all three call sites."
             % (len(literal_step), ", ".join(literal_step), resolution))
    elif call_sites < 3:
        fail("ego-map", planner_iface_src,
             "only %d AstarSearch call site(s) found; 3 are expected "
             "(planner_interface fallback + bspline_optimizer initControlPoints "
             "x2). One of them has been removed or renamed - re-check that each "
             "still passes the map resolution" % call_sites)
    elif not resolution_used:
        fail("ego-map", planner_iface_src,
             "AstarSearch no longer reads grid_map_->getResolution() in one of "
             "the two files, so the A* lattice can drift away from the map "
             "lattice again")
    else:
        print("  planner clearance: A* step follows map_resolution at %d call "
              "site(s), no numeric step literal" % call_sites)



def _launch_arg(root, name, default=None):
    for arg in root.iter("arg"):
        if arg.get("name") == name:
            return (arg.get("default") or "").strip()
    return default


def _resolve_arg_text(root, text):
    """Resolve a value of the form '$(arg name)' against the launch's own args.

    Needed because the values worth checking are hand-set arguments that the
    launch file forwards verbatim: reading the <param> element alone returns the
    literal '$(arg rescue_drop_x)', which is not a number and says nothing about
    whether the run is configured.
    """
    if text is None:
        return None
    match = re.fullmatch(r"\$\(arg\s+([A-Za-z0-9_]+)\s*\)", text.strip())
    if not match:
        return text
    return _launch_arg(root, match.group(1), text)


def _resolve_find(text, packages):
    """Resolve a path of the form '$(find pkg)/rest' to a real path, or None.

    Same job as the tail end of _launch_includes, split out because the vision
    check needs to look at a *file* a launch names (the detector's default label
    file), not at a launch it includes.
    """
    if not text:
        return None
    match = re.search(r"\$\(find\s+([^)\s]+)\)", text)
    if not match:
        return None
    package = match.group(1).strip()
    if package not in packages:
        return None
    rest = text[match.end():]
    return os.path.normpath(packages[package] + rest)


_UNQUOTE_RE = re.compile(r"^['\"](.*)['\"]$", re.DOTALL)
# Both spellings turn up in this workspace: $(arg x) for a whole value, and
# arg('x') inside $(eval ...), where roslaunch exposes it as a function rather
# than as a substitution.
_ARG_REF_RE = re.compile(
    r"^(?:\$\(arg\s+([A-Za-z0-9_]+)\s*\)"
    r"|arg\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\))$")
# Two shapes are in use:
#   $(eval <YES> if arg('x') else <NO>)              - truthiness
#   $(eval <YES> if arg('x') == '<value>' else <NO>) - comparison
# The second one is a trap, see _roslaunch_convert_auto().
_ARG_EVAL_RE = re.compile(
    r"^\$\(eval\s+(?P<yes>.+?)\s+if\s+arg\(\s*'(?P<name>[^']+)'\s*\)"
    r"(?:\s*==\s*'(?P<value>[^']*)')?\s+else\s+(?P<no>.+?)\s*\)$")


def _roslaunch_convert_auto(value):
    """roslaunch.loader.convert_value(value, 'auto'), faithfully.

    $(eval ...) does not hand arg() the raw string. substitution_args._eval
    wraps it in convert_value(..., 'auto'), so arg('use_usb_cam') is the
    BOOLEAN True, never the string 'true'. Which means

        $(eval A if arg('use_usb_cam') == 'true' else B)

    evaluates True == 'true' -> False and picks B for both values: a switch
    that cannot move, whatever the operator passes. That is not hypothetical,
    it is what this workspace shipped until 2026-10-17 - the aircraft was
    pinned to the simulation's Python detector, the node could not be located
    there, and the failure read like a missing file:

        ERROR: cannot launch node of type [ssd_detection/ssd_detection_node.py]

    Anything that reads these expressions has to convert the same way, or it
    certifies the switch that the author meant instead of the one roslaunch
    evaluates. (A string literal in the expression stays a string - only arg()
    is converted. That asymmetry is the whole trap.)
    """
    if value is None:
        return None
    try:
        return float(value) if "." in value else int(value)
    except (TypeError, ValueError):
        pass
    low = str(value).lower()
    if low == "true":
        return True
    if low == "false":
        return False
    return value


def _resolve_arg_expr(root, text, override=None, _depth=0):
    """Resolve an argument default written as $(arg x) or as one $(eval ...).

    The configuration this file guards deliberately writes its defaults as
    expressions, so that one switch cannot drift away from the things it has
    to drive (use_usb_cam drives both input_image_topic and which detector
    implementation is included). Reading such a value literally therefore says
    nothing about the run - and worse, it silently degrades into "the value
    starts with $(, give up", which is how this check used to read them.

    Two expression shapes are understood, both of the form

        $(eval <YES> if arg('<name>') else <NO>)
        $(eval <YES> if arg('<name>') == '<value>' else <NO>)

    where each branch is either an argument reference, arg('x'), or a quoted
    literal. The condition is evaluated the way roslaunch evaluates it, i.e.
    with arg() type-converted (see _roslaunch_convert_auto); that is the point
    of this function, and the reason the second shape is reported rather than
    quietly resolved. Anything else is handed back unchanged, which is the
    honest answer and keeps an unparseable value visible to whatever asked.

    `override` pins a switch to a chosen value first, so a check can ask "what
    does this become when the operator passes use_usb_cam:=true?" without
    editing the launch file.
    """
    if text is None:
        return None
    text = text.strip()
    if _depth > 8:
        return text

    match = _ARG_REF_RE.match(text)
    if match:
        name = match.group(1) or match.group(2)
        if override and name in override:
            return override[name]
        inner = _launch_arg(root, name, None)
        if inner is None:
            return text
        return _resolve_arg_expr(root, inner, override, _depth + 1)

    match = _ARG_EVAL_RE.match(text)
    if match:
        name = match.group("name")
        raw = override.get(name) if override and name in override else \
            _launch_arg(root, name, "")
        converted = _roslaunch_convert_auto(raw)
        if match.group("value") is None:
            take_yes = bool(converted)
        else:
            # The literal on the right is NOT converted - it is a Python string
            # in the eval'd expression. Comparing a converted arg to it is the
            # dead-switch trap; resolving it faithfully keeps it dead here too
            # instead of pretending it works.
            take_yes = converted == match.group("value")
        branch = (match.group("yes") if take_yes else match.group("no")).strip()
        literal = _UNQUOTE_RE.match(branch)
        if literal:
            return literal.group(1)
        return _resolve_arg_expr(root, branch, override, _depth + 1)

    return text


def _launch_node_types(path):
    """Every <node type="..."/> in a launch file, without importing roslaunch."""
    import xml.etree.ElementTree as ET
    return [n.get("type") for n in ET.parse(path).getroot().iter("node")
            if n.get("type")]


def _strip_cpp_comments(text):
    """Drop // and /* */ comments while leaving string and char literals intact.

    Needed because the checks below look for the *absence* of trigonometry in a
    function body, and every one of those functions now carries a comment
    explaining what used to be there. Matching raw text would either fire on the
    explanation or force the explanation to be deleted - both wrong.
    """
    out = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if ch == "/" and nxt == "/":
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif ch == "/" and nxt == "*":
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
        elif ch in ("'", '"'):
            quote = ch
            out.append(ch)
            i += 1
            while i < n:
                if text[i] == "\\" and i + 1 < n:
                    out.append(text[i:i + 2])
                    i += 2
                    continue
                out.append(text[i])
                if text[i] == quote:
                    i += 1
                    break
                i += 1
        else:
            out.append(ch)
            i += 1
    return "".join(out)


def _cpp_function_body(text, name):
    """Brace-matched body of a function definition, comments already stripped.

    Two spellings have to work: 'Class::name(...)' for a definition in a .cpp
    with an out-of-line body, and a bare 'void name(...)' inside a class body
    (ego_trajectory_controller.cpp defines its node entirely inline). A call
    site cannot be mistaken for either, because both patterns require a return
    type or a scope qualifier immediately before the name.

    Returns None when the definition is not found, so the caller can fail loudly
    instead of silently checking nothing.
    """
    text = _strip_cpp_comments(text)
    pos = -1
    match = re.search(r"::\s*" + re.escape(name) + r"\s*\(", text)
    if match:
        pos = match.start()
    else:
        match = re.search(r"\b(?:void|bool|int|double|float|size_t)\s+" +
                          re.escape(name) + r"\s*\(", text)
        if match:
            pos = match.start()
    if pos < 0:
        return None
    brace = text.find("{", pos)
    if brace < 0:
        return None
    semicolon = text.find(";", pos, brace)
    if semicolon >= 0:
        # 'name(...);' is a declaration or a call, not a definition.
        return None
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace:i + 1]
    return None


def _launch_files():
    """Every .launch under src/, so a resurrected switch cannot hide."""
    found = []
    for dirpath, _dirnames, filenames in os.walk(os.path.join(ROOT, "src")):
        for name in filenames:
            if name.endswith(".launch"):
                found.append(os.path.join(dirpath, name))
    return sorted(found)


def check_cmd_vel_convention():
    """There is exactly one rotation in the chain, and this gate keeps it that way.

    What went wrong (2026-09 / 2026-10, the reported left-right sway): /cmd_vel
    used to change frames twice. ego_trajectory_controller rotated plan -> body
    using yaw(odom -> base_link) - for the SITL rig published from Gazebo ground
    truth - and rescue_mission rotated body -> world using current_yaw_, which is
    PX4's EKF attitude from /mavros/local_position/pose. Two mutually inverse
    rotations taken from two DIFFERENT estimators do not cancel; the residual is
    the disagreement between them, applied to the velocity vector. The mission
    could not see the resulting push in its own error term, because it pins its
    position setpoint to the vehicle's measured position (zero position error)
    and drives with velocity alone.

    The fix was to delete the switches, not to tune them: output_frame and
    cmd_vel_frame no longer exist, both nodes speak the plan frame, and one
    rotation disappeared. That is a property of the code, so this gate asserts it
    structurally rather than by comparing two parameters that could disagree:

      1. No launch file under src/ declares output_frame, cmd_vel_frame,
         planner_backend or vision_backend any more. A switch that cannot be
         declared cannot be set to the wrong value.
      2. Neither end of /cmd_vel contains trigonometry in the function that
         touches it. A rotation needs sin/cos; if there is none, there is no
         rotation to double.
      3. The passthrough assignments still exist, so "no rotation" is not
         achieved by publishing nothing at all.

    Also checked: lookahead_distance. A pure-pursuit follower steers by
    atan(cross_track / lookahead), so a lookahead shorter than about one second
    of travel turns centimetres of tracking lag into degrees of heading change -
    the other half of the sway.
    """
    heading(9, "cmd_vel: one frame, no switch, and a lookahead that can steer")

    import xml.etree.ElementTree as ET

    # (1) the switches must not come back
    forbidden = ("output_frame", "cmd_vel_frame", "cmd_vel_in_odom",
                 "planner_backend", "vision_backend")
    launch_count = 0
    for path in _launch_files():
        launch_count += 1
        try:
            root = ET.parse(path).getroot()
        except Exception as exc:
            fail("cmdvel", path, "cannot parse: %s" % exc)
            continue
        for tag in ("arg", "param"):
            for node in root.iter(tag):
                name = node.get("name") or ""
                if name in forbidden:
                    fail("cmdvel", path,
                         "<%s name=%r> is back. The frame/backend switches were "
                         "deleted on 2026-10-17 on purpose: each one was a silent "
                         "way to make the vehicle fly sideways or see nothing. "
                         "If a new option is genuinely needed, it has to come with "
                         "the check that catches its wrong setting."
                         % (tag, name))
    print("  %d launch file(s) scanned, none declares %s"
          % (launch_count, "/".join(forbidden)))

    sources = {
        "mission": os.path.join(ROOT, "src/navigation_2d/src/rescue_mission.cpp"),
        "follower": os.path.join(ROOT, "src/navigation_2d/src/ego_trajectory_controller.cpp"),
    }
    functions = {"mission": "plannerVelocityCallback", "follower": "controlCallback"}
    required = {
        "mission": ["planner_velocity_.x = msg->linear.x",
                    "planner_velocity_.y = msg->linear.y"],
        "follower": ["command.linear.x = map_vx", "command.linear.y = map_vy"],
    }
    for role, path in sources.items():
        if not os.path.exists(path):
            fail("cmdvel", path, "not found")
            continue
        raw = io.open(path, encoding="utf-8").read()
        body = _cpp_function_body(raw, functions[role])
        if body is None:
            fail("cmdvel", path,
                 "cannot find %s::%s(); this gate has nothing to check and a "
                 "gate that checks nothing must not report OK"
                 % (os.path.basename(path), functions[role]))
            continue
        # (2) no rotation can exist without trigonometry
        trig = [fn for fn in ("cos(", "sin(", "atan2(") if fn in body]
        if trig:
            fail("cmdvel", path,
                 "%s::%s now calls %s. Exactly one rotation used to exist in the "
                 "whole /cmd_vel chain and it was removed on 2026-10-17 together "
                 "with output_frame/cmd_vel_frame; re-introducing a conversion "
                 "here means the other end has to convert back, and the two "
                 "attitude estimates it would use are not the same estimator. "
                 "Publish the plan frame and let one node own the geometry."
                 % (os.path.basename(path), functions[role], ", ".join(trig)))
        # (3) the passthrough is really there
        missing = [snippet for snippet in required[role]
                   if snippet.replace(" ", "") not in body.replace(" ", "")]
        if missing:
            fail("cmdvel", path,
                 "%s::%s no longer contains %r. The one-rotation arrangement was "
                 "not removed, it was replaced by nothing - the velocity that "
                 "leaves this node is not the velocity that entered it."
                 % (os.path.basename(path), functions[role], missing[0]))
        if not trig and not missing:
            print("  %s::%-24s no rotation, %d passthrough assignment(s) present"
                  % (os.path.basename(path).ljust(30), functions[role],
                     len(required[role])))

    # The lookahead half still reads the live launch file, because that number is
    # a tuning value rather than a structural property - it can legitimately be
    # changed, and the only thing worth asserting is the ratio it implies.
    launch = os.path.join(ROOT, "src", "navigation_2d", "launch", "navigation_2d.launch")
    try:
        root = ET.parse(launch).getroot()
    except Exception as exc:
        fail("cmdvel", launch, "cannot parse: %s" % exc)
        return

    lookahead = _launch_param(root, "ego_trajectory_controller_node", "lookahead_distance")
    max_vel = _launch_param(root, "ego_trajectory_controller_node", "max_velocity")
    planning_rate = _launch_param(root, "ego_planner_2d_node", "planning_rate")
    try:
        lookahead = float(lookahead)
        max_vel = float(max_vel)
        planning_rate = float(planning_rate)
    except (TypeError, ValueError):
        warn("cmdvel", launch, "lookahead_distance/max_velocity/planning_rate not numeric; skipped")
        return

    if lookahead < max_vel / max(planning_rate, 1e-6):
        fail("cmdvel", launch,
             "lookahead_distance=%.2f m is shorter than the %.2f m the vehicle "
             "covers in one planning period (%.2f m/s at %.1f Hz). The pursuit "
             "target can end up behind the vehicle between two plans, which makes "
             "the command reverse - a hard left-right oscillation."
             % (lookahead, max_vel / planning_rate, max_vel, planning_rate))
    elif lookahead < max_vel:
        warn("cmdvel", launch,
             "lookahead_distance=%.2f m is shorter than one second of travel at "
             "max_velocity=%.2f m/s. Steer angle is atan(cross_track / lookahead), "
             "so at %.2f m a 0.05 m tracking lag already commands %.0f deg of "
             "heading change and the vehicle zig-zags. Pure pursuit normally wants "
             "roughly 1 s of travel - about %.2f m here."
             % (lookahead, max_vel, lookahead,
                math.degrees(math.atan2(0.05, max(lookahead, 1e-6))), max_vel))
    else:
        print("  lookahead_distance=%.2f m (%.2f s of travel at %.2f m/s), "
              "planning_rate=%.1f Hz"
              % (lookahead, lookahead / max(max_vel, 1e-6), max_vel, planning_rate))


def _world_model_poses(path):
    """name -> (x, y) for every model in the world file that has a full pose."""
    import xml.etree.ElementTree as ET

    if not os.path.exists(path):
        return {}
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return {}
    out = {}
    for model in root.iter("model"):
        name = model.get("name")
        pose = model.find("pose")
        if not name or pose is None:
            continue
        vals = [float(v) for v in (pose.text or "").split()]
        if len(vals) >= 2:
            out[name] = (vals[0], vals[1])
    return out


def check_stage_mission():
    """The two competition rounds, the drawn drop position, and the tour.

    Round 1 drops the fire material at the 危险品任务区, flies over every
    人员救助任务区 and comes home. Round 2 does the same and adds the rescue
    material at a position drawn by lot on the day, which therefore cannot be
    compiled into a route table - it arrives as a command-line parameter.

    What that makes checkable, and what this check does:

      * `stage` and `drop_align_mode` must be values the node understands. A typo
        in either one is not an error anywhere in ROS; it silently changes how
        the flight behaves, which is the worst kind of wrong for a value that is
        set by hand minutes before a round.
      * A stage-2 launch must carry a drop position inside the enclosure. The
        launch default is a sentinel outside the field precisely so that
        forgetting it is loud, and this is where it becomes loud.
      * The drop position is a place a 1.2 m hover has to be reachable in, so it
        gets the same clearance test as the route waypoints in check 8.
      * Every 人员救助任务区 in the world must have a person_route waypoint over
        it. "Fly over all of them before coming home" is a requirement about the
        route, and the route used to stop at the first card.
      * The last hazard_route waypoint must sit on a hazard card, because that
        point is both the final goal and the reference the waypoint-drop fallback
        uses. The mission then drops on whichever detection is nearest the image
        centre (since 2026-10-17; it used to pick by index, which nobody could
        verify without remembering the detector's ordering), so the hover point
        being over the intended card is the whole premise - it is printed here
        rather than asserted, because only a hover image can confirm that the
        neighbouring cards stay out of the crop.
    """
    import xml.etree.ElementTree as ET

    heading(10, "competition stage, drop point and person task-area coverage")

    launch = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if not os.path.exists(launch):
        warn("stage", launch, "not found; skipped")
        return
    try:
        root = ET.parse(launch).getroot()
    except Exception as exc:
        fail("stage", launch, "cannot parse: %s" % exc)
        return

    # ---- stage and drop mode ------------------------------------------------
    raw_stage = _launch_arg(root, "stage", None)
    if raw_stage is None:
        fail("stage", launch,
             "no 'stage' argument. Round 1 and round 2 differ by a whole drop, so "
             "the round must be explicit rather than a default that was never "
             "looked at.")
        stage = None
    else:
        try:
            stage = int(float(raw_stage))
        except ValueError:
            stage = None
            fail("stage", launch, "stage='%s' is not a number" % raw_stage)
        else:
            if stage not in (1, 2):
                fail("stage", launch,
                     "stage=%d is neither 1 (hazard drop only) nor 2 (hazard drop "
                     "+ rescue-material drop)" % stage)

    mode = _launch_arg(root, "drop_align_mode", None)
    if mode is None:
        fail("stage", launch, "no 'drop_align_mode' argument")
    elif mode not in ("vision", "waypoint", "vision_with_fallback"):
        fail("stage", launch,
             "drop_align_mode='%s' is not one of vision / waypoint / "
             "vision_with_fallback; the node would fall back to "
             "vision_with_fallback with a warning, which is not what this run "
             "asked for" % mode)
    else:
        print("  stage=%s, drop_align_mode=%s" % (stage, mode))

    # ---- the mission node must actually receive the arguments ---------------
    forwarded = {}
    for name in ("stage", "rescue_drop_x", "rescue_drop_y", "drop_align_mode",
                 "field_yaw_deg"):
        value = _launch_param(root, "rescue_mission_node", name)
        if value is not None:
            # The node receives '$(arg name)'; what the run actually flies is the
            # argument's value, which is what everything below has to judge.
            value = _resolve_arg_text(root, value)
        forwarded[name] = value
        if value is None:
            fail("stage", launch,
                 "rescue_mission_node does not receive '%s'; the argument would be "
                 "set and then ignored" % name)

    # ---- the drawn drop position -------------------------------------------
    def as_float(text, name):
        try:
            return float(text)
        except (TypeError, ValueError):
            fail("stage", launch, "%s is not a number (%r)" % (name, text))
            return None

    drop_x = as_float(forwarded.get("rescue_drop_x"), "rescue_drop_x")
    drop_y = as_float(forwarded.get("rescue_drop_y"), "rescue_drop_y")

    # ---- the route frame ----------------------------------------------------
    #
    # The route tables in navigation_2d.launch are FIELD-frame numbers, while the
    # mission publishes its goals stamped "odom". Those are the same frame only in
    # the simulation, where px4_sensor_rig_node derives odom -> base_link from
    # Gazebo ground truth and the iris spawns with zero yaw. On the vehicle odom
    # is cartographer's own frame and its yaw is whatever heading the airframe had
    # when tracking started, so one hand-set angle decides where the whole route
    # lands. That angle is worthless if the launch declares it and then forgets to
    # forward it: the node keeps its own 0.0 default and the flight is rotated
    # with no hint in the launch file, which is exactly how the servo bench lost a
    # session to `angle_debug`. The forwarding itself is covered by the loop
    # above; what is checked here is that the value is a heading. A missing
    # forward is already reported by that loop, so this stays quiet about it.
    raw_field_yaw = forwarded.get("field_yaw_deg")
    if raw_field_yaw is not None:
        field_yaw = as_float(raw_field_yaw, "field_yaw_deg")
        if field_yaw is not None and not -180.0 <= field_yaw <= 180.0:
            fail("stage", launch,
                 "field_yaw_deg=%s is outside (-180, 180]. The node folds it into "
                 "that range, but a whole turn of disagreement is a typo rather "
                 "than a heading." % field_yaw)
        elif field_yaw is not None and abs(field_yaw) < 1e-6:
            print("  field_yaw_deg=0: odom and the field frame are taken to be the "
                  "same frame, which is the simulation's arrangement")
        elif field_yaw is not None:
            print("  field_yaw_deg=%s: the airframe's heading in the FIELD frame at "
                  "boot, so every field-frame route point is published rotated by "
                  "%s into odom. Verify the rotated safety field against the map "
                  "before arming; the node prints its bounding box at startup." %
                  (field_yaw, -field_yaw))

    min_x = float(_launch_param(root, "rescue_mission_node", "safe_field_min_x", "-0.60") or -0.60)
    max_x = float(_launch_param(root, "rescue_mission_node", "safe_field_max_x", "3.30") or 3.30)
    min_y = float(_launch_param(root, "rescue_mission_node", "safe_field_min_y", "-3.30") or -3.30)
    max_y = float(_launch_param(root, "rescue_mission_node", "safe_field_max_y", "0.60") or 0.60)

    drop_inside = (drop_x is not None and drop_y is not None and
                   min_x <= drop_x <= max_x and min_y <= drop_y <= max_y)
    if stage is not None and stage >= 2 and not drop_inside:
        fail("stage", launch,
             "stage 2 always drops the rescue material, but the drop position "
             "(%s, %s) is not inside the field (x %.2f..%.2f, y %.2f..%.2f). The "
             "node refuses to take off in this state; pass the position drawn by "
             "lot as rescue_drop_x / rescue_drop_y, e.g. rescue_drop_x:=2.70 "
             "rescue_drop_y:=-1.85."
             % (drop_x, drop_y, min_x, max_x, min_y, max_y))
    elif drop_inside:
        print("  drawn drop position (%.2f, %.2f), inside the field" % (drop_x, drop_y))
    elif stage == 1 or stage is None:
        print("  no drawn drop position (stage 1 does not use one)")
    else:
        print("  drawn drop position not supplied yet; stage-2 runs will refuse to start")

    # ---- coordinates in the world file the routes must agree with -----------
    world = os.path.join(ROOT, "src/gc_2026_sim/gazebo/worlds/gc_2026_field.world")
    models = _world_model_poses(world)
    if not models:
        warn("stage", world, "no models with poses; route coverage not checked")
        return

    person_route = _launch_rosparam(root, "rescue_mission_node", "person_route")
    person_cards = sorted(n for n in models if n.startswith("person_zone_card"))
    if not person_cards:
        warn("stage", world, "no person_zone_card_* models; coverage not checked")
    else:
        if not person_route:
            fail("stage", launch,
                 "%d person task areas exist in the world but person_route is "
                 "empty; the aircraft would never pass over any of them"
                 % len(person_cards))
        else:
            for card in person_cards:
                cx, cy = models[card]
                nearest = min(math.hypot(wx - cx, wy - cy) for wx, wy in person_route)
                if nearest > 0.10:
                    fail("stage", launch,
                         "%s is at (%.2f, %.2f) but the nearest person_route "
                         "waypoint is %.3f m away. The flight has to pass over "
                         "every person task area, and a card that moves without "
                         "its waypoint is exactly how one gets skipped."
                         % (card, cx, cy, nearest))
                else:
                    print("  %-20s (%.2f, %.2f) covered by person_route (%.3f m from "
                          "the nearest waypoint)" % (card, cx, cy, nearest))

    hazard_route = _launch_rosparam(root, "rescue_mission_node", "hazard_route")
    hazard_cards = sorted(n for n in models if n.startswith("hazard_zone_card"))
    if hazard_route and hazard_cards:
        last_x, last_y = hazard_route[-1]
        best = None
        for card in hazard_cards:
            cx, cy = models[card]
            d = math.hypot(last_x - cx, last_y - cy)
            if best is None or d < best[1]:
                best = (card, d)
        if best[1] > 0.10:
            fail("stage", launch,
                 "the last hazard_route waypoint (%.2f, %.2f) is %.3f m from the "
                 "nearest hazard card ('%s'). That waypoint is both the final goal "
                 "and the reference the waypoint-drop fallback drops on, so it has "
                 "to be the card the detector is pointed at."
                 % (last_x, last_y, best[1], best[0]))
        else:
            # The mission no longer picks a detection by index: since 2026-10-17
            # MissionVision selects the candidate closest to the image centre,
            # which - with the camera pointing down and the aircraft hovering
            # over the target - is the one closest to the aircraft. What this
            # line still checks is the premise that makes that work: the hover
            # point is over the card, so the nearest detection IS the right one.
            print("  last hazard_route waypoint (%.2f, %.2f) is on '%s' (%.3f m); "
                  "the detector's nearest-to-centre pick is that card as long as "
                  "the hover leaves it the only one in frame"
                  % (last_x, last_y, best[0], best[1]))

    if not drop_inside:
        return

    # ---- is the drop position somewhere a hover can be? ---------------------
    task_height = float(_launch_arg(root, "task_height", "1.2"))
    footprints = _world_footprints(task_height)
    if not footprints:
        warn("stage", world, "no footprints at task height; drop clearance not checked")
        return
    resolution = float(_launch_param(root, "ego_planner_2d_node", "map_resolution", "0.10") or 0.10)
    inflate = float(_launch_param(root, "ego_planner_2d_node", "map_inflate_value", "0.35") or 0.35)
    threshold = inflate + resolution / 2.0

    nearest, clearance = None, None
    for name, cx, cy, yaw, geom in footprints:
        clear = _footprint_clearance(drop_x, drop_y, cx, cy, yaw, geom)
        if clearance is None or clear < clearance:
            nearest, clearance = name, clear
    if clearance is None:
        return
    if clearance < threshold:
        fail("stage", launch,
             "the drawn drop position (%.2f, %.2f) is only %.4f m from '%s', below "
             "inflate + resolution/2 = %.4f m. The hover has nowhere to sit, so the "
             "rescue drop could not be flown even though it was drawn there - move "
             "the position or lower map_inflate_value."
             % (drop_x, drop_y, clearance, nearest, threshold))
    else:
        print("  drop position clearance %.4f m to '%s' (needs >= %.4f m)"
              % (clearance, nearest, threshold))


# ------------------------------------------------- 11: world model resources
# Gazebo resolves model://<name> by searching GAZEBO_MODEL_PATH.  A name it
# cannot find there is offered to the ONLINE MODEL DATABASE instead, and when
# that database is unreachable or disabled (start_px4_slam.sh sets
# GAZEBO_MODEL_DATABASE_URI="") gzserver simply never finishes loading the
# world.  Nothing fails loudly - the GUI sits at "Preparing your world" and the
# ROS log shows no error at all.
#
# 2026-09-30: gc_2026_field.world started referencing model://gc_task_markers
# for the task-zone material script and card textures.  The model exists at
# src/gc_2026_sim/gazebo/models/gc_task_markers, but the only launch file that
# exported GAZEBO_MODEL_PATH was gazebo_field.launch - and start_px4_slam.sh
# runs px4_gazebo_slam.launch, which includes gazebo_ros/empty_world.launch
# directly.  gazebo_field.launch is not on that path, so gzserver never learned
# where the model was.  The world loaded fine before only because the single
# model:// it used then was model://sun.
#
# A <env> applies to every node in its own launch file AND to everything that
# launch includes, so the search below runs UPWARD: a launch is covered if it,
# or any launch that transitively includes it, exports GAZEBO_MODEL_PATH.

# Shipped with Gazebo itself, so they resolve without any project model path.
GAZEBO_BUILTIN_MODELS = {"sun", "ground_plane"}

_ENV_TAG = re.compile(r"<env\b[^>]*>")
_ATTR = r'%s="([^"]*)"'


def _read_text(path):
    return open(path, encoding="utf-8", errors="replace").read()


def _package_dirs():
    """ROS package name -> source directory, from src/*/package.xml"""
    dirs = {}
    for pkg in glob.glob(os.path.join(ROOT, "src/*/package.xml")):
        m = re.search(r"<name>\s*([^<\s]+)\s*</name>", _read_text(pkg))
        if m:
            dirs[m.group(1).strip()] = os.path.dirname(pkg)
    return dirs


def _launch_includes(path, packages):
    """launch files this one includes, resolved to real paths"""
    out = set()
    for m in re.finditer(r'<include\s+file="([^"]*)"', _read_text(path)):
        target = m.group(1)
        g = re.search(r"\$\(find\s+([^)\s]+)\)", target)
        if not g:
            continue
        pkg = g.group(1).strip()
        rel = target[g.end():].lstrip("/")
        if pkg in packages and rel:
            out.add(os.path.normpath(os.path.join(packages[pkg], rel)))
    return out


def _launch_model_paths(path):
    """GAZEBO_MODEL_PATH values exported by one launch file"""
    values = []
    for tag in _ENV_TAG.findall(_read_text(path)):
        name = re.search(_ATTR % "name", tag)
        value = re.search(_ATTR % "value", tag)
        if name and value and name.group(1) == "GAZEBO_MODEL_PATH":
            values.append(value.group(1))
    return values


def check_world_resources():
    heading(11, "world model:// resources vs GAZEBO_MODEL_PATH")
    packages = _package_dirs()
    launches = sorted(glob.glob(os.path.join(ROOT, "src/**/*.launch"), recursive=True))
    worlds = sorted(glob.glob(os.path.join(ROOT, "src/**/*.world"), recursive=True))

    # launch -> every launch that includes it (env travels downward, so we walk up)
    includers = {l: set() for l in launches}
    for loader in launches:
        for child in _launch_includes(loader, packages):
            if child in includers:
                includers[child].add(loader)

    models = {}
    for d in glob.glob(os.path.join(ROOT, "src/**/models/*"), recursive=True):
        if os.path.isdir(d):
            models[os.path.basename(d)] = d

    problems = 0
    checked = 0
    for world in worlds:
        rel = os.path.relpath(world, ROOT)
        text = _read_text(world)
        if "<sdf" not in text:
            fail("world-res", world,
                 "does not look like an SDF world file (no <sdf> element, %d bytes); "
                 "gzserver would report nothing useful" % os.path.getsize(world))
            problems += 1
            continue
        uris = sorted(set(re.findall(r"model://([A-Za-z0-9_\-]+)", text)))
        if not uris:
            # Never let this pass silently: a world that references no model at
            # all is almost always a truncated or clobbered file, and gzserver
            # would say nothing useful about it.
            warn("world-res", world,
                 "no model:// references at all (%d bytes); is the file empty or "
                 "truncated?" % os.path.getsize(world))
            continue
        project = [u for u in uris if u not in GAZEBO_BUILTIN_MODELS]

        for name in uris:
            if name in GAZEBO_BUILTIN_MODELS:
                continue
            d = models.get(name)
            if d is None:
                fail("world-res", world,
                     "model://%s is not under any src/**/models/ directory in this "
                     "workspace; gazebo would offer the URI to the online model "
                     "database and the world would never finish loading" % name)
                problems += 1
            elif not os.path.exists(os.path.join(d, "model.config")):
                fail("world-res", world,
                     "model://%s resolves to %s but that directory has no "
                     "model.config, so gazebo will not recognise it as a model"
                     % (name, os.path.relpath(d, ROOT)))
                problems += 1

        if not project:
            print("  %-52s only gazebo built-ins (%s)" % (rel, ", ".join(uris)))
            continue

        # Every launch that loads this world must be able to resolve the models.
        # Match the world name on an <arg name="world"> / <arg name="world_name">
        # line only: a bare mention in a comment is not a loader, and treating one
        # as such both over-reports and hides the real entry point.
        basename = os.path.basename(world)
        loader_re = re.compile(r'<arg\s+name="world(?:_name)?"[^>]*%s' % re.escape(basename))
        loaders = [l for l in launches if loader_re.search(_read_text(l))]
        if not loaders:
            warn("world-res", world, "no launch file mentions %s" % basename)
            continue

        covered_by = {}
        for loader in loaders:
            checked += 1
            frontier = [loader]
            seen = set()
            found = None
            while frontier:
                cur = frontier.pop()
                if cur in seen:
                    continue
                seen.add(cur)
                values = _launch_model_paths(cur)
                if values:
                    found = (cur, values[0])
                    break
                frontier.extend(includers.get(cur, ()))
            if found is None:
                fail("world-res", loader,
                     "loads %s, which needs %s, but neither it nor any launch that "
                     "includes it exports GAZEBO_MODEL_PATH - gzserver will stall at "
                     "'Preparing your world'"
                     % (basename, ", ".join("model://%s" % u for u in project)))
                problems += 1
            else:
                print("  %-52s via %s" % (os.path.relpath(loader, ROOT),
                                          os.path.relpath(found[0], ROOT)))
                covered_by[found[0]] = found[1]
        for exp, value in sorted(covered_by.items()):
            if "$(optenv" not in value:
                warn("world-res", exp,
                     "GAZEBO_MODEL_PATH REPLACES the inherited value; append "
                     "\"$(optenv GAZEBO_MODEL_PATH)\" so a PX4-sourced shell keeps "
                     "its own model directories")
    if not problems:
        print("  %d world(s), %d loader launch(es), %d local model(s)"
              % (len(worlds), checked, len(models)))


def check_servo():
    """Drop servo angles, and the PCA9685 settings every angle is computed from.

    servo_controller_node accepts only four fixed command strings and resolves
    each one to a parameter, so the numbers in navigation_2d.launch are the only
    thing that decides which container releases.  The node does validate them -
    but it prints WARN and flies anyway, and ServoController::setAngle() clamps
    to 0..180 before computing the pulse width, so an angle outside that range
    silently moves the servo somewhere other than where the parameter says.
    Failing on the ground costs a rebuild; failing in the air costs a drop.

    Deliberately mirrors, and must stay in step with,
    ServoController::ServoController() and ServoController::setAngle() in
    src/servo_control/src/servo_controller_node.cpp.  The tick arithmetic here
    is the one cross-checked against the C++ by
    tools/servo_tick_check.cpp / tools/servo_angle_calib.py.

    Also covers two things the node cannot see from inside itself:
      * the mission log in rescue_mission.cpp::beginDrop hard-codes "(servo 1
        -> ...)", so a drop_channel other than 1 makes that line lie;
      * restore_after_drop must stay false, or after the fire drop the servo
        sweeps back across the rescue angle while the aircraft is still over
        the hazard task area.

    And one thing the launch must NOT carry: angle_debug.  It opens
    /servo_debug_cmd, the bench-only entry that accepts an arbitrary angle; the
    node leaves that topic unsubscribed while the parameter is false, and the
    whole point of the four fixed commands is that a stray message cannot move
    the drop servo.  True here is a FAIL, not a warning - the obvious thing to
    paste into a flight launch is exactly that.  It cannot be switched on from
    the command line either: this launch does not declare it as an <arg>, so
    `angle_debug:=true` is discarded without a message.  Bench use goes through
    src/servo_control/launch/servo_bench.launch instead, which keeps the flag
    out of the flight chain entirely.
    """
    heading(12, "drop servo angles, PCA9685 settings and the bench entry")
    import xml.etree.ElementTree as ET

    launch = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if not os.path.exists(launch):
        warn("servo", ROOT, "navigation_2d.launch not found; servo checks skipped")
        return
    try:
        root = ET.parse(launch).getroot()
    except Exception as exc:
        fail("servo", launch, "cannot parse: %s" % exc)
        return

    node_type = "servo_controller_node"

    def number(name):
        raw = _launch_param(root, node_type, name)
        raw = _resolve_arg_text(root, raw)
        if raw is None:
            return None
        try:
            return float(raw)
        except ValueError:
            fail("servo", launch,
                 "param %s = %r is not a number" % (name, raw))
            return None

    def integer(name):
        value = number(name)
        return None if value is None else int(value)

    # The node keys its own validation on these, and FATALs on the ones below.
    i2c_address = number("i2c_address")
    camera_channel = integer("camera_channel")
    drop_channel = integer("drop_channel")
    pwm_frequency = number("pwm_frequency")
    min_pulse_us = number("min_pulse_us")
    max_pulse_us = number("max_pulse_us")

    front_angle = number("front_angle")
    down_angle = number("down_angle")
    drop_safe_angle = number("drop_safe_angle")
    fire_drop_angle = number("fire_drop_angle")
    rescue_drop_angle = number("rescue_drop_angle")
    restore_after_drop = _resolve_arg_text(
        root, _launch_param(root, node_type, "restore_after_drop"))

    if fire_drop_angle is None or rescue_drop_angle is None:
        fail("servo", launch,
             "fire_drop_angle / rescue_drop_angle missing: the two drops cannot "
             "be distinguished without them")
        return

    enabled = _launch_arg(root, "use_real_servos", "true")
    print("  servo_controller_node is started by default: use_real_servos=%s" % enabled)
    print("  i2c %s addr 0x%02x  ch camera=%s drop=%s  %g Hz  pulse %.0f..%.0f us"
          % (_launch_param(root, node_type, "i2c_device"),
             int(i2c_address) if i2c_address is not None else -1,
             camera_channel, drop_channel, pwm_frequency or 0,
             min_pulse_us or 0, max_pulse_us or 0))

    # ---- the settings the node refuses to start on -------------------------
    if i2c_address is not None and not (0x03 <= i2c_address <= 0x77):
        fail("servo", launch,
             "i2c_address %g is outside 0x03..0x77; the node FATALs and refuses "
             "to drive any servo" % i2c_address)
    if camera_channel is not None and drop_channel is not None \
            and camera_channel == drop_channel:
        fail("servo", launch,
             "camera_channel == drop_channel == %d; the node FATALs, and on the "
             "hardware a drop command would move the camera" % drop_channel)
    if pwm_frequency is not None and not (40 <= pwm_frequency <= 1000):
        fail("servo", launch,
             "pwm_frequency %g Hz is outside 40..1000; the node FATALs"
             % pwm_frequency)
    if min_pulse_us is not None and max_pulse_us is not None \
            and not (0 < min_pulse_us < max_pulse_us):
        fail("servo", launch,
             "pulse range %.0f..%.0f us is empty; the node FATALs"
             % (min_pulse_us, max_pulse_us))

    # ---- the angles --------------------------------------------------------
    # The node validates safe/fire/rescue but not the two camera angles, so a
    # camera angle outside 0..180 still gets silently clamped by setAngle.
    drop_angles = [("drop_safe_angle", drop_safe_angle),
                   ("fire_drop_angle", fire_drop_angle),
                   ("rescue_drop_angle", rescue_drop_angle)]
    for name, value in drop_angles:
        if value is None:
            continue
        if not (0.0 <= value <= 180.0):
            fail("servo", launch,
                 "%s %g is outside 0..180; setAngle clamps, so the servo would "
                 "not go where the parameter says" % (name, value))
    for name, value in [("front_angle", front_angle), ("down_angle", down_angle)]:
        if value is None:
            continue
        if not (0.0 <= value <= 180.0):
            warn("servo", launch,
                 "%s %g is outside 0..180; setAngle clamps it silently (the node "
                 "does not validate the camera angles at all)" % (name, value))

    separation = abs(fire_drop_angle - rescue_drop_angle)
    if separation < 5.0:
        fail("servo", launch,
             "fire_drop_angle (%g) and rescue_drop_angle (%g) are only %g deg "
             "apart; the mechanism cannot tell the two materials apart, so one "
             "of the two drops would release the wrong container"
             % (fire_drop_angle, rescue_drop_angle, separation))
    else:
        print("  fire %.1f  rescue %.1f  separation %.1f deg (need >= 5.0)"
              % (fire_drop_angle, rescue_drop_angle, separation))

    if drop_safe_angle is not None:
        if abs(drop_safe_angle - fire_drop_angle) < 5.0 or \
                abs(drop_safe_angle - rescue_drop_angle) < 5.0:
            warn("servo", launch,
                 "drop_safe_angle (%g) is within 5 deg of a drop angle (fire %g, "
                 "rescue %g); the parked position would already be a release "
                 "position" % (drop_safe_angle, fire_drop_angle,
                               rescue_drop_angle))

    # ---- the pulse widths those angles become -------------------------------
    # Same expression as setAngle; a pulse the PCA9685 cannot express (ticks
    # outside 1..4095) would wrap the compare register instead of erroring.
    if (pwm_frequency and min_pulse_us and max_pulse_us is not None
            and max_pulse_us > min_pulse_us):
        for name, value in drop_angles + [("front_angle", front_angle),
                                          ("down_angle", down_angle)]:
            if value is None:
                continue
            angle = max(0.0, min(180.0, value))
            pulse_us = min_pulse_us + (max_pulse_us - min_pulse_us) * angle / 180.0
            ticks = int(math.floor(
                pulse_us * pwm_frequency * 4096.0 / 1000000.0 + 0.5))
            if not (1 <= ticks <= 4095):
                fail("servo", launch,
                     "%s %g -> %d ticks, outside 1..4095; the compare register "
                     "wraps and the servo gets a pulse that is not the one "
                     "computed" % (name, value, ticks))

    # ---- two things the node cannot see from inside itself ------------------
    if drop_channel is not None and drop_channel != 1:
        warn("servo", launch,
             "drop_channel is %d, but rescue_mission.cpp::beginDrop logs "
             "\"(servo 1 -> ...)\" verbatim, so the log would name the wrong "
             "channel" % drop_channel)
    if str(restore_after_drop).strip().lower() == "true":
        warn("servo", launch,
             "restore_after_drop is true: after the fire drop the servo sweeps "
             "back across the rescue angle while the aircraft is still over the "
             "hazard task area. The launch comment says it must stay false.")

    # ---- the bench escape hatch has to be shut for a flight -----------------
    # servo_controller_node grew /servo_debug_cmd so an arbitrary angle can be
    # typed at the drop servo and watched move - that is the only way to bracket
    # the two release angles on the real mechanism.  The subscription is gated:
    # with angle_debug absent the parameter is false, so the topic is never
    # subscribed and the four fixed commands remain the whole vocabulary.
    #
    # That default is what makes this a FAIL rather than a warn.  The bench
    # command line carries angle_debug:=true, and the tempting way to keep a
    # working setup around is to copy it into navigation_2d.launch.  After that
    # a single stray String on /servo_debug_cmd can drive the drop servo to any
    # angle, which is exactly the property the four-command design exists to
    # provide.  An absent parameter reads as off, so only an explicit true is
    # rejected.
    angle_debug = _resolve_arg_text(
        root, _launch_param(root, node_type, "angle_debug"))
    if str(angle_debug).strip().lower() == "true":
        fail("servo", launch,
             "angle_debug is true in the flight launch: /servo_debug_cmd is "
             "subscribed, so any String message can drive the drop servo to an "
             "arbitrary angle. Use src/servo_control/launch/servo_bench.launch "
             "for calibration instead - it is the only launch that sets this "
             "flag, and it is not part of the flight chain.")
    else:
        print("  angle_debug is off in this launch (bench-only entry, correct)")

    # ---- the bench launch is the one place the flag may be true ------------
    # The check above only proves the flight launch is clean.  It says nothing
    # about whether the bench path still works, and the failure mode when it is
    # broken is silent: the launch starts, the node starts, the servos move for
    # the four flight commands, and /servo_debug_cmd simply has no publisher -
    # so the calibration proceeds against a servo that ignores every typed
    # angle and the numbers get written into the flight launch regardless.
    bench = os.path.join(ROOT, "src/servo_control/launch/servo_bench.launch")
    if not os.path.exists(bench):
        warn("servo", ROOT,
             "src/servo_control/launch/servo_bench.launch is missing, so the "
             "calibration path has no entry point. Without it angle_debug cannot "
             "be turned on at all - it is deliberately not an arg of "
             "navigation_2d.launch, so `angle_debug:=true` on that launch is "
             "discarded without a message and the typed angles go nowhere")
    else:
        try:
            bench_root = ET.parse(bench).getroot()
        except ET.ParseError as error:
            fail("servo", bench,
                 "servo_bench.launch does not parse as XML: %s. roslaunch would "
                 "refuse it and the calibration path would look like a dead "
                 "/servo_debug_cmd" % error)
        else:
            bench_flag = _launch_param(bench_root, node_type, "angle_debug")
            if str(_resolve_arg_text(bench_root, bench_flag)).strip().lower() \
                    != "true":
                fail("servo", bench,
                     "servo_bench.launch does not set angle_debug=true on %s, so "
                     "it cannot reach /servo_debug_cmd and the whole point of the "
                     "bench launch is lost" % node_type)
            else:
                print("  bench entry: servo_bench.launch sets angle_debug=true "
                      "(calibration path intact)")

    # ---- the bench front end must name the same two topics as the node ------
    # tools/servo_debug.py spells the topic names as literals, because it has to
    # run without the package's headers and without a build.  That is one rename
    # away from "the servo never answers", which on a bench looks identical to
    # bad wiring and costs the most time of any failure here.
    node_src = os.path.join(ROOT, "src/servo_control/src/servo_controller_node.cpp")
    tool_src = os.path.join(ROOT, "tools/servo_debug.py")
    if os.path.exists(node_src) and os.path.exists(tool_src):
        with open(node_src, encoding="utf-8", errors="replace") as handle:
            node_text = handle.read()
        with open(tool_src, encoding="utf-8", errors="replace") as handle:
            tool_text = handle.read()

        def names(text, topic):
            # Whole quoted literal, not a substring.  A plain `topic in text`
            # test passes for "/servo_debug_ack2", which is exactly the rename
            # this check exists to catch - it was the first version of this
            # check, and it reported "agree" on a deliberately broken pair.
            return re.search(r"""['"]%s['"]""" % re.escape(topic), text) is not None

        both_agree = True
        for topic, what in (("/servo_debug_cmd", "command"), ("/servo_debug_ack",
                                                             "reply")):
            in_node = names(node_text, topic)
            in_tool = names(tool_text, topic)
            if in_node != in_tool:
                both_agree = False
                holder = "servo_controller_node.cpp" if in_node else "servo_debug.py"
                missing = "servo_debug.py" if in_node else "servo_controller_node.cpp"
                fail("servo", tool_src if in_node else node_src,
                     "the %s topic %s is in %s but not in %s; the node and the "
                     "bench front end must name the same topic or nothing "
                     "answers" % (what, topic, holder, missing))
        if both_agree:
            print("  bench topics agree: /servo_debug_cmd and /servo_debug_ack "
                  "in the node and in tools/servo_debug.py")


def _check_label_file(source, label_path, already_checked):
    """Check one label file; `already_checked` keeps both detectors honest."""
    if label_path is None:
        warn(13, source,
             "label_file default is not a $(find ...) path, cannot check it")
        return
    if label_path in already_checked:
        return
    already_checked.add(label_path)
    if not os.path.isfile(label_path):
        fail(13, source,
             "the detector's default label file does not exist: %s"
             % os.path.relpath(label_path, ROOT))
        return
    lines = [line.strip() for line in _read_text(label_path).splitlines()]
    while lines and not lines[-1]:
        lines.pop()
    if not lines:
        fail(13, label_path, "label file is empty")
        return
    # Case-insensitive: this chain's label file is written by jetson-inference's
    # train_ssd.py, which inherits classes.insert(0, 'BACKGROUND') from its own
    # VOCDataset, so it comes out upper case; the old ssd_ros files are lower
    # case. The criterion is "is line 0 the background line", not how it is
    # spelled - and because the class id IS the line number, one wrong line
    # shifts every class by one slot with nothing logged anywhere.
    if lines[0].lower() != "background":
        fail(13, label_path,
             "line 0 of the label file is %r, not a background line; "
             "detection_classes are looked up by line number, so every class "
             "would come out one slot wrong with nothing logged" % lines[0])
    persons = [line for line in lines if line.startswith("person")]
    if not persons:
        warn(13, label_path,
             "no label starts with 'person'; zone is derived from the name "
             "prefix, so the person round could never produce a candidate")
    print("    label file          : %d lines, line0=%r, person*=%d"
          % (len(lines), lines[0], len(persons)))


def check_vision_handshake():
    """The detector include, and the handshake that makes it reach the mission.

    Until 2026-10-17 there were two front ends and this gate was mostly about
    the switch between them: HSV colour windows read from a camera the mission
    opened itself, or the SSD detector in the ssd_detection package. The HSV
    path is gone - MissionVision now only consumes detections, and every image
    source question moved into the detector's own launch. What is left here is
    the handshake, which was always the part that could fail silently:

      * the detector publishes to one topic and the mission subscribes to
        another - two topics, no messages, no error anywhere, and the symptom is
        indistinguishable from "the detector is bad";
      * the mission opens no camera at all any more, so the detector is the only
        image source. On the aircraft it opens the camera itself through
        videoSource, driven by input_URI; an empty one means nobody opens it,
        the node falls through to the subscribe branch with an empty topic, and
        the run produces no detections and no error - the mission quietly drops
        everything at the fallback waypoints and the log reads like a detector
        problem;
      * which detector implementation runs is still decided by use_usb_cam:
        Python + OpenCV DNN where there is no CUDA, the C++ detectNet node on
        the flight computer. Pointing either machine at the other's launch is
        not a slower version of the same run - it is a node that never starts,
        and it takes the whole vision chain with it. And the $(eval ...) that
        makes that choice must not compare arg() against a quoted string:
        roslaunch converts the arg to a boolean first, so `arg('x') == 'true'`
        is False for both values and the switch is dead while looking correct.
        That line shipped, and it is why the aircraft kept asking for the
        simulation's Python detector on 2026-10-07 and again on 2026-10-17;
      * a <param> the launch sets but the node never reads is not an error in
        ROS: it sits on the parameter server unused while the node runs on its
        built-in default, which is indistinguishable from a tuning problem;
      * the label file's line 0 must be `background`, because the id the model
        reports IS the line number. Shift that by one and every class is wrong
        while every box stays in the right place, with nothing logged;
      * the mission filters candidates by zone, and zone is derived from the
        class name prefix, so a label file with no `person*` line silently
        makes the person round unachievable.

    None of these is visible from the node's own startup logs, which is why
    they are checked here rather than trusted to a run.
    """
    heading(13, "vision: detector include and the detection topic handshake")

    import xml.etree.ElementTree as ET

    packages = _package_dirs()
    mission = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if not os.path.isfile(mission):
        fail(13, mission, "navigation_2d.launch not found")
        return
    root = ET.parse(mission).getroot()

    topic = _launch_arg(root, "vision_detection_topic", "")
    threshold = _launch_arg(root, "ssd_score_threshold", "")
    usb_cam = _resolve_arg_expr(root, _launch_arg(root, "use_usb_cam", "false"))
    ssd_raw = _launch_arg(root, "ssd_detection_launch", "")

    # A $(eval ...) that compares arg() to a quoted literal can never be true:
    # roslaunch converts 'true'/'false' into real booleans before handing them
    # to the expression, so arg('use_usb_cam') == 'true' is True == 'true' and
    # evaluates to False whatever the operator passes. The switch then looks
    # derived while being pinned to one branch - reported here by name, because
    # resolving it silently produces two identical detector paths and the only
    # symptom is a node that cannot be located on the flight computer.
    if ssd_raw.strip().startswith("$(eval") and \
            re.search(r"arg\(\s*'[^']+'\s*\)\s*==\s*'", ssd_raw):
        fail(13, mission,
             "ssd_detection_launch chooses the detector with "
             "arg('use_usb_cam') == '<text>'. roslaunch's $(eval) converts the "
             "arg to a boolean first ('true' -> True), so the comparison is "
             "True == 'true' -> False for both values: this branch can never be "
             "taken, the aircraft keeps the simulation's Python detector, and "
             "the launch dies with 'cannot launch node of type "
             "[ssd_detection/ssd_detection_node.py]'. Use the boolean directly: "
             "$(eval A if arg('use_usb_cam') else B)")

    # use_usb_cam is the one switch left, and it is not a backend choice: it says
    # which machine this is, because the two machines cannot open the same
    # camera. On both machines it is passed on the roslaunch command line and
    # never written into the file, so checking only the default would never see
    # the combination that actually flies on the aircraft.
    #   sim      use_usb_cam:=false - Gazebo camera, Python + OpenCV DNN
    #   aircraft use_usb_cam:=true  - GStreamer direct capture, C++ detectNet
    world_sim = {"use_usb_cam": "false"}
    world_aircraft = {"use_usb_cam": "true"}
    ssd_launch_sim = _resolve_arg_expr(root, ssd_raw, world_sim)
    ssd_launch_aircraft = _resolve_arg_expr(root, ssd_raw, world_aircraft)

    print("  use_usb_cam            = %s (default) / true (aircraft)" % usb_cam)
    print("  vision_detection_topic = %s" % topic)
    print("  detector launch, sim      : %s" % ssd_launch_sim)
    print("  detector launch, aircraft : %s" % ssd_launch_aircraft)

    if "ssd_detection" not in packages:
        # The detector is the only front end now, so a missing package is always
        # fatal: the include cannot resolve, nothing publishes detections, and
        # require_vision_ready then holds the vehicle on the pad for the whole
        # vision_ready_timeout with a message about a detector that was never
        # built.
        fail(13, mission,
             "there is no ssd_detection package in src/; it is the only vision "
             "front end, so the detector include cannot resolve and the pre-arm "
             "vision gate will never open")
        return

    # ---- which detector runs is one switch, checked in both worlds ---------
    # The two detector launches take the same arguments and differ only in who
    # computes: Python + OpenCV DNN is what runs where there is no CUDA, the
    # C++ detectNet node is what runs on the flight computer. Pointing the
    # aircraft at the Python one is not a slower version of the same run - it
    # is a node that never starts, and it takes the whole vision chain with it.
    # The mirror (the simulation pointed at the C++ node) fails the same way,
    # because that target needs jetson-inference and is built for the Jetson
    # alone, so it does not exist on the PC.
    ssd_paths = []
    for what, world, text in (("sim", world_sim, ssd_launch_sim),
                              ("aircraft", world_aircraft,
                               ssd_launch_aircraft)):
        path = _resolve_find(text, packages) if text else None
        if path is None:
            fail(13, mission,
                 "ssd_detection_launch for the %s is %r; expected a "
                 "$(find ssd_detection)/launch/... path" % (what, text))
            continue
        if not os.path.isfile(path):
            fail(13, mission,
                 "ssd_detection_launch for the %s resolves to %s, which does "
                 "not exist" % (what, os.path.relpath(path, ROOT)))
            continue
        types = _launch_node_types(path)
        print("  detector launch, %-9s: %s -> %s"
              % (what, ", ".join(types) or "no <node>",
                 os.path.relpath(path, ROOT)))
        python_nodes = [t for t in types if t.endswith(".py")]
        cpp_nodes = [t for t in types if not t.endswith(".py")]
        if what == "aircraft" and python_nodes:
            fail(13, path,
                 "this is the detector launch the aircraft gets, and it starts "
                 "%s. The Python detector is the fallback for a machine with "
                 "no CUDA; on the Jetson it aborts the launch with 'cannot "
                 "launch node of type [ssd_detection/ssd_detection_node.py]', "
                 "after which the mission runs with no detections and no error "
                 "at all. Point it at the detectNet launch."
                 % ", ".join(python_nodes))
        if what == "sim" and cpp_nodes:
            fail(13, path,
                 "this is the detector launch the simulation gets, and it "
                 "starts %s. That target is built for the Jetson alone (it "
                 "needs jetson-inference), so on the PC the launch aborts with "
                 "'cannot launch node of type "
                 "[ssd_detection/ssd_detection_node]' and the vision chain "
                 "goes silent. Point it at the Python launch."
                 % ", ".join(cpp_nodes))
        ssd_paths.append(path)

    # ---- what the include forwards to whichever launch the switch picks -----
    forwarded = {}
    for include in root.iter("include"):
        if "$(arg ssd_detection_launch)" not in (include.get("file") or ""):
            continue
        for child in include:
            if child.tag == "arg":
                forwarded[child.get("name")] = (child.get("value") or "").strip()
    if not forwarded:
        fail(13, mission,
             "no include passes $(arg ssd_detection_launch); the detector "
             "would never start in ssd mode")

    # The include forwards one fixed list of arguments to both launches, so the
    # two files must declare the same set of names or the list cannot stay
    # uniform: a name only one of them declares is dropped for the other (or
    # the launch refuses to start), and which of the two happens depends on
    # the switch - i.e. on which machine is being brought up.
    declared = {}
    for path in dict.fromkeys(ssd_paths):
        declared[path] = {a.get("name") for a in
                          ET.parse(path).getroot().iter("arg")}
    for name in sorted(forwarded):
        for path, names in declared.items():
            if name not in names:
                fail(13, path,
                     "%s is forwarded by the detector include in "
                     "navigation_2d.launch but is not declared here; the two "
                     "detector launches are supposed to accept the same "
                     "parameter set, and this one does not" % name)

    # ---- the topic handshake, at the include rather than at the default -----
    # Reading the detector launch's own default is not enough: the include is
    # what actually decides, and a dropped forward leaves the detector on its
    # default while the mission watches another name.
    want_topic = _resolve_arg_expr(root, topic)
    if "output_topic" not in forwarded:
        fail(13, mission,
             "the detector include does not forward output_topic; the detector "
             "falls back to its own default topic, which need not be the one "
             "the mission is watching")
    else:
        got_topic = _resolve_arg_expr(root, forwarded["output_topic"])
        if want_topic and got_topic != want_topic:
            fail(13, mission,
                 "the detector include forwards output_topic=%r but the "
                 "mission subscribes to %r; ssd mode would receive nothing and "
                 "report nothing" % (got_topic, want_topic))
        elif want_topic:
            print("    topic handshake    : %s" % want_topic)

    # ---- the image source: the sim subscribes, the aircraft captures -------
    # input_image_topic is the simulation detector's image source (Gazebo's
    # camera). It has to reach the detector as the very topic the wrappers
    # publish, or the detector listens to its own default while Gazebo
    # publishes somewhere else - no error, no frames.
    if "input_image_topic" not in forwarded:
        fail(13, mission,
             "the detector include does not forward input_image_topic; the "
             "detector falls back to its own default topic, which need not be "
             "the one that is publishing")
    elif forwarded["input_image_topic"] != "$(arg input_image_topic)":
        fail(13, mission,
             "the detector include forwards input_image_topic=%r, not "
             "$(arg input_image_topic); the detector would be reading a topic "
             "that is not the one the wrappers publish"
             % forwarded["input_image_topic"])
    else:
        sim_topic = _resolve_arg_expr(root, forwarded["input_image_topic"],
                                      world_sim)
        if not sim_topic:
            warn(13, mission,
                 "with use_usb_cam:=false the detector's input_image_topic "
                 "evaluates to %r as written. Both simulation wrappers pass it "
                 "explicitly (px4_sitl_field.launch, gazebo_field.launch), so "
                 "this only traps whoever starts this launch by hand"
                 % sim_topic)

    # On the aircraft the camera is opened by the detector node itself, through
    # videoSource. An empty URI means nobody opens it: in ssd mode
    # MissionVision opens no camera either, and a detector with an empty URI
    # takes the subscribe branch with an empty topic. Nothing is logged and
    # nothing is received, so every drop happens at the fallback waypoint for
    # the whole run and the log looks like a detector problem.
    if "input_URI" not in forwarded:
        fail(13, mission,
             "the detector include does not forward input_URI; on the aircraft "
             "the detector would fall back to its own default URI instead of "
             "the camera this launch names")
    else:
        uri = _resolve_arg_expr(root, forwarded["input_URI"], world_aircraft)
        if not uri:
            fail(13, mission,
                 "with use_usb_cam:=true the detector's input_URI evaluates to "
                 "%r, so nobody opens the camera: in ssd mode MissionVision "
                 "does not open one either, and the detector subscribes to an "
                 "empty topic. The run produces no detections and no error, "
                 "and every drop happens at the fallback waypoint" % uri)
        else:
            print("    aircraft camera URI: %s" % uri)

    # ---- and the rest of the forward list is an interface, not a habit -----
    # The include is the only place the mission's vision arguments reach the
    # detector. A name that goes missing is dropped in silence: the detector
    # falls back to its own default, which equals the mission's default today
    # and diverges the moment somebody overrides one on the command line
    # (vision_max_rate:=10 with no forward is a no-op, and nothing says so).
    # The three that fail loudly above are not repeated here.
    for name in ("score_threshold", "input_width", "input_height",
                 "input_rate", "input_codec", "image_topic", "video_width",
                 "jpeg_quality", "max_rate"):
        if name not in forwarded:
            fail(13, mission,
                 "the detector include does not forward %s; the detector falls "
                 "back to its own default, which matches the mission's default "
                 "today and diverges the moment somebody overrides it on the "
                 "command line" % name)

    # ---- a parameter nobody reads is a silent default ----------------------
    for path in dict.fromkeys(ssd_paths):
        droot = ET.parse(path).getroot()
        for node in droot.iter("node"):
            ntype = node.get("type") or ""
            pkg_dir = os.path.dirname(os.path.dirname(path))
            source = (os.path.join(pkg_dir, "scripts", ntype)
                      if ntype.endswith(".py")
                      else os.path.join(pkg_dir, "src", ntype + ".cpp"))
            if not os.path.isfile(source):
                continue
            text = _read_text(source)
            for param in node.iter("param"):
                name = param.get("name")
                if not name:
                    continue
                # C++ reads private_nh_.param<T>("name", ...); Python reads
                # private("~name", ...). The tilde is optional in the pattern
                # so both spellings count as a read.
                if not re.search(r"['\"]~?%s['\"]" % re.escape(name), text):
                    fail(13, path,
                         "%s sets parameter %r but the node never reads it. ROS "
                         "does not complain about that: the value sits on the "
                         "parameter server unused while the node runs on its "
                         "built-in default, which is indistinguishable from a "
                         "tuning problem. Rename it in both places."
                         % (ntype, name))

    # The handshake, against every detector launch this file can reach rather
    # than just the one its default selects: the two agree today and nothing
    # in the launch is keeping them agreeing tomorrow.
    labels_seen = set()
    for path in dict.fromkeys(ssd_paths):
        droot = ET.parse(path).getroot()
        detector_topic = _launch_arg(droot, "output_topic", "")
        print("    publishes %s on %s"
              % (detector_topic or "<nothing>", os.path.relpath(path, ROOT)))
        if want_topic and detector_topic and want_topic != detector_topic:
            fail(13, path,
                 "the mission subscribes to %r but the detector publishes to "
                 "%r; ssd mode would receive nothing and report nothing"
                 % (want_topic, detector_topic))
        label_file = _launch_arg(droot, "label_file", "")
        _check_label_file(path, _resolve_find(label_file, packages),
                          labels_seen)

    try:
        value = float(threshold)
    except (TypeError, ValueError):
        fail(13, mission, "ssd_score_threshold %r is not a number" % threshold)
    else:
        if not 0.0 < value <= 1.0:
            fail(13, mission,
                 "ssd_score_threshold %.3f is outside (0, 1]" % value)


def check_startup_gates():
    """The two conditions that decide whether the vehicle may leave the ground,
    and whether a leg can ever be declared arrived.

    (a) Vision readiness before arming.
        auto_start used to depend on the FCU being connected, the servos being
        ready and 100 setpoints (2 s at 50 Hz) - and on nothing else. On the
        Jetson the TensorRT engine takes ~26 s to deserialise with the CPU
        saturated, so the vehicle could be airborne half a minute before the
        detector produced its first frame. That is the documented aggravator of
        the 2026-10-07 crash (cartographer starving during takeoff) and it also
        means the task phase starts with no detections at all.
        rescue_mission now refuses to request OFFBOARD until a
        /mission/vision/detection message has arrived. MissionVision publishes
        one such message per processed frame - in IDLE too, which is what makes
        it a usable heartbeat before HAZARD_HOVER (and it publishes it even when
        the detection array is empty, because the heartbeat is the message, not
        a detection). This check keeps the gate wired up and its timeout long
        enough for a cold start.

    (b) Mission arrival tolerance vs the follower's stop tolerance.
        EgoTrajectoryController stops when it is within goal_tolerance of
        poses.back(), and poses.back() is the end of the LOCAL plan, not the
        route goal (the reference path is truncated to the local window). So a
        healthy planner can stop the vehicle short of the waypoint, and if the
        mission's position_tolerance is tighter than the point the follower is
        willing to stop at, the leg hangs "not arrived" with a planner that is
        working perfectly. 2026-10-17: both were ~0.10-0.12 m and the vehicle
        sat 0.097 m from hazard_route[1].

    (c) A leg the planner never starts moving on is not a stall.
        route_no_motion_timeout has to fire before the generic
        planner_failure_abort_timeout, otherwise every "the planner will not
        accept this goal" ends up reported as "the planner stalled", and the
        mission retries a goal it cannot reach.

    (d) The align envelope.
        HAZARD_ALIGN/RESCUE_ALIGN do not go through the planner at all: they
        P-control target_position_ at 50 Hz, so no planner-side bound applies to
        them. On 2026-10-17 the correction clamp was 0.45 m, the drop point was
        0.65 m from the north wall and the rotor radius is 0.352 m (450 mm axis
        with 10-inch props) - the tips went through the wall. Note the 0.22 m
        that was configured at the time understated this by 13 cm, which is why
        the first fix did not close the hole. The three numbers that now bound
        that command are checked here, and so is the structural premise behind
        them: every align entry clamps its setpoint, stamps its own timer and
        sets the flag.
    """
    import xml.etree.ElementTree as ET

    heading(14, "pre-arm gates, arrival tolerance and the align envelope")

    launch_path = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    if not os.path.exists(launch_path):
        warn(14, launch_path, "not found; skipped")
        return
    try:
        root = ET.parse(launch_path).getroot()
    except ET.ParseError as error:
        fail(14, launch_path, "cannot parse: %s" % error)
        return

    def number(node_type, name):
        raw = _resolve_arg_text(root, _launch_param(root, node_type, name))
        if raw is None:
            return None
        try:
            return float(raw)
        except (TypeError, ValueError):
            fail(14, launch_path,
                 "%s.%s is %r, which is not a number" % (node_type, name, raw))
            return None

    # (a) vision readiness
    require_raw = _resolve_arg_text(
        root, _launch_param(root, "rescue_mission_node", "require_vision_ready"))
    if require_raw is None:
        fail(14, launch_path,
             "rescue_mission_node has no require_vision_ready parameter, so the "
             "pre-arm camera gate is not wired up and auto_start races the "
             "detector again")
    elif require_raw.strip().lower() != "true":
        fail(14, launch_path,
             "require_vision_ready resolves to %r; the pre-arm gate must default "
             "to true (pass require_vision_ready:=false at launch time to fly "
             "without vision on purpose)" % require_raw.strip())

    timeout = number("rescue_mission_node", "vision_ready_timeout")
    if timeout is None:
        fail(14, launch_path, "rescue_mission_node has no vision_ready_timeout")
    elif timeout < 30.0:
        fail(14, launch_path,
             "vision_ready_timeout %.1f s does not cover the 26.5 s TensorRT "
             "engine load measured on the Jetson; auto_start would give up, or "
             "worse, be raised to something that fits" % timeout)
    elif timeout > 180.0:
        warn(14, launch_path,
             "vision_ready_timeout %.0f s is longer than any takeoff should wait; "
             "a dead camera would hold the vehicle on the pad for that long"
             % timeout)

    # (b) no-motion escalation before the generic stall timer
    no_motion = number("rescue_mission_node", "route_no_motion_timeout")
    stall = number("rescue_mission_node", "planner_failure_abort_timeout")
    if no_motion is None:
        fail(14, launch_path,
             "rescue_mission_node has no route_no_motion_timeout, so a leg the "
             "planner refuses is only reported by the generic stall timer")
    elif no_motion < 2.0:
        fail(14, launch_path,
             "route_no_motion_timeout %.1f s is short enough to abort a healthy "
             "leg on one slow replan" % no_motion)
    elif stall is not None and no_motion >= stall:
        fail(14, launch_path,
             "route_no_motion_timeout %.1f s does not fire before "
             "planner_failure_abort_timeout %.1f s, so the specific 'planner "
             "refuses this goal' reason is never printed" % (no_motion, stall))

    # Print the values that were read, not only the failures. A check that is
    # silent when it passes cannot be audited from a log paste, and a resolved
    # string like "true" is exactly the kind of thing a later $(eval) can turn
    # into something else without anyone noticing. Same reasoning as the
    # "9 file(s) checked" / "43 file(s) checked" counters in gates 1-4: the
    # number on screen is the evidence that the check ran at all.
    print("  require_vision_ready=%s  vision_ready_timeout=%s (cold start needs "
          "> 26.5 s of TensorRT engine load)  route_no_motion_timeout=%s%s"
          % (require_raw.strip() if require_raw is not None else "MISSING",
             "%.0f s" % timeout if timeout is not None else "MISSING",
             "%.1f s" % no_motion if no_motion is not None else "MISSING",
             " < planner_failure_abort_timeout %.1f s" % stall
             if (stall is not None and no_motion is not None) else ""))

    # (c) arrival tolerance vs the follower's stop tolerance
    position_tolerance = number("rescue_mission_node", "position_tolerance")
    goal_tolerance = number("ego_trajectory_controller_node", "goal_tolerance")
    if position_tolerance is None or goal_tolerance is None:
        warn(14, launch_path,
             "cannot read position_tolerance / goal_tolerance; arrival agreement "
             "not checked")
    else:
        print("  mission position_tolerance=%.3f m  follower goal_tolerance=%.3f m "
              "(the follower stops against the END OF THE PLAN, not the goal)"
              % (position_tolerance, goal_tolerance))
        if position_tolerance < goal_tolerance:
            fail(14, launch_path,
                 "the mission demands arrival within %.3f m but the follower "
                 "stops at %.3f m, so a leg can never be declared arrived"
                 % (position_tolerance, goal_tolerance))
        elif position_tolerance - goal_tolerance < 0.05:
            warn(14, launch_path,
                 "arrival margin is only %.3f m (%.3f vs %.3f). Necessary, not "
                 "sufficient: the plan end can itself be short of the goal, "
                 "which is why route_no_motion_timeout also exists"
                 % (position_tolerance - goal_tolerance,
                    position_tolerance, goal_tolerance))

    # (d) the align envelope: how far a visual correction may drag the vehicle
    #
    # On 2026-10-17 the aircraft flew into the north wall while dropping. The
    # planner was not involved: HAZARD_ALIGN is not a route leg, it falls
    # through controlCallback's final else and P-controls target_position_
    # directly at 50 Hz. The target was current position + a visual correction
    # clipped to max_task_correction, which was 0.45 m - and the drop point is
    # 0.65 m from that wall. So the setpoint could be commanded 0.45 m toward a
    # wall with a 0.352 m rotor radius: the tips went through.
    #
    # Three bounds now apply, and each one is checked here because each one is
    # a number that can be typed wrong:
    #   * max_task_correction, the per-axis correction clamp;
    #   * align_body_clearance, the rotor radius the setpoint envelope is
    #     shrunk by inside clampAlignTarget();
    #   * align_timeout, measured from align entry and never reset while short
    #     of the target, so an unreachable setpoint ends instead of being
    #     pursued forever.
    # The fourth bound is structural and therefore checked structurally: every
    # place that enters align must clamp, stamp the timer and set the flag. A
    # new align entry that forgets one of the three is silently unbounded again,
    # which is exactly how the crash happened.
    correction = number("rescue_mission_node", "max_task_correction")
    clearance = number("rescue_mission_node", "align_body_clearance")
    align_timeout = number("rescue_mission_node", "align_timeout")

    # 机身半径 = 轴距 / 2 + 桨半径。450 mm 轴距（电机对电机）+ 10 寸桨：
    #     0.225 + 0.127 = 0.352 m
    # 这是 launch 之外唯一写下机身尺寸的地方，所以算式直接写在这里：换桨的人
    # 必须同时改这两处，写成 import 反而看不出该改哪个数。
    # 2026-10-17 之前这里写的是 0.22 —— 那是"桨尖对桨尖 450"的读法，比真机小
    # 13 cm，于是 clampAlignTarget 用 0.22 内缩时仍然允许桨尖越墙 0.13 m。
    min_rotor_radius = 0.45 / 2.0 + 0.254 / 2.0   # 0.352 m

    if correction is None:
        fail(14, launch_path,
             "rescue_mission_node has no max_task_correction; the align "
             "correction falls back to the node's built-in default, and the "
             "2026-10-17 crash was caused by that default being 0.45 m")
    elif correction > 0.20:
        fail(14, launch_path,
             "max_task_correction=%.2f m. The drop points sit 0.65 m from a wall "
             "and the airframe radius is %.3f m, so the worst-case setpoint is "
             "%.2f m from the wall with %.2f m of rotor left to cross it. "
             "Above 0.20 m the per-axis clamp alone can no longer be argued "
             "safe on this field."
             % (correction, min_rotor_radius, 0.65 - correction,
                0.65 - correction - min_rotor_radius))

    if clearance is None:
        fail(14, launch_path,
             "rescue_mission_node has no align_body_clearance; clampAlignTarget "
             "would fall back to its built-in value and the setpoint envelope "
             "would no longer be tied to the airframe")
    elif clearance < min_rotor_radius - 0.01:
        fail(14, launch_path,
             "align_body_clearance=%.2f m is smaller than the %.3f m rotor "
             "radius (450 mm axis / 2 + 10-inch prop / 2). The envelope is "
             "applied to the COMMANDED SETPOINT, and a setpoint drags the rotor "
             "tips with it - an envelope that only keeps the airframe centre "
             "inside the field is the same mistake as insideSafeField(), which "
             "is an abort test at the centre and not a command limit. This is "
             "the 2026-10-17 crash: 0.22 m left the tips 0.13 m of wall."
             % (clearance, min_rotor_radius))
    elif clearance > 0.50:
        warn(14, launch_path,
             "align_body_clearance=%.2f m is more than twice the rotor radius; "
             "the envelope would refuse most of the field near a wall" % clearance)
    else:
        print("  align_body_clearance=%.2f m vs derived rotor radius %.3f m "
              "(450 mm axis / 2 + 10-inch prop / 2)%s"
              % (clearance, min_rotor_radius,
                 "" if clearance <= min_rotor_radius + 0.005 else
                 "  +%.0f mm of margin" % (1000 * (clearance - min_rotor_radius))))

    if align_timeout is None:
        fail(14, launch_path,
             "rescue_mission_node has no align_timeout; an unreachable align "
             "setpoint would be pursued until the task timeout")
    elif align_timeout < 1.0:
        fail(14, launch_path,
             "align_timeout=%.1f s is too short to walk a correction down; the "
             "correction is re-issued as current + clamped offset at 50 Hz and "
             "needs a fraction of a second to converge" % align_timeout)
    elif align_timeout > 30.0:
        warn(14, launch_path,
             "align_timeout=%.0f s lets the vehicle hover over a target it "
             "cannot reach for that long" % align_timeout)

    # the shrunk envelope must still be a rectangle, both axes
    field = {}
    for name in ("safe_field_min_x", "safe_field_max_x",
                 "safe_field_min_y", "safe_field_max_y"):
        raw = _resolve_arg_text(root,
                                _launch_param(root, "rescue_mission_node", name))
        try:
            field[name] = float(raw)
        except (TypeError, ValueError):
            field[name] = None
    if clearance is not None and None not in field.values():
        for axis, lo, hi in (("x", "safe_field_min_x", "safe_field_max_x"),
                             ("y", "safe_field_min_y", "safe_field_max_y")):
            width = field[hi] - field[lo] - 2.0 * clearance
            if width <= 0.0:
                fail(14, launch_path,
                     "the safe field spans %.2f m in %s but align_body_clearance="
                     "%.2f m is removed from BOTH sides, leaving %.2f m. "
                     "clampAlignTarget would collapse every align setpoint to a "
                     "line and the vehicle would align along it forever"
                     % (field[hi] - field[lo], axis, clearance, width))
            else:
                print("  align envelope: %s %.2f..%.2f m, shrunk by %.2f m -> "
                      "%.2f m usable"
                      % (axis, field[lo], field[hi], clearance, width))

    # structural: enter align => start the clock; compute a correction => clamp.
    # Counted, not trusted. There are three ways into the align state and only
    # one of them computes a visual correction (the other two keep the hover
    # setpoint and just dwell, which is why they do not need the clamp - their
    # setpoint came from the route table and gate 8 already vetted it).
    mission_src = os.path.join(ROOT, "src/navigation_2d/src/rescue_mission.cpp")
    if not os.path.exists(mission_src):
        fail(14, mission_src, "not found")
    else:
        code = _strip_cpp_comments(io.open(mission_src, encoding="utf-8").read())
        align_entries = code.count("enterState(align_state)")
        corrections = code.count("task_alignment_started_ = true")
        clamps = code.count("clampAlignTarget(target_position_)")
        stamps = len(re.findall(r"align_entered_at_\s*=\s*now\b", code))
        resets = code.count("align_entered_at_ = 0.0")
        print("  align entries: %d  vision corrections: %d  clamped: %d  "
              "timer stamped: %d  cleared on beginTask: %d"
              % (align_entries, corrections, clamps, stamps, resets))
        if align_entries == 0:
            fail(14, mission_src,
                 "no enterState(align_state) site found; this gate can no longer "
                 "see the align entries, so it must be updated rather than left "
                 "reporting OK")
        else:
            if stamps < align_entries:
                fail(14, mission_src,
                     "%d entry site(s) into the align state but only %d stamp(s) "
                     "of align_entered_at_. An entry that does not start the clock "
                     "is an align with no timeout, and an unreachable setpoint is "
                     "then pursued until the task timeout - that is the loop that "
                     "walked the aircraft into the north wall on 2026-10-17"
                     % (align_entries, stamps))
            if corrections and clamps < corrections:
                fail(14, mission_src,
                     "%d align site(s) compute a visual correction but only %d "
                     "pass the result through clampAlignTarget(). A correction "
                     "bounded only by max_task_correction can still travel toward "
                     "a wall that is closer than the clamp" % (corrections, clamps))
            if corrections == 0:
                warn(14, mission_src,
                     "no 'task_alignment_started_ = true' site found; if the vision "
                     "align path was renamed, update this gate - it is the only "
                     "thing asserting that every correction is clamped")
            if resets == 0:
                warn(14, mission_src,
                     "align_entered_at_ is never cleared in beginTask(); a stale "
                     "stamp from the previous task would fire the new align's "
                     "timeout on its first cycle")

        # ---- the vision fallback must be able to finish the drop alone ------
        #
        # 2026-10-18: in vision_with_fallback the branch after
        # drop_vision_timeout_ only dropped when the vehicle was ALREADY inside
        # drop_waypoint_tolerance_ (0.20 m) and otherwise just warned and waited
        # for task_timeout_ to abort. NAV hands over within position_tolerance_,
        # so a normal run usually passed - but whenever it did not, "no stable
        # detection" still meant "no drop", which is the exact outcome this mode
        # exists to prevent. The fix is that the branch commands the authored
        # waypoint; this asserts it stays that way.
        if code.count("target_position_.x = task_reference_.x") == 0:
            fail(14, mission_src,
                 "the vision_with_fallback drop no longer re-targets the authored "
                 "waypoint when the vehicle is off it. Without that line an "
                 "off-waypoint hover with no detection holds until task_timeout_ "
                 "and aborts instead of dropping")
        else:
            print("  vision fallback re-targets the authored waypoint when off it, "
                  "so 'no detection' cannot mean 'no drop'")

        # ---- takeoff must be a pure position setpoint, not pos+vel ---------
        #
        # 2026-10-18: TAKEOFF used to share the hover/align branch, which sends
        # position AND velocity together (setpoint_raw_local_pos_vel's type_mask
        # only ignores acceleration/FORCE/yaw_rate) with the position built from
        # the measured pose plus a 0.2 s lead. That is two parallel position
        # loops and a setpoint that copies the estimate; the 0.2 s lead cancels
        # the mavros+EKF delay, so the net damping is ~0 and every disturbance
        # at liftoff survives as a fore/aft sway. It also shows up in SITL,
        # which rules out SLAM: nothing publishes /mavros/vision_pose/pose in
        # sim, so the estimate isPX4's own EKF. Takeoff's XY target is a
        # constant (home), so it needs none of that soft-start; hover/align
        # still does because its target jumps.
        control_at = code.find("void RescueMission::controlCallback")
        takeoff_at = code.find("state_ == TAKEOFF", control_at) if control_at >= 0 else -1
        if takeoff_at < 0:
            fail(14, mission_src,
                 "no 'state_ == TAKEOFF' branch inside controlCallback(); this "
                 "gate can no longer see how the climb is commanded, so it must "
                 "be updated rather than left reporting OK")
        else:
            window = code[takeoff_at:takeoff_at + 2400]
            pos_at = window.find("setpoint_raw_local_pos(")
            vel_at = window.find("setpoint_raw_local_pos_vel(")
            ramp = window.count("takeoff_cmd_z_ +")
            if pos_at < 0 or (vel_at >= 0 and vel_at < pos_at):
                fail(14, mission_src,
                     "the TAKEOFF branch does not send a pure position setpoint. "
                     "Sending position and velocity together there gives PX4 two "
                     "parallel position loops with a setpoint copied from the "
                     "measured pose, which is the undamped sway seen on "
                     "2026-10-18 in both sim and on the real aircraft")
            elif ramp < 1:
                fail(14, mission_src,
                     "the TAKEOFF branch never advances takeoff_cmd_z_ (expected "
                     "'takeoff_cmd_z_ +'). A fixed z setpoint there is a step, "
                     "and PX4 follows a step at MPC_Z_VEL_MAX_UP instead of at "
                     "the 0.35 m/s the climb was commanded at")
            else:
                print("  takeoff: pure position setpoint with a z ramp "
                      "(takeoff_cmd_z_ advanced at %d site(s)); hover/align "
                      "keeps the pos+vel soft-start" % ramp)

        # ---- abort must end in a landing, not in a hover -------------------
        #
        # 2026-10-17: the mission aborted in the person area with "Planner never
        # started moving toward the waypoint" (3.0 m from home, i.e. outside
        # abort_land_home_radius) and then held that position for the rest of
        # the run - the log contains no further intention to land, and the
        # aircraft was lost to an unrelated EKF failure while hovering there.
        # "Far from home" is not a reason to refuse to land on a 4 m field.
        #
        # This asserts the behaviour, not the wording: the ABORT branch of
        # missionCallback must reach a mode request, and abortMission must
        # clear the "already requested" latch so every abort gets its own try.
        callback_at = code.find("void RescueMission::missionCallback")
        abort_at = code.find("case ABORT:", callback_at) if callback_at >= 0 else -1
        abort_resets = code.count("abort_land_requested_ = false")
        if abort_at < 0:
            fail(14, mission_src,
                 "cannot find 'case ABORT:' inside missionCallback(); this gate "
                 "can no longer see the abort behaviour, so it must be updated "
                 "rather than left reporting OK")
        elif "AUTO.LAND" not in code[abort_at:abort_at + 1600]:
            fail(14, mission_src,
                 "the ABORT branch of missionCallback does not request AUTO.LAND. "
                 "Without it an aborted mission holds its position until the "
                 "battery ends - that is exactly what happened on 2026-10-17, "
                 "3 m from home, with the aircraft still armed")
        elif abort_resets < 2:
            fail(14, mission_src,
                 "abort_land_requested_ is cleared in %d place(s); expected one on "
                 "each abort path (landing failure, far from home, not airborne). "
                 "A stale latch makes the next abort skip its AUTO.LAND request"
                 % abort_resets)
        else:
            print("  abort landing: AUTO.LAND in the ABORT branch, "
                  "abort_land_requested_ cleared at %d site(s)" % abort_resets)


def _code_mask(text):
    """Same length as text; False in comments and string literals.

    Brace counting must not see the braces inside comments - and this project's
    comments are dense prose in two languages, so they are exactly where a stray
    brace would come from.
    """
    mask = [True] * len(text)
    i, n = 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            for k in range(i, j):
                mask[k] = False
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                mask[k] = False
            i = j
        elif text[i] in "\"'":
            quote, j = text[i], i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == quote:
                    j += 1
                    break
                j += 1
            for k in range(i, j):
                mask[k] = False
            i = j
        else:
            i += 1
    return mask


def _body_span(text, mask, signature):
    """(open_brace, close_brace) of the first function whose name matches."""
    m = re.search(re.escape(signature), text)
    if not m:
        return None
    start = text.find("{", m.end())
    if start < 0:
        return None
    depth = 0
    for i in range(start, len(text)):
        if not mask[i]:
            continue
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return start, i
    return None


def _depth_array(text, mask, start, end):
    """Brace depth *before* each character, relative to the function's own brace.

    depth[start] == 0 (the opening brace itself) and the body sits at 1.
    """
    depth = 0
    out = [0] * (end + 1)
    for i in range(start, end + 1):
        out[i] = depth
        if mask[i]:
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
    return out


def check_vision_locking():
    """MissionVision must not lock detection_mutex_ twice on one thread.

    This gate exists because of a bug that cost a whole run on 2026-10-18 and
    produced no error message anywhere:

        detectionCallback()          std::lock_guard lock(detection_mutex_)
          -> processDetections()     std::lock_guard lock(mutex_)
            -> findDetectionCandidates()   std::lock_guard lock(detection_mutex_)

    Same thread, same non-recursive std::mutex, second acquisition. That is
    undefined behaviour; glibc deadlocks. The shape of the failure is what makes
    it worth a gate rather than a comment:

      * the IDLE branch returns before findDetectionCandidates, so before takeoff
        the node publishes its found=false heartbeat perfectly and
        require_vision_ready happily clears the aircraft for flight;
      * the first frame after setVisionMode(HAZARD|PERSON) reaches the double
        lock and the node stops for good - no log line, no exception, nothing;
      * the mission then reports "no stable detection (last: found=0 stable=0)",
        which is byte-identical to what a detector seeing nothing reports. The
        detector's own annotated JPEG keeps showing boxes, so the one piece of
        evidence on screen points at the wrong layer;
      * the deadlocked thread holds mutex_, so the next setVisionMode() - the one
        inside abortMission() - blocks the mission timer thread too, and the
        aircraft never gets its AUTO.LAND.

    Two things are checked, because two separate edits are needed to reintroduce
    it: the call site must sit at function scope (nothing can be held across it),
    and detection_mutex_ must still guard the cache read (so nobody "fixes" it by
    deleting the inner lock and racing instead).
    """
    heading(16, "vision: detection_mutex_ is never taken twice on one thread")

    header = os.path.join(ROOT, "src/navigation_2d/include/mission_vision.h")
    source = os.path.join(ROOT, "src/navigation_2d/src/mission_vision.cpp")
    for path in (header, source):
        if not os.path.isfile(path):
            fail(16, path, "not found")
            return

    header_text = open(header, encoding="utf-8").read()
    if re.search(r"recursive_mutex\s+detection_mutex_", header_text):
        fail(16, header,
             "detection_mutex_ is a recursive_mutex. That does not fix a double "
             "lock, it hides it: the inner acquisition succeeds while the outer "
             "one is still held, so the critical section silently spans the whole "
             "callback and the read/write race it was meant to prevent is back. "
             "Scope the lock instead (see processDetections' call site)")
    elif not re.search(r"std::mutex\s+detection_mutex_", header_text):
        fail(16, header, "no `std::mutex detection_mutex_` declaration found")

    text = open(source, encoding="utf-8").read()
    mask = _code_mask(text)

    span = _body_span(text, mask, "MissionVision::detectionCallback(")
    if span is None:
        fail(16, source, "detectionCallback() not found")
        return
    start, end = span
    calls = [m.start() for m in
             re.finditer(r"\bprocessDetections\s*\(", text[start:end])]
    calls = [start + c for c in calls if mask[start + c]]
    # The declaration in the header-looking place does not exist in this file, so
    # every match inside the body is a call. The definition lives in another
    # function and is outside this span.
    if not calls:
        fail(16, source,
             "detectionCallback() no longer calls processDetections(); this gate "
             "can no longer see the frame path it is guarding - update it rather "
             "than deleting it")
        return
    # Anything holding detection_mutex_ while the call is made is a deadlock,
    # whether the guard is brace-scoped or a plain statement in the function body
    # (the 2026-10-18 shape was the plain-statement one, which is why counting
    # braces alone proves nothing). So the test is lifetime-based: a guard is
    # live from its declaration to the closing brace of the block that declares
    # it, and the call must not fall inside that window.
    depth = _depth_array(text, mask, start, end)
    guards = []
    for m in re.finditer(
            r"(?:lock_guard|unique_lock)\s*<\s*std::mutex\s*>\s+(\w+)\s*\(\s*"
            r"detection_mutex_\s*\)", text[start:end]):
        at = start + m.start()
        if not mask[at]:
            continue
        scope = depth[at]
        closes = end
        for j in range(at + 1, end + 1):
            if j <= end and mask[j] and text[j] == "}" and depth[j] == scope:
                closes = j
                break
        guards.append((at, closes, m.group(1)))

    live = []
    blocked = False
    for call in calls:
        covering = [g for g in guards if g[0] < call < g[1]]
        if covering:
            blocked = True
            at, _, name = covering[0]
            fail(16, source,
                 "processDetections() (line %d) is called while '%s' still holds "
                 "detection_mutex_ (declared line %d). processDetections -> "
                 "findDetectionCandidates re-locks the same mutex on the same "
                 "thread, and std::mutex is not recursive: the node deadlocks on "
                 "the first non-IDLE frame, keeps answering nothing, and the "
                 "mission reads that as 'no stable detection, found=0'. Scope "
                 "the guard to the swap" %
                 (text.count("\n", 0, call) + 1, name, text.count("\n", 0, at) + 1))
        else:
            line = text.count("\n", 0, call) + 1
            live.append(line)
    if live and not blocked:
        print("  detectionCallback -> processDetections() at line %s with no "
              "detection_mutex_ guard alive: %d guard(s) in the function, none "
              "covering the call" % (", ".join(str(n) for n in live), len(guards)))

    span = _body_span(text, mask, "MissionVision::findDetectionCandidates(")
    if span is None:
        fail(16, source, "findDetectionCandidates() not found")
        return
    start, end = span
    body = text[start:end]
    held = re.search(r"lock_guard<std::mutex>\s+\w+\s*\(\s*detection_mutex_\s*\)", body)
    if not held:
        fail(16, source,
             "findDetectionCandidates() reads latest_detections_ without taking "
             "detection_mutex_; detectionCallback swaps that vector from another "
             "spinner thread, so this is a data race, not a simplification")
    else:
        print("  findDetectionCandidates(): read of the detection cache is still "
              "under detection_mutex_")

    # The other half of the 2026-10-18 failure: setMode()/mode() are called from
    # the MISSION thread (setVisionMode <- abortMission <- processTask, i.e. a
    # timer callback). If either of them takes a lock the vision layer also holds,
    # a wedged detector can stop the aircraft from ever landing: abortMission
    # printed "Mission aborted", blocked on MissionVision's mutex, and
    # processMission's ABORT branch - the only place that sends AUTO.LAND - never
    # ran again. The aircraft hovered until the battery would have ended.
    # A single enum makes locking unnecessary: one atomic exchange.
    for signature, label in (("MissionVision::setMode(", "setMode()"),
                             ("MissionVision::mode(", "mode()")):
        span = _body_span(text, mask, signature)
        if span is None:
            fail(16, source, "%s not found" % label)
            continue
        guard = None
        for m in re.finditer(r"(?:lock_guard|unique_lock)\s*<\s*std::mutex\s*>\s+"
                             r"\w+\s*\(\s*mutex_\s*\)", text[span[0]:span[1]]):
            if mask[span[0] + m.start()]:
                guard = span[0] + m.start()
                break
        if guard is not None:
            fail(16, source,
                 "%s takes mutex_ (line %d). It is called from the mission timer "
                 "thread, so a wedged vision thread makes abortMission() block "
                 "before it can set state_ = ABORT, and AUTO.LAND is never "
                 "requested - the aircraft hovers with 'Mission aborted' already "
                 "in the log. Mode is one enum: use the atomic exchange"
                 % (label, text.count("\n", 0, guard) + 1))
        else:
            print("  %-11s takes no lock: the mission thread cannot be blocked by "
                  "the vision layer" % label)


def check_sim_payload():
    """A drop command in simulation must create something that falls.

    On 2026-10-18 the mission reported a completed HAZARD_DROP - it had moved on
    to person_route[1] - while Gazebo showed nothing being released. Nothing was
    broken: use_real_servos is false in both simulation wrappers, so no servo
    node runs, px4_sensor_rig answers /gpio_cmd with "<cmd>_DONE" instead, and
    the mission believes the drop because that is the only evidence it has. The
    payload simply did not exist anywhere.

    That is why this is a gate and not a comment. Every failure mode below is
    silent in the same direction: the log says the drop succeeded.
    """
    heading(17, "sim: a drop command creates a payload that falls")

    source = os.path.join(ROOT, "src/gc_2026_sim/src/px4_sensor_rig.cpp")
    launch = os.path.join(ROOT, "src/gc_2026_sim/launch/px4_sitl_field.launch")
    for path in (source, launch):
        if not os.path.isfile(path):
            fail(17, path, "not found")
            return

    text = open(source, encoding="utf-8").read()
    mask = _code_mask(text)

    if re.search(r'nh_\.subscribe\(\s*"/gpio_cmd"', text):
        print("  /gpio_cmd subscribed here: a drop command is observable in sim")
    else:
        fail(17, source,
             "/gpio_cmd is no longer subscribed in this node. In simulation "
             "use_real_servos=false, so no servo node exists: this node is both "
             "the ack source and the only place a drop command can be seen")

    if re.search(r'"/gazebo/spawn_sdf_model"', text):
        print("  spawn service: /gazebo/spawn_sdf_model")
    else:
        fail(17, source,
             "/gazebo/spawn_sdf_model is not used; whatever service is named "
             "here, it must be one gazebo_ros actually advertises")

    if re.search(r'advertise<visualization_msgs::MarkerArray>\s*\(\s*'
                 r'"/simulation/payloads"', text):
        print("  /simulation/payloads: the released payload is visible in "
              "Foxglove, not only in Gazebo's own GUI")
    else:
        fail(17, source,
             "nothing is published for the payload. Gazebo's GUI is not where "
             "this project watches a run: without a marker the released object "
             "exists in the physics engine and nowhere the operator is looking")

    span = _body_span(text, mask, "void gpioCallback(")
    if span is None:
        fail(17, source, "gpioCallback() not found")
        return
    body = text[span[0]:span[1]]
    missing = [c for c in ("DROP_FIRE", "DROP_RESCUE") if c not in body]
    if missing:
        fail(17, source,
             "gpioCallback() has no branch for %s. It acks every command, so the "
             "mission leaves its drop state either way - a missing branch here "
             "produces a drop that never happened and no log line about it"
             % " / ".join(missing))
    else:
        print("  gpioCallback(): DROP_FIRE and DROP_RESCUE both queue a release")

    if re.search(r"spawn_client_\.call\s*\(", body):
        fail(17, source,
             "gpioCallback() calls spawn_client_.call() directly. That callback "
             "runs on the same spinner thread as /gazebo/model_states, and a "
             "blocking service call there stalls odom->base_link - the exact stall "
             "sync_timer_ was introduced to remove. Queue it for the timer")
    else:
        print("  gpioCallback(): no service call inside the callback body")

    span = _body_span(text, mask, "static std::string payloadSdf(")
    if span is None:
        fail(17, source, "payloadSdf() not found")
    else:
        body = text[span[0]:span[1]]
        if "<static>false</static>" not in body:
            fail(17, source,
                 "the payload SDF is not <static>false</static>. A static model "
                 "spawns without error and then hangs in mid-air, which on screen "
                 "is indistinguishable from a drop that did not happen")
        elif not re.search(r"<collision\b", body):
            fail(17, source,
                 "the payload SDF has no <collision>: it accelerates straight "
                 "through the ground plane and vanishes")
        else:
            print("  payload SDF: <static>false</static> plus a collision, so it "
                  "falls and comes to rest")

    depth = re.search(r'param<double>\("payload_release_depth",\s*'
                      r'payload_release_depth_,\s*([0-9.]+)\)', text)
    if depth is None:
        fail(17, source, "payload_release_depth is not read in the constructor")
    else:
        value = float(depth.group(1))
        if value < 0.20:
            fail(17, source,
                 "payload_release_depth default is %.2f m. The rig's camera sits "
                 "at -0.15 m, so anything shallower than 0.20 m puts an 80 mm box "
                 "against the lens for the first frames after the drop" % value)
        else:
            print("  payload_release_depth: %.2f m below the body "
                  "(camera is at -0.15 m)" % value)

    launch_text = open(launch, encoding="utf-8").read()
    m = re.search(r'<param\s+name="payload_on_drop"\s+value="([^"]*)"', launch_text)
    if m is None:
        fail(17, launch,
             "payload_on_drop is not set in the node block. The code default is "
             "true, but this is the only launch that starts the rig; an implicit "
             "switch is how a simulation quietly stops simulating")
    elif m.group(1).strip().lower() not in ("true", "1"):
        fail(17, launch,
             "payload_on_drop=%r: every drop will be acknowledged and then "
             "discarded, and nothing will appear in Gazebo" % m.group(1))
    else:
        print("  launch: payload_on_drop=true (the rig is started by this file "
              "only)")

    # The CMakeLists / package.xml pair, because this is the one dependency
    # mistake that is invisible until a build and then costs a full cmake run.
    # catkin's own words:
    #   catkin_package() the catkin package 'visualization_msgs' has been
    #   find_package()-ed but is not listed as a build dependency in the
    #   package.xml
    # That is exactly what happened on 2026-10-18 when visualization_msgs was
    # added to CMakeLists for the payload marker and package.xml was forgotten.
    cmake_path = os.path.join(ROOT, "src/gc_2026_sim/CMakeLists.txt")
    pkg_path = os.path.join(ROOT, "src/gc_2026_sim/package.xml")
    if not (os.path.isfile(cmake_path) and os.path.isfile(pkg_path)):
        fail(17, cmake_path, "CMakeLists.txt / package.xml not found")
        return
    cmake_text = open(cmake_path, encoding="utf-8").read()
    pkg_text = open(pkg_path, encoding="utf-8").read()
    declared = set(re.findall(r"<build_depend>\s*([^<]+?)\s*</build_depend>",
                              pkg_text))
    declared |= set(re.findall(r"<depend>\s*([^<]+?)\s*</depend>", pkg_text))
    named = []
    m = re.search(r"find_package\(\s*catkin\s+REQUIRED\s+COMPONENTS(.*?)\)",
                  cmake_text, re.S)
    if m:
        named += re.findall(r"[A-Za-z_]\w*", m.group(1))
    m = re.search(r"catkin_package\((.*?)\)", cmake_text, re.S)
    if m:
        cut = re.search(r"CATKIN_DEPENDS(.*?)(?:\bDEPENDS\b|\bINCLUDE_DIRS\b|"
                        r"\bLIBRARIES\b|\bCFG_EXTRAS\b|$)", m.group(1), re.S)
        if cut:
            named += re.findall(r"[A-Za-z_]\w*", cut.group(1))
    missing = sorted(set(named) - declared)
    if missing:
        fail(17, pkg_path,
             "CMakeLists names %s but package.xml declares no <build_depend> for "
             "it. catkin refuses to configure - after the build has already "
             "started - and the message points at catkin_package, not at the "
             "missing line" % ", ".join(missing))
    else:
        print("  CMakeLists dependencies: %d named, all declared in package.xml"
              % len(set(named)))


def check_route_goal_topic():
    """The mission's waypoint stream must own its goal topic outright.

    mavros_extras' guided_target plugin advertises /move_base_simple/goal and
    re-publishes every POSITION_TARGET_GLOBAL_INT the FCU streams (~10 Hz in any
    position-control mode) as a PoseStamped stamped "map" with z zeroed - because
    is_map_init is false here, there is no EKF origin, and the log line that says
    so is

        [WARN] PositionTargetGlobal failed because no origin

    printed at the full 10 Hz. On 2026-10-18 that was the loudest thing in the
    log, and it looked like the localisation package failing. It is not: it is
    mavros, and it is cosmetic on its own.

    What is not cosmetic is the other end of the same plugin. EGO's goal input
    was remapped onto /move_base_simple/goal, and EgoPlanner2DNode::goalCallback
    answers any goal whose frame_id differs from global_frame ("odom") with
    clearPlan(). So a foreign publisher sitting on a shared default topic could
    wipe the plan of a leg already being flown, and the only trace was
        "Ignoring goal in frame 'map'; expected 'odom'."
    on a line that looks like a harmless warning.

    The collision is the bug, so it is removed structurally: the mission
    advertises /mission/route_goal, the launch points EGO at it, and this gate
    keeps the two ends equal and keeps that name off /move_base_simple/goal.
    Once the topic is private, the frame check in goalCallback is a real
    invariant instead of a load-bearing safety net.
    """
    heading(18, "route goal topic is private to the mission")

    source = os.path.join(ROOT, "src/navigation_2d/src/rescue_mission.cpp")
    launch = os.path.join(ROOT, "src/navigation_2d/launch/navigation_2d.launch")
    shared = "/move_base_simple/goal"
    for path in (source, launch):
        if not os.path.isfile(path):
            fail("route-goal", path, "not found")
            return

    source_text = open(source, encoding="utf-8").read()
    advertised = re.findall(
        r'goal_pub_\s*=\s*nh_\.advertise<geometry_msgs::PoseStamped>\(\s*'
        r'"([^"]+)"', source_text)
    if not advertised:
        fail("route-goal", source,
             "cannot find the goal_pub_ advertise() topic; this gate reads it "
             "by name, so it must stay on the goal_pub_ = nh_.advertise<...> form")
        return
    goal_topic = advertised[0]

    try:
        import xml.etree.ElementTree as ET
        root = ET.parse(launch).getroot()
    except Exception as exc:
        fail("route-goal", launch, "cannot parse: %s" % exc)
        return

    remapped = []
    for node in root.iter("node"):
        for remap in node.iter("remap"):
            if (remap.get("from") or "") == "goal":
                remapped.append(((remap.get("to") or ""), node.get("name") or "?"))

    if not remapped:
        fail("route-goal", launch,
             "no node remaps its 'goal' input any more. The planner is then fed "
             "by whatever happens to publish on the default topic - which is "
             "exactly the 2026-10-18 failure")
        return

    print("  mission publishes goals on: %s" % goal_topic)
    for target, name in remapped:
        print("  %-24s subscribes 'goal' <- %s" % (name, target))
        if target != goal_topic:
            fail("route-goal", launch,
                 "node %s consumes goals from %r but the mission publishes on %r. "
                 "A mismatch produces no error: the planner just keeps its last "
                 "plan and the vehicle hovers (2026-09-25). Make them the same topic."
                 % (name, target, goal_topic))
        if target == shared:
            fail("route-goal", launch,
                 "node %s is wired to %s, which mavros_extras' guided_target plugin "
                 "also advertises. It republishes the FCU's POSITION_TARGET_GLOBAL_INT "
                 "as a 'map'-frame PoseStamped at ~10 Hz, and the planner's frame "
                 "check answers that with clearPlan() - a remote wipe of the current "
                 "leg. Use a topic only the mission publishes." % (name, shared))

    # Nothing in this workspace may publish on the shared default name.
    offenders = []
    for path in glob.glob(os.path.join(ROOT, "src/**/*.cpp"), recursive=True) + \
                glob.glob(os.path.join(ROOT, "src/**/*.py"), recursive=True):
        body = open(path, encoding="utf-8").read()
        if re.search(r'advertise<[^>]*>\(\s*"%s"' % re.escape(shared), body):
            offenders.append(os.path.relpath(path, ROOT))
    if offenders:
        fail("route-goal", ROOT,
             "these nodes advertise %s, the topic guided_target also publishes: %s"
             % (shared, ", ".join(offenders)))
    else:
        print("  no node in src/ publishes %s (guided_target owns it)" % shared)


def check_baseline(path):
    heading(15, "parameter diff vs baseline")

    try:
        import yaml
    except ImportError:
        warn("baseline", ROOT, "PyYAML not installed; skipped")
        return

    def flat(d, p=""):
        out = {}
        if isinstance(d, dict):
            for k, v in d.items():
                out.update(flat(v, "%s%s." % (p, k)))
        else:
            out[p[:-1]] = d
        return out

    pairs = []
    for ours in sorted(glob.glob(os.path.join(ROOT, "src/*/param/*.yaml"))):
        base = os.path.join(path, os.path.relpath(ours, os.path.join(ROOT, "src")))
        if os.path.exists(base):
            pairs.append((ours, base))
    if not pairs:
        warn("baseline", path, "no matching files; wrong --baseline?")
        return

    total = differing = 0
    for ours, base in pairs:
        b = flat(yaml.safe_load(open(base, encoding="utf-8")) or {})
        o = flat(yaml.safe_load(open(ours, encoding="utf-8")) or {})
        keys = set(b) | set(o)
        total += len(keys)
        d = [k for k in sorted(keys) if b.get(k) != o.get(k)]
        differing += len(d)
        print("  %-40s keys=%3d differing=%d" % (os.path.relpath(ours, ROOT), len(keys), len(d)))
        for k in d:
            print("      %-42s baseline=%-12r ours=%r"
                  % (k, b.get(k, "<absent>"), o.get(k, "<absent>")))
    print("  TOTAL keys=%d differing=%d" % (total, differing))
    print("  Every differing key above must carry a written reason in its file header.")
    print("  A difference without a reason is not an exception, it is a leftover.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline", metavar="SRC_DIR",
                    help="path to the reference workspace's src/ for a parameter diff")
    args = ap.parse_args()

    print("workspace: %s" % ROOT)
    check_yaml()
    check_launch()
    check_lua()
    check_frames()
    check_planner_frames()
    check_ego_local_planner()
    check_cmd_vel_convention()
    check_stage_mission()
    check_world_resources()
    check_servo()
    check_vision_handshake()
    check_startup_gates()
    check_vision_locking()
    check_sim_payload()
    check_route_goal_topic()
    if args.baseline:
        check_baseline(args.baseline)

    print("\n" + "=" * 70)
    if FAILURES:
        print("%d FAILURE(S) - do not rebuild yet:" % len(FAILURES))
        for c, p, m in FAILURES:
            print("  [%s] %s: %s" % (c, os.path.relpath(p, ROOT), m))
    else:
        print("no failures")
    if WARNINGS:
        print("%d warning(s) - read them, they are how this project has burned runs" % len(WARNINGS))
    print("=" * 70)
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
