#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐条腿核算航线在 EGO 眼里还剩多宽 —— 并且**扫遍随机障碍物的每一种摆法**。

为什么需要这个工具：EGO 的 `GridMap2D` 判占用是
    occupied(cell) = distance(cell_centre, obstacle_surface) <= map_inflate_value
即**机身中心**必须比膨胀半径更远，才允许进入该栅格。所以"障碍之间物理上有
0.9 m" 并不意味着过得去：两边各要吃掉 0.375 m，剩下的才是机身中心能走的带宽。
走廊一窄，任务侧看到的就是"到位了但不往下走"，而日志里只有一句
`Planner never started moving toward the waypoint`。

⚠️ 为什么必须扫摆法：`random_obstacle_1/2` 是**现场随机摆放**的，每列各有两个
候选位置（见 gc_2026_field.world 的注释），而 world 文件只写死了一种（both-"1"）。
航线表是固定的 ⇒ 同一条腿在不同的摆法下余量可以差 0.3 m 以上。只核对 world 里
那一种，等于只验证了四分之一的比赛。

对每条航线腿做三件事：
  1. 连续量化：沿腿采样，报到障碍表面的最小水平距离，以及它比膨胀半径多/少多少
  2. 栅格量化：把采样点吸到 map_resolution 的栅格上再判一次 —— 这才是 A* 看到的
     （吸格误差 ±resolution*sqrt(2)/2，0.05 m 时是 ±0.035 m）
  3. 横截面：在最窄那一处沿垂直方向朝两边走，直到撞上"机身中心禁止进入"的边界，
     得到该截面**可通行的净宽**（已扣掉膨胀半径）。小于 0 表示这个点自己就在带里。

用法：
    python3 tools/check_route_corridor.py                 # 扫全部摆法，报最坏值
    python3 tools/check_route_corridor.py --draw L1R1     # 只看 world 里写死的那一种
    python3 tools/check_route_corridor.py --route return_route --step 0.005

退出码：任一腿在任一摆法下"连续最小距离"小于膨胀半径 ⇒ 1；否则 0。
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_config as cc  # noqa: E402  复用 world 足迹解析，避免第二份几何真相


LAUNCH = os.path.join(cc.ROOT, "src/navigation_2d/launch/navigation_2d.launch")
NODE_TYPE = "rescue_mission_node"
PLANNER_TYPE = "ego_planner_2d_node"

# 现场随机摆放：每列两个候选位置（mat 坐标换到 gazebo 的推导见 world 注释）。
RANDOM_CANDIDATES = {
    "random_obstacle_1": [("1", (0.70, -1.45)), ("2", (0.70, -2.45))],
    "random_obstacle_2": [("1", (2.10, -1.45)), ("2", (2.10, -1.95))],
}
RANDOM_RADIUS = 0.10


def _arg_default(root, name, fallback):
    for arg in root.iter("arg"):
        if arg.get("name") == name and arg.get("default") is not None:
            return arg.get("default")
    return fallback


def _planner_number(root, name, fallback):
    """读 ego 节点的参数，能把 $(arg x) 解析成默认值。"""
    raw = cc._launch_param(root, PLANNER_TYPE, name)
    if raw is None:
        return float(fallback)
    raw = raw.strip()
    if raw.startswith("$(arg ") and raw.endswith(")"):
        raw = _arg_default(root, raw[6:-1].strip(), str(fallback))
    try:
        return float(raw)
    except ValueError:
        return float(fallback)


def _draws(base):
    """-> [(label, footprints, text)]，四种摆法 + world 里写死的那种。"""
    fixed = [f for f in base if f[0] not in RANDOM_CANDIDATES]
    out = []
    for lname, left in RANDOM_CANDIDATES["random_obstacle_1"]:
        for rname, right in RANDOM_CANDIDATES["random_obstacle_2"]:
            f = list(fixed)
            f.append(("random_obstacle_1", left[0], left[1], 0.0,
                      ("cylinder", RANDOM_RADIUS)))
            f.append(("random_obstacle_2", right[0], right[1], 0.0,
                      ("cylinder", RANDOM_RADIUS)))
            out.append(("L%sR%s" % (lname, rname), f,
                        "左列@%.2f 右列@%.2f" % (left[1], right[1])))
    return out


def _distance(px, py, cx, cy, yaw, geom):
    return cc._footprint_clearance(px, py, cx, cy, yaw, geom)


def _nearest(px, py, footprints):
    """-> (distance, name)，距离为到实体表面的水平最短距离。"""
    best = (float("inf"), "?")
    for name, cx, cy, yaw, geom in footprints:
        d = _distance(px, py, cx, cy, yaw, geom)
        if d < best[0]:
            best = (d, name)
    return best


def _free_width(px, py, ux, uy, footprints, inflate, limit=1.5, step=0.005):
    """从 (px,py) 沿 ±(ux,uy) 各走一步，直到"机身中心禁止进入"，返回可通行净宽。"""
    if _nearest(px, py, footprints)[0] <= inflate:
        return -1.0
    total = 0.0
    for sign in (1.0, -1.0):
        s = 0.0
        while s < limit:
            d, _ = _nearest(px + sign * (s + step) * ux,
                            py + sign * (s + step) * uy, footprints)
            if d <= inflate:
                break
            s += step
        total += s
    return total


def check_leg(a, b, footprints, inflate, resolution, step):
    ax, ay = a
    bx, by = b
    length = math.hypot(bx - ax, by - ay)
    n = max(1, int(math.ceil(length / step)))
    worst = (float("inf"), "?", 0.0)
    lattice_blocked = 0
    for i in range(n + 1):
        t = i / float(n)
        px = ax + (bx - ax) * t
        py = ay + (by - ay) * t
        d, who = _nearest(px, py, footprints)
        if d < worst[0]:
            worst = (d, who, t)
        gx = round(px / resolution) * resolution
        gy = round(py / resolution) * resolution
        dl, _ = _nearest(gx, gy, footprints)
        if dl <= inflate:
            lattice_blocked += 1
    d, who, t = worst
    px = ax + (bx - ax) * t
    py = ay + (by - ay) * t
    ux, uy = -(by - ay) / length, (bx - ax) / length
    return {
        "min_d": d, "who": who, "at": (px, py), "margin": d - inflate,
        "lattice_blocked": lattice_blocked, "samples": n + 1,
        "width": _free_width(px, py, ux, uy, footprints, inflate),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", default=None)
    parser.add_argument("--draw", default=None, help="只看某一种摆法，如 L1R1")
    parser.add_argument("--step", type=float, default=0.01)
    args = parser.parse_args()

    root = ET.parse(LAUNCH).getroot()
    resolution = _planner_number(root, "map_resolution", 0.05)
    inflate = _planner_number(root, "map_inflate_value", 0.375)
    fly_height = float(_arg_default(root, "fly_height", "1.5"))

    base = cc._world_footprints(fly_height)
    if not base:
        print("FAIL: 读不出 world 的障碍足迹")
        return 1

    draws = _draws(base)
    if args.draw:
        draws = [d for d in draws if d[0] == args.draw]
        if not draws:
            print("没有这种摆法：%s（可选 %s）"
                  % (args.draw, ", ".join(d[0] for d in _draws(base))))
            return 1

    print("map_resolution=%.3f m  map_inflate_value=%.3f m  fly_height=%.2f m"
          % (resolution, inflate, fly_height))
    print("占用判据: distance(cell_centre, surface) <= inflate  (EGO GridMap2D)")
    print("吸格误差: +-%.3f m（半个栅格对角）" % (resolution * math.sqrt(2) / 2.0))
    print("摆法: %d 种  %s\n"
          % (len(draws), "  ".join("%s(%s)" % (d[0], d[2]) for d in draws)))

    routes = [("hazard_route", cc._launch_rosparam(root, NODE_TYPE, "hazard_route")),
              ("person_route", cc._launch_rosparam(root, NODE_TYPE, "person_route")),
              ("return_route", cc._launch_rosparam(root, NODE_TYPE, "return_route"))]

    header = ("%-13s %-3s %-31s %16s %16s %9s"
              % ("route", "leg", "segment", "margin(min@draw)", "width(min@draw)",
                 "blocked"))
    print(header)
    print("-" * len(header))

    offenders = []
    for route_name, pts in routes:
        if args.route and route_name != args.route:
            continue
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            per = [(label, check_leg(a, b, fp, inflate, resolution, args.step))
                   for label, fp, _ in draws]
            wm = min(per, key=lambda kv: kv[1]["margin"])
            ww = min(per, key=lambda kv: kv[1]["width"])
            blocked = max(kv[1]["lattice_blocked"] for kv in per)
            samples = per[0][1]["samples"]
            flag = ""
            if wm[1]["margin"] < 0:
                flag = "  <== 摆法 %s 下直线在膨胀带里" % wm[0]
            elif ww[1]["width"] < 2 * resolution:
                flag = "  <== 横截面不足两个栅格"
            print("%-13s %-3d (%5.2f,%6.2f)->(%5.2f,%6.2f) %+9.3f @%-5s %8.3f @%-5s %4d/%-4d%s"
                  % (route_name, i, a[0], a[1], b[0], b[1],
                     wm[1]["margin"], wm[0], ww[1]["width"], ww[0],
                     blocked, samples, flag))
            if wm[1]["margin"] < 0 or ww[1]["width"] < 2 * resolution:
                offenders.append((route_name, i, a, b, wm, ww, per))

    print()
    if offenders:
        print("需要盯的腿（按最坏摆法）：")
        for route_name, i, a, b, wm, ww, per in offenders:
            r = wm[1]
            print("  %s leg %d (%5.2f,%6.2f)->(%5.2f,%6.2f): 摆法 %s 时最窄点 "
                  "(%.3f, %.3f) 距 '%s' %.3f m（膨胀 %.3f，margin %+.3f），"
                  "该截面机身中心可通行 %.3f m = %.1f 格"
                  % (route_name, i, a[0], a[1], b[0], b[1], wm[0],
                     r["at"][0], r["at"][1], r["who"], r["min_d"], inflate,
                     r["margin"], ww[1]["width"], ww[1]["width"] / resolution))
            bad = [kv[0] for kv in per if kv[1]["margin"] < 0]
            if bad:
                print("      直线落在带里的摆法: %s（共 %d/%d 种）"
                      % (", ".join(bad), len(bad), len(per)))
    else:
        print("每种摆法下，每条腿的直线都在膨胀带之外且横截面够宽。")

    blocked_legs = sum(1 for o in offenders if o[4][1]["margin"] < 0)
    if blocked_legs:
        print("\n%d 条腿在至少一种摆法下直线落在膨胀带里：EGO 必须绕行，绕不过去时它返回"
              "空轨迹，任务侧表现为 'Planner never started moving'。" % blocked_legs)
    return 1 if blocked_legs else 0


if __name__ == "__main__":
    sys.exit(main())
