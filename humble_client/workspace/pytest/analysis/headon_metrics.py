#!/usr/bin/env python3
"""
headon_metrics.py <bag_dir>

Offline metrics for a head-on / walk-past run on the real TurtleBot4.

Robot pose  : map->odom (AMCL) composed with odom->base_link, from /turtlebot4/tf
Person pose : /person_positions_fused  "stable_id,conf,x,y,...,source,cam_id" (map frame)
Commands    : /turtlebot4/cmd_vel (Twist or TwistStamped)

Reports
  first detection      time + robot-person distance when the person first appears
  closest approach     centre-to-centre and clearance (minus robot radius 0.189 m)
  stops                cmd_vel ~0 for >= 0.5 s after motion started
  detour               max lateral deviation of the driven path from the start->end line
  per-id summary       how many stable ids were published for the run

Note: person position is the PIPELINE ESTIMATE, not ground truth.
"""
import math
import sys
import bisect

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROBOT_RADIUS = 0.189
STOP_V = 0.02
STOP_W = 0.05
STOP_MIN_S = 0.5
MATCH_S = 0.3          # max time gap robot pose <-> person sample
LIDAR_MOUNT_YAW_DEG = 90.0
FRONT_HALF_DEG = 30.0
LIDAR_X = -0.04        # base_link -> rplidar_link x offset (tf2_echo)
SIDE = "all"           # argv[2]: all | right | left
BIN_S = 1.0            # argv[3]: bin width in seconds
MAP_MARGIN_CELLS = 3   # scan points within this many cells of an occupied map cell count as walls  # scan sector used as a pipeline-independent proximity check


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    return (ax + math.cos(ath) * bx - math.sin(ath) * by,
            ay + math.sin(ath) * bx + math.cos(ath) * by,
            ath + bth)


def nearest(ts, vals, t):
    i = bisect.bisect_left(ts, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(ts) and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
            best = j
    if best is None or abs(ts[best] - t) > MATCH_S:
        return None
    return vals[best]


def main(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {"/turtlebot4/tf", "/person_positions_fused", "/turtlebot4/cmd_vel", "/turtlebot4/scan", "/turtlebot4/map"}
    missing = [w for w in want if w not in types]
    if missing:
        print("missing topics:", missing)

    map_odom_t, map_odom = [], []
    odom_base_t, odom_base = [], []
    persons = []            # (t, sid, x, y, source)
    cmd = []                # (t, vx, wz)
    front = []              # (t, min range, robot bearing deg) in the front sector
    scans = []              # (t, [(range, robot_bearing_rad)]) for the unmapped-point check
    grid = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in want or topic not in types:
            continue
        t = t_ns * 1e-9
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/turtlebot4/tf":
            for tr in msg.transforms:
                p = (tr.transform.translation.x, tr.transform.translation.y,
                     yaw_of(tr.transform.rotation))
                parent = tr.header.frame_id.split("/")[-1]
                child = tr.child_frame_id.split("/")[-1]
                if parent == "map" and child == "odom":
                    map_odom_t.append(t); map_odom.append(p)
                elif parent == "odom" and child in ("base_link", "base_footprint"):
                    odom_base_t.append(t); odom_base.append(p)
        elif topic == "/person_positions_fused":
            f = msg.data.split(",")
            try:
                persons.append((t, int(f[0]), float(f[2]), float(f[3]),
                                f[11] if len(f) > 11 else "?"))
            except (ValueError, IndexError):
                pass
        elif topic == "/turtlebot4/scan":
            best = None
            a = msg.angle_min
            for r in msg.ranges:
                # rplidar_link is mounted at +90 deg yaw: robot bearing = lidar angle + 90
                rb = math.degrees(a) + LIDAR_MOUNT_YAW_DEG
                rb = (rb + 180.0) % 360.0 - 180.0
                if abs(rb) <= FRONT_HALF_DEG and 0.15 < r < 12.0 and not math.isinf(r):
                    if best is None or r < best[0]:
                        best = (r, rb)
                a += msg.angle_increment
            if best:
                front.append((t, best[0], best[1]))
            pts = []
            a = msg.angle_min
            for r in msg.ranges:
                if 0.15 < r < 8.0 and not math.isinf(r):
                    pts.append((r, a + math.radians(LIDAR_MOUNT_YAW_DEG)))
                a += msg.angle_increment
            scans.append((t, pts))
        elif topic == "/turtlebot4/map":
            grid = msg
        elif topic == "/turtlebot4/cmd_vel":
            tw = msg.twist if hasattr(msg, "twist") else msg
            cmd.append((t, tw.linear.x, tw.angular.z))

    if not map_odom or not odom_base:
        print(f"no pose: map->odom {len(map_odom)} samples, odom->base {len(odom_base)} samples")
        return

    def robot_at(t):
        mo = nearest(map_odom_t, map_odom, t) or map_odom[max(0, bisect.bisect_left(map_odom_t, t) - 1)]
        ob = nearest(odom_base_t, odom_base, t)
        return None if ob is None else compose(mo, ob)

    # --- motion span and path ---
    moving = [c for c in cmd if abs(c[1]) > STOP_V or abs(c[2]) > STOP_W]
    t0 = moving[0][0] if moving else odom_base_t[0]
    t1 = moving[-1][0] if moving else odom_base_t[-1]
    path = [robot_at(t) for t in odom_base_t if t0 <= t <= t1]
    path = [p for p in path if p]

    # --- distances ---
    rows = []
    for t, sid, x, y, src in persons:
        r = robot_at(t)
        if r is None:
            continue
        rows.append((t, sid, math.hypot(x - r[0], y - r[1]), src))

    print(f"bag: {bag}")
    bag_start = min([odom_base_t[0]] + ([cmd[0][0]] if cmd else []) + ([map_odom_t[0]] if map_odom_t else []))
    print(f"t+0 = {t0:.2f} (bag time), {t0 - bag_start:.1f} s after the first message -> "
          f"use --start-offset {max(0.0, t0 - bag_start):.1f} plus the t+ value to replay a moment")
    print(f"robot motion: {t1 - t0:.1f} s   person samples: {len(persons)}   matched: {len(rows)}")
    if not rows:
        print("no person samples matched to robot pose")
        return

    first = rows[0]
    print(f"\nfirst detection : t+{first[0] - t0:5.1f} s  id {first[1]}  distance {first[2]:.2f} m  ({first[3]})")

    close = min(rows, key=lambda r: r[2])
    print(f"closest approach: t+{close[0] - t0:5.1f} s  id {close[1]}  centre {close[2]:.2f} m  "
          f"clearance {close[2] - ROBOT_RADIUS:.2f} m  ({close[3]})")
    v_at = nearest([c[0] for c in cmd], [c[1] for c in cmd], close[0])
    if v_at is not None:
        print(f"                  robot vx at closest: {v_at:.2f} m/s")

    # distance profile
    print("\ndistance profile (min per 1 s bin):")
    bins = {}
    for t, sid, d, src in rows:
        k = int(t - t0)
        if k not in bins or d < bins[k][0]:
            bins[k] = (d, sid, src)
    for k in sorted(bins):
        d, sid, src = bins[k]
        print(f"  t+{k:3d}s  {d:5.2f} m  id {sid}  {src}")

    # --- stops ---
    stops, start = [], None
    for t, vx, wz in cmd:
        if t < t0 or t > t1:
            continue
        still = abs(vx) <= STOP_V and abs(wz) <= STOP_W
        if still and start is None:
            start = t
        elif not still and start is not None:
            if t - start >= STOP_MIN_S:
                stops.append((start - t0, t - start))
            start = None
    print(f"\nstops (>= {STOP_MIN_S} s): {len(stops)}" +
          "".join(f"\n  t+{s:.1f} s for {d:.1f} s" for s, d in stops))

    # --- detour ---
    if len(path) > 2:
        (sx, sy, _), (ex, ey, _) = path[0], path[-1]
        L = math.hypot(ex - sx, ey - sy)
        if L > 0.1:
            dev = [((ex - sx) * (sy - py) - (sx - px) * (ey - sy)) / L for px, py, _ in path]
            m = max(dev, key=abs)
            print(f"\ndetour: path length {sum(math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]) for i in range(1, len(path))):.2f} m, "
                  f"straight {L:.2f} m, max lateral deviation {abs(m):.2f} m")

    # --- pipeline-independent check: nearest scan return ahead ---
    fr = [f for f in front if t0 <= f[0] <= t1]
    if fr:
        m = min(fr, key=lambda f: f[1])
        print(f"\nnearest LiDAR return within +/-{FRONT_HALF_DEG:.0f} deg ahead (includes walls/objects):")
        print(f"  minimum {m[1]:.2f} m at t+{m[0] - t0:.1f} s, bearing {m[2]:+.0f} deg "
              f"(clearance {m[1] - ROBOT_RADIUS:.2f} m)")
        fb = {}
        for t, r, b in fr:
            k = int(t - t0)
            if k not in fb or r < fb[k][0]:
                fb[k] = (r, b)
        pk = {int(t - t0) for t, *_ in rows}
        print("  per 1 s bin (min range, bearing; * = person published that second):")
        for k in sorted(fb):
            r, b = fb[k]
            print(f"    t+{k:3d}s  {r:5.2f} m  {b:+4.0f} deg {'*' if k in pk else ''}")

    # --- pipeline-independent closest approach: scan points NOT on the static map ---
    if grid is None:
        print("\n(no /turtlebot4/map in bag - unmapped-point check skipped)")
    else:
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        W, H, G = grid.info.width, grid.info.height, grid.data

        def on_map(x, y):
            cx, cy = int((x - ox) / res), int((y - oy) / res)
            for dy in range(-MAP_MARGIN_CELLS, MAP_MARGIN_CELLS + 1):
                for dx in range(-MAP_MARGIN_CELLS, MAP_MARGIN_CELLS + 1):
                    X, Y = cx + dx, cy + dy
                    if 0 <= X < W and 0 <= Y < H and G[Y * W + X] >= 50:
                        return True
            return False

        un = []
        for t, pts in scans:
            if not (t0 <= t <= t1):
                continue
            rp = robot_at(t)
            if rp is None:
                continue
            lx = rp[0] + math.cos(rp[2]) * LIDAR_X
            ly = rp[1] + math.sin(rp[2]) * LIDAR_X
            best = None
            for r, b in pts:
                bd = (math.degrees(b) + 180) % 360 - 180
                if SIDE == "right" and not (-160 <= bd <= -20):
                    continue
                if SIDE == "left" and not (20 <= bd <= 160):
                    continue
                ang = rp[2] + b
                x, y = lx + r * math.cos(ang), ly + r * math.sin(ang)
                if not on_map(x, y):
                    d = math.hypot(x - rp[0], y - rp[1])
                    if best is None or d < best[0]:
                        bb = (math.degrees(b - math.radians(0)) + 180) % 360 - 180
                        best = (d, bb)
            if best:
                un.append((t, best[0], best[1]))
        if un:
            m = min(un, key=lambda u: u[1])
            print(f"\nnearest UNMAPPED LiDAR point, side={SIDE} (person or unmapped object; margin {MAP_MARGIN_CELLS} cells):")
            print(f"  minimum {m[1]:.2f} m at t+{m[0] - t0:.1f} s, bearing {m[2]:+.0f} deg "
                  f"(clearance {m[1] - ROBOT_RADIUS:.2f} m)")
            ub = {}
            for t, d, b in un:
                k = round(int((t - t0) / BIN_S) * BIN_S, 2)
                if k not in ub or d < ub[k][0]:
                    ub[k] = (d, b)
            pk = {int(t - t0) for t, *_ in rows}
            print("  per 1 s bin (* = person published that second):")
            for k in sorted(ub):
                d, b = ub[k]
                print(f"    t+{k:5.1f}s  {d:5.2f} m  {b:+5.0f} deg {'*' if int(k) in pk else ''}")

    ids = sorted({r[1] for r in rows})
    print(f"\nstable ids published: {len(ids)} -> {ids}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    if len(sys.argv) >= 3:
        SIDE = sys.argv[2]
    if len(sys.argv) >= 4:
        BIN_S = float(sys.argv[3])
    main(sys.argv[1])
