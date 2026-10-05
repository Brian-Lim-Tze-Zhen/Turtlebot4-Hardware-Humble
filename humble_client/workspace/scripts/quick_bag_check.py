#!/usr/bin/env python3
"""
quick_bag_check.py <bag_dir>

Fast check of a hardware run right after recording, so a bad bag is
noticed while the robot is still set up. Not an analysis script; the
thesis numbers come from pytest/analysis/headon_metrics.py and
hardware_analysis/hw_extra.py.

Reports
  topics      required topics present / empty / missing, with counts
  motion      duration of commanded motion, distance driven (odom)
  reversing   time with commanded vx < -REV_V and with odom vx < -REV_V
  stops       commanded standstill >= STOP_MIN_S inside the motion span
  bumper      hazard_detection entries by type and frame
  LiDAR       nearest return within +/-FRONT_HALF_DEG ahead while moving
              (includes walls)
  people      samples and stable ids on /person_positions_fused, groups
"""
import math
import sys
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

P = "/turtlebot4"
REQUIRED = [
    f"{P}/scan", f"{P}/tf", f"{P}/tf_static", f"{P}/odom", f"{P}/cmd_vel",
    f"{P}/hazard_detection", f"{P}/plan", f"{P}/map",
    f"{P}/global_costmap/costmap", f"{P}/local_costmap/costmap",
    "/person_positions_map", "/camera_ray_clusters",
    "/person_positions_fused", "/predicted_person_positions",
    "/predicted_person_cloud",
    f"{P}/oakd/rgb/preview/image_raw/compressed",
]
# Needed only for the standing-pair runs; reported but not counted as missing
# for a head-on run where the group nodes are not started.
GROUP = ["/social_groups", "/social_zone_map"]
# hazard_detection may legitimately have 0 messages (no hazard occurred).
MAY_BE_EMPTY = {f"{P}/hazard_detection", "/social_groups"}

REV_V = 0.01
STOP_V = 0.02
STOP_W = 0.05
STOP_MIN_S = 0.5
LIDAR_MOUNT_YAW_DEG = 90.0
FRONT_HALF_DEG = 30.0
HAZARD_NAMES = {0: "backup_limit", 1: "bump", 2: "cliff", 3: "stall",
                4: "wheel_drop", 5: "object_proximity"}


def main(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    read = {f"{P}/cmd_vel", f"{P}/odom", f"{P}/scan", f"{P}/hazard_detection",
            "/person_positions_fused", "/social_groups"}
    cls = {}
    for t in read:
        if t in types:
            try:
                cls[t] = get_message(types[t])
            except Exception as e:  # message package not installed
                print(f"cannot decode {t} ({types[t]}): {e}")

    counts = Counter()
    t_first = t_last = None
    cmd, odom, front, hazards = [], [], [], []
    person_ids, sources, groups = Counter(), Counter(), Counter()

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        counts[topic] += 1
        t_first = t if t_first is None else t_first
        t_last = t
        if topic not in cls:
            continue
        msg = deserialize_message(data, cls[topic])
        if topic == f"{P}/cmd_vel":
            tw = msg.twist if hasattr(msg, "twist") else msg
            cmd.append((t, tw.linear.x, tw.angular.z))
        elif topic == f"{P}/odom":
            odom.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                         msg.twist.twist.linear.x))
        elif topic == f"{P}/scan":
            best = None
            a = msg.angle_min
            for r in msg.ranges:
                rb = (math.degrees(a) + LIDAR_MOUNT_YAW_DEG + 180.0) % 360.0 - 180.0
                if abs(rb) <= FRONT_HALF_DEG and 0.15 < r < 12.0:
                    if best is None or r < best[0]:
                        best = (r, rb)
                a += msg.angle_increment
            if best:
                front.append((t, best[0], best[1]))
        elif topic == f"{P}/hazard_detection":
            for d in msg.detections:
                hazards.append((t, d.type, d.header.frame_id))
        elif topic == "/person_positions_fused":
            f = msg.data.split(",")
            if f and f[0].lstrip("-").isdigit():
                person_ids[int(f[0])] += 1
                sources[f[11] if len(f) > 11 else "?"] += 1
        elif topic == "/social_groups":
            f = msg.data.split(",")
            if len(f) > 1:
                groups[f[1].strip()] += 1

    # In a run without people the person topics are expected to be silent.
    may_be_empty = set(MAY_BE_EMPTY)
    if "/empty_" in bag or bag.startswith("empty_"):
        may_be_empty |= {"/person_positions_map", "/camera_ray_clusters",
                         "/person_positions_fused", "/predicted_person_positions"}

    dur = (t_last - t_first) if t_first is not None else 0.0
    print(f"bag: {bag}")
    print(f"duration: {dur:.1f} s   topics: {len(types)}   messages: {sum(counts.values())}")

    # --- topics ---
    print("\nrequired topics:")
    bad = 0
    for t in REQUIRED + GROUP:
        n = counts.get(t, 0)
        if t not in types:
            state = "not recorded" if t in GROUP else "MISSING"
            bad += t not in GROUP
        elif n == 0:
            state = "ok (no messages)" if t in may_be_empty else "EMPTY"
            bad += t not in may_be_empty and t not in GROUP
        else:
            state = "ok"
        rate = f"{n / dur:6.1f} Hz" if dur > 0 and n else ""
        print(f"  {state:17s} {n:7d} {rate:10s} {t}")

    # --- motion ---
    moving = [c for c in cmd if abs(c[1]) > STOP_V or abs(c[2]) > STOP_W]
    if not moving:
        print("\nno commanded motion in this bag")
    else:
        t0, t1 = moving[0][0], moving[-1][0]
        dist = sum(math.hypot(odom[i][1] - odom[i - 1][1], odom[i][2] - odom[i - 1][2])
                   for i in range(1, len(odom)) if t0 <= odom[i][0] <= t1)
        print(f"\nmotion: {t1 - t0:.1f} s commanded, {dist:.2f} m driven (odom), "
              f"starts {t0 - t_first:.1f} s into the bag")

        def time_below(samples, idx):
            tot = 0.0
            for i in range(1, len(samples)):
                dt = samples[i][0] - samples[i - 1][0]
                if t0 <= samples[i][0] <= t1 and samples[i][idx] < -REV_V and dt < 0.5:
                    tot += dt
            return tot

        print(f"reversing: commanded {time_below(cmd, 1):.1f} s, "
              f"odom {time_below(odom, 3):.1f} s (vx < -{REV_V} m/s)")
        if cmd:
            print(f"commanded vx: max {max(c[1] for c in cmd):.2f}, "
                  f"min {min(c[1] for c in cmd):.2f} m/s; "
                  f"|wz| max {max(abs(c[2]) for c in cmd):.2f} rad/s")

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
        print(f"stops (>= {STOP_MIN_S} s): {len(stops)}" +
              "".join(f"\n  t+{s:.1f} s for {d:.1f} s" for s, d in stops))

        fr = [f for f in front if t0 <= f[0] <= t1]
        if fr:
            m = min(fr, key=lambda f: f[1])
            print(f"nearest LiDAR return within +/-{FRONT_HALF_DEG:.0f} deg ahead while moving "
                  f"(includes walls): {m[1]:.2f} m at t+{m[0] - t0:.1f} s, bearing {m[2]:+.0f} deg")

    # --- bumper / hazards ---
    if f"{P}/hazard_detection" not in cls:
        print("\nhazards: topic not decoded")
    elif not hazards:
        print("\nhazards: none")
    else:
        by = Counter((HAZARD_NAMES.get(ty, str(ty)), fr) for _, ty, fr in hazards)
        print("\nhazards:")
        for (name, frame), n in sorted(by.items()):
            first = min(t for t, ty, fr in hazards
                        if HAZARD_NAMES.get(ty, str(ty)) == name and fr == frame)
            print(f"  {name:16s} {frame:22s} x{n}  first at {first - t_first:.1f} s into the bag")
        if any(ty == 1 for _, ty, _ in hazards):
            print("  ** BUMPER CONTACT in this run **")

    # --- people ---
    print(f"\npeople: {sum(person_ids.values())} fused samples, "
          f"{len(person_ids)} stable id(s) {dict(person_ids)}")
    if sources:
        print(f"  sources: {dict(sources)}")
    if groups:
        print(f"  groups published: {dict(groups)}")

    print("\nRESULT: " + ("recording complete" if bad == 0
                          else f"{bad} required topic(s) missing or empty"))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
