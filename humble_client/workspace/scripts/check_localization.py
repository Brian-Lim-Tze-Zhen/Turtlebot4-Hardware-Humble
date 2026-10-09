#!/usr/bin/env python3
"""check_localization.py - pre-run check of the AMCL pose against the map (9 Oct 2026).

Why: the pass side is chosen from the walker's sideways offset at ~6.4 m. An AMCL heading error of
2 deg moves that walker 0.22 m sideways, 3.5 deg 0.39 m; on 9 Oct this sent the robot LEFT in runs
where the walker was centred (Task/HARDWARE_DAY_20261009.md, "Why LEFT").

What: collects ~2 s of LiDAR scans with the robot standing still, places them on the map with the
current map->base_link transform, and searches heading (+/-8 deg, 0.25 deg steps) and position
(+/-0.20 m, 0.05 m steps) for the best fit (mean distance to the nearest occupied map cell, capped at
0.5 m). Prints the correction and OK / NOT OK.

--fix publishes the corrected pose on /turtlebot4/initialpose (like RViz "2D Pose Estimate", small
covariance). Run the check again afterwards.

usage (in the container, robot standing still at the start):
  python3 scripts/check_localization.py [--map map/hall_map_name.yaml] [--fix] \
    --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
exit: 0 OK, 1 NOT OK, 2 no data
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

NS = '/turtlebot4'
YAW_OK_DEG = 1.0
XY_OK_M = 0.10
COLLECT_S = 2.0


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


class Check(Node):
    def __init__(self):
        super().__init__('check_localization')
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.scans = []
        self.create_subscription(LaserScan, f'{NS}/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, f'{NS}/initialpose', 10)

    def scan_cb(self, m):
        try:
            tr = self.buf.lookup_transform('map', m.header.frame_id, Time(), Duration(seconds=0.2))
        except Exception:
            return
        rg = np.array(m.ranges)
        a = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > 0.2) & (rg < 8.0)
        self.scans.append((rg[ok], a[ok], tr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='/root/thesis_social_navigation_ws/map/hall_map_name.yaml')
    ap.add_argument('--fix', action='store_true')
    args, _ = ap.parse_known_args()

    M = yaml.safe_load(open(args.map))
    img = np.array(Image.open(os.path.join(os.path.dirname(args.map), M['image'])))
    occ = img < 100
    res = M['resolution']
    ox0, oy0 = M['origin'][:2]
    rows = img.shape[0]
    dist = distance_transform_edt(~occ) * res

    def fit(X, Y):
        c = ((X - ox0) / res).astype(int)
        r = (rows - 1 - ((Y - oy0) / res)).astype(int)
        ok = (c >= 0) & (c < img.shape[1]) & (r >= 0) & (r < rows)
        d = np.full(X.shape, 0.5)
        d[ok] = np.minimum(dist[r[ok], c[ok]], 0.5)
        return float(np.mean(d))

    rclpy.init(args=sys.argv)
    n = Check()
    t0 = time.time()
    while time.time() - t0 < COLLECT_S + 3.0 and len(n.scans) < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.scans) < 3:
        print('NO DATA: fewer than 3 scans with a map transform (Nav2/AMCL running? /tf remapped?)')
        sys.exit(2)

    # Robot pose (map -> base_link) and all scan points in the map frame as given by AMCL.
    tr = n.buf.lookup_transform('map', 'base_link', Time(), Duration(seconds=0.5))
    rx, ry = tr.transform.translation.x, tr.transform.translation.y
    ryaw = yaw_of(tr.transform.rotation)
    P = []
    for rg, a, t in n.scans:
        th = yaw_of(t.transform.rotation)
        P.append(np.stack([t.transform.translation.x + rg * np.cos(a + th),
                           t.transform.translation.y + rg * np.sin(a + th)]))
    P = np.concatenate(P, axis=1)
    if P.shape[1] > 4000:   # keep the grid search quick
        P = P[:, np.random.default_rng(0).choice(P.shape[1], 4000, replace=False)]
    # Points relative to the robot, so a heading change rotates them about the robot.
    dx, dy = P[0] - rx, P[1] - ry

    base = fit(P[0], P[1])
    best = (base, 0.0, 0.0, 0.0)
    for dyaw in np.arange(-8.0, 8.001, 0.25):
        c, s = math.cos(math.radians(dyaw)), math.sin(math.radians(dyaw))
        X0 = rx + c * dx - s * dy
        Y0 = ry + s * dx + c * dy
        for sx in np.arange(-0.20, 0.201, 0.05):
            for sy in np.arange(-0.20, 0.201, 0.05):
                f = fit(X0 + sx, Y0 + sy)
                if f < best[0] - 1e-6:
                    best = (f, dyaw, sx, sy)
    f, dyaw, sx, sy = best
    ok = abs(dyaw) <= YAW_OK_DEG and math.hypot(sx, sy) <= XY_OK_M
    print(f'AMCL pose  x {rx:.2f}  y {ry:.2f}  heading {math.degrees(ryaw):.1f} deg')
    print(f'scan fit   {base:.3f} m as is, {f:.3f} m after correction '
          f'(heading {dyaw:+.2f} deg, x {sx:+.2f} m, y {sy:+.2f} m), {P.shape[1]} points')
    # dyaw > 0: the true heading is further left than AMCL thinks, so everything ahead is placed too far
    # to the robot's right.
    shift = 6.4 * math.sin(math.radians(dyaw))
    print(f"a walker 6.4 m ahead is placed {abs(shift):.2f} m too far to the robot's {'right' if shift > 0 else 'left'}")
    print('OK' if ok else f'NOT OK (limits: heading {YAW_OK_DEG} deg, position {XY_OK_M} m)')

    if args.fix:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.pose.position.x = rx + sx
        m.pose.pose.position.y = ry + sy
        yaw = ryaw + math.radians(dyaw)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(1.0) ** 2
        m.pose.covariance = cov
        for _ in range(3):
            n.pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.2)
        print(f'initialpose sent: x {rx + sx:.2f} y {ry + sy:.2f} heading {math.degrees(yaw):.1f} deg - run the check again')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
