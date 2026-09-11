#!/usr/bin/env python3
import math, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

FX, CX, YAW, CAMX = 287.31, 124.95, math.radians(2.25), 0.165
WIN = math.radians(10.0)      # search +/-10 deg around camera bearing
RMIN, RMAX = 0.4, 3.5         # m
LEG = 0.35                    # m, leg cluster radius from nearest point

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Check(Node):
    def __init__(self):
        super().__init__("bearing_check_scan")
        self.tf = Buffer(); self.tl = TransformListener(self.tf, self)
        self.pts, self.pts_t, self.diffs, self.last = None, 0.0, [], 0.0
        for topic in ("/scan", "/turtlebot4/scan"):
            self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/person_positions_map", self.on_cam, 10)

    def on_scan(self, m):
        try: t = self.tf.lookup_transform("base_link", m.header.frame_id, Time())
        except Exception:
            print(f"no TF base_link->{m.header.frame_id}"); return
        tx, ty = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        c, s = math.cos(yw), math.sin(yw)
        pts, a = [], m.angle_min
        for r in m.ranges:
            if math.isfinite(r) and RMIN <= r <= RMAX:
                sx, sy = r*math.cos(a), r*math.sin(a)
                pts.append((tx + c*sx - s*sy, ty + s*sx + c*sy))
            a += m.angle_increment
        self.pts, self.pts_t = pts, time.time()

    def on_cam(self, m):
        p = m.data.split(",")
        if len(p) < 6 or self.pts is None or time.time() - self.pts_t > 0.5: return
        x1, x2 = float(p[2]), float(p[4])
        k = []
        if len(p) >= 7 and p[6].strip():
            for pr in p[6].split(";"):
                try: a, b = pr.split(":"); k.append((float(a), float(b)))
                except ValueError: pass
        if k:
            k.sort(key=lambda q: -q[1]); use = k[:2]
            u = sum(q[0] for q in use) / len(use)
        else:
            u = (x1 + x2) / 2
        cb = -math.atan((u - CX) / FX) + YAW
        cand = [q for q in self.pts if abs(wrap(math.atan2(q[1], q[0] - CAMX) - cb)) < WIN]
        if not cand: return
        near = min(cand, key=lambda q: math.hypot(q[0], q[1]))
        cl = [q for q in cand if math.hypot(q[0]-near[0], q[1]-near[1]) < LEG]
        mx = sum(q[0] for q in cl) / len(cl); my = sum(q[1] for q in cl) / len(cl)
        lb = math.atan2(my, mx - CAMX)
        d = math.degrees(wrap(lb - cb))
        self.diffs = (self.diffs + [d])[-40:]
        now = time.time()
        if now - self.last < 0.5: return
        self.last = now
        sd = statistics.pstdev(self.diffs) if len(self.diffs) > 1 else 0.0
        print(f"cam {math.degrees(cb):+6.1f} | scan {math.degrees(lb):+6.1f} at base "
              f"x={mx:.2f} y={my:+.2f} | diff {d:+5.1f} | median40 "
              f"{statistics.median(self.diffs):+5.1f} std {sd:4.1f}")

rclpy.init(); n = Check()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
