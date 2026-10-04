#!/usr/bin/env python3
import math, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

FX, CX, YAW, CAMX = 287.31, 124.95, math.radians(0.75), 0.165

class Check(Node):
    def __init__(self):
        super().__init__("bearing_check")
        self.tf = Buffer(); self.tl = TransformListener(self.tf, self)
        self.lidar, self.diffs, self.last = {}, [], 0.0
        self.create_subscription(String, "/lidar_person_clusters", self.on_lidar, 10)
        self.create_subscription(String, "/person_positions_map", self.on_cam, 10)

    def on_lidar(self, m):
        p = m.data.split(",")
        if len(p) < 4: return
        try: self.lidar[int(float(p[0]))] = (float(p[2]), float(p[3]), time.time())
        except ValueError: pass

    def pose(self):
        try: t = self.tf.lookup_transform("map", "base_link", Time())
        except Exception: return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))

    def on_cam(self, m):
        p = m.data.split(",")
        if len(p) < 6: return
        x1, x2 = float(p[2]), float(p[4])
        k = []
        if len(p) >= 7 and p[6].strip():
            for pr in p[6].split(";"):
                try: a, b = pr.split(":"); k.append((float(a), float(b)))
                except ValueError: pass
        if k:
            k.sort(key=lambda q: -q[1]); use = k[:2]
            u = sum(q[0] for q in use) / len(use); src = "kpt"
        else:
            u = (x1 + x2) / 2; src = "bbox"
        cb = -math.atan((u - CX) / FX) + YAW
        ps = self.pose()
        if ps is None:
            print("no TF map->base_link"); return
        rx, ry, ryaw = ps; now = time.time(); best = None
        for lid, (lx, ly, t) in self.lidar.items():
            if now - t > 0.5: continue
            dx, dy = lx - rx, ly - ry
            bx = math.cos(ryaw)*dx + math.sin(ryaw)*dy
            by = -math.sin(ryaw)*dx + math.cos(ryaw)*dy
            lb = math.atan2(by, bx - CAMX)
            d = math.atan2(math.sin(lb - cb), math.cos(lb - cb))
            if best is None or abs(d) < abs(best[0]): best = (d, lid, bx, by, lb)
        if best is None: return
        d, lid, bx, by, lb = best
        self.diffs = (self.diffs + [math.degrees(d)])[-30:]
        if now - self.last < 0.5: return
        self.last = now
        print(f"{src:4s} cam {math.degrees(cb):+6.1f} | lidar:{lid} {math.degrees(lb):+6.1f} "
              f"at base x={bx:.2f} y={by:+.2f} | diff {math.degrees(d):+5.1f} "
              f"| median30 {statistics.median(self.diffs):+5.1f}")

rclpy.init(); n = Check()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
