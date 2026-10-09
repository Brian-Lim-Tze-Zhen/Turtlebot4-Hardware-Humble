#!/usr/bin/env python3
"""turn_in_place.py - slow turn on the spot to a map heading, without a Nav2 goal (9 Oct 2026).

Why: the last step of the return to the start was a Nav2 goal at the same position with a new heading. A turn on the
spot keeps the robot's position still, so blocked_person_beep_node_front.py (goal active + no headway + something within
1 m ahead) took Brian sitting at his table for a blocking person, beeped and held the robot. Without a Nav2 goal that
node stays out of it.

What: if something is within --clear m and +/-30 deg ahead (raw LiDAR), beep 3 times as a warning first. Then turn on
the spot at up to --speed rad/s (slower over the last 15 deg) until the map heading (TF map -> base_link, i.e. AMCL) is
within --tol deg of the target. Commands go to /turtlebot4/cmd_vel_nav, so the velocity smoother and the collision
monitor still apply. vx is always 0. Stops after --timeout s, and on Ctrl+C.

usage (in the container):
  python3 scripts/turn_in_place.py 141 --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
exit: 0 heading reached, 1 timeout, 2 no TF
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration as MsgDuration
import tf2_ros

NS = '/turtlebot4'
LIDAR_MOUNT_DEG = 90.0   # base_link -> rplidar_link yaw (camera_ray_person_node logs +90.00 deg)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class Turn(Node):
    def __init__(self):
        super().__init__('turn_in_place')
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Twist, f'{NS}/cmd_vel_nav', 10)
        self.audio = self.create_publisher(AudioNoteVector, f'{NS}/cmd_audio', 10)
        self.front = None
        self.create_subscription(LaserScan, f'{NS}/scan', self.scan_cb, qos_profile_sensor_data)

    def scan_cb(self, m):
        best = math.inf
        for i, r in enumerate(m.ranges):
            a = wrap(math.degrees(m.angle_min + i * m.angle_increment) + LIDAR_MOUNT_DEG)
            if abs(a) <= 30.0 and m.range_min < r < best:
                best = r
        self.front = best

    def heading(self):
        try:
            tr = self.buf.lookup_transform('map', 'base_link', Time(), Duration(seconds=0.2))
        except Exception:
            return None
        q = tr.transform.rotation
        return math.degrees(2.0 * math.atan2(q.z, q.w))

    def beep(self):
        v = AudioNoteVector()
        v.append = False
        n = AudioNote()
        n.frequency = 880
        n.max_runtime = MsgDuration(sec=0, nanosec=300000000)
        v.notes.append(n)
        self.audio.publish(v)

    def cmd(self, wz):
        t = Twist()
        t.angular.z = float(wz)
        self.pub.publish(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('heading', type=float, help='target map heading, deg')
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--tol', type=float, default=2.0)
    ap.add_argument('--clear', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=40.0)
    a, _ = ap.parse_known_args()
    rclpy.init(args=sys.argv)
    n = Turn()
    t0 = time.time()
    h = None
    while time.time() - t0 < 8.0 and (h is None or n.front is None):
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - t0 > 1.0:   # let the TF buffer fill first
            h = n.heading()
    if h is None:
        print('NO TF map -> base_link (AMCL running? /tf remapped?)')
        sys.exit(2)
    if n.front is not None and n.front < a.clear:
        print(f'something {n.front:.2f} m ahead - 3 warning beeps, then turning slowly')
        for _ in range(3):
            n.beep()
            t1 = time.time()
            while time.time() - t1 < 1.0:
                rclpy.spin_once(n, timeout_sec=0.05)
    print(f'turning from {h:.1f} to {a.heading:.1f} deg at up to {a.speed:.2f} rad/s')
    rc = 1
    t0 = time.time()
    try:
        while time.time() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.05)
            h = n.heading()
            if h is None:
                n.cmd(0.0)
                continue
            err = wrap(a.heading - h)
            if abs(err) <= a.tol:
                rc = 0
                break
            w = a.speed if abs(err) > 15.0 else max(0.12, a.speed * abs(err) / 15.0)
            n.cmd(math.copysign(w, err))
    finally:
        for _ in range(5):
            n.cmd(0.0)
            time.sleep(0.05)
    h = n.heading()
    print(('REACHED' if rc == 0 else 'TIMEOUT') + f' heading {h:.1f} deg (target {a.heading:.1f})')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
