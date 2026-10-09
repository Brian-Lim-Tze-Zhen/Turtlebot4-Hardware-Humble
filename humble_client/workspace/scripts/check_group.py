#!/usr/bin/env python3
"""check_group.py - check through the robot's speaker that two standing people are detected as a group (9 Oct 2026).

For the standing-pair runs: the people forming the pair cannot watch the laptop. Waits up to --wait s for
/social_groups (social_group_detector_node_lidarhold.py) and needs the group for --hold s in a row.
  "conversation" (facing) for --hold s  -> two short high beeps, prints OK
  only "side_by_side"                   -> one middle tone, prints SIDE BY SIDE (detected, not as a conversation)
  no group within --wait s              -> three short low beeps, prints NO GROUP
Also prints the members' positions, their distance, the zone buffer (0.40 = wide, 0.05 = narrow) and where the pair stands
in the hallway frame (s from the start towards the goal, l sideways, + = robot's left).
Message: group_id,group_type,cx,cy,axis_x,axis_y,half_length,half_width,member_ids,member_xy,buffer

usage (in the container): python3 scripts/check_group.py [--wait 120] [--hold 3]
exit: 0 conversation, 1 side_by_side only, 2 no group
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration

SX, SY, GX, GY = -6.88, 13.41, -12.20, 17.75   # start and goal of the hallway runs (since 14:25)
H = math.atan2(GY - SY, GX - SX)


def sl(x, y):
    dx, dy = x - SX, y - SY
    return dx * math.cos(H) + dy * math.sin(H), -dx * math.sin(H) + dy * math.cos(H)


class Check(Node):
    def __init__(self):
        super().__init__('check_group')
        self.msgs = []   # (t, type, members [(x, y)], buffer)
        self.create_subscription(String, '/social_groups', self.cb, 10)
        self.audio = self.create_publisher(AudioNoteVector, '/turtlebot4/cmd_audio', 10)

    def cb(self, m):
        f = m.data.split(',')
        if len(f) < 11:
            return
        try:
            xy = [tuple(float(v) for v in p.split(';')) for p in f[9].split('|')]
            self.msgs.append((time.time(), f[1], xy, float(f[10])))
        except ValueError:
            pass

    def play(self, notes):
        v = AudioNoteVector()
        v.append = False
        for hz, s in notes:
            n = AudioNote()
            n.frequency = int(hz)
            n.max_runtime = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            v.notes.append(n)
        for _ in range(3):
            self.audio.publish(v)
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wait', type=float, default=120.0)
    ap.add_argument('--hold', type=float, default=3.0)
    a, _ = ap.parse_known_args()
    rclpy.init(args=sys.argv)
    n = Check()
    t0 = time.time()
    print(f'waiting up to {a.wait:.0f} s for a group that lasts {a.hold:.0f} s ...', flush=True)
    result = None
    while time.time() - t0 < a.wait:
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.time()
        recent = [m for m in n.msgs if now - m[0] <= a.hold + 0.5]
        conv = [m for m in recent if m[1] == 'conversation']
        if conv and now - conv[0][0] >= a.hold and len(conv) >= 2:
            result = ('conversation', conv[-1]); break
        sbs = [m for m in recent if m[1] == 'side_by_side']
        if sbs and now - sbs[0][0] >= a.hold and len(sbs) >= 2 and not conv:
            result = ('side_by_side', sbs[-1])
    if result is None:
        print('NO GROUP: /social_groups had no group lasting %.0f s (both standing still in camera view, ~1 m apart?)' % a.hold)
        n.play([(330, 0.2), (440, 0.1), (330, 0.2), (440, 0.1), (330, 0.2)])
        sys.exit(2)
    kind, (_, _, xy, buf) = result
    d = math.dist(xy[0], xy[1]) if len(xy) >= 2 else float('nan')
    pos = ', '.join(f'({x:.2f}, {y:.2f}) = s {sl(x, y)[0]:.1f} m, l {sl(x, y)[1]:+.2f} m' for x, y in xy)
    print(f'members: {pos}')
    print(f'distance between them {d:.2f} m, zone buffer {buf:.2f} ({"wide" if buf > 0.2 else "narrow"})')
    if kind == 'conversation':
        print('OK: conversation group')
        n.play([(1320, 0.15), (500, 0.1), (1320, 0.15)])
        sys.exit(0)
    print('SIDE BY SIDE: a group, but not detected as a conversation (face each other more)')
    n.play([(660, 0.8)])
    sys.exit(1)


if __name__ == '__main__':
    main()
