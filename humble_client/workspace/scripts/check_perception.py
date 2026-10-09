#!/usr/bin/env python3
"""check_perception.py - one-person check of the person pipeline before a run (9 Oct 2026).

Why: run headon_rec20noghost_run01_20261009_114928 started with a broken TF tree after a Create 3 restart; YOLO saw the
walker but camera_ray_person_node could not place him on the map, so the critic never saw a person. The walker cannot
watch the laptop, so this check answers through the robot's speaker.

What: waits up to --wait s for a YOLO detection (/person_positions_map), then for --window s counts messages with a person
on /camera_ray_clusters and /predicted_person_positions.
  all three stages present -> two short high beeps, prints OK
  a stage missing          -> one long low tone, prints which stage
  nobody seen in --wait s  -> three short low beeps, prints NO PERSON
Stand 2-5 m in front of the robot, inside the camera view.

usage (in the container): python3 scripts/check_perception.py [--wait 120] [--window 3]
exit: 0 OK, 1 stage missing, 2 no person
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration

AUDIO = '/turtlebot4/cmd_audio'
STAGES = [('YOLO', '/person_positions_map'), ('camera ray', '/camera_ray_clusters'),
          ('tracker', '/predicted_person_positions')]


def has_person(msg):
    return any(len(e.split(',')) > 3 for e in msg.data.split(';') if e.strip())


class Check(Node):
    def __init__(self):
        super().__init__('check_perception')
        self.count = {n: 0 for n, _ in STAGES}
        self.first_yolo = None
        for name, topic in STAGES:
            self.create_subscription(String, topic, lambda m, n=name: self.cb(n, m), 10)
        self.audio = self.create_publisher(AudioNoteVector, AUDIO, 10)

    def cb(self, name, msg):
        if not has_person(msg):
            return
        if name == 'YOLO' and self.first_yolo is None:
            self.first_yolo = time.time()
        if self.first_yolo is not None:
            self.count[name] += 1

    def play(self, notes):
        v = AudioNoteVector()
        v.append = False
        for hz, s in notes:
            n = AudioNote()
            n.frequency = int(hz)
            n.max_runtime = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            v.notes.append(n)
        for _ in range(3):   # the robot link drops single messages now and then
            self.audio.publish(v)
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wait', type=float, default=120.0)
    ap.add_argument('--window', type=float, default=3.0)
    a, _ = ap.parse_known_args()
    rclpy.init(args=sys.argv)
    n = Check()
    t0 = time.time()
    print(f'waiting up to {a.wait:.0f} s for a person in front of the robot ...', flush=True)
    while n.first_yolo is None and time.time() - t0 < a.wait:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.first_yolo is None:
        print('NO PERSON: YOLO saw nobody (camera running? standing in view?)')
        n.play([(330, 0.2), (440, 0.1), (330, 0.2), (440, 0.1), (330, 0.2)])
        sys.exit(2)
    while time.time() - n.first_yolo < a.window:
        rclpy.spin_once(n, timeout_sec=0.05)
    print('messages with a person in the first %.0f s: ' % a.window +
          ', '.join(f'{k} {v}' for k, v in n.count.items()))
    missing = [k for k, v in n.count.items() if v == 0]
    if missing:
        print('FAIL: no person at ' + ', '.join(missing) +
              ' (camera ray empty = check the TF tree map->rplidar_link and odom; tracker empty = predictor node)')
        n.play([(220, 1.5)])
        sys.exit(1)
    print('OK: YOLO -> camera ray -> tracker all see the person')
    n.play([(1320, 0.15), (500, 0.1), (1320, 0.15)])
    sys.exit(0)


if __name__ == '__main__':
    main()
