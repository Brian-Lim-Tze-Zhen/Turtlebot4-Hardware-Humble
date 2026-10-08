#!/usr/bin/env python3
"""nav_goal_wait.py - send one Nav2 goal on /turtlebot4/goal_pose and wait for its result.

8 Oct 2026, for run_headon_hw.sh. The goal goes out on the goal_pose topic (as on
5 Oct, so beep_retry_node and the BT navigator see it the usual way); the result is
read from the navigate_to_pose action status, not from a launch log.

A retry sent by beep_retry_node after an abort is a new goal id; the script keeps
waiting while such a retry is active and ends on the first success, or when a goal
ended without success and nothing new started within --settle seconds.

usage: python3 nav_goal_wait.py X Y QZ QW [--timeout 120] [--settle 8]
prints: RESULT <SUCCEEDED|ABORTED|CANCELED|TIMEOUT> after <s> s, goals <n>
exit:   0 succeeded, 1 otherwise
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus

NS = '/turtlebot4'
TERMINAL = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('qz', type=float)
    ap.add_argument('qw', type=float)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--settle', type=float, default=8.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node('nav_goal_wait')
    status = {}        # goal id -> latest status
    first_seen = {}    # goal id -> time first seen

    def on_status(msg):
        now = time.time()
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            status[gid] = s.status
            first_seen.setdefault(gid, now)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(GoalStatusArray, NS + '/navigate_to_pose/_action/status', on_status, qos)
    pub = node.create_publisher(PoseStamped, NS + '/goal_pose', 10)

    # goals that existed before ours are ignored
    t_end = time.time() + 2.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    old = set(status)
    t_end = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('RESULT NO_SUBSCRIBER on goal_pose (is Nav2 running?)', flush=True)
        return 1

    g = PoseStamped()
    g.header.frame_id = 'map'
    g.header.stamp = node.get_clock().now().to_msg()
    g.pose.position.x, g.pose.position.y = a.x, a.y
    g.pose.orientation.z, g.pose.orientation.w = a.qz, a.qw
    pub.publish(g)
    t0 = time.time()
    print(f'GOAL sent ({a.x:.2f}, {a.y:.2f}) {time.strftime("%H:%M:%S")}', flush=True)

    result, last_end = 'TIMEOUT', None
    while time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        new = {gid: st for gid, st in status.items() if gid not in old}
        if not new:
            continue
        if any(st == GoalStatus.STATUS_SUCCEEDED for st in new.values()):
            result = 'SUCCEEDED'
            break
        active = [gid for gid, st in new.items() if st not in TERMINAL]
        if active:
            last_end = None
            continue
        if last_end is None:
            last_end = time.time()
            result = TERMINAL[new[max(new, key=lambda k: first_seen[k])]]
            print(f'goal ended {result} at {last_end - t0:.1f} s, waiting {a.settle:.0f} s for a retry',
                  flush=True)
        elif time.time() - last_end > a.settle:
            break
    else:
        if last_end is None:
            result = 'TIMEOUT'
    n_goals = len([gid for gid in status if gid not in old])
    print(f'RESULT {result} after {time.time() - t0:.1f} s, goals {n_goals}', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result == 'SUCCEEDED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
