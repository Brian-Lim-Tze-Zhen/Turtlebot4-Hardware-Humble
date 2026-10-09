#!/usr/bin/env python3
"""Recompute the critic's pass-side decision from bags for several window / threshold / hold settings.

Same formula as social_critic_lane_noghost.cpp: anchor = robot position when the lane starts, direction = robot -> goal
(goal_pose; side_ref_goal_topic), offset = sideways position of the walker's KF estimate (/predicted_person_positions,
fields 2,3) from that line, positive = robot's left, sampled at the controller's 20 Hz. The 6 cm commit is ignored (it
depends on how the robot moves, which the bag cannot replay for a different side).
Loc correction: the walker is turned about the robot by the AMCL heading error found by align.py at the start.
"""
import sys, math, json, bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

runs = json.load(open(sys.argv[1]))
out = []
for R in runs:
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=R['bag'], storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf', '/turtlebot4/odom', '/predicted_person_positions',
                                                   '/turtlebot4/goal_pose', '/goal_pose']))
    m2o = None; poses = []; walker = []; goal = None
    t_lane = R['lane_abs']
    while rd.has_next():
        t, d, ts = rd.read_next(); tt = ts * 1e-9
        if tt > t_lane + 4.0:
            break
        if t.endswith('/tf'):
            for tr in deserialize_message(d, TFMessage).transforms:
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    q = tr.transform.rotation
                    m2o = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
        elif t.endswith('odom') and m2o:
            m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
            ox, oy, oth = m2o
            poses.append((tt, ox + p.x * math.cos(oth) - p.y * math.sin(oth), oy + p.x * math.sin(oth) + p.y * math.cos(oth),
                          oth + 2 * math.atan2(q.z, q.w)))
        elif 'goal_pose' in t:
            g = deserialize_message(d, PoseStamped).pose.position; goal = (g.x, g.y)
        elif t == '/predicted_person_positions' and tt >= t_lane - 0.6:
            for e in deserialize_message(d, String).data.split(';'):
                f = e.split(',')
                if len(f) > 3 and int(f[0]) == R['track']:
                    walker.append((tt, float(f[2]), float(f[3])))
    if goal is None:
        goal = (-12.20, 17.75)
    pt = [p[0] for p in poses]
    i = max(0, bisect.bisect_right(pt, t_lane) - 1)
    _, rx, ry, ryaw = poses[i]
    gx, gy = goal[0] - rx, goal[1] - ry; n = math.hypot(gx, gy); ux, uy = gx / n, gy / n
    c = math.radians(R.get('corr', 0.0))
    wt = [w[0] for w in walker]

    def sample(tt, corr):
        j = bisect.bisect_right(wt, tt) - 1
        if j < 0 or tt - wt[j] > 0.5:
            return None
        wx, wy = walker[j][1] - rx, walker[j][2] - ry
        if corr:
            wx, wy = wx * math.cos(c) - wy * math.sin(c), wx * math.sin(c) + wy * math.cos(c)
        return -uy * wx + ux * wy, math.hypot(wx, wy)

    res = {'name': R['name'], 'actual': R['actual'], 'corr': R.get('corr', 0.0), 'note': R.get('note', '')}
    for corr in (False, True):
        S = []
        k = 0
        while k * 0.05 <= 3.0:
            s = sample(t_lane + k * 0.05, corr)
            if s:
                S.append((k * 0.05, s[0], s[1]))
            k += 1
        key = 'loc' if corr else 'raw'
        res[key] = S
    out.append(res)


def decide(S, W, th, hold):
    first = None; side = None; sm = 0; n = 0
    for (dt, off, dist) in S:
        if dt > W + 1e-9:
            break
        sm += off; n += 1; mean = sm / n
        side = -1 if mean < -th else 1
        if n == 1:
            first = side
        elif hold > 0 and side != first:
            clear = mean < -hold if side < 0 else mean > hold
            if not clear:
                side = first
    dist = [d for (dt, o, d) in S if dt <= W][-1] if S else float('nan')
    return side, (sm / n if n else float('nan')), dist


print('Per run (offset + = robot left; reference = mean over 3 s after lane start, loc-corrected):')
print(f"{'run':34s} {'actual':6s} corr  first(raw/loc)  mean0.6(raw/loc)  mean1.5 loc  ref3s loc  dist@0/0.6/1.5")
for r in out:
    L = r['loc']; Rw = r['raw']
    if not L:
        print(r['name'], 'no samples'); continue
    m = lambda S, W: sum(o for dt, o, d in S if dt <= W) / max(1, len([1 for dt, o, d in S if dt <= W]))
    dd = lambda W: [d for dt, o, d in L if dt <= W][-1]
    print(f"{r['name']:34s} {r['actual']:6s} {r['corr']:+4.1f}  {Rw[0][1]:+.2f}/{L[0][1]:+.2f}     {m(Rw,0.6):+.2f}/{m(L,0.6):+.2f}       {m(L,1.5):+.2f}      {m(L,3.0):+.2f}     {dd(0):.1f}/{dd(0.6):.1f}/{dd(1.5):.1f}  {r['note']}")

print()
print('Decision per setting (R = keep right / pass on walker\'s left, L = keep left), loc-corrected inputs; raw in [ ]:')
names = [r['name'].split('_')[1] + '_' + r['name'].split('_')[2][-2:] for r in out]
print(f"{'W s  th   hold':16s} " + ' '.join(f'{n[:11]:>11s}' for n in names))
for W in (0.6, 1.0, 1.5):
    for th in (0.15, 0.20, 0.25, 0.30):
        for hold in (0.3, 0.0):
            cells = []
            for r in out:
                if not r['loc']:
                    cells.append('-'); continue
                s1, _, _ = decide(r['loc'], W, th, hold); s0, _, _ = decide(r['raw'], W, th, hold)
                f = lambda s: 'R' if s > 0 else 'L'
                cells.append(f'{f(s1)}[{f(s0)}]')
            print(f'{W:.1f} {th:.2f} {hold:.1f}    ' + ' '.join(f'{c:>11s}' for c in cells))
