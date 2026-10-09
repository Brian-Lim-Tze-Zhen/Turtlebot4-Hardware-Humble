#!/usr/bin/env python3
"""pair_run.py - standing-pair run analysis (9 Oct 2026). Run in the robot container (domain 77).
usage: python3 pair_run.py <bag folder> <t0> [controller log]
Prints: group messages over time (type, members, gap, buffer), the robot's path past the pair in the hallway
frame (origin (-6.88, 13.41), axis to the goal (-12.20, 17.75), + = robot's left), the side it passed on, the
nearest LiDAR return not on the map, and critic lane/side lines for the run.
"""
import sys, math, collections
import numpy as np, yaml
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

import os
# hallway frame; FRAME="sx,sy,gx,gy" overrides (narrow section: FRAME=-1.18,8.82,-6.80,13.50)
SX, SY, GX, GY = [float(v) for v in os.environ.get('FRAME', '-6.88,13.41,-12.20,17.75').split(',')]
H = math.atan2(GY - SY, GX - SX); C, S = math.cos(H), math.sin(H)
def SL(x, y):
    dx, dy = x - SX, y - SY
    return dx * C + dy * S, -dx * S + dy * C
M = yaml.safe_load(open('/root/thesis_social_navigation_ws/map/hall_map_name.yaml'))
img = np.array(Image.open('/root/thesis_social_navigation_ws/map/' + M['image'])); occ = img < 100
res = M['resolution']; ox0, oy0 = M['origin'][:2]; rows = img.shape[0]
def occupied(x, y, m=3):
    cx = int((x - ox0) / res); cy = rows - 1 - int((y - oy0) / res)
    a = occ[max(cy - m, 0):cy + m + 1, max(cx - m, 0):cx + m + 1]
    return a.size > 0 and bool(a.any())

bag, T0 = sys.argv[1], float(sys.argv[2])
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=bag + '/bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/social_groups', '/social_zone_map', '/turtlebot4/tf', '/turtlebot4/tf_static',
                                               '/turtlebot4/odom', '/turtlebot4/scan']))
groups, zone, path, near = [], [], [], []
m2o = laser = robot = None; lastp = -9
while rd.has_next():
    t, d, ts = rd.read_next(); tt = ts * 1e-9 - T0
    if t.endswith('tf') or t.endswith('tf_static'):
        for tr in deserialize_message(d, TFMessage).transforms:
            q = tr.transform.rotation
            if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                m2o = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
            if tr.child_frame_id.endswith('rplidar_link'):
                laser = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
    elif t == '/social_groups':
        f = deserialize_message(d, String).data.split(',')
        xy = [tuple(float(v) for v in p.split(';')) for p in f[9].split('|')]
        groups.append((tt, f[1], xy, float(f[10])))
    elif t == '/social_zone_map':
        g = deserialize_message(d, OccupancyGrid)
        zone.append((tt, sum(1 for v in g.data if v > 0)))
    elif t.endswith('odom') and m2o:
        m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation; ox, oy, oth = m2o
        x = ox + p.x * math.cos(oth) - p.y * math.sin(oth); y = oy + p.x * math.sin(oth) + p.y * math.cos(oth)
        robot = (x, y, oth + 2 * math.atan2(q.z, q.w))
        if tt - lastp >= 0.5 and -1 <= tt <= 30:
            s, l = SL(x, y); path.append((round(tt, 1), round(s, 2), round(l, 2), round(m.twist.twist.linear.x, 2))); lastp = tt
    elif t.endswith('scan') and robot and laser and 0 <= tt <= 30:
        m = deserialize_message(d, LaserScan); rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > 0.15) & (rg < 2.5); rx, ry, ryaw = robot
        lx = rx + laser[0] * math.cos(ryaw) - laser[1] * math.sin(ryaw); ly = ry + laser[0] * math.sin(ryaw) + laser[1] * math.cos(ryaw)
        for r, an in sorted(zip(rg[ok], a[ok])):
            x = lx + r * math.cos(ryaw + laser[2] + an); y = ly + r * math.sin(ryaw + laser[2] + an)
            if not occupied(x, y):
                s, l = SL(x, y); near.append((round(tt, 1), round(float(r), 2), round(s, 2), round(l, 2))); break

print(f'group messages: {len(groups)}, from t{groups[0][0]:+.1f} to t{groups[-1][0]:+.1f} s' if groups else 'NO group messages')
print('types:', dict(collections.Counter(g[1] for g in groups)), ' buffers:', dict(collections.Counter(round(g[3], 2) for g in groups)))
gaps = [b[0] - a[0] for a, b in zip(groups, groups[1:])]
if gaps: print(f'largest gap between group messages: {max(gaps):.1f} s at t{groups[gaps.index(max(gaps))][0]:+.1f}')
for g in groups[::max(1, len(groups) // 8)]:
    ms = ' | '.join(f's {SL(*p)[0]:.2f} l {SL(*p)[1]:+.2f}' for p in g[2])
    print(f'  t{g[0]:+6.1f} {g[1]:12s} members {ms}  gap {math.dist(*g[2][:2]):.2f} m  buffer {g[3]:.2f}')
if zone: print(f'zone mask: {len(zone)} messages, non-empty {sum(1 for z in zone if z[1] > 0)}, max cells {max(z[1] for z in zone)}')
pair = [g for g in groups if 0 <= g[0] <= 10] or groups
if pair:
    ps = [SL(*p) for g in pair for p in g[2]]
    s_mid = np.mean([p[0] for p in ps]); l_lo = min(p[1] for p in ps); l_hi = max(p[1] for p in ps)
    print(f'pair (t 0-10 s): s {s_mid:.2f} m from the start, members between l {l_lo:+.2f} and {l_hi:+.2f}')
    at = min(path, key=lambda p: abs(p[1] - s_mid))
    side = 'LEFT of the pair (+l)' if at[2] > l_hi else 'RIGHT of the pair (-l)' if at[2] < l_lo else 'BETWEEN the members'
    print(f'robot at the pair (t{at[0]:+.1f} s): l {at[2]:+.2f} -> passed {side}')
nn = [n for n in near if n[2] > -0.2]
if nn:
    c = min(nn, key=lambda n: n[1]); print(f'nearest LiDAR return not on the map: {c[1]:.2f} m at t{c[0]:+.1f} s (s {c[2]:.2f}, l {c[3]:+.2f})')
if pair and nn:
    np_ = [n for n in nn if s_mid - 1.5 <= n[2] <= s_mid + 1.0]
    if np_:
        c = min(np_, key=lambda n: n[1]); print(f'nearest return not on the map NEAR THE PAIR (s {s_mid - 1.5:.1f}..{s_mid + 1.0:.1f}): {c[1]:.2f} m at t{c[0]:+.1f} s (s {c[2]:.2f}, l {c[3]:+.2f})')
print('path (t, s, l, vx):', path[:: 2][:26])
if len(sys.argv) > 3:
    for line in open(sys.argv[3]):
        if 'SocialLaneCritic' in line and any(k in line for k in ('lane rule', 'pass side', 'occlusion', 'ghost')):
            try:
                tw = float(line.split('[')[2].split(']')[0])
            except Exception:
                continue
            if -15 < tw - T0 < 30: print(f'  critic t{tw - T0:+.1f}: {line.split("SocialLaneCritic: ")[-1].strip()[:120]}')
