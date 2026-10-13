#!/usr/bin/env python3
"""compare_days_centre.py - copy of compare_days.py (10 Oct 2026) that measures the nearest LiDAR return not on the map
from the ROBOT CENTRE (base_link) instead of from the LiDAR, which is mounted 0.04 m behind the centre.
Only the scan block differs: every return is converted to the map, and the smallest centre distance among the
unmapped returns is kept. Original: compare_days.py - the same measures for robot head-on and pair runs of different days (9 Oct 2026).

usage (robot image, offline): python3 compare_days.py <label>=<bag folder>[,<bag folder>...] ...
Per run, from motion start (first odom vx > 0.02 m/s) to motion end:
  time to goal (motion duration), stops >= 0.5 s, max sideways offset from the start-goal line and max heading off it,
  side passed (sign of the extreme sideways offset; head-on) or relative to the pair (pair runs),
  nearest LiDAR return within 2.5 m that is not on the map (3-cell margin), ahead of the start (head-on) or within 1.5 m
  before / 1.0 m after the pair (pair runs, pair from /social_groups).
Frame per run: origin = robot position at motion start, axis = towards the robot position at motion end.
"""
import sys, math, statistics as st
import numpy as np, yaml
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

M = yaml.safe_load(open('/root/thesis_social_navigation_ws/map/hall_map_name.yaml'))
img = np.array(Image.open('/root/thesis_social_navigation_ws/map/' + M['image'])); occ = img < 100
res = M['resolution']; ox0, oy0 = M['origin'][:2]; rows = img.shape[0]
def occupied(x, y, m=3):
    cx = int((x - ox0) / res); cy = rows - 1 - int((y - oy0) / res)
    a = occ[max(cy - m, 0):cy + m + 1, max(cx - m, 0):cx + m + 1]
    return a.size > 0 and bool(a.any())

def run(bag):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag + '/bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
    topics = {t.name for t in rd.get_all_topics_and_types()}
    want = [t for t in ('/turtlebot4/tf', '/turtlebot4/tf_static', '/turtlebot4/odom', '/turtlebot4/scan', '/social_groups') if t in topics]
    rd.set_filter(rosbag2_py.StorageFilter(topics=want))
    m2o = laser = robot = None; path = []; scans = []; members = []
    while rd.has_next():
        t, d, ts = rd.read_next(); tt = ts * 1e-9
        if t.endswith('tf') or t.endswith('tf_static'):
            for tr in deserialize_message(d, TFMessage).transforms:
                q = tr.transform.rotation
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    m2o = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
                if tr.child_frame_id.endswith('rplidar_link'):
                    laser = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
        elif t.endswith('odom') and m2o:
            m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation; ox, oy, oth = m2o
            robot = (ox + p.x * math.cos(oth) - p.y * math.sin(oth), oy + p.x * math.sin(oth) + p.y * math.cos(oth), oth + 2 * math.atan2(q.z, q.w))
            path.append((tt, robot[0], robot[1], robot[2], m.twist.twist.linear.x))
        elif t == '/social_groups':
            f = deserialize_message(d, String).data.split(',')
            members.append((tt, [tuple(float(v) for v in p.split(';')) for p in f[9].split('|')]))
        elif t.endswith('scan') and robot and laser:
            m = deserialize_message(d, LaserScan); rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
            ok = np.isfinite(rg) & (rg > 0.15) & (rg < 2.5); rx, ry, ryaw = robot
            lx = rx + laser[0] * math.cos(ryaw) - laser[1] * math.sin(ryaw); ly = ry + laser[0] * math.sin(ryaw) + laser[1] * math.cos(ryaw)
            best = None
            for r_, an in zip(rg[ok], a[ok]):
                x = lx + r_ * math.cos(ryaw + laser[2] + an); y = ly + r_ * math.sin(ryaw + laser[2] + an)
                dc = math.hypot(x - rx, y - ry)
                if (best is None or dc < best[0]) and not occupied(x, y):
                    best = (dc, x, y)
            if best:
                scans.append((tt, float(best[0]), best[1], best[2]))
    mv = [p for p in path if p[4] > 0.02]
    t0, t1 = mv[0][0], mv[-1][0]
    sx, sy = mv[0][1], mv[0][2]; gx, gy = mv[-1][1], mv[-1][2]; H = math.atan2(gy - sy, gx - sx)
    SL = lambda x, y: ((x - sx) * math.cos(H) + (y - sy) * math.sin(H), -(x - sx) * math.sin(H) + (y - sy) * math.cos(H))
    win = [p for p in path if t0 <= p[0] <= t1]
    lat = [SL(p[1], p[2])[1] for p in win]
    ext = max(lat, key=abs)
    head = max(abs((math.degrees(p[3] - H) + 180) % 360 - 180) for p in win)
    stops = 0; run_ = 0.0
    for p, q in zip(win, win[1:]):
        if abs(p[4]) < 0.02: run_ += q[0] - p[0]
        else:
            if run_ >= 0.5: stops += 1
            run_ = 0.0
    pair = [mm for (tt, ms) in members if t0 - 5 <= tt <= t0 + 10 for mm in ms]
    if pair:
        ps = [SL(*p) for p in pair]; s_mid = st.median(p[0] for p in ps)
        l_lo = min(p[1] for p in ps); l_hi = max(p[1] for p in ps)
        cand = [(r, *SL(x, y)) for (tt, r, x, y) in scans if t0 <= tt <= t1 and s_mid - 1.5 <= SL(x, y)[0] <= s_mid + 1.0]
        at = min(win, key=lambda p: abs(SL(p[1], p[2])[0] - s_mid)); la = SL(at[1], at[2])[1]
        side = 'left of pair' if la > l_hi else 'right of pair' if la < l_lo else 'between'
    else:
        cand = [(r, *SL(x, y)) for (tt, r, x, y) in scans if t0 + 1 <= tt <= min(t1, t0 + 15) and SL(x, y)[0] > -0.2]
        side = 'left' if ext > 0 else 'right'
    c = min(cand)[0] if cand else float('nan')
    return {'clear': c, 'side': side, 'lat': abs(ext), 'head': head, 'stops': stops, 'time': t1 - t0}

for arg in sys.argv[1:]:
    label, bags = arg.split('=', 1)
    R = []
    for b in bags.split(','):
        try:
            r = run(b); R.append(r)
            print(f'  {label:28s} {b.split("/")[-1][:44]:44s} clear {r["clear"]:.2f} m  {r["side"]:13s} lat {r["lat"]:.2f}  head {r["head"]:3.0f}  stops {r["stops"]}  {r["time"]:.1f} s')
        except Exception as e:
            print(f'  {label:28s} {b.split("/")[-1][:44]:44s} FAILED {e}')
    if R:
        f = lambda k: f'{st.mean(r[k] for r in R):.2f} +/- {st.stdev(r[k] for r in R) if len(R) > 1 else 0:.2f}'
        sides = {}
        for r in R: sides[r['side']] = sides.get(r['side'], 0) + 1
        print(f'SET {label:24s} n {len(R):2d} | clear {f("clear")} m | sides {sides} | lat {f("lat")} m | head {st.mean(r["head"] for r in R):.0f} deg | stops {sum(r["stops"] for r in R)} | time {f("time")} s')
