#!/usr/bin/env python3
"""extract_pairs.py - per-run data of the standing-pair runs (wide, narrow) for the results page (9 Oct 2026).

Run in the robot container (domain 77):  python3 extract_pairs.py > pairs_page_data.json
Frames: wide = head-on hallway (origin (-6.88, 13.41), axis to (-12.20, 17.75)); narrow = narrow section (origin (-1.18, 8.82),
axis to (-6.80, 13.50)); s along, l sideways, + = robot's left. Per run: robot path, pair member positions (from /social_groups),
group types and buffers, nearest LiDAR return not on the map (3-cell margin) near the pair (s pair-1.5 .. pair+1.0), the side
passed, time to goal, stops, stop-and-beep events, AMCL heading error at the start
(scan-to-map fit, as align.py). Lanes for pair members are not in the bags (critic log only); the page takes them from
Task/HARDWARE_DAY_20261009.md (the robot container and its Nav2 logs were removed at the end of the day).
"""
import glob, json, math, re, statistics as st
import numpy as np, yaml
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

WS = '/root/thesis_social_navigation_ws'
FRAMES = {'pairwide': (-6.88, 13.41, -12.20, 17.75), 'pairnarrow': (-1.18, 8.82, -6.80, 13.50)}
M = yaml.safe_load(open(f'{WS}/map/hall_map_name.yaml'))
img = np.array(Image.open(f'{WS}/map/' + M['image'])); occ = img < 100
res = M['resolution']; ox0, oy0 = M['origin'][:2]; rows = img.shape[0]
from scipy.ndimage import distance_transform_edt
dist = distance_transform_edt(~occ) * res
def occupied(x, y, m=3):
    cx = int((x - ox0) / res); cy = rows - 1 - int((y - oy0) / res)
    a = occ[max(cy - m, 0):cy + m + 1, max(cx - m, 0):cx + m + 1]
    return a.size > 0 and bool(a.any())
def dmap(X, Y):
    c = ((X - ox0) / res).astype(int); r = (rows - 1 - ((Y - oy0) / res)).astype(int)
    ok = (c >= 0) & (c < img.shape[1]) & (r >= 0) & (r < rows); out = np.full(X.shape, 0.5); out[ok] = np.minimum(dist[r[ok], c[ok]], 0.5)
    return out

def frame(sc):
    SX, SY, GX, GY = FRAMES[sc]; H = math.atan2(GY - SY, GX - SX); C, S = math.cos(H), math.sin(H)
    return (lambda x, y: ((x - SX) * C + (y - SY) * S, -(x - SX) * S + (y - SY) * C)), (SX, SY, H)

def walls(sc):
    SX, SY, GX, GY = FRAMES[sc]; H = math.atan2(GY - SY, GX - SX); C, S = math.cos(H), math.sin(H); out = []
    for sv in np.arange(-0.5, 7.76, 0.25):
        side = {}
        for sign in (1, -1):
            for lv in np.arange(0.0, 2.5, 0.025):
                x = SX + sv * C - sign * lv * S; y = SY + sv * S + sign * lv * C
                if occupied(x, y, 0): side[sign] = round(sign * lv, 3); break
        out.append([round(float(sv), 2), side.get(1), side.get(-1)])
    return out

def run(bag, sc, T0):
    SL, (SX, SY, H) = frame(sc)
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag + '/bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/social_groups', '/turtlebot4/tf', '/turtlebot4/tf_static', '/turtlebot4/odom',
                                                   '/turtlebot4/scan', '/blocked_person_event']))
    groups, path, near, holds, startscans = [], [], [], [], []
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
            groups.append((tt, f[1], [SL(*p) for p in xy], float(f[10])))
        elif t == '/blocked_person_event':
            holds.append((round(tt, 1), deserialize_message(d, String).data.split(',')[0]))
        elif t.endswith('odom') and m2o:
            m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation; ox, oy, oth = m2o
            x = ox + p.x * math.cos(oth) - p.y * math.sin(oth); y = oy + p.x * math.sin(oth) + p.y * math.cos(oth)
            robot = (x, y, oth + 2 * math.atan2(q.z, q.w))
            if -1 <= tt <= 45 and tt - lastp >= 0.2:
                s, l = SL(x, y); path.append([round(tt, 2), round(s, 3), round(l, 3), round(m.twist.twist.linear.x, 3)]); lastp = tt
        elif t.endswith('scan') and robot and laser:
            m = deserialize_message(d, LaserScan); rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
            rx, ry, ryaw = robot
            lx = rx + laser[0] * math.cos(ryaw) - laser[1] * math.sin(ryaw); ly = ry + laser[0] * math.sin(ryaw) + laser[1] * math.cos(ryaw)
            th = ryaw + laser[2]
            if -1.5 <= tt < 0:
                ok = np.isfinite(rg) & (rg > 0.2) & (rg < 8)
                startscans.append((lx, ly, th, rg[ok], a[ok], rx, ry))
            if 0 <= tt <= 45:
                ok = np.isfinite(rg) & (rg > 0.15) & (rg < 2.5)
                for r_, an in sorted(zip(rg[ok], a[ok])):
                    xx = lx + r_ * math.cos(th + an); yy = ly + r_ * math.sin(th + an)
                    if not occupied(xx, yy):
                        s, l = SL(xx, yy); near.append([round(tt, 2), round(float(r_), 3), round(s, 2), round(l, 2)]); break
    # AMCL heading error at the start (best rotation of the start scans about the robot)
    errs = []
    for lx, ly, th, rg, a, rx, ry in startscans:
        X0 = lx + rg * np.cos(th + a) - rx; Y0 = ly + rg * np.sin(th + a) - ry
        best = min((float(np.mean(dmap(rx + np.cos(math.radians(dy)) * X0 - np.sin(math.radians(dy)) * Y0,
                                       ry + np.sin(math.radians(dy)) * X0 + np.cos(math.radians(dy)) * Y0))), dy)
                   for dy in np.arange(-8, 8.01, 0.25))
        errs.append(best[1])
    pair = [g for g in groups if 0 <= g[0] <= 10] or groups
    mem = [[], []]
    for g in pair:
        for i, p in enumerate(sorted(g[2], key=lambda p: p[1])[:2]): mem[i].append(p)
    members = [[round(st.median(c[0] for c in m), 2), round(st.median(c[1] for c in m), 2)] for m in mem if m]
    s_mid = st.mean(m[0] for m in members); l_lo = min(m[1] for m in members); l_hi = max(m[1] for m in members)
    at = min(path, key=lambda p: abs(p[1] - s_mid))
    side = 'left' if at[2] > l_hi else 'right' if at[2] < l_lo else 'between'
    nn = [n for n in near if s_mid - 1.5 <= n[2] <= s_mid + 1.0]
    c = min(nn, key=lambda n: n[1]) if nn else None
    mv = [p for p in path if p[3] > 0.02]
    stops = 0; run_ = 0.0
    for p, q in zip(path, path[1:]):
        if mv and mv[0][0] <= p[0] <= mv[-1][0] and abs(p[3]) < 0.02:
            run_ += q[0] - p[0]
        else:
            if run_ >= 0.5: stops += 1
            run_ = 0.0
    types = {}
    for g in groups:
        if -1 <= g[0] <= 35: types[g[1]] = types.get(g[1], 0) + 1
    buf = {}
    for g in groups:
        if -1 <= g[0] <= 35: k = 'narrow' if g[3] < 0.2 else 'wide'; buf[k] = buf.get(k, 0) + 1
    name = bag.split('/')[-1]
    return {'name': name, 'run': re.search(r'run(\d\d)', name).group(1), 'scenario': sc,
            'loc_err_deg': round(st.median(errs), 2) if errs else None,
            'members': members, 'side': side, 'robot_l_at_pair': round(at[2], 2), 'pair_s': round(s_mid, 2),
            'clearance': c[1] if c else None, 'clearance_t': c[0] if c else None, 'clearance_sl': [c[2], c[3]] if c else None,
            'time_to_goal': round(mv[-1][0] - mv[0][0], 1) if mv else None, 'stops': stops,
            'max_lateral': round(max(abs(p[2]) for p in path if 0 <= p[0]), 2), 'holds': holds,
            'group_types': types, 'buffers': buf,
            'path': [[p[0], p[1], p[2]] for p in path[::2] if p[0] <= 40]}

out = {'frames': {k: list(v) for k, v in FRAMES.items()}, 'walls': {k: walls(k) for k in FRAMES}, 'runs': []}
for sc in ('pairwide', 'pairnarrow'):
    for bag in sorted(glob.glob(f'{WS}/bags/{sc}_rec20noghost_run*')):
        T0 = float(re.search(r't\+0 = ([0-9.]+)', open(bag + '/headon_metrics.txt').read()).group(1))
        out['runs'].append(run(bag, sc, T0))
print(json.dumps(out, separators=(',', ':')))
