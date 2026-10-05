#!/usr/bin/env python3
"""pair_pass.py <bag_dir>: how the robot passed a standing pair.
Robot pose from /turtlebot4/tf (map->odom composed with odom->base_link); pair from /social_groups."""
import sys, math, collections, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf', '/social_groups', '/social_zone_map', '/turtlebot4/cmd_vel']))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
mo = None; poses = []; groups = []; zone = []; cmd = []
while r.has_next():
    tp, data, t = r.read_next(); t *= 1e-9
    m = deserialize_message(data, get_message(types[tp]))
    if tp == '/turtlebot4/tf':
        for tr in m.transforms:
            T = tr.transform
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                mo = (T.translation.x, T.translation.y, yaw(T.rotation))
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link' and mo:
                c, s = math.cos(mo[2]), math.sin(mo[2])
                poses.append((t, mo[0]+c*T.translation.x-s*T.translation.y, mo[1]+s*T.translation.x+c*T.translation.y))
    elif tp == '/social_groups':
        for g in m.data.split('|||') if '|||' in m.data else [m.data]:
            f = g.split(',')
            if len(f) >= 11:
                try:
                    a, b = f[9].split('|'); ax, ay = map(float, a.split(';')); bx, by = map(float, b.split(';'))
                    groups.append((t, f[0], f[1], ax, ay, bx, by, float(f[10])))
                except Exception: pass
    elif tp == '/social_zone_map':
        zone.append((t, sum(1 for v in m.data if v > 0)))
    else: cmd.append((t, m.linear.x))
t0 = next(t for t, v in cmd if abs(v) > 0.02)
print('groups: %d messages; types %s' % (len(groups), dict(collections.Counter(g[2] for g in groups))))
bufs = collections.Counter(round(g[7], 2) for g in groups)
print('zone buffer values (0.40 = wide, 0.05 = narrow):', dict(bufs))
print('group published from t%+.1f to t%+.1f s' % (groups[0][0]-t0, groups[-1][0]-t0) if groups else 'no group')
print('zone mask: %d messages, non-empty %d, max marked cells %d' % (len(zone), sum(1 for z in zone if z[1] > 0), max([z[1] for z in zone] or [0])))
# optional: --ref-window S  -> pair position = mean over the last S seconds before the robot starts
# (robot standing, people standing), with the two members matched by position, because the
# member order in the message changes with the group type. Without it: mean over the whole bag.
REF = float(sys.argv[sys.argv.index('--ref-window')+1]) if '--ref-window' in sys.argv else None
if groups:
    ref = groups
    if REF is not None:
        ref = [g for g in groups if t0-REF <= g[0] <= t0]
        a0 = (ref[0][3], ref[0][4]); fixed = []
        for g in ref:
            swap = math.hypot(g[5]-a0[0], g[6]-a0[1]) < math.hypot(g[3]-a0[0], g[4]-a0[1])
            fixed.append((g[0], g[1], g[2], g[5], g[6], g[3], g[4], g[7]) if swap else g)
        ref = fixed
        print('pair position from the %d group messages in the %.0f s before the start; types %s' % (len(ref), REF, dict(collections.Counter(g[2] for g in ref))))
        sp = max(max(math.hypot(g[3]-h[3], g[4]-h[4]), math.hypot(g[5]-h[5], g[6]-h[6])) for g in ref for h in ref)
        print('largest change of a reported member position inside that window: %.2f m' % sp)
    ax = sum(g[3] for g in ref)/len(ref); ay = sum(g[4] for g in ref)/len(ref)
    bx = sum(g[5] for g in ref)/len(ref); by = sum(g[6] for g in ref)/len(ref)
    sep = math.hypot(bx-ax, by-ay); ux, uy = (bx-ax)/sep, (by-ay)/sep
    print('pair: A (%.2f, %.2f)  B (%.2f, %.2f)  separation %.2f m' % (ax, ay, bx, by, sep))
    mv = [p for p in poses if p[0] >= t0]
    dA = min(mv, key=lambda p: math.hypot(p[1]-ax, p[2]-ay)); dB = min(mv, key=lambda p: math.hypot(p[1]-bx, p[2]-by))
    print('closest robot centre to A: %.2f m at t+%.1f s; to B: %.2f m at t+%.1f s' % (math.hypot(dA[1]-ax, dA[2]-ay), dA[0]-t0, math.hypot(dB[1]-bx, dB[2]-by), dB[0]-t0))
    # where the path crosses the pair axis: s = position along A->B (0 = A, sep = B)
    prev = None; cross = None
    for p in mv:
        side = -(p[1]-ax)*uy + (p[2]-ay)*ux
        if prev is not None and (side > 0) != (prev[1] > 0):
            cross = ((p[1]-ax)*ux + (p[2]-ay)*uy, p[0]-t0); break
        prev = (p, side)
    if cross:
        s = cross[0]
        where = 'BETWEEN the two' if 0 < s < sep else ('AROUND, outside A by %.2f m' % (-s) if s <= 0 else 'AROUND, outside B by %.2f m' % (s-sep))
        print('path crosses the pair line at t+%.1f s, %.2f m from A along A->B: %s' % (cross[1], s, where))
    else: print('path never crosses the pair line')
