#!/usr/bin/env python3
"""hold_test_hwday.py <bag_dir>: stationary-hold test. Per second: source of each identity in
/person_positions_fused (camera_confirmed, lidar_hold, ...), its position, robot yaw and the tones
sent on /turtlebot4/cmd_audio. Then the lidar_hold episodes with duration and anchor drift."""
import sys, math, collections, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/turtlebot4/tf', '/person_positions_fused', '/turtlebot4/cmd_audio']
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
mo = None; yaws = []; ppl = []; tones = []; first = None
while r.has_next():
    tp, data, t = r.read_next(); t *= 1e-9
    if first is None: first = t
    m = deserialize_message(data, get_message(types[tp]))
    if tp == '/turtlebot4/tf':
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom': mo = yaw(tr.transform.rotation)
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link' and mo is not None:
                yaws.append((t, math.degrees(mo + yaw(tr.transform.rotation))))
    elif tp == '/turtlebot4/cmd_audio':
        tones.append((t, [n.frequency for n in m.notes]))
    else:
        for rec in m.data.split('|'):
            f = rec.split(',')
            if len(f) >= 12:
                try: ppl.append((t, f[0], float(f[2]), float(f[3]), f[11]))
                except ValueError: pass
print('bag start = t 0; tones: ' + ', '.join('t+%.1f s %s Hz' % (t-first, fr) for t, fr in tones))
print('identities and samples per source:', dict(collections.Counter((p[1], p[4]) for p in ppl)))
print('\nper second: robot yaw | id:source@(x,y) ...')
T = int((max([p[0] for p in ppl] + [y[0] for y in yaws]) - first)) + 1
yi = 0
for s in range(T):
    while yi < len(yaws)-1 and yaws[yi][0]-first < s: yi += 1
    cur = {}
    for p in ppl:
        if s <= p[0]-first < s+1: cur[p[1]] = p
    txt = '  '.join('%s:%s@(%.2f,%.2f)' % (k, v[4], v[2], v[3]) for k, v in sorted(cur.items())) or '-'
    print('t+%3d s  yaw %6.1f | %s' % (s, ((yaws[yi][1]+180) % 360)-180 if yaws else float('nan'), txt))
print('\nlidar_hold episodes (gap > 1.5 s starts a new one):')
ep = {}
for p in ppl:
    if p[4] != 'lidar_hold': continue
    e = ep.setdefault(p[1], [])
    if e and p[0]-e[-1][-1][0] <= 1.5: e[-1].append(p)
    else: e.append([p])
if not ep: print('  none')
for k, es in sorted(ep.items()):
    for e in es:
        drift = max(math.hypot(q[2]-e[0][2], q[3]-e[0][3]) for q in e)
        after = [p for p in ppl if p[1] == k and p[0] > e[-1][0]]
        nxt = ('next sample of this id %.1f s later, source %s' % (after[0][0]-e[-1][0], after[0][4])) if after else 'id not published again'
        print('  id %s: t+%.1f to t+%.1f s = %.1f s, %d samples, anchor (%.2f, %.2f), max drift %.2f m; %s'
              % (k, e[0][0]-first, e[-1][0]-first, e[-1][0]-e[0][0], len(e), e[0][2], e[0][3], drift, nxt))
