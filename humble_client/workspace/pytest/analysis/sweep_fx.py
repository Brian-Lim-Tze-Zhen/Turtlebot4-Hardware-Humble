#!/usr/bin/env python3
"""Offline fx sweep for camera_ray_person_node.

Replays a bag's /person_positions_map + /scan, converts each camera
detection's leg-keypoint bearing to a LiDAR ray under several fx
values, and reports the residual (camera bearing - bearing of the
nearest scan return in the window). A correct fx makes the residual
flat across the frame; a wrong fx makes it grow toward the edges.

Usage:  python3 sweep_fx.py <bag.db3>
"""
import sqlite3, struct, math, sys
from collections import defaultdict

SCAN_TOPIC = "/turtlebot4/scan"
CAM_TOPIC = "/person_positions_map"
CX = 124.95
YAW_OFF = math.radians(2.25)
HALF_W = math.radians(15.0)
RMIN, RMAX = 0.20, 8.0
FX_LIST = [201.308, 287.31]

def align(o, a): return (o + a - 1)//a*a

def parse_scan(d):
    o = 4
    sec, nsec = struct.unpack_from('<iI', d, o); o += 8
    (ln,) = struct.unpack_from('<I', d, o); o += 4
    o += ln
    o = align(o, 4)
    amin, amax, ainc, tinc, stime, rmin, rmax = struct.unpack_from('<7f', d, o)
    o += 28
    (n,) = struct.unpack_from('<I', d, o); o += 4
    ranges = struct.unpack_from('<%df' % n, d, o)
    return sec + nsec*1e-9, amin, ainc, ranges

def parse_str(d):
    (ln,) = struct.unpack_from('<I', d, 4)
    return d[8:8+ln-1].decode('utf8', 'replace')

def bearing(u, fx): return -math.atan((u - CX)/fx) + YAW_OFF

def wrap(x):
    while x > math.pi: x -= 2*math.pi
    while x < -math.pi: x += 2*math.pi
    return x

def hit(amin, ainc, ranges, b):
    best_r = best_a = None
    for i, r in enumerate(ranges):
        a = amin + i*ainc
        if abs(wrap(a - b)) <= HALF_W and RMIN <= r <= RMAX \
           and not math.isinf(r) and not math.isnan(r):
            if best_r is None or r < best_r:
                best_r, best_a = r, a
    return best_r, best_a

def main(path):
    c = sqlite3.connect(path)
    T = {r[1]: r[0] for r in c.execute('select id,name from topics')}
    for t in (SCAN_TOPIC, CAM_TOPIC):
        if t not in T:
            print("missing topic:", t); return
    scans = [(ts, parse_scan(d)) for ts, d in c.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',
        (T[SCAN_TOPIC],))]
    cams = list(c.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',
        (T[CAM_TOPIC],)))
    print(f"{len(scans)} scans, {len(cams)} camera detections\n")

    res = {fx: defaultdict(list) for fx in FX_LIST}
    si = 0
    for ts, d in cams:
        while si + 1 < len(scans) and scans[si+1][0] <= ts:
            si += 1
        _, amin, ainc, ranges = scans[si][1]
        p = parse_str(d).split(',')
        if len(p) < 7 or not p[6].strip():
            continue
        k = []
        for pair in p[6].split(';'):
            if ':' in pair:
                try:
                    px, py = pair.split(':'); k.append((float(px), float(py)))
                except ValueError: pass
        if not k: continue
        k.sort(key=lambda q: -q[1])
        u = sum(q[0] for q in k[:2])/len(k[:2])
        for fx in FX_LIST:
            b = bearing(u, fx)
            r, a = hit(amin, ainc, ranges, b)
            if r is None: continue
            bucket = 5*round(math.degrees(b)/5)
            res[fx][bucket].append(math.degrees(wrap(a - b)))

    print(f"{'bearing':>9} | " + " | ".join(f"fx={fx:<8.2f} n" for fx in FX_LIST))
    print("-"*52)
    for bkt in sorted(set().union(*[set(res[fx]) for fx in FX_LIST])):
        row = f"{bkt:>6}deg  | "
        cells = []
        for fx in FX_LIST:
            v = res[fx].get(bkt, [])
            cells.append(f"{sum(v)/len(v):+6.2f} {len(v):>4}" if v else "    --    ")
        print(row + " | ".join(cells))
    print()
    for fx in FX_LIST:
        allv = [x for v in res[fx].values() for x in v]
        if not allv: continue
        m = sum(allv)/len(allv)
        sd = (sum((x-m)**2 for x in allv)/len(allv))**0.5
        print(f"fx={fx:.2f}: n={len(allv)} mean={m:+.2f} deg  sd={sd:.2f} deg  "
              f"max|res|={max(abs(x) for x in allv):.2f} deg")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "rosbag_0.db3")
