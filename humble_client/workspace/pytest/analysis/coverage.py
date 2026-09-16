#!/usr/bin/env python3
"""Coverage comparison: LiDAR-primary vs camera-primary detection.

Derives stationary segments from the camera-ray position stream, then
measures what fraction of each segment each pipeline covered.

IMPORTANT (fair comparison): the detector in these bags was already
patched to publish unconfirmed candidates at conf 0.00. The ORIGINAL
LiDAR-primary behaviour is therefore reconstructed by counting only
conf >= 1.00 messages on /lidar_person_clusters - those are exactly
the ones the unpatched detector would have published.

Usage: python3 coverage.py <bag.db3>
"""
import sqlite3, struct, math, sys

CAMRAY = "/camera_ray_clusters"
LIDAR = "/lidar_person_clusters"

STATIONARY_WIN_S = 3.0    # segment must last at least this long
STATIONARY_MAX_M = 0.25   # max position spread to count as "standing still"
COVER_BIN_S = 0.5         # a bin counts as covered if >=1 msg lands in it

def pstr(d):
    (ln,) = struct.unpack_from('<I', d, 4)
    return d[8:8+ln-1].decode('utf8', 'replace')

def load(c, T, topic):
    if topic not in T: return []
    out = []
    for ts, d in c.execute('select timestamp,data from messages where '
                           'topic_id=? order by timestamp', (T[topic],)):
        p = pstr(d).split(',')
        if len(p) < 4: continue
        try:
            out.append((ts/1e9, float(p[1]), float(p[2]), float(p[3])))
        except ValueError:
            pass
    return out

def segments(pts):
    """Maximal runs where position spread stays under STATIONARY_MAX_M."""
    segs, i = [], 0
    while i < len(pts):
        j = i
        while j + 1 < len(pts):
            xs = [p[2] for p in pts[i:j+2]]; ys = [p[3] for p in pts[i:j+2]]
            if (max(xs)-min(xs)) > STATIONARY_MAX_M or \
               (max(ys)-min(ys)) > STATIONARY_MAX_M:
                break
            j += 1
        if pts[j][0] - pts[i][0] >= STATIONARY_WIN_S:
            segs.append((pts[i][0], pts[j][0]))
            i = j + 1
        else:
            i += 1
    return segs

def coverage(msgs, t0, t1):
    n = max(1, int((t1-t0)/COVER_BIN_S))
    hit = set()
    for t, *_ in msgs:
        if t0 <= t <= t1:
            hit.add(min(n-1, int((t-t0)/COVER_BIN_S)))
    return len(hit)/n*100

def main(path):
    c = sqlite3.connect(path)
    T = {r[1]: r[0] for r in c.execute('select id,name from topics')}
    cam = load(c, T, CAMRAY)
    lid = load(c, T, LIDAR)
    lid_conf = [m for m in lid if m[1] >= 1.0]   # original behaviour
    if not cam:
        print("no /camera_ray_clusters in bag"); return

    segs = segments(cam)
    print(f"camera-ray msgs {len(cam)}, lidar msgs {len(lid)} "
          f"({len(lid_conf)} at conf>=1.00)")
    print(f"{len(segs)} stationary segment(s) "
          f">={STATIONARY_WIN_S}s, spread <{STATIONARY_MAX_M}m\n")
    print(f"{'segment':>18} {'dur':>6} | {'camera-ray':>11} "
          f"{'lidar(conf1)':>13} {'lidar(all)':>11}")
    print("-"*68)
    tot = [0.0, 0.0, 0.0, 0.0]
    for t0, t1 in segs:
        d = t1-t0
        a = coverage(cam, t0, t1)
        b = coverage(lid_conf, t0, t1)
        e = coverage(lid, t0, t1)
        print(f"{t0-cam[0][0]:7.1f}-{t1-cam[0][0]:6.1f}s {d:5.1f}s | "
              f"{a:10.1f}% {b:12.1f}% {e:10.1f}%")
        tot[0]+=d; tot[1]+=a*d; tot[2]+=b*d; tot[3]+=e*d
    if tot[0]:
        print("-"*68)
        print(f"{'weighted mean':>18} {tot[0]:5.1f}s | "
              f"{tot[1]/tot[0]:10.1f}% {tot[2]/tot[0]:12.1f}% "
              f"{tot[3]/tot[0]:10.1f}%")

if __name__ == "__main__":
    main(sys.argv[1])
