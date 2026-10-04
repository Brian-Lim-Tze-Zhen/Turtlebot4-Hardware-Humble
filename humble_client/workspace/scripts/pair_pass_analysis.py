#!/usr/bin/env python3
"""Why did the robot oscillate in front of social pair (3,4)? Reads the rosbag offline."""
import os, re, math, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Window is found automatically: newest bag, starting 5 s before the first
# "Pair (x,y) confirmed" log line, lasting WINDOW_S seconds.
WINDOW_S = 150.0
T0 = T1 = 0.0
SNAPS = []
A_DEF, B_DEF = (-4.00, 12.60), (-4.85, 11.45)  # fallback pair positions from log
SKIP_DIRS = {'build', 'install', 'log', 'src', 'node_modules'}


def find_bag():
    hits = []
    for root, dirs, files in os.walk('/root'):
        dirs[:] = [d for d in dirs if not d.startswith('.') and d not in SKIP_DIRS]
        if 'metadata.yaml' not in files:
            continue
        try:
            info = yaml.safe_load(open(os.path.join(root, 'metadata.yaml')))['rosbag2_bagfile_information']
            s = info['starting_time']['nanoseconds_since_epoch'] / 1e9
            e = s + info['duration']['nanoseconds'] / 1e9
            hits.append((root, s, e, info.get('storage_identifier', 'sqlite3')))
        except Exception:
            pass
    if not hits:
        print('No bags found under /root'); raise SystemExit(1)
    return max(hits, key=lambda h: h[1])   # newest recording


def open_reader(bag, sid, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inv_apply(T, x, y):
    dx, dy = x - T[0], y - T[1]
    c, s = math.cos(T[2]), math.sin(T[2])
    return (c * dx + s * dy, -s * dx + c * dy)


def seg_cross(p1, p2, p3, p4):
    ccw = lambda a, b, c: (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)


bag, bs, be, sid = find_bag()
print(f'BAG: {bag}  ({sid})')
types = {t.name: t.type for t in open_reader(bag, sid, None).get_all_topics_and_types()}

# ---- pass 0: find when the pair was confirmed ----
rosout = [n for n, t in types.items() if t == 'rcl_interfaces/msg/Log']
t_conf = None
if rosout:
    r = open_reader(bag, sid, rosout)
    while r.has_next():
        n, d, t = r.read_next()
        m = deserialize_message(d, get_message(types[n]))
        if 'confirmed:' in m.msg and m.msg.startswith('Pair'):
            t_conf = t / 1e9
            print(f'FIRST GROUP: +{t_conf - bs:.1f}s into bag  ({m.msg})')
            break
if t_conf is None:
    print('No "Pair ... confirmed" in /rosout - analysing from bag start')
    t_conf = bs + 5.0
T0 = t_conf - 5.0
T1 = min(be, T0 + WINDOW_S)
SNAPS = [T0 + k for k in range(5, int(T1 - T0), 5)]

# ---- pass 1: pair positions from /rosout ----
As, Bs = [], []
pat = re.compile(r'A=\((-?[\d.]+),(-?[\d.]+)\) B=\((-?[\d.]+),(-?[\d.]+)\)')
warns = []
sd_lines = []
hold_log = []
pen_log = []
node_log = []
pen_pat = re.compile(r'penalised (\d+)/(\d+), cost min/mean/max ([\d.]+)/([\d.]+)/([\d.]+), spread ([\d.]+)')
if rosout:
    r = open_reader(bag, sid, rosout)
    while r.has_next():
        n, d, t = r.read_next()
        ts = t / 1e9
        if ts < T0 or ts > T1:
            continue
        m = deserialize_message(d, get_message(types[n]))
        if 'lidar-hold' in m.msg and 'held on LIDAR' in m.msg:
            g = pat.search(m.msg)
            if g:
                As.append((float(g[1]), float(g[2]))); Bs.append((float(g[3]), float(g[4])))
                hold_log.append((ts, (float(g[1]), float(g[2])), (float(g[3]), float(g[4]))))
        elif 'SocialCritic' in m.msg and 'penalised' in m.msg:
            g = pen_pat.search(m.msg)
            if g:
                pen_log.append((ts, int(g[1]), int(g[2]), float(g[4]), float(g[5]), float(g[6])))
            else:
                g0 = re.search(r'penalised 0/(\d+)', m.msg)
                if g0:
                    pen_log.append((ts, 0, int(g0[1]), 0.0, 0.0, 0.0))
        if m.name in ('social_group_detector', 'social_zone_costmap_node') and '[diag]' not in m.msg:
            node_log.append((ts, m.name, m.msg))
        if 'sd=' in m.msg and 'SocialCritic' in m.msg:
            sd_lines.append((ts, m.msg))
        elif m.level >= 30 and m.name not in ('social_group_detector', 'social_zone_costmap_node'):
            warns.append((ts, m.name, m.msg))
med = lambda L, i: sorted(p[i] for p in L)[len(L) // 2]
A = (med(As, 0), med(As, 1)) if As else A_DEF
B = (med(Bs, 0), med(Bs, 1)) if Bs else B_DEF
M = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
print(f'PAIR: A={A[0]:.2f},{A[1]:.2f}  B={B[0]:.2f},{B[1]:.2f}  centre gap={math.dist(A, B):.2f} m  '
      f'(source: {"rosout" if As else "fallback"})')

# ---- pass 2: plans, costmaps, tf, cmd_vel ----
plan_t = [n for n, t in types.items() if t == 'nav_msgs/msg/Path']
grid_t = [n for n, t in types.items() if t == 'nav_msgs/msg/OccupancyGrid'
          and ('costmap' in n or 'social_zone' in n)]
upd_t = [n for n, t in types.items() if t == 'map_msgs/msg/OccupancyGridUpdate' and n[:-8] in grid_t]
tf_t = [n for n, t in types.items() if t == 'tf2_msgs/msg/TFMessage']
cmd_t = [n for n, t in types.items() if 'cmd_vel' in n and t.startswith('geometry_msgs/msg/Twist')]
person_t = [n for n, t in types.items() if t == 'std_msgs/msg/String' and n.endswith('person_positions_fused')]
print('TOPICS: plans=%s\n        grids=%s\n        cmd=%s\n        people=%s' % (plan_t, grid_t, cmd_t, person_t))

grids, tfs = {}, {}
plans = {n: [] for n in plan_t}
cmds = []
tracks = []   # (ts, id, x, y)
zone_counts = []
poses = []
snaps = []
snap_i = 0
gcm = next((g for g in grid_t if 'global_costmap' in g), None)
szm = next((g for g in grid_t if 'social_zone' in g), None)


def cost_at(topic, x, y):
    g = grids.get(topic)
    if not g:
        return None
    if g['frame'] != 'map':
        T = tfs.get(('map', g['frame']))
        if not T:
            return None
        x, y = inv_apply(T, x, y)
    inf = g['info']
    i = int(math.floor((x - inf.origin.position.x) / inf.resolution))
    j = int(math.floor((y - inf.origin.position.y) / inf.resolution))
    if 0 <= i < inf.width and 0 <= j < inf.height:
        return g['data'][j * inf.width + i]
    return None


def robot_pose():
    a, b = tfs.get(('map', 'odom')), tfs.get(('odom', 'base_link'))
    return compose(a, b) if a and b else None


r = open_reader(bag, sid, plan_t + grid_t + upd_t + tf_t + cmd_t + person_t)
while r.has_next():
    n, d, t = r.read_next()
    ts = t / 1e9
    if ts > T1 + 1:
        break
    m = deserialize_message(d, get_message(types[n]))
    if n in tf_t:
        for tr in m.transforms:
            q = tr.transform.rotation
            tfs[(tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))] = (
                tr.transform.translation.x, tr.transform.translation.y, yaw(q))
    elif n in grid_t:
        grids[n] = dict(info=m.info, frame=m.header.frame_id.lstrip('/'), data=list(m.data))
        if n == szm:
            zone_counts.append((ts, sum(1 for v in m.data if v > 0)))
    elif n in upd_t:
        g = grids.get(n[:-8])
        if g:
            W = g['info'].width
            for row in range(m.height):
                s = (m.y + row) * W + m.x
                g['data'][s:s + m.width] = list(m.data[row * m.width:(row + 1) * m.width])
    elif ts < T0:
        continue
    elif n in person_t:
        f = m.data.split(',')
        try:
            tracks.append((ts, f[0].strip(), float(f[2]), float(f[3])))
        except (IndexError, ValueError):
            pass
        continue
    elif n in plan_t:
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        if len(pts) < 2:
            plans[n].append((ts, 'EMPTY', 0, None, None)); continue
        thru = any(seg_cross(pts[k], pts[k + 1], A, B) for k in range(len(pts) - 1))
        length = sum(math.dist(pts[k], pts[k + 1]) for k in range(len(pts) - 1))
        gmax = max((c for c in (cost_at(gcm, *p) for p in pts) if c is not None), default=None)
        dmid = min(math.dist(p, M) for p in pts)
        plans[n].append((ts, 'THROUGH' if thru else 'AROUND', length, gmax, dmid))
    elif n in cmd_t:
        tw = m.twist if hasattr(m, 'twist') else m
        cmds.append((ts, tw.linear.x, tw.angular.z))
        p = robot_pose()
        if p:
            poses.append((ts, p))
    while snap_i < len(SNAPS) and ts >= SNAPS[snap_i]:
        prof = {}
        for gname in grid_t:
            vals = [cost_at(gname, A[0] + (B[0] - A[0]) * k / 20, A[1] + (B[1] - A[1]) * k / 20) for k in range(21)]
            prof[gname] = vals
        snaps.append((SNAPS[snap_i], robot_pose(), prof))
        snap_i += 1

# ---- report ----
rel = lambda ts: ts - T0
print('\n=== 1. GLOBAL PLAN: through the pair or around? (runs of same decision) ===')
for n, L in plans.items():
    if not L:
        continue
    flips = sum(1 for k in range(1, len(L)) if L[k][1] != L[k - 1][1])
    print(f'{n}: {len(L)} plans, {flips} switches')
    k = 0
    while k < len(L):
        j = k
        while j + 1 < len(L) and L[j + 1][1] == L[k][1]:
            j += 1
        seg = L[k:j + 1]
        lens = [s[2] for s in seg]
        gm = [s[3] for s in seg if s[3] is not None]
        dm = [s[4] for s in seg if s[4] is not None]
        print(f'  +{rel(seg[0][0]):5.1f}..+{rel(seg[-1][0]):5.1f}s  {seg[0][1]:7s} x{len(seg):<3d} '
              f'len {min(lens):.1f}-{max(lens):.1f} m  max global cost {max(gm) if gm else "-"}  '
              f'closest to gap centre {min(dm):.2f} m' if dm else '')
        k = j + 1

print('\n=== 2. COST ACROSS THE GAP, A -> B (21 samples, 0..100, "." = outside map) ===')
last_p = None
for ts, p, prof in snaps:
    # only print when the robot has moved >= 0.3 m since the last printed one
    if p and last_p and math.dist(p[:2], last_p[:2]) < 0.3:
        continue
    last_p = p
    ps = f'robot ({p[0]:.2f},{p[1]:.2f})' if p else 'robot ?'
    print(f'-- +{rel(ts):.0f}s  {ps}')
    for gname, vals in prof.items():
        free = sum(1 for v in vals if v is not None and 0 <= v < 99)
        print(f'  {gname[-32:]:>32s}: ' + ' '.join(f'{v:3d}' if v is not None else '  .' for v in vals)
              + f'  | <99 cells: {free}/21')

print('\n=== 3. ROBOT MOTION per second (cmd_vel + pose) ===')
for sec in range(int(T1 - T0) + 1):
    c = [x for x in cmds if sec <= rel(x[0]) < sec + 1]
    if not c:
        continue
    vx = sum(x[1] for x in c) / len(c)
    wz = sum(x[2] for x in c) / len(c)
    wflip = sum(1 for k in range(1, len(c)) if c[k][2] * c[k - 1][2] < 0)
    p = [x[1] for x in poses if sec <= rel(x[0]) < sec + 1]
    ps = f'({p[-1][0]:6.2f},{p[-1][1]:6.2f}) d_gap {math.dist(p[-1][:2], M):.2f}m' if p else ''
    print(f'  +{sec:2d}s  vx {vx:+.2f}  wz {wz:+.2f}  wz sign flips {wflip:2d}  {ps}')

print('\n=== 4. NAV2 WARNINGS/ERRORS in window (deduplicated) ===')
seen = set()
for ts, name, msg in warns:
    key = (name, msg[:70])
    if key in seen:
        continue
    seen.add(key)
    print(f'  +{rel(ts):5.1f}s [{name}] {msg[:150]}')
    if len(seen) >= 40:
        break
if not rosout:
    print('  (no /rosout in bag)')

print('\n=== 6. HESITATION: per second near the gap (d_gap < 2.5 m) ===')
print('  robot speed | SocialCritic penalised/batch, mean cost | detector A,B vs nearest tracked people')
for sec in range(int(T1 - T0) + 1):
    p = [x[1] for x in poses if sec <= rel(x[0]) < sec + 1]
    if not p or math.dist(p[-1][:2], M) > 2.5:
        continue
    rp = p[-1][:2]
    c = [x for x in cmds if sec <= rel(x[0]) < sec + 1]
    vx = sum(x[1] for x in c) / len(c) if c else 0.0
    pen = [x for x in pen_log if sec <= rel(x[0]) < sec + 1]
    pen_s = (f'{pen[-1][1]:4d}/{pen[-1][2]} mean {pen[-1][3]:7.1f}' if pen else '   -        ')
    hl = [x for x in hold_log if rel(x[0]) < sec + 1]
    a_d, b_d = (hl[-1][1], hl[-1][2]) if hl else (A, B)
    tr = {}
    for x in tracks:
        if sec <= rel(x[0]) < sec + 1:
            tr[x[1]] = (x[2], x[3])
    near = sorted(tr.items(), key=lambda kv: math.dist(kv[1], M))[:3]
    def off(q):
        da, db = math.dist(q, a_d), math.dist(q, b_d)
        return f'A+{da:.2f}' if da < db else f'B+{db:.2f}'
    tr_s = '  '.join(f'id{k}({v[0]:.2f},{v[1]:.2f}) {off(v)} r{math.dist(v, rp):.2f}' for k, v in near
                     if math.dist(v, M) < 2.0) or 'no tracks'
    print(f'  +{sec:3d}s vx {vx:+.2f} | {pen_s} | {tr_s}')
print('  (A+x / B+x = track distance from detector member A/B; r = distance to robot)')

print('\n=== 7. PEOPLE NEAR THE ROBOT (any track within 3.5 m) ===')
print('  robot (x,y) vx wz | nearest person: id, dist, speed toward robot | critic penalised, mean | zone cells')
for sec in range(int(T1 - T0) + 1):
    p = [x[1] for x in poses if sec <= rel(x[0]) < sec + 1]
    if not p:
        continue
    rp = p[-1]
    tr = {}
    for x in tracks:
        if sec - 1 <= rel(x[0]) < sec + 1:
            tr.setdefault(x[1], []).append((x[0], x[2], x[3]))
    best = None
    for tid, L in tr.items():
        L = [q for q in L if rel(q[0]) >= sec] or L[-1:]
        q = L[-1]
        d = math.dist((q[1], q[2]), rp[:2])
        if d > 3.5:
            continue
        allq = tr[tid]
        closing = 0.0
        if len(allq) >= 2 and allq[-1][0] > allq[0][0]:
            d0 = math.dist((allq[0][1], allq[0][2]), rp[:2])
            closing = (d0 - d) / (allq[-1][0] - allq[0][0])
        if best is None or d < best[1]:
            best = (tid, d, closing)
    if best is None:
        continue
    c = [x for x in cmds if sec <= rel(x[0]) < sec + 1]
    vx = sum(x[1] for x in c) / len(c) if c else 0.0
    wz = sum(x[2] for x in c) / len(c) if c else 0.0
    pen = [x for x in pen_log if sec <= rel(x[0]) < sec + 1]
    pen_s = f'{pen[-1][1]:4d}/{pen[-1][2]} mean {pen[-1][3]:7.1f}' if pen else '   -              '
    zc = [x[1] for x in zone_counts if rel(x[0]) < sec + 1]
    print(f'  +{sec:3d}s ({rp[0]:6.2f},{rp[1]:6.2f}) vx {vx:+.2f} wz {wz:+.2f} | id{best[0]} {best[1]:.2f}m '
          f'closing {best[2]:+.2f}m/s | {pen_s} | {zc[-1] if zc else "-"}')

print('\n=== 8. GROUP DETECTOR / ZONE NODE log (no [diag], max 40) ===')
seen8 = set()
for ts, name, msg in node_log:
    key = msg[:60]
    if key in seen8:
        continue
    seen8.add(key)
    print(f'  +{rel(ts):5.1f}s [{name}] {msg[:140]}')
    if len(seen8) >= 40:
        break
if not node_log:
    print('  (no detector / zone node lines in /rosout)')

print('\n=== 5. SOCIALCRITIC CLEARANCE (sd) over time ===')
sds = {}
for ts, msg in sd_lines:
    g = re.search(r'sd=([\d.]+)', msg)
    if g:
        sds.setdefault(int(rel(ts) // 5) * 5, []).append(float(g[1]))
for k in sorted(sds):
    v = sds[k]
    print(f'  +{k:3d}..+{k + 5}s  sd min {min(v):.2f}  max {max(v):.2f}  ({len(v)} lines)')
if not sds:
    print('  no "sd=" lines - new SocialCritic not loaded, or /rosout not recorded')
