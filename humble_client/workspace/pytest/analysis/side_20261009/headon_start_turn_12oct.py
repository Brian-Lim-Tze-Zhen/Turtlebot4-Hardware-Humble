#!/usr/bin/env python3
"""headon_start_turn_12oct.py - why run 6 of the final head-on set turned on the spot at the start (13 Oct 2026).
For every counted head-on run of 12 Oct, at the first motion command (t0): turn-on-the-spot commands of the
controller in the first 2 s; lethal/inscribed cells of the local costmap beside the robot on its left and closer
than the wall (fwd -0.2..0.6 m, lateral 0..0.65 m), and the same in the first costmap message of the bag; laser
returns in that area; static-map cells and person-cloud points near the robot. For run 6 it also draws the local
costmap around the robot. Robot frame: x forward, y left.
usage (robot image, offline): python3 headon_start_turn_12oct.py <bag dir> ... [--draw <bag dir>]"""
import sys, os, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2]); return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2])
def to_robot(P, x, y):
    dx, dy = x - P[0], y - P[1]; c, s = math.cos(-P[2]), math.sin(-P[2]); return c*dx - s*dy, s*dx + c*dy
BOX = lambda f, l: -0.2 <= f <= 0.6 and 0.0 <= l <= 0.65
TOPICS = ['/turtlebot4/tf_static', '/turtlebot4/map', '/turtlebot4/amcl_pose', '/turtlebot4/scan', '/turtlebot4/local_costmap/costmap',
          '/turtlebot4/cmd_vel_nav', '/turtlebot4/odom', '/predicted_person_cloud']

def load(d):
    rd = SequentialReader(); rd.open(StorageOptions(uri=os.path.join(d, 'bag'), storage_id='sqlite3'), ConverterOptions('', ''))
    ty = {t.name: t.type for t in rd.get_all_topics_and_types()}; M = {k: [] for k in TOPICS}
    while rd.has_next():
        tp, dat, s = rd.read_next()
        if tp in M: M[tp].append((s * 1e-9, deserialize_message(dat, get_message(ty[tp]))))
    return M
def near(L, t): return min(L, key=lambda a: abs(a[0] - t))[1]
def lethal_box(M, t):
    lc = near(M['/turtlebot4/local_costmap/costmap'], t); li = lc.info
    od = near(M['/turtlebot4/odom'], t).pose.pose; O = (od.position.x, od.position.y, yaw(od.orientation)); out = []
    for j in range(li.height):
        for i in range(li.width):
            if lc.data[j*li.width + i] >= 99:
                bx, by = to_robot(O, li.origin.position.x + (i+0.5)*li.resolution, li.origin.position.y + (j+0.5)*li.resolution)
                if BOX(bx, by): out.append((bx, by))
    return out, lc, O

def analyse(d, draw=False):
    M = load(d)
    st = {}
    for _, m in M['/turtlebot4/tf_static']:
        for tr in m.transforms: st[tr.child_frame_id] = (tr.header.frame_id, (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation)))
    f = M['/turtlebot4/scan'][0][1].header.frame_id; T = (0, 0, 0)
    while f in st and f != 'base_link': p, t = st[f]; T = comp(t, T); f = p
    t0 = next(t for t, m in M['/turtlebot4/cmd_vel_nav'] if abs(m.linear.x) > 0.02 or abs(m.angular.z) > 0.1)
    spin = sum(1 for t, m in M['/turtlebot4/cmd_vel_nav'] if 0 <= t - t0 <= 2.0 and abs(m.linear.x) < 0.05 and abs(m.angular.z) > 0.8)
    cells, lc, O = lethal_box(M, t0)
    first, _, _ = lethal_box(M, M['/turtlebot4/local_costmap/costmap'][0][0])
    pa = [a for a in M['/turtlebot4/amcl_pose'] if a[0] <= t0][-1][1].pose.pose; R = (pa.position.x, pa.position.y, yaw(pa.orientation))
    sc = near(M['/turtlebot4/scan'], t0); hits = 0; pts = []
    for i, r in enumerate(sc.ranges):
        if sc.range_min < r < sc.range_max and math.isfinite(r):
            a = sc.angle_min + i*sc.angle_increment; pb = comp(T, (r*math.cos(a), r*math.sin(a), 0)); pts.append(pb)
            if BOX(pb[0], pb[1]): hits += 1
    mp = M['/turtlebot4/map'][0][1]; res = mp.info.resolution; ox, oy = mp.info.origin.position.x, mp.info.origin.position.y; W = mp.info.width
    stat = [to_robot(R, ox + (i+0.5)*res, oy + (j+0.5)*res) for j in range(mp.info.height) for i in range(W) if mp.data[j*W+i] > 50]
    stat_left = min((by for bx, by in stat if -0.2 <= bx <= 0.6 and 0 <= by <= 1.2), default=float('nan'))
    cl = [c for c in M['/predicted_person_cloud'] if c[0] <= t0]
    cloud_near = sum(1 for q in point_cloud2.read_points(cl[-1][1], field_names=('x', 'y'), skip_nans=True) if math.hypot(*to_robot(R, q[0], q[1])) < 1.2) if cl else 0
    print(f'{os.path.basename(d.rstrip("/"))} | turn-on-the-spot commands in the first 2 s: {spin} | lethal cells beside the robot (left, closer than the wall): '
          f'{len(cells)} at the start, {len(first)} in the first costmap ({t0 - M["/turtlebot4/local_costmap/costmap"][0][0]:.1f} s before) '
          f'| laser returns there: {hits} | nearest static-map cell on the left: {stat_left:.2f} m | person-cloud points within 1.2 m: {cloud_near}', flush=True)
    if draw:
        hitc = {(round(p[0]/0.1), round(p[1]/0.1)) for p in pts}
        print('   local costmap at the start (robot R facing up; columns lateral +1.2 (left) .. -1.2 m; rows forward +1.2 .. -0.6 m; 0.1 m)')
        print('   # lethal/inscribed  + cost 50-98  . low  o laser return  @ laser return on a lethal cell')
        li = lc.info
        for fx in range(12, -7, -1):
            row = ''
            for ly in range(12, -13, -1):
                if fx == 0 and ly == 0: row += 'R'; continue
                x, y, _ = comp(O, (fx*0.1, ly*0.1, 0))
                i = int((x - li.origin.position.x)/li.resolution); j = int((y - li.origin.position.y)/li.resolution)
                v = lc.data[j*li.width + i] if 0 <= i < li.width and 0 <= j < li.height else -1
                ch = '#' if v >= 99 else '+' if v >= 50 else '.'
                if (fx, ly) in hitc: ch = '@' if ch == '#' else 'o'
                row += ch
            print(f'   {fx*0.1:+.1f} {row}')

args = sys.argv[1:]; draw = set()
if '--draw' in args: k = args.index('--draw'); draw.add(args[k+1]); args = args[:k] + args[k+2:]
for d in args + [x for x in draw if x not in args]: analyse(d, d in draw)
