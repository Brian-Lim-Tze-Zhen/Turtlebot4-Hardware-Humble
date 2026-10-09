#!/usr/bin/env python3
"""extract_runs.py - per-run data of the rec20 noghost head-on runs for the results page (9 Oct 2026).

Run in the robot container (domain 77):
  python3 extract_runs.py runs_rec20.json > rec20_runs_data.json
runs_rec20.json: [{"name", "bag" (bag folder with bag/), "t0", "track", "events": [[t, text], ...], "loc_err_deg"}, ...]

Hallway frame: origin (-6.88, 13.41) (start since 14:25), axis towards the goal (-12.20, 17.75): s along, l sideways,
+l = robot's left. Per run: robot path (odom placed with map->odom), walker estimates (/person_positions_fused, with
source), the nearest LiDAR return that is NOT on the static map (3-cell margin, as blocked_person_beep_node_front.py),
speeds; walls of the hallway from the map.
"""
import sys, json, math, bisect
import numpy as np, yaml
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SX, SY, GX, GY = -6.88, 13.41, -12.20, 17.75
H = math.atan2(GY - SY, GX - SX)
c, s_ = math.cos(H), math.sin(H)
def SL(x, y):
    dx, dy = x - SX, y - SY
    return dx * c + dy * s_, -dx * s_ + dy * c

M = yaml.safe_load(open('/root/thesis_social_navigation_ws/map/hall_map_name.yaml'))
img = np.array(Image.open('/root/thesis_social_navigation_ws/map/' + M['image']))
occ = img < 100; res = M['resolution']; ox0, oy0 = M['origin'][:2]; rows = img.shape[0]
def occupied(x, y, margin=3):
    cx = int((x - ox0) / res); cy = rows - 1 - int((y - oy0) / res)
    a = occ[max(cy - margin, 0):cy + margin + 1, max(cx - margin, 0):cx + margin + 1]
    return a.size > 0 and bool(a.any())

def walls():
    out = []
    for sv in np.arange(-0.5, 7.51, 0.25):
        side = {}
        for sign in (1, -1):
            for lv in np.arange(0.0, 2.5, 0.025):
                x = SX + sv * c - sign * lv * s_; y = SY + sv * s_ + sign * lv * c
                if occupied(x, y, 0):
                    side[sign] = round(sign * lv, 3); break
        out.append([round(float(sv), 2), side.get(1), side.get(-1)])
    return out

def run(R):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=R['bag'] + '/bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/turtlebot4/tf', '/turtlebot4/tf_static', '/turtlebot4/odom',
                                                   '/turtlebot4/scan', '/person_positions_fused']))
    T0 = R['t0']; m2o = None; laser = None; robot = None
    path, walker, near = [], [], []
    last_p = -1
    while rd.has_next():
        t, d, ts = rd.read_next(); tt = ts * 1e-9 - T0
        if t.endswith('tf') or t.endswith('tf_static'):
            for tr in deserialize_message(d, TFMessage).transforms:
                q = tr.transform.rotation
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    m2o = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
                if tr.child_frame_id.endswith('rplidar_link'):
                    laser = (tr.transform.translation.x, tr.transform.translation.y, 2 * math.atan2(q.z, q.w))
            continue
        if tt < -1.0 or tt > 32:
            continue
        if t.endswith('odom') and m2o:
            m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
            ox, oy, oth = m2o
            x = ox + p.x * math.cos(oth) - p.y * math.sin(oth); y = oy + p.x * math.sin(oth) + p.y * math.cos(oth)
            yaw = oth + 2 * math.atan2(q.z, q.w)
            robot = (x, y, yaw)
            if tt - last_p >= 0.1:
                sv, lv = SL(x, y)
                path.append([round(tt, 2), round(sv, 3), round(lv, 3),
                             round((math.degrees(yaw - H) + 180) % 360 - 180, 1), round(m.twist.twist.linear.x, 3)])
                last_p = tt
        elif t.endswith('scan') and robot and laser:
            m = deserialize_message(d, LaserScan)
            rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
            ok = np.isfinite(rg) & (rg > 0.15) & (rg < 2.5)
            rx, ry, ryaw = robot
            lx = rx + laser[0] * math.cos(ryaw) - laser[1] * math.sin(ryaw)
            ly = ry + laser[0] * math.sin(ryaw) + laser[1] * math.cos(ryaw)
            th = ryaw + laser[2]
            best = None
            for r, an in sorted(zip(rg[ok], a[ok])):
                x = lx + r * math.cos(th + an); y = ly + r * math.sin(th + an)
                if not occupied(x, y):
                    best = (float(r), x, y); break
            if best:
                sv, lv = SL(best[1], best[2])
                near.append([round(tt, 2), round(best[0], 3), round(sv, 2), round(lv, 2)])
        elif t == '/person_positions_fused':
            for e in deserialize_message(d, String).data.split(';'):
                f = e.split(',')
                if len(f) > 3:
                    sv, lv = SL(float(f[2]), float(f[3]))
                    walker.append([round(tt, 2), int(f[0]), round(sv, 2), round(lv, 2), f[-2] if len(f) > 11 else ''])
    return {'name': R['name'], 'events': R['events'], 'loc_err_deg': R['loc_err_deg'], 'track': R['track'],
            'path': path, 'walker': walker, 'near': near}

runs = json.load(open(sys.argv[1]))
print(json.dumps({'frame': {'origin': [SX, SY], 'goal': [GX, GY], 'heading_deg': round(math.degrees(H), 2)},
                  'walls': walls(), 'runs': [run(R) for R in runs]}))
