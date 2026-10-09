import sys, math, rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
for arg in sys.argv[1:]:
    b, dlane = arg.split(':'); dlane = float(dlane)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/predicted_person_positions', '/sim_ground_truth_pose']))
    robot = None; samples = []
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/sim_ground_truth_pose':
            from rosidl_runtime_py.utilities import get_message
            m = deserialize_message(d, get_message(types[t]))
            p = m.pose.pose.position if hasattr(m.pose, 'pose') else (m.pose.position if hasattr(m, 'pose') else m.position)
            robot = (p.x, p.y)
        elif robot:
            for e in deserialize_message(d, String).data.split(';'):
                f = e.split(',')
                if len(f) > 3 and f[0] == '0':
                    wx, wy = float(f[2]), float(f[3]); dist = math.hypot(wx - robot[0], wy - robot[1])
                    if dist <= dlane + 0.05 and len(samples) < 12:
                        samples.append((round(dist, 2), round(wy, 2)))
    # robot drives +x towards the goal (8,0) on y=0, so its left is +y: offset = walker y
    print(b.split('/')[-1], 'first estimates (dist, walker y; offset = y, + = robot left):', [(d, y) for d, y in samples[:6]])
