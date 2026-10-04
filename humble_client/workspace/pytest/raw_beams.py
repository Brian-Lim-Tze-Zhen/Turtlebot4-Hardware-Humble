import rclpy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

rclpy.init()
node = rclpy.create_node('raw_beam_debugger')

def cb(msg):
    ranges = np.array(msg.ranges)
    forward_angle = math.radians(-90.0)
    center_idx = int(round((forward_angle - msg.angle_min) / msg.angle_increment))
    window = int(math.radians(12.0) / msg.angle_increment)

    print('\033[H\033[J')
    print(f'=== RAW BEAMS, +/-12 deg ===')
    print(f'angle_increment = {math.degrees(msg.angle_increment):.3f} deg/beam')
    print(f'beams in window = {2 * window + 1}')
    print('Angle (deg) | Distance (m)')
    print('---------------------------')

    hits = 0
    for idx in range(center_idx - window, center_idx + window + 1):
        if 0 <= idx < len(ranges):
            ang = math.degrees(msg.angle_min + idx * msg.angle_increment - forward_angle)
            r = ranges[idx]
            mark = ''
            if np.isfinite(r) and r < 2.0:
                mark = '  <== TARGET'
                hits += 1
            print(f'{ang:11.2f} | {r:12.3f}{mark}')
    print('---------------------------')
    print(f'beams under 2.0 m: {hits}')
    rclpy.shutdown()

sub = node.create_subscription(LaserScan, '/turtlebot4/scan', cb, 10)
rclpy.spin(node)