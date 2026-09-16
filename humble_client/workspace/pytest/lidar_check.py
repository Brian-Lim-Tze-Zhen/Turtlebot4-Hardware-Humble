import rclpy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

rclpy.init()
node = rclpy.create_node('scan_debugger')

def cb(msg):
    ranges = np.array(msg.ranges)
    forward_angle = math.radians(-90.0)
    center_idx = int(round((forward_angle - msg.angle_min) / msg.angle_increment))
    
    print('\033[H\033[J')  # Clear console
    print(f'=== FORWARD SCAN PROFILE (Center Heading: -90°) ===')
    print(f'Angle (deg) | Distance (m) | Visual Bar')
    print('----------------------------------------------------')
    
    # Sample from -35 deg to +35 deg in steps of ~2 deg
    step = int(math.radians(2.0) / msg.angle_increment)
    window = int(math.radians(35.0) / msg.angle_increment)
    
    for idx in range(center_idx - window, center_idx + window + 1, step):
        if 0 <= idx < len(ranges):
            ang = math.degrees(msg.angle_min + idx * msg.angle_increment - forward_angle)
            r = ranges[idx]
            bar = '#' * int(min(r, 4.0) * 10) if not np.isinf(r) else 'INF'
            marker = ' <-- [0° Boresight]' if abs(ang) < 1.0 else ''
            print(f'{ang:11.1f}° | {r:12.3f} | {bar}{marker}')
    rclpy.shutdown()

sub = node.create_subscription(LaserScan, '/turtlebot4/scan', cb, 10)
rclpy.spin(node)
