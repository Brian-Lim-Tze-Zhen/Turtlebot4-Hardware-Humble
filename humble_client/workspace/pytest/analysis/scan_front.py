import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

got = []
def cb(m):
    if not got:
        got.append(m)

rclpy.init()
n = Node('scan_front')
n.create_subscription(LaserScan, '/turtlebot4/scan', cb, qos_profile_sensor_data)
while rclpy.ok() and not got:
    rclpy.spin_once(n, timeout_sec=1.0)
m = got[0]
a = m.angle_min
for i, r in enumerate(m.ranges):
    d = (math.degrees(a) + 90.0 + 180.0) % 360.0 - 180.0
    if -15 <= d <= 15 and i % 3 == 0:
        print(f'robot {d:+6.1f} deg  {r:.2f} m', flush=True)
    a += m.angle_increment
n.destroy_node()
rclpy.shutdown()
