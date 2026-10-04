import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthCenter(Node):
    def __init__(self):
        super().__init__('depth_center')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/turtlebot4/oakd/stereo/image_raw', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        h, w = depth.shape
        d = depth[h // 2, w // 2]
        self.get_logger().info(f'{msg.encoding} {w}x{h} | center: {d} mm ({d / 1000:.2f} m)')

rclpy.init()
rclpy.spin(DepthCenter())
