import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

class ClickAngleInspector(Node):
    def __init__(self):
        super().__init__('click_angle_inspector')
        self.bridge = CvBridge()
        self.fx = 201.3076
        self.cx = 124.9516
        self.clicked_u = None
        self.sub = self.create_subscription(Image, '/turtlebot4/oakd/rgb/preview/image_raw', self.cb, 10)
        cv2.namedWindow('Click Inspector')
        cv2.setMouseCallback('Click Inspector', self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_u = x
            # Camera pinhole: angle from optical axis
            # Points left of cx have u < cx, so theta is positive to the left
            angle_rad = math.atan2(self.cx - x, self.fx)
            angle_deg = math.degrees(angle_rad)
            print(f'Clicked pixel column u={x} -> Visual Angle: {angle_deg:+.2f} deg (relative to center)')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        cv2.line(frame, (int(round(self.cx)), 0), (int(round(self.cx)), h), (0, 255, 0), 1)
        if self.clicked_u is not None:
            cv2.line(frame, (self.clicked_u, 0), (self.clicked_u, h), (255, 0, 0), 2)
        cv2.imshow('Click Inspector', frame)
        cv2.waitKey(1)

rclpy.init()
node = ClickAngleInspector()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
cv2.destroyAllWindows()
rclpy.shutdown()