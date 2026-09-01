#!/usr/bin/env python3
import os
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

SAVE_DIR = os.path.expanduser("~/Desktop/Turtlebot4/humble_client/Pictures")
TOPIC = "/turtlebot4/oakd/rgb/preview/image_raw/compressed"


class PhotoCapture(Node):
    def __init__(self):
        super().__init__("photo_capture")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.capturing = False
        self.capture_start = None
        self.sub = self.create_subscription(
            CompressedImage, TOPIC, self.image_cb, 10
        )
        os.makedirs(SAVE_DIR, exist_ok=True)

    def image_cb(self, msg):
        self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def spin_loop(self):
        cv2.namedWindow("OAK-D Preview", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_frame is None:
                continue

            frame = self.latest_frame.copy()

            if self.capturing:
                remaining = 5 - (time.time() - self.capture_start)
                if remaining <= 0:
                    filename = os.path.join(
                        SAVE_DIR, f"photo_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                    )
                    cv2.imwrite(filename, self.latest_frame)
                    self.get_logger().info(f"Saved: {filename}")
                    break
                else:
                    cv2.putText(
                        frame, f"{remaining:.1f}s", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3
                    )
            else:
                cv2.putText(
                    frame, "Press 'c' to capture, 'q' to quit", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                )

            cv2.imshow("OAK-D Preview", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c") and not self.capturing:
                self.capturing = True
                self.capture_start = time.time()
            elif key == ord("q"):
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = PhotoCapture()
    try:
        node.spin_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
