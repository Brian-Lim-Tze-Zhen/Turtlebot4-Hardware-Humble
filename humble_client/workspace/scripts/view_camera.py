#!/usr/bin/env python3
"""
Subscribe to the TurtleBot4 OAKD depth camera topic, visualize it as a colormap,
and display metric distance readings in OpenCV.

Run inside the Humble container (ROS Humble sourced, domain 2, matching
CYCLONEDDS_URI already set via docker-compose environment).

Usage:
    # For raw depth image:
    python3 view_depth.py --topic /oakd/rgbd/depth

    # For compressedDepth topic:
    python3 view_depth.py --topic /oakd/rgbd/depth/compressedDepth --compressed

Press 'q' or Ctrl+C to quit.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class DepthViewer(Node):
    def __init__(self, topic: str, compressed: bool, max_depth: float):
        super().__init__('depth_viewer')
        self.bridge = CvBridge()
        self.compressed = compressed
        self.max_depth = max_depth  # Maximum distance in meters for colormap normalization
        self.frame_count = 0
        self.mouse_x = -1
        self.mouse_y = -1
        self.latest_depth_mm = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        msg_type = CompressedImage if compressed else Image
        self.subscription = self.create_subscription(
            msg_type, topic, self.depth_callback, qos
        )
        self.get_logger().info(f"Subscribed to '{topic}' (compressed={compressed})")
        self.get_logger().info("Hover over pixels to inspect distance. Press 'q' in the window to quit.")

        cv2.namedWindow("TurtleBot4 Depth Camera")
        cv2.setMouseCallback("TurtleBot4 Depth Camera", self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_x = x
            self.mouse_y = y

    def depth_callback(self, msg):
        try:
            if self.compressed:
                # ROS compressedDepth messages contain a 12-byte header before the PNG payload
                depth_bytes = np.frombuffer(msg.data[12:], dtype=np.uint8)
                depth_image = cv2.imdecode(depth_bytes, cv2.IMREAD_UNCHANGED)
            else:
                # passthrough retains 16UC1 (millimeters) or 32FC1 (meters)
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        if depth_image is None:
            return

        # Normalize depth array to metric units (meters)
        if depth_image.dtype == np.uint16:
            depth_m = depth_image.astype(np.float32) / 1000.0
        elif depth_image.dtype == np.float32:
            depth_m = depth_image
        else:
            self.get_logger().warn(f"Unexpected image dtype: {depth_image.dtype}")
            return

        h, w = depth_image.shape[:2]
        self.frame_count += 1

        # Clip and convert to 8-bit for colormap visualization (0 to max_depth meters)
        clipped = np.clip(depth_m, 0.0, self.max_depth)
        depth_8u = ((clipped / self.max_depth) * 255.0).astype(np.uint8)
        color_depth = cv2.applyColorMap(depth_8u, cv2.COLORMAP_TURBO)

        # Handle invalid / zero-depth pixels (render as black)
        color_depth[depth_m <= 0.01] = [0, 0, 0]

        # Calculate depth at image center
        center_x, center_y = w // 2, h // 2
        center_dist = depth_m[center_y, center_x]

        # Draw crosshair at center
        cv2.drawMarker(
            color_depth, (center_x, center_y),
            (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2
        )

        # Draw text overlays
        cv2.putText(
            color_depth, f"Frame: {self.frame_count}", (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        center_text = f"Center: {center_dist:.2f} m" if center_dist > 0 else "Center: Out of range"
        cv2.putText(
            color_depth, center_text, (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
        )

        # Show hovered pixel distance if cursor is inside the window
        if 0 <= self.mouse_x < w and 0 <= self.mouse_y < h:
            hover_dist = depth_m[self.mouse_y, self.mouse_x]
            hover_text = f"Cursor ({self.mouse_x},{self.mouse_y}): {hover_dist:.2f} m" if hover_dist > 0 else f"Cursor ({self.mouse_x},{self.mouse_y}): No reading"
            cv2.putText(
                color_depth, hover_text, (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

        cv2.imshow("TurtleBot4 Depth Camera", color_depth)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit requested, shutting down.")
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="View TurtleBot4 Depth camera feed with metric values")
    parser.add_argument(
        '--topic', default='/oakd/rgbd/depth/compressedDepth',
        help="Depth topic to subscribe to (default: %(default)s)"
    )
    parser.add_argument(
        '--compressed', action='store_true', default=True,
        help="Set if the topic is compressedDepth (default: True)"
    )
    parser.add_argument(
        '--max-range', type=float, default=5.0,
        help="Maximum range in meters for color gradient scaling (default: 5.0)"
    )
    args = parser.parse_args()

    rclpy.init()
    node = DepthViewer(args.topic, args.compressed, args.max_range)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()