#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CompressedImage
import numpy as np
import cv2
import math

class CalibratedCompressedProjection(Node):
    def __init__(self):
        super().__init__('calibrated_compressed_projection')

        # Calibrated Optical Intrinsics (250x250 Preview)
        self.fx = 287.31
        self.fy = 287.31
        self.cx = 124.9516
        self.cy = 124.9516

        # Calibrated Sensor Offsets (base_link)
        self.rplidar_x = -0.040
        self.rplidar_z =  0.193
        self.cam_x     =  0.165
        self.cam_y     =  0.000
        self.cam_z     =  0.123

        # Calibrated Extrinsic Yaw Bias
        self.yaw_deg = 0.75
        self.yaw_rad = math.radians(self.yaw_deg)
        # Expected target range (m) - pick the cluster nearest this,
        # not the nearest to the robot, so clutter between robot and
        # target cannot capture the overlay.
        self.expect_r = 3.0

        self.u_proj = None
        self.v_proj = None
        self.target_dist = 0.0
        self.target_deg = 0.0

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/turtlebot4/scan',
            self.scan_cb,
            10
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/turtlebot4/oakd/rgb/preview/image_raw/compressed',
            self.compressed_cam_cb,
            10
        )

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Polar to base_link Cartesian frame
        x_base = -ranges * np.sin(angles) + self.rplidar_x
        y_base =  ranges * np.cos(angles)

        # Spatial gating
        fov_mask = (x_base > 0.3) & (x_base < 3.5) & (np.abs(y_base) < 2.0) & np.isfinite(ranges)
        valid_indices = np.where(fov_mask)[0]

        if len(valid_indices) < 2:
            self.u_proj = None
            return

        # Euclidean Breakpoint Clustering
        clusters = []
        cur = [valid_indices[0]]
        for i in range(1, len(valid_indices)):
            p = valid_indices[i - 1]
            c = valid_indices[i]
            if (c - p == 1) and math.hypot(x_base[c] - x_base[p], y_base[c] - y_base[p]) < 0.12:
                cur.append(c)
            else:
                if len(cur) >= 2:
                    clusters.append(cur)
                cur = [c]
        if len(cur) >= 2:
            clusters.append(cur)

        # Pole width filter (2 cm to 15 cm)
        candidates = []
        for c in clusters:
            width = math.hypot(x_base[c[-1]] - x_base[c[0]], y_base[c[-1]] - y_base[c[0]])
            if 0.02 <= width <= 0.15:
                candidates.append((c, float(np.mean(ranges[c]))))

        self.get_logger().info(
            f"valid={len(valid_indices)} clusters={len(clusters)} cands={len(candidates)} "
            f"widths={[round(math.hypot(x_base[c[-1]]-x_base[c[0]], y_base[c[-1]]-y_base[c[0]]), 3) for c in clusters][:6]} "
            f"ranges={[round(float(np.mean(ranges[c])), 2) for c in clusters][:6]}",
            throttle_duration_sec=1.0)

        if candidates:
            best, _ = min(candidates, key=lambda item: abs(item[1] - self.expect_r))
            x_t = float(np.mean(x_base[best]))
            y_t = float(np.mean(y_base[best]))

            # 1. Apply calibrated yaw correction
            x_rot =  x_t * math.cos(self.yaw_rad) + y_t * math.sin(self.yaw_rad)
            y_rot = -x_t * math.sin(self.yaw_rad) + y_t * math.cos(self.yaw_rad)

            # 2. Map to Camera Optical Frame
            x_cam = -(y_rot - self.cam_y)
            y_cam = -(self.rplidar_z - self.cam_z)
            z_cam =  (x_rot - self.cam_x)

            # 3. Pinhole Projection
            if z_cam > 0.1:
                self.u_proj = int(round(self.cx + self.fx * (x_cam / z_cam)))
                self.v_proj = int(round(self.cy + self.fy * (y_cam / z_cam)))
                self.target_dist = math.hypot(x_t, y_t)
                self.target_deg = math.degrees(math.atan2(y_t, x_t))
        else:
            self.u_proj = None

    def compressed_cam_cb(self, msg: CompressedImage):
        # Decode JPEG buffer directly to OpenCV BGR array
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        h, w, _ = frame.shape

        # Optical boresight crosshair
        cx_i = int(round(self.cx))
        cv2.line(frame, (cx_i, 0), (cx_i, h), (0, 255, 0), 1)

        # Calibrated LiDAR Overlay
        if self.u_proj is not None and 0 <= self.u_proj < w:
            cv2.line(frame, (self.u_proj, 0), (self.u_proj, h), (0, 0, 255), 2)
            cv2.circle(frame, (self.u_proj, self.v_proj), 5, (0, 0, 255), -1)

            cv2.putText(frame, f'{self.target_dist:.2f}m @ {self.target_deg:+.1f} deg (u={self.u_proj}) exp={self.expect_r:.1f}m',
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 255), 1)

        cv2.imshow('Calibrated Compressed Projection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CalibratedCompressedProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()