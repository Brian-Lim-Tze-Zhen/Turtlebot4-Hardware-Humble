#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
import tf2_ros


def quat_to_rot_matrix(q):
  """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
  x, y, z, w = q
  return np.array([
      [
          1.0 - 2.0 * (y**2 + z**2),
          2.0 * (x * y - z * w),
          2.0 * (x * z + y * w),
      ],
      [
          2.0 * (x * y + z * w),
          1.0 - 2.0 * (x**2 + z**2),
          2.0 * (y * z - x * w),
      ],
      [
          2.0 * (x * z - y * w),
          2.0 * (y * z + x * w),
          1.0 - 2.0 * (x**2 + y**2),
      ],
  ])


class LidarCameraProjector(Node):

  def __init__(self):
    super().__init__('lidar_camera_projector')

    # TF Buffer & Listener
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # Re-route standard TF subscriber topics if they are published inside the robot namespace
    tf_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=100,
    )
    tf_static_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=10,
    )

    from tf2_msgs.msg import TFMessage

    self.sub_tb4_tf = self.create_subscription(
        TFMessage,
        '/turtlebot4/tf',
        lambda msg: [
            self.tf_buffer.set_transform(t, 'default_authority')
            for t in msg.transforms
        ],
        tf_qos,
    )
    self.sub_tb4_tf_static = self.create_subscription(
        TFMessage,
        '/turtlebot4/tf_static',
        lambda msg: [
            self.tf_buffer.set_transform_static(t, 'default_authority')
            for t in msg.transforms
        ],
        tf_static_qos,
    )

    # Sensor QoS profiles
    sensor_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=10,
    )
    camera_info_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=10,
    )

    # State variables
    self.K = None
    self.latest_cv_img = None
    self.latest_projected_pts = []  # List of (u, v, dist)
    self.tf_resolved = False

    # Default calibrated TurtleBot 4 mechanical extrinsics:
    # Transforms rplidar_link to oakd_rgb_camera_optical_frame:
    # Camera optical frame: X right, Y down, Z forward
    # RPLiDAR frame: X forward, Y left, Z up
    self.extrinsic_R = np.array(
        [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
    )
    self.extrinsic_t = np.array(
        [0.0, 0.12, 0.05]
    ).reshape(  # LiDAR is ~12cm higher and ~5cm behind camera lens
        3, 1
    )

    # Subscriptions
    self.sub_info = self.create_subscription(
        CameraInfo,
        '/turtlebot4/oakd/rgb/preview/camera_info',
        self.info_callback,
        camera_info_qos,
    )
    self.sub_image = self.create_subscription(
        CompressedImage,
        '/turtlebot4/oakd/rgb/preview/image_raw/compressed',
        self.image_callback,
        sensor_qos,
    )
    self.sub_scan = self.create_subscription(
        LaserScan, '/turtlebot4/scan', self.scan_callback, sensor_qos
    )

    self.get_logger().info(
        'Lidar-Camera Projection node started. Window initialized.'
    )

  def info_callback(self, msg: CameraInfo):
    if self.K is None:
      self.K = np.array(msg.k).reshape(3, 3)
      self.get_logger().info(
          f'Camera Intrinsics K loaded: fx={self.K[0,0]:.1f},'
          f' cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}'
      )

  def resolve_extrinsics(self, target_frame, source_frame):
    try:
      t_geom = self.tf_buffer.lookup_transform(
          target_frame, source_frame, rclpy.time.Time(), Duration(seconds=0.05)
      )
      tx = t_geom.transform.translation.x
      ty = t_geom.transform.translation.y
      tz = t_geom.transform.translation.z
      self.extrinsic_t = np.array([tx, ty, tz]).reshape(3, 1)

      q = [
          t_geom.transform.rotation.x,
          t_geom.transform.rotation.y,
          t_geom.transform.rotation.z,
          t_geom.transform.rotation.w,
      ]
      self.extrinsic_R = quat_to_rot_matrix(q)
      return True
    except Exception:
      return False

  def scan_callback(self, msg: LaserScan):
    if self.K is None:
      return

    # Attempt to resolve exact dynamic TF from bag
    if not self.tf_resolved:
      target_optical_frame = 'oakd_rgb_camera_optical_frame'
      source_lidar_frame = msg.header.frame_id
      if self.resolve_extrinsics(target_optical_frame, source_lidar_frame):
        self.tf_resolved = True
        self.get_logger().info(
            f"Successfully resolved dynamic TF '{source_lidar_frame}' ->"
            f" '{target_optical_frame}'"
        )

    # 1. Convert LaserScan polar coordinates to Cartesian
    angles = np.linspace(
        msg.angle_min, msg.angle_max, len(msg.ranges), endpoint=True
    )
    ranges = np.array(msg.ranges)

    valid_mask = (
        (ranges >= msg.range_min)
        & (ranges <= msg.range_max)
        & np.isfinite(ranges)
    )
    r = ranges[valid_mask]
    th = angles[valid_mask]

    x_l = r * np.cos(th)
    y_l = r * np.sin(th)
    z_l = np.zeros_like(r)
    pts_lidar = np.vstack((x_l, y_l, z_l))

    # 2. Transform LiDAR points into Camera Optical Frame
    pts_cam = self.extrinsic_R @ pts_lidar + self.extrinsic_t
    Xc, Yc, Zc = pts_cam[0], pts_cam[1], pts_cam[2]

    # 3. Filter points in front of the lens
    in_front = Zc > 0.1
    Xc, Yc, Zc, r_draw = Xc[in_front], Yc[in_front], Zc[in_front], r[in_front]

    if len(Zc) == 0:
      self.latest_projected_pts = []
      return

    # 4. Project onto image plane
    u = (self.K[0, 0] * Xc / Zc) + self.K[0, 2]
    v = (self.K[1, 1] * Yc / Zc) + self.K[1, 2]

    self.latest_projected_pts = list(zip(u, v, r_draw))

  def image_callback(self, msg: CompressedImage):
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img is None:
      return

    h, w, _ = img.shape

    # Draw all currently active projected LiDAR points
    for px, py, dist in self.latest_projected_pts:
      ix, iy = int(px), int(py)
      if 0 <= ix < w and 0 <= iy < h:
        # Distance normalized between 0.5m (close/red) and 5.0m (far/blue)
        norm_dist = np.clip((dist - 0.5) / 4.5, 0.0, 1.0)
        depth_u8 = np.uint8(norm_dist * 255)

        bgr_pixel = cv2.applyColorMap(
            np.array([[255 - depth_u8]], dtype=np.uint8), cv2.COLORMAP_JET
        )[0, 0]
        color = (int(bgr_pixel[0]), int(bgr_pixel[1]), int(bgr_pixel[2]))

        cv2.circle(img, (ix, iy), 3, color, -1)

    # Status text overlay
    status_str = 'TF: Synced' if self.tf_resolved else 'TF: Baseline Fallback'
    status_col = (0, 255, 0) if self.tf_resolved else (0, 165, 255)
    cv2.putText(
        img,
        status_str,
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        status_col,
        1,
        cv2.LINE_AA,
    )

    cv2.imshow('2D LiDAR Projection', img)
    cv2.waitKey(1)


def main(args=None):
  rclpy.init(args=args)
  node = LidarCameraProjector()
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