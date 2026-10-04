#!/usr/bin/env python3
"""
social_zone_speed_limiter.py

Caps the robot's speed while it is inside a conversation zone that the
corridor forced it to enter.

WHY THIS EXISTS
  social_group_detector_node.py's Option 1 shrinks ZONE_BUFFER when the
  local costmap says there is not enough lateral clearance to route
  around a group. Once shrunk, the o-space published by
  social_group_cloud_node.py is narrow enough that Nav2 will plan
  THROUGH it rather than fail. That is the intended outcome - but at
  full speed it is the socially worst outcome. This node makes that
  forced pass a slow one.

HOW "FORCED" IS DECIDED
  Purely from the effective buffer the detector reports in field 10 of
  /social_groups. If buffer < ZONE_BUFFER - EPS, the detector already
  measured insufficient clearance via its costmap probe. No second
  threshold is introduced here - the probe is the single source of
  truth, matching the design decision made for this pipeline.

WHAT IT PUBLISHES
  nav2_msgs/SpeedLimit on /turtlebot4/speed_limit, percentage-based.
  SLOW_PERCENTAGE while inside a shrunk zone, 100.0 once clear.
  Messages are sent only on transition plus a slow heartbeat, not every
  cycle, so the controller is not spammed.

REQUIRED NAV2 PARAM
  controller_server must be subscribing to this topic. In Humble the
  parameter is speed_limit_topic (default "speed_limit"). Under the
  /turtlebot4 namespace that resolves to /turtlebot4/speed_limit, which
  is this node's default output. Verify before trusting the behaviour:
      ros2 param get /turtlebot4/controller_server speed_limit_topic

TF
  Needs the robot pose in the zone frame (map). Namespaced TF must be
  remapped at launch, same as social_group_cloud_node.py:
      -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static

UNTESTED against real hardware. Test in open space first (expect no
speed limit ever applied), narrow corridor second.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros

from std_msgs.msg import String
from nav2_msgs.msg import SpeedLimit


# MUST match ZONE_BUFFER in social_group_detector_node.py. Only used to
# decide whether the reported buffer was shrunk, never for geometry.
NOMINAL_ZONE_BUFFER = 0.4
SHRINK_EPS = 0.01

# Speed cap applied while inside a shrunk zone, as a percentage of the
# controller's configured max velocity.
SLOW_PERCENTAGE = 30.0

# Robot counts as "inside" when within this much of the zone's footprint.
# Padded outward so the robot is already slow on entry, not braking
# hard once it is between the two people.
ENTRY_PAD = 0.5

# Drop a zone this long after its last /social_groups message. Matches
# GROUP_TIMEOUT in social_group_cloud_node.py.
GROUP_TIMEOUT = 4.0

CHECK_RATE_HZ = 10.0
HEARTBEAT_S = 1.0

ROBOT_FRAME = "base_link"


class SocialZoneSpeedLimiter(Node):
    def __init__(self):
        super().__init__("social_zone_speed_limiter")

        self.declare_parameter("input_topic", "/social_groups")
        self.declare_parameter("output_topic", "/turtlebot4/speed_limit")
        self.declare_parameter("frame_id", "map")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        # group_id -> dict(cx, cy, ax, ay, half_length, half_width,
        #                  buffer, last_seen)
        self.zones = {}
        self.limited = False
        self.last_sent = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, self.input_topic, self.group_callback, 10)
        self.pub = self.create_publisher(SpeedLimit, self.output_topic, 10)

        self.create_timer(1.0 / CHECK_RATE_HZ, self.check)

        self.get_logger().info("Social zone speed limiter started")
        self.get_logger().info(f"Input : {self.input_topic}")
        self.get_logger().info(f"Output: {self.output_topic}")
        self.get_logger().info(
            f"Slow to {SLOW_PERCENTAGE:.0f}% inside zones whose buffer is "
            f"below {NOMINAL_ZONE_BUFFER:.2f} m (entry pad {ENTRY_PAD:.2f} m)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -----------------------------------------------------------------
    def group_callback(self, msg):
        parts = msg.data.split(",")
        if len(parts) < 11:
            # Field 10 (effective buffer) is what this node exists to
            # read. Without it there is no way to tell a shrunk zone
            # from a nominal one, so refuse rather than guess.
            self.get_logger().warn(
                "Ignoring /social_groups msg without buffer field - is "
                "social_group_detector_node.py the patched version?",
                throttle_duration_sec=10.0)
            return

        try:
            group_id = parts[0].strip()
            group_type = parts[1].strip()
            cx, cy = float(parts[2]), float(parts[3])
            axis_x, axis_y = float(parts[4]), float(parts[5])
            half_length = float(parts[6])
            half_width = float(parts[7])
            buffer = float(parts[10])
        except ValueError:
            return

        if group_type not in ("conversation", "side_by_side"):
            return

        norm = math.hypot(axis_x, axis_y)
        if norm < 1e-6:
            return

        self.zones[group_id] = {
            "cx": cx, "cy": cy,
            "ax": axis_x / norm, "ay": axis_y / norm,
            "half_length": half_length,
            "half_width": half_width,
            "buffer": buffer,
            "last_seen": self.now(),
        }

    # -----------------------------------------------------------------
    def _inside(self, rx, ry, z):
        """Robot inside the zone's padded rectangle, in zone-local axes."""
        dx, dy = rx - z["cx"], ry - z["cy"]
        u = dx * z["ax"] + dy * z["ay"]          # along the pair's axis
        v = -dx * z["ay"] + dy * z["ax"]         # across it
        return (abs(u) <= z["half_length"] + ENTRY_PAD
                and abs(v) <= z["half_width"] + ENTRY_PAD)

    def check(self):
        now = self.now()

        for gid in [g for g, z in self.zones.items()
                    if now - z["last_seen"] > GROUP_TIMEOUT]:
            del self.zones[gid]

        try:
            tfm = self.tf_buffer.lookup_transform(
                self.frame_id, ROBOT_FRAME,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            # No pose means no basis to restrict. Release any active
            # limit rather than leaving the robot crawling on stale TF.
            if self.limited:
                self._send(100.0, "TF unavailable - releasing limit")
            return

        rx = tfm.transform.translation.x
        ry = tfm.transform.translation.y

        hit = None
        for gid, z in self.zones.items():
            if z["buffer"] >= NOMINAL_ZONE_BUFFER - SHRINK_EPS:
                continue  # zone is full size - robot can route around it
            if self._inside(rx, ry, z):
                hit = (gid, z)
                break

        if hit is not None:
            gid, z = hit
            if not self.limited or now - self.last_sent > HEARTBEAT_S:
                self._send(
                    SLOW_PERCENTAGE,
                    f"inside shrunk zone {gid} "
                    f"(buffer {z['buffer']:.2f} m) - slowing to "
                    f"{SLOW_PERCENTAGE:.0f}%")
        elif self.limited:
            self._send(100.0, "clear of all shrunk zones - speed restored")

    def _send(self, percentage, reason):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.percentage = True
        msg.speed_limit = float(percentage)
        self.pub.publish(msg)
        self.limited = percentage < 100.0
        self.last_sent = self.now()
        self.get_logger().info(f"[speed] {reason}")

    def destroy_node(self):
        if self.limited:
            self._send(100.0, "shutdown - restoring full speed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SocialZoneSpeedLimiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
