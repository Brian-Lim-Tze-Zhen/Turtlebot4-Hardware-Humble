#!/usr/bin/env python3
"""
odom_qos_relay.py

TurtleBot4 publishes /turtlebot4/odom as BEST_EFFORT. Nav2's
controller_server and bt_navigator subscribe RELIABLE, and that pair is
incompatible, so those two nodes receive NO odometry at all - confirmed
with `ros2 topic info /turtlebot4/odom --verbose` (1 BEST_EFFORT
publisher, 2 RELIABLE subscribers, zero messages delivered).

Consequence: MPPI's optimizer has no current velocity and rolls out
trajectories from a stale/zero initial state; bt_navigator's stuck and
distance-travelled checks never fire.

Nav2 hardcodes the subscription QoS, so it cannot be relaxed by
parameter. This node bridges instead: subscribe BEST_EFFORT, republish
RELIABLE on /turtlebot4/odom_reliable. Point Nav2's odom_topic at the
new name.

Pure passthrough - the message is republished unmodified, so odometry
content and stamps are unchanged.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry


class OdomQoSRelay(Node):
    def __init__(self):
        super().__init__("odom_qos_relay")

        self.declare_parameter("input_topic", "/turtlebot4/odom")
        self.declare_parameter("output_topic", "/turtlebot4/odom_reliable")

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value

        # Match the publisher exactly, or we get the same mismatch again.
        qos_in = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # What Nav2 expects.
        qos_out = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(Odometry, self.out_topic, qos_out)
        self.create_subscription(Odometry, self.in_topic, self.cb, qos_in)

        self.count = 0
        self.create_timer(5.0, self.heartbeat)

        self.get_logger().info(f"odom QoS relay: {self.in_topic} "
                               f"(BEST_EFFORT) -> {self.out_topic} (RELIABLE)")

    def cb(self, msg):
        self.count += 1
        self.pub.publish(msg)

    def heartbeat(self):
        if self.count == 0:
            self.get_logger().warn(
                f"no messages received on {self.in_topic} - check the topic "
                f"name and that the robot is publishing")
        else:
            self.get_logger().info(f"relayed {self.count} messages")


def main(args=None):
    rclpy.init(args=args)
    node = OdomQoSRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
