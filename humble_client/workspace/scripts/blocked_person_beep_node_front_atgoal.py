#!/usr/bin/env python3
"""
blocked_person_beep_node_front_atgoal.py - HARDWARE, 9 Oct 2026. Copy of blocked_person_beep_node_front.py
(unchanged there) with ONE change: never "blocked" while the robot is already at its goal position
(within AT_GOAL_M of the newest goal on goal_pose). There it only turns to the final heading, so nobody in
front blocks its way; but a turn on the spot shows no headway, and on 9 Oct the node took Brian sitting at
his table 1 m in front of the start for a blocking person during the return to the start: it beeped and
held the robot, which then could not finish the turn.

blocked_person_beep_node_front.py - HARDWARE, 8 Oct 2026. Copy of
blocked_person_beep_node.py with TWO changes to the person test:
  1. it counts only a person IN FRONT of the robot - within PERSON_BLOCK_M =
     1.0 m (was 1.5 m) AND within +/-FRONT_HALF_DEG (30 deg, the cone of the
     LiDAR test) of the robot's heading. Reason: in the hallway on 8 Oct a
     static person beside or behind the robot, not on its way, was counted as
     blocking (the original test is distance only, any direction), so the
     robot beeped and was held.
  2. PERSON_MEMORY_S 5 -> 1 s, so the hold releases about 1 s after the person
     has left the cone (with 5 s it switched on after the walker had passed,
     bag headon_v4front_run02_20261008_111240).
The original node, which matches the simulation, is unchanged.

Original header:
blocked_person_beep_node.py - HARDWARE (Humble, real TurtleBot 4), 7 Oct 2026.

Port of blocked_person_beep_node_sim.py (simulation final configuration).
HARDWARE CHANGES vs simulation:
  - /turtlebot4 namespace for the robot topics (person topic stays /person_positions_fused)
  - LiDAR frame from the scan header (it is in the robot's TF), as in beep_retry_node.py
  - hold: the Humble collision monitor (1.1.20) cannot take a polygon from a
    topic (no polygon_sub_topic). The config therefore has a FIXED BlockedHold
    stop square (+/-1.6 m around base_link, the square the simulation node
    publishes) with enabled: false, and this node switches it with the
    collision monitor's parameter BlockedHold.enabled. The switch is sent on
    every change and repeated every HOLD_RESEND_S; the node turns it off at
    start and at shutdown. Needs launch/lab2_bringup_cm.launch.py and
    config/social_nav2_headon_F_hwreq_block_approach_hw.yaml.
  If this node dies while holding, the robot stays stopped; release it with
    ros2 param set /turtlebot4/collision_monitor BlockedHold.enabled false

Original description (simulation):

Beeps when the robot has a goal, is making (almost) no headway, and a person
is in front of it.

It does not cancel or resend anything: Nav2 keeps the goal and drives on by
itself when the person has moved. It beeps within a few seconds instead of
after the goal has ABORTED, which is when beep_retry_node.py acts. Run
both: this one for the prompt beep, that one to resend the goal if Nav2 gives
up before the person moves.

hold:=true (needs the collision monitor's "BlockedHold" zone, see above): once blocked, the
robot is also held at zero velocity until the person test (3) has been false
for its memory time. Without it the robot stays in place but keeps turning
and creeping while Nav2 looks for a way round.

Beep when ALL of these hold:
  1. a NavigateToPose goal is executing
  2. the robot's position has stayed within STILL_DIST_M over the last STILL_S
  3. "person in front":
       a) (FRONT VARIANT) a person from /person_positions_fused within
          PERSON_BLOCK_M of the robot AND within +/-FRONT_HALF_DEG of its
          heading, in the last PERSON_MEMORY_S, or
       b) a LiDAR point NOT on the static map within LIDAR_BLOCK_M,
          +/- FRONT_HALF_DEG ahead, in the last LIDAR_MEMORY_S
The beep repeats every BEEP_PERIOD_S while they hold.

Publishes:
  /cmd_audio             irobot_create_msgs/AudioNoteVector
  /blocked_person_event  std_msgs/String  "blocked,<sim t>,<why>" /
                                          "clear,<sim t>,<blocked for s>"
  (hold:=true) sets /turtlebot4/collision_monitor BlockedHold.enabled

Run:
  python3 blocked_person_beep_node.py --ros-args -p hold:=true \
    -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
"""
import collections
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray, GoalStatus
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration as DurationMsg
import tf2_ros

NS = "/turtlebot4"
NAV_STATUS_TOPIC = f"{NS}/navigate_to_pose/_action/status"
ODOM_TOPIC = f"{NS}/odom"
AUDIO_TOPIC = f"{NS}/cmd_audio"
SCAN_TOPIC = f"{NS}/scan"
MAP_TOPIC = f"{NS}/map"
PERSON_TOPIC = "/person_positions_fused"
EVENT_TOPIC = "/blocked_person_event"
GOAL_TOPIC = f"{NS}/goal_pose"   # ATGOAL: nav_goal_wait.py and RViz "Nav2 Goal" publish the goal here
AT_GOAL_M = 0.40   # ATGOAL: robot this close to the goal position = only turning (Nav2 xy_goal_tolerance 0.25)
# HARDWARE: the hold zone is fixed in the Nav2 config (BlockedHold, +/-1.6 m,
# the simulation's HOLD_HALF_M); this node only switches it on and off.
HOLD_SERVICE = f"{NS}/collision_monitor/set_parameters"
HOLD_PARAM = "BlockedHold.enabled"
HOLD_RESEND_S = 2.0

# "Not moving" is judged on headway, not on standing dead still. In front of
# two people closing the corridor the robot stood for ~4 s and then crept
# sideways at about 0.1 m/s looking for a way round (pilot
# headon_blockedbeep_pair_pilot1); with a 0.05 m / 2 s test the beeping
# stopped there although it was still blocked. 0.5 m in 4 s is a mean of
# 0.125 m/s: below the 0.15 m/s occlusion slow-down and the 0.31 m/s cruise.
STILL_S = 4.0                 # s window
STILL_DIST_M = 0.5            # m; position spread over STILL_S below this = no headway
BEEP_PERIOD_S = 3.0
# Person / unmapped-LiDAR tests: values of beep_retry_node.py, except
# PERSON_BLOCK_M (FRONT VARIANT: 1.5 -> 1.0 m, and only within FRONT_HALF_DEG).
PERSON_BLOCK_M = 1.0
PERSON_MEMORY_S = 1.0           # FRONT VARIANT (8 Oct): 5.0 -> 1.0 s. With 5 s the hold switched on 2.6 s AFTER
                                # the walker had left the front cone and held the robot 2 s with nobody
                                # in front (bag headon_v4front_run02_20261008_111240, t 24.5-29.1 s).
LIDAR_BLOCK_M = 1.0
LIDAR_MEMORY_S = 2.0
FRONT_HALF_DEG = 30.0
MAP_MARGIN_CELLS = 3

MAP_FRAME = "map"
BASE_FRAME = "base_link"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BlockedPersonBeepNode(Node):
    def __init__(self):
        super().__init__("blocked_person_beep_node")   # same name: run only one of the two
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(GoalStatusArray, NAV_STATUS_TOPIC, self.on_status, latched)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, sensor)
        self.create_subscription(String, PERSON_TOPIC, self.on_person, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, sensor)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, latched)
        self.create_subscription(PoseStamped, GOAL_TOPIC, self.on_goal, 10)
        self.goal_xy = None
        self.audio_pub = self.create_publisher(AudioNoteVector, AUDIO_TOPIC, 10)
        self.event_pub = self.create_publisher(String, EVENT_TOPIC, 10)
        self.declare_parameter("hold", False)
        self.hold = bool(self.get_parameter("hold").value)
        self.hold_cli = self.create_client(SetParameters, HOLD_SERVICE) if self.hold else None
        self.hold_state = None        # last value sent
        self.hold_sent_at = -1e9
        self.hold_warned = None     # hold value last warned about

        self.goal_active = False
        self.odom = collections.deque()   # (sim t, x, y) over the last STILL_S
        self.last_person_block = -1e9
        self.last_lidar_block = -1e9
        self.grid = None
        self.mount_yaw = None
        self.blocked_since = None
        self.last_beep = None
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f"blocked_person_beep_node_front: still = < {STILL_DIST_M} m in {STILL_S:.0f} s, "
            f"person {PERSON_BLOCK_M} m +/-{FRONT_HALF_DEG:.0f} deg ahead / {PERSON_MEMORY_S:.0f} s, "
            f"lidar unmapped {LIDAR_BLOCK_M} m +/-{FRONT_HALF_DEG:.0f} deg / {LIDAR_MEMORY_S:.0f} s, "
            f"beep every {BEEP_PERIOD_S:.0f} s, hold {'on' if self.hold else 'off'}, "
            f"never blocked within {AT_GOAL_M} m of the goal on {GOAL_TOPIC} [atgoal]")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ inputs
    def on_status(self, msg):
        if not msg.status_list:
            return
        newest = max(msg.status_list,
                     key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        self.goal_active = newest.status in (GoalStatus.STATUS_ACCEPTED,
                                             GoalStatus.STATUS_EXECUTING)

    def on_goal(self, msg):
        p = msg.pose.position
        self.goal_xy = (p.x, p.y)

    def at_goal(self):
        # ATGOAL: at the goal position the robot only turns; nobody in front blocks its way.
        if self.goal_xy is None:
            return False
        rp = self.robot_pose()
        return rp is not None and math.hypot(rp[0] - self.goal_xy[0], rp[1] - self.goal_xy[1]) < AT_GOAL_M

    def on_odom(self, msg):
        now = self.now_s()
        p = msg.pose.pose.position
        self.odom.append((now, p.x, p.y))
        while self.odom and now - self.odom[0][0] > STILL_S + 0.5:
            self.odom.popleft()

    def on_map(self, msg):
        self.grid = msg

    def robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def on_person(self, msg):
        f = msg.data.split(",")
        try:
            px, py = float(f[2]), float(f[3])
        except (ValueError, IndexError):
            return
        rp = self.robot_pose()
        if rp is None:
            return
        dx, dy = px - rp[0], py - rp[1]
        # FRONT VARIANT: close AND ahead (bearing from the robot's heading).
        bearing = (math.degrees(math.atan2(dy, dx) - rp[2]) + 180.0) % 360.0 - 180.0
        if math.hypot(dx, dy) <= PERSON_BLOCK_M and abs(bearing) <= FRONT_HALF_DEG:
            self.last_person_block = self.now_s()

    def on_map_cell_occupied(self, x, y):
        g = self.grid
        res = g.info.resolution
        cx = int((x - g.info.origin.position.x) / res)
        cy = int((y - g.info.origin.position.y) / res)
        W, H = g.info.width, g.info.height
        for dy in range(-MAP_MARGIN_CELLS, MAP_MARGIN_CELLS + 1):
            for dx in range(-MAP_MARGIN_CELLS, MAP_MARGIN_CELLS + 1):
                X, Y = cx + dx, cy + dy
                if 0 <= X < W and 0 <= Y < H and g.data[Y * W + X] >= 50:
                    return True
        return False

    def on_scan(self, msg):
        if self.grid is None:
            return
        if self.mount_yaw is None:
            try:
                tr = self.tf_buffer.lookup_transform(BASE_FRAME, msg.header.frame_id, rclpy.time.Time())
                self.mount_yaw = yaw_of(tr.transform.rotation)
            except Exception:
                return
        try:
            tr = self.tf_buffer.lookup_transform(MAP_FRAME, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        lx, ly = tr.transform.translation.x, tr.transform.translation.y
        lyaw = yaw_of(tr.transform.rotation)
        a = msg.angle_min
        for r in msg.ranges:
            if 0.15 < r <= LIDAR_BLOCK_M and not math.isinf(r) and not math.isnan(r):
                bearing = (math.degrees(a + self.mount_yaw) + 180.0) % 360.0 - 180.0
                if abs(bearing) <= FRONT_HALF_DEG:
                    x = lx + r * math.cos(lyaw + a)
                    y = ly + r * math.sin(lyaw + a)
                    if not self.on_map_cell_occupied(x, y):
                        self.last_lidar_block = self.now_s()
                        return
            a += msg.angle_increment

    # ------------------------------------------------------------ decision
    def is_still(self, now):
        if not self.odom or now - self.odom[0][0] < STILL_S:
            return False              # not enough history yet
        if now - self.odom[-1][0] > 1.0:
            return False              # odom is stale: unknown, do not beep
        xs = [o[1] for o in self.odom if now - o[0] <= STILL_S]
        ys = [o[2] for o in self.odom if now - o[0] <= STILL_S]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < STILL_DIST_M

    def tick(self):
        now = self.now_s()
        person = now - self.last_person_block <= PERSON_MEMORY_S
        lidar = now - self.last_lidar_block <= LIDAR_MEMORY_S
        if self.hold and self.blocked_since is not None:
            # Held: the robot cannot move, so headway says nothing. Stay
            # blocked for as long as a person is still in front.
            blocked = self.goal_active and (person or lidar)
        else:
            blocked = self.goal_active and self.is_still(now) and (person or lidar)
        if blocked and self.at_goal():
            blocked = False   # ATGOAL: turning on the spot at the goal, not blocked
        if self.hold:
            self.publish_hold(blocked)
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = now
                self.last_beep = None
                why = "+".join(w for w, on in (("person", person), ("unmapped lidar", lidar)) if on)
                self.event_pub.publish(String(data=f"blocked,{now:.2f},{why}"))
                self.get_logger().info(f"Blocked by a person ({why}) - beeping.")
            if self.last_beep is None or now - self.last_beep >= BEEP_PERIOD_S:
                self.beep()
                self.last_beep = now
        elif self.blocked_since is not None:
            dur = now - self.blocked_since
            self.event_pub.publish(String(data=f"clear,{now:.2f},{dur:.2f}"))
            self.get_logger().info(f"No longer blocked after {dur:.1f} s.")
            self.blocked_since = None

    def publish_hold(self, on, force=False):
        # HARDWARE: switch the fixed BlockedHold zone of the collision monitor.
        # Sent when the state changes, and repeated every HOLD_RESEND_S in case
        # a call was lost or the collision monitor restarted.
        now = self.now_s()
        if not force and on == self.hold_state and now - self.hold_sent_at < HOLD_RESEND_S:
            return
        if not self.hold_cli.service_is_ready():
            if self.hold_warned != on:
                self.get_logger().warn(f"{HOLD_SERVICE} not available - hold {'on' if on else 'off'} not sent "
                                       "(is the collision monitor running? lab2_bringup_cm.launch.py)")
                self.hold_warned = on
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=HOLD_PARAM, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(on)))]
        self.hold_cli.call_async(req).add_done_callback(lambda f, on=on: self.on_hold_reply(f, on))
        if on != self.hold_state:
            self.get_logger().info(f"Hold {'ON' if on else 'off'} ({HOLD_PARAM} = {str(bool(on)).lower()}).")
        self.hold_state = on
        self.hold_sent_at = now

    def on_hold_reply(self, fut, on):
        # The collision monitor refuses the switch until it is configured
        # (BlockedHold.enabled not declared yet): forget the state so that the
        # next tick sends it again.
        ok = fut.result() is not None and all(r.successful for r in fut.result().results)
        if ok:
            self.hold_warned = None
            return
        if self.hold_state == on:
            self.hold_state = None
        if self.hold_warned != on:
            why = fut.result().results[0].reason if fut.result() and fut.result().results else "no reply"
            self.get_logger().warn(f"Hold {'on' if on else 'off'} refused by the collision monitor ({why}); retrying.")
            self.hold_warned = on

    def beep(self):
        # Same triple beep as beep_retry_node.py's "blocked" signal.
        msg = AudioNoteVector()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.append = False
        for freq, dur in ((880, 0.2), (1100, 0.2), (880, 0.2)):
            n = AudioNote()
            n.frequency = freq
            n.max_runtime = DurationMsg(sec=0, nanosec=int(dur * 1e9))
            msg.notes.append(n)
        self.audio_pub.publish(msg)


def main():
    # HARDWARE: own signal handling, so that ROS is still up in the finally
    # block below and the hold can be released on Ctrl-C and on SIGTERM.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BlockedPersonBeepNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # Teardown (SIGTERM) can invalidate the context mid-spin.
        if rclpy.ok():
            raise
    finally:
        if node.hold and rclpy.ok() and node.hold_cli.service_is_ready():
            # Stop the decision timer first: it would switch the hold back on
            # while the release below is being spun.
            node.destroy_timer(node.timer)
            fut = node.hold_cli.call_async(SetParameters.Request(parameters=[Parameter(
                name=HOLD_PARAM, value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))]))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            node.get_logger().info("Hold released at shutdown." if fut.done() else
                                   "Hold release at shutdown NOT confirmed - check BlockedHold.enabled.")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
