#!/usr/bin/env python3
"""
social_group_cloud_node.py (HARDWARE VERSION)

Adapted from the simulation version for the TurtleBot4 hardware
pipeline built this session. Two changes from the simulation version:

  1. group_callback now accepts group_type "side_by_side" as well as
     "conversation" - social_group_detector_node.py (the hardware
     detector) publishes "conversation" only for facing pairs and
     "side_by_side" for the other classified pair type; it never
     publishes "queue". The simulation version only recognized
     "conversation"/"queue", so side_by_side pairs were silently
     dropped and never reached the costmap. Both types get identical
     o-space (gap-filling) treatment.

  2. output_topic's default is now the NAMESPACED costmap topic
     (/turtlebot4/social_group_cloud), since on hardware the local
     costmap plugin runs inside the /turtlebot4 namespace. Every other
     custom pipeline topic (input_topic, LIDAR_TOPIC,
     QUEUE_POSITIONS_TOPIC) stays UNNAMESPACED, matching the rest of
     this session's pipeline (social_group_detector_node.py,
     identity_fusion_node_lidar.py, human_kf_predictor_lidar.py, etc.
     all publish/subscribe their own custom topics unnamespaced and
     only remap actual robot-native topics).

  TF is still standard tf2 (/tf, /tf_static) - these are not exposed
  as node parameters by tf2_ros.TransformListener, so they still need
  a launch-time remap, same as every other node in this pipeline that
  touches TF on hardware:

    python3 social_group_cloud_node.py --ros-args \\
      -r /tf:=/turtlebot4/tf \\
      -r /tf_static:=/turtlebot4/tf_static

  output_topic and input_topic are declared ROS parameters, so they
  can still be overridden with -p if your setup differs from the
  defaults below.

Everything else (o-space geometry, lidar hold-open, queue gap-filling,
robot keep-out) is UNCHANGED from the simulation version - see that
file's docstring (reproduced below) for the full design rationale.

-----------------------------------------------------------------------
WHAT THIS DOES
-----------------------------------------------------------------------
Converts each conversation/side-by-side group into a synthetic
PointCloud2 obstacle region that Nav2's costmap layer consumes, so the
planner routes around the group instead of through it.

-----------------------------------------------------------------------
WHAT REGION IS FILLED, AND WHY
-----------------------------------------------------------------------
This fills the O-SPACE only - the shared space BETWEEN the two people -
not their bodies. Walking through it is the socially disruptive act;
passing behind one member is not. The people's own bodies are NOT
covered here on purpose - predicted_person_cloud_node.py already
places a disk on each person.

-----------------------------------------------------------------------
SIZING, AND THE PROXEMIC BASIS
-----------------------------------------------------------------------
Hall's proxemic zones: intimate <0.45m, personal 0.45-1.2m, social
1.2-3.6m. CONV_MAX_DIST=1.8m covers all of personal plus the lower
part of social. Below MIN_O_SPACE_HALF_LENGTH the gap is too small to
be worth blocking and nothing is published for that group.

-----------------------------------------------------------------------
COUPLING - READ BEFORE CHANGING CONSTANTS
-----------------------------------------------------------------------
/social_groups publishes half_length = separation/2 + ZONE_BUFFER,
where ZONE_BUFFER is defined in social_group_detector_node.py
(currently 0.4). This node subtracts that buffer back out to recover
the raw separation, so SOURCE_ZONE_BUFFER below MUST match ZONE_BUFFER
there. If they drift apart, every zone silently becomes the wrong
length - there is no error, just wrong geometry. Change them together.

-----------------------------------------------------------------------
QUEUE HANDLING
-----------------------------------------------------------------------
social_group_detector_node.py (hardware) does NOT implement queue
detection - it only ever publishes "conversation" or "side_by_side".
The queue-handling code below is inert on this pipeline (never
triggered) but left in unchanged in case queue detection is ported
over later; it is harmless as dead code.

-----------------------------------------------------------------------
INPUT / OUTPUT
-----------------------------------------------------------------------
Subscribes:
  /social_groups (String) - default, unnamespaced, matches
      social_group_detector_node.py's own default output_topic
      group_id,group_type,cx,cy,axis_x,axis_y,half_length,half_width,
      member_ids,member_xy
  /lidar_person_clusters (String) - default, unnamespaced, matches
      lidar_person_detector.py's own default output topic
  /predicted_person_positions (String) - default, unnamespaced,
      matches human_kf_predictor_lidar.py's own default output topic -
      queue member position lookup only

Publishes:
  /turtlebot4/social_group_cloud (PointCloud2, frame "map") - default,
      NAMESPACED to match the local costmap running under /turtlebot4

Costmap plugin: use NonPersistentVoxelLayer, NOT VoxelLayer. VoxelLayer
clears via raytracing, which fails for synthetic clouds that have no
real sensor origin, leaving ghost obstacles behind forever.
NonPersistentVoxelLayer rebuilds from scratch each cycle.
"""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros

from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField


# =======================================================================
# Geometry
# =======================================================================

# MUST match ZONE_BUFFER in social_group_detector_node.py - see COUPLING.
SOURCE_ZONE_BUFFER = 0.4

# Backed off from each person so the zone covers the GAP, not the bodies.
# Roughly a body radius; predicted_person_cloud_node already marks the
# people themselves with a 0.40m disk.
BODY_CLEARANCE = 0.25

# Half-extent across the pair's axis. Narrower than the source message's
# half_width (which is just ZONE_BUFFER) because Nav2's inflation layer
# expands whatever is published - this is the raw obstacle, not the
# final keep-out distance the robot will actually respect.
O_SPACE_HALF_WIDTH = 0.35

# Below this the pair are effectively shoulder to shoulder and there is
# no meaningful gap to protect. Publish nothing rather than a sliver.
MIN_O_SPACE_HALF_LENGTH = 0.10

POINT_SPACING = 0.10   # m between synthetic points
POINT_HEIGHT = 0.30    # m; matches predicted_person_cloud_node


# =======================================================================
# Queue gap-filling (inert on hardware - see QUEUE HANDLING above)
# =======================================================================
QUEUE_POSITIONS_TOPIC = "/predicted_person_positions"

# How stale a cached member position may be before it's refused for gap
# geometry. Matches FRESH_POSITION_TIMEOUT in social_group_detector_node.py.
QUEUE_POSITION_MAX_AGE = 1.0   # s


# =======================================================================
# Timing / safety
# =======================================================================

PUBLISH_RATE_HZ = 10.0

# =======================================================================
# Lidar-anchored persistence (CONVERSATION + SIDE_BY_SIDE path only)
# =======================================================================
# PROBLEM: the zone vanished exactly when the robot got close, which is
# when it matters most. social_group_detector_node.py only emits a
# group while both members have fresh camera positions, which needs
# fresh camera detection - the OAK-D preview is narrow, so approaching
# or rotating can drop one of them, /social_groups goes quiet, and
# GROUP_TIMEOUT would clear the zone.
#
# FIX: the lidar does not share the camera's occlusion geometry. While
# the camera is blind, hold the zone open as long as lidar tracks still
# sit near where the members were, and MOVE the zone to follow those
# tracks so it stays correct if the pair drifts.
#
# /social_groups carries stable CAMERA-fusion ids while
# /lidar_person_clusters carries lidar track ids - different numbering,
# so members are matched to anchors BY POSITION instead.
LIDAR_TOPIC = "/lidar_person_clusters"

# Max distance from a member's last known position to a lidar track for
# that track to count as its anchor.
LIDAR_ANCHOR_GATE = 0.35

# Drop a cached lidar track this long after its last message.
LIDAR_TRACK_TIMEOUT = 1.0

# How long a group may be held open on lidar anchors alone, with no
# camera confirmation at all.
MAX_LIDAR_ONLY_HOLD = 60.0

# Recomputed separation limit while running on lidar anchors. MUST match
# CONV_MAX_DIST in social_group_detector_node.py.
CONV_MAX_DIST = 1.8

# Drop a group this long after its last message. Sized against the
# MEASURED publish period of /social_groups, not the nominal timer -
# see the simulation version's history for why a too-tight value here
# causes repeated false costmap dropouts.
GROUP_TIMEOUT = 4.0

# Robot keep-out. Without this, a zone can place lethal cost under the
# robot's own footprint.
ROBOT_FRAME = "base_link"
ROBOT_KEEPOUT_RADIUS = 0.25


class SocialGroupCloudNode(Node):
    def __init__(self):
        super().__init__("social_group_cloud_node")

        self.declare_parameter("input_topic", "/social_groups")
        self.declare_parameter("output_topic", "/turtlebot4/social_group_cloud")
        self.declare_parameter("frame_id", "map")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        # group_id -> dict(...); shape differs by group type, see
        # group_callback.
        self.active_groups = {}
        self.lidar_tracks = {}      # lidar_id -> (x, y, last_seen)
        self.person_positions = {}  # track_id -> (x, y, last_seen); queue lookup only
        self.last_robot_xy = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            String, self.input_topic, self.group_callback, 10)

        self.create_subscription(
            String, LIDAR_TOPIC, self.lidar_callback, 10)

        self.create_subscription(
            String, QUEUE_POSITIONS_TOPIC, self.position_callback, 10)

        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, 10)

        # Publish on a timer rather than per-message, so all active groups
        # appear together in one cloud.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_cloud)

        self.get_logger().info("Social group cloud node started (hardware)")
        self.get_logger().info(f"Input : {self.input_topic}")
        self.get_logger().info(f"Output: {self.output_topic}")
        self.get_logger().info(f"Frame : {self.frame_id}")
        self.get_logger().info(
            f"O-space width: {O_SPACE_HALF_WIDTH * 2:.2f} m | "
            f"body clearance: {BODY_CLEARANCE:.2f} m")
        self.get_logger().info(
            f"Robot keep-out: {ROBOT_KEEPOUT_RADIUS:.2f} m "
            f"around '{ROBOT_FRAME}' (remap /tf, /tf_static if namespaced)")
        self.get_logger().info(
            "Group types handled: conversation, side_by_side "
            "(queue path present but inert on this pipeline)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -----------------------------------------------------------------
    def lidar_callback(self, msg):
        parts = msg.data.split(",")
        if len(parts) < 3:
            return
        try:
            lid = int(float(parts[0]))
            x = float(parts[1])
            y = float(parts[2])
        except ValueError:
            return
        self.lidar_tracks[lid] = (x, y, self.now())

    # -----------------------------------------------------------------
    def position_callback(self, msg):
        parts = msg.data.split(",")
        if len(parts) < 10:
            return
        try:
            track_id = int(float(parts[0]))
            x = float(parts[2])
            y = float(parts[3])
        except ValueError:
            return
        self.person_positions[track_id] = (x, y, self.now())

    def find_anchor(self, x, y, exclude=None):
        """Nearest lidar track to (x, y) within LIDAR_ANCHOR_GATE.
        `exclude` prevents both members of a pair matching the same
        track, which would otherwise collapse the zone to a point."""
        best_id, best_d = None, LIDAR_ANCHOR_GATE
        for lid, (lx, ly, _) in self.lidar_tracks.items():
            if lid == exclude:
                continue
            d = math.hypot(x - lx, y - ly)
            if d < best_d:
                best_id, best_d = lid, d
        return best_id

    # -----------------------------------------------------------------
    def group_callback(self, msg):
        parts = msg.data.split(",")
        if len(parts) < 9:
            self.get_logger().warn(f"Invalid /social_groups msg: {msg.data}")
            return

        try:
            group_id = parts[0].strip()
            group_type = parts[1].strip()
            cx = float(parts[2])
            cy = float(parts[3])
            axis_x = float(parts[4])
            axis_y = float(parts[5])
            half_length = float(parts[6])
        except ValueError:
            self.get_logger().warn(f"Parse failed: {msg.data}")
            return

        # THESIS FIX (hardware): social_group_detector_node.py publishes
        # "conversation" for facing pairs and "side_by_side" for the
        # other classified pair type - it never publishes "queue". Both
        # get identical o-space (gap-filling) treatment; the internal
        # dict still stores them as type "conversation" so all the
        # downstream lidar hold-open / geometry logic is shared as-is.
        if group_type in ("conversation", "side_by_side"):
            # Recover the two members' positions from centre + axis +
            # separation, so they can be matched to lidar anchors later.
            separation = 2.0 * (half_length - SOURCE_ZONE_BUFFER)
            norm = math.hypot(axis_x, axis_y)
            if norm < 1e-6:
                return
            ax, ay = axis_x / norm, axis_y / norm
            h = separation / 2.0

            existing = self.active_groups.get(group_id, {})

            self.active_groups[group_id] = {
                "type": "conversation",
                "source_type": group_type,
                "member_a": (cx - h * ax, cy - h * ay),
                "member_b": (cx + h * ax, cy + h * ay),
                # Anchor ids are LEARNED while the camera is fresh (see
                # publish_cloud) and reused directly once it goes blind, so a
                # lost member is not re-matched by proximity to whatever
                # happens to be nearby.
                "anchor_a": existing.get("anchor_a"),
                "anchor_b": existing.get("anchor_b"),
                "last_seen": self.now(),
                "camera_fresh": True,
            }

        elif group_type == "queue":
            # Inert on this pipeline (see QUEUE HANDLING) - left in
            # unchanged in case queue detection is added later.
            member_ids_str = parts[8].strip()
            try:
                member_ids = [int(v) for v in member_ids_str.split(";") if v]
            except ValueError:
                self.get_logger().warn(f"Bad member_ids in queue msg: {msg.data}")
                return

            if len(member_ids) < 2:
                return

            new_members = set(member_ids)
            matched_gid = None
            for gid, g in self.active_groups.items():
                if g.get("type") == "queue" and set(g["member_ids"]) & new_members:
                    matched_gid = gid
                    break

            target_gid = matched_gid if matched_gid is not None else group_id

            member_xy = None
            if len(parts) >= 10 and parts[9].strip():
                try:
                    member_xy = [tuple(float(v) for v in pair.split(";"))
                                 for pair in parts[9].strip().split("|")]
                except ValueError:
                    member_xy = None
                if member_xy is not None and len(member_xy) != len(member_ids):
                    self.get_logger().warn(
                        "member_xy length does not match member_ids - ignoring")
                    member_xy = None

            self.active_groups[target_gid] = {
                "type": "queue",
                "member_ids": member_ids,
                "member_xy": member_xy,
                "last_seen": self.now(),
            }

        else:
            # Unknown group_type - ignore rather than guess a shape for it.
            return

    # -----------------------------------------------------------------
    def o_space_half_length(self, source_half_length):
        """Recover the pair's raw separation from the published
        half_length, then back off a body radius at each end so the zone
        covers only the gap between them. See COUPLING in the docstring."""
        separation = 2.0 * (source_half_length - SOURCE_ZONE_BUFFER)
        return separation / 2.0 - BODY_CLEARANCE

    def make_o_space_points(self, cx, cy, axis_x, axis_y, half_length):
        """Fill an ellipse centred on the pair's midpoint, elongated along
        their shared axis."""
        points = []

        norm = math.hypot(axis_x, axis_y)
        if norm < 1e-6:
            return points
        ax, ay = axis_x / norm, axis_y / norm

        a = half_length
        b = O_SPACE_HALF_WIDTH

        steps_u = int(a / POINT_SPACING)
        steps_v = int(b / POINT_SPACING)

        for iu in range(-steps_u, steps_u + 1):
            u = iu * POINT_SPACING
            for iv in range(-steps_v, steps_v + 1):
                v = iv * POINT_SPACING

                if (u / a) ** 2 + (v / b) ** 2 <= 1.0:
                    # rotate local (u, v) onto the group's axis
                    x = cx + u * ax - v * ay
                    y = cy + u * ay + v * ax
                    points.append((x, y, POINT_HEIGHT))

        return points

    # -----------------------------------------------------------------
    def make_queue_gap_points(self, member_ids, member_xy=None):
        points = []
        now = self.now()

        if member_xy is not None:
            pairs = list(zip(member_xy, member_xy[1:]))
        else:
            pairs = None

        if pairs is not None:
            for (xa, ya), (xb, yb) in pairs:
                dx, dy = xb - xa, yb - ya
                separation = math.hypot(dx, dy)
                a = separation / 2.0 - BODY_CLEARANCE
                if a < MIN_O_SPACE_HALF_LENGTH:
                    continue
                points.extend(self.make_o_space_points(
                    (xa + xb) / 2.0, (ya + yb) / 2.0, dx, dy, a))
            return points

        for tid_a, tid_b in zip(member_ids, member_ids[1:]):
            cached_a = self.person_positions.get(tid_a)
            cached_b = self.person_positions.get(tid_b)
            if cached_a is None or cached_b is None:
                continue

            xa, ya, ta = cached_a
            xb, yb, tb = cached_b
            if (now - ta > QUEUE_POSITION_MAX_AGE
                    or now - tb > QUEUE_POSITION_MAX_AGE):
                continue

            dx, dy = xb - xa, yb - ya
            separation = math.hypot(dx, dy)

            a = separation / 2.0 - BODY_CLEARANCE
            if a < MIN_O_SPACE_HALF_LENGTH:
                continue

            gx = (xa + xb) / 2.0
            gy = (ya + yb) / 2.0
            points.extend(self.make_o_space_points(gx, gy, dx, dy, a))

        return points

    # -----------------------------------------------------------------
    def publish_cloud(self):
        now = self.now()

        for lid in [l for l, v in self.lidar_tracks.items()
                    if now - v[2] > LIDAR_TRACK_TIMEOUT]:
            del self.lidar_tracks[lid]

        for tid in [t for t, v in self.person_positions.items()
                    if now - v[2] > QUEUE_POSITION_MAX_AGE]:
            del self.person_positions[tid]

        points = []
        published = []
        drop = []

        for gid, g in self.active_groups.items():
            if g["type"] == "queue":
                if now - g["last_seen"] > GROUP_TIMEOUT:
                    drop.append((gid, "queue: no update within GROUP_TIMEOUT"))
                    continue

                gap_points = self.make_queue_gap_points(
                    g["member_ids"], g.get("member_xy"))
                points.extend(gap_points)
                if gap_points:
                    published.append(("queue", gid, len(g["member_ids"])))
                continue

            # --- conversation / side_by_side path ---
            camera_age = now - g["last_seen"]
            camera_fresh = camera_age <= GROUP_TIMEOUT

            if camera_fresh:
                ax_pos, bx_pos = g["member_a"], g["member_b"]

                a_id = self.find_anchor(*ax_pos)
                b_id = self.find_anchor(*bx_pos, exclude=a_id)
                if a_id is not None:
                    g["anchor_a"] = a_id
                if b_id is not None:
                    g["anchor_b"] = b_id
            else:
                if camera_age > MAX_LIDAR_ONLY_HOLD:
                    drop.append((gid, "lidar-only hold expired"))
                    continue

                lid_a = g.get("anchor_a")
                lid_b = g.get("anchor_b")

                if lid_a is None:
                    lid_a = self.find_anchor(*g["member_a"], exclude=lid_b)
                if lid_b is None:
                    lid_b = self.find_anchor(*g["member_b"], exclude=lid_a)

                if lid_a not in self.lidar_tracks:
                    drop.append((gid, f"anchor {lid_a} for member a gone"))
                    continue
                if lid_b not in self.lidar_tracks:
                    drop.append((gid, f"anchor {lid_b} for member b gone"))
                    continue
                if lid_a == lid_b:
                    drop.append((gid, "both members collapsed onto one anchor"))
                    continue

                ax_pos = self.lidar_tracks[lid_a][:2]
                bx_pos = self.lidar_tracks[lid_b][:2]

                if math.hypot(ax_pos[0] - bx_pos[0],
                              ax_pos[1] - bx_pos[1]) > CONV_MAX_DIST:
                    drop.append((gid, "members moved apart"))
                    continue

                if g["camera_fresh"]:
                    g["camera_fresh"] = False
                    self.get_logger().info(
                        f"Group {gid}: camera lost, holding zone on "
                        f"lidar anchors {lid_a}/{lid_b}")

                g["member_a"], g["member_b"] = ax_pos, bx_pos

            cx = (ax_pos[0] + bx_pos[0]) / 2.0
            cy = (ax_pos[1] + bx_pos[1]) / 2.0
            dx, dy = bx_pos[0] - ax_pos[0], bx_pos[1] - ax_pos[1]
            separation = math.hypot(dx, dy)

            a = separation / 2.0 - BODY_CLEARANCE
            if a < MIN_O_SPACE_HALF_LENGTH:
                continue

            points.extend(self.make_o_space_points(cx, cy, dx, dy, a))
            published.append((g.get("source_type", "conversation"), gid, a, camera_fresh))

        for gid, reason in drop:
            del self.active_groups[gid]
            self.get_logger().info(f"Group {gid} cleared - {reason}")

        # --- robot keep-out ------------------------------------------
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.frame_id, ROBOT_FRAME,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
            self.last_robot_xy = (tfm.transform.translation.x,
                                  tfm.transform.translation.y)
        except Exception:
            pass  # reuse last known pose

        if self.last_robot_xy is not None and points:
            rx, ry = self.last_robot_xy
            r2 = ROBOT_KEEPOUT_RADIUS ** 2
            before = len(points)
            points = [p for p in points
                      if (p[0] - rx) ** 2 + (p[1] - ry) ** 2 > r2]
            removed = before - len(points)
            if removed:
                self.get_logger().info(
                    f"Keep-out: removed {removed} point(s) within "
                    f"{ROBOT_KEEPOUT_RADIUS:.2f} m of robot")

        self.pub.publish(self.create_cloud(points))

        if published:
            parts_desc = []
            for entry in published:
                if entry[0] == "queue":
                    _, gid, n_members = entry
                    parts_desc.append(f"{gid}(queue,{n_members} members)")
                else:
                    src_type, gid, a, fresh = entry
                    parts_desc.append(
                        f"{gid}({src_type},len={2*a:.2f}m"
                        f"{'' if fresh else ',LIDAR-HELD'})")
            desc = ", ".join(parts_desc)
            self.get_logger().info(
                f"Published {len(published)} zone(s): {desc} "
                f"points={len(points)}")

    # -----------------------------------------------------------------
    def create_cloud(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * len(points)
        cloud.data = b"".join(struct.pack("fff", x, y, z) for x, y, z in points)
        cloud.is_dense = True
        return cloud

    def destroy_node(self):
        self.pub.publish(self.create_cloud([]))
        self.get_logger().info("Published empty cloud to clear costmap on shutdown")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SocialGroupCloudNode()
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
