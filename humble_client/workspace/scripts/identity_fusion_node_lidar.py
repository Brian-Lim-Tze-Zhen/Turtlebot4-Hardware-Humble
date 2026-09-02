#!/usr/bin/env python3
"""
identity_fusion_node.py

THESIS ADDITION - camera/lidar identity fusion (re-ID across occlusion).

-----------------------------------------------------------------------
THE PROBLEM
-----------------------------------------------------------------------
Every downstream layer keys its state by ByteTrack's track_id. A ~2s+
camera occlusion makes ByteTrack assign a NEW track_id to the same
physical person, which resets:

  - human_kf_predictor.py : new HumanTrackKF, update_count=0,
                            vx_filt=None, covariance reset, velocity
                            suppressed for 5 updates
  - group_formation_detector.py : close_since dict emptied, so a
                            conversation pair established for 30s must
                            re-accumulate CONV_MIN_DURATION from zero
  - social_group_cloud_node.py (planned) : group_id string changes,
                            costmap zone ages out and rebuilds -> flicker

-----------------------------------------------------------------------
THE FIX
-----------------------------------------------------------------------
The 2D lidar does not share the camera's occlusion geometry. Measured
this session: through a full walk-out / 12s hold / walk-back cycle, the
camera assigned a new track_id on return while ONE lidar track
(leg_detector_node id:2, 659 msgs, y spanning -3.95 to -0.42) stayed
continuous and was never pruned.

So: bind camera track_ids to lidar tracks while both are visible. When
a NEW camera track_id appears near a lidar track that still carries a
binding from a previously-lost camera track, adopt that old identity
instead of allocating a fresh one.

This node is a TRANSPARENT SHIM. It republishes /person_positions_map
verbatim except field [0] (track_id), which is rewritten to a stable
identity. Point human_kf_predictor's existing `input_topic` parameter
at the output topic and nothing else in the pipeline needs to change -
the KF, the close_since timers, and any future group_id strings simply
never observe the switch.

-----------------------------------------------------------------------
WHY SPURIOUS LIDAR TRACKS ARE HARMLESS HERE (design note)
-----------------------------------------------------------------------
leg_detector_node produces spurious tracks from wall fragments: a
person standing in front of a wall splits its continuous return into
short segments, and at ~4.9m range a real leg yields only 2-3 points,
making legs and fragments genuinely indistinguishable by width (measured:
fragment 0.061m sits BETWEEN person clusters at 0.048m and 0.100m) and
by shape (a 2-point cluster is exactly collinear by definition, so
curvature is undefined - no shape test can exist).

Rather than trying to purify the detector, this node makes purity
unnecessary: a lidar track can only ever be adopted as an anchor if it
was CAMERA-CONFIRMED first, i.e. it appeared in stable_of_lidar because
a real camera detection was matched to it. A wall fragment is never
camera-confirmed, so it can never donate an identity no matter how long
it persists. The detector does not need to be clean - only continuous,
which was verified independently.

-----------------------------------------------------------------------
INPUT / OUTPUT
-----------------------------------------------------------------------
Subscribes:
  /person_positions_map    (String) from yolo_leg_detector_lidar.py
      track_id,conf,x1,y1,x2,y2,leg_kpts
      (pixel-space only - the camera node no longer computes depth or
      a map-frame position; LIDAR is the sole range/position source)
  /lidar_person_clusters   (String) from lidar_person_detector.py
      lidar_id,conf,map_x,map_y   (conf is a placeholder, always 1.00)

Publishes:
  /person_positions_fused  (String) -
      stable_id,conf,x,y,depth,u,v,x1,y1,x2,y2,source
      Field [0] is the stable id (not camera's raw track_id). Fields
      [2],[3] (x,y) are always the LIDAR track's position - the camera
      node has no position of its own to fall back on anymore. depth/u/v
      (fields [4]-[6]) are always "0.0","0","0" now (no longer computed);
      kept only so field count/indices stay stable for any downstream
      parser. x1,y1,x2,y2 (fields [7]-[10]) are the camera's pixel bbox
      when a camera detection drove this message.
      Field [11], "source", is either "camera_confirmed" (this cycle's
      camera detection matched a LIDAR track by bearing) or "lidar_only"
      (person is camera-occluded; position is LIDAR alone, coasting on
      a previously camera-confirmed binding).
      A camera detection with NO lidar match this cycle publishes
      nothing - there is no camera-only position to report anymore, so
      unlike the old depth-based version, an unmatched camera track is
      silent rather than emitting a stale/wrong position.
      human_kf_predictor.py should apply higher measurement/process
      noise on "lidar_only" updates.

      lidar_only messages are emitted from a timer, not from
      camera_callback - there is no camera detection to piggyback on
      while occluded, so this is the only path that keeps a person's
      track alive (and SocialCritic fed) through the occlusion window.

-----------------------------------------------------------------------
CALIBRATION STATUS - READ BEFORE TRUSTING THE ANGLE GATE
-----------------------------------------------------------------------
Camera/lidar matching is now bearing-based, not position-based: the
camera contributes a pixel bbox center converted to a bearing relative
to the robot's heading (assumes the OAK-D's optical axis is aligned
with base_link's forward axis - true for TurtleBot4's fixed mount), and
each lidar track's map-frame (x,y) is converted to its own bearing from
the robot via TF (map -> base_link). ANGLE_GATE_DEG bounds how far
apart those two bearings may be and still count as the same person.

Like the old distance gate, this is an initial estimate, not a measured
one. This node logs every camera-lidar pairing bearing it evaluates
(look for "pair-bearing" lines, in degrees) precisely so the gate can be
set from the observed distribution after one run instead of guessed.
Do that before relying on any result from this node. Two people
standing at similar bearings but different ranges will alias under
bearing-only matching in a way the old distance gate would not have -
worth watching for in multi-person scenes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


# =======================================================================
# Tunables
# =======================================================================

# THESIS CHANGE (bearing-based matching): the camera node no longer
# computes depth or a map-frame position (see yolo_leg_detector_lidar.py -
# depth/TF removed, LIDAR is now the sole range source). camera_callback
# therefore has nothing but a pixel bbox to offer, so matching switched
# from "camera (x,y) vs lidar (x,y) within CAMERA_LIDAR_GATE metres" to
# "camera bbox-center bearing vs lidar track bearing (both relative to
# the robot, via TF) within ANGLE_GATE degrees".
#
# Max angular separation to consider a camera detection and a lidar
# track the same person. PROVISIONAL like the old distance gate was -
# log lines below ("pair-bearing") let this be tuned from the observed
# distribution instead of guessed.
ANGLE_GATE_DEG = 20.0

# OAK-D Pro RGB horizontal FOV. Approximate for the un-cropped sensor;
# the preview stream is a center crop so this is close but not exact -
# fine for a coarse re-ID gate, not for anything needing sub-degree
# accuracy.
CAMERA_HFOV_DEG = 69.0

# Preview stream width in pixels (matches oakd_pro.yaml i_preview_size).
CAMERA_PREVIEW_WIDTH = 250

# Frames for the TF lookup used to convert lidar map-frame (x,y) into a
# bearing relative to the robot's current heading.
MAP_FRAME = "map"
BASE_FRAME = "base_link"

# How long a camera track_id -> stable_id mapping is retained after the
# camera stops reporting it. Only needs to outlive brief detection
# dropouts; the lidar binding is what survives real occlusions.
CAMERA_TRACK_TIMEOUT = 2.0    # s

# A stable id is considered "in use" if some camera track reported it
# this recently. Prevents two SIMULTANEOUSLY visible camera tracks from
# collapsing onto one stable id (which would merge two people).
CAMERA_ACTIVE_WINDOW = 1.0    # s

# How long a lidar->stable binding survives without camera confirmation.
# This is the real occlusion budget: it must exceed the longest camera
# gap to be bridged.
#
# MEASURED, not guessed: an earlier value of 30s was set by (wrongly)
# equating the blind window with the mover's 12s hold phase. The actual
# camera-blind window is far longer - the person exits the FOV partway
# through the 35s walk-out and only re-enters late in the 35s walk-back,
# giving ~60s. The binding expired 0.85s before the returning camera
# track appeared, so the re-ID had nothing left to adopt and allocated a
# fresh identity instead. Confirmed the lidar anchor itself was never
# pruned during that window.
#
# Note this timeout is NOT the safety property. The real guard against
# adopting a stale identity is the continuity check in prune(): a
# binding is dropped the moment its lidar track disappears, which is
# exactly what happens if the person genuinely leaves lidar range. That
# is the "long absence" case the design scope explicitly excludes. So
# this can be generous - it only bounds how long a CONTINUOUSLY TRACKED
# but camera-invisible person keeps their identity.
LIDAR_BINDING_TIMEOUT = 300.0  # s

# Drop a cached lidar track this long after its last message.
LIDAR_TRACK_TIMEOUT = 2.0     # s

# How often to emit "lidar_only" coasting updates for stable ids whose
# camera track has gone quiet but whose lidar binding is still alive.
# Independent of scan rate - this is a publish cadence, not a sensor rate.
LIDAR_ONLY_PUBLISH_PERIOD = 0.1   # s


class IdentityFusionNode(Node):
    def __init__(self):
        super().__init__("identity_fusion_node")

        self.declare_parameter("camera_topic", "/person_positions_map")
        self.declare_parameter("lidar_topic", "/lidar_person_clusters")
        self.declare_parameter("output_topic", "/person_positions_fused")
        self.declare_parameter("angle_gate_deg", ANGLE_GATE_DEG)
        self.declare_parameter("camera_hfov_deg", CAMERA_HFOV_DEG)
        self.declare_parameter("camera_preview_width", CAMERA_PREVIEW_WIDTH)

        self.camera_topic = self.get_parameter("camera_topic").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.angle_gate_rad = math.radians(
            float(self.get_parameter("angle_gate_deg").value))
        self.hfov_rad = math.radians(
            float(self.get_parameter("camera_hfov_deg").value))
        self.preview_width = float(
            self.get_parameter("camera_preview_width").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera track_id -> [stable_id, last_seen]
        self.stable_of_camera = {}
        # lidar_id -> [stable_id, last_confirmed]   (camera-confirmed only)
        self.stable_of_lidar = {}
        # lidar_id -> (x, y, last_seen)
        self.lidar_tracks = {}

        # THESIS FIX (bearing-only matching flapping): nearest_lidar()
        # re-picks the single best-bearing lidar cluster from scratch
        # every camera_callback, with zero range disambiguation. When
        # two people stand at similar bearings from the robot - exactly
        # the "close pair" scenario this pipeline targets - small noise
        # in either bearing estimate can flip which lidar cluster wins
        # the argmin from one cycle to the next, silently reassigning an
        # already-known camera track's STABLE ID to a DIFFERENT PHYSICAL
        # PERSON's position. Measured effect: /predicted_person_positions
        # showing the same track_id's position jump ~1-2m between
        # consecutive samples, apparent speed 1-2 m/s for two stationary
        # people, permanently failing social_group_detector's
        # CONV_MAX_SPEED gate.
        #
        # Fix: once a camera track is bound to a lidar track, stay bound
        # to that SAME lidar_id next cycle as long as it's still within
        # the angle gate, instead of re-searching argmin every time. Only
        # fall back to a fresh search if the sticky candidate has left
        # the gate or its lidar track vanished (genuine loss, not noise).
        self.lidar_of_camera = {}

        self.next_stable_id = 0
        self.reid_count = 0

        # stable_id -> last time ANY message (camera or lidar-only) was
        # published under it. Used only to space out lidar-only coasting
        # publishes from a just-happened camera-confirmed one.
        self.last_published = {}

        self.create_subscription(
            String, self.lidar_topic, self.lidar_callback, 10)
        self.create_subscription(
            String, self.camera_topic, self.camera_callback, 10)

        self.pub = self.create_publisher(String, self.output_topic, 10)

        self.create_timer(1.0, self.prune)
        self.create_timer(LIDAR_ONLY_PUBLISH_PERIOD, self.publish_lidar_only)

        self.get_logger().info("Identity fusion node started")
        self.get_logger().info(f"Camera in : {self.camera_topic}")
        self.get_logger().info(f"Lidar  in : {self.lidar_topic}")
        self.get_logger().info(f"Fused out: {self.output_topic}")
        self.get_logger().info(
            f"Camera-lidar angle gate: {math.degrees(self.angle_gate_rad):.1f} deg "
            f"(PROVISIONAL - calibrate from 'pair-bearing' log lines)")
        self.get_logger().info(
            f"Occlusion budget (lidar binding timeout): "
            f"{LIDAR_BINDING_TIMEOUT:.0f} s")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -----------------------------------------------------------------
    def lidar_callback(self, msg):
        parts = msg.data.split(",")
        # THESIS FIX (field mismatch): lidar_person_detector.py publishes
        # "id,conf,x,y" (conf is a placeholder, currently always 1.00 -
        # see that file's comments). This previously assumed "id,x,y",
        # so parts[1] (conf) was read as x and parts[2] (real x) was
        # read as y - every LIDAR position landed in the wrong place
        # with no error raised. Kept conf in the message (not stripped
        # from the publisher) since it's the intended slot for a future
        # LIDAR detection-quality metric.
        if len(parts) < 4:
            return
        try:
            lid = int(float(parts[0]))
            x = float(parts[2])
            y = float(parts[3])
        except ValueError:
            return
        self.lidar_tracks[lid] = (x, y, self.now())

    # -----------------------------------------------------------------
    def robot_pose(self):
        """(x, y, yaw) of base_link in map, or None if TF isn't ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, Time())
        except Exception:
            return None

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        # yaw from quaternion (planar robot, roll/pitch ~0)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return rx, ry, yaw

    @staticmethod
    def wrap_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def pixel_center_to_bearing(self, x1, x2):
        """Bbox horizontal center -> bearing relative to robot heading.

        Assumes the camera's optical axis is aligned with the robot's
        forward (base_link x) direction - true for the TurtleBot4's
        fixed forward-facing OAK-D mount. Positive = person is to the
        robot's left, matching the standard CCW-positive yaw convention.
        """
        u = (x1 + x2) / 2.0
        normalized = (u - self.preview_width / 2.0) / (self.preview_width / 2.0)
        # image-right (+u) = robot's right = negative yaw
        return -normalized * (self.hfov_rad / 2.0)

    def sticky_lidar(self, cam_id, cam_bearing_rel):
        """If cam_id was bound to a lidar track last cycle and that track
        is still within the angle gate, keep using it - avoids the
        argmin flip-flop when two lidar tracks have close bearings.
        Returns (lidar_id, ang_diff) or (None, None) if no sticky match
        applies (never bound before, or genuinely out of gate now)."""
        prev_lid = self.lidar_of_camera.get(cam_id)
        if prev_lid is None or prev_lid not in self.lidar_tracks:
            return None, None

        pose = self.robot_pose()
        if pose is None:
            return None, None
        rx, ry, ryaw = pose

        lx, ly, _ = self.lidar_tracks[prev_lid]
        bearing_to_track = math.atan2(ly - ry, lx - rx)
        track_bearing_rel = self.wrap_angle(bearing_to_track - ryaw)
        diff = abs(self.wrap_angle(track_bearing_rel - cam_bearing_rel))

        if diff < self.angle_gate_rad:
            return prev_lid, diff
        return None, None

    def nearest_lidar(self, cam_bearing_rel):
        """Nearest cached lidar track within ANGLE_GATE -> (lidar_id, ang_diff).

        Converts each lidar track's map-frame (x,y) into a bearing
        relative to the robot's current heading via TF, then compares
        against the camera's pixel-derived relative bearing.
        """
        pose = self.robot_pose()
        if pose is None:
            return None, None
        rx, ry, ryaw = pose

        best_id, best_diff = None, self.angle_gate_rad
        for lid, (lx, ly, _) in self.lidar_tracks.items():
            bearing_to_track = math.atan2(ly - ry, lx - rx)
            track_bearing_rel = self.wrap_angle(bearing_to_track - ryaw)
            diff = abs(self.wrap_angle(track_bearing_rel - cam_bearing_rel))
            if diff < best_diff:
                best_id, best_diff = lid, diff
        return best_id, best_diff

    def stable_id_in_use(self, stable_id, exclude_cam_id, now):
        for cam_id, (sid, last) in self.stable_of_camera.items():
            if cam_id == exclude_cam_id:
                continue
            if sid == stable_id and (now - last) < CAMERA_ACTIVE_WINDOW:
                # THESIS FIX: don't block re-ID just because the OLD
                # camera track was updated recently - if its own lidar
                # anchor has already vanished, it's very likely the
                # same physical person handed off to a new ByteTrack
                # id, not a second person genuinely still being seen.
                # Only treat this as a real conflict if the old track
                # still has a live lidar anchor backing it up.
                old_lidar = self.lidar_of_camera.get(cam_id)
                if old_lidar is None or old_lidar not in self.lidar_tracks:
                    continue
                return True
        return False

    def allocate(self):
        sid = self.next_stable_id
        self.next_stable_id += 1
        return sid

    # -----------------------------------------------------------------
    def camera_callback(self, msg):
        # THESIS CHANGE: yolo_leg_detector_lidar.py no longer computes
        # depth/map position - it publishes pixel-space detections only:
        #   track_id,conf,x1,y1,x2,y2,leg_kpts
        # (leg_kpts is "px:py;px:py;..." for 0-4 visible leg keypoints,
        # may be empty; harmless here since we split on "," and it's
        # already the last field with no further commas inside it.)
        parts = msg.data.split(",")
        if len(parts) < 6:
            self.get_logger().warn(f"Invalid camera msg: {msg.data}")
            return

        try:
            cam_id = int(float(parts[0]))
            conf = parts[1]
            x1 = float(parts[2])
            y1 = float(parts[3])
            x2 = float(parts[4])
            y2 = float(parts[5])
        except ValueError:
            self.get_logger().warn(f"Parse failed: {msg.data}")
            return

        now = self.now()
        cam_bearing = self.pixel_center_to_bearing(x1, x2)

        lid, diff = self.sticky_lidar(cam_id, cam_bearing)
        if lid is not None:
            self.get_logger().info(
                f"pair-bearing cam:{cam_id} <-> lidar:{lid} = "
                f"{math.degrees(diff):.1f} deg (sticky)")
        else:
            lid, diff = self.nearest_lidar(cam_bearing)
            if lid is not None:
                # Logged so ANGLE_GATE_DEG can be calibrated from the
                # observed distribution rather than left at its guess.
                self.get_logger().info(
                    f"pair-bearing cam:{cam_id} <-> lidar:{lid} = "
                    f"{math.degrees(diff):.1f} deg")

        if lid is not None:
            self.lidar_of_camera[cam_id] = lid

        known = self.stable_of_camera.get(cam_id)

        if known is not None:
            stable_id = known[0]
        else:
            stable_id = None

            # --- the re-ID step ---------------------------------------
            # A new camera id landed near a lidar track that still holds
            # a binding from an earlier, now-lost camera id. Adopt it.
            if lid is not None and lid in self.stable_of_lidar:
                candidate = self.stable_of_lidar[lid][0]
                if not self.stable_id_in_use(candidate, cam_id, now):
                    stable_id = candidate
                    self.reid_count += 1
                    self.get_logger().info(
                        f"RE-ID #{self.reid_count}: new camera track "
                        f"{cam_id} adopted stable id {stable_id} via "
                        f"lidar track {lid} (bearing diff "
                        f"{math.degrees(diff):.1f} deg)")
                else:
                    self.get_logger().info(
                        f"Lidar {lid} suggests stable {candidate} for "
                        f"camera {cam_id}, but that identity is already "
                        f"active on another camera track - allocating new")

            if stable_id is None:
                stable_id = self.allocate()
                self.get_logger().info(
                    f"New identity: camera track {cam_id} -> stable "
                    f"{stable_id}"
                    + (f" (anchored on lidar {lid})" if lid is not None
                       else " (no lidar anchor in range)"))

        self.stable_of_camera[cam_id] = [stable_id, now]

        # Confirm/refresh the lidar binding. THIS is what makes a lidar
        # track eligible to donate an identity later - and what keeps
        # never-confirmed wall fragments permanently ineligible.
        if lid is not None:
            self.stable_of_lidar[lid] = [stable_id, now]

        # Position: LIDAR is the only real-world position source now
        # (camera node no longer computes depth/map position at all).
        # Without a lidar match this cycle there is no position to
        # report - skip publishing rather than emit a bogus (0,0).
        if lid is None:
            return

        lx, ly, _ = self.lidar_tracks[lid]
        fields = [
            str(stable_id), conf, f"{lx:.3f}", f"{ly:.3f}",
            "0.0", "0", "0",
            f"{int(x1)}", f"{int(y1)}", f"{int(x2)}", f"{int(y2)}",
            "camera_confirmed",
        ]

        out = String()
        out.data = ",".join(fields)
        self.pub.publish(out)
        self.last_published[stable_id] = now

    # -----------------------------------------------------------------
    def publish_lidar_only(self):
        """Keep a person's stable_id alive while camera-occluded.

        For every lidar->stable binding that is still camera-confirmed
        (i.e. was donated by a real camera detection at some point and
        hasn't been pruned), if no camera track is currently reporting
        that stable_id, publish the lidar position under it anyway.
        Without this, the moment camera loses someone the whole
        downstream chain (KF, predicted cloud, SocialCritic) sees
        nothing for them until camera re-acquires - exactly the gap
        that caused "no person data this cycle" during occlusion.
        """
        now = self.now()

        active_stable_ids = {
            sid for sid, last in self.stable_of_camera.values()
            if (now - last) < CAMERA_ACTIVE_WINDOW
        }

        for lid, (stable_id, _confirmed_at) in list(self.stable_of_lidar.items()):
            if stable_id in active_stable_ids:
                continue  # camera has this one this cycle, camera_callback publishes it
            if lid not in self.lidar_tracks:
                continue  # lidar track already gone; prune() will drop the binding

            last_pub = self.last_published.get(stable_id, 0.0)
            if (now - last_pub) < LIDAR_ONLY_PUBLISH_PERIOD * 0.5:
                continue  # camera_callback just published this stable_id, skip

            lx, ly, _ = self.lidar_tracks[lid]

            # Message shape matches camera_callback's output so
            # human_kf_predictor parses both the same way. depth/u/v/
            # bbox fields have no camera detection to report this
            # cycle - left at 0 rather than omitted, so field count
            # (and therefore field indices) never changes with source.
            fields = [
                str(stable_id), "0.00", f"{lx:.3f}", f"{ly:.3f}",
                "0.0", "0", "0", "0", "0", "0", "0", "lidar_only",
            ]

            out = String()
            out.data = ",".join(fields)
            self.pub.publish(out)
            self.last_published[stable_id] = now

    # -----------------------------------------------------------------
    def prune(self):
        now = self.now()

        for cam_id in [c for c, (_, t) in self.stable_of_camera.items()
                       if now - t > CAMERA_TRACK_TIMEOUT]:
            del self.stable_of_camera[cam_id]
            self.lidar_of_camera.pop(cam_id, None)

        for cam_id in [c for c in self.lidar_of_camera
                       if c not in self.stable_of_camera]:
            del self.lidar_of_camera[cam_id]

        for lid in [l for l, v in self.lidar_tracks.items()
                    if now - v[2] > LIDAR_TRACK_TIMEOUT]:
            del self.lidar_tracks[lid]

        for lid in [l for l, (_, t) in self.stable_of_lidar.items()
                    if now - t > LIDAR_BINDING_TIMEOUT
                    or l not in self.lidar_tracks]:
            sid = self.stable_of_lidar[lid][0]
            age = now - self.stable_of_lidar[lid][1]
            reason = ("lidar track vanished"
                      if lid not in self.lidar_tracks
                      else f"binding expired after {age:.1f}s")
            del self.stable_of_lidar[lid]
            self.get_logger().info(
                f"Dropped lidar binding {lid} -> stable {sid} ({reason})")


def main(args=None):
    rclpy.init(args=args)
    node = IdentityFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    # See lidar_person_detector.py's main() for why this is guarded.
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()