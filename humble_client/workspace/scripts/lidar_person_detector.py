#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class LidarPersonDetector(Node):
    def __init__(self):
        super().__init__("lidar_person_detector")

        # =====================================
        # Parameters
        # =====================================
        self.declare_parameter("scan_topic", "/turtlebot4/scan")
        # THESIS FIX (topic mismatch): identity_fusion_node.py's
        # lidar_topic parameter defaults to "/lidar_person_clusters".
        # This was previously "person_positions_base" (no leading slash,
        # different name) - identity_fusion_node never received any
        # LIDAR data, nearest_lidar() always returned None, and every
        # camera detection silently fell back to camera-only position
        # with no error raised. Matched to identity_fusion_node's
        # default so the two nodes connect out of the box.
        self.declare_parameter("output_topic", "/lidar_person_clusters")
        # THESIS FIX (frame mismatch): this was "odom". yolo_detector.py
        # publishes camera positions in "map". identity_fusion_node.py
        # compares the two topics' (x,y) as raw floats with no frame
        # transform of its own - map and odom share an origin only at
        # zero localization drift, so every LIDAR position was off from
        # every camera position by whatever that drift was. Confirmed:
        # across 5 separate people over ~400s, nearest_lidar() found a
        # match ZERO times ("no lidar anchor in range" on every single
        # new identity) - not an occasional miss, a 100% miss rate.
        # Matching this to yolo_detector's "map" fixes the root cause.
        self.declare_parameter("target_frame", "map")

        # Gap-based clustering: new cluster starts when consecutive
        # points are farther apart than this (meters)
        self.declare_parameter("cluster_gap", 0.15)

        # Leg-pair size heuristic (cluster width in meters)
        # THESIS TUNE: widened defaults (0.05-0.60m, min_points=3) let
        # ~30-40 non-person clusters pass the width filter every single
        # scan in a cluttered room (chair/table legs, wall corners,
        # cables) - real human legs/ankles are much narrower than the
        # old 0.60m ceiling, and thin clutter edges rarely return this
        # many points. Tightened both to cut false positives at the
        # source, since no amount of downstream ID-stability logic can
        # fix track association when 30+ candidates compete every scan.
        # Re-tune per-room if this starts rejecting real legs too often
        # (loosen) or still lets furniture through (tighten further).
        self.declare_parameter("min_cluster_width", 0.05)
        self.declare_parameter("max_cluster_width", 0.40)
        self.declare_parameter("min_points_per_cluster", 4)

        # Track association: max distance (m) to match a new cluster
        # to an existing track between scans
        self.declare_parameter("track_match_dist", 0.7)
        self.declare_parameter("track_timeout", 3.0)

        # THESIS ADDITION (velocity-predicted matching). Caps the
        # per-scan velocity estimate used to predict a track's expected
        # position - see the matching loop below. Set above realistic
        # combined closing speed (measured 1.46 m/s: 1.2 m/s walker +
        # 0.26 m/s robot) with margin, so genuine fast motion is
        # trusted but a single bad association can't make the
        # predictor run away and start matching further and further
        # from reality on subsequent scans.
        self.declare_parameter("max_track_speed", 2.5)

        # Static-object rejection: a track must move at least this far
        # (meters) within static_check_window seconds to be published as
        # a person. Filters out furniture/walls/corners that pass the
        # leg-width heuristic but never move.
        self.declare_parameter("static_move_threshold", 0.08)
        # THESIS TUNE: lowered 1.0s -> 0.5s -> 0.3s across two rounds
        # of measurement. Each halving of the window bought ~0.6-0.8m
        # of earlier detection at 1.2 m/s (measured: 1.0s window
        # detected at 5.57m robot-person distance, 0.5s window at
        # 6.39m). Swept 0.5/0.4/0.3/0.25/0.2s against synthetic walkers
        # at 1.2, 0.3, and 0.15 m/s: flicker reappears at 0.2s (0.3 m/s
        # walker produces 5 spurious transitions), but 0.3s stays clean
        # across all three speeds with no transitions beyond the single
        # genuine detect. Not pushed to 0.25s despite also testing
        # clean, to keep margin rather than sit exactly on the edge.
        self.declare_parameter("static_check_window", 0.3)

        # Two separate leg clusters within this distance (meters) of each
        # other are merged into one person track — otherwise each leg gets
        # its own ID/velocity, and gait leg-swing shows up as false vy.
        self.declare_parameter("leg_pair_merge_dist", 0.35)

        # Static-map subtraction: candidates that land on a known-occupied
        # cell of the SLAM map (chairs, tables, walls) are dropped before
        # clustering ever sees them - robust to pose jitter, unlike the
        # move-threshold check alone, since furniture is occupied in the
        # map regardless of any single scan's noise.
        self.declare_parameter("map_topic", "/map")
        # OccupancyGrid cell values: 0=free, 100=occupied, -1=unknown.
        # Cells at/above this are treated as static obstacles.
        self.declare_parameter("static_occupancy_threshold", 50)
        # Inflate occupied cells by this many grid cells before testing a
        # candidate against them - map resolution/localization noise can
        # place a real chair-leg point just outside its exact occupied
        # cell otherwise.
        self.declare_parameter("static_inflation_cells", 2)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.cluster_gap = self.get_parameter("cluster_gap").value
        self.min_width = self.get_parameter("min_cluster_width").value
        self.max_width = self.get_parameter("max_cluster_width").value
        self.min_points = self.get_parameter("min_points_per_cluster").value
        self.track_match_dist = self.get_parameter("track_match_dist").value
        self.max_track_speed = self.get_parameter("max_track_speed").value
        self.track_timeout = self.get_parameter("track_timeout").value
        self.static_move_threshold = self.get_parameter("static_move_threshold").value
        self.static_check_window = self.get_parameter("static_check_window").value
        self.leg_pair_merge_dist = self.get_parameter("leg_pair_merge_dist").value
        self.map_topic = self.get_parameter("map_topic").value
        self.static_occupancy_threshold = self.get_parameter("static_occupancy_threshold").value
        self.static_inflation_cells = self.get_parameter("static_inflation_cells").value

        # =====================================
        # TF
        # =====================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =====================================
        # Track state: {id: {"x":.., "y":.., "last_seen": t}}
        # Simple nearest-neighbor ID persistence across scans —
        # actual KF/velocity is handled downstream by human_kf_predictor.py
        # =====================================
        self.tracks = {}
        self.next_id = 0

        # State-change logging (THESIS FIX - readability): the raw
        # per-scan DEBUG line printed identical text every second
        # regardless of whether anything changed, burying the one
        # moment that mattered (publishing stopping) in ~130 identical
        # lines. These track what was true last scan, so we only log
        # when it's different this scan.
        self._was_publishing = False
        self._published_ids_prev = set()
        self._last_publish_time = None
        self._last_heartbeat_time = None
        self._heartbeat_every = 10.0   # s; "still alive, still idle" ping

        # THESIS FIX (log noise from short-lived clutter): "New person
        # id(s) publishing" used to fire the instant ANY candidate first
        # started publishing - including furniture/wall-fragment noise
        # that only survives a scan or two before its cluster splits
        # differently and it dies again. next_id is shared across every
        # candidate, real person or not, so this made the log look like
        # constant churn even when the actual tracked person's identity
        # (visible in identity_fusion_node's stable_of_lidar bindings)
        # was completely stable the whole encounter.
        #
        # Fix: only announce an id once it has been publishing
        # CONTINUOUSLY for NEW_ID_LOG_MIN_AGE - long enough that it's
        # very unlikely to be a single-scan clustering fluke. Purely a
        # logging change; does not affect what gets published on
        # /lidar_person_clusters or matched by identity_fusion_node.
        self._publish_start_time = {}   # id -> start of current streak
        self._announced_ids = set()     # ids that passed the min-age gate
        self._new_id_log_min_age = 0.5  # s; below this, likely clutter

        # =====================================
        # Static map (for static-map subtraction)
        # =====================================
        self.map_data = None      # tuple(width, height, resolution, origin_x, origin_y)
        self.map_grid = None      # list[int], row-major, len = width*height

        # =====================================
        # ROS interfaces
        # =====================================
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.pub = self.create_publisher(String, self.output_topic, 10)

        # map_server publishes with TRANSIENT_LOCAL durability (latched) -
        # match it, or a map published before this node starts is never
        # received.
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )

        self.get_logger().info("LiDAR person detector started")
        self.get_logger().info(f"Scan topic  : {self.scan_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Target frame: {self.target_frame}")
        self.get_logger().info(f"Map topic   : {self.map_topic} (static-map subtraction)")

    def map_callback(self, msg: OccupancyGrid):
        # Stored once here rather than re-read on every scan - occupancy
        # grids only change on a fresh SLAM/localization launch, not per
        # scan, and this callback only fires again if the map is
        # re-published.
        self.map_data = (
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
        )
        self.map_grid = msg.data
        self.get_logger().info(
            f"Static map received: {msg.info.width}x{msg.info.height} "
            f"@ {msg.info.resolution:.3f} m/cell")

    def is_static_obstacle(self, x, y):
        """True if (x, y) in map frame lands on/near a known-occupied
        map cell - i.e. furniture/wall, not a dynamic person. False
        (never rejects) if no map has been received yet."""
        if self.map_grid is None:
            return False

        width, height, res, origin_x, origin_y = self.map_data
        col = int((x - origin_x) / res)
        row = int((y - origin_y) / res)

        k = self.static_inflation_cells
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                r, c = row + dr, col + dc
                if 0 <= r < height and 0 <= c < width:
                    val = self.map_grid[r * width + c]
                    if val >= self.static_occupancy_threshold:
                        return True
        return False

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()

        # ---- 1. Convert ranges to (x, y) points in the scan frame ----
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment

        if not points:
            return

        # ---- 2. Gap-based clustering (sequential, O(n)) ----
        clusters = []
        current = [points[0]]
        for i in range(1, len(points)):
            px, py = points[i - 1]
            cx, cy = points[i]
            dist = math.hypot(cx - px, cy - py)
            if dist > self.cluster_gap:
                clusters.append(current)
                current = [points[i]]
            else:
                current.append(points[i])
        clusters.append(current)

        # ---- 3. Filter clusters by leg-pair size heuristic ----
        candidates = []
        for c in clusters:
            if len(c) < self.min_points:
                continue
            xs = [p[0] for p in c]
            ys = [p[1] for p in c]
            width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if self.min_width <= width <= self.max_width:
                cx = sum(xs) / len(xs)
                cy = sum(ys) / len(ys)
                candidates.append((cx, cy))

        self.get_logger().info(
            f"DEBUG: {len(points)} pts -> {len(clusters)} clusters -> "
            f"{len(candidates)} pass width filter",
            throttle_duration_sec=1.0,
        )

        if not candidates:
            return

        # ---- 3b. Merge nearby leg-pair clusters into one person ----
        # Greedy merge: any two candidates within leg_pair_merge_dist are
        # treated as the same person's two legs and averaged into one point.
        merged = []
        used = [False] * len(candidates)
        for i in range(len(candidates)):
            if used[i]:
                continue
            group = [candidates[i]]
            used[i] = True
            for j in range(i + 1, len(candidates)):
                if used[j]:
                    continue
                d = math.hypot(
                    candidates[j][0] - candidates[i][0],
                    candidates[j][1] - candidates[i][1],
                )
                if d <= self.leg_pair_merge_dist:
                    group.append(candidates[j])
                    used[j] = True
            gx = sum(p[0] for p in group) / len(group)
            gy = sum(p[1] for p in group) / len(group)
            merged.append((gx, gy))
        candidates = merged

        # ---- 4. Transform candidates into target_frame ----
        # NOTE: the scan message's header.frame_id
        # ("turtlebot4/rplidar_link/rplidar") does not match the TF tree,
        # which registers the frame as "rplidar_link" (child of shell_link).
        # Use the known-good TF frame name instead of trusting the scan header.
        source_frame = "rplidar_link"

        # THESIS FIX (ego-motion position jitter, only visible while
        # the robot is moving): Time() with no argument requests the
        # LATEST available transform, not the transform at the scan's
        # actual capture time (msg.header.stamp). While stationary
        # these are identical - no error. While moving, the
        # map->rplidar_link transform keeps changing between capture
        # and processing (a few ms of latency), so every cluster was
        # being transformed with a DIFFERENT robot pose than the one
        # true when the LIDAR actually captured it - an ego-motion-
        # proportional position error, worse the faster the robot
        # moves. This surfaced as clusters jumping frame-to-frame by
        # more than track_match_dist purely from robot motion, not
        # person motion - causing exactly the "New/stopped publishing"
        # id churn seen only while the robot moves.
        #
        # Same fix as yolo_detector.py's rotation-jump fix: try the
        # scan's own capture stamp first (correct), fall back to
        # latest if TF hasn't caught up yet (this node's own logs have
        # shown real "extrapolation into the past" errors - using the
        # exact stamp unconditionally would make MORE lookups fail
        # outright, not fewer).
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.3),
                )
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")
                return

        transformed = []
        for cx, cy in candidates:
            pt = PointStamped()
            pt.header = msg.header
            pt.header.frame_id = source_frame
            pt.point.x = cx
            pt.point.y = cy
            pt.point.z = 0.0
            tpt = do_transform_point(pt, tf)
            transformed.append((tpt.point.x, tpt.point.y))

        # ---- 4b. Static-map subtraction: drop candidates on known
        # obstacles (chairs, tables, walls) before they ever reach
        # clustering/tracking. No-op (nothing dropped) until a map is
        # received.
        before = len(transformed)
        transformed = [
            (x, y) for (x, y) in transformed if not self.is_static_obstacle(x, y)
        ]
        if len(transformed) != before:
            self.get_logger().info(
                f"DEBUG: static-map dropped {before - len(transformed)} "
                f"candidate(s) on known obstacles",
                throttle_duration_sec=1.0,
            )

        if not transformed:
            return

        # ---- 5. Nearest-neighbor track association ----
        now_s = now.nanoseconds * 1e-9
        unmatched = list(range(len(transformed)))
        used_ids = set()

        # THESIS FIX (id churn under fast/consistent motion): matching
        # was purely "is the new cluster within track_match_dist of
        # where this track LAST was" - fine for a near-stationary
        # target, but breaks down whenever real displacement between
        # two processed scans approaches or exceeds track_match_dist
        # (0.5 m). At the measured head-on closing speed (1.46 m/s,
        # robot + person combined) even a modest processing gap of
        # ~0.35s covers that whole radius on its own - no bug required,
        # just the fixed-radius match running out of room. Observed as
        # persistent id churn that the earlier TF-timestamp fix (a
        # different, also-real bug) did not resolve on its own.
        #
        # Fix: match against each track's PREDICTED current position
        # (last position + velocity * elapsed time), not its last raw
        # position. This is standard practice for anything faster than
        # "basically not moving between scans" - a consistently-moving
        # target then always has a nearby prediction to match against,
        # while genuine jitter (no consistent velocity) gets no such
        # help and is rejected the same as before.
        for track_id, t in list(self.tracks.items()):
            dt_pred = now_s - t["last_seen"]
            pred_x = t["x"] + t.get("vx", 0.0) * dt_pred
            pred_y = t["y"] + t.get("vy", 0.0) * dt_pred

            # THESIS FIX (cold-start gate): a track's FIRST re-
            # observation has no velocity estimate yet (vx=vy=0), so
            # its prediction equals the last raw position - identical
            # to the old behavior, and identically broken by fast
            # motion: at 1.46 m/s with a 0.35s gap, that first real
            # step (0.51 m) already exceeds track_match_dist (0.5 m)
            # before velocity ever gets a chance to be learned. Every
            # subsequent scan repeated the same failure, since each
            # broken match spawns a fresh vx=0 track. Confirmed by
            # direct test: the plain predicted-position fix alone gave
            # ZERO improvement for this case.
            #
            # Fix: widen the gate specifically while a track has no
            # established velocity - bound it by max_track_speed over
            # the elapsed time, since that's the true worst case with
            # no better estimate available. Once a track has matched
            # at least once with a real dt, its velocity estimate
            # exists and the gate tightens back to the normal radius,
            # since the prediction itself now accounts for the motion.
            if t.get("vx", 0.0) == 0.0 and t.get("vy", 0.0) == 0.0 \
                    and not t.get("vel_established", False):
                gate = self.track_match_dist + self.max_track_speed * dt_pred
            else:
                gate = self.track_match_dist

            best_i, best_d = None, gate
            for i in unmatched:
                d = math.hypot(
                    transformed[i][0] - pred_x, transformed[i][1] - pred_y
                )
                if d < best_d:
                    best_i, best_d = i, d
            if best_i is not None:
                x, y = transformed[best_i]

                # Velocity estimate for the NEXT prediction: finite
                # difference against the position just before this
                # update, lightly smoothed (alpha=0.5) so one noisy
                # jump doesn't fully overwrite a track's established
                # velocity. Capped at max_track_speed so a single bad
                # association can't make the predictor run away and
                # start matching further and further from reality.
                if dt_pred > 1e-3:
                    vx_raw = (x - t["x"]) / dt_pred
                    vy_raw = (y - t["y"]) / dt_pred
                    speed_raw = math.hypot(vx_raw, vy_raw)
                    if speed_raw > self.max_track_speed:
                        scale = self.max_track_speed / speed_raw
                        vx_raw *= scale
                        vy_raw *= scale
                    vx = 0.5 * vx_raw + 0.5 * t.get("vx", 0.0)
                    vy = 0.5 * vy_raw + 0.5 * t.get("vy", 0.0)
                else:
                    vx, vy = t.get("vx", 0.0), t.get("vy", 0.0)

                # THESIS FIX (flicker): this used to snap origin_x/y to
                # the CURRENT position every time static_check_window
                # elapsed, to avoid comparing against a position from
                # minutes ago. But that reset also zeroes accumulated
                # displacement for a person who is continuously walking
                # - right after each reset, only one scan interval's
                # worth of movement exists, usually under
                # static_move_threshold, so publishing stops for a beat
                # and resumes once enough scans re-accumulate past the
                # threshold. Confirmed: this produced a ~1s-period
                # detect/lose flicker for a person walking the entire
                # time, not an actual static/moving ambiguity.
                #
                # Fix: keep a short rolling history and compare against
                # the position from ~static_check_window ago (sliding),
                # instead of snapping origin to "now" on a fixed clock.
                # A continuously-moving person then always has a
                # comparison point that's genuinely static_check_window
                # old, so displacement never artificially zeroes.
                history = t.get("history", [])
                history.append((now_s, x, y))
                # drop entries older than 2x the window - only need
                # enough back-history to find the oldest-still-relevant
                # sample, no need to keep growing forever
                cutoff = now_s - 2 * self.static_check_window
                history = [h for h in history if h[0] >= cutoff]

                # oldest sample still within the window = our comparison
                # origin. Falls back to the first-ever sample (track
                # just created) if the track is younger than the window.
                origin_t, origin_x, origin_y = history[0]
                for h in history:
                    if now_s - h[0] <= self.static_check_window:
                        break
                    origin_t, origin_x, origin_y = h

                self.tracks[track_id] = {
                    "x": x, "y": y, "last_seen": now_s,
                    "origin_x": origin_x, "origin_y": origin_y,
                    "origin_t": origin_t, "history": history,
                    "vx": vx, "vy": vy,
                    "vel_established": dt_pred > 1e-3,
                    # THESIS FIX (stationary person dropped as furniture):
                    # static_move_threshold below re-checks displacement
                    # EVERY scan to reject furniture/walls, but a real
                    # person who simply stops walking fails that same
                    # check and silently stops publishing - which then
                    # cascades into identity_fusion_node dropping their
                    # binding within LIDAR_TRACK_TIMEOUT (2s). Once a
                    # track has proven itself by moving >=
                    # static_move_threshold at least once, latch that
                    # and never re-apply the furniture filter to it
                    # again - furniture never earns "confirmed" in the
                    # first place, so it stays filtered forever, but a
                    # person who stops moving keeps publishing.
                    "confirmed": t.get("confirmed", False) or (
                        math.hypot(x - origin_x, y - origin_y)
                        >= self.static_move_threshold),
                }
                unmatched.remove(best_i)
                used_ids.add(track_id)

        # New tracks for unmatched clusters
        for i in unmatched:
            x, y = transformed[i]
            self.tracks[self.next_id] = {
                "x": x, "y": y, "last_seen": now_s,
                "origin_x": x, "origin_y": y, "origin_t": now_s,
                "history": [(now_s, x, y)],
                "vx": 0.0, "vy": 0.0,
                "confirmed": False,
            }
            used_ids.add(self.next_id)
            self.next_id += 1

        # Prune stale tracks
        for track_id in list(self.tracks.keys()):
            if now_s - self.tracks[track_id]["last_seen"] > self.track_timeout:
                del self.tracks[track_id]

        # ---- 6. Publish in the format human_kf_predictor.py expects ----
        # id,conf,base_x,base_y   (conf fixed at 1.0 — no classification confidence from LiDAR alone)
        published = 0
        published_ids = set()
        for track_id in used_ids:
            t = self.tracks.get(track_id)
            if t is None:
                continue

            moved = math.hypot(t["x"] - t["origin_x"], t["y"] - t["origin_y"])
            if not t.get("confirmed", False) and moved < self.static_move_threshold:
                continue  # never yet proven to move — likely furniture/wall

            out = String()
            out.data = f"{track_id},1.00,{t['x']:.3f},{t['y']:.3f}"
            self.pub.publish(out)
            published += 1
            published_ids.add(track_id)

        # ---- state-change logging ----
        is_publishing = published > 0

        if is_publishing and not self._was_publishing:
            self.get_logger().info(
                f"PERSON DETECTED — publishing started, id(s) {sorted(published_ids)} "
                f"({len(used_ids)} candidate(s) tracked total)")
        elif not is_publishing and self._was_publishing:
            gap_ms = ((now_s - self._last_publish_time) * 1000
                      if self._last_publish_time is not None else float('nan'))
            self.get_logger().warn(
                f"PERSON LOST — publishing stopped ({gap_ms:.0f}ms since the "
                f"actual last publish - may be far less than time since the "
                f"last log line, since only state CHANGES are logged). Still "
                f"tracking {len(used_ids)} candidate(s), none moving >= "
                f"{self.static_move_threshold}m in the last "
                f"{self.static_check_window}s.")
        elif is_publishing and published_ids != self._published_ids_prev:
            gained = published_ids - self._published_ids_prev
            lost = self._published_ids_prev - published_ids
            # Only warn about ids that were actually announced - an
            # un-announced clutter id dying silently is not interesting,
            # it never got a "New person" line in the first place.
            lost_announced = lost & self._announced_ids
            if lost_announced:
                self.get_logger().warn(
                    f"Person id(s) stopped publishing: {sorted(lost_announced)}")
            for tid in lost:
                self._publish_start_time.pop(tid, None)
                self._announced_ids.discard(tid)
            for tid in gained:
                self._publish_start_time[tid] = now_s

        # Persistence-gated announcement: check every cycle (not only on
        # a set-change) so an id that quietly crosses the min-age
        # threshold between "gained" events still gets announced.
        newly_announced = []
        for tid in published_ids:
            if tid in self._announced_ids:
                continue
            start = self._publish_start_time.get(tid, now_s)
            if now_s - start >= self._new_id_log_min_age:
                self._announced_ids.add(tid)
                newly_announced.append(tid)
        if newly_announced:
            self.get_logger().info(
                f"New person id(s) publishing: {sorted(newly_announced)}")

        if is_publishing:
            self._last_publish_time = now_s
        elif (self._last_heartbeat_time is None
              or now_s - self._last_heartbeat_time > self._heartbeat_every):
            # Idle heartbeat so a silent node still proves it's alive,
            # without repeating every second like the old DEBUG line.
            self.get_logger().info(
                f"(idle) {len(used_ids)} static/non-moving candidate(s) tracked, "
                f"nothing publishing")
            self._last_heartbeat_time = now_s

        self._was_publishing = is_publishing
        self._published_ids_prev = published_ids


def main(args=None):
    rclpy.init(args=args)
    node = LidarPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    # THESIS FIX: on some rclpy versions (observed: ROS 2 Jazzy),
    # rclpy.init() installs its own SIGINT handler that can already
    # shut the context down on Ctrl+C, before this line runs. Calling
    # rclpy.shutdown() again then raises RCLError ("rcl_shutdown
    # already called") - cosmetic (the node was already shutting down
    # correctly either way), but noisy. Guard with rclpy.ok().
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()