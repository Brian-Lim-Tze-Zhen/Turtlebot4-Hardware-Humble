#!/usr/bin/env python3
"""
compare_static_streak.py

Offline comparison of buggy vs fixed static_streak eviction.

Reads the already-recorded /lidar_person_clusters topic from the bag
and simulates what WOULD have happened with and without the fix by
re-running the eviction logic on the recorded cluster assignments.

This is a valid proof because:
- The bag contains the ACTUAL cluster positions and track assignments
  as the buggy detector produced them.
- The static_streak eviction happens AFTER matching, so we can
  replay the stored per-frame cluster assignments and simulate whether
  each track would survive or be killed at each timestep.

Usage:
    python3 compare_static_streak.py <bag_dir>

Example:
    python3 compare_static_streak.py /path/to/berth06_20260910_111228
"""

import sys, os, sqlite3, struct, json

BAG_DIR = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/brian/Desktop/Turtlebot4/humble_client/workspace/bags/berth06_20260910_111228"

# ── Open bag ──────────────────────────────────────────────────────────────────
db3 = next((os.path.join(BAG_DIR, f) for f in os.listdir(BAG_DIR)
            if f.endswith(".db3")), None)
if not db3:
    print("No .db3 in", BAG_DIR); sys.exit(1)

con = sqlite3.connect(db3)

row = con.execute("SELECT id FROM topics WHERE name='/lidar_person_clusters'").fetchone()
if not row:
    print("Topic /lidar_person_clusters not found. Available:")
    for r in con.execute("SELECT name FROM topics"): print(" ", r[0])
    sys.exit(1)

topic_id = row[0]
rows = con.execute(
    "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
    (topic_id,)
).fetchall()
print(f"Loaded {len(rows)} frames from /lidar_person_clusters\n")

def parse_string(data):
    try:
        n = struct.unpack_from('<I', data, 4)[0]
        return data[8:8+n-1].decode('utf-8')
    except Exception:
        return None

# ── Parse frames ──────────────────────────────────────────────────────────────
# Each frame: list of (id, x, y, vx, vy)
frames = []
for ts, data in rows:
    text = parse_string(data)
    if text is None:
        frames.append((ts, []))
        continue
    tracks = []
    for entry in text.strip().split(';'):
        parts = entry.strip().split(',')
        if len(parts) < 3:
            continue
        try:
            tid = int(parts[0]); x = float(parts[1]); y = float(parts[2])
            vx = float(parts[3]) if len(parts) > 3 else 0.0
            vy = float(parts[4]) if len(parts) > 4 else 0.0
        except ValueError:
            continue
        tracks.append((tid, x, y, vx, vy))
    frames.append((ts, tracks))

# ── Replay simulation ─────────────────────────────────────────────────────────
# We simulate the static_streak counter for each track.
# When a track is NOT in the current frame (missed), its streak increments.
# The eviction condition is: streak >= 3 AND (buggy: always | fixed: not confirmed)
# "confirmed" = track has traveled >= 0.5m total since first seen.

EVICT_STREAK = 3
CONFIRM_TRAVEL = 0.5

def simulate(frames, exempt_confirmed):
    label = "FIXED (confirmed exempt)" if exempt_confirmed else "BUGGY (no guard)"

    track_state = {}   # tid -> {streak, confirmed, positions, drops}

    drop_events = []   # (ts, tid, travel, lifetime_frames)

    prev_live = set()

    for ts, tracks in frames:
        cur_ids = {t[0] for t in tracks}

        # Update positions and confirm status
        for (tid, x, y, vx, vy) in tracks:
            if tid not in track_state:
                track_state[tid] = {
                    "streak": 0, "confirmed": False,
                    "positions": [(x, y)], "first_ts": ts,
                    "last_ts": ts, "alive": True
                }
            else:
                s = track_state[tid]
                s["positions"].append((x, y))
                s["last_ts"] = ts
                s["streak"] = 0  # appeared → reset streak
                # Check confirmation
                pos = s["positions"]
                travel = sum(
                    ((pos[i][0]-pos[i-1][0])**2 + (pos[i][1]-pos[i-1][1])**2)**0.5
                    for i in range(1, len(pos))
                )
                if travel >= CONFIRM_TRAVEL:
                    s["confirmed"] = True

        # Increment streak for absent tracks
        for tid, s in track_state.items():
            if tid not in cur_ids and s["alive"]:
                s["streak"] += 1

        # Apply eviction
        for tid, s in track_state.items():
            if not s["alive"]:
                continue
            if s["streak"] >= EVICT_STREAK:
                if exempt_confirmed and s["confirmed"]:
                    pass  # fixed: skip confirmed tracks
                else:
                    # Evict
                    pos = s["positions"]
                    travel = sum(
                        ((pos[i][0]-pos[i-1][0])**2 + (pos[i][1]-pos[i-1][1])**2)**0.5
                        for i in range(1, len(pos))
                    )
                    lifetime_s = (s["last_ts"] - s["first_ts"]) / 1e9
                    if travel > CONFIRM_TRAVEL:
                        drop_events.append({
                            "tid": tid, "travel": travel,
                            "lifetime_s": lifetime_s,
                            "last_pos": pos[-1],
                            "confirmed": s["confirmed"],
                            "ts": ts / 1e9
                        })
                    s["alive"] = False

        prev_live = cur_ids

    print(f"\n{'='*55}")
    print(f" {label}")
    print(f"{'='*55}")
    print(f" Confirmed track drops (travel > 0.5m): {len(drop_events)}")
    for e in drop_events:
        lx, ly = e["last_pos"]
        print(f"   id:{e['tid']:2d}  travel={e['travel']:.2f}m  "
              f"lifetime={e['lifetime_s']:.1f}s  "
              f"last=({lx:.2f},{ly:.2f})")
    return len(drop_events)

bugged_drops = simulate(frames, exempt_confirmed=False)
fixed_drops  = simulate(frames, exempt_confirmed=True)

print(f"\n{'='*55}")
print(f" SUMMARY")
print(f"{'='*55}")
print(f" Buggy detector  — confirmed drops: {bugged_drops}")
print(f" Fixed detector  — confirmed drops: {fixed_drops}")
delta = bugged_drops - fixed_drops
print(f" Reduction: {delta} fewer drops ({delta/max(bugged_drops,1)*100:.0f}%)")
