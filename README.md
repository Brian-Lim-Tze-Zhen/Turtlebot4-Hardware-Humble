# Turtlebot4-hardware

Real-robot side of the thesis **"Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding"** (Deggendorf Institute of Technology).
Runs the social navigation pipeline on a physical TurtleBot4 from a Docker container on the host PC.

**Stack:** ROS 2 **Humble** · CycloneDDS (direct, no Discovery Server / zenoh) · AMCL + Nav2 MPPI · YOLO (Ultralytics/OpenVINO) + MobileCLIP-S1 · RPLiDAR · OAK-D

## Pipeline
```
OAK-D ─► YOLO ─► camera_ray_person_node ─► camera_ray_identity_node ─┐
RPLiDAR /scan ──────────────────────────────────────────────────────┘
      ─► Kalman filter predictor ─► predicted person cloud ─► Nav2 costmap ─► MPPI (+ SocialCritic)
      ─► social group detector ─► social group cloud / zone layer
```
- Default launch: `launch/perception_camray_bringup.launch.py` (camera-primary)
- LiDAR-primary launch kept for ablation comparison

## Requirements
| Item | Value |
|---|---|
| Host OS | Ubuntu 22.04 / 24.04 with Docker + Docker Compose |
| GPU | Intel iGPU (Mesa `iris` driver) for RViz |
| Robot | TurtleBot4 (Pi + Create3) on **ROS 2 Humble** |
| Network | Host and robot on same Wi-Fi subnet `192.168.50.0/24` |
| DDS | `ROS_DOMAIN_ID=1`, `rmw_cyclonedds_cpp` |

> ⚠️ **Hard-coded values — change these to match your machine:**
> - Wi-Fi interface `wlp0s20f3` in `Dockerfile` (CycloneDDS config, step 7) — check yours with `ip link`
> - Chrony subnet `192.168.50.0/24` in `Dockerfile` (step 6b)
> - Docker `group_add` IDs `44` (video) / `992` (render) in `docker-compose.yaml` — check with `getent group video render`

## Structure
```
humble_client/
├── Dockerfile / docker-compose.yaml   # container setup
├── run_humble_client.sh               # container launch script
├── models/                            # detection models
├── Turtlebot4_Documentation           # hardware notes
└── workspace/                         # mounted to /root/thesis_social_navigation_ws
    ├── src/                           # ROS 2 packages (social_critic, nonpersistent_voxel_layer, ...)
    ├── scripts/                       # perception / prediction nodes, analysis scripts
    ├── launch/                        # bringup launch files
    ├── config/                        # Nav2 / social nav configs
    ├── behavior_trees/                # navigate_to_pose_fast_replan.xml
    ├── map/                           # saved maps
    └── test_data/                     # sample test images
```

## Setup (from scratch)

**1. Clone**
```bash
git clone https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-hardware.git ~/Desktop/Turtlebot4
```

**2. Clone dependency**
```bash
git clone https://github.com/SteveMacenski/nonpersistent_voxel_layer.git ~/Desktop/Turtlebot4/humble_client/workspace/src/nonpersistent_voxel_layer
```

**3. Allow container to open GUI windows**
```bash
xhost +local:root
```

**4. Build image** (first time takes a while — MobileCLIP is downloaded at build)
```bash
cd ~/Desktop/Turtlebot4/humble_client && docker compose build
```

**5. Start container**
```bash
cd ~/Desktop/Turtlebot4/humble_client && ./run_humble_client.sh
```

**6. Build workspace** (inside container)
```bash
cd /root/thesis_social_navigation_ws && colcon build --symlink-install
```

## Verify connection to robot
Inside container:
```bash
ros2 topic list | grep turtlebot4
```
Should show `/turtlebot4/scan`, `/turtlebot4/tf`, etc. If empty → check Wi-Fi interface name, `ROS_DOMAIN_ID`, and that robot is on.

```bash
ros2 topic hz /turtlebot4/scan
```

## Notes / known gotchas
- Nav2 runs **namespaced under `/turtlebot4`** — standalone nodes need `/tf → /turtlebot4/tf` and `/tf_static → /turtlebot4/tf_static` remaps.
- Container runs **chrony as NTP server** for the Pi (started from `.bashrc`) — robot clock must sync or TF lookups fail.
- YOLO runs in venv `/root/venv`; `numpy==1.26.4` and `setuptools==70.0.0` are pinned on purpose — don't upgrade.
- Custom nodes are started with `python3 <script>.py`, not `ros2 run`.

## Recorded data
Rosbags are stored as `bags/headon_real_<timestamp>/`, each with `config_used.yaml`, node snapshots and md5sums for reproducibility. (Not in repo — size.)
