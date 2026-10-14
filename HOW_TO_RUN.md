# How to run on the robot

Details behind the [README](README.md): set-up, the final configuration and how its runs were recorded, the earlier configurations compared in the thesis, the folder structure and known problems.

Contents: 1. Requirements · 2. Set-up · 3. Final configuration · 4. Folder structure · 5. Known problems · 6. Recorded data

## 1. Requirements
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

## 2. Set-up (from scratch)

**1. Clone**
```bash
git clone https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Hardware-Humble.git ~/Desktop/Turtlebot4
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

### Check the connection to the robot
Inside container:
```bash
ros2 topic list | grep turtlebot4
```
Should show `/turtlebot4/scan`, `/turtlebot4/tf`, etc. If empty → check Wi-Fi interface name, `ROS_DOMAIN_ID`, and that robot is on.

```bash
ros2 topic hz /turtlebot4/scan
```

## 3. Final configuration

The thesis results on the robot (Sections 7.5 and 7.6) use one Nav2 file and one launch file:

| | File |
|---|---|
| Nav2 configuration | `workspace/config/social_nav2_headon_F_hwreq_block_approach_tf05_rec20_noghost_groupskip_hw.yaml` |
| Everything except the recorder | `workspace/launch/robot_day_all.launch.py` (Nav2 + localisation, zone stack, perception, hold and beep nodes) |
| One recorded run | `workspace/scripts/run_scenario_hw_v4.sh` (on the laptop) |
| Critic | `workspace/src/social_critic_lane_noghost` |

Inside the container, after the time sync (see Setup):

```bash
source /opt/ros/humble/setup.bash
cd /root/thesis_social_navigation_ws && source install/setup.bash
ros2 launch launch/robot_day_all.launch.py
```

On the laptop, one recorded run per command (the robot must stand at the start; the header of the script lists all options):

```bash
cd ~/Desktop/Turtlebot4/humble_client/workspace
CFG=config/social_nav2_headon_F_hwreq_block_approach_tf05_rec20_noghost_groupskip_hw.yaml \
    scripts/run_scenario_hw_v4.sh rec20gs 1                  # head-on walker
SCEN=pairwide   CFG=... scripts/run_scenario_hw_v4.sh rec20gs 1   # standing pair, wide hallway
SCEN=pairnarrow CFG=... scripts/run_scenario_hw_v4.sh rec20gs 1   # standing pair, narrow corridor
```

The run record of that day, with every counted and discarded run and its reason, is `humble_client/Task/HARDWARE_DAY_20261012.md`.

### 3.1 Earlier robot configurations compared in the thesis

| Date | What changed | Nav2 file in `workspace/config/` | Robot sets (bag names) |
|---|---|---|---|
| 5 Oct 2026 | person cloud with the social zone, before the lane rules | `social_nav2_ablation_F_socialzone_keepout.yaml` (and `_vxmin0`, `_cloudoff`) | `headon_cloudon`, `headon_cloudonvx0` |
| 5 Oct 2026 | lane rules of the simulation ported | `social_nav2_headon_F_hwreq_block_confirm_goalside_hw.yaml` | `headon_lanerule`, `pairwide_hwdetmap`, `pairnarrow_hwdetmap` |
| 8 Oct 2026 | simulation version of the critic ported | `social_nav2_headon_F_hwreq_block_approach_hw.yaml`, `…_approach_tf05_hw.yaml` | `headon_v4front`, `headon_v4noghost`, `headon_v4noghosttf05` |
| 9 Oct 2026 | final configuration without group skip | `social_nav2_headon_F_hwreq_block_approach_tf05_rec20_noghost_hw.yaml` | `headon_rec20noghost`, `pairwide_rec20noghost`, `pairnarrow_rec20noghost` |
| 12 Oct 2026 | final configuration with group skip | `…_tf05_rec20_noghost_groupskip_hw.yaml` | `headon_rec20gs`, `pairwide_rec20gs`, `pairnarrow_rec20gs` |

The comparison across these days is computed by `workspace/pytest/analysis/side_20261009/compare_days.py` and `compare_days_precise.py` (results in `*_result.txt` next to them). Run records: `humble_client/Task/HARDWARE_DAY_20261005.md`, `_20261008.md`, `_20261009.md`, `_20261012.md`.

## 4. Folder structure
```
humble_client/
├── Dockerfile / docker-compose.yaml   container set-up
├── run_humble_client.sh               starts the container
├── Turtlebot4_Documentation.md        hardware notes and issue log (cited in the thesis)
├── Task/                              run records of the robot days (5, 8, 9 and 12 October 2026) and the notes of 9 September
└── workspace/                         mounted as /root/thesis_social_navigation_ws
    ├── src/                           critic plugins: social_critic (no lanes), social_critic_lane,
    │                                  social_critic_lane_hold, social_critic_lane_noghost (final)
    ├── launch/                        bring-up launch files; robot_day_all.launch.py starts the final stack
    ├── config/                        Nav2 files of the final and the compared configurations, localisation
    ├── scripts/                       perception, prediction, group and zone nodes, hold/beep nodes, run scripts
    ├── behavior_trees/  map/  models/ behaviour trees, maps of the lab and the hallway, OpenVINO models
    ├── pytest/analysis/               analysis of the robot runs (side_20261009/: comparison across days, thesis Tables 7.9 and 7.10)
    ├── debug_crops_labels/            labels and script of the facing classifier (Table 6.1); the image crops are not published
    └── debug_crops_vlm_report/        results of the rejected VLM facing test
```

## 5. Known problems
- Nav2 runs **namespaced under `/turtlebot4`** — standalone nodes need `/tf → /turtlebot4/tf` and `/tf_static → /turtlebot4/tf_static` remaps.
- Container runs **chrony as NTP server** for the Pi (started from `.bashrc`) — robot clock must sync or TF lookups fail.
- YOLO runs in venv `/root/venv`; `numpy==1.26.4` and `setuptools==70.0.0` are pinned on purpose — don't upgrade.
- Custom nodes are started with `python3 <script>.py`, not `ros2 run`.

## 6. Recorded data
Rosbags are stored as `workspace/bags/<scenario>_<condition>_runNN_<time>/`, each with the Nav2 file that was used, the launch files, the running commands and md5 sums of the critic library. They are not in the repository (size, and the camera images show people); they are archived with the thesis data.
