# How to run on the robot

Details behind the [README](README.md): set-up, the final configuration and how its runs were recorded, the earlier configurations compared in the thesis, the folder structure and known problems.

Contents: 1. Requirements · 2. Set-up · 3. Final configuration (3.1 a robot day step by step, 3.2 another building or map, 3.3 earlier configurations) · 4. Folder structure · 5. Known problems · 6. Recorded data

What the configuration names mean (`hwreq`, `approach`, `tf05`, `rec20`, `noghost`, `groupskip`, ...), and which robot file matches which simulation file, is in [CONFIGURATIONS.md](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md) of the simulation repository.

## 1. Requirements

Three machines are involved, and every command below says where it runs:

| Name used here | What it is |
|---|---|
| **laptop** | the host PC that runs Docker and RViz; it is on the robot's Wi-Fi |
| **container** | the Docker container on the laptop (`humble_client`), where Nav2 and the perception nodes run |
| **robot** | the TurtleBot 4: a Raspberry Pi with the LiDAR and the OAK-D camera, on an iRobot Create 3 base |
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

**4. Build image** (first time takes a while; it also downloads MobileCLIP-S1, which is not used, see below)
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

**7. Time sync (only when the robot's router has no internet access)**

The robot's Pi sets its clock from internet time servers. When the router has no internet access, the Pi's clock drifts from the laptop's; RViz then shows no **2D Pose Estimate** and TF lookups fail. The container runs a chrony time server for the robot's subnet (`Dockerfile`, step 6b, started from `.bashrc`); point the Pi at it.

On the robot, once (`<LAPTOP_IP>` is the laptop's address on the robot's Wi-Fi, from `ip addr show | grep inet` on the laptop):
```bash
sudo bash -c 'echo "server <LAPTOP_IP> iburst" >> /etc/chrony/chrony.conf'
sudo systemctl restart chronyd
```

On the robot, at the start of a robot day and after the Pi has been off:
```bash
sudo systemctl stop chronyd
sudo chronyd -q 'server <LAPTOP_IP> iburst'
sudo systemctl start chronyd
sudo chronyc makestep
chronyc sources -v          # the laptop must be listed and selected (marked *)
sudo systemctl restart turtlebot4.service
```

If the Pi still reports `No suitable source`, the chrony server has no time source of its own (no internet), and an unsynchronised server serves no clients. In the container, add `local stratum 10` to `/etc/chrony/chrony.conf` and run `service chrony restart` (seen on 12 October 2026, `humble_client/Task/HARDWARE_DAY_20261012.md`).

With internet access on the router, skip this step: the Pi and the laptop both use internet time.

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

Inside the container (after the time sync of section 2, step 7, if the router has no internet access):

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

### 3.1 A robot day, step by step

The order used on the robot days. The record of each day is in `humble_client/Task/HARDWARE_DAY_<date>.md`.

1. **Robot:** switch it on and wait until it is on the Wi-Fi.
2. **Laptop:** `xhost +local:root`, then `cd ~/Desktop/Turtlebot4/humble_client && ./run_humble_client.sh`.
3. **Container:** check the link: `ros2 topic list | grep turtlebot4` and `ros2 topic hz /turtlebot4/scan` (section 2).
4. **Robot:** time sync, only if the router has no internet access (section 2, step 7).
5. **Container:** start the stack: `ros2 launch launch/robot_day_all.launch.py` (section 3). RViz opens on the laptop.
6. **RViz:** if the robot is not where the map shows it, set its pose with **2D Pose Estimate**. Send it to the start with **Nav2 Goal**. The starts used are in the day records (head-on start about (-6.88, 13.41), heading 140°; wide pair (-6.44, 13.69), 137°).
7. **Container:** with the robot standing still, check the localisation and correct it once if needed:
   ```bash
   python3 scripts/check_localization.py --fix --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
   python3 scripts/check_localization.py --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
   ```
   The run script repeats this check and does not start a run unless the heading is within 1° and the position within 0.10 m.
8. **People:** the walker or the pair take their places. For the pair, the run script waits until the group detector reports a `conversation` group for 3 s (two high beeps).
9. **Laptop** (not the container; the script calls `docker exec` itself): one recorded run per command, as in section 3, for example `SCEN=pairwide CFG=... scripts/run_scenario_hw_v4.sh rec20gs 1`. It refuses a run number that already has a bag and prints the next free one.
10. **After the run:** the script prints the run's metrics. Ctrl+C during a run stops the recorder, cancels the goal and renames the bag `VOID_...`. Note every run, counted or not, with its reason in the day record.

### 3.2 Another building or map

The maps in `workspace/map/` and the goals in the run script are for the university hallway (`map/hall_map_name.yaml`). Elsewhere:

1. **Make a map** (container): start SLAM, drive the robot round with the keyboard, and save the map (commands from `humble_client/Turtlebot4_Documentation.md`):
   ```bash
   ros2 launch turtlebot4_navigation slam.launch.py sync:=false namespace:=/turtlebot4
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtlebot4/cmd_vel
   ros2 run nav2_map_server map_saver_cli -f map/<name> --ros-args -p map_subscribe_transient_local:=true -r __ns:=/turtlebot4
   ```
2. **Use it:** `ros2 launch launch/robot_day_all.launch.py map:=$PWD/map/<name>.yaml`, and pass the same file to `check_localization.py --map map/<name>.yaml`.
3. **Set the goal of a run:** the run script's goal is set with `GX GY GQZ GQW` (default: the far end of the hallway, (-12.20, 17.75)); read new values off the map in RViz.
4. **Check the wide/narrow test:** the group detector measures the free space beside a pair on the static map, so a pair counts as narrow or wide by the walls of your map.

### 3.3 Earlier robot configurations compared in the thesis

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
- **MobileCLIP is installed but not used.** The image installs `open_clip_torch` and downloads MobileCLIP-S1 (`Dockerfile`, step 3b) only because `scripts/social_group_detector_node_lidarhold.py` imports `open_clip`. Its VLM fallback never runs: every case in which the gaze check has no answer is a missing-keypoints case, which skips the fallback. The facing decision on the robot is the gaze check alone; the rejected VLM test is in `workspace/debug_crops_vlm_report/`. The code is kept as it ran, so that it matches the checksums saved with the bags.

## 6. Recorded data
Rosbags are stored as `workspace/bags/<scenario>_<condition>_runNN_<time>/`, each with the Nav2 file that was used, the launch files, the running commands and md5 sums of the critic library. They are not in the repository (size, and the camera images show people); they are archived with the thesis data.
