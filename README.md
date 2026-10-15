# Social Navigation on a real TurtleBot 4 (ROS 2 Humble)

Robot side of the master's thesis **"Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding"** (Deggendorf Institute of Technology, Campus Cham, 2026). The simulation, the results in simulation and the ablation are in the companion repository [Turtlebot4-Social-Navigation-Jazzy-Simulation-](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-).

The pipeline runs on a host PC in a Docker container and drives a physical TurtleBot 4 over Wi-Fi.

**Stack:** ROS 2 **Humble** · CycloneDDS (direct, no Discovery Server / zenoh) · AMCL + Nav2 MPPI · YOLO (Ultralytics/OpenVINO) · RPLiDAR · OAK-D. MobileCLIP-S1 is installed but not used ([HOW_TO_RUN.md](HOW_TO_RUN.md), section 5).

## Pipeline

```mermaid
flowchart TD
  cam[camera image]:::input --> yolo[Person detector<br/>YOLOv8n-pose and ByteTrack]
  yolo -- "/person_positions_map" --> ray[Ray localisation<br/>range along the camera bearing]
  in1[LiDAR scan, static map, odometry]:::input --> ray
  ray -- "/camera_ray_clusters" --> id[Identity node<br/>position continuity, stationary hold]
  in2[LiDAR scan, static map]:::input --> id
  id -- "/person_positions_fused" --> kf[Kalman predictor<br/>velocity and predicted position]
  in3[odometry]:::input --> kf
  kf -- "/predicted_person_positions" --> cloud[Person cloud node<br/>obstacle points per person]
  kf -- "/predicted_person_positions" --> grp[Group detector<br/>pairs and queues]
  id -- "/person_positions_fused" --> grp
  in4[camera image, LiDAR scan, static map]:::input --> grp
  grp -- "/social_groups" --> zone[Social-zone node<br/>graded zone around a group]
  grp -- "/social_groups" --> cloud
  kf -- "/predicted_person_positions" --> critic[SocialCritic<br/>in the MPPI controller]:::nav
  grp -- "/social_groups" --> critic
  zone -- "/social_zone_map" --> gc[Global costmap<br/>KeepoutFilter]:::nav
  critic -- "/social_critic/lane_block" --> gc
  cloud -- "/predicted_person_cloud" --> gc
  cloud -- "/predicted_person_cloud" --> lc[Local costmap]:::nav
  classDef default fill:#e8eef7,stroke:#2b4c7e,color:#1a2a40
  classDef input fill:#f2f2f2,stroke:#999,color:#333
  classDef nav fill:#e8f3e8,stroke:#3a7a3a,color:#1f3d1f
```

*Final robot configuration (12 October 2026). Blue: perception and group nodes, with the topics between them; grey: their inputs; green: the navigation stack. The SocialCritic reads only the predicted positions, which carry velocity, and the groups, so that members of a standing group get no lane. The groups also reach the zone node, whose mask the global costmap's KeepoutFilter applies, and the person cloud node, which leaves group members to the zone. The SocialCritic's lane block steers the global plan to one side. On the robot the camera image is compressed and Nav2 runs under `/turtlebot4`. The simulation runs the same wiring; matching files are listed in [CONFIGURATIONS.md](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md) of the simulation repository. Thesis Figure 3.5 (`docs/media/pipeline.png`) shows the perception chain only, and with `/person_positions_fused` as an input of the SocialCritic, which the final configuration no longer has.*

## Results on the robot

![Head-on walker on the real robot](docs/media/headon_robot.gif)

*Head-on walker, final robot configuration, 12 October 2026 (filmed extra run `headon_rec20gsvideo_run01`, not one of the ten counted runs). The robot keeps to its right and passes without stopping.*

![Standing pair on the real robot](docs/media/pair_robot.gif)

*Standing pair, same day: around the pair in the wide hallway (counted run 1) and between the members in the narrow section (counted run 3). Both played at 1.5x speed; faces blurred frame by frame.*

Final robot configuration, 12 October 2026, ten counted runs per case in the hallway of the university (thesis Section 7.6.2). On the robot there is no ground truth: the distance is the nearest LiDAR return that is not on the map, and the surface clearance is that distance minus the robot radius of 0.189 m. Mean ± SD in metres.

| Case | Nearest return | Surface clearance | Robot's path | Stops | Time (s) |
|---|---|---|---|---|---|
| Head-on walker | 0.669 ± 0.150 | 0.480 ± 0.150 | passed on the right 10 / 10 | 1 (turn at the start of one run) | 26.5 ± 1.1 |
| Standing pair, wide hallway | 0.766 ± 0.068 | 0.577 ± 0.068 | around the pair 10 / 10 | 0 | 25.6 ± 1.0 |
| Standing pair, narrow section | 0.556 ± 0.046 | 0.367 ± 0.046 | between the members 10 / 10 | 0 | 28.8 ± 0.9 |

Without group skip (9 October 2026) the robot reached similar distances, but a lane was started for a member of the standing pair in seven of ten runs in each place, because a standing member's estimate jumps at the hand-over between camera and LiDAR. With group skip a lane started in one of twenty runs, for a short extra track in front of the pair. Measured the same way in simulation, the wide pair matches the robot within 0.001 m. Photos and RViz views of one run of each case (faces blurred) are in the thesis, Figures 7.5 to 7.7.

## Quick start

```bash
git clone https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Hardware-Humble.git ~/Desktop/Turtlebot4
git clone https://github.com/SteveMacenski/nonpersistent_voxel_layer.git ~/Desktop/Turtlebot4/humble_client/workspace/src/nonpersistent_voxel_layer
xhost +local:root
cd ~/Desktop/Turtlebot4/humble_client && docker compose build && ./run_humble_client.sh

# inside the container
cd /root/thesis_social_navigation_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch launch/robot_day_all.launch.py          # the final configuration, everything except the recorder
```

Before building, change the Wi-Fi interface and subnet in the `Dockerfile` and the group IDs in `docker-compose.yaml` to match your machine ([HOW_TO_RUN.md](HOW_TO_RUN.md), section 1). If the robot's router has no internet access, sync the robot's clock first (section 2, step 7). Recorded runs are started from the laptop, not the container, with `workspace/scripts/run_scenario_hw_v4.sh`; a whole robot day step by step is in section 3.1.

## Documentation

| File | What it holds |
|---|---|
| [HOW_TO_RUN.md](HOW_TO_RUN.md) | requirements, set-up and time sync, the final configuration and its run commands, a robot day step by step, using another building, the earlier configurations compared in the thesis, folder structure, known problems |
| [CONFIGURATIONS.md](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md) (simulation repository) | what the parts of the configuration names mean, and the robot and simulation files side by side |
| [humble_client/workspace/scripts/README.md](humble_client/workspace/scripts/README.md) | which scripts the final configuration uses, and which are earlier versions |
| [humble_client/Task/](humble_client/Task/) | run records of the robot days (5, 8, 9 and 12 October 2026): every counted and discarded run with its reason |
| [humble_client/Turtlebot4_Documentation.md](humble_client/Turtlebot4_Documentation.md) | hardware notes and issue log |
