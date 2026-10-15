# Social Navigation on a real TurtleBot 4 (ROS 2 Humble)

Robot side of the master's thesis **"Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding"** (Deggendorf Institute of Technology, Campus Cham, 2026). The simulation, the results in simulation and the ablation are in the companion repository [Turtlebot4-Social-Navigation-Jazzy-Simulation-](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-).

The pipeline runs on a host PC in a Docker container and drives a physical TurtleBot 4 over Wi-Fi.

**Stack:** ROS 2 **Humble** · CycloneDDS (direct, no Discovery Server / zenoh) · AMCL + Nav2 MPPI · YOLO (Ultralytics/OpenVINO) · RPLiDAR · OAK-D. MobileCLIP-S1 is installed but not used ([HOW_TO_RUN.md](HOW_TO_RUN.md), section 5).

## Pipeline

![The final configuration from the camera to the wheels](docs/media/final_configuration.png)

*The final configuration from the camera to the wheels (thesis Figure 5.2, Section 5.3). Blue: existing components (sensors, detector, tracker, Nav2 and the base); orange: nodes developed in the thesis; green: the SocialCritic with the lane rules. The labelled arrows into the costmaps carry what these nodes write into them. On the robot the camera image is compressed and Nav2 runs under `/turtlebot4`. The ROS topics between the nodes, and the robot and simulation files side by side, are in [CONFIGURATIONS.md](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md) of the simulation repository.*

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

## Credits and third-party software

### Included in this repository

| What | Where | Origin | License |
|---|---|---|---|
| YOLOv8n and YOLOv8n-pose, exported to OpenVINO | `humble_client/workspace/models/` | Ultralytics [38]; author and license in each `metadata.yaml` | AGPL-3.0 |
| Nav2 launch file with the collision monitor | `humble_client/workspace/launch/navigation_cm_launch.py` | `navigation_launch.py` of `nav2_bringup`, Intel Corporation; copyright header kept | Apache-2.0 |
| TurtleBot 4 Nav2 launch file with the collision monitor | `humble_client/workspace/launch/nav2_cm.launch.py` | `nav2.launch.py` of `turtlebot4_navigation`, Clearpath Robotics; copyright header kept | Apache-2.0 |
| Behaviour trees | `humble_client/workspace/behavior_trees/` | modified from the navigate-to-pose tree of Nav2 | Apache-2.0 |
| Nav2 parameter files | `humble_client/workspace/config/` | start from the TurtleBot 4 and Nav2 parameter files, with the thesis changes | Apache-2.0 |

### Used, not included

Installed by the Dockerfile or cloned during set-up. Numbers in brackets are the references of the thesis.

| Software | Author | License | Used for |
|---|---|---|---|
| [ROS 2](https://docs.ros.org/) [20] | Open Robotics and contributors | Apache-2.0 | middleware |
| [Nav2](https://github.com/ros-navigation/navigation2) [5, 21] | Steve Macenski and contributors | Apache-2.0, BSD-3-Clause, LGPL-2.1 (AMCL) | navigation stack: costmaps, KeepoutFilter, NavFn, behaviour tree, collision monitor |
| [Nav2 MPPI controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller) [23, 24] | Steve Macenski, Aleksei Budyakov | MIT | the controller the SocialCritic plugs into (the critic builds on its `CriticFunction` interface) |
| [AMCL](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl) [27] | Nav2 contributors | LGPL-2.1-or-later | localisation |
| [nonpersistent_voxel_layer](https://github.com/SteveMacenski/nonpersistent_voxel_layer) [5, 21] | Steve Macenski | BSD | costmap layer that takes the person cloud |
| [TurtleBot 4 packages](https://github.com/turtlebot/turtlebot4) [70] | Clearpath Robotics | Apache-2.0 | robot description, bring-up and navigation launch files |
| [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) [38] | Glenn Jocher, Ayush Chaurasia, Jing Qiu | AGPL-3.0 | person and pose detection |
| [ByteTrack](https://github.com/ifzhang/ByteTrack) [46] | Yifu Zhang et al. | MIT | multi-object tracking (as built into Ultralytics) |
| [OpenVINO](https://github.com/openvinotoolkit/openvino) [78] | Intel | Apache-2.0 | YOLO inference on the CPU |
| [Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) [79] | Eclipse Foundation | EPL-2.0 or BSD-3-Clause (EDL-1.0) | middleware on the robot |
| [open_clip](https://github.com/mlfoundations/open_clip) with MobileCLIP-S1 | open_clip contributors; Apple (MobileCLIP) | MIT (open_clip); MobileCLIP code and weights under Apple's own licenses, see [ml-mobileclip](https://github.com/apple/ml-mobileclip) | installed, not used ([HOW_TO_RUN.md](HOW_TO_RUN.md), section 5) |
| [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) [28] | Steve Macenski | LGPL-2.1 | mapping, through `turtlebot4_navigation` ([HOW_TO_RUN.md](HOW_TO_RUN.md), section 3.2) |
