# Social Navigation on a real TurtleBot 4 (ROS 2 Humble)

Robot side of the master's thesis *Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding* (Deggendorf Institute of Technology, Campus Cham, 2026). The simulation, its results and the ablation are in the [simulation repository](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-).

The pipeline runs on a laptop in a Docker container and drives a physical TurtleBot 4 over Wi-Fi. **Stack:** ROS 2 Humble · Cyclone DDS · AMCL and Nav2 with the MPPI controller · YOLO (Ultralytics, OpenVINO) · RPLiDAR · OAK-D. MobileCLIP-S1 is installed but [not used](HOW_TO_RUN.md#5-known-problems).

**Contents:** [Results](#results) · [Pipeline](#pipeline) · [Analysis](#analysis) · [Quick start](#quick-start) · [Documentation](#documentation) · [References](#references) · [Citation](#citation)

## Results

![Head-on walker on the real robot](docs/media/headon_robot.gif)

*Head-on walker, final robot configuration, 12 October 2026 (a filmed extra run, not one of the ten counted runs). The robot keeps to its right and passes without stopping.*

![Standing pair on the real robot](docs/media/pair_robot.gif)

*Standing pair, same day: around the pair in the wide hallway (counted run 1) and between the members in the narrow section (counted run 3). Both played at 1.5x speed; faces blurred frame by frame.*

Final robot configuration, 12 October 2026, ten counted runs per case in the hallway of the university (thesis Section 7.6.2). On the robot there is no ground truth: the distance is the nearest LiDAR return that is not on the map, and the surface clearance is that distance minus the robot radius of 0.189 m. Mean ± SD in metres.

| Case | Nearest return | Surface clearance | Robot's path | Stops | Time (s) |
|---|---|---|---|---|---|
| Head-on walker | 0.669 ± 0.150 | 0.480 ± 0.150 | passed on the right 10 / 10 | 1 (turn at the start of one run) | 26.5 ± 1.1 |
| Standing pair, wide hallway | 0.766 ± 0.068 | 0.577 ± 0.068 | around the pair 10 / 10 | 0 | 25.6 ± 1.0 |
| Standing pair, narrow section | 0.556 ± 0.046 | 0.367 ± 0.046 | between the members 10 / 10 | 0 | 28.8 ± 0.9 |

Every counted and discarded run, with its reason, is in the [run record of 12 October](humble_client/Task/HARDWARE_DAY_20261012.md). Photos and RViz views of one run of each case (faces blurred) are in the thesis, Figures 7.5 to 7.7.

## Pipeline

![The final configuration from the camera to the wheels](docs/media/final_configuration.png)

*The final configuration from the camera to the wheels (thesis Figure 5.2). Blue: existing components (sensors, detector, tracker, Nav2 and the base); orange: nodes developed in the thesis; green: the SocialCritic with the lane rules. The labelled arrows into the costmaps carry what these nodes write into them. On the robot the camera image is compressed and Nav2 runs under `/turtlebot4`. The [ROS topics between the nodes](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md#topics-of-the-final-configuration) and the [robot and simulation files side by side](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md#simulation-vs-robot) are listed in the simulation repository.*

## Analysis

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/media/results_dark.png">
  <img alt="Left: closest distance between centres in ten head-on trials in simulation for the baseline, the lane rules and the final configuration. Right: nearest LiDAR return not on the map for the final configuration in simulation and on the robot, head-on walker, wide pair and narrow pair." src="docs/media/results_light.png">
</picture>

*Dots: single runs; bars: means. Left: the head-on walker in simulation, closest distance between centres from Gazebo's ground truth. Right: the final configuration on the robot and in simulation, measured the same way on both, as the nearest LiDAR return that is not on the map. Robot data from the [comparison of the robot days](humble_client/workspace/pytest/analysis/side_20261009/); simulation data and the chart script in the [simulation repository](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-#analysis).*

- **The standing pair matches between robot and simulation.** Wide pair: 0.766 m on the robot, 0.765 m in simulation. Narrow pair: 0.56 m on the robot, 0.60 m in simulation.
- **The head-on walker varies more on the robot.** 0.67 ± 0.15 m on the robot, against 0.54 ± 0.07 m in simulation.
- **Group skip stops lanes for standing pair members.** Without it (9 October 2026), a lane was started for a member of the standing pair in seven of ten runs in each place, because a standing member's estimate jumps at the hand-over between camera and LiDAR. With group skip, a lane started in one of twenty runs, for a short extra track in front of the pair. The distances were similar on both days.

## Quick start

You need a laptop with Ubuntu, Docker and Docker Compose and Intel graphics, on the same Wi-Fi as a TurtleBot 4 running ROS 2 Humble. The [requirements](HOW_TO_RUN.md#1-requirements) list the details.

**1. Get the code and the voxel layer.** This downloads the repository, with the robot workspace in `humble_client/workspace`, and the source of the nonpersistent voxel layer into that workspace. The Humble image does not install the voxel layer as a package, so it is built from source with the workspace.

```bash
git clone https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Hardware-Humble.git ~/Desktop/Turtlebot4
git clone https://github.com/SteveMacenski/nonpersistent_voxel_layer.git ~/Desktop/Turtlebot4/humble_client/workspace/src/nonpersistent_voxel_layer
```

**2. Set the values of your machine.** Three things are written for the author's laptop; change them before building (see the [requirements](HOW_TO_RUN.md#1-requirements)):

- the Wi-Fi interface `wlp0s20f3` in the `Dockerfile` (Cyclone DDS settings), from `ip link`;
- the robot's subnet `192.168.50.0/24` in the `Dockerfile` (time server);
- the group IDs `44` and `992` in `docker-compose.yaml`, from `getent group video render`.

**3. Build and start the container.** The build installs ROS 2 Humble, Nav2, the TurtleBot 4 packages, Cyclone DDS, a time server for the robot, and the Python environment for YOLO. It is needed once. `up -d` starts the container `thesis_social_nav_real` in the background, on the host network so that it reaches the robot, with the workspace mounted. `xhost` lets RViz open on your screen.

```bash
cd ~/Desktop/Turtlebot4/humble_client
docker compose build
docker compose up -d
xhost +local:root
```

**4. Open a shell in the container and build the workspace.** `colcon build` compiles the SocialCritic plugins and the voxel layer; sourcing `install/setup.bash` makes them visible in this shell. Then check that the robot's topics arrive.

```bash
docker exec -it thesis_social_nav_real bash
cd /root/thesis_social_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 topic list | grep turtlebot4          # /turtlebot4/scan, /turtlebot4/tf, ...
```

If the robot's router has no internet access, [sync the robot's clock](HOW_TO_RUN.md#2-set-up-from-scratch) now.

**5. Start the final configuration with one command.** This starts Nav2 with localisation on the hallway map, the social-zone stack and the group detector, the perception pipeline, and the hold-and-beep and goal-retry nodes, and opens RViz. Everything except the recorder.

```bash
ros2 launch launch/robot_day_all.launch.py
```

Recorded runs are started from the laptop, not the container, one command per run; the whole procedure, from switching the robot on to checking a bag, is described in [a robot day, step by step](HOW_TO_RUN.md#31-a-robot-day-step-by-step).

## Documentation

| Document | What it holds |
|---|---|
| [How to run on the robot](HOW_TO_RUN.md) | requirements, set-up and time sync, the final configuration and its run commands, a robot day step by step, using another building, the earlier configurations compared in the thesis, folder structure, known problems |
| [Configurations](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-/blob/main/docs/CONFIGURATIONS.md) | what the parts of the configuration names mean, the ROS topics, and the robot and simulation files side by side (in the simulation repository) |
| [Scripts](humble_client/workspace/scripts/README.md) | which scripts the final configuration uses, and which are earlier versions |
| [Run records](humble_client/Task/) | the robot days of 5, 8, 9 and 12 October 2026: every counted and discarded run with its reason |
| [Hardware notes](humble_client/Turtlebot4_Documentation.md) | hardware notes and issue log |

## References

This work builds on [ROS 2](https://docs.ros.org/), [Nav2](https://github.com/ros-navigation/navigation2) and its [MPPI controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller), the [nonpersistent voxel layer](https://github.com/SteveMacenski/nonpersistent_voxel_layer), the [TurtleBot 4 packages](https://github.com/turtlebot/turtlebot4), [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics), [ByteTrack](https://github.com/ifzhang/ByteTrack), [OpenVINO](https://github.com/openvinotoolkit/openvino) and [Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds). It includes the YOLOv8 models of Ultralytics and copies of a few launch files. Authors, licenses and the matching references of the thesis: [credits and licenses](CREDITS.md).

## Citation

If you use this work, please cite the thesis:

```bibtex
@mastersthesis{limtzezhen2026socialnav,
  author = {{Brian Lim Tze Zhen}},
  title  = {Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding},
  school = {Deggendorf Institute of Technology, Campus Cham},
  type   = {Master's thesis (M.Eng.)},
  year   = {2026}
}
```
