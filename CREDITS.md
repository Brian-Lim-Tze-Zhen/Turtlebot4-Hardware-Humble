# Credits and third-party software

This repository builds on the work of others. Numbers in brackets are the references of the thesis *Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding*.

## Included in this repository

| What | Where | Origin | License |
|---|---|---|---|
| YOLOv8n and YOLOv8n-pose, exported to OpenVINO | `humble_client/workspace/models/` | Ultralytics [38]; author and license in each `metadata.yaml` | AGPL-3.0 |
| Nav2 launch file with the collision monitor | `humble_client/workspace/launch/navigation_cm_launch.py` | `navigation_launch.py` of `nav2_bringup`, Intel Corporation; copyright header kept | Apache-2.0 |
| TurtleBot 4 Nav2 launch file with the collision monitor | `humble_client/workspace/launch/nav2_cm.launch.py` | `nav2.launch.py` of `turtlebot4_navigation`, Clearpath Robotics; copyright header kept | Apache-2.0 |
| Behaviour trees | `humble_client/workspace/behavior_trees/` | modified from the navigate-to-pose tree of Nav2 | Apache-2.0 |
| Nav2 parameter files | `humble_client/workspace/config/` | start from the TurtleBot 4 and Nav2 parameter files, with the thesis changes | Apache-2.0 |

## Used, not included

Installed by the Dockerfile or cloned during set-up.

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
