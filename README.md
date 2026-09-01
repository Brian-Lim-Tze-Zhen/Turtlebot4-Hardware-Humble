# Turtlebot4-hardware

Physical TurtleBot4 hardware bring-up and configuration — ROS 2 Jazzy, Discovery Server networking, AMCL localization, and sim-to-real deployment for a social navigation thesis pipeline.

## Overview
This repo contains the code and configuration used to run the social navigation pipeline (YOLOv8 → tracking → Kalman filter prediction → Nav2 MPPI) on a physical TurtleBot4.

## Structure
```
humble_client/
├── Dockerfile / docker-compose.yaml   # container setup
├── run_humble_client.sh               # container launch script
├── models/                            # detection models
├── Turtlebot4_Documentation           # hardware notes
└── workspace/
    ├── src/                           # ROS 2 packages (social_critic, etc.)
    ├── scripts/                       # utility/test scripts
    ├── launch/                        # bringup launch files
    ├── config/                        # Nav2 / social nav configs
    ├── behavior_trees/                # custom BT XML
    ├── map/                           # saved maps
    └── test_data/                     # sample test images
```

## Dependencies
Clone into `humble_client/workspace/`:
```
git clone https://github.com/turtlebot/turtlebot4_desktop.git humble_client/workspace/turtlebot4_desktop
git clone https://github.com/SteveMacenski/nonpersistent_voxel_layer.git humble_client/workspace/src/nonpersistent_voxel_layer
```

## Usage
```
./humble_client/run_humble_client.sh
```
