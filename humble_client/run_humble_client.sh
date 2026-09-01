#!/bin/bash
xhost +local:docker >/dev/null 2>&1

docker run -it --rm \
  --net=host \
  --name humble-client \
  -e ROS_DOMAIN_ID=2 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>' \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop \
  bash -c "apt-get update -qq && apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp ros-humble-turtlebot4-description ros-humble-irobot-create-description ros-humble-turtlebot4-viz && bash"