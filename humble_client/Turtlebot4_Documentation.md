source /etc/turtlebot4/setup.bash

ros2 launch turtlebot4_navigation localization.launch.py map:=/root/thesis_social_navigation_ws/map/map_name.yaml namespace:=/turtlebot4

ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/turtlebot4

ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/turtlebot4

ros2 launch turtlebot4_navigation slam.launch.py sync:=false namespace:=/turtlebot4

ros2 launch /root/thesis_social_navigation_ws/launch/lab1_bringup.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtlebot4/cmd_vel


ros2 run nav2_map_server map_saver_cli -f "hall_map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/turtlebot4

ros2 launch turtlebot4_navigation localization.launch.py map:=/root/thesis_social_navigation_ws/map/lab2_map.yaml namespace:=/turtlebot4

python3 lidar_person_detector.py --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static -r /map:=/turtlebot4/map
python3 yolo_leg_detector_lidar.py --ros-args -r /oakd/rgb/preview/image_raw/compressed:=/turtlebot4/oakd/rgb/preview/image_raw/compressed
python3 identity_fusion_node_lidar.py --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
python3 person_marker_publisher.py --ros-args -p output_topic:=/turtlebot4/person_markers
python3 human_kf_predictor_lidar.py --ros-args -r /odom:=/turtlebot4/odom
python3 predicted_person_cloud_node_lidar.py --ros-args \
  -r /tf:=/turtlebot4/tf \
  -r /tf_static:=/turtlebot4/tf_static \
  -r /local_costmap/clear_entirely_local_costmap:=/turtlebot4/local_costmap/clear_entirely_local_costmap \
  -r /global_costmap/clear_entirely_global_costmap:=/turtlebot4/global_costmap/clear_entirely_global_costmap \
  -r /local_costmap/clear_around_pose_local_costmap:=/turtlebot4/local_costmap/clear_around_pose_local_costmap

ros2 launch /root/thesis_social_navigation_ws/launch/perception_bringup.launch.py 2>&1 | tee /root/thesis_social_navigation_ws/perception_bringup_output.log

ros2 launch /root/thesis_social_navigation_ws/launch/lab2_bringup.launch.py map:=/root/thesis_social_navigation_ws/map/lab2_map.yaml namespace:=/turtlebot4 params_file:=/root/thesis_social_navigation_ws/config/social_nav2_ablation_E_critweight20_socialcritic_on.yaml 2>&1 | tee /root/thesis_social_navigation_ws/lab2_bringup_output.log

python3 /root/thesis_social_navigation_ws/scripts/social_group_detector_node.py --ros-args -r /turtlebot4/oakd/rgb/preview/image_raw/compressed:=/turtlebot4/oakd/rgb/preview/image_raw/compressed -p costmap_topic:=/turtlebot4/local_costmap/costmap 2>&1 | tee /root/thesis_social_navigation_ws/social_group_output.log

Issues:
1. Wifi issues - too many devices using the same network caused the slam and navigation able to receive data 
                 caused: no map received, unable to see Robot in rviz, Navigation fail
                 solution: Change a router, only one device access.
                 Effect: Slam, Navigation, Camera nodes all ran stable
    
        2. Time sync issues - The time on turtlebot4 is different compare on host pc
                        caused: Unable to receive timestamp as it drift, no 2d pose estimation on rviz
                        solution: Set up chrony, use pc as the main chrony server, so pi will not depend on network.
                        Effect: Receive Timestamp with no delay, stable 2d pose estimation and real time updates on rviz

                Find PC's LAN subnet and IP:
                        ip addr show | grep inet
                        (note the inet line for your LAN interface, e.g. 192.168.X.X/24)
                On host (replace subnet with your own, e.g. 192.168.X.0/24):
                        sudo bash -c 'echo "allow 192.168.50.0/24" >> /etc/chrony/chrony.conf'
                        sudo systemctl restart chronyd
                On pi(replace with your PC's actual LAN IP):
                        sudo bash -c 'echo "server <PC_LAN_IP> iburst" >> /etc/chrony/chrony.conf'
                        sudo systemctl restart chronyd
                Verify:
                        chronyc tracking
                        chronyc sources -v

                sudo systemctl stop chronyd
                sudo chronyd -q 'server 192.168.50.220 iburst'
                sudo systemctl start chronyd
                sudo chronyc makestep
                chronyc sources
                sudo systemctl restart turtlebot4.service
        
3. Wifi power saving on Pi - power management can put wifi radio to sleep between packets, adding latency/jitter
                caused: potential packet delay/jitter, risk of intermittent topic drops or delayed TF between Pi and PC
                solution: Checked radio state directly - iw wlan0 get power_save confirmed already off, no change needed
                effect: Wifi radio stays fully active, no added latency from power management
         On pi:
                iw wlan0 get power_save
                sudo iw wlan0 set power_save off # if it shows "on"
                sudo iw wlan0 set power_save on # to re-enable if needed

4. No internet on host PC - dedicated robot router has no WAN, but PC still needs internet
                caused: Only one device allowed on robot AP (fix #1), so PC lost internet access;
                                pulling packages/docs required switching networks and dropping the robot link
                solution: iPhone USB tethering for internet, WiFi kept on the robot AP.
                                Set route metrics so tether is the default route, robot subnet stays on WiFi.
                effect: Internet via tether, robot traffic (192.168.50.x) still on WiFi, no interference

        Check interfaces and current routes:
                nmcli device status
                        # tether appears as type "ethernet", device enx<MAC> (USB NCM)
                ip route
                        # note "metric" on each default line, lower wins

        Set tether as default route:
                nmcli connection modify "Iphone E" ipv4.route-metric 50
                nmcli connection up "Iphone E"
                        # modify only writes config, "up" applies it

        Verify:
                ip route
                        # default via 172.20.10.1 dev enx162d4de8a981 metric 50   <- internet
                        # default via 192.168.50.1 dev wlp0s20f3   metric 20600   <- fallback
                        # 192.168.50.0/24 dev wlp0s20f3            metric 600     <- robot, unaffected
                ip route get 1.1.1.1
                        # should show dev enx162d4de8a981
                ping 192.168.50.<robot_ip>
                        # confirm robot still reachable over WiFi

        Revert:
                nmcli connection modify "Iphone E" ipv4.route-metric -1
                nmcli connection up "Iphone E"



ros2 bag record -o headon2 /turtlebot4/scan /turtlebot4/tf /turtlebot4/tf_static /turtlebot4/map /turtlebot4/odom /turtlebot4/amcl_pose /turtlebot4/oakd/rgb/preview/image_raw/compressed /turtlebot4/oakd/rgb/preview/camera_info /turtlebot4/global_costmap/costmap /turtlebot4/local_costmap/costmap /turtlebot4/plan /turtlebot4/cmd_vel /turtlebot4/cmd_vel_nav /turtlebot4/person_markers /person_positions_map /lidar_person_clusters /person_positions_fused /predicted_person_positions /predicted_person_cloud /turtlebot4/behavior_tree_log
ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -0.974, y: 7.088, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.275, w: 0.961}}}"
ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.123, y: 3.617, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.954, w: 0.300}}}"
ros2 launch /root/thesis_social_navigation_ws/launch/lab2_bringup.launch.py map:=/root/thesis_social_navigation_ws/map/map_name.yaml namespace:=/turtlebot4 params_file:=/root/thesis_social_navigation_ws/config/social_nav2_ablation_E_critweight20_socialcritic_on.yaml 2>&1 | tee /root/thesis_social_navigation_ws/lab2_bringup_output.log
ros2 launch /root/thesis_social_navigation_ws/launch/perception_bringup.launch.py 2>&1 | tee /root/thesis_social_navigation_ws/perception_bringup_output.log



ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -6.839, y: 13.278, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.354, w: 0.935}}}"

ros2 launch /root/thesis_social_navigation_ws/launch/lab2_bringup.launch.py map:=/root/thesis_social_navigation_ws/map/hall_map_name.yaml namespace:=/turtlebot4 params_file:=/root/thesis_social_navigation_ws/config/social_nav2_ablation_E_critweight20_socialcritic_on.yaml 2>&1 | tee /root/thesis_social_navigation_ws/lab2_bringup_output.log

ros2 launch /root/thesis_social_navigation_ws/launch/perception_bringup.launch.py 2>&1 | tee /root/thesis_social_navigation_ws/perception_bringup_output.log

cd ~/thesis_social_navigation_ws && ros2 bag record -o hallway_7m_04 /turtlebot4/scan /turtlebot4/odom /turtlebot4/tf /turtlebot4/tf_static /turtlebot4/map /turtlebot4/amcl_pose /turtlebot4/cmd_vel /turtlebot4/cmd_vel_nav /turtlebot4/plan /turtlebot4/global_costmap/costmap /turtlebot4/local_costmap/costmap /turtlebot4/oakd/rgb/preview/image_raw/compressed /turtlebot4/oakd/rgb/preview/camera_info /turtlebot4/person_markers /lidar_person_clusters /person_positions_map /person_positions_fused /predicted_person_positions

ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -1.286, y: 8.291, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.932, w: 0.362}}}"

ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -6.839, y: 13.278, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.354, w: 0.935}}}"

python3 /root/thesis_social_navigation_ws/odom_qos_relay.py



Hallway navigation test:
ros2 launch /root/thesis_social_navigation_ws/launch/lab2_bringup.launch.py map:=/root/thesis_social_navigation_ws/map/hall_map_name.yaml namespace:=/turtlebot4 params_file:=/root/thesis_social_navigation_ws/config/social_nav2_ablation_E_critweight20_socialcritic_on.yaml 2>&1 | tee /root/thesis_social_navigation_ws/lab2_bringup_output.log

mkdir -p /root/thesis_social_navigation_ws/logs && ros2 launch /root/thesis_social_navigation_ws/launch/perception_bringup.launch.py coast_timeout:=0.6 2>&1 | tee /root/thesis_social_navigation_ws/logs/berth06_06.txt 

ros2 bag record -o /root/thesis_social_navigation_ws/bags/berth06_$(date +%Y%m%d_%H%M%S) /turtlebot4/scan /turtlebot4/tf /turtlebot4/tf_static /turtlebot4/odom /turtlebot4/map /turtlebot4/amcl_pose /turtlebot4/oakd/rgb/preview/image_raw/compressed /turtlebot4/oakd/rgb/preview/camera_info /person_positions_map /lidar_person_clusters /person_positions_fused /predicted_person_positions /predicted_person_cloud /turtlebot4/plan /turtlebot4/cmd_vel /turtlebot4/cmd_vel_nav /turtlebot4/local_costmap/costmap /turtlebot4/global_costmap/costmap /rosout

python3 /root/thesis_social_navigation_ws/scripts/stop_diag.py --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static

Occupancy:

ros2 launch /root/thesis_social_navigation_ws/launch/lab2_bringup.launch.py map:=/root/thesis_social_navigation_ws/map/map_name.yaml namespace:=/turtlebot4 params_file:=/root/thesis_social_navigation_ws/config/social_nav2_occupancy.yaml 2>&1 | tee /root/thesis_social_navigation_ws/social_occupancy.log


Headon record 3 configs
Start position: ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -9.144237916334216, y: 15.333447075223367, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9345992252254677, w: 0.355702527694077}}}"

Goal Position: ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -9.144237916334216, y: 15.333447075223367, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.3320365108167037, w: 0.943266534699853}}}"

