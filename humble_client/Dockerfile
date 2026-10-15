# =====================================================
# Thesis Social Navigation — ROS 2 Humble (REAL TurtleBot4)
# Base: matches Pi/Create3's Humble version, for use over
# the zenoh bridge from host. No Gazebo/sim packages here.
# =====================================================

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble


# -----------------------------------------------------
# 1. Base system tools
# -----------------------------------------------------
RUN apt update && apt install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    nano \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-va-drivers \
    && rm -rf /var/lib/apt/lists/*


# -----------------------------------------------------
# 2. Pin setuptools for colcon symlink installs
#    (same fix as the sim container)
# -----------------------------------------------------
RUN pip install "setuptools==70.0.0"


# -----------------------------------------------------
# 3. Python venv for YOLO / Ultralytics
#    Kept separate from system Python. Same pin rationale
#    as the sim container's Dockerfile.
# -----------------------------------------------------
RUN apt update && apt install -y python3-venv && \
    python3 -m venv /root/venv && \
    /root/venv/bin/pip install --upgrade pip && \
    /root/venv/bin/pip install "setuptools==70.0.0" && \
    /root/venv/bin/pip install --no-cache-dir \
        "numpy==1.26.4" \
        ultralytics \
        "lap>=0.5.12" \
        "openvino>=2024.0.0"

ENV PYTHONPATH=/root/venv/lib/python3.10/site-packages:$PYTHONPATH

# -----------------------------------------------------
# 3b. MobileCLIP-S1 (via open_clip), pre-downloaded at
#     build time — same rationale as sim container.
#     NOT USED: installed only because
#     scripts/social_group_detector_node_lidarhold.py imports
#     open_clip; its VLM fallback never runs (HOW_TO_RUN.md,
#     section 5).
# -----------------------------------------------------
RUN /root/venv/bin/pip install --no-cache-dir open_clip_torch && \
    /root/venv/bin/python3 -c "\
import open_clip; \
open_clip.create_model_and_transforms('MobileCLIP-S1', pretrained='datacompdr'); \
print('MobileCLIP-S1 checkpoint cached at build time.')"


# -----------------------------------------------------
# 4. ROS navigation / control packages (real-robot set —
#    no ros-humble-ros-gz*, no Gazebo)
# -----------------------------------------------------
RUN apt update && apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*


# -----------------------------------------------------
# 5. TurtleBot4 / Create3 real-robot packages
# -----------------------------------------------------
RUN apt update && apt install -y \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-bringup \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-viz \
    ros-humble-irobot-create-description \
    ros-humble-irobot-create-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*


# -----------------------------------------------------
# 6. ROS dependency tools
# -----------------------------------------------------
RUN rosdep init || true
RUN rosdep update


# -----------------------------------------------------
# 6b. Chrony — this container acts as NTP server for the
#     Pi (no systemd in container, so chronyd is started
#     manually via /etc/init.d/chrony in .bashrc below)
# -----------------------------------------------------
RUN apt update && apt install -y chrony && \
    rm -rf /var/lib/apt/lists/* && \
    echo "allow 192.168.50.0/24" >> /etc/chrony/chrony.conf


# -----------------------------------------------------
# 7. TurtleBot4-style setup.bash + CycloneDDS config
#    (matches official PC setup guide). Baked in so
#    CYCLONEDDS_URI is valid with zero manual steps.
# -----------------------------------------------------
RUN mkdir -p /etc/turtlebot4 && \
    printf '%s\n' \
      '<CycloneDDS>' \
      '    <Domain id="1">' \
      '        <General>' \
      '            <Interfaces>' \
      '                <NetworkInterface name="wlp0s20f3" priority="default" multicast="default"/>' \
      '            </Interfaces>' \
      '            <DontRoute>true</DontRoute>' \
      '        </General>' \
      '    </Domain>' \
      '</CycloneDDS>' \
      > /etc/turtlebot4/cyclonedds_pc.xml && \
    echo "source /opt/ros/humble/setup.bash" >> /etc/turtlebot4/setup.bash && \
    echo "export ROS_DOMAIN_ID=1" >> /etc/turtlebot4/setup.bash && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /etc/turtlebot4/setup.bash && \
    echo "export CYCLONEDDS_URI=/etc/turtlebot4/cyclonedds_pc.xml" >> /etc/turtlebot4/setup.bash


# -----------------------------------------------------
# 8. Workspace
# -----------------------------------------------------
WORKDIR /root/thesis_social_navigation_ws

RUN echo "source /etc/turtlebot4/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /root/thesis_social_navigation_ws/install/setup.bash ]; then source /root/thesis_social_navigation_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'service chrony start > /dev/null 2>&1' >> /root/.bashrc


CMD ["bash"]
