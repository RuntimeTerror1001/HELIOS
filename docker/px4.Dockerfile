FROM px4io/px4-dev-simulation-jammy:latest

# Avoid interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# ====================
# ROS 2 Humble 
# ==================== 
# Add ROS 2 apt repo
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ament-cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ====================
# Gazebo Garden ROS 2 bridge
# ====================
# px4-dev-simulation-jammy ships Gazebo Garden already
# We just need the ROS 2 bridge on top
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gzgarden \
    ros-humble-actuator-msgs \
    ros-humble-vision-msgs \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

# ====================
# px4_msgs
# ====================
RUN mkdir -p /px4_msgs_ws/src && \
    cd /px4_msgs_ws/src && \
    git clone --branch release/1.14 --depth 1 \
        https://github.com/PX4/px4_msgs.git && \
    cd /px4_msgs_ws && \
    bash -c "source /opt/ros/humble/setup.bash && \
             colcon build \
             --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
    rm -rf /px4_msgs_ws/build

# ====================
# rosdep
# ====================
RUN rosdep init 2>/dev/null || true && rosdep update

# ====================
# Shell environment
# ====================
RUN echo "source /opt/ros/humble/setup.bash"      >> /root/.bashrc && \
    echo "source /px4_msgs_ws/install/setup.bash" >> /root/.bashrc

# PX4-Autopilot mounted as volume at /px4 — built at runtime
WORKDIR /px4
CMD ["bash"]