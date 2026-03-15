FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# ====================
# Add OSRF Gazebo apt repo
# ====================
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# ====================
# Core ROS 2 tools
# ====================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-graph \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

# ====================
# Gazebo Garden ROS 2 bridge (from OSRF repo)
# ====================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gzgarden \
    ros-humble-actuator-msgs \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# ====================
# Nav2 Stack
# ====================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

# ====================
# Estimation Libraries
# ====================
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libpcl-dev \
    ros-humble-ompl \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# ====================
# px4_msgs — release/1.14
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
# Python Tooling
# ====================
RUN pip3 install --no-cache-dir black ruff mypy

# ====================
# rosdep
# ====================
RUN rosdep init 2>/dev/null || true && rosdep update

# ====================
# Shell Environment
# ====================
RUN echo "source /opt/ros/humble/setup.bash"        >> /root/.bashrc && \
    echo "source /px4_msgs_ws/install/setup.bash"   >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]