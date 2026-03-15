#!/usr/bin/env bash
# Build the ROS 2 workspace inside the ros2-dev container.
# Run this from the helios/ root directory.
set -euo pipefail

PACKAGE="${1:-}"   # optional: pass a specific package name

if [[ -z "$PACKAGE" ]]; then
    echo "[build.sh] Building all packages..."
    docker exec ros2_dev bash -c "
        source /opt/ros/humble/setup.bash &&
        source /px4_msgs_ws/install/setup.bash &&
        cd /ros2_ws &&
        colcon build \
            --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --event-handlers console_cohesion+
    "
else
    echo "[build.sh] Building package: $PACKAGE"
    docker exec ros2_dev bash -c "
        source /opt/ros/humble/setup.bash &&
        source /px4_msgs_ws/install/setup.bash &&
        cd /ros2_ws &&
        colcon build \
            --symlink-install \
            --packages-select $PACKAGE \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --event-handlers console_cohesion+
    "
fi

echo "[build.sh] Done."