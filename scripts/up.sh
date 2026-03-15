#!/usr/bin/env bash
# Bring up the full stack for Phase 0.
# Run from helios/ root.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Allow X11 connections from Docker containers
xhost +local:docker 2>/dev/null || true

echo "[up.sh] Building Docker images if needed..."
docker compose -f "$PROJECT_DIR/docker/docker-compose.yml" build

echo "[up.sh] Starting micro-xrce and ros2-dev services..."
docker compose -f "$PROJECT_DIR/docker/docker-compose.yml" \
    up -d micro-xrce ros2-dev

echo "[up.sh] Starting PX4 SITL + Gazebo in foreground..."
echo "[up.sh] Make target: make px4_sitl gz_x500"
echo ""
docker exec -it px4_sitl bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /px4 &&
    make px4_sitl gz_x500
"