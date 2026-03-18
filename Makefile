# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  Helios Drone Simulation — Project Makefile                             ║
# ║  Usage: make <target> [PKG=package_name]                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝

COMPOSE     := docker compose -f docker/docker-compose.yml
ROS2_EXEC   := docker exec ros2_dev bash -c
PX4_EXEC    := docker exec -it px4_sitl bash -c

# ====================
# Default target — show help
# ====================
.DEFAULT_GOAL := help

.PHONY: help up down restart build-image \
        px4 shell \
        build build-pkg test clean-build \
        launch logs

# Help
help:
	@echo ""
	@echo "  Helios Makefile"
	@echo "  ───────────────────────────────────────────────────────"
	@echo "  Infrastructure:"
	@echo "    make up           Start micro-xrce and ros2-dev services"
	@echo "    make down         Stop and remove all containers"
	@echo "    make restart      down + up"
	@echo ""
	@echo "  PX4 / Gazebo:"
	@echo "    make px4          Start PX4 SITL + Gazebo in px4-sitl container"
	@echo ""
	@echo "  Development:"
	@echo "    make shell        Open interactive shell in ros2-dev"
	@echo "    make build        Build all ROS 2 packages"
	@echo "    make build PKG=X  Build a specific package"
	@echo "    make test         Run all colcon tests"
	@echo "    make clean-build  Remove build/ install/ log/"
	@echo ""
	@echo "  Runtime:"
	@echo "    make launch       Launch full Helios bringup"
	@echo "    make logs         Tail logs from all containers"
	@echo ""

# ====================
# INFRASTRUCTURE
# ====================
up:
	xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d micro-xrce ros2-dev
	@echo "[helios] micro-xrce and ros2-dev are up."

down:
	$(COMPOSE) down
	@echo "[helios] All containers stopped."

rebuild:
ifdef SVC
	@echo "[helios] Rebuilding and restarting service: $(SVC)..."
	$(COMPOSE) stop $(SVC)
	$(COMPOSE) build $(SVC)
	$(COMPOSE) up -d $(SVC)
else
	@echo "[helios] Tearing down and rebuilding ALL services..."
	$(COMPOSE) down
	$(COMPOSE) build
	$(COMPOSE) up -d
endif

restart:
ifdef SVC
	@echo "[helios] Applying compose changes and restarting: $(SVC)..."
	$(COMPOSE) up -d --force-recreate $(SVC)
else
	@echo "[helios] Restarting ALL services..."
	$(COMPOSE) down
	xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d micro-xrce ros2-dev
endif

# ====================
# PX4/ GAZEBO
# ====================
px4:
	@echo "[helios] Starting PX4 SITL + Gazebo..."
	xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d px4-sitl
	docker exec -it px4_sitl bash -c \
		"cd /px4 && make px4_sitl gz_x500"

# ====================
# DEVELOPMENT SHELL
# ====================
shell:
	docker exec -it ros2_dev bash

# ====================
# BUILD
# ====================
build:
ifdef PKG
	@echo "[helios] Building package: $(PKG)"
	$(ROS2_EXEC) "source /opt/ros/humble/setup.bash && source /px4_msgs_ws/install/setup.bash && cd /ros2_ws && colcon build \
		--packages-select $(PKG) --symlink-install \
		--cmake-args \
			-DCMAKE_BUILD_TYPE=RelWithDebInfo \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		--event-handlers console_cohesion+"
else
	@echo "[helios] Building all packages..."
	$(ROS2_EXEC) "source /opt/ros/humble/setup.bash && source /px4_msgs_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install \
		--cmake-args \
			-DCMAKE_BUILD_TYPE=RelWithDebInfo \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		--event-handlers console_cohesion+"
endif

# ====================
# TEST
# ====================
test:
ifdef PKG
	@echo "[helios] Testing package: $(PKG)"
	$(ROS2_EXEC) "source /opt/ros/humble/setup.bash && source /px4_msgs_ws/install/setup.bash && cd /ros2_ws && colcon test \
		--packages-select $(PKG) \
		--event-handlers console_cohesion+"
else
	@echo "[helios] Running all tests..."
	$(ROS2_EXEC) "cd /ros2_ws && colcon test \
		--event-handlers console_cohesion+"
endif

# ====================
# CLEAN
# ====================
clean-build:
	@echo "[helios] Removing build/ install/ log/..."
	rm -rf build/ install/ log/
	@echo "[helios] Clean done."

# ====================
# LAUNCH
# ====================
launch:
	@echo "[helios] Launching Helios bringup..."
	$(ROS2_EXEC) "source /opt/ros/humble/setup.bash && source /px4_msgs_ws/install/setup.bash && cd /ros2_ws && \
		ros2 launch helios_bringup bringup.launch.py"

# ====================
# LOGS
# ====================
logs:
	$(COMPOSE) logs -f