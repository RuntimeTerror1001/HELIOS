-- Cartographer configuration file
include "/home/redpaladin/Projects/helios_ws/src/helios/config/map_builder.lua"

MAP_BUILDER.use_trajectory_builder_2d = false  -- Use 3D SLAM
MAP_BUILDER.use_trajectory_builder_3d = true   -- Enable 3D SLAM

-- LiDAR configuration
TRAJECTORY_BUILDER_3D.laser_scan_period = 0.1  -- 10 Hz (1/10)
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.rangefinder_sensor_ids = {"front_laser"}  -- Your LiDAR sensor

-- Laser scan angles
TRAJECTORY_BUILDER_3D.horizontal_laser_min_angle = -3.14159
TRAJECTORY_BUILDER_3D.horizontal_laser_max_angle = 3.14159
TRAJECTORY_BUILDER_3D.vertical_laser_min_angle = -0.261799
TRAJECTORY_BUILDER_3D.vertical_laser_max_angle = 0.261799

-- Range
TRAJECTORY_BUILDER_3D.min_range = 0.05
TRAJECTORY_BUILDER_3D.max_range = 100.0

-- LiDAR noise
TRAJECTORY_BUILDER_3D.rangefinder_noise = 0.03
