cmake_minimum_required(VERSION 3.8)
project(helios_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

# ====================
# CORE ESTIMATION LIBRARY (zero ROS dependency)
# ====================
add_library(helios_estimator_core SHARED
  src/esekf.cpp
)
target_include_directories(helios_estimator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(helios_estimator_core PUBLIC
  Sophus::Sophus
  Eigen3::Eigen
)

# ====================
# ROS2 COMPOSABLE NODE
# ====================
add_library(estimator_node_component SHARED
  src/estimator_node.cpp)

target_link_libraries(estimator_node_component
  helios_estimator_core
  Sophus::Sophus
  Eigen3::Eigen
)

ament_target_dependencies(estimator_node_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  px4_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
)

rclcpp_components_register_node(estimator_node_component
  PLUGIN "helios::EstimatorNode"
  EXECUTABLE estimator_node
)

# ====================
# INSTALL
# ====================
install(TARGETS
    helios_estimator_core
    estimator_node_component
    EXPORT export_helios_estimator_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL)

# ====================
# UNIT TESTS
# ====================
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)

    # ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_esekf test/test_esekf.cpp)
    target_link_libraries(test_esekf helios_estimator_core)
endif()

ament_export_targets(export_helios_estimator_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Sophus Eigen3)

ament_package()
