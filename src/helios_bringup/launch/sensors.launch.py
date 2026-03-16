from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("helios_bringup")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock"
    )

    # ====================
    # ros_gz_bridge : GAZEBO -> ROS 2
    # ====================    
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        namespace="helios",
        output="screen",
        parameters=[{
            "config_file": PathJoinSubstitution([
                pkg, "config", "ros_gz_bridge.yaml"
            ]),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "qos_overrides./helios/lidar/points.publisher.reliability": "best_effort",
            "qos_overrides./helios/imu/data.publisher.reliability": "best_effort",
            "qos_overrides./helios/gps/fix.publisher.reliability": "best_effort"
        }]
    )

    # ====================
    # STATIC TF PUBLISHERS
    # ====================
    
    # base_link -> lidar_link (matches SDF pose: z=0.15)
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.08",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ]
    )

    # base_link -> camera_left_link
    camera_left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_left_tf",
        output="screen",
        arguments=[
            "--x", "0.18",
            "--y", "0.06",
            "--z", "0.05",
            "--roll", "0.0",
            "--pitch", "0.2618",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_left_link",
        ],
    )

    # base_link -> camera_right_link  (0.12m baseline, right = negative y in ROS)
    camera_right_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_right_tf",
        output="screen",
        arguments=[
            "--x", "0.18",
            "--y", "-0.06",
            "--z", "0.05",
            "--roll", "0.0",
            "--pitch", "0.2618",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_right_link",
        ],
    )

    # base_link -> imu_link (at origin — IMU at CoM)
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        bridge,
        lidar_tf,
        camera_left_tf,
        camera_right_tf,
        imu_tf,
    ])