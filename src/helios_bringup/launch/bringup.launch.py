from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("helios_bringup")

    # 1. Sensor Bridge + static TF
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "sensors.launch.py"])
        )
    )

    # 2. Offboard Bridge Node
    offboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "offboard.launch.py"])
        )
    )

    # 3. Lifecycle Manager - give nodes 1s to come up
    lifecycle = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "lifecycle.launch.py"])
                )
            )
        ],
    )

    return LaunchDescription([sensors, offboard, lifecycle])