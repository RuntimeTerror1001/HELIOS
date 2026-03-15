from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("helios_bringup")

    # Start offboard bridge first
    offboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "offboard.launch.py"])
        )
    )

    # Give nodes 1 second to come up, then start lifecycle manager
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

    return LaunchDescription([offboard, lifecycle])