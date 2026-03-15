from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    """
    Manages lifecycle transitions for all Helios lifecycle nodes.
    Automatically configures and activates them in order.
    """
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="helios_lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [
                "offboard_bridge",
            ],
            "bond_timeout": 4.0,
        }],
    )

    return LaunchDescription([lifecycle_manager])