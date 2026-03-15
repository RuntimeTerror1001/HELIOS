from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    # ====================
    # ARGUMENTS
    # ====================
    cruise_alt_arg = DeclareLaunchArgument(
        "cruise_altitude",
        default_value="3.0",
        description="Cruise altitude in metres above ground"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("helios_bringup"),
            "config",
            "offboard_params.yaml"
        ]),
        description="Path to the offboard bridge params file"
    )

    # ====================
    # COMPOSABLE NODE CONTAINER
    # ====================
    container = ComposableNodeContainer(
        name="helios_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt", # multithreaded executor
        composable_node_descriptions=[
            ComposableNode(
                package="helios_offboard",
                plugin="helios::OffboardBridge",
                name="offboard_bridge",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"cruise_altitude": LaunchConfiguration("cruise_altitude")},
                ],
                extra_arguments=[{"use_intra_process_comms": True}]
            )
        ],
        output="screen"
    )

    # ====================
    # SHUTDOWN EVERYTHING IF CONTAINER DIES
    # ====================
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=container,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        cruise_alt_arg,
        params_file_arg,
        container,
        shutdown_on_exit
    ])