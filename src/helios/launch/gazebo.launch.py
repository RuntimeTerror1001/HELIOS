from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
import tempfile


def generate_launch_description():

    # Declare the launch argument
    xacro_arg = DeclareLaunchArgument(
        'helios_main_xacro',
        default_value='/home/redpaladin/helios_ws/src/helios/urdfs/helios_main.urdf.xacro',
        description='Path to the main XACRO file'
    )

    # Resolve the XACRO file path
    xacro_file = LaunchConfiguration('helios_main_xacro')

    # Process the XACRO file
    xacro_path = '/home/redpaladin/helios_ws/src/helios/urdfs/helios_main.urdf.xacro'  # Provide fallback for runtime resolution
    if os.path.exists(xacro_path):
        robot_desc_config = xacro.process_file(xacro_path)
        robot_desc = robot_desc_config.toxml()
    else:
        raise FileNotFoundError(f"The XACRO file at {xacro_path} was not found!")

    # Write URDF to a temporary file
    urdf_temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
    urdf_temp_file.write(robot_desc.encode('utf-8'))
    urdf_temp_file_path = urdf_temp_file.name
    urdf_temp_file.close()  # Correctly call the close method

    # Start Ignition Gazebo
    start_ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '/home/redpaladin/helios_ws/src/helios/worlds/collapsed_industrial/collapsed_industrial.sdf'],
        #cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # Spawn the entity in Ignition Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_temp_file_path,
            '-name', 'helios',
            '-allow_renaming', 'true',
            '-x', '10',  # Move far from (0, 0)
            '-y', '-10', # Move to a clearer space
            '-z', '2'    # Ensure it's above the ground
        ],
        output='screen'
    )

    # Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc}
        ],
        output='screen'
    )

    return LaunchDescription([
        xacro_arg,
        start_ign_gazebo,
        spawn_entity,
        robot_state_publisher,
        ros_gz_bridge
    ])
