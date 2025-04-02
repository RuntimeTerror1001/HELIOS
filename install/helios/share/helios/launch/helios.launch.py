from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    # Declare the launch argument
    #xacro_arg = DeclareLaunchArgument(
    #    'helios_main_xacro',
    #    default_value='/home/redpaladin/Projects/helios_ws/src/helios/urdfs/helios_main.urdf.xacro',
    #    description='Path to the main XACRO file'
    #)

    # Resolve the XACRO file path
    #xacro_file = LaunchConfiguration('helios_main_xacro')

    # Process the XACRO file
    #xacro_path = '/home/redpaladin/Projects/helios_ws/src/helios/urdfs/helios_main.urdf.xacro'  # Provide fallback for runtime resolution
    #if os.path.exists(xacro_path):
    #    robot_desc_config = xacro.process_file(xacro_path)
    #    robot_desc = robot_desc_config.toxml()
    #else:
    #    raise FileNotFoundError(f"The XACRO file at {xacro_path} was not found!")

    # Write URDF to a temporary file
    #urdf_temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
    #urdf_temp_file.write(robot_desc.encode('utf-8'))
    #urdf_temp_file_path = urdf_temp_file.name
    #urdf_temp_file.close()  # Correctly call the close method

    # Start Ignition Gazebo
    start_ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '/home/redpaladin/Projects/helios_ws/src/helios/worlds/collapsed_industrial/collapsed_industrial.sdf'],
        #cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model'
        ],
        remappings=[
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/camera_front/image', '/camera/image_raw'),
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/front_laser/scan', '/lidar/scan'),
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # Spawn the entity in Ignition Gazebo
    #spawn_entity = Node(
    #    package='ros_gz_sim',
    #    executable='create',
    #    arguments=[
    #        '-file', urdf_temp_file_path,
    #        '-name', 'helios',
    #        '-allow_renaming', 'true',
    #        '-x', '10',  # Move far from (0, 0)
    #        '-y', '-10', # Move to a clearer space
    #        '-z', '5',    # Ensure it's above the ground
    #        '-R', '0', 
    #        '-P', '0', 
    #       '-Y', '-0.916'
    #    ],
    #   output='screen'
    #)

    # Publish robot state
    #robot_state_publisher = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    parameters=[
    #        {'robot_description': robot_desc}
    #    ],
    #    output='screen'
    #)

    helios_tf_publisher = Node(
        package='helios',
        executable='helios_tf_publisher',
        output='screen'
    )

    keyboard_controller = Node(
        package='helios',
        executable='keyboard_control',
        output='screen'
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True  # Enable simulation time
            #'cartographer_ros_config': '/home/redpaladin/Projects/helios_ws/src/helios/config/cartographer_config.lua'  # Path to your cartographer config file
        }],
        remappings=[
            ('/scan', '/lidar/scan'),  # Remap LiDAR topic
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ],
        arguments=[
            '--configuration_directory', '/home/redpaladin/Projects/helios_ws/src/helios/config/',  # Path to your Cartographer config
            '--configuration_basename', 'cartographer_config.lua'  # Lua config file name
        ]
    )

    # Cartographer's Trajectory Publisher Node (to publish the map)
    #trajectory_publisher_node = Node(
    #    package='cartographer_ros',
    #    executable='cartographer_trajectory_upload_node',
    #    name='cartographer_trajectory_upload_node',
    #    output='screen',
    #    parameters=[{
    #        'use_sim_time': True,
    #    }],
    #)

    return LaunchDescription([
        #xacro_arg,
        start_ign_gazebo,
        #spawn_entity,
        #robot_state_publisher,
        helios_tf_publisher,
        #keyboard_controller,
        #cartographer_node,
        #trajectory_publisher_node,
        ros_gz_bridge
    ])
