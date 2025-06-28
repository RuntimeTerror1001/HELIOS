from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Start Ignition Gazebo (your existing code)
    start_ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '/home/redpaladin/Projects/helios_ws/src/helios/worlds/collapsed_industrial/collapsed_industrial.sdf'],
        output='screen'
    )

    urdf_path = os.path.expanduser(
        '~/Projects/helios_ws/src/helios/urdfs/model.xacro.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_desc_str = f.read()

    robot_description = {'robot_description':robot_desc_str}

    # Start ros2_control controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=['/home/redpaladin/Projects/helios_ws/src/helios/config/helios_controllers.yaml'],
        output='screen'
    )

    node_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    helios_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rotor_controller'],
        output='screen'
    )
    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Your existing bridges
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/imu_sensor@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        remappings=[
            # Your existing remappings
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/camera_front/image', '/camera/image_raw'),
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/front_laser/scan', '/lidar/scan'),
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/joint_state', '/joint_states'),
            # Add IMU remap
            ('/world/collapsed_industrial/model/HELIOS/model/HELIOS/link/base_link/sensor/imu_sensor', '/imu/data')
        ],
        output='screen'
    )

    # Keep your existing nodes
    helios_tf_publisher = Node(
        package='helios',
        executable='helios_tf_publisher_updated',
        output='screen'
    )

    # Add everything to the launch description
    return LaunchDescription([
        #px4_sitl,               # Add PX4 SITL first
        start_ign_gazebo,       # Then start Gazebo
        node_robot_state_pub,
        #micrortps_agent,        # Add microRTPS agent
        #mavros_node,            # Add MAVROS
        ros_gz_bridge,          # Your updated bridge 
        controller_manager,
        joint_state_broadcaster,
        helios_effort_controller,
        helios_tf_publisher,    # Your existing TF publisher
        #px4_ros_gz_bridge,
        #keyboard_controller,   
        #cartographer_node,
    ])