import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'prosthesis_description'
    pkg_share = get_package_share_directory(pkg_name)
    world_file = os.path.join(pkg_share, 'worlds', 'prosthesis_world.sdf')
    
    # Include the base robot_state_publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'  # Using simulation time in Gazebo
        }.items()
    )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # Run the spawner node from the gazebo_ros package after a delay to ensure Gazebo is ready
    spawn_entity = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'prosthesis',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.2'
                ],
                parameters=[{
                    'use_sim_time': True
                }],
                output='screen'
            )
        ]
    )

    #  converts Gazebo clock and contact sensor data to ROS
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/model/prosthesis/link/index_middle_link/sensor/index_fsr/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/default/model/prosthesis/link/middle_middle_link/sensor/middle_fsr/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/default/model/prosthesis/link/ring_middle_link/sensor/ring_fsr/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/default/model/prosthesis/link/little_middle_link/sensor/little_fsr/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/default/model/prosthesis/link/thumb_distal_link/sensor/thumb_fsr/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Nodes to automatically configure joint_state_broadcaster and joint_state_controller
    load_joint_state_broadcaster = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen'
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_launch,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])