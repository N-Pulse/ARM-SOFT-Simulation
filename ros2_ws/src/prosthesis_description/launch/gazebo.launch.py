import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'prosthesis_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Include the base robot_state_publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'  # Using simulation time in Gazebo
        }.items()
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )
    
    # Spawn entity in Gazebo
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
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher_launch,
        spawn_entity
    ])