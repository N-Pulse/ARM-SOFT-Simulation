import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
            'use_sim_time': 'false'  # Not using simulation time in RViz alone
        }.items()
    )
    
    # RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'config', 'prosthesis.rviz.yaml')
    
    # run RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )
    
    # run Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_launch,
        rviz_node,
        joint_state_publisher_gui_node
    ])