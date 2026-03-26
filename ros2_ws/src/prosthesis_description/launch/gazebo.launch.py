import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'prosthesis_description'
    pkg_share = get_package_share_directory(pkg_name)
    world_file_path = os.path.join(pkg_share, 'worlds', 'prosthesis_world.sdf')
    gz_bridge_params_path = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    robot_controllers_path = os.path.join(pkg_share, 'config', 'prosthesis_controllers.yaml')
    
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
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items()
    )
    
    # Run the spawner node from the gazebo_ros package after a delay to ensure Gazebo is ready
    spawn_entity = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'prosthesis',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.05'
                ],
                parameters=[{
                    'use_sim_time': True
                }],
                output='screen'
    )

    #  ROS Gazebo bridge node with loaded config file
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers_path
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers_path
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    '''     load_fsr_broadcaster = Node(
        package='prosthesis_description',
        executable='fsr_broadcaster.py',
        output='screen'
    )

    load_ft_broadcaster = Node(
        package='prosthesis_description',
        executable='ft_broadcaster.py',
        output='screen'
    )
    '''


    return LaunchDescription([
        robot_state_publisher_launch,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
        #load_fsr_broadcaster,
        #load_ft_broadcaster
    ])