import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false' # sim time not necessary in Rviz for example
    )
    
    # Use xacro to process the URDF
    xacro_file = os.path.join(get_package_share_directory('prosthesis_description'), 'model/prosthesis.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # run Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node
    ])