import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'prosthesis_description'
    file_subpath = 'urdf/prosthesis.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Rviz node
    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'rviz/prosthesis.rviz'
    )

    node_rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )


    # Run the nodes
    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz
    ])