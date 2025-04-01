import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Locate the Xacro file
    pkg_path = os.path.join(get_package_share_directory('AVONE_description'))
    xacro_file = os.path.join(pkg_path, 'AVONE', 'robot.urdf.xacro')

    # Process the Xacro and convert it to string
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config
        }]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        joint_state_publisher_gui_node,
        rsp_node
    ])
