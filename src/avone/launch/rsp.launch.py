import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import xacro

def generate_launch_description():

    # Declare sim time flag
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get xacro file and process robot description
    pkg_path = os.path.join(get_package_share_directory('avone'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)


    
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # Robot State Publisher
    node_robot_state_publisher = Node(
          package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[
        {'use_sim_time': True},
        {'robot_description': robot_description_config.toxml()},
    ]
    )
    
    # node_joint_state_publisher = Node(
#     package='joint_state_publisher_gui',
#     executable='joint_state_publisher_gui',
#     name='joint_state_publisher_gui',
#     output='screen'
# )
            # RViz Node (without config file)








    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value= 'true',
            description='Use sim time if true'),

        node_robot_state_publisher,
        #node_joint_state_publisher,
        #node_rviz,
        
        
    ])
