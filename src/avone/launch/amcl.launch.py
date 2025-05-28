import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the launch argument for the parameters file
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('avone'),
            'config',
            'amcl_params.yaml'
        ),
        description='~/ros2_ws/src/avone/config/amcl_params.yaml'
    )

    # Use the LaunchConfiguration substitution to get the params file
    params_file = LaunchConfiguration('params_file')

    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Lifecycle manager for AMCL
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['amcl']
            }
        ]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        amcl_node,
        lifecycle_manager_node
    ])
