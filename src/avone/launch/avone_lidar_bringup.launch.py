from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to quanergy client launch
    client_launch_path = os.path.join(
        get_package_share_directory('quanergy_client_ros'),
        'client.launch.py'
    )

    quanergy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(client_launch_path),
        launch_arguments={'host': '192.168.1.57'}.items()
    )

    # --- Static TF publisher ---
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='quanergy_static_tf',
        arguments=[
            '0', '0', '0',        # x y z
            '3.14159', '0', '0',  # roll pitch yaw
            'laser_frame', 'quanergy'
        ],
        output='screen'
    )

    # --- Collision detection node ---
    collision_detection = Node(
        package='avone_nav',
        executable='collision_detection',
        name='collision_detection',
        output='screen'
    )

    return LaunchDescription([
        quanergy_launch,
        static_tf,
        collision_detection
    ])
