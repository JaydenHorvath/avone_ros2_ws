from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('trolley_description')
    urdf_file = os.path.join(pkg_path, 'description', 'trolley.urdf')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
        output='screen'
    )

    return LaunchDescription([robot_state_publisher])
