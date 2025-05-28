import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Static transform publisher: map -> odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=[
            '0', '0', '0',  # xyz
            '0', '0', '0',  # rpy
            'map', 'odom'   # frames
        ]
    )

    # Cone landmark mapper node
    cone_mapper_node = Node(
        package='cone_mapper',
        executable='conelandmarkmapper',
        name='cone_landmark_mapper',
        output='screen'
    )

    # Live cone map publisher
    live_map_node = Node(
        package='cone_mapper',
        executable='live_cone_map',
        name='live_cone_map',
        output='screen'
    )

    # Include YOLO launch from avone package
    avone_share_dir = get_package_share_directory('avone')
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(avone_share_dir, 'launch', 'yolo.launch.py')
        )
    )

    return LaunchDescription([
        static_tf,
        cone_mapper_node,
        live_map_node,
        yolo_launch,
    ])
