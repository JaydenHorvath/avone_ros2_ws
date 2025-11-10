from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- VectorNav IMU bringup node ---
        Node(
            package='avone_localisation',
            executable='imu_start',
            name='imu_start',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # --- Static transform: imu_link → vn100_imu_link (180° about X) ---
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='imu_tf_broadcaster',
        #     arguments=[
        #         '--x', '0',
        #         '--y', '0',
        #         '--z', '0',
        #         '--roll', '3.14159',
        #         '--pitch', '3.14159',
        #         '--yaw', '0',
        #         '--frame-id', 'imu_link',
        #         '--child-frame-id', 'vn100_imu_link'
        #     ],
        #     output='screen',
        # ),
    ])
