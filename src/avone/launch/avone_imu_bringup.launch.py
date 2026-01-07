# AV.ONE IMU Bringup
# File: avone_imu_bringup.launch.py
# Launch Command: ros2 launch avone avone_imu_bringup.launch.py


# Purpose:
#   - Start the VectorNav VN-100T IMU driver node (imu_start)
#   - Optionally provide a static TF between imu_link and a VN100-specific frame

# Notes:
#   - imu_start publishes sensor_msgs/Imu on /imu/data.
#   - use_sim_time is explicitly set to False here for real hardware operation.
#   - The static_transform_publisher block is kept commented out as an optional frame fix.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # VectorNav VN-100T IMU driver (serial -> sensor_msgs/Imu)
            Node(
                package="avone_localisation",
                executable="imu_start",
                name="imu_start",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # Optional static transform: imu_link → vn100_imu_link (180° about X)            # Node(
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
        ]
    )
