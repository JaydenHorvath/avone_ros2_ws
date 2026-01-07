# AV.ONE LiDAR Bringup
# File: avone_lidar_bringup.launch.py
# Launch Command: ros2 launch avone avone_lidar_bringup.launch.py


# Purpose:
#   - Start the Quanergy client driver (quanergy_client_ros) to publish LiDAR point clouds
#   - Start the collision_detection node to provide basic obstacle / safety checking from LiDAR data
#   - Optional static TF between your robot LiDAR frame (laser_frame) and the driver frame (quanergy)


# Notes:
#   - The Quanergy driver is included via its provided launch file (client.launch.py).
#   - The 'host' launch argument must match the LiDAR IP address on your network. Current Quanergy IP is 192.168.1.57


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ============================================================
    # Include: Quanergy client launch
    # ============================================================

    # Path to quanergy client launch
    client_launch_path = os.path.join(
        get_package_share_directory("quanergy_client_ros"), "client.launch.py"
    )

    # Start the Quanergy driver client, pointing to the LiDAR IP
    quanergy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(client_launch_path),
        launch_arguments={"host": "192.168.1.57"}.items(),
    )

    # --- Collision detection node ---
    collision_detection = Node(
        package="avone_nav",
        executable="collision_detection",
        name="collision_detection",
        output="screen",
    )

    # Optional static lidar transform
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="quanergy_static_tf",
    #     arguments=[
    #         "0",
    #         "0",
    #         "0",  # x y z
    #         "0",
    #         "0",
    #         "0",  # roll pitch yaw
    #         "laser_frame",
    #         "quanergy",
    #     ],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            quanergy_launch,
            collision_detection,
            # static_tf_node,
        ]
    )
