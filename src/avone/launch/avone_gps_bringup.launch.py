# AV.ONE GPS Bringup
# File: avone_gps_bringup.launch.py
# Launch Command: ros2 launch avone avone_gps_bringup.launch.py

# Purpose:
#   - Start the NMEA GPS serial driver (nmea_navsat_driver)
#   - Start heading_to_imu to publish a yaw-only IMU message derived from heading

# Notes:
#   - The NMEA driver reads from a serial port and publishes NavSatFix data.
#   - heading_to_imu typically publishes /imu_gps (and optionally markers for debugging).
#   - This file does not declare launch arguments yet, so port/baud/frame_id are fixed.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # GPS serial driver (NMEA -> NavSatFix)
            Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                name="gps_driver",
                output="screen",
                parameters=[
                    {"port": "/dev/ttyUSB1"},
                    {"baud": 115200},
                    {"frame_id": "gps_link1"},
                ],
            ),
            # GPS Heading -> yaw-only IMU publisher (used for navsat_transform / EKF yaw)
            Node(
                package="avone_localisation",
                executable="heading_to_imu",
                name="heading_to_imu",
                output="screen",
            ),
        ]
    )
