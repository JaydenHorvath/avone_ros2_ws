from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud': 115200},
                {'frame_id': 'gps_link'}
            ],
        ),
        Node(
            package='avone_localisation',
            executable='heading_to_imu',
            name='heading_to_imu',
            output='screen',
        ),
    ])
