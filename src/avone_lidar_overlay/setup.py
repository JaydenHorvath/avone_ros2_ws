#!/usr/bin/env python3
from setuptools import find_packages, setup

package_name = 'avone_lidar_overlay'

setup(
    name=package_name,
    version='0.0.1',                             # bump to match your new package
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'tf2_ros',
        'tf_transformations',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Jayden H.',
    maintainer_email='my_email@email.com',
    description='ROS 2 node that overlays LaserScan points on camera images',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # this line makes `ros2 run avone_lidar_overlay lidar_image_overlay` work
            'lidar_image_overlay = avone_lidar_overlay.lidar_image_overlay:main',
        ],
    },
)
