#!/usr/bin/env python3
from setuptools import setup

package_name = 'avone'

setup(
    name=package_name,
    version='0.0.1',
    packages=['scripts'],
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
    maintainer_email='you@example.com',
    description='AV.One utilities',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # this makes `ros2 run avone lidar_image_overlay` invoke scripts/lidar_image_overlay.py
            'lidar_overlay = scripts.lidar_overlay:main',
        ],
    },
)
