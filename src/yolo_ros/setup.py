from setuptools import setup, find_packages
from glob import glob

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    python_requires='>=3.8',
    data_files=[
        # Install package.xml and resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        # Core dependencies
        'numpy',
        'torch>=1.13.0',
        'ultralytics>=8.0.0',
        # Note: cv_bridge, rclpy, tf2_ros, visualization_msgs,
        # and other ROS packages are provided by the ROS2 environment,
        # so should NOT be listed here.
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A ROS2 node that runs YOLO inference on incoming Image topics',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_ros.yolo_node:main',
            'cone_posestimation = yolo_ros.cone_posestimation:main',
            'cone_validator = yolo_ros.cone_validator:main',
            'video_publisher = yolo_ros.video_publisher:main',
            'depthanythingtest = yolo_ros.depthanythingtest:main',
            'yolo_base = yolo_ros.yolo_base:main',
            'landmark_mapper_yolo = yolo_ros.landmark_mapper_yolo:main',
            'cone_kf = yolo_ros.cone_kf:main',
            'dualcamera_yolonode = yolo_ros.dualcamera_yolonode:main',
            'wideview_cam = yolo_ros.wideview_cam:main',
            'depthyolo = yolo_ros.depthyolo:main',
            'depth_test = yolo_ros.depth_test:main',
            'yolo_dev = yolo_ros.yolo_dev:main',
            'infofake = yolo_ros.infofake:main',
            'image_publisher = yolo_ros.image_publisher:main',
            'lidaroverlay = yolo_ros.lidaroverlay:main',
            'filterlidar = yolo_ros.filterlidar:main',

        ],
    },
)
