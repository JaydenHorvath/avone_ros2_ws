from setuptools import setup, find_packages

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # install package.xml into share/<package>
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A ROS2 node that runs YOLO inference on incoming Image topics',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # this makes “ros2 run yolo_ros yolo_node” invoke your main()
            'yolo_node = yolo_ros.yolo_node:main',
            'cone_posestimation = yolo_ros.cone_posestimation:main',
            'cone_validator = yolo_ros.cone_validator:main'
        ],
    },
)
