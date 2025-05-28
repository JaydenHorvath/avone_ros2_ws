from setuptools import find_packages, setup

package_name = 'avone_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav2_msgs',
        'tf-transformations'
    ],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='c3350128@uon.edu.au',
    description='A simple waypoint array publisher for Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_publisher = avone_nav.waypoint_publisher:main'
        ],
    },
)
