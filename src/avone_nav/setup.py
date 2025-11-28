from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'avone_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install Behaviour Tree files
        (os.path.join('share', package_name, 'behaviour_trees'), glob('behaviour_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='c3350128@uon.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_detection = avone_nav.collision_detection:main',
            'nav2_cancel = avone_nav.nav2_cancel:main',
            'nav2_lidar = avone_nav.nav2_lidar:main',
        ],
    },
)
