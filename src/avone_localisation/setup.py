from setuptools import setup
import os
from glob import glob

package_name = 'avone_localisation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Install launch files
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # Install config files
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayden',
    maintainer_email='you@example.com',
    description='AVONE localisation launch files for EKF and NavSat',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
