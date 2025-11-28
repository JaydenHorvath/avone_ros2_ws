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
    entry_points={
        'console_scripts': [
            'fix_to_odom = avone_localisation.fix_to_odom:main',
            'heading_to_imu = avone_localisation.heading_to_imu:main',
            'imu_start = avone_localisation.imu_start:main',
            'imu_cov_ekf = avone_localisation.imu_cov_ekf:main',
        ],
    },
)
