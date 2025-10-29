from setuptools import setup

package_name = 'avone_gps'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Publish Odometry from NavSatFix + heading + vel for RViz visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fix_to_odom = avone_gps.fix_to_odom:main',
            'odom_gps_fix = avone_gps.odom_gps_fix:main',
            'gps_serial_replay = avone_gps.gps_serial_replay:main',
            'gps_topics_vis = avone_gps.gps_topics_vis:main',
            'vel_cov_repub= avone_gps.vel_cov_repub:main',
            'gps_imu_sim = avone_gps.gps_imu_sim:main',
            'imu_to_vel = avone_gps.imu_to_vel:main',
            'imu_heading = avone_gps.imu_heading:main',
            'test_imu_tf = avone_gps.test_imu_tf:main',
            'gps_logger = avone_gps.gps_logger:main',
        ],
    },
)
