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
            'heading_to_imu = avone_gps.heading_to_imu:main',
        ],
    },
)
