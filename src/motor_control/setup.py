from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='c3350128@uon.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = motor_control.motor_control:main',
            'imu_test = motor_control.imu_test:main',
            'gps_odom_test = motor_control.gps_odom_test:main',
            'gps_old_test = motor_control.gps_old_test:main',
            'gps_sim = motor_control.gps_sim:main',
            'CAN_motorcontrollers = motor_control.CAN_motorcontrollers:main',
             'wheelspeedtest = motor_control.wheelspeedtest:main',
             'gpsheading_to_imu = motor_control.gpsheading_to_imu:main',
              'gpsodom = motor_control.gpsodom:main',
    
        ],
    },
)
