from setuptools import find_packages, setup

package_name = 'can_simulation'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steer_id = can_simulation.steer_id:main',
            'wheel_vel_test = can_simulation.wheel_vel_test:main',
            'test_cantoros = can_simulation.test_cantoros:main',
            'steer_pid_tune = can_simulation.steer_pid_tune:main',

        ],  
    },
)
