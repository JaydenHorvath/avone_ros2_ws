from setuptools import setup

package_name = 'cone_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # let ROS 2 know about this package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # add any pure-Python dependencies here, e.g.:
        # 'numpy',
    ],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='c3350128@uon.edu.au',
    description='Maps cone landmarks using an RGB-D camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # <executable_name> = <module>:<function>
            'conelandmarkmapper = cone_mapper.conelandmarkmapper:main',
             'live_cone_map = cone_mapper.live_cone_map:main',
        ],
    },
)
