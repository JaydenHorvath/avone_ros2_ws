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
        'rclpy',
        'sensor_msgs',
        'vision_msgs',
        'message_filters',   # <<< add this
        'tf2_ros',
        'tf2_geometry_msgs',
    ],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='c3350128@uon.edu.au',
    description='Maps cone landmarks using an RGB-D camera',
    license='Apache License 2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # <executable_name> = <module>:<function>
            'conelandmarkmapper = cone_mapper.conelandmarkmapper:main',
              'groundremoval = cone_mapper.groundremoval:main',
              'lidargroundremoval = cone_mapper.lidargroundremoval:main',
              'clustering_cone = cone_mapper.clustering_cone:main',
              'conecenterline = cone_mapper.conecenterline:main',
              'yolobasedlandmark = cone_mapper.yolobasedlandmark:main',
              'lidarconemapper = cone_mapper.lidarconemapper:main',
              'DEMOlidarconemapper = cone_mapper.DEMOlidarconemapper:main',
              'debug_image = cone_mapper.debug_image:main',
              'delaunay = cone_mapper.delaunay:main',
              'nav2waypoints = cone_mapper.nav2waypoints:main',
              'trackmapper = cone_mapper.trackmapper:main',
              'cone_export = cone_mapper.cone_export:main',
              'coneselector = cone_mapper.coneselector:main',
              'trolley_cone_marker = cone_mapper.trolley_cone_marker:main',
        ],
    },
)
