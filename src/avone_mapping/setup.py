from setuptools import setup

package_name = "avone_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        # let ROS 2 know about this package
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # install package.xml
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="jay",
    maintainer_email="c3350128@uon.edu.au",
    description="AVONE Mapping package - cone detection/path planning",
    license="Proprietary",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "lidar_groundremoval = avone_mapping.lidar_groundremoval:main",
            "delaunay_planner = avone_mapping.delaunay_planner:main",
            "coloured_cone_cluster = avone_mapping.coloured_cone_cluster:main",
        ],
    },
)
