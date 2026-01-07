from setuptools import find_packages, setup

package_name = "avone_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jayden Horvath",
    maintainer_email="c3350128@uon.edu.au",
    description="AV.ONES Misc Utilities Package",
    license="Proprietary",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "drive_state_led = avone_utils.drive_state_led:main",
            "CAN_to_ROS = avone_utils.CAN_to_ROS:main",
            "cmd_vel_filter = avone_utils.cmd_vel_filter:main",
            "sensor_timeout = avone_utils.sensor_timeout:main",
            "ros_cmd_heartbeat = avone_utils.ros_cmd_heartbeat:main",
        ],
    },
)
