from setuptools import find_packages, setup

package_name = "avone_can"

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
    description="V.ONE CAN interface package containing a CAN-to-ROS bridge node and simulation publishers for wheel speed and steering angle CAN signals",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_steer_ang = avone_can.sim_steer_ang:main",
            "sim_wheel_vel = avone_can.sim_wheel_vel:main",
            "can_2_ros = avone_can.can_2_ros:main",
        ],
    },
)
