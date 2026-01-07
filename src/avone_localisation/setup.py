from setuptools import setup
import os
from glob import glob

package_name = "avone_localisation"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Install config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jayden Horvath",
    maintainer_email="c3350128@uon.edu.au",
    description="AV.ONES Package for Localisation and position Estimation with both a single and dual ekf filter solutions ",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fix_to_odom = avone_localisation.fix_to_odom:main",
            "heading_to_imu = avone_localisation.heading_to_imu:main",
            "imu_start = avone_localisation.imu_start:main",
        ],
    },
)
