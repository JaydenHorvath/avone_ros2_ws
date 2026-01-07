from setuptools import setup, find_packages
from glob import glob

package_name = "avone_yolo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    python_requires=">=3.8",
    data_files=[
        # Install package.xml and resource index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Config files
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/weights", glob("avone_yolo/weights/*.pt")),
    ],
    install_requires=[
        # Core dependencies
        "numpy",
        "torch>=1.13.0",
        "ultralytics>=8.0.0",
    ],
    zip_safe=True,
    maintainer="Jayden Horvatj",
    maintainer_email="c3350128@uon.edu.au",
    description="AVONES YOLO Cone Detection Packag",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "yolo_node = avone_yolo.yolo_node:main",
            "video_publisher = avone_yolo.video_publisher:main",
            "yolo_base = avone_yolo.yolo_base:main",
            "yolo_dev = avone_yolo.yolo_dev:main",
            "image_publisher = avone_yolo.image_publisher:main",
        ],
    },
)
