from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = "autoware_carla_interface"

setup(
    name=package_name,
    version="0.48.0",
    packages=find_packages(where="src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name), ["config/raw_vehicle_cmd_converter.param.yaml"]),
        (os.path.join("share", package_name, "config"), ["config/sensor_mapping.yaml"]),
        (os.path.join("share", package_name), glob("calibration_maps/*.csv")),
        (os.path.join("share", package_name), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Muhammad Raditya GIOVANNI, Maxime CLEMENT",
    maintainer_email="mradityagio@gmail.com, maxime.clement@tier4.jp",
    description="CARLA ROS 2 bridge for AUTOWARE",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_carla_interface = autoware_carla_interface.carla_autoware:main",
            "multi_camera_combiner = autoware_carla_interface.multi_camera_combiner_node:main",
        ],
    },
    package_dir={"": "src"},
)
