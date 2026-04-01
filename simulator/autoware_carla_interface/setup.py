from setuptools import find_packages
from setuptools import setup

package_name = "autoware_carla_interface"

setup(
    name=package_name,
    version="0.50.0",
    packages=find_packages(where="src"),
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
